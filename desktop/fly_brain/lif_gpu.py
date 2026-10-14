"""El mismo modelo LIF de lif.py, en la GPU (CUDA vía CuPy).

Misma API que lif.Brain (stimulate, run, rate, t). Por paso dt lanza dos kernels
y nunca espera a la CPU:

1. integrate: un hilo por neurona. Suma la entrada que llega, aplica las entradas
   Poisson, integra v y g y, si dispara, se apunta en una lista con atomicAdd.
2. propagate: cada bloque recoge espigas de esa lista y suma su columna de pesos
   a la fila del buffer de retardo (atomicAdd). Lee cuántas espigas hay de la
   memoria de la GPU, así que no hace falta sincronizar.

    pip install "cupy-cuda12x[ctk]"
"""
import numpy as np

from lif import G_EPS, Params, exact_coefficients, refractory_steps

try:
    import warnings
    with warnings.catch_warnings():
        # Con cupy-cuda12x[ctk] CUDA viene de pip y CUDA_PATH no hace falta
        warnings.filterwarnings('ignore', message='CUDA path could not be detected')
        import cupy as cp
except ImportError:  # sin GPU / sin CuPy: usar lif.Brain
    cp = None

_SRC = r"""
__device__ __forceinline__ float uniform(unsigned long long x) {
    // splitmix64 -> [0, 1)
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
    x ^= x >> 31;
    return (x >> 40) * (1.0f / 16777216.0f);
}

extern "C" __global__ void integrate(
        float* v, float* g, short* ref_left, const bool* no_refractory, float* row,
        const float* stim_p, int* counts, int* spikes, int* n_spk, const long long* step_dev,
        int n, int delay, unsigned long long seed,
        float w_poi, float v_0, float v_rst, float v_th, float a_vv, float a_vg, float a_gg,
        float eps, short ref_steps) {
    long long step = *step_dev;
    int cur = (int)(step & 1);
    row += (step % delay) * (long long)n;
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i == 0) n_spk[1 - cur] = 0;  // contador del paso siguiente (propagate ya lo leyó)
    if (i >= n) return;
    // Orden de Brian2: integrar -> umbral -> llegan sinapsis y Poisson -> reset
    float vi = v[i], gi = g[i];
    short r = ref_left[i];
    bool fired = false;
    if (r > 0) {
        r -= 1;  // refractario: v y g congeladas y la entrada sináptica se descarta
    } else {
        vi = v_0 + a_vv * (vi - v_0) + a_vg * gi;  // solución exacta (method='linear')
        gi *= a_gg;
        if (fabsf(gi) < eps) gi = 0.0f;  // evita subnormales
        fired = vi > v_th;
        gi += row[i];  // g es "(unless refractory)": Brian2 solo la suma fuera del refractario
    }
    row[i] = 0.0f;
    float p = stim_p[i];
    if (p > 0.0f && uniform(seed ^ ((unsigned long long)step * n + i)) < p)
        vi += w_poi;  // entrada Poisson tipo optogenética
    if (fired) {  // el reset borra lo que llegó en este mismo paso, como en Brian2
        vi = v_rst;
        gi = 0.0f;
        r = no_refractory[i] ? 0 : ref_steps;
        counts[i] += 1;
        spikes[atomicAdd(&n_spk[cur], 1)] = i;
    }
    ref_left[i] = r;
    v[i] = vi;
    g[i] = gi;
}

extern "C" __global__ void propagate(
        const int* spikes, const int* n_spk, const long long* step_dev, int n_neurons, int delay,
        const long long* indptr, const int* indices, const float* data, float* row) {
    long long step = *step_dev;
    row += (step % delay) * (long long)n_neurons;
    int n = n_spk[step & 1];
    for (int s = blockIdx.x; s < n; s += gridDim.x) {
        int pre = spikes[s];
        for (long long k = indptr[pre] + threadIdx.x; k < indptr[pre + 1]; k += blockDim.x)
            atomicAdd(&row[indices[k]], data[k]);
    }
}

extern "C" __global__ void advance(long long* step_dev) {
    *step_dev += 1;
}
"""

THREADS = 256
PROPAGATE_BLOCKS = 256
GRAPH_STEPS = 100  # pasos por CUDA Graph


def available():
    """¿Hay CuPy y una GPU CUDA utilizable?"""
    if cp is None:
        return False
    try:
        return cp.cuda.runtime.getDeviceCount() > 0
    except cp.cuda.runtime.CUDARuntimeError:
        return False


class BrainGPU:
    def __init__(self, connectome, params=Params(), seed=None):
        if not available():
            raise RuntimeError("CuPy/CUDA no disponible; usa lif.Brain")
        self.c = connectome
        self.p = params
        n = self.n = connectome.n
        w = connectome.weights.tocsc()
        self.w_data = cp.asarray((w.data * params.w_syn).astype(np.float32))
        self.w_indices = cp.asarray(w.indices.astype(np.int32))
        self.w_indptr = cp.asarray(w.indptr.astype(np.int64))
        self.v = cp.full(n, params.v_0, cp.float32)
        self.g = cp.zeros(n, cp.float32)
        self.ref_left = cp.zeros(n, cp.int16)
        self.no_refractory = cp.zeros(n, cp.bool_)
        self.stim_p = cp.zeros(n, cp.float32)  # probabilidad de evento Poisson por paso
        self.pending = cp.zeros((max(1, int(round(params.t_dly / params.dt))), n), cp.float32)
        self.counts = cp.zeros(n, cp.int32)
        self.spikes = cp.zeros(n, cp.int32)
        self.n_spk = cp.zeros(2, cp.int32)
        self.step_dev = cp.zeros(1, cp.int64)  # número de paso, en la GPU para los grafos
        self.step_i = 0
        self.seed = np.uint64(np.random.SeedSequence(seed).generate_state(1, np.uint64)[0])
        module = cp.RawModule(code=_SRC)
        self._integrate = module.get_function('integrate')
        self._propagate = module.get_function('propagate')
        self._advance = module.get_function('advance')
        self._consts = (np.float32(params.w_syn * params.f_poi), np.float32(params.v_0),
                        np.float32(params.v_rst), np.float32(params.v_th), *exact_coefficients(params),
                        np.float32(G_EPS), np.int16(refractory_steps(params)))
        self._grid = ((n + THREADS - 1) // THREADS,)
        # Bloqueante: se ordena con el stream por defecto, así que tocar v, g… desde fuera es seguro
        self._stream = cp.cuda.Stream(non_blocking=False)
        self._graph = None
        cp.cuda.Device().synchronize()  # las copias de arriba van por el stream por defecto

    @property
    def t(self):
        return self.step_i * self.p.dt

    def reset(self):
        """Vuelve al reposo sin volver a subir los pesos ni regrabar el grafo."""
        with self._stream:
            self.v.fill(self.p.v_0)
            for a in (self.g, self.ref_left, self.pending, self.counts, self.n_spk, self.step_dev):
                a.fill(0)
        self.stimulate()
        self.step_i = 0

    def stimulate(self, *groups):
        """groups: pares (índices, Hz). Sustituye la estimulación anterior; sin grupos la apaga."""
        with self._stream:  # mismo stream que la simulación: sin carreras con los grafos
            self.stim_p.fill(0)
            self.no_refractory.fill(False)
            for idx, hz in groups:
                if hz > 0 and len(idx):
                    idx = cp.asarray(np.asarray(idx, np.int64))
                    self.stim_p[idx] = hz * self.p.dt * 1e-3
                    self.no_refractory[idx] = True

    def _launch_step(self):
        delay = np.int32(self.pending.shape[0])
        n = np.int32(self.n)
        self._integrate(self._grid, (THREADS,), (
            self.v, self.g, self.ref_left, self.no_refractory, self.pending, self.stim_p,
            self.counts, self.spikes, self.n_spk, self.step_dev, n, delay, self.seed,
            *self._consts))
        self._propagate((PROPAGATE_BLOCKS,), (THREADS,), (
            self.spikes, self.n_spk, self.step_dev, n, delay,
            self.w_indptr, self.w_indices, self.w_data, self.pending))
        self._advance((1,), (1,), (self.step_dev,))

    def run(self, ms):
        """Simula ms milisegundos. Devuelve (en la CPU) las espigas por neurona en ese intervalo.

        Los pasos se lanzan en bloques de GRAPH_STEPS grabados como CUDA Graph: lanzar
        cada kernel desde Python costaba más que el propio cálculo.
        """
        n_steps = int(round(ms / self.p.dt))
        with self._stream:
            start = self.counts.copy()
            if self._graph is None and n_steps >= GRAPH_STEPS:
                self._stream.begin_capture()
                for _ in range(GRAPH_STEPS):
                    self._launch_step()
                self._graph = self._stream.end_capture()
            for _ in range(n_steps // GRAPH_STEPS if self._graph else 0):
                self._graph.launch(self._stream)
            for _ in range(n_steps - (n_steps // GRAPH_STEPS) * GRAPH_STEPS if self._graph else n_steps):
                self._launch_step()
            counts = cp.asnumpy(self.counts - start, stream=self._stream)
        self._stream.synchronize()
        self.step_i += n_steps
        return counts.astype(np.int64)

    @staticmethod
    def rate(counts, ms, idx):
        """Frecuencia media (Hz) de un grupo de neuronas a partir de un conteo de run()."""
        return counts[idx].mean() * 1000.0 / ms if len(idx) else 0.0
