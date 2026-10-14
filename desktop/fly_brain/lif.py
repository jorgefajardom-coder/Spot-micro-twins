"""Modelo LIF del cerebro completo de la mosca (Shiu et al., Nature 2024).

Mismas ecuaciones y parámetros que github.com/philshiu/Drosophila_brain_model
(model.py, implementado allí con Brian2), reescrito con numba para no depender
de un compilador de C++ y poder avanzar a trozos desde el bucle del robot.

    dv/dt = (v_0 - v + g) / t_mbr      (congelado durante el periodo refractario)
    dg/dt = -g / tau                    (ídem)
    v > v_th  ->  v = v_rst, g = 0, refractario t_rfc
    cada espiga presináptica suma w = (signo x n_sinapsis) * w_syn a g tras t_dly

La estimulación imita la optogenética del artículo: entradas Poisson de peso
w_syn * f_poi (68.75 mV) sobre v, y las neuronas estimuladas sin refractario.

Integración exacta y mismo orden de operaciones por paso que Brian2, para reproducir
el modelo original (validate_brian2.py lo compara neurona a neurona).
"""
from dataclasses import dataclass

import numpy as np
from numba import njit

# |g| por debajo de esto (mV) se redondea a 0: despreciable frente a los 7 mV hasta el umbral.
G_EPS = 1e-4


@dataclass(frozen=True)
class Params:
    """Tensiones en mV, tiempos en ms."""
    dt: float = 0.1
    v_0: float = -52.0
    v_rst: float = -52.0
    v_th: float = -45.0
    t_mbr: float = 20.0
    tau: float = 5.0
    t_rfc: float = 2.2
    t_dly: float = 1.8
    w_syn: float = 0.275
    f_poi: float = 250.0


def refractory_steps(p):
    """Pasos congelados tras la espiga. Brian2 vuelve a integrar cuando han pasado
    round(t_rfc / dt) pasos desde la espiga, es decir, congela uno menos."""
    return max(0, int(round(p.t_rfc / p.dt)) - 1)


def exact_coefficients(p):
    """Solución exacta del sistema lineal en un paso dt (lo que hace method='linear' en Brian2):
    v - v_0 <- a_vv (v - v_0) + a_vg g,   g <- a_gg g."""
    a_vv = np.exp(-p.dt / p.t_mbr)
    a_gg = np.exp(-p.dt / p.tau)
    a_vg = p.tau / (p.tau - p.t_mbr) * (a_gg - a_vv)
    return np.float32(a_vv), np.float32(a_vg), np.float32(a_gg)


@njit(cache=True, fastmath=True)
def _integrate(row, v, g, ref_left, v_0, v_th, a_vv, a_vg, a_gg, eps, fired):
    """Un paso para todas las neuronas, sin ramas para que el compilador use SIMD (con ramas
    por neurona el patrón es aleatorio y los fallos de predicción lo hacían ~7 veces más lento).

    Orden de Brian2 en cada paso: integrar -> umbral -> llegan sinapsis -> reset. Y como g
    es "(unless refractory)", Brian2 descarta la entrada sináptica que llega mientras la
    neurona es refractaria. Acumularla daba ~20 % más actividad que el modelo original.
    """
    for i in range(v.shape[0]):
        r = ref_left[i]
        vi = v[i]
        gi = g[i]
        act = r == 0                # en refractario v y g quedan congeladas
        vn = v_0 + a_vv * (vi - v_0) + a_vg * gi
        gn = gi * a_gg
        gn = gn if abs(gn) >= eps else np.float32(0.0)  # evita subnormales (muy lentos)
        fired[i] = act and vn > v_th
        v[i] = vn if act else vi
        g[i] = gn + row[i] if act else gi  # en refractario la entrada se descarta
        row[i] = 0.0
        ref_left[i] = r - 1 if r > 0 else 0


@njit(cache=True)
def _run(n_steps, step0, v, g, ref_left, no_refractory, pending, w_data, w_indices, w_indptr,
         stim_idx, stim_p, w_poi, v_0, v_rst, v_th, a_vv, a_vg, a_gg, eps, ref_steps,
         fired, counts):
    """Simula n_steps pasos dt. pending[fila] es la entrada sináptica que llega en ese paso
    (buffer circular de t_dly); se vacía al leerla y recibe lo que se emite ahora."""
    delay = pending.shape[0]
    for step in range(step0, step0 + n_steps):
        row = pending[step % delay]
        _integrate(row, v, g, ref_left, v_0, v_th, a_vv, a_vg, a_gg, eps, fired)
        for j in range(stim_idx.shape[0]):  # entradas Poisson (como las sinapsis, antes del reset)
            if np.random.random() < stim_p[j]:
                v[stim_idx[j]] += w_poi
        for pre in range(v.shape[0]):
            if fired[pre]:  # pocas por paso: la rama casi nunca se toma
                v[pre] = v_rst
                g[pre] = 0.0
                if not no_refractory[pre]:
                    ref_left[pre] = ref_steps
                counts[pre] += 1
                # Propagación dirigida por eventos: solo la columna de quien disparó
                for k in range(w_indptr[pre], w_indptr[pre + 1]):
                    row[w_indices[k]] += w_data[k]


@njit(cache=True)
def _seed(seed):
    np.random.seed(seed)


class Brain:
    def __init__(self, connectome, params=Params(), seed=None):
        self.c = connectome
        self.p = params
        n = connectome.n
        w = connectome.weights.tocsc()
        self.w_data = (w.data * params.w_syn).astype(np.float32)
        self.w_indices = w.indices.astype(np.int32)
        self.w_indptr = w.indptr.astype(np.int64)
        self.v = np.full(n, params.v_0, np.float32)
        self.g = np.zeros(n, np.float32)
        self.ref_left = np.zeros(n, np.int16)  # pasos de refractario que quedan
        self.no_refractory = np.zeros(n, np.bool_)
        self.pending = np.zeros((max(1, int(round(params.t_dly / params.dt))), n), np.float32)
        self.spike_count = np.zeros(n, np.int64)
        self._fired = np.zeros(n, np.bool_)
        self.step_i = 0
        self.stim_idx = np.empty(0, np.int64)
        self.stim_p = np.empty(0)  # probabilidad de evento Poisson por paso
        self._consts = (np.float32(params.w_syn * params.f_poi), np.float32(params.v_0),
                        np.float32(params.v_rst), np.float32(params.v_th), *exact_coefficients(params),
                        np.float32(G_EPS), refractory_steps(params))
        _seed(np.random.SeedSequence(seed).generate_state(1)[0] if seed is not None
              else np.random.randint(2**31))

    @property
    def t(self):
        return self.step_i * self.p.dt

    def reset(self):
        """Vuelve al reposo (para reutilizar el mismo Brain en varios experimentos)."""
        self.v.fill(self.p.v_0)
        for a in (self.g, self.ref_left, self.pending, self.spike_count):
            a.fill(0)
        self.stimulate()
        self.step_i = 0

    def stimulate(self, *groups):
        """groups: pares (índices, Hz). Sustituye la estimulación anterior; sin grupos la apaga."""
        groups = [(np.asarray(i, np.int64), r) for i, r in groups if r > 0 and len(i)]
        self.stim_idx = np.concatenate([i for i, _ in groups]) if groups else np.empty(0, np.int64)
        self.stim_p = (np.concatenate([np.full(len(i), r * self.p.dt * 1e-3) for i, r in groups])
                       if groups else np.empty(0))
        self.no_refractory[:] = False
        self.no_refractory[self.stim_idx] = True

    def run(self, ms):
        """Simula ms milisegundos. Devuelve el número de espigas por neurona en ese intervalo."""
        start = self.spike_count.copy()
        n_steps = int(round(ms / self.p.dt))
        _run(n_steps, self.step_i, self.v, self.g, self.ref_left, self.no_refractory, self.pending,
             self.w_data, self.w_indices, self.w_indptr, self.stim_idx, self.stim_p,
             *self._consts, self._fired, self.spike_count)
        self.step_i += n_steps
        return self.spike_count - start

    @staticmethod
    def rate(counts, ms, idx):
        """Frecuencia media (Hz) de un grupo de neuronas a partir de un conteo de run()."""
        return counts[idx].mean() * 1000.0 / ms if len(idx) else 0.0


def make_brain(connectome, params=Params(), seed=None, device='auto'):
    """Brain en la GPU si hay CUDA (lif_gpu.BrainGPU), si no en la CPU. device: auto|gpu|cpu."""
    if device != 'cpu':
        import lif_gpu
        if lif_gpu.available():
            return lif_gpu.BrainGPU(connectome, params, seed)
        if device == 'gpu':
            raise RuntimeError("No hay GPU CUDA disponible (pip install \"cupy-cuda12x[ctk]\")")
    return Brain(connectome, params, seed)
