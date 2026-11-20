"""Mide el rendimiento del simulador para la wiki (docs/wiki/cerebro/05-reduccion-del-tiempo.md).

Cuatro medidas, en la CPU y (si hay CUDA) en la GPU:

1. velocidad   veces tiempo real y µs por paso, con dt 0,1 y 0,5 ms (looming LPLC2 + LC4)
2. actividad   espigas por paso y sinapsis entregadas por paso: cuánto ahorra de verdad la
               propagación por eventos frente al producto matriz-vector completo
3. grafos      GPU con y sin CUDA Graphs (lanzando cada kernel desde Python)
4. ramas       CPU: integración sin ramas (la de lif.py) frente a la versión con `if`

    python benchmark.py                  # todo, con el conectoma de FlyWire
    python benchmark.py --only speed     # una sola medida (speed|activity|graphs|branches)
    python benchmark.py --markdown       # además, tablas en Markdown para pegar en la wiki
    python benchmark.py --synthetic      # red aleatoria del mismo tamaño, sin datos (solo
                                         # para probar el script: sus cifras no son las del cerebro)

Guarda los resultados en data/benchmark.json.
"""
import argparse
import json
import platform
import time

import numpy as np
import pandas as pd
import scipy.sparse as sp
from numba import njit

import lif_gpu
from connectome import DATA, Connectome
from lif import G_EPS, Params, _integrate, exact_coefficients, make_brain

LOOMING = ('LPLC2', 'LC4')
HZ = 150.0


# ----------------------------------------------------------------------------- datos
def synthetic_connectome(n=138_639, edges=15_000_000, seed=0):
    """Red aleatoria con el tamaño del conectoma (sinapsis log-normales, 70 % excitatorias)."""
    rng = np.random.default_rng(seed)
    pre = rng.integers(0, n, edges)
    post = rng.integers(0, n, edges)
    syn = np.maximum(1, rng.lognormal(0.8, 1.0, edges)).astype(np.float32)
    sign = np.where(rng.random(n) < 0.7, 1.0, -1.0).astype(np.float32)
    w = sp.csc_matrix((syn * sign[pre], (post, pre)), shape=(n, n))
    w.sum_duplicates()
    types = np.full(n, None, object)
    types[:120], types[120:220] = 'LPLC2', 'LC4'
    ann = pd.DataFrame({'root_id': np.arange(n), 'cell_type': types, 'hemibrain_type': None,
                        'side': np.where(np.arange(n) % 2, 'left', 'right'), 'super_class': 'central'})
    return Connectome(w, np.arange(n), ann)


def devices():
    return ['cpu', 'gpu'] if lif_gpu.available() else ['cpu']


def looming(c):
    return np.concatenate([c.find(t) for t in LOOMING])


# ----------------------------------------------------------------------------- 1. velocidad
def bench_speed(c, sim_ms=1000.0):
    out = []
    stim = looming(c)
    for dev in devices():
        for dt in (0.1, 0.5):
            b = make_brain(c, Params(dt=dt), seed=0, device=dev)
            b.stimulate((stim, HZ))
            b.run(100.0)  # calentamiento: compila numba / graba el CUDA Graph
            t0 = time.perf_counter()
            b.run(sim_ms)
            wall = time.perf_counter() - t0
            steps = sim_ms / dt
            out.append({'device': dev, 'dt_ms': dt, 'x_real_time': sim_ms / 1000.0 / wall,
                        'us_per_step': wall / steps * 1e6, 'wall_s': wall})
            print(f"  {dev} dt {dt} ms: {out[-1]['x_real_time']:.2f}x tiempo real, "
                  f"{out[-1]['us_per_step']:.1f} µs/paso")
    return out


# ----------------------------------------------------------------------------- 2. actividad
def bench_activity(c, sim_ms=500.0):
    """Espigas por paso en reposo con looming. Las sinapsis entregadas son la suma del grado de
    salida de las que disparan: el trabajo real de la propagación por eventos."""
    p = Params(dt=0.1)
    b = make_brain(c, p, seed=0, device=devices()[-1])
    b.stimulate((looming(c), HZ))
    out_degree = np.diff(c.weights.tocsc().indptr)
    nnz = int(c.weights.nnz)
    spikes, deliveries = [], []
    b.run(50.0)  # transitorio
    for _ in range(int(sim_ms / p.dt)):
        counts = b.run(p.dt)
        fired = np.flatnonzero(counts)
        spikes.append(len(fired))
        deliveries.append(int(out_degree[fired].sum()))
    spikes, deliveries = np.array(spikes), np.array(deliveries)
    res = {'connections': nnz, 'mean_out_degree': nnz / c.n,
           'spikes_per_step_mean': float(spikes.mean()), 'spikes_per_step_median': float(np.median(spikes)),
           'spikes_per_step_p95': float(np.percentile(spikes, 95)),
           'deliveries_per_step_mean': float(deliveries.mean()),
           'saving_vs_full_product': nnz / max(1.0, float(deliveries.mean())),
           'fraction_firing_per_step': float(spikes.mean()) / c.n}
    print(f"  espigas/paso: media {res['spikes_per_step_mean']:.0f}, p95 {res['spikes_per_step_p95']:.0f} "
          f"({100 * res['fraction_firing_per_step']:.3f} % de las neuronas)")
    print(f"  sinapsis entregadas/paso: {res['deliveries_per_step_mean']:,.0f} de {nnz:,} "
          f"-> {res['saving_vs_full_product']:.0f}x menos trabajo que el producto completo")
    return res


# ----------------------------------------------------------------------------- 3. grafos
def bench_graphs(c, sim_ms=500.0):
    if not lif_gpu.available():
        print("  sin GPU CUDA: se omite")
        return None
    stim = looming(c)
    res = {}
    for name, chunk_steps in (('without_graphs', lif_gpu.GRAPH_STEPS - 1), ('with_graphs', lif_gpu.GRAPH_STEPS)):
        b = lif_gpu.BrainGPU(c, Params(dt=0.1), seed=0)  # nuevo: sin grafo grabado
        b.stimulate((stim, HZ))
        chunk_ms = chunk_steps * 0.1
        b.run(chunk_ms)  # calentamiento (con 100 pasos graba el grafo)
        n = int(sim_ms / chunk_ms)
        t0 = time.perf_counter()
        for _ in range(n):
            b.run(chunk_ms)
        wall = time.perf_counter() - t0
        res[name] = {'x_real_time': n * chunk_ms / 1000.0 / wall, 'us_per_step': wall / (n * chunk_steps) * 1e6}
        print(f"  {name}: {res[name]['x_real_time']:.2f}x tiempo real, {res[name]['us_per_step']:.1f} µs/paso")
    res['speedup'] = res['with_graphs']['x_real_time'] / res['without_graphs']['x_real_time']
    print(f"  CUDA Graphs: {res['speedup']:.1f}x más rápido")
    return res


# ----------------------------------------------------------------------------- 4. ramas
@njit(cache=True, fastmath=True)
def _integrate_branchy(row, v, g, ref_left, v_0, v_th, a_vv, a_vg, a_gg, eps, fired):
    """La versión 'natural' de _integrate: se salta las refractarias con un if."""
    for i in range(v.shape[0]):
        if ref_left[i] > 0:
            ref_left[i] -= 1
            row[i] = 0.0
            fired[i] = False
            continue
        vn = v_0 + a_vv * (v[i] - v_0) + a_vg * g[i]
        gn = g[i] * a_gg
        if abs(gn) < eps:
            gn = np.float32(0.0)
        fired[i] = vn > v_th
        v[i] = vn
        g[i] = gn + row[i]
        row[i] = 0.0


def bench_branches(n=138_639, reps=2000, refractory_fraction=0.1, seed=0):
    """Solo la fase de integración, con un patrón aleatorio de neuronas refractarias."""
    rng = np.random.default_rng(seed)
    p = Params(dt=0.1)
    a_vv, a_vg, a_gg = exact_coefficients(p)
    template = np.where(rng.random(n) < refractory_fraction, rng.integers(1, 22, n), 0).astype(np.int16)
    v = rng.uniform(-52, -45.5, n).astype(np.float32)
    g = rng.normal(0, 2, n).astype(np.float32)
    row = np.zeros(n, np.float32)
    ref = template.copy()
    fired = np.zeros(n, np.bool_)
    args = (np.float32(p.v_0), np.float32(p.v_th), a_vv, a_vg, a_gg, np.float32(G_EPS), fired)
    res = {}
    for name, fn in (('branch_free', _integrate), ('branchy', _integrate_branchy)):
        fn(row, v.copy(), g.copy(), ref, *args)  # compilar
        total = 0.0
        for _ in range(reps):
            ref[:] = template  # mismo patrón aleatorio en cada repetición
            vv, gg = v.copy(), g.copy()
            t0 = time.perf_counter()
            fn(row, vv, gg, ref, *args)
            total += time.perf_counter() - t0
        res[name] = {'us_per_step': total / reps * 1e6}
        print(f"  {name}: {res[name]['us_per_step']:.1f} µs por paso de integración")
    res['speedup'] = res['branchy']['us_per_step'] / res['branch_free']['us_per_step']
    print(f"  sin ramas: {res['speedup']:.1f}x más rápido")
    return res


# ----------------------------------------------------------------------------- salida
def hardware():
    info = {'cpu': platform.processor() or platform.machine(), 'python': platform.python_version()}
    if lif_gpu.available():
        cp = lif_gpu.cp
        info['gpu'] = cp.cuda.runtime.getDeviceProperties(0)['name'].decode()
    return info


def markdown(r):
    lines = []
    if r.get('speed'):
        lines += ["| Dispositivo | dt | Velocidad | Coste por paso |", "|---|---|---|---|"]
        lines += [f"| {s['device'].upper()} | {s['dt_ms']} ms | {s['x_real_time']:.2f}× tiempo real | "
                  f"{s['us_per_step']:.1f} µs |" for s in r['speed']]
        lines.append("")
    if r.get('activity'):
        a = r['activity']
        lines += ["| Por paso de 0,1 ms (looming) | Valor |", "|---|---|",
                  f"| Espigas (media / p95) | {a['spikes_per_step_mean']:.0f} / {a['spikes_per_step_p95']:.0f} |",
                  f"| Sinapsis entregadas | {a['deliveries_per_step_mean']:,.0f} |",
                  f"| Producto completo | {a['connections']:,} |",
                  f"| Ahorro de la propagación por eventos | **{a['saving_vs_full_product']:.0f}×** |", ""]
    if r.get('graphs'):
        gr = r['graphs']
        lines += ["| GPU | Velocidad | Coste por paso |", "|---|---|---|",
                  f"| Kernels lanzados uno a uno | {gr['without_graphs']['x_real_time']:.2f}× | "
                  f"{gr['without_graphs']['us_per_step']:.1f} µs |",
                  f"| CUDA Graphs de 100 pasos | {gr['with_graphs']['x_real_time']:.2f}× | "
                  f"{gr['with_graphs']['us_per_step']:.1f} µs |", ""]
    if r.get('branches'):
        br = r['branches']
        lines += ["| Integración en CPU | Coste por paso |", "|---|---|",
                  f"| Con `if` por neurona | {br['branchy']['us_per_step']:.1f} µs |",
                  f"| Sin ramas (SIMD) | {br['branch_free']['us_per_step']:.1f} µs |", ""]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--only', choices=['speed', 'activity', 'graphs', 'branches'])
    ap.add_argument('--synthetic', action='store_true', help="red aleatoria del mismo tamaño (sin datos)")
    ap.add_argument('--synthetic-edges', type=int, default=15_000_000)
    ap.add_argument('--markdown', action='store_true')
    args = ap.parse_args()

    c = (synthetic_connectome(edges=args.synthetic_edges) if args.synthetic else Connectome.load())
    print(f"{'Red sintética' if args.synthetic else 'FlyWire v783'}: {c.n:,} neuronas, "
          f"{c.weights.nnz:,} conexiones · {', '.join(d.upper() for d in devices())}")
    results = {'synthetic': args.synthetic, 'hardware': hardware()}
    steps = {'speed': bench_speed, 'activity': bench_activity, 'graphs': bench_graphs,
             'branches': lambda _c: bench_branches()}
    for name, fn in steps.items():
        if args.only in (None, name):
            print(f"\n[{name}]")
            results[name] = fn(c)
    DATA.mkdir(exist_ok=True)
    path = DATA / ('benchmark_synthetic.json' if args.synthetic else 'benchmark.json')
    path.write_text(json.dumps(results, indent=2, ensure_ascii=False))
    print(f"\nGuardado en {path}")
    if args.markdown:
        print("\n" + markdown(results))


if __name__ == '__main__':
    main()
