"""Valida nuestro simulador contra el modelo ORIGINAL de Shiu et al. (Brian2).

Descarga su model.py (MIT) en data/reference/, corre el mismo experimento con su
código y con el nuestro, y compara las frecuencias neurona a neurona.

    python validate_brian2.py                  # looming (LPLC2 + LC4), 8 ensayos
    python validate_brian2.py LC10a --trials 4

Necesita brian2 y joblib, y el compilador de Visual Studio (ver msvc_env.py).
"""
import argparse
import sys
import time
import urllib.request

import numpy as np
import pandas as pd

import msvc_env
from connectome import DATA, Connectome
from lif import Params, make_brain

REF = DATA / "reference"
REPO = "https://raw.githubusercontent.com/philshiu/Drosophila_brain_model/main/"


def reference_model():
    REF.mkdir(parents=True, exist_ok=True)
    for name in ("model.py", "LICENSE"):
        if not (REF / name).exists():
            urllib.request.urlretrieve(REPO + name, REF / name)
    if not msvc_env.setup():
        sys.exit("No se encontró el compilador de C++ de Visual Studio (ver msvc_env.py)")
    sys.path.insert(0, str(REF))
    import model  # el model.py original
    return model


def run_reference(model, name, root_ids, trials, ms, reuse=False):
    """Frecuencia media (Hz) por neurona con el código original. Devuelve (Hz por root_id, s, ensayos)."""
    res = REF / "results"
    res.mkdir(exist_ok=True)
    path = res / f"{name}.parquet"
    t0 = time.time()
    if not (reuse and path.exists()):
        params = dict(model.default_params, n_run=trials, t_run=ms * model.ms)
        model.run_exp(exp_name=name, neu_exc=list(map(int, root_ids)), path_res=res,
                      path_comp=DATA / "Completeness_783.csv", path_con=DATA / "Connectivity_783.parquet",
                      params=params, n_proc=min(trials, 8), force_overwrite=True)
    spikes = pd.read_parquet(path)
    trials = spikes.trial.nunique()
    return spikes.flywire_id.value_counts() / trials / (ms / 1000.0), time.time() - t0, trials


def run_ours(c, stim, trials, ms):
    brain = make_brain(c, Params(dt=0.1), seed=0)
    counts = np.zeros(c.n)
    t0 = time.time()
    for _ in range(trials):
        brain.reset()
        brain.stimulate((stim, 150))
        counts += brain.run(ms)
    return counts / trials / (ms / 1000.0), time.time() - t0, type(brain).__name__


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("cell_types", nargs="*", default=["LPLC2", "LC4"])
    ap.add_argument("--trials", type=int, default=8)
    ap.add_argument("--ms", type=float, default=1000)
    ap.add_argument("--reuse", action="store_true", help="reutilizar el resultado de Brian2 ya guardado")
    args = ap.parse_args()

    c = Connectome.load()
    stim = np.concatenate([c.find(t) for t in args.cell_types])
    name = "_".join(args.cell_types)
    print(f"Estímulo {args.cell_types}: {len(stim)} neuronas a 150 Hz, {args.trials} ensayos de {args.ms:.0f} ms")

    model = reference_model()
    ref_by_id, t_ref, ref_trials = run_reference(model, name, c.root_ids[stim], args.trials, args.ms,
                                                 args.reuse)
    ref = ref_by_id.reindex(c.root_ids).fillna(0).to_numpy()
    our_trials = max(ref_trials, 30)  # los nuestros son baratos: menos ruido Poisson
    ours, t_ours, backend = run_ours(c, stim, our_trials, args.ms)

    others = np.setdiff1d(np.arange(c.n), stim)  # sin las estimuladas (triviales)
    active = others[(ref[others] > 0) | (ours[others] > 0)]
    r = np.corrcoef(ref[active], ours[active])[0, 1]
    slope = np.sum(ref[active] * ours[active]) / np.sum(ref[active] ** 2)
    print(f"\nOriginal (Brian2): {ref_trials} ensayos, {t_ref:.0f} s · "
          f"nuestro ({backend}): {our_trials} ensayos, {t_ours:.1f} s")
    print(f"Neuronas activas (sin las estimuladas): original {(ref[others] > 0).sum()}, "
          f"nuestro {(ours[others] > 0).sum()}")
    print(f"Correlación neurona a neurona: r = {r:.3f} · pendiente nuestro/original = {slope:.3f}")

    a = c.annotations
    dn = others[(a.super_class.to_numpy()[others] == "descending")]
    top = dn[np.argsort(-ref[dn])][:12]
    print("\nDescendentes más activas         original   nuestro")
    for i in top:
        print(f"  {a.cell_type[i]!s:>8} {a.side[i]!s:>5}            {ref[i]:7.1f}   {ours[i]:7.1f} Hz")


if __name__ == "__main__":
    main()
