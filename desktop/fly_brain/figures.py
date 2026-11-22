"""Figuras de la wiki del cerebro (docs/wiki/assets/).

    python figures.py speed     # velocidad de cada versión (no necesita datos)
    python figures.py scatter   # original contra nuestro, neurona a neurona
                                # (antes: python validate_brian2.py, que guarda data/validation_*.npz)
    python figures.py raster    # espigas durante un escape (necesita el conectoma de FlyWire)
    python figures.py neuron    # una neurona LIF recibiendo espigas (no necesita datos)
    python figures.py all

Cada figura se guarda en dos idiomas: <nombre>_es.png y <nombre>_en.png.
"""
import argparse
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE.parents[1] / "docs" / "wiki" / "assets"

# Paleta de referencia (skill de visualización): superficie clara, tinta y un solo tono
SURFACE, INK, INK_2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#8a8984", "#e6e5e0"
BLUE, BLUE_LIGHT = "#2a78d6", "#a9c8ef"

# Velocidad (s de mosca por s de reloj). RTX 4060 Laptop / i7-12650H, looming LPLC2 + LC4.
# Brian2: 8 ensayos de 1 s en 740 s (incluye generar y compilar su C++).
SPEED = [
    ("brian2", 8 / 740),
    ("cpu01", 0.29),
    ("cpu05", 0.99),
    ("gpu01", 8.3),
    ("gpu05", 33.0),
]
TEXT = {
    "es": {
        "brian2": "Brian2 original · dt 0,1 ms", "cpu01": "lif.py CPU · dt 0,1 ms",
        "cpu05": "lif.py CPU · dt 0,5 ms", "gpu01": "lif_gpu.py GPU · dt 0,1 ms",
        "gpu05": "lif_gpu.py GPU · dt 0,5 ms",
        "speed_title": "Segundos de mosca simulados por segundo de reloj",
        "speed_sub": "Escala logarítmica · la línea marca el tiempo real (1×)",
        "realtime": "tiempo real",
        "scatter_title": "Original (Brian2) contra nuestro simulador",
        "scatter_x": "Original, Brian2 (Hz)", "scatter_y": "Nuestro, GPU (Hz)",
        "raster_title": "Espigas durante un escape",
        "raster_x": "Tiempo (ms)", "raster_on": "looming encendido",
        "neuron_title": "Una neurona LIF recibiendo espigas",
        "neuron_sub": "lif.py con dos neuronas: A dispara a 150 Hz (Poisson) y llega a B con 40 sinapsis",
        "neuron_pre": "Espigas de A", "neuron_g": "g de B (mV)", "neuron_v": "v de B (mV)",
        "neuron_th": "umbral −45 mV", "neuron_rst": "reposo y reset −52 mV", "neuron_x": "Tiempo (ms)",
        "neuron_spike": "B dispara",
        "groups": {"LPLC2": "LPLC2 (entrada)", "LC4": "LC4 (entrada)", "DNp01": "Giant Fiber (DNp01)",
                   "MDN": "MDN (atrás)", "DNp09": "P9 (avance)"},
    },
    "en": {
        "brian2": "Original Brian2 · dt 0.1 ms", "cpu01": "lif.py CPU · dt 0.1 ms",
        "cpu05": "lif.py CPU · dt 0.5 ms", "gpu01": "lif_gpu.py GPU · dt 0.1 ms",
        "gpu05": "lif_gpu.py GPU · dt 0.5 ms",
        "speed_title": "Seconds of fly simulated per wall-clock second",
        "speed_sub": "Log scale · the line marks real time (1×)",
        "realtime": "real time",
        "scatter_title": "Original (Brian2) vs our simulator",
        "scatter_x": "Original, Brian2 (Hz)", "scatter_y": "Ours, GPU (Hz)",
        "raster_title": "Spikes during an escape",
        "raster_x": "Time (ms)", "raster_on": "looming on",
        "neuron_title": "One LIF neuron receiving spikes",
        "neuron_sub": "lif.py with two neurons: A fires at 150 Hz (Poisson) and reaches B through 40 synapses",
        "neuron_pre": "A's spikes", "neuron_g": "B's g (mV)", "neuron_v": "B's v (mV)",
        "neuron_th": "threshold −45 mV", "neuron_rst": "rest and reset −52 mV", "neuron_x": "Time (ms)",
        "neuron_spike": "B fires",
        "groups": {"LPLC2": "LPLC2 (input)", "LC4": "LC4 (input)", "DNp01": "Giant Fiber (DNp01)",
                   "MDN": "MDN (backward)", "DNp09": "P9 (forward)"},
    },
}


def _style(plt):
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "font.family": "DejaVu Sans", "font.size": 11, "text.color": INK,
        "axes.edgecolor": GRID, "axes.labelcolor": INK_2, "xtick.color": INK_2, "ytick.color": INK_2,
        "axes.spines.top": False, "axes.spines.right": False,
    })


def _title(fig, title, sub=None):
    fig.text(0.02, 0.965, title, fontsize=14, fontweight="bold", color=INK, va="top")
    if sub:
        fig.text(0.02, 0.905, sub, fontsize=10.5, color=INK_2, va="top")


def speed(lang):
    import matplotlib.pyplot as plt
    _style(plt)
    t = TEXT[lang]
    labels = [t[k] for k, _ in SPEED][::-1]
    vals = [v for _, v in SPEED][::-1]
    fig, ax = plt.subplots(figsize=(9, 4.2), dpi=200)
    fig.subplots_adjust(left=0.29, right=0.93, top=0.8, bottom=0.12)
    ax.set_xscale("log")
    ax.set_xlim(0.005, 120)
    ax.set_ylim(-0.6, len(vals) - 0.4)
    h = 0.56
    for y, (lab, v) in enumerate(zip(labels, vals)):
        key = [k for k, _ in SPEED][::-1][y]
        color = BLUE if key == "gpu01" else BLUE_LIGHT  # la versión que usa el robot
        # Barra anclada al borde izquierdo del eje (en escala log no hay cero)
        ax.barh(y, v - 0.005, left=0.005, height=h, color=color, linewidth=0)
        txt = f"{v:.3f}×" if v < 0.1 else f"{v:.2f}×" if v < 1 else f"{v:g}×"
        ax.text(v * 1.12, y, txt.replace(".", ",") if lang == "es" else txt, va="center",
                fontsize=10.5, color=INK, fontweight="bold" if key == "gpu01" else "normal")
    ax.set_yticks(range(len(vals)), labels)
    ax.tick_params(axis="y", length=0)
    ax.axvline(1, color=INK_2, lw=1, ls=(0, (3, 3)))
    ax.text(1.06, len(vals) - 0.5, t["realtime"], color=INK_2, fontsize=9.5, va="bottom")
    ax.xaxis.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    ax.spines["left"].set_visible(False)
    ax.set_xticks([0.01, 0.1, 1, 10, 100], ["0,01×", "0,1×", "1×", "10×", "100×"] if lang == "es"
                  else ["0.01×", "0.1×", "1×", "10×", "100×"])
    _title(fig, t["speed_title"], t["speed_sub"])
    return fig


def scatter(lang, name="LPLC2_LC4"):
    import matplotlib.pyplot as plt
    _style(plt)
    t = TEXT[lang]
    z = np.load(HERE / "data" / f"validation_{name}.npz")
    a, b = z["original"], z["ours"]
    fig, ax = plt.subplots(figsize=(6.4, 6.2), dpi=200)
    fig.subplots_adjust(left=0.14, right=0.96, top=0.84, bottom=0.11)
    lim = max(a.max(), b.max()) * 1.05
    ax.plot([0, lim], [0, lim], color=MUTED, lw=1, ls=(0, (3, 3)), zorder=1)
    ax.scatter(a, b, s=9, color=BLUE, alpha=0.45, linewidths=0, zorder=2)
    ax.set_xlim(0, lim)
    ax.set_ylim(0, lim)
    ax.set_xlabel(t["scatter_x"])
    ax.set_ylabel(t["scatter_y"])
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    r, s = float(z["r"]), float(z["slope"])
    if lang == "es":  # coma decimal y punto de miles
        n = f"{len(a):,}".replace(",", ".")
        sub = (f"r = {r:.3f} · pendiente {s:.3f}".replace(".", ",")
               + f" · {n} neuronas activas")
    else:
        sub = f"r = {r:.3f} · slope {s:.3f} · {len(a):,} active neurons"
    _title(fig, t["scatter_title"], sub)
    return fig


def raster(lang, onset=150.0, total=500.0):
    """Espigas de las entradas y de las descendentes al encender el looming en ambos ojos."""
    import matplotlib.pyplot as plt
    from connectome import Connectome
    from lif import Params, make_brain
    _style(plt)
    t = TEXT[lang]
    c = Connectome.load()
    groups = {k: c.find(k) for k in ("LPLC2", "LC4", "DNp01", "MDN", "DNp09")}
    stim = np.concatenate([groups["LPLC2"], groups["LC4"]])
    p = Params(dt=0.1)
    b = make_brain(c, p, seed=0)
    events = {k: ([], []) for k in groups}
    for step in range(int(total / p.dt)):
        now = step * p.dt
        if abs(now - onset) < p.dt / 2:
            b.stimulate((stim, 150.0))
        counts = b.run(p.dt)
        for k, idx in groups.items():
            fired = np.flatnonzero(counts[idx])
            events[k][0].extend([now] * len(fired))
            events[k][1].extend(fired)
    fig, ax = plt.subplots(figsize=(9, 5.2), dpi=200)
    fig.subplots_adjust(left=0.22, right=0.97, top=0.86, bottom=0.11)
    y0, ticks = 0, []
    # Las entradas tienen ~100 neuronas; las descendentes, 1-4: se reparten filas iguales por grupo
    rows = 40
    for k, idx in groups.items():
        times, who = events[k]
        n = max(1, len(idx))
        yy = y0 + (np.asarray(who) % rows if n > rows else np.asarray(who) * rows / n)
        ax.scatter(times, yy, s=2.5, color=INK, linewidths=0)
        ticks.append(y0 + rows / 2)
        y0 += rows + 8
        ax.axhline(y0 - 4, color=GRID, lw=0.8)
    ax.axvspan(onset, total, color=BLUE, alpha=0.07, lw=0)
    ax.text(onset + 4, -3, t["raster_on"], color=BLUE, fontsize=9.5, va="top")
    ax.set_yticks(ticks, [t["groups"][k] for k in groups])
    ax.tick_params(axis="y", length=0)
    ax.set_ylim(-4, y0 + 10)
    ax.invert_yaxis()
    ax.set_xlim(0, total)
    ax.set_xlabel(t["raster_x"])
    ax.spines["left"].set_visible(False)
    _title(fig, t["raster_title"])
    return fig


def neuron(lang, n_syn=40, rate=150.0, total=300.0):
    """Dos neuronas con el simulador real: A recibe Poisson y conecta con B (v, g y espigas de B)."""
    import matplotlib.pyplot as plt
    import scipy.sparse as sp
    from types import SimpleNamespace
    from lif import Brain, Params
    _style(plt)
    t = TEXT[lang]
    p = Params()
    c = SimpleNamespace(n=2, weights=sp.csc_matrix(([float(n_syn)], ([1], [0])), shape=(2, 2)))
    b = Brain(c, p, seed=1)
    b.stimulate((np.array([0]), rate))
    steps = int(total / p.dt)
    time = np.arange(1, steps + 1) * p.dt
    v, g, pre, post = np.empty(steps), np.empty(steps), [], []
    for i in range(steps):
        k = b.run(p.dt)
        v[i], g[i] = b.v[1], b.g[1]
        if k[0]:
            pre.append(time[i])
        if k[1]:
            post.append(time[i])
            v[i] = p.v_th + 8  # el reset ocurre en el mismo paso: se dibuja la espiga
    fig, (a0, a1, a2) = plt.subplots(3, 1, figsize=(9, 5.6), dpi=200, sharex=True,
                                     gridspec_kw={"height_ratios": [0.5, 1.2, 2]})
    fig.subplots_adjust(left=0.12, right=0.97, top=0.84, bottom=0.1, hspace=0.18)
    a0.vlines(pre, 0, 1, color=INK, lw=1)
    a0.set_yticks([])
    a0.set_ylabel(t["neuron_pre"], rotation=0, ha="right", va="center")
    a0.spines["left"].set_visible(False)
    a1.plot(time, g, color=BLUE_LIGHT, lw=1.2)
    a1.set_ylabel(t["neuron_g"])
    a2.plot(time, v, color=BLUE, lw=1.2)
    a2.axhline(p.v_th, color=INK_2, lw=1, ls=(0, (3, 3)))
    a2.axhline(p.v_rst, color=MUTED, lw=0.8)
    a2.text(total, p.v_th + 0.3, t["neuron_th"], color=INK_2, fontsize=9, ha="right", va="bottom")
    a2.text(total, p.v_rst + 0.3, t["neuron_rst"], color=MUTED, fontsize=9, ha="right", va="bottom")
    if post:
        a2.text(post[0] + 2, p.v_th + 7, t["neuron_spike"], color=BLUE, fontsize=9, va="top")
    a2.set_ylim(p.v_rst - 1, p.v_th + 9)
    a2.set_ylabel(t["neuron_v"])
    a2.set_xlabel(t["neuron_x"])
    a2.set_xlim(0, total)
    for a in (a1, a2):
        a.grid(True, axis="y", color=GRID, lw=0.8)
        a.set_axisbelow(True)
    _title(fig, t["neuron_title"], t["neuron_sub"])
    return fig


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("which", choices=["speed", "scatter", "raster", "neuron", "all"])
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    for name, fn in (("speed", speed), ("scatter", scatter), ("raster", raster), ("neuron", neuron)):
        if args.which in (name, "all"):
            for lang in ("es", "en"):
                fig = fn(lang)
                path = OUT / f"brain_{name}_{lang}.png"
                fig.savefig(path)
                print(f"Guardado {path}")


if __name__ == "__main__":
    main()
