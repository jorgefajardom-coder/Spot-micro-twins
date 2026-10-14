"""Experimentos con el cerebro de mosca: estimular un tipo celular y ver qué neuronas
descendentes (las que bajan al cordón nervioso y mueven el cuerpo) responden.

    python probe.py LPLC2 LC4                 # looming en ambos ojos
    python probe.py LPLC2 --side left         # solo el lado izquierdo
    python probe.py --scan                    # recorre todos los tipos sensoriales/visuales
                                              # y guarda data/scan.csv

Usa la GPU si hay CUDA (dt = 0.1 ms, el del artículo); en la CPU, dt = 0.5 ms para ir
a tiempo real (<5 % de diferencia con 0.1 ms en estas pruebas).
"""
import argparse
import sys
import time

import numpy as np
import pandas as pd

from connectome import DATA, Connectome
from lif import Params, make_brain

# Descendentes con función motora conocida (Namiki 2018, Bidaye 2014/2020, Cande 2018…)
MOTOR_DNS = {
    'DNp01': 'Giant Fiber: escape (salto)',
    'DNp09': 'P9: caminar hacia delante',
    'DNa02': 'giro (lado ipsilateral)',
    'DNa01': 'giro / dirección',
    'MDN':   'moonwalker: caminar hacia atrás',
    'DNp02': 'respuesta a looming',
    'DNp04': 'respuesta a looming',
    'DNp11': 'respuesta a looming',
    'DNg11': 'aseo de patas delanteras',
}


def response(brain, stim, ms=500, hz=150):
    """Frecuencia (Hz) de cada neurona al estimular `stim` (índices) desde el reposo."""
    b = brain
    b.reset()
    b.stimulate((stim, hz))
    return b.run(ms) * 1000.0 / ms


def summarize(c, rates, top=12):
    a = c.annotations
    dn = np.flatnonzero((a.super_class == 'descending').to_numpy())
    rows = []
    for t in MOTOR_DNS:
        for side in ('left', 'right'):
            idx = c.find(t, side)
            if len(idx):
                rows.append((t, side, rates[idx].mean(), MOTOR_DNS[t]))
    print("\nDescendentes motoras conocidas:")
    for t, side, r, desc in rows:
        bar = '#' * int(min(r, 300) / 10)
        print(f"  {t:>6} {side:>5} {r:6.0f} Hz  {bar:<30} {desc}")
    print(f"\nTop {top} descendentes:")
    for i in dn[np.argsort(-rates[dn])][:top]:
        if rates[i] > 0:
            print(f"  {a.cell_type[i]!s:>8} {a.side[i]!s:>5} {rates[i]:6.0f} Hz")


def scan(c, brain, ms=300, min_neurons=2):
    """Estimula cada tipo sensorial/visual por lado y guarda la respuesta de las DN motoras."""
    a = c.annotations
    sensory = a[a.super_class.isin(['sensory', 'visual_projection', 'sensory_ascending'])]
    keys = sensory.groupby(['cell_type', 'side']).size()
    keys = keys[keys >= min_neurons]
    dn_idx = {(t, s): c.find(t, s) for t in MOTOR_DNS for s in ('left', 'right')}
    dn_idx = {k: v for k, v in dn_idx.items() if len(v)}
    out = []
    t0 = time.time()
    for n, ((cell_type, side), size) in enumerate(keys.items()):
        stim = c.find(cell_type, side)
        r = response(brain, stim, ms=ms)
        row = {'cell_type': cell_type, 'side': side, 'n': size,
               'super_class': sensory.loc[sensory.cell_type == cell_type, 'super_class'].iloc[0]}
        row.update({f'{t}_{s}': r[i].mean() for (t, s), i in dn_idx.items()})
        out.append(row)
        if n % 25 == 0:
            print(f"  {n}/{len(keys)} tipos · {time.time() - t0:.0f} s", file=sys.stderr)
    df = pd.DataFrame(out)
    df.to_csv(DATA / 'scan.csv', index=False)
    return df


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('cell_types', nargs='*', help="tipos celulares a estimular (FlyWire cell_type)")
    ap.add_argument('--side', choices=['left', 'right'])
    ap.add_argument('--hz', type=float, default=150)
    ap.add_argument('--ms', type=float, default=500)
    ap.add_argument('--dt', type=float, help="ms (por defecto 0.1 en GPU, 0.5 en CPU)")
    ap.add_argument('--device', choices=['auto', 'gpu', 'cpu'], default='auto')
    ap.add_argument('--scan', action='store_true')
    args = ap.parse_args()

    import lif_gpu
    gpu = args.device != 'cpu' and lif_gpu.available()
    c = Connectome.load()
    params = Params(dt=args.dt or (0.1 if gpu else 0.5))
    print(f"{'GPU' if gpu else 'CPU'} · dt = {params.dt} ms")
    brain = make_brain(c, params, seed=0, device=args.device)
    if args.scan:
        df = scan(c, brain)
        print(f"Guardado {DATA / 'scan.csv'} ({len(df)} estímulos)")
        return
    if not args.cell_types:
        ap.error("indica tipos celulares o --scan")
    stim = np.concatenate([c.find(t, args.side) for t in args.cell_types])
    if not len(stim):
        sys.exit(f"No hay neuronas de tipo {args.cell_types} ({args.side or 'ambos lados'})")
    print(f"Estimulando {len(stim)} neuronas {args.cell_types} {args.side or ''} a {args.hz:.0f} Hz "
          f"durante {args.ms:.0f} ms")
    summarize(c, response(brain, stim, args.ms, args.hz))


if __name__ == '__main__':
    main()
