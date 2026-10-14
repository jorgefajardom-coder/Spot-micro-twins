"""Descarga la morfología (esqueleto) de cada neurona de FlyWire 783 y la simplifica.

Fuente: esqueletos precomputados que publica el grupo de Jefferis (MRC LMB), los
mismos que usa fafbseg.flywire.get_skeletons (formato binario de neuroglancer).
Schlegel et al. 2024 / Dorkenwald et al. 2024, CC BY 4.0.

Cada esqueleto (~1.700 vértices) se reduce agrupando vértices en celdas de
--cell µm y se guarda por bloques en data/skeletons/, así que se puede cortar y
reanudar. Al final se juntan en data/skeletons_lite.npz, más simplificado
(--view-cell µm) para que el visor lo anime a tiempo real:

    python download_skeletons.py              # todas (~1 h, ~6 GB de tráfico)
    python download_skeletons.py --limit 5000 # una muestra para probar
    python download_skeletons.py --merge-only --view-cell 10
"""
import argparse
import sys
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from connectome import DATA, Connectome

BASE = "https://flyem.mrc-lmb.cam.ac.uk/flyconnectome/flywire_skeletons_783/"
PARTS = DATA / "skeletons"
OUT = DATA / "skeletons_lite.npz"
CHUNK = 2000


def fetch(root_id, retries=4):
    for k in range(retries):
        try:
            with urllib.request.urlopen(BASE + str(root_id), timeout=60) as r:
                return r.read()
        except Exception:
            if k == retries - 1:
                return None
            time.sleep(1.5 * (k + 1))


def parse(buf):
    """Formato neuroglancer: nv, ne (uint32), vértices (float32 nm), aristas (uint32)."""
    nv, ne = np.frombuffer(buf, np.uint32, 2)
    v = np.frombuffer(buf, np.float32, nv * 3, 8).reshape(-1, 3)
    e = np.frombuffer(buf, np.uint32, ne * 2, 8 + nv * 12).reshape(-1, 2)
    return v, e


def simplify(v_nm, e, cell_um):
    """Agrupa vértices en celdas de cell_um y conserva las aristas entre celdas distintas."""
    v = v_nm / 1000.0  # µm
    key = np.floor(v / cell_um).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    n = inv.max() + 1
    sv = np.zeros((n, 3))
    np.add.at(sv, inv, v)
    sv /= np.bincount(inv, minlength=n)[:, None]
    se = inv[e]
    se = se[se[:, 0] != se[:, 1]]
    se = np.unique(np.sort(se, axis=1), axis=0)
    return sv.astype(np.float32), se.astype(np.int32)


def download_chunk(k, root_ids, cell_um, threads):
    path = PARTS / f"part_{k:03d}.npz"
    if path.exists():
        return 0
    with ThreadPoolExecutor(threads) as ex:
        bufs = list(ex.map(fetch, root_ids))
    verts, segs, vowner, missing = [], [], [], 0
    offset = 0
    for j, buf in enumerate(bufs):
        if not buf:
            missing += 1
            continue
        v, e = simplify(*parse(buf), cell_um)
        verts.append(v)
        segs.append(e + offset)
        vowner.append(np.full(len(v), k * CHUNK + j, np.int32))
        offset += len(v)
    np.savez(path, verts=np.concatenate(verts) if verts else np.zeros((0, 3), np.float32),
             segs=np.concatenate(segs) if segs else np.zeros((0, 2), np.int32),
             vowner=np.concatenate(vowner) if vowner else np.zeros(0, np.int32))
    return missing


def merge(view_cell):
    """Junta los bloques y simplifica todas las neuronas a la vez (vectorizado):
    agrupa los vértices de cada neurona en celdas de view_cell µm."""
    verts, segs, vowner, offset = [], [], [], 0
    for path in sorted(PARTS.glob("part_*.npz")):
        z = np.load(path)
        verts.append(z["verts"])
        segs.append(z["segs"] + offset)
        vowner.append(z["vowner"])
        offset += len(z["verts"])
    v, e, o = np.concatenate(verts), np.concatenate(segs), np.concatenate(vowner)
    key = np.column_stack([o, np.floor(v / view_cell).astype(np.int32)])
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    n = inv.max() + 1
    cnt = np.bincount(inv, minlength=n)[:, None]
    sv = np.column_stack([np.bincount(inv, v[:, k], minlength=n) for k in range(3)]) / cnt
    so = np.zeros(n, np.int32)
    so[inv] = o
    se = inv[e]
    se = se[se[:, 0] != se[:, 1]]
    se = np.unique(np.sort(se, axis=1), axis=0)
    np.savez_compressed(OUT, verts=sv.astype(np.float32), segs=se.astype(np.int32),
                        owner=so[se[:, 0]], view_cell=view_cell)
    return n, len(se)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--limit", type=int, help="solo las primeras N neuronas (prueba)")
    ap.add_argument("--cell", type=float, default=3.0, help="tamaño de celda de simplificación (µm)")
    ap.add_argument("--view-cell", type=float, default=10.0, help="celda para el visor (µm)")
    ap.add_argument("--threads", type=int, default=48)
    ap.add_argument("--merge-only", action="store_true", help="solo rehacer skeletons_lite.npz")
    args = ap.parse_args()
    if args.merge_only:
        nv, ns = merge(args.view_cell)
        print(f"Guardado {OUT}: {nv:,} vértices, {ns:,} segmentos ({OUT.stat().st_size / 1e6:.0f} MB)")
        return

    c = Connectome.load()
    root_ids = c.root_ids[:args.limit] if args.limit else c.root_ids
    PARTS.mkdir(parents=True, exist_ok=True)
    n_chunks = -(-len(root_ids) // CHUNK)
    t0, missing = time.time(), 0
    for k in range(n_chunks):
        missing += download_chunk(k, root_ids[k * CHUNK:(k + 1) * CHUNK], args.cell, args.threads)
        done = min((k + 1) * CHUNK, len(root_ids))
        el = time.time() - t0
        print(f"  {done}/{len(root_ids)} neuronas · {el / 60:.1f} min · sin esqueleto {missing}",
              flush=True)
    nv, ns = merge(args.view_cell)
    print(f"Guardado {OUT}: {nv:,} vértices, {ns:,} segmentos ({OUT.stat().st_size / 1e6:.0f} MB)")


if __name__ == "__main__":
    sys.exit(main())
