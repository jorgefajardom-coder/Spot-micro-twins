"""Convierte el STL completo del SpotMicro (exportado de Fusion) en piezas por hueso.

    python models/build_model.py "C:/ruta/SPOT MICRO.stl"

Pasos:
  1. Lee el STL binario con sus colores por cara (formato COLOR= de Fusion/Materialise).
  2. Lo pasa al marco del robot (x adelante, y izquierda, z arriba, metros, suelo en z=0).
  3. Separa los cuerpos sólidos y asigna cada uno al hueso más cercano del esqueleto
     calculado con la IK en la pose del CAD (pose de referencia).
  4. Guarda cada grupo en coordenadas locales de su hueso, simplificado, en spot_model.npz.

Si el CAD cambia, basta con volver a exportar el STL y correr este script.
"""

import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from spot_model3d import solve_skeleton  # noqa: E402

OUT = os.path.join(HERE, "spot_model.npz")

# --- CAD -> robot -----------------------------------------------------------------
# En el ensamble de Fusion: +Y = adelante, -X = izquierda, Z = arriba, milímetros.
CAD_BODY_CENTER = np.array([150.6, 150.15])  # centro del cuerpo en (x, y) del CAD
CAD_FLOOR_Z = 236.3                          # z del punto más bajo de los pies

# Presupuesto de caras (matplotlib dibuja fluido hasta ~6-8k caras por robot)
FACES_BODY = 2600
FACES_BONE = {"shoulder": 260, "upper": 420, "lower": 460}
MIN_PART_SIZE = 0.006  # m: descarta tornillería y piezas diminutas


def load_colored_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack_from("<I", data, 80)[0]
    if 84 + 50 * n != len(data):
        raise ValueError("Se esperaba un STL binario")
    rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)),
                                              ("a", "<u2")]), count=n, offset=84)
    tris = rec["v"].astype(np.float64)
    a = rec["a"].astype(np.int64)
    colors = np.stack([a & 31, (a >> 5) & 31, (a >> 10) & 31], 1) / 31.0
    colors[(a >> 15) == 1] = (0.9, 0.68, 0.13)  # sin color válido -> amarillo
    return tris, colors


def cad_to_robot(p):
    x = p[..., 1] - CAD_BODY_CENTER[1]
    y = -(p[..., 0] - CAD_BODY_CENTER[0])
    z = p[..., 2] - CAD_FLOOR_Z
    return np.stack([x, y, z], -1) / 1000.0


def components(tris):
    """Etiqueta de componente conexo por cara (vértices compartidos)."""
    verts, inv = np.unique(np.round(tris.reshape(-1, 3), 7), axis=0, return_inverse=True)
    faces = inv.reshape(-1, 3)
    lab = np.arange(len(verts))
    while True:
        fl = lab[faces].min(1)
        new = lab.copy()
        for k in range(3):
            np.minimum.at(new, faces[:, k], fl)
        new = new[new]
        if np.array_equal(new, lab):
            break
        lab = new
    return lab[faces[:, 0]]


def reference_pose(tris, comp):
    """Pose del CAD: cuerpo (x, y, z de j1) y pies (centro de cada pie)."""
    feet = []
    cents = {}
    for c in np.unique(comp):
        v = tris[comp == c].reshape(-1, 3)
        cents[c] = (v.min(0), v.max(0))
    # Los pies: componentes cuyo punto más bajo toca el suelo
    for c, (lo, hi) in cents.items():
        if lo[2] < 0.004 and (hi - lo).max() > 0.015:
            feet.append((lo + hi) / 2)
    feet = np.array(feet)
    if len(feet) != 4:
        raise RuntimeError(f"Se esperaban 4 pies tocando el suelo, hay {len(feet)}")
    order = [np.argmin(np.linalg.norm(feet[:, :2] - np.array(t), axis=1))
             for t in ((1, 1), (-1, 1), (-1, -1), (1, -1))]  # LF, LB, RB, RF
    return feet[order]


def seg_dist(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0, 1)
    return np.linalg.norm(p - (a + t * ab))


def simplify_colored(v, f, cols, max_faces):
    """Vertex clustering conservando el color de cada cara."""
    if len(f) <= max_faces:
        return v, f, cols
    size = np.ptp(v, axis=0).max()
    lo, hi = size / 2000, size / 6
    best = None
    for _ in range(22):
        cell = (lo + hi) / 2
        keys = np.floor((v - v.min(0)) / cell).astype(np.int64)
        _, cl, cnt = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
        cl = cl.ravel()
        nv = np.zeros((cnt.size, 3))
        np.add.at(nv, cl, v)
        nv /= cnt[:, None]
        nf = cl[f]
        keep = (nf[:, 0] != nf[:, 1]) & (nf[:, 1] != nf[:, 2]) & (nf[:, 0] != nf[:, 2])
        nf, nc = nf[keep], cols[keep]
        _, idx = np.unique(np.sort(nf, axis=1), axis=0, return_index=True)
        nf, nc = nf[idx], nc[idx]
        if len(nf) > max_faces:
            lo = cell
        else:
            hi, best = cell, (nv, nf, nc)
    return best if best is not None else (v, f, cols)


def pack(tris_local, cols, max_faces):
    v, inv = np.unique(np.round(tris_local.reshape(-1, 3), 6), axis=0, return_inverse=True)
    f = inv.reshape(-1, 3)
    v, f, c = simplify_colored(v, f, cols, max_faces)
    used, remap = np.unique(f, return_inverse=True)
    return v[used].astype(np.float32), remap.reshape(-1, 3).astype(np.int32), (c * 255).astype(np.uint8)


def main(stl_path):
    from spotmicro_gui import Kinematics

    print("Leyendo", stl_path)
    tris_cad, cols = load_colored_stl(stl_path)
    tris = cad_to_robot(tris_cad)
    comp = components(tris_cad)
    ids = np.unique(comp)
    print(f"{len(tris)} caras, {len(ids)} cuerpos")

    kin = Kinematics()
    feet = reference_pose(tris, comp)
    body_z = tris[..., 2].max() - 0.035  # ajustado abajo con los hombros
    # Altura del eje del hombro: centro de los bloques de hombro (a ±~48 mm laterales)
    shoulder_z = []
    for c in ids:
        v = tris[comp == c].reshape(-1, 3)
        ctr = (v.min(0) + v.max(0)) / 2
        if 0.040 < abs(ctr[1]) < 0.056 and abs(abs(ctr[0]) - 0.093) < 0.01:
            shoulder_z.append(ctr[2])
    body_z = float(np.median(shoulder_z)) if shoulder_z else body_z
    skel = solve_skeleton(kin, feet.tolist(), [0, 0, 0], [0, 0, body_z])
    if not skel["ok"]:
        raise RuntimeError("La IK no alcanza la pose del CAD")
    print("Pose de referencia: cuerpo z=%.3f  pies=%s" % (body_z, np.round(feet, 3).tolist()))

    groups = {"body": []}
    for leg in range(4):
        for bone in ("shoulder", "upper", "lower"):
            groups[f"{leg}_{bone}"] = []

    half_w = kin.width / 2 + 0.004
    dropped = 0
    for c in ids:
        sel = np.where(comp == c)[0]
        v = tris[sel].reshape(-1, 3)
        lo, hi = v.min(0), v.max(0)
        if (hi - lo).max() < MIN_PART_SIZE:
            dropped += len(sel)
            continue
        ctr = (lo + hi) / 2
        if abs(ctr[1]) < half_w + 0.004 and ctr[2] > 0.12:
            groups["body"].append(sel)
            continue
        best, key = 1e9, "body"
        for leg, bones in enumerate(skel["legs"]):
            j1, j2, j3, j4 = bones["joints"]
            for bone, (a, b) in (("shoulder", (j1, j2)), ("upper", (j2, j3)), ("lower", (j3, j4))):
                d = seg_dist(ctr, a, b)
                if d < best:
                    best, key = d, f"{leg}_{bone}"
        groups[key].append(sel)

    out = {}
    for key, sels in groups.items():
        if not sels:
            continue
        sel = np.concatenate(sels)
        if key == "body":
            r, p = skel["body"]
            budget = FACES_BODY
        else:
            leg, bone = key.split("_")
            r, p = skel["legs"][int(leg)][bone]
            budget = FACES_BONE[bone]
        local = (tris[sel] - p) @ r  # mundo -> local del hueso
        v, f, c = pack(local, cols[sel], budget)
        out[f"{key}_v"], out[f"{key}_f"], out[f"{key}_c"] = v, f, c
        print(f"  {key:12s} {len(sel):7d} -> {len(f):5d} caras")
    print(f"  descartadas {dropped} caras de piezas < {MIN_PART_SIZE * 1000:.0f} mm")
    np.savez_compressed(OUT, **out)
    print("Guardado", OUT, f"({os.path.getsize(OUT) / 1024:.0f} KB)")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1])
