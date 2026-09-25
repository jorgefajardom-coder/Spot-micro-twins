"""Modelo 3D del SpotMicro pegado al esqueleto de la cinemática.

- El esqueleto sale de `Kinematics.leg_IK` (hombro j1, cadera j2, rodilla j3, pie j4).
- Cada hueso tiene un marco local; cada pieza (STL o provisional) se pega a su marco.
- Las piezas se configuran en models/spot_parts.json. Si falta un STL se usa una
  pieza provisional con las medidas del robot, así la vista funciona siempre.

Marcos locales (todos dextrógiros, y = hacia la izquierda del robot):
  body      origen en el centro del cuerpo, x adelante, z arriba
  shoulder  origen en j1 (eje del hombro), y hacia afuera en patas izquierdas
  upper     origen en j2 (cadera), la pieza cuelga hacia -z hasta la rodilla
  lower     origen en j3 (rodilla), la pieza cuelga hacia -z hasta el pie
  foot      origen en j4, misma orientación que lower
"""

import json
import os
import struct
from math import cos, radians, sin

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PARTS_FILE = os.path.join(HERE, "models", "spot_parts.json")

LEG_NAMES = ["LF", "LB", "RB", "RF"]  # mismo orden que Kinematics
RIGHT_LEGS = (2, 3)
BONES = ["shoulder", "upper", "lower", "foot"]

YELLOW = (0.96, 0.71, 0.0)
BLACK = (0.16, 0.16, 0.17)
GREY = (0.45, 0.46, 0.48)


# =============================================================================
# Utilidades de rotación (mismas convenciones que spotmicro_gui.RotMatrix3D)
# =============================================================================

def rot_zyx(roll, pitch, yaw):
    """Rz(yaw) @ Ry(pitch) @ Rx(roll), ángulos en radianes."""
    cr, sr, cp, sp, cy, sy = cos(roll), sin(roll), cos(pitch), sin(pitch), cos(yaw), sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def _frame(y_hint, z_axis):
    """Marco ortonormal con z exacto e y lo más cercano posible a y_hint."""
    z = _unit(z_axis)
    y = _unit(y_hint - np.dot(y_hint, z) * z)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


# =============================================================================
# Esqueleto
# =============================================================================

def solve_skeleton(kin, foot_targets, rot, body_pos, is_radians=False):
    """Resuelve la IK y devuelve marcos de cada hueso en coordenadas mundo.

    Retorna dict con:
      ok:      False si alguna pata no alcanza su objetivo
      angles:  lista 4x3 (rad) igual que plot_robot
      body:    (R, p)
      legs:    lista de 4 dicts {bone: (R, p)} + 'joints': array 4x3 (j1..j4)
    """
    rpy = rot if is_radians else [radians(a) for a in rot]
    r_body = rot_zyx(*rpy)
    p_body = np.asarray(body_pos, dtype=float)
    body_x, body_y = r_body[:, 0], r_body[:, 1]

    out = {"ok": True, "angles": [], "body": (r_body, p_body), "legs": []}
    for leg in range(4):
        res = kin.leg_IK(foot_targets[leg], rot, leg, is_radians, body_pos)
        if any(np.isnan(res[:3])):
            out["ok"] = False
            out["angles"].append([float("nan")] * 3)
            out["legs"].append(None)
            continue

        origin = kin.leg_origins[leg]
        joints = np.array([r_body @ (np.asarray(j) + origin) + p_body for j in res[3:6]]
                          + [np.asarray(foot_targets[leg], dtype=float)])
        j1, j2, j3, j4 = joints

        # Eje lateral (hacia la izquierda del robot) según la dirección j1 -> j2
        side = 1.0 if leg not in RIGHT_LEGS else -1.0
        lateral = _unit(j2 - j1) * side
        shoulder_r = _frame(lateral, np.cross(body_x, lateral))
        upper_r = _frame(lateral, j2 - j3)
        lower_r = _frame(lateral, j3 - j4)

        out["angles"].append(list(res[:3]))
        out["legs"].append({
            "joints": joints,
            "shoulder": (shoulder_r, j1),
            "upper": (upper_r, j2),
            "lower": (lower_r, j3),
            "foot": (lower_r, j4),
        })
    return out


# =============================================================================
# Mallas
# =============================================================================

def load_stl(path):
    """Carga STL binario o ASCII -> (V, F) con vértices únicos."""
    with open(path, "rb") as f:
        data = f.read()
    tris = None
    if len(data) >= 84:
        n = struct.unpack_from("<I", data, 80)[0]
        if 84 + n * 50 == len(data):
            rec = np.frombuffer(data, dtype=np.dtype([
                ("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)
            tris = rec["v"].astype(float)
    if tris is None:  # ASCII
        vals = [line.split()[1:4] for line in data.decode("utf-8", "ignore").splitlines()
                if line.strip().startswith("vertex")]
        tris = np.array(vals, dtype=float).reshape(-1, 3, 3)
    verts, inv = np.unique(tris.reshape(-1, 3), axis=0, return_inverse=True)
    return verts, inv.reshape(-1, 3)


def simplify(verts, faces, max_faces):
    """Reduce caras por agrupamiento de vértices en una rejilla (vertex clustering)."""
    if len(faces) <= max_faces:
        return verts, faces
    size = np.ptp(verts, axis=0).max()
    lo, hi = size / 400, size / 4
    best = (verts, faces)
    for _ in range(18):  # búsqueda binaria del tamaño de celda
        cell = (lo + hi) / 2
        keys = np.floor((verts - verts.min(0)) / cell).astype(np.int64)
        _, cluster, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
        cluster = cluster.ravel()
        new_v = np.zeros((counts.size, 3))
        np.add.at(new_v, cluster, verts)
        new_v /= counts[:, None]
        f = cluster[faces]
        f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
        f = np.unique(np.sort(f, axis=1), axis=0) if len(f) else f
        if len(f) > max_faces:
            lo = cell
        else:
            hi, best = cell, (new_v, f)
    return best


def box(x0, x1, y0, y1, z0, z1, taper=1.0):
    """Caja (opcionalmente estrechada en z0) como (V, F)."""
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    v = []
    for z in (z0, z1):
        k = taper if z == z0 else 1.0
        for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            v.append((cx + (x - cx) * k, cy + (y - cy) * k, z))
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
         (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return np.array(v, float), np.array(f)


def merge(*meshes):
    vs, fs, off = [], [], 0
    for v, f in meshes:
        vs.append(v)
        fs.append(f + off)
        off += len(v)
    return np.vstack(vs), np.vstack(fs)


def placeholder_parts(kin):
    """Piezas provisionales con las medidas reales (lado izquierdo)."""
    l1, l2, l3 = kin.link_1, kin.link_2, kin.link_3
    body_len, body_w = kin.length + 0.075, kin.width + 0.035
    return {
        "body": [
            (merge(box(-body_len / 2, body_len / 2, -body_w / 2, body_w / 2, -0.028, 0.030),
                   box(body_len / 2 - 0.004, body_len / 2 + 0.018, -body_w / 2 + 0.008,
                       body_w / 2 - 0.008, -0.022, 0.024, taper=0.8)), YELLOW),
            (box(-body_len / 2 + 0.035, body_len / 2 - 0.035, -body_w / 2 - 0.002,
                 body_w / 2 + 0.002, -0.034, -0.026), BLACK),
        ],
        "shoulder": [(box(-0.026, 0.026, -0.006, l1 - 0.012, -0.024, 0.024), BLACK)],
        "upper": [(box(-0.021, 0.021, -0.010, 0.020, -l2 - 0.006, 0.016), YELLOW)],
        "lower": [(box(-0.013, 0.013, -0.009, 0.013, -l3 + 0.010, 0.010, taper=0.55), BLACK)],
        "foot": [(box(-0.014, 0.014, -0.011, 0.015, -0.012, 0.012), YELLOW)],
    }


def _mirror_y(v, f):
    return v * np.array([1.0, -1.0, 1.0]), f[:, ::-1]


class SpotModel:
    """Conjunto de piezas por hueso, en coordenadas locales (metros)."""

    def __init__(self, kin, parts_file=DEFAULT_PARTS_FILE, max_faces_per_part=500):
        self.kin = kin
        self.max_faces = max_faces_per_part
        self.parts = placeholder_parts(kin)  # {bone: [(V, F, color)]} lado izquierdo
        self.right = {}                      # piezas específicas del lado derecho
        self.using_stl = []
        self._load_config(parts_file)

    def _load_config(self, parts_file):
        if not os.path.exists(parts_file):
            return
        with open(parts_file, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        base = os.path.dirname(parts_file)
        for key, entries in cfg.get("parts", {}).items():
            meshes = []
            for e in entries:
                path = os.path.join(base, e["file"])
                if not os.path.exists(path):
                    meshes = None
                    break
                v, f = load_stl(path)
                v, f = simplify(v, f, e.get("max_faces", self.max_faces))
                v = (v - np.asarray(e.get("origin", [0, 0, 0]))) * e.get("scale", 0.001)
                v = v @ rot_zyx(*[radians(a) for a in e.get("rotation_deg", [0, 0, 0])]).T
                v = v + np.asarray(e.get("offset", [0, 0, 0]))
                meshes.append((v, f, tuple(e.get("color", YELLOW))))
            if meshes:  # solo se reemplaza la pieza si están todos sus archivos
                bone, _, side = key.partition("_")
                (self.right if side == "right" else self.parts)[bone] = meshes
                self.using_stl.append(key)

    def meshes_for(self, bone, right_side):
        if right_side and bone in self.right:
            return self.right[bone]
        meshes = self.parts[bone]
        if right_side:
            return [(*_mirror_y(m[0], m[1]), m[2]) for m in (_to3(p) for p in meshes)]
        return [_to3(p) for p in meshes]

    def world_triangles(self, skel, offset=(0, 0, 0), include_body=True):
        """Triángulos en mundo (N,3,3) y color base (N,3) para un esqueleto."""
        tris, cols = [], []
        off = np.asarray(offset, float)

        def add(meshes, r, p):
            for v, f, c in meshes:
                w = v @ r.T + p + off
                tris.append(w[f])
                cols.append(np.tile(c, (len(f), 1)))

        if include_body:
            add(self.meshes_for("body", False), *skel["body"])
        for leg, bones in enumerate(skel["legs"]):
            if bones is None:
                continue
            for bone in BONES:
                add(self.meshes_for(bone, leg in RIGHT_LEGS), *bones[bone])
        if not tris:
            return np.zeros((0, 3, 3)), np.zeros((0, 3))
        return np.concatenate(tris), np.concatenate(cols)


def _to3(p):
    """Normaliza (mesh, color) o (V, F, color) a (V, F, color)."""
    if len(p) == 2:
        (v, f), c = p
        return v, f, c
    return p


# =============================================================================
# Render con matplotlib
# =============================================================================

LIGHT = _unit(np.array([0.4, -0.5, 0.8]))


def shade(tris, base_colors, ambient=0.38):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    k = ambient + (1 - ambient) * np.abs(n @ LIGHT)  # iluminación a dos caras
    return np.clip(base_colors * k[:, None], 0, 1)


class SpotScene:
    """Dibuja uno o varios Spots en un Axes3D, actualizando la misma colección."""

    def __init__(self, ax, model, show_model=True, show_skeleton=True, show_floor=True):
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        self.ax, self.model = ax, model
        self.show_model, self.show_skeleton, self.show_floor = show_model, show_skeleton, show_floor
        self.mesh = Poly3DCollection([], linewidths=0.15, edgecolors=(0, 0, 0, 0.25))
        ax.add_collection3d(self.mesh)
        self.lines = []
        self.floor = None
        self._configure_axes()

    def _configure_axes(self):
        ax = self.ax
        ax.set_facecolor((0.08, 0.1, 0.13))
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.set_pane_color((0.1, 0.12, 0.15, 1.0))
            axis.label.set_color("0.75")
        ax.tick_params(colors="0.6", labelsize=7)
        ax.set_box_aspect((1, 1, 0.55))
        ax.view_init(elev=18, azim=-60)

    def draw(self, robots, center=None, span=0.32):
        """robots: lista de (skeleton, offset_xyz, etiqueta)."""
        tris, cols = [], []
        for line in self.lines:
            line.remove()
        self.lines = []

        for skel, off, label in robots:
            off = np.asarray(off, float)
            if self.show_model:
                t, c = self.model.world_triangles(skel, off)
                tris.append(t)
                cols.append(c)
            if self.show_skeleton:
                self._draw_skeleton(skel, off)
            if label:
                p = skel["body"][1] + off
                self.lines.append(self.ax.text(p[0], p[1], p[2] + 0.12, label, color="w",
                                               ha="center", fontsize=9, weight="bold"))

        if tris:
            t, c = np.concatenate(tris), np.concatenate(cols)
            self.mesh.set_verts(t)
            self.mesh.set_facecolor(shade(t, c))
        else:
            self.mesh.set_verts([])

        if center is None:
            offs = np.array([r[1] for r in robots]) if robots else np.zeros((1, 3))
            center = offs.mean(0)
        half = span + np.ptp(np.array([r[1] for r in robots]), axis=0).max() / 2 if robots else span
        cx, cy = center[0], center[1]
        self.ax.set_xlim(cx - half, cx + half)
        self.ax.set_ylim(cy - half, cy + half)
        self.ax.set_zlim(0, 1.1 * half)
        self._draw_floor(cx, cy, half)

    def _draw_skeleton(self, skel, off):
        r_body, p_body = skel["body"]
        corners = (r_body @ self.model.kin.leg_origins.T).T + p_body + off
        loop = np.vstack([corners, corners[:1]])
        self.lines += self.ax.plot(loop[:, 0], loop[:, 1], loop[:, 2], color="#ff5252", lw=1.5)
        for bones in skel["legs"]:
            if bones is None:
                continue
            j = bones["joints"] + off
            self.lines += self.ax.plot(j[:, 0], j[:, 1], j[:, 2], "-o", color="#40c4ff",
                                       lw=1.6, ms=3)

    def _draw_floor(self, cx, cy, half):
        if not self.show_floor:
            return
        g = np.linspace(-half, half, 9)
        for v in g:
            self.lines += self.ax.plot([cx + v, cx + v], [cy - half, cy + half], [0, 0],
                                       color="0.3", lw=0.5)
            self.lines += self.ax.plot([cx - half, cx + half], [cy + v, cy + v], [0, 0],
                                       color="0.3", lw=0.5)
