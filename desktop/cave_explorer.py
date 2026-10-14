"""Cueva reconstruida a partir del escaneo del Spot: modelo 3D completo para explorar,
con el gas y la temperatura medidos integrados en la roca.

- La superficie sale de los puntos del LiDAR (ocupación -> isosuperficie, ver
  reconstruct_surface).
- Gas y temperatura: cada vértice toma la media ponderada por distancia de las mediciones
  reales de los sensores cercanas (hasta INTERP_RADIUS). Donde el robot no midió no se
  inventa nada: queda en gris como "sin datos".
- Se guarda en scans/ como .vtp (con todas las capas) y .ply con colores (Blender, MeshLab).

    fly_brain\\.venv\\Scripts\\python cave_explorer.py scans\\cueva_20261014_1200.vtp

Ratón: girar · rueda: zoom · mayús + arrastrar: desplazar · f: volar al punto bajo el ratón.
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
SCANS = HERE / "scans"
MAP_BG = "#1c1f25"
# Colores del escaneo por altura: suelo turquesa, paredes azules, arriba verde amarillento
SCAN_COLORS = ["#8fe9ee", "#55c3dc", "#3f86c9", "#3a5fa8", "#4a7f8c", "#86a85a", "#d2d36a"]
GAS_COLORS = ["#16324a", "#1f6f5c", "#3fae49", "#b5e550", "#fff27a", "#ff9f43", "#ff4d4d"]
NAN_COLOR = "#59616d"
INTERP_RADIUS = 1.1  # m: hasta dónde llega la influencia de una medición


def gradient(colors, name="degradado"):
    """Degradado continuo entre colores (con una lista, PyVista hace una paleta discreta: franjas)."""
    import matplotlib.colors as mcolors
    return mcolors.LinearSegmentedColormap.from_list(name, colors)


LAYERS = {
    "altura": dict(cmap=gradient(SCAN_COLORS), title="Roca (altura, m)"),
    "gas_ppm": dict(cmap=gradient(GAS_COLORS), title="Gas medido (ppm)"),
    "temp_c": dict(cmap="inferno", title="Temperatura medida (°C)"),
}


def reconstruct_surface(pv, points, ground, cut=None, voxel=0.04, sigma=1.25, level=0.11):
    """Superficie continua a partir de los puntos del LiDAR (como un escaneo de cueva).

    Ocupación en una rejilla de vóxeles -> suavizado gaussiano -> isosuperficie (flying edges
    de VTK) -> alisado de Taubin. Con `cut` solo entra lo que está por debajo de esa altura
    sobre el suelo (para ver el interior desde arriba).
    """
    from scipy.ndimage import gaussian_filter
    pts = np.asarray(points, float)
    height = pts[:, 2] - ground(pts[:, 0], pts[:, 1])
    if cut is not None:
        pts = pts[height < cut]
    if len(pts) < 200:
        return None
    lo = pts.min(0) - 3 * voxel
    dims = np.ceil((pts.max(0) + 3 * voxel - lo) / voxel).astype(int) + 1
    occ = np.zeros(dims, np.float32)
    i = np.floor((pts - lo) / voxel).astype(int)
    occ[i[:, 0], i[:, 1], i[:, 2]] = 1.0
    occ = gaussian_filter(occ, sigma)
    img = pv.ImageData(dimensions=dims, spacing=(voxel,) * 3, origin=lo)
    img.point_data["occ"] = occ.ravel(order="F")
    surf = img.contour([level], scalars="occ", method="flying_edges")
    if surf.n_points == 0:
        return None
    surf = surf.smooth_taubin(n_iter=20, pass_band=0.08).compute_normals(auto_orient_normals=False)
    rng = np.random.default_rng(len(pts))
    z = surf.points[:, 2] - ground(surf.points[:, 0], surf.points[:, 1])
    z = z + 0.05 * rng.standard_normal(surf.n_points)  # textura
    surf.point_data["altura"] = np.clip(z, 0, cut) if cut is not None else np.maximum(z, 0)
    return surf


def integrate_sensors(mesh, samples, radius=INTERP_RADIUS, k=12):
    """Gas y temperatura en cada vértice: media ponderada (1/d²) de las mediciones cercanas."""
    from scipy.spatial import cKDTree
    s = np.asarray(samples, float)
    dist, idx = cKDTree(s[:, :3]).query(mesh.points, k=min(k, len(s)), distance_upper_bound=radius)
    dist, idx = np.atleast_2d(dist.T).T, np.atleast_2d(idx.T).T
    valid = np.isfinite(dist)
    w = np.where(valid, 1.0 / (dist ** 2 + 0.02), 0.0)
    total = w.sum(axis=1)
    idx = np.where(valid, idx, 0)
    for name, col in (("gas_ppm", 3), ("temp_c", 4)):
        value = (w * s[idx, col]).sum(axis=1) / np.where(total > 0, total, 1)
        mesh.point_data[name] = np.where(total > 0, value, np.nan)
    return mesh


def build_cave_model(world, voxel=None):
    """Modelo completo (con techo) del escaneo de un CaveWorld, con gas y temperatura."""
    import pyvista as pv
    voxel = voxel or 0.875 * getattr(world, "map_voxel", 0.04)
    mesh = reconstruct_surface(pv, world.map_points, world.ground, cut=None, voxel=voxel, sigma=1.15)
    if mesh is None:
        return None, None
    samples = np.array(world.samples, float)
    if len(samples):
        integrate_sensors(mesh, samples)
    hottest = samples[np.argmax(samples[:, 4])] if len(samples) else None
    meta = {
        "samples": samples.tolist(),
        "gas_found": None if world.gas_found is None else list(map(float, world.gas_found)),
        "hottest": None if hottest is None else list(map(float, hottest)),
        "voxels": int(len(world.map_points)),
        "recorrido_m": float(world.progress),
    }
    return mesh, meta


def save_model(mesh, meta):
    """Guarda .vtp (todas las capas), .json (registro de sensores) y .ply con colores."""
    SCANS.mkdir(exist_ok=True)
    base = SCANS / time.strftime("cueva_%Y%m%d_%H%M%S")
    mesh.save(base.with_suffix(".vtp"))
    base.with_suffix(".json").write_text(json.dumps(meta), encoding="utf-8")
    cmap = gradient(SCAN_COLORS, "scan")
    a = mesh.point_data["altura"]
    rgb = (cmap((a - a.min()) / max(np.ptp(a), 1e-6))[:, :3] * 255).astype(np.uint8)
    export = mesh.copy()
    export.point_data["rgb"] = rgb
    export.save(base.with_suffix(".ply"), texture="rgb")
    return base.with_suffix(".vtp")


class Explorer:
    """Ventana para recorrer la cueva: capas Roca / Gas / Temperatura y corte del techo."""

    def __init__(self, mesh, meta, off_screen=False, size=(1600, 950)):
        import pyvista as pv
        self.pv, self.mesh, self.meta = pv, mesh, meta
        self.layer = "gas_ppm" if "gas_ppm" in mesh.point_data else "altura"
        self.cut = 0.6
        self.ground_z = float(np.percentile(mesh.points[:, 2], 3))
        pv.global_theme.font.family = "arial"
        pl = self.pl = pv.Plotter(window_size=size, off_screen=off_screen,
                                  title="Spot Micro Twins · Cueva reconstruida")
        pl.set_background(MAP_BG)
        b = mesh.bounds
        cx, cy = (b[0] + b[1]) / 2, (b[2] + b[3]) / 2
        span = max(b[1] - b[0], b[3] - b[2])
        grid = pv.Plane(center=(cx, cy, self.ground_z - 0.05), i_size=span * 2, j_size=span * 2,
                        i_resolution=int(span * 4), j_resolution=int(span * 4))
        pl.add_mesh(grid, style="wireframe", color="#2c3038", line_width=1, lighting=False)

        samples = np.array(meta.get("samples") or [], float)
        if len(samples) > 1:
            path = pv.lines_from_points(samples[:, :3] + [0, 0, 0.02])
            path.point_data["gas_ppm"] = samples[:, 3]
            path.point_data["temp_c"] = samples[:, 4]
            self.path = path.tube(radius=0.025)
        else:
            self.path = None
        if meta.get("gas_found"):
            gx, gy = meta["gas_found"]
            ppm = max(samples[:, 3]) if len(samples) else 0
            pl.add_mesh(pv.Sphere(radius=0.09, center=(gx, gy, self.ground_z + 0.3)), color="#7dff6b", ambient=0.6)
            pl.add_point_labels([(gx, gy, self.ground_z + 0.45)], [f"Fuga de gas · {ppm:.0f} ppm"], font_size=14,
                                text_color="w", shape_color="#1f6f5c", shape_opacity=0.85, always_visible=True,
                                show_points=False)
        hot = meta.get("hottest")
        if hot and hot[4] > 30:
            pl.add_point_labels([(hot[0], hot[1], hot[2] + 0.25)], [f"Calor · {hot[4]:.0f} °C máx."],
                                font_size=14, text_color="w", shape_color="#a3361f", shape_opacity=0.85,
                                always_visible=True, show_points=False)

        pl.add_text("CUEVA RECONSTRUIDA", position=(24, size[1] - 56), font_size=20, color="#f4c542")
        pl.add_text(f"{meta.get('voxels', 0):,} vóxeles de LiDAR · {mesh.n_cells:,} triángulos · "
                    f"recorrido {meta.get('recorrido_m', 0):.1f} m · {len(samples)} mediciones".replace(",", "."),
                    position=(26, size[1] - 86), font_size=11, color="#8fa5be")
        pl.add_text("ratón: girar · rueda: zoom · mayús+arrastrar: desplazar · f: volar al punto",
                    position=(26, 16), font_size=10, color="#8fa5be")
        # Capas (botones como radio) y corte del techo
        self.buttons = {}
        for i, (name, label) in enumerate((("altura", "Roca"), ("gas_ppm", "Gas"), ("temp_c", "Temperatura"))):
            y = size[1] - 150 - i * 46
            self.buttons[name] = pl.add_checkbox_button_widget(
                lambda on, n=name: self.set_layer(n), value=name == self.layer, position=(26, y), size=32,
                border_size=2, color_on="#f4c542", color_off="#2a3646", background_color="#8fa5be")
            pl.add_text(label, position=(68, y + 6), font_size=13, color="w")
        pl.add_slider_widget(self.set_cut, [0.2, 1.6], value=self.cut, title="Corte del techo (m)",
                             pointa=(0.62, 0.08), pointb=(0.95, 0.08), color="#8fa5be", style="modern")
        pl.camera.focal_point = (cx, cy, self.ground_z)
        pl.camera.position = (cx - span * 0.55, cy - span * 0.7, self.ground_z + span * 0.75)
        pl.camera.up = (0, 0, 1)
        self.draw()

    def set_layer(self, name):
        self.layer = name
        for n, w in self.buttons.items():
            w.GetRepresentation().SetState(int(n == name))
        self.draw()

    def set_cut(self, value):
        self.cut = float(value)
        self.draw()

    def draw(self):
        pl, cfg = self.pl, LAYERS[self.layer]
        visible = self.mesh.clip("z", origin=(0, 0, self.ground_z + self.cut), invert=True)
        args = dict(scalars=self.layer, cmap=cfg["cmap"], nan_color=NAN_COLOR, smooth_shading=True,
                    ambient=0.25, diffuse=0.85, specular=0.15, specular_power=12, name="cueva")
        if self.layer != "altura":
            # Escala ajustada a lo medido (si no, una zona templada apenas se distingue)
            values = self.mesh.point_data[self.layer]
            lo, hi = np.nanmin(values), np.nanmax(values)
            args["clim"] = [lo, max(hi, lo + (20 if self.layer == "temp_c" else 100))]
        bar = dict(title=cfg["title"], color="w", vertical=True, position_x=0.9, position_y=0.25,
                   height=0.5, width=0.04, title_font_size=14, label_font_size=12, n_labels=5)
        for title in list(pl.scalar_bars.keys()):  # la leyenda de la capa anterior
            pl.remove_scalar_bar(title)
        pl.add_mesh(visible, scalar_bar_args=bar, show_scalar_bar=self.layer != "altura", **args)
        if self.path is not None:
            if self.layer == "altura":
                pl.add_mesh(self.path, color="#f4c542", name="ruta")
            else:
                pl.add_mesh(self.path, scalars=self.layer, cmap=cfg["cmap"], clim=args["clim"],
                            show_scalar_bar=False, name="ruta")

    def show(self):
        self.pl.show()

    def screenshot(self, path, layer=None, cut=None):
        if layer:
            self.set_layer(layer)
        if cut is not None:
            self.set_cut(cut)
        self.pl.screenshot(str(path))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("model", help="archivo .vtp guardado por spot_brain.py (botón 'Detener escaneo')")
    args = ap.parse_args()
    import pyvista as pv
    path = Path(args.model)
    meta_path = path.with_suffix(".json")
    meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.exists() else {}
    Explorer(pv.read(path), meta).show()


if __name__ == "__main__":
    sys.exit(main())
