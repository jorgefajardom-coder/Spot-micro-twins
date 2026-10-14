"""Spot Micro con el cerebro de la mosca explorando una cueva: busca una fuga de gas,
rodea las fuentes de calor y mapea la cueva con un LiDAR 3D.

El conectoma completo de FlyWire (138.639 neuronas) decide; gait.py camina; sim_world.py
simula la cueva, el gas, el calor, el terreno y el LiDAR. Tres paneles:

    robot en la cueva  |  cerebro de la mosca  |  mapa 3D del LiDAR

Los sensores entran al cerebro por sustitución sensorial (ver sim_world.CaveWorld.senses):
en este modelo las vías olfativa y térmica saturan y no dan dirección, así que el gradiente
de gas usa el circuito de persecución visual (LC10/LC9) y el calor fuerte el de amenaza
(LPLC2/LC4 -> Giant Fiber). Render con PyVista/VTK (OpenGL, z-buffer, FXAA, SSAO).

Se ejecuta con el entorno de fly_brain (CuPy/numba, PyVista y los datos de FlyWire):

    fly_brain\\.venv\\Scripts\\python spot_brain.py
    fly_brain\\.venv\\Scripts\\python spot_brain.py --record spot.mp4 --seconds 30 --speed 2

Teclas:  t terreno escarpado (o el botón) · espacio lanzar un obstáculo · r reiniciar
"""
import argparse
import math
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "fly_brain"))

from cave_explorer import (MAP_BG, SCAN_COLORS, Explorer, build_cave_model, gradient,  # noqa: E402
                           reconstruct_surface, save_model)
from gait import BODY_HEIGHT, GaitManager  # noqa: E402
from kinematics import Kinematics  # noqa: E402
from sim_world import TEMP_AVOID, CaveWorld  # noqa: E402
from spot_model3d import BONES, DEFAULT_MODEL_FILE, SpotModel, solve_skeleton  # noqa: E402

# Modelo de alta resolución (models/build_model.py --full); si no existe, el ligero
HD_MODEL_FILE = HERE / "models" / "spot_model_hd.npz"

WINDOW = (1920, 1000)
COLUMNS = [0.42, 0.29, 0.29]                   # robot | cerebro | mapa
BG = "#0e1218"
YELLOW, CYAN, RED, GREEN, GREY = "#f4c542", "#40c4ff", "#ff5252", "#69f0ae", "#8fa5be"
ORANGE, GAS_GREEN = "#ff7a1a", "#7dff6b"
VOXEL_UM = np.array([0.004, 0.004, 0.040])    # tamaño de vóxel de FlyWire en µm (x, y, z)
ACTIVITY_TAU_MS = 120.0                        # cuánto tarda en apagarse una neurona en pantalla
HISTORY_S = 8.0                                # segundos en las gráficas de historia
BRAIN_SPIN_DEG_S = 6.0                         # giro lento de la vista del cerebro
MAX_SYNAPSE_SOURCES = 220                      # neuronas que dibujan sus sinapsis por fotograma
SYNAPSES_PER_NEURON = 3                        # sus conexiones más fuertes
SYN_EXCITATORY, SYN_INHIBITORY = "#ffb347", "#4fc3f7"
# Colores de neurona al estilo de las renderizaciones de FlyWire
NEURON_PALETTE = ["#ff4fd8", "#b45cff", "#7a5cff", "#4f7dff", "#3db8ff", "#29e3d8", "#ffd23f", "#ff6f91"]
ACTIVE_THRESHOLD = 0.12                        # actividad a partir de la que una neurona brilla
MAX_ACTIVE_DRAWN = 1500                        # neuronas activas dibujadas con su morfología
CAMERA_OFFSET = np.array([-0.78, -1.0, 0.68])   # cámara del robot respecto a él (~1,4 m)
MAP_CAMERA_OFFSET = np.array([-4.8, -6.2, 7.6])  # cámara del mapa: lejos y desde arriba (toda la galería)
MAP_CUT = 0.5                                     # m: el mapa enseña solo lo de debajo (sin el techo)
MAP_MESH_PERIOD = 1.5                             # s: cada cuánto se reconstruye la superficie del mapa

GAIT_SPEED = 1.5


# =============================================================================
# Utilidades de render
# =============================================================================

def transform(r, p):
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = p
    return m


def part_mesh(pv, v, f, c, smooth=False):
    """Malla con color por cara y normales suavizadas (sombreado de Phong sin dientes)."""
    faces = np.hstack([np.full((len(f), 1), 3), f]).astype(np.int64).ravel()
    mesh = pv.PolyData(np.asarray(v, float), faces)
    mesh.cell_data["rgb"] = (np.asarray(c) * 255).astype(np.uint8)
    if smooth:
        # El modelo ligero (agrupación de vértices) tiene escalones: Taubin los alisa sin encoger
        mesh = mesh.clean(tolerance=1e-6).smooth_taubin(n_iter=30, pass_band=0.08,
                                                        boundary_smoothing=False, feature_smoothing=False)
    # El STL tiene caras con orientación mixta: consistent_normals las arregla
    return mesh.compute_normals(cell_normals=False, split_vertices=True, feature_angle=50,
                                consistent_normals=True, auto_orient_normals=False)


class LiveLines:
    """Malla de líneas que se crea una vez y se actualiza en sitio.

    Rehacer el actor en cada fotograma (add_mesh con name=...) obligaba a VTK a reconstruir
    el pipeline y costaba más que dibujar la escena entera.
    """

    def __init__(self, pl, pv, points=None, scalars=None, **style):
        pts = np.zeros((2, 3), np.float32) if points is None else points
        self.mesh = pv.PolyData(pts, lines=np.array([2, 0, 1]))
        self.scalars = scalars
        if scalars:
            self.mesh.cell_data[scalars] = np.zeros(1, np.float32)
        self.actor = pl.add_mesh(self.mesh, scalars=scalars, show_scalar_bar=False, lighting=False, **style)
        self.actor.visibility = False

    def update(self, segs, points=None, values=None):
        if len(segs) == 0:
            self.actor.visibility = False
            return
        if points is not None:
            self.mesh.points = points
        self.mesh.lines = np.column_stack([np.full(len(segs), 2), segs]).ravel()
        if self.scalars:
            self.mesh.cell_data[self.scalars] = values
        self.actor.visibility = True


class LivePoints:
    """Nube de puntos que se crea una vez y se actualiza en sitio (escaneo y mapa del LiDAR)."""

    def __init__(self, pl, pv, scalars=None, **style):
        self.mesh = pv.PolyData(np.zeros((1, 3), np.float32))
        self.scalars = scalars
        if scalars:
            self.mesh.point_data[scalars] = np.zeros(1, np.float32)
        self.actor = pl.add_mesh(self.mesh, scalars=scalars, show_scalar_bar=False, style="points",
                                 render_points_as_spheres=True, **style)
        self.actor.visibility = False

    def update(self, points, values=None):
        if len(points) == 0:
            self.actor.visibility = False
            return
        self.mesh.points = np.asarray(points, np.float32)
        self.mesh.verts = np.column_stack([np.ones(len(points), np.int64), np.arange(len(points))]).ravel()
        if self.scalars:
            self.mesh.point_data[self.scalars] = values
        self.actor.visibility = True


def top_targets(weights, k):
    """Las k dianas más fuertes (|peso|) de cada neurona y su signo; -1 si tiene menos."""
    from numba import njit

    @njit(cache=True)
    def _top(indptr, indices, data, k):
        n = indptr.shape[0] - 1
        post = np.full((n, k), -1, np.int64)
        sign = np.zeros((n, k), np.float32)
        for i in range(n):
            lo, hi = indptr[i], indptr[i + 1]
            order = np.argsort(-np.abs(data[lo:hi]))[:k]
            for j in range(order.shape[0]):
                post[i, j] = indices[lo + order[j]]
                sign[i, j] = 1.0 if data[lo + order[j]] > 0 else -1.0
        return post, sign

    w = weights.tocsc()
    return _top(w.indptr.astype(np.int64), w.indices.astype(np.int64), w.data.astype(np.float32), k)


def hex_rgb(color):
    return tuple(int(color[i:i + 2], 16) / 255 for i in (1, 3, 5))


def style_chart(chart, title):
    """Gráfica 2D compacta con el estilo oscuro de la interfaz."""
    chart.background_color = (0.06, 0.08, 0.11, 0.88)
    chart.border_color = "#35465b"
    chart.title = title
    chart.GetTitleProperties().SetColor(1, 1, 1)
    chart.GetTitleProperties().SetFontSize(12)
    for ax in (chart.x_axis, chart.y_axis):
        ax.label = ""
        ax.pen.color = "#35465b"
        ax.grid = False
        ax.GetLabelProperties().SetColor(*hex_rgb(GREY))
        ax.GetLabelProperties().SetFontSize(10)
    return chart


def bar_chart(pv, title, labels, colors, xmax, loc, size):
    """Barras horizontales; una serie por color. Devuelve (gráfica, [(serie, posiciones)])."""
    chart = pv.Chart2D(size=size, loc=loc)
    y = np.arange(len(labels))[::-1].astype(float)
    series = []
    for color in dict.fromkeys(colors):
        pos = y[[i for i, c in enumerate(colors) if c == color]]
        series.append((chart.bar(pos, np.zeros(len(pos)), color=color, orientation="H"), pos))
    chart.y_axis.tick_locations = list(y)
    chart.y_axis.tick_labels = labels
    chart.y_range = [-0.7, len(labels) - 0.3]
    chart.x_range = [0, xmax]
    chart.legend_visible = False
    return style_chart(chart, title), series


def history_chart(pv, title, series, y_range, loc, size):
    """Líneas de los últimos segundos. series: [(clave, color, etiqueta)]."""
    chart = pv.Chart2D(size=size, loc=loc)
    style_chart(chart, title)
    lines = {k: chart.line([0, 1], [0, 0], color=c, width=2, label=lbl) for k, c, lbl in series}
    chart.x_range = [-HISTORY_S, 0]
    chart.y_range = list(y_range)
    chart.legend_visible = len(series) > 1
    chart.GetLegend().GetLabelProperties().SetColor(1, 1, 1)
    chart.GetLegend().GetLabelProperties().SetFontSize(9)
    chart.GetLegend().GetBrush().SetColorF(0.06, 0.08, 0.11)
    return chart, lines


def cave_shell(pv, cave, n_arc=48, every=1):
    """Paredes y techo de la cueva: un arco por cada punto de la línea central.

    Coincide con el hueco que ve el LiDAR (sim_world._free): a una distancia q*r del centro
    el techo está a ceil * sqrt(1 - q²). Normales hacia fuera, para recortar las caras
    que miran a la cámara y ver el interior desde fuera.
    """
    idx = np.arange(0, len(cave.s), every)
    theta = np.linspace(0, np.pi, n_arc)
    c, t = cave.center[idx], cave.tangent[idx]
    normal = np.column_stack([-t[:, 1], t[:, 0]])
    r, h = cave.radius[idx, None], cave.ceiling[idx, None]
    lat = r * np.cos(theta)[None, :]
    X = c[:, 0, None] + lat * normal[:, 0, None]
    Y = c[:, 1, None] + lat * normal[:, 1, None]
    Z = h * np.sin(theta)[None, :]
    # Roca rugosa: misma deformación que ve el LiDAR (la sección se escala alrededor del eje)
    axis = np.repeat(c[:, None, :], n_arc, axis=1)
    pts = np.stack([X, Y, Z], -1).reshape(-1, 3)
    f = cave.wall_factor(pts).reshape(X.shape)
    X = axis[..., 0] + (X - axis[..., 0]) * f
    Y = axis[..., 1] + (Y - axis[..., 1]) * f
    Z = Z * f - 0.08 * (np.abs(np.cos(theta)) > 0.995)[None, :]
    grid = pv.StructuredGrid(X, Y, Z)
    rng = np.random.default_rng(0)
    grid.point_data["roca"] = (0.55 + 0.25 * np.sin(X * 7.1 + Y * 3.3) * np.cos(Z * 9.0)
                               + 0.12 * rng.standard_normal(X.size).reshape(X.shape)).ravel(order="F")
    surface = grid.extract_surface(algorithm="dataset_surface")
    shell = surface.compute_normals(auto_orient_normals=False, consistent_normals=True)
    # Orientar hacia fuera (del eje de la galería hacia la roca), por mayoría. Se invierte sobre la
    # superficie original: el recorte de caras usa el orden de los vértices, no el array de normales.
    _, k = cave._tree.query(shell.points[:, :2])
    outward = np.column_stack([shell.points[:, :2] - cave.center[k], shell.points[:, 2]])
    if np.mean(np.sum(shell.point_data["Normals"] * outward, axis=1) > 0) < 0.5:
        shell = surface.compute_normals(flip_normals=True, auto_orient_normals=False, consistent_normals=True)
    return shell


def cave_floor(pv, world, cell=0.03):
    """Suelo de la cueva (rejilla con la altura del terreno), solo dentro de la galería."""
    cave = world.cave
    lo, hi = cave.center.min(0) - 1.4, cave.center.max(0) + 1.4
    gx, gy = np.arange(lo[0], hi[0], cell), np.arange(lo[1], hi[1], cell)
    X, Y = np.meshgrid(gx, gy, indexing="ij")
    grid = pv.StructuredGrid(X, Y, world.ground(X, Y))
    d, i = cave._tree.query(np.column_stack([X.ravel(order="F"), Y.ravel(order="F")]))
    grid.point_data["dentro"] = (d < cave.radius[i] + 0.06).astype(float)
    floor = grid.threshold(0.5, scalars="dentro").extract_surface()
    rng = np.random.default_rng(1)
    floor.point_data["roca"] = 0.5 + 0.08 * rng.standard_normal(floor.n_points)  # grano fino
    return floor


def gas_particles(world, n=2600, seed=5):
    """Partículas del penacho de gas: más densas donde hay más concentración."""
    rng = np.random.default_rng(seed)
    cave = world.cave
    s_lo, s_hi = max(0, world.gas_s - 5.0), min(cave.s[-1], world.gas_s + 1.5)
    pts = []
    while sum(len(p) for p in pts) < n:
        s = rng.uniform(s_lo, s_hi, 4000)
        c, t, r, h = zip(*(cave.point_at(v) for v in s))
        c, t, r, h = np.array(c), np.array(t), np.array(r), np.array(h)
        q = rng.uniform(-0.85, 0.85, len(s))
        normal = np.column_stack([-t[:, 1], t[:, 0]])
        xy = c + normal * (q * r)[:, None]
        z = rng.uniform(0.02, 1, len(s)) * h * np.sqrt(1 - q ** 2) * 0.9
        ppm = world.gas_ppm(xy)
        keep = rng.uniform(0, 1, len(s)) < ((ppm - 5) / 900) ** 0.6
        pts.append(np.column_stack([xy[keep], z[keep]]))
    return np.concatenate(pts)[:n]


# =============================================================================
# Interfaz
# =============================================================================

class BrainViewer:
    def __init__(self, device="auto", off_screen=False, size=WINDOW):
        import pyvista as pv
        from connectome import Connectome
        from pilot import FlyPilot, Senses
        self.pv, self.Senses = pv, Senses
        self.W, self.H = size

        print("Cargando el conectoma de FlyWire…", flush=True)
        self.connectome = Connectome.load()
        self.pilot = FlyPilot(self.connectome, device=device)
        gain = self.pilot.calibrate()
        self.backend = "GPU" if type(self.pilot.brain).__name__ == "BrainGPU" else "CPU"
        print(f"Cerebro en {self.backend} · calibración {({k: round(v) for k, v in gain.items()})}")

        self.kin = Kinematics()
        self.gait = GaitManager(self.kin)
        self.gait.set_gait("trotting")
        self.gait.gait_speed = GAIT_SPEED
        self.world = CaveWorld()
        self.readings = {"gas_ppm": 0.0, "temp_c": 0.0, "modo": "explorando"}
        self.activity = np.zeros(self.connectome.n, np.float32)
        self.cmd = None
        self.focus = np.r_[self.world.pose[:2], 0.12]
        self.map_focus = self.focus.copy()
        self.last = time.perf_counter()
        self.last_dt = 0.04
        self.t = 0.0
        self.hist_t, self.hist = [], {}
        self.scanning = True
        self.off_screen = off_screen
        self.saved_model = None
        # Dianas más fuertes de cada neurona, precalculadas: en cada fotograma solo se indexan
        self.syn_post, self.syn_sign = top_targets(self.connectome.weights, SYNAPSES_PER_NEURON)

        pv.global_theme.font.family = "arial"
        self.pl = pv.Plotter(shape=(1, 3), col_weights=COLUMNS, window_size=size, off_screen=off_screen,
                             border=False, title="Spot Micro Twins · Cerebro de mosca en la cueva")
        self.pl.set_background(BG)
        self._build_robot()
        self._build_brain()
        self._build_map()
        # FXAA: SSAA renderiza al doble (descoloca las gráficas 2D) y MSAA rompe el SSAO
        self.pl.enable_anti_aliasing("fxaa")

    # ----- panel 1: el robot en la cueva -----
    def _build_robot(self):
        pv, pl, H = self.pv, self.pl, self.H
        pl.subplot(0, 0)
        hd = HD_MODEL_FILE.exists()
        model = SpotModel(self.kin, str(HD_MODEL_FILE if hd else DEFAULT_MODEL_FILE))
        print(f"Modelo del robot: {model.source} ({model.face_count:,} caras)".replace(",", "."))
        style = dict(scalars="rgb", rgb=True, smooth_shading=True, ambient=0.25, diffuse=0.8,
                     specular=0.35, specular_power=25)
        mesh = lambda p: part_mesh(pv, *p, smooth=not hd)  # noqa: E731
        self.body_actors = [pl.add_mesh(mesh(p), **style) for p in model.body]
        self.leg_actors = [{bone: [pl.add_mesh(mesh(p), **style) for p in model.legs[leg][bone]]
                            for bone in BONES} for leg in range(4)]

        rock = dict(scalars="roca", cmap=gradient(["#2a211c", "#4d4036", "#6e6253", "#8a7c69"]), clim=[0.2, 1.0],
                    show_scalar_bar=False, smooth_shading=True, ambient=0.3, diffuse=0.75, specular=0.05)
        shell = pl.add_mesh(cave_shell(pv, self.world.cave), **rock)
        shell.prop.culling = "front"  # la pared que mira a la cámara no tapa al robot
        self.floor_mesh = cave_floor(pv, self.world)
        pl.add_mesh(self.floor_mesh, **rock)

        # Fuente de calor: núcleo incandescente y halo
        hx, hy = self.world.heat_xy
        pl.add_mesh(pv.Sphere(radius=0.07, center=(hx, hy, 0.06)), color=ORANGE, ambient=1.0, diffuse=0.0)
        pl.add_mesh(pv.Sphere(radius=0.32, center=(hx, hy, 0.12)), color=RED, opacity=0.12, ambient=1.0,
                    diffuse=0.0)
        # Penacho de gas
        pl.add_mesh(pv.PolyData(gas_particles(self.world)), color=GAS_GREEN, point_size=7, opacity=0.22,
                    render_points_as_spheres=True, ambient=0.8)
        # LiDAR: puntos del último barrido
        self.scan_points = LivePoints(pl, pv, color=CYAN, point_size=3, opacity=0.9, ambient=1.0)
        self.obstacle_actor = pl.add_mesh(pv.Sphere(radius=0.06, center=(0, 0, 0.06)), color=RED,
                                          smooth_shading=True, specular=0.4, opacity=0.9)
        self.obstacle_actor.visibility = False
        self.trail = LiveLines(pl, pv, color=YELLOW, line_width=2)

        # Luz de tres puntos (clave, relleno y contraluz) que acompaña a la cámara
        pl.remove_all_lights()
        for pos, intensity in (((1.0, -1.5, 2.0), 0.85), ((-1.5, -0.5, 1.0), 0.35), ((-0.5, 2.0, 1.5), 0.45)):
            pl.add_light(pv.Light(position=pos, focal_point=(0, 0, 0), intensity=intensity,
                                  light_type="camera light"))
        pl.enable_ssao(radius=0.03, bias=0.002, kernel_size=64)  # sombras de contacto entre piezas

        # Historia de los últimos segundos (compacta, abajo a la derecha)
        self.history, lines = history_chart(
            pv, "Últimos segundos", (("vx", "#ffffff", "avance"), ("giro", GREEN, "giro"),
                                     ("P9", CYAN, "P9"), ("GF", RED, "Giant Fiber")),
            (-1.1, 1.1), loc=(0.55, 0.03), size=(0.43, 0.26))
        self.hist_lines = dict(lines)
        pl.add_chart(self.history)

        pl.add_text("SPOT MICRO  /  CEREBRO DE MOSCA", position=(20, H - 50), font_size=18, color=YELLOW)
        pl.add_text(f"Conectoma FlyWire · 138.639 neuronas · {self.backend}  ·  cueva simulada con LiDAR",
                    position=(22, H - 78), font_size=10, color=GREY)
        pl.add_text("t terreno · espacio obstáculo · r reiniciar", position=(22, 14), font_size=9, color=GREY)
        self.state_text = pl.add_text(" ", position=(22, 100), font_size=12, color=GREEN)
        # Botón: terreno escarpado
        self.rough_button = pl.add_checkbox_button_widget(
            lambda on: self.set_rough(on), value=False, position=(22, 44), size=30, border_size=2,
            color_on=YELLOW, color_off="#2a3646", background_color="#8fa5be")
        pl.add_text("Terreno escarpado", position=(62, 50), font_size=11, color="w")

    # ----- panel 2: el cerebro -----
    def _build_brain(self):
        pv, pl, c, H = self.pv, self.pl, self.connectome, self.H
        pl.subplot(0, 1)
        ann = pd_read_positions(c)
        um = ann * VOXEL_UM
        self.brain_center = np.array([um[:, 0].mean(), um[:, 2].mean(), -um[:, 1].mean()])
        pts = to_view(um) - self.brain_center
        self.brain_pts = pts
        self.skeletons = load_skeletons(self.brain_center)
        if self.skeletons is None:
            # Sin morfología descargada: una nube de puntos (una por neurona) da la forma
            pl.add_mesh(pv.PolyData(pts), style="points", color="#6b8cba", point_size=1.5, opacity=0.3,
                        render_points_as_spheres=False)
        else:
            sk = self.skeletons
            print(f"Morfología: {sk['n_neurons']:,} neuronas, {len(sk['segs']):,} segmentos".replace(",", "."))
            mesh = sk["mesh"]
            # Cada neurona con un color vivo, como las renderizaciones de FlyWire
            palette = np.array([hex_rgb(h) for h in NEURON_PALETTE])
            hue = (np.asarray(c.root_ids) * 2654435761 % 2**32) % len(palette)
            mesh.cell_data["rgb"] = (palette[hue[sk["owner"]]] * 255 * 0.62).astype(np.uint8)
            pl.add_mesh(mesh, scalars="rgb", rgb=True, line_width=1.0, opacity=0.55, lighting=False)

        groups = {"Giant Fiber": ("DNp01", None), "DNa02 izq": ("DNa02", "left"),
                  "DNa02 der": ("DNa02", "right"), "P9 (avanzar)": ("DNp09", None),
                  "MDN (atrás)": ("MDN", None), "LC10 izq": ("LC10a", "left"),
                  "LC10 der": ("LC10a", "right"), "LPLC2 izq": ("LPLC2", "left"),
                  "LPLC2 der": ("LPLC2", "right")}
        labels, where = [], []
        for name, (t, side) in groups.items():
            idx = c.find(t, side)
            if len(idx):
                labels.append(name)
                where.append(pts[idx].mean(axis=0))
        pl.add_point_labels(np.array(where), labels, font_size=9, text_color="w", point_color=YELLOW,
                            point_size=5, render_points_as_spheres=True, shape_color="#1b2433",
                            shape_opacity=0.65, always_visible=True)
        title = "CEREBRO  ·  morfología real (FlyWire)" if self.skeletons is not None else "CEREBRO"
        pl.add_text(title, position=(18, H - 48), font_size=13, color="w")
        self.n_active_text = pl.add_text(" ", position=(20, H - 74), font_size=10, color=GREY)
        if self.skeletons is not None:
            self.active_lines = LiveLines(pl, pv, points=self.skeletons["verts"], scalars="act",
                                          cmap="hot", clim=[-0.25, 1.0], line_width=2.2)
        self.synapse_lines = LiveLines(pl, pv, scalars="signo", cmap=[SYN_INHIBITORY, SYN_EXCITATORY],
                                       clim=[-1, 1], line_width=1.0, opacity=0.4)
        pl.add_text("brillo: disparando · naranja/azul: sinapsis excit./inhib.", position=(20, int(H * 0.47)),
                    font_size=8, color=GREY)
        self.brain_angle = -25.0  # grados; la vista gira despacio alrededor del cerebro

        # Gráficas compactas debajo del cerebro: lo que ve y lo que manda al cuerpo
        self.in_keys = ["objetivo_izq", "objetivo_der", "delante_izq", "delante_der",
                        "looming_izq", "looming_der"]
        self.in_chart, self.in_series = bar_chart(
            pv, "Ojos: estímulo (Hz)", ["LC10 izq", "LC10 der", "LC9 izq", "LC9 der", "LPLC2 izq", "LPLC2 der"],
            [GREEN, GREEN, CYAN, CYAN, RED, RED], 160, loc=(0.03, 0.235), size=(0.94, 0.215))
        self.out_keys = ["P9_izq", "P9_der", "DNa02_izq", "DNa02_der", "MDN", "GF"]
        self.out_chart, self.out_series = bar_chart(
            pv, "Descendentes: cerebro -> cuerpo (Hz)",
            ["P9 izq", "P9 der", "DNa02 izq", "DNa02 der", "MDN atrás", "Giant Fiber"],
            [CYAN, CYAN, GREEN, GREEN, YELLOW, RED], 300, loc=(0.03, 0.01), size=(0.94, 0.215))
        pl.add_chart(self.in_chart, self.out_chart)
        self._place_brain_camera()

    def _place_brain_camera(self):
        """Cerebro entero en la parte de arriba, visto en diagonal desde arriba y girando."""
        az, el = math.radians(self.brain_angle), math.radians(24)
        direction = np.array([math.sin(az) * math.cos(el), -math.cos(az) * math.cos(el), math.sin(el)])
        focal = np.array([0.0, 0.0, -330.0])  # el cerebro queda arriba, encima de las gráficas
        cam = self.pl.camera
        cam.focal_point = focal
        cam.position = focal + 2900.0 * direction
        cam.up = (0, 0, 1)
        self.pl.renderer.reset_camera_clipping_range()

    # ----- panel 3: el mapa del LiDAR -----
    def _build_map(self):
        pv, pl, H = self.pv, self.pl, self.H
        pl.subplot(0, 2)
        pl.set_background(MAP_BG, all_renderers=False)
        grid = pv.Plane(center=(0, 0, -0.06), i_size=60, j_size=60, i_resolution=120, j_resolution=120)
        pl.add_mesh(grid, style="wireframe", color="#2c3038", line_width=1, lighting=False)
        # Superficie reconstruida (se rehace cada MAP_MESH_PERIOD s) y barrido actual del LiDAR
        dummy = pv.Sphere(radius=0.01)
        dummy.point_data["altura"] = np.zeros(dummy.n_points)
        self.map_surface = dummy
        self.map_surface_actor = pl.add_mesh(dummy, scalars="altura", cmap=gradient(SCAN_COLORS), clim=[0, MAP_CUT],
                                             show_scalar_bar=False, smooth_shading=True, ambient=0.25,
                                             diffuse=0.85, specular=0.15, specular_power=12)
        self.map_surface_actor.visibility = False
        self.map_surface_t = -1e9
        self.map_scan = LivePoints(pl, pv, color=CYAN, point_size=3, opacity=0.9, lighting=False)
        self.map_path = LiveLines(pl, pv, color=YELLOW, line_width=3)
        self.map_robot = pl.add_mesh(pv.Cone(center=(0, 0, 0), direction=(1, 0, 0), height=0.3, radius=0.11),
                                     color=YELLOW, ambient=0.6)
        self.map_gas = pl.add_mesh(pv.Sphere(radius=0.16), color=GAS_GREEN, ambient=0.8, opacity=0.85)
        self.map_gas.visibility = False
        self.map_heat = pl.add_mesh(pv.Sphere(radius=0.13), color=ORANGE, ambient=0.8, opacity=0.85)
        self.map_heat.visibility = False
        self.heat_seen = False
        pl.add_text("MAPA LiDAR  ·  cueva reconstruida", position=(18, H - 48), font_size=13, color="w")
        self.map_text = pl.add_text(" ", position=(20, H - 74), font_size=10, color=GREY)
        pl.add_text("superficie reconstruida (sin techo)  ·  cian: barrido actual  ·  verde: gas  ·  naranja: calor",
                    position=(20, int(H * 0.47)), font_size=8, color=GREY)
        self.gas_chart, gas_lines = history_chart(pv, "Gas (ppm)", (("gas", GAS_GREEN, "gas"),), (0, 1000),
                                                  loc=(0.03, 0.235), size=(0.94, 0.215))
        self.temp_chart, temp_lines = history_chart(pv, "Temperatura (°C)", (("temp", ORANGE, "temp"),), (15, 100),
                                                    loc=(0.03, 0.01), size=(0.94, 0.215))
        self.hist_lines.update(gas_lines)
        self.hist_lines.update(temp_lines)
        self.hist = {k: [] for k in self.hist_lines}
        pl.add_chart(self.gas_chart, self.temp_chart)
        # El mapa ocupa la mitad de arriba (las gráficas la de abajo): centro de proyección más arriba
        pl.camera.SetWindowCenter(0.0, -0.48)
        # Botón: detener el escaneo y generar la cueva completa para explorarla
        self.stop_button = pl.add_checkbox_button_widget(  # posición relativa al panel, como el texto
            lambda on: self.stop_scan(on), value=False, position=(20, int(H * 0.5)), size=30, border_size=2,
            color_on="#7dff6b", color_off="#2a3646", background_color="#8fa5be")
        pl.add_text("Detener escaneo y generar la cueva 3D", position=(60, int(H * 0.5) + 6), font_size=11,
                    color="w")

    # ----- control -----
    def set_rough(self, on):
        """Botón / tecla t: terreno escarpado. El suelo, el LiDAR y los pies lo notan a la vez."""
        self.world.terrain.rough = bool(on)
        pts = self.floor_mesh.points
        pts[:, 2] = self.world.ground(pts[:, 0], pts[:, 1])
        self.floor_mesh.points = pts
        if self.rough_button.GetRepresentation().GetState() != int(bool(on)):
            self.rough_button.GetRepresentation().SetState(int(bool(on)))

    def stop_scan(self, on):
        """Botón: para el robot, reconstruye la cueva entera (con gas y temperatura) y la abre
        en el explorador. Al desmarcarlo, el escaneo continúa."""
        if not on:
            self.scanning = True
            return
        self.scanning = False
        self.state_text.SetInput("ESCANEO DETENIDO  ·  generando la cueva 3D…")
        self.state_text.GetTextProperty().SetColor(*hex_rgb(GAS_GREEN))
        self.pl.render()
        mesh, meta = build_cave_model(self.world)
        if mesh is None:
            self.state_text.SetInput("ESCANEO DETENIDO  ·  aún no hay suficientes datos")
            return
        self.saved_model = save_model(mesh, meta)
        print(f"Cueva guardada: {self.saved_model} (+ .ply para Blender)")
        self.state_text.SetInput(f"CUEVA GUARDADA  ·  scans/{self.saved_model.name}")
        if not self.off_screen:
            import subprocess
            subprocess.Popen([sys.executable, str(HERE / "cave_explorer.py"), str(self.saved_model)])

    def on_key(self, key):
        if key == "space":
            self.world.launch_obstacle()
        elif key == "t":
            self.set_rough(not self.world.terrain.rough)
        elif key == "r":
            self.world.reset_robot()
            self.pilot.reset()
            self.scanning = True
            self.stop_button.GetRepresentation().SetState(0)
            self.activity[:] = 0
            self.heat_seen = False
            self.map_gas.visibility = self.map_heat.visibility = False

    def _body_velocity(self):
        p = self.gait.gait_parameters[self.gait.active_gait]
        v = p["step_length"] / (p["duty_factor"] * p["cycle_time"]) * self.gait.gait_speed
        x, y, _ = self.gait.neutral_positions[0]
        vx, vy, wz = self.gait.command
        return vx * v, vy * v, wz * v / math.hypot(x, y)

    def tick(self, dt):
        """Un paso: mundo y sensores -> cerebro (dt de tiempo neuronal) -> marcha."""
        self.last_dt = dt
        w = self.world
        w.update(dt, self._body_velocity())
        w.lidar_scan(BODY_HEIGHT + float(w.ground(w.pose[0], w.pose[1])))
        bearing, dist, looming, loom_bearing, self.readings = w.senses()
        self.cmd = self.pilot.step(self.Senses(bearing, dist, looming, loom_bearing), ms=dt * 1000.0)
        self.gait.set_command(self.cmd.vx, 0.0, self.cmd.wz)
        # Actividad visible: cada espiga enciende la neurona y se apaga con ACTIVITY_TAU_MS
        spikes = self.pilot.last_counts
        self.activity *= math.exp(-dt * 1000.0 / ACTIVITY_TAU_MS)
        self.activity = np.minimum(1.0, self.activity + 0.5 * spikes).astype(np.float32)
        return np.array(self.gait.step(dt))

    # ----- dibujo -----
    def draw(self, feet):
        self._draw_robot(feet)
        self._draw_brain()
        self._draw_map()
        self._draw_history()

    def _draw_robot(self, feet):
        pl, w = self.pl, self.world
        c, s = math.cos(w.pose[2]), math.sin(w.pose[2])
        rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        feet_w, rpy, body = w.body_on_terrain(feet @ rz.T + [w.pose[0], w.pose[1], 0], BODY_HEIGHT)
        skel = solve_skeleton(self.kin, feet_w.tolist(), rpy, body.tolist())
        for a in self.body_actors:
            a.user_matrix = transform(*skel["body"])
        for leg, bones in enumerate(skel["legs"]):
            for bone in BONES:
                for a in self.leg_actors[leg][bone]:
                    a.visibility = bones is not None
                    if bones is not None:
                        a.user_matrix = transform(*bones[bone])

        pl.subplot(0, 0)
        self.obstacle_actor.visibility = w.obstacle is not None
        if w.obstacle is not None:
            self.obstacle_actor.position = (*w.obstacle, float(w.ground(*w.obstacle)))
        hits, _ = w.last_scan
        self.scan_points.update(hits[~np.isnan(hits[:, 0])])
        if len(w.trail) > 1:
            tr = np.array(w.trail)
            tr = np.column_stack([tr, w.ground(tr[:, 0], tr[:, 1]) + 0.005]).astype(np.float32)
            n = len(tr)
            self.trail.update(np.column_stack([np.arange(n - 1), np.arange(1, n)]), points=tr)
        # Cámara cerca del robot, suavizada para no marear
        robot = np.r_[w.pose[:2], body[2] - 0.07]
        self.focus += 0.35 * (robot - self.focus)
        pl.camera.focal_point = self.focus
        pl.camera.position = self.focus + CAMERA_OFFSET
        pl.camera.up = (0, 0, 1)
        pl.renderer.reset_camera_clipping_range()  # sin esto VTK recorta el robot al moverse

        rd, cmd = self.readings, self.cmd
        if cmd.escape:
            msg, col = "¡ESCAPE!  Giant Fiber: retrocede", RED
        elif rd["modo"] == "fuga localizada":
            msg, col = f"FUGA LOCALIZADA  ·  {rd['gas_ppm']:.0f} ppm", GAS_GREEN
        elif rd["temp_c"] > TEMP_AVOID:
            msg, col = f"RODEANDO CALOR  ·  {rd['temp_c']:.0f} °C", ORANGE
        elif rd["modo"] == "siguiendo gas":
            msg, col = f"SIGUIENDO EL GAS  ·  {rd['gas_ppm']:.0f} ppm", GAS_GREEN
        else:
            msg, col = "EXPLORANDO LA CUEVA", CYAN
        self.state_text.SetInput(msg)
        self.state_text.GetTextProperty().SetColor(*hex_rgb(col))

    def _draw_brain(self):
        self.pl.subplot(0, 1)
        if self.skeletons is not None:
            self._draw_active_neurons()
        self._draw_synapses()
        self._update_bars(self.in_series, self.in_keys, self.cmd.inputs)
        self._update_bars(self.out_series, self.out_keys, self.cmd.rates)
        n_active = f"{int((self.activity > 0.3).sum()):,}".replace(",", ".")
        self.n_active_text.SetInput(f"neuronas activas ahora: {n_active}")
        self.brain_angle += BRAIN_SPIN_DEG_S * self.last_dt
        self._place_brain_camera()

    def _draw_map(self):
        pl, w = self.pl, self.world
        pl.subplot(0, 2)
        pts = w.map_points
        if self.t - self.map_surface_t >= MAP_MESH_PERIOD and len(pts):
            self.map_surface_t = self.t
            surf = reconstruct_surface(self.pv, pts, w.ground, cut=MAP_CUT)
            if surf is not None:
                self.map_surface.copy_from(surf)  # mismo actor: VTK solo sube la malla nueva
                self.map_surface_actor.visibility = True
        hits, _ = w.last_scan
        self.map_scan.update(hits[~np.isnan(hits[:, 0])])
        tr = np.array(w.trail)
        if len(tr) > 1:
            tr = np.column_stack([tr, np.full(len(tr), 0.05)]).astype(np.float32)
            n = len(tr)
            self.map_path.update(np.column_stack([np.arange(n - 1), np.arange(1, n)]), points=tr)
        self.map_robot.position = (w.pose[0], w.pose[1], 0.2)
        self.map_robot.orientation = (0, 0, math.degrees(w.pose[2]))
        if w.gas_found is not None and not self.map_gas.visibility:
            self.map_gas.position = (*w.gas_found, 0.3)
            self.map_gas.visibility = True
        if self.readings["temp_c"] > TEMP_AVOID and not self.heat_seen:
            self.heat_seen = True
            self.map_heat.position = (*w.heat_xy, 0.2)
            self.map_heat.visibility = True
        self.map_text.SetInput(f"{len(pts):,} vóxeles de 4 cm  ·  recorrido {w.progress:.1f} m".replace(",", "."))
        robot = np.r_[w.pose[:2], 0.3]
        self.map_focus += 0.2 * (robot - self.map_focus)
        pl.camera.focal_point = self.map_focus
        pl.camera.position = self.map_focus + MAP_CAMERA_OFFSET
        pl.camera.up = (0, 0, 1)
        pl.renderer.reset_camera_clipping_range()

    def _draw_active_neurons(self):
        """Las neuronas activas se dibujan encima con toda su morfología, brillando."""
        sk = self.skeletons
        on = np.flatnonzero(self.activity[: len(sk["start"])] > ACTIVE_THRESHOLD)
        on = on[sk["count"][on] > 0]
        if not len(on):
            self.active_lines.update([])
            return
        if len(on) > MAX_ACTIVE_DRAWN:
            on = on[np.argsort(-self.activity[on])[:MAX_ACTIVE_DRAWN]]
        # Índices de segmentos de cada neurona activa (están ordenados por neurona)
        counts = sk["count"][on]
        seg_idx = np.repeat(sk["start"][on] - np.cumsum(counts) + counts, counts) + np.arange(counts.sum())
        # Los puntos (toda la morfología) son fijos en la GPU; solo cambian qué líneas se dibujan
        self.active_lines.update(sk["segs"][seg_idx], values=np.repeat(self.activity[on], counts))

    def _update_bars(self, series, keys, values):
        """Cada serie (un color) tiene sus posiciones; los valores van en el orden de keys."""
        k = 0
        for bars, pos in series:
            bars.update(pos, [values[key] for key in keys[k:k + len(pos)]])
            k += len(pos)

    def _draw_history(self):
        g, r = self.pilot.gain, self.cmd.rates
        self.t += self.last_dt
        self.hist_t.append(self.t)
        for k, val in (("vx", self.cmd.vx), ("giro", self.cmd.wz),
                       ("P9", min(1.0, (r["P9_izq"] + r["P9_der"]) / 2 / g["forward"])),
                       ("GF", min(1.0, r["GF"] / 250)),
                       ("gas", self.readings["gas_ppm"]), ("temp", self.readings["temp_c"])):
            self.hist[k].append(val)
        while self.hist_t[0] < self.t - HISTORY_S:
            self.hist_t.pop(0)
            for v in self.hist.values():
                v.pop(0)
        t = np.array(self.hist_t) - self.t
        if len(t) > 1:
            for k, line in self.hist_lines.items():
                line.update(t, self.hist[k])

    def _draw_synapses(self):
        """Líneas desde las neuronas que acaban de disparar a sus dianas más fuertes (conectoma real)."""
        spiking = np.flatnonzero(self.pilot.last_counts)
        if len(spiking) > MAX_SYNAPSE_SOURCES:  # las más activas, para no saturar la vista
            spiking = spiking[np.argsort(-self.activity[spiking])[:MAX_SYNAPSE_SOURCES]]
        post = self.syn_post[spiking]
        valid = post >= 0
        pre = np.repeat(spiking, valid.sum(axis=1))
        post, sign = post[valid], self.syn_sign[spiking][valid]
        m = len(pre)
        pts = np.empty((2 * m, 3), np.float32)
        pts[0::2], pts[1::2] = self.brain_pts[pre], self.brain_pts[post]
        segs = np.column_stack([np.arange(0, 2 * m, 2), np.arange(1, 2 * m, 2)])
        self.synapse_lines.update(segs, points=pts, values=sign)

    # ----- bucles -----
    def update(self, *_):
        now = time.perf_counter()
        dt = min(max(now - self.last, 0.02), 0.08)
        self.last = now
        if self.scanning:
            self.draw(self.tick(dt))
        self.pl.render()

    def run(self):
        for key in ("space", "t", "r"):
            self.pl.add_key_event(key, lambda k=key: self.on_key(k))
        self.draw(self.tick(0.04))
        self.pl.add_timer_event(max_steps=10**9, duration=1, callback=self.update)  # sin tope de fps
        self.pl.show()

    def record(self, path, seconds=30.0, fps=24, speed=2.0, rough_at=0.35):
        """Misión grabada con dt fijo (vídeo .mp4 o GIF, sin ventana). speed > 1 acelera la
        simulación respecto al vídeo; a rough_at (fracción) se activa el terreno escarpado."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.suffix == ".gif":
            self.pl.open_gif(str(path), fps=fps)
        else:
            self.pl.open_movie(str(path), framerate=fps, quality=8)
        n = int(seconds * fps)
        for i in range(n):
            if i == int(rough_at * n):
                self.set_rough(True)
            for _ in range(int(round(speed))):
                feet = self.tick(1.0 / fps)
            self.draw(feet)
            self.pl.write_frame()
        self.pl.close()
        mesh, meta = build_cave_model(self.world)
        if mesh is not None:
            self.saved_model = save_model(mesh, meta)
            explorer = Explorer(mesh, meta, off_screen=True)
            for layer in ("altura", "gas_ppm", "temp_c"):
                explorer.screenshot(path.with_name(f"{path.stem}_cueva_{layer}.png"), layer=layer)
        return n


def to_view(um):
    """Coordenadas de FlyWire (µm; x a la derecha, y hacia ventral, z en profundidad) a la vista:
    x a la derecha, y en profundidad, z hacia arriba (dorsal)."""
    return np.column_stack([um[:, 0], um[:, 2], -um[:, 1]])


def load_skeletons(center):
    """Morfología simplificada (fly_brain/download_skeletons.py) lista para dibujar, o None."""
    import pyvista as pv
    from connectome import DATA
    path = DATA / "skeletons_lite.npz"
    if not path.exists():
        return None
    z = np.load(path)
    order = np.argsort(z["owner"], kind="stable")  # segmentos agrupados por neurona
    segs, owner = z["segs"][order], z["owner"][order]
    verts = (to_view(z["verts"].astype(float)) - center).astype(np.float32)
    n = int(owner.max()) + 1
    start = np.searchsorted(owner, np.arange(n))
    count = np.bincount(owner, minlength=n)
    lines = np.column_stack([np.full(len(segs), 2), segs]).ravel()
    return {"verts": verts, "segs": segs, "owner": owner, "start": start, "count": count,
            "n_neurons": int((count > 0).sum()), "mesh": pv.PolyData(verts, lines=lines)}


def pd_read_positions(connectome):
    """Posiciones (vóxeles de FlyWire) de cada neurona en el orden del simulador."""
    import pandas as pd
    from connectome import DATA
    pos = pd.read_csv(DATA / "neuron_annotations.tsv", sep="\t", usecols=["root_id", "pos_x", "pos_y", "pos_z"])
    pos = pos.drop_duplicates("root_id").set_index("root_id").reindex(connectome.root_ids)
    return pos[["pos_x", "pos_y", "pos_z"]].fillna(pos.mean()).to_numpy(float)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--device", choices=["auto", "gpu", "cpu"], default="auto")
    ap.add_argument("--record", help="guardar un vídeo (.mp4) o GIF en esta ruta en vez de abrir la ventana")
    ap.add_argument("--seconds", type=float, default=30.0, help="duración del vídeo")
    ap.add_argument("--speed", type=float, default=2.0, help="pasos de simulación por fotograma del vídeo")
    args = ap.parse_args()
    viewer = BrainViewer(args.device, off_screen=bool(args.record))
    if args.record:
        n = viewer.record(args.record, args.seconds, speed=args.speed)
        print(f"Vídeo: {args.record} ({n} fotogramas)")
    else:
        viewer.run()


if __name__ == "__main__":
    main()
