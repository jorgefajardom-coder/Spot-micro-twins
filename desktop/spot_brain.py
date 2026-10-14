"""Spot Micro con el cerebro de la mosca explorando una cueva: busca una fuga de gas,
rodea las fuentes de calor y mapea la cueva con un LiDAR 3D.

El conectoma completo de FlyWire (138.639 neuronas) decide; gait.py camina; sim_world.py
simula la cueva, el gas, el calor, el terreno y el LiDAR. Por defecto la cueva es real: la
Grotte de la Sourde escaneada con LiDAR (KarstConduitCatalogue, Racine et al. 2025).

    robot en la cueva  |  cerebro           |  mapa LiDAR
                       |  ojos              |  descendentes
                       |  últimos segundos  |  sensores

Los sensores entran al cerebro por sustitución sensorial (ver sim_world.CaveWorld.senses):
en este modelo las vías olfativa y térmica saturan y no dan dirección, así que el gradiente
de gas usa el circuito de persecución visual (LC10/LC9) y el calor fuerte el de amenaza
(LPLC2/LC4 -> Giant Fiber). Render con PyVista/VTK (OpenGL, z-buffer, FXAA, SSAO).

Se ejecuta con el entorno de fly_brain (CuPy/numba, PyVista y los datos de FlyWire):

    fly_brain\\.venv\\Scripts\\python spot_brain.py
    fly_brain\\.venv\\Scripts\\python spot_brain.py --record spot.mp4 --seconds 30 --speed 2
    fly_brain\\.venv\\Scripts\\python spot_brain.py --cueva generada

Teclas:  t terreno escarpado (o el botón) · espacio lanzar un obstáculo · r reiniciar
"""
import argparse
import math
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "fly_brain"))

from cave_explorer import (MAP_BG, SCAN_COLORS, Explorer, build_cave_model, gradient,  # noqa: E402
                           reconstruct_surface, save_model)
from gait import BODY_HEIGHT, GaitManager  # noqa: E402
from kinematics import Kinematics  # noqa: E402
from sim_world import TEMP_AVOID, CaveWorld, RealCave  # noqa: E402
import spot_arm  # noqa: E402
from spot_model3d import BONES, DEFAULT_MODEL_FILE, SpotModel, solve_skeleton  # noqa: E402

# Modelo de alta resolución (models/build_model.py --full); si no existe, el ligero
HD_MODEL_FILE = HERE / "models" / "spot_model_hd.npz"

WINDOW = (1920, 1040)
ROBOT_WIDTH = 0.44                             # fracción del ancho para el robot; el resto, pestañas
TAB_BAR = 0.055                                # fracción del alto para la barra de pestañas
BORDER = "#26354a"                              # borde entre pantallas
BG = "#0e1218"
YELLOW, CYAN, RED, GREEN, GREY = "#f4c542", "#40c4ff", "#ff5252", "#69f0ae", "#8fa5be"
ORANGE, GAS_GREEN = "#ff7a1a", "#7dff6b"
VOXEL_UM = np.array([0.004, 0.004, 0.040])    # tamaño de vóxel de FlyWire en µm (x, y, z)
ACTIVITY_TAU_MS = 120.0                        # cuánto tarda en apagarse una neurona en pantalla
HISTORY_S = 8.0                                # segundos en las gráficas de historia
BRAIN_DISTANCE = 940.0                         # µm: cámara frontal, el cerebro llena la pantalla
BRAIN_BG = "#0b0c22"                           # azul noche, como las renderizaciones de FlyWire
ACTIVE_CMAP = ["#ff4d00", "#ff9a1f", "#ffd36b"]   # actividad: naranja intenso -> ámbar
ACTIVE_SEGMENTS = 15_000                       # segmentos dibujados como actividad por fotograma
MAX_SYNAPSE_SOURCES = 220                      # neuronas que dibujan sus sinapsis por fotograma
SYNAPSES_PER_NEURON = 3                        # sus conexiones más fuertes
SYN_EXCITATORY, SYN_INHIBITORY = "#ffb347", "#4fc3f7"
# Colores de neurona al estilo de las renderizaciones de FlyWire
NEURON_PALETTE = ["#ff4fd8", "#b45cff", "#7a5cff", "#4f7dff", "#3db8ff", "#29e3d8", "#ffd23f", "#ff6f91"]
ACTIVE_THRESHOLD = 0.12                        # actividad a partir de la que una neurona brilla
MAX_ACTIVE_DRAWN = 600                         # neuronas activas candidatas (las más activas)
CAMERA_OFFSET = np.array([-1.0, -1.3, 0.95])    # cámara del robot respecto a él (~1,9 m)
REAL_CAMERA_OFFSET = np.array([-2.1, -2.6, 1.0])   # en la galería de mina, más lejos y bajo (~3,5 m)
MAP_CAMERA_OFFSET = np.array([-3.4, -4.4, 5.6])  # cámara del mapa: desde arriba, siguiendo al robot
MAP_CUT = 0.5                                     # m: el mapa enseña solo lo de debajo (sin el techo)
REAL_MAP_CUT = 1.2                                # m: lo mismo en la galería de mina
MAP_MESH_PERIOD = 1.5                             # s: cada cuánto se reconstruye la superficie del mapa
CAVE_CULLING = "front"                            # se ve la cara de la galería, no la de delante
FLOOR_TILE_M = 2.5                                # m que cubre una tesela de la textura del suelo
REVEAL_RADIUS = 0.35                              # m: lo que un impacto del LiDAR descubre de la cueva
REVEAL_NEAR = 0.9                                 # m: alrededor del sensor se descubre todo
REVEAL_GLOW_S = 0.8                               # s: destello cian al descubrirse, apagándose
REVEAL_RGB = (90, 230, 255)                       # color del destello
UNSEEN_RGB = (6, 8, 12)                           # la cueva aún sin escanear: a oscuras
REAL_CAVE_CUT = 1.3                               # m sobre el suelo: techo quitado para ver al robot

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


def cave_floor(pv, world, cell=0.03, margin=0.06):
    """Suelo de la cueva (rejilla con la altura del terreno que pisan los pies), solo dentro de
    la galería, con coordenadas de textura en metros del mundo."""
    cave = world.cave
    lo, hi = cave.center.min(0) - 1.4, cave.center.max(0) + 1.4
    gx, gy = np.arange(lo[0], hi[0], cell), np.arange(lo[1], hi[1], cell)
    X, Y = np.meshgrid(gx, gy, indexing="ij")
    grid = pv.StructuredGrid(X, Y, world.ground(X, Y))
    d, i = cave._tree.query(np.column_stack([X.ravel(order="F"), Y.ravel(order="F")]))
    grid.point_data["dentro"] = (d < cave.radius[i] + margin).astype(float)
    floor = grid.threshold(0.5, scalars="dentro").extract_surface(algorithm="dataset_surface")
    floor.active_texture_coordinates = floor.points[:, :2] / FLOOR_TILE_M
    rng = np.random.default_rng(1)
    floor.point_data["roca"] = 0.5 + 0.08 * rng.standard_normal(floor.n_points)  # grano fino
    return floor


def periodic_noise(rng, size, scale):
    """Ruido suave que se repite sin costuras (filtrado en frecuencia); scale en píxeles."""
    f = np.fft.fftfreq(size)
    k = np.hypot(*np.meshgrid(f, f, indexing="ij"))
    spectrum = np.fft.fft2(rng.standard_normal((size, size))) * np.exp(-(k * scale) ** 2)
    n = np.real(np.fft.ifft2(spectrum))
    return (n - n.mean()) / n.std()


def floor_texture(size=1024, seed=11):
    """Textura del suelo de la cueva (tesela sin costuras): sedimento, grava con volumen,
    grietas y grano fino, iluminada a partir de un mapa de alturas."""
    rng = np.random.default_rng(seed)
    big, mid, fine = (periodic_noise(rng, size, s) for s in (90, 22, 2.5))
    height = 0.6 * big + 0.35 * mid + 0.12 * fine
    ridge = np.abs(periodic_noise(rng, size, 40))
    cracks = np.clip(1 - ridge / 0.06, 0, 1) * (periodic_noise(rng, size, 120) > 0.2)
    height -= 1.2 * cracks
    sediment = np.array(hex_rgb("#6e5a46")), np.array(hex_rgb("#9a8670"))
    t = np.clip(0.5 + 0.3 * big + 0.15 * mid, 0, 1)[..., None]
    albedo = sediment[0] + t * (sediment[1] - sediment[0])
    # Grava: piedras con forma de cúpula (dan sombra con la iluminación), envueltas en la tesela
    yy, xx = np.mgrid[0:size, 0:size]
    for _ in range(520):
        r = rng.uniform(3, 9) if rng.random() < 0.8 else rng.uniform(10, 26)
        aspect, ang = rng.uniform(0.45, 1.0), rng.uniform(0, np.pi)
        cx, cy = rng.uniform(0, size, 2)
        x0, y0 = int(cx - r) - 1, int(cy - r) - 1
        n = int(2 * r) + 3
        ix, iy = (np.arange(x0, x0 + n) % size), (np.arange(y0, y0 + n) % size)
        dx = (np.arange(x0, x0 + n) - cx)[:, None]
        dy = (np.arange(y0, y0 + n) - cy)[None, :]
        u = dx * np.cos(ang) + dy * np.sin(ang)
        v = (-dx * np.sin(ang) + dy * np.cos(ang)) / aspect
        q = 1 - (u ** 2 + v ** 2) / r ** 2 + 0.35 * mid[np.ix_(ix, iy)] * (r / 26)  # canto irregular
        inside = q > 0
        dome = np.sqrt(np.clip(q, 0, 1)) * r * 0.08
        sub = np.ix_(ix, iy)
        height[sub] = np.where(inside, np.maximum(height[sub], dome + 0.2), height[sub])
        tone = np.array(hex_rgb(rng.choice(["#8d8478", "#a39a8c", "#6f665b", "#b3a48c", "#5d5248"])))
        tone = tone * rng.uniform(0.85, 1.1)
        albedo[sub] = np.where(inside[..., None], tone, albedo[sub])
    # Iluminación horneada (luz rasante desde arriba a la izquierda) y grano fino
    gx, gy = np.gradient(height)
    shade = np.clip(0.75 - 0.9 * gx - 0.6 * gy, 0.25, 1.25)
    damp = np.clip(periodic_noise(rng, size, 70) - 0.4, 0, 1)  # manchas de humedad, más oscuras
    img = (albedo * shade[..., None] * (1 + 0.06 * fine[..., None]) * (1 - 0.55 * cracks[..., None])
           * (1 - 0.3 * damp[..., None]))
    return (np.clip(img, 0, 1) * 255).astype(np.uint8)


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
        z = world.ground(xy[:, 0], xy[:, 1]) + rng.uniform(0.02, 1, len(s)) * min(h.min(), 0.8) * np.sqrt(1 - q ** 2) * 0.9
        ppm = world.gas_ppm(xy)
        keep = rng.uniform(0, 1, len(s)) < ((ppm - 5) / 900) ** 0.6
        pts.append(np.column_stack([xy[keep], z[keep]]))
    return np.concatenate(pts)[:n]


def rock_detail(points, seed=0):
    """Grano fino de roca (varias octavas de ondas 3D) para dar textura al suelo y las paredes."""
    rng = np.random.default_rng(seed)
    n = np.zeros(len(points))
    for octave in range(5):
        k = rng.normal(size=(3, 3)) * (6.0 * 2.1 ** octave)
        n += np.sin(points @ k + rng.uniform(0, 2 * np.pi, 3)).sum(axis=1) / (1.7 ** octave)
    return n / np.abs(n).max()


def real_cave_mesh(pv, cave, cut=REAL_CAVE_CUT):
    """Superficie de la cueva real con los colores del escáner y grano de roca, sin el techo.

    La superficie sale de los puntos LiDAR (misma reconstrucción que el mapa) y cada vértice
    toma el color del punto escaneado más cercano. Se quita lo que queda a más de `cut` sobre
    el suelo para ver al robot desde arriba.
    """
    surf = reconstruct_surface(pv, cave.points, cave.floor, cut=None, voxel=0.07 * cave.scale, sigma=1.0)
    height = surf.points[:, 2] - cave.floor(surf.points[:, 0], surf.points[:, 1])
    surf.point_data["h"] = height
    surf = surf.clip_scalar(scalars="h", value=cut, invert=True)
    # La reconstrucción es una lámina con dos caras: se queda solo la que mira a la galería
    # (sin la de fuera, la pared cercana no tapa al robot con culling)
    surf = surf.compute_normals(cell_normals=True, point_normals=False, auto_orient_normals=False,
                                consistent_normals=False)
    centers = surf.cell_centers().points
    axis = np.column_stack([cave.center, cave.floor(cave.center[:, 0], cave.center[:, 1]) + 0.25])
    _, i = cKDTree(cave.center).query(centers[:, :2])
    to_axis = axis[i] - centers  # hacia el aire de la galería
    dist = np.linalg.norm(to_axis, axis=1)
    facing = np.einsum("ij,ij->i", surf.cell_data["Normals"], to_axis) > 0
    inner = facing if np.median(dist[facing]) < np.median(dist[~facing]) else ~facing
    surf = surf.extract_cells(np.flatnonzero(inner)).extract_surface(algorithm="dataset_surface")
    # Fuera las astillas sueltas que deja el recorte (se cruzarían delante del robot)
    surf = surf.connectivity("all")
    sizes = np.bincount(surf.cell_data["RegionId"])
    keep = np.flatnonzero(sizes[surf.cell_data["RegionId"]] > 0.02 * surf.n_cells)
    surf = surf.extract_cells(keep).extract_surface(algorithm="dataset_surface")
    height = surf.point_data["h"]
    height = surf.point_data["h"]
    # Luminosidad del escáner promediada entre vecinos (un punto suelto es puro ruido)
    if cave.colors is not None:
        _, idx = cKDTree(cave.points).query(surf.points, k=16)
        lum = cave.colors[idx].astype(float).mean(axis=(1, 2))
        lo, hi = np.percentile(lum, [3, 97])
        lum = np.clip((lum - lo) / (hi - lo), 0, 1)
    else:
        lum = np.full(surf.n_points, 0.5)
    rock = np.array([hex_rgb(c) for c in ("#3a332b", "#b8aa92")]) * 255   # caliza: sombra / luz
    rgb = rock[0] + lum[:, None] * (rock[1] - rock[0])
    mud = np.array(hex_rgb("#6f5a45")) * 255                              # sedimento del suelo
    floor = np.clip(1 - height / 0.12, 0, 1)[:, None]
    rgb = rgb * (1 - 0.55 * floor) + mud * 0.55 * floor
    grain = rock_detail(surf.points)
    shade = 1.0 + (0.18 + 0.12 * floor[:, 0]) * grain                      # el suelo, más granulado
    surf.point_data["rgb"] = np.clip(rgb * shade[:, None], 0, 255).astype(np.uint8)
    return surf


# =============================================================================
# Interfaz
# =============================================================================

class BrainViewer:
    """Robot en la cueva a la izquierda; a la derecha dos pantallas a la vez, en tres pestañas:

        1  cerebro          / mapa LiDAR
        2  ojos (estímulo)  / descendentes
        3  últimos segundos / sensores (gas y temperatura)
    """

    # Cada pantalla es un renderer propio; el layout (viewports) lo fija _layout según la pestaña
    ROBOT, TABBAR = (0, 0), (0, 1)
    BRAIN, MAP = (1, 0), (1, 1)
    EYES, DNS = (2, 0), (2, 1)
    HISTORY, SENSORS = (3, 0), (3, 1)
    TABS = (("Brain · LiDAR map", BRAIN, MAP), ("Eyes · Descending", EYES, DNS),
            ("History · Sensors", HISTORY, SENSORS))

    def __init__(self, device="auto", off_screen=False, size=WINDOW, cave="real"):
        import pyvista as pv
        from connectome import Connectome
        from pilot import FlyPilot, Senses
        self.pv, self.Senses = pv, Senses
        self.W, self.H = size

        print("Loading the FlyWire connectome…", flush=True)
        self.connectome = Connectome.load()
        self.pilot = FlyPilot(self.connectome, device=device)
        gain = self.pilot.calibrate()
        self.backend = "GPU" if type(self.pilot.brain).__name__ == "BrainGPU" else "CPU"
        print(f"Brain on {self.backend} · calibration {({k: round(v) for k, v in gain.items()})}")

        self.kin = Kinematics()
        self.gait = GaitManager(self.kin)
        self.gait.set_gait("trotting")
        self.gait.gait_speed = GAIT_SPEED
        self.world = CaveWorld(cave=RealCave() if cave == "real" else None)
        self.real = getattr(self.world.cave, "kind", "") == "real"
        self.readings = {"gas_ppm": 0.0, "temp_c": 0.0, "modo": "explorando"}
        self.activity = np.zeros(self.connectome.n, np.float32)
        self.cmd = None
        z0 = float(self.world.ground(*self.world.pose[:2]))
        self.focus = np.r_[self.world.pose[:2], z0 + 0.12]
        self.map_focus = np.r_[self.world.pose[:2], z0 + 0.3]
        self.last = time.perf_counter()
        self.last_dt = 0.04
        self.t = 0.0
        self.hist_t, self.hist, self.hist_lines = [], {}, {}
        self.scanning = True
        self.off_screen = off_screen
        self.saved_model = None
        # Dianas más fuertes de cada neurona, precalculadas: en cada fotograma solo se indexan
        self.syn_post, self.syn_sign = top_targets(self.connectome.weights, SYNAPSES_PER_NEURON)

        pv.global_theme.font.family = "arial"
        self.pl = pv.Plotter(shape=(4, 2), window_size=size, off_screen=off_screen, border=False,
                             title="Spot Micro Twins · Fly brain in the cave")
        self.pl.set_background(BG)
        self._build_robot()
        self._build_brain()
        self._build_map()
        self._build_charts()
        self._build_tabs()
        # FXAA: SSAA renderiza al doble (descoloca las gráficas 2D) y MSAA rompe el SSAO
        self.pl.enable_anti_aliasing("fxaa")

    def _panel_height(self):
        return int(self.H * (1 - TAB_BAR) / 2)

    def _renderer(self, loc):
        return self.pl.renderers[self.pl.renderers.loc_to_index(loc)]

    def _layout(self):
        """Robot a la izquierda, barra de pestañas arriba a la derecha y las dos pantallas de la
        pestaña activa una encima de otra; las demás no se dibujan."""
        rw, top = ROBOT_WIDTH, 1 - TAB_BAR
        self._renderer(self.ROBOT).SetViewport(0, 0, rw, 1)
        self._renderer(self.TABBAR).SetViewport(rw, top, 1, 1)
        _, upper, lower = self.TABS[self.tab]
        for _, *locs in self.TABS:
            for loc in locs:
                ren = self._renderer(loc)
                shown = loc in (upper, lower)
                ren.SetDraw(shown)
                ren.SetInteractive(shown)
                if shown:
                    ren.SetViewport(rw, top / 2 if loc == upper else 0, 1, top if loc == upper else top / 2)
                else:
                    ren.SetViewport(0, 0, 1e-4, 1e-4)
        brain = self._renderer(self.BRAIN)
        self.brain_overlay.SetViewport(brain.GetViewport())
        self.brain_overlay.SetDraw(brain.GetDraw())

    def set_tab(self, i):
        """Pestañas 1-3 (botones o teclas): qué dos pantallas se ven."""
        self.tab = i
        for k, button in enumerate(self.tab_buttons):
            button.GetRepresentation().SetState(int(k == i))
        self._layout()
        self.pl.render()

    def _build_tabs(self):
        pl = self.pl
        pl.subplot(*self.TABBAR)
        pl.set_background("#151c27", all_renderers=False)
        self.tab_buttons = []
        for i, (name, *_) in enumerate(self.TABS):
            x = 16 + i * 330
            self.tab_buttons.append(pl.add_checkbox_button_widget(
                lambda on, i=i: self.set_tab(i), value=i == 0, position=(x, 14), size=26, border_size=2,
                color_on=CYAN, color_off="#2a3646", background_color="#8fa5be"))
            pl.add_text(f"{i + 1}  {name}", position=(x + 36, 17), font_size=11, color="w")
        for ren in self.pl.renderers:
            ren.add_border(color=BORDER, width=2)  # en coordenadas del panel: sigue al layout
        self.tab = 0
        self._layout()

    # ----- el robot en la cueva -----
    def _build_robot(self):
        pv, pl, H, w = self.pv, self.pl, self.H, self.world
        pl.subplot(*self.ROBOT)
        hd = HD_MODEL_FILE.exists()
        model = SpotModel(self.kin, str(HD_MODEL_FILE if hd else DEFAULT_MODEL_FILE))
        print(f"Robot model: {model.source} ({model.face_count:,} faces)")
        style = dict(scalars="rgb", rgb=True, smooth_shading=True, ambient=0.25, diffuse=0.8,
                     specular=0.35, specular_power=25)
        mesh = lambda p: part_mesh(pv, *p, smooth=not hd)  # noqa: E731
        self.body_actors = [pl.add_mesh(mesh(p), **style) for p in model.body]
        # Brazo con cabeza (models/arm_mod, si está descargado) y el LiDAR en la cabeza
        self.arm, self.arm_actors = None, {}
        if spot_arm.available():
            self.arm = spot_arm.SpotArm(pv)
            for link, parts in self.arm.parts.items():
                self.arm_actors[link] = [pl.add_mesh(m, color=col, smooth_shading=True, ambient=0.25,
                                                     diffuse=0.8, specular=0.35, specular_power=25)
                                         for m, col in parts]
            level = transform(np.eye(3), [0, 0, BODY_HEIGHT])
            lo = self.arm.lidar_origin(level)
            w.lidar_mount = (float(lo[0]), float(lo[2] - BODY_HEIGHT))
        self.leg_actors = [{bone: [pl.add_mesh(mesh(p), **style) for p in model.legs[leg][bone]]
                            for bone in BONES} for leg in range(4)]

        self.floor_texture = pv.Texture(floor_texture())
        self.floor_texture.repeat = True
        self.floor_texture.mipmap = True
        self.floor_texture.interpolate = True
        if self.real:
            self.cave_mesh = real_cave_mesh(pv, w.cave)
            # La cueva aparece a medida que el LiDAR la toca: empieza a oscuras
            self.cave_rgb = np.asarray(self.cave_mesh.point_data["rgb"], np.float32).copy()
            self.cave_tree = cKDTree(self.cave_mesh.points)
            self.cave_seen = np.full(self.cave_mesh.n_points, -np.inf, np.float32)
            self.cave_mesh.point_data["rgb"][:] = UNSEEN_RGB
            # El suelo no se descubre: está desde el principio y no cambia mientras el Spot avanza
            low = self.cave_mesh.point_data["h"] < 0.15
            self.cave_seen[low] = -1e6
            self.cave_mesh.point_data["rgb"][low] = self.cave_rgb[low].astype(np.uint8)
            real = pl.add_mesh(self.cave_mesh, scalars="rgb", rgb=True, smooth_shading=True, ambient=0.3,
                               diffuse=0.8, specular=0.08)
            real.prop.culling = CAVE_CULLING  # la pared que mira a la cámara no tapa al robot
            # Suelo propio: la misma altura que pisan los pies (la malla del escaneo queda algo
            # más baja) y la textura de sedimento y grava
            self.floor_mesh = cave_floor(pv, w, cell=0.06, margin=0.3)
            pl.add_mesh(self.floor_mesh, texture=self.floor_texture, smooth_shading=True, ambient=0.3,
                        diffuse=0.85, specular=0.06)
        else:
            rock = dict(scalars="roca", cmap=gradient(["#2a211c", "#4d4036", "#6e6253", "#8a7c69"]),
                        clim=[0.2, 1.0], show_scalar_bar=False, smooth_shading=True, ambient=0.3,
                        diffuse=0.75, specular=0.05)
            shell = pl.add_mesh(cave_shell(pv, w.cave), **rock)
            shell.prop.culling = "front"  # la pared que mira a la cámara no tapa al robot
            self.floor_mesh = cave_floor(pv, w)
            pl.add_mesh(self.floor_mesh, texture=self.floor_texture, smooth_shading=True, ambient=0.3,
                        diffuse=0.85, specular=0.06)

        # Fuente de calor: núcleo incandescente y halo
        hx, hy = w.heat_xy
        hz = float(w.ground(hx, hy))
        pl.add_mesh(pv.Sphere(radius=0.07, center=(hx, hy, hz + 0.08)), color=ORANGE, ambient=1.0, diffuse=0.0)
        pl.add_mesh(pv.Sphere(radius=0.32, center=(hx, hy, hz + 0.14)), color=RED, opacity=0.12, ambient=1.0,
                    diffuse=0.0)
        # Penacho de gas
        pl.add_mesh(pv.PolyData(gas_particles(w)), color=GAS_GREEN, point_size=7, opacity=0.22,
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

        cave_name = (f"Grotte de la Sourde ×{w.cave.scale:g} · mine gallery"
                     if self.real else "generated cave")
        pl.add_text(f"SPOT MICRO  ·  {cave_name}", position=(20, H - 40), font_size=12, color=YELLOW)
        self.state_text = pl.add_text(" ", position=(22, 128), font_size=13, color=GREEN)
        # Botón: detener el escaneo y generar la cueva completa para explorarla
        self.stop_button = pl.add_checkbox_button_widget(  # posición relativa al panel, como el texto
            lambda on: self.stop_scan(on), value=False, position=(22, 82), size=30, border_size=2,
            color_on=GAS_GREEN, color_off="#2a3646", background_color="#8fa5be")
        pl.add_text("Stop scan and build the 3D cave", position=(62, 88), font_size=11, color="w")
        self.rough_button = pl.add_checkbox_button_widget(
            lambda on: self.set_rough(on), value=False, position=(22, 44), size=30, border_size=2,
            color_on=YELLOW, color_off="#2a3646", background_color="#8fa5be")
        pl.add_text("Rough terrain", position=(62, 50), font_size=11, color="w")
        pl.add_text("1-3 tabs · t terrain · space obstacle · r reset", position=(22, 14), font_size=9, color=GREY)
        if self.real:
            pl.add_text(w.cave.attribution, position="upper_right", font_size=7, color=GREY)

    # ----- cerebro -----
    def _build_brain(self):
        pv, pl, c = self.pv, self.pl, self.connectome
        pl.subplot(*self.BRAIN)
        pl.set_background(BRAIN_BG, all_renderers=False)
        h = self._panel_height()
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
            print(f"Morphology: {sk['n_neurons']:,} neurons, {len(sk['segs']):,} segments")
            # Las sinapsis salen del centro de la morfología de cada neurona (dentro del neuropilo),
            # no del soma, que está en la corteza
            v = sk["verts"][sk["segs"][:, 0]]
            cent = np.zeros((c.n, 3))
            np.add.at(cent, sk["owner"], v)
            has = sk["count"] > 0
            self.brain_pts = np.where(has[:, None], cent / np.maximum(sk["count"], 1)[:, None], pts)
            # Colores al estilo de FlyWire: uno por tipo celular, cian/turquesa/amarillo en los
            # lóbulos ópticos y violeta/magenta/azul en el cerebro central
            mesh = sk["mesh"]
            mesh.cell_data["rgb"] = (neuron_colors(c.annotations)[sk["owner"]] * 255 * 0.78).astype(np.uint8)
            pl.add_mesh(mesh, scalars="rgb", rgb=True, line_width=1.0, opacity=1.0, lighting=False)

        self.n_active_text = pl.add_text(" ", position=(12, h - 28), font_size=10, color="w")
        # Actividad en una capa encima que comparte la cámara: las neuronas que disparan se ven
        # brillar dentro del cerebro aunque haya otras neuronas delante
        self.brain_overlay = pv.Renderer(pl, border=False)
        self.brain_overlay.SetLayer(1)
        self.brain_overlay.SetActiveCamera(pl.renderer.GetActiveCamera())
        self.brain_overlay.SetInteractive(False)
        pl.ren_win.AddRenderer(self.brain_overlay)
        if self.skeletons is not None:
            self.active_lines = LiveLines(pl, pv, points=self.skeletons["verts"], scalars="act",
                                          cmap=ACTIVE_CMAP, clim=[0.0, 1.0], line_width=1.4,
                                          opacity=0.75)
        self.synapse_lines = LiveLines(pl, pv, scalars="signo", cmap=[SYN_INHIBITORY, SYN_EXCITATORY],
                                       clim=[-1, 1], line_width=1.0, opacity=0.35)
        for live in (getattr(self, "active_lines", None), self.synapse_lines):
            if live is not None:
                pl.renderer.remove_actor(live.actor, render=False)
                self.brain_overlay.add_actor(live.actor)
        self._place_brain_camera()  # vista frontal fija; se gira con el ratón si se quiere

    def _place_brain_camera(self):
        """Cerebro entero en vista frontal, llenando la pantalla."""
        cam = self.pl.camera
        cam.focal_point = (0.0, 0.0, 0.0)
        cam.position = (0.0, -BRAIN_DISTANCE, 0.0)
        cam.up = (0, 0, 1)
        self.pl.renderer.reset_camera_clipping_range()

    # ----- mapa del LiDAR -----
    def _build_map(self):
        pv, pl, w = self.pv, self.pl, self.world
        pl.subplot(*self.MAP)
        h = self._panel_height()
        self.map_cut = REAL_MAP_CUT if self.real else MAP_CUT
        pl.set_background(MAP_BG, all_renderers=False)
        z0 = float(np.percentile(w.ground(w.cave.center[:, 0], w.cave.center[:, 1]), 2)) - 0.1
        c = w.cave.center.mean(axis=0)
        grid = pv.Plane(center=(c[0], c[1], z0), i_size=60, j_size=60, i_resolution=120, j_resolution=120)
        pl.add_mesh(grid, style="wireframe", color="#2c3038", line_width=1, lighting=False)
        # Superficie reconstruida (se rehace cada MAP_MESH_PERIOD s) y barrido actual del LiDAR
        dummy = pv.Sphere(radius=0.01)
        dummy.point_data["altura"] = np.zeros(dummy.n_points)
        self.map_surface = dummy
        self.map_surface_actor = pl.add_mesh(dummy, scalars="altura", cmap=gradient(SCAN_COLORS),
                                             clim=[0, self.map_cut], show_scalar_bar=False, smooth_shading=True,
                                             ambient=0.25, diffuse=0.85, specular=0.15, specular_power=12)
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
        self.map_text = pl.add_text(" ", position=(12, h - 28), font_size=10, color="w")

    # ----- gráficas: una pantalla cada una -----
    def _build_charts(self):
        pv, pl = self.pv, self.pl
        full = dict(loc=(0.02, 0.03), size=(0.96, 0.94))
        pl.subplot(*self.EYES)
        self.in_keys = ["objetivo_izq", "objetivo_der", "delante_izq", "delante_der",
                        "looming_izq", "looming_der"]
        chart, self.in_series = bar_chart(
            pv, "Eyes: stimulus (Hz)", ["LC10 L", "LC10 R", "LC9 L", "LC9 R", "LPLC2 L", "LPLC2 R"],
            [GREEN, GREEN, CYAN, CYAN, RED, RED], 160, **full)
        pl.add_chart(chart)
        pl.subplot(*self.DNS)
        self.out_keys = ["P9_izq", "P9_der", "DNa02_izq", "DNa02_der", "MDN", "GF"]
        chart, self.out_series = bar_chart(
            pv, "Descending: brain to body (Hz)",
            ["P9 L", "P9 R", "DNa02 L", "DNa02 R", "MDN back", "Giant Fiber"],
            [CYAN, CYAN, GREEN, GREEN, YELLOW, RED], 300, **full)
        pl.add_chart(chart)
        pl.subplot(*self.HISTORY)
        chart, lines = history_chart(
            pv, "Last seconds", (("vx", "#ffffff", "forward"), ("giro", GREEN, "turn"),
                                     ("P9", CYAN, "P9"), ("GF", RED, "Giant Fiber")), (-1.1, 1.1), **full)
        self.hist_lines.update(lines)
        pl.add_chart(chart)
        pl.subplot(*self.SENSORS)
        gas, lines = history_chart(pv, "Gas (ppm)", (("gas", GAS_GREEN, "gas"),), (0, 1000),
                                   loc=(0.02, 0.52), size=(0.96, 0.46))
        self.hist_lines.update(lines)
        temp, lines = history_chart(pv, "Temperature (°C)", (("temp", ORANGE, "temp"),), (15, 100),
                                    loc=(0.02, 0.03), size=(0.96, 0.46))
        self.hist_lines.update(lines)
        pl.add_chart(gas, temp)
        self.hist = {k: [] for k in self.hist_lines}

    # ----- control -----
    def set_rough(self, on):
        """Botón / tecla t: terreno escarpado. El suelo, el LiDAR y los pies lo notan a la vez."""
        w = self.world
        w.terrain.rough = bool(on)
        pts = self.floor_mesh.points
        pts[:, 2] = w.ground(pts[:, 0], pts[:, 1])
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
        self.state_text.SetInput("SCAN STOPPED  ·  building the 3D cave…")
        self.state_text.GetTextProperty().SetColor(*hex_rgb(GAS_GREEN))
        self.pl.render()
        mesh, meta = build_cave_model(self.world)
        if mesh is None:
            self.state_text.SetInput("SCAN STOPPED  ·  not enough data yet")
            return
        self.saved_model = save_model(mesh, meta)
        print(f"Cave saved: {self.saved_model} (+ .ply for Blender)")
        self.state_text.SetInput(f"CAVE SAVED  ·  scans/{self.saved_model.name}")
        if not self.off_screen:
            import subprocess
            subprocess.Popen([sys.executable, str(HERE / "cave_explorer.py"), str(self.saved_model)])

    def on_key(self, key):
        if key == "space":
            self.world.launch_obstacle()
        elif key == "t":
            self.set_rough(not self.world.terrain.rough)
        elif key in ("1", "2", "3"):
            self.set_tab(int(key) - 1)
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
        if self.tab == 0:  # solo se dibujan las pantallas visibles (las gráficas acumulan siempre)
            self._draw_brain()
            self._draw_map()
        self._draw_charts()

    def _draw_robot(self, feet):
        pl, w = self.pl, self.world
        c, s = math.cos(w.pose[2]), math.sin(w.pose[2])
        rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        feet_w, rpy, body = w.body_on_terrain(feet @ rz.T + [w.pose[0], w.pose[1], 0], BODY_HEIGHT)
        skel = solve_skeleton(self.kin, feet_w.tolist(), rpy, body.tolist())
        body_tf = transform(*skel["body"])
        for a in self.body_actors:
            a.user_matrix = body_tf
        if self.arm is not None:
            for link, m in self.arm.transforms(body_tf).items():
                for a in self.arm_actors[link]:
                    a.user_matrix = m
        for leg, bones in enumerate(skel["legs"]):
            for bone in BONES:
                for a in self.leg_actors[leg][bone]:
                    a.visibility = bones is not None
                    if bones is not None:
                        a.user_matrix = transform(*bones[bone])

        pl.subplot(*self.ROBOT)
        self.obstacle_actor.visibility = w.obstacle is not None
        if w.obstacle is not None:
            self.obstacle_actor.position = (*w.obstacle, float(w.ground(*w.obstacle)))
        hits, _ = w.last_scan
        hits = hits[~np.isnan(hits[:, 0])]
        self.scan_points.update(hits)
        if self.real:
            self._reveal_cave(hits, np.r_[w.pose[:2], body[2]])
        if len(w.trail) > 1:
            tr = np.array(w.trail)
            tr = np.column_stack([tr, w.ground(tr[:, 0], tr[:, 1]) + 0.005]).astype(np.float32)
            n = len(tr)
            self.trail.update(np.column_stack([np.arange(n - 1), np.arange(1, n)]), points=tr)
        # Cámara cerca del robot, suavizada para no marear
        robot = np.r_[w.pose[:2], body[2] - 0.07]
        self.focus += 0.35 * (robot - self.focus)
        pl.camera.focal_point = self.focus
        pl.camera.position = self.focus + (REAL_CAMERA_OFFSET if self.real else CAMERA_OFFSET)
        pl.camera.up = (0, 0, 1)
        pl.renderer.reset_camera_clipping_range()  # sin esto VTK recorta el robot al moverse

        rd, cmd = self.readings, self.cmd
        if cmd.escape:
            msg, col = "ESCAPE!  Giant Fiber: backing off", RED
        elif rd["modo"] == "fuga localizada":
            msg, col = f"LEAK FOUND  ·  {rd['gas_ppm']:.0f} ppm", GAS_GREEN
        elif rd["temp_c"] > TEMP_AVOID:
            msg, col = f"AVOIDING HEAT  ·  {rd['temp_c']:.0f} °C", ORANGE
        elif rd["modo"] == "siguiendo gas":
            msg, col = f"FOLLOWING THE GAS  ·  {rd['gas_ppm']:.0f} ppm", GAS_GREEN
        else:
            msg, col = "EXPLORING THE CAVE", CYAN
        self.state_text.SetInput(msg)
        self.state_text.GetTextProperty().SetColor(*hex_rgb(col))

    def _reveal_cave(self, hits, sensor):
        """Los vértices cerca de los impactos del LiDAR se descubren con un destello cian que se
        apaga hasta el color de la roca; lo que no se ha escaneado sigue a oscuras."""
        n = len(self.cave_seen)
        if len(hits):
            _, idx = self.cave_tree.query(hits, k=24, distance_upper_bound=REVEAL_RADIUS)
            idx = np.unique(np.r_[idx[idx < n], self.cave_tree.query_ball_point(sensor, REVEAL_NEAR)])
            idx = idx[~np.isfinite(self.cave_seen[idx])]
            self.cave_seen[idx] = self.t
        seen = np.isfinite(self.cave_seen)
        glowing = seen & (self.t - self.cave_seen < 4 * REVEAL_GLOW_S)
        if not glowing.any() and not len(hits):
            return
        rgb = self.cave_mesh.point_data["rgb"]
        g = np.exp(-(self.t - self.cave_seen[glowing]) / REVEAL_GLOW_S)[:, None]
        base = self.cave_rgb[glowing]
        rgb[glowing] = (base + g * (np.array(REVEAL_RGB) - base)).astype(np.uint8)
        done = seen & ~glowing
        rgb[done] = self.cave_rgb[done]
        self.cave_mesh.GetPointData().GetArray("rgb").Modified()

    def _draw_brain(self):
        self.pl.subplot(*self.BRAIN)
        if self.skeletons is not None:
            self._draw_active_neurons()
        self._draw_synapses()
        n_active = f"{int((self.activity > 0.3).sum()):,}"
        self.n_active_text.SetInput(f"FlyWire brain  ·  {n_active} active neurons  ·  drag to rotate")

    def _draw_map(self):
        pl, w = self.pl, self.world
        pl.subplot(*self.MAP)
        pts = w.map_points
        if self.t - self.map_surface_t >= MAP_MESH_PERIOD and len(pts):
            self.map_surface_t = self.t
            surf = reconstruct_surface(self.pv, pts, w.ground, cut=self.map_cut, voxel=w.map_voxel)
            if surf is not None:
                self.map_surface.copy_from(surf)  # mismo actor: VTK solo sube la malla nueva
                self.map_surface_actor.visibility = True
        hits, _ = w.last_scan
        self.map_scan.update(hits[~np.isnan(hits[:, 0])])
        tr = np.array(w.trail)
        if len(tr) > 1:
            tr = np.column_stack([tr, w.ground(tr[:, 0], tr[:, 1]) + 0.05]).astype(np.float32)
            n = len(tr)
            self.map_path.update(np.column_stack([np.arange(n - 1), np.arange(1, n)]), points=tr)
        z = float(w.ground(*w.pose[:2]))
        self.map_robot.position = (w.pose[0], w.pose[1], z + 0.2)
        self.map_robot.orientation = (0, 0, math.degrees(w.pose[2]))
        if w.gas_found is not None and not self.map_gas.visibility:
            self.map_gas.position = (*w.gas_found, float(w.ground(*w.gas_found)) + 0.3)
            self.map_gas.visibility = True
        if self.readings["temp_c"] > TEMP_AVOID and not self.heat_seen:
            self.heat_seen = True
            self.map_heat.position = (*w.heat_xy, float(w.ground(*w.heat_xy)) + 0.2)
            self.map_heat.visibility = True
        self.map_text.SetInput(f"LiDAR map  ·  {len(pts):,} voxels  ·  {w.progress:.1f} m")
        robot = np.r_[w.pose[:2], z + 0.3]
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
        # Las más activas primero, hasta un presupuesto de segmentos: unas pocas neuronas de campo
        # amplio cubrirían el cerebro entero
        on = on[np.argsort(-self.activity[on])[:MAX_ACTIVE_DRAWN]]
        on = on[np.cumsum(sk["count"][on]) <= ACTIVE_SEGMENTS]
        if not len(on):
            self.active_lines.update([])
            return
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

    def _draw_charts(self):
        self._update_bars(self.in_series, self.in_keys, self.cmd.inputs)
        self._update_bars(self.out_series, self.out_keys, self.cmd.rates)
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
        for key in ("space", "t", "r", "1", "2", "3"):
            self.pl.add_key_event(key, lambda k=key: self.on_key(k))
        self.draw(self.tick(0.04))
        self.pl.add_timer_event(max_steps=10**9, duration=1, callback=self.update)  # sin tope de fps
        self.pl.show()

    def record(self, path, seconds=30.0, fps=24, speed=2.0, rough_at=0.35):
        """Misión grabada con dt fijo (vídeo .mp4 o GIF, sin ventana). speed > 1 acelera la
        simulación respecto al vídeo; a rough_at (fracción) se activa el terreno escarpado. Las
        pestañas se recorren: la mitad del vídeo la 1 y un cuarto cada una de las otras."""
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
            tab = 0 if i < n / 2 else 1 if i < 3 * n / 4 else 2
            if tab != self.tab:
                self.set_tab(tab)
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
            explorer.screenshot(path.with_name(f"{path.stem}_cueva_cerrada.png"), layer="altura", closed=True)
        return n


OPTIC_COLORS = ["#27d8e8", "#3ea8ff", "#62e0c8", "#d6e84a", "#f2c230", "#7f7bff", "#4fd1ff", "#2fe39a"]
CENTRAL_COLORS = ["#b45cff", "#8b5cff", "#5d6dff", "#e34fd6", "#ff5fa2", "#ff7a45", "#3d8bff", "#c77dff"]
OTHER_COLORS = ["#ff4f81", "#ffd23f", "#3df5a8", "#ff9f1c", "#59e0ff", "#f05bff"]


def neuron_colors(annotations):
    """Color RGB (0-1) por neurona: mismo color para el mismo tipo celular, con la paleta de su
    región (lóbulo óptico, cerebro central u otras), como las renderizaciones de FlyWire."""
    import zlib
    kind = annotations["cell_type"].fillna(annotations["cell_class"]).fillna(annotations["super_class"])
    kind = kind.fillna("?").astype(str).to_numpy()
    sc = annotations["super_class"].fillna("").to_numpy()
    out = np.zeros((len(kind), 3))
    palettes = {"optic": OPTIC_COLORS, "visual_projection": OPTIC_COLORS, "visual_centrifugal": OPTIC_COLORS,
                "central": CENTRAL_COLORS, "descending": CENTRAL_COLORS, "ascending": CENTRAL_COLORS,
                "endocrine": CENTRAL_COLORS, "motor": CENTRAL_COLORS}
    cache = {}
    for i, (k, g) in enumerate(zip(kind, sc)):
        key = (k, g)
        if key not in cache:
            pal = palettes.get(g, OTHER_COLORS)
            cache[key] = hex_rgb(pal[zlib.crc32(k.encode()) % len(pal)])
        out[i] = cache[key]
    return out


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
    ap.add_argument("--cueva", choices=["real", "generada"], default="real")
    args = ap.parse_args()
    viewer = BrainViewer(args.device, off_screen=bool(args.record), cave=args.cueva)
    if args.record:
        n = viewer.record(args.record, args.seconds, speed=args.speed)
        print(f"Video: {args.record} ({n} frames)")
    else:
        viewer.run()


if __name__ == "__main__":
    main()
