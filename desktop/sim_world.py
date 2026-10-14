"""Mundo simulado para el Spot Micro: una cueva con una fuga de gas, una fuente de calor,
terreno escarpado opcional y un LiDAR 3D que va mapeando la cueva.

Sin gráficos ni hardware: spot_brain.py lo dibuja y tests/test_sim_world.py lo prueba.

La cueva es una galería que serpentea: una línea central, un radio que cambia y un techo
en arco (sección elíptica), con paredes de roca rugosas (ruido 3D que escala la sección).
Para que el LiDAR sea rápido se precalcula en una rejilla 2D la distancia de cada punto a
la línea central, el radio y la altura del techo; cada rayo avanza en pasos cortos (numba)
hasta salir del hueco.
"""
import math

import numpy as np
from numba import njit
from scipy.spatial import cKDTree

GRID_CELL = 0.04            # m, rejilla de la cueva para el LiDAR
LIDAR_RANGE = 6.0           # m
LIDAR_STEP = 0.02           # m por paso del trazado de rayos
FOOT_SOLE = 0.017           # m, del punto del pie (cinemática) a la suela del modelo
LIDAR_HEIGHT = 0.10         # m por encima del centro del cuerpo
LIDAR_AZIMUTHS = 96         # rayos por vuelta
LIDAR_ELEVATIONS = np.radians([-84, -72, -60, -48, -36, -26, -16, -8, 0, 8, 16, 26, 36, 48, 60, 72, 84, 90])  # barrido vertical completo: suelo, paredes y techo
MAP_VOXEL = 0.04            # m, resolución del mapa 3D acumulado
WALL_ROUGH = 0.16           # rugosidad de la roca (fracción del tamaño de la galería)

GAS_BACKGROUND = 5.0        # ppm
GAS_DETECT = 20.0           # ppm: a partir de aquí lo persigue
GAS_FOUND = 450.0           # ppm: fuga localizada, se para a distancia segura
TEMP_AMBIENT = 22.0         # °C
SENSOR_HEIGHT = 0.25        # m sobre el suelo: altura de los sensores de gas y temperatura
TEMP_AVOID = 28.0           # °C: empieza a rodear la fuente de calor
TEMP_STEER_FULL = 42.0      # °C: giro máximo para rodearla
TEMP_ESCAPE = 48.0          # °C: además, looming -> la Giant Fiber lo hace huir


@njit(cache=True)
def _terrain(x, y, rough, kx, ky, amp, phase):
    if not rough:
        return 0.0
    h = 0.0
    for k in range(kx.shape[0]):
        h += amp[k] * math.sin(kx[k] * x + ky[k] * y + phase[k])
    return h


@njit(cache=True)
def _wall_factor(x, y, z, wk, wphase, wamp, wall_rough):
    """Escala local de la sección (1 ± wall_rough): bultos y entrantes de la roca."""
    n = 0.0
    for k in range(wk.shape[0]):
        n += wamp[k] * math.sin(wk[k, 0] * x + wk[k, 1] * y + wk[k, 2] * z + wphase[k])
    return 1.0 + wall_rough * n


@njit(cache=True)
def _free(x, y, z, x0, y0, cell, dist, radius, ceil, rough, kx, ky, amp, phase, wk, wphase, wamp, wall_rough):
    """¿Está el punto dentro del hueco de la cueva (ni pared, ni techo, ni suelo)?

    Sección elíptica de semiejes radio x techo, escalada por la rugosidad de la roca.
    """
    i = int((x - x0) / cell)
    j = int((y - y0) / cell)
    if i < 0 or j < 0 or i >= dist.shape[0] or j >= dist.shape[1]:
        return False
    floor = _terrain(x, y, rough, kx, ky, amp, phase)
    if z <= floor:
        return False
    f = _wall_factor(x, y, z, wk, wphase, wamp, wall_rough)
    q = dist[i, j] / (radius[i, j] * f)
    zt = z / (ceil[i, j] * f)
    return q * q + zt * zt < 1.0


@njit(cache=True)
def _raycast(origin, dirs, max_range, step, x0, y0, cell, dist, radius, ceil, rough, kx, ky, amp, phase,
             wk, wphase, wamp, wall_rough):
    n = dirs.shape[0]
    hits = np.full((n, 3), np.nan)
    ranges = np.full(n, np.nan)
    for r in range(n):
        t = step
        while t < max_range:
            x = origin[0] + dirs[r, 0] * t
            y = origin[1] + dirs[r, 1] * t
            z = origin[2] + dirs[r, 2] * t
            if not _free(x, y, z, x0, y0, cell, dist, radius, ceil, rough, kx, ky, amp, phase,
                         wk, wphase, wamp, wall_rough):
                hits[r, 0], hits[r, 1], hits[r, 2] = x, y, z
                ranges[r] = t
                break
            t += step
    return hits, ranges


class Cave:
    """Galería serpenteante de ~28 m con radio y techo variables (determinista por semilla)."""

    def __init__(self, seed=3, length=16.0, ds=0.1):
        rng = np.random.default_rng(seed)
        s = np.arange(0, length, ds)
        a = rng.uniform(0, 2 * np.pi, 3)
        heading = 0.55 * np.sin(s / 3.8 + a[0]) + 0.35 * np.sin(s / 2.2 + a[1]) + 0.2 * np.sin(s / 1.3 + a[2])
        heading -= heading[0]
        self.s = s
        self.center = np.column_stack([np.cumsum(np.cos(heading)) * ds, np.cumsum(np.sin(heading)) * ds])
        self.center -= self.center[0]
        self.tangent = np.column_stack([np.cos(heading), np.sin(heading)])
        self.radius = 0.72 + 0.16 * np.sin(s / 1.7 + a[1]) + 0.08 * np.sin(s / 0.9 + a[2])
        self.ceiling = 0.78 + 0.12 * np.sin(s / 2.1 + a[0])
        # Ensanche (sala) a media galería, donde está la fuente de calor
        hall = np.exp(-((s - 0.45 * length) / 1.4) ** 2)
        self.radius += 0.45 * hall
        self.ceiling += 0.2 * hall
        self._tree = cKDTree(self.center)
        # Rugosidad de la roca: ondas 3D de 0,15 a 0,8 m en direcciones al azar
        n_waves = 14
        wavelength = rng.uniform(0.15, 0.8, n_waves)
        direction = rng.normal(size=(n_waves, 3))
        direction /= np.linalg.norm(direction, axis=1, keepdims=True)
        self.wall_k = 2 * np.pi * direction / wavelength[:, None]
        self.wall_phase = rng.uniform(0, 2 * np.pi, n_waves)
        amp = wavelength ** 0.8  # las ondas largas, más marcadas (como la roca real)
        self.wall_amp = amp / (np.sqrt(0.5 * np.sum(amp ** 2)) * 2.2)  # ruido ~[-1, 1]

        # Rejillas para el LiDAR: distancia a la línea central, radio y techo del punto más cercano
        lo, hi = self.center.min(0) - 1.6, self.center.max(0) + 1.6
        self.x0, self.y0 = lo
        nx, ny = np.ceil((hi - lo) / GRID_CELL).astype(int)
        gx = self.x0 + (np.arange(nx) + 0.5) * GRID_CELL
        gy = self.y0 + (np.arange(ny) + 0.5) * GRID_CELL
        X, Y = np.meshgrid(gx, gy, indexing="ij")
        d, idx = self._tree.query(np.column_stack([X.ravel(), Y.ravel()]))
        self.grid_dist = d.reshape(nx, ny)
        self.grid_idx = idx.reshape(nx, ny)
        self.grid_radius = self.radius[self.grid_idx]
        self.grid_ceil = self.ceiling[self.grid_idx]
        self.grid_x, self.grid_y = gx, gy

    def wall_factor(self, xyz):
        """Versión vectorizada de _wall_factor (para deformar la malla igual que el LiDAR)."""
        xyz = np.atleast_2d(xyz)
        return 1.0 + WALL_ROUGH * np.sum(self.wall_amp * np.sin(xyz @ self.wall_k.T + self.wall_phase), axis=1)


    def floor(self, x, y):
        return np.zeros(np.broadcast(np.asarray(x), np.asarray(y)).shape)

    def wall_limit(self, i):
        return self.radius[i] * (1 - WALL_ROUGH) - 0.2  # contando con los salientes de la roca

    def raycast(self, origin, dirs, terrain, max_range, step):
        t = terrain
        return _raycast(origin, dirs, max_range, step, self.x0, self.y0, GRID_CELL, self.grid_dist,
                        self.grid_radius, self.grid_ceil, t.rough, t.kx, t.ky, t.amp, t.phase,
                        self.wall_k, self.wall_phase, self.wall_amp, WALL_ROUGH)

    def nearest(self, xy):
        """Índice del punto de la línea central más cercano y distancia a él."""
        d, i = self._tree.query(xy)
        return int(i), float(d)

    def point_at(self, s):
        i = int(np.clip(round(s / (self.s[1] - self.s[0])), 0, len(self.s) - 1))
        return self.center[i], self.tangent[i], self.radius[i], self.ceiling[i]


@njit(cache=True)
def _raycast_occ(origin, dirs, max_range, step, lo, cell, occ, occ_cell, floor, rough, kx, ky, amp, phase):
    """LiDAR contra una cueva real: ocupación 3D de la roca escaneada (rejilla occ_cell) + suelo
    (rejilla cell, con el terreno escarpado opcional sumado encima)."""
    n = dirs.shape[0]
    hits = np.full((n, 3), np.nan)
    ranges = np.full(n, np.nan)
    nx, ny, nz = occ.shape
    for r in range(n):
        t = step
        while t < max_range:
            x = origin[0] + dirs[r, 0] * t
            y = origin[1] + dirs[r, 1] * t
            z = origin[2] + dirs[r, 2] * t
            i = int((x - lo[0]) / occ_cell)
            j = int((y - lo[1]) / occ_cell)
            k = int((z - lo[2]) / occ_cell)
            if i < 0 or j < 0 or k < 0 or i >= nx or j >= ny or k >= nz:
                break  # fuera del escaneo: el rayo no vuelve
            fi = min(int((x - lo[0]) / cell), floor.shape[0] - 1)
            fj = min(int((y - lo[1]) / cell), floor.shape[1] - 1)
            if occ[i, j, k] or z <= floor[fi, fj] + _terrain(x, y, rough, kx, ky, amp, phase):
                hits[r, 0], hits[r, 1], hits[r, 2] = x, y, z
                ranges[r] = t
                break
            t += step
    return hits, ranges


class RealCave:
    """Cueva real escaneada con LiDAR (caves/real_cave.py). A escala 2:1 es una galería de mina
    por la que pasan personas (~2,7 m de alto y 1,2-1,8 m de ancho; a 1:1 apenas 1,3 m).

    A partir de los puntos: ocupación 3D (el LiDAR choca con la roca real), mapa del suelo
    (los pies pisan el suelo real), zona transitable (techo > CLEARANCE y pendiente suave) y
    una ruta por el centro de la galería (el camino más ancho de un extremo al otro), con la
    misma interfaz que Cave: s, center, tangent, radius, ceiling, nearest, point_at.
    """

    CELL = 0.05        # m, vóxel de la ocupación y del mapa del suelo (a escala 1:2)
    CLEARANCE = 0.42   # m de techo mínimo para que el robot pase (cuerpo + LiDAR)
    MAX_SLOPE = 0.6    # pendiente máxima transitable (tan ~31°)

    def __init__(self, name="GrotteDeLaSourde/EntranceGallery", scale=2.0, ds=0.1):
        import sys
        from pathlib import Path
        from scipy import ndimage
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import dijkstra
        sys.path.insert(0, str(Path(__file__).resolve().parent / "caves"))
        from real_cave import ATTRIBUTION, load_points

        self.kind, self.name, self.attribution, self.scale = "real", name, ATTRIBUTION, scale
        pts, rgb, normals = load_points(name)
        pts = (pts - pts.min(0)) * scale
        self.points, self.colors = pts, rgb
        # La rejilla escala con la cueva: la densidad del escáner es la misma a cualquier escala
        self.CELL = c = self.CELL * scale / 0.5
        # El LiDAR choca con una ocupación más fina: con la rejilla grande, el suelo dilatado
        # taparía el sensor (rejilla de la ruta y del suelo: CELL; de la roca: occ_cell)
        self.occ_cell = oc = 0.05 * max(1.0, scale)
        lo_occ = np.array([-2 * c, -2 * c, -2 * c])
        dims_o = np.ceil((pts.max(0) - lo_occ) / oc).astype(int) + 3
        fine = np.zeros(dims_o, bool)
        io = np.floor((pts - lo_occ) / oc).astype(int)
        fine[io[:, 0], io[:, 1], io[:, 2]] = True
        self.lo = np.array([-2 * c, -2 * c, -2 * c])
        dims = np.ceil((pts.max(0) - self.lo) / c).astype(int) + 3
        occ = np.zeros(dims, bool)
        idx = np.floor((pts - self.lo) / c).astype(int)
        occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True
        self.occ = ndimage.binary_dilation(fine, iterations=1)  # sin huecos entre muestras

        # Suelo y techo según la normal (las paredes no cuentan): así un hueco del muestreo en el
        # suelo no hace que el techo pase por suelo. En este escaneo las normales apuntan hacia
        # la roca: el suelo tiene la normal hacia abajo y el techo hacia arriba.
        ij = np.floor((pts[:, :2] - self.lo[:2]) / c).astype(int)
        shape = dims[:2]
        nz = normals[:, 2] if normals is not None else np.zeros(len(pts))
        up, down = nz < -0.5, nz > 0.3  # up: suelo, down: techo
        floor = np.full(shape, np.inf)
        np.minimum.at(floor, (ij[up, 0], ij[up, 1]), pts[up, 2])
        ceil = np.full(shape, np.inf)
        np.minimum.at(ceil, (ij[down, 0], ij[down, 1]), pts[down, 2])
        inside = ndimage.binary_closing(occ.any(axis=2), iterations=3)
        has_floor = np.isfinite(floor)
        _, (ii, jj) = ndimage.distance_transform_edt(~has_floor, return_indices=True)
        self.floor_grid = np.where(has_floor, floor, floor[ii, jj])  # huecos: la celda con suelo más cercana
        self.floor_grid = ndimage.median_filter(self.floor_grid, 3)
        self.floor_grid = ndimage.gaussian_filter(self.floor_grid, 2.0)  # sin escalones para los pies
        ceil = np.where(np.isfinite(ceil) & (ceil > self.floor_grid + 0.05), ceil, self.floor_grid + 3.0)
        clearance = ceil - self.floor_grid
        gy, gx = np.gradient(ndimage.gaussian_filter(self.floor_grid, 2), c)
        walk = inside & (clearance > self.CLEARANCE) & (np.hypot(gx, gy) < self.MAX_SLOPE)
        walk = ndimage.binary_opening(walk, iterations=1)
        lab, n = ndimage.label(walk)
        walk = lab == (np.argmax(np.bincount(lab.ravel())[1:]) + 1)  # la zona transitable mayor
        width = ndimage.distance_transform_edt(walk) * c

        # Ruta: de un extremo de la galería al otro por donde es más ancha
        cells = np.argwhere(walk)
        xy = cells * c + self.lo[:2]
        axis = np.linalg.svd(xy - xy.mean(0), full_matrices=False)[2][0]
        along = xy @ axis
        a, b = cells[np.argmin(along)], cells[np.argmax(along)]
        index = -np.ones(walk.shape, int)
        index[walk] = np.arange(len(cells))
        rows, cols, cost = [], [], []
        for di, dj in ((1, 0), (0, 1), (1, 1), (1, -1)):
            src = cells
            dst = cells + (di, dj)
            ok = (dst[:, 0] < walk.shape[0]) & (dst[:, 1] >= 0) & (dst[:, 1] < walk.shape[1])
            src, dst = src[ok], dst[ok]
            ok = walk[dst[:, 0], dst[:, 1]]
            src, dst = src[ok], dst[ok]
            w = np.hypot(di, dj) / (0.05 + np.minimum(width[src[:, 0], src[:, 1]], width[dst[:, 0], dst[:, 1]])) ** 2
            rows.append(index[src[:, 0], src[:, 1]])
            cols.append(index[dst[:, 0], dst[:, 1]])
            cost.append(w)
        graph = coo_matrix((np.concatenate(cost), (np.concatenate(rows), np.concatenate(cols))),
                           shape=(len(cells), len(cells))).tocsr()
        _, pred = dijkstra(graph, directed=False, indices=index[tuple(a)], return_predecessors=True)
        path, node = [], index[tuple(b)]
        while node >= 0:
            path.append(node)
            node = pred[node]
        route = cells[path[::-1]] * c + self.lo[:2] + c / 2
        route = ndimage.uniform_filter1d(route, 15, axis=0, mode="nearest")  # suavizar
        seg = np.r_[0, np.cumsum(np.hypot(*np.diff(route, axis=0).T))]
        self.s = np.arange(0, seg[-1], ds)
        self.center = np.column_stack([np.interp(self.s, seg, route[:, k]) for k in (0, 1)])
        tangent = np.gradient(self.center, axis=0)
        self.tangent = tangent / np.linalg.norm(tangent, axis=1, keepdims=True)
        ij = np.floor((self.center - self.lo[:2]) / c).astype(int)
        self.radius = np.maximum(width[ij[:, 0], ij[:, 1]], 0.3)
        self.ceiling = clearance[ij[:, 0], ij[:, 1]]
        self.walkable = walk
        self._tree = cKDTree(self.center)

    def nearest(self, xy):
        d, i = self._tree.query(xy)
        return int(i), float(d)

    def point_at(self, s):
        i = int(np.clip(round(s / (self.s[1] - self.s[0])), 0, len(self.s) - 1))
        return self.center[i], self.tangent[i], self.radius[i], self.ceiling[i]

    def floor(self, x, y):
        """Altura del suelo, interpolada entre celdas (continua: los pies y el suelo dibujado
        coinciden; con la celda más cercana había escalones de hasta varios cm)."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        g = self.floor_grid
        u = np.clip((x - self.lo[0]) / self.CELL - 0.5, 0, g.shape[0] - 1.001)
        v = np.clip((y - self.lo[1]) / self.CELL - 0.5, 0, g.shape[1] - 1.001)
        i, j = u.astype(int), v.astype(int)
        fu, fv = u - i, v - j
        return ((g[i, j] * (1 - fu) + g[i + 1, j] * fu) * (1 - fv)
                + (g[i, j + 1] * (1 - fu) + g[i + 1, j + 1] * fu) * fv)

    def wall_limit(self, i):
        """Distancia máxima del cuerpo a la ruta (la zona transitable menos el cuerpo)."""
        return max(0.05, self.radius[i] - 0.15)

    def raycast(self, origin, dirs, terrain, max_range, step):
        t = terrain
        return _raycast_occ(origin, dirs, max_range, step, self.lo, self.CELL, self.occ, self.occ_cell,
                            self.floor_grid, t.rough, t.kx, t.ky, t.amp, t.phase)


class Terrain:
    """Suelo: plano o escarpado (suma de ondas en direcciones al azar, ~±4 cm)."""

    def __init__(self, seed=7, n_waves=9):
        rng = np.random.default_rng(seed)
        wavelength = rng.uniform(0.25, 1.1, n_waves)
        ang = rng.uniform(0, 2 * np.pi, n_waves)
        self.kx = 2 * np.pi / wavelength * np.cos(ang)
        self.ky = 2 * np.pi / wavelength * np.sin(ang)
        self.amp = 0.045 * wavelength / wavelength.sum() * 2.2  # las ondas largas pesan más
        self.phase = rng.uniform(0, 2 * np.pi, n_waves)
        self.rough = False

    def height(self, x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        if not self.rough:
            return np.zeros(np.broadcast(x, y).shape)
        return np.sum(self.amp * np.sin(np.multiply.outer(x, self.kx) + np.multiply.outer(y, self.ky)
                                        + self.phase), axis=-1)


class CaveWorld:
    """Estado del mundo: robot, cueva, gas, calor, LiDAR y mapa acumulado."""

    def __init__(self, seed=3, cave=None):
        self.cave = cave if cave is not None else Cave(seed)
        self.terrain = Terrain(seed + 4)
        n = len(self.cave.s)
        real = getattr(self.cave, "kind", "") == "real"
        # Fuga al fondo de la galería (en la mina, a 14 m como mucho: la misión dura ~1 min)
        self.gas_s = min(self.cave.s[int(0.85 * n)], 14.0) if real else self.cave.s[int(0.85 * n)]
        self.map_voxel = MAP_VOXEL * max(1.0, getattr(self.cave, "scale", 1.0))  # galería grande, mapa más grueso
        self.gas_xy = self.cave.point_at(self.gas_s)[0]
        # Fuente de calor: una grieta caliente en la pared a mitad de recorrido. En la galería
        # generada está dentro de la sala; en la real, en el borde de la roca, para que se pueda
        # pasar rodeándola sin quedar bloqueado.
        c, t, r, _ = self.cave.point_at(0.5 * self.gas_s if real else self.cave.s[int(0.45 * n)])
        normal = np.array([-t[1], t[0]])
        self.heat_xy = c + normal * (r + 0.3 if real else 0.3)
        self.obstacle = None
        self.lidar_mount = (0.0, 0.0, LIDAR_HEIGHT)  # (adelante, lado, arriba) respecto al cuerpo
        self.reset_robot()

    def reset_robot(self):
        c, t, _, _ = self.cave.point_at(0.6)
        self.pose = np.array([c[0], c[1], math.atan2(t[1], t[0])])
        self.trail = [self.pose[:2].copy()]
        self.progress = 0.0
        self.gas_found = None
        self.map_keys = np.zeros(0, np.int64)
        self.map_points = np.zeros((0, 3), np.float32)
        self.map_gas = np.zeros(0, np.float32)
        self.last_scan = (np.zeros((0, 3)), np.zeros(0))
        self.samples = []  # registro de sensores: (x, y, z, ppm, °C) en cada paso

    # ----- campos físicos -----
    def gas_ppm(self, xy):
        """El gas se difunde a lo largo de la galería desde la fuga (y un poco de turbulencia)."""
        xy = np.atleast_2d(xy)
        _, idx = self.cave._tree.query(xy)
        ds = self.cave.s[idx] - self.gas_s
        sigma = np.where(ds > 0, 1.6, 3.2)  # la corriente lo arrastra hacia la entrada
        return GAS_BACKGROUND + 900.0 * np.exp(-(ds / sigma) ** 2)

    def temperature(self, xy):
        xy = np.atleast_2d(xy)
        d2 = np.sum((xy - self.heat_xy) ** 2, axis=1)
        return TEMP_AMBIENT + 75.0 * np.exp(-d2 / (2 * 0.32 ** 2))

    def ground(self, x, y):
        return self.cave.floor(x, y) + self.terrain.height(x, y)

    # ----- LiDAR -----
    def lidar_scan(self, body_z):
        """Una vuelta del LiDAR: puntos de impacto (mundo) y distancias por rayo."""
        az = np.linspace(-np.pi, np.pi, LIDAR_AZIMUTHS, endpoint=False) + self.pose[2]
        A, E = np.meshgrid(az, LIDAR_ELEVATIONS, indexing="ij")
        dirs = np.column_stack([(np.cos(E) * np.cos(A)).ravel(), (np.cos(E) * np.sin(A)).ravel(),
                                np.sin(E).ravel()])
        fwd, side, up = self.lidar_mount
        c, s_ = np.cos(self.pose[2]), np.sin(self.pose[2])
        origin = np.array([self.pose[0] + fwd * c - side * s_, self.pose[1] + fwd * s_ + side * c, body_z + up])
        hits, ranges = self.cave.raycast(origin, dirs, self.terrain, LIDAR_RANGE, LIDAR_STEP)
        self.last_scan = (hits, ranges.reshape(LIDAR_AZIMUTHS, len(LIDAR_ELEVATIONS)))
        self._add_to_map(hits[~np.isnan(ranges)])
        return self.last_scan

    def _add_to_map(self, pts):
        if not len(pts):
            return
        v = np.floor(pts / self.map_voxel).astype(np.int64) + 50_000
        keys = (v[:, 0] * 100_000 + v[:, 1]) * 100_000 + v[:, 2]
        keys, first = np.unique(keys, return_index=True)
        new = ~np.isin(keys, self.map_keys, assume_unique=True)
        if new.any():
            p = pts[first[new]]
            self.map_keys = np.concatenate([self.map_keys, keys[new]])
            self.map_points = np.concatenate([self.map_points, p.astype(np.float32)])
            self.map_gas = np.concatenate([self.map_gas, self.gas_ppm(p[:, :2]).astype(np.float32)])

    def clearance(self):
        """Distancia libre (m) a la altura del cuerpo: delante, a la izquierda y a la derecha."""
        _, ranges = self.last_scan
        if ranges.size == 0:
            return 9.0, 9.0, 9.0
        flat = np.nan_to_num(ranges[:, int(np.argmin(np.abs(LIDAR_ELEVATIONS)))], nan=LIDAR_RANGE)
        ang = (np.linspace(-np.pi, np.pi, LIDAR_AZIMUTHS, endpoint=False) + np.pi) % (2 * np.pi) - np.pi
        sector = lambda lo, hi: float(flat[(ang >= np.radians(lo)) & (ang <= np.radians(hi))].min())  # noqa: E731
        return sector(-20, 20), sector(20, 80), sector(-80, -20)

    # ----- sentidos del robot -----
    def senses(self):
        """Qué percibe el robot y qué quiere hacer (sustitución sensorial hacia la mosca).

        Devuelve (rumbo del objetivo en rad o None, distancia equivalente, looming 0..1,
        rumbo del looming, lecturas de los sensores).
        """
        x, y, yaw = self.pose
        fwd = np.array([math.cos(yaw), math.sin(yaw)])
        left = np.array([-fwd[1], fwd[0]])
        head = self.pose[:2] + 0.15 * fwd
        ppm = float(self.gas_ppm(head)[0])
        temp = float(self.temperature(head)[0])
        readings = {"gas_ppm": ppm, "temp_c": temp}
        z = float(self.ground(*head)) + SENSOR_HEIGHT
        if not self.samples or np.hypot(*(head - self.samples[-1][:2])) > 0.03:
            self.samples.append((head[0], head[1], z, ppm, temp))

        # Objetivo: el gradiente de gas si lo detecta (dos "antenas"), si no, seguir la galería
        if self.gas_found is None and ppm >= GAS_FOUND:
            self.gas_found = head.copy()
        if self.gas_found is not None:
            bearing, dist = 0.0, 0.15                       # localizada: se queda quieto
            readings["modo"] = "fuga localizada"
        elif ppm >= GAS_DETECT:
            e = 0.1
            g_f = self.gas_ppm(head + e * fwd)[0] - self.gas_ppm(head - e * fwd)[0]
            g_l = self.gas_ppm(head + e * left)[0] - self.gas_ppm(head - e * left)[0]
            bearing = math.atan2(g_l, g_f) if abs(g_f) + abs(g_l) > 1e-6 else 0.0
            dist = 0.6
            readings["modo"] = "siguiendo gas"
        else:
            i, _ = self.cave.nearest(self.pose[:2])
            self.progress = max(self.progress, self.cave.s[i])
            ahead = self.cave.point_at(self.cave.s[i] + 1.0)[0] - self.pose[:2]
            bearing = math.atan2(ahead @ left, ahead @ fwd)
            dist = 0.8
            readings["modo"] = "explorando"
        # Paredes cerca: girar hacia el lado más despejado
        front, cl, cr = self.clearance()
        push = max(0.0, 0.55 - cl) - max(0.0, 0.55 - cr)
        # Calor moderado: rodearlo por el lado contrario (o por el más despejado si está delante).
        # Solo el escape de la mosca lo haría retroceder y volver a acercarse una y otra vez.
        d = self.heat_xy - self.pose[:2]
        heat_bearing = math.atan2(d @ left, d @ fwd)
        heat_push = float(np.clip((temp - TEMP_AVOID) / (TEMP_STEER_FULL - TEMP_AVOID), 0, 1))
        heat_side = (np.sign(heat_bearing) if abs(heat_bearing) > 0.2 else (-1.0 if cl > cr else 1.0))
        bearing = float(np.clip(bearing - 1.6 * push - 1.5 * heat_push * heat_side, -1.6, 1.6))

        # Looming: calor fuerte (amenaza: escape) u obstáculo lanzado
        looming, loom_bearing = 0.0, 0.0
        if temp > TEMP_ESCAPE:
            looming = float(np.clip((temp - TEMP_ESCAPE) / 20.0, 0, 1))
            loom_bearing = heat_bearing
        if self.obstacle is not None:
            d = self.obstacle - self.pose[:2]
            lo = float(np.clip((0.75 - np.hypot(*d)) / 0.45, 0, 1))
            if lo > looming:
                looming, loom_bearing = lo, math.atan2(d @ left, d @ fwd)
        return bearing, dist, looming, loom_bearing, readings

    # ----- movimiento -----
    def launch_obstacle(self):
        ang = self.pose[2] + np.random.uniform(-0.4, 0.4)
        self.obstacle = self.pose[:2] + 1.0 * np.array([math.cos(ang), math.sin(ang)])

    def update(self, dt, vel):
        vx, vy, wz = vel
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        self.pose += dt * np.array([vx * c - vy * s, vx * s + vy * c, wz])
        # La pared es sólida: si el cuerpo se mete, vuelve hacia el centro de la galería
        i, d = self.cave.nearest(self.pose[:2])
        limit = self.cave.wall_limit(i)
        if d > limit:
            centre = self.cave.center[i]
            self.pose[:2] = centre + (self.pose[:2] - centre) * (limit / d)
        if np.linalg.norm(self.pose[:2] - self.trail[-1]) > 0.02:
            self.trail = (self.trail + [self.pose[:2].copy()])[-2000:]
        if self.obstacle is not None:
            step = self.pose[:2] - self.obstacle
            n = np.linalg.norm(step)
            self.obstacle = self.obstacle + dt * 0.9 * step / max(n, 1e-6)
            if n < 0.12:
                self.obstacle = None

    def body_on_terrain(self, feet_world, body_height):
        """Pies sobre el suelo y cuerpo inclinado según el terreno.

        feet_world: 4x3 con z relativa al suelo (0 = apoyado). Devuelve (pies, rpy en grados, cuerpo)."""
        feet = np.array(feet_world, float)
        ground = self.ground(feet[:, 0], feet[:, 1])
        # El punto del pie es el centro de la pieza redondeada: la suela queda FOOT_SOLE más abajo
        feet[:, 2] += ground + FOOT_SOLE
        # Orden de patas: LF, LB, RB, RF
        length = np.linalg.norm(feet[0, :2] - feet[1, :2]) or 0.2
        width = np.linalg.norm(feet[0, :2] - feet[3, :2]) or 0.16
        front, back = (ground[0] + ground[3]) / 2, (ground[1] + ground[2]) / 2
        left, right = (ground[0] + ground[1]) / 2, (ground[2] + ground[3]) / 2
        pitch = -math.degrees(math.atan2(front - back, length))
        roll = math.degrees(math.atan2(left - right, width))
        body = np.array([self.pose[0], self.pose[1], body_height + ground.mean() + FOOT_SOLE])
        return feet, [float(np.clip(roll, -15, 15)), float(np.clip(pitch, -15, 15)),
                      math.degrees(self.pose[2])], body
