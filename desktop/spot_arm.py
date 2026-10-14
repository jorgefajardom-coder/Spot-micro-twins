"""Brazo con cabeza al estilo del Spot Arm de Boston Dynamics, a la escala del Spot Micro.

Proporciones del Spot Arm (mm): brazo 338,5, antebrazo 403,3, muñeca 117,45, pinza 124,82;
aquí escaladas ×0,28 (el Spot Micro mide ~0,28 veces el Spot). Cadena:

    base (guiñada) -> hombro (cabeceo) -> brazo -> codo (cabeceo) -> antebrazo
    -> muñeca (cabeceo) -> cabeza con pinza, cámara y el LiDAR encima

Como en las fotos: zócalo negro acampanado, hombro de dos tambores negros con estrías, brazo
negro aplanado con una franja amarilla, codo negro doblado, antebrazo amarillo ancho con
bandas negras, ranuras y aleta, muñeca de anillos estriados y pinza negra con la mandíbula
fija abajo y la móvil arriba, amarillas y en ángulo.

La cabeza mira alrededor (look): la base barre a un lado y a otro, y la muñeca la mantiene
nivelada con un ligero cabeceo. Las mallas se construyen con primitivas en el marco de cada
eslabón: x a lo largo del eslabón, y el eje de la articulación y z hacia arriba.
"""
import math

import numpy as np

SCALE = 0.28
UPPER = 0.3385 * SCALE                    # m, hombro -> codo
FORE = np.array([0.4033 * SCALE, 0.0, 0.026])  # m, codo -> muñeca (el tramo negro sube un poco)
WRIST = 0.11745 * SCALE                   # m, muñeca -> cabeza
GRIPPER = 0.12482 * SCALE                 # m, largo de la pinza
MOUNT = np.array([0.060, 0.0, 0.025])     # m, sobre la parte plana del lomo, hacia delante
SHOULDER_UP = 0.048                       # m, eje del hombro sobre la base
# Postura de las fotos: brazo casi vertical, antebrazo hacia delante y abajo, cabeza nivelada
POSE = {"yaw": 0.0, "shoulder": 74.0, "elbow": -112.0, "wrist": 38.0}  # grados
LOOK_YAW, LOOK_PERIOD = 65.0, 9.0         # barrido de la cabeza: ±grados y periodo (s)
LIDAR_ON_HEAD = np.array([0.016, 0.0, 0.029])  # m, centro del LiDAR respecto a la muñeca

# Los mismos colores que el CAD del cuerpo (amarillo, negro y gris del plástico)
YELLOW, BLACK, DARK, GREY = "#e6ac20", "#181818", "#2a2a2a", "#737373"


def _rot_y(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _tf(r=np.eye(3), p=(0, 0, 0)):
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = p
    return m


def _tube(pv, p0, p1, ry, rz=None, res=48):
    """Tubo de p0 a p1 (en el plano x-z) con sección elíptica: ry de ancho, rz de alto."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    length = float(np.linalg.norm(d))
    t = pv.Cylinder(center=(length / 2, 0, 0), direction=(1, 0, 0), radius=1.0, height=length, resolution=res)
    t.points = t.points * [1, ry, rz or ry]
    ang = math.degrees(math.atan2(d[2], d[0]))
    t.points = t.points @ _rot_y(-ang).T + p0
    return t


def _drum(pv, center, r, width, ribs=4):
    """Tambor de motor (eje lateral y) con estrías: anillos alternos más finos."""
    c = np.asarray(center, float)
    out = [(pv.Cylinder(center=c, direction=(0, 1, 0), radius=r, height=width, resolution=56), BLACK)]
    for k in range(ribs):
        y = -width / 2 + (k + 0.5) * width / ribs
        out.append((pv.Cylinder(center=c + [0, y, 0], direction=(0, 1, 0), radius=r * 1.035, height=width / ribs * 0.45,
                                resolution=56), DARK))
    out.append((pv.Cylinder(center=c, direction=(0, 1, 0), radius=r * 0.55, height=width * 1.06, resolution=40), DARK))
    return out


def _prism(pv, profile_xz, y0, y1, taper=None):
    """Pieza plana: un perfil en x-z extruido de y0 a y1 (mandíbulas, aletas). taper=(x0, x1, f)
    estrecha el ancho de 1 en x0 a f en x1 (la punta de la pinza)."""
    p = np.asarray(profile_xz, float)
    n = len(p)
    poly = pv.PolyData(np.column_stack([p[:, 0], np.full(n, y0), p[:, 1]]), faces=np.r_[n, np.arange(n)])
    m = poly.extrude((0, y1 - y0, 0), capping=True).triangulate()
    if taper is not None:
        x0, x1, f = taper
        q = m.points
        k = 1 - (1 - f) * np.clip((q[:, 0] - x0) / (x1 - x0), 0, 1)
        q[:, 1] = (q[:, 1] - (y0 + y1) / 2) * k + (y0 + y1) / 2
        m.points = q
    return m.compute_normals(auto_orient_normals=True)


class SpotArm:
    """Mallas del brazo en el marco de cada eslabón y sus transformaciones para una postura."""

    def __init__(self, pv, pose=None):
        self.pose = dict(POSE if pose is None else pose)
        self.base_pose = dict(self.pose)
        fz = FORE[2]
        G = 1.6 * GRIPPER  # algo más larga que la escala exacta, como se ve en las fotos
        self.parts = {
            "base": [  # zócalo hundido en el lomo (pegado a la carcasa) y torreta
                (_prism(pv, [(-0.040, -0.008), (0.040, -0.008), (0.032, 0.014), (-0.032, 0.014)],
                        -0.034, 0.034), BLACK),
                (pv.Cylinder(center=(0, 0, 0.022), direction=(0, 0, 1), radius=0.028, height=0.018,
                             resolution=56), DARK),
                (pv.Cylinder(center=(0, 0, 0.0145), direction=(0, 0, 1), radius=0.031, height=0.003,
                             resolution=56), GREY),                                     # aro de giro
            ],
            "shoulder": [*_drum(pv, (0, 0.0175, 0), 0.026, 0.024), *_drum(pv, (0, -0.0175, 0), 0.026, 0.024)],
            "arm": [  # brazo negro aplanado con franja amarilla en el costado
                (_tube(pv, (0.012, 0, 0), (UPPER - 0.012, 0, 0), 0.013, 0.016), BLACK),
                (pv.Box(bounds=(0.030, UPPER - 0.030, 0.0118, 0.0142, -0.004, 0.009)), YELLOW),
                (pv.Box(bounds=(0.030, UPPER - 0.030, -0.0142, -0.0118, -0.004, 0.009)), YELLOW),
            ],
            "forearm": [
                (pv.Cylinder(center=(0, 0, 0), direction=(0, 1, 0), radius=0.019, height=0.034,
                             resolution=56), BLACK),                                        # codo
                (pv.Sphere(radius=0.0185, center=(0.004, 0, 0.004)), BLACK),
                (_tube(pv, (0.0, 0, 0.0), (0.030, 0, fz), 0.0135, 0.015), BLACK),            # tramo doblado
                (_tube(pv, (0.026, 0, fz), (0.040, 0, fz), 0.0172), BLACK),                 # banda
                (_tube(pv, (0.038, 0, fz), (FORE[0] - 0.006, 0, fz), 0.0168), YELLOW),      # antebrazo ancho
                (_tube(pv, (0.072, 0, fz), (0.0745, 0, fz), 0.0172), BLACK),                # junta
                (pv.Box(bounds=(0.050, 0.060, 0.0155, 0.0172, fz - 0.004, fz + 0.000)), BLACK),  # ranuras
                (pv.Box(bounds=(0.050, 0.060, 0.0155, 0.0172, fz + 0.004, fz + 0.008)), BLACK),
                (pv.Box(bounds=(0.050, 0.060, -0.0172, -0.0155, fz - 0.004, fz + 0.000)), BLACK),
                (pv.Box(bounds=(0.050, 0.060, -0.0172, -0.0155, fz + 0.004, fz + 0.008)), BLACK),
                (_prism(pv, [(0.040, fz + 0.015), (0.068, fz + 0.015), (0.062, fz + 0.021), (0.046, fz + 0.021)],
                        -0.003, 0.003), YELLOW),                                          # aleta
            ],
            "head": [
                (pv.Cylinder(center=(0, 0, 0), direction=(0, 1, 0), radius=0.0145, height=0.030,
                             resolution=48), BLACK),                                        # muñeca (cabeceo)
                *[(_tube(pv, (0.004 + 0.0045 * k, 0, 0), (0.0075 + 0.0045 * k, 0, 0), 0.0150 if k % 2 else 0.0142),
                   DARK if k % 2 else BLACK) for k in range(5)],                         # anillos estriados
                (_tube(pv, (0.026, 0, 0), (WRIST, 0, 0), 0.0140), YELLOW),                  # giro de muñeca
                # Pinza alargada que se estrecha hacia la punta (de lado y desde arriba), como un pico:
                # cuerpo negro, mandíbula superior amarilla que nace encima del cuerpo y la inferior
                # plana; todo en una pieza
                (_prism(pv, [(WRIST - 0.002, -0.013), (WRIST + 0.014, -0.013), (WRIST + 0.020, -0.008),
                             (WRIST + 0.020, 0.008), (WRIST + 0.012, 0.0135), (WRIST - 0.002, 0.0135)],
                        -0.0135, 0.0135), BLACK),
                (_prism(pv, [(WRIST + 0.006, -0.0140), (WRIST + G - 0.006, -0.0105), (WRIST + G + 0.001, -0.0075),
                             (WRIST + G - 0.002, -0.0050), (WRIST + 0.024, -0.0062), (WRIST + 0.006, -0.0070)],
                        -0.0125, 0.0125, taper=(WRIST + 0.012, WRIST + G, 0.55)), YELLOW),
                (_prism(pv, [(WRIST + 0.000, 0.0060), (WRIST + 0.002, 0.0150), (WRIST + 0.026, 0.0145),
                             (WRIST + G + 0.001, 0.0002), (WRIST + G - 0.004, -0.0028), (WRIST + 0.026, 0.0018),
                             (WRIST + 0.010, 0.0030)],
                        -0.0140, 0.0140, taper=(WRIST + 0.012, WRIST + G, 0.5)), YELLOW),
                (_prism(pv, [(WRIST + 0.016, -0.0068), (WRIST + G - 0.008, -0.0055), (WRIST + G - 0.008, -0.0012),
                             (WRIST + 0.026, 0.0020), (WRIST + 0.016, 0.0030)],
                        -0.0105, 0.0105, taper=(WRIST + 0.016, WRIST + G, 0.6)), BLACK),        # interior
                (pv.Cylinder(center=(WRIST + 0.012, 0, 0.0075), direction=(0, 1, 0), radius=0.0042,
                             height=0.0292, resolution=32), BLACK),                       # bisagra
                (pv.Cylinder(center=(WRIST + 0.012, 0, 0.0075), direction=(0, 1, 0), radius=0.0020,
                             height=0.0300, resolution=24), GREY),
                (pv.Sphere(radius=0.0030, center=(WRIST + 0.0195, 0.0090, -0.004)), "#0b3d5c"),  # cámara
                (pv.Cylinder(center=(LIDAR_ON_HEAD[0], 0, 0.016), direction=(0, 0, 1), radius=0.007,
                             height=0.010, resolution=24), DARK),                           # mástil
                (pv.Cylinder(center=LIDAR_ON_HEAD, direction=(0, 0, 1), radius=0.015, height=0.016,
                             resolution=48), BLACK),                                        # LiDAR
                (pv.Cylinder(center=LIDAR_ON_HEAD + [0, 0, 0.0025], direction=(0, 0, 1), radius=0.0153,
                             height=0.0045, resolution=48), "#40c4ff"),
            ],
        }

    def look(self, t):
        """La cabeza mira alrededor: barrido suave de la base y un leve cabeceo de la muñeca."""
        q, b = self.pose, self.base_pose
        q["yaw"] = b["yaw"] + LOOK_YAW * math.sin(2 * math.pi * t / LOOK_PERIOD)
        q["wrist"] = b["wrist"] + 8.0 * math.sin(2 * math.pi * t / (0.5 * LOOK_PERIOD) + 1.0)

    def transforms(self, body_tf):
        """Matriz 4x4 (mundo) de cada eslabón, dada la del cuerpo. Ángulos positivos suben."""
        q = self.pose
        base = body_tf @ _tf(_rot_z(q["yaw"]), MOUNT)
        shoulder = base @ _tf(p=(0, 0, SHOULDER_UP))
        arm = shoulder @ _tf(_rot_y(-q["shoulder"]))
        fore = arm @ _tf(_rot_y(-q["elbow"]), (UPPER, 0, 0))
        head = fore @ _tf(_rot_y(-q["wrist"]), FORE)  # wrist = -(hombro + codo): cabeza nivelada
        return {"base": base, "shoulder": shoulder, "arm": arm, "forearm": fore, "head": head}

    def lidar_origin(self, body_tf):
        """Posición del LiDAR (mundo)."""
        return (self.transforms(body_tf)["head"] @ np.r_[LIDAR_ON_HEAD, 1.0])[:3]


def available():
    return True
