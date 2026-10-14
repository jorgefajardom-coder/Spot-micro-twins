"""Brazo con cabeza al estilo del Spot Arm de Boston Dynamics, a la escala del Spot Micro.

Proporciones del Spot Arm (mm): brazo 338,5, antebrazo 403,3, muñeca 117,45, pinza 124,82;
aquí escaladas ×0,28 (el Spot Micro mide ~0,28 veces el Spot). Cadena:

    base (guiñada) -> hombro (cabeceo) -> brazo -> codo (cabeceo) -> antebrazo
    -> muñeca (cabeceo) -> cabeza con pinza, cámara y el LiDAR encima

La cabeza mira alrededor (look): la base barre a un lado y a otro, y la muñeca la mantiene
nivelada con un ligero cabeceo. Las mallas se construyen con primitivas en el marco de cada
eslabón: x a lo largo del eslabón, y el eje de la articulación y z hacia arriba.
"""
import math

import numpy as np

SCALE = 0.28
UPPER = 0.3385 * SCALE                    # m, hombro -> codo
FORE = np.array([0.4033 * SCALE, 0.0, 0.028])  # m, codo -> muñeca (el tramo negro sube un poco)
WRIST = 0.11745 * SCALE                   # m, muñeca -> cabeza
GRIPPER = 0.12482 * SCALE                 # m, largo de la pinza
MOUNT = np.array([0.105, 0.0, 0.039])     # m, base sobre el lomo, hacia delante (como el Spot)
SHOULDER_UP = 0.042                       # m, eje del hombro sobre la base
POSE = {"yaw": 0.0, "shoulder": 50.0, "elbow": -64.0, "wrist": 14.0}  # grados
LOOK_YAW, LOOK_PERIOD = 65.0, 9.0         # barrido de la cabeza: ±grados y periodo (s)
LIDAR_ON_HEAD = np.array([0.010, 0.0, 0.030])  # m, centro del LiDAR respecto a la muñeca

YELLOW, BLACK, DARK, GREY = "#f4c542", "#1a1c20", "#2b2f36", "#5a606b"


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


def _tube(pv, p0, p1, r, res=40):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    return pv.Cylinder(center=(p0 + p1) / 2, direction=d, radius=r, height=float(np.linalg.norm(d)),
                       resolution=res)


def _joint(pv, center, r, width, axis=(0, 1, 0)):
    """Articulación: cilindro con los cantos redondeados (el cuerpo de un servo)."""
    c = np.asarray(center, float)
    a = np.asarray(axis, float)
    return [pv.Cylinder(center=c, direction=a, radius=r, height=width, resolution=48),
            pv.Cylinder(center=c, direction=a, radius=r * 0.72, height=width * 1.12, resolution=48)]


def _jaw(pv, side):
    """Mandíbula de la pinza: una cuña amarilla que se estrecha hacia la punta."""
    jaw = pv.Box(bounds=(0.0, GRIPPER, -0.006, 0.006, 0.0, 0.009)).triangulate().subdivide(2)
    p = jaw.points
    t = p[:, 0] / GRIPPER
    p[:, 2] *= 1.0 - 0.55 * t                  # más fina hacia la punta
    p[:, 1] *= 1.0 - 0.3 * t
    p[:, 2] = side * p[:, 2] + side * 0.002
    jaw.points = p + [WRIST, 0, 0]
    return jaw


class SpotArm:
    """Mallas del brazo en el marco de cada eslabón y sus transformaciones para una postura."""

    def __init__(self, pv, pose=None):
        self.pose = dict(POSE if pose is None else pose)
        self.base_pose = dict(self.pose)
        self.parts = {
            "base": [  # zócalo negro y torreta que gira
                (pv.Box(bounds=(-0.036, 0.030, -0.030, 0.030, 0.0, 0.012)), BLACK),
                (pv.Cylinder(center=(0, 0, 0.020), direction=(0, 0, 1), radius=0.026, height=0.018,
                             resolution=48), DARK),
            ],
            "shoulder": [(m, BLACK) for m in _joint(pv, (0, 0, 0), 0.026, 0.050)],
            "arm": [
                (_tube(pv, (0.018, 0, 0), (UPPER - 0.016, 0, 0), 0.0165), YELLOW),
                (_tube(pv, (0.018, 0, -0.004), (UPPER * 0.55, 0, -0.004), 0.0155), BLACK),
            ],
            "forearm": [
                *[(m, BLACK) for m in _joint(pv, (0, 0, 0), 0.021, 0.040)],
                (_tube(pv, (0.0, 0, 0.0), (0.035, 0, FORE[2]), 0.0145), BLACK),   # tramo negro doblado
                (_tube(pv, (0.035, 0, FORE[2]), FORE - [0.008, 0, 0], 0.0145), YELLOW),
                (_tube(pv, (0.060, 0, FORE[2]), (0.078, 0, FORE[2]), 0.0150), BLACK),  # banda
            ],
            "head": [
                (pv.Cylinder(center=(0.0, 0, 0), direction=(1, 0, 0), radius=0.0135, height=0.012,
                             resolution=48), GREY),                                 # anillo de muñeca
                (_tube(pv, (0.006, 0, 0), (WRIST, 0, 0), 0.0120), BLACK),
                (pv.Box(bounds=(WRIST - 0.004, WRIST + 0.012, -0.014, 0.014, -0.012, 0.012)), BLACK),
                (_jaw(pv, 1), YELLOW), (_jaw(pv, -1), YELLOW),
                (pv.Sphere(radius=0.0045, center=(WRIST + 0.012, 0.008, 0.004)), "#0b3d5c"),  # cámara
                (pv.Cylinder(center=LIDAR_ON_HEAD, direction=(0, 0, 1), radius=0.016, height=0.018,
                             resolution=48), BLACK),                                # LiDAR
                (pv.Cylinder(center=LIDAR_ON_HEAD + [0, 0, 0.003], direction=(0, 0, 1), radius=0.0163,
                             height=0.005, resolution=48), "#40c4ff"),
                (pv.Cylinder(center=(LIDAR_ON_HEAD[0], 0, 0.0115), direction=(0, 0, 1), radius=0.008,
                             height=0.012, resolution=24), DARK),                   # mástil
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
