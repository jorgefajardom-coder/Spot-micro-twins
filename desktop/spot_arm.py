"""Brazo con cabeza para el Spot Micro (piezas de github.com/Meddodan/ARM_MOD, sin licencia
publicada: se descargan aparte a models/arm_mod/ y no se suben al repositorio).

Cadena: base giratoria (guiñada) sobre el lomo -> hombro -> brazo -> codo -> antebrazo ->
muñeca -> cabeza, con el LiDAR montado encima de la cabeza.

Las piezas vienen tumbadas como se imprimen (mm, centradas): el eje largo del brazo es y, los
ejes de las articulaciones son x y el perfil está en el plano y-z. Los centros de los agujeros
de cada articulación se midieron cortando las piezas (ver JOINTS).
"""
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ARM_DIR = HERE / "models" / "arm_mod"
MM = 0.001

# Centros de los ejes en coordenadas de cada pieza (mm, plano y-z)
JOINTS = {
    "shoulder_on_base": (-1.1, 30.5),    # eje del hombro en la base giratoria
    "arm_shoulder": (-75.7, 13.8),       # brazo: agujero del hombro
    "arm_elbow": (67.4, 13.1),           # brazo: agujero del codo
    "forearm_elbow": (-74.4, 47.2),      # antebrazo: cubo del servo del codo
    "forearm_wrist": (58.6, 13.9),       # antebrazo: eje de la muñeca
}
MOUNT = np.array([0.055, 0.0, 0.039])    # m, base del brazo sobre el lomo (cuerpo: x adelante)
SIDE_GAP = 0.021                         # m, separación lateral de las dos mitades de cada eslabón
# Postura: guiñada de la base, hombro (sube), codo (baja) y muñeca (cabeza nivelada), en grados
POSE = {"yaw": 0.0, "shoulder": 52.0, "elbow": -62.0, "wrist": 10.0}
LIDAR_ON_HEAD = np.array([0.012, 0.0, 0.062])  # m, centro del LiDAR respecto a la muñeca

YELLOW, DARK, BLACK = "#f4c542", "#2b2f36", "#15171b"


def available():
    return all((ARM_DIR / f"{n}.stl").exists() for n in
               ("Rotating Base", "Arm 1", "Arm 1B", "Forward Arm A", "Forward Arm B", "Head A", "Head B"))


def _rot_y(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _tf(r=np.eye(3), p=(0, 0, 0)):
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = p
    return m


def _to_link(points, origin_yz, side=0.0, mirror=False):
    """Pieza (mm) -> eslabón (m): el eje largo (y) hacia delante (x), el eje de la articulación
    (x) hacia el lado (y), con la articulación en el origen."""
    p = np.asarray(points, float)
    x = -p[:, 0] if mirror else p[:, 0]
    return np.column_stack([p[:, 1] - origin_yz[0], -x, p[:, 2] - origin_yz[1]]) * MM + [0, side, 0]


class SpotArm:
    """Mallas del brazo en el marco de cada eslabón y sus transformaciones para una postura."""

    def __init__(self, pv, pose=None):
        self.pose = dict(POSE if pose is None else pose)
        read = lambda n: pv.read(str(ARM_DIR / f"{n}.stl"))  # noqa: E731
        j = JOINTS

        def link(name, origin, side=0.0, mirror=False):
            m = read(name)
            m.points = _to_link(m.points, origin, side, mirror)
            if mirror:
                m = m.flip_faces()  # el espejo invierte el sentido de las caras
            return m

        # La base gira sobre su eje vertical: el origen es el centro de su fondo
        base = read("Rotating Base")
        base.points = _to_link(base.points, (0.0, 0.0))
        self.parts = {
            "base": [(base, YELLOW)],
            "arm": [(link("Arm 1", j["arm_shoulder"], SIDE_GAP), YELLOW),
                    (link("Arm 1B", j["arm_shoulder"], -SIDE_GAP, mirror=True), YELLOW)],
            "forearm": [(link("Forward Arm A", j["forearm_elbow"], SIDE_GAP), YELLOW),
                        (link("Forward Arm B", j["forearm_elbow"], -SIDE_GAP, mirror=True), YELLOW)],
            "head": self._head(pv, read),
        }
        # Distancias entre ejes de cada eslabón (m, en su marco)
        self.shoulder = np.array([j["shoulder_on_base"][0], 0, j["shoulder_on_base"][1]]) * MM
        self.upper = np.array([j["arm_elbow"][0] - j["arm_shoulder"][0], 0,
                               j["arm_elbow"][1] - j["arm_shoulder"][1]]) * MM
        self.fore = np.array([j["forearm_wrist"][0] - j["forearm_elbow"][0], 0,
                              j["forearm_wrist"][1] - j["forearm_elbow"][1]]) * MM

    @staticmethod
    def _head(pv, read):
        """Cabeza: carcasa en U (A) con el cuello (B) detrás, y el LiDAR encima."""
        shell = read("Head A")
        p = shell.points
        # A: 107 mm de ancho (x -> lado), 43 de fondo (y -> delante), 54 de alto
        shell.points = np.column_stack([p[:, 1] + 30.0, -p[:, 0], p[:, 2] + 8.0]) * MM
        neck = read("Head B")
        p = neck.points
        neck.points = np.column_stack([p[:, 1] - 12.0, -p[:, 0], p[:, 2] + 2.0]) * MM
        lidar = pv.Cylinder(center=LIDAR_ON_HEAD, direction=(0, 0, 1), radius=0.024, height=0.03)
        ring = pv.Cylinder(center=LIDAR_ON_HEAD + [0, 0, 0.004], direction=(0, 0, 1), radius=0.0245,
                           height=0.008)
        return [(shell, DARK), (neck, YELLOW), (lidar, BLACK), (ring, "#40c4ff")]

    def transforms(self, body_tf):
        """Matriz 4x4 (mundo) de cada eslabón, dada la del cuerpo."""
        q = self.pose
        base = body_tf @ _tf(_rot_z(q["yaw"]), MOUNT)
        # Ángulos positivos suben el eslabón (rotación sobre y negativa en este marco)
        arm = base @ _tf(_rot_y(-q["shoulder"]), self.shoulder)
        fore = arm @ _tf(_rot_y(-q["elbow"]), self.upper)
        head = fore @ _tf(_rot_y(-q["wrist"]), self.fore)  # wrist = -(hombro + codo): cabeza nivelada
        return {"base": base, "arm": arm, "forearm": fore, "head": head}

    def lidar_origin(self, body_tf):
        """Posición del LiDAR (mundo)."""
        return (self.transforms(body_tf)["head"] @ np.r_[LIDAR_ON_HEAD, 1.0])[:3]
