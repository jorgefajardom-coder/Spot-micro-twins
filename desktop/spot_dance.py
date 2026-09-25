"""Coreografía de baile del SpotMicro.

Una coreografía es una lista de movimientos de N tiempos (beats). Cada movimiento es una
función u∈[0,1] -> Pose relativa a la postura neutra, y vale cero al inicio y al final
para que los movimientos se encadenen sin saltos.

Uso:
    python spot_dance.py --gif ../docs/media/spot_dance.gif      # render para la documentación
    python spot_dance.py --check                                  # valida IK y velocidades
"""

import argparse
from dataclasses import dataclass, field
from math import cos, pi, sin

import numpy as np

# Postura neutra (misma que "Restablecer" en la GUI)
NEUTRAL_FEET = np.array([[0.10, 0.10, 0.0], [-0.10, 0.10, 0.0],
                         [-0.10, -0.10, 0.0], [0.10, -0.10, 0.0]])
NEUTRAL_HEIGHT = 0.19
LF, LB, RB, RF = range(4)


@dataclass
class Pose:
    body: np.ndarray = field(default_factory=lambda: np.zeros(3))  # x, y, z (m)
    rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))   # roll, pitch, yaw (grados)
    feet: np.ndarray = field(default_factory=lambda: np.zeros((4, 3)))  # desplazamiento por pata

    def absolute(self):
        """-> (foot_targets 4x3, rot [r,p,y] grados, body_pos) listo para Kinematics."""
        body = self.body + np.array([0.0, 0.0, NEUTRAL_HEIGHT])
        return (NEUTRAL_FEET + self.feet).tolist(), self.rpy.tolist(), body.tolist()

    def mirrored(self):
        """Espejo izquierda/derecha (para el gemelo): y, roll y yaw cambian de signo."""
        flip = np.array([1.0, -1.0, 1.0])
        feet = self.feet[[RF, RB, LB, LF]] * flip
        return Pose(self.body * flip, self.rpy * np.array([-1.0, 1.0, -1.0]), feet)


def bump(u):
    """0 -> 1 -> 0 suave (sin^2)."""
    return sin(pi * u) ** 2


def wave(u, cycles):
    """Oscilación que empieza y termina en 0 con envolvente suave."""
    return sin(2 * pi * cycles * u) * min(1.0, 4 * u, 4 * (1 - u))


# ---------------------------------------------------------------------------
# Movimientos (u ∈ [0,1] -> Pose)
# ---------------------------------------------------------------------------

def crouch(u):
    return Pose(body=np.array([0, 0, -0.035 * bump(u)]))


def bounce(u, beats=4):
    return Pose(body=np.array([0, 0, -0.018 * abs(sin(pi * beats * u))]))


def sway(u, beats=4):
    s = wave(u, beats / 2)
    return Pose(body=np.array([0, 0.022 * s, 0]), rpy=np.array([10 * s, 0, 0]))


def twist(u, beats=4):
    s = wave(u, beats / 2)
    return Pose(rpy=np.array([0, 0, 14 * s]))


def nod(u, beats=4):
    s = wave(u, beats / 2)
    return Pose(body=np.array([0, 0, -0.01 * abs(s)]), rpy=np.array([0, 9 * s, 0]))


def paw_wave(u):
    """Levanta la pata delantera izquierda y saluda, con el peso hacia atrás-derecha."""
    shift = min(1.0, 3 * u, 3 * (1 - u))  # peso atrás-derecha, sostenido en el centro
    lift = min(1.0, 4 * max(0.0, u - 0.12), 4 * max(0.0, 0.88 - u))
    p = Pose(body=np.array([-0.022 * shift, -0.022 * shift, -0.012 * shift]),
             rpy=np.array([-5 * shift, 3 * shift, 0]))
    p.feet[LF] = [0.035 * lift, -0.01 * lift, 0.07 * lift + 0.012 * lift * sin(6 * pi * u)]
    return p


def trot_in_place(u, beats=4):
    """Pares diagonales se levantan alternadamente."""
    p = Pose()
    phase = (u * beats) % 1.0
    pair = int(u * beats) % 2
    h = 0.032 * sin(pi * phase) * min(1.0, 8 * u, 8 * (1 - u))
    for leg in ((LF, RB) if pair == 0 else (LB, RF)):
        p.feet[leg, 2] = h
    p.body[2] = -0.006 * sin(pi * phase)
    return p


def hip_circle(u, turns=2):
    """Círculo de cadera: el cuerpo dibuja un círculo inclinándose hacia donde va."""
    a = 2 * pi * turns * u
    env = min(1.0, 5 * u, 5 * (1 - u))
    return Pose(body=np.array([0.018 * sin(a) * env, 0.018 * (1 - cos(a)) * env, 0]),
                rpy=np.array([6 * (1 - cos(a)) * env, -6 * sin(a) * env, 0]))


def play_bow(u):
    s = bump(u)
    return Pose(body=np.array([-0.018 * s, 0, -0.03 * s]), rpy=np.array([0, 14 * s, 0]))


# Coreografía por defecto: (movimiento, beats)
DEFAULT_ROUTINE = [
    (crouch, 2),
    (bounce, 4),
    (sway, 4),
    (twist, 4),
    (nod, 4),
    (paw_wave, 6),
    (trot_in_place, 4),
    (hip_circle, 4),
    (play_bow, 3),
    (crouch, 2),
]


class Dance:
    def __init__(self, routine=DEFAULT_ROUTINE, bpm=96):
        self.routine = routine
        self.beat = 60.0 / bpm
        self.starts = np.cumsum([0] + [b for _, b in routine])[:-1] * self.beat
        self.duration = sum(b for _, b in routine) * self.beat

    def pose_at(self, t, mirror=False):
        t = min(max(t, 0.0), self.duration - 1e-9)
        i = int(np.searchsorted(self.starts, t, side="right") - 1)
        move, beats = self.routine[i]
        u = (t - self.starts[i]) / (beats * self.beat)
        pose = move(u)
        return pose.mirrored() if mirror else pose

    def move_name_at(self, t):
        i = int(np.searchsorted(self.starts, min(t, self.duration - 1e-9), side="right") - 1)
        return self.routine[i][0].__name__

    def check(self, kin, rate_hz=100, max_joint_speed=360.0):
        """Valida que toda la coreografía sea alcanzable y no exceda la velocidad de los servos.
        Devuelve (ok, mensajes)."""
        msgs, prev, peak = [], None, 0.0
        dt = 1.0 / rate_hz
        for mirror in (False, True):
            prev = None
            for t in np.arange(0, self.duration, dt):
                feet, rot, body = self.pose_at(t, mirror).absolute()
                angles = []
                for leg in range(4):
                    a = kin.leg_IK(feet[leg], rot, leg, False, body)[:3]
                    if any(np.isnan(a)):
                        msgs.append(f"t={t:.2f}s {self.move_name_at(t)}: pata {leg} fuera de alcance")
                        break
                    angles.append(a)
                else:
                    angles = np.degrees(np.array(angles, dtype=float))
                    if prev is not None:
                        d = (angles - prev + 180) % 360 - 180
                        speed = np.abs(d).max() / dt
                        peak = max(peak, speed)
                        if speed > max_joint_speed:
                            msgs.append(f"t={t:.2f}s {self.move_name_at(t)}: {speed:.0f}°/s")
                    prev = angles
        return not msgs, msgs, peak


# ---------------------------------------------------------------------------
# Render GIF para la documentación
# ---------------------------------------------------------------------------

def render_gif(path, kin, dance, fps=15, twins=True, size=(8, 4.6), dpi=90):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    from spot_model3d import SpotModel, SpotScene, solve_skeleton

    model = SpotModel(kin)
    fig = plt.figure(figsize=size, dpi=dpi, facecolor=(0.08, 0.1, 0.13))
    ax = fig.add_axes([-0.12, -0.18, 1.24, 1.36], projection="3d")
    scene = SpotScene(ax, model, show_skeleton=False)
    ax.set_axis_off()
    title = fig.text(0.03, 0.92, "", color="w", fontsize=13, weight="bold")
    fig.text(0.97, 0.05, "SpotMicro V3 · Twins", color="0.6", fontsize=9, ha="right")

    offsets = [(0, 0.19, 0), (0, -0.19, 0)] if twins else [(0, 0, 0)]
    frames = []
    for i, t in enumerate(np.arange(0, dance.duration, 1.0 / fps)):
        robots = []
        for k, off in enumerate(offsets):
            feet, rot, body = dance.pose_at(t, mirror=(k == 1)).absolute()
            robots.append((solve_skeleton(kin, feet, rot, body), off, None))
        scene.draw(robots, center=(0, 0, 0), span=0.2)
        ax.set_zlim(0, 0.33)
        ax.view_init(elev=16, azim=-58 + 18 * sin(2 * pi * t / dance.duration))
        title.set_text(dance.move_name_at(t).replace("_", " ").upper())
        fig.canvas.draw()
        img = Image.frombuffer("RGBA", fig.canvas.get_width_height(),
                               fig.canvas.buffer_rgba()).convert("RGB")
        frames.append(img.quantize(colors=96, method=Image.Quantize.MEDIANCUT))
    plt.close(fig)
    frames[0].save(path, save_all=True, append_images=frames[1:],
                   duration=int(1000 / fps), loop=0, optimize=True)
    return len(frames)


def main():
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from spotmicro_gui import Kinematics

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--gif", help="ruta del GIF a generar")
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--bpm", type=float, default=96)
    ap.add_argument("--single", action="store_true", help="un solo Spot en vez de gemelos")
    ap.add_argument("--check", action="store_true", help="validar alcance y velocidades")
    args = ap.parse_args()

    kin, dance = Kinematics(), Dance(bpm=args.bpm)
    if args.check or not args.gif:
        ok, msgs, peak = dance.check(kin)
        print(f"Duración {dance.duration:.1f} s · velocidad pico {peak:.0f}°/s · "
              + ("OK" if ok else f"{len(msgs)} problemas"))
        for m in msgs[:20]:
            print("  ", m)
    if args.gif:
        n = render_gif(args.gif, kin, dance, fps=args.fps, twins=not args.single)
        print(f"GIF: {args.gif} ({n} frames, {os.path.getsize(args.gif) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
