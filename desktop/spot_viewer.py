"""Visor 3D del SpotMicro: reproduce la coreografía con el modelo pegado al esqueleto.

    python spot_viewer.py            # gemelos bailando en espejo
    python spot_viewer.py --single   # un solo Spot

Arrastra con el mouse para rotar la cámara.
"""

import argparse
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, CheckButtons

from spot_dance import Dance
from spot_model3d import SpotModel, SpotScene, solve_skeleton
from spotmicro_gui import Kinematics

BG = (0.08, 0.1, 0.13)
TWIN_OFFSETS = [(0, 0.19, 0), (0, -0.19, 0)]


class Viewer:
    def __init__(self, twins=True, bpm=96):
        self.kin = Kinematics()
        self.model = SpotModel(self.kin)
        self.dance = Dance(bpm=bpm)
        self.twins = twins
        self.playing = True
        self.t = 0.0
        self.last = time.perf_counter()

        self.fig = plt.figure("SpotMicro · Vista 3D", figsize=(11, 6.5), facecolor=BG)
        self.ax = self.fig.add_axes([0.0, 0.08, 1.0, 0.9], projection="3d")
        self.ax.set_axis_off()
        self.scene = SpotScene(self.ax, self.model, show_skeleton=False)
        self.ax.view_init(elev=18, azim=-58)

        self.title = self.fig.text(0.02, 0.94, "", color="w", fontsize=14, weight="bold")
        self.info = self.fig.text(0.02, 0.905, "", color="0.65", fontsize=9)
        self.fig.text(0.98, 0.02, f"modelo: {self.model.source} · {self.model.face_count} caras",
                      color="0.5", fontsize=8, ha="right")
        self._build_controls()

    def _build_controls(self):
        def button(x, label, cb):
            b = Button(self.fig.add_axes([x, 0.015, 0.1, 0.045]), label,
                       color=(0.16, 0.19, 0.23), hovercolor=(0.25, 0.3, 0.36))
            b.label.set_color("w")
            b.on_clicked(cb)
            return b

        self.btn_play = button(0.02, "Pausa", self.toggle_play)
        self.btn_restart = button(0.13, "Reiniciar", self.restart)
        self.btn_twins = button(0.24, "1 Spot" if self.twins else "Gemelos", self.toggle_twins)
        ax = self.fig.add_axes([0.36, 0.005, 0.2, 0.065], facecolor=BG)
        self.checks = CheckButtons(ax, ["Modelo", "Esqueleto"], [True, False],
                                   frame_props={"edgecolor": "w", "facecolor": BG},
                                   check_props={"color": "#40c4ff"})
        for label in self.checks.labels:
            label.set_color("w")
        self.checks.on_clicked(self.toggle_layer)

    # ----- callbacks -----
    def toggle_play(self, _):
        self.playing = not self.playing
        self.btn_play.label.set_text("Pausa" if self.playing else "Play")

    def restart(self, _):
        self.t = 0.0

    def toggle_twins(self, _):
        self.twins = not self.twins
        self.btn_twins.label.set_text("1 Spot" if self.twins else "Gemelos")

    def toggle_layer(self, label):
        if label == "Modelo":
            self.scene.show_model = not self.scene.show_model
        else:
            self.scene.show_skeleton = not self.scene.show_skeleton

    # ----- animación -----
    def update(self, _frame):
        now = time.perf_counter()
        if self.playing:
            self.t = (self.t + now - self.last) % self.dance.duration
        self.last = now

        offsets = TWIN_OFFSETS if self.twins else [(0, 0, 0)]
        robots = []
        for k, off in enumerate(offsets):
            feet, rot, body = self.dance.pose_at(self.t, mirror=(k == 1)).absolute()
            robots.append((solve_skeleton(self.kin, feet, rot, body), off, None))
        self.scene.draw(robots, center=(0, 0, 0), span=0.22)
        self.ax.set_zlim(0, 0.33)

        self.title.set_text(self.dance.move_name_at(self.t).replace("_", " ").upper())
        self.info.set_text(f"{self.t:5.1f} / {self.dance.duration:.1f} s  ·  "
                           f"{'gemelos en espejo' if self.twins else '1 Spot'}")
        return []

    def run(self):
        self._anim = FuncAnimation(self.fig, self.update, interval=40, cache_frame_data=False)
        plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--single", action="store_true")
    ap.add_argument("--bpm", type=float, default=96)
    args = ap.parse_args()
    Viewer(twins=not args.single, bpm=args.bpm).run()


if __name__ == "__main__":
    main()
