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

BG = (0.055, 0.07, 0.095)
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
        self.zoom = 1.3
        self.wireframe = False

        self.fig = plt.figure("Spot Micro Twins · Visor 3D", figsize=(12, 8), facecolor=BG)
        self.ax = self.fig.add_axes([0.02, 0.18, 0.96, 0.69], projection="3d")
        self.ax.set_axis_off()
        self.scene = SpotScene(self.ax, self.model, show_skeleton=False)
        self.ax.view_init(elev=23, azim=-32)

        self.fig.text(0.04, 0.95, "SPOT MICRO  /  TWINS", color="#f4c542", fontsize=18, weight="bold")
        self.fig.text(0.96, 0.95, "VISOR 3D", color="#8fa5be", fontsize=10, ha="right")
        self.title = self.fig.text(0.04, 0.905, "", color="w", fontsize=11)
        self.info = self.fig.text(0.96, 0.905, "", color="#8fa5be", fontsize=10, ha="right")
        self.fig.text(0.04, 0.025, "Arrastra para girar · Rueda para acercar", color="#8fa5be", fontsize=9)
        self.fig.text(0.96, 0.025, "Modelo CAD articulado", color="#8fa5be", fontsize=9, ha="right")
        self._build_controls()
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)

    def _build_controls(self):
        def button(x, label, cb, y=0.115):
            b = Button(self.fig.add_axes([x, y, 0.13, 0.043]), label,
                       color="#192536", hovercolor="#30435a")
            b.label.set_color("w")
            b.label.set_fontsize(9)
            for spine in b.ax.spines.values():
                spine.set_edgecolor('#35465b')
            b.on_clicked(cb)
            return b

        self.btn_play = button(0.04, "Pausa", self.toggle_play)
        self.btn_restart = button(0.18, "Reiniciar", self.restart)
        self.btn_twins = button(0.32, "Ver un robot" if self.twins else "Ver gemelos", self.toggle_twins)
        self.camera_buttons = [button(x, name, lambda _, view=view: self.set_camera(view), y=0.062)
                               for x, name, view in [(0.04, 'Perspectiva', (23, -32)),
                                                     (0.18, 'Lateral', (0, -90)),
                                                     (0.32, 'Superior', (90, -90))]]
        ax = self.fig.add_axes([0.69, 0.055, 0.27, 0.11], facecolor=BG)
        for spine in ax.spines.values():
            spine.set_visible(False)
        self.checks = CheckButtons(ax, ["Modelo", "Esqueleto", "Suelo", "Malla"], [True, False, True, False],
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
        self.btn_twins.label.set_text("Ver un robot" if self.twins else "Ver gemelos")

    def toggle_layer(self, label):
        if label == "Modelo":
            self.scene.show_model = not self.scene.show_model
        elif label == "Esqueleto":
            self.scene.show_skeleton = not self.scene.show_skeleton
        elif label == "Suelo":
            self.scene.show_floor = not self.scene.show_floor
        elif label == "Malla":
            self.wireframe = not self.wireframe
            self.scene.set_wireframe(self.wireframe)

    def set_camera(self, view):
        self.ax.view_init(elev=view[0], azim=view[1])
        self.zoom = 1.3
        self.fig.canvas.draw_idle()

    def on_scroll(self, event):
        if event.inaxes == self.ax:
            self.zoom = min(1.9, max(0.7, self.zoom * (1.08 ** event.step)))

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
        self.scene.draw(robots, center=(0, 0, 0), span=0.20, zoom=self.zoom)

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
