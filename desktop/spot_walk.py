"""Visor de marcha: maneja el SpotMicro con el teclado usando gait.py, sin hardware.

    python spot_walk.py                 # trote
    python spot_walk.py --gait walking

Teclas:  W/S adelante/atrás · A/D lateral · Q/E giro · espacio parar
         1 crawling · 2 walking · 3 trotting
"""
import argparse
import time
from math import cos, degrees, sin

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from gait import BODY_HEIGHT, GaitManager
from kinematics import Kinematics
from spot_model3d import SpotModel, SpotScene, solve_skeleton

BG = (0.055, 0.07, 0.095)
KEYS = {'w': (0, 0.25), 's': (0, -0.25), 'a': (1, 0.25), 'd': (1, -0.25),
        'q': (2, 0.25), 'e': (2, -0.25)}
GAITS = {'1': 'crawling', '2': 'walking', '3': 'trotting'}


class WalkViewer:
    def __init__(self, gait='trotting'):
        # matplotlib usa q (cerrar) y s (guardar) por defecto
        for name in ('keymap.quit', 'keymap.save'):
            plt.rcParams[name] = [k for k in plt.rcParams[name] if k not in ('q', 's')]
        self.kin = Kinematics()
        self.gait = GaitManager(self.kin)
        self.gait.set_gait(gait)
        self.cmd = [0.0, 0.0, 0.0]
        self.pose = np.zeros(3)  # x, y, yaw del cuerpo en el mundo (odometría ideal)
        self.trail = [self.pose[:2].copy()]
        self.last = time.perf_counter()

        self.fig = plt.figure("Spot Micro Twins · Marcha", figsize=(11, 7.5), facecolor=BG)
        self.ax = self.fig.add_axes([0.0, 0.08, 1.0, 0.82], projection="3d")
        self.ax.set_axis_off()
        self.scene = SpotScene(self.ax, SpotModel(self.kin), show_skeleton=False)
        self.ax.view_init(elev=28, azim=-50)
        self.fig.text(0.04, 0.95, "SPOT MICRO  /  MARCHA", color="#f4c542", fontsize=18, weight="bold")
        self.info = self.fig.text(0.04, 0.91, "", color="w", fontsize=11, family="monospace")
        self.fig.text(0.04, 0.03, "W/S avanzar · A/D lateral · Q/E girar · espacio parar · 1/2/3 marcha",
                      color="#8fa5be", fontsize=10)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

    def on_key(self, event):
        key = (event.key or '').lower()
        if key in KEYS:
            axis, delta = KEYS[key]
            self.cmd[axis] = float(np.clip(self.cmd[axis] + delta, -1, 1))
        elif key == ' ':
            self.cmd = [0.0, 0.0, 0.0]
        elif key in GAITS:
            self.gait.set_gait(GAITS[key])
        self.gait.set_command(*self.cmd)

    def _body_velocity(self):
        """Velocidad que produce la marcha: el pie recorre step_length durante el apoyo."""
        p = self.gait.gait_parameters[self.gait.active_gait]
        v = p['step_length'] / (p['duty_factor'] * p['cycle_time']) * self.gait.gait_speed
        x, y, _ = self.gait.neutral_positions[0]
        vx, vy, wz = self.gait.command
        return vx * v, vy * v, wz * v / np.hypot(x, y)

    def update(self, _frame):
        now = time.perf_counter()
        dt, self.last = min(now - self.last, 0.1), now
        feet = np.array(self.gait.step(dt))

        vx, vy, wz = self._body_velocity()
        yaw = self.pose[2]
        self.pose += dt * np.array([vx * cos(yaw) - vy * sin(yaw), vx * sin(yaw) + vy * cos(yaw), wz])
        if np.linalg.norm(self.pose[:2] - self.trail[-1]) > 0.01:
            self.trail = (self.trail + [self.pose[:2].copy()])[-300:]

        # Pies del marco del cuerpo al mundo: misma IK, solo cambia dónde está el robot
        c, s = cos(self.pose[2]), sin(self.pose[2])
        rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        offset = np.array([self.pose[0], self.pose[1], 0.0])
        feet_world = feet @ rz.T + offset
        body = offset + [0, 0, BODY_HEIGHT]
        skel = solve_skeleton(self.kin, feet_world.tolist(), [0, 0, degrees(self.pose[2])], body.tolist())

        self.scene.draw([(skel, (0, 0, 0), None)], center=(self.pose[0], self.pose[1], 0), span=0.25)
        trail = np.array(self.trail)
        self.scene.lines += self.ax.plot(trail[:, 0], trail[:, 1], np.zeros(len(trail)),
                                         color="#f4c542", lw=1.2)
        state = "OK" if skel["ok"] else "FUERA DE ALCANCE"
        self.info.set_text(f"{self.gait.active_gait:<9} vx {self.cmd[0]:+.2f}  vy {self.cmd[1]:+.2f}  "
                           f"giro {self.cmd[2]:+.2f}   ·  IK {state}")
        return []

    def run(self):
        self._anim = FuncAnimation(self.fig, self.update, interval=40, cache_frame_data=False)
        plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--gait", default="trotting", choices=list(GAITS.values()))
    WalkViewer(ap.parse_args().gait).run()


if __name__ == "__main__":
    main()
