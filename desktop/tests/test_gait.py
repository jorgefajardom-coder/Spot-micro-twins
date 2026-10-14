"""Marcha con comando en vivo, sin ventanas ni hardware."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gait import GaitManager, check
from kinematics import Calibration, Kinematics


def gait(name='trotting', cmd=(1, 0, 0)):
    g = GaitManager(Kinematics())
    g.set_gait(name)
    g.set_command(*cmd)
    return g


def trajectory(g, seconds=2.0, dt=0.01):
    return np.array([g.step(dt) for _ in range(int(seconds / dt))])


class LiveCommand(unittest.TestCase):
    def test_feet_never_jump_between_frames(self):
        # Antes el balanceo empezaba en +L y el apoyo terminaba en -L: salto de 2L.
        for name in ('crawling', 'walking', 'trotting'):
            with self.subTest(gait=name):
                feet = trajectory(gait(name))
                jump = np.linalg.norm(np.diff(feet, axis=0), axis=2).max()
                self.assertLess(jump, 0.01)

    def test_command_changes_while_walking(self):
        g = gait(cmd=(1, 0, 0))
        forward = np.ptp(trajectory(g)[:, :, 0], axis=0)
        g.set_command(0, 1, 0)
        sideways = trajectory(g)
        self.assertGreater(forward.min(), 0.05)
        self.assertLess(np.ptp(sideways[:, :, 0], axis=0).max(), 1e-9)
        self.assertGreater(np.ptp(sideways[:, :, 1], axis=0).min(), 0.05)

    def test_turn_moves_left_and_right_legs_opposite(self):
        g = gait(cmd=(0, 0, 1))
        g.gait_phase = 0.0
        p = g.gait_parameters['trotting']
        g.step(0.01)  # en apoyo: el pie se mueve contra el giro del cuerpo
        before = np.array(g.current_leg_positions)
        g.step(0.05)
        delta = np.array(g.current_leg_positions) - before
        lf, rb = delta[0], delta[2]  # misma fase en trote, lados opuestos
        self.assertEqual(p['phase_offset'][0], p['phase_offset'][2])
        self.assertLess(np.dot(lf[:2], rb[:2]), 0)

    def test_zero_command_keeps_feet_on_ground(self):
        feet = trajectory(gait(cmd=(0, 0, 0)))
        self.assertLess(np.abs(feet[:, :, 2]).max(), 1e-9)
        self.assertLess(np.abs(feet - feet[0]).max(), 1e-9)

    def test_command_is_clipped(self):
        g = gait(cmd=(5, -5, math.inf))
        self.assertEqual(g.command, (1.0, -1.0, 1.0))


class Reachability(unittest.TestCase):
    def test_all_gaits_and_commands_reachable_within_calibration(self):
        kin = Kinematics()
        ok, msgs, peak = check(kin, GaitManager(kin), calibration=Calibration.load())
        self.assertTrue(ok, msgs[:5])
        self.assertLess(peak, 360)


if __name__ == '__main__':
    unittest.main()
