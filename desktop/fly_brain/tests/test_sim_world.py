"""Mundo simulado de la cueva (sim_world.py): geometría, LiDAR, gas, calor y terreno.

    cd desktop/fly_brain && python -m unittest discover -s tests -v
"""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from sim_world import (GAS_FOUND, LIDAR_ELEVATIONS, MAP_VOXEL, TEMP_AMBIENT, WALL_ROUGH, CaveWorld,
                       _free)


def free(world, x, y, z):
    c, t = world.cave, world.terrain
    return _free(x, y, z, c.x0, c.y0, 0.04, c.grid_dist, c.grid_radius, c.grid_ceil, t.rough,
                 t.kx, t.ky, t.amp, t.phase, c.wall_k, c.wall_phase, c.wall_amp, WALL_ROUGH)


class Cave(unittest.TestCase):
    def setUp(self):
        self.w = CaveWorld()

    def test_inside_and_outside(self):
        c, t, r, h = self.w.cave.point_at(5.0)
        normal = np.array([-t[1], t[0]])
        self.assertTrue(free(self.w, *c, 0.2))                          # centro de la galería
        self.assertFalse(free(self.w, *(c + normal * r * (1 + WALL_ROUGH + 0.05)), 0.2))  # en la roca
        self.assertFalse(free(self.w, *c, h * (1 + WALL_ROUGH) + 0.05))                    # sobre el techo
        self.assertFalse(free(self.w, *c, -0.01))                       # bajo el suelo

    def test_lidar_measures_the_wall(self):
        # Rayo horizontal perpendicular a la galería: distancia ≈ radio - desplazamiento lateral
        w = self.w
        i = 50
        c, t, r = w.cave.center[i], w.cave.tangent[i], w.cave.radius[i]
        w.pose = np.array([c[0], c[1], math.atan2(t[1], t[0])])
        _, ranges = w.lidar_scan(0.19)
        flat = ranges[:, int(np.argmin(np.abs(LIDAR_ELEVATIONS)))]
        left = flat[3 * len(flat) // 4]  # azimut +90°
        # A la altura del LiDAR (0,29 m) la pared está algo más cerca que el radio (techo en arco),
        # y la roca rugosa la acerca o la aleja hasta WALL_ROUGH * radio
        self.assertLess(abs(left - r), 0.25 + WALL_ROUGH * r)
        self.assertTrue(np.isfinite(ranges).mean() > 0.9)

    def test_map_accumulates_unique_voxels(self):
        w = self.w
        w.lidar_scan(0.19)
        n = len(w.map_points)
        w.lidar_scan(0.19)  # mismo sitio: nada nuevo
        self.assertEqual(len(w.map_points), n)
        keys = np.floor(w.map_points / MAP_VOXEL).astype(int)
        self.assertEqual(len(np.unique(keys, axis=0)), n)

    def test_robot_cannot_go_through_walls(self):
        w = self.w
        for _ in range(400):
            w.update(0.05, (0.0, 0.5, 0.0))  # andar de lado contra la pared
        i, d = w.cave.nearest(w.pose[:2])
        self.assertLessEqual(d, w.cave.radius[i] * (1 - WALL_ROUGH) - 0.2 + 1e-6)


class Sensors(unittest.TestCase):
    def setUp(self):
        self.w = CaveWorld()

    def test_gas_grows_toward_the_leak(self):
        s = np.array([2.0, 6.0, 10.0, self.w.gas_s])
        ppm = [self.w.gas_ppm(self.w.cave.point_at(v)[0])[0] for v in s]
        self.assertTrue(np.all(np.diff(ppm) > 0))
        self.assertGreater(ppm[-1], GAS_FOUND)

    def test_heat_peaks_at_the_source(self):
        self.assertGreater(self.w.temperature(self.w.heat_xy)[0], 90)
        far = self.w.cave.point_at(1.0)[0]
        self.assertAlmostEqual(self.w.temperature(far)[0], TEMP_AMBIENT, places=1)

    def test_gas_gradient_points_to_the_leak(self):
        w = self.w
        c, t, _, _ = w.cave.point_at(w.gas_s - 3.0)
        w.pose = np.array([c[0], c[1], math.atan2(t[1], t[0]) + 0.6])  # mirando algo a la izquierda
        w.lidar_scan(0.19)
        bearing, _, _, _, rd = w.senses()
        self.assertEqual(rd["modo"], "siguiendo gas")
        self.assertLess(bearing, 0)  # gira a la derecha, de vuelta a la galería

    def test_heat_makes_it_steer_away(self):
        w = self.w
        d = w.heat_xy - w.cave.point_at(0.0)[0]
        w.pose = np.array([*(w.heat_xy - 0.45 * d / np.linalg.norm(d)), math.atan2(d[1], d[0]) + 0.5])
        w.lidar_scan(0.19)
        _, _, looming, loom_bearing, rd = w.senses()
        self.assertGreater(rd["temp_c"], 40)
        self.assertGreater(looming, 0)       # calor fuerte: amenaza para la mosca
        self.assertLess(loom_bearing, 0)     # está a la derecha del robot


class Terrain(unittest.TestCase):
    def test_flat_terrain_keeps_body_level(self):
        w = CaveWorld()
        feet = np.array([[0.1, 0.08, 0], [-0.1, 0.08, 0], [-0.1, -0.08, 0], [0.1, -0.08, 0]]) + [*w.pose[:2], 0]
        f, rpy, body = w.body_on_terrain(feet, 0.19)
        self.assertEqual(rpy[:2], [0.0, 0.0])
        self.assertAlmostEqual(body[2], 0.19)

    def test_rough_terrain_puts_feet_on_the_ground(self):
        w = CaveWorld()
        w.terrain.rough = True
        feet = np.array([[0.1, 0.08, 0], [-0.1, 0.08, 0], [-0.1, -0.08, 0], [0.1, -0.08, 0]]) + [*w.pose[:2], 0]
        f, rpy, _ = w.body_on_terrain(feet, 0.19)
        np.testing.assert_allclose(f[:, 2], w.ground(f[:, 0], f[:, 1]))
        self.assertGreater(np.ptp(f[:, 2]), 0.005)
        self.assertTrue(all(abs(a) <= 15 for a in rpy[:2]))
        self.assertTrue(free(w, *w.cave.point_at(3.0)[0], 0.15))  # el hueco sigue ahí


if __name__ == "__main__":
    unittest.main()
