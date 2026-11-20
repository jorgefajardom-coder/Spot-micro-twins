"""El piloto elige el paso de integración según dónde corre el cerebro.

    cd desktop/fly_brain && python -m unittest discover -s tests -v
"""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from lif import Params
from pilot import DT_CPU, DT_GPU, FlyPilot, Senses, gpu_available
from test_lif import network


class PilotTimeStep(unittest.TestCase):
    def test_cpu_uses_coarse_step(self):
        p = FlyPilot(network([], 2), device='cpu')
        self.assertEqual(p.brain.p.dt, DT_CPU)

    @unittest.skipUnless(gpu_available(), "sin GPU CUDA")
    def test_gpu_uses_paper_step(self):
        p = FlyPilot(network([], 2), device='gpu')
        self.assertEqual(p.brain.p.dt, DT_GPU)

    def test_explicit_params_are_kept(self):
        p = FlyPilot(network([], 2), params=Params(dt=0.1), device='cpu')
        self.assertEqual(p.brain.p.dt, 0.1)

    def test_step_returns_bounded_command(self):
        p = FlyPilot(network([(0, 1, 200)], 3), device='cpu')
        cmd = p.step(Senses(target_bearing=0.3, target_dist=0.6), ms=40.0)
        self.assertLessEqual(abs(cmd.vx), 1.0)
        self.assertLessEqual(abs(cmd.wz), 1.0)

    def test_leftover_time_is_carried_over(self):
        # 10 fotogramas de 0,7 ms con dt 0,5: el redondeo daría 10 pasos (5 ms);
        # guardando el resto son 14 pasos (7 ms), igual que el tiempo del mundo
        p = FlyPilot(network([], 2), params=Params(dt=0.5), device='cpu')
        for _ in range(10):
            p.step(Senses(), ms=0.7)
        self.assertEqual(p.brain.step_i, 14)

    def test_less_than_one_step_keeps_last_command(self):
        p = FlyPilot(network([], 2), params=Params(dt=0.5), device='cpu')
        first = p.step(Senses(target_bearing=0.2, target_dist=0.6), ms=40.0)
        again = p.step(Senses(), ms=0.1)
        self.assertIs(again, first)
        self.assertEqual(p.last_counts.sum(), 0)


if __name__ == '__main__':
    unittest.main()
