"""Simulador LIF sobre redes pequeñas hechas a mano (no necesita los datos de FlyWire).

    cd desktop/fly_brain && python -m unittest discover -s tests -v
"""
from pathlib import Path
import sys
import unittest

import numpy as np
import pandas as pd
import scipy.sparse as sp

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from connectome import Connectome
import lif_gpu
from lif import Brain, Params


def network(edges, n):
    """edges: (pre, post, sinapsis con signo)."""
    pre, post, w = zip(*edges) if edges else ((), (), ())
    weights = sp.csc_matrix((np.array(w, np.float32), (post, pre)), shape=(n, n))
    ann = pd.DataFrame({'root_id': np.arange(n), 'cell_type': [f'N{i}' for i in range(n)],
                        'hemibrain_type': None, 'side': 'left', 'super_class': 'central'})
    return Connectome(weights, np.arange(n), ann)


def first_spike_ms(brain, neuron, ms):
    for k in range(int(ms / brain.p.dt)):
        if brain.run(brain.p.dt)[neuron]:
            return (k + 1) * brain.p.dt
    return None


class LIF(unittest.TestCase):
    Brain = Brain
    def test_rest_is_silent(self):
        b = self.Brain(network([(0, 1, 50)], 2), seed=0)
        self.assertEqual(b.run(200).sum(), 0)

    def test_poisson_stimulus_rate(self):
        b = self.Brain(network([], 1), seed=0)
        b.stimulate((np.array([0]), 100))
        rate = b.run(5000)[0] / 5.0
        self.assertAlmostEqual(rate, 100, delta=10)

    def test_strong_synapse_relays_after_delay(self):
        # El pico de v es ~0.16 x g (tau_syn 5 ms frente a tau_m 20 ms): cruzar los 7 mV
        # hasta el umbral pide ~160 sinapsis. 200 * 0.275 mV = 55 mV en g.
        b = self.Brain(network([(0, 1, 200)], 2), seed=0)
        b.v[0] = 0.0  # fuerza una espiga de la neurona 0 en el primer paso
        t = first_spike_ms(b, 1, 20)
        self.assertIsNotNone(t)
        self.assertGreaterEqual(t, b.p.t_dly)

    def test_weak_synapse_stays_subthreshold(self):
        b = self.Brain(network([(0, 1, 20)], 2), seed=0)
        b.v[0] = 0.0
        self.assertEqual(b.run(50)[1], 0)
        self.assertGreater(b.v[1], b.p.v_0)  # sí la despolarizó

    def test_inhibition_blocks_relay(self):
        edges = [(0, 2, 200), (1, 2, -400)]
        b = self.Brain(network(edges, 3), seed=0)
        b.v[[0, 1]] = 0.0
        self.assertEqual(b.run(30)[2], 0)

    def test_refractory_caps_rate(self):
        # Una neurona forzada con g enorme no puede pasar de 1 / t_rfc
        b = self.Brain(network([], 1), seed=0)
        b.g[0] = 1e4
        b.ref_left[0] = 0
        counts = 0
        for _ in range(100):
            b.g[0] = 1e4
            counts += b.run(1)[0]
        self.assertLessEqual(counts, 100 / b.p.t_rfc + 1)

    def test_coarser_dt_keeps_relay(self):
        for dt in (0.1, 0.5):
            with self.subTest(dt=dt):
                b = self.Brain(network([(0, 1, 200), (1, 2, 200)], 3), Params(dt=dt), seed=0)
                b.stimulate((np.array([0]), 100))
                counts = b.run(2000)
                self.assertGreater(counts[2], 0.5 * counts[0])



@unittest.skipUnless(lif_gpu.available(), "sin GPU CUDA")
class LIFGPU(LIF):
    """Las mismas pruebas con el simulador CUDA."""
    Brain = lif_gpu.BrainGPU


if __name__ == '__main__':
    unittest.main()
