"""Piloto mosca: lo que ve el robot -> cerebro de la mosca -> comando de marcha.

Codificación (qué se estimula), según el barrido de probe.py:
- Objetivo a un lado        -> LC10 de ese lado     -> DNa02 ipsilateral (girar hacia él)
- Objetivo delante          -> LC9 + LC31a          -> DNp09 / P9 (avanzar)
- Algo se acerca (looming)  -> LPLC2 + LC4          -> Giant Fiber (escape), MDN (atrás)

El conectoma no es simétrico (p. ej. LC10a izquierda lleva DNa02 a 130 Hz y la
derecha a 77 Hz), así que calibrate() mide la respuesta máxima de cada lado al
arrancar y la decodificación usa respuestas normalizadas.

Decodificación (qué se lee): frecuencias de las descendentes en la última ventana,
convertidas a vx (avance) y wz (giro) en [-1, 1] para gait.GaitManager.set_command.
El hemisferio izquierdo del cerebro procesa el ojo izquierdo, así que "left" en
FlyWire es el lado izquierdo del robot.
"""
from dataclasses import dataclass, field
import math

import numpy as np

from lif import Params, make_brain

MAX_HZ = 150.0  # frecuencia de estímulo a saliencia 1 (la del artículo)

LC10 = ('LC10a', 'LC10c-1', 'LC10c-2', 'LC10d')
INPUTS = {
    'objetivo_izq': (LC10, 'left'),
    'objetivo_der': (LC10, 'right'),
    'delante_izq': (('LC9', 'LC31a'), 'left'),
    'delante_der': (('LC9', 'LC31a'), 'right'),
    'looming_izq': (('LPLC2', 'LC4'), 'left'),
    'looming_der': (('LPLC2', 'LC4'), 'right'),
}
OUTPUTS = {
    'P9_izq': ('DNp09', 'left'), 'P9_der': ('DNp09', 'right'),
    'DNa02_izq': ('DNa02', 'left'), 'DNa02_der': ('DNa02', 'right'),
    'MDN': ('MDN', None), 'GF': ('DNp01', None),
}


@dataclass
class Senses:
    """Lo que percibe el robot, en su propio marco (ángulos en rad, + = izquierda)."""
    target_bearing: float | None = None   # None = no se ve
    target_dist: float = 1.0              # m
    looming: float = 0.0                  # 0..1, algo se acerca rápido
    looming_bearing: float = 0.0


@dataclass
class Command:
    vx: float = 0.0
    wz: float = 0.0
    escape: bool = False
    rates: dict = field(default_factory=dict)   # Hz por grupo de salida
    inputs: dict = field(default_factory=dict)  # Hz por grupo de entrada


class FlyPilot:
    def __init__(self, connectome, params=Params(dt=0.1), device='auto', seed=0, smooth_ms=150.0):
        self.c = connectome
        self.brain = make_brain(connectome, params, seed=seed, device=device)

        def group(types, side):
            types = (types,) if isinstance(types, str) else types
            return np.concatenate([connectome.find(t, side) for t in types])

        self.inputs = {k: group(*v) for k, v in INPUTS.items()}
        self.outputs = {k: group(*v) for k, v in OUTPUTS.items()}
        self.smooth_ms = smooth_ms
        self.gain = {'turn_izq': 1.0, 'turn_der': 1.0, 'forward': 1.0}
        self.reset()

    def reset(self):
        self.brain.reset()
        self.vx = self.wz = 0.0
        self.escape_left = 0.0  # ms que quedan de la maniobra de escape

    def _probe(self, inputs, ms=300.0):
        self.brain.reset()
        self.brain.stimulate(*[(self.inputs[k], MAX_HZ) for k in inputs])
        counts = self.brain.run(ms)
        return {k: self.brain.rate(counts, ms, idx) for k, idx in self.outputs.items()}

    def calibrate(self):
        """Respuesta máxima de cada lado (Hz) para normalizar: compensa la asimetría del conectoma."""
        self.gain['turn_izq'] = max(10.0, self._probe(['objetivo_izq'])['DNa02_izq'])
        self.gain['turn_der'] = max(10.0, self._probe(['objetivo_der'])['DNa02_der'])
        r = self._probe(['delante_izq', 'delante_der'])
        self.gain['forward'] = max(10.0, (r['P9_izq'] + r['P9_der']) / 2)
        self.reset()
        return dict(self.gain)

    def encode(self, s: Senses):
        """Frecuencias de estímulo (Hz) para cada grupo de entrada."""
        hz = dict.fromkeys(self.inputs, 0.0)
        if s.target_bearing is not None and abs(s.target_bearing) < math.radians(120):
            # Saliencia: más fuerte de cerca, se apaga al llegar (para no chocar)
            sal = float(np.clip((s.target_dist - 0.15) / 0.25, 0, 1)) * float(np.clip(0.9 / s.target_dist, 0, 1))
            side = math.sin(s.target_bearing)
            hz['objetivo_izq'] = MAX_HZ * sal * max(0.0, side) ** 0.5
            hz['objetivo_der'] = MAX_HZ * sal * max(0.0, -side) ** 0.5
            front = max(0.0, math.cos(s.target_bearing)) ** 2
            hz['delante_izq'] = hz['delante_der'] = MAX_HZ * sal * front
        if s.looming > 0:
            left = 0.5 + 0.5 * math.sin(s.looming_bearing)
            hz['looming_izq'] = MAX_HZ * s.looming * min(1.0, 2 * left)
            hz['looming_der'] = MAX_HZ * s.looming * min(1.0, 2 * (1 - left))
        return hz

    def step(self, senses: Senses, ms=40.0) -> Command:
        """Corre el cerebro `ms` con lo que ve el robot y devuelve el comando de marcha."""
        hz = self.encode(senses)
        self.brain.stimulate(*[(self.inputs[k], r) for k, r in hz.items()])
        counts = self.brain.run(ms)
        self.last_counts = counts  # espigas de todas las neuronas en esta ventana (para visualizar)
        rates = {k: self.brain.rate(counts, ms, idx) for k, idx in self.outputs.items()}

        g = self.gain
        forward = math.tanh(1.5 * (rates['P9_izq'] + rates['P9_der']) / 2 / g['forward'])
        backward = math.tanh(rates['MDN'] / 30.0)
        turn = math.tanh(1.5 * (rates['DNa02_izq'] / g['turn_izq'] - rates['DNa02_der'] / g['turn_der']))
        if rates['GF'] > 80.0:  # Giant Fiber: escape, retroceder un momento
            self.escape_left = 600.0
        escaping = self.escape_left > 0
        self.escape_left = max(0.0, self.escape_left - ms)

        vx_target = -1.0 if escaping else forward - backward
        wz_target = turn
        a = 1.0 - math.exp(-ms / self.smooth_ms)  # filtro: los servos no deben dar tirones
        self.vx += a * (vx_target - self.vx)
        self.wz += a * (wz_target - self.wz)
        return Command(self.vx, self.wz, escaping, rates, hz)
