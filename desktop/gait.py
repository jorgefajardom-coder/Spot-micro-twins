"""Marchas del SpotMicro con comando de velocidad en vivo.

Extraído de spotmicro_gui.py. Mantiene la API que usa la GUI (set_gait,
start_gait, stop_gait, get_current_positions, update_gait_parameter) y añade:

- set_command(vx, vy, wz): avance, lateral y giro en [-1, 1], se puede cambiar
  mientras camina (joystick, teclado o un controlador externo).
- step(dt): avanza la marcha sin hilos y devuelve las posiciones de los pies,
  para bucles de control propios y pruebas.

    python gait.py --check      # valida alcance de la IK y velocidad de servos
"""
import argparse
import math
import threading
import time

import numpy as np

# Por debajo de esta magnitud de comando las patas dejan de levantarse.
COMMAND_DEADBAND = 0.15


class GaitManager:
    def __init__(self, kinematics):
        self.kin = kinematics
        self.active_gait = None
        self.gait_phase = 0.0
        self.gait_speed = 1.0
        self.is_moving = False
        self.gait_thread = None
        self.stop_gait_flag = False
        self.command = (0.0, 0.0, 0.0)  # vx, vy, wz en [-1, 1]

        # Parámetros de gaits
        self.gait_parameters = {
            'crawling': {
                'step_height': 0.05,
                'step_length': 0.08,
                'cycle_time': 2.0,
                'duty_factor': 0.75,
                'phase_offset': [0.0, 0.25, 0.5, 0.75]
            },
            'walking': {
                'step_height': 0.04,
                'step_length': 0.06,
                'cycle_time': 1.5,
                'duty_factor': 0.6,
                'phase_offset': [0.0, 0.5, 0.25, 0.75]
            },
            'trotting': {
                'step_height': 0.03,
                'step_length': 0.10,
                'cycle_time': 1.0,
                'duty_factor': 0.5,
                'phase_offset': [0.0, 0.5, 0.0, 0.5]
            },
            'standing': {
                'step_height': 0.0,
                'step_length': 0.0,
                'cycle_time': 1.0,
                'duty_factor': 1.0,
                'phase_offset': [0.0, 0.0, 0.0, 0.0]
            }
        }

        self.neutral_positions = [
            [0.10, 0.08, -0],
            [-0.10, 0.08, -0],
            [-0.10, -0.08, -0],
            [0.10, -0.08, -0]
        ]

        self.current_leg_positions = [p.copy() for p in self.neutral_positions]

    def set_command(self, vx, vy=0.0, wz=0.0):
        """Velocidad deseada: vx adelante, vy izquierda, wz giro antihorario. Cada una en [-1, 1]."""
        clip = lambda v: max(-1.0, min(1.0, float(v)))
        self.command = (clip(vx), clip(vy), clip(wz))

    def _leg_direction(self, leg_id):
        """Dirección de paso de una pata: traslación + giro alrededor del centro del cuerpo."""
        vx, vy, wz = self.command
        x, y, _ = self.neutral_positions[leg_id]
        r = math.hypot(x, y) or 1.0
        dx, dy = vx - wz * y / r, vy + wz * x / r
        mag = math.hypot(dx, dy)
        if mag > 1.0:
            dx, dy = dx / mag, dy / mag
        return dx, dy

    def _calculate_leg_positions(self, params):
        """Calcular posiciones de las patas basado en la fase del gait"""
        positions = []
        activity = min(1.0, max(map(abs, self.command)) / COMMAND_DEADBAND)

        for leg_id in range(4):
            leg_phase = (self.gait_phase + params['phase_offset'][leg_id]) % 1.0
            base_x, base_y, base_z = self.neutral_positions[leg_id]
            dx, dy = self._leg_direction(leg_id)
            length = params['step_length']

            if leg_phase < params['duty_factor']:
                # Apoyo: el pie va de +L/2 a -L/2 respecto a la posición neutral
                s = 0.5 - leg_phase / params['duty_factor']
                lift = 0.0
            else:
                # Balanceo: vuelve de -L/2 a +L/2 por el aire
                swing = (leg_phase - params['duty_factor']) / (1.0 - params['duty_factor'])
                s = swing - 0.5
                lift = math.sin(swing * math.pi) * params['step_height'] * activity

            positions.append([base_x + dx * length * s,
                              base_y + dy * length * s,
                              base_z + lift])

        return positions

    def step(self, dt):
        """Avanza la fase dt segundos y devuelve las posiciones de los 4 pies."""
        params = self.gait_parameters[self.active_gait or 'standing']
        cycle_time = params['cycle_time'] / self.gait_speed
        self.gait_phase = (self.gait_phase + dt / cycle_time) % 1.0
        self.current_leg_positions = self._calculate_leg_positions(params)
        return self.current_leg_positions

    def set_gait(self, gait_name):
        """Configurar el gait activo"""
        if gait_name in self.gait_parameters:
            self.active_gait = gait_name
            print(f"✅ Gait configurado: {gait_name}")
            return True
        else:
            print(f"❌ Gait no reconocido: {gait_name}")
            return False

    def start_gait(self, direction=(1, 0), speed_multiplier=1.0, turn=0.0):
        """Iniciar el movimiento con el gait activo"""
        if self.active_gait is None:
            print("❌ No hay gait configurado")
            return False

        if self.is_moving:
            self.stop_gait()

        self.set_command(direction[0], direction[1], turn)
        self.is_moving = True
        self.stop_gait_flag = False
        self.gait_speed = speed_multiplier

        # Iniciar hilo de movimiento
        self.gait_thread = threading.Thread(target=self._gait_loop, daemon=True)
        self.gait_thread.start()
        print(f"🚀 Iniciando gait: {self.active_gait}")
        return True

    def stop_gait(self):
        """Detener el movimiento y volver a posición inicial (standing)"""
        self.stop_gait_flag = True
        self.is_moving = False

        if self.gait_thread and self.gait_thread.is_alive():
            self.gait_thread.join(timeout=1.0)

        # Transición suave a posiciones neutrales (standing)
        steps = 10
        current_positions = [p.copy() for p in self.current_leg_positions]

        def smooth_transition():
            for step in range(1, steps + 1):
                if self.is_moving:  # Si se reactiva el movimiento, cancelar transición
                    return
                alpha = step / steps
                self.current_leg_positions = [
                    [c + (t - c) * alpha for c, t in zip(current, target)]
                    for current, target in zip(current_positions, self.neutral_positions)
                ]
                time.sleep(0.05)
            self.current_leg_positions = [p.copy() for p in self.neutral_positions]

        threading.Thread(target=smooth_transition, daemon=True).start()
        print("🛑 Gait detenido - Volviendo a posición inicial")

    def _gait_loop(self):
        """Loop principal de movimiento del gait (50 Hz)"""
        last_time = time.time()
        while not self.stop_gait_flag and self.is_moving:
            now = time.time()
            self.step(now - last_time)
            last_time = now
            time.sleep(0.02)

    def get_current_positions(self):
        """Obtener las posiciones actuales de las patas"""
        return self.current_leg_positions

    def set_neutral_positions(self, positions):
        """Establecer nuevas posiciones neutrales"""
        self.neutral_positions = [list(p) for p in positions]

    def update_gait_parameter(self, gait_name, parameter, value):
        """Actualizar parámetros específicos de un gait"""
        if gait_name in self.gait_parameters and parameter in self.gait_parameters[gait_name]:
            self.gait_parameters[gait_name][parameter] = value
            print(f"✅ Parámetro {parameter} de {gait_name} actualizado a {value}")
            return True
        return False


# Altura del cuerpo con la que se validan las marchas (la misma del baile).
BODY_HEIGHT = 0.19

CHECK_COMMANDS = {
    'adelante': (1, 0, 0), 'atrás': (-1, 0, 0), 'lateral': (0, 1, 0),
    'giro': (0, 0, 1), 'curva': (0.7, 0, -0.5), 'quieto': (0, 0, 0),
}


def check(kin, gait, rate_hz=100, max_joint_speed=360.0, calibration=None):
    """Recorre cada marcha y comando dos ciclos. Devuelve (ok, mensajes, velocidad pico °/s)."""
    msgs, peak = [], 0.0
    dt = 1.0 / rate_hz
    for name in ('crawling', 'walking', 'trotting'):
        gait.set_gait(name)
        cycle = gait.gait_parameters[name]['cycle_time']
        for label, cmd in CHECK_COMMANDS.items():
            gait.set_command(*cmd)
            gait.gait_phase, prev = 0.0, None
            for i in range(int(2 * cycle * rate_hz)):
                feet = gait.step(dt)
                angles = [kin.leg_IK(feet[leg], [0, 0, 0], leg, False, [0, 0, BODY_HEIGHT])[:3]
                          for leg in range(4)]
                if any(math.isnan(a) for leg in angles for a in leg):
                    msgs.append(f"{name}/{label} t={i * dt:.2f}s: pata fuera de alcance")
                    break
                if calibration is not None:
                    _, errors = calibration.to_servo_degrees(angles)
                    if errors:
                        msgs.append(f"{name}/{label} t={i * dt:.2f}s: {errors[0]}")
                        break
                angles = np.degrees(np.array(angles, dtype=float))
                if prev is not None:
                    speed = np.abs((angles - prev + 180) % 360 - 180).max() / dt
                    peak = max(peak, speed)
                    if speed > max_joint_speed:
                        msgs.append(f"{name}/{label} t={i * dt:.2f}s: {speed:.0f}°/s")
                        break
                prev = angles
    return not msgs, msgs, peak


def main():
    from kinematics import Calibration, Kinematics

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--check", action="store_true", help="validar alcance y velocidades")
    ap.parse_args()

    kin = Kinematics()
    ok, msgs, peak = check(kin, GaitManager(kin), calibration=Calibration.load())
    print(f"Velocidad pico {peak:.0f}°/s · " + ("OK" if ok else f"{len(msgs)} problemas"))
    for m in msgs[:20]:
        print("  ", m)


if __name__ == "__main__":
    main()
