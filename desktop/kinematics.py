"""Cinemática del SpotMicro sin dependencias de interfaz.

Extraído de spotmicro_gui.py para que la GUI, el visor, la marcha (gait.py) y
cualquier controlador externo usen exactamente la misma IK.
"""
import json
import math
from math import sin, cos, asin, acos, atan2, radians, pi, degrees
from pathlib import Path

import numpy as np
from numpy.linalg import norm

DEFAULT_CALIBRATION = Path(__file__).resolve().parents[1] / "calibration" / "offset-default.json"

# -----------------------------
# Utilidades matemáticas
# -----------------------------
def RotMatrix3D(rot, is_radians=True):
    """Matriz de rotación 3D ZYX (yaw, pitch, roll)."""
    if not is_radians:
        rot = [radians(r) for r in rot]
    roll, pitch, yaw = rot
    Rz = np.array([
        [cos(yaw), -sin(yaw), 0],
        [sin(yaw),  cos(yaw), 0],
        [0, 0, 1]
    ])
    Ry = np.array([
        [cos(pitch), 0, sin(pitch)],
        [0, 1, 0],
        [-sin(pitch), 0, cos(pitch)]
    ])
    Rx = np.array([
        [1, 0, 0],
        [0, cos(roll), -sin(roll)],
        [0, sin(roll),  cos(roll)]
    ])
    return Rz @ Ry @ Rx

def point_to_rad(y, z):
    return atan2(z, y)

def normalize_angle(angle_rad):
    """Normaliza un ángulo en radianes al rango [-pi, pi]"""
    angle_rad = angle_rad % (2 * pi)
    if angle_rad > pi:
        angle_rad -= 2 * pi
    elif angle_rad < -pi:
        angle_rad += 2 * pi
    return angle_rad


# -----------------------------
# Clase Cinemática
# -----------------------------
class Kinematics:
    def __init__(self):
        self.left_front, self.left_back, self.right_back, self.right_front = 0, 1, 2, 3
        self.right_legs = [self.right_front, self.right_back]
        self.link_1, self.link_2, self.link_3 = 0.052, 0.118, 0.135   #0.053, 0.105, 0.132
        self.phi = radians(90)
        self.length, self.width = 0.185, 0.075
        self.leg_origins = np.array([
            [ self.length/2,  self.width/2, 0.0],
            [-self.length/2,  self.width/2, 0.0],
            [-self.length/2, -self.width/2, 0.0],
            [ self.length/2, -self.width/2, 0.0],
        ], dtype=float)
        
        # Calcular límites físicos
        self.min_reach = 0.01  # Distancia mínima alcanzable
        self.max_reach = self.link_1 + self.link_2 + self.link_3  # Distancia máxima alcanzable

    def check_physical_limits(self, xyz_local, legID=0):
        """Verificar si el punto está dentro del espacio de trabajo físico"""
        x, y, z = xyz_local
        
        # Calcular distancia desde el origen de la pierna
        distance = norm([x, y, z])
        
        # Verificar límites de distancia
        if distance < self.min_reach:
            return False, f"Pierna {legID}: Punto demasiado cercano (dist: {distance:.3f}m < min: {self.min_reach:.3f}m)"
        elif distance > self.max_reach:
            return False, f"Pierna {legID}: Punto demasiado lejano (dist: {distance:.3f}m > max: {self.max_reach:.3f}m)"
        
        # Verificar límites en el plano YZ para theta1
        if abs(y) < 1e-6 and abs(z) < 1e-6:
            return False, f"Pierna {legID}: Punto en el eje de rotación (Y,Z ≈ 0)"
        
        # Verificar límites de la cinemática inversa
        len_A = norm([0.0, y, z])
        sin_val = (sin(self.phi) * self.link_1) / len_A
        
        if abs(sin_val) > 0.9999:
            return False, f"Pierna {legID}: Límite físico alcanzado en theta1"
        
        return True, ""

    def leg_IK(self, xyz, rot=[0,0,0], legID=0, is_radians=True, body_pos=[0,0,0]):
        xyz = np.asarray(xyz, dtype=float).flatten()
        R_body = np.asarray(RotMatrix3D(rot, is_radians)).reshape(3,3)
        leg_origin = np.asarray(self.leg_origins[legID], dtype=float).flatten()
        j1_world = R_body.dot(leg_origin) + np.asarray(body_pos)
        foot_world = np.asarray(xyz)
        foot_local = R_body.T.dot(foot_world - j1_world)
        
        # Verificar límites físicos antes de calcular IK
        reachable, error_msg = self.check_physical_limits(foot_local, legID)
        if not reachable:
            # Retornar ángulos NaN para indicar error
            return [float('nan'), float('nan'), float('nan'), 
                    np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)]
        
        return self.leg_IK_calc(foot_local, is_right=(legID in self.right_legs))

    def leg_IK_calc(self, xyz, is_right=False):
        x, y, z = xyz
        
        # Mejorar el cálculo del primer ángulo
        len_A = max(norm([0.0, y, z]), 1e-8)  # Evitar división por cero
        a_1 = atan2(z, y)  # Ya usas atan2, está bien
        
        # Mejorar el cálculo de a_2 con límites más estrictos
        sin_val = (sin(self.phi) * self.link_1) / len_A
        sin_val = max(-0.9999, min(0.9999, sin_val))  # Evitar valores en los límites
        a_2 = asin(sin_val)
        
        a_3 = pi - a_2 - self.phi
        
        if is_right:
            theta_1 = a_1 - a_3
        else:
            theta_1 = a_1 + a_3
        
        # Normalizar theta_1 inmediatamente
        theta_1 = normalize_angle(theta_1)
        
        # Cálculo de juntas
        j2 = np.array([0.0, self.link_1 * cos(theta_1), self.link_1 * sin(theta_1)])
        j4 = np.array([x, y, z])
        j4_2_vec = j4 - j2
        
        R = theta_1 - self.phi - pi/2 if is_right else theta_1 + self.phi - pi/2
        R = normalize_angle(R)  # Normalizar R también
        
        rot_mtx = RotMatrix3D([-R, 0, 0], is_radians=True)
        j4_2_vec_ = rot_mtx.dot(j4_2_vec)
        x_, _, z_ = j4_2_vec_
        
        len_B = min(norm([x_, z_]), (self.link_2 + self.link_3) * 0.9999)
        
        # Verificar límites de la triangulación
        if len_B > (self.link_2 + self.link_3) * 0.999:
            return [float('nan'), float('nan'), float('nan'), 
                    np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)]
        
        # Mejorar cálculos de ángulos con atan2
        b_1 = atan2(z_, x_)  # Más robusto que point_to_rad
        
        # Limitar valores para acos
        cos_b2 = (self.link_2**2 + len_B**2 - self.link_3**2) / (2 * self.link_2 * len_B)
        cos_b2 = max(-0.9999, min(0.9999, cos_b2))
        b_2 = acos(cos_b2)
        
        cos_b3 = (self.link_2**2 + self.link_3**2 - len_B**2) / (2 * self.link_2 * self.link_3)
        cos_b3 = max(-0.9999, min(0.9999, cos_b3))
        b_3 = acos(cos_b3)
        
        theta_2 = b_1 - b_2
        theta_3 = pi - b_3
        
        # Normalizar theta_2 y theta_3
        theta_2 = normalize_angle(theta_2)
        theta_3 = normalize_angle(theta_3)
        
        # Cálculo de posiciones de juntas para visualización
        j1 = np.zeros(3)
        j3_rot = np.array([self.link_2 * cos(theta_2), 0.0, self.link_2 * sin(theta_2)])
        j3_world = j2 + np.linalg.inv(rot_mtx).dot(j3_rot)
        j4_rot = j3_rot + np.array([self.link_3 * cos(theta_2 + theta_3), 0.0, self.link_3 * sin(theta_2 + theta_3)])
        j4_world = j2 + np.linalg.inv(rot_mtx).dot(j4_rot)
        
        angles = self.angle_corrector([theta_1, theta_2, theta_3], is_right)
        return [*angles, j1, j2, j3_world.flatten(), j4_world.flatten()]
    
    def angle_corrector(self, angles, is_right=True):
        a0, a1, a2 = angles
        a1 -= 1.5*pi
        if is_right:
            t1, t2 = a0 - pi, a1 + radians(45)
        else:
            t1 = a0 - 2*pi if a0 > pi else a0
            t2 = -a1 - radians(45)
        t3 = -a2 + radians(45)
        return [t1, t2, t3]

    def compute_jacobian(self, angles, legID=0, is_right=False):
        """Calcula el Jacobiano para una pata específica"""
        theta1, theta2, theta3 = angles
        
        # Parámetros de la pata
        l1, l2, l3 = self.link_1, self.link_2, self.link_3
        phi = self.phi
        
        # Para patas derechas
        if is_right:
            # Derivadas parciales para posición del efector final
            dx_dtheta1 = -l3*cos(theta2 + theta3)*sin(theta1 - phi - pi/2) - l2*cos(theta2)*sin(theta1 - phi - pi/2)
            dx_dtheta2 = -l3*sin(theta2 + theta3)*cos(theta1 - phi - pi/2) - l2*sin(theta2)*cos(theta1 - phi - pi/2)
            dx_dtheta3 = -l3*sin(theta2 + theta3)*cos(theta1 - phi - pi/2)
            
            dy_dtheta1 = l1*cos(theta1) + l3*cos(theta2 + theta3)*cos(theta1 - phi - pi/2) + l2*cos(theta2)*cos(theta1 - phi - pi/2)
            dy_dtheta2 = -l3*sin(theta2 + theta3)*sin(theta1 - phi - pi/2) - l2*sin(theta2)*sin(theta1 - phi - pi/2)
            dy_dtheta3 = -l3*sin(theta2 + theta3)*sin(theta1 - phi - pi/2)
            
            dz_dtheta1 = l1*sin(theta1) + l3*cos(theta2 + theta3)*sin(theta1 - phi - pi/2) + l2*cos(theta2)*sin(theta1 - phi - pi/2)
            dz_dtheta2 = l3*sin(theta2 + theta3)*cos(theta1 - phi - pi/2) + l2*sin(theta2)*cos(theta1 - phi - pi/2)
            dz_dtheta3 = l3*sin(theta2 + theta3)*cos(theta1 - phi - pi/2)
        else:
            # Para patas izquierdas
            dx_dtheta1 = -l3*cos(theta2 + theta3)*sin(theta1 + phi - pi/2) - l2*cos(theta2)*sin(theta1 + phi - pi/2)
            dx_dtheta2 = -l3*sin(theta2 + theta3)*cos(theta1 + phi - pi/2) - l2*sin(theta2)*cos(theta1 + phi - pi/2)
            dx_dtheta3 = -l3*sin(theta2 + theta3)*cos(theta1 + phi - pi/2)
            
            dy_dtheta1 = l1*cos(theta1) + l3*cos(theta2 + theta3)*cos(theta1 + phi - pi/2) + l2*cos(theta2)*cos(theta1 + phi - pi/2)
            dy_dtheta2 = -l3*sin(theta2 + theta3)*sin(theta1 + phi - pi/2) - l2*sin(theta2)*sin(theta1 + phi - pi/2)
            dy_dtheta3 = -l3*sin(theta2 + theta3)*sin(theta1 + phi - pi/2)
            
            dz_dtheta1 = l1*sin(theta1) + l3*cos(theta2 + theta3)*sin(theta1 + phi - pi/2) + l2*cos(theta2)*sin(theta1 + phi - pi/2)
            dz_dtheta2 = l3*sin(theta2 + theta3)*cos(theta1 + phi - pi/2) + l2*sin(theta2)*cos(theta1 + phi - pi/2)
            dz_dtheta3 = l3*sin(theta2 + theta3)*cos(theta1 + phi - pi/2)
        
        # Construir matriz Jacobiana
        J = np.array([
            [dx_dtheta1, dx_dtheta2, dx_dtheta3],
            [dy_dtheta1, dy_dtheta2, dy_dtheta3],
            [dz_dtheta1, dz_dtheta2, dz_dtheta3]
        ])
        
        return J

    def differential_ik(self, current_angles, velocity, legID=0, dt=0.1):
        """Cinemática diferencial para mover el efector con una velocidad específica"""
        is_right = (legID in self.right_legs)
        
        # Calcular el Jacobiano en la configuración actual
        J = self.compute_jacobian(current_angles, legID, is_right)
        
        # Verificar si el Jacobiano es singular
        if np.linalg.matrix_rank(J) < 3:
            print(f"⚠️ Jacobiano singular para pata {legID}")
            return current_angles
        
        # Calcular la pseudoinversa del Jacobiano
        J_pinv = np.linalg.pinv(J)
        
        # Calcular el cambio en los ángulos de las articulaciones
        delta_theta = J_pinv @ velocity * dt
        
        # Aplicar el cambio a los ángulos actuales
        new_angles = current_angles + delta_theta
        
        return new_angles

    def plot_robot(self, ax, foot_targets, rot=[0,0,0], body_pos=[0,0,0], is_radians=False, limit=0.25):
        viewport = getattr(ax, '_robot_viewport', None)
        if viewport is not None:
            return viewport.draw(foot_targets, rot, body_pos, is_radians, limit)
        ax.clear()
        R_body = RotMatrix3D(rot, is_radians)
        base_points = (R_body.dot(self.leg_origins.T)).T + np.asarray(body_pos)

        xs, ys, zs = np.append(base_points[:,0], base_points[0,0]), np.append(base_points[:,1], base_points[0,1]), np.append(base_points[:,2], base_points[0,2])
        ax.plot3D(xs, ys, zs, 'r-', linewidth=2)
        ax.scatter(base_points[:,0], base_points[:,1], base_points[:,2], color='red', s=50)

        angles_list = []
        physical_errors = []
        
        for legID in range(4):
            angles_joints = self.leg_IK(foot_targets[legID], rot, legID, is_radians, body_pos)
            
            # Verificar si hay errores físicos (ángulos NaN)
            if any(np.isnan(angles_joints[:3])):
                physical_errors.append(legID)
                # Dibujar el punto objetivo en rojo para indicar error
                j4g = np.asarray(foot_targets[legID])
                ax.scatter(j4g[0], j4g[1], j4g[2], color='red', s=100, marker='X')
                angles_list.append([float('nan'), float('nan'), float('nan')])
                continue
                
            j1, j2, j3, j4 = angles_joints[3:]
            leg_origin = self.leg_origins[legID]
            j1g = R_body.dot(j1 + leg_origin) + body_pos
            j2g = R_body.dot(j2 + leg_origin) + body_pos
            j3g = R_body.dot(j3 + leg_origin) + body_pos
            j4g = np.asarray(foot_targets[legID])
            p = np.vstack([j1g, j2g, j3g, j4g])
            ax.plot3D(p[:,0], p[:,1], p[:,2], 'b-o', linewidth=2)
            ax.scatter(j4g[0], j4g[1], j4g[2], color='green', s=80, marker='*')
            angles_list.append(angles_joints[:3])
            
        ax.set_xlim(-limit, limit); ax.set_ylim(-limit, limit); ax.set_zlim(0, 0.25)
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
        ax.set_title("Robot SpotMini - Cinemática Inversa"); ax.grid(True)
        
        # Retornar también los errores físicos
        return angles_list, physical_errors


class Calibration:
    """Offsets, inversión y límites por servo leídos del JSON de la GUI.

    Reproduce RobotGUI.apply_calibration + get_calibrated_angles_for_esp32
    sin variables de Tk, para enviar ángulos desde scripts.
    """

    def __init__(self, servos):
        self.servos = servos  # [pata][servo] -> {'offset','invert','min','max'}

    @classmethod
    def load(cls, path=DEFAULT_CALIBRATION):
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
        return cls([leg['servos'] for leg in data['legs']])

    def to_servo_degrees(self, angles_list):
        """Ángulos de la IK (rad, 4x3) -> 12 grados para el ESP32, o (None, errores)."""
        flat, errors = [], []
        for leg, angles in enumerate(angles_list):
            for j, angle in enumerate(angles):
                cal = self.servos[leg][j]
                a = angle + radians(cal['offset'])
                if cal['invert']:
                    a = pi - a
                d = degrees(normalize_angle(a)) if math.isfinite(a) else float('nan')
                if not math.isfinite(d):
                    errors.append(f"pata {leg} servo {j}: ángulo no finito")
                elif not cal['min'] <= d <= cal['max']:
                    errors.append(f"pata {leg} servo {j}: {d:.1f}° fuera de [{cal['min']}, {cal['max']}]")
                flat.append(d)
        if len(flat) != 12:
            errors.append("se requieren 12 ángulos")
        return (None, errors) if errors else (flat, [])
