import numpy as np
import math  
from numpy.linalg import norm
from math import sin, cos, asin, acos, atan2, radians, pi, degrees
from tkinter import ttk, messagebox, simpledialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk, messagebox
import json
import os
import time
import threading
import queue
from collections import deque


# --- WebSocket Único y Compartido ---
try:
    import websocket
except ImportError:
    print("⚠️ Instala websocket-client: pip install websocket-client")
    websocket = None

ESP32_IP = "10.62.84.130"
ESP32_PORT = 81
ws = None
ws_connected = False

def connect_ws():
    global ws, ws_connected
    if websocket is None:
        return False
    
    try:
        # Primero desconectar si ya hay una conexión
        if ws is not None:
            try:
                ws.close()
            except:
                pass
        
        ws = websocket.WebSocket()
        ws.connect(f"ws://{ESP32_IP}:{ESP32_PORT}")
        ws_connected = True
        print(f"✅ Conectado a {ESP32_IP}")
        
        return True
    except Exception as e:
        print(f"⚠️ Error conexión: {e}")
        ws = None
        ws_connected = False
        return False

def disconnect_ws():
    global ws, ws_connected
    try:
        if ws:
            ws.close()
    except:
        pass
    ws = None
    ws_connected = False
    print("🔌 WebSocket desconectado")
    
    # Detener también el listener del MPU
    mpu_handler.stop_mpu_listener()

def send_angles(angle_list):
    global ws, ws_connected
    if angle_list is None:
        return False
    if len(angle_list) != 12 or any(not math.isfinite(a) or not 0 <= a <= 180 for a in angle_list):
        return False
    if not send_to_esp32_flag.get():
        return
    
    if not ws_connected or ws is None:
        if not connect_ws():
            return False
    
    try:
        start_time = time.time()
        ws.send(json.dumps(angle_list))
        end_time = time.time()
        latency_ms = (end_time - start_time) * 1000
        print(f"📤 Ángulos enviados: {angle_list}")
        print(f"⏱️ Latencia envío: {latency_ms:.2f} ms")
        return True
    except Exception as e:
        print("⚠️ Error envío:", e)
        ws_connected = False
        ws = None
        return False

def send_command(command):
    """Enviar comando al ESP32 usando la conexión compartida"""
    global ws, ws_connected
    if not send_to_esp32_flag.get():
        return False

    if not ws_connected or ws is None:
        if not connect_ws():
            return False

    try:
        mpu_handler.start_mpu_listener(request_data=False)
        ws.send(command)
        print(f"📤 Comando enviado: {command}")
        return True
    except Exception as e:
        print(f"⚠️ Error enviando comando: {e}")
        ws_connected = False
        ws = None
        return False

HEARTBEAT_MS = 200  # el ESP32 suelta los servos tras 1 s sin mensajes (FAILSAFE_TIMEOUT_MS)

def send_heartbeat():
    """Mantener vivo el fail-safe del ESP32 aunque la pose no cambie. No reconecta."""
    global ws, ws_connected
    if not send_to_esp32_flag.get() or not ws_connected or ws is None:
        return False
    try:
        ws.send("cmd:ping")
        return True
    except Exception as e:
        print(f"⚠️ Error enviando heartbeat: {e}")
        ws_connected = False
        ws = None
        return False

# Cinemática y marchas viven en kinematics.py y gait.py
from kinematics import RotMatrix3D, point_to_rad, normalize_angle, Kinematics
from gait import GaitManager

# --- Clase para manejar datos del MPU-6050 ---
class MPU6050Handler:
    def __init__(self):
        self.accel_data = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.gyro_data = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.orientation = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0} 
        self.temperature = 0.0
        self.data_queue = queue.Queue()
        self.response_queue = queue.Queue()
        self.is_running = False
        self.thread = None
        self.data_sending_enabled = False  # ✅ NUEVO: Controlar envío de datos

    def start_mpu_listener(self, request_data=True):
        """Iniciar hilo para recibir datos del MPU-6050 usando conexión compartida"""
        if self.is_running:
            self.data_sending_enabled = self.data_sending_enabled or request_data
            return
            
        self.is_running = True
        self.data_sending_enabled = request_data
        self.thread = threading.Thread(target=self._mpu_listener_thread, daemon=True)
        self.thread.start()
        print("✅ Receptor MPU-6050 iniciado")
        
    def stop_mpu_listener(self):
        """Detener hilo del MPU-6050"""
        self.is_running = False
        self.data_sending_enabled = False  # ✅ Deshabilitar envío de datos
        if self.thread:
            self.thread.join(timeout=1.0)
        print("🛑 Listener MPU-6050 detenido - Envío de datos DESACTIVADO")

    def _mpu_listener_thread(self):
        """Hilo para recibir datos del sensor usando conexión compartida"""
        last_request_time = 0
        request_interval = 0.1  # 100ms (10Hz)
        
        while self.is_running:
            try:
                # Verificar que tenemos conexión WebSocket
                if not ws_connected or ws is None:
                    time.sleep(0.5)
                    continue
                
                # ✅ SOLO solicitar datos si el envío está habilitado
                if self.data_sending_enabled:
                    current_time = time.time()
                    if current_time - last_request_time >= request_interval:
                        self.request_mpu_data()
                        last_request_time = current_time
                
                # Recibir datos (con timeout corto)
                ws.settimeout(0.1)  # Timeout corto para no bloquear
                try:
                    data = ws.recv()
                    if data:
                        self._process_mpu_data(data)
                except websocket.WebSocketTimeoutException:
                    pass  # Timeout normal, continuar
                except Exception as e:
                    print(f"⚠️ Error recibiendo datos MPU: {e}")
                    break
                        
            except Exception as e:
                print(f"⚠️ Error en listener MPU: {e}")
                time.sleep(0.5)

        self.is_running = False

    def request_mpu_data(self):
        """Solicitar datos del MPU-6050 al ESP32 usando conexión compartida"""
        global ws, ws_connected
        if ws_connected and ws and self.data_sending_enabled:  # ✅ Solo si está habilitado
            try:
                request_msg = json.dumps({"command": "get_mpu_data"})
                ws.send(request_msg)
            except Exception as e:
                print(f"⚠️ Error solicitando datos MPU: {e}")

    def get_latest_data(self):
        """Obtener los datos más recientes del MPU"""
        try:
            # Vaciar la cola y quedarse con el último elemento
            latest_data = None
            while not self.data_queue.empty():
                latest_data = self.data_queue.get_nowait()
            return latest_data
        except:
            return None

    def _process_mpu_data(self, data_str):
        """Procesar datos JSON del MPU-6050 incluyendo YAW"""
        try:
            data = json.loads(data_str)
            if 'status' in data or 'accel_offsets' in data or 'gyro_offsets' in data:
                self.response_queue.put(data)
                return
            
            # Si son datos del sensor MPU-6050
            if 'sensor' in data and data['sensor'] == 'mpu6050':
                # Actualizar datos del acelerómetro
                if 'accel' in data:
                    self.accel_data = {
                        'x': data['accel']['x'],
                        'y': data['accel']['y'], 
                        'z': data['accel']['z']
                    }
                    
                # Actualizar datos del giroscopio
                if 'gyro' in data:
                    self.gyro_data = {
                        'x': data['gyro']['x'],
                        'y': data['gyro']['y'],
                        'z': data['gyro']['z']
                    }
                    
                #  ACTUALIZAR ORIENTACIÓN INCLUYENDO YAW
                if 'orientation' in data:
                    self.orientation = {
                        'roll': data['orientation']['roll'],
                        'pitch': data['orientation']['pitch'],
                        'yaw': data['orientation']['yaw']  # ✅ Yaw recibido del ESP32
                    }
                    
                # Actualizar temperatura
                if 'temp' in data:
                    self.temperature = data['temp']
                    
                # Poner en cola para actualizar GUI
                self.data_queue.put({
                    'accelerometer': self.accel_data.copy(),
                    'gyroscope': self.gyro_data.copy(),
                    'orientation': self.orientation.copy(),  # ✅ Incluye yaw
                    'temperature': self.temperature
                })
                
        except Exception as e:
            print(f"⚠️ Error procesando datos MPU: {e}")  
# Crear instancia global del manejador MPU
mpu_handler = MPU6050Handler()


    # -----------------------------
    # GUI Principal con Pestañas
    # -----------------------------
class RobotGUI:
    # -----------------------------
    # Setup and GUI elements
    # -----------------------------
    def __init__(self, root):
        self.root = root
        self.root.title("Spot Micro Twins - Control del Robot")
        self.root.geometry("1370x765")
        self.kin = Kinematics()
        self.leg_vars, self.platform_vars = [], {}
        self.calibration_vars = []
        self.limits_loaded = False
        self.limit_errors = set()
        self.routine_steps = []
        self.current_step_index = -1
        self.differential_routine_steps = []
        self.diff_current_step_index = -1
        self.diff_platform_labels = {}

        # Variables para control de velocidad
        self.step_delay_ms = tk.IntVar(value=1000)
        self.intermediate_steps = tk.IntVar(value=5)
        self.global_speed = tk.DoubleVar(value=1.0)
        
        # Variables para cinemática diferencial
        self.differential_leg_selected = tk.IntVar(value=0)
        self.velocity_step = tk.DoubleVar(value=0.005)
        self.current_leg_positions = [
            [0.10, 0.1, 0],
            [-0.10, 0.1, 0],
            [-0.10, -0.1, 0],
            [0.10, -0.1, 0],
        ]
        
        self.diagonal_legs_enabled = tk.BooleanVar(value=False)
        
        global send_to_esp32_flag
        send_to_esp32_flag = tk.BooleanVar(value=False)
        self.heartbeat_loop()

        # Variables para almacenar valores de todos los modos
        self.joystick_values = {
            "yaw_height": {
                "yaw": 0.0,
                "height": 0.18
            },
            "roll_pitch": {
                "roll": 0.0,
                "pitch": 0.0
            },
            "xy_position": {
                "x": 0.0,
                "y": 0.0
            }
        }
        
        # Inicializar variables del joystick
        self.joystick_yaw = 0.0
        self.joystick_height = 0.18
        self.joystick_x = 0.0
        self.joystick_y = 0.0
        self.joystick_roll = tk.DoubleVar(value=0.0)
        self.joystick_pitch = tk.DoubleVar(value=0.0)
        self.joystick_sensitivity = tk.DoubleVar(value=1.0)
        self.joystick_yaw = 0.0
        self.joystick_mode = tk.StringVar(value="yaw_height")

        # Inicializar el gestor de gaits
        self.gait_manager = GaitManager(self.kin)
        
        # Variables para control de gaits
        self.active_gait_var = tk.StringVar(value="standing")
        self.gait_direction_x = tk.DoubleVar(value=0.0)
        self.gait_direction_y = tk.DoubleVar(value=0.0)
        self.gait_turn = tk.DoubleVar(value=0.0)
        self.gait_speed_var = tk.DoubleVar(value=1.0)

        # =============================================================================
        # MOVER LA INICIALIZACIÓN DE MPU_VARS AQUÍ, ANTES DE setup_gui()
        # =============================================================================
        
        # Variables para datos MPU-6050
        self.mpu_vars = {
            'accelerometer': {
                'x': tk.StringVar(value="0.000"),
                'y': tk.StringVar(value="0.000"), 
                'z': tk.StringVar(value="0.000")
            },
            'gyroscope': {
                'x': tk.StringVar(value="0.000"),
                'y': tk.StringVar(value="0.000"),
                'z': tk.StringVar(value="0.000")
            },
            'orientation': {
                'roll': tk.StringVar(value="0.000"),
                'pitch': tk.StringVar(value="0.000"),
                'yaw': tk.StringVar(value="0.000")
            },
            'temperature': tk.StringVar(value="0.0")
        }
        
        # Variables para control PID
        self.pid_vars = {
            'enabled': tk.BooleanVar(value=False),
            # Roll
            'kp_roll': tk.DoubleVar(value=1.0),
            'ki_roll': tk.DoubleVar(value=0.0),
            'kd_roll': tk.DoubleVar(value=0.1),
            'setpoint_roll': tk.DoubleVar(value=0.0),
            'output_roll': tk.StringVar(value="0.0"),
            # Pitch
            'kp_pitch': tk.DoubleVar(value=1.0),
            'ki_pitch': tk.DoubleVar(value=0.0),
            'kd_pitch': tk.DoubleVar(value=0.1),
            'setpoint_pitch': tk.DoubleVar(value=0.0),
            'output_pitch': tk.StringVar(value="0.0"),
            # Yaw
            'kp_yaw': tk.DoubleVar(value=0.5),
            'ki_yaw': tk.DoubleVar(value=0.0),
            'kd_yaw': tk.DoubleVar(value=0.05),
            'setpoint_yaw': tk.DoubleVar(value=0.0),
            'output_yaw': tk.StringVar(value="0.0"),
            'enable_yaw_pid': tk.BooleanVar(value=False)
        }
        
        # Variables para el cálculo PID
        self.pid_integrals = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.pid_previous_errors = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.last_pid_time = time.time()

        # =============================================================================
        # AHORA SÍ LLAMAR A setup_gui()
        # =============================================================================
        self.mpu_status_var = tk.StringVar(value="Sensor: Inactivo")
        self.setup_gui()
        
        # Iniciar actualización de datos MPU
       
        self.start_mpu_updates()
        
        
        
        # Variables específicas para ejecución
        self.routine_executing = False
        self.differential_routine_executing = False
        self.current_routine_thread = None
        self.stop_routine_flag = False

        # Variables específicas para diferencial
        self.diff_routine_executing = False
        self.diff_stop_routine_flag = False
        self.current_diff_routine_thread = None
 
    def setup_gui(self):
        # Crear notebook para pestañas
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Pestaña de cinemática inversa
        self.inverse_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.inverse_tab, text="Cinemática Inversa")
        
        # Pestaña de cinemática diferencial
        self.differential_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.differential_tab, text="Cinemática Diferencial")
        
        self.setup_inverse_kinematics_tab()
        self.setup_differential_kinematics_tab()

    def setup_global_speed_control(self, parent):
        """Configurar control de velocidad global"""
        speed_frame = ttk.LabelFrame(parent, text="Control de Velocidad Global")
        speed_frame.pack(fill=tk.X, pady=5)
        
        # Frame para slider y valor
        slider_frame = ttk.Frame(speed_frame)
        slider_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Etiqueta de velocidad mínima
        ttk.Label(slider_frame, text="Lento", font=('Arial', 8)).pack(side=tk.LEFT)
        
        # Slider de velocidad
        speed_slider = ttk.Scale(slider_frame, from_=0.1, to=3.0, 
                                orient=tk.HORIZONTAL, variable=self.global_speed,
                                command=self.on_global_speed_change)
        speed_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # Etiqueta de velocidad máxima
        ttk.Label(slider_frame, text="Rápido", font=('Arial', 8)).pack(side=tk.LEFT)
        
        # Frame para mostrar valor numérico
        value_frame = ttk.Frame(speed_frame)
        value_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Label(value_frame, text="Velocidad:", font=('Arial', 9)).pack(side=tk.LEFT)
        self.speed_value_label = ttk.Label(value_frame, text="1.0x", 
                                        font=('Arial', 9, 'bold'), foreground="blue")
        self.speed_value_label.pack(side=tk.LEFT, padx=5)
        
        # Frame para botones de velocidad predefinida
        preset_frame = ttk.Frame(speed_frame)
        preset_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Button(preset_frame, text="0.5x", width=5,
                command=lambda: self.set_speed_preset(0.5)).pack(side=tk.LEFT, padx=2)
        ttk.Button(preset_frame, text="1.0x", width=5,
                command=lambda: self.set_speed_preset(1.0)).pack(side=tk.LEFT, padx=2)
        ttk.Button(preset_frame, text="1.5x", width=5,
                command=lambda: self.set_speed_preset(1.5)).pack(side=tk.LEFT, padx=2)
        ttk.Button(preset_frame, text="2.0x", width=5,
                command=lambda: self.set_speed_preset(2.0)).pack(side=tk.LEFT, padx=2)

    def on_global_speed_change(self, value):
        """Actualizar cuando cambia la velocidad global"""
        speed = float(value)
        self.speed_value_label.config(text=f"{speed:.1f}x")
        
        # Actualizar velocidad en cinemática diferencial también
        self.update_differential_speed()

    def set_speed_preset(self, speed):
        """Establecer velocidad predefinida"""
        self.global_speed.set(speed)
        self.speed_value_label.config(text=f"{speed:.1f}x")
        self.update_differential_speed()

    def setup_differential_kinematics_tab(self):
        main_frame = ttk.Frame(self.differential_tab)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Frame izquierdo para controles
        left_frame = ttk.Frame(main_frame, width=500)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        left_frame.pack_propagate(False)
        
        # Frame central para visualización
        center_frame = ttk.Frame(main_frame)
        center_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Frame derecho para rutinas
        right_frame = ttk.Frame(main_frame, width=450)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=5)
        right_frame.pack_propagate(False)
        
        # Configurar controles de cinemática diferencial
        self.setup_differential_controls(left_frame)
        
        # Configurar visualización (gráfico y tabla de ángulos)
        self.setup_differential_visualization_frame(center_frame)
        
        # Configurar rutinas (igual que en cinemática inversa)
        self.setup_differential_routine_controls(right_frame)
        
        # Inicializar visualización
        self.update_differential_visualization()

    def setup_inverse_kinematics_tab(self):
        main_frame = ttk.Frame(self.inverse_tab)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Crear frame con scrollbar para los controles izquierdos
        left_container = ttk.Frame(main_frame, width=650)
        left_container.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        left_container.pack_propagate(False)
        
        # Canvas y scrollbar para los controles
        self.control_canvas = tk.Canvas(left_container, width=380, height=650)
        scrollbar = ttk.Scrollbar(left_container, orient="vertical", command=self.control_canvas.yview)
        self.scrollable_frame = ttk.Frame(self.control_canvas)
        
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.control_canvas.configure(scrollregion=self.control_canvas.bbox("all"))
        )
        
        self.control_canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.control_canvas.configure(yscrollcommand=scrollbar.set)
        
        self.control_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Bind mouse wheel to canvas
        self.control_canvas.bind("<MouseWheel>", self._on_mousewheel)
        
        # Control de velocidad global (al inicio de los controles)
        self.setup_global_speed_control(self.scrollable_frame)
        
        # Frame central para visualización y rutinas
        center_frame = ttk.Frame(main_frame)
        center_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Frame para visualización (gráfico y tabla de ángulos)
        viz_frame = ttk.Frame(center_frame, width=400)
        viz_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Frame para rutinas (a la derecha del gráfico)
        routine_frame = ttk.Frame(center_frame, width=450)
        routine_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=5)
        routine_frame.pack_propagate(False)
        
        self.setup_plot(viz_frame)
        self.setup_angle_display(viz_frame)
        self.setup_routine_controls(routine_frame)
        
        # =============================================================================
        # CONTROLES PRINCIPALES EN EL PANEL IZQUIERDO (SCROLLABLE)
        # =============================================================================
        
        # 1. Controles de patas individuales
        self.setup_leg_controls(self.scrollable_frame)
        
        # 2. Controles de plataforma (cuerpo)
        self.setup_platform_controls(self.scrollable_frame)
        
        # 3. Sistema de Gaits
        self.setup_gait_controls(self.scrollable_frame)
        
        # 4. Calibración de servos
        self.setup_calibration_controls(self.scrollable_frame)
        
        # 5. Conexión ESP32 y WebSocket
        self.setup_websocket_controls(self.scrollable_frame)
        
        # 6. Sensor MPU-6050 y Control PID
        self.setup_mpu_display()
        
        # Botón de actualización
        ttk.Button(self.scrollable_frame, text="Actualizar Visualización", 
                command=self.update_visualization).pack(pady=10)
        
        # Inicializar visualización
        self.update_visualization()

    def setup_websocket_controls(self, parent):
        """Checkbox para controlar el envío WebSocket"""
        ws_frame = ttk.LabelFrame(parent, text="Conexión ESP32")
        ws_frame.pack(fill=tk.X, pady=5)
        
        # Checkbox para activar/desactivar envío
        ttk.Checkbutton(ws_frame, text="Enviar ángulos al ESP32", 
                    variable=send_to_esp32_flag).pack(pady=5)
        
        # Frame para botones de conexión/desconexión
        button_frame = ttk.Frame(ws_frame)
        button_frame.pack(fill=tk.X, pady=2)
        
        # Botón para conectar manualmente
        ttk.Button(button_frame, text="Conectar WebSocket", 
                command=self.connect_websocket).pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        
        # ✅ NUEVO: Botón para desconectar
        ttk.Button(button_frame, text="Desconectar WebSocket", 
                command=self.disconnect_websocket).pack(side=tk.RIGHT, padx=2, fill=tk.X, expand=True)
        
        # Estado de conexión
        self.ws_status_label = ttk.Label(ws_frame, text="Estado: No conectado", 
                                        foreground="red")
        self.ws_status_label.pack(pady=2)

    def connect_websocket(self):
        if connect_ws():  # Esta función ahora maneja ambas conexiones
            self.ws_status_label.config(text="Estado: Conectado", foreground="green")
            
        else:
            self.ws_status_label.config(text="Estado: Error conexión", foreground="red")

    def disconnect_websocket(self):
        """Desconectar WebSocket y detener sensor MPU"""
        disconnect_ws()  # Llama a la función global de desconexión
        self.ws_status_label.config(text="Estado: Desconectado", foreground="red")
        
        # ✅ CAMBIO: Sincronizar estado del sensor al desconectar
        self.mpu_status_var.set("Sensor: Inactivo")
        self.mpu_status_label.config(foreground="red")
        
        print("🔌 WebSocket desconectado manualmente")

    def update_websocket_status(self):
        """Actualizar el label de estado de WebSocket"""
        global ws_connected
        if ws_connected:
            self.ws_status_label.config(text="Estado: Conectado", foreground="green")
        else:
            self.ws_status_label.config(text="Estado: Desconectado", foreground="red")
    
        
    # -----------------------------
    # Metodos para Mpu
    # -----------------------------
    
    def setup_mpu_display(self):
        """Configurar display para datos del MPU-6050 y control PID CON INPUTBOXES"""
        # Encontrar el frame de conexión ESP32 (debe existir en tu GUI)
        ws_frame = None
        for widget in self.scrollable_frame.winfo_children():
            if hasattr(widget, 'winfo_class') and 'labelframe' in widget.winfo_class().lower():
                if "ESP32" in widget.cget('text'):
                    ws_frame = widget
                    break
        
        if ws_frame:
            # Frame para datos MPU-6050
            mpu_frame = ttk.LabelFrame(ws_frame, text="Sensor MPU-6050")
            mpu_frame.pack(fill=tk.X, pady=5, padx=5)
            
            # Botones de control MPU
            control_frame = ttk.Frame(mpu_frame)
            control_frame.pack(fill=tk.X, pady=2)
            
            ttk.Button(control_frame, text="Iniciar Sensor", 
                    command=self.start_mpu_sensor).pack(side=tk.LEFT, padx=2)
            ttk.Button(control_frame, text="Detener Sensor", 
                    command=self.stop_mpu_sensor).pack(side=tk.LEFT, padx=2)
            
            # Estado del sensor CON COLOR
            self.mpu_status_label = ttk.Label(control_frame, textvariable=self.mpu_status_var, 
                                            foreground="red")
            self.mpu_status_label.pack(side=tk.RIGHT, padx=5)
            
            # Frame para datos del acelerómetro
            accel_frame = ttk.LabelFrame(mpu_frame, text="Acelerómetro (g)")
            accel_frame.pack(fill=tk.X, pady=2, padx=5)
            
            for axis in ['x', 'y', 'z']:
                axis_frame = ttk.Frame(accel_frame)
                axis_frame.pack(fill=tk.X, pady=1)
                ttk.Label(axis_frame, text=f"{axis.upper()}:", width=4).pack(side=tk.LEFT)
                ttk.Label(axis_frame, textvariable=self.mpu_vars['accelerometer'][axis],
                        width=8, font=('Arial', 9, 'bold')).pack(side=tk.LEFT)
            
            # Frame para datos del giroscopio
            gyro_frame = ttk.LabelFrame(mpu_frame, text="Giroscopio (°/s)")
            gyro_frame.pack(fill=tk.X, pady=2, padx=5)
            
            for axis in ['x', 'y', 'z']:
                axis_frame = ttk.Frame(gyro_frame)
                axis_frame.pack(fill=tk.X, pady=1)
                ttk.Label(axis_frame, text=f"{axis.upper()}:", width=4).pack(side=tk.LEFT)
                ttk.Label(axis_frame, textvariable=self.mpu_vars['gyroscope'][axis],
                        width=8, font=('Arial', 9, 'bold')).pack(side=tk.LEFT)
            
            # Frame para orientación
            orient_frame = ttk.LabelFrame(mpu_frame, text="Orientación (°)")
            orient_frame.pack(fill=tk.X, pady=2, padx=5)
            
            for angle in ['roll', 'pitch', 'yaw']:
                angle_frame = ttk.Frame(orient_frame)
                angle_frame.pack(fill=tk.X, pady=1)
                ttk.Label(angle_frame, text=f"{angle.title()}:", width=8).pack(side=tk.LEFT)
                ttk.Label(angle_frame, textvariable=self.mpu_vars['orientation'][angle],
                        width=8, font=('Arial', 9, 'bold')).pack(side=tk.LEFT)
            
            # Temperatura
            temp_frame = ttk.Frame(mpu_frame)
            temp_frame.pack(fill=tk.X, pady=2, padx=5)
            ttk.Label(temp_frame, text="Temperatura:").pack(side=tk.LEFT)
            ttk.Label(temp_frame, textvariable=self.mpu_vars['temperature'],
                    width=6, font=('Arial', 9, 'bold')).pack(side=tk.LEFT)
            ttk.Label(temp_frame, text="°C").pack(side=tk.LEFT)

            # NUEVO: Frame para comandos del IMU
            imu_commands_frame = ttk.LabelFrame(ws_frame, text="Comandos IMU")
            imu_commands_frame.pack(fill=tk.X, pady=5, padx=5)
            
            # Botones de calibración
            cal_buttons_frame = ttk.Frame(imu_commands_frame)
            cal_buttons_frame.pack(fill=tk.X, pady=2)
            
            ttk.Button(cal_buttons_frame, text="🔄 Calibrar IMU", 
                    command=self.calibrate_imu).pack(side=tk.LEFT, padx=2)
            
            ttk.Button(cal_buttons_frame, text="🔄 Resetear Orientación", 
                    command=self.reset_imu).pack(side=tk.LEFT, padx=2)
            
            ttk.Button(cal_buttons_frame, text="📊 Obtener Offsets", 
                    command=self.get_imu_offsets).pack(side=tk.LEFT, padx=2)
            
            # Frame para offsets manuales
            offsets_frame = ttk.LabelFrame(imu_commands_frame, text="Offsets Manuales")
            offsets_frame.pack(fill=tk.X, pady=2)
            
            # Entradas para offsets
            self.offset_vars = {}
            for i, sensor in enumerate(['Acelerómetro', 'Giroscopio']):
                sensor_frame = ttk.Frame(offsets_frame)
                sensor_frame.pack(fill=tk.X, pady=1)
                
                ttk.Label(sensor_frame, text=f"{sensor}:", width=12).pack(side=tk.LEFT)
                
                for axis in ['X', 'Y', 'Z']:
                    ttk.Label(sensor_frame, text=axis, width=2).pack(side=tk.LEFT)
                    var = tk.DoubleVar(value=0.0)
                    entry = ttk.Entry(sensor_frame, width=6, textvariable=var, 
                                    font=('Arial', 8))
                    entry.pack(side=tk.LEFT, padx=1)
                    self.offset_vars[f'{sensor}_{axis}'.lower()] = var
            
            ttk.Button(offsets_frame, text="💾 Aplicar Offsets", 
                    command=self.set_imu_offsets).pack(pady=2)


            # MODIFICADO: Frame para control PID de Estabilidad CON INPUTBOXES
            pid_frame = ttk.LabelFrame(ws_frame, text="Control PID de Estabilidad")
            pid_frame.pack(fill=tk.X, pady=5, padx=5)
            
            # Checkbox para activar/desactivar PID general
            enable_frame = ttk.Frame(pid_frame)
            enable_frame.pack(fill=tk.X, pady=2)
            ttk.Checkbutton(enable_frame, text="Activar Control PID", 
                        variable=self.pid_vars['enabled'],
                        command=self.on_pid_toggle).pack(side=tk.LEFT)
            
            ttk.Checkbutton(enable_frame, text="Incluir Yaw", 
                        variable=self.pid_vars['enable_yaw_pid']).pack(side=tk.RIGHT)
            
            # MODIFICADO: Frame para ganancias Roll CON INPUTBOXES
            roll_frame = ttk.LabelFrame(pid_frame, text="Roll PID - Estabilidad")
            roll_frame.pack(fill=tk.X, pady=2, padx=5)
            
            # Kp Roll - CAMBIADO A ENTRY
            kp_roll_frame = ttk.Frame(roll_frame)
            kp_roll_frame.pack(fill=tk.X, pady=1)
            ttk.Label(kp_roll_frame, text="Kp:", width=4).pack(side=tk.LEFT)
            kp_roll_entry = ttk.Entry(kp_roll_frame, width=8, font=('Arial', 9), justify='center')
            kp_roll_entry.insert(0, "1.0")
            kp_roll_entry.pack(side=tk.LEFT, padx=2)
            kp_roll_entry.bind('<Return>', lambda e: self.on_pid_entry_change('kp_roll', kp_roll_entry))
            kp_roll_entry.bind('<FocusOut>', lambda e: self.on_pid_entry_change('kp_roll', kp_roll_entry))
            
            # Ki Roll - CAMBIADO A ENTRY
            ki_roll_frame = ttk.Frame(roll_frame)
            ki_roll_frame.pack(fill=tk.X, pady=1)
            ttk.Label(ki_roll_frame, text="Ki:", width=4).pack(side=tk.LEFT)
            ki_roll_entry = ttk.Entry(ki_roll_frame, width=8, font=('Arial', 9), justify='center')
            ki_roll_entry.insert(0, "0.0")
            ki_roll_entry.pack(side=tk.LEFT, padx=2)
            ki_roll_entry.bind('<Return>', lambda e: self.on_pid_entry_change('ki_roll', ki_roll_entry))
            ki_roll_entry.bind('<FocusOut>', lambda e: self.on_pid_entry_change('ki_roll', ki_roll_entry))
            
            # Kd Roll - CAMBIADO A ENTRY
            kd_roll_frame = ttk.Frame(roll_frame)
            kd_roll_frame.pack(fill=tk.X, pady=1)
            ttk.Label(kd_roll_frame, text="Kd:", width=4).pack(side=tk.LEFT)
            kd_roll_entry = ttk.Entry(kd_roll_frame, width=8, font=('Arial', 9), justify='center')
            kd_roll_entry.insert(0, "0.1")
            kd_roll_entry.pack(side=tk.LEFT, padx=2)
            kd_roll_entry.bind('<Return>', lambda e: self.on_pid_entry_change('kd_roll', kd_roll_entry))
            kd_roll_entry.bind('<FocusOut>', lambda e: self.on_pid_entry_change('kd_roll', kd_roll_entry))
            
            # Setpoint Roll - CAMBIADO A ENTRY
            sp_roll_frame = ttk.Frame(roll_frame)
            sp_roll_frame.pack(fill=tk.X, pady=1)
            ttk.Label(sp_roll_frame, text="Setpoint:", width=8).pack(side=tk.LEFT)
            sp_roll_entry = ttk.Entry(sp_roll_frame, width=8, font=('Arial', 9), justify='center')
            sp_roll_entry.insert(0, "0.0")
            sp_roll_entry.pack(side=tk.LEFT, padx=2)
            sp_roll_entry.bind('<Return>', lambda e: self.on_pid_entry_change('setpoint_roll', sp_roll_entry))
            sp_roll_entry.bind('<FocusOut>', lambda e: self.on_pid_entry_change('setpoint_roll', sp_roll_entry))
            
            # Output Roll (solo lectura)
            out_roll_frame = ttk.Frame(roll_frame)
            out_roll_frame.pack(fill=tk.X, pady=1)
            ttk.Label(out_roll_frame, text="Output:", width=8).pack(side=tk.LEFT)
            ttk.Label(out_roll_frame, textvariable=self.pid_vars['output_roll'], 
                    width=8, font=('Arial', 9, 'bold')).pack(side=tk.RIGHT)
            
            # MODIFICADO: Frame para ganancias Pitch CON INPUTBOXES
            pitch_frame = ttk.LabelFrame(pid_frame, text="Pitch PID - Estabilidad")
            pitch_frame.pack(fill=tk.X, pady=2, padx=5)
            
            # Kp Pitch - CAMBIADO A ENTRY
            kp_pitch_frame = ttk.Frame(pitch_frame)
            kp_pitch_frame.pack(fill=tk.X, pady=1)
            ttk.Label(kp_pitch_frame, text="Kp:", width=4).pack(side=tk.LEFT)
            kp_pitch_entry = ttk.Entry(kp_pitch_frame, width=8, font=('Arial', 9), justify='center')
            kp_pitch_entry.insert(0, "1.0")
            kp_pitch_entry.pack(side=tk.LEFT, padx=2)
            kp_pitch_entry.bind('<Return>', lambda e: self.on_pid_entry_change('kp_pitch', kp_pitch_entry))
            kp_pitch_entry.bind('<FocusOut>', lambda e: self.on_pid_entry_change('kp_pitch', kp_pitch_entry))
            
            # Ki Pitch - CAMBIADO A ENTRY
            ki_pitch_frame = ttk.Frame(pitch_frame)
            ki_pitch_frame.pack(fill=tk.X, pady=1)
            ttk.Label(ki_pitch_frame, text="Ki:", width=4).pack(side=tk.LEFT)
            ki_pitch_entry = ttk.Entry(ki_pitch_frame, width=8, font=('Arial', 9), justify='center')
            ki_pitch_entry.insert(0, "0.0")
            ki_pitch_entry.pack(side=tk.LEFT, padx=2)
            ki_pitch_entry.bind('<Return>', lambda e: self.on_pid_entry_change('ki_pitch', ki_pitch_entry))
            ki_pitch_entry.bind('<FocusOut>', lambda e: self.on_pid_entry_change('ki_pitch', ki_pitch_entry))
            
            # Kd Pitch - CAMBIADO A ENTRY
            kd_pitch_frame = ttk.Frame(pitch_frame)
            kd_pitch_frame.pack(fill=tk.X, pady=1)
            ttk.Label(kd_pitch_frame, text="Kd:", width=4).pack(side=tk.LEFT)
            kd_pitch_entry = ttk.Entry(kd_pitch_frame, width=8, font=('Arial', 9), justify='center')
            kd_pitch_entry.insert(0, "0.1")
            kd_pitch_entry.pack(side=tk.LEFT, padx=2)
            kd_pitch_entry.bind('<Return>', lambda e: self.on_pid_entry_change('kd_pitch', kd_pitch_entry))
            kd_pitch_entry.bind('<FocusOut>', lambda e: self.on_pid_entry_change('kd_pitch', kd_pitch_entry))
            
            # Setpoint Pitch - CAMBIADO A ENTRY
            sp_pitch_frame = ttk.Frame(pitch_frame)
            sp_pitch_frame.pack(fill=tk.X, pady=1)
            ttk.Label(sp_pitch_frame, text="Setpoint:", width=8).pack(side=tk.LEFT)
            sp_pitch_entry = ttk.Entry(sp_pitch_frame, width=8, font=('Arial', 9), justify='center')
            sp_pitch_entry.insert(0, "0.0")
            sp_pitch_entry.pack(side=tk.LEFT, padx=2)
            sp_pitch_entry.bind('<Return>', lambda e: self.on_pid_entry_change('setpoint_pitch', sp_pitch_entry))
            sp_pitch_entry.bind('<FocusOut>', lambda e: self.on_pid_entry_change('setpoint_pitch', sp_pitch_entry))
            
            # Output Pitch (solo lectura)
            out_pitch_frame = ttk.Frame(pitch_frame)
            out_pitch_frame.pack(fill=tk.X, pady=1)
            ttk.Label(out_pitch_frame, text="Output:", width=8).pack(side=tk.LEFT)
            ttk.Label(out_pitch_frame, textvariable=self.pid_vars['output_pitch'], 
                    width=8, font=('Arial', 9, 'bold')).pack(side=tk.RIGHT)
            
            # MODIFICADO: Frame para ganancias Yaw CON INPUTBOXES
            yaw_frame = ttk.LabelFrame(pid_frame, text="Yaw PID - Orientación")
            yaw_frame.pack(fill=tk.X, pady=2, padx=5)
            
            # Kp Yaw - CAMBIADO A ENTRY
            kp_yaw_frame = ttk.Frame(yaw_frame)
            kp_yaw_frame.pack(fill=tk.X, pady=1)
            ttk.Label(kp_yaw_frame, text="Kp:", width=4).pack(side=tk.LEFT)
            kp_yaw_entry = ttk.Entry(kp_yaw_frame, width=8, font=('Arial', 9), justify='center')
            kp_yaw_entry.insert(0, "0.5")
            kp_yaw_entry.pack(side=tk.LEFT, padx=2)
            kp_yaw_entry.bind('<Return>', lambda e: self.on_pid_entry_change('kp_yaw', kp_yaw_entry))
            kp_yaw_entry.bind('<FocusOut>', lambda e: self.on_pid_entry_change('kp_yaw', kp_yaw_entry))
            
            # Ki Yaw - CAMBIADO A ENTRY
            ki_yaw_frame = ttk.Frame(yaw_frame)
            ki_yaw_frame.pack(fill=tk.X, pady=1)
            ttk.Label(ki_yaw_frame, text="Ki:", width=4).pack(side=tk.LEFT)
            ki_yaw_entry = ttk.Entry(ki_yaw_frame, width=8, font=('Arial', 9), justify='center')
            ki_yaw_entry.insert(0, "0.0")
            ki_yaw_entry.pack(side=tk.LEFT, padx=2)
            ki_yaw_entry.bind('<Return>', lambda e: self.on_pid_entry_change('ki_yaw', ki_yaw_entry))
            ki_yaw_entry.bind('<FocusOut>', lambda e: self.on_pid_entry_change('ki_yaw', ki_yaw_entry))
            
            # Kd Yaw - CAMBIADO A ENTRY
            kd_yaw_frame = ttk.Frame(yaw_frame)
            kd_yaw_frame.pack(fill=tk.X, pady=1)
            ttk.Label(kd_yaw_frame, text="Kd:", width=4).pack(side=tk.LEFT)
            kd_yaw_entry = ttk.Entry(kd_yaw_frame, width=8, font=('Arial', 9), justify='center')
            kd_yaw_entry.insert(0, "0.05")
            kd_yaw_entry.pack(side=tk.LEFT, padx=2)
            kd_yaw_entry.bind('<Return>', lambda e: self.on_pid_entry_change('kd_yaw', kd_yaw_entry))
            kd_yaw_entry.bind('<FocusOut>', lambda e: self.on_pid_entry_change('kd_yaw', kd_yaw_entry))
            
            # Setpoint Yaw - CAMBIADO A ENTRY
            sp_yaw_frame = ttk.Frame(yaw_frame)
            sp_yaw_frame.pack(fill=tk.X, pady=1)
            ttk.Label(sp_yaw_frame, text="Setpoint:", width=8).pack(side=tk.LEFT)
            sp_yaw_entry = ttk.Entry(sp_yaw_frame, width=8, font=('Arial', 9), justify='center')
            sp_yaw_entry.insert(0, "0.0")
            sp_yaw_entry.pack(side=tk.LEFT, padx=2)
            sp_yaw_entry.bind('<Return>', lambda e: self.on_pid_entry_change('setpoint_yaw', sp_yaw_entry))
            sp_yaw_entry.bind('<FocusOut>', lambda e: self.on_pid_entry_change('setpoint_yaw', sp_yaw_entry))
            
            # Output Yaw (solo lectura)
            out_yaw_frame = ttk.Frame(yaw_frame)
            out_yaw_frame.pack(fill=tk.X, pady=1)
            ttk.Label(out_yaw_frame, text="Output:", width=8).pack(side=tk.LEFT)
            ttk.Label(out_yaw_frame, textvariable=self.pid_vars['output_yaw'], 
                    width=8, font=('Arial', 9, 'bold')).pack(side=tk.RIGHT)
            
            # Botones de control PID
            pid_buttons_frame = ttk.Frame(pid_frame)
            pid_buttons_frame.pack(fill=tk.X, pady=5)

            ttk.Button(pid_buttons_frame, text="Reset PID", 
                    command=self.reset_pid).pack(side=tk.LEFT, padx=2)
            ttk.Button(pid_buttons_frame, text="Aplicar Corrección", 
                    command=self.apply_pid_correction).pack(side=tk.LEFT, padx=2)

            # Estado del PID
            self.pid_status_var = tk.StringVar(value="PID: Inactivo")
            ttk.Label(pid_buttons_frame, textvariable=self.pid_status_var,
                    foreground="red", font=('Arial', 9)).pack(side=tk.RIGHT, padx=5)
            
            # Guardar referencias a los entries para acceso posterior
            self.pid_entries = {
                'kp_roll': kp_roll_entry, 'ki_roll': ki_roll_entry, 'kd_roll': kd_roll_entry, 'setpoint_roll': sp_roll_entry,
                'kp_pitch': kp_pitch_entry, 'ki_pitch': ki_pitch_entry, 'kd_pitch': kd_pitch_entry, 'setpoint_pitch': sp_pitch_entry,
                'kp_yaw': kp_yaw_entry, 'ki_yaw': ki_yaw_entry, 'kd_yaw': kd_yaw_entry, 'setpoint_yaw': sp_yaw_entry
            }
    
    def start_mpu_sensor(self):
        """Iniciar recepción de datos del MPU-6050 SOLO si hay conexión WebSocket"""
        if not ws_connected or ws is None:
            messagebox.showwarning("Advertencia", "Primero conecta el WebSocket")
            return
            
        try:
            # Solo iniciar el listener - la conexión ya está establecida
            mpu_handler.start_mpu_listener()
            self.mpu_status_var.set("Sensor: Activo - Enviando datos")
            
            # ✅ CAMBIO: Cambiar color a verde cuando está activo
            self.mpu_status_label.config(foreground="green")
            
            print("✅ Sensor MPU-6050 iniciado correctamente - Envío de datos ACTIVADO")
        except Exception as e:
            print(f"❌ Error iniciando sensor MPU: {e}")

    def stop_mpu_sensor(self):
        """Detener recepción de datos del MPU-6050"""
        mpu_handler.stop_mpu_listener()
        self.mpu_status_var.set("Sensor: Inactivo")  
        
        # ✅ CAMBIO: Cambiar color a rojo cuando está inactivo
        self.mpu_status_label.config(foreground="red")
        
        print("🛑 Sensor MPU-6050 detenido - Envío de datos DESACTIVADO")

    def start_mpu_updates(self):
        """Iniciar actualización periódica de la GUI con datos del MPU"""
        self.update_mpu_display()
        # Iniciar aplicación automática de PID
        self.auto_apply_pid_correction()
    
    def update_mpu_display(self):
        """Actualizar display con los últimos datos del MPU-6050 y calcular PID"""
        try:
            while not mpu_handler.response_queue.empty():
                self._process_mpu_data(json.dumps(mpu_handler.response_queue.get_nowait()))
            # Obtener datos más recientes
            data = mpu_handler.get_latest_data()
            if data:
                # Actualizar variables de la GUI con los nuevos datos
                # Acelerómetro
                if 'accelerometer' in data:
                    accel = data['accelerometer']
                    for axis in ['x', 'y', 'z']:
                        if axis in accel:
                            self.mpu_vars['accelerometer'][axis].set(f"{accel[axis]:.3f}")
                
                # Giroscopio
                if 'gyroscope' in data:
                    gyro = data['gyroscope']
                    for axis in ['x', 'y', 'z']:
                        if axis in gyro:
                            self.mpu_vars['gyroscope'][axis].set(f"{gyro[axis]:.3f}")
                
                # Orientación
                if 'orientation' in data:
                    orient = data['orientation']
                    for angle in ['roll', 'pitch', 'yaw']:
                        if angle in orient:
                            self.mpu_vars['orientation'][angle].set(f"{orient[angle]:.3f}")
                
                # Temperatura
                if 'temperature' in data:
                    self.mpu_vars['temperature'].set(f"{data['temperature']:.1f}")
                
                # Calcular PID automáticamente si está activado
                if self.pid_vars['enabled'].get():
                    current_roll = data['orientation'].get('roll', 0.0)
                    current_pitch = data['orientation'].get('pitch', 0.0)
                    current_yaw = data['orientation'].get('yaw', 0.0)
                    self.calculate_pid(current_roll, current_pitch, current_yaw)
            
            # Debug: mostrar que se está actualizando (puedes quitar esto después)
            # print("📊 Actualizando datos MPU...")
            
        except Exception as e:
            print(f"⚠️ Error actualizando display MPU: {e}")
        
        # Programar próxima actualización
        self.root.after(100, self.update_mpu_display)

    def heartbeat_loop(self):
        """Heartbeat periódico para el fail-safe del ESP32"""
        send_heartbeat()
        self.root.after(HEARTBEAT_MS, self.heartbeat_loop)

    def _process_mpu_data(self, data_str):
        """Procesar datos JSON del MPU-6050 y respuestas de comandos"""
        try:
            data = json.loads(data_str)
            
            # Si es respuesta de comando
            if 'status' in data:
                status = data['status']
                print(f"📨 ESP32: {status}")
                
                if status == "IMU calibrated":
                    messagebox.showinfo("Calibración", "✅ IMU calibrado exitosamente")
                elif status == "IMU reset":
                    print("✅ Orientación del IMU reseteada")
                elif status == "Offsets updated":
                    messagebox.showinfo("Offsets", "Offsets aplicados correctamente")
                elif status == "error":
                    messagebox.showerror("ESP32", data.get('error', 'Error de comando'))
                    
            # Si son offsets
            if 'accel_offsets' in data and 'gyro_offsets' in data:
                accel_offsets = data['accel_offsets']
                gyro_offsets = data['gyro_offsets']
                
                print(f"📊 Offsets actuales - Acelerómetro: {accel_offsets}")
                print(f"📊 Offsets actuales - Giroscopio: {gyro_offsets}")
                
                # Mostrar en ventana de mensajes
                offsets_msg = f"""Offsets actuales del IMU:

    Acelerómetro:
    X: {accel_offsets[0]}
    Y: {accel_offsets[1]}  
    Z: {accel_offsets[2]}

    Giroscopio:
    X: {gyro_offsets[0]}
    Y: {gyro_offsets[1]}
    Z: {gyro_offsets[2]}"""
                
                messagebox.showinfo("Offsets del IMU", offsets_msg)
                
            # Si son datos normales del sensor
            elif 'accelerometer' in data:
                # ... código existente para procesar datos del sensor ...
                pass
                
        except Exception as e:
            print(f"⚠️ Error procesando datos MPU: {e}")

    def calibrate_imu(self):
        """Calibrar el IMU usando la conexión compartida"""
        if not send_to_esp32_flag.get():
            messagebox.showwarning("Advertencia", "Primero activa 'Enviar ángulos al ESP32'")
            return
        
        try:
            send_command("cmd:calibrate_imu")  # Usar send_command en lugar de ws.send directo
            messagebox.showinfo("Calibración", "Calibrando IMU...\nMantén el robot inmóvil")
            print("🔄 Calibrando IMU...")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo enviar comando: {e}")

    def reset_imu(self):
        """Resetear orientación del IMU"""
        if not send_to_esp32_flag.get():
            messagebox.showwarning("Advertencia", "Primero activa 'Enviar ángulos al ESP32'")
            return
        
        try:
            if not send_command("cmd:reset_imu"):
                raise ConnectionError("No se pudo enviar el comando al ESP32")
            print("🔄 Orientación del IMU reseteada")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo enviar comando: {e}")

    def get_imu_offsets(self):
        """Obtener offsets actuales del IMU"""
        if not send_to_esp32_flag.get():
            messagebox.showwarning("Advertencia", "Primero activa 'Enviar ángulos al ESP32'")
            return
        
        try:
            if not send_command("cmd:get_offsets"):
                raise ConnectionError("No se pudo enviar el comando al ESP32")
            print("📊 Solicitando offsets del IMU...")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo enviar comando: {e}")

    def set_imu_offsets(self):
        """Establecer offsets manualmente"""
        if not send_to_esp32_flag.get():
            messagebox.showwarning("Advertencia", "Primero activa 'Enviar ángulos al ESP32'")
            return
        
        try:
            # Obtener valores de las entradas
            ax = self.offset_vars['acelerómetro_x'].get()
            ay = self.offset_vars['acelerómetro_y'].get()
            az = self.offset_vars['acelerómetro_z'].get()
            gx = self.offset_vars['giroscopio_x'].get()
            gy = self.offset_vars['giroscopio_y'].get()
            gz = self.offset_vars['giroscopio_z'].get()
            
            # Formatear comando
            cmd = f"cmd:set_offsets:{ax},{ay},{az},{gx},{gy},{gz}"
            if not send_command(cmd):
                raise ConnectionError("No se pudo enviar el comando al ESP32")
            
            print(f"💾 Aplicando offsets: A({ax},{ay},{az}) G({gx},{gy},{gz})")
            # La confirmación se procesa al recibir 'Offsets updated'.
            
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo enviar offsets: {e}")

    # -----------------------------
    # Metodos DIFERENCIALES TAP
    # -----------------------------


    def sync_with_inverse(self):
            """Sincroniza la configuración actual desde la pestaña de cinemática inversa"""
            try:
                # Obtener posiciones actuales de las patas desde cinemática inversa
                for i, leg_vars in enumerate(self.leg_vars):
                    self.current_leg_positions[i] = [
                        leg_vars['x'].get(),
                        leg_vars['y'].get(),
                        leg_vars['z'].get()
                    ]
                
                # Actualizar visualización en cinemática diferencial
                self.update_differential_visualization()
                
                # Actualizar display de posición
                self.update_differential_position_display(self.differential_leg_selected.get())
                
                messagebox.showinfo("Sincronizado", "Configuración sincronizada exitosamente")
                
            except Exception as e:
                messagebox.showerror("Error", f"Error al sincronizar: {e}")

    def update_differential_speed(self):
        """Actualizar velocidad en cinemática diferencial basada en velocidad global"""
        base_speed = 0.005  # Velocidad base original
        adjusted_speed = base_speed * self.global_speed.get()
        self.velocity_step.set(adjusted_speed)

    def setup_differential_visualization_frame(self, parent):
        """Frame para visualización (gráfico y tabla de ángulos)"""
        # Gráfico 3D
        self.setup_differential_plot(parent)
        
        # Tabla de ángulos
        self.setup_differential_angle_display(parent)

    def setup_differential_controls(self, parent):
        # Botón para sincronizar con cinemática inversa
        sync_frame = ttk.Frame(parent)
        sync_frame.pack(fill=tk.X, pady=5)
        ttk.Button(sync_frame, text="Sincronizar con Cinemática Inversa", 
                command=self.sync_with_inverse).pack(fill=tk.X, pady=5)
        
        # CONTROLES DE PLATAFORMA
        self.setup_differential_platform_controls(parent)
        
        # Configuración de velocidad
        speed_frame = ttk.LabelFrame(parent, text="Configuración de Velocidad")
        speed_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(speed_frame, text="Paso de velocidad (m/s):").pack(anchor=tk.W, padx=5)
        ttk.Scale(speed_frame, from_=0.001, to=0.02, orient=tk.HORIZONTAL,
                variable=self.velocity_step).pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(speed_frame, textvariable=self.velocity_step).pack(anchor=tk.W, padx=5)
        
        # Selección de pata
        leg_frame = ttk.LabelFrame(parent, text="Selección de Pata")
        leg_frame.pack(fill=tk.X, pady=5)
        
        leg_names = ["Delantera Izquierda", "Trasera Izquierda", "Trasera Derecha", "Delantera Derecha"]
        for i, name in enumerate(leg_names):
            ttk.Radiobutton(leg_frame, text=name, variable=self.differential_leg_selected, 
                        value=i, command=self.on_differential_leg_change).pack(anchor=tk.W, padx=5, pady=2)
        
        # Controles de velocidad por pata
        control_frame = ttk.LabelFrame(parent, text="Control de Velocidad")
        control_frame.pack(fill=tk.X, pady=5)
        
        # Controles para patas individuales
        individual_control_frame = ttk.Frame(control_frame)
        individual_control_frame.pack(fill=tk.X, pady=2)
        
        # Frame para controles X
        x_frame = ttk.Frame(individual_control_frame)
        x_frame.pack(fill=tk.X, pady=1)
        ttk.Label(x_frame, text="X:", width=3).pack(side=tk.LEFT)
        ttk.Button(x_frame, text="X-", command=lambda: self.move_leg_differential(-1, 0, 0)).pack(side=tk.LEFT, padx=1)
        ttk.Button(x_frame, text="X+", command=lambda: self.move_leg_differential(1, 0, 0)).pack(side=tk.LEFT, padx=1)
        
        # Frame para controles Y
        y_frame = ttk.Frame(individual_control_frame)
        y_frame.pack(fill=tk.X, pady=1)
        ttk.Label(y_frame, text="Y:", width=3).pack(side=tk.LEFT)
        ttk.Button(y_frame, text="Y-", command=lambda: self.move_leg_differential(0, -1, 0)).pack(side=tk.LEFT, padx=1)
        ttk.Button(y_frame, text="Y+", command=lambda: self.move_leg_differential(0, 1, 0)).pack(side=tk.LEFT, padx=1)
        
        # Frame para controles Z
        z_frame = ttk.Frame(individual_control_frame)
        z_frame.pack(fill=tk.X, pady=1)
        ttk.Label(z_frame, text="Z:", width=3).pack(side=tk.LEFT)
        ttk.Button(z_frame, text="Z-", command=lambda: self.move_leg_differential(0, 0, -1)).pack(side=tk.LEFT, padx=1)
        ttk.Button(z_frame, text="Z+", command=lambda: self.move_leg_differential(0, 0, 1)).pack(side=tk.LEFT, padx=1)
        
        # CONTROLES PARA PATAS DIAGONALES
        diagonal_frame = ttk.LabelFrame(parent, text="Control de Patas Diagonales")
        diagonal_frame.pack(fill=tk.X, pady=5)
        
        # Checkbox para activar/desactivar patas diagonales
        diagonal_check_frame = ttk.Frame(diagonal_frame)
        diagonal_check_frame.pack(fill=tk.X, pady=2)
        
        ttk.Checkbutton(diagonal_check_frame, text="Activar Patas Diagonales", 
                    variable=self.diagonal_legs_enabled,
                    command=self.on_diagonal_toggle).pack(anchor=tk.W, padx=5)
        
        # Frame para selección de diagonales (solo visible cuando está activado)
        self.diagonal_selection_frame = ttk.Frame(diagonal_frame)
        self.diagonal_selection_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(self.diagonal_selection_frame, text="Diagonal:").pack(side=tk.LEFT, padx=5)
        
        # Radio buttons para selección de diagonal
        self.diagonal_var = tk.IntVar(value=1)
        ttk.Radiobutton(self.diagonal_selection_frame, text="Diagonal 1 (0,2)", 
                    variable=self.diagonal_var, value=1,
                    command=self.on_diagonal_change).pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(self.diagonal_selection_frame, text="Diagonal 2 (1,3)", 
                    variable=self.diagonal_var, value=2,
                    command=self.on_diagonal_change).pack(side=tk.LEFT, padx=5)
        
        # Controles para diagonales (solo visible cuando está activado)
        self.diagonal_control_frame = ttk.Frame(diagonal_frame)
        self.diagonal_control_frame.pack(fill=tk.X, pady=2)
        
        # Frame para controles X
        diag_x_frame = ttk.Frame(self.diagonal_control_frame)
        diag_x_frame.pack(fill=tk.X, pady=1)
        ttk.Label(diag_x_frame, text="X:", width=3).pack(side=tk.LEFT)
        ttk.Button(diag_x_frame, text="X-", command=lambda: self.move_diagonal_legs(-1, 0, 0)).pack(side=tk.LEFT, padx=1)
        ttk.Button(diag_x_frame, text="X+", command=lambda: self.move_diagonal_legs(1, 0, 0)).pack(side=tk.LEFT, padx=1)
        
        # Frame para controles Y
        diag_y_frame = ttk.Frame(self.diagonal_control_frame)
        diag_y_frame.pack(fill=tk.X, pady=1)
        ttk.Label(diag_y_frame, text="Y:", width=3).pack(side=tk.LEFT)
        ttk.Button(diag_y_frame, text="Y-", command=lambda: self.move_diagonal_legs(0, -1, 0)).pack(side=tk.LEFT, padx=1)
        ttk.Button(diag_y_frame, text="Y+", command=lambda: self.move_diagonal_legs(0, 1, 0)).pack(side=tk.LEFT, padx=1)
        
        # Frame para controles Z
        diag_z_frame = ttk.Frame(self.diagonal_control_frame)
        diag_z_frame.pack(fill=tk.X, pady=1)
        ttk.Label(diag_z_frame, text="Z:", width=3).pack(side=tk.LEFT)
        ttk.Button(diag_z_frame, text="Z-", command=lambda: self.move_diagonal_legs(0, 0, -1)).pack(side=tk.LEFT, padx=1)
        ttk.Button(diag_z_frame, text="Z+", command=lambda: self.move_diagonal_legs(0, 0, 1)).pack(side=tk.LEFT, padx=1)
        
        # Inicializar estado de las diagonales
        self.diagonal_legs = [0, 2]  # Diagonal por defecto
        self.update_diagonal_controls_visibility()
        
        # Display de posición actual
        pos_frame = ttk.LabelFrame(parent, text="Posición Actual de la Pata")
        pos_frame.pack(fill=tk.X, pady=5)
        
        self.differential_pos_labels = []
        for i, coord in enumerate(['X', 'Y', 'Z']):
            coord_frame = ttk.Frame(pos_frame)
            coord_frame.pack(fill=tk.X, pady=1)
            ttk.Label(coord_frame, text=f"{coord}:", width=5).pack(side=tk.LEFT)
            pos_label = ttk.Label(coord_frame, text="0.000", width=8)
            pos_label.pack(side=tk.LEFT)
            self.differential_pos_labels.append(pos_label)

    def on_diagonal_toggle(self):
        """Manejar activación/desactivación de patas diagonales"""
        self.update_diagonal_controls_visibility()

    def on_diagonal_change(self):
        """Manejar cambio de diagonal seleccionada"""
        if self.diagonal_var.get() == 1:
            self.diagonal_legs = [0, 2]  # Diagonal 1
        else:
            self.diagonal_legs = [1, 3]  # Diagonal 2
    
    def update_diagonal_controls_visibility(self):
        """Actualizar visibilidad de controles diagonales"""
        if self.diagonal_legs_enabled.get():
            self.diagonal_selection_frame.pack(fill=tk.X, pady=2)
            self.diagonal_control_frame.pack(fill=tk.X, pady=2)
        else:
            self.diagonal_selection_frame.pack_forget()
            self.diagonal_control_frame.pack_forget()

    def move_diagonal_legs(self, vx, vy, vz):
        """Mover patas diagonales simultáneamente"""
        step_size = self.velocity_step.get()
        
        for leg_id in self.diagonal_legs:
            current_pos = self.current_leg_positions[leg_id].copy()
            
            new_pos = [
                current_pos[0] + vx * step_size,
                current_pos[1] + vy * step_size,
                current_pos[2] + vz * step_size
            ]
            
            # Verificar límites físicos
            reachable, error_msg = self.kin.check_physical_limits(new_pos, leg_id)
            if not reachable:
                messagebox.showwarning("Límite Alcanzado", f"Pata {leg_id}: {error_msg}")
                continue
            
            # Actualizar posición
            self.current_leg_positions[leg_id] = new_pos
        
        # Actualizar visualización
        self.update_differential_visualization()
        self.update_differential_position_display(self.differential_leg_selected.get())

    def setup_differential_platform_controls(self, parent):
        """Controles de plataforma para cinemática diferencial - CORREGIDO"""
        pf = ttk.LabelFrame(parent, text="Plataforma (Cuerpo)")
        pf.pack(fill=tk.X, pady=5)

        # Crear variables específicas para diferencial si no existen
        if not hasattr(self, 'diff_platform_vars'):
            initial_pos = {'x': 0, 'y': 0.0, 'z': 0.18}
            initial_rot = {'roll': 0, 'pitch': 0.0, 'yaw': 0.0}
            
            self.diff_platform_vars = {
                'position': {c: tk.DoubleVar(value=initial_pos[c]) for c in ['x','y','z']},
                'rotation': {a: tk.DoubleVar(value=initial_rot[a]) for a in ['roll','pitch','yaw']}
            }

        # Frame para posición
        pos_frame = ttk.Frame(pf)
        pos_frame.pack(fill=tk.X, pady=2)
        ttk.Label(pos_frame, text="Posición:", font=('Arial', 9, 'bold'), width=10).pack(side=tk.LEFT)
        
        for coord in ['x','y','z']:
            coord_frame = ttk.Frame(pos_frame)
            coord_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
            
            ttk.Label(coord_frame, text=f"{coord.upper()}", width=2, 
                    font=('Arial', 8, 'bold')).pack(side=tk.LEFT)
            
            # Slider más compacto para posición
            slider = ttk.Scale(coord_frame, from_=-0.03, to=0.25,
                    orient=tk.HORIZONTAL, variable=self.diff_platform_vars['position'][coord],
                    command=lambda v, c=coord: self.on_diff_platform_change('position', c),
                    length=60)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
            
            # Etiqueta de valor - CREAR NUEVAS ETIQUETAS COMO EN CINEMÁTICA INVERSA
            if not hasattr(self, 'diff_platform_labels'):
                self.diff_platform_labels = {}
            
            self.diff_platform_labels[f'pos_{coord}'] = ttk.Label(coord_frame, 
                    text=f"{self.diff_platform_vars['position'][coord].get():.3f}", 
                    width=5, font=('Arial', 8))
            self.diff_platform_labels[f'pos_{coord}'].pack(side=tk.RIGHT, padx=2)
        
        # Frame para rotación
        rot_frame = ttk.Frame(pf)
        rot_frame.pack(fill=tk.X, pady=2)
        ttk.Label(rot_frame, text="Rotación:", font=('Arial', 9, 'bold'), width=10).pack(side=tk.LEFT)
        
        for ang in ['roll','pitch','yaw']:
            ang_frame = ttk.Frame(rot_frame)
            ang_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
            
            ttk.Label(ang_frame, text=ang[0].upper(), width=2,
                    font=('Arial', 8, 'bold')).pack(side=tk.LEFT)
            
            # Slider más compacto para rotación
            slider = ttk.Scale(ang_frame, from_=-180, to=180,
                    orient=tk.HORIZONTAL, variable=self.diff_platform_vars['rotation'][ang],
                    command=lambda v, a=ang: self.on_diff_platform_change('rotation', a),
                    length=60)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
            
            # Etiqueta de valor - CREAR NUEVAS ETIQUETAS COMO EN CINEMÁTICA INVERSA
            self.diff_platform_labels[f'rot_{ang}'] = ttk.Label(ang_frame,
                    text=f"{self.diff_platform_vars['rotation'][ang].get():.1f}", 
                    width=5, font=('Arial', 8))
            self.diff_platform_labels[f'rot_{ang}'].pack(side=tk.RIGHT, padx=2)

    def on_diff_platform_change(self, control_type, axis):
        """Actualizar etiquetas cuando cambian los sliders de plataforma diferencial - CORREGIDO"""
        if control_type == 'position':
            value = self.diff_platform_vars['position'][axis].get()
            # Actualizar la etiqueta correspondiente
            if f'pos_{axis}' in self.diff_platform_labels:
                self.diff_platform_labels[f'pos_{axis}'].config(text=f"{value:.3f}")
        else:
            value = self.diff_platform_vars['rotation'][axis].get()
            # Actualizar la etiqueta correspondiente
            if f'rot_{axis}' in self.diff_platform_labels:
                self.diff_platform_labels[f'rot_{axis}'].config(text=f"{value:.1f}")
        
        self.update_differential_visualization()

    def setup_differential_plot(self, parent):
        from robot_view import RobotViewport
        self.diff_fig = plt.Figure(figsize=(6, 5))
        self.diff_ax = self.diff_fig.add_subplot(111, projection='3d')
        self.diff_viewport = RobotViewport(self.diff_ax, self.kin)
        self.diff_ax._robot_viewport = self.diff_viewport
        self.diff_viewport.add_controls(parent)
        self.diff_canvas = FigureCanvasTkAgg(self.diff_fig, parent)
        self.diff_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def setup_differential_angle_display(self, parent):
        angle_frame = ttk.Frame(parent)
        angle_frame.pack(fill=tk.X, pady=2)
        
        self.diff_angle_table = ttk.Treeview(angle_frame, columns=("theta1","theta2","theta3"), 
                                           show="headings", height=4)
        self.diff_angle_table.pack(fill=tk.X, pady=2)
        
        self.diff_angle_table.heading("theta1", text="Theta 1 (°)")
        self.diff_angle_table.heading("theta2", text="Theta 2 (°)")
        self.diff_angle_table.heading("theta3", text="Theta 3 (°)")
        
        self.diff_angle_table.column("theta1", width=80, anchor="center")
        self.diff_angle_table.column("theta2", width=80, anchor="center")
        self.diff_angle_table.column("theta3", width=80, anchor="center")
        
        for i in range(4):
            self.diff_angle_table.insert("", "end", values=("--","--","--"))

    def setup_differential_routine_controls(self, parent):
        """Rutinas idénticas a las de cinemática inversa CON SCROLL"""
        routine_frame = ttk.LabelFrame(parent, text="Rutinas - Cinemática Diferencial")
        routine_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Crear contenedor con scroll para rutinas diferenciales
        routine_container = ttk.Frame(routine_frame)
        routine_container.pack(fill=tk.BOTH, expand=True)
        
        # Canvas y scrollbar
        self.diff_routine_canvas = tk.Canvas(routine_container, highlightthickness=0)
        diff_routine_scrollbar = ttk.Scrollbar(routine_container, orient="vertical", command=self.diff_routine_canvas.yview)
        
        # Frame scrollable
        self.scrollable_diff_routine_frame = ttk.Frame(self.diff_routine_canvas)
        
        self.scrollable_diff_routine_frame.bind(
            "<Configure>",
            lambda e: self.diff_routine_canvas.configure(scrollregion=self.diff_routine_canvas.bbox("all"))
        )
        
        self.diff_routine_canvas.create_window((0, 0), window=self.scrollable_diff_routine_frame, anchor="nw")
        self.diff_routine_canvas.configure(yscrollcommand=diff_routine_scrollbar.set)
        
        # Empaquetar
        self.diff_routine_canvas.pack(side="left", fill="both", expand=True)
        diff_routine_scrollbar.pack(side="right", fill="y")
        
        # Bind mouse wheel
        self.diff_routine_canvas.bind("<MouseWheel>", self._on_diff_routine_mousewheel)
        
        # Todo el contenido de rutinas diferenciales va aquí dentro...
        
        # Frame para controles de velocidad
        speed_frame = ttk.LabelFrame(self.scrollable_diff_routine_frame, text="Control de Velocidad")
        speed_frame.pack(fill=tk.X, pady=5, padx=5)
        
        # Control de delay entre pasos
        delay_frame = ttk.Frame(speed_frame)
        delay_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(delay_frame, text="Delay entre pasos (ms):", 
                font=('Arial', 9)).pack(side=tk.LEFT, padx=5)
        
        self.diff_step_delay_ms = tk.IntVar(value=1000)
        delay_entry = ttk.Entry(delay_frame, width=8, 
                            textvariable=self.diff_step_delay_ms,
                            font=('Arial', 9))
        delay_entry.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(delay_frame, text="(1000 ms = 1 segundo)").pack(side=tk.LEFT, padx=5)
        
        # Control de pasos intermedios
        intermediate_frame = ttk.Frame(speed_frame)
        intermediate_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(intermediate_frame, text="Pasos intermedios:", 
                font=('Arial', 9)).pack(side=tk.LEFT, padx=5)
        
        self.diff_intermediate_steps = tk.IntVar(value=5)
        intermediate_entry = ttk.Entry(intermediate_frame, width=8,
                                    textvariable=self.diff_intermediate_steps,
                                    font=('Arial', 9))
        intermediate_entry.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(intermediate_frame, text="(más pasos = movimiento más suave)").pack(side=tk.LEFT, padx=5)
        
        # Display de pasos
        steps_frame = ttk.LabelFrame(self.scrollable_diff_routine_frame, text="Pasos de Rutina Diferencial")
        steps_frame.pack(fill=tk.BOTH, expand=True, pady=5, padx=5)
        
        # Listbox para mostrar pasos
        self.diff_steps_listbox = tk.Listbox(steps_frame, height=8, font=('Arial', 9))
        self.diff_steps_listbox.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Scrollbar para el listbox
        listbox_scrollbar = ttk.Scrollbar(self.diff_steps_listbox)
        listbox_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.diff_steps_listbox.config(yscrollcommand=listbox_scrollbar.set)
        listbox_scrollbar.config(command=self.diff_steps_listbox.yview)
        
        # Frame para botones de control de pasos
        steps_buttons_frame = ttk.Frame(self.scrollable_diff_routine_frame)
        steps_buttons_frame.pack(fill=tk.X, pady=5)
        
        # Botones básicos
        ttk.Button(steps_buttons_frame, text="Guardar Paso", 
                command=self.save_differential_step).pack(fill=tk.X, pady=2)
        ttk.Button(steps_buttons_frame, text="Borrar Paso", 
                command=self.delete_differential_step).pack(fill=tk.X, pady=2)
        ttk.Button(steps_buttons_frame, text="Editar Paso", 
                command=self.edit_differential_step).pack(fill=tk.X, pady=2)
        ttk.Button(steps_buttons_frame, text="Actualizar Paso", 
                command=self.update_differential_step).pack(fill=tk.X, pady=2)
        ttk.Button(steps_buttons_frame, text="Ejecutar Paso", 
                command=self.execute_differential_step).pack(fill=tk.X, pady=2)
        
        # Frame para botones de rutina completa
        routine_buttons_frame = ttk.Frame(self.scrollable_diff_routine_frame)
        routine_buttons_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(routine_buttons_frame, text="Ejecutar Rutina", 
                command=self.execute_differential_routine).pack(fill=tk.X, pady=2)
        ttk.Button(routine_buttons_frame, text="Borrar Toda la Rutina", 
                command=self.clear_differential_routine).pack(fill=tk.X, pady=2)
        
        # Frame para guardar/cargar rutina
        file_buttons_frame = ttk.Frame(self.scrollable_diff_routine_frame)
        file_buttons_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(file_buttons_frame, text="Guardar Rutina", 
                command=self.save_differential_routine).pack(fill=tk.X, pady=2)
        ttk.Button(file_buttons_frame, text="Cargar Rutina", 
                command=self.load_differential_routine).pack(fill=tk.X, pady=2)

        # Frame para botón de detener diferencial
        diff_stop_frame = ttk.Frame(self.scrollable_diff_routine_frame)
        diff_stop_frame.pack(fill=tk.X, pady=2)
        
        self.diff_stop_button = ttk.Button(diff_stop_frame, text="⏹️ Detener Rutina", 
                                        command=self.stop_differential_routine, state="disabled")
        self.diff_stop_button.pack(fill=tk.X)

        # Ajustar el tamaño del frame scrollable
        self.scrollable_diff_routine_frame.update_idletasks()
        self.diff_routine_canvas.config(scrollregion=self.diff_routine_canvas.bbox("all"))

    def _on_diff_routine_mousewheel(self, event):
        """Manejar scroll del mouse en el canvas de rutinas diferenciales"""
        self.diff_routine_canvas.yview_scroll(int(-1*(event.delta/120)), "units")

    def on_differential_leg_change(self):
        """Actualiza el display de posición cuando cambia la pata seleccionada"""
        leg_id = self.differential_leg_selected.get()
        self.update_differential_position_display(leg_id)

    def move_leg_differential(self, vx, vy, vz):
        """Mueve una pata usando cinemática diferencial"""
        # Si las patas diagonales están activadas, mover las diagonales
        if self.diagonal_legs_enabled.get():
            self.move_diagonal_legs(vx, vy, vz)
            return
        
        # Si no, mover la pata individual seleccionada
        leg_id = self.differential_leg_selected.get()
        step_size = self.velocity_step.get()
        
        # Obtener posición actual
        current_pos = self.current_leg_positions[leg_id].copy()
        
        # Calcular nueva posición directamente
        new_pos = [
            current_pos[0] + vx * step_size,
            current_pos[1] + vy * step_size,
            current_pos[2] + vz * step_size
        ]
        
        # Verificar límites físicos
        reachable, error_msg = self.kin.check_physical_limits(new_pos, leg_id)
        if not reachable:
            messagebox.showwarning("Límite Alcanzado", error_msg)
            return
        
        # Actualizar posición
        self.current_leg_positions[leg_id] = new_pos
        
        # Actualizar visualización
        self.update_differential_visualization()
        
        # Actualizar display de posición
        self.update_differential_position_display(leg_id)

    def update_differential_position_display(self, leg_id):
        """Actualiza el display de posición para la pata seleccionada"""
        pos = self.current_leg_positions[leg_id]
        for i, coord in enumerate(pos):
            self.differential_pos_labels[i].config(text=f"{coord:.3f}")

    def update_differential_visualization(self):
        """Actualiza la visualización en la pestaña de cinemática diferencial"""
        # Usar configuración de plataforma diferencial
        if hasattr(self, 'diff_platform_vars'):
            rot = [self.diff_platform_vars['rotation'][a].get() for a in ['roll','pitch','yaw']]
            body_pos = [self.diff_platform_vars['position'][a].get() for a in ['x','y','z']]
        else:
            # Fallback a plataforma inversa
            rot = [self.platform_vars['rotation'][a].get() for a in ['roll','pitch','yaw']]
            body_pos = [self.platform_vars['position'][a].get() for a in ['x','y','z']]
        
        # Calcular ángulos para las nuevas posiciones
        angles_list, physical_errors = self.kin.plot_robot(
            self.diff_ax, self.current_leg_positions, rot, body_pos, is_radians=False
        )
        
        self.diff_canvas.draw()
        
        # Actualizar tabla de ángulos
        self.update_differential_angle_display(angles_list)
        
        # Enviar ángulos al ESP32 si está activado
        if not physical_errors and send_to_esp32_flag.get():
            calibrated_flat_angles = self.get_calibrated_angles_for_esp32(angles_list)
            send_angles(calibrated_flat_angles)

    def update_differential_angle_display(self, angles_list):
        """Actualiza la tabla de ángulos en cinemática diferencial"""
        # Aplicar calibración a TODAS las piernas
        calibrated_angles = self.apply_calibration(angles_list)
        
        for i, angles in enumerate(calibrated_angles):
            if any(np.isnan(angles)):
                self.diff_angle_table.item(self.diff_angle_table.get_children()[i], values=("--","--","--"))
            else:
                degs = [degrees(normalize_angle(a)) for a in angles]
                degs = [(d + 180) % 360 - 180 for d in degs]
                self.diff_angle_table.item(self.diff_angle_table.get_children()[i], 
                                        values=[f"{d:.1f}" for d in degs])


    # =============================================================================
    # MÉTODOS CONTROL PID 
    # =============================================================================

    def on_pid_toggle(self):
        """Manejar activación/desactivación del PID"""
        if self.pid_vars['enabled'].get():
            self.reset_pid()
            print("✅ Control PID activado")
        else:
            print("❌ Control PID desactivado")

    def reset_pid(self):
        """Resetear los valores integrales y anteriores del PID"""
        self.pid_integrals = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.pid_previous_errors = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.last_pid_time = time.time()
        self.pid_vars['output_roll'].set("0.0")
        self.pid_vars['output_pitch'].set("0.0")
        self.pid_vars['output_yaw'].set("0.0")
        print("🔄 PID reseteado")

    def apply_pid_correction(self):
        """Aplicar corrección PID a la plataforma del robot"""
        if not self.pid_vars['enabled'].get():
            return
        
        try:
            # Obtener orientación actual del MPU
            current_roll = float(self.mpu_vars['orientation']['roll'].get())
            current_pitch = float(self.mpu_vars['orientation']['pitch'].get())
            current_yaw = float(self.mpu_vars['orientation']['yaw'].get())
            
            # Calcular corrección PID
            correction_roll, correction_pitch, correction_yaw = self.calculate_pid(
                current_roll, current_pitch, current_yaw
            )
            
            # Aplicar corrección a la plataforma
            current_platform_roll = self.platform_vars['rotation']['roll'].get()
            current_platform_pitch = self.platform_vars['rotation']['pitch'].get()
            current_platform_yaw = self.platform_vars['rotation']['yaw'].get()
            
            # Aplicar corrección (puedes ajustar la ganancia de aplicación)
            application_gain = 0.1
            new_roll = current_platform_roll + correction_roll * application_gain
            new_pitch = current_platform_pitch + correction_pitch * application_gain
            
            # Solo aplicar corrección de yaw si está habilitado
            if self.pid_vars['enable_yaw_pid'].get():
                new_yaw = current_platform_yaw + correction_yaw * application_gain
                # Manejar wrap-around de yaw
                new_yaw = new_yaw % 360
                if new_yaw > 180:
                    new_yaw -= 360
                self.platform_vars['rotation']['yaw'].set(new_yaw)
            
            # Limitar corrección de roll y pitch
            new_roll = max(min(new_roll, 30.0), -30.0)
            new_pitch = max(min(new_pitch, 30.0), -30.0)
            
            # Aplicar a la plataforma
            self.platform_vars['rotation']['roll'].set(new_roll)
            self.platform_vars['rotation']['pitch'].set(new_pitch)
            
            # Actualizar visualización
            self.update_visualization()
            
            print(f"🔧 PID aplicado - Roll: {correction_roll:.2f}°, Pitch: {correction_pitch:.2f}°, Yaw: {correction_yaw:.2f}°")
            
        except ValueError as e:
            print(f"⚠️ Error aplicando PID: {e}")

    def auto_apply_pid_correction(self):
        """Aplicar corrección PID automáticamente si está activado"""
        if self.pid_vars['enabled'].get():
            self.apply_pid_correction()
        
        # Programar próxima aplicación (cada 500ms para no sobrecargar)
        self.root.after(500, self.auto_apply_pid_correction)

    def calculate_pid(self, current_roll, current_pitch, current_yaw=None):
        """Calcular corrección PID para roll, pitch y yaw"""
        if not self.pid_vars['enabled'].get():
            return 0.0, 0.0, 0.0
        
        current_time = time.time()
        dt = current_time - self.last_pid_time
        
        if dt <= 0:
            return 0.0, 0.0, 0.0
        
        # Calcular errores
        error_roll = self.pid_vars['setpoint_roll'].get() - current_roll
        error_pitch = self.pid_vars['setpoint_pitch'].get() - current_pitch
        
        # Inicializar error_yaw
        error_yaw = 0.0
        if current_yaw is not None and self.pid_vars['enable_yaw_pid'].get():
            # Manejar el wrap-around de yaw (de -180 a 180)
            setpoint_yaw = self.pid_vars['setpoint_yaw'].get()
            error_yaw = setpoint_yaw - current_yaw
            
            # Ajustar error_yaw para el camino más corto
            if error_yaw > 180:
                error_yaw -= 360
            elif error_yaw < -180:
                error_yaw += 360
        
        # Término proporcional
        p_roll = self.pid_vars['kp_roll'].get() * error_roll
        p_pitch = self.pid_vars['kp_pitch'].get() * error_pitch
        p_yaw = self.pid_vars['kp_yaw'].get() * error_yaw
        
        # Término integral
        self.pid_integrals['roll'] += error_roll * dt
        self.pid_integrals['pitch'] += error_pitch * dt
        self.pid_integrals['yaw'] += error_yaw * dt
        
        # Limitar integral para evitar windup
        integral_limit = 10.0
        self.pid_integrals['roll'] = max(min(self.pid_integrals['roll'], integral_limit), -integral_limit)
        self.pid_integrals['pitch'] = max(min(self.pid_integrals['pitch'], integral_limit), -integral_limit)
        self.pid_integrals['yaw'] = max(min(self.pid_integrals['yaw'], integral_limit), -integral_limit)
        
        i_roll = self.pid_vars['ki_roll'].get() * self.pid_integrals['roll']
        i_pitch = self.pid_vars['ki_pitch'].get() * self.pid_integrals['pitch']
        i_yaw = self.pid_vars['ki_yaw'].get() * self.pid_integrals['yaw']
        
        # Término derivativo
        d_roll = self.pid_vars['kd_roll'].get() * (error_roll - self.pid_previous_errors['roll']) / dt
        d_pitch = self.pid_vars['kd_pitch'].get() * (error_pitch - self.pid_previous_errors['pitch']) / dt
        d_yaw = self.pid_vars['kd_yaw'].get() * (error_yaw - self.pid_previous_errors['yaw']) / dt
        
        # Calcular salida total
        output_roll = p_roll + i_roll + d_roll
        output_pitch = p_pitch + i_pitch + d_pitch
        output_yaw = p_yaw + i_yaw + d_yaw
        
        # Limitar salida
        output_limit = 30.0
        output_roll = max(min(output_roll, output_limit), -output_limit)
        output_pitch = max(min(output_pitch, output_limit), -output_limit)
        output_yaw = max(min(output_yaw, output_limit), -output_limit)
        
        # Actualizar errores anteriores y tiempo
        self.pid_previous_errors['roll'] = error_roll
        self.pid_previous_errors['pitch'] = error_pitch
        self.pid_previous_errors['yaw'] = error_yaw
        self.last_pid_time = current_time
        
        # Actualizar display de outputs
        self.pid_vars['output_roll'].set(f"{output_roll:.2f}")
        self.pid_vars['output_pitch'].set(f"{output_pitch:.2f}")
        self.pid_vars['output_yaw'].set(f"{output_yaw:.2f}")
        
        return output_roll, output_pitch, output_yaw

    def on_pid_entry_change(self, pid_var_name, entry_widget):
        """Manejar cambios en los campos de entrada del PID"""
        try:
            # Obtener el valor del entry
            new_value = float(entry_widget.get())
            
            # Validar rangos según el tipo de parámetro
            if pid_var_name.startswith('kp'):
                if 0.0 <= new_value <= 10.0:
                    self.pid_vars[pid_var_name].set(new_value)
                else:
                    raise ValueError("Kp debe estar entre 0.0 y 5.0")
                    
            elif pid_var_name.startswith('ki'):
                if 0.0 <= new_value <= 2.0:
                    self.pid_vars[pid_var_name].set(new_value)
                else:
                    raise ValueError("Ki debe estar entre 0.0 y 2.0")
                    
            elif pid_var_name.startswith('kd'):
                if 0.0 <= new_value <= 2.0:
                    self.pid_vars[pid_var_name].set(new_value)
                else:
                    raise ValueError("Kd debe estar entre 0.0 y 2.0")
                    
            elif pid_var_name.startswith('setpoint_roll') or pid_var_name.startswith('setpoint_pitch'):
                if -30.0 <= new_value <= 30.0:
                    self.pid_vars[pid_var_name].set(new_value)
                else:
                    raise ValueError("Setpoint Roll/Pitch debe estar entre -30.0 y 30.0")
                    
            elif pid_var_name.startswith('setpoint_yaw'):
                if -180.0 <= new_value <= 180.0:
                    self.pid_vars[pid_var_name].set(new_value)
                else:
                    raise ValueError("Setpoint Yaw debe estar entre -180.0 y 180.0")
            
            print(f"✅ PID {pid_var_name} actualizado a: {new_value}")
            
        except ValueError as e:
            # Valor inválido, revertir al valor anterior
            current_value = self.pid_vars[pid_var_name].get()
            entry_widget.delete(0, tk.END)
            entry_widget.insert(0, f"{current_value}")
            error_msg = str(e) if str(e) else "Por favor ingresa un número válido"
            messagebox.showerror("Error en PID", f"Error en {pid_var_name}: {error_msg}")

    # =============================================================================
    # MÉTODOS PARA RUTINAS DE CINEMÁTICA DIFERENCIAL 
    # =============================================================================
    def update_differential_speed_realtime(self):
        """Actualizar velocidad en tiempo real durante ejecución"""
        if hasattr(self, 'diff_velocity_label'):
            current_speed = self.global_speed.get()
            adjusted_step = 0.005 * current_speed
            self.velocity_step.set(adjusted_step)
            self.diff_velocity_label.config(text=f"Vel: {current_speed:.1f}x")
    
    def reset_differential_positions(self):
        """Resetear posiciones de patas en diferencial a valores iniciales"""
        initial_positions = [
            [0.10, 0.08, -0],
            [-0.10, 0.08, -0],
            [-0.10, -0.08, -0],
            [0.10, -0.08, -0],
        ]
        
        # Usar movimiento suave para reset
        self.smooth_move_leg_differential(initial_positions, duration_ms=800)

    def smooth_move_leg_differential(self, target_positions, duration_ms=1000):
        """Mover patas a posiciones objetivo de forma suave"""
        if self.diff_routine_executing:
            return
        
        initial_positions = self.current_leg_positions.copy()
        steps = max(5, int(duration_ms / 50))  # Mínimo 5 pasos, máximo según duración
        step_delay = max(20, int(duration_ms / steps))
        
        def smooth_movement():
            for step in range(1, steps + 1):
                if self.diff_stop_routine_flag:
                    return
                    
                alpha = step / steps
                for i in range(4):
                    current_pos = initial_positions[i]
                    target_pos = target_positions[i]
                    
                    new_pos = [
                        current_pos[0] + (target_pos[0] - current_pos[0]) * alpha,
                        current_pos[1] + (target_pos[1] - current_pos[1]) * alpha,
                        current_pos[2] + (target_pos[2] - current_pos[2]) * alpha
                    ]
                    
                    # Verificar límites
                    reachable, _ = self.kin.check_physical_limits(new_pos, i)
                    if reachable:
                        self.current_leg_positions[i] = new_pos
                
                self.update_differential_visualization()
                self.update_differential_position_display(self.differential_leg_selected.get())
                
                if not self.diff_smart_delay(step_delay):
                    return
        
        # Ejecutar en hilo
        thread = threading.Thread(target=smooth_movement, daemon=True)
        thread.start()

    def diff_smart_delay(self, delay_ms):
        """Delay no bloqueante específico para diferencial"""
        start_time = time.time()
        target_time = start_time + (delay_ms / 1000.0)
        
        while time.time() < target_time and not self.diff_stop_routine_flag:
            remaining = (target_time - time.time()) * 1000
            if remaining > 100:
                self.root.update()
                time.sleep(0.1)
            else:
                time.sleep(0.01)
        
        return not self.diff_stop_routine_flag

    def enable_diff_stop_button(self):
        """Habilitar botón de detener en diferencial"""
        if hasattr(self, 'diff_stop_button'):
            self.diff_stop_button.config(state="normal")

    def disable_diff_stop_button(self):
        """Deshabilitar botón de detener en diferencial"""
        if hasattr(self, 'diff_stop_button'):
            self.diff_stop_button.config(state="disabled")

    def stop_differential_routine(self):
        """Detener ejecución de rutina diferencial"""
        self.diff_stop_routine_flag = True
        self.disable_diff_stop_button()
        messagebox.showinfo("Rutina Detenida", "La ejecución de la rutina diferencial ha sido detenida")

    def save_differential_step(self):
        """Guardar el paso actual en la rutina diferencial"""
        try:
            # Obtener configuración actual de patas
            leg_positions = []
            for pos in self.current_leg_positions:
                leg_pos = {
                    'x': pos[0],
                    'y': pos[1],
                    'z': pos[2]
                }
                leg_positions.append(leg_pos)
            
            # Obtener configuración actual de plataforma diferencial
            if hasattr(self, 'diff_platform_vars'):
                platform_pos = {
                    'x': self.diff_platform_vars['position']['x'].get(),
                    'y': self.diff_platform_vars['position']['y'].get(),
                    'z': self.diff_platform_vars['position']['z'].get()
                }
                
                platform_rot = {
                    'roll': self.diff_platform_vars['rotation']['roll'].get(),
                    'pitch': self.diff_platform_vars['rotation']['pitch'].get(),
                    'yaw': self.diff_platform_vars['rotation']['yaw'].get()
                }
            else:
                # Fallback a plataforma inversa
                platform_pos = {
                    'x': self.platform_vars['position']['x'].get(),
                    'y': self.platform_vars['position']['y'].get(),
                    'z': self.platform_vars['position']['z'].get()
                }
                
                platform_rot = {
                    'roll': self.platform_vars['rotation']['roll'].get(),
                    'pitch': self.platform_vars['rotation']['pitch'].get(),
                    'yaw': self.platform_vars['rotation']['yaw'].get()
                }
            
            # Calcular ángulos actuales
            rot = [platform_rot[a] for a in ['roll','pitch','yaw']]
            body_pos_list = [platform_pos[a] for a in ['x','y','z']]
            
            angles_list, _ = self.kin.plot_robot(self.diff_ax, self.current_leg_positions, rot, body_pos_list, is_radians=False)
            
            # Convertir ángulos a lista serializable
            angles_serializable = []
            for angles in angles_list:
                if any(np.isnan(angles)):
                    angles_serializable.append([0.0, 0.0, 0.0])
                else:
                    if hasattr(angles, 'tolist'):
                        angles_serializable.append(angles.tolist())
                    else:
                        angles_serializable.append([float(ang) for ang in angles])
            
            # Crear paso
            step = {
                'step_number': len(self.differential_routine_steps) + 1,
                'leg_positions': leg_positions,
                'platform_position': platform_pos,
                'platform_rotation': platform_rot,
                'angles': angles_serializable,
                'timestamp': time.time()
            }
            
            self.differential_routine_steps.append(step)
            self.update_differential_steps_display()
            
            messagebox.showinfo("Paso Guardado", f"Paso {step['step_number']} guardado exitosamente")
            
        except Exception as e:
            messagebox.showerror("Error", f"Error al guardar paso: {e}")

    def update_differential_step(self):
        """Actualizar el paso actualmente cargado en diferencial"""
        if self.diff_current_step_index == -1:  # <- CORREGIDO: usar diff_current_step_index
            messagebox.showwarning("Advertencia", "Primero debes cargar un paso para editar usando 'Editar Paso'")
            return
        
        try:
            # Obtener configuración actual de patas
            leg_positions = []
            for pos in self.current_leg_positions:
                leg_pos = {
                    'x': pos[0],
                    'y': pos[1],
                    'z': pos[2]
                }
                leg_positions.append(leg_pos)
            
            # Obtener configuración actual de plataforma DIFERENCIAL
            if hasattr(self, 'diff_platform_vars'):
                platform_pos = {
                    'x': self.diff_platform_vars['position']['x'].get(),
                    'y': self.diff_platform_vars['position']['y'].get(),
                    'z': self.diff_platform_vars['position']['z'].get()
                }
                
                platform_rot = {
                    'roll': self.diff_platform_vars['rotation']['roll'].get(),
                    'pitch': self.diff_platform_vars['rotation']['pitch'].get(),
                    'yaw': self.diff_platform_vars['rotation']['yaw'].get()
                }
            else:
                # Fallback a plataforma inversa
                platform_pos = {
                    'x': self.platform_vars['position']['x'].get(),
                    'y': self.platform_vars['position']['y'].get(),
                    'z': self.platform_vars['position']['z'].get()
                }
                
                platform_rot = {
                    'roll': self.platform_vars['rotation']['roll'].get(),
                    'pitch': self.platform_vars['rotation']['pitch'].get(),
                    'yaw': self.platform_vars['rotation']['yaw'].get()
                }
            
            # Calcular ángulos actuales
            rot = [platform_rot[a] for a in ['roll','pitch','yaw']]
            body_pos_list = [platform_pos[a] for a in ['x','y','z']]
            
            angles_list, _ = self.kin.plot_robot(self.diff_ax, self.current_leg_positions, rot, body_pos_list, is_radians=False)
            
            # Convertir ángulos a lista serializable
            angles_serializable = []
            for angles in angles_list:
                if any(np.isnan(angles)):
                    angles_serializable.append([0.0, 0.0, 0.0])
                else:
                    if hasattr(angles, 'tolist'):
                        angles_serializable.append(angles.tolist())
                    else:
                        angles_serializable.append([float(ang) for ang in angles])
            
            # Actualizar el paso existente en DIFERENCIAL
            updated_step = {
                'step_number': self.diff_current_step_index + 1,  # <- USAR diff_current_step_index
                'leg_positions': leg_positions,
                'platform_position': platform_pos,
                'platform_rotation': platform_rot,
                'angles': angles_serializable,
                'timestamp': time.time()
            }
            
            self.differential_routine_steps[self.diff_current_step_index] = updated_step  # <- CORREGIDO
            self.update_differential_steps_display()
            
            messagebox.showinfo("Paso Actualizado", f"Paso {self.diff_current_step_index + 1} actualizado exitosamente")
            
        except Exception as e:
            messagebox.showerror("Error", f"Error al actualizar paso: {e}")

    def delete_differential_step(self):
        """Borrar paso en cinemática diferencial"""
        selected = self.diff_steps_listbox.curselection()
        if not selected:
            messagebox.showwarning("Advertencia", "Selecciona un paso para borrar")
            return
        
        index = selected[0]
        if 0 <= index < len(self.differential_routine_steps):  # <- CAMBIAR AQUÍ
            self.differential_routine_steps.pop(index)  # <- CAMBIAR AQUÍ
            self.update_differential_steps_display()
            messagebox.showinfo("Paso Borrado", "Paso borrado exitosamente")

    def edit_differential_step(self):
        """Cargar un paso existente para editarlo en diferencial"""
        if not self.differential_routine_steps:
            messagebox.showwarning("Advertencia", "No hay pasos en la rutina diferencial para editar")
            return
        
        # Pedir número de paso al usuario
        step_number = self.ask_differential_step_number()
        if step_number is None:
            return
        
        # Verificar que el paso existe
        if step_number < 1 or step_number > len(self.differential_routine_steps):
            messagebox.showerror("Error", f"Paso {step_number} no existe. La rutina tiene {len(self.differential_routine_steps)} pasos.")
            return
        
        index = step_number - 1
        step = self.differential_routine_steps[index]
        
        # Actualizar posiciones de patas
        for i, leg_pos in enumerate(step['leg_positions']):
            self.current_leg_positions[i] = [
                leg_pos['x'],
                leg_pos['y'],
                leg_pos['z']
            ]
        
        # Actualizar plataforma diferencial si existe
        if hasattr(self, 'diff_platform_vars'):
            platform_pos = step['platform_position']
            self.diff_platform_vars['position']['x'].set(platform_pos['x'])
            self.diff_platform_vars['position']['y'].set(platform_pos['y'])
            self.diff_platform_vars['position']['z'].set(platform_pos['z'])
            
            platform_rot = step['platform_rotation']
            self.diff_platform_vars['rotation']['roll'].set(platform_rot['roll'])
            self.diff_platform_vars['rotation']['pitch'].set(platform_rot['pitch'])
            self.diff_platform_vars['rotation']['yaw'].set(platform_rot['yaw'])
        
        # Guardar el índice del paso que estamos editando
        self.diff_current_step_index = index
        
        # Actualizar visualización
        self.update_differential_visualization()
        self.update_differential_position_display(self.differential_leg_selected.get())
        
        # Resaltar paso en el listbox
        self.diff_steps_listbox.selection_clear(0, tk.END)
        self.diff_steps_listbox.selection_set(index)
        self.diff_steps_listbox.see(index)
        
        messagebox.showinfo("Paso Cargado", f"Paso {step_number} cargado para edición")

    def ask_differential_step_number(self):
        """Diálogo para pedir número de paso en diferencial"""
        while True:
            result = tk.simpledialog.askinteger(
                "Número de Paso", 
                f"Ingresa el número de paso a editar (1-{len(self.differential_routine_steps)}):",
                parent=self.root,
                minvalue=1, 
                maxvalue=len(self.differential_routine_steps)
            )
            
            if result is None:
                return None
            elif 1 <= result <= len(self.differential_routine_steps):
                return result
            else:
                messagebox.showerror("Error", f"Número inválido. Debe ser entre 1 y {len(self.differential_routine_steps)}")

    def execute_differential_step(self):
        """Ejecutar paso en cinemática diferencial"""
        selected = self.diff_steps_listbox.curselection()
        if not selected:
            messagebox.showwarning("Advertencia", "Selecciona un paso para ejecutar")
            return
        
        index = selected[0]
        self.execute_differential_single_step(index)

    def execute_differential_single_step(self, index):
        """Ejecutar un paso específico en diferencial - VERSIÓN MEJORADA"""
        if 0 <= index < len(self.differential_routine_steps):
            step = self.differential_routine_steps[index]
            
            # Actualizar posiciones de patas
            for i, leg_pos in enumerate(step['leg_positions']):
                self.current_leg_positions[i] = [
                    leg_pos['x'],
                    leg_pos['y'],
                    leg_pos['z']
                ]
            
            # Actualizar plataforma diferencial
            if hasattr(self, 'diff_platform_vars'):
                platform_pos = step['platform_position']
                platform_rot = step['platform_rotation']
                
                # Actualizar variables
                self.diff_platform_vars['position']['x'].set(platform_pos['x'])
                self.diff_platform_vars['position']['y'].set(platform_pos['y'])
                self.diff_platform_vars['position']['z'].set(platform_pos['z'])
                
                self.diff_platform_vars['rotation']['roll'].set(platform_rot['roll'])
                self.diff_platform_vars['rotation']['pitch'].set(platform_rot['pitch'])
                self.diff_platform_vars['rotation']['yaw'].set(platform_rot['yaw'])
                
                # Actualizar etiquetas inmediatamente
                self.diff_platform_labels[f'pos_x'].config(text=f"{platform_pos['x']:.3f}")
                self.diff_platform_labels[f'pos_y'].config(text=f"{platform_pos['y']:.3f}")
                self.diff_platform_labels[f'pos_z'].config(text=f"{platform_pos['z']:.3f}")
                
                self.diff_platform_labels[f'rot_roll'].config(text=f"{platform_rot['roll']:.1f}")
                self.diff_platform_labels[f'rot_pitch'].config(text=f"{platform_rot['pitch']:.1f}")
                self.diff_platform_labels[f'rot_yaw'].config(text=f"{platform_rot['yaw']:.1f}")
            
            # Actualizar visualización
            self.update_differential_visualization()
            self.update_differential_position_display(self.differential_leg_selected.get())
            
            # Resaltar paso actual
            self.diff_steps_listbox.selection_clear(0, tk.END)
            self.diff_steps_listbox.selection_set(index)
            self.diff_steps_listbox.see(index)
            
            print(f"🔧 Ejecutando paso diferencial {index + 1}")

    def execute_differential_routine(self):
        """Ejecutar rutina completa en diferencial con interpolación - VERSIÓN OPTIMIZADA"""
        if not self.differential_routine_steps:
            messagebox.showwarning("Advertencia", "No hay pasos en la rutina diferencial")
            return
        
        if self.diff_routine_executing:
            messagebox.showwarning("Advertencia", "Ya hay una rutina diferencial en ejecución")
            return
        
        # Obtener parámetros de velocidad
        base_delay = self.diff_step_delay_ms.get()
        adjusted_delay = max(50, int(base_delay / self.global_speed.get()))
        intermediate_steps = max(0, self.diff_intermediate_steps.get())
        
        # Iniciar ejecución
        self.diff_stop_routine_flag = False
        self.diff_routine_executing = True
        
        def diff_routine_thread():
            try:
                for step_index in range(len(self.differential_routine_steps)):
                    if self.diff_stop_routine_flag:
                        break
                    
                    # Ejecutar paso actual
                    self.root.after(0, lambda idx=step_index: self.execute_differential_single_step(idx))
                    
                    # Pasos intermedios si no es el último paso
                    if intermediate_steps > 0 and step_index < len(self.differential_routine_steps) - 1:
                        if not self.execute_differential_intermediate_steps_optimized(step_index, step_index + 1, intermediate_steps):
                            break
                    
                    # Delay entre pasos principales
                    if step_index < len(self.differential_routine_steps) - 1:
                        if not self.diff_smart_delay(adjusted_delay):
                            break
                
                # Finalizar ejecución
                self.root.after(0, self.finish_differential_routine_execution)
                
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("Error", f"Error en rutina diferencial: {e}"))
                self.root.after(0, self.finish_differential_routine_execution)
        
        # Ejecutar en hilo
        self.current_diff_routine_thread = threading.Thread(target=diff_routine_thread, daemon=True)
        self.current_diff_routine_thread.start()
        
        # Habilitar botón de detener
        self.enable_diff_stop_button()

    def finish_differential_routine_execution(self):
        """Finalizar ejecución de rutina diferencial"""
        self.diff_routine_executing = False
        self.diff_stop_routine_flag = False
        
        if not self.diff_stop_routine_flag:
            self.root.after(0, lambda: messagebox.showinfo("Rutina Completada", "Todos los pasos diferenciales ejecutados"))
        
        self.disable_diff_stop_button()

    def execute_differential_intermediate_steps_optimized(self, current_index, next_index, num_steps):
        """Ejecutar pasos intermedios optimizados para diferencial"""
        if current_index >= len(self.differential_routine_steps) or next_index >= len(self.differential_routine_steps):
            return True
        
        if self.diff_stop_routine_flag:
            return False
        
        current_step = self.differential_routine_steps[current_index]
        next_step = self.differential_routine_steps[next_index]
        
        # Calcular todos los pasos intermedios primero
        intermediate_positions = []
        for step_num in range(1, num_steps + 1):
            if self.diff_stop_routine_flag:
                return False
                
            alpha = step_num / (num_steps + 1)
            positions = self.calculate_differential_intermediate_positions(current_step, next_step, alpha)
            intermediate_positions.append(positions)
        
        # Ejecutar pasos con delay optimizado
        base_interpolation_delay = max(20, int(100 / self.global_speed.get()))
        
        for positions in intermediate_positions:
            if self.diff_stop_routine_flag:
                return False
                
            # Aplicar posiciones calculadas
            self.apply_differential_intermediate_positions(positions)
            
            # Delay breve
            if not self.diff_smart_delay(base_interpolation_delay):
                return False
        
        return True

    def calculate_differential_intermediate_positions(self, current_step, next_step, alpha):
        """Calcular posiciones intermedias para diferencial"""
        positions = {
            'leg_positions': [],
            'platform_pos': {},
            'platform_rot': {}
        }
        
        # Interpolar posiciones de patas
        for i in range(4):
            current_leg = current_step['leg_positions'][i]
            next_leg = next_step['leg_positions'][i]
            
            x = current_leg['x'] + (next_leg['x'] - current_leg['x']) * alpha
            y = current_leg['y'] + (next_leg['y'] - current_leg['y']) * alpha
            z = current_leg['z'] + (next_leg['z'] - current_leg['z']) * alpha
            
            positions['leg_positions'].append((i, x, y, z))
        
        # Interpolar plataforma diferencial
        current_pos = current_step['platform_position']
        next_pos = next_step['platform_position']
        current_rot = current_step['platform_rotation']
        next_rot = next_step['platform_rotation']
        
        positions['platform_pos'] = {
            'x': current_pos['x'] + (next_pos['x'] - current_pos['x']) * alpha,
            'y': current_pos['y'] + (next_pos['y'] - current_pos['y']) * alpha,
            'z': current_pos['z'] + (next_pos['z'] - current_pos['z']) * alpha
        }
        
        positions['platform_rot'] = {
            'roll': current_rot['roll'] + (next_rot['roll'] - current_rot['roll']) * alpha,
            'pitch': current_rot['pitch'] + (next_rot['pitch'] - current_rot['pitch']) * alpha,
            'yaw': current_rot['yaw'] + (next_rot['yaw'] - current_rot['yaw']) * alpha
        }
        
        return positions

    def apply_differential_intermediate_positions(self, positions):
        """Aplicar posiciones calculadas a la cinemática diferencial"""
        # Aplicar posiciones de patas
        for leg_idx, x, y, z in positions['leg_positions']:
            self.current_leg_positions[leg_idx] = [x, y, z]
        
        # Aplicar plataforma diferencial
        platform_pos = positions['platform_pos']
        platform_rot = positions['platform_rot']
        
        if hasattr(self, 'diff_platform_vars'):
            for coord, value in platform_pos.items():
                self.diff_platform_vars['position'][coord].set(value)
                if f'pos_{coord}' in self.diff_platform_labels:
                    self.diff_platform_labels[f'pos_{coord}'].config(text=f"{value:.3f}")
            
            for ang, value in platform_rot.items():
                self.diff_platform_vars['rotation'][ang].set(value)
                if f'rot_{ang}' in self.diff_platform_labels:
                    self.diff_platform_labels[f'rot_{ang}'].config(text=f"{value:.1f}")
        
        # Actualizar visualización diferencial
        self.update_differential_visualization()
        self.update_differential_position_display(self.differential_leg_selected.get())

    def clear_differential_routine(self):
        """Borrar rutina en diferencial"""
        if not self.differential_routine_steps:
            messagebox.showinfo("Información", "No hay pasos en la rutina diferencial para borrar")
            return
        
        if messagebox.askyesno("Confirmar", "¿Estás seguro de que quieres borrar TODA la rutina diferencial?"):
            self.differential_routine_steps.clear()
            self.update_differential_steps_display()
            messagebox.showinfo("Rutina Borrada", "Toda la rutina diferencial ha sido borrada exitosamente")

    def save_differential_routine(self):
        """Guardar rutina diferencial completa en archivo JSON"""
        try:
            if not self.differential_routine_steps:
                messagebox.showwarning("Advertencia", "No hay pasos en la rutina diferencial para guardar")
                return
            
            from tkinter import filedialog
            file_path = filedialog.asksaveasfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Guardar rutina diferencial como"
            )
            
            if file_path:
                routine_data = {
                    'routine_name': os.path.basename(file_path),
                    'created': time.time(),
                    'execution_parameters': {
                        'step_delay_ms': self.diff_step_delay_ms.get(),
                        'intermediate_steps': self.diff_intermediate_steps.get()
                    },
                    'steps': self.differential_routine_steps
                }
                
                with open(file_path, 'w') as f:
                    json.dump(routine_data, f, indent=4)
                
                messagebox.showinfo("Rutina Guardada", f"Rutina diferencial guardada en:\n{file_path}")
                
        except Exception as e:
            messagebox.showerror("Error", f"Error al guardar rutina diferencial: {e}")

    def load_differential_routine(self):
        """Cargar rutina diferencial desde archivo JSON"""
        try:
            from tkinter import filedialog
            file_path = filedialog.askopenfilename(
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Cargar rutina diferencial"
            )
            
            if file_path and os.path.exists(file_path):
                with open(file_path, 'r') as f:
                    routine_data = json.load(f)
                
                self.differential_routine_steps = routine_data.get('steps', [])
                
                # Cargar parámetros de ejecución si existen
                execution_params = routine_data.get('execution_parameters', {})
                if 'step_delay_ms' in execution_params:
                    self.diff_step_delay_ms.set(execution_params['step_delay_ms'])
                if 'intermediate_steps' in execution_params:
                    self.diff_intermediate_steps.set(execution_params['intermediate_steps'])
                
                self.update_differential_steps_display()
                
                messagebox.showinfo("Rutina Cargada", f"Rutina diferencial cargada desde:\n{file_path}")
                
        except Exception as e:
            messagebox.showerror("Error", f"Error al cargar rutina diferencial: {e}")

    def update_differential_steps_display(self):
        """Actualizar display de pasos en diferencial"""
        self.diff_steps_listbox.delete(0, tk.END)
        for i, step in enumerate(self.differential_routine_steps):
            try:
                angle_strings = []
                for leg in step['angles']:
                    leg_angles = []
                    for ang in leg[:2]:
                        try:
                            leg_angles.append(f"{float(ang):.1f}")
                        except (ValueError, TypeError):
                            leg_angles.append("0.0")
                    angle_strings.extend(leg_angles)
                
                angles_str = " | ".join(angle_strings[:6])
                self.diff_steps_listbox.insert(tk.END, f"Paso {i+1}: {angles_str}...")
            except Exception as e:
                self.diff_steps_listbox.insert(tk.END, f"Paso {i+1}: [ángulos guardados]")

    # =============================================================================
    # MÉTODOS EXISTENTES DE CINEMÁTICA INVERSA 
    # =============================================================================

    def setup_joystick_controls(self, parent):
        """Configurar controles de joystick para roll y altura"""
        joystick_frame = ttk.LabelFrame(parent, text="Joystick - Control de Movimiento")
        joystick_frame.pack(fill=tk.X, pady=5)
        
        # Checkbox para seleccionar modo de control
        mode_frame = ttk.Frame(joystick_frame)
        mode_frame.pack(fill=tk.X, pady=5)
        
        self.joystick_mode = tk.StringVar(value="yaw_height")
        
        ttk.Radiobutton(mode_frame, text="Yaw y Altura",
                    variable=self.joystick_mode, value="yaw_height",
                    command=self.on_joystick_mode_change).pack(side=tk.LEFT, padx=10)
        
        ttk.Radiobutton(mode_frame, text="Roll y Pitch", 
                    variable=self.joystick_mode, value="roll_pitch",
                    command=self.on_joystick_mode_change).pack(side=tk.LEFT, padx=10)
        
        ttk.Radiobutton(mode_frame, text="XY Position", 
                    variable=self.joystick_mode, value="xy_position",
                    command=self.on_joystick_mode_change).pack(side=tk.LEFT, padx=10)
        
        # Frame para sensibilidad
        sensitivity_frame = ttk.Frame(joystick_frame)
        sensitivity_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(sensitivity_frame, text="Sensibilidad:").pack(side=tk.LEFT)
        sensitivity_scale = ttk.Scale(sensitivity_frame, from_=0.1, to=5.0, 
                                    orient=tk.HORIZONTAL, variable=self.joystick_sensitivity)
        sensitivity_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Label(sensitivity_frame, textvariable=self.joystick_sensitivity).pack(side=tk.RIGHT)
        
        # Frame principal para el joystick
        joystick_main_frame = ttk.Frame(joystick_frame)
        joystick_main_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # Canvas para dibujar el joystick
        self.joystick_canvas = tk.Canvas(joystick_main_frame, width=200, height=200, 
                                        bg='lightgray', relief='sunken', bd=2)
        self.joystick_canvas.pack(pady=10)

        # Variables del joystick
        self.joystick_handle = None
        self.joystick_center_x = 100
        self.joystick_center_y = 100
        self.joystick_radius = 80
        self.handle_radius = 15

        # Dibujar el joystick base
        self.draw_joystick_base()
        
        # Etiquetas para valores actuales
        values_frame = ttk.Frame(joystick_main_frame)
        values_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(values_frame, text="Roll:").pack(side=tk.LEFT, padx=10)
        self.roll_value_label = ttk.Label(values_frame, text="0.0°", width=8)
        self.roll_value_label.pack(side=tk.LEFT, padx=5)
        
        # Etiqueta dinámica para el segundo control
        self.second_control_label = ttk.Label(values_frame, text="Pitch:", width=6)
        self.second_control_label.pack(side=tk.LEFT, padx=10)
        self.second_value_label = ttk.Label(values_frame, text="0.0°", width=8)
        self.second_value_label.pack(side=tk.LEFT, padx=5)
        
        # Botón para resetear
        ttk.Button(joystick_main_frame, text="Resetear Joystick", 
                command=self.reset_joystick).pack(pady=5)
        
        # Bind eventos del mouse
        self.joystick_canvas.bind("<B1-Motion>", self.on_joystick_drag)
        self.joystick_canvas.bind("<ButtonRelease-1>", self.on_joystick_release)
        
        # Dibujar handle inicial
        self.draw_joystick_handle(0, 0)

    def on_joystick_release(self, event):
        """Manejar la liberación del joystick - MANTENER POSICIÓN ACTUAL"""
        # NO hacer nada - mantener la posición actual del joystick
        # Solo actualizar la visualización si es necesario
        self.apply_joystick_to_platform()
        print(f"🎮 Joystick liberado - Manteniendo posición: Roll={self.joystick_roll.get():.1f}°, Pitch={self.joystick_pitch.get():.1f}°")

    def draw_joystick_handle(self, dx, dy):
        """Dibujar el handle del joystick en la posición especificada"""
        # Eliminar handle anterior si existe
        if hasattr(self, 'joystick_handle'):
            self.joystick_canvas.delete(self.joystick_handle)
        
        # Calcular posición del handle
        handle_x = self.joystick_center_x + dx
        handle_y = self.joystick_center_y + dy
        
        # Dibujar nuevo handle
        self.joystick_handle = self.joystick_canvas.create_oval(
            handle_x - self.handle_radius,
            handle_y - self.handle_radius,
            handle_x + self.handle_radius,
            handle_y + self.handle_radius,
            fill="blue", outline="darkblue", width=2, tags="handle"
        )

    def draw_joystick_base(self):
        """Dibujar la base del joystick con etiquetas dinámicas"""
        self.joystick_canvas.delete("base")
        
        # Dibujar círculo exterior
        self.joystick_canvas.create_oval(
            self.joystick_center_x - self.joystick_radius,
            self.joystick_center_y - self.joystick_radius,
            self.joystick_center_x + self.joystick_radius,
            self.joystick_center_y + self.joystick_radius,
            outline="darkgray", width=2, tags="base"
        )
        
        # Dibujar ejes cruzados
        self.joystick_canvas.create_line(
            self.joystick_center_x, self.joystick_center_y - self.joystick_radius,
            self.joystick_center_x, self.joystick_center_y + self.joystick_radius,
            fill="gray", width=1, tags="base"
        )
        self.joystick_canvas.create_line(
            self.joystick_center_x - self.joystick_radius, self.joystick_center_y,
            self.joystick_center_x + self.joystick_radius, self.joystick_center_y,
            fill="gray", width=1, tags="base"
        )
        
        # Etiquetas de ejes DINÁMICAS según el modo
        mode = self.joystick_mode.get()
        
        if mode == "yaw_height":
            # Modo Yaw y Altura
            self.joystick_canvas.create_text(
                self.joystick_center_x, self.joystick_center_y - self.joystick_radius - 10,
                text="Altura+", fill="green", tags="base"
            )
            self.joystick_canvas.create_text(
                self.joystick_center_x, self.joystick_center_y + self.joystick_radius + 10,
                text="Altura-", fill="green", tags="base"
            )
            self.joystick_canvas.create_text(
                self.joystick_center_x - self.joystick_radius - 15, self.joystick_center_y,
                text="Yaw-", fill="purple", tags="base"
            )
            self.joystick_canvas.create_text(
                self.joystick_center_x + self.joystick_radius + 15, self.joystick_center_y,
                text="Yaw+", fill="purple", tags="base"
            )
        
        elif mode == "roll_pitch":
            # Modo Roll y Pitch
            self.joystick_canvas.create_text(
                self.joystick_center_x, self.joystick_center_y - self.joystick_radius - 10,
                text="Pitch+", fill="blue", tags="base"
            )
            self.joystick_canvas.create_text(
                self.joystick_center_x, self.joystick_center_y + self.joystick_radius + 10,
                text="Pitch-", fill="blue", tags="base"
            )
            self.joystick_canvas.create_text(
                self.joystick_center_x - self.joystick_radius - 15, self.joystick_center_y,
                text="Roll-", fill="red", tags="base"
            )
            self.joystick_canvas.create_text(
                self.joystick_center_x + self.joystick_radius + 15, self.joystick_center_y,
                text="Roll+", fill="red", tags="base"
            )
        
        else:  # Modo xy_position
            # Modo XY Position
            self.joystick_canvas.create_text(
                self.joystick_center_x, self.joystick_center_y - self.joystick_radius - 10,
                text="Y+", fill="orange", tags="base"
            )
            self.joystick_canvas.create_text(
                self.joystick_center_x, self.joystick_center_y + self.joystick_radius + 10,
                text="Y-", fill="orange", tags="base"
            )
            self.joystick_canvas.create_text(
                self.joystick_center_x - self.joystick_radius - 15, self.joystick_center_y,
                text="X-", fill="brown", tags="base"
            )
            self.joystick_canvas.create_text(
                self.joystick_center_x + self.joystick_radius + 15, self.joystick_center_y,
                text="X+", fill="brown", tags="base"
            )

    def on_joystick_mode_change(self):
        """Cambiar entre modos de control del joystick MANTENIENDO LOS VALORES"""
        old_mode = self.joystick_mode.get()  # Guardar el modo actual antes de cambiar
        new_mode = self.joystick_mode.get()  # El nuevo modo ya está establecido por el RadioButton
        
        print(f"🎮 Cambiando modo: {old_mode} -> {new_mode}")
        
        # Guardar valores actuales del modo anterior
        if old_mode == "yaw_height":
            self.joystick_values[old_mode]["yaw"] = getattr(self, 'joystick_yaw', 0.0)
            self.joystick_values[old_mode]["height"] = getattr(self, 'joystick_height', 0.18)
        elif old_mode == "roll_pitch":
            self.joystick_values[old_mode]["roll"] = self.joystick_roll.get()
            self.joystick_values[old_mode]["pitch"] = self.joystick_pitch.get()
        elif old_mode == "xy_position":  # NUEVO: Guardar valores XY
            self.joystick_values[old_mode]["x"] = getattr(self, 'joystick_x', 0.0)
            self.joystick_values[old_mode]["y"] = getattr(self, 'joystick_y', 0.0)
        
        # Actualizar variables internas con los valores guardados del NUEVO modo
        if new_mode == "yaw_height":
            self.joystick_yaw = self.joystick_values[new_mode]["yaw"]
            self.joystick_height = self.joystick_values[new_mode]["height"]
        elif new_mode == "roll_pitch":
            self.joystick_roll.set(self.joystick_values[new_mode]["roll"])
            self.joystick_pitch.set(self.joystick_values[new_mode]["pitch"])
        elif new_mode == "xy_position":  # NUEVO: Cargar valores XY
            self.joystick_x = self.joystick_values[new_mode]["x"]
            self.joystick_y = self.joystick_values[new_mode]["y"]
        
        # ACTUALIZAR ETIQUETAS PRIMERO
        if new_mode == "yaw_height":
            self.roll_value_label.config(text=f"{self.joystick_values[new_mode]['yaw']:.1f}°")
            self.second_control_label.config(text="Altura:")
            self.second_value_label.config(text=f"{self.joystick_values[new_mode]['height']:.3f} m")
            print(f"🎮 Modo: Yaw y Altura (Yaw: {self.joystick_values[new_mode]['yaw']:.1f}°, Altura: {self.joystick_values[new_mode]['height']:.3f}m)")
        elif new_mode == "roll_pitch":
            self.roll_value_label.config(text=f"{self.joystick_values[new_mode]['roll']:.1f}°")
            self.second_control_label.config(text="Pitch:")
            self.second_value_label.config(text=f"{self.joystick_values[new_mode]['pitch']:.1f}°")
            print(f"🎮 Modo: Roll y Pitch (Roll: {self.joystick_values[new_mode]['roll']:.1f}°, Pitch: {self.joystick_values[new_mode]['pitch']:.1f}°)")
        else:  # Modo xy_position
            self.roll_value_label.config(text=f"{self.joystick_values[new_mode]['x']:.3f} m")
            self.second_control_label.config(text="Y:")
            self.second_value_label.config(text=f"{self.joystick_values[new_mode]['y']:.3f} m")
            print(f"🎮 Modo: XY Position (X: {self.joystick_values[new_mode]['x']:.3f}m, Y: {self.joystick_values[new_mode]['y']:.3f}m)")
        
        # Redibujar base para actualizar etiquetas
        self.draw_joystick_base()
        
        # Actualizar posición visual del joystick basado en los valores guardados
        self.update_joystick_handle_from_values()
        
        # Aplicar los valores a la plataforma
        self.apply_joystick_to_platform()

    def update_joystick_handle_from_values(self):
        """Actualizar la posición visual del joystick basado en los valores actuales"""
        mode = self.joystick_mode.get()
        
        if mode == "yaw_height":
            # Convertir yaw y altura a coordenadas del joystick
            yaw_normalized = self.joystick_values[mode]["yaw"] / 180.0  # ±180° -> ±1.0
            height_normalized = (self.joystick_values[mode]["height"] - 0.18) / 0.07  # 0.18±0.07 -> ±1.0
            
            # Limitar a [-1, 1]
            yaw_normalized = max(-1.0, min(1.0, yaw_normalized))
            height_normalized = max(-1.0, min(1.0, height_normalized))
            
            dx = yaw_normalized * self.joystick_radius
            dy = -height_normalized * self.joystick_radius  # Invertir Y
            
        elif mode == "roll_pitch":
            # Convertir roll y pitch a coordenadas del joystick
            roll_normalized = self.joystick_values[mode]["roll"] / 30.0  # ±30° -> ±1.0
            pitch_normalized = self.joystick_values[mode]["pitch"] / 30.0  # ±30° -> ±1.0
            
            dx = roll_normalized * self.joystick_radius
            dy = -pitch_normalized * self.joystick_radius  # Invertir Y
        
        else:  # Modo xy_position
            # Convertir X e Y a coordenadas del joystick
            # Rango: X: ±0.25m, Y: ±0.25m
            x_normalized = self.joystick_values[mode]["x"] / 0.25  # ±0.25m -> ±1.0
            y_normalized = self.joystick_values[mode]["y"] / 0.25  # ±0.25m -> ±1.0
            
            # Limitar a [-1, 1]
            x_normalized = max(-1.0, min(1.0, x_normalized))
            y_normalized = max(-1.0, min(1.0, y_normalized))
            
            dx = x_normalized * self.joystick_radius
            dy = -y_normalized * self.joystick_radius  # Invertir Y
        
        # Dibujar handle en la posición calculada
        self.draw_joystick_handle(dx, dy)

    def on_joystick_drag(self, event):
        """Manejar el arrastre del joystick para todos los modos"""
        # Calcular offset desde el centro
        dx = event.x - self.joystick_center_x
        dy = event.y - self.joystick_center_y
        
        # Limitar al radio del joystick
        distance = (dx**2 + dy**2)**0.5
        if distance > self.joystick_radius:
            dx = dx * self.joystick_radius / distance
            dy = dy * self.joystick_radius / distance
        
        # Actualizar posición del handle
        self.draw_joystick_handle(dx, dy)
        
        # Calcular valores normalizados a [-1, 1]
        horizontal_normalized = dx / self.joystick_radius
        vertical_normalized = -dy / self.joystick_radius  # Invertir Y para intuición natural
        
        # Aplicar sensibilidad
        sensitivity = self.joystick_sensitivity.get()
        
        # Controlar según el modo
        mode = self.joystick_mode.get()
        
        if mode == "yaw_height":
            # Controlar yaw (rotación horizontal)
            yaw_degrees = horizontal_normalized * 180 * sensitivity  # ±180 grados máximo para yaw
            self.joystick_yaw = yaw_degrees
            self.roll_value_label.config(text=f"{yaw_degrees:.1f}°")
            
            # Controlar altura (coordenada Z de la plataforma)
            # Rango de altura: 0.10m a 0.25m (centro en 0.18m)
            height_range = 0.07  # ±7cm desde el centro
            base_height = 0.18   # Altura base (centro)
            
            height_change = vertical_normalized * height_range * sensitivity
            new_height = base_height + height_change
            
            # Limitar altura
            new_height = max(0.10, min(0.25, new_height))
            
            # Actualizar variable y etiqueta
            self.joystick_height = new_height
            self.second_value_label.config(text=f"{new_height:.3f} m")
            
            # GUARDAR VALORES PARA ESTE MODO
            self.joystick_values[mode]["yaw"] = yaw_degrees
            self.joystick_values[mode]["height"] = new_height
            
            print(f"🎮 Yaw: {yaw_degrees:.1f}°, Altura: {new_height:.3f} m")
            
        elif mode == "roll_pitch":
            # Controlar roll y pitch (modo original)
            roll_degrees = horizontal_normalized * 30 * sensitivity  # ±30 grados máximo
            pitch_degrees = vertical_normalized * 30 * sensitivity  # ±30 grados máximo
            
            self.joystick_roll.set(roll_degrees)
            self.joystick_pitch.set(pitch_degrees)
            
            self.roll_value_label.config(text=f"{roll_degrees:.1f}°")
            self.second_value_label.config(text=f"{pitch_degrees:.1f}°")
            
            # GUARDAR VALORES PARA ESTE MODO
            self.joystick_values[mode]["roll"] = roll_degrees
            self.joystick_values[mode]["pitch"] = pitch_degrees
            
            print(f"🎮 Roll: {roll_degrees:.1f}°, Pitch: {pitch_degrees:.1f}°")
        
        else:  # Modo xy_position
            # Controlar posición X e Y de la plataforma
            # Rango: ±0.25 metros
            position_range = 0.25  # ±25cm desde el centro
            
            x_position = horizontal_normalized * position_range * sensitivity
            y_position = vertical_normalized * position_range * sensitivity
            
            # Actualizar variables y etiquetas
            self.joystick_x = x_position
            self.joystick_y = y_position
            
            self.roll_value_label.config(text=f"{x_position:.3f} m")
            self.second_value_label.config(text=f"{y_position:.3f} m")
            
            # GUARDAR VALORES PARA ESTE MODO
            self.joystick_values[mode]["x"] = x_position
            self.joystick_values[mode]["y"] = y_position
            
            print(f"🎮 X: {x_position:.3f} m, Y: {y_position:.3f} m")
        
        # Aplicar a la plataforma
        self.apply_joystick_to_platform()

    def apply_joystick_to_platform(self):
        """Aplicar los valores del joystick a la plataforma del robot según el modo"""
        mode = self.joystick_mode.get()
        
        if mode == "yaw_height":
            # Usar valores almacenados en lugar de variables temporales
            yaw_degrees = self.joystick_values[mode]["yaw"]
            new_height = self.joystick_values[mode]["height"]
            
            # Limitar yaw
            yaw_degrees = max(min(yaw_degrees, 180.0), -180.0)
            
            # Aplicar yaw a la plataforma
            self.platform_vars['rotation']['yaw'].set(yaw_degrees)
            self.platform_vars['rotation']['yaw_label'].config(text=f"{yaw_degrees:.1f}")
            
            # Actualizar campo de entrada de yaw
            if 'yaw' in self.platform_vars['rotation_entries']:
                self.platform_vars['rotation_entries']['yaw'].delete(0, tk.END)
                self.platform_vars['rotation_entries']['yaw'].insert(0, f"{yaw_degrees:.1f}")
            
            # Limitar altura
            new_height = max(0.10, min(0.25, new_height))
            
            # Aplicar a la plataforma
            self.platform_vars['position']['z'].set(new_height)
            self.platform_vars['position']['z_label'].config(text=f"{new_height:.3f}")
            
            # Actualizar campo de entrada de altura
            if 'z' in self.platform_vars['position_entries']:
                self.platform_vars['position_entries']['z'].delete(0, tk.END)
                self.platform_vars['position_entries']['z'].insert(0, f"{new_height:.3f}")
                
        elif mode == "roll_pitch":
            # Usar valores almacenados en lugar de variables temporales
            new_roll = self.joystick_values[mode]["roll"]
            new_pitch = self.joystick_values[mode]["pitch"]
            
            # Limitar roll y pitch
            new_roll = max(min(new_roll, 30.0), -30.0)
            new_pitch = max(min(new_pitch, 30.0), -30.0)
            
            # Aplicar roll
            self.platform_vars['rotation']['roll'].set(new_roll)
            self.platform_vars['rotation']['roll_label'].config(text=f"{new_roll:.1f}")
            
            # Aplicar pitch
            self.platform_vars['rotation']['pitch'].set(new_pitch)
            self.platform_vars['rotation']['pitch_label'].config(text=f"{new_pitch:.1f}")
            
            # Actualizar campos de entrada
            if 'roll' in self.platform_vars['rotation_entries']:
                self.platform_vars['rotation_entries']['roll'].delete(0, tk.END)
                self.platform_vars['rotation_entries']['roll'].insert(0, f"{new_roll:.1f}")
            
            if 'pitch' in self.platform_vars['rotation_entries']:
                self.platform_vars['rotation_entries']['pitch'].delete(0, tk.END)
                self.platform_vars['rotation_entries']['pitch'].insert(0, f"{new_pitch:.1f}")
        
        else:  # Modo xy_position
            # Usar valores almacenados en lugar de variables temporales
            x_position = self.joystick_values[mode]["x"]
            y_position = self.joystick_values[mode]["y"]
            
            # Limitar posición (mismos límites que los sliders)
            x_position = max(min(x_position, 0.25), -0.25)
            y_position = max(min(y_position, 0.25), -0.25)
            
            # Aplicar posición X
            self.platform_vars['position']['x'].set(x_position)
            self.platform_vars['position']['x_label'].config(text=f"{x_position:.3f}")
            
            # Aplicar posición Y
            self.platform_vars['position']['y'].set(y_position)
            self.platform_vars['position']['y_label'].config(text=f"{y_position:.3f}")
            
            # Actualizar campos de entrada
            if 'x' in self.platform_vars['position_entries']:
                self.platform_vars['position_entries']['x'].delete(0, tk.END)
                self.platform_vars['position_entries']['x'].insert(0, f"{x_position:.3f}")
            
            if 'y' in self.platform_vars['position_entries']:
                self.platform_vars['position_entries']['y'].delete(0, tk.END)
                self.platform_vars['position_entries']['y'].insert(0, f"{y_position:.3f}")
        
        # Actualizar visualización
        self.update_visualization()

    def reset_joystick(self):
        """Resetear el joystick a la posición central - SOLO cuando se presiona el botón"""
        if hasattr(self, 'joystick_handle') and hasattr(self, 'joystick_center_x'):
            # Mover handle visualmente al centro
            self.draw_joystick_handle(0, 0)
            
            mode = self.joystick_mode.get()
            
            if mode == "yaw_height":
                # Resetear valores almacenados
                self.joystick_values[mode]["yaw"] = 0.0
                self.joystick_values[mode]["height"] = 0.18
                
                # Actualizar variables internas
                self.joystick_yaw = 0.0
                self.joystick_height = 0.18
                
                self.roll_value_label.config(text="0.0°")
                self.second_value_label.config(text="0.180 m")
                
                print("🔄 Joystick RESETEADO - Yaw: 0.0°, Altura: 0.180 m")
            
            elif mode == "roll_pitch":
                # Resetear valores almacenados
                self.joystick_values[mode]["roll"] = 0.0
                self.joystick_values[mode]["pitch"] = 0.0
                
                # Actualizar variables internas
                self.joystick_roll.set(0.0)
                self.joystick_pitch.set(0.0)
                
                self.roll_value_label.config(text="0.0°")
                self.second_value_label.config(text="0.0°")
                
                print("🔄 Joystick RESETEADO - Roll: 0.0°, Pitch: 0.0°")
            
            else:  # Modo xy_position
                # Resetear valores almacenados
                self.joystick_values[mode]["x"] = 0.0
                self.joystick_values[mode]["y"] = 0.0
                
                # Actualizar variables internas
                self.joystick_x = 0.0
                self.joystick_y = 0.0
                
                self.roll_value_label.config(text="0.000 m")
                self.second_value_label.config(text="0.000 m")
                
                print("🔄 Joystick RESETEADO - X: 0.000 m, Y: 0.000 m")
            
            # Aplicar cambios a la plataforma
            self.apply_joystick_to_platform()





    def setup_gait_controls(self, parent):
        """Configurar controles de gaits"""
        gait_frame = ttk.LabelFrame(parent, text="Sistema de Gaits")
        gait_frame.pack(fill=tk.X, pady=5)
        
        # Selección de gait
        gait_select_frame = ttk.Frame(gait_frame)
        gait_select_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(gait_select_frame, text="Tipo de Gait:").pack(side=tk.LEFT, padx=5)
        
        gaits = [("Standing", "standing"), ("Crawling", "crawling"), 
                ("Walking", "walking"), ("Trotting", "trotting")]
        
        for text, value in gaits:
            ttk.Radiobutton(gait_select_frame, text=text, variable=self.active_gait_var,
                        value=value, command=self.on_gait_change).pack(side=tk.LEFT, padx=5)
        
        # Control de dirección y velocidad
        control_frame = ttk.Frame(gait_frame)
        control_frame.pack(fill=tk.X, pady=5)
        
        # Dirección X
        dir_x_frame = ttk.Frame(control_frame)
        dir_x_frame.pack(fill=tk.X, pady=2)
        ttk.Label(dir_x_frame, text="Dirección X:").pack(side=tk.LEFT, padx=5)
        ttk.Scale(dir_x_frame, from_=-1.0, to=1.0, orient=tk.HORIZONTAL,
                variable=self.gait_direction_x).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Label(dir_x_frame, textvariable=self.gait_direction_x, width=4).pack(side=tk.RIGHT, padx=5)
        
        # Dirección Y
        dir_y_frame = ttk.Frame(control_frame)
        dir_y_frame.pack(fill=tk.X, pady=2)
        ttk.Label(dir_y_frame, text="Dirección Y:").pack(side=tk.LEFT, padx=5)
        ttk.Scale(dir_y_frame, from_=-1.0, to=1.0, orient=tk.HORIZONTAL,
                variable=self.gait_direction_y).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Label(dir_y_frame, textvariable=self.gait_direction_y, width=4).pack(side=tk.RIGHT, padx=5)

        # Giro (positivo = antihorario)
        turn_frame = ttk.Frame(control_frame)
        turn_frame.pack(fill=tk.X, pady=2)
        ttk.Label(turn_frame, text="Giro:").pack(side=tk.LEFT, padx=5)
        ttk.Scale(turn_frame, from_=-1.0, to=1.0, orient=tk.HORIZONTAL,
                variable=self.gait_turn).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Label(turn_frame, textvariable=self.gait_turn, width=4).pack(side=tk.RIGHT, padx=5)

        # Velocidad
        speed_frame = ttk.Frame(control_frame)
        speed_frame.pack(fill=tk.X, pady=2)
        ttk.Label(speed_frame, text="Velocidad:").pack(side=tk.LEFT, padx=5)
        ttk.Scale(speed_frame, from_=0.1, to=3.0, orient=tk.HORIZONTAL,
                variable=self.gait_speed_var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Label(speed_frame, textvariable=self.gait_speed_var, width=4).pack(side=tk.RIGHT, padx=5)
        
        # Botones de control
        button_frame = ttk.Frame(gait_frame)
        button_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(button_frame, text="Iniciar Gait", 
                command=self.start_gait).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Detener Gait", 
                command=self.stop_gait).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Configurar Parámetros", 
                command=self.configure_gait_parameters).pack(side=tk.RIGHT, padx=5)
        
        # Estado del gait
        self.gait_status_var = tk.StringVar(value="Gait: Inactivo")
        ttk.Label(gait_frame, textvariable=self.gait_status_var, 
                font=('Arial', 10, 'bold'), foreground="red").pack(pady=2)

    def on_gait_change(self):
        """Manejar cambio de gait seleccionado"""
        gait_name = self.active_gait_var.get()
        if self.gait_manager.set_gait(gait_name):
            self.gait_status_var.set(f"Gait: {gait_name.capitalize()} (Listo)")
        
        # Si es "standing", detener cualquier movimiento
        if gait_name == "standing":
            self.stop_gait()

    def start_gait(self):
        """Iniciar el gait seleccionado"""
        direction = (self.gait_direction_x.get(), self.gait_direction_y.get())
        speed = self.gait_speed_var.get()
        
        if self.gait_manager.start_gait(direction, speed, turn=self.gait_turn.get()):
            self.gait_status_var.set(f"Gait: {self.active_gait_var.get().capitalize()} (Activo)")
            # Iniciar actualización de visualización solo si el gait se inició correctamente
            self.start_gait_visualization_update()

    def stop_gait(self):
        """Detener el gait y activar modo standing con retorno a posiciones iniciales"""
        self.gait_manager.stop_gait()
        
        # Sincronizar las posiciones actuales con los controles de la interfaz
        current_positions = self.gait_manager.get_current_positions()
        for i, pos in enumerate(current_positions):
            if i < len(self.leg_vars):
                self.leg_vars[i]['x'].set(pos[0])
                self.leg_vars[i]['y'].set(pos[1])
                self.leg_vars[i]['z'].set(pos[2])
                # Actualizar etiquetas
                self.leg_vars[i]['x_label'].config(text=f"{pos[0]:.3f}")
                self.leg_vars[i]['y_label'].config(text=f"{pos[1]:.3f}")
                self.leg_vars[i]['z_label'].config(text=f"{pos[2]:.3f}")
                # Actualizar campos de entrada
                self.leg_vars[i]['x_entry'].delete(0, tk.END)
                self.leg_vars[i]['x_entry'].insert(0, f"{pos[0]:.3f}")
                self.leg_vars[i]['y_entry'].delete(0, tk.END)
                self.leg_vars[i]['y_entry'].insert(0, f"{pos[1]:.3f}")
                self.leg_vars[i]['z_entry'].delete(0, tk.END)
                self.leg_vars[i]['z_entry'].insert(0, f"{pos[2]:.3f}")
        
        # Cambiar el gait activo a standing
        self.active_gait_var.set("standing")
        self.gait_manager.set_gait("standing")
        self.gait_status_var.set("Gait: Standing (Posición Inicial)")
        
        # Actualizar visualización
        self.update_visualization()
        
        print("✅ Modo standing activado - Posiciones iniciales restauradas")
    
    def get_gait_parameter_description(self, param):
        """Obtener descripción de los parámetros del gait"""
        descriptions = {
            'step_height': 'Altura máxima del paso (metros)',
            'step_length': 'Longitud del paso (metros)', 
            'cycle_time': 'Tiempo para completar un ciclo (segundos)',
            'duty_factor': 'Proporción de tiempo que la pata está en el suelo (0-1)',
            'phase_offset': 'Desfase temporal entre patas (0-1)'
        }
        return descriptions.get(param, "Parámetro del gait")

    def configure_gait_parameters(self):
        """Diálogo para configurar parámetros avanzados del gait"""
        gait_name = self.active_gait_var.get()
        
        if gait_name not in self.gait_manager.gait_parameters:
            messagebox.showerror("Error", f"Gait {gait_name} no encontrado")
            return
        
        params = self.gait_manager.gait_parameters[gait_name]
        
        # Crear ventana de configuración
        config_window = tk.Toplevel(self.root)
        config_window.title(f"Configuración de {gait_name.capitalize()}")
        config_window.geometry("400x500")
        config_window.transient(self.root)
        config_window.grab_set()
        
        # Frame principal
        main_frame = ttk.Frame(config_window)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Título
        title_label = ttk.Label(main_frame, text=f"Parámetros de {gait_name}", 
                            font=('Arial', 12, 'bold'))
        title_label.pack(pady=10)
        
        # Frame para contenido con scroll (SOLO los parámetros)
        content_frame = ttk.Frame(main_frame)
        content_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Canvas y scrollbar para el contenido
        canvas = tk.Canvas(content_frame, height=350)
        scrollbar = ttk.Scrollbar(content_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Empaquetar canvas y scrollbar
        canvas.pack(side="left", fill="both", expand=True, padx=(0, 5))
        scrollbar.pack(side="right", fill="y")
        
        entries = {}
        
        # Parámetros numéricos
        numeric_params = ['step_height', 'step_length', 'cycle_time', 'duty_factor']
        for param in numeric_params:
            if param in params:
                # Frame para cada parámetro
                param_frame = ttk.Frame(scrollable_frame)
                param_frame.pack(fill=tk.X, pady=8, padx=5)
                
                # Etiqueta del parámetro
                ttk.Label(param_frame, text=f"{param.replace('_', ' ').title()}:", 
                        width=20, font=('Arial', 9)).pack(side=tk.LEFT)
                
                # Campo de entrada
                entry = ttk.Entry(param_frame, width=12, font=('Arial', 9))
                entry.insert(0, str(params[param]))
                entry.pack(side=tk.RIGHT, padx=5)
                entries[param] = entry
                
                # Descripción del parámetro
                desc_text = self.get_gait_parameter_description(param)
                desc_label = ttk.Label(scrollable_frame, text=desc_text, 
                                    font=('Arial', 8), foreground="gray", 
                                    wraplength=350, justify=tk.LEFT)
                desc_label.pack(fill=tk.X, pady=(0, 10), padx=5)
        
        # Phase offsets (array)
        if 'phase_offset' in params:
            # Título para phase offsets
            phase_title = ttk.Label(scrollable_frame, text="Desfases de Fase (LF, LR, RR, RF):",
                                font=('Arial', 10, 'bold'))
            phase_title.pack(pady=(15, 10), padx=5)
            
            # Frame para los phase offsets
            phases_frame = ttk.Frame(scrollable_frame)
            phases_frame.pack(fill=tk.X, pady=10, padx=5)
            
            phase_entries = []
            leg_names = ["LF", "LR", "RR", "RF"]
            
            for i, (leg_name, phase_val) in enumerate(zip(leg_names, params['phase_offset'])):
                leg_frame = ttk.Frame(phases_frame)
                leg_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
                
                # Etiqueta de la pata
                ttk.Label(leg_frame, text=leg_name, font=('Arial', 9, 'bold')).pack()
                
                # Campo de entrada
                entry = ttk.Entry(leg_frame, width=8, font=('Arial', 9), justify='center')
                entry.insert(0, str(phase_val))
                entry.pack(pady=2)
                phase_entries.append(entry)
            
            entries['phase_offset'] = phase_entries
            
            # Descripción de phase offsets
            phase_desc = ttk.Label(scrollable_frame, 
                                text="Valores entre 0.0 y 1.0 que definen el desfase temporal entre patas",
                                font=('Arial', 8), foreground="gray", wraplength=350)
            phase_desc.pack(fill=tk.X, pady=(5, 15), padx=5)
        
        # =============================================================================
        # FUNCIONES PARA LOS BOTONES - DEFINIR ANTES DE CREAR LOS BOTONES
        # =============================================================================
        
        def apply_parameters():
            """Aplicar parámetros sin cerrar la ventana"""
            try:
                # Guardar parámetros numéricos
                for param in numeric_params:
                    if param in entries:
                        new_value = float(entries[param].get())
                        self.gait_manager.update_gait_parameter(gait_name, param, new_value)
                
                # Guardar phase offsets
                if 'phase_offset' in entries:
                    new_phases = []
                    for entry in entries['phase_offset']:
                        new_phases.append(float(entry.get()))
                    self.gait_manager.update_gait_parameter(gait_name, 'phase_offset', new_phases)
                
                messagebox.showinfo("Éxito", f"Parámetros de {gait_name} aplicados")
                
            except ValueError as e:
                messagebox.showerror("Error", f"Valor inválido: {str(e)}")

        def save_parameters():
            """Guardar parámetros y cerrar ventana"""
            try:
                # Guardar parámetros numéricos
                for param in numeric_params:
                    if param in entries:
                        new_value = float(entries[param].get())
                        self.gait_manager.update_gait_parameter(gait_name, param, new_value)
                
                # Guardar phase offsets
                if 'phase_offset' in entries:
                    new_phases = []
                    for entry in entries['phase_offset']:
                        new_phases.append(float(entry.get()))
                    self.gait_manager.update_gait_parameter(gait_name, 'phase_offset', new_phases)
                
                config_window.destroy()
                messagebox.showinfo("Éxito", f"Parámetros de {gait_name} guardados")
                
            except ValueError as e:
                messagebox.showerror("Error", f"Valor inválido: {str(e)}")

        # =============================================================================
        # FRAME DE BOTONES (FUERA del área de scroll)
        # =============================================================================
        
        # Frame separador
        separator = ttk.Separator(main_frame, orient='horizontal')
        separator.pack(fill=tk.X, pady=10)
        
        # Frame para botones (SIEMPRE VISIBLE)
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=10)
        
        # Botón Aplicar
        apply_btn = ttk.Button(button_frame, text="Aplicar", 
                            command=apply_parameters, width=12)
        apply_btn.pack(side=tk.LEFT, padx=10)
        
        # Botón Guardar
        save_btn = ttk.Button(button_frame, text="Guardar", 
                            command=save_parameters, width=12)
        save_btn.pack(side=tk.LEFT, padx=10)
        
        # Botón Cerrar
        close_btn = ttk.Button(button_frame, text="Cerrar", 
                            command=config_window.destroy, width=12)
        close_btn.pack(side=tk.RIGHT, padx=10)
        
        # Asegurar que el scroll region se actualice
        def update_scroll_region():
            scrollable_frame.update_idletasks()
            canvas.config(scrollregion=canvas.bbox("all"))
        
        config_window.after(100, update_scroll_region)

    def start_gait_visualization_update(self):
        """Iniciar actualización periódica de la visualización durante el movimiento del gait"""
        if self.gait_manager.is_moving:
            # Los sliders mandan en vivo mientras camina
            self.gait_manager.set_command(self.gait_direction_x.get(),
                                          self.gait_direction_y.get(),
                                          self.gait_turn.get())
            # Obtener las posiciones actuales del gait manager
            current_positions = self.gait_manager.get_current_positions()
            
            # Actualizar los controles de la interfaz
            for i, pos in enumerate(current_positions):
                if i < len(self.leg_vars):
                    self.leg_vars[i]['x'].set(pos[0])
                    self.leg_vars[i]['y'].set(pos[1])
                    self.leg_vars[i]['z'].set(pos[2])
                    # Actualizar etiquetas
                    self.leg_vars[i]['x_label'].config(text=f"{pos[0]:.3f}")
                    self.leg_vars[i]['y_label'].config(text=f"{pos[1]:.3f}")
                    self.leg_vars[i]['z_label'].config(text=f"{pos[2]:.3f}")
                    # Actualizar campos de entrada
                    self.leg_vars[i]['x_entry'].delete(0, tk.END)
                    self.leg_vars[i]['x_entry'].insert(0, f"{pos[0]:.3f}")
                    self.leg_vars[i]['y_entry'].delete(0, tk.END)
                    self.leg_vars[i]['y_entry'].insert(0, f"{pos[1]:.3f}")
                    self.leg_vars[i]['z_entry'].delete(0, tk.END)
                    self.leg_vars[i]['z_entry'].insert(0, f"{pos[2]:.3f}")
            
            # Actualizar visualización
            self.update_visualization()
            
            # Programar próxima actualización (50Hz = 20ms)
            self.root.after(20, self.start_gait_visualization_update)



    def smart_delay(self, delay_ms):
        """Delay no bloqueante que permite detener la ejecución"""
        start_time = time.time()
        target_time = start_time + (delay_ms / 1000.0)
        
        while time.time() < target_time and not self.stop_routine_flag:
            remaining = (target_time - time.time()) * 1000
            if remaining > 100:  # Actualizar GUI cada 100ms
                self.root.update()
                time.sleep(0.1)
            else:
                time.sleep(0.01)
        
        return not self.stop_routine_flag

    def _on_mousewheel(self, event):
        self.control_canvas.yview_scroll(int(-1*(event.delta/120)), "units")

    def setup_routine_controls(self, parent):
        """Configuración del bloque de rutinas CON SCROLL"""
        routine_frame = ttk.LabelFrame(parent, text="Rutinas")
        routine_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Crear contenedor con scroll para todo el contenido de rutinas
        routine_container = ttk.Frame(routine_frame)
        routine_container.pack(fill=tk.BOTH, expand=True)
        
        # Canvas y scrollbar para el contenido de rutinas
        self.routine_canvas = tk.Canvas(routine_container, highlightthickness=0)
        routine_scrollbar = ttk.Scrollbar(routine_container, orient="vertical", command=self.routine_canvas.yview)
        
        # Frame scrollable para todo el contenido
        self.scrollable_routine_frame = ttk.Frame(self.routine_canvas)
        
        self.scrollable_routine_frame.bind(
            "<Configure>",
            lambda e: self.routine_canvas.configure(scrollregion=self.routine_canvas.bbox("all"))
        )
        
        self.routine_canvas.create_window((0, 0), window=self.scrollable_routine_frame, anchor="nw")
        self.routine_canvas.configure(yscrollcommand=routine_scrollbar.set)
        
        # Empaquetar canvas y scrollbar
        self.routine_canvas.pack(side="left", fill="both", expand=True)
        routine_scrollbar.pack(side="right", fill="y")
        
        # Bind mouse wheel al canvas de rutinas
        self.routine_canvas.bind("<MouseWheel>", self._on_routine_mousewheel)
        
        # --- NUEVO: AGREGAR JOYSTICK AL INICIO ---
        self.setup_joystick_controls(self.scrollable_routine_frame)

        # Ahora todo el contenido va dentro del scrollable_routine_frame
        
        # Frame para controles de velocidad
        speed_frame = ttk.LabelFrame(self.scrollable_routine_frame, text="Control de Velocidad")
        speed_frame.pack(fill=tk.X, pady=5, padx=5)
        
        # Control de delay entre pasos
        delay_frame = ttk.Frame(speed_frame)
        delay_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(delay_frame, text="Delay entre pasos (ms):", 
                font=('Arial', 9)).pack(side=tk.LEFT, padx=5)
        
        delay_entry = ttk.Entry(delay_frame, width=8, 
                            textvariable=self.step_delay_ms,
                            font=('Arial', 9))
        delay_entry.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(delay_frame, text="(1000 ms = 1 segundo)").pack(side=tk.LEFT, padx=5)
        
        # Control de pasos intermedios
        intermediate_frame = ttk.Frame(speed_frame)
        intermediate_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(intermediate_frame, text="Pasos intermedios:", 
                font=('Arial', 9)).pack(side=tk.LEFT, padx=5)
        
        intermediate_entry = ttk.Entry(intermediate_frame, width=8,
                                    textvariable=self.intermediate_steps,
                                    font=('Arial', 9))
        intermediate_entry.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(intermediate_frame, text="(más pasos = movimiento más suave)").pack(side=tk.LEFT, padx=5)
        
        # Display de pasos
        steps_frame = ttk.LabelFrame(self.scrollable_routine_frame, text="Pasos de Rutina")
        steps_frame.pack(fill=tk.BOTH, expand=True, pady=5, padx=5)
        
        # Listbox para mostrar pasos con altura fija
        self.steps_listbox = tk.Listbox(steps_frame, height=8, font=('Arial', 9))
        self.steps_listbox.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Scrollbar para el listbox
        listbox_scrollbar = ttk.Scrollbar(self.steps_listbox)
        listbox_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.steps_listbox.config(yscrollcommand=listbox_scrollbar.set)
        listbox_scrollbar.config(command=self.steps_listbox.yview)
        
        # Frame para botones de control de pasos
        steps_buttons_frame = ttk.Frame(self.scrollable_routine_frame)
        steps_buttons_frame.pack(fill=tk.X, pady=5)
        
        # Botones básicos
        ttk.Button(steps_buttons_frame, text="Guardar Paso", 
                command=self.save_step).pack(fill=tk.X, pady=2)
        ttk.Button(steps_buttons_frame, text="Borrar Paso", 
                command=self.delete_step).pack(fill=tk.X, pady=2)
        ttk.Button(steps_buttons_frame, text="Editar Paso", 
                command=self.edit_step).pack(fill=tk.X, pady=2)
        ttk.Button(steps_buttons_frame, text="Actualizar Paso", 
                command=self.update_step).pack(fill=tk.X, pady=2)
        ttk.Button(steps_buttons_frame, text="Ejecutar Paso", 
                command=self.execute_step).pack(fill=tk.X, pady=2)
        
        # Frame para botones de rutina completa
        routine_buttons_frame = ttk.Frame(self.scrollable_routine_frame)
        routine_buttons_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(routine_buttons_frame, text="Ejecutar Rutina", 
                command=self.execute_routine).pack(fill=tk.X, pady=2)
        ttk.Button(routine_buttons_frame, text="Borrar Toda la Rutina", 
                command=self.clear_routine).pack(fill=tk.X, pady=2)
        
        # Frame para guardar/cargar rutina
        file_buttons_frame = ttk.Frame(self.scrollable_routine_frame)
        file_buttons_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(file_buttons_frame, text="Guardar Rutina", 
                command=self.save_routine).pack(fill=tk.X, pady=2)
        ttk.Button(file_buttons_frame, text="Cargar Rutina", 
                command=self.load_routine).pack(fill=tk.X, pady=2)

        # Frame para botón de detener
        stop_frame = ttk.Frame(self.scrollable_routine_frame)
        stop_frame.pack(fill=tk.X, pady=2)
        
        self.stop_button = ttk.Button(stop_frame, text="⏹️ Detener Rutina", 
                                    command=self.stop_routine, state="disabled")
        self.stop_button.pack(fill=tk.X)

        # Ajustar el tamaño del frame scrollable
        self.scrollable_routine_frame.update_idletasks()
        self.routine_canvas.config(scrollregion=self.routine_canvas.bbox("all"))

    def _on_routine_mousewheel(self, event):
        """Manejar scroll del mouse en el canvas de rutinas"""
        self.routine_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
    
    def enable_stop_button(self):
        """Habilitar botón de detener"""
        self.stop_button.config(state="normal")

    def disable_stop_button(self):
        """Deshabilitar botón de detener"""
        self.stop_button.config(state="disabled")

    def stop_routine(self):
        """Detener ejecución de rutina"""
        self.stop_routine_flag = True
        self.disable_stop_button()
        messagebox.showinfo("Rutina Detenida", "La ejecución de la rutina ha sido detenida")    

    def save_step(self):
        """Guardar el paso actual en la rutina"""
        try:
            # Obtener configuración actual de patas
            leg_positions = []
            for leg_vars in self.leg_vars:
                leg_pos = {
                    'x': leg_vars['x'].get(),
                    'y': leg_vars['y'].get(),
                    'z': leg_vars['z'].get()
                }
                leg_positions.append(leg_pos)
            
            #  VERIFICAR: Obtener configuración actual de plataforma
            platform_pos = {
                'x': self.platform_vars['position']['x'].get(),
                'y': self.platform_vars['position']['y'].get(),
                'z': self.platform_vars['position']['z'].get()
            }
            
            platform_rot = {
                'roll': self.platform_vars['rotation']['roll'].get(),
                'pitch': self.platform_vars['rotation']['pitch'].get(),
                'yaw': self.platform_vars['rotation']['yaw'].get()
            }
            
            # Calcular ángulos actuales
            foot_targets = [[lv['x'].get(), lv['y'].get(), lv['z'].get()] for lv in self.leg_vars]
            rot = [self.platform_vars['rotation'][a].get() for a in ['roll','pitch','yaw']]
            body_pos = [self.platform_vars['position'][a].get() for a in ['x','y','z']]
            
            angles_list, _ = self.kin.plot_robot(self.ax, foot_targets, rot, body_pos, is_radians=False)
            
            # Convertir ángulos a lista serializable
            angles_serializable = []
            for angles in angles_list:
                if any(np.isnan(angles)):
                    # Si hay errores, guardar ceros
                    angles_serializable.append([0.0, 0.0, 0.0])
                else:
                    # Convertir a lista de Python
                    if hasattr(angles, 'tolist'):
                        # Si es array de NumPy
                        angles_serializable.append(angles.tolist())
                    else:
                        # Si ya es lista de Python
                        angles_serializable.append([float(ang) for ang in angles])
            
            # Crear paso (INCLUYENDO PLATAFORMA)
            step = {
                'step_number': len(self.routine_steps) + 1,
                'leg_positions': leg_positions,
                'platform_position': platform_pos,      #  INCLUIR PLATAFORMA
                'platform_rotation': platform_rot,      #  INCLUIR PLATAFORMA
                'angles': angles_serializable,
                'timestamp': time.time()
            }
            
            self.routine_steps.append(step)
            self.update_steps_display()
            
            messagebox.showinfo("Paso Guardado", f"Paso {step['step_number']} guardado exitosamente")
            
        except Exception as e:
            messagebox.showerror("Error", f"Error al guardar paso: {e}")

    def update_step(self):
        """Actualizar el paso actualmente cargado con los valores actuales de los controles"""
        if self.current_step_index == -1:
            messagebox.showwarning("Advertencia", "Primero debes cargar un paso para editar usando 'Editar Paso'")
            return
        
        try:
            # Obtener configuración actual de patas
            leg_positions = []
            for leg_vars in self.leg_vars:
                leg_pos = {
                    'x': leg_vars['x'].get(),
                    'y': leg_vars['y'].get(),
                    'z': leg_vars['z'].get()
                }
                leg_positions.append(leg_pos)
            
            # Obtener configuración actual de plataforma
            platform_pos = {
                'x': self.platform_vars['position']['x'].get(),
                'y': self.platform_vars['position']['y'].get(),
                'z': self.platform_vars['position']['z'].get()
            }
            
            platform_rot = {
                'roll': self.platform_vars['rotation']['roll'].get(),
                'pitch': self.platform_vars['rotation']['pitch'].get(),
                'yaw': self.platform_vars['rotation']['yaw'].get()
            }
            
            # Calcular ángulos actuales
            foot_targets = [[lv['x'].get(), lv['y'].get(), lv['z'].get()] for lv in self.leg_vars]
            rot = [self.platform_vars['rotation'][a].get() for a in ['roll','pitch','yaw']]
            body_pos = [self.platform_vars['position'][a].get() for a in ['x','y','z']]
            
            angles_list, _ = self.kin.plot_robot(self.ax, foot_targets, rot, body_pos, is_radians=False)
            
            # Convertir ángulos a lista serializable
            angles_serializable = []
            for angles in angles_list:
                if any(np.isnan(angles)):
                    # Si hay errores, guardar ceros
                    angles_serializable.append([0.0, 0.0, 0.0])
                else:
                    # Convertir a lista de Python
                    if hasattr(angles, 'tolist'):
                        # Si es array de NumPy
                        angles_serializable.append(angles.tolist())
                    else:
                        # Si ya es lista de Python
                        angles_serializable.append([float(ang) for ang in angles])
            
            # Actualizar el paso existente
            updated_step = {
                'step_number': self.current_step_index + 1,
                'leg_positions': leg_positions,
                'platform_position': platform_pos,
                'platform_rotation': platform_rot,
                'angles': angles_serializable,
                'timestamp': time.time()
            }
            
            self.routine_steps[self.current_step_index] = updated_step
            self.update_steps_display()
            
            messagebox.showinfo("Paso Actualizado", f"Paso {self.current_step_index + 1} actualizado exitosamente")
            
        except Exception as e:
            messagebox.showerror("Error", f"Error al actualizar paso: {e}")
    def clear_routine(self):
        """Borrar toda la rutina y limpiar el display"""
        if not self.routine_steps:
            messagebox.showinfo("Información", "No hay pasos en la rutina para borrar")
            return
        
        if messagebox.askyesno("Confirmar", "¿Estás seguro de que quieres borrar TODA la rutina?"):
            self.routine_steps.clear()
            self.update_steps_display()
            messagebox.showinfo("Rutina Borrada", "Toda la rutina ha sido borrada exitosamente")
            
    def delete_step(self):
        """Borrar el paso seleccionado"""
        selected = self.steps_listbox.curselection()
        if not selected:
            messagebox.showwarning("Advertencia", "Selecciona un paso para borrar")
            return
        
        index = selected[0]
        if 0 <= index < len(self.routine_steps):
            self.routine_steps.pop(index)
            self.update_steps_display()
            messagebox.showinfo("Paso Borrado", "Paso borrado exitosamente")

    def edit_step(self):
        """Cargar un paso existente para editarlo"""
        if not self.routine_steps:
            messagebox.showwarning("Advertencia", "No hay pasos en la rutina para editar")
            return
        
        # Pedir número de paso al usuario
        step_number = self.ask_step_number()
        if step_number is None:  # Usuario canceló
            return
        
        # Verificar que el paso existe
        if step_number < 1 or step_number > len(self.routine_steps):
            messagebox.showerror("Error", f"Paso {step_number} no existe. La rutina tiene {len(self.routine_steps)} pasos.")
            return
        
        index = step_number - 1  # Convertir a índice base 0
        step = self.routine_steps[index]
        
        # Actualizar controles con los valores del paso
        # Patas
        for i, leg_pos in enumerate(step['leg_positions']):
            if i < len(self.leg_vars):
                self.leg_vars[i]['x'].set(leg_pos['x'])
                self.leg_vars[i]['y'].set(leg_pos['y'])
                self.leg_vars[i]['z'].set(leg_pos['z'])
                # ACTUALIZAR ETIQUETAS DE LAS PATAS
                self.leg_vars[i]['x_label'].config(text=f"{leg_pos['x']:.3f}")
                self.leg_vars[i]['y_label'].config(text=f"{leg_pos['y']:.3f}")
                self.leg_vars[i]['z_label'].config(text=f"{leg_pos['z']:.3f}")
        
        # ✅ VERIFICAR: Actualizar PLATAFORMA también
        platform_pos = step['platform_position']
        self.platform_vars['position']['x'].set(platform_pos['x'])
        self.platform_vars['position']['y'].set(platform_pos['y'])
        self.platform_vars['position']['z'].set(platform_pos['z'])
        
        platform_rot = step['platform_rotation']
        self.platform_vars['rotation']['roll'].set(platform_rot['roll'])
        self.platform_vars['rotation']['pitch'].set(platform_rot['pitch'])
        self.platform_vars['rotation']['yaw'].set(platform_rot['yaw'])
        
        # ✅ VERIFICAR: ACTUALIZAR ETIQUETAS DE LA PLATAFORMA
        self.platform_vars['position']['x_label'].config(text=f"{platform_pos['x']:.3f}")
        self.platform_vars['position']['y_label'].config(text=f"{platform_pos['y']:.3f}")
        self.platform_vars['position']['z_label'].config(text=f"{platform_pos['z']:.3f}")
        
        self.platform_vars['rotation']['roll_label'].config(text=f"{platform_rot['roll']:.1f}")
        self.platform_vars['rotation']['pitch_label'].config(text=f"{platform_rot['pitch']:.1f}")
        self.platform_vars['rotation']['yaw_label'].config(text=f"{platform_rot['yaw']:.1f}")
        
        # Guardar el índice del paso que estamos editando
        self.current_step_index = index
        self.update_visualization()
        
        # Resaltar paso en el listbox
        self.steps_listbox.selection_clear(0, tk.END)
        self.steps_listbox.selection_set(index)
        self.steps_listbox.see(index)
        
        messagebox.showinfo("Paso Cargado", 
                        f"Paso {step_number} cargado para edición.\n"
                        f"Realiza los cambios necesarios y luego presiona 'Actualizar Paso' para guardar los cambios.")
        

    def ask_step_number(self):
        """Diálogo para pedir número de paso"""
        while True:
            result = tk.simpledialog.askinteger(
                "Número de Paso", 
                f"Ingresa el número de paso a editar (1-{len(self.routine_steps)}):",
                parent=self.root,
                minvalue=1, 
                maxvalue=len(self.routine_steps)
            )
            
            if result is None:  # Usuario presionó Cancelar
                return None
            elif 1 <= result <= len(self.routine_steps):
                return result
            else:
                messagebox.showerror("Error", f"Número inválido. Debe ser entre 1 y {len(self.routine_steps)}")

    def execute_step(self):
        """Ejecutar el paso seleccionado"""
        selected = self.steps_listbox.curselection()
        if not selected:
            messagebox.showwarning("Advertencia", "Selecciona un paso para ejecutar")
            return
        
        index = selected[0]
        self.execute_single_step(index)

    def execute_single_step(self, index):
        """Ejecutar un paso específico - CORREGIDO para incluir plataforma"""
        # Resetear el índice de edición cuando ejecutamos un paso
        self.current_step_index = -1

        if 0 <= index < len(self.routine_steps):
            step = self.routine_steps[index]
            
            # Actualizar controles DE PATAS
            for i, leg_pos in enumerate(step['leg_positions']):
                if i < len(self.leg_vars):
                    self.leg_vars[i]['x'].set(leg_pos['x'])
                    self.leg_vars[i]['y'].set(leg_pos['y'])
                    self.leg_vars[i]['z'].set(leg_pos['z'])
                    # ACTUALIZAR ETIQUETAS DE LAS PATAS
                    self.leg_vars[i]['x_label'].config(text=f"{leg_pos['x']:.3f}")
                    self.leg_vars[i]['y_label'].config(text=f"{leg_pos['y']:.3f}")
                    self.leg_vars[i]['z_label'].config(text=f"{leg_pos['z']:.3f}")
            
            #  CORRECCIÓN: ACTUALIZAR PLATAFORMA TAMBIÉN
            platform_pos = step['platform_position']
            self.platform_vars['position']['x'].set(platform_pos['x'])
            self.platform_vars['position']['y'].set(platform_pos['y'])
            self.platform_vars['position']['z'].set(platform_pos['z'])
            
            platform_rot = step['platform_rotation']
            self.platform_vars['rotation']['roll'].set(platform_rot['roll'])
            self.platform_vars['rotation']['pitch'].set(platform_rot['pitch'])
            self.platform_vars['rotation']['yaw'].set(platform_rot['yaw'])
            
            #  CORRECCIÓN: ACTUALIZAR ETIQUETAS DE LA PLATAFORMA
            self.platform_vars['position']['x_label'].config(text=f"{platform_pos['x']:.3f}")
            self.platform_vars['position']['y_label'].config(text=f"{platform_pos['y']:.3f}")
            self.platform_vars['position']['z_label'].config(text=f"{platform_pos['z']:.3f}")
            
            self.platform_vars['rotation']['roll_label'].config(text=f"{platform_rot['roll']:.1f}")
            self.platform_vars['rotation']['pitch_label'].config(text=f"{platform_rot['pitch']:.1f}")
            self.platform_vars['rotation']['yaw_label'].config(text=f"{platform_rot['yaw']:.1f}")
            
            # Actualizar visualización y enviar ángulos
            self.update_visualization()
            
            # Resaltar paso actual en el listbox
            self.steps_listbox.selection_clear(0, tk.END)
            self.steps_listbox.selection_set(index)
            self.steps_listbox.see(index)
            
            print(f" Ejecutando paso {index + 1}")

    def execute_routine(self):
        """Ejecutar toda la rutina con interpolación suave - VERSIÓN MEJORADA"""
        if not self.routine_steps:
            messagebox.showwarning("Advertencia", "No hay pasos en la rutina")
            return
        
        if self.routine_executing:
            messagebox.showwarning("Advertencia", "Ya hay una rutina en ejecución")
            return
        
        # Obtener parámetros de velocidad
        base_delay = self.step_delay_ms.get()
        adjusted_delay = max(50, int(base_delay / self.global_speed.get()))  # Mínimo 50ms
        
        intermediate_steps = max(0, self.intermediate_steps.get())
        
        # Iniciar ejecución en hilo separado
        self.stop_routine_flag = False
        self.routine_executing = True
        
        def routine_thread():
            try:
                for step_index in range(len(self.routine_steps)):
                    if self.stop_routine_flag:
                        break
                    
                    # Ejecutar paso actual
                    self.root.after(0, lambda idx=step_index: self.execute_single_step(idx))
                    
                    # Si hay pasos intermedios y no es el último paso
                    if intermediate_steps > 0 and step_index < len(self.routine_steps) - 1:
                        if not self.execute_intermediate_steps_optimized(step_index, step_index + 1, intermediate_steps):
                            break  # Se solicitó detener
                    
                    # Delay entre pasos principales (si no es el último paso)
                    if step_index < len(self.routine_steps) - 1:
                        if not self.smart_delay(adjusted_delay):
                            break
                
                # Finalizar ejecución
                self.root.after(0, self.finish_routine_execution)
                
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("Error", f"Error en rutina: {e}"))
                self.root.after(0, self.finish_routine_execution)
        
        # Ejecutar en hilo separado
        self.current_routine_thread = threading.Thread(target=routine_thread, daemon=True)
        self.current_routine_thread.start()
        
        # Habilitar botón de detener
        self.enable_stop_button()

    def finish_routine_execution(self):
        """Finalizar ejecución de rutina"""
        self.routine_executing = False
        self.differential_routine_executing = False
        self.stop_routine_flag = False
        
        if not self.stop_routine_flag:
            self.root.after(0, lambda: messagebox.showinfo("Rutina Completada", "Todos los pasos ejecutados"))

    def execute_intermediate_steps_optimized(self, current_index, next_index, num_steps):
        """Ejecutar pasos intermedios de forma optimizada"""
        if current_index >= len(self.routine_steps) or next_index >= len(self.routine_steps):
            return True
        
        if self.stop_routine_flag:
            return False
        
        current_step = self.routine_steps[current_index]
        next_step = self.routine_steps[next_index]
        
        # Calcular todos los pasos intermedios primero
        intermediate_positions = []
        for step_num in range(1, num_steps + 1):
            if self.stop_routine_flag:
                return False
                
            alpha = step_num / (num_steps + 1)
            positions = self.calculate_intermediate_positions(current_step, next_step, alpha)
            intermediate_positions.append(positions)
        
        # Ejecutar pasos con delay optimizado
        base_interpolation_delay = max(20, int(100 / self.global_speed.get()))  # Mínimo 20ms
        
        for positions in intermediate_positions:
            if self.stop_routine_flag:
                return False
                
            # Aplicar posiciones calculadas
            self.apply_intermediate_positions(positions)
            
            # Delay breve entre pasos intermedios
            if not self.smart_delay(base_interpolation_delay):
                return False
        
        return True

    def calculate_intermediate_positions(self, current_step, next_step, alpha):
        """Calcular posiciones intermedias sin aplicar aún"""
        positions = {
            'legs': [],
            'platform_pos': {},
            'platform_rot': {}
        }
        
        # Interpolar posiciones de patas
        for i in range(4):
            if i < len(self.leg_vars):
                current_leg = current_step['leg_positions'][i]
                next_leg = next_step['leg_positions'][i]
                
                x = current_leg['x'] + (next_leg['x'] - current_leg['x']) * alpha
                y = current_leg['y'] + (next_leg['y'] - current_leg['y']) * alpha
                z = current_leg['z'] + (next_leg['z'] - current_leg['z']) * alpha
                
                positions['legs'].append((i, x, y, z))
        
        # Interpolar plataforma
        current_pos = current_step['platform_position']
        next_pos = next_step['platform_position']
        current_rot = current_step['platform_rotation']
        next_rot = next_step['platform_rotation']
        
        positions['platform_pos'] = {
            'x': current_pos['x'] + (next_pos['x'] - current_pos['x']) * alpha,
            'y': current_pos['y'] + (next_pos['y'] - current_pos['y']) * alpha,
            'z': current_pos['z'] + (next_pos['z'] - current_pos['z']) * alpha
        }
        
        positions['platform_rot'] = {
            'roll': current_rot['roll'] + (next_rot['roll'] - current_rot['roll']) * alpha,
            'pitch': current_rot['pitch'] + (next_rot['pitch'] - current_rot['pitch']) * alpha,
            'yaw': current_rot['yaw'] + (next_rot['yaw'] - current_rot['yaw']) * alpha
        }
        
        return positions

    def apply_intermediate_positions(self, positions):
        """Aplicar posiciones calculadas a la GUI"""
        # Aplicar posiciones de patas
        for leg_idx, x, y, z in positions['legs']:
            self.leg_vars[leg_idx]['x'].set(x)
            self.leg_vars[leg_idx]['y'].set(y)
            self.leg_vars[leg_idx]['z'].set(z)
            self.leg_vars[leg_idx]['x_label'].config(text=f"{x:.3f}")
            self.leg_vars[leg_idx]['y_label'].config(text=f"{y:.3f}")
            self.leg_vars[leg_idx]['z_label'].config(text=f"{z:.3f}")
        
        # Aplicar plataforma
        platform_pos = positions['platform_pos']
        platform_rot = positions['platform_rot']
        
        for coord, value in platform_pos.items():
            self.platform_vars['position'][coord].set(value)
            self.platform_vars['position'][f'{coord}_label'].config(text=f"{value:.3f}")
        
        for ang, value in platform_rot.items():
            self.platform_vars['rotation'][ang].set(value)
            self.platform_vars['rotation'][f'{ang}_label'].config(text=f"{value:.1f}")
        
        # Actualizar visualización
        self.update_visualization()

    def save_routine(self):
        """Guardar rutina completa en archivo JSON"""
        try:
            if not self.routine_steps:
                messagebox.showwarning("Advertencia", "No hay pasos para guardar")
                return
            
            from tkinter import filedialog
            file_path = filedialog.asksaveasfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Guardar rutina como"
            )
            
            if file_path:
                routine_data = {
                    'routine_name': os.path.basename(file_path),
                    'created': time.time(),
                    'execution_parameters': {
                        'step_delay_ms': self.step_delay_ms.get(),
                        'intermediate_steps': self.intermediate_steps.get()
                    },
                    'steps': self.routine_steps
                }
                
                with open(file_path, 'w') as f:
                    json.dump(routine_data, f, indent=4)
                
                messagebox.showinfo("Rutina Guardada", f"Rutina guardada en:\n{file_path}")
                
        except Exception as e:
            messagebox.showerror("Error", f"Error al guardar rutina: {e}")

    def load_routine(self):
        """Cargar rutina desde archivo JSON"""
        try:
            from tkinter import filedialog
            file_path = filedialog.askopenfilename(
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Cargar rutina"
            )
            
            if file_path and os.path.exists(file_path):
                with open(file_path, 'r') as f:
                    routine_data = json.load(f)
                
                self.routine_steps = routine_data.get('steps', [])
                
                # Cargar parámetros de ejecución si existen
                execution_params = routine_data.get('execution_parameters', {})
                if 'step_delay_ms' in execution_params:
                    self.step_delay_ms.set(execution_params['step_delay_ms'])
                if 'intermediate_steps' in execution_params:
                    self.intermediate_steps.set(execution_params['intermediate_steps'])
                
                self.update_steps_display()
                
                messagebox.showinfo("Rutina Cargada", f"Rutina cargada desde:\n{file_path}")
                
        except Exception as e:
            messagebox.showerror("Error", f"Error al cargar rutina: {e}")

    def update_steps_display(self):
        """Actualizar el display de pasos"""
        self.steps_listbox.delete(0, tk.END)
        for i, step in enumerate(self.routine_steps):
            try:
                # Crear string de ángulos de forma segura
                angle_strings = []
                for leg in step['angles']:
                    # Asegurarse de que los ángulos sean números
                    leg_angles = []
                    for ang in leg[:2]:  # Solo primeros 2 ángulos para mostrar
                        try:
                            leg_angles.append(f"{float(ang):.1f}")
                        except (ValueError, TypeError):
                            leg_angles.append("0.0")
                    angle_strings.extend(leg_angles)
                
                angles_str = " | ".join(angle_strings[:6])  # Máximo 6 valores
                self.steps_listbox.insert(tk.END, f"Paso {i+1}: {angles_str}...")
            except Exception as e:
                # En caso de error, mostrar paso básico
                self.steps_listbox.insert(tk.END, f"Paso {i+1}: [ángulos guardados]")

    # -------------------
    # Bloque de calibración con offsets, inversión y límites
    # -------------------
    def setup_calibration_controls(self, parent):
        cal_frame = ttk.LabelFrame(parent, text="Calibración de Servos")
        cal_frame.pack(fill=tk.X, pady=5)
        leg_names = ["Del Izq", "Tras Izq", "Tras Der", "Del Der"]
        servo_names = ["T1", "T2", "T3"]  # Nombres más cortos
        
        # Frame principal con dos columnas
        columns_frame = ttk.Frame(cal_frame)
        columns_frame.pack(fill=tk.X, pady=5)
        
        # Crear dos columnas
        left_column = ttk.Frame(columns_frame)
        left_column.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)
        
        right_column = ttk.Frame(columns_frame)
        right_column.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=2)
        
        self.calibration_vars = []
        columns = [left_column, right_column]

        for leg_idx, leg_name in enumerate(leg_names):
            # Alternar entre columnas
            current_column = columns[leg_idx % 2]
            
            leg_cal_frame = ttk.LabelFrame(current_column, text=leg_name)
            leg_cal_frame.pack(fill=tk.X, pady=2, padx=2)
            
            leg_vars = {}
            for servo_idx, servo_name in enumerate(servo_names):
                servo_frame = ttk.Frame(leg_cal_frame)
                servo_frame.pack(fill=tk.X, pady=1, padx=1)

                # Offset, inversión, min/max
                offset_var = tk.DoubleVar(value=0.0)
                invert_var = tk.BooleanVar(value=False)
                min_var = tk.DoubleVar(value=0)
                max_var = tk.DoubleVar(value=180)

                # Frame de control principal
                ctrl_frame = ttk.Frame(servo_frame)
                ctrl_frame.pack(fill=tk.X)
                
                # Checkbox de inversión más pequeño
                invert_cb = ttk.Checkbutton(ctrl_frame, text="Inv", variable=invert_var,
                        command=self.update_visualization, width=4)
                invert_cb.pack(side=tk.LEFT, padx=1)

                # Slider de offset más compacto
                offset_scale = ttk.Scale(ctrl_frame, from_=-180, to=180, orient=tk.HORIZONTAL,
                        variable=offset_var,
                        command=lambda v, l=leg_idx, s=servo_idx: self.on_calibration_change(l, s),
                        length=80)
                offset_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

                offset_label = ttk.Label(ctrl_frame, text="0.0", width=4, font=('Arial', 8))
                offset_label.pack(side=tk.RIGHT, padx=1)

                # Frame para min/max
                limits_frame = ttk.Frame(servo_frame)
                limits_frame.pack(fill=tk.X, pady=1)
                
                # Min
                min_subframe = ttk.Frame(limits_frame)
                min_subframe.pack(side=tk.LEFT, fill=tk.X, expand=True)
                ttk.Label(min_subframe, text="Min:", font=('Arial', 7)).pack(side=tk.LEFT)
                min_entry = ttk.Entry(min_subframe, width=4, textvariable=min_var, font=('Arial', 8))
                min_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
                
                # Max
                max_subframe = ttk.Frame(limits_frame)
                max_subframe.pack(side=tk.RIGHT, fill=tk.X, expand=True)
                ttk.Label(max_subframe, text="Max:", font=('Arial', 7)).pack(side=tk.LEFT)
                max_entry = ttk.Entry(max_subframe, width=4, textvariable=max_var, font=('Arial', 8))
                max_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)

                leg_vars[servo_idx] = {
                    'offset': offset_var,
                    'invert': invert_var,
                    'label': offset_label,
                    'min': min_var,
                    'max': max_var,
                    'scale': offset_scale
                }
            self.calibration_vars.append(leg_vars)

        # Frame para botones (debajo de las dos columnas)
        button_frame = ttk.Frame(cal_frame)
        button_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(button_frame, text="Guardar Calibración", 
                command=self.save_calibration).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cargar Calibración", 
                command=self.load_calibration).pack(side=tk.LEFT, padx=5)
        
        # Frame para mostrar errores de límites
        self.error_frame = ttk.Frame(cal_frame)
        self.error_frame.pack(fill=tk.X, pady=5)
        self.error_label = ttk.Label(self.error_frame, text="", foreground="red", 
                                    font=('Arial', 9, 'bold'))
        self.error_label.pack(fill=tk.X)

    
    def setup_plot(self, parent):
        from robot_view import RobotViewport
        # Reducir el tamaño de la figura
        self.fig = plt.Figure(figsize=(8, 6))  # Cambiar de (8, 6) a (6, 5)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.viewport = RobotViewport(self.ax, self.kin)
        self.ax._robot_viewport = self.viewport
        self.viewport.add_controls(parent)
        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def check_angle_limits(self, calibrated_angles_deg):
        """Verificar si algún ángulo calibrado (en grados) excede los límites establecidos"""
        errors = []
        leg_idx = 0
        servo_idx = 0
        
        # calibrated_angles_deg es una lista plana de 12 ángulos [leg0_theta1, leg0_theta2, leg0_theta3, leg1_theta1, ...]
        for i in range(0, len(calibrated_angles_deg), 3):
            leg_idx = i // 3
            for j in range(3):
                servo_idx = j
                angle_deg = calibrated_angles_deg[i + j]
                
                if leg_idx < len(self.calibration_vars) and servo_idx < 3:
                    cal_vars = self.calibration_vars[leg_idx][servo_idx]
                    min_deg = cal_vars['min'].get()
                    max_deg = cal_vars['max'].get()
                    
                    if angle_deg < min_deg:
                        errors.append(f"ERROR servo{servo_idx+1} pierna{leg_idx+1} al límite (min: {min_deg:.1f}°, actual: {angle_deg:.1f}°)")
                    elif angle_deg > max_deg:
                        errors.append(f"ERROR servo{servo_idx+1} pierna{leg_idx+1} al límite (max: {max_deg:.1f}°, actual: {angle_deg:.1f}°)")
        
        return errors

    def apply_calibration(self, angles_list):
        """Aplicar offsets e inversión a los ángulos calculados"""
        calibrated_angles = []
        
        for leg_idx, angles in enumerate(angles_list):
            # Si es un array de numpy, convertir a lista
            if hasattr(angles, 'tolist'):
                angles = angles.tolist()
            
            calibrated_leg_angles = []
            for servo_idx, angle in enumerate(angles):
                if leg_idx < len(self.calibration_vars) and servo_idx < 3:
                    cal_vars = self.calibration_vars[leg_idx][servo_idx]
                    offset_rad = radians(cal_vars['offset'].get())
                    invert = cal_vars['invert'].get()
                    
                    # Aplicar offset
                    calibrated_angle = angle + offset_rad
                    
                    # Aplicar inversión si está activada
                    if invert:
                        calibrated_angle = pi - calibrated_angle
                    
                    calibrated_leg_angles.append(calibrated_angle)
                else:
                    calibrated_leg_angles.append(angle)
            
            calibrated_angles.append(calibrated_leg_angles)
        
        return calibrated_angles

    def get_display_angles_deg(self, angles_list):
        """Obtener ángulos calibrados en grados para mostrar en display"""
        calibrated_angles = self.apply_calibration(angles_list)
        
        # Convertir a grados y normalizar
        display_angles = []
        for leg_angles in calibrated_angles:
            # Saltar piernas con errores
            if any(np.isnan(leg_angles)):
                display_angles.extend([float('nan'), float('nan'), float('nan')])
            else:
                degs = [degrees(normalize_angle(a)) for a in leg_angles]
                display_angles.extend(degs)
        
        return display_angles

    def save_calibration(self):
        """Guardar la configuración de calibración en un archivo JSON"""
        try:
            calibration_data = {
                'legs': [],
                'limits_configured': True
            }
            
            for leg_idx, leg_vars in enumerate(self.calibration_vars):
                leg_data = {
                    'servos': []
                }
                for servo_idx in range(3):
                    servo_vars = leg_vars[servo_idx]
                    servo_data = {
                        'offset': servo_vars['offset'].get(),
                        'invert': servo_vars['invert'].get(),
                        'min': servo_vars['min'].get(),
                        'max': servo_vars['max'].get()
                    }
                    leg_data['servos'].append(servo_data)
                calibration_data['legs'].append(leg_data)
            
            # Abrir diálogo para guardar archivo
            from tkinter import filedialog
            file_path = filedialog.asksaveasfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Guardar calibración como"
            )
            
            if file_path:
                with open(file_path, 'w') as f:
                    json.dump(calibration_data, f, indent=4)
                print(f"Calibración guardada en: {file_path}")
                messagebox.showinfo("Calibración Guardada", f"Calibración guardada exitosamente en:\n{file_path}")
                
        except Exception as e:
            print(f"Error al guardar calibración: {e}")
            messagebox.showerror("Error", f"Error al guardar calibración:\n{e}")

    def load_calibration(self):
        """Cargar la configuración de calibración desde un archivo JSON"""
        try:
            # Abrir diálogo para seleccionar archivo
            from tkinter import filedialog
            file_path = filedialog.askopenfilename(
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Seleccionar archivo de calibración"
            )
            
            if file_path and os.path.exists(file_path):
                with open(file_path, 'r') as f:
                    calibration_data = json.load(f)
                
                # Aplicar los valores cargados a la interfaz
                for leg_idx, leg_data in enumerate(calibration_data['legs']):
                    for servo_idx, servo_data in enumerate(leg_data['servos']):
                        if leg_idx < len(self.calibration_vars) and servo_idx < 3:
                            servo_vars = self.calibration_vars[leg_idx][servo_idx]
                            servo_vars['offset'].set(servo_data['offset'])
                            servo_vars['invert'].set(servo_data['invert'])
                            servo_vars['min'].set(servo_data['min'])
                            servo_vars['max'].set(servo_data['max'])
                            # Actualizar etiqueta
                            self.on_calibration_change(leg_idx, servo_idx)
                
                # Marcar que los límites están cargados
                self.limits_loaded = calibration_data.get('limits_configured', False)
                
                print(f"Calibración cargada desde: {file_path}")
                messagebox.showinfo("Calibración Cargada", f"Calibración cargada exitosamente desde:\n{file_path}")
                self.update_visualization()
                
        except Exception as e:
            print(f"Error al cargar calibración: {e}")
            messagebox.showerror("Error", f"Error al cargar calibración:\n{e}")

    def update_single_leg_display(self, leg_idx):
        """Actualizar solo el display de ángulos para una pata específica"""
        # Obtener la configuración actual
        foot_targets = [[lv['x'].get(), lv['y'].get(), lv['z'].get()] for lv in self.leg_vars]
        rot = [self.platform_vars['rotation'][a].get() for a in ['roll','pitch','yaw']]
        body_pos = [self.platform_vars['position'][a].get() for a in ['x','y','z']]
        
        # Calcular ángulos solo para la pata específica
        angles_joints = self.kin.leg_IK(foot_targets[leg_idx], rot, leg_idx, False, body_pos)
        
        if any(np.isnan(angles_joints[:3])):
            # Si hay error, mostrar "--"
            self.angle_table.item(self.angle_table.get_children()[leg_idx], values=("--","--","--"))
        else:
            # Aplicar calibración y mostrar
            calibrated_angles = self.apply_calibration([angles_joints[:3]])[0]
            degs = [degrees(normalize_angle(a)) for a in calibrated_angles]
            degs = [(d + 180) % 360 - 180 for d in degs]
            self.angle_table.item(self.angle_table.get_children()[leg_idx], 
                                values=[f"{d:.1f}" for d in degs])
        
        # Enviar ángulos actualizados al ESP32 si está activado
        if send_to_esp32_flag.get():
            # Recalcular todos los ángulos para enviar
            all_angles_list, _ = self.kin.plot_robot(self.ax, foot_targets, rot, body_pos, is_radians=False)
            if not any(any(np.isnan(ang)) for ang in all_angles_list):
                calibrated_flat_angles = self.get_calibrated_angles_for_esp32(all_angles_list)
                send_angles(calibrated_flat_angles)
    
    def get_calibrated_angles_for_esp32(self, angles_list):
        """Validar el paquete completo antes de enviar desde cualquier control.

        Conserva las vueltas equivalentes de la IK, pero nunca transforma un
        ángulo negativo en el extremo opuesto del servo. None bloquea el envío.
        """
        flat_angles = self.get_display_angles_deg(angles_list)
        errors = []
        if len(flat_angles) != 12 or any(not math.isfinite(a) for a in flat_angles):
            errors.append("Se requieren 12 ángulos finitos")
        elif any(a < 0 or a > 180 for a in flat_angles):
            errors.append("Ángulos fuera del rango físico 0–180°")
        else:
            errors.extend(self.check_angle_limits(flat_angles))
        if errors:
            self.error_label.config(text="ERRORES:\n" + "\n".join(errors))
            return None
        self.error_label.config(text="")
        return flat_angles

    def display_angles(self, angles_list):
        """Muestra ángulos normalizados en la tabla"""
        # Aplicar calibración a TODAS las piernas antes de mostrar (igual que en el código que funciona)
        calibrated_angles = self.apply_calibration(angles_list)
        
        for i, angles in enumerate(calibrated_angles):
            # Verificar si hay ángulos NaN (errores físicos)
            if any(np.isnan(angles)):
                self.angle_table.item(self.angle_table.get_children()[i], values=("--","--","--"))
            else:
                # Convertir a grados y normalizar
                degs = [degrees(normalize_angle(a)) for a in angles]
                # Asegurar que estén en [-180, 180]
                degs = [(d + 180) % 360 - 180 for d in degs]
                self.angle_table.item(self.angle_table.get_children()[i], 
                                    values=[f"{d:.1f}" for d in degs])

    def setup_angle_display(self, parent):
        """Tabla de ángulos debajo del gráfico."""
        angle_frame = ttk.Frame(parent)
        angle_frame.pack(fill=tk.X, pady=2)  # Reducir padding
        
        self.angle_table = ttk.Treeview(angle_frame, columns=("theta1","theta2","theta3"), 
                                    show="headings", height=4)
        self.angle_table.pack(fill=tk.X, pady=2)  # Reducir padding
        
        self.angle_table.heading("theta1", text="Theta 1 (°)")
        self.angle_table.heading("theta2", text="Theta 2 (°)")
        self.angle_table.heading("theta3", text="Theta 3 (°)")
        
        # Reducir el ancho de las columnas
        self.angle_table.column("theta1", width=80, anchor="center")   # Reducir de 120 a 80
        self.angle_table.column("theta2", width=80, anchor="center")   # Reducir de 120 a 80
        self.angle_table.column("theta3", width=80, anchor="center")   # Reducir de 120 a 80
        
        for i in range(4):
            self.angle_table.insert("", "end", values=("--","--","--"))

    def setup_leg_controls(self, parent):
        leg_frame = ttk.LabelFrame(parent, text="Patas Individuales")
        leg_frame.pack(fill=tk.X, pady=5)
        leg_names = ["Delantera Izquierda", "Trasera Izquierda", "Trasera Derecha", "Delantera Derecha"]
        initial_positions = [
            [0.1,  0.10, 0],
            [-0.1, 0.10, 0],
            [-0.1,-0.10, 0],
            [0.10, -0.10, 0],
        ]
        
        # Configurar grid para mejor organización
        for i, name in enumerate(leg_names):
            init = initial_positions[i]
            leg_vars = {c: tk.DoubleVar(value=v) for c, v in zip(['x', 'y', 'z'], init)}
            self.leg_vars.append(leg_vars)
            
            # Frame para cada pata
            leg_row_frame = ttk.Frame(leg_frame)
            leg_row_frame.pack(fill=tk.X, pady=2)
            
            # Etiqueta de la pata
            ttk.Label(leg_row_frame, text=name, font=('Arial', 9, 'bold'), width=18).pack(side=tk.LEFT, padx=2)
            
            # Frame para controles XYZ
            controls_frame = ttk.Frame(leg_row_frame)
            controls_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)
            
            for j, coord in enumerate(['x','y','z']):
                coord_frame = ttk.Frame(controls_frame)
                coord_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
                
                # Etiqueta de coordenada
                ttk.Label(coord_frame, text=f"{coord.upper()}", width=2, 
                        font=('Arial', 8, 'bold')).pack(side=tk.LEFT)
                
                # Slider más compacto
                start = init[['x','y','z'].index(coord)] - 0.15
                end   = init[['x','y','z'].index(coord)] + 0.15
                
                # Slider más corto
                slider = ttk.Scale(coord_frame, from_=start, to=end,
                        orient=tk.HORIZONTAL, variable=leg_vars[coord],
                        command=lambda v, i=i: self.on_leg_slider_change(i),
                        length=60)  # Longitud reducida
                slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
                
                # Campo de entrada para valor numérico
                entry_frame = ttk.Frame(coord_frame)
                entry_frame.pack(side=tk.RIGHT, padx=2)
                
                entry = ttk.Entry(entry_frame, width=6, font=('Arial', 8), justify='center')
                entry.insert(0, f"{init[['x','y','z'].index(coord)]:.3f}")
                entry.pack(side=tk.TOP)
                
                # Guardar referencia al entry
                leg_vars[f'{coord}_entry'] = entry
                
                # Bind eventos al entry
                entry.bind('<Return>', lambda e, idx=i, c=coord: self.on_leg_entry_change(idx, c))
                entry.bind('<FocusOut>', lambda e, idx=i, c=coord: self.on_leg_entry_change(idx, c))
                
                # Etiqueta de valor más compacta (opcional, puedes quitarla si prefieres solo el entry)
                leg_vars[f'{coord}_label'] = ttk.Label(coord_frame, 
                        text=f"{init[['x','y','z'].index(coord)]:.3f}", 
                        width=5, font=('Arial', 8))
                leg_vars[f'{coord}_label'].pack(side=tk.RIGHT, padx=2)

        # AGREGAR BOTÓN DE VOLVER A POSICIONES INICIALES
        button_frame = ttk.Frame(leg_frame)
        button_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(button_frame, text="Volver a Posiciones Iniciales", 
                command=self.reset_to_initial_positions).pack(fill=tk.X, padx=5, pady=2)

    def on_leg_entry_change(self, leg_idx, coord):
        """Manejar cambios en los campos de entrada de las patas"""
        try:
            leg_vars = self.leg_vars[leg_idx]
            entry = leg_vars[f'{coord}_entry']
            new_value = float(entry.get())
            
            # Validar límites (ajusta según tus necesidades)
            min_val = leg_vars[coord].get() - 0.15  # Mismo rango que el slider
            max_val = leg_vars[coord].get() + 0.15
            
            if min_val <= new_value <= max_val:
                leg_vars[coord].set(new_value)
                leg_vars[f'{coord}_label'].config(text=f"{new_value:.3f}")
                self.update_visualization()
            else:
                # Valor fuera de rango, revertir al valor anterior
                entry.delete(0, tk.END)
                entry.insert(0, f"{leg_vars[coord].get():.3f}")
                messagebox.showwarning("Valor fuera de rango", 
                                    f"El valor debe estar entre {min_val:.3f} y {max_val:.3f}")
                
        except ValueError:
            # Entrada no válida, revertir al valor anterior
            entry = self.leg_vars[leg_idx][f'{coord}_entry']
            entry.delete(0, tk.END)
            entry.insert(0, f"{self.leg_vars[leg_idx][coord].get():.3f}")
            messagebox.showerror("Error", "Por favor ingresa un número válido")

    def on_calibration_change(self, leg_idx, servo_idx):
        """Actualizar etiqueta cuando cambia el slider de calibración"""
        cal_vars = self.calibration_vars[leg_idx][servo_idx]
        offset_value = cal_vars['offset'].get()
        cal_vars['label'].config(text=f"{offset_value:.1f}")
        
        # Forzar la actualización de la visualización
        self.update_visualization()

    def setup_platform_controls(self, parent):
        pf = ttk.LabelFrame(parent, text="Plataforma (Cuerpo)")
        pf.pack(fill=tk.X, pady=5)

        # Posición inicial de la base
        initial_pos = {'x': 0, 'y': 0.0, 'z': 0.19}
        initial_rot = {'roll': 0, 'pitch': 0.0, 'yaw': 0.0}

        pos_vars = {c: tk.DoubleVar(value=initial_pos[c]) for c in ['x','y','z']}
        rot_vars = {a: tk.DoubleVar(value=initial_rot[a]) for a in ['roll','pitch','yaw']}
        
        # Inicializar diccionarios para almacenar entries - CORREGIDO
        pos_entries = {}
        rot_entries = {}
        
        # Frame para posición
        pos_frame = ttk.Frame(pf)
        pos_frame.pack(fill=tk.X, pady=2)
        ttk.Label(pos_frame, text="Posición:", font=('Arial', 9, 'bold'), width=10).pack(side=tk.LEFT)
        
        for coord in ['x','y','z']:
            coord_frame = ttk.Frame(pos_frame)
            coord_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
            
            ttk.Label(coord_frame, text=f"{coord.upper()}", width=2, 
                    font=('Arial', 8, 'bold')).pack(side=tk.LEFT)
            
            # Slider más compacto para posición
            slider = ttk.Scale(coord_frame, from_=-0.25, to=0.25,
                    orient=tk.HORIZONTAL, variable=pos_vars[coord],
                    command=lambda v, c=coord: self.on_platform_slider_change('position', c),
                    length=50)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
            
            # Campo de entrada para posición - CORREGIDO: almacenar en diccionario separado
            entry_frame = ttk.Frame(coord_frame)
            entry_frame.pack(side=tk.RIGHT, padx=2)
            
            entry = ttk.Entry(entry_frame, width=6, font=('Arial', 8), justify='center')
            entry.insert(0, f"{initial_pos[coord]:.3f}")
            entry.pack(side=tk.TOP)
            
            # Guardar referencia al entry en el diccionario SEPARADO - CORREGIDO
            pos_entries[coord] = entry
            
            # Bind eventos al entry
            entry.bind('<Return>', lambda e, c=coord: self.on_platform_pos_entry_change(c))
            entry.bind('<FocusOut>', lambda e, c=coord: self.on_platform_pos_entry_change(c))
            
            # Etiqueta de valor
            pos_vars[f'{coord}_label'] = ttk.Label(coord_frame, 
                    text=f"{initial_pos[coord]:.3f}", width=5, font=('Arial', 8))
            pos_vars[f'{coord}_label'].pack(side=tk.RIGHT, padx=2)
        
        # Frame para rotación
        rot_frame = ttk.Frame(pf)
        rot_frame.pack(fill=tk.X, pady=2)
        ttk.Label(rot_frame, text="Rotación:", font=('Arial', 9, 'bold'), width=10).pack(side=tk.LEFT)
        
        for ang in ['roll','pitch','yaw']:
            ang_frame = ttk.Frame(rot_frame)
            ang_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=1)
            
            ttk.Label(ang_frame, text=ang[0].upper(), width=2,
                    font=('Arial', 8, 'bold')).pack(side=tk.LEFT)
            
            # Slider más compacto para rotación
            slider = ttk.Scale(ang_frame, from_=-180, to=180,
                    orient=tk.HORIZONTAL, variable=rot_vars[ang],
                    command=lambda v, a=ang: self.on_platform_slider_change('rotation', a),
                    length=50)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
            
            # Campo de entrada para rotación - CORREGIDO: almacenar en diccionario separado
            entry_frame = ttk.Frame(ang_frame)
            entry_frame.pack(side=tk.RIGHT, padx=2)
            
            entry = ttk.Entry(entry_frame, width=6, font=('Arial', 8), justify='center')
            entry.insert(0, f"{initial_rot[ang]:.1f}")
            entry.pack(side=tk.TOP)
            
            # Guardar referencia al entry en el diccionario SEPARADO - CORREGIDO
            rot_entries[ang] = entry
            
            # Bind eventos al entry
            entry.bind('<Return>', lambda e, a=ang: self.on_platform_rot_entry_change(a))
            entry.bind('<FocusOut>', lambda e, a=ang: self.on_platform_rot_entry_change(a))
            
            # Etiqueta de valor
            rot_vars[f'{ang}_label'] = ttk.Label(ang_frame,
                    text=f"{initial_rot[ang]:.1f}", width=5, font=('Arial', 8))
            rot_vars[f'{ang}_label'].pack(side=tk.RIGHT, padx=2)
        
        # Guardar las referencias en las variables principales - CORREGIDO
        self.platform_vars = {
            'position': pos_vars, 
            'rotation': rot_vars,
            'position_entries': pos_entries,  # Diccionario separado para entries de posición
            'rotation_entries': rot_entries   # Diccionario separado para entries de rotación
        }

    def on_platform_pos_entry_change(self, coord):
        """Manejar cambios en los campos de entrada de posición de la plataforma - CORREGIDO"""
        try:
            pos_vars = self.platform_vars['position']
            # CORREGIDO: Acceder al entry desde el diccionario separado
            entry = self.platform_vars['position_entries'][coord]
            new_value = float(entry.get())
            
            # Validar límites
            if -0.25 <= new_value <= 0.25:
                pos_vars[coord].set(new_value)
                pos_vars[f'{coord}_label'].config(text=f"{new_value:.3f}")
                self.update_visualization()
            else:
                # Valor fuera de rango
                entry.delete(0, tk.END)
                entry.insert(0, f"{pos_vars[coord].get():.3f}")
                messagebox.showwarning("Valor fuera de rango", 
                                    "El valor debe estar entre -0.25 y 0.25")
                
        except ValueError:
            # Entrada no válida
            entry = self.platform_vars['position_entries'][coord]
            entry.delete(0, tk.END)
            entry.insert(0, f"{self.platform_vars['position'][coord].get():.3f}")
            messagebox.showerror("Error", "Por favor ingresa un número válido")

    def on_platform_rot_entry_change(self, ang):
        """Manejar cambios en los campos de entrada de rotación de la plataforma - CORREGIDO"""
        try:
            rot_vars = self.platform_vars['rotation']
            # CORREGIDO: Acceder al entry desde el diccionario separado
            entry = self.platform_vars['rotation_entries'][ang]
            new_value = float(entry.get())
            
            # Validar límites
            if -180 <= new_value <= 180:
                rot_vars[ang].set(new_value)
                rot_vars[f'{ang}_label'].config(text=f"{new_value:.1f}")
                self.update_visualization()
            else:
                # Valor fuera de rango
                entry.delete(0, tk.END)
                entry.insert(0, f"{rot_vars[ang].get():.1f}")
                messagebox.showwarning("Valor fuera de rango", 
                                    "El valor debe estar entre -180 y 180")
                
        except ValueError:
            # Entrada no válida
            entry = self.platform_vars['rotation_entries'][ang]
            entry.delete(0, tk.END)
            entry.insert(0, f"{self.platform_vars['rotation'][ang].get():.1f}")
            messagebox.showerror("Error", "Por favor ingresa un número válido")

    def on_platform_slider_change(self, control_type, axis):
        """Manejar cambios en los sliders de la plataforma - CORREGIDO"""
        if control_type == 'position':
            value = self.platform_vars['position'][axis].get()
            # Actualizar etiqueta
            self.platform_vars['position'][f'{axis}_label'].config(text=f"{value:.3f}")
            # Actualizar campo de entrada - CORREGIDO: usar diccionario separado
            if axis in self.platform_vars['position_entries']:
                entry = self.platform_vars['position_entries'][axis]
                entry.delete(0, tk.END)
                entry.insert(0, f"{value:.3f}")
        else:
            value = self.platform_vars['rotation'][axis].get()
            # Actualizar etiqueta
            self.platform_vars['rotation'][f'{axis}_label'].config(text=f"{value:.1f}")
            # Actualizar campo de entrada - CORREGIDO: usar diccionario separado
            if axis in self.platform_vars['rotation_entries']:
                entry = self.platform_vars['rotation_entries'][axis]
                entry.delete(0, tk.END)
                entry.insert(0, f"{value:.1f}")
        
        # Actualizar visualización
        self.update_visualization()

    def reset_to_initial_positions(self):
        """Restablecer todas las patas a sus posiciones iniciales - CORREGIDO"""
        initial_positions = [
            [0.10,  0.1, 0],   # Delantera Izquierda
            [-0.10, 0.1, 0],   # Trasera Izquierda  
            [-0.10,-0.1, 0],   # Trasera Derecha
            [0.10, -0.1, 0],   # Delantera Derecha
        ]
        
        # Restablecer posiciones de las patas
        for i, leg_vars in enumerate(self.leg_vars):
            if i < len(initial_positions):
                leg_vars['x'].set(initial_positions[i][0])
                leg_vars['y'].set(initial_positions[i][1]) 
                leg_vars['z'].set(initial_positions[i][2])
                
                # Actualizar etiquetas y campos de entrada
                leg_vars['x_label'].config(text=f"{initial_positions[i][0]:.3f}")
                leg_vars['y_label'].config(text=f"{initial_positions[i][1]:.3f}")
                leg_vars['z_label'].config(text=f"{initial_positions[i][2]:.3f}")
                
                # Actualizar campos de entrada
                leg_vars['x_entry'].delete(0, tk.END)
                leg_vars['x_entry'].insert(0, f"{initial_positions[i][0]:.3f}")
                leg_vars['y_entry'].delete(0, tk.END)
                leg_vars['y_entry'].insert(0, f"{initial_positions[i][1]:.3f}")
                leg_vars['z_entry'].delete(0, tk.END)
                leg_vars['z_entry'].insert(0, f"{initial_positions[i][2]:.3f}")
        
        # Restablecer plataforma - CORREGIDO
        initial_platform_pos = {'x': 0, 'y': 0.0, 'z': 0.19}
        initial_platform_rot = {'roll': 0, 'pitch': 0.0, 'yaw': 0.0}
        
        for coord, value in initial_platform_pos.items():
            self.platform_vars['position'][coord].set(value)
            self.platform_vars['position'][f'{coord}_label'].config(text=f"{value:.3f}")
            # CORREGIDO: Actualizar campo de entrada usando el diccionario separado
            if coord in self.platform_vars['position_entries']:
                self.platform_vars['position_entries'][coord].delete(0, tk.END)
                self.platform_vars['position_entries'][coord].insert(0, f"{value:.3f}")
        
        for ang, value in initial_platform_rot.items():
            self.platform_vars['rotation'][ang].set(value)
            self.platform_vars['rotation'][f'{ang}_label'].config(text=f"{value:.1f}")
            # CORREGIDO: Actualizar campo de entrada usando el diccionario separado
            if ang in self.platform_vars['rotation_entries']:
                self.platform_vars['rotation_entries'][ang].delete(0, tk.END)
                self.platform_vars['rotation_entries'][ang].insert(0, f"{value:.1f}")
        
        # Actualizar visualización
        self.update_visualization()
        
        print("✅ Posiciones restablecidas a valores iniciales")

    def on_leg_slider_change(self, idx):
        leg_vars = self.leg_vars[idx]
        for c in ['x','y','z']:
            value = leg_vars[c].get()
            leg_vars[f'{c}_label'].config(text=f"{value:.3f}")
            # Actualizar campo de entrada también
            leg_vars[f'{c}_entry'].delete(0, tk.END)
            leg_vars[f'{c}_entry'].insert(0, f"{value:.3f}")
        self.update_visualization()

    def update_visualization(self):
        # NOTA: Ya no necesitamos actualizar los campos de entrada aquí porque
        # ahora se manejan en on_platform_slider_change y on_leg_slider_change
        
        # Obtener posiciones actuales de las patas
        foot_targets = [[lv['x'].get(), lv['y'].get(), lv['z'].get()] for lv in self.leg_vars]
        rot = [self.platform_vars['rotation'][a].get() for a in ['roll','pitch','yaw']]
        body_pos = [self.platform_vars['position'][a].get() for a in ['x','y','z']]
        
        # Obtener ángulos y errores físicos
        angles_list, physical_errors = self.kin.plot_robot(self.ax, foot_targets, rot, body_pos, is_radians=False)
        self.canvas.draw()
        
        # Procesar errores físicos
        physical_error_messages = []
        for leg_id in physical_errors:
            leg_names = ["Delantera Izquierda", "Trasera Izquierda", "Trasera Derecha", "Delantera Derecha"]
            physical_error_messages.append(f"PUNTO NO ALCANZABLE: {leg_names[leg_id]}")
        
        # Obtener ángulos para display (ya calibrados) - solo si no hay errores físicos
        if not physical_errors:
            display_angles_deg = self.get_display_angles_deg(angles_list)
        else:
            display_angles_deg = []
            # Mostrar "--" en la tabla para piernas con errores
            for i in range(4):
                if i in physical_errors:
                    self.angle_table.item(self.angle_table.get_children()[i], values=("--","--","--"))
        
        # Combinar todos los errores
        all_errors = []
        
        # Agregar errores físicos primero
        all_errors.extend(physical_error_messages)
        
        # Verificar límites de servos solo si no hay errores físicos
        if self.limits_loaded and not physical_errors:
            limit_errors = self.check_angle_limits(display_angles_deg)
            all_errors.extend(limit_errors)
        
        # Mostrar errores
        if all_errors:
            error_text = "ERRORES:\n" + "\n".join(all_errors)
            self.error_label.config(text=error_text)
            # No enviar ángulos si hay errores
            print("❌ No se envían ángulos - Errores detectados")
            
            # Mostrar "--" en la tabla para valores inválidos
            for i, angles in enumerate(angles_list):
                if any(np.isnan(angles)):
                    self.angle_table.item(self.angle_table.get_children()[i], values=("--","--","--"))
        else:
            self.error_label.config(text="")
            # Actualizar display de ángulos
            self.display_angles(angles_list)
            
            # Enviar ángulos calibrados al ESP32 si está activado
            if not physical_errors:  # Solo enviar si no hay errores físicos
                calibrated_flat_angles = self.get_calibrated_angles_for_esp32(angles_list)
                send_angles(calibrated_flat_angles)
        
        # Actualizar estado de conexión WebSocket
        self.update_websocket_status()

    

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotGUI(root)
    root.mainloop()
