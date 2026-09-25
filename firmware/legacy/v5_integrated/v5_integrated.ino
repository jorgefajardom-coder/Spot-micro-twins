#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

using namespace websockets;

// ===============================
// CONFIGURACIÓN DE RED
// ===============================
const char* ssid = "TU_SSID";
const char* password = "TU_PASSWORD";

// ===============================
// CONFIGURACIÓN WEBSOCKET
// ===============================
WebsocketsServer socket_server;
WebsocketsClient ws_client;

// ===============================
// CONFIGURACIÓN MPU6050
// ===============================
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
float pitch = 0, roll = 0, yaw = 0;
unsigned long last_read = 0;
const unsigned long READ_INTERVAL = 100; // 100ms = 10Hz

// Variables para calibración y offsets
float accel_offsets[3] = {0, 0, 0};
float gyro_offsets[3] = {0, 0, 0};
bool imu_calibrated = false;

// Variables para el filtro complementario
float comp_angle_roll = 0, comp_angle_pitch = 0, comp_angle_yaw = 0;
unsigned long last_comp_time = 0;
const float COMP_FILTER_RATIO = 0.98;

// Variables para cálculo de yaw con giroscopio
float yaw_gyro = 0;
unsigned long last_yaw_time = 0;

// Controlar envío automático de datos
bool auto_send_data = false;
unsigned long last_auto_send = 0;
const unsigned long AUTO_SEND_INTERVAL = 100; // 100ms = 10Hz

// ===============================
// CONFIGURACIÓN PCA9685
// ===============================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x42);
const int SERVOMIN = 150; // valor PWM mínimo (0°)
const int SERVOMAX = 600; // valor PWM máximo (180°)
const int NUM_SERVOS = 12;

// ===============================
// CONFIGURACIÓN LEDs
// ===============================
const int LED_WIFI = 16;        // LED para conexión WiFi
const int LED_WEBSOCKET = 17;   // LED para conexión WebSocket
const int LED_SENDING = 18;     // LED para envío de datos del sensor
const int LED_RECEIVING = 27;  // LED para recepción de datos
const int LED_PCA = 26;        // ✅ NUEVO: LED para actividad PCA

// Variables para control de LEDs
bool wifi_connected = false;
bool websocket_connected = false;
bool sending_data = false;
bool receiving_data = false;
bool pca_active = false;       // ✅ NUEVO: Actividad PCA
unsigned long last_led_update = 0;
const unsigned long LED_UPDATE_INTERVAL = 100;

// Variables para control de tiempo de actividad
unsigned long sending_activity_time = 0;
unsigned long receiving_activity_time = 0;
unsigned long pca_activity_time = 0;  // ✅ NUEVO: Tiempo actividad PCA
const unsigned long ACTIVITY_DURATION = 300;

// Variable para verificar conexión WebSocket
unsigned long last_websocket_check = 0;
const unsigned long WEBSOCKET_CHECK_INTERVAL = 1000;

// ===============================
// FUNCIONES PCA9685
// ===============================

// Convierte grados (0-180) a pulsos PWM
uint16_t angleToPWM(float angle) {
  angle = constrain(angle, 0, 180);
  return SERVOMIN + (angle / 180.0) * (SERVOMAX - SERVOMIN);
}

// Mueve un servo específico
void moveServo(int servo_index, float angle) {
  if (servo_index >= 0 && servo_index < NUM_SERVOS) {
    pwm.setPWM(servo_index, 0, angleToPWM(angle));
    
    // ✅ NUEVO: Indicar actividad PCA
    pca_active = true;
    pca_activity_time = millis();
    
    Serial.printf("⚙️ Servo %d → %.2f°\n", servo_index + 1, angle);
  }
}

// ✅ CORREGIDO: Manejar array lineal de 12 ángulos
void moveAllServosLinear(JsonArray angles_array) {
  if (angles_array.size() == NUM_SERVOS) {
    Serial.println("🎯 Moviendo servos (formato lineal):");
    
    for (int i = 0; i < NUM_SERVOS; i++) {
      float angle = angles_array[i].as<float>();
      moveServo(i, angle);
      
      // Imprimir en formato de patas
      if (i % 3 == 0) {
        Serial.printf("  Pata %d: ", (i / 3) + 1);
      }
      Serial.printf("%.1f°", angle);
      if (i % 3 == 2) {
        Serial.println();
      } else {
        Serial.print(", ");
      }
    }
    Serial.println("----------------------------------------");
    
    // Confirmar movimiento
    if (websocket_connected) {
      receiving_data = true;
      receiving_activity_time = millis();
      ws_client.send("{\"status\":\"angles_received_linear\"}");
    }
  } else {
    Serial.printf("❌ Error: Se esperaban %d ángulos, se recibieron %d\n", 
                  NUM_SERVOS, angles_array.size());
  }
}

// ✅ MANTENER: Para formato de array de arrays (4x3)
void moveAllServosNested(JsonArray angles_array) {
  if (angles_array.size() >= 4) {
    Serial.println("🎯 Moviendo servos (formato anidado):");
    
    for (int leg = 0; leg < 4; leg++) {
      JsonArray leg_angles = angles_array[leg];
      if (leg_angles.size() >= 3) {
        // Calcular índices de servos para esta pata
        int base_index = leg * 3;
        
        // Mover los 3 servos de la pata
        moveServo(base_index, leg_angles[0].as<float>());
        moveServo(base_index + 1, leg_angles[1].as<float>());
        moveServo(base_index + 2, leg_angles[2].as<float>());
        
        Serial.printf("  Pata %d: %.1f°, %.1f°, %.1f°\n", 
                     leg + 1, 
                     leg_angles[0].as<float>(), 
                     leg_angles[1].as<float>(), 
                     leg_angles[2].as<float>());
      }
    }
    Serial.println("----------------------------------------");
    
    // Confirmar movimiento
    if (websocket_connected) {
      receiving_data = true;
      receiving_activity_time = millis();
      ws_client.send("{\"status\":\"angles_received_nested\"}");
    }
  }
}

// ===============================
// FUNCIONES MPU6050
// ===============================

void calculateOrientation() {
  // Aplicar offsets a los datos del acelerómetro
  float ax = a.acceleration.x - accel_offsets[0];
  float ay = a.acceleration.y - accel_offsets[1];
  float az = a.acceleration.z - accel_offsets[2];

  // Cálculo de orientación a partir del acelerómetro
  float accel_roll = atan2(ay, az) * 180 / PI;
  float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
  
  // Aplicar offsets al giroscopio y calcular cambio de ángulo
  float gyro_x = g.gyro.x - gyro_offsets[0];
  float gyro_y = g.gyro.y - gyro_offsets[1];
  float gyro_z = g.gyro.z - gyro_offsets[2];
  
  // Calcular intervalo de tiempo
  unsigned long current_time = millis();
  float dt = (current_time - last_comp_time) / 1000.0;
  last_comp_time = current_time;
  
  // Integrar giroscopio para obtener ángulos
  float gyro_roll = comp_angle_roll + gyro_x * dt;
  float gyro_pitch = comp_angle_pitch + gyro_y * dt;
  
  // Filtro complementario
  comp_angle_roll = COMP_FILTER_RATIO * gyro_roll + (1.0 - COMP_FILTER_RATIO) * accel_roll;
  comp_angle_pitch = COMP_FILTER_RATIO * gyro_pitch + (1.0 - COMP_FILTER_RATIO) * accel_pitch;
  
  // Cálculo de YAW con giroscopio
  if (last_yaw_time > 0) {
    float yaw_dt = (current_time - last_yaw_time) / 1000.0;
    yaw_gyro += gyro_z * yaw_dt * 180 / PI;
    
    // Normalizar yaw entre -180 y 180 grados
    if (yaw_gyro > 180) yaw_gyro -= 360;
    if (yaw_gyro < -180) yaw_gyro += 360;
  }
  last_yaw_time = current_time;
  
  // Asignar los valores finales
  roll = comp_angle_roll;
  pitch = comp_angle_pitch;
  yaw = yaw_gyro;
}

void sendMPUData() {
  DynamicJsonDocument doc(512);
  
  doc["sensor"] = "mpu6050";
  
  // Datos del acelerómetro
  JsonObject accel = doc.createNestedObject("accel");
  accel["x"] = a.acceleration.x - accel_offsets[0];
  accel["y"] = a.acceleration.y - accel_offsets[1];
  accel["z"] = a.acceleration.z - accel_offsets[2];
  
  // Datos del giroscopio
  JsonObject gyro = doc.createNestedObject("gyro");
  gyro["x"] = g.gyro.x - gyro_offsets[0];
  gyro["y"] = g.gyro.y - gyro_offsets[1];
  gyro["z"] = g.gyro.z - gyro_offsets[2];
  
  // Orientación
  JsonObject orientation = doc.createNestedObject("orientation");
  orientation["roll"] = roll;
  orientation["pitch"] = pitch;
  orientation["yaw"] = yaw;
  
  // Temperatura
  doc["temp"] = temp.temperature;
  doc["calibrated"] = imu_calibrated;
  
  String output;
  serializeJson(doc, output);
  
  if (websocket_connected) {
    sending_data = true;
    sending_activity_time = millis();
    ws_client.send(output);
    
    Serial.print("📊 MPU - Roll: ");
    Serial.print(roll);
    Serial.print("°, Pitch: ");
    Serial.print(pitch);
    Serial.print("°, Yaw: ");
    Serial.print(yaw);
    Serial.println("°");
  }
}

void calibrateIMU() {
  Serial.println("🎯 Calibrando IMU...");
  Serial.println("📍 Mantén el robot inmóvil y plano...");
  
  const int num_samples = 100;
  float accel_sum[3] = {0, 0, 0};
  float gyro_sum[3] = {0, 0, 0};
  
  for (int i = 0; i < num_samples; i++) {
    mpu.getEvent(&a, &g, &temp);
    
    accel_sum[0] += a.acceleration.x;
    accel_sum[1] += a.acceleration.y;
    accel_sum[2] += a.acceleration.z;
    
    gyro_sum[0] += g.gyro.x;
    gyro_sum[1] += g.gyro.y;
    gyro_sum[2] += g.gyro.z;
    
    delay(30);
  }
  
  // Calcular offsets
  accel_offsets[0] = accel_sum[0] / num_samples;
  accel_offsets[1] = accel_sum[1] / num_samples;
  accel_offsets[2] = (accel_sum[2] / num_samples) - 9.81;
  
  gyro_offsets[0] = gyro_sum[0] / num_samples;
  gyro_offsets[1] = gyro_sum[1] / num_samples;
  gyro_offsets[2] = gyro_sum[2] / num_samples;
  
  imu_calibrated = true;
  resetOrientation();
  
  Serial.println("✅ Calibración completada");
}

void resetOrientation() {
  Serial.println("🔄 Reseteando orientación...");
  
  pitch = 0;
  roll = 0;
  yaw = 0;
  yaw_gyro = 0;
  
  comp_angle_roll = 0;
  comp_angle_pitch = 0;
  comp_angle_yaw = 0;
  
  last_comp_time = millis();
  last_yaw_time = millis();
  
  mpu.getEvent(&a, &g, &temp);
  calculateOrientation();
  
  Serial.println("✅ Orientación reseteada");
}

void sendOffsets() {
  DynamicJsonDocument doc(512);
  
  doc["status"] = "offsets_data";
  
  JsonArray accel_offsets_array = doc.createNestedArray("accel_offsets");
  accel_offsets_array.add(accel_offsets[0]);
  accel_offsets_array.add(accel_offsets[1]);
  accel_offsets_array.add(accel_offsets[2]);
  
  JsonArray gyro_offsets_array = doc.createNestedArray("gyro_offsets");
  gyro_offsets_array.add(gyro_offsets[0]);
  gyro_offsets_array.add(gyro_offsets[1]);
  gyro_offsets_array.add(gyro_offsets[2]);
  
  String output;
  serializeJson(doc, output);
  
  if (websocket_connected) {
    sending_data = true;
    sending_activity_time = millis();
    ws_client.send(output);
    Serial.println("📊 Offsets enviados");
  }
}

// ===============================
// FUNCIONES COMANDOS Y PROCESAMIENTO
// ===============================

void setManualOffsets(String data) {
  int firstColon = data.indexOf(':');
  int secondColon = data.indexOf(':', firstColon + 1);
  String offsetsStr = data.substring(secondColon + 1);
  
  float values[6];
  int lastIndex = 0;
  
  for (int i = 0; i < 6; i++) {
    int nextIndex = offsetsStr.indexOf(',', lastIndex);
    if (nextIndex == -1) nextIndex = offsetsStr.length();
    
    String valueStr = offsetsStr.substring(lastIndex, nextIndex);
    values[i] = valueStr.toFloat();
    lastIndex = nextIndex + 1;
  }
  
  accel_offsets[0] = values[0];
  accel_offsets[1] = values[1];
  accel_offsets[2] = values[2];
  gyro_offsets[0] = values[3];
  gyro_offsets[1] = values[4];
  gyro_offsets[2] = values[5];
  
  imu_calibrated = true;
  resetOrientation();
  
  Serial.println("💾 Offsets manuales aplicados");
}

void processCommand(String command) {
  Serial.print("🎯 Procesando comando: ");
  Serial.println(command);
  
  if (command == "cmd:calibrate_imu") {
    calibrateIMU();
    if (websocket_connected) {
      sending_data = true;
      sending_activity_time = millis();
      ws_client.send("{\"status\":\"IMU calibrated\"}");
    }
  }
  else if (command == "cmd:reset_imu") {
    resetOrientation();
    if (websocket_connected) {
      sending_data = true;
      sending_activity_time = millis();
      ws_client.send("{\"status\":\"IMU reset\"}");
    }
  }
  else if (command == "cmd:get_offsets") {
    sendOffsets();
  }
  else if (command.startsWith("cmd:set_offsets:")) {
    setManualOffsets(command);
    if (websocket_connected) {
      sending_data = true;
      sending_activity_time = millis();
      ws_client.send("{\"status\":\"Offsets updated\"}");
    }
  }
  else if (command == "cmd:get_mpu_data") {
    mpu.getEvent(&a, &g, &temp);
    calculateOrientation();
    sendMPUData();
  }
  else if (command == "cmd:start_mpu_stream") {
    auto_send_data = true;
    Serial.println("🚀 Stream MPU ACTIVADO");
    if (websocket_connected) {
      sending_data = true;
      sending_activity_time = millis();
      ws_client.send("{\"status\":\"MPU stream started\"}");
    }
  }
  else if (command == "cmd:stop_mpu_stream") {
    auto_send_data = false;
    Serial.println("🛑 Stream MPU DESACTIVADO");
    if (websocket_connected) {
      sending_data = true;
      sending_activity_time = millis();
      ws_client.send("{\"status\":\"MPU stream stopped\"}");
    }
  }
  else {
    Serial.println("❌ Comando no reconocido");
  }
}

// ✅ CORREGIDO: Función para procesar JSON que maneja ambos formatos
void processJSON(String jsonData) {
  DynamicJsonDocument doc(1024); // Aumentar tamaño para arrays grandes
  DeserializationError error = deserializeJson(doc, jsonData);
  
  if (error) {
    Serial.print("❌ Error parseando JSON: ");
    Serial.println(error.c_str());
    return;
  }
  
  if (doc.containsKey("command")) {
    String command = doc["command"].as<String>();
    
    if (command == "get_mpu_data") {
      mpu.getEvent(&a, &g, &temp);
      calculateOrientation();
      sendMPUData();
    }
    else if (command == "start_mpu_stream") {
      auto_send_data = true;
      Serial.println("🚀 Stream MPU ACTIVADO");
    }
    else if (command == "stop_mpu_stream") {
      auto_send_data = false;
      Serial.println("🛑 Stream MPU DESACTIVADO");
    }
  }
  // ✅ CORREGIDO: Manejar array lineal de 12 ángulos
  else if (doc.is<JsonArray>()) {
    JsonArray angles_array = doc.as<JsonArray>();
    
    Serial.printf("📊 Array recibido con %d elementos\n", angles_array.size());
    
    // Detectar formato automáticamente
    if (angles_array.size() == NUM_SERVOS) {
      // Formato lineal: [ang1, ang2, ..., ang12]
      Serial.println("🔹 Formato lineal detectado (12 ángulos)");
      moveAllServosLinear(angles_array);
    }
    else if (angles_array.size() == 4 && angles_array[0].is<JsonArray>()) {
      // Formato anidado: [[ang1,ang2,ang3], [ang4,ang5,ang6], ...]
      Serial.println("🔹 Formato anidado detectado (4x3 ángulos)");
      moveAllServosNested(angles_array);
    }
    else {
      Serial.printf("❌ Formato no reconocido. Tamaño: %d\n", angles_array.size());
    }
  }
}

// ===============================
// FUNCIONES WEBSOCKET
// ===============================

bool isWebSocketConnected() {
  if (!ws_client.available()) {
    return false;
  }
  
  static unsigned long last_ping = 0;
  if (millis() - last_ping > 5000) {
    last_ping = millis();
    return ws_client.ping();
  }
  
  return true;
}

void setupWebSocketHandlers() {
  ws_client.onMessage([&](WebsocketsClient &client, WebsocketsMessage message) {
    String data = message.data();
    
    receiving_data = true;
    receiving_activity_time = millis();
    
    Serial.print("📩 Mensaje recibido: ");
    Serial.println(data);
    
    if (data.startsWith("cmd:")) {
      processCommand(data);
    }
    else if (data.startsWith("{") || data.startsWith("[")) {
      processJSON(data);
    }
    else {
      Serial.println("❌ Formato no reconocido");
    }
  });
  
  ws_client.onEvent([&](WebsocketsClient &client, WebsocketsEvent event, String data) {
    if (event == WebsocketsEvent::ConnectionOpened) {
      Serial.println("✅ WebSocket conectado");
      websocket_connected = true;
      client.send("{\"status\":\"Conectado al ESP32\"}");
    } else if (event == WebsocketsEvent::ConnectionClosed) {
      Serial.println("❌ WebSocket desconectado");
      websocket_connected = false;
    } else if (event == WebsocketsEvent::GotPing) {
      Serial.println("🏓 Ping recibido");
    } else if (event == WebsocketsEvent::GotPong) {
      Serial.println("🏓 Pong enviado");
    }
  });
}

// ===============================
// FUNCIONES LEDs
// ===============================

void updateLEDs() {
  unsigned long current_time = millis();
  
  if (current_time - last_led_update >= LED_UPDATE_INTERVAL) {
    last_led_update = current_time;
    
    // LED WiFi
    digitalWrite(LED_WIFI, wifi_connected ? HIGH : LOW);
    
    // LED WebSocket
    digitalWrite(LED_WEBSOCKET, websocket_connected ? HIGH : LOW);
    
    // LED Envío
    static bool sending_led_state = false;
    bool sending_active = (current_time - sending_activity_time < ACTIVITY_DURATION);
    digitalWrite(LED_SENDING, sending_active ? (sending_led_state ? HIGH : LOW) : LOW);
    if (sending_active) sending_led_state = !sending_led_state;
    
    // LED Recepción
    static bool receiving_led_state = false;
    bool receiving_active = (current_time - receiving_activity_time < ACTIVITY_DURATION);
    digitalWrite(LED_RECEIVING, receiving_active ? (receiving_led_state ? HIGH : LOW) : LOW);
    if (receiving_active) receiving_led_state = !receiving_led_state;
    
    // ✅ NUEVO: LED PCA
    static bool pca_led_state = false;
    bool pca_active_now = (current_time - pca_activity_time < ACTIVITY_DURATION);
    digitalWrite(LED_PCA, pca_active_now ? (pca_led_state ? HIGH : LOW) : LOW);
    if (pca_active_now) pca_led_state = !pca_led_state;
    
    // Debug cada 2 segundos
    static unsigned long last_debug = 0;
    if (current_time - last_debug >= 2000) {
      last_debug = current_time;
      Serial.print("💡 LEDs - WiFi:");
      Serial.print(wifi_connected ? "ON" : "OFF");
      Serial.print(", WS:");
      Serial.print(websocket_connected ? "ON" : "OFF");
      Serial.print(", Send:");
      Serial.print(sending_active ? "SI" : "NO");
      Serial.print(", Recv:");
      Serial.print(receiving_active ? "SI" : "NO");
      Serial.print(", PCA:");
      Serial.println(pca_active_now ? "SI" : "NO");
    }
  }
}

// ===============================
// SETUP PRINCIPAL
// ===============================

void setup() {
  Serial.begin(115200);
  
  // Configurar LEDs
  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED_WEBSOCKET, OUTPUT);
  pinMode(LED_SENDING, OUTPUT);
  pinMode(LED_RECEIVING, OUTPUT);
  pinMode(LED_PCA, OUTPUT); // ✅ NUEVO: LED PCA
  
  digitalWrite(LED_WIFI, LOW);
  digitalWrite(LED_WEBSOCKET, LOW);
  digitalWrite(LED_SENDING, LOW);
  digitalWrite(LED_RECEIVING, LOW);
  digitalWrite(LED_PCA, LOW);
  
  // Inicializar MPU6050
  Serial.println("Inicializando MPU6050...");
  if (!mpu.begin()) {
    Serial.println("❌ Error MPU6050!");
    while (1) delay(10);
  }
  Serial.println("✅ MPU6050 OK");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // ✅ NUEVO: Inicializar PCA9685
  Serial.println("Inicializando PCA9685...");
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  Serial.println("✅ PCA9685 OK");
  
  // Conectar WiFi
  WiFi.begin(ssid, password);
  Serial.print("Conectando WiFi");
  
  unsigned long wifi_timeout = millis() + 15000;
  while (WiFi.status() != WL_CONNECTED && millis() < wifi_timeout) {
    Serial.print(".");
    digitalWrite(LED_WIFI, !digitalRead(LED_WIFI));
    delay(250);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    digitalWrite(LED_WIFI, HIGH);
    Serial.println();
    Serial.print("✅ WiFi OK. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("❌ Error WiFi");
    digitalWrite(LED_WIFI, LOW);
  }

  socket_server.listen(81);
  Serial.println("🛰️ WebSocket en puerto 81");
}

// ===============================
// LOOP PRINCIPAL
// ===============================

void loop() {
  // Verificar WiFi
  static unsigned long last_wifi_check = 0;
  if (millis() - last_wifi_check >= 5000) {
    last_wifi_check = millis();
    bool previous_wifi_state = wifi_connected;
    wifi_connected = (WiFi.status() == WL_CONNECTED);
    
    if (wifi_connected != previous_wifi_state) {
      Serial.println(wifi_connected ? "✅ WiFi reconectado" : "❌ WiFi desconectado");
    }
  }

  // Verificar WebSocket
  if (millis() - last_websocket_check >= WEBSOCKET_CHECK_INTERVAL) {
    last_websocket_check = millis();
    
    bool previous_websocket_state = websocket_connected;
    
    if (!ws_client.available()) {
      ws_client = socket_server.accept();
      if (ws_client.available()) {
        Serial.println("🤝 Cliente WebSocket conectado");
        setupWebSocketHandlers();
        websocket_connected = true;
        last_comp_time = millis();
        last_yaw_time = millis();
      } else {
        websocket_connected = false;
      }
    } else {
      websocket_connected = isWebSocketConnected();
      
      if (!websocket_connected) {
        Serial.println("🔌 WebSocket desconectado");
        ws_client.close();
      }
    }
    
    if (websocket_connected != previous_websocket_state) {
      Serial.println(websocket_connected ? "🔵 WebSocket CONECTADO" : "🔴 WebSocket DESCONECTADO");
    }
  }

  if (websocket_connected) {
    ws_client.poll();
    
    // Leer datos MPU6050
    if (millis() - last_read >= READ_INTERVAL) {
      mpu.getEvent(&a, &g, &temp);
      calculateOrientation();
      last_read = millis();
    }
    
    // Envío automático
    if (auto_send_data && millis() - last_auto_send >= AUTO_SEND_INTERVAL) {
      sendMPUData();
      last_auto_send = millis();
    }
  }

  // Actualizar LEDs
  updateLEDs();

  delay(10);
}