#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

using namespace websockets;

const char* ssid = "TU_SSID";
const char* password = "TU_PASSWORD";

WebsocketsServer socket_server;
WebsocketsClient ws_client;
Adafruit_MPU6050 mpu;

// Variables para el sensor
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

// ✅ NUEVO: LEDs indicadores
const int LED_WIFI = 13;        // LED para conexión WiFi (GPIO2 - LED integrado en muchas placas ESP32)
const int LED_WEBSOCKET = 12;   // LED para conexión WebSocket
const int LED_SENDING = 14;     // LED para envío de datos del sensor
const int LED_RECEIVING = 27;  // LED para recepción de datos

// ✅ NUEVO: Variables para control de LEDs
bool wifi_connected = false;
bool websocket_connected = false;
bool sending_data = false;
bool receiving_data = false;
unsigned long last_led_update = 0;
const unsigned long LED_UPDATE_INTERVAL = 100; // Actualizar LEDs cada 100ms (más rápido para mejor parpadeo)

// ✅ NUEVO: Variables para control de tiempo de actividad
unsigned long sending_activity_time = 0;
unsigned long receiving_activity_time = 0;
const unsigned long ACTIVITY_DURATION = 300; // Los LEDs de actividad permanecen activos por 300ms

// ✅ NUEVO: Variable para verificar conexión WebSocket
unsigned long last_websocket_check = 0;
const unsigned long WEBSOCKET_CHECK_INTERVAL = 1000; // Verificar cada segundo

void setup() {
  Serial.begin(115200);
  
  // ✅ NUEVO: Configurar pines de LEDs
  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED_WEBSOCKET, OUTPUT);
  pinMode(LED_SENDING, OUTPUT);
  pinMode(LED_RECEIVING, OUTPUT);
  
  // Apagar todos los LEDs inicialmente
  digitalWrite(LED_WIFI, LOW);
  digitalWrite(LED_WEBSOCKET, LOW);
  digitalWrite(LED_SENDING, LOW);
  digitalWrite(LED_RECEIVING, LOW);
  
  // Inicializar MPU6050
  Serial.println("Inicializando MPU6050...");
  if (!mpu.begin()) {
    Serial.println("❌ Error al inicializar MPU6050!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("✅ MPU6050 inicializado correctamente");
  
  // Configurar MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Conectar WiFi
  WiFi.begin(ssid, password);
  Serial.print("Conectando WiFi");
  
  unsigned long wifi_timeout = millis() + 15000; // Timeout de 15 segundos
  while (WiFi.status() != WL_CONNECTED && millis() < wifi_timeout) {
    Serial.print(".");
    // Parpadeo rápido del LED WiFi durante conexión
    digitalWrite(LED_WIFI, !digitalRead(LED_WIFI));
    delay(250);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    digitalWrite(LED_WIFI, HIGH); // LED fijo cuando conectado
    Serial.println();
    Serial.print("✅ WiFi conectado. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("❌ Error: No se pudo conectar al WiFi");
    digitalWrite(LED_WIFI, LOW);
  }

  socket_server.listen(81);
  Serial.println("🛰️ Servidor WebSocket escuchando en puerto 81");
}

// ✅ NUEVO: Función mejorada para verificar conexión WebSocket
bool isWebSocketConnected() {
  if (!ws_client.available()) {
    return false;
  }
  
  // Intentar enviar un ping para verificar la conexión
  static unsigned long last_ping = 0;
  if (millis() - last_ping > 5000) { // Ping cada 5 segundos
    last_ping = millis();
    return ws_client.ping(); // Devuelve true si el ping se envía correctamente
  }
  
  return true;
}

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
  float dt = (current_time - last_comp_time) / 1000.0; // Convertir a segundos
  last_comp_time = current_time;
  
  // Integrar giroscopio para obtener ángulos
  float gyro_roll = comp_angle_roll + gyro_x * dt;
  float gyro_pitch = comp_angle_pitch + gyro_y * dt;
  
  // Filtro complementario: combinar acelerómetro y giroscopio
  comp_angle_roll = COMP_FILTER_RATIO * gyro_roll + (1.0 - COMP_FILTER_RATIO) * accel_roll;
  comp_angle_pitch = COMP_FILTER_RATIO * gyro_pitch + (1.0 - COMP_FILTER_RATIO) * accel_pitch;
  
  // CÁLCULO DE YAW MEJORADO - Solo con giroscopio (integración)
  if (last_yaw_time > 0) {
    float yaw_dt = (current_time - last_yaw_time) / 1000.0;
    yaw_gyro += gyro_z * yaw_dt * 180 / PI; // Convertir a grados
    
    // Normalizar yaw entre -180 y 180 grados
    if (yaw_gyro > 180) yaw_gyro -= 360;
    if (yaw_gyro < -180) yaw_gyro += 360;
  }
  last_yaw_time = current_time;
  
  // Asignar los valores finales
  roll = comp_angle_roll;
  pitch = comp_angle_pitch;
  yaw = yaw_gyro; // Usar el yaw calculado con giroscopio
}

void sendMPUData() {
  DynamicJsonDocument doc(512);
  
  doc["sensor"] = "mpu6050";
  
  // Datos del acelerómetro (con offsets aplicados)
  JsonObject accel = doc.createNestedObject("accel");
  accel["x"] = a.acceleration.x - accel_offsets[0];
  accel["y"] = a.acceleration.y - accel_offsets[1];
  accel["z"] = a.acceleration.z - accel_offsets[2];
  
  // Datos del giroscopio (con offsets aplicados)
  JsonObject gyro = doc.createNestedObject("gyro");
  gyro["x"] = g.gyro.x - gyro_offsets[0];
  gyro["y"] = g.gyro.y - gyro_offsets[1];
  gyro["z"] = g.gyro.z - gyro_offsets[2];
  
  // Orientación - INCLUYENDO YAW
  JsonObject orientation = doc.createNestedObject("orientation");
  orientation["roll"] = roll;
  orientation["pitch"] = pitch;
  orientation["yaw"] = yaw; // ✅ Yaw incluido
  
  // Temperatura
  doc["temp"] = temp.temperature;

  // Estado de calibración
  doc["calibrated"] = imu_calibrated;
  
  String output;
  serializeJson(doc, output);
  
  if (websocket_connected) {
    sending_data = true;
    sending_activity_time = millis(); // ✅ Registrar tiempo de actividad
    ws_client.send(output);
    
    // ✅ NUEVO: Imprimir ángulos en consola
    Serial.print("📊 Datos MPU - Roll: ");
    Serial.print(roll);
    Serial.print("°, Pitch: ");
    Serial.print(pitch);
    Serial.print("°, Yaw: ");
    Serial.print(yaw);
    Serial.println("°");
  }
}

void calibrateIMU() {
  Serial.println("🎯 Iniciando calibración del IMU...");
  Serial.println("📍 Mantén el robot inmóvil y plano durante 3 segundos...");
  
  const int num_samples = 100;
  float accel_sum[3] = {0, 0, 0};
  float gyro_sum[3] = {0, 0, 0};
  
  // Tomar muestras para calibración
  for (int i = 0; i < num_samples; i++) {
    mpu.getEvent(&a, &g, &temp);
    
    accel_sum[0] += a.acceleration.x;
    accel_sum[1] += a.acceleration.y;
    accel_sum[2] += a.acceleration.z;
    
    gyro_sum[0] += g.gyro.x;
    gyro_sum[1] += g.gyro.y;
    gyro_sum[2] += g.gyro.z;
    
    delay(30); // 30ms entre muestras
  }
  
  // Calcular offsets
  accel_offsets[0] = accel_sum[0] / num_samples;
  accel_offsets[1] = accel_sum[1] / num_samples;
  accel_offsets[2] = (accel_sum[2] / num_samples) - 9.81; // Restar gravedad
  
  gyro_offsets[0] = gyro_sum[0] / num_samples;
  gyro_offsets[1] = gyro_sum[1] / num_samples;
  gyro_offsets[2] = gyro_sum[2] / num_samples;
  
  imu_calibrated = true;
  
  // Resetear orientación después de calibrar
  resetOrientation();
  
  Serial.println("✅ Calibración completada");
  Serial.print("📊 Offsets Acelerómetro: ");
  Serial.print(accel_offsets[0]); Serial.print(", ");
  Serial.print(accel_offsets[1]); Serial.print(", ");
  Serial.println(accel_offsets[2]);
  
  Serial.print("📊 Offsets Giroscopio: ");
  Serial.print(gyro_offsets[0]); Serial.print(", ");
  Serial.print(gyro_offsets[1]); Serial.print(", ");
  Serial.println(gyro_offsets[2]);
}

void resetOrientation() {
  Serial.println("🔄 Reseteando orientación...");
  
  // Reiniciar ángulos de orientación
  pitch = 0;
  roll = 0;
  yaw = 0;
  yaw_gyro = 0; // ✅ Resetear también el yaw del giroscopio
  
  // Reiniciar ángulos del filtro complementario
  comp_angle_roll = 0;
  comp_angle_pitch = 0;
  comp_angle_yaw = 0;
  
  // Reiniciar tiempo para el filtro
  last_comp_time = millis();
  last_yaw_time = millis(); // ✅ Resetear tiempo del yaw
  
  // Forzar una lectura inmediata para recalcular con los nuevos valores
  mpu.getEvent(&a, &g, &temp);
  calculateOrientation();
  
  Serial.println("✅ Orientación reseteada a 0°");
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
    sending_activity_time = millis(); // ✅ Registrar tiempo de actividad
    ws_client.send(output);
    Serial.println("📊 Offsets enviados");
  }
}

void setManualOffsets(String data) {
  // Formato esperado: "cmd:set_offsets:ax,ay,az,gx,gy,gz"
  int firstColon = data.indexOf(':');
  int secondColon = data.indexOf(':', firstColon + 1);
  String offsetsStr = data.substring(secondColon + 1);
  
  // Parsear los valores
  float values[6];
  int currentIndex = 0;
  int lastIndex = 0;
  
  for (int i = 0; i < 6; i++) {
    int nextIndex = offsetsStr.indexOf(',', lastIndex);
    if (nextIndex == -1) nextIndex = offsetsStr.length();
    
    String valueStr = offsetsStr.substring(lastIndex, nextIndex);
    values[i] = valueStr.toFloat();
    lastIndex = nextIndex + 1;
  }
  
  // Aplicar offsets
  accel_offsets[0] = values[0];
  accel_offsets[1] = values[1];
  accel_offsets[2] = values[2];
  gyro_offsets[0] = values[3];
  gyro_offsets[1] = values[4];
  gyro_offsets[2] = values[5];
  
  imu_calibrated = true;
  
  // Resetear orientación después de aplicar offsets manuales
  resetOrientation();
  
  Serial.println("💾 Offsets manuales aplicados:");
  Serial.print("Acelerómetro: ");
  Serial.print(accel_offsets[0]); Serial.print(", ");
  Serial.print(accel_offsets[1]); Serial.print(", ");
  Serial.println(accel_offsets[2]);
  Serial.print("Giroscopio: ");
  Serial.print(gyro_offsets[0]); Serial.print(", ");
  Serial.print(gyro_offsets[1]); Serial.print(", ");
  Serial.println(gyro_offsets[2]);
}

// ✅ NUEVO: Función para imprimir ángulos recibidos desde Python
void printReceivedAngles(String jsonData) {
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, jsonData);
  
  if (error) {
    Serial.print("❌ Error parseando JSON de ángulos: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Verificar si es un array de ángulos de servos
  if (doc.is<JsonArray>()) {
    JsonArray angles_array = doc.as<JsonArray>();
    
    if (angles_array.size() >= 4) {
      Serial.println("🎯 Ángulos de servos recibidos desde Python:");
      for (int i = 0; i < 4; i++) {
        JsonArray leg_angles = angles_array[i];
        if (leg_angles.size() >= 3) {
          Serial.print("  Pata "); 
          Serial.print(i + 1); 
          Serial.print(": ");
          Serial.print(leg_angles[0].as<float>()); Serial.print("°, ");
          Serial.print(leg_angles[1].as<float>()); Serial.print("°, ");
          Serial.print(leg_angles[2].as<float>()); Serial.println("°");
        }
      }
      Serial.println("----------------------------------------");
    }
  }
}

void processCommand(String command) {
  Serial.print("🎯 Procesando comando: ");
  Serial.println(command);
  
  if (command == "cmd:calibrate_imu") {
    calibrateIMU();
    if (websocket_connected) {
      sending_data = true;
      sending_activity_time = millis(); // ✅ Registrar tiempo de actividad
      ws_client.send("{\"status\":\"IMU calibrated\"}");
    }
  }
  else if (command == "cmd:reset_imu") {
    resetOrientation();
    if (websocket_connected) {
      sending_data = true;
      sending_activity_time = millis(); // ✅ Registrar tiempo de actividad
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
      sending_activity_time = millis(); // ✅ Registrar tiempo de actividad
      ws_client.send("{\"status\":\"Offsets updated\"}");
    }
  }
  else if (command == "cmd:get_mpu_data") {
    // Leer y enviar datos inmediatamente
    mpu.getEvent(&a, &g, &temp);
    calculateOrientation();
    sendMPUData();
  }
  // Comandos para controlar envío automático de datos
  else if (command == "cmd:start_mpu_stream") {
    auto_send_data = true;
    Serial.println("🚀 Envío automático de datos MPU ACTIVADO");
    if (websocket_connected) {
      sending_data = true;
      sending_activity_time = millis(); // ✅ Registrar tiempo de actividad
      ws_client.send("{\"status\":\"MPU stream started\"}");
    }
  }
  else if (command == "cmd:stop_mpu_stream") {
    auto_send_data = false;
    Serial.println("🛑 Envío automático de datos MPU DESACTIVADO");
    if (websocket_connected) {
      sending_data = true;
      sending_activity_time = millis(); // ✅ Registrar tiempo de actividad
      ws_client.send("{\"status\":\"MPU stream stopped\"}");
    }
  }
  else {
    Serial.println("❌ Comando no reconocido: " + command);
  }
}

void processJSON(String jsonData) {
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, jsonData);
  
  if (error) {
    Serial.print("❌ Error parseando JSON: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Comando para obtener datos del MPU
  if (doc.containsKey("command")) {
    String command = doc["command"].as<String>();
    
    if (command == "get_mpu_data") {
      // Leer y enviar datos inmediatamente
      mpu.getEvent(&a, &g, &temp);
      calculateOrientation();
      sendMPUData();
    }
    // Comandos para controlar envío automático
    else if (command == "start_mpu_stream") {
      auto_send_data = true;
      Serial.println("🚀 Envío automático de datos MPU ACTIVADO");
    }
    else if (command == "stop_mpu_stream") {
      auto_send_data = false;
      Serial.println("🛑 Envío automático de datos MPU DESACTIVADO");
    }
  }
  // ✅ NUEVO: Procesar ángulos de servos (array de 4x3) e imprimirlos
  else if (doc.is<JsonArray>()) {
    printReceivedAngles(jsonData);
    
    // Aquí iría el código para mover los servos reales
    JsonArray angles_array = doc.as<JsonArray>();
    
    if (angles_array.size() >= 4) {
      for (int i = 0; i < 4; i++) {
        JsonArray leg_angles = angles_array[i];
        if (leg_angles.size() >= 3) {
          // Ejemplo de cómo mover servos reales:
          // moverServo(i, 0, leg_angles[0].as<float>());
          // moverServo(i, 1, leg_angles[1].as<float>());
          // moverServo(i, 2, leg_angles[2].as<float>());
        }
      }
    }
  }
}

void setupWebSocketHandlers() {
  ws_client.onMessage([&](WebsocketsClient &client, WebsocketsMessage message) {
    String data = message.data();
    
    // ✅ NUEVO: Indicar recepción de datos
    receiving_data = true;
    receiving_activity_time = millis(); // ✅ Registrar tiempo de actividad
    
    Serial.print("📩 Mensaje recibido: ");
    Serial.println(data);
    
    // Procesar comandos de texto (empiezan con "cmd:")
    if (data.startsWith("cmd:")) {
      processCommand(data);
    }
    // Procesar JSON
    else if (data.startsWith("{") || data.startsWith("[")) {
      processJSON(data);
    }
    else {
      Serial.println("❌ Formato de mensaje no reconocido");
    }
  });
  
  ws_client.onEvent([&](WebsocketsClient &client, WebsocketsEvent event, String data) {
    if (event == WebsocketsEvent::ConnectionOpened) {
      Serial.println("✅ Conexión WebSocket establecida");
      websocket_connected = true;
      client.send("{\"status\":\"Conectado al ESP32\"}");
    } else if (event == WebsocketsEvent::ConnectionClosed) {
      Serial.println("❌ Conexión WebSocket cerrada");
      websocket_connected = false;
    } else if (event == WebsocketsEvent::GotPing) {
      Serial.println("🏓 Ping recibido");
    } else if (event == WebsocketsEvent::GotPong) {
      Serial.println("🏓 Pong enviado");
    }
  });
}

// ✅ NUEVO: Función para actualizar los LEDs
void updateLEDs() {
  unsigned long current_time = millis();
  
  if (current_time - last_led_update >= LED_UPDATE_INTERVAL) {
    last_led_update = current_time;
    
    // LED WiFi - fijo cuando conectado
    digitalWrite(LED_WIFI, wifi_connected ? HIGH : LOW);
    
    // LED WebSocket - fijo cuando conectado
    digitalWrite(LED_WEBSOCKET, websocket_connected ? HIGH : LOW);
    
    // LED Envío - parpadeo rápido cuando hay actividad reciente
    static bool sending_led_state = false;
    bool sending_active = (current_time - sending_activity_time < ACTIVITY_DURATION);
    
    if (sending_active) {
      digitalWrite(LED_SENDING, sending_led_state ? HIGH : LOW);
      sending_led_state = !sending_led_state;
    } else {
      digitalWrite(LED_SENDING, LOW);
      sending_led_state = false;
    }
    
    // LED Recepción - parpadeo rápido cuando hay actividad reciente
    static bool receiving_led_state = false;
    bool receiving_active = (current_time - receiving_activity_time < ACTIVITY_DURATION);
    
    if (receiving_active) {
      digitalWrite(LED_RECEIVING, receiving_led_state ? HIGH : LOW);
      receiving_led_state = !receiving_led_state;
    } else {
      digitalWrite(LED_RECEIVING, LOW);
      receiving_led_state = false;
    }
    
    // Debug: imprimir estado de LEDs cada 2 segundos
    static unsigned long last_debug = 0;
    if (current_time - last_debug >= 2000) {
      last_debug = current_time;
      Serial.print("💡 Estado LEDs - WiFi: ");
      Serial.print(wifi_connected ? "ON" : "OFF");
      Serial.print(", WebSocket: ");
      Serial.print(websocket_connected ? "ON" : "OFF");
      Serial.print(", Enviando: ");
      Serial.print(sending_active ? "SI" : "NO");
      Serial.print(", Recibiendo: ");
      Serial.println(receiving_active ? "SI" : "NO");
    }
  }
}

void loop() {
  // ✅ NUEVO: Verificar estado WiFi periódicamente
  static unsigned long last_wifi_check = 0;
  if (millis() - last_wifi_check >= 5000) { // Cada 5 segundos
    last_wifi_check = millis();
    bool previous_wifi_state = wifi_connected;
    wifi_connected = (WiFi.status() == WL_CONNECTED);
    
    if (wifi_connected != previous_wifi_state) {
      if (wifi_connected) {
        Serial.println("✅ WiFi reconectado");
      } else {
        Serial.println("❌ WiFi desconectado");
      }
    }
  }

  // ✅ NUEVO: Verificar conexión WebSocket periódicamente
  if (millis() - last_websocket_check >= WEBSOCKET_CHECK_INTERVAL) {
    last_websocket_check = millis();
    
    bool previous_websocket_state = websocket_connected;
    
    if (!ws_client.available()) {
      // Intentar aceptar nueva conexión
      ws_client = socket_server.accept();
      if (ws_client.available()) {
        Serial.println("🤝 Cliente WebSocket conectado");
        setupWebSocketHandlers();
        websocket_connected = true;
        // Inicializar tiempo para el filtro complementario
        last_comp_time = millis();
        last_yaw_time = millis();
      } else {
        websocket_connected = false;
      }
    } else {
      // Verificar si el cliente existente sigue conectado
      websocket_connected = isWebSocketConnected();
      
      if (!websocket_connected) {
        Serial.println("🔌 WebSocket desconectado (verificación fallida)");
        ws_client.close(); // Cerrar conexión inválida
      }
    }
    
    // Actualizar estado WebSocket si cambió
    if (websocket_connected != previous_websocket_state) {
      if (websocket_connected) {
        Serial.println("🔵 WebSocket CONECTADO");
      } else {
        Serial.println("🔴 WebSocket DESCONECTADO");
      }
    }
  }

  if (websocket_connected) {
    // Procesar mensajes WebSocket
    ws_client.poll();
    
    // Leer datos del MPU6050 siempre, pero enviar solo si está habilitado
    if (millis() - last_read >= READ_INTERVAL) {
      mpu.getEvent(&a, &g, &temp);
      calculateOrientation();
      last_read = millis();
    }
    
    // Envío automático de datos solo si está habilitado
    if (auto_send_data && millis() - last_auto_send >= AUTO_SEND_INTERVAL) {
      sendMPUData();
      last_auto_send = millis();
    }
  }

  // ✅ NUEVO: Actualizar LEDs
  updateLEDs();

  delay(10);
}