#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

using namespace websockets;

// ===============================
// CONFIGURACIÓN DE RED
// ===============================
const char* ssid     = "TU_SSID";
const char* password = "TU_PASSWORD";

// ===============================
// CONFIGURACIÓN WEBSOCKET SERVER
// ===============================
WebsocketsServer server;
WebsocketsClient activeClient;  // cliente activo persistente

const int NUM_SERVOS = 12;

// ===============================
// CONFIGURACIÓN PCA9685
// ===============================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x42);
const int SERVOMIN = 150; // valor PWM mínimo (0°)
const int SERVOMAX = 600; // valor PWM máximo (180°)

// Convierte grados (0-180) a pulsos PWM
uint16_t angleToPWM(float angle) {
  angle = constrain(angle, 0, 180);
  return SERVOMIN + (angle / 180.0) * (SERVOMAX - SERVOMIN);
}

// ===============================
// CALLBACK: cuando se recibe un mensaje
// ===============================
void onMessageCallback(WebsocketsClient &client, WebsocketsMessage msg) {
  String message = msg.data();
  Serial.println("\n📩 Mensaje recibido:");
  Serial.println(message);

  // Parsear JSON
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.print("❌ Error al parsear JSON: ");
    Serial.println(error.c_str());
    return;
  }

  if (!doc.is<JsonArray>() || doc.size() != NUM_SERVOS) {
    Serial.println("⚠️ Formato JSON inválido o tamaño incorrecto");
    return;
  }

  Serial.println("✅ Ángulos recibidos:");
  for (int i = 0; i < NUM_SERVOS; i++) {
    float angle = doc[i].as<float>();
    Serial.printf("Servo %d → %.2f°\n", i + 1, angle);

    // Enviar al PCA9685
    pwm.setPWM(i, 0, angleToPWM(angle));
  }
}

// ===============================
// SETUP
// ===============================
void setup() {
  Serial.begin(115200);

  // ----- Conectar WiFi -----
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("✅ WiFi conectado. IP local: ");
  Serial.println(WiFi.localIP());

  // ----- Iniciar servidor WebSocket -----
  server.listen(81);
  Serial.println("🌐 Servidor WebSocket escuchando en puerto 81");
  Serial.println("👉 Conéctate desde Python con: ws://<esta_IP>:81");

  // ----- Inicializar PCA9685 -----
  pwm.begin();
  pwm.setPWMFreq(50); // Frecuencia de servos 50Hz
  delay(10);

  Serial.println("✅ PCA9685 inicializado");
}

// ===============================
// LOOP PRINCIPAL
// ===============================
void loop() {
  // Si no hay cliente activo, intenta aceptar uno nuevo
  if (!activeClient.available()) {
    if (server.poll()) {
      activeClient = server.accept();
      if (activeClient.available()) {
        activeClient.onMessage(onMessageCallback);
        Serial.println("🔗 Cliente WebSocket conectado");
      }
    }
  }

  // Mantener la conexión viva
  if (activeClient.available()) {
    activeClient.poll();
  }

  delay(5);
}