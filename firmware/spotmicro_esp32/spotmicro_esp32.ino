// SpotMicro V3 — firmware ESP32 (V6, refactor optimizado de V5)
// Hardware: ESP32 + PCA9685 (12 servos) + MPU6050, control por WebSocket.
// Antes de compilar: copia secrets.example.h a secrets.h.

#include <ESPmDNS.h>
#include <WiFi.h>
#include <Wire.h>
#include "comms.h"
#include "config.h"
#include "imu.h"
#include "leds.h"
#include "servos.h"

static bool wifi_up = false;

static void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(cfg::HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Sin modem-sleep: evita ~100 ms de latencia extra en cada paquete recibido
  WiFi.setSleep(false);

  LOGI("Conectando WiFi");
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < cfg::WIFI_CONNECT_TIMEOUT_MS) {
    leds::setWifi((millis() / 250) % 2);
    LOGI(".");
    delay(250);
  }
  LOGI("\n");
}

static void checkWifi() {
  static uint32_t last_check = 0;
  if (millis() - last_check < 1000) return;
  last_check = millis();

  const bool up = WiFi.status() == WL_CONNECTED;
  if (up == wifi_up) return;
  wifi_up = up;
  leds::setWifi(up);
  if (up) {
    LOGI("WiFi OK. IP: %s  (ws://%s:%u o ws://%s.local:%u)\n",
         WiFi.localIP().toString().c_str(), WiFi.localIP().toString().c_str(),
         cfg::WS_PORT, cfg::HOSTNAME, cfg::WS_PORT);
    MDNS.end();
    if (MDNS.begin(cfg::HOSTNAME)) MDNS.addService("ws", "tcp", cfg::WS_PORT);
  } else {
    LOGI("WiFi desconectado, reintentando...\n");
  }
}

void setup() {
  Serial.begin(115200);
  leds::begin();

  Wire.begin();
  // Si falta el IMU se sigue funcionando: los servos no dependen de él (V5 se colgaba aquí).
  LOGI(imu::begin() ? "MPU6050 OK\n" : "MPU6050 no encontrado, sigo sin IMU\n");
  LOGI(servos::begin() ? "PCA9685 OK\n" : "PCA9685 no responde\n");
  // Después de los begin(): las librerías llaman Wire.begin() internamente
  Wire.setClock(cfg::I2C_CLOCK_HZ);

  connectWifi();
  checkWifi();
  comms::begin();
}

void loop() {
  checkWifi();
  comms::loop();
  imu::update();
  leds::update();
  delay(1);  // cede al stack WiFi sin frenar el loop (V5 usaba 10 ms)
}
