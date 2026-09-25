#pragma once
#include <Arduino.h>

// ===============================
// CREDENCIALES WIFI (fuera del repo)
// ===============================
#if __has_include("secrets.h")
#include "secrets.h"
#else
#warning "secrets.h no existe: copia secrets.example.h a secrets.h y pon tu WiFi"
#include "secrets.example.h"
#endif

// 1 = logs detallados por paquete (lento). 0 = solo eventos importantes.
#ifndef SPOT_DEBUG
#define SPOT_DEBUG 0
#endif

#if SPOT_DEBUG
#define LOGD(...) Serial.printf(__VA_ARGS__)
#else
#define LOGD(...) do {} while (0)
#endif
#define LOGI(...) Serial.printf(__VA_ARGS__)

namespace cfg {

// ----- Red -----
constexpr char     HOSTNAME[]          = "spotmicro";  // ws://spotmicro.local:81
constexpr uint16_t WS_PORT             = 81;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
// Enviar {"status":"angles_received_*"} por cada paquete de ángulos.
// Desactivado: el notebook no lo usa y, si nadie lee el socket, llena el buffer TCP.
constexpr bool     ACK_ANGLE_PACKETS   = false;

// ----- I2C -----
constexpr uint32_t I2C_CLOCK_HZ        = 400000;  // PCA9685 y MPU6050 soportan Fast-mode

// ----- PCA9685 / servos -----
constexpr uint8_t  PCA9685_ADDR        = 0x42;
constexpr uint16_t SERVO_FREQ_HZ       = 50;
constexpr uint16_t SERVO_PWM_MIN       = 150;  // 0°
constexpr uint16_t SERVO_PWM_MAX       = 600;  // 180°
constexpr uint8_t  NUM_LEGS            = 4;
constexpr uint8_t  JOINTS_PER_LEG      = 3;
constexpr uint8_t  NUM_SERVOS          = NUM_LEGS * JOINTS_PER_LEG;

// ----- MPU6050 -----
constexpr uint32_t IMU_UPDATE_US       = 10000;  // filtro a 100 Hz
constexpr uint32_t IMU_STREAM_MS       = 100;    // stream a 10 Hz
constexpr float    COMP_FILTER_ALPHA   = 0.98f;
constexpr uint16_t IMU_CALIB_SAMPLES   = 100;
constexpr uint16_t IMU_CALIB_DELAY_MS  = 10;
constexpr float    GRAVITY             = 9.81f;

// ----- LEDs -----
constexpr uint8_t  LED_WIFI            = 16;
constexpr uint8_t  LED_WEBSOCKET       = 17;
constexpr uint8_t  LED_SENDING         = 18;
constexpr uint8_t  LED_RECEIVING       = 27;
constexpr uint8_t  LED_PCA             = 26;
constexpr uint32_t LED_BLINK_MS        = 100;
constexpr uint32_t ACTIVITY_MS         = 300;

}  // namespace cfg
