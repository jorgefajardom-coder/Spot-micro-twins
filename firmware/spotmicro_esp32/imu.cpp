#include "imu.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "config.h"

namespace imu {
namespace {

Adafruit_MPU6050 mpu;
bool present = false;
bool calibrated = false;

float accel_off[3] = {0, 0, 0};
float gyro_off[3] = {0, 0, 0};

// Última lectura ya corregida con offsets
float accel[3] = {0, 0, 0};  // m/s²
float gyro[3] = {0, 0, 0};   // rad/s
float temperature = 0;

float roll = 0, pitch = 0, yaw = 0;  // grados
uint32_t last_update_us = 0;
bool has_prev_sample = false;

void read() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  accel[0] = a.acceleration.x - accel_off[0];
  accel[1] = a.acceleration.y - accel_off[1];
  accel[2] = a.acceleration.z - accel_off[2];
  gyro[0] = g.gyro.x - gyro_off[0];
  gyro[1] = g.gyro.y - gyro_off[1];
  gyro[2] = g.gyro.z - gyro_off[2];
  temperature = t.temperature;
}

void fuse(float dt) {
  const float accel_roll = atan2f(accel[1], accel[2]) * RAD_TO_DEG;
  const float accel_pitch =
      atan2f(-accel[0], sqrtf(accel[1] * accel[1] + accel[2] * accel[2])) * RAD_TO_DEG;

  if (!has_prev_sample) {
    // Primera muestra: arrancar desde el acelerómetro, sin integrar
    roll = accel_roll;
    pitch = accel_pitch;
    has_prev_sample = true;
    return;
  }

  // El giroscopio viene en rad/s: convertir a grados antes de fusionar
  // (V5 mezclaba rad con grados en roll/pitch).
  const float a = cfg::COMP_FILTER_ALPHA;
  roll = a * (roll + gyro[0] * RAD_TO_DEG * dt) + (1.0f - a) * accel_roll;
  pitch = a * (pitch + gyro[1] * RAD_TO_DEG * dt) + (1.0f - a) * accel_pitch;

  yaw += gyro[2] * RAD_TO_DEG * dt;
  if (yaw > 180.0f) yaw -= 360.0f;
  else if (yaw < -180.0f) yaw += 360.0f;
}

}  // namespace

bool begin() {
  present = mpu.begin();
  if (!present) return false;
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  resetOrientation();
  return true;
}

bool ok() { return present; }

void update() {
  if (!present) return;
  const uint32_t now = micros();
  const uint32_t elapsed = now - last_update_us;
  if (elapsed < cfg::IMU_UPDATE_US) return;
  last_update_us = now;

  read();
  // Si el loop estuvo bloqueado mucho tiempo (p. ej. calibrando), no integrar ese hueco
  if (elapsed > 500000) has_prev_sample = false;
  fuse(elapsed * 1e-6f);
}

void calibrate() {
  if (!present) return;
  LOGI("Calibrando IMU: mantén el robot inmóvil y plano...\n");

  float a_sum[3] = {0, 0, 0}, g_sum[3] = {0, 0, 0};
  sensors_event_t a, g, t;
  for (uint16_t i = 0; i < cfg::IMU_CALIB_SAMPLES; i++) {
    mpu.getEvent(&a, &g, &t);
    a_sum[0] += a.acceleration.x;
    a_sum[1] += a.acceleration.y;
    a_sum[2] += a.acceleration.z;
    g_sum[0] += g.gyro.x;
    g_sum[1] += g.gyro.y;
    g_sum[2] += g.gyro.z;
    delay(cfg::IMU_CALIB_DELAY_MS);
  }

  const float n = cfg::IMU_CALIB_SAMPLES;
  accel_off[0] = a_sum[0] / n;
  accel_off[1] = a_sum[1] / n;
  accel_off[2] = a_sum[2] / n - cfg::GRAVITY;
  for (int i = 0; i < 3; i++) gyro_off[i] = g_sum[i] / n;

  calibrated = true;
  resetOrientation();
  LOGI("Calibración completada\n");
}

void resetOrientation() {
  roll = pitch = yaw = 0;
  has_prev_sample = false;
  last_update_us = micros();
  if (present) {
    read();
    fuse(0);
  }
}

void setOffsets(const float a[3], const float g[3]) {
  for (int i = 0; i < 3; i++) {
    accel_off[i] = a[i];
    gyro_off[i] = g[i];
  }
  calibrated = true;
  resetOrientation();
}

void writeTelemetry(JsonDocument& doc) {
  doc["sensor"] = "mpu6050";
  JsonObject a = doc["accel"].to<JsonObject>();
  a["x"] = accel[0];
  a["y"] = accel[1];
  a["z"] = accel[2];
  JsonObject g = doc["gyro"].to<JsonObject>();
  g["x"] = gyro[0];
  g["y"] = gyro[1];
  g["z"] = gyro[2];
  JsonObject o = doc["orientation"].to<JsonObject>();
  o["roll"] = roll;
  o["pitch"] = pitch;
  o["yaw"] = yaw;
  doc["temp"] = temperature;
  doc["calibrated"] = calibrated;
}

void writeOffsets(JsonDocument& doc) {
  doc["status"] = "offsets_data";
  JsonArray a = doc["accel_offsets"].to<JsonArray>();
  JsonArray g = doc["gyro_offsets"].to<JsonArray>();
  for (int i = 0; i < 3; i++) {
    a.add(accel_off[i]);
    g.add(gyro_off[i]);
  }
}

}  // namespace imu
