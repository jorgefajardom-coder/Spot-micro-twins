#pragma once
#include <ArduinoJson.h>

namespace imu {

bool begin();
bool ok();                 // false si el MPU6050 no respondió al arrancar
void update();             // lee y filtra a IMU_UPDATE_US; llamar en cada loop
void calibrate();          // bloquea ~1 s: robot quieto y plano
void resetOrientation();
void setOffsets(const float accel[3], const float gyro[3]);

// Rellenan el JSON con el mismo formato que V5 (compatible con el notebook).
void writeTelemetry(JsonDocument& doc);
void writeOffsets(JsonDocument& doc);

}  // namespace imu
