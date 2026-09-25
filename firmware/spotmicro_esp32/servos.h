#pragma once
#include <Arduino.h>
#include "config.h"

namespace servos {

bool begin();
// Aplica los 12 ángulos (grados, 0-180) en orden de canal.
// Solo escribe por I2C los canales cuyo pulso cambió.
void setAll(const float (&angles)[cfg::NUM_SERVOS]);

}  // namespace servos
