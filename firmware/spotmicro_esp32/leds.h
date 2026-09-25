#pragma once
#include <Arduino.h>

namespace leds {

enum class Activity : uint8_t { Sending, Receiving, Pca };

void begin();
void setWifi(bool on);
void setWebSocket(bool on);
void markActivity(Activity a);
void update();  // llamar en cada loop, no bloquea

}  // namespace leds
