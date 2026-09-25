#include "leds.h"
#include "config.h"

namespace leds {
namespace {

constexpr uint8_t ACTIVITY_PINS[] = {cfg::LED_SENDING, cfg::LED_RECEIVING, cfg::LED_PCA};
constexpr size_t NUM_ACTIVITY = sizeof(ACTIVITY_PINS);

uint32_t last_activity[NUM_ACTIVITY] = {};
bool has_activity[NUM_ACTIVITY] = {};
bool blink_state = false;
uint32_t last_blink = 0;

}  // namespace

void begin() {
  for (uint8_t pin : {cfg::LED_WIFI, cfg::LED_WEBSOCKET, cfg::LED_SENDING,
                      cfg::LED_RECEIVING, cfg::LED_PCA}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
}

void setWifi(bool on) { digitalWrite(cfg::LED_WIFI, on ? HIGH : LOW); }
void setWebSocket(bool on) { digitalWrite(cfg::LED_WEBSOCKET, on ? HIGH : LOW); }

void markActivity(Activity a) {
  const size_t i = static_cast<size_t>(a);
  last_activity[i] = millis();
  has_activity[i] = true;
}

void update() {
  const uint32_t now = millis();
  if (now - last_blink < cfg::LED_BLINK_MS) return;
  last_blink = now;
  blink_state = !blink_state;

  for (size_t i = 0; i < NUM_ACTIVITY; i++) {
    const bool active = has_activity[i] && (now - last_activity[i] < cfg::ACTIVITY_MS);
    digitalWrite(ACTIVITY_PINS[i], (active && blink_state) ? HIGH : LOW);
  }
}

}  // namespace leds
