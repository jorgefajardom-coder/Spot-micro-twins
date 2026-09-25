#include "servos.h"
#include <Adafruit_PWMServoDriver.h>
#include "leds.h"

namespace servos {
namespace {

Adafruit_PWMServoDriver pwm(cfg::PCA9685_ADDR);

// 0xFFFF = canal nunca escrito, fuerza la primera escritura.
uint16_t last_pulse[cfg::NUM_SERVOS];

uint16_t angleToPulse(float angle) {
  angle = constrain(angle, 0.0f, 180.0f);
  return cfg::SERVO_PWM_MIN +
         static_cast<uint16_t>(angle * (cfg::SERVO_PWM_MAX - cfg::SERVO_PWM_MIN) / 180.0f + 0.5f);
}

}  // namespace

bool begin() {
  for (auto& p : last_pulse) p = 0xFFFF;
  if (!pwm.begin()) return false;
  pwm.setPWMFreq(cfg::SERVO_FREQ_HZ);
  return true;
}

void setAll(const float (&angles)[cfg::NUM_SERVOS]) {
  uint8_t written = 0;
  for (uint8_t ch = 0; ch < cfg::NUM_SERVOS; ch++) {
    const uint16_t pulse = angleToPulse(angles[ch]);
    if (pulse == last_pulse[ch]) continue;
    pwm.setPWM(ch, 0, pulse);
    last_pulse[ch] = pulse;
    written++;
  }
  if (written) leds::markActivity(leds::Activity::Pca);

#if SPOT_DEBUG
  for (uint8_t leg = 0; leg < cfg::NUM_LEGS; leg++) {
    const float* a = &angles[leg * cfg::JOINTS_PER_LEG];
    LOGD("  Pata %u: %.1f, %.1f, %.1f\n", leg + 1, a[0], a[1], a[2]);
  }
  LOGD("  (%u canales escritos)\n", written);
#endif
}

}  // namespace servos
