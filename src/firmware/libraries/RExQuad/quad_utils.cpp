#include "quad_utils.hpp"

#include <Arduino.h>

namespace rexquad {

void Blink(int pin, int delay_ms, int n) {
  for (int i = 0; i < n; ++i) {
    digitalWrite(pin, HIGH);
    delay(delay_ms);
    digitalWrite(pin, LOW);
    delay(delay_ms);
  }
}

Heartbeat::Heartbeat()
    : timeout_ms_(kDefaultTimeoutMs), isactive_(false), time_last_pulse_ms_(millis()) {}

void Heartbeat::SetTimeoutMs(int timeout_ms) { timeout_ms_ = timeout_ms; }
int Heartbeat::GetTimeoutMs() const { return timeout_ms_; }
void Heartbeat::Activate() { isactive_ = true; }
void Heartbeat::Deactivate() { isactive_ = false; }
bool Heartbeat::IsActive() const { return isactive_;  }
void Heartbeat::Pulse() {
  time_last_pulse_ms_ = millis();
}
bool Heartbeat::IsDead() const {
  int time_now_ms = millis();
  int time_since_last_pulse_ms = time_now_ms - time_last_pulse_ms_;
  return time_since_last_pulse_ms > timeout_ms_;
}

}  // namespace rexquad