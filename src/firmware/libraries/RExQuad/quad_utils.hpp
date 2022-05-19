#pragma once

namespace rexquad {

void Blink(int pin, int delay_ms, int n);

class Heartbeat {
public:
static constexpr int kDefaultTimeoutMs = 500;

Heartbeat();
void SetTimeoutMs(int timeout_ms);
int GetTimeoutMs() const;
void Activate();
void Deactivate();
bool IsActive() const;
void Pulse();
bool IsDead();
bool IsNewlyDead() const;
bool IsNewlyAlive() const;

private:
  int timeout_ms_;
  bool isactive_;
  int time_last_pulse_ms_;
  bool isdead_;
  bool justdied_;
  bool justresurrected_;
};

}