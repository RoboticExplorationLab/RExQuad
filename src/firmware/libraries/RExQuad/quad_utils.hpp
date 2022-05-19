#pragma once

namespace rexquad {

void Blink(int pin, int delay_ms, int n);

/**
 * @brief A class to help check if a process is active or stale
 * 
 * If something should happen at a regular interval and you want to know if it's no 
 * longer happening (like incoming communications), this class will help you keep track of 
 * when it "dies" or "comes back."
 * 
 * Whenever the event you want to track occurs, you should use `Pulse()` to let the 
 * heartbeat know it's still going. If `Pulse()` isn't called after a given number of 
 * milliseconds, `IsDead()` will return `true`. To query if the status changed from the 
 * last time you called `IsDead()`, use `IsNewlyDead()` or `IsNewlyAlive()` after the call
 * to `IsDead()`. These cheap calls should typically occur right in the main loop.
 */
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