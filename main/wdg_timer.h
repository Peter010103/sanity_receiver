/*      Implementation for a simple watchdog timer.
  This is implemented in an 'hourglass' style without callbacks.
  A function/thread/task "A" that needs to prove its liveliness can call 
  `reset()` method with the object (i.e., flip the hourglass) whenever
  it can.
  Another function/thread/task "B" that needs to check expiration can call
  `expired()` method with the same object (returns bool).

  Generally, an expired watchdog may not be reset later simply by "A" waking
  up again (needs explicit reinit, usually due to another event within "A").
  For convenience, while resetting periodically, "A" can also specify a flag
  to indicate this re-init, and request that the watchdog be resettable.
  The expiration check allows overriding an expiration if such re-init is
  indicated. This is done during the expiry check (which, presumably, happens
  less often), and is implemented as an abjuncton (NIMPLY operation).

  note: adapted to C from freyja_utils::Watchdog.

  ~ aj // Jan 2025.
 */  

struct wdg_timer {
  TickType_t tick_last_;
  uint16_t timeout_ms_;
  bool can_override_expiry_;
  bool is_expired_;
};

void wdg_reset(struct wdg_timer* WT, bool override_exp) {
  WT->tick_last_ = xTaskGetTickCount();
  WT->can_override_expiry_ = override_exp;
}

bool wdg_expired(struct wdg_timer *WT) {
  uint16_t elapsed_ms = (xTaskGetTickCount() - WT->tick_last_)*portTICK_PERIOD_MS;
  WT->is_expired_ = (WT->is_expired_ || elapsed_ms > WT->timeout_ms_) && (!WT->can_override_expiry_);
  return WT->is_expired_;
}

void wdg_init(struct wdg_timer* WT, const uint16_t t) {
  WT->timeout_ms_ = t;
  WT->is_expired_ = false;
  wdg_reset(WT, true);
}

void wdg_reinit(struct wdg_timer* WT) {
  WT->can_override_expiry_ = true;
}
