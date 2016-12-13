#include "Debounce.h"

static const int DEFAULT_DELAY_MS = 100;

Debounce::Debounce()
{
  debounceDelayMS = DEFAULT_DELAY_MS;
}

Debounce::Debounce(int i_debounceDelay)
{
  debounceDelayMS = i_debounceDelay;
}

void Debounce::update(boolean switchPressed, int elapsedMillis)
{
  debounceCounter += (switchPressed ? elapsedMillis : -elapsedMillis);
  if (debounceCounter <= 0) {
    debounceCounter = 0;
    lastState = false;  
  } else if (debounceCounter >= debounceDelayMS) {
    debounceCounter = debounceDelayMS;
    lastState = true;
  }
}

boolean Debounce::isPressed()
{
  return lastState;
}

