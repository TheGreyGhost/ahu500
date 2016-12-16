#include "DebounceToggle.h"
#include "Debounce.h"

DebounceToggle::DebounceToggle()
{
}

DebounceToggle::DebounceToggle(int i_debounceDelay) : debounce(i_debounceDelay)
{
}

void DebounceToggle::update(boolean switchPressed, int elapsedMillis)
{
  debounce.update(switchPressed, elapsedMillis);
  boolean newSwitchState = debounce.isPressed();
  if (newSwitchState && !lastSwitchState) {
    togglePosition = !togglePosition;  
  }
  lastSwitchState = newSwitchState;
}

boolean DebounceToggle::isOn()
{
  return togglePosition;
}

