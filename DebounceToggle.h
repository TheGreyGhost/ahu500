/* Toggle switch with debounce
 *  Switch must be depressed for a given time before it toggles to the other state.
 *  Once the switch changes state, it must be released before it will toggle again
  
Usage:
  (1) construct with desired debounce delay in ms
  (2) call update(position, elapsedmillis) periodically 
  (3) isPressed() to read debounced value
  */
#ifndef DebounceToggle_h   // if x.h hasn't been included yet...
#define DebounceToggle_h   //   #define this so the compiler knows it has been included
#include <Arduino.h>
#include "Debounce.h"

class DebounceToggle
{
  public:
    DebounceToggle();
    DebounceToggle(int i_debounceDelay);
    void update(boolean switchPressed, int elapsedMillis);
    boolean isOn();

  private:
    boolean lastSwitchState = false;  
    boolean togglePosition = false;
    Debounce debounce;
};

#endif
