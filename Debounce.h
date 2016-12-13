/* Debounce a switch; switch must be depressed for a given time before it changes state.  Uses hysteresis.
  
Usage:
  (1) construct with desired debounce delay in ms
  (2) call update(position, elapsedmillis) periodically 
  (3) isPressed() to read debounced value
  */
#ifndef Debounce_h   // if x.h hasn't been included yet...
#define Debounce_h   //   #define this so the compiler knows it has been included
#include <Arduino.h>

class Debounce
{
  public:
    Debounce();
    Debounce(int i_debounceDelay);
    void update(boolean switchPressed, int elapsedMillis);
    boolean isPressed();

  private:
    int debounceDelayMS;
    int debounceCounter = 0;
    boolean lastState;  
};

#endif
