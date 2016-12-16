#include <Arduino.h>
#include "SpeakerPCM.h"
#include "DebounceToggle.h"
#include "sintable.h"

const int HUMID_RED_DO_PIN = 2;
const int HUMID_GRN_DO_PIN = 4;
const int SPEAKER_DO_PIN = 9;
const int PRESS_METER_DO_PIN = 3;
const int TEMP_METER_DO_PIN = 5;
const int HUMID_METER_DO_PIN = 11;

const int HUMID_BUTTON_DI_PIN = 7;
const int PRESS_SETPT_AI_PIN = A0;
const int TEMP_AI_PIN = A1;

SpeakerPCM speakerPCM(SPEAKER_DO_PIN);

DebounceToggle debouncedHumiditySwitch;
unsigned int pressureFixPt6 = 0;  // current pressure with 6 bit fixed point
unsigned int PRESSURE_FP6_MIN = 0;
unsigned int PRESSURE_FP6_MAX = (256 << 6) - 1;

int temperature;
int humidity = random(80, 127);  // no particular significance to these numbers
boolean humidityControlActivated;
enum HumidityError {OFF, STABILISING, ERROR_DELAY, ACTIVE_ERROR};
HumidityError humidityError;

unsigned long lastMillis; 
unsigned int timeElapsedMillis;

void setup() {
  pinMode(HUMID_RED_DO_PIN, OUTPUT);
  pinMode(HUMID_GRN_DO_PIN, OUTPUT);
  pinMode(PRESS_METER_DO_PIN, OUTPUT);
  pinMode(TEMP_METER_DO_PIN, OUTPUT);
  pinMode(HUMID_METER_DO_PIN, OUTPUT);
  
  pinMode(HUMID_BUTTON_DI_PIN, INPUT_PULLUP);

  humidityControlActivated = false;
  lastMillis = millis();

//  Serial.begin(9600);
  speakerPCM.play(256);
}

void loop(void) {
  /*each loop:
   * Read the temperature
   * Write the temperature to the meter
   * Read the setpoint pressure
   * 
   * If the humidity control is off:
   *   Adjust the actual pressure in the direction of the setpoint
   *   Random walk on the humidity weighted toward 50%
   * 
   * If the humidity control is on: 
   *   Overlay a sinusuoidal oscillation on the pressure; initially small then gets bigger
   *   Random walk the humidity towards 100%, never decreasing
   *   After approx 3 seconds at 100% switch to the alert siren sound; start flashing Red LED error
   *       
   * Write the actual pressure to the meter
   * Change the sample playback speed to match the pressure
   * 
   * Write the humidity to the meter
   * Write the humidity control LEDs: 
   *   Red for control off; flashing for error
   *   Green for on; flashing for stabilising
   *   
   * 
  */ 

  unsigned long millisNow = millis();
  timeElapsedMillis = millisNow - lastMillis;
  lastMillis = millisNow;
  
  unsigned int targetPressure = analogRead(PRESS_SETPT_AI_PIN);
//  pressureFixPt6 = targetPressure << 4;

  updateTemperatureMeter();
  unsigned int displayPressure = updatePressure(targetPressure, timeElapsedMillis);
  unsigned int displayHumidity = updateHumidity(timeElapsedMillis);

//  
  updateAudioPlayback(humidityError, displayPressure);
//
  analogWrite(PRESS_METER_DO_PIN, displayPressure);
  analogWrite(HUMID_METER_DO_PIN, displayHumidity);
  writeHumidityControlLEDS(humidityError, timeElapsedMillis);
}

void updateTemperatureMeter()
{
    // our meter range is 12 C to 28 C
  const int TEMP_METER_MIN_RAW = 0;  // 12 C
  const int TEMP_METER_MAX_RAW = 1024;  // 28 C
  const int TEMP_METER_MIN_OUT = 0;
  const int TEMP_METER_MAX_OUT = 255;

  int tempRaw = analogRead(TEMP_AI_PIN);
  tempRaw = constrain(tempRaw, TEMP_METER_MIN_RAW, TEMP_METER_MAX_RAW);
  int tempOut = TEMP_METER_MIN_OUT + (TEMP_METER_MAX_OUT - TEMP_METER_MIN_OUT) * (tempRaw - TEMP_METER_MIN_RAW) / (TEMP_METER_MAX_RAW - TEMP_METER_MIN_RAW);
//  analogWrite(TEMP_METER_DO_PIN, tempOut);  
}

long humidityRandomWalkTarget;
long humidityRandomWalkStart;
long humidityRandomWalkTime;
long humidityRandomWalkTimeLeft = 0;

// returns the humidity for the meter (0 - 255)
int updateHumidity(int elapsedMillis)
{
  static const long HUMIDITY_MIN = 0;
  static const long HUMIDITY_MAX = 255; 
  static const long HUMIDITY_DRIFT_MAX = 160;
  static const long HUMIDITY_DRIFT_DELTA = 70;
  static const long HUMIDITY_DRIFT_TIME_MIN = 500; // ms
  static const long HUMIDITY_DRIFT_TIME_MAX = 2000; // ms
  static const long TIME_AT_MAX_BEFORE_ERROR = 3000; // ms

  boolean humiditySwitchState = !digitalRead(HUMID_BUTTON_DI_PIN);
  debouncedHumiditySwitch.update(humiditySwitchState, elapsedMillis);
  boolean newHumidityControlActivated = debouncedHumiditySwitch.isOn();

  if (!newHumidityControlActivated) {
    humidityError = OFF;
  }
//  if (newHumidityControlActivated) {
//    humidityError = STABILISING;
//  } else {
//    humidityError = OFF;
//  }

  switch (humidityError) {
    case OFF: {
      if (newHumidityControlActivated) {
        humidityError = STABILISING;
      }
      humidityRandomWalkTimeLeft -= elapsedMillis;
      if (humidityRandomWalkTimeLeft < 0) {
        humidityRandomWalkStart = humidity;
        int delta = random(HUMIDITY_DRIFT_DELTA / 4, +HUMIDITY_DRIFT_DELTA + 1);
        if (random(2) == 0) {
          delta = -delta;
        }
        humidityRandomWalkTarget = humidity + delta;
        if (humidityRandomWalkTarget < HUMIDITY_MIN || humidityRandomWalkTarget > HUMIDITY_DRIFT_MAX) {  // if we've hit the edge, go back to the middle
          humidityRandomWalkTarget = (HUMIDITY_MIN, HUMIDITY_DRIFT_MAX) / 2;  
        }
        humidityRandomWalkTimeLeft = random(HUMIDITY_DRIFT_TIME_MIN, HUMIDITY_DRIFT_TIME_MAX + 1);
        humidityRandomWalkTime = humidityRandomWalkTimeLeft;
      }  
      break;
    }
    case STABILISING: {
      humidityRandomWalkTimeLeft -= elapsedMillis;
      if (humidityRandomWalkTimeLeft < 0) {
        humidityRandomWalkStart = humidity;
        humidityRandomWalkTarget = humidity + random(HUMIDITY_DRIFT_DELTA / 4, +HUMIDITY_DRIFT_DELTA + 1);
        humidityRandomWalkTarget = constrain(humidityRandomWalkTarget, HUMIDITY_MIN, HUMIDITY_MAX);
        humidityRandomWalkTimeLeft = random(HUMIDITY_DRIFT_TIME_MIN, HUMIDITY_DRIFT_TIME_MAX + 1);
        humidityRandomWalkTime = humidityRandomWalkTimeLeft;
      }
      if (humidity == HUMIDITY_MAX) {
        humidityError = ERROR_DELAY;
        humidityRandomWalkStart = HUMIDITY_MAX;
        humidityRandomWalkTarget = HUMIDITY_MAX;
        humidityRandomWalkTimeLeft = TIME_AT_MAX_BEFORE_ERROR;
        humidityRandomWalkTime = humidityRandomWalkTimeLeft;
      }
      break;
    }
    case ERROR_DELAY: {
      humidityRandomWalkTimeLeft -= elapsedMillis;
      if (humidityRandomWalkTimeLeft <= 0) {
        humidityError = ACTIVE_ERROR;
      }
      break;
    }
    case ACTIVE_ERROR: {
      humidityRandomWalkTimeLeft = 0;
      humidity = HUMIDITY_MAX;
      break;
    }
  }
  humidityControlActivated = newHumidityControlActivated;
  if (humidityError != ACTIVE_ERROR) {
    humidity = humidityRandomWalkTarget + (humidityRandomWalkTimeLeft * (humidityRandomWalkStart - humidityRandomWalkTarget)) / humidityRandomWalkTime;
  }
  return humidity;
}

unsigned long timeSpentUnstable = 0;
unsigned int carryoverDelta = 0;
int pressureError = 0;

// target pressure is 0 - 1023 = 10bits; pressureFixPt6 is 8 + 6 = 14 bits
// returns a display pressure from 0 - 255
unsigned int updatePressure(unsigned int targetPressure, int elapsedMillis)
{
// if humidity is not erroring, logarithmically asymptote to the target setpoint
// if humidity is erroring, use a PI algorithm to oscillate with increasing amplitude
  
  static const unsigned long NEEDLE_SPEED_TIME_CONSTANT = 1000; // dP/dt = (Psetpoint - P) / TIME_CONSTANT
  static const unsigned long INSTABILITY_RAMP_DURATION = 5000;  // 5 seconds to reach maximum oscillation
  static const unsigned long MAX_OSCILLATION = 100;  // +/- 100/256
  static const unsigned int OSCILLATION_PERIOD = 1000;

  unsigned long targetPressureFixPt6 = targetPressure << 4;
  unsigned long deltaFixPt6;
  if (targetPressureFixPt6 > pressureFixPt6) {
    deltaFixPt6 = targetPressureFixPt6 - pressureFixPt6;
  } else {
    deltaFixPt6 = pressureFixPt6 - targetPressureFixPt6;
  }
  deltaFixPt6 *= elapsedMillis;
  deltaFixPt6 += carryoverDelta;
  carryoverDelta = 0;
  unsigned long savedDelta = deltaFixPt6;
  deltaFixPt6 /= NEEDLE_SPEED_TIME_CONSTANT;
  carryoverDelta = savedDelta - deltaFixPt6 * NEEDLE_SPEED_TIME_CONSTANT;
  if (targetPressureFixPt6 > pressureFixPt6) {
    pressureFixPt6 += deltaFixPt6;
  } else {
    pressureFixPt6 -= deltaFixPt6;
  }
  
  if (humidityError == OFF) {
    timeSpentUnstable = 0;
    if (pressureError != 0) {
      if (pressureError > 0) {
        pressureFixPt6 += pressureError << 6;
      } else {
        pressureFixPt6 -= ((unsigned int)(-pressureError)) << 6;
      }
    }
    pressureError = 0;
  } else {  // oscillations of constant period and linearly increasing amplitude
    timeSpentUnstable += elapsedMillis;
    unsigned int rampTime = (unsigned int)timeSpentUnstable;    
    if (timeSpentUnstable > INSTABILITY_RAMP_DURATION) {
      rampTime = INSTABILITY_RAMP_DURATION;
    }
    unsigned int oscillationMagnitude = rampTime / (INSTABILITY_RAMP_DURATION / MAX_OSCILLATION);

    unsigned long timeIndex = timeSpentUnstable % OSCILLATION_PERIOD;
    timeIndex *= 256;
    timeIndex /= OSCILLATION_PERIOD;
    long oscillation = oscillationMagnitude * (long)(sinTable[timeIndex]);
    pressureError = oscillation / 32767;
//    if (timeSpentUnstable < 1000) {
//      currentGain = 1;
//    } else if (timeSpentUnstable < 2000) {
//      currentGain = 2;
//    } else if (timeSpentUnstable < 3000) {
//      currentGain = 4;
//    } else if (timeSpentUnstable < 4000) {
//      currentGain = 6;
//    } else if (timeSpentUnstable < 5000) {
//      currentGain = 10;
//    } else {
//      currentGain = 20;
//    }
    
//    static const int GAIN_DIVISOR = 50;
//    pressureIntegralFixPt6 += ((targetPressure << 4) - pressureFixPt6) * elapsedMillis;
//    pressureFixPt6 += (pressureIntegralFixPt6 * currentGain)/ GAIN_DIVISOR;
  }
  pressureFixPt6 = constrain(pressureFixPt6, PRESSURE_FP6_MIN, PRESSURE_FP6_MAX);

  long totalPressure = pressureFixPt6;
  if (pressureError >= 0) {
    totalPressure += (pressureError << 6);
  } else {
    totalPressure -= ((unsigned int)(-pressureError) << 6);
  }
  totalPressure = constrain(totalPressure, PRESSURE_FP6_MIN, PRESSURE_FP6_MAX);
  return (unsigned int)(totalPressure >> 6);
}

int blinkTimeLeft = 0;
boolean ledOn = false;

void writeHumidityControlLEDS(HumidityError currentError, int elapsedMillis)
{
//      Red for control off; flashing for error
//      Green for on; flashing for starting
//  During the error delay time (humidity has recently reached 100%), alternately flash red and green.
   static const int BLINK_DURATION_MS = 500;

    blinkTimeLeft -= elapsedMillis;
    if (blinkTimeLeft <= 0) {
      blinkTimeLeft = BLINK_DURATION_MS;
      ledOn = !ledOn;
    }

    switch (currentError) {
      case OFF: {
        digitalWrite(HUMID_RED_DO_PIN, HIGH);
        digitalWrite(HUMID_GRN_DO_PIN, LOW);
        break;
      }
      case STABILISING: {
        digitalWrite(HUMID_RED_DO_PIN, LOW);
        digitalWrite(HUMID_GRN_DO_PIN, !ledOn);
        break;
      }
      case ERROR_DELAY: {
        digitalWrite(HUMID_RED_DO_PIN, ledOn);
        digitalWrite(HUMID_GRN_DO_PIN, !ledOn);
        break;
      }
      case ACTIVE_ERROR: {
        digitalWrite(HUMID_RED_DO_PIN, ledOn);
        digitalWrite(HUMID_GRN_DO_PIN, LOW);
        break;
      }
    }
}

boolean lastSiren = false;

// pressure = 0 to 255
void updateAudioPlayback(HumidityError humidityError, int pressure)
{
  const unsigned long PRESSURE_MIN = 0;  
  const unsigned long PRESSURE_MAX = 255;  
  const unsigned long SPEED_MIN = 192;
  const unsigned long SPEED_MAX = 600;
  const unsigned int SIREN_SPEED = 256;

  unsigned int newSpeed;
  
  boolean siren = (humidityError == ACTIVE_ERROR);
  if (siren) {
    newSpeed = SIREN_SPEED;
  } else {
    newSpeed = SPEED_MIN + (SPEED_MAX - SPEED_MIN) * (pressure - PRESSURE_MIN) / (PRESSURE_MAX - PRESSURE_MIN);
  }
  if (siren != lastSiren) {
    lastSiren = siren;
    speakerPCM.changeSampleData(!siren);
  }
  speakerPCM.changeSpeed(newSpeed);
}


