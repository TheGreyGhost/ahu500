#include <Arduino.h>
#include "SpeakerPCM.h"
#include "Debounce.h"

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

Debounce debouncedHumiditySwitch;
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
  
  int targetPressure = analogRead(PRESS_SETPT_AI_PIN);

  updateTemperatureMeter();
  updateHumidity(timeElapsedMillis);
  updatePressure(targetPressure, timeElapsedMillis);
  
  updateAudioPlayback(humidityError, pressureFixPt6 >> 6);

  analogWrite(PRESS_METER_DO_PIN, pressureFixPt6 >> 6);
  analogWrite(HUMID_METER_DO_PIN, humidity);
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
  analogWrite(TEMP_METER_DO_PIN, tempOut);  
}

int humidityRandomWalkTarget;
int humidityRandomWalkStart;
int humidityRandomWalkTime;
int humidityRandomWalkTimeLeft = 0;

void updateHumidity(int elapsedMillis)
{
  static const int HUMIDITY_MIN = 0;
  static const int HUMIDITY_MAX = 255; 
  static const int HUMIDITY_DRIFT_MAX = 235;
  static const int HUMIDITY_DRIFT_DELTA = 70;
  static const int HUMIDITY_DRIFT_TIME_MIN = 200; // ms
  static const int HUMIDITY_DRIFT_TIME_MAX = 1000; // ms
  static const int TIME_AT_MAX_BEFORE_ERROR = 3000; // ms

  boolean humiditySwitchState = digitalRead(HUMID_BUTTON_DI_PIN);
  debouncedHumiditySwitch.update(humiditySwitchState, elapsedMillis);
  boolean newHumidityControlActivated = debouncedHumiditySwitch.isPressed();

  switch (humidityError) {
    case OFF: {
      if (newHumidityControlActivated) {
        humidityError = STABILISING;
      }
      humidityRandomWalkTimeLeft -= elapsedMillis;
      if (humidityRandomWalkTimeLeft < 0) {
        humidityRandomWalkStart = humidity;
        humidityRandomWalkTarget = humidity + random(-HUMIDITY_DRIFT_DELTA, +HUMIDITY_DRIFT_DELTA + 1);
        humidityRandomWalkTarget = constrain(humidityRandomWalkTarget, HUMIDITY_MIN, HUMIDITY_DRIFT_MAX);
        humidityRandomWalkTimeLeft = random(HUMIDITY_DRIFT_TIME_MIN, HUMIDITY_DRIFT_TIME_MAX + 1);
      }  
      break;
    }
    case STABILISING: {
      humidityRandomWalkTimeLeft -= elapsedMillis;
      if (humidityRandomWalkTimeLeft < 0) {
        humidityRandomWalkStart = humidity;
        humidityRandomWalkTarget = humidity + random(0, +HUMIDITY_DRIFT_DELTA + 1);
        humidityRandomWalkTarget = constrain(humidityRandomWalkTarget, HUMIDITY_MIN, HUMIDITY_MAX);
        humidityRandomWalkTimeLeft = random(HUMIDITY_DRIFT_TIME_MIN, HUMIDITY_DRIFT_TIME_MAX + 1);
      }
      if (humidity == HUMIDITY_MAX) {
        humidityError = ERROR_DELAY;
        humidityRandomWalkTimeLeft = TIME_AT_MAX_BEFORE_ERROR;
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
    humidity = humidityRandomWalkTarget + humidityRandomWalkTimeLeft * (humidityRandomWalkStart - humidityRandomWalkTarget) / humidityRandomWalkTime;
  }
}

unsigned long pressureIntegralFixPt6 = 0;
byte currentGain = 0;
unsigned int timeSpentUnstable = 0;

// target pressure is 0 - 1023 = 10bits; pressureFixPt6 is 8 + 6 = 14 bits
void updatePressure(int targetPressure, int elapsedMillis)
{
// if humidity is not erroring, logarithmically asymptote to the target setpoint
// if humidity is erroring, use a PI algorithm to oscillate with increasing amplitude
  
  static const int NEEDLE_SPEED_TIME_CONSTANT = 1000; // dP/dt = (Psetpoint - P) / TIME_CONSTANT
  static const int INSTABILITY_RAMP_DURATION = 5000;  // 5 seconds to reach maximum oscillation
  if (humidityError == OFF) {
    int deltaFixPt6 = ((targetPressure << 4) - pressureFixPt6) * elapsedMillis / NEEDLE_SPEED_TIME_CONSTANT;
    pressureFixPt6 += deltaFixPt6;
    pressureIntegralFixPt6 = 0;
    currentGain = 0;
    timeSpentUnstable = 0;
  } else {  // use a pure I algorithm with increasing gain
    timeSpentUnstable = constrain(timeSpentUnstable + elapsedMillis, 0, INSTABILITY_RAMP_DURATION);
    if (timeSpentUnstable < 1000) {
      currentGain = 1;
    } else if (timeSpentUnstable < 2000) {
      currentGain = 2;
    } else if (timeSpentUnstable < 3000) {
      currentGain = 4;
    } else if (timeSpentUnstable < 4000) {
      currentGain = 6;
    } else if (timeSpentUnstable < 5000) {
      currentGain = 10;
    } else {
      currentGain = 20;
    }
    
    static const int GAIN_DIVISOR = 50;
    pressureIntegralFixPt6 += ((targetPressure << 4) - pressureFixPt6) * elapsedMillis;
    pressureFixPt6 += (pressureIntegralFixPt6 * currentGain)/ GAIN_DIVISOR;
    pressureFixPt6 = constrain(pressureFixPt6, PRESSURE_FP6_MIN, PRESSURE_FP6_MAX);
  }
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
        digitalWrite(HUMID_GRN_DO_PIN, ledOn);
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

void updateAudioPlayback(HumidityError humidityError, int pressure)
{
    // our meter range is 12 C to 28 C
  const unsigned int PRESSURE_MIN = 0;  
  const unsigned int PRESSURE_MAX = 255;  
  const unsigned int SPEED_MIN = 128;
  const unsigned int SPEED_MAX = 512;

  unsigned int newSpeed = SPEED_MIN + (SPEED_MAX - SPEED_MIN) * (pressure - PRESSURE_MIN) / (PRESSURE_MAX - PRESSURE_MIN);
  
  speakerPCM.changeSpeed(newSpeed);
  boolean siren = (humidityError == ACTIVE_ERROR);
  if (siren != lastSiren) {
    lastSiren = siren;
    speakerPCM.changeSampleData(!siren);
  }
}


