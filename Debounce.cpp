/*Speaker PCM
  Released into the public domain.*/

//#include <Arduino.h>
#include "SpeakerPCM.h"
//#include "Machine8bit11kHzC.h"
//#include "Machine8bit11kHzB.h"
#include "Machine8bit11kHzA.h"
#include "siren8bit11kHzA.h"
//#include "100hzTone8bit11Khz.h"

static unsigned int bufferSize; 
static volatile unsigned int interruptCount;

// we use fixed point fraction to vary the sample playback rate
// the fractional part is 8 bits
static volatile unsigned int countSpeedFixPt8;
static volatile unsigned int sampleDataIndex;
static volatile unsigned int sampleDataIndexFixPt8;
static int speakerPin;
static boolean bufferA;

SpeakerPCM::SpeakerPCM(int i_speakerPin){
	speakerPin = i_speakerPin;
}

void SpeakerPCM::play(int speed){  // speed = 256 == normal speed.  eg 512 = double speed
  countSpeedFixPt8 = speed;
  changeSampleData(true);
  pinMode(speakerPin, OUTPUT);
  stopPlayback();
  startPlayback();
}

void SpeakerPCM::changeSpeed(int newspeed) {
  countSpeedFixPt8 = newspeed;
}

void SpeakerPCM::changeSampleData(boolean newBufferA) {
  bufferA = newBufferA;
  bufferSize = bufferA ? sizeof sampleDataA : sizeof sampleDataB;
  sampleDataIndex = 0;
  sampleDataIndexFixPt8 = 0; 
}

void SpeakerPCM::startPlayback() {
  unsigned int resolution = 256;  // the interrupt will be called every 256 clock cycles
  sampleDataIndex = 0;
  sampleDataIndexFixPt8 = 0; 
  
  noInterrupts();
  ICR1 = resolution;
  OCR1A = 1;
  
  TCCR1A = _BV(WGM11) | _BV(COM1A1); //WGM11,12,13 all set to 1 = fast PWM/w ICR TOP
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  TIMSK1 = ( _BV(ICIE1) | _BV(TOIE1) );
  interrupts();
}


void SpeakerPCM::stopPlayback() {
  TIMSK1 &= ~( _BV(ICIE1) | _BV(TOIE1) );
  OCR1A = 10;
}

void SpeakerPCM::disable() {
  TIMSK1 &= ~( _BV(ICIE1) | _BV(TOIE1) );
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
}

ISR(TIMER1_OVF_vect) {
// only update the sample every 4 interrupts;
// this allows us to use a much higher DAC frequency (> 20 kHz and therefore inaudible)
  if (++interruptCount < 4) {
    return;
  }
  interruptCount = 0;

  if (bufferA) {
    OCR1A = pgm_read_byte_near(sampleDataA + sampleDataIndex);
  } else {
    OCR1A = pgm_read_byte_near(sampleDataB + sampleDataIndex);
  }
  sampleDataIndexFixPt8 += countSpeedFixPt8;
  sampleDataIndex += (sampleDataIndexFixPt8 >> 8);
  sampleDataIndexFixPt8 &= 0xff;
  if (sampleDataIndex >= bufferSize){
    sampleDataIndex -= bufferSize;
  }
}

//ISR(TIMER1_CAPT_vect){
//  // The first step is to disable this interrupt before manually enabling global interrupts.
//  // This allows this interrupt vector (COMPB) to continue loading data while allowing the overflow interrupt
//  // to interrupt it. ( Nested Interrupts )
//
// TIMSK1 &= ~_BV(ICIE1);
//
// //Now enable global interupts before this interrupt is finished, so the music can interrupt the buffering
//  sei();
//
//  if (paused) { //if paused, disable overflow vector and leave this one enabled
//    TIMSK1 = _BV(ICIE1); OCR1A = 10; TIMSK1 &= ~_BV(TOIE1); 
//  }   else if (playing) { //re-enable this interrupt vector and the overflow vector
//    TIMSK1 = ( _BV(ICIE1) | _BV(TOIE1) );
//  }
//
//}


