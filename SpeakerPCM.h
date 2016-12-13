/* Pulse Code Modulation playing to speaker
  Released into the public domain.
  
Usage:
  (1) Include the byte array of sample data in SpeakerPCM.cpp following this example:
  const static PROGMEM byte sampleData[] = {0x7F, 0x7F, 0x7F, 0x7F};
  The data should be 8 bit wav format, 11.025 kHz which will play back at approximately 15 kHz (TO VERIFY!)

  (2) Instantiate a SpeakerPCM with the desired output pin (must be either 9 or 10 on a UNO)
  (3)a) Start the playback using play()
     b) Change the playback speed using newSpeed()
  (4) Stop the playback using stopPlayback()
    
  */
#ifndef SpeakerPCM_h   // if x.h hasn't been included yet...
#define SpeakerPCM_h   //   #define this so the compiler knows it has been included
#include <Arduino.h>
#include <avr/pgmspace.h>

class SpeakerPCM
{
  public:
  	SpeakerPCM(int i_speakerPin);
    void play(int speed);         // Start playing the sample at a relative speed of speed/256; i.e. a speed of 256 = 1 sample per cycle.
    void changeSpeed(int newSpeed);  // Change sample playback speed
    void changeSampleData(boolean newBufferA);  // Change between sample buffer A and buffer B
  	void stopPlayback();          // stops the playback but doesn't disable the time
  	void disable();               // disables the timer
   	void startPlayback();         // restart the playback
};

#endif
