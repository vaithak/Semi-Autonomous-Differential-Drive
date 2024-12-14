/*
 * header for MEAM510 hacks for vive interface
 * May 2021
 * Use at your own risk
 * 
 */

#ifndef VIVE510
#define VIVE510

#include <Arduino.h>

// vive status errors
#define VIVE_NO_SIGNAL 0
#define VIVE_SYNC_ONLY 1 
#define VIVE_RECEIVING 2

#define KTYPE 2
#define JTYPE 1

class Vive510
{
private:

  volatile uint32_t m_usRising ; // updated by interrupts 
  volatile uint32_t m_usFalling ;
  uint16_t m_xCoord;
  uint16_t m_yCoord;
  int m_vivestatus = 0;
  int m_pin;               // signal input pin
  int m_sweepWidth=50;
  int m_pulseType; // 1 is J, 2 is K
  
  uint32_t m_lastFalling;
  int m_spurious;

  int isJPulse(uint32_t pulsewidth);
  int isKPulse(uint32_t pulsewidth);
  void processPulse();
  
public:
  Vive510(int pin);
  uint16_t xCoord();
  uint16_t yCoord();
  uint32_t sync(int);
  int status();
  void stop();
  void start();
  void begin();
  void begin(int);
  
   void  pulseISR(uint32_t); // need public for global interrupt Arduino hack
};


#endif
