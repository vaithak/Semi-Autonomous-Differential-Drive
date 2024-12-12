/*
 * MEAM510 hacks for Vive Interface V2
 * Dec 2021
 * Use at your own risk
 * 
 * Mark Yim
 * University of Pennsylvania
 * copyright (c) 2021 All Rights Reserved
 */

#include "vive510.h"
//#define DEBUG
//#define DEBUG2

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// need global interrupt Arduino attachInterrupt won't take member function as interrupt
// need to pass THIS as arg since all vive object interrupts call same global interrupt 
void IRAM_ATTR m_pulseISR(void *v) {
  portENTER_CRITICAL_ISR(&mux);
  static_cast<Vive510 *>(v)->pulseISR(micros());
  portEXIT_CRITICAL_ISR(&mux);  
}

Vive510::Vive510(int pin) {
  m_pin = pin;
}

void IRAM_ATTR Vive510::pulseISR(uint32_t us) {
    if (digitalRead(m_pin)==HIGH) {
      m_usRising = us;
    }
    else { 
      m_usFalling = us;
    }
    if (m_vivestatus == VIVE_RECEIVING)   processPulse();
}


uint16_t Vive510::xCoord(){
  return m_xCoord;
}

uint16_t Vive510::yCoord(){
  return m_yCoord;
}

int Vive510::isKPulse(uint32_t pulsewidth){
  if (pulsewidth < 75 || 
      (pulsewidth > 85 && pulsewidth < 95) || 
      (pulsewidth > 106 && pulsewidth < 117) ||
      (pulsewidth > 127 && pulsewidth < 137) 
  ) return false;
  else return true;
}

int Vive510::isJPulse(uint32_t pulsewidth){
  if (pulsewidth < 75 || 
      (pulsewidth > 85 && pulsewidth < 95) || 
      (pulsewidth > 106 && pulsewidth < 117) ||
      (pulsewidth > 127 && pulsewidth < 137) 
  ) return true;
  else return false;
}

// move checkflag to be backwards checking...think about whether tu link w/spuriuos
void Vive510::processPulse() {
 // static int checkflag=0;

  if (m_lastFalling != m_usFalling) {
    int pulsewidth = m_usFalling-m_usRising;
    
    if (pulsewidth > m_sweepWidth) {
      if (pulsewidth > 140) { //pulse too long. bad pulse
#ifdef DEBUG2
        ets_printf("P%d Spur %d width:%d \n",m_pin,m_spurious,pulsewidth);
#endif
        m_pulseType = 0;
      }
      else if (isKPulse(pulsewidth)) {
        m_pulseType = KTYPE;
#ifdef DEBUG
  ets_printf("\nKPin%d width=%d ", m_pin, pulsewidth);
#endif
      } 
      else if (isJPulse(pulsewidth)) {
        m_pulseType = JTYPE;
#ifdef DEBUG
  ets_printf("\tJPin%d width=%d", m_pin, pulsewidth);
#endif
      }
    }
    else { // x sweep or y sweep
#ifdef DEBUG
      ets_printf("Pin%d:%d  r=%d", m_pin,pulsewidth,m_usRising-m_lastFalling);
#endif
      if (m_pulseType == JTYPE)  m_yCoord = m_usRising-m_lastFalling;
      if (m_pulseType == KTYPE)  m_xCoord = m_usRising-m_lastFalling;
      
       m_spurious = 0;
    }
    
    if (m_spurious++ > 60) m_vivestatus=VIVE_SYNC_ONLY; 
    
    m_lastFalling = m_usFalling;
  }
}

void Vive510::start() {
  // use ESP32 version of attachInterrupt to allow THIS argument
  attachInterruptArg(digitalPinToInterrupt(m_pin), m_pulseISR,  this,  CHANGE);
}

void Vive510::begin() {
  pinMode(m_pin, INPUT);  
  start();
}
  
void Vive510::begin(int pin) {
  m_pin = pin;
  pinMode(m_pin, INPUT);  
  start();
}
  
void Vive510::stop() {
  detachInterrupt(digitalPinToInterrupt(m_pin));
}

int Vive510::status() {
  return m_vivestatus;
}

uint32_t  Vive510::sync(int reps){
  int i=0;
  uint32_t m_lastFalling;
  uint32_t startms = millis();
  m_lastFalling = m_usFalling;
  
  while (millis()-startms < (reps+1)*1000/120) { // count  pulses
    if (m_lastFalling != m_usFalling) {
      m_lastFalling = m_usFalling;
      i++;
    }
    yield();
  }

  if (i == 0) {
    m_vivestatus = VIVE_NO_SIGNAL; // just for debugging info
#ifdef DEBUG2
    ets_printf("no signal ");
#endif
  }
  else if (i < 2*reps) {
    m_vivestatus = VIVE_SYNC_ONLY; // just for debugging info
#ifdef DEBUG2
 ets_printf("missing some pulses %d/%d ",i,2*reps);
#endif
  } 
  else {
    m_vivestatus = VIVE_RECEIVING;
#ifdef DEBUG2
  ets_printf(" Sweep and Sync received ");
#endif
  }
 
  return m_vivestatus;
}
