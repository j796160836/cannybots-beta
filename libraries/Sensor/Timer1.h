// David A. Mellis, 13 May 2010

#include <inttypes.h>

#ifndef TIMER1_H
#define TIMER1_H

class Timer1Class {
  public:
    static void (*isrCompareA)(void);
  	static void (*isrCompareB)(void);
  
  	static const uint8_t PRESCALE1 = 1;
    static const uint8_t PRESCALE8 = 2;
  	static const uint8_t PRESCALE64 = 3;
    static const uint8_t PRESCALE256 = 4;
  	static const uint8_t PRESCALE1024 = 5;
    
    const static uint8_t NORMAL = 0;
    const static uint8_t CTC = 4;
    
    void setPrescaleFactor(uint8_t factor);
  	void setMode(uint8_t mode);
    uint16_t read();
    void write(uint16_t val);
    void writeCompareA(uint16_t val);
    void writeCompareB(uint16_t val);
    void attachCompareAInterrupt(void (*f)(void));
    void attachCompareBInterrupt(void (*f)(void));
    void detachCompareAInterrupt();
    void detachCompareBInterrupt();
};

extern Timer1Class Timer1;

#endif