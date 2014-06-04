// David A. Mellis, 13 May 2010

#include <inttypes.h>

#ifndef TIMER2_H
#define TIMER2_H

class Timer2Class {
  public:
  	static void (*isrOverflow)(void);
    static void (*isrCompareA)(void);
  	static void (*isrCompareB)(void);
  
  	static const uint8_t PRESCALE1 = 1;
    static const uint8_t PRESCALE8 = 2;
  	static const uint8_t PRESCALE32 = 3;
    static const uint8_t PRESCALE64 = 4;
  	static const uint8_t PRESCALE128 = 5;
    static const uint8_t PRESCALE256 = 6;
  	static const uint8_t PRESCALE1024 = 7;
    
    const static uint8_t NORMAL = 0;
    const static uint8_t CTC = 2;
    
    void setPrescaleFactor(uint8_t factor);
  	void setMode(uint8_t mode);
    uint8_t read();
    void write(uint8_t val);
    void attachOverflowInterrupt(void (*f)(void));
    void detachOverflowInterrupt();
    void writeCompareA(uint8_t val);
    void writeCompareB(uint8_t val);
    void attachCompareAInterrupt(void (*f)(void));
    void attachCompareBInterrupt(void (*f)(void));
    void detachCompareAInterrupt();
    void detachCompareBInterrupt();
};

extern Timer2Class Timer2;

#endif