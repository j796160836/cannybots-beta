// David A. Mellis, 13 May 2010

#include <inttypes.h>

#ifndef ADC_H
#define ADC_H

#undef ADC

class ADCClass {
  public:
    static void (*isr)(void);

  	static const uint8_t PRESCALE2 = 1;
    static const uint8_t PRESCALE4 = 2;
  	static const uint8_t PRESCALE8 = 3;
    static const uint8_t PRESCALE16 = 4;
  	static const uint8_t PRESCALE32 = 5;
    static const uint8_t PRESCALE64 = 6;
  	static const uint8_t PRESCALE128 = 7;
    
    static const uint8_t MUX1V1 = 14;
    
    static const uint8_t FREE_RUNNING = 0;
    static const uint8_t ANALOG_COMPARATOR = 1;
    static const uint8_t TIMER1_COMPAREB = 5;
    
    void begin();
    bool isRunning();
  	void setPrescaleFactor(uint8_t factor);
    void setMUX(uint8_t input);
    void setReference(uint8_t reference);
    void setAutoTriggerSource(uint8_t source);
    void autoTrigger();
    void noAutoTrigger();
    void attachInterrupt(void (*f)(void));
    void detachInterrupt();
    int read();
};

extern ADCClass ADC;

#endif