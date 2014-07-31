// David A. Mellis, 13 May 2010

#include <avr/io.h>
#include <avr/interrupt.h>

#include "ADC.h"

void (*ADCClass::isr)(void) = 0;

void ADCClass::begin()
{
	ADCSRA |= (1 << ADSC);
}

bool ADCClass::isRunning()
{
	return !!(ADCSRA & (1 << ADSC));
}

void ADCClass::setPrescaleFactor(uint8_t factor)
{
	const uint8_t ADPS_MASK = ~0x07;
	ADCSRA = (ADCSRA & ADPS_MASK) | factor;
}

void ADCClass::setMUX(uint8_t input)
{
	const uint8_t MUX_MASK = ~0x0F;
  ADMUX = (ADMUX & MUX_MASK) | input;
}

void ADCClass::setReference(uint8_t reference)
{
  const uint8_t REFS_MASK = ~0xC0;
  ADMUX = (ADMUX & REFS_MASK) | (reference << 6);
}

void ADCClass::setAutoTriggerSource(uint8_t source)
{
	const uint8_t ADTS_MASK = ~0x07;
	ADCSRB = (ADCSRB & ADTS_MASK) | source;
}

void ADCClass::autoTrigger()
{
  ADCSRA |= (1 << ADATE);
}

void ADCClass::noAutoTrigger()
{
  ADCSRA &= ~(1 << ADATE);
}

void ADCClass::attachInterrupt(void (*f)(void))
{
	isr = f;
  ADCSRA |= (1 << ADIE);
}

void ADCClass::detachInterrupt()
{
  ADCSRA &= ~(1 << ADIE);
  isr = 0;
}

int ADCClass::read()
{
	uint8_t low, high;

	// start the conversion
	ADCSRA |= (1 << ADSC);

	// ADSC is cleared when the conversion finishes
	while (bit_is_set(ADCSRA, ADSC));

	// we have to read ADCL first; doing so locks both ADCL
	// and ADCH until ADCH is read.  reading ADCL second would
	// cause the results of each conversion to be discarded,
	// as ADCL and ADCH would be locked when it completed.
	low = ADCL;
	high = ADCH;

	// combine the two bytes
	return (high << 8) | low;	
}

ISR(ADC_vect)
{
	if (ADC.isr) ADC.isr();
}

ADCClass ADC;