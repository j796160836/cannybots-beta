// David A. Mellis, 13 May 2010

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Timer1.h"

void (*Timer1Class::isrCompareA)(void) = 0;
void (*Timer1Class::isrCompareB)(void) = 0;

void Timer1Class::setPrescaleFactor(uint8_t factor)
{
	const uint8_t CS_MASK = ~0x07;
	TCCR1B = (TCCR1B & CS_MASK) | factor;
}

void Timer1Class::setMode(uint8_t mode)
{
	TCCR1A = (TCCR1A & ~0x03) | (mode & 0x03);
  TCCR1B = (TCCR1B & ~0x18) | ((mode >> 2) << 3);
}

uint16_t Timer1Class::read()
{
	return TCNT1;
}

void Timer1Class::write(uint16_t val)
{
	TCNT1 = val;
}

void Timer1Class::writeCompareA(uint16_t val)
{
	OCR1A = val;
}

void Timer1Class::writeCompareB(uint16_t val)
{
	OCR1B = val;
}

void Timer1Class::attachCompareAInterrupt(void (*f)(void))
{
	isrCompareA = f;
  TIMSK1 |= (1 << OCIE1A);
}

void Timer1Class::attachCompareBInterrupt(void (*f)(void))
{
	isrCompareB = f;
  TIMSK1 |= (1 << OCIE1B);
}

void Timer1Class::detachCompareAInterrupt()
{
  TIMSK1 &= ~(1 << OCIE1A);	
	isrCompareA = 0;
}

void Timer1Class::detachCompareBInterrupt()
{
  TIMSK1 &= ~(1 << OCIE1B);
	isrCompareB = 0;
}

ISR(TIMER1_COMPA_vect)
{
	if (Timer1.isrCompareA) Timer1.isrCompareA();
}

ISR(TIMER1_COMPB_vect)
{
	if (Timer1.isrCompareB) Timer1.isrCompareB();
}

Timer1Class Timer1;