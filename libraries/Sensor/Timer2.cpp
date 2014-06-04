// David A. Mellis, 13 May 2010

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Timer2.h"

void (*Timer2Class::isrOverflow)(void) = 0;
void (*Timer2Class::isrCompareA)(void) = 0;
void (*Timer2Class::isrCompareB)(void) = 0;

void Timer2Class::setPrescaleFactor(uint8_t factor)
{
	const uint8_t CS_MASK = ~0x07;
	TCCR2B = (TCCR2B & CS_MASK) | factor;
}

void Timer2Class::setMode(uint8_t mode)
{
	TCCR2A = (TCCR2A & ~0x03) | (mode & 0x03);
  TCCR2B = (TCCR2B & ~0x08) | ((mode >> 2) << 3);
}

uint8_t Timer2Class::read()
{
	return TCNT2;
}

void Timer2Class::write(uint8_t val)
{
	TCNT2 = val;
}

void Timer2Class::attachOverflowInterrupt(void (*f)(void))
{
	isrOverflow = f;
  TIMSK2 |= (1 << TOIE2);
}

void Timer2Class::detachOverflowInterrupt()
{
  TIMSK2 &= ~(1 << TOIE2);
	isrOverflow = 0;
}

void Timer2Class::writeCompareA(uint8_t val)
{
	OCR2A = val;
}

void Timer2Class::writeCompareB(uint8_t val)
{
	OCR2B = val;
}

void Timer2Class::attachCompareAInterrupt(void (*f)(void))
{
	isrCompareA = f;
  TIMSK2 |= (1 << OCIE2A);
}

void Timer2Class::attachCompareBInterrupt(void (*f)(void))
{
	isrCompareB = f;
  TIMSK2 |= (1 << OCIE2B);
}

void Timer2Class::detachCompareAInterrupt()
{
  TIMSK2 &= ~(1 << OCIE2A);	
	isrCompareA = 0;
}

void Timer2Class::detachCompareBInterrupt()
{
  TIMSK2 &= ~(1 << OCIE2B);
	isrCompareB = 0;
}

ISR(TIMER2_OVF_vect)
{
	if (Timer2.isrOverflow) Timer2.isrOverflow();
}

ISR(Timer2_COMPA_vect)
{
	if (Timer2.isrCompareA) Timer2.isrCompareA();
}

ISR(Timer2_COMPB_vect)
{
	if (Timer2.isrCompareB) Timer2.isrCompareB();
}

Timer2Class Timer2;