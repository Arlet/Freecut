/*
 * timer.c
 *
 * Timer 0 is used as overall (slow) event timer, as well sleep delay timer.
 * Timer 1 is used as solenoid PWM, through OC1B output
 * Timer 2 is used for stepper timing 
 * Timer 3 is used to generate tones on the speaker through OC3A output, period is adjusted for tone.
 *
 *
 * Copyright 2010 <freecutfirmware@gmail.com> 
 *
 * This file is part of Freecut.
 *
 * Freecut is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2.
 *
 * Freecut is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Freecut. If not, see http://www.gnu.org/licenses/.
 *
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <string.h>
#include <stdio.h>
#include "timer.h"
#include "stepper.h"

static uint8_t count_Hz = 250;
static uint8_t count_25Hz = 10;
volatile uint8_t flag_Hz;
volatile uint8_t flag_25Hz;

/*
 * called @250 Hz, divide further in software for slow events 
 */
SIGNAL( SIG_OUTPUT_COMPARE0 ) 
{
    if( --count_25Hz == 0 )
    {
        count_25Hz = 10;
        flag_25Hz = 1;
    }
    if( --count_Hz == 0 )
    {
	count_Hz = 250;
	flag_Hz = 1;
    }
}        

/*
 * Timer 2 compare match, update stepper motors.
 */
SIGNAL( SIG_OUTPUT_COMPARE2 ) 
{
    stepper_tick( );
}        

/*
 * Turn on beeper. Hz specifies frequency of the tone.
 */
void beeper_on( int Hz )
{
    DDRE |= (1 << 3);
    OCR3A = (8000000L + Hz/2) / Hz - 1;
}

void beeper_off( void )
{
    DDRE &= ~(1 << 3);
}

/*
 * usleep: sleep (approximate/minimum) number of microseconds. We use timer0 which runs at 62.50 kHz, 
 * or at 16 usec/tick. Maximum delay is about 2 milliseconds . For longer delays, use msleep().
 *
 */
void usleep( int usecs )
{
    signed char end = TCNT0 + usecs / 16;

    while( (signed char) (TCNT0 - end) < 0 )
	continue;
}

void msleep( unsigned msecs )
{
    while( msecs-- != 0 )
	usleep( 1000 );
}

void timer_set_stepper_speed( int delay )
{
    uint8_t prescaler = 4; // default 1:64 prescaler

    if( delay < 30 )
        delay = 30; 
    else if( delay > 1000 )
        delay = 1000;
    TCCR2 &= ~7;  // stop timer, and clear prescaler bit
    if( delay > 256 )
    {
        delay /= 4;
	prescaler = 5;
    }
    OCR2 = delay - 1;
    TCCR2 |= prescaler;
}

void timer_set_pen_pressure( int pressure )
{
    if( pressure > 1023 )
        pressure = 1023;
    OCR1B  = pressure;	
}

/*
 * Init timers 
 */
void timer_init( void )
{
    // set timer 0 for 250 Hz period
    TCCR0  = (1 << WGM01) | 6;     				// prescaler 1:256 -> 62.50 kHz timer
    OCR0   = 249; 		   				// 125 kHz / 250 = 250 Hz 
    TIMSK  = (1 << OCIE0); 					// enable OVF interrupt 

    // set timer 1, WGM mode 7, fast PWM 10 bit
    DDRB   |= (1 << 6);						// PB6 is output
    TCCR1A = (1 << WGM11) | (1 << WGM10) | (1 << COM1B1);	// mode 7, clear OC1B on compare match
    TCCR1B = (1 << WGM12) | 1;					// mode 7, prescaler = 1
    OCR1B  = 1023;						// lowest pressure

    // set timer 2, variable period for stepper
    TCCR2  = (1 << WGM21) | 4;					// prescaler 1/256 -> 250 kHz
    OCR2   = 65;						// default frequency = 250/125 = 2 kHz step
    TIMSK |= (1 << OCIE2);					//

    // Timer 3, WGM mode 15 (1111), Fast PWM using OCR3A 
    TCCR3A = (1 << COM3A0) | (1 << WGM31) | (1 << WGM30);	// mode 15, toggle OC3A on match
    TCCR3B = (1 << WGM33) | (1 << WGM32) | 1;   		// 
}
