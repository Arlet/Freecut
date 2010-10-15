/*
 * keypad.c
 *
 * Driver for Cricut Personal keypad, with built-in LED indicators.
 * 
 * keypad is designed as a matrix with 15 rows, and 5 colums. The 15 rows are selected using
 * a 16 bit shift register, with CLK/Data inputs. 
 *
 * NOTE: Due to this design, if you press two (or more) keys in the same row simultaneously, the
 * the two corresponding outputs of the shift register are shorted through the keys, and the CPU 
 * gets undefined results.  
 * 
 * Pinout of the connector is as follows:
 *
 * Pin 	|  Name	| AVR  | Description
 *------+-------+------+------------
 *  1   |  GND 	|      |
 *  2   |  Vcc  |      |
 *  3   |  Stop | PD0  | Direct connection to stop button
 *  4   |  O4   | PG4  | Row output 4 (bottom)
 *  5   |  O3   | PG3  | Row output 3
 *  6   |  O2 	| PG2  | Row output 2
 *  7   |  O0   | PG0  | Row output 0 (top row)
 *  8   |  LED 	| PD5  | LED Enable (0=LEDs on)
 *  9   |  O1   | PG1  | Row output 1
 * 10   | CLK   | PD7  | Shift register clock
 * 11   | Data  | PD6  | Shift register data in
 * 12   | Test  |      | Shift register data out (for testing, apparently not used)
 *
 * Key layout is straightforward: column 0 is left, column 13 is right, row 0 is top, row 4 is bottom.
 * The grey arrow keys and CUT key are mapped in column 14:
 * 
 * Row  | Key
 *------+------
 *  0   | Right
 *  1   | Left 
 *  2   | CUT 
 *  3   | Up 
 *  4   | Down 
 * 
 * The various LEDs are also connected to the shift register outputs, and can be turned on with the 
 * LED Enable pin (0=On, 1=Off).  
 *
 * LED Layout is as follows:
 *
 * COL0  COL1
 * COL2  COL3
 * COL4  COL5				COL10 (LED between CUT and Up arrow)
 * COL6  COL7
 * COL8  COL9
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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <stdio.h>

// pin assignment
#define STOP	(1 << 0)	// PD0
#define LEDS	(1 << 5)	// PD5
#define DATA	(1 << 6) 	// PD6
#define CLK	(1 << 7)	// PD7
#define ROWS    (0x1f)		// mask for PG4-0

#define MAX_COLS	16	
#define MAX_ROWS 	5	

#define leds_on()	do { PORTD &= ~LEDS; } while(0)
#define leds_off()	do { PORTD |=  LEDS; } while(0)
#define clk_h()		do { PORTD |=  CLK;  } while(0)
#define clk_l()		do { PORTD &= ~CLK;  } while(0)
#define data_h()	do { PORTD |=  DATA; } while(0)
#define data_l()	do { PORTD &= ~DATA; } while(0)
#define get_rows()	(~PING & ROWS)

uint8_t keypad_state[MAX_COLS];	// current state
uint8_t keypad_prev[MAX_COLS];  // previous state
uint16_t leds;

/*
 * keypad_write_cols: write all 16 column bits in 'val' to shift register.
 */
static void keypad_write_cols( short val )
{
    int i;

    for( i = 0; i < MAX_COLS; i++ )
    {
        if( val < 0 ) 
	    data_h( );
	else
	    data_l( );
	clk_h( );
	val <<= 1;
	clk_l( );
    }
}

void keypad_set_leds( uint16_t mask )
{
    leds = mask;
    leds_off( );
    keypad_write_cols( ~leds );
    leds_on( );
}

/*
 * return state of 'STOP' key.
 */
char keypad_stop_pressed( void )
{
    return !(PIND & STOP);
}

/* 
 * keypad_scan: perform a single scan of keyboard. Returns keycode of key that was 
 * pressed (or -1 if nothing).  
 */
int keypad_scan( void )
{
    int row, col;
    int pressed = -1;

    leds_off( );		// turn off LEDs during scan
    keypad_write_cols( ~1 );	// single zero at column 0
    data_h( );			// shift in ones
    for( col = 0; col < MAX_COLS; col++ )
    {
	keypad_state[col] = get_rows( );
	clk_h( );
	clk_l( );
    }
    keypad_write_cols( ~leds );
    leds_on( );	

    // keyboard has been scanned, now look for pressed keys
    for( col = 0; col < MAX_COLS; col++ )
    {
	uint8_t diff = keypad_state[col] ^ keypad_prev[col];

	if( diff )
	{
	    for( row = 0; row < MAX_ROWS; row++ )
	    {
		uint8_t mask = 1 << row;

	        if( diff & mask & keypad_state[col] )
		    pressed = row * 16 + col;
	    }
	}
	keypad_prev[col] = keypad_state[col];
    }
    return pressed;
}

void keypad_init( void )
{
    clk_l( );
    leds_off( );
    DDRD |= (LEDS | DATA | CLK); 
    PORTD |= STOP;
    DDRG &= ~ROWS;  // rows are inputs
    PORTG |= ROWS;  // enable internal pull-ups
}
