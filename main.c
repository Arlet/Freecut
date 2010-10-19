/*
 * main.c
 * 
 * Freecut firmware, main program
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
#include <avr/wdt.h>
#include <inttypes.h>
#include <stdio.h>
#include "usb.h"
#include "keypad.h"
#include "lcd.h"
#include "timer.h"
#include "stepper.h"
#include "cli.h"
#include "flash.h"
#include "version.h"
#include "dial.h"

static FILE usb = FDEV_SETUP_STREAM( usb_putchar, usb_getchar, _FDEV_SETUP_RW );

#define LOAD_PAPER	0x4c
#define UNLOAD_PAPER	0x4d

void poll_keypad( void )
{
    int key = keypad_scan( );

    switch( key )
    {
	case LOAD_PAPER: 
	    stepper_load_paper(); 
	    break;

	case UNLOAD_PAPER:
	    stepper_unload_paper(); 
	    break;
	
	default:
	    if( key >= 0 )
		printf( "# unknown key %02x\n", key );
    }
}

int main( void )
{
    wdt_enable( WDTO_30MS );
    usb_init( );
    timer_init( );
    stepper_init( );
    sei( );
    lcd_init( );
    keypad_init( );
    flash_init( );
    dial_init( );

    wdt_reset( );

    // short beep to show we're awake
    beeper_on( 1760 );
    msleep( 10 );
    beeper_off( );

    // connect stdout to USB port
    stdout = &usb; 
    fprintf( &lcd, "Freecut v" VERSION );
    while( 1 )
    {
        cli_poll( );
	wdt_reset( );
	if( flag_25Hz )
	{
	    flag_25Hz = 0;
	    dial_poll( );
	    poll_keypad( );
	}
	if( flag_Hz )
	{
	    flag_Hz = 0;
	}
    }
}
