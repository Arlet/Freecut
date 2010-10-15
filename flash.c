/*
 * flash.c
 *
 * Driver for serial dataflash AT45DB041B on main board, attached as follows:
 *
 * Pin 	|  Func	| AVR
 *------+-------+---------
 *  1   |  SI 	| PB7
 *  2   |  SCK  | PB1 
 *  4   |  !CS  | PB4 
 *  8   |  SO   | PB0 
 *
 * only contains a test/dump function for now.
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
#include <stdio.h>
#include <inttypes.h>
#include "flash.h"
#include "usb.h"

#define MISO	(1 << 0)
#define SCK	(1 << 1)
#define CS	(1 << 4)
#define MOSI 	(1 << 7)

#define cs_low( )	do { PORTB &= ~CS; } while(0)
#define cs_high( )	do { PORTB |=  CS; } while(0)
#define mosi_low( )	do { PORTB &= ~MOSI; } while(0)
#define mosi_high( )	do { PORTB |=  MOSI; } while(0)
#define sck_low( )	do { PORTB &= ~SCK; } while(0)
#define sck_high( )	do { PORTB |=  SCK; } while(0)
#define get_miso( )	(PINB & 1)

#define AT_STATUS 	0xd7		// status byte 
#define AT_CAR	 	0xe8		// continuous array read

/*
 * write a single byte to flash chip
 */
static void flash_write_byte( uint8_t data )
{
    char i;

    for( i = 0; i < 8; i++ )
    {
        if( data & 0x80 )
	    mosi_high( );
	else
	    mosi_low( );
	sck_high( );
	data <<= 1;
	sck_low( );
    }
}

/*
 * read a single byte to flash chip
 */
static uint8_t flash_read_byte( void )
{
    uint8_t data = 0;
    char i;

    for( i = 0; i < 8; i++ )
    {
	sck_high( );
	data = (data << 1) + get_miso( );
	sck_low( );
    }
    return data;
}

/*
 * provide 'count' dummy clocks
 */
static void flash_clocks( char count )
{
    while( --count >= 0 )
    {
        sck_high( );
	sck_low( );
    }
}

/*
 * read the Flash status byte
 */
static uint8_t flash_read_status( void )
{
    uint8_t status;

    cs_low( );
    flash_write_byte( AT_STATUS );
    status = flash_read_byte( );
    cs_high( );
    return status;
}

/*
 * write a command, consisting of 8 cmd bits, and 24 address bits.
 */
static void flash_write_cmd( uint8_t cmd, uint32_t addr )
{
    cs_low( );
    flash_write_byte( cmd );
    flash_write_byte( addr >> 24 );
    flash_write_byte( addr >> 16 );
    flash_write_byte( addr >>  8 );
}

/* 
 * simple test: dump all contents of built-in cartridge
 */
void flash_test( void )
{
    uint8_t i = 0;
    long size = 2048L * 264L;

    flash_write_cmd( AT_CAR, 0 );
    flash_clocks( 32 );
    while( --size >= 0 )
    {
        printf( "%02x%c", flash_read_byte(), ++i % 16 ? ' ' : '\n' );
    }
    cs_high( );
}

void flash_init( void )
{
    DDRB |= MOSI | SCK | CS;
}
