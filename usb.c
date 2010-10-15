/*
 * usb.c
 *
 * The USB port is a UART, connected to a FTDI chip. 
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
#include <inttypes.h>
#include <stdio.h>
#include "usb.h"

// these must be powers of two
#define USB_INBUF_LEN  128 
#define USB_OUTBUF_LEN 32

static uint8_t usb_tx_buf[USB_OUTBUF_LEN];
static uint8_t usb_rx_buf[USB_INBUF_LEN];

volatile static uint8_t usb_tx_head = 0;
volatile static uint8_t usb_tx_tail = 0;
volatile static uint8_t usb_rx_head = 0;
volatile static uint8_t usb_rx_tail = 0;

/* 
 * transmit interrupt handler
 */
SIGNAL( SIG_UART1_DATA ) 
{
    if( usb_tx_tail == usb_tx_head )
        UCSR1B &= ~(1 << UDRIE1);
    else
	UDR1 = usb_tx_buf[usb_tx_tail++ % USB_OUTBUF_LEN];
}

/*
 * receive interrupt handler
 */
SIGNAL( SIG_UART1_RECV )
{
    char c = UDR1;

    if( (uint8_t) (usb_rx_head - usb_rx_tail) >= USB_INBUF_LEN )
	return;
    usb_rx_buf[usb_rx_head++ % USB_INBUF_LEN] = c;
}

/*
 * set the baud rate. Assumes 2X flag is set
 */
void usb_set_baud( long baud )
{
    int div = (FCLK + baud*4) / (baud * 8) - 1;

    UBRR1H = div >> 8;
    UBRR1L = div;
}

/*
 * write a single character to output queue, and enable Tx interrupts.
 */
int usb_putchar( char c, FILE *stream ) 
{
    if( c == '\n' )
        usb_putchar( '\r', stream );
    while( (uint8_t) (usb_tx_head - usb_tx_tail) == USB_OUTBUF_LEN )
	continue;
    usb_tx_buf[usb_tx_head++ % USB_OUTBUF_LEN] = c;
    UCSR1B |= (1 << UDRIE1 );
    return 0;
}

/* 
 * look to see if Rx character is available, return -1 if not.
 */
int usb_peek( void )
{
    if( usb_rx_head == usb_rx_tail )
        return -1;
    return usb_rx_buf[usb_rx_tail++ % USB_INBUF_LEN];
}

/*
 * wait for character to be available.
 */
int usb_getchar( FILE *stream )
{
    int c;

    while( (c = usb_peek()) < 0 )
        ;
    return c;
}

/*
 * initialize USB UART. Assume fixed baudrate of 115200, and default 8N1 settings.
 */
void usb_init( void ) 
{
    UCSR1A = (1 << U2X1);
    UCSR1B = (1 << RXCIE1) | (1 << RXEN1) | (1 << UDRIE1) | (1 << TXEN1);
    usb_set_baud( 115200L );
}
