/*
 * cli.c
 *
 * command line interface
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

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <setjmp.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <math.h>
#include "printf.h"
#include "timer.h"
#include "cli.h"
#include "usb.h"
#include "stepper.h"
#include "flash.h"
#include "version.h"

#define BUFLEN 100 
#define MAX_ARGS 10

static char buf[BUFLEN + 1];
static char *argv[MAX_ARGS+1];
static uint8_t buf_len = 0;
static uint8_t argc;
static uint8_t curtok;
static char echo = 1;
static jmp_buf cmd_error;
static char *errorbuf = (char *) argv;

enum
{
    C_EOL = 0, 
    C_BAD,
    C_NUM,
    C_MOVE,
    C_DRAW,
    C_VERSION,
    C_SPEED,
    C_PRESS,
    C_CURVE,
    C_FLASH,
};

static struct token
{
    char *lex;
    uint8_t class;
    long val;
} token;

static const struct keyword
{
    char str[9];
    uint8_t class;
} keyword_list[] PROGMEM = 
{
    // commands
    { "version",  C_VERSION },
    { "move", 	  C_MOVE },
    { "draw", 	  C_DRAW },
    { "speed", 	  C_SPEED },
    { "curve",    C_CURVE },
    { "press",    C_PRESS },
    { "flash", 	  C_FLASH },
};

#define MAX_KEYWORDS (sizeof(keyword_list) / sizeof(*keyword_list))

static const struct keyword *find_keyword( char *str )
{
    unsigned i;
    const struct keyword *keyword;

    for( i = 0, keyword = keyword_list; i < MAX_KEYWORDS; i++, keyword++ )
    {
	if( !strcmp_P(str, keyword->str) )
	    return( keyword );
    }
    return( 0 );
}

/* 
 * get next token 
 */
static uint8_t get_token( void )
{
    const struct keyword *keyword;
    char *lex = argv[curtok];
    uint8_t class;

    token.lex = lex;
    if( curtok >= argc )
    {
	token.class = C_EOL;
	return C_EOL;
    }
    curtok++;
    keyword = find_keyword( lex );
    if( keyword )
	memcpy_P( &class, &keyword->class, sizeof(class) );
    else if( lex[0] == '-' || isdigit(lex[0]) )
    {
	token.val = strtol( lex, 0, 10 ); 
        class = C_NUM;
    }
    else
	class = C_BAD;
    token.class = class;
    return( class );
}

static void expect_token( uint8_t class, const char *what )
{
    if( get_token() == class )
	return;
    printf( "expected %s, got '%s'", what, token.lex );
    longjmp( cmd_error, 1 );
}

static long parse_num( void )
{
    expect_token( C_NUM, "number" );
    return( token.val );
}

/*
 * split_line: split line into args at spaces.
 */
static char split_line( char *line, char **argv )
{
    uint8_t argc = 0;
    char c;

    for( argc = 0; argc < MAX_ARGS; argc++ )
    {
        while( *line == ' ' )
            *line++ = 0;
        if( !*line )
            break;
        argv[argc] = line;
        while( (c = *line) != 0 && c != ' ' )
	    line++;
   }
   argv[argc] = "";
   return( argc );
}

void version( void )
{
    printf( "Freecut version " VERSION "\n");
}

static void parse_speed( void )
{
    stepper_speed( parse_num() );
}

static void parse_move( void )
{
    uint16_t x = parse_num( );
    uint16_t y = parse_num( );

    stepper_move( x, y );
}

static void parse_draw( void )
{
    uint16_t x = parse_num( );
    uint16_t y = parse_num( );

    stepper_draw( x, y );
}

static void bezier_prep( int *k, int *p )
{
    k[0] =   -p[0] + 3*p[1] - 3*p[2] + p[3];
    k[1] =  3*p[0] - 6*p[1] + 3*p[2];
    k[2] = -3*p[0] + 3*p[1];
    k[3] =    p[0];
}

static int bezier_eval( int *p, float t )
{
    float s;
    
    s  = p[0]; s *= t;
    s += p[1]; s *= t;
    s += p[2]; s *= t;
    s += p[3];
    
    return (int) (s + 0.5);
}

static void parse_curve( void )
{
    int i, x, y;
    int ox, oy;
    int Px[4], Py[4]; // control points, x and y
    int Kx[4], Ky[4]; // cubic polynomial coefficients 

    for( i = 0; i < 4; i++ )
    {
        Px[i] = parse_num( );
	Py[i] = parse_num( );
    }
    ox = Px[0];
    oy = Py[0];
    stepper_move( ox, oy );
    bezier_prep( Kx, Px );
    bezier_prep( Ky, Py );
    for( i = 0; i <= 128; i++ )
    {
        float t = i / 128.0;

	x = bezier_eval( Kx, t );
	y = bezier_eval( Ky, t );
	if( x != ox || y != oy )
	    stepper_draw( x, y );
	ox = x;
	oy = y;
    }
}

static void parse_flash( void )
{
    flash_test( );
}

static void parse_pressure( void )
{
    stepper_pressure( parse_num() );
}

static void parse( char *buf )
{
    curtok = 0;

    switch( get_token() )
    {
	case C_VERSION: version( ); break;
	case C_MOVE:	parse_move( ); break;
	case C_DRAW:	parse_draw( ); break;
	case C_SPEED:   parse_speed( ); break;
	case C_PRESS:   parse_pressure( ); break;
	case C_CURVE: 	parse_curve( ); break;
	case C_FLASH:   parse_flash( ); break;

	case C_EOL:
	    break;

	default:
	    printf( "unknown command '%s'", token.lex );
	    return;
    }
    if( get_token() != C_EOL )
	printf( "unrecognized parameter '%s'", token.lex );
}

void prompt( void )
{
    printf( ">" );
}

void cli_poll( void )
{
    int c = usb_peek( );

    if( c < 0 )
	return;
    if( c == '\r' || c == '\n' )
    {
	printf( "\n" );
	buf[buf_len] = 0;
	buf_len = 0;
	argc = split_line( buf, argv );
	if( !setjmp(cmd_error) )
	{
	    parse( buf );
	}
	prompt( );
    }
    else if( c == '\b' || c == 0x7f )
    {
	if( buf_len > 0 )
	{
	    buf_len--;
	    if( echo )
		printf( "\b \b" );
	}
    }
    else if( isprint(c) && buf_len < BUFLEN )
    {
	if( echo )
	    putchar( c );
	buf[buf_len++] = c;
    }
}
