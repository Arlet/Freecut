/*
 * stepper.h
 *
 * stepper X/Y/Z movement
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

#ifndef STEPPER_H
#define STEPPER_H

void stepper_init( void );
void stepper_tick( void );
void stepper_move( int x, int y );
char stepper_draw( int x, int y );
void stepper_speed( int delay );
void stepper_pressure( int pressure );
void stepper_get_pos( int *x, int *y );
int  stepper_queued( void );
void stepper_unload_paper( void );
void stepper_load_paper( void );

#endif
