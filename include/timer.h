/*
 * Copyright (c) 2011 Joseph Gaeddert
 * Copyright (c) 2011 Virginia Polytechnic Institute & State University
 *
 * This file is part of liquid.
 *
 * liquid is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * liquid is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with liquid.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// timer
//

#ifndef __TIMER_H__
#define __TIMER_H__

// 
// timer object interface declarations
//

typedef struct timer_s * timer;

// create timer object
timer timer_create();

// destroy timer object
void timer_destroy(timer _q);

// reset timer
void timer_tic(timer _q);

// get elapsed time since 'tic' in seconds
float timer_toc(timer _q);

#endif // __TIMER_H__

