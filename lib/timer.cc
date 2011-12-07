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

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include "timer.h"

// tiemr data structure
struct timer_s {
    struct timeval tic;
    struct timeval toc;

    int timer_started;
};

// create timer object
timer timer_create()
{
    timer q = (timer) malloc(sizeof(struct timer_s));

    q->timer_started = 0;

    return q;
}

// destroy timer object
void timer_destroy(timer _q)
{
    // free main object memory
    free(_q);
}

// reset timer
void timer_tic(timer _q)
{
    int rc = gettimeofday(&_q->tic, NULL);
    if (rc != 0) {
        fprintf(stderr,"warning: timer_tic(), gettimeofday() returned invalid flag\n");
    }
    _q->timer_started = 1;
}

// get elapsed time since 'tic' in seconds
float timer_toc(timer _q)
{
    if (!_q->timer_started) {
        fprintf(stderr,"warning: timer_toc(), timer was never started\n");
        return 0;
    }

    int rc = gettimeofday(&_q->toc, NULL);
    if (rc != 0) {
        fprintf(stderr,"warning: timer_toc(), gettimeofday() returned invalid flag\n");
    }

    // compute execution time (in seconds)
    float s  = (float)(_q->toc.tv_sec  - _q->tic.tv_sec);
    float us = (float)(_q->toc.tv_usec - _q->tic.tv_usec);

    return s + us*1e-6f;
}

