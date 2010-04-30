/*
 * Copyright (c) 2007, 2008, 2009, 2010 Joseph Gaeddert
 * Copyright (c) 2007, 2008, 2009, 2010 Virginia Polytechnic
 *                                      Institute & State University
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

#include <math.h>
#include <iostream>
#include <complex>
#include <getopt.h>
#include <liquid/liquid.h>

#include "usrp_io.h"
#include "iqpr.h"
 
void usage() {
    printf("ping:\n");
    printf("  u,h   :   usage/help\n");
}

int main (int argc, char **argv)
{
    //
    int d;
    while ((d = getopt(argc,argv,"uh")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':
        default:
            usage();
            return 0;
        }
    }

    iqpr q = iqpr_create();

    iqpr_destroy(q);

    return 0;
}

