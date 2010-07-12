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

#include <iostream>
#include <complex>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <time.h>
#include <liquid/liquid.h>

#include "usrp_io.h"
#include "iqpr.h"
 
void usage() {
    printf("iqpr_test:\n");
    printf("  u,h   :   usage/help\n");
    printf("  i<id> :   node id, 0:255, default: random\n");
}

int main (int argc, char **argv)
{
    srand(time(NULL));

    // options
    unsigned int node_id = rand() % 256;

    //
    int d;
    while ((d = getopt(argc,argv,"uhi:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                return 0;
        case 'i':   node_id = atoi(optarg); break;
        default:
            fprintf(stderr,"error: %s, unsupported option\n", argv[0]);
            exit(1);
        }
    }

#if 0
    iqpr q = iqpr_create(node_id);

    iqpr_destroy(q);
#endif

    return 0;
}

