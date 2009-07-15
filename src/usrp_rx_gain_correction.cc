//
//
//

#include <stdio.h>
#include <math.h>
#include "usrp_rx_gain_correction.h"

#define POLY1A  (-1.6832)
#define POLY1B  ( 5.9594)

#define POLY2A  (0.94000)
#define POLY2B  (6.18000)

// correct for usrp gain mismatch
float usrp_rx_gain_correction(unsigned int _decim_rate)
{
    float d = log2f((float)(_decim_rate)-2.0f);
    float f = 0.0f; // fracpart(d)
    f = d;
    while (f >= 0.0f)
        f -= 1.0f;
    f += 1.0f;
    float sdB = (POLY1A*d + POLY1B) + f*(POLY2A*d + POLY2B);
    //printf("decim : %4u, d : %12.8f, f : %12.8f, s_hat : %12.8f\n", _decim_rate,d,f,sdB);

    return powf(10.0f,-sdB/10.0f);
}

