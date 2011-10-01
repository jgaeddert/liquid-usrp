//
//
//

#include <stdio.h>
#include <stdlib.h>
#include "usrp_rx_gain_correction.h"

#define OUTPUT_FILENAME "usrp_rx_gain_correction_test.m"

int main() {
    unsigned int decim;
    float g;
    unsigned int i=0;

    FILE * fid = fopen(OUTPUT_FILENAME,"w");
    fprintf(fid,"%% auto-generated file : %s\n",OUTPUT_FILENAME);
    //fprintf(fid,"clear all;\nclose all;\n\n");
    for (decim=10; decim<128; decim+=2) {
        g = usrp_rx_gain_correction(decim);
        fprintf(fid,"d(%3u) = %3u; g(%3u) = %12.4e;\n", i+1, decim, i+1, g);
        i++;
    }
    fprintf(fid,"figure;\n");
    fprintf(fid,"semilogx(d,-10*log10(g),'-x');\n");
    fclose(fid);
    printf("results written to %s\n", OUTPUT_FILENAME);
}
