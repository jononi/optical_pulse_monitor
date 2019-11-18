
/*
*
*/
#pragma once

#ifndef __PULSE_PROCESSING_H
#define __PULSE_PROCESSING_H


#ifdef PARTICLE
#include "Particle.h"
#endif

#define SAMPLE_LENGTH       250   // sampling duration in seconds = SAMPLE_LENGTH / SAMPLING_RATE_HZ
#define SAMPLING_RATE_HZ    50


class Pulse_processing {     
    private:
        const static int sample_length = SAMPLE_LENGTH;
        int S_XiMXbar_2 = 13020625; //N = 100: 833250 this is multiplied by 10 to compensate for X[i]-X_bar that is multiplied by 10 to keep it integer  
        // int X_iMinusX_bar[sample_length]; // holds X[i]-X_bar terms, computed only once since it depends on sample size     
        int     X_bar = 0;
        int     average = 0;
        float   beta = 0.0;
        float   alpha = 0.0;
        int     p_min = 0;
        int     p_max = 0;

    public:
        Pulse_processing(void);

        int X_iMinusX_bar[sample_length];
        int ir_signal[sample_length];
        int red_signal[sample_length];
        int drawn_signal[sample_length];
        // int ir_raw[sample_length];
        // int ir_centered[sample_length];
        // int ir_corrected[sample_length];
        // int ir_drawing[sample_length];

        bool    setup(void);
        inline int     get_sample_length(void) {return sample_length;}
        void    centering(uint8_t color);
        void    std_normalizing(uint8_t color, float *normalized_signal);
        float   remove_linear_trend(uint8_t color);
        void    remap_for_drawing(uint8_t color);
        void    filter(void);
        float   autocorrelation(uint8_t color, uint8_t shift);
        float   get_bpm(uint8_t color, int *m_star);
        float   sample_variance(int *signal, int length);
};
#endif
