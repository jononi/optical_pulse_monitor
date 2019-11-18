
/*
*
*/

#include "Pulse_processing.h"
#include "math.h"

Pulse_processing::Pulse_processing() {
    // nothing to initialize in constructor
}

bool Pulse_processing::setup() {
    // calculate average of helper array 
    X_bar = (int)(10*(sample_length - 1)) / 2; // multiply by 10 to keep it INT
    
    // compute helper array for affine linear regression
    for (int i=0;i<sample_length;i++) {
        X_iMinusX_bar[i] = 10*i - X_bar; // values are 10 times what's needed to keep it INT
    }
    return true;
}

void Pulse_processing::centering(uint8_t color) {
    // first get the sample mean of the raw signal
    average = 0;
    for (int index = 0;index<sample_length;index++) {
        // sum to get average later
        if (color == 1) average += red_signal[index];
        else average += ir_signal[index];
    }
    average = (int)(average/sample_length);
    Serial.printf("%s average: %i\n", color==1?"Red":"IR", average);

    // now center the sample: Y_c[i] = Y[i]-Y_bar
    for (int index = 0;index<sample_length;index++) {
        if (color == 1) red_signal[index] = red_signal[index] - average;
        else ir_signal[index] = ir_signal[index] - average;
    }
}

// store a standard normalized signal in a referenced float array + center raw signal in place (keep it INT)
void Pulse_processing::std_normalizing(uint8_t color, float *normalized_signal) {
    // 1- get the sample mean of the raw signal
    average = 0;
    for (int index = 0;index<sample_length;index++) {
        // sum to get average later
        if (color == 1) average += red_signal[index];
        else average += ir_signal[index];
    }
    average = (int)(average/sample_length);
    float variance = 0.0;

    // 2- center the sample: Y_c[i] = Y[i]-Y_bar and calculate sample variance
    for (int index = 0;index<sample_length;index++) {
        if (color == 1) {
            red_signal[index] = red_signal[index] - average;
            variance += (float)(red_signal[index]*red_signal[index]);
        } else {
            ir_signal[index] = ir_signal[index] - average;
            variance += (float)(ir_signal[index]*ir_signal[index]);
        } 
    }
    variance /= sample_length - 1; // unbiased sample variance estimator calculated using two-pass method
    float sigma = sqrt(variance); // X_std = (X - x_bar) / sigma
    // 3- standard adjust the signal (assuming normally distributed)
    for (int index = 0;index<sample_length;index++) {
        if (color == 1) normalized_signal[index] = red_signal[index] / sigma;
        else normalized_signal[index] = ir_signal[index] / sigma;
    }
}

float Pulse_processing::remove_linear_trend(uint8_t color) {
    // calculate the slope of affine linear regression (linear trend) of the centered sample
    for (int index = 0;index<sample_length;index++) {
        if (color == 1) beta += X_iMinusX_bar[index]*red_signal[index];
        else beta += X_iMinusX_bar[index]*ir_signal[index];
        // beta += X_iMinusX_bar[index]*ir_centered[index]; //sum{10*(X[i]-X_bar)*(Y[i]-Y_bar)}
        // Serial.printf("%i : %i x %i\n", index, X_iMinusX_bar[index], ir_centered[index]);
    }
    beta =   beta / S_XiMXbar_2; // Sum{ 10*(X[i]-X_bar) * (Y[i]-Y_bar) } / (10*{Sum{(X[i]-X_bar)^2})
    alpha =  average - beta*X_bar; // y_bar - beta*x_bar. For debug purpose only

    // Serial.printf("alpha: %.2f, beta: %.2f\n", alpha, beta);
    
    // remove the linear trend from centered samples
    for (int index = 0;index<sample_length;index++) {
        if (color == 1) red_signal[index] = (int)(red_signal[index]- beta*X_iMinusX_bar[index]/10);
        else ir_signal[index] = (int)(ir_signal[index]- beta*X_iMinusX_bar[index]/10);
    }
    return beta;
}

void Pulse_processing::remap_for_drawing(uint8_t color) {
    // get min and max of corrected pulse signal
    for (int index = 0;index<sample_length;index++) {
        if (color == 1) {
            if (red_signal[index] < p_min) p_min = red_signal[index];
            if (red_signal[index] > p_max) p_max = red_signal[index];
        } else {
            if (ir_signal[index] < p_min) p_min = ir_signal[index];
            if (ir_signal[index] > p_max) p_max = ir_signal[index];
        }

    }        
    // remap the corrected signal to [0:50] for display
    for (uint8_t index = 0;index<sample_length;index++) {
        if (color == 1) drawn_signal[index] = map(red_signal[index], p_min, p_max, 0, 50);
        else drawn_signal[index] = map(ir_signal[index], p_min, p_max, 0, 50);
    }
}

// WIP,  to be replaced with a butterworth band pass filter
void Pulse_processing::filter() {
    float alpha_f = 0.95;  //needs ~ 20 iterations to get to ~0.01
    ir_signal[0] = ir_signal[0] - average;
    int ir_signal_old = ir_signal[0];
    for (uint8_t index = 1;index<sample_length;index++) {
        ir_signal[index] = alpha_f*(ir_signal[index-1] + ir_signal[index] - ir_signal_old);
        ir_signal_old = ir_signal[index-1];
    }
}

float Pulse_processing::autocorrelation(uint8_t color, uint8_t lag) {
    float r = 0.0;
    int max_index = sample_length - lag;
    // corrected signal (actually residual component epsilon_i of simple linear regression) is already centered
    for (int index = 0;index<max_index;index++) {
        if (color == 1) r += red_signal[index] * red_signal[index+lag];
        else r += ir_signal[index] * ir_signal[index+lag];
    }
    //r += ir_corrected[index] * ir_corrected[index+lag];
    return (float)(r / max_index);  // r(m) = 1/(n-m) * sum_i=0..n-m-1{Y[i]*Y[i+m]} (there's a up to Var[Y] biased estimator)
}

float Pulse_processing::get_bpm(uint8_t color, int* bpm) {
    // m* = argmax_m{r_xx(m)}
    // at 25 sps, m* = 11 => HR 136 bpm, m* = 26 => HR 57 bpm
    // start from 26 and go backward down to 11
    // returns: lag at first maximum of autocorrelation
    //          r_xx(m*)/r_xx(0) (0.0..1.0) => strength of the local max
    float r_m = -1.0;
    float r_0 = autocorrelation(color,0);
    int   m_star = 0;
    // bool  m_star_found = false;// for debug
    // for 25 SPS: (int lag = 26;lag>10;lag--)
    for (int lag = 52;lag>21;lag--) {
        float r = autocorrelation(color, lag) / r_0; // relative (to r_xx(0)) autocorrelation
        Serial.printf("%s r_xx[%d]=%.3f\n",color==1?"Red":"IR", lag, r);
        if (r > r_m + 0.002) {
            // relative autocorrelation at this lag is better, update value
            r_m = r;
            m_star = lag;
        } /*else {
            // relative autocorrelation was at a local max
            if (!m_star_found)  {
                // do this only once
                m_star = lag + 1; // previous value is the lag we look for.
                m_star_found = true;
            }
            //break; // stop checking 
        };*/
    }

    Serial.printf("%s m*:=%i\n",color==1?"Red":"IR", m_star);
    
    // quality 
    if (m_star>10) {
        // there was a local max detected
        *bpm = (int)(60*SAMPLING_RATE_HZ / m_star);
        return r_m;
    } else {
        *bpm = 0;
        return -1.0;
    }  
}

float Pulse_processing::sample_variance(int *signal, int length) {
    float variance = 0.0;
    for (int index =0;index<length;index++) {
        float signal_centered = (float)(signal[index]-average);
        variance += signal_centered*signal_centered;
    }
    variance /= (float)(length-1);
    return variance;
}

