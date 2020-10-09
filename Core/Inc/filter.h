#ifndef __FILTER_H
#define __FILTER_H

#include "main.h"

#define LENGTH_SAMPLES 512
float arctg(float x);
void filter_FFT(void);
int16_t filter_EMA_ADC(float adc_data, uint8_t alpha_x10);
void filter_average(int16_t *axis_X, int16_t *axis_Y, int16_t *axis_Z);
void filtr_Chebyshev(int16_t *axis_X, int16_t *axis_Y, int16_t *axis_Z);
void filter_EMA(int16_t *axis_X, int16_t *axis_Y, int16_t *axis_Z, uint8_t alpha_x10);

#endif // __FILTER_H 
