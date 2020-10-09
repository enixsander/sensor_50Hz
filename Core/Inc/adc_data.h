#ifndef __ADC_DATA_H
#define __ADC_DATA_H

#include "main.h"

void ADC_Start(void);
void anemometer(void);
void ADC_average(void);
void SDADC_Start(void);
void antenna_tuning(void);
void out_state(uint8_t, uint8_t);
uint16_t current_amper_x10(uint8_t);

#endif // __ADC_DATA_H 
