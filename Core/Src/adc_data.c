#include "adc_data.h"
#include "filter.h"

extern ADC_HandleTypeDef hadc1;

uint16_t wind_speed_x10 = 0;

uint8_t flag_KZ[2] = {0};
uint8_t output_state[2] = {0};
uint8_t flag_limit_switch = 0;

__IO uint8_t time_power_key = 0;
__IO uint8_t flag_DMA_interrupt = 0;
__IO uint16_t current_P1 = 0;
__IO uint16_t current_P2 = 0;
__IO uint16_t average_adc_data[NUMB_ADC_CH] = {0};
__IO uint16_t adc_data[NUMB_ADC_CH * DMA_SIZE] = {0};
__IO uint16_t time_anemometer = 0;
__IO uint16_t time_overcurrent_P1 = OVERCURRENT_TIMEOUT;
__IO uint16_t time_overcurrent_P2 = OVERCURRENT_TIMEOUT;

void ADC_Start(void) {
  //P1 - Габаритные огни (ГС), P2 (БПК SB13) - подсветка крюка
  DEN_GPIO_Port->BSRR |= DEN_Pin;	//6030-2ERA 

	HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_Delay(5);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_data, NUMB_ADC_CH * DMA_SIZE);
}

void ADC_average(void) {
	if(flag_DMA_interrupt) {

    for(uint8_t n = 0; n < NUMB_ADC_CH; n++) {
      uint32_t result = 0;
      uint16_t min = 0xFFFF;
      uint16_t max = 0x0000;
      for(uint8_t i = 0; i < DMA_SIZE; i++) 
      {
        if(adc_data[NUMB_ADC_CH * i + n] < min) min = adc_data[NUMB_ADC_CH * i + n];
        if(adc_data[NUMB_ADC_CH * i + n] > max) max = adc_data[NUMB_ADC_CH * i + n];
        result += adc_data[NUMB_ADC_CH * i + n];
      }
      average_adc_data[n] = ((result - min - max) / (DMA_SIZE - 2));
    }
		flag_DMA_interrupt = 0;
	  //усреднение происходит раз в ~22мс
	}

	//переключение DSEL в силовом ключе для измерения тока P1, P2
  if(!time_power_key)
  {
    time_power_key = 50;
    if(DSEL_GPIO_Port->ODR & DSEL_Pin) { //DSEL
      current_P2 = average_adc_data[0];
      DSEL_GPIO_Port->BSRR |= DSEL_Pin << 16;   //DSEL  diagnostic  P1
    }
    else {
      current_P1 = average_adc_data[0];
      DSEL_GPIO_Port->BSRR |= DSEL_Pin;        	//DSEL  diagnostic  P2
    }
  }

	//перегрузка по току
  if(!time_overcurrent_P1) {
    out_state(OUT1, OFF);
    flag_KZ[OUT1] = 1;
  }
  if(!time_overcurrent_P2) {
    out_state(OUT2, OFF);
    flag_KZ[OUT2] = 1;
  }

  //концевик
  //V_in = 13V, adc = 0x1CF
  //V_in = 14V, adc = 0x1F4
  //V_in = 24V, adc = 0x367
  //V_in = 30V, adc = 0x447
  if(average_adc_data[1] >= 0x100) {
    flag_limit_switch = ON;
    LED1_GPIO_Port->BSRR |= LED1_Pin;
  }
  else {
    flag_limit_switch = OFF;
    LED1_GPIO_Port->BSRR |= (LED1_Pin << 16);
  }
}

void out_state(uint8_t out, uint8_t state) {

	uint8_t offset = 16 & (~state);	//ON -- << 0, OFF -- << 16

	if(out == OUT1 && (!flag_KZ[out] || state == OFF)) {
		OUT1_GPIO_Port->BSRR |= (OUT1_Pin << offset);  //OUT1
    output_state[out] = state;
	}
	if(out == OUT2 && (!flag_KZ[out] || state == OFF)) {
		OUT2_GPIO_Port->BSRR |= (OUT2_Pin << offset);  //OUT2
		output_state[out] = state;
	}
}

//BT6030-2ERA
//I = 50mA k = 2450
//I = 0,5A k = 2360 
//I = 2A k = 2240
//I = 7A k = 2240
//V_ref_adc = 3,3V
//R = 1 kOm
//I_input = I_is * k  - Nominal load current (4А - номинальный ток через нагрузку)
//ADC_data = I_input * R * 4096 / 3,3 / k

uint16_t current_amper_x10(uint8_t out) {

  float I_is_mA = 0, I_in_x10 = 0, k = 0;
  
  if(output_state[out] == ON) {
    if(out == OUT1) {
      I_is_mA = (float)current_P1 * 3.3 / 4096; //I_is = (ADC_data / 4096 * 3,3V) / R - Sense current
    }
    if(out == OUT2) {
      I_is_mA = (float)current_P2 * 3.3 / 4096;
    }
  }
  else
    I_is_mA = 0;

  if(I_is_mA > 0.215)       //I_is = 500mA / 2360 = 0,212mA
    k = 22.4;               //делим на 100, чтобы получить 0,1 A/bit
  else 
    if(I_is_mA > 0.02)     //I_is = 50mA / 2450 = 0,02mA
      k = 23.6;
    else
      k = 24.5;
  I_in_x10 = I_is_mA * k;

  return (uint16_t)(I_in_x10 + 0.5);
}

//v = 2*pi*R / T;
//wind_speed = 2 * M_PI * radius / (time_anemometer / 1000 / impulse_count) m/s
void anemometer(void) {

  float radius = 1; //meters
  uint32_t impulse_count = 0;
  
  impulse_count = TIM2->CNT;

    /*
  if(time_anemometer >= 100 && impulse_count > 5) { //100ms
    ...
  }
  else  ...
*/

  if(time_anemometer >= 1000) { //1s
    wind_speed_x10 = (2 * M_PI * radius * impulse_count * 100 / time_anemometer) + 0.5;
    time_anemometer = 0;
    TIM2->CNT = 0;
  }
}
