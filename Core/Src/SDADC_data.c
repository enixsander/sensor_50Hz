#include "adc_data.h"
#include "filter.h"

extern TIM_HandleTypeDef htim3;
extern SDADC_HandleTypeDef hsdadc1;

/*  
  SDADC работает от триггера TIM3. Макс частота 6MHz / 360 = 16,6kHz (почему то работает макс. на 8kHz).
  В fast mode прерывание возникает с частотой ~50kHz, но не работает без continuous mode 

  * The conversion of each channel normally requires 360 SDADC clock cycles (60 μs at 6 MHz). 
  * In fast continuous mode (FAST=1), the first conversion takes still 360 SDADC clocks, 
  * but then each subsequent conversion finishes in 120 SDADC clocks.
*/

//FREQ  5120 = 256 * 20 -- 256 значений приходят раз в 1сек / 20 = 50ms (max 32ms)
#define FREQ  5120 // = 256 * 20 Hz

extern __IO uint8_t flag_sdadc_end;

__IO int16_t input_50Hz[2][LENGTH_SAMPLES];
uint8_t array_numb = 0; //2 массива, в один записываем, по второму проводим расчеты, потом наоборот

void SDADC_Start(void) {
  //SDADC - 6 MHz
  if (HAL_SDADC_CalibrationStart(&hsdadc1, SDADC_CALIBRATION_SEQ_1) != HAL_OK) {
    ;// An error occurs during the starting phase of the calibration
  }
  if (HAL_SDADC_PollForCalibEvent(&hsdadc1, HAL_MAX_DELAY) != HAL_OK) {  //Pool for the end of calibration
    ;//An error occurs while waiting for the end of the calibration
  }

  uint16_t freq = FREQ; //Hz (8kHz - max, 1Hz - min)
  TIM3->ARR = (uint32_t)(SystemCoreClock / 375 / freq) - 1;  //48MHz / 375 = 128kHz
  //TIM3->PSC = 375 - 1;// Set the Prescaler value

  HAL_SDADC_InjectedStart_IT(&hsdadc1);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);  //default -- 8kHz ( 48MHz / (374 + 1) / (15 + 1) )
}

void HAL_SDADC_InjectedConvCpltCallback(SDADC_HandleTypeDef *hsdadc) {
  //прерывание выполняется за 102 такта процессора
  static uint16_t i = 0;
  static uint8_t numb = 1;
  static int32_t valueADC = 0;
  uint32_t InjChannel = SDADC_CHANNEL_4;
  valueADC = HAL_SDADC_InjectedGetValue(hsdadc, &InjChannel);


  input_50Hz[numb][i++] = valueADC;
  input_50Hz[numb][i++] = 0;

  if(i > LENGTH_SAMPLES) {
	  i = 0;
    if(!array_numb) {
      array_numb = 1; //заполнили данными input_50Hz[0](закидываем в fft), далее input_50Hz[1] (во избежание перезаписи данных)
      numb = 0;
    }
    else {
      array_numb = 0;
      numb = 1;
    }

    flag_sdadc_end = 1;
  }
  //прерывание вызывается раз в 5986 тактов, при таймере - 8кГц
}
