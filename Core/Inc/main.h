/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
enum {OFF = 0x00, ON = 0xFF};
enum {OUT1, OUT2};

#define BIK_ID 0x00
#define BVV_ID 0x01
#define BPK_ID 0x02
#define LEP_ID 0x03
#define DDS_ID 0x04
//TX
#define ACCELEROM_ID  (0x08F02E00 | LEP_ID)
#define STATE_ID      (0x08F1F100 | LEP_ID)
//RX
#define BPK_Input_ID  (0x08357C00 | BPK_ID)
#define BUTTONS_ID    (0x18FED900 | BIK_ID)
#define ANTENNA_ID    (0x18FEDA00 | BIK_ID)

//BT6030-2ERA
//I = 50mA k = 2450
//I = 0,5A k = 2360
//I = 2A k = 2240
//I = 7A k = 2240
//V_ref_adc = 3,3V
//R= 1 kOm
//I_IS = (ADC_data / 4096 * 3,3V) / R - Sense current
//I_input = I_IS * k  - Nominal load current (4А - номинальный ток через нагрузку)

//ADC_data = I_input * R * 4096 / 3,3 / k
//3A - ~0x680 (1664)
#define DMA_SIZE     50
#define NUMB_ADC_CH  2
#define MAX_CURRENT_P1 0x680
#define MAX_CURRENT_P2 0x680
#define OVERCURRENT_TIMEOUT 1000

#define NUMB_OF_CHECKS 10
//#define alpha_x10      5  //5 = 0,5 * 10 = 50%

#define M_PI ((float)3.141592653589793)
#define M_PI12 (M_PI/12.F)
#define M_PI6 (M_PI/6.F)
#define M_PI2 (M_PI/2.F)
#define SQRT3 ((float)1.732050807569)   //square root of 3
  
#define MASK_12 0x03
#define MASK_34 0x0C
#define MASK_56 0x30
#define MASK_78 0xC0
  
#define bit1 0x01
#define bit3 0x04
#define bit5 0x10
#define bit7 0x40 
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAN_RS_Pin GPIO_PIN_13
#define CAN_RS_GPIO_Port GPIOC
#define OUT1_Pin GPIO_PIN_0
#define OUT1_GPIO_Port GPIOA
#define DEN_Pin GPIO_PIN_1
#define DEN_GPIO_Port GPIOA
#define DSEL_Pin GPIO_PIN_3
#define DSEL_GPIO_Port GPIOA
#define OUT2_Pin GPIO_PIN_4
#define OUT2_GPIO_Port GPIOA
#define SEL2_Pin GPIO_PIN_8
#define SEL2_GPIO_Port GPIOE
#define SEL1_Pin GPIO_PIN_9
#define SEL1_GPIO_Port GPIOE
#define ADXL_IN1_Pin GPIO_PIN_8
#define ADXL_IN1_GPIO_Port GPIOA
#define ADXL_IN2_Pin GPIO_PIN_9
#define ADXL_IN2_GPIO_Port GPIOA
#define ADXL_EN1_Pin GPIO_PIN_10
#define ADXL_EN1_GPIO_Port GPIOA
#define SPI2_NSS_Pin GPIO_PIN_11
#define SPI2_NSS_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
