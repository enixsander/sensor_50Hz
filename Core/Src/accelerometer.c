#include "filter.h"
#include "accelerometer.h"
#include "math.h"

extern SPI_HandleTypeDef hspi2;

float YX = 0, ZY = 0;
uint8_t accelerometer_ID = 0;
int16_t axis_X = 0, axis_Y = 0, axis_Z = 0;

void getData_accelerometer(void)
{
  //0x0E   XDATA_L
  //0x0F   XDATA_H
  //0x10   YDATA_L
  //0x11   YDATA_H
  //0x12   ZDATA_L
  //0x13   ZDATA_H

  uint8_t read_data[8];
  uint8_t flag_data_ready = 0;
  
  //read STATUS from accelerometer
  read_data[0] = 0x0B;   //instruction     read register
  read_data[1] = 0x0B;   //reg.addr      STATUS
  SPI2_NSS_GPIO_Port->BSRR |= (SPI2_NSS_Pin << 16);  //CS  active Low-level
  HAL_SPI_Receive(&hspi2, read_data, 3, 20);
  SPI2_NSS_GPIO_Port->BSRR |= SPI2_NSS_Pin;  //CS off
  flag_data_ready = (read_data[2] & 0x01);


  read_data[0] = 0x0B;   //instruction     read register
  read_data[1] = 0x0E;   //reg.addr      XDATA_L
  for(uint8_t i = 2; i < 8; i++)
    read_data[i] = 0;
  
  //max 625Hz
  if(flag_data_ready) {
    //read data from accelerometer
    SPI2_NSS_GPIO_Port->BSRR |= (SPI2_NSS_Pin << 16);  //CS  active Low-level
    HAL_SPI_Receive(&hspi2, read_data, 8, 20);
    //HAL_Delay(5);
    SPI2_NSS_GPIO_Port->BSRR |= SPI2_NSS_Pin;  //CS off

    axis_X = read_data[2];    //X
    axis_X |= ((int16_t)read_data[3]) << 8;

    axis_Y = read_data[4];    //Y
    axis_Y |= ((int16_t)read_data[5]) << 8;

    axis_Z = read_data[6];    //Z
    axis_Z |= ((int16_t)read_data[7]) << 8;
/*
    //из дополнительного(отрицательное чи?ло) в пр?мой код(положительное чи?ло)
    if(axis_X & 0x8000) axis_X = (~axis_X) + 1;
    if(axis_Y & 0x8000) axis_Y = (~axis_Y) + 1;
    if(axis_Z & 0x8000) axis_Z = (~axis_Z) + 1;
*/
    //filter_average(&axis_X, &axis_Y, &axis_Z);
    filtr_Chebyshev(&axis_X, &axis_Y, &axis_Z);
    filter_EMA(&axis_X, &axis_Y, &axis_Z, 1);

    /*
      arm_mult_q31(&axis_Z, &axis_Z, &cosSquareOutput, 1);
      arm_mult_f32(&sinOutput, &sinOutput, &sinSquareOutput, 1);
      arm_add_f32(&cosSquareOutput, &sinSquareOutput, &testOutput, 1);
    */

    if(axis_Z || axis_X)
      YX = arctg((float)axis_Y / hypot(axis_X,axis_Z));
    else
      YX = 90;
    if(axis_Z || axis_X)
      ZY = arctg((float)axis_Z / hypot(axis_Y,axis_X));
    else
      ZY = 90;
  }
  //float X = (float)tmp;
  //float Y = (float)tmp;
  //float Zt = (float)tmp;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //read acc-r ID
  read_data[0] = 0x0B;   //instruction     read register
  read_data[1] = 0x00;   //reg.addr        ID
  read_data[2] = 0x00;

  SPI2_NSS_GPIO_Port->BSRR |= (SPI2_NSS_Pin << 16);  //CS  active Low-level
  HAL_SPI_Receive(&hspi2, read_data, 3, 20);
  SPI2_NSS_GPIO_Port->BSRR |= SPI2_NSS_Pin;  //CS off
  HAL_Delay(20);
  accelerometer_ID = read_data[2];
}

void ADXL363_accelerometer(void)
{
  //HAL_Delay(1);  //питание акселерометра до запуска должно быть равно 0 
  ADXL_EN1_GPIO_Port->BSRR |= ADXL_EN1_Pin;  //3.3 V
  HAL_Delay(50);

  //a resolution of 1 mg/LSB on the ±2 g range
  //0x0A: write register
  //0x0B: read register 
  //0x0D: read FIFO
  uint8_t data[17];
  data[0] = 0x0A;   //instruction     write register
  data[1] = 0x20;     //reg.addr      THRESH_ACT_L    

  data[2] = 0x00;   //THRESH_ACT_L
  data[3] = 0x00;   //THRESH_ACT_H    
  data[4] = 0xFA;   //TIME_ACT              reg.addr = 0x22
  data[5] = 0x00;   //THRESH_INACT_L
  data[6] = 0x00;   //THRESH_INACT_H  150mg
  data[7] = 0x78;   //TIME_INACT_L
  data[8] = 0x00;   //TIME_INACT_H    120 samples / 400 Hz = 0.3 sec
  data[9] = 0x3F;   //ACT_INACT_CTL   loop mode, in- activity detection   
  data[10] = 0x00;  //FIFO_CONTROL    reg.addr = 0x28
  data[11] = 0x80;  //FIFO_SAMPLES    default
  data[12] = 0x00;  //INTMAP1
  data[13] = 0x00;  //INTMAP2        
  data[14] = 0x05;  //FILTER_CTL      default: ±2 g, 1/2 the ODR for a wider bandwidth (output data rate), 400 Hz
  data[15] = 0x02;  //POWER_CTL       reg.addr = 0x2D 
                    //ADC is disabled, without external clock, Low noise = 0, 
                    //Wake-Up mode = 0, Autosleep = 0, Measurement mode (2)
  data[16] = 0x00;  //SELF_TEST       reg.addr = 0x2E

  /*data[1] = 0x2C;     //reg.addr      
  data[2] = 0x05;     
  data[3] = 0x02; */

  SPI2_NSS_GPIO_Port->BSRR |= (SPI2_NSS_Pin << 16);  //CS  active Low-level
  HAL_SPI_Transmit(&hspi2, data, 17, 0x50);
  SPI2_NSS_GPIO_Port->BSRR |= SPI2_NSS_Pin;  //CS off
}
