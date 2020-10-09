/**************************************************************
WinFilter version 0.8

Filter type: Low Pass
Filter model: Bessel
Filter order: 6
Sampling Frequency: 200 Hz
Cut Frequency: 5 Hz
Pass band Ripple: 0.100000 dB
Coefficents Quantization: 16-bit
***************************************************************/
#include "filter.h"

#define Ntap 50
#define DCgain 524288

void filtr_Chebyshev(int16_t *axis_X, int16_t *axis_Y, int16_t *axis_Z) {
  int16_t FIRCoef[Ntap] = { 
    706, 1049, 1463, 1954, 2528, 3189, 3939, 4782, 5715, 6736, 7841, 9020,10262,
    11552, 12873, 14201, 15513, 16780, 17972, 19058, 20005, 20782, 21361, 21719, 
    21840, 21719, 21361, 20782, 20005, 19058, 17972, 16780, 15513, 14201, 12873,
    11552, 10262, 9020, 7841, 6736, 5715, 4782, 3939, 3189, 2528, 1954, 1463, 1049, 706, 427
  };

  static uint8_t count = 0;

  //static int16_t x[Ntap]; //input samples
  static int16_t accel_dataX[Ntap] = {0};
  static int16_t accel_dataY[Ntap] = {0};
  static int16_t accel_dataZ[Ntap] = {0};
  int32_t x = 0, y = 0, z = 0;   //output sample

  //shift the old samples
  for(uint8_t n = Ntap - 1; n > 0; n--) {
    accel_dataX[n] = accel_dataX[n-1];
    accel_dataY[n] = accel_dataY[n-1];
    accel_dataZ[n] = accel_dataZ[n-1];
  }

  //Calculate the new output
  accel_dataX[0] = (int16_t)*axis_X;
  accel_dataY[0] = (int16_t)*axis_Y;
  accel_dataZ[0] = (int16_t)*axis_Z;
  for(uint8_t n = 0; n < Ntap; n++) {
    x += FIRCoef[n] * accel_dataX[n];
    y += FIRCoef[n] * accel_dataY[n];
    z += FIRCoef[n] * accel_dataZ[n];
  }
  
  //не менять пока массив полностью не заполнится
  if(count >= Ntap - 2) {
    *axis_X = x / DCgain;
    *axis_Y = y / DCgain;
    *axis_Z = z / DCgain;
  }
  else
    count++;
}

//Экспоненциально взвешенное скользящее среднее EMA (exponential moving average)
int16_t filter_EMA_ADC(float adc_data, uint8_t alpha_x10) {

    //alpha от 0 до 10, чем меньше его значение тем больше влияние предыдущих значений на текущую величину среднего
    static float old_adc = 0;

    //axis(t) = alpha * axis(t-1) + (1 - alpha) * old_axis = old_axis + alpha * (axis(t-1) - old_axis)
    //axis(0) = old_axis(0);
    if(!old_adc)
      old_adc = adc_data;

    adc_data = old_adc + (alpha_x10 * (adc_data - old_adc) / 10);

    old_adc = adc_data;
    return (int16_t)(adc_data + 0.5);
}

//Экспоненциально взвешенное скользящее среднее EMA (exponential moving average)
void filter_EMA(int16_t *axis_X, int16_t *axis_Y, int16_t *axis_Z, uint8_t alpha_x10) {

    //alpha от 0 до 10, чем меньше его значение тем больше влияние предыдущих значений на текущую величину среднего
  
    static int16_t old_axis_X = 0, old_axis_Y = 0, old_axis_Z = 0;

    //axis(t) = alpha * axis(t-1) + (1 - alpha) * old_axis = old_axis + alpha * (axis(t-1) - old_axis)
    //axis(0) = old_axis(0);
    if(!old_axis_X)
        old_axis_X = *axis_X;
    if(!old_axis_Y)
        old_axis_Y = *axis_Y;
    if(!old_axis_Z)
        old_axis_Z = *axis_Z;
    
    *axis_X = old_axis_X + ((alpha_x10 * (*axis_X - old_axis_X) + 5) / 10);
    *axis_Y = old_axis_Y + ((alpha_x10 * (*axis_Y - old_axis_Y) + 5) / 10);
    *axis_Z = old_axis_Z + ((alpha_x10 * (*axis_Z - old_axis_Z) + 5) / 10);

    old_axis_X = *axis_X;
    old_axis_Y = *axis_Y;
    old_axis_Z = *axis_Z;
}

void filter_average(int16_t *axis_X, int16_t *axis_Y, int16_t *axis_Z) {

  static uint8_t  count = 0;
  static int16_t accel_dataX[NUMB_OF_CHECKS] = {0};
  static int16_t accel_dataY[NUMB_OF_CHECKS] = {0};
  static int16_t accel_dataZ[NUMB_OF_CHECKS] = {0};
  static int16_t old_axis_X = 0, old_axis_Y = 0, old_axis_Z = 0;

  //пока массив не заполнится (10 значений), выводим предыдущее расчитанное значение, иначе расчет среднего значения
  if(count < NUMB_OF_CHECKS) {
    accel_dataX[count] = *axis_X;   //X
    accel_dataY[count] = *axis_Y;   //Y
    accel_dataZ[count] = *axis_Z;   //Z
    *axis_X = old_axis_X;
    *axis_Y = old_axis_Y;
    *axis_Z = old_axis_Z;
    count++;
  }
  else {
    int16_t min1 = accel_dataX[0];
    int16_t min2 = accel_dataY[0];
    int16_t min3 = accel_dataZ[0];
    for(uint8_t m = 1; m < NUMB_OF_CHECKS; m++) {
      if(accel_dataX[m] < min1) min1 = accel_dataX[m];
      if(accel_dataY[m] < min2) min2 = accel_dataY[m];
      if(accel_dataZ[m] < min3) min3 = accel_dataZ[m];
    }

    int16_t max1 = accel_dataX[0];
    int16_t max2 = accel_dataY[0];
    int16_t max3 = accel_dataZ[0];
    for(uint8_t m = 1; m < NUMB_OF_CHECKS; m++) {
      if(accel_dataX[m] < max1) max1 = accel_dataX[m];
      if(accel_dataY[m] < max2) max2 = accel_dataY[m];
      if(accel_dataZ[m] < max3) max3 = accel_dataZ[m];
    }

    int32_t result1 = 0, result2 = 0, result3 = 0;
    for(uint8_t m = 0; m < NUMB_OF_CHECKS; m++) {
      result1 += accel_dataX[m];
      result2 += accel_dataY[m];
      result3 += accel_dataZ[m];
    }
    *axis_X = ((result1 - min1 - max1) / (NUMB_OF_CHECKS - 2));
    *axis_Y = ((result2 - min2 - max2) / (NUMB_OF_CHECKS - 2));
    *axis_Z = ((result3 - min3 - max3) / (NUMB_OF_CHECKS - 2));

    //от 0 до 10 скорость изменения
    //filter_EMA(axis_X, axis_Y, axis_Z, 5);

    filtr_Chebyshev(axis_X, axis_Y, axis_Z);

    old_axis_X = *axis_X;
    old_axis_Y = *axis_Y;
    old_axis_Z = *axis_Z;

    count = 0;
  }
}

//atan(x)=pi/6+atan((x*sqrt(3)-1)/(x+sqrt(3))).
//Здесь sqrt(3) квадратный корень из 3. При этом необходимо запомнить число шагов (возможно, ноль).
//После этого, арктангенс на интервале [0,pi/12] аппроксимируется формулой:
//atan(x) = x*(0.55913709/(1.4087812+x2) +0.60310579-0.05160454*x2)
//Затем к полученному результату добавляется столько pi/6, сколько было шагов сокращения области определения. 
//Затем, в случае обращения, аргумента, результат вычитается из pi/2.
float arctg(float x) {

  uint8_t flag_minus = 0;
  if(x < 0) {x = -x; flag_minus = 1;}

  int invertion = 0, sp=0;
  float x2,angle;

  // check up the invertation 
  if(x > 1.F) {
    x=1.F / x;
    invertion = 1;
  }
  // process shrinking the domain until x<PI/12
  while(x > M_PI12) {
    sp++; 
    angle = x + SQRT3; 
    angle = 1.F / angle; 
    x *= SQRT3; 
    x -= 1.F; 
    x *= angle;
  }
  // calculation core 
  //atan(x) = x*(0.55913709/(1.4087812+x2) +0.60310579-0.05160454*x2)
  x2=x*x; 
  angle=x2+1.4087812F; 
  angle=0.55913709F / angle; 
  angle += 0.60310579F;
  angle -= 0.05160454F * x2; 
  angle *= x;
  // process until sp=0 
  while(sp > 0) {
    angle += M_PI6;
    sp--;
  }
  // invertation took place 
  if(invertion) 
    angle = M_PI2 - angle;

  //radians to degrees
  angle = angle * 180 / M_PI;

  if(flag_minus)
    angle = -angle;

  return(angle);
}
