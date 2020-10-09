#include "can_handling.h"
#include "adc_data.h"

extern CAN_HandleTypeDef hcan;

//Time for one bit(ns) = 1 / 250 Kbit/s = 4000ns
//Time Quantum = 4000ns / (1 + 5 + 2) = 500ns
//Prescaler = 8 MHz / (1 / 500ns) = 4

extern uint8_t flag_KZ[2];
extern uint8_t antenna_gain;
extern uint8_t output_state[2];
extern uint16_t wind_speed_x10;
extern uint8_t accelerometer_ID;
extern uint16_t power_line_50Hz;
extern uint8_t flag_limit_switch;
extern uint8_t antenna_sensitivity;
extern int16_t axis_X, axis_Y, axis_Z;
extern uint8_t Rx_Data_BIK_buttons[8], Rx_Data_BPK_Input[8];

CAN_TxHeaderTypeDef tx_j1939_acceleration, tx_j1939_state_message;
__IO uint8_t time_j1939_accel = 0, time_j1939_info = 0;
__IO uint8_t new_rx[2] = {0};

uint8_t tx_j1939_accel_Data[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t tx_j1939_info_Data[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF};
uint32_t mail_Box = 0;

static void tx_header_Init(CAN_TxHeaderTypeDef *tx, uint32_t id) {
  tx->ExtId = id; //(uint32_t)ACCELEROM_ID << 3) | 0x00000004;//(uint32_t)(61485 << 11) | 0x00000004;
  tx->IDE = CAN_ID_EXT;
  tx->DLC = 8;
}

void CAN_j1939_Init(void) {
  //CAN
  CAN_RS_GPIO_Port->BSRR |= (CAN_RS_Pin << 16); // CAN_RS
  //CAN_RS_GPIO_Port->BSRR |= CAN_RS_Pin; // CAN_RS = 1 -- LOW Power

  CAN_j1939_Filter();
  __HAL_CAN_ENABLE_IT(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan);

  //PGN 61485, SPN 5347-5349, Acceleration Sensor, 1-6 byte position, 10ms
  tx_header_Init(&tx_j1939_acceleration, ACCELEROM_ID);
  tx_header_Init(&tx_j1939_state_message, STATE_ID);
}

void CAN_j1939_Transmit() {

  //PGN 61485, SPN 5347-5349, Acceleration Sensor, 10ms
  if(!time_j1939_accel) {
    time_j1939_accel = 10;

    tx_j1939_accel_Data[0] = (uint8_t)axis_X;       //Lateral Acceleration
    tx_j1939_accel_Data[1] = (uint8_t)(axis_X >> 8);
    tx_j1939_accel_Data[2] = (uint8_t)axis_Y;       //Longitudinal Acceleration
    tx_j1939_accel_Data[3] = (uint8_t)(axis_Y >> 8);
    tx_j1939_accel_Data[4] = (uint8_t)axis_Z;       //Vertical Acceleration
    tx_j1939_accel_Data[5] = (uint8_t)(axis_Z >> 8);

    tx_j1939_accel_Data[6] = (uint8_t)wind_speed_x10;       //скорость ветра 0,1м/с / bit
    tx_j1939_accel_Data[7] = (uint8_t)(wind_speed_x10 >> 8);

    HAL_CAN_AddTxMessage(&hcan, &tx_j1939_acceleration, (uint8_t*)tx_j1939_accel_Data, &mail_Box);
  }
  if(!time_j1939_info) {
    time_j1939_info = 50;

    tx_j1939_info_Data[0] = (uint8_t)power_line_50Hz;
    tx_j1939_info_Data[1] = (uint8_t)(power_line_50Hz >> 8);
    tx_j1939_info_Data[2] = 0xC0;
    if(output_state[0] == ON)
      tx_j1939_info_Data[2] |= bit1;  //P1 - Габаритные огни (ГС)
    if(output_state[1] == ON)
      tx_j1939_info_Data[2] |= bit3;  //P2 (БПК SB13) - подсветка крюка
    if(flag_limit_switch == ON)
      tx_j1939_info_Data[2] |= bit5;  //концевик

    //ток на выходе 1 - Габаритные огни
    uint16_t amper_x10 = current_amper_x10(OUT1);
    tx_j1939_info_Data[3] = (uint8_t)amper_x10;
    tx_j1939_info_Data[4] = (uint8_t)(amper_x10 >> 8);
    //ток на выходе 2 - подсветка крюка
    amper_x10 = current_amper_x10(OUT2);
    tx_j1939_info_Data[5] = (uint8_t)amper_x10;
    tx_j1939_info_Data[6] = (uint8_t)(amper_x10 >> 8);

    HAL_CAN_AddTxMessage(&hcan, &tx_j1939_state_message, tx_j1939_info_Data, &mail_Box);
  }
}

void CAN_j1939_Receive(void) 
{
  if(new_rx[0]) //BPK_Input_ID
  {
    //6030-2ERA   POWER_KEY
    if((Rx_Data_BPK_Input[0] & MASK_34) == bit3)  //P1 - Габаритные огни (ГС)
      out_state(OUT1, ON);   
    else if((Rx_Data_BPK_Input[0] & MASK_34) != MASK_34) {
      out_state(OUT1, OFF);
      flag_KZ[OUT1] = 0;
    }

    new_rx[0] = 0;
  }

  if(new_rx[1]) {  //BUTTONS_ID

    //БИК SB13 - подсветка крюка
    if((Rx_Data_BIK_buttons[1] & MASK_78) == bit7)
      out_state(OUT2, ON);
    else if((Rx_Data_BIK_buttons[1] & MASK_78) != MASK_78) {
      out_state(OUT2, OFF);
      flag_KZ[OUT2] = 0;
    }

    //Настройка антенны
    //усиление, от 0 до 127
    if(Rx_Data_BIK_buttons[4] <= 0x7F)
      antenna_gain = Rx_Data_BIK_buttons[4];
    //чувствительность
    if(Rx_Data_BIK_buttons[5] < 0x04)
      antenna_sensitivity = Rx_Data_BIK_buttons[5];

    antenna_tuning();
    new_rx[1] = 0;
  }
}

void CAN_j1939_Filter(void) {
  uint32_t temp;
  CAN_FilterTypeDef filter;
  //ID = 0x00D000xx
  //((000xb)(xx00b) D0 00 xx) << 3
  //  xxx - priority, 00b - reserved, 0xD000 - PGN, xx - source addr
 
  // PGN... ID = 0x18EEEE00u 
  temp = (uint32_t)(BPK_Input_ID << 3) | 0x00000004;
  filter.FilterIdHigh = (uint16_t) (temp >> 16);
  filter.FilterIdLow = (uint16_t) temp;
  filter.FilterMaskIdHigh = 0x1FFF;
  filter.FilterMaskIdLow = 0xF000;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterBank = 0;
  filter.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan,&filter);

  temp = (uint32_t)(BUTTONS_ID << 3) | 0x00000004;
  filter.FilterIdHigh = (uint16_t) (temp >> 16);
  filter.FilterIdLow = (uint16_t) temp;
  filter.FilterBank = 1;
  HAL_CAN_ConfigFilter(&hcan,&filter);
}
