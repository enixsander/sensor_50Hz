#ifndef __CAN_HANDLING_H
#define __CAN_HANDLING_H

#include "main.h"

void CAN_j1939_Init(void);
void CAN_j1939_Filter(void);
void CAN_j1939_Receive(void);
void CAN_j1939_Transmit(void);

  
#define MASK_12 0x03
#define MASK_34 0x0C
#define MASK_56 0x30
#define MASK_78 0xC0
  
#define bit1 0x01
#define bit3 0x04
#define bit5 0x10
#define bit7 0x40

#endif // __CAN_HANDLING_H 
