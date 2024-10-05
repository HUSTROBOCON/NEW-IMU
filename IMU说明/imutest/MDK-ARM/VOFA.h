#ifndef VOFA_H_
#define VOFA_H_

#include "stm32f4xx.h"

void JustFloat_send(float data1,float data2,float data3,float data4);
void RawData_send(uint8_t* buffer,uint8_t size);
void FireWater_send_int(int data1,int data2,int data3,int data4);
void FireWater_send_f(float data1,float data2,float data3,float data4);
void ClearScreen();
#endif 