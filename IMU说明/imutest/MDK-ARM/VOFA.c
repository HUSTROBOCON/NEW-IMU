#include "stm32f4xx.h"
#include "usart.h"
#include "VOFA.h"
#include "stdio.h"

uint8_t tail[4]={0x00,0x00,0x80,0x7f};

//printf重定向
int fputc(int ch, FILE *f)
 
{
  HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

void JustFloat_send(float data1,float data2,float data3,float data4)
{
	HAL_UART_Transmit(&huart4,(uint8_t*)&data1,sizeof(float)*1,5);
	HAL_UART_Transmit(&huart4,(uint8_t*)&data2,sizeof(float)*1,5);
	HAL_UART_Transmit(&huart4,(uint8_t*)&data3,sizeof(float)*1,5);
	HAL_UART_Transmit(&huart4,(uint8_t*)&data4,sizeof(float)*1,5);
	HAL_UART_Transmit(&huart4,(uint8_t*)tail,4,5);
}

void RawData_send(uint8_t* buffer,uint8_t size)
{
	HAL_UART_Transmit(&huart4,(uint8_t*)buffer,size,5);
}

void FireWater_send_int(int data1,int data2,int data3,int data4)
{
	printf("%d,%d,%d,%d\r\n",data1,data2,data3,data4);
}

void FireWater_send_f(float data1,float data2,float data3,float data4)
{
	printf("%.2f,%.2f,%.2f,%.2f\r\n",data1,data2,data3,data4);
}

//用来刷新屏幕的，因为vofa好像没有清空波形的功能，所以有需要的话可以拿这个放在开始的时候刷一下
void ClearScreen()
{
	float clear=0;
	for(int i=0;i<=200;i++)
	{
		JustFloat_send(0,0,0,0);
	}
}