#ifndef __IMU_H
#define __IMU_H
#include "parameter.h"

float DATA_Trans(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4);
long long timestamp(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4,uint8_t Data_5,uint8_t Data_6,uint8_t Data_7,uint8_t Data_8);
uint8_t TTL_Hex2Dec(void);

#define DT_US_COUNT   ((uint16_t)(TIM6->CNT))
#define IMU_INTEGRAL_TIME 500

#define RAW_DATA_DEADZONE 0.001
#define IMU_POSTIVE_PARA 0.0008
#define IMU_NEGATIVE_PARA -0.0001875

#define IMU_RATE_DEAD 0.2f
extern float yaw;
extern float yaw_ZeroDraft;
extern float yaw_process;
extern float look;
typedef struct Imu_data_t
{
	float Yaw_process;  //处理角度的过程变量
	float Yaw_speed;
	float Yaw_last;
	float Yaw_d;
	float Yaw_speed_last[35];
	float Yaw_Integral;  //角度行程的积分
	float start_yaw;   //IMU的起始角度
	float Yaw_draft;   //IMU角速度漂移
	float Yaw;
	float Yaw_predict;
}Imu_data_t;

//采用角速度积分算法的IMU结构体，精度并不如采用原始数据的算法
typedef struct Imu_data_i
{
	float Yaw_i;
	float Yaw_i_draft;
	float Yaw_i_speed;
	float Yaw_i_last;
	float Yaw_i_delta;
	float Yaw_i_CoeffNegative[3];
  float Yaw_i_CoeffPositive[3];
}Imu_data_i;

extern Imu_data_t Imu_data;
extern Imu_data_i Imu_i_data;
extern uint8_t IMU_rdy;
void Imu_Init(Imu_data_t *Imu_t,Imu_data_i *Imu_i);
void Imu_Get_Data(Imu_data_t *Self);
void Imu_Get_Data_Integral(Imu_data_i *Self);
#endif