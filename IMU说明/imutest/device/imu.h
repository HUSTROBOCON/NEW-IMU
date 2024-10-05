#ifndef __IMU_H
#define __IMU_H
#include "parameter.h"

float DATA_Trans(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4);
long long timestamp(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4,uint8_t Data_5,uint8_t Data_6,uint8_t Data_7,uint8_t Data_8);
uint8_t TTL_Hex2Dec(void);

#define DT_US_COUNT   ((uint16_t)(TIM6->CNT))
#define IMU_INTEGRAL_TIME 500

#define RAW_DATA_DEADZONE 0.001
#define IMU_POSTIVE_PARA 0.0005
#define IMU_NEGATIVE_PARA 0.0005125

extern float yaw;
extern float yaw_ZeroDraft;
extern float yaw_process;
typedef struct Imu_data_t
{
	float Roll;
	float Pitch;
	
	float Yaw_process;  //处理角度的过程变量
	float Roll_spped;
	float Pitch_speed;
	float Yaw_speed;
	float Yaw_last;
	float Yaw_d;
	
	float Yaw_Integral;  //角度行程的积分
	float start_yaw;   //IMU的起始角度
	float Yaw_draft;   //IMU角速度漂移
	float Yaw;
}Imu_data_t;

extern Imu_data_t Imu_data;

void Imu_init(Imu_data_t *Self);
void Imu_Get_Data(Imu_data_t *Self);
#endif