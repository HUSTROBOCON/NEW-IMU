#include "stdio.h"
#include "stm32f4xx.h" 
#include "imu.h"
#include "parameter.h"
#include "math.h"
#include "cmsis_os.h"
#include "time.h"
#include "tim.h"
#include "Gyro.h"
int rs_imutype =0;
int rs_ahrstype =0;
extern int Time_count;


int start_time;
int delta_time;

uint8_t IMU_rdy=0;
void IMU_msg_process(uint8_t 	Usart_Receive);


void IMU_msg_process(uint8_t Usart_Receive)
{
	static uint8_t Count=0;
  static uint8_t last_rsnum=0;
	static uint8_t rsimu_flag=0;
	static uint8_t rsacc_flag=0;
	
	Fd_data[Count] = Usart_Receive;  

		if(((last_rsnum == FRAME_END)&&(Fd_data[0]==FRAME_HEAD))||Count>0)
		{
			Count++;
			if((Fd_data[1]==TYPE_IMU)&&(Fd_data[2]==IMU_LEN))
				rsimu_flag=1;
			if((Fd_data[1]==TYPE_AHRS)&&(Fd_data[2]==AHRS_LEN))
				rsacc_flag=1;
		}
		else
		{
			Count=0;
		}
		last_rsnum = Usart_Receive;
		
		if((rsimu_flag==1)&&(Count == IMU_RS))
		{
				Count=0;
	      rsimu_flag=0;
		    rs_imutype=1;
				if(Fd_data[IMU_RS-1]==FRAME_END) //֡βУѩ
				memcpy(Fd_rsimu, Fd_data, sizeof(Fd_data));  
		}
		if(rsacc_flag==1 && Count==AHRS_RS) //
		{
			Count=0;
			rsacc_flag=0;
			rs_ahrstype=1;
			if(Fd_data[AHRS_RS-1]==FRAME_END)
			{
				memcpy(Fd_rsahrs, Fd_data, sizeof(Fd_data));
			}
			for(int i=0;i<sizeof(Fd_data);i++) 
			{
				Fd_data[i]=0;
			}
		}
}

uint8_t TTL_Hex2Dec(void)  
{
	 if(rs_ahrstype==1)
	{ 
		if(Fd_rsahrs[1]==TYPE_AHRS&&Fd_rsahrs[2]==AHRS_LEN)
		{	
		AHRSData_Packet.RollSpeed=DATA_Trans(Fd_rsahrs[7],Fd_rsahrs[8],Fd_rsahrs[9],Fd_rsahrs[10]);       //横滚角速度
		AHRSData_Packet.PitchSpeed=DATA_Trans(Fd_rsahrs[11],Fd_rsahrs[12],Fd_rsahrs[13],Fd_rsahrs[14]);   //俯仰角速度
		AHRSData_Packet.HeadingSpeed=DATA_Trans(Fd_rsahrs[15],Fd_rsahrs[16],Fd_rsahrs[17],Fd_rsahrs[18]); //偏航角速度
			
    AHRSData_Packet.Roll=DATA_Trans(Fd_rsahrs[19],Fd_rsahrs[20],Fd_rsahrs[21],Fd_rsahrs[22]);      //横滚角
		AHRSData_Packet.Pitch=DATA_Trans(Fd_rsahrs[23],Fd_rsahrs[24],Fd_rsahrs[25],Fd_rsahrs[26]);     //俯仰角
		AHRSData_Packet.Heading=DATA_Trans(Fd_rsahrs[27],Fd_rsahrs[28],Fd_rsahrs[29],Fd_rsahrs[30]);	 //偏航角
			
		AHRSData_Packet.Qw=DATA_Trans(Fd_rsahrs[31],Fd_rsahrs[32],Fd_rsahrs[33],Fd_rsahrs[34]);  //四元数
		AHRSData_Packet.Qx=DATA_Trans(Fd_rsahrs[35],Fd_rsahrs[36],Fd_rsahrs[37],Fd_rsahrs[38]);
		AHRSData_Packet.Qy=DATA_Trans(Fd_rsahrs[39],Fd_rsahrs[40],Fd_rsahrs[41],Fd_rsahrs[42]);
		AHRSData_Packet.Qz=DATA_Trans(Fd_rsahrs[43],Fd_rsahrs[44],Fd_rsahrs[45],Fd_rsahrs[46]);
		AHRSData_Packet.Timestamp=timestamp(Fd_rsahrs[47],Fd_rsahrs[48],Fd_rsahrs[49],Fd_rsahrs[50],Fd_rsahrs[51],Fd_rsahrs[52],Fd_rsahrs[53],Fd_rsahrs[54]);   //时间戳
		}
	rs_ahrstype=0;
 }
	if(rs_imutype==1)
	{
		if(Fd_rsimu[1]==TYPE_IMU&&Fd_rsimu[2]==IMU_LEN)
		{
		IMUData_Packet.gyroscope_x=DATA_Trans(Fd_rsimu[7],Fd_rsimu[8],Fd_rsimu[9],Fd_rsimu[10]);  //角速度
		IMUData_Packet.gyroscope_y=DATA_Trans(Fd_rsimu[11],Fd_rsimu[12],Fd_rsimu[13],Fd_rsimu[14]);
		IMUData_Packet.gyroscope_z=DATA_Trans(Fd_rsimu[15],Fd_rsimu[16],Fd_rsimu[17],Fd_rsimu[18]);
			
		IMUData_Packet.accelerometer_x=DATA_Trans(Fd_rsimu[19],Fd_rsimu[20],Fd_rsimu[21],Fd_rsimu[22]);  //线加速度
		IMUData_Packet.accelerometer_y=DATA_Trans(Fd_rsimu[23],Fd_rsimu[24],Fd_rsimu[25],Fd_rsimu[26]);
		IMUData_Packet.accelerometer_z=DATA_Trans(Fd_rsimu[27],Fd_rsimu[28],Fd_rsimu[29],Fd_rsimu[30]);

		IMUData_Packet.magnetometer_x=DATA_Trans(Fd_rsimu[31],Fd_rsimu[32],Fd_rsimu[33],Fd_rsimu[34]);  //磁力计数据
		IMUData_Packet.magnetometer_y=DATA_Trans(Fd_rsimu[35],Fd_rsimu[36],Fd_rsimu[37],Fd_rsimu[38]);
		IMUData_Packet.magnetometer_z=DATA_Trans(Fd_rsimu[39],Fd_rsimu[40],Fd_rsimu[41],Fd_rsimu[42]);
			
	  	IMUData_Packet.Timestamp=timestamp(Fd_rsimu[55],Fd_rsimu[56],Fd_rsimu[57],Fd_rsimu[58],Fd_rsimu[59],Fd_rsimu[60],Fd_rsimu[61],Fd_rsimu[62]);   //时间戳
		}
		rs_imutype=0;
		
		
 }
	if(IMU_rdy)
	{
		Imu_Get_Data_Integral(&Imu_i_data);
		Imu_data.Yaw_speed = -1 * GyroSystem.Gyro.Rate;
		Imu_Get_Data(&Imu_data);
	}
	return 0;
}

/*************
实现16进制的can数据转换成浮点型数据，感觉不用看，直接抄
****************/
float DATA_Trans(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4)
{
  uint32_t transition_32;
	float tmp=0;
	int sign=0;
	int exponent=0;
	float mantissa=0;
	double exponent_dou = 0;
	double pow_X = 2;
  transition_32 = 0;
  transition_32 |=  Data_4<<24;   
  transition_32 |=  Data_3<<16; 
	transition_32 |=  Data_2<<8;
	transition_32 |=  Data_1;
  sign = (transition_32 & 0x80000000) ? -1 : 1;//符号位
	//先右移操作，再按位与计算，出来结果是30到23位对应的e
	exponent = ((transition_32 >> 23) & 0xff) - 127;
	exponent_dou = (double)exponent;
	//将22~0转化为10进制，得到对应的x系数 
	mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
	tmp=sign * mantissa * pow(pow_X,exponent_dou);
	return tmp;
}

//移位有bug，我表示不理解
long long timestamp(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4,uint8_t Data_5,uint8_t Data_6,uint8_t Data_7,uint8_t Data_8)
{
  long long transition_64;
  transition_64 = 0; 
  transition_64 |=  Data_8<<56;   
  transition_64 |=  Data_7<<48; 
	transition_64 |=  Data_6<<40;
	transition_64 |=  Data_5<<32;	
  transition_64 |=  Data_4<<24;
  transition_64 |=  Data_3<<16; 
	transition_64 |=  Data_2<<8;
	transition_64 |=  Data_1;
	return transition_64;
}
//测量周期
float measurement_period;

void IMU_Zero_Drift(AHRSData_Packet_t *Self)
{
	double Startyaw=0.0f;
	static float last_yaw_orgin=0;
	float delta_yaw_orgin=0;
	
//这里delay5秒是因为IMU刚启动的时候温漂比较离谱，过至少5秒之后才正常下来
	osDelay(3000);
//这里需要测量两个参数：陀螺仪数据的起始角度和角速度温漂
    for (int i = 0; i < IMU_INTEGRAL_TIME; i++)
    {
			Startyaw+=Self->Heading;
			delta_yaw_orgin+=Self->Heading-last_yaw_orgin;
			last_yaw_orgin=Self->Heading;
			
			Imu_i_data.Yaw_i_draft+=Self->HeadingSpeed*57.3;
      osDelay(1);
    }
		Startyaw /= IMU_INTEGRAL_TIME;
		Imu_data.Yaw_draft=delta_yaw_orgin/IMU_INTEGRAL_TIME*57.3;
		Imu_data.start_yaw=(float)Startyaw*57.3-Imu_data.Yaw_draft*measurement_period;
		
		Imu_i_data.Yaw_i_draft /= IMU_INTEGRAL_TIME;
}
//标志位
uint8_t first_data=0;

void Imu_t_Init(Imu_data_t *Self)
{
	Self->Yaw_d = 0;
	Self->Yaw_last = 0;
	Self->Yaw_speed = 0;
	
	Self->start_yaw=0;
	Self->Yaw_Integral=0;
	Self->Yaw=0;
	first_data=0;
	
	for(int i=0;i<35;i++)
	{
		Self->Yaw_speed_last[i]=0;
	}
	Self->Yaw_predict=0;
	measurement_period=(__HAL_TIM_GetAutoreload(&htim5)+1)/1000;
	
}

void Imu_i_Init(Imu_data_i *Self)
{
	Self->Yaw_i = 0.0f;
  Self->Yaw_i_last = 0.0f;
  Self->Yaw_i_delta = 0.0f;
  Self->Yaw_i_speed = 0.0f;
  Self->Yaw_i_draft = 0.0f;
  Self->Yaw_i_CoeffNegative[0] = 1.00170693779270000000f;//1.00320887876701000000f;  // 1.00210532936093000000f;//1.00219154201177000000f;
  Self->Yaw_i_CoeffNegative[1] = 0.00001729850040699250f;//0.00006424997938747620f;  // 0.00003796811222844810f;//0.00007270645661039410f;
  Self->Yaw_i_CoeffNegative[2] = 0.00000005641299708533f;//0.00000196653028001516f;  // 0.00000179981794081602f;//0.00000201023445480647f;
  Self->Yaw_i_CoeffPositive[0] = 1.00221700036771000000f;//1.00399417065620000000f;  // 1.00486598170788000000f;//1.00251033404716000000f;
  Self->Yaw_i_CoeffPositive[1] = -0.00002428023356171320f;//-0.00008559299350530130f; //-0.00011541488496424900f;//-0.00007309072921035660f;
  Self->Yaw_i_CoeffPositive[2] = 0.00000007733011907814f;//0.00000206866180874066f;  // 0.00000221231502608651f;//0.00000201668114984364f;
}

void Imu_Init(Imu_data_t *Imu_t,Imu_data_i *Imu_i)
{
	Imu_t_Init(Imu_t);
	Imu_i_Init(Imu_i);
	IMU_Zero_Drift(&AHRSData_Packet);
	IMU_rdy=1;
}
//下面两个变量都是用来修正角速度漂移的
int count=0;
uint8_t flag=0;
float look;
void Imu_Get_Data(Imu_data_t *Self)
{
	//做过死区限制的原始数据，和下面没做过死区的rawdata要区分开
		static float last_rawdata_dz=0;
	//静态标志位
		uint8_t static_flag=0;
	//以下变量都是未经过死区处理的原始数据
		static float last_rawdata=0;
		float delta_rawdata=0;
		float rawdata=AHRSData_Packet.Heading;
		float last_draft;
		
//		AHRSData_Packet.Heading-=measurement_period*Self->Yaw_draft/57.3;
	//死区处理	
		if(fabs(AHRSData_Packet.Heading-last_rawdata_dz)<=RAW_DATA_DEADZONE)
		{
			AHRSData_Packet.Heading=last_rawdata_dz;
			//如果角度变化量大于死区约束，则认为IMU进入动态状态，否则为静态
			static_flag=1;
		}
		else
		{
			last_rawdata_dz=AHRSData_Packet.Heading;
		}
		
	//由于IMU的漂移会随时间变化，所以在这里进行漂移量重测
		if(static_flag)
		{
			if(!flag)
			{
				//如果某次静态状态下测量的数据超过了5组，则认为是有效的，将最新测量结果作为新的漂移偏置
				if(count>=5)
				{
					last_draft=Self->Yaw_draft;
					Self->Yaw_draft=delta_rawdata/count/measurement_period*57.3f;
					Self->start_yaw-=Imu_data.Yaw_draft*measurement_period - last_draft * measurement_period;
				}
				count=0;
			}
			flag=1;
		}
		//静态下开始重新累计漂移量
		else
		{
			count++;
			delta_rawdata+=rawdata-last_rawdata;
			flag=0;
		}
		last_rawdata=rawdata;
		
		//将过程角度减去初始角度并限制在0-360
	  Self->Yaw_process = 57.3*AHRSData_Packet.Heading;
		if(Self->Yaw_process >0 && Self->Yaw_process-Self->start_yaw <0)
		{
			Self->Yaw_process=360.0f- Self->start_yaw +Self->Yaw_process;
		}
		else
		{
			Self->Yaw_process -= Self->start_yaw ;
		}
		
		//由于动态过程中的漂移很难观测到，所以这里只在静态的状态下进行漂移的修正
		if(static_flag)
		{
			Self->Yaw_process -= Self->Yaw_draft;
		}
		//第一次测量得到的数据不要
		if(!first_data)
		{
			first_data=1;
		}
		else
		{
			//为了防止角度从360->0或0->360跳变时出现Yaw_d异常
			if(Self->Yaw_process<10.0f&&Self->Yaw_last>350.0f)
			{
				Self->Yaw_d=Self->Yaw_process+360.0f-Self->Yaw_last;
			}
			else if(Self->Yaw_process>350.0f&&Self->Yaw_last<10.0f)
			{
				Self->Yaw_d=Self->Yaw_process-360.0f-Self->Yaw_last;
			}
			else
			{
				Self->Yaw_d=Self->Yaw_process-Self->Yaw_last;
			}
		}
		
		Self->Yaw-=Self->Yaw_last;
		Self->Yaw_last=Self->Yaw_process;
		Self->Yaw_Integral+=Self->Yaw_d;
		//为了防止角度积分初始为360°
		if(first_data==1)
		{
			first_data=2;
			if(Self->Yaw_Integral>300)
			{
				Self->Yaw_Integral=0;
			}
		}
		
		//对角度进行修正得到最终角度输出
		if(Self->Yaw_d>=0)
		{
			Self->Yaw+=Self->Yaw_process+IMU_POSTIVE_PARA*Self->Yaw_d;
		}
		else
		{
			Self->Yaw+=Self->Yaw_process+IMU_NEGATIVE_PARA*Self->Yaw_d;
		}
		//对角度进行预测修正，这里认为延迟为175ms，所以用陀螺仪前35帧数据做积分
		Self->Yaw -= Self->Yaw_predict;
		
		
		look=Self->Yaw;
		
		Self->Yaw_predict=0;
		for(int i=0;i<34;i++)
		{
			Self->Yaw_speed_last[34-i]=Self->Yaw_speed_last[34-i-1];
			Self->Yaw_predict += Self->Yaw_speed_last[34-i]*5.0f/1000.0f;
		}
		Self->Yaw_speed_last[0]=Self->Yaw_speed;
		Self->Yaw_predict += Self->Yaw_speed_last[0]*5.0f/1000.0f;
		Self->Yaw += Self->Yaw_predict;
		
		//将最终角度限制在0-360间
		if(Self->Yaw>360)
		{
			Self->Yaw-=360;
		}
		if(Self->Yaw<0)
		{
			Self->Yaw+=360;
		}
}

//采用角速度积分得到imu角度的算法，具体内容和陀螺仪一样
void Imu_Get_Data_Integral(Imu_data_i *Self)
{
		static bool Is_First_Cal_Angle;
    static float Gyro_Pre_Rate;
    uint16_t Now_Time;
    static uint16_t Pre_Time = 0;
    uint16_t Delta_Time;
    float Gyro_Mid_Rate;

		Self->Yaw_i_speed=AHRSData_Packet.HeadingSpeed*57.3f;
	
	  if (Self->Yaw_i_speed < 0)
		{
        Self->Yaw_i_speed = Self->Yaw_i_CoeffNegative[2] * Self->Yaw_i_speed * Self->Yaw_i_speed *
                        Self->Yaw_i_speed +
                    Self->Yaw_i_CoeffNegative[1] * Self->Yaw_i_speed * Self->Yaw_i_speed +
                    Self->Yaw_i_CoeffNegative[0] * Self->Yaw_i_speed;
		}
    else if (Self->Yaw_i_speed > 0)
		{
        Self->Yaw_i_speed = Self->Yaw_i_CoeffPositive[2] * Self->Yaw_i_speed * Self->Yaw_i_speed *
                        Self->Yaw_i_speed +
                    Self->Yaw_i_CoeffPositive[1] * Self->Yaw_i_speed * Self->Yaw_i_speed +
                    Self->Yaw_i_CoeffPositive[0] * Self->Yaw_i_speed;
		}
    Self->Yaw_i_speed -= Self->Yaw_i_draft;
    Now_Time = DT_US_COUNT; // unit : us
		if(Pre_Time>Now_Time)
		{
			Delta_Time=Now_Time - Pre_Time +65535;
		}
		else
		{
			Delta_Time = Now_Time - Pre_Time;
		}
    Pre_Time = Now_Time;

    if (Is_First_Cal_Angle == true)
    {
				Gyro_Pre_Rate = Self->Yaw_i_speed; 
        Is_First_Cal_Angle = false;
    }
    else
    {
        Gyro_Mid_Rate = (Self->Yaw_i_speed + Gyro_Pre_Rate) * 0.5f;
        Gyro_Pre_Rate = Self->Yaw_i_speed;

        if (fabs(Gyro_Mid_Rate) <= IMU_RATE_DEAD)
            Gyro_Mid_Rate = 0.0f;

        Self->Yaw_i_last = Self->Yaw_i;
        Self->Yaw_i_delta = Gyro_Mid_Rate * Delta_Time / 1000000.0f;
        Self->Yaw_i += Self->Yaw_i_delta;
    }
}