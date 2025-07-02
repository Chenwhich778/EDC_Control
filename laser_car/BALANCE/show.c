#include "show.h"
int Voltage_Show;
unsigned char i;
unsigned char Send_Count;
extern SEND_DATA Send_Data;
extern int MPU9250ErrorCount, EncoderA_Count, EncoderB_Count, EncoderC_Count, EncoderD_Count;
extern int MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;

u8 oled_refresh_flag;
u8 oled_page = 0;
/**************************************************************************
Function: Read the battery voltage, buzzer alarm, start the self-test, send data to APP, OLED display task
Input   : none
Output  : none
函数功能：读取电池电压、蜂鸣器报警、开启自检、向APP发送数据、OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
int Buzzer_count = 25;
void show_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		int i = 0;
		static int LowVoltage_1 = 0, LowVoltage_2 = 0;
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ)); // This task runs at 10Hz //此任务以10Hz的频率运行

		// 开机时蜂鸣器短暂蜂鸣，开机提醒
		// The buzzer will beep briefly when the machine is switched on
		if (SysVal.Time_count < 50)
			Buzzer = 1;
		else if (SysVal.Time_count >= 51 && SysVal.Time_count < 100)
			Buzzer = 0;

		if (LowVoltage_1 == 1 || LowVoltage_2 == 1)
			Buzzer_count = 0;
		if (Buzzer_count < 5)
			Buzzer_count++;
		if (Buzzer_count < 5)
			Buzzer = 1; // The buzzer is buzzing //蜂鸣器蜂鸣
		else if (Buzzer_count == 5)
			Buzzer = 0;

		// Read the battery voltage //读取电池电压
		for (i = 0; i < 100; i++)
		{
			Voltage_All += Get_battery_volt();
		}
		Voltage = Voltage_All / 100;
		Voltage = VolMean_Filter(Voltage);
		Voltage_All = 0;

		if (Get_Charging_HardWare == 1)
		{ // 电压过低，收到红外信号，开启自动回充功能
			if (Voltage < 10 && RED_STATE)
				Allow_Recharge = 1;
		}

		if (LowVoltage_1 == 1)
			LowVoltage_1++; // Make sure the buzzer only rings for 0.5 seconds //确保蜂鸣器只响0.5秒
		if (LowVoltage_2 == 1)
			LowVoltage_2++; // Make sure the buzzer only rings for 0.5 seconds //确保蜂鸣器只响0.5秒
		if (Voltage >= 12.6f)
			Voltage = 12.6f;
		else if (10 <= Voltage && Voltage < 10.5f && LowVoltage_1 < 2)
			LowVoltage_1++; // 10.5V, first buzzer when low battery //10.5V，低电量时蜂鸣器第一次报警
		else if (Voltage < 10 && LowVoltage_1 < 2)
			LowVoltage_2++; // 10V, when the car is not allowed to control, the buzzer will alarm the second time //10V，小车禁止控制时蜂鸣器第二次报警

		APP_Show(); // Send data to the APP //向APP发送数据

		if (oled_refresh_flag)
			OLED_Clear(), oled_refresh_flag = 0;
		else
			oled_show(); // Tasks are displayed on the screen //显示屏显示任务
	}
}

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
函数功能：OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
extern volatile u8 clear_state;

uint8_t show_usb_enum = 0;
void OLED_ShowGamePadState(void)
{
	show_usb_enum = 1;
}

void oled_show(void)
{
	int Car_Mode_Show;

	// Collect the tap information of the potentiometer,
	// and display the car model to be fitted when the car starts up in real time
	// 采集电位器档位信息，实时显示小车开机时要适配的小车型号
	Divisor_Mode = 2048 / CAR_NUMBER + 5;
	POT_val = Get_adc_Average(Potentiometer, 10);
	Car_Mode_Show = (int)(POT_val / Divisor_Mode);
	if (Car_Mode_Show > 7)
		Car_Mode_Show = 7;
	// Car_Mode_Show=0;

	memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8)); // GRAM清零但不立即刷新，防止花屏
	if (Check == 0)
	{


		// 第一行：显示检测状态
		OLED_ShowString(0, 0, "Detected:");
		OLED_ShowNumber(70, 0, info.detected, 1, 12); // 显示检测状态（0或1）

		// 第二行：显示形状
		OLED_ShowString(0, 12, "Shape:");
		OLED_ShowString(52, 12, info.shape);

		// 第三行：显示颜色
		OLED_ShowString(0, 24, "Color:");
		OLED_ShowString(52, 24, info.color);

		// 第四行：显示尺寸
		OLED_ShowString(0, 36, "Size:");
		oled_showfloat(info.size, 60, 36, 1, 3); // 显示尺寸值
		OLED_ShowString(120, 36, "m");

		// 第五行：显示距离
		OLED_ShowString(0, 48, "Distance:");
		oled_showfloat(info.distance, 80, 48, 1, 3); // 显示距离值
		OLED_ShowString(120, 48, "m");

		OLED_Refresh_Gram(); // 刷新显示
	}
}
/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
int voltage_to_percentage(float voltage)
{
	const float V_MIN = 9.5f;  // 最小电压 (0%)
	const float V_MAX = 12.2f; // 最大电压 (100%)

	// 确保电压在有效范围内
	if (voltage < V_MIN)
		return 0;
	if (voltage > V_MAX)
		return 100;

	// 线性插值计算百分比并取整
	int percentage = (int)(((voltage - V_MIN) / (V_MAX - V_MIN)) * 100.0f + 0.5f); // 四舍五入

	if (percentage > 100)
		percentage = 100;
	if (percentage <= 0)
		percentage = 0;

	return percentage;
}

void APP_Show(void)
{
	// static u8 flag_show;
	// int Left_Figure, Right_Figure, Voltage_Show;

	// // The battery voltage is processed as a percentage
	// // 对电池电压处理成百分比形式
	// Voltage_Show = voltage_to_percentage(Voltage);
	// //	 Voltage_Show=(Voltage*1000-10000)/27;
	// //	 if(Voltage_Show>100)Voltage_Show=100;

	// // Wheel speed unit is converted to 0.01m/s for easy display in APP
	// // 车轮速度单位转换为0.01m/s，方便在APP显示
	// Left_Figure = MOTOR_A.Encoder * 100;
	// if (Left_Figure < 0)
	// 	Left_Figure = -Left_Figure;
	// Right_Figure = MOTOR_B.Encoder * 100;
	// if (Right_Figure < 0)
	// 	Right_Figure = -Right_Figure;

	// // Used to alternately print APP data and display waveform
	// // 用于交替打印APP数据和显示波形
	// flag_show = !flag_show;

	// if (PID_Send == 1)
	// {
	// 	// Send parameters to the APP, the APP is displayed in the debug screen
	// 	// 发送参数到APP，APP在调试界面显示
	// 	printf("{C%d:%d:%d:%d}$", (int)RC_Velocity, (int)Velocity_KP, (int)Velocity_KI, LineDiffParam);
	// 	PID_Send = 0;
	// }
	// else if (flag_show == 0)
	// {
	// 	// Send parameters to the APP and the APP will be displayed on the front page
	// 	// 发送参数到APP，APP在首页显示
	// 	printf("{A%d:%d:%d:%d}$", (u8)Left_Figure, (u8)Right_Figure, Voltage_Show, (int)imu.gyro.z);
	// }
	// else
	// {
	// 	// Send parameters to the APP, the APP is displayed in the waveform interface
	// 	// 发送参数到APP，APP在波形界面显示
	// 	printf("{B%d:%d:%d}$", (int)imu.gyro.x, (int)imu.gyro.y, (int)imu.gyro.z);
	// }
}

float base_vol = 11.5f;
#define VOL_COUNT 100
float VolMean_Filter(float data)
{
	u8 i;
	double Sum_Speed = 0;
	float Filter_Speed;
	static float Speed_Buf[VOL_COUNT] = {0};

	/*----------- 数组初始化 -----------*/
	static u8 once = 1;
	if (once)
	{
		once = 0;
		for (i = 0; i < VOL_COUNT; i++)
			Speed_Buf[i] = base_vol;
	}
	/*-------------------------------*/

	for (i = 1; i < VOL_COUNT; i++)
	{
		Speed_Buf[i - 1] = Speed_Buf[i];
	}
	Speed_Buf[VOL_COUNT - 1] = data;

	for (i = 0; i < VOL_COUNT; i++)
	{
		Sum_Speed += Speed_Buf[i];
	}
	Filter_Speed = (float)(Sum_Speed / VOL_COUNT);
	return Filter_Speed;
}
