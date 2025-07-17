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
�������ܣ���ȡ��ص�ѹ�������������������Լￄ1�7?��APP�������ݡ�OLED��ʾ����ʾ����
��ڲ������ￄ1�7?
����  ֵ����
**************************************************************************/
int Buzzer_count = 25;
void show_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		int i = 0;
		static int LowVoltage_1 = 0, LowVoltage_2 = 0;
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ)); // This task runs at 10Hz //��������10Hz��Ƶ������

		// ����ʱ���������ݷ�������������
		// The buzzer will beep briefly when the machine is switched on
		if (SysVal.Time_count < 50)
			Buzzer = 1;
		else if (SysVal.Time_count >= 51 && SysVal.Time_count < 100)
			Buzzer = 0;

		if (buzzer_mark == 1)
			Buzzer_count = 0;
		if (Buzzer_count < 8)
			Buzzer_count++;
		if (Buzzer_count < 8)
			Buzzer = 1; // The buzzer is buzzing //����������
		else if (Buzzer_count == 8)
			{
				Buzzer = 0;
				buzzer_mark = 0; // Reset the buzzer flag //����������
			}

		// Read the battery voltage //��ȡ��ص��?
		for (i = 0; i < 100; i++)
		{
			Voltage_All += Get_battery_volt();
		}
		Voltage = Voltage_All / 100;
		Voltage = VolMean_Filter(Voltage);
		Voltage_All = 0;

		if (Get_Charging_HardWare == 1)
		{ // ��ѹ���ͣ��յ������źţ������Զ��س书��
			if (Voltage < 10 && RED_STATE)
				Allow_Recharge = 1;
		}

		if (LowVoltage_1 == 1)
			LowVoltage_1++; // Make sure the buzzer only rings for 0.5 seconds //ȷ��������ֻ��0.5��
		if (LowVoltage_2 == 1)
			LowVoltage_2++; // Make sure the buzzer only rings for 0.5 seconds //ȷ��������ֻ��0.5��
		if (Voltage >= 12.6f)
			Voltage = 12.6f;
		else if (10 <= Voltage && Voltage < 10.5f && LowVoltage_1 < 2)
			LowVoltage_1++; // 10.5V, first buzzer when low battery //10.5V���͵���ʱ��������һ�α���
		else if (Voltage < 10 && LowVoltage_1 < 2)
			LowVoltage_2++; // 10V, when the car is not allowed to control, the buzzer will alarm the second time //10V��С����ֹ����ʱ�������ڶ��α���

		APP_Show(); // Send data to the APP //��APP��������

		if (oled_refresh_flag)
			OLED_Clear(), oled_refresh_flag = 0;
		else
			oled_show(); // Tasks are displayed on the screen //��ʾ����ʾ����
	}
}

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
�������ܣ�OLED��ʾ����ʾ����
��ڲ������ￄ1�7?
����  ֵ����
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
	// �ɼ���λ����λ��Ϣ��ʵʱ��ʾС������ʱҪ�����С���ͺￄ1�7?
	Divisor_Mode = 2048 / CAR_NUMBER + 5;
	POT_val = Get_adc_Average(Potentiometer, 10);
	Car_Mode_Show = (int)(POT_val / Divisor_Mode);
	if (Car_Mode_Show > 7)
		Car_Mode_Show = 7;
	// Car_Mode_Show=0;

	memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8)); // GRAM���㵫������ˢ�£���ֹ����
	if (Check == 0)
	{
		if (show_usb_enum == 1)
		{
			OLED_DrawBMP(22, 2, 22 + 84, 2 + 5, gImage_usb);
			static u8 show_delay = 0;
			if (++show_delay < RATE_10_HZ * 2)
			{
				return;
			}
			else
			{
				show_delay = 0;
				show_usb_enum = 0;
			}
		}
		//		if( show_usb_enum && GamePadDebug.enmu_state == EnumDone)
		//		{
		//			if(clear)
		//			{
		//				clear=0,oled_refresh_flag=1; //ö����,������ʾ������Ϣ
		//				return;//ִ��1������
		//			}
		//			OLED_DrawBMP(32,1,96,7,gImage_usb_bmp);
		//			OLED_ShowString(12,50,"USB Init..");
		//			OLED_Refresh_Line();
		//			show_done = 1;
		//			return;
		//		}
		//		else if( show_usb_enum && show_done )
		//		{
		//			if(show_done) //ö�ٳɹ�,��ʱ��ʾ1����ʾ����Ϣ
		//			{
		//				static u8 show_delay=0;
		//				if(++show_delay<RATE_10_HZ)
		//				{
		//					OLED_ShowString(12,50,"USB Init OK   ");
		//					OLED_Refresh_Line();
		//					return;
		//				}
		//				show_done = 0;
		//				show_delay=0;
		//				oled_refresh_flag = 1;//�ӳ���ￄ1�7?,������ʾ��������
		//				show_usb_enum = 0;
		//			}
		//			clear=1; //ö����ￄ1�7?,�ȴ��´�ö��ʱ��������
		//		}
		///////////// usb ps2 �豸������? /////////////

		if (oled_page == 0)
		{
			Voltage_Show = Voltage * 100;
			memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8)); // GRAM���㵫������ˢ�£���ֹ����
			if (Check == 0)								// The car displays normally when the self-check mode is not enabled //û�п����Լ�ģʽʱС��������ʾ
			{
				// The first line of the display displays the content//
				// ��ʾ����1����ʾ����//
				if (Allow_Recharge)
					OLED_ShowString(0, 0, "RCM ");
				else
				{
					switch (Car_Mode_Show)
					{
					case Mec_Car:
						OLED_ShowString(0, 0, "Mec  ");
						break;
					case Omni_Car:
						OLED_ShowString(0, 0, "Omni ");
						break;
					case Akm_Car:
						OLED_ShowString(0, 0, "Akm  ");
						break;
					case Diff_Car:
						OLED_ShowString(0, 0, "Diff ");
						break;
					case FourWheel_Car:
						OLED_ShowString(0, 0, "4WD  ");
						break;
					case Tank_Car:
						OLED_ShowString(0, 0, "Tank ");
						break;
					case Mec_Car_V550:
						OLED_ShowString(0, 0, "Mec-V");
						break;
					case FourWheel_Car_V550:
						OLED_ShowString(0, 0, "4WD-V");
						break;
					}
				}

				if (Car_Mode == Mec_Car || Car_Mode == Omni_Car || Car_Mode == Mec_Car_V550)
				{
					// The Mec_car and omni_car show Z-axis angular velocity
					// ���֡�ȫ����С����ʾZ����ٶￄ1�7?
					if (Charging)
					{ // С�����ʱ��ʾ�����ￄ1�7?
						OLED_ShowString(80, 0, " ");
						OLED_ShowString(85, 0, "   ");
						OLED_ShowString(60, 0, "Cur:");
						oled_showfloat(Charging_Current / 1000.0f, 90, 0, 1, 2);
					}
					else
					{ // С���ǳ��״�?��ʾZ����ٶￄ1�7?
						// Display Z-axis angular velocity //��ʾZ����ٶￄ1�7?
						OLED_ShowString(76, 0, "    ");
						OLED_ShowString(120, 0, " ");
						OLED_ShowString(60, 0, "GZ");
						if (yaw < 0)
							OLED_ShowString(80, 0, "-"), oled_showfloat(-yaw, 90, 0, 3, 2);
						else
							OLED_ShowString(80, 0, "+"), oled_showfloat(yaw, 90, 0, 3, 2);
					}
				}
				else if (Car_Mode == Akm_Car || Car_Mode == Diff_Car || Car_Mode == FourWheel_Car || Car_Mode == Tank_Car || Car_Mode == FourWheel_Car_V550)
				{
					// ps2�Ƿ�������?
					//				 if(usb_ps2_ready==1)
					//				 {
					//					OLED_ShowString(55,0,"ps2 ready  ");
					//				 }
					//				 else
					//				 {
					//					 OLED_ShowString(55,0,"ps2 outline");
					//				 }
					//
					// Akm_Car, Diff_Car, FourWheel_Car and Tank_Car Displays gyroscope zero
					// �����������١��������Ĵ�����ʾ��������ￄ1�7?
					OLED_ShowString(55, 0, "BIAS");
					if (imu.Deviation_accel.z < 0)
						OLED_ShowString(90, 0, "-"), OLED_ShowNumber(100, 0, -imu.Deviation_accel.z, 3, 12); // Zero-drift data of gyroscope Z axis
					else
						OLED_ShowString(90, 0, "+"), OLED_ShowNumber(100, 0, imu.Deviation_accel.z, 3, 12); // ������z������?������
				}
				// The first line of the display displays the content//
				// ��ʾ����1����ʾ����//

				// The second line of the display displays the content//
				// ��ʾ����2����ʾ����//
				if (Car_Mode == Mec_Car || Car_Mode == Omni_Car || Car_Mode == FourWheel_Car || Car_Mode == Mec_Car_V550 || Car_Mode == FourWheel_Car_V550)
				{
					// 	//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor A
					// 	//���֡�ȫ���֡���������ʾ���A��Ŀ���ٶȺ͵�ǰʵ���ٶ�
					// 	OLED_ShowString(0,10,"A");
					// 	if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
					// 												OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
					// 	else                 	OLED_ShowString(15,10,"+"),
					// 												OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12);

					// 	if( MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
					// 												OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
					// 	else                 	OLED_ShowString(60,10,"+"),
					// 												OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);
					//  }
					//  else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
					//  {
					// 	 //The Akm_Car, Diff_Car and Tank_Car show Z-axis angular velocity
					// 	 //�����������١�̹��С����ʾZ����ٶￄ1�7?
					// 	 OLED_ShowString(00,10,"GYRO_Z:");
					// 	 if( imu.gyro.z<0)  OLED_ShowString(60,10,"-"),
					// 									 OLED_ShowNumber(75,10,-imu.gyro.z,5,12);
					// 	 else            OLED_ShowString(60,10,"+"),
					// 	OLED_ShowNumber(75,10, imu.gyro.z,5,12);
					OLED_ShowNumber(20, 10, x1, 5, 12);
					OLED_ShowNumber(75, 10, y1, 5, 12); // ������ʾ
				}
				// The second line of the display displays the content//
				// ��ʾ����2����ʾ����//

				// Lines 3 and 4 of the display screen display content//
				// ��ʾ����3��4����ʾ����//
				if (Car_Mode == Mec_Car || Car_Mode == Omni_Car || Car_Mode == FourWheel_Car || Car_Mode == FourWheel_Car_V550 || Car_Mode == Mec_Car_V550)
				{
					// Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor B
					// ���֡�ȫ���֡���������ʾ���B��Ŀ���ٶȺ͵�ǰʵ���ٶ�
					OLED_ShowString(0, 20, "B");
					oled_showfloat(X_car, 20, 20, 1, 3);
					oled_showfloat(Y_car, 75, 20, 1, 3);
					// if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
					// 											OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
					// else                 	OLED_ShowString(15,20,"+"),
					// 											OLED_ShowNumber(20,20, MOTOR_B.Target*1000,5,12);

					// if( MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
					// 											OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
					// else                 	OLED_ShowString(60,20,"+"),
					// 											OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);

					// Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor C
					// ���֡�ȫ���֡���������ʾ���C��Ŀ���ٶȺ͵�ǰʵ���ٶ�
					
					oled_showfloat(X_enemy, 20, 30, 1, 3);
					oled_showfloat(Y_enemy, 75, 30, 1, 3);
					OLED_ShowString(0, 30, "C");
					// 	if( MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
					// 												OLED_ShowNumber(20,30,- MOTOR_C.Target*1000,5,12);
					// 	else                 	OLED_ShowString(15,30,"+"),
					// 												OLED_ShowNumber(20,30,  MOTOR_C.Target*1000,5,12);

					// 	if( MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
					// 												OLED_ShowNumber(75,30,-MOTOR_C.Encoder*1000,5,12);
					// 	else                 	OLED_ShowString(60,30,"+"),
					// 												OLED_ShowNumber(75,30, MOTOR_C.Encoder*1000,5,12);
				}
				else if (Car_Mode == Akm_Car || Car_Mode == Diff_Car || Car_Mode == Tank_Car)
				{
					// Akm_Car, Diff_Car and Tank_Car Display the target speed and current actual speed of motor A
					// �����������١��Ĵ�����ʾ���A��Ŀ���ٶȺ͵�ǰʵ���ٶ�
					
					OLED_ShowString(0, 20, "L:");
					//  if( MOTOR_A.Target<0)	OLED_ShowString(15,20,"-"),
					// 												OLED_ShowNumber(20,20,-MOTOR_A.Target*1000,5,12);
					//  else                 	OLED_ShowString(15,20,"+"),
					// 												OLED_ShowNumber(20,20, MOTOR_A.Target*1000,5,12);
					//  if( MOTOR_A.Encoder<0)	OLED_ShowString(60,20,"-"),
					// 												OLED_ShowNumber(75,20,-MOTOR_A.Encoder*1000,5,12);
					//  else                 	OLED_ShowString(60,20,"+"),
					// 												OLED_ShowNumber(75,20, MOTOR_A.Encoder*1000,5,12);
					// Akm_Car, Diff_Car and Tank_Car Display the target speed and current actual speed of motor B
					// �����������١��Ĵ�����ʾ���B��Ŀ���ٶȺ͵�ǰʵ���ٶ�
					OLED_ShowString(0, 30, "R:");
					//  if( MOTOR_B.Target<0)	OLED_ShowString(15,30,"-"),
					// 												OLED_ShowNumber(20,30,-MOTOR_B.Target*1000,5,12);
					//  else                 	OLED_ShowString(15,30,"+"),
					// 												OLED_ShowNumber(20,30,  MOTOR_B.Target*1000,5,12);

					//  if( MOTOR_B.Encoder<0)	OLED_ShowString(60,30,"-"),
					// 												OLED_ShowNumber(75,30,-MOTOR_B.Encoder*1000,5,12);
					//  else                 	OLED_ShowString(60,30,"+"),
					// 												OLED_ShowNumber(75,30, MOTOR_B.Encoder*1000,5,12);

					//			 if( Remoter_Ch1<0)	    OLED_ShowString(15,20,"-"),
					//															OLED_ShowNumber(20,20,-Remoter_Ch1,5,12);
					//			 else                 	OLED_ShowString(15,20,"+"),
					//															OLED_ShowNumber(20,20, Remoter_Ch1,5,12);
					//			 if( Remoter_Ch2<0)	    OLED_ShowString(60,20,"-"),
					//															OLED_ShowNumber(75,20,-Remoter_Ch2,5,12);
					//			 else                 	OLED_ShowString(60,20,"+"),
					//															OLED_ShowNumber(75,20, Remoter_Ch2,5,12);
					//			 if( Remoter_Ch3<0)	    OLED_ShowString(15,30,"-"),
					//															OLED_ShowNumber(20,30,-Remoter_Ch3,5,12);
					//			 else                 	OLED_ShowString(15,30,"+"),
					//															OLED_ShowNumber(20,30, Remoter_Ch3,5,12);
					//			 if( Remoter_Ch4<0)	    OLED_ShowString(60,30,"-"),
					//															OLED_ShowNumber(75,30,-Remoter_Ch4,5,12);
					//			 else                 	OLED_ShowString(60,30,"+"),
					//															OLED_ShowNumber(75,30, Remoter_Ch4,5,12);
				}
				// Lines 3 and 4 of the display screen display content//
				// ��ʾ����3��4����ʾ����//

				// Line 5 of the display displays the content//
				// ��ʾ����5����ʾ����//
				if (Car_Mode == Mec_Car || Car_Mode == FourWheel_Car || Car_Mode == Mec_Car_V550 || Car_Mode == FourWheel_Car_V550)
				{
					// Mec_Car Display the target speed and current actual speed of motor D
					// ����С����ʾ���D��Ŀ���ٶȺ͵�ǰʵ���ٶ�
					OLED_ShowString(0, 40, "D");
					OLED_ShowNumber(20, 40, Lock_count, 5, 12);
					// if (MOTOR_D.Target < 0)
					// 	OLED_ShowString(15, 40, "-"),
					// 		OLED_ShowNumber(20, 40, -MOTOR_D.Target * 1000, 5, 12);
					// else
					// 	OLED_ShowString(15, 40, "+"),
					// 		OLED_ShowNumber(20, 40, MOTOR_D.Target * 1000, 5, 12);
					// if (MOTOR_D.Encoder < 0)
					// 	OLED_ShowString(60, 40, "-"),
					// 		OLED_ShowNumber(75, 40, -MOTOR_D.Encoder * 1000, 5, 12);
					// else
					// 	OLED_ShowString(60, 40, "+"),
					// 		OLED_ShowNumber(75, 40, MOTOR_D.Encoder * 1000, 5, 12);
				}
				else if (Car_Mode == Omni_Car)
				{
					// The Omni_car shows Z-axis angular velocity (1000 times magnification) in rad/s
					// ȫ����С����ʾZ����ٶￄ1�7?(�Ŵ�1000��)����λrad/s
					OLED_ShowString(0, 40, "MOVE_Z");
					if (Send_Data.Sensor_Str.X_speed < 0)
						OLED_ShowString(60, 40, "-"),
							OLED_ShowNumber(75, 40, -Send_Data.Sensor_Str.X_speed, 5, 12);
					else
						OLED_ShowString(60, 40, "+"),
							OLED_ShowNumber(75, 40, Send_Data.Sensor_Str.X_speed, 5, 12);
				}
				else if (Car_Mode == Akm_Car)
				{
					// Akm_Car displays the PWM value of the Servo
					// ������С����ʾ�����PWM����ֵ
					OLED_ShowString(00, 40, "SERVO:");
					if (Servo < 0)
						OLED_ShowString(60, 40, "-"),
							OLED_ShowNumber(80, 40, -Servo, 4, 12);
					else
						OLED_ShowString(60, 40, "+"),
							OLED_ShowNumber(80, 40, Servo, 4, 12);
				}
				else if (Car_Mode == Diff_Car || Car_Mode == Tank_Car)
				{
					// The Diff_Car and Tank_Car displays the PWM values of the left and right motors
					// ����С�����Ĵ�����ʾ���ҵ����PWM����ֵ
					OLED_ShowString(00, 40, "MA");
					if (MOTOR_A.Motor_Pwm < 0)
						OLED_ShowString(20, 40, "-"),
							OLED_ShowNumber(30, 40, -MOTOR_A.Motor_Pwm, 4, 12);
					else
						OLED_ShowString(20, 40, "+"),
							OLED_ShowNumber(30, 40, MOTOR_A.Motor_Pwm, 4, 12);
					OLED_ShowString(60, 40, "MB");
					if (MOTOR_B.Motor_Pwm < 0)
						OLED_ShowString(80, 40, "-"),
							OLED_ShowNumber(90, 40, -MOTOR_B.Motor_Pwm, 4, 12);
					else
						OLED_ShowString(80, 40, "+"),
							OLED_ShowNumber(90, 40, MOTOR_B.Motor_Pwm, 4, 12);
				}
				// Line 5 of the display displays the content//
				// ��ʾ����5����ʾ����//

				// Displays the current control mode //��ʾ��ǰ����ģʽ
				if (PS2_ON_Flag == 1)
					OLED_ShowString(0, 50, "PS2  ");
				else if (APP_ON_Flag == 1)
					OLED_ShowString(0, 50, "APP  ");
				else if (Remote_ON_Flag == 1)
					OLED_ShowString(0, 50, "R-C  ");
				else if (CAN_ON_Flag == 1)
					OLED_ShowString(0, 50, "CAN  ");
				else if ((Usart1_ON_Flag || Usart5_ON_Flag) == 1)
					OLED_ShowString(0, 50, "USART");
				else if (nav_walk == 1)
					OLED_ShowString(0, 50, "nav");
				else
					OLED_ShowString(0, 50, "ROS  ");

				// Displays whether controls are allowed in the current car
				// ��ʾ��ǰС���Ƿ���������
				if (EN == 1 && Flag_Stop == 0)
					OLED_ShowString(45, 50, "O N");
				else
					OLED_ShowString(45, 50, "OFF");

				OLED_ShowNumber(75, 50, Voltage_Show / 100, 2, 12);
				OLED_ShowString(88, 50, ".");
				OLED_ShowNumber(98, 50, Voltage_Show % 100, 2, 12);
				OLED_ShowString(110, 50, "V");
				if (Voltage_Show % 100 < 10)
					OLED_ShowNumber(92, 50, 0, 2, 12);
			}

			OLED_Refresh_Gram();
		}

		else if (oled_page == 1)
		{
			// �Զ��س��׼�Debug��Ϣ
			OLED_ShowString(07, 00, "LA  LB  RB  RA");
			OLED_ShowNumber(0 + 9, 10, L_A, 1, 12);
			OLED_ShowNumber(30 + 9, 10, L_B, 1, 12);
			OLED_ShowNumber(60 + 9, 10, R_B, 1, 12);
			OLED_ShowNumber(90 + 9, 10, R_A, 1, 12);
			OLED_ShowString(0, 30, "cur:");
			OLED_ShowString(75, 30, "A");
			oled_showfloat(Charging_Current / 1000.0f, 30, 30, 2, 2);
			OLED_Refresh_Gram();
		}
	}
	else
	{
		if (Proc_Flag == 0) // �û��Լ���ￄ1�7?
		{
			OLED_ShowCHinese(00, 00, "�����Ķ�ʹ���ֲ�");
			OLED_ShowCHinese(00, 16, "  �����û�����  ");
			OLED_ShowCHinese(00, 32, "    ȷ�Ͽ�ʼ    ");
			OLED_ShowString(104, 50, "0/8");
		}

		if (Proc_Flag == 1)
		{
			OLED_Show_POT();
			OLED_ShowCHinese12(8, 9, "��ת����λ����");
			switch (Car_Mode_Show)
			{
			case Mec_Car:
				OLED_ShowString(92, 10, "Mec ");
				break;
			case Omni_Car:
				OLED_ShowString(92, 10, "Omni");
				break;
			case Akm_Car:
				OLED_ShowString(92, 10, "Akm ");
				break;
			case Diff_Car:
				OLED_ShowString(92, 10, "Diff");
				break;
			case FourWheel_Car:
				OLED_ShowString(92, 10, "4WD ");
				break;
			case Tank_Car:
				OLED_ShowString(92, 10, "Tank");
				break;
			case Mec_Car_V550:
				OLED_ShowString(92, 10, "MecV");
				break;
			case FourWheel_Car_V550:
				OLED_ShowString(92, 10, "4WDV");
				break;
			}
			// OLED_ShowCHinese12(12, 22, "�˳�ǰ������?����");
			// OLED_ShowCHinese(00, 34, "  ģʽ����λ��  ");
			OLED_ShowString(104, 50, "1/8");
		}
		if (Proc_Flag == 2)
		{
			OLED_ShowCHinese(24, 00, "�ܿճ��ֺ�");
			OLED_ShowCHinese(00, 16, "������ʼ����ת��");
			// OLED_ShowCHinese(24, 32, "ģʽ����ￄ1�7?");
			// OLED_ShowString(104, 50, "2/8");
		}
		if (Proc_Flag == 3)
		{
			if (MOTOR_A.Encoder < 0)
				OLED_ShowString(00, 5, "A-"),
					OLED_ShowNumber(20, 5, -MOTOR_A.Encoder * 1000, 5, 12);
			else
				OLED_ShowString(00, 5, "A+"),
					OLED_ShowNumber(20, 5, MOTOR_A.Encoder * 1000, 5, 12);

			if (MOTOR_B.Encoder < 0)
				OLED_ShowString(70, 5, "B-"),
					OLED_ShowNumber(90, 5, -MOTOR_B.Encoder * 1000, 5, 12);
			else
				OLED_ShowString(70, 5, "B+"),
					OLED_ShowNumber(90, 5, MOTOR_B.Encoder * 1000, 5, 12);

			if (MOTOR_C.Encoder < 0)
				OLED_ShowString(00, 20, "C-"),
					OLED_ShowNumber(20, 20, -MOTOR_C.Encoder * 1000, 5, 12);
			else
				OLED_ShowString(00, 20, "C+"),
					OLED_ShowNumber(20, 20, MOTOR_C.Encoder * 1000, 5, 12);

			if (MOTOR_D.Encoder < 0)
				OLED_ShowString(70, 20, "D-"),
					OLED_ShowNumber(90, 20, -MOTOR_D.Encoder * 1000, 5, 12);
			else
				OLED_ShowString(70, 20, "D+"),
					OLED_ShowNumber(90, 20, MOTOR_D.Encoder * 1000, 5, 12);
			// OLED_ShowCHinese(24, 32, "ģʽ����ￄ1�7?");
			// OLED_ShowString(104, 50, "2/8");
		}
		if (Proc_Flag == 4)
		{
			OLED_ShowCHinese(00, 00, "    ��������    ");
			OLED_ShowCHinese(00, 32, "  ģʽ��������  ");
			OLED_ShowString(104, 50, "3/8");
		}
		if (Proc_Flag == 5)
		{
			OLED_ShowCHinese(00, 00, "�ζ�С���۲�����");
			OLED_ShowString(00, 16, "G");
			if (imu.gyro.x < 0)
				OLED_ShowString(16, 16, "-"),
					OLED_ShowNumber(24, 16, -imu.gyro.x, 5, 12);
			else
				OLED_ShowString(16, 16, "+"),
					OLED_ShowNumber(24, 16, imu.gyro.x, 5, 12);

			if (imu.gyro.y < 0)
				OLED_ShowString(16, 26, "-"),
					OLED_ShowNumber(24, 26, -imu.gyro.y, 5, 12);
			else
				OLED_ShowString(16, 26, "+"),
					OLED_ShowNumber(24, 26, imu.gyro.y, 5, 12);

			if (imu.gyro.z < 0)
				OLED_ShowString(16, 36, "-"),
					OLED_ShowNumber(24, 36, -imu.gyro.z, 5, 12);
			else
				OLED_ShowString(16, 36, "+"),
					OLED_ShowNumber(24, 36, imu.gyro.z, 5, 12);

			OLED_ShowString(64, 16, "|");
			OLED_ShowString(64, 26, "|");
			OLED_ShowString(64, 36, "|");

			OLED_ShowString(72, 16, "A");
			if (imu.accel.x < 0)
				OLED_ShowString(80, 16, "-"),
					OLED_ShowNumber(88, 16, -imu.accel.x, 5, 12);
			else
				OLED_ShowString(80, 16, "+"),
					OLED_ShowNumber(88, 16, imu.accel.x, 5, 12);

			if (imu.accel.y < 0)
				OLED_ShowString(80, 26, "-"),
					OLED_ShowNumber(88, 26, -imu.accel.y, 5, 12);
			else
				OLED_ShowString(80, 26, "+"),
					OLED_ShowNumber(88, 26, imu.accel.y, 5, 12);

			if (imu.accel.z < 0)
				OLED_ShowString(80, 36, "-"),
					OLED_ShowNumber(88, 36, -imu.accel.z, 5, 12);
			else
				OLED_ShowString(80, 36, "+"),
					OLED_ShowNumber(88, 36, imu.accel.z, 5, 12);

			OLED_ShowString(104, 50, "3/8");
		}
		if (Proc_Flag == 6)
		{
			OLED_ShowCHinese(00, 00, "    ��������    ");
			// OLED_ShowCHinese(8, 32, "ģʽ����·��ￄ1�7?");
			// OLED_ShowString(104, 50, "4/8");
		}
		if (Proc_Flag == 7 || Proc_Flag == 8 || Proc_Flag == 9 || Proc_Flag == 10 || Proc_Flag == 11 || Proc_Flag == 12)
		{
			OLED_ShowString(00, 00, "1:"), OLED_ShowNumber(16, 00, Servo_Count[0], 5, 12);
			OLED_ShowString(64, 00, "2:"), OLED_ShowNumber(80, 00, Servo_Count[1], 5, 12);
			OLED_ShowString(00, 10, "3:"), OLED_ShowNumber(16, 10, Servo_Count[2], 5, 12);
			OLED_ShowString(64, 10, "4:"), OLED_ShowNumber(80, 10, Servo_Count[3], 5, 12);
			OLED_ShowString(00, 20, "5:"), OLED_ShowNumber(16, 20, Servo_Count[4], 5, 12);
			OLED_ShowString(64, 20, "6:"), OLED_ShowNumber(80, 20, Servo_Count[5], 5, 12);
			// OLED_ShowCHinese(8, 32, "ģʽ����·��ￄ1�7?");
			// OLED_ShowString(104, 50, "4/8");
		}
		if (Proc_Flag == 13)
		{
			OLED_ShowCHinese(00, 00, "    ��������    ");
			OLED_ShowCHinese(00, 32, "  ģʽ��������  ");
			OLED_ShowString(104, 50, "5/8");
		}
		if (Proc_Flag == 14)
		{
			OLED_ShowCHinese(24, 16, "��������");
			if (Buzzer == 1)
				OLED_ShowCHinese(88, 16, "��");
			else
				OLED_ShowCHinese(88, 16, "��");
			OLED_ShowCHinese(00, 32, "  ģʽ��������  ");
			OLED_ShowString(104, 50, "5/8");
		}
		if (Proc_Flag == 15)
		{
			OLED_ShowCHinese(00, 00, "  ����ʹ�ܿ���  ");
			OLED_ShowCHinese(32, 16, "���أ�");
			if (EN == 1)
				OLED_ShowCHinese(80, 16, "��");
			else
				OLED_ShowCHinese(80, 16, "��");
			OLED_ShowCHinese(8, 32, "ģʽ��ʹ�ܿ���");
			OLED_ShowString(104, 50, "6/8");
		}
		if (Proc_Flag == 16)
		{
			// OLED_ShowCHinese(00, 00, "ȷ��������Ƴ��ￄ1�7?");
			// OLED_ShowCHinese(24, 16, "�󵥻�����");
			OLED_ShowCHinese(24, 32, "ģʽ������");
			OLED_ShowString(104, 50, "7/8");
		}
		if (Proc_Flag == 17)
		{
			OLED_ShowCHinese(8, 00, "С���ж�״̬��");
			if (uart2_send_flag == 1)
				OLED_ShowCHinese(48, 16, "��ǰ");
			else if (uart2_send_flag == 2)
				OLED_ShowCHinese(48, 16, "����");
			else if (uart2_send_flag == 3)
			// 	OLED_ShowCHinese(48, 16, "��ￄ1�7?");
			// else if (uart2_send_flag == 4)
				OLED_ShowCHinese(48, 16, "����");
			else if (uart2_send_flag == 5)
				OLED_ShowCHinese(48, 16, "ֹͣ");
			//			 OLED_ShowCHinese(00, 00, "��������");
			//			 OLED_ShowString16(64, 00, "WHEELTEC");
			//			 OLED_ShowCHinese(00, 16, "���յ�");
			//			 OLED_ShowString16(48, 16, "ASCII:");
			//			 OLED_ShowNumber(96, 18, (USART2->DR), 4, 12);		//��APP���͵����ַ�����������ʹ��Ascii����ʾ
			OLED_ShowCHinese(24, 32, "ģʽ������");
			OLED_ShowString(104, 50, "7/8");
		}
		if (Proc_Flag == 18)
		{
			OLED_ShowCHinese(8, 00, "ʹ������������");
			// OLED_ShowCHinese(20, 16, "�м�ￄ1�7?");
			// OLED_ShowString16(68, 16, "USB");
			OLED_ShowCHinese(92, 16, "��");
			OLED_ShowCHinese(20, 32, "ģʽ������");
			OLED_ShowString16(100, 32, "3");
			OLED_ShowString(104, 50, "8/8");
		}
		if (Proc_Flag == 19)
		{
			OLED_ShowCHinese(00, 00, "����");
			OLED_ShowString16(36, 00, "#");
			OLED_ShowCHinese(48, 00, "��β������");

			OLED_ShowCHinese(8, 16, "�����ʣ�");
			OLED_ShowString16(72, 16, "115200");
			//			 OLED_ShowCHinese(00, 00, "��������");
			//			 OLED_ShowString16(64, 00, "WHEELTEC");
			//			 OLED_ShowCHinese(00, 16, "���յ�");
			//			 OLED_ShowString16(48, 16, "ASCII:");
			//			 OLED_ShowNumber(96, 18, (USART3->DR), 4, 12);		//�ڴ��������Ϸ��͵����ַ�����������ʹ��Ascii����ʾ
			OLED_ShowCHinese(20, 32, "ģʽ������");
			OLED_ShowString16(100, 32, "3");
			OLED_ShowString(104, 50, "8/8");
		}
		if (Proc_Flag == 20)
		{
			OLED_ShowCHinese(00, 00, "�����˳��ʼ�ģʽ");
			OLED_ShowCHinese(00, 16, "��ת����λ���ص�");
			OLED_ShowCHinese(12, 32, "��ǰ���ͣ�");
			switch (Car_Mode_Show)
			{
			case Mec_Car:
				OLED_ShowString16(92, 32, "Mec ");
				break;
			case Omni_Car:
				OLED_ShowString16(92, 32, "Omni");
				break;
			case Akm_Car:
				OLED_ShowString16(92, 32, "Akm ");
				break;
			case Diff_Car:
				OLED_ShowString16(92, 32, "Diff");
				break;
			case FourWheel_Car:
				OLED_ShowString16(92, 32, "4WD ");
				break;
			case Tank_Car:
				OLED_ShowString16(92, 32, "Tank");
				break;
			case Mec_Car_V550:
				OLED_ShowString(92, 32, "MecV");
				break;
			case FourWheel_Car_V550:
				OLED_ShowString(92, 32, "4WDV");
				break;
			}
			OLED_ShowString(104, 50, "8/8");
		}
		OLED_ShowCHinese12(00, 50, "��������˫���˳�");
		OLED_Refresh_Gram();
	}
}
/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
�������ܣ���APP��������
��ڲ������ￄ1�7?
����  ֵ����
**************************************************************************/
int voltage_to_percentage(float voltage)
{
	const float V_MIN = 9.5f;  // ��С��ѹ (0%)
	const float V_MAX = 12.2f; // ����ѹ (100%)

	// ȷ����ѹ����Ч��Χ��
	if (voltage < V_MIN)
		return 0;
	if (voltage > V_MAX)
		return 100;

	// ���Բ�ֵ����ٷֱȲ�ȡ�ￄ1�7?
	int percentage = (int)(((voltage - V_MIN) / (V_MAX - V_MIN)) * 100.0f + 0.5f); // ��������

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
	// // �Ե�ص�ѹ�����ɰٷֱ����?
	// Voltage_Show = voltage_to_percentage(Voltage);
	// //	 Voltage_Show=(Voltage*1000-10000)/27;
	// //	 if(Voltage_Show>100)Voltage_Show=100;

	// // Wheel speed unit is converted to 0.01m/s for easy display in APP
	// // �����ٶȵ�λת��Ϊ0.01m/s��������APP��ʾ
	// Left_Figure = MOTOR_A.Encoder * 100;
	// if (Left_Figure < 0)
	// 	Left_Figure = -Left_Figure;
	// Right_Figure = MOTOR_B.Encoder * 100;
	// if (Right_Figure < 0)
	// 	Right_Figure = -Right_Figure;

	// // Used to alternately print APP data and display waveform
	// // ���ڽ�����?APP���ݺ���ʾ����
	// flag_show = !flag_show;

	// if (PID_Send == 1)
	// {
	// 	// Send parameters to the APP, the APP is displayed in the debug screen
	// 	// ���Ͳ�����APP��APP�ڵ��Խ�����ʾ
	// 	printf("{C%d:%d:%d:%d}$", (int)RC_Velocity, (int)Velocity_KP, (int)Velocity_KI, LineDiffParam);
	// 	PID_Send = 0;
	// }
	// else if (flag_show == 0)
	// {
	// 	// Send parameters to the APP and the APP will be displayed on the front page
	// 	// ���Ͳ�����APP��APP����ҳ��ʾ
	// 	printf("{A%d:%d:%d:%d}$", (u8)Left_Figure, (u8)Right_Figure, Voltage_Show, (int)imu.gyro.z);
	// }
	// else
	// {
	// 	// Send parameters to the APP, the APP is displayed in the waveform interface
	// 	// ���Ͳ�����APP��APP�ڲ��ν�����ʾ
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

	/*----------- �����ʼ�ￄ1�7? -----------*/
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
