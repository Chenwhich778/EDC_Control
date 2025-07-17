#include "usartx.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
SEND_AutoCharge_DATA Send_AutoCharge_Data;

// 定义环形缓冲区和溢出计数器
RingBuffer USART1_RxBuffer = {0};
RingBuffer USART3_RxBuffer = {0};
volatile uint32_t usart1_overflow_count = 0;
volatile uint32_t usart3_overflow_count = 0;

uint8_t detect = 0; // 用于检测是否有数据到来

int x1 = -1, y1 = -1, x2 = -1, y2 = -1, x3 = -1, y3 = -1, x4 = -1, y4 = -1;

int Center_x = 371, Center_y = 257;

void parsePositions(const char *str, float *x_car, float *y_car, float *x_enemy, float *y_enemy);
void parseCoordinates(const char *str, int *x1, int *y1, int *x2, int *y2,
					  int *x3, int *y3, int *x4, int *y4);

// 初始化环形缓冲区
void RingBuffer_Init(RingBuffer *rb)
{
	rb->head = 0;
	rb->tail = 0;
	rb->frame_ready = 0;
}

// 向缓冲区添加数据，带溢出检测
uint8_t RingBuffer_Put(RingBuffer *rb, uint8_t data)
{
	uint16_t next_head = (rb->head + 1) % RING_BUF_SIZE;
	if (next_head == rb->tail)
	{
		if (rb == &USART1_RxBuffer)
			usart1_overflow_count++;
		else if (rb == &USART3_RxBuffer)
			usart3_overflow_count++;
		return 0; // 缓冲区满
	}
	rb->buffer[rb->head] = data;
	rb->head = next_head;
	return 1;
}

// 从缓冲区获取数据
uint8_t RingBuffer_Get(RingBuffer *rb, uint8_t *data)
{
	if (rb->tail == rb->head)
		return 0; // 缓冲区空

	*data = rb->buffer[rb->tail];
	rb->tail = (rb->tail + 1) % RING_BUF_SIZE;
	return 1;
}

// 获取可用数据量
uint16_t RingBuffer_Available(RingBuffer *rb)
{
	if (rb->head >= rb->tail)
		return rb->head - rb->tail;
	return RING_BUF_SIZE - rb->tail + rb->head;
}

// 检查缓冲区是否为空
uint8_t RingBuffer_IsEmpty(RingBuffer *rb)
{
	return rb->head == rb->tail;
}
// 新增环形缓冲区

// USART1 帧处理函数（已禁用）
void USART1_ProcessFrame(void)
{
	// 串口1接收功能已禁用，保留空实现用于兼容
}

// USART3 帧处理函数（可同时解析两种数据格式）
void USART3_ProcessFrame(void)
{
	if (!USART3_RxBuffer.frame_ready)
		return;
	uint8_t data;
#define FRAME_MAX_LEN 128
	char frame[FRAME_MAX_LEN];
	uint16_t index = 0;
	while (RingBuffer_Get(&USART3_RxBuffer, &data) && index < FRAME_MAX_LEN - 1)
	{
		if (data == '\n')
		{
			frame[index] = '\0';
			break;
		}
		if (data != '\r')
			frame[index++] = data;
	}
	frame[FRAME_MAX_LEN - 1] = '\0';
	USART3_RxBuffer.frame_ready = 0;

	// 只处理带#或!标志的数据
	if (frame[0] == '#')
	{
		// 解析float型，赋值给X_car/Y_car/X_enemy/Y_enemy

		parsePositions(frame + 1, &X_car, &Y_car, &X_enemy, &Y_enemy);
	}
	else if (frame[0] == '!')
	{

		parseCoordinates(frame + 1, &x1, &y1, &x2, &y2, &x3, &y3, &x4, &y4);
		if (x1 == 0 && y1 == 0)
		{
			Center_x = 371; // 重置中心点
			Center_y = 257; // 重置中心点
		}
		else
		{
			Center_x = (x1 + x2 + x3 + x4) / 4; // 计算中心点
			Center_y = (y1 + y2 + y3 + y4) / 4; // 计算中心点
			// detect = 1; // 设置检测标志位
		}
	}
	// 其他情况不赋值
}

float X_car = 3.8, Y_car = 0;
float X_enemy = 0, Y_enemy = -1;
char position_str[BUF_SIZE] = {0};

/**************************************************************************
Function: Usartx3, Usartx1,Usartx5 and CAN send data task
Input   : none
Output  : none
函数功能：串口3、串口1、串口5、CAN发送数据任务
入口参数：无
返回  值：无
**************************************************************************/
void data_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();

	// 初始化环形缓冲区
	RingBuffer_Init(&USART1_RxBuffer);
	RingBuffer_Init(&USART3_RxBuffer);

	while (1)
	{
		// The task is run at 20hz
		// 此任务以100Hz的频率运行
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));
		if (Check == 0)
		{
			// Assign the data to be sent
			// 对要进行发送的数据进行赋值
			data_transition();
			USART1_SEND(); // Serial port 1 sends data //串口1发送数据
			USART3_SEND(); // Serial port 3 (ROS) sends data  //串口3(ROS)发送数据
			// USART5_SEND();		 //Serial port 5 sends data //串口5发送数据
			USART6_SEND(); // everloss
			// CAN_SEND();        //CAN send data //CAN发送数据

			// 处理接收到的数据帧
			USART1_ProcessFrame();
			USART3_ProcessFrame();
		}
	}
}
/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; // Frame_header //帧头
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;	  // Frame_tail //帧尾

	// According to different vehicle types, different kinematics algorithms were selected to carry out the forward kinematics solution,
	// and the three-axis velocity was obtained from each wheel velocity
	// 根据不同车型选择不同运动学算法进行运动学正解，从各车轮速度求出三轴速度
	switch (Car_Mode)
	{
	case Mec_Car:
	case Mec_Car_V550:
		Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder + MOTOR_B.Encoder + MOTOR_C.Encoder + MOTOR_D.Encoder) / 4) * 1000;
		Send_Data.Sensor_Str.Y_speed = ((MOTOR_A.Encoder - MOTOR_B.Encoder + MOTOR_C.Encoder - MOTOR_D.Encoder) / 4) * 1000;
		Send_Data.Sensor_Str.Z_speed = ((-MOTOR_A.Encoder - MOTOR_B.Encoder + MOTOR_C.Encoder + MOTOR_D.Encoder) / 4 / (Axle_spacing + Wheel_spacing)) * 1000;
		break;

	case Omni_Car:
		Send_Data.Sensor_Str.X_speed = ((MOTOR_C.Encoder - MOTOR_B.Encoder) / 2 / X_PARAMETER) * 1000;
		Send_Data.Sensor_Str.Y_speed = ((MOTOR_A.Encoder * 2 - MOTOR_B.Encoder - MOTOR_C.Encoder) / 3) * 1000;
		Send_Data.Sensor_Str.Z_speed = ((MOTOR_A.Encoder + MOTOR_B.Encoder + MOTOR_C.Encoder) / 3 / Omni_turn_radiaus) * 1000;
		break;

	case Akm_Car:
		Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder + MOTOR_B.Encoder) / 2) * 1000;
		Send_Data.Sensor_Str.Y_speed = 0;
		Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder - MOTOR_A.Encoder) / Wheel_spacing) * 1000;
		break;

	case Diff_Car:
		Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder + MOTOR_B.Encoder) / 2) * 1000;
		Send_Data.Sensor_Str.Y_speed = 0;
		Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder - MOTOR_A.Encoder) / Wheel_spacing) * 1000;
		break;

	case FourWheel_Car:
	case FourWheel_Car_V550:
		Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder + MOTOR_B.Encoder + MOTOR_C.Encoder + MOTOR_D.Encoder) / 4) * 1000;
		Send_Data.Sensor_Str.Y_speed = 0;
		Send_Data.Sensor_Str.Z_speed = ((-MOTOR_B.Encoder - MOTOR_A.Encoder + MOTOR_C.Encoder + MOTOR_D.Encoder) / 2 / (Axle_spacing + Wheel_spacing)) * 1000;
		break;

	case Tank_Car:
		Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder + MOTOR_B.Encoder) / 2) * 1000;
		Send_Data.Sensor_Str.Y_speed = 0;
		Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder - MOTOR_A.Encoder) / (Wheel_spacing) * 1000);
		break;
	}

	// The acceleration of the triaxial acceleration //加速度计三轴加速度
	Send_Data.Sensor_Str.Accelerometer.X_data = imu.accel.y;  // The accelerometer Y-axis is converted to the ros coordinate X axis //加速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Accelerometer.Y_data = -imu.accel.x; // The accelerometer X-axis is converted to the ros coordinate y axis //加速度计X轴转换到ROS坐标Y轴
	Send_Data.Sensor_Str.Accelerometer.Z_data = imu.accel.z;  // The accelerometer Z-axis is converted to the ros coordinate Z axis //加速度计Z轴转换到ROS坐标Z轴

	// The Angle velocity of the triaxial velocity //角速度计三轴角速度
	Send_Data.Sensor_Str.Gyroscope.X_data = imu.gyro.y;	 // The Y-axis is converted to the ros coordinate X axis //角速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Gyroscope.Y_data = -imu.gyro.x; // The X-axis is converted to the ros coordinate y axis //角速度计X轴转换到ROS坐标Y轴
	if (Flag_Stop == 0)
		// If the motor control bit makes energy state, the z-axis velocity is sent normall
		// 如果电机控制位使能状态，那么正常发送Z轴角速度
		Send_Data.Sensor_Str.Gyroscope.Z_data = imu.gyro.z;
	else
		// If the robot is static (motor control dislocation), the z-axis is 0
		// 如果机器人是静止的（电机控制位失能），那么发送的Z轴角速度为0
		Send_Data.Sensor_Str.Gyroscope.Z_data = 0;

	// Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
	// 电池电压(这里将浮点数放大一千倍传输，相应的在接收端在接收到数据后也会缩小一千倍)
	Send_Data.Sensor_Str.Power_Voltage = Voltage * 1000;

	Send_Data.buffer[0] = Send_Data.Sensor_Str.Frame_Header; // Frame_heade //帧头
	Send_Data.buffer[1] = Flag_Stop;						 // Car software loss marker //小车软件失能标志位

	// The three-axis speed of / / car is split into two eight digit Numbers
	// 小车三轴速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[2] = Send_Data.Sensor_Str.X_speed >> 8;
	Send_Data.buffer[3] = Send_Data.Sensor_Str.X_speed;
	Send_Data.buffer[4] = Send_Data.Sensor_Str.Y_speed >> 8;
	Send_Data.buffer[5] = Send_Data.Sensor_Str.Y_speed;
	Send_Data.buffer[6] = Send_Data.Sensor_Str.Z_speed >> 8;
	Send_Data.buffer[7] = Send_Data.Sensor_Str.Z_speed;

	// The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
	// IMU加速度计三轴加速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[8] = Send_Data.Sensor_Str.Accelerometer.X_data >> 8;
	Send_Data.buffer[9] = Send_Data.Sensor_Str.Accelerometer.X_data;
	Send_Data.buffer[10] = Send_Data.Sensor_Str.Accelerometer.Y_data >> 8;
	Send_Data.buffer[11] = Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12] = Send_Data.Sensor_Str.Accelerometer.Z_data >> 8;
	Send_Data.buffer[13] = Send_Data.Sensor_Str.Accelerometer.Z_data;

	// The axis of the triaxial velocity of the / /imu is divided into two eight digits
	// IMU角速度计三轴角速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[14] = Send_Data.Sensor_Str.Gyroscope.X_data >> 8;
	Send_Data.buffer[15] = Send_Data.Sensor_Str.Gyroscope.X_data;
	Send_Data.buffer[16] = Send_Data.Sensor_Str.Gyroscope.Y_data >> 8;
	Send_Data.buffer[17] = Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18] = Send_Data.Sensor_Str.Gyroscope.Z_data >> 8;
	Send_Data.buffer[19] = Send_Data.Sensor_Str.Gyroscope.Z_data;

	// Battery voltage, split into two 8 digit Numbers
	// 电池电压,拆分为两个8位数据发送
	Send_Data.buffer[20] = Send_Data.Sensor_Str.Power_Voltage >> 8;
	Send_Data.buffer[21] = Send_Data.Sensor_Str.Power_Voltage;

	// Data check digit calculation, Pattern 1 is a data check
	// 数据校验位计算，模式1是发送数据校验
	Send_Data.buffer[22] = Check_Sum(22, 1);

	Send_Data.buffer[23] = Send_Data.Sensor_Str.Frame_Tail; // Frame_tail //帧尾

	///////////////////////自动回充相关变量赋值/////////////////////
	Send_AutoCharge_Data.AutoCharge_Str.Frame_Header = AutoCharge_HEADER;			// 帧头赋值0x7C
	Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail = AutoCharge_TAIL;				// 帧尾赋值0x7F
	Send_AutoCharge_Data.AutoCharge_Str.Charging_Current = (short)Charging_Current; // 充电电流赋值

	Send_AutoCharge_Data.AutoCharge_Str.RED = RED_STATE;	 // 红外标志位赋值
	Send_AutoCharge_Data.AutoCharge_Str.Charging = Charging; // 是否在充电标志位赋值

	Send_AutoCharge_Data.buffer[0] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Header;			// 帧头0x7C
	Send_AutoCharge_Data.buffer[1] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current >> 8; // 充电电流高8位
	Send_AutoCharge_Data.buffer[2] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current;		// 充电电流低8位
	Send_AutoCharge_Data.buffer[3] = Send_AutoCharge_Data.AutoCharge_Str.RED;					// 是否接收到红外标志位
	Send_AutoCharge_Data.buffer[4] = Send_AutoCharge_Data.AutoCharge_Str.Charging;				// 是否在充电标志位
	Send_AutoCharge_Data.buffer[5] = Allow_Recharge;											// 是否开启自动回充
	Send_AutoCharge_Data.buffer[6] = Check_Sum_AutoCharge(6, 1);								// 校验位
	Send_AutoCharge_Data.buffer[7] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail;			// 帧尾0x7F
	///////////////////////自动回充相关变量赋值/////////////////////
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
函数功能：串口1发送数据
入口参数：无
返回  值：无
**************************************************************************/
uint16_t turn_time = 1;
void USART1_SEND(void)
{
	unsigned char i = 0;
	//
	//	for(i=0; i<24; i++)
	//	{
	//		usart1_send(Send_Data.buffer[i]);
	//	}
	//	if(Get_Charging_HardWare==1)
	//	{
	//		//存在回充装备时，向上层发送自动回充相关变量
	//		for(i=0; i<8; i++)
	//		{
	//			usart1_send(Send_AutoCharge_Data.buffer[i]);
	//		}
	//	}

	// 直接发送字符串和数值，无需snprintf
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : none
Output  : none
函数功能：串口3发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART3_SEND(void)
{
}
void USART3_Return(void)
{
	// for(int i=0; i<message_count; i++)
	// {
	// 	usart3_send(uart3_receive_message[i]);
	// }
	// usart3_send('\r');
	// usart3_send('\n');
}
void USART2_Return(void)
{
	// printf("{#");
	// for(int i=0; i<app_count; i++)
	// {
	// 	printf("%c",uart2_receive_message[i]);
	// }
	// printf("}$");
	// printf("\r\n");
}
/**************************************************************************
Function: Serial port 5 sends data
Input   : none
Output  : none
函数功能：串口5发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART5_SEND(void)
{
	//   unsigned char i = 0;
	// 	for(i=0; i<24; i++)
	// 	{
	// 		usart5_send(Send_Data.buffer[i]);
	// 	}
	// 	if(Get_Charging_HardWare==1)
	// 	{
	// 		//存在回充装备时，向上层发送自动回充相关变量
	// 		for(i=0; i<8; i++)
	// 		{
	// 			usart5_send(Send_AutoCharge_Data.buffer[i]);
	// 		}
	// 	}
}
/**************************************************************************
Function: Serial port 5 sends data
Input   : none
Output  : none
函数功能：串口5发送数据
入口参数：无
返回  值：无
**************************************************************************/
// int Radar_aiming_counter = 5000;
void USART6_SEND(void)
{
	// Radar_aiming_counter--;
	
		// 总是尝试雷达预瞄准（函数内部会判断是否需要执行）
    radar_pre_aim(X_enemy, Y_enemy);
    
    // 只有在摄像头模式下才执行追踪
    if (system_mode == MODE_CAMERA_TRACKING) {
        control_camera(Center_x, Center_y);
    }
}

//  unsigned char i = 0;
// double xSpeed = Send_Data.Sensor_Str.X_speed / 100.0;
// double ySpeed = Send_Data.Sensor_Str.Y_speed / 100.0;
// double zSpeed = Send_Data.Sensor_Str.Z_speed / 100.0;

// double xAccel = Send_Data.Sensor_Str.Accelerometer.X_data / 100.0;
// double yAccel = Send_Data.Sensor_Str.Accelerometer.Y_data / 100.0;
// double zAccel = Send_Data.Sensor_Str.Accelerometer.Z_data / 100.0;

// double xGyro = Send_Data.Sensor_Str.Gyroscope.X_data / 100.0;
// double yGyro = Send_Data.Sensor_Str.Gyroscope.Y_data / 100.0;
// double zGyro = Send_Data.Sensor_Str.Gyroscope.Z_data / 100.0;

// char tempStr[128];
// snprintf(tempStr, sizeof(tempStr),
//         "v:%.1f,%.1f,%.1f,a:%.1f,%.1f,%.1f,g:%.1f,%.1f,%.1f\r\n",
//         xSpeed, ySpeed, zSpeed,
//         xAccel, yAccel, zAccel,
//         xGyro, yGyro, zGyro
//);
// for(i = 0; tempStr[i] != '\0'; i++)
//     {
//         usart6_send(tempStr[i]);
//     }
/**************************************************************************
Function: CAN sends data
Input   : none
Output  : none
函数功能：CAN发送数据
入口参数：无
返 回 值：无
**************************************************************************/
void CAN_SEND(void)
{
	// u8 CAN_SENT[8],i;

	// for(i=0;i<8;i++)
	// {
	//   CAN_SENT[i]=Send_Data.buffer[i];
	// }
	// CAN1_Send_Num(0x101,CAN_SENT);

	// for(i=0;i<8;i++)
	// {
	//   CAN_SENT[i]=Send_Data.buffer[i+8];
	// }
	// CAN1_Send_Num(0x102,CAN_SENT);

	// for(i=0;i<8;i++)
	// {
	//   CAN_SENT[i]=Send_Data.buffer[i+16];
	// }
	// CAN1_Send_Num(0x103,CAN_SENT);

	// //////////////////自动回充相关数据发送//////////////////
	// if(Get_Charging_HardWare) CAN_Send_AutoRecharge();
}
/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
函数功能：串口1初始化
入口参数：无
返 回 值：无
**************************************************************************/
void uart1_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // Enable the gpio clock //使能GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // Enable the Usart clock //使能USART时钟

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	  // 输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	  // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 高速50MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	  // 上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);			  // 初始化

	// UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	// Preempt priority //抢占优先级（数值越大优先级越低，建议6）
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	// Subpriority //子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	// Enable the IRQ channel //IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// Initialize the VIC register with the specified parameters
	// 根据指定的参数初始化VIC寄存器
	NVIC_Init(&NVIC_InitStructure);

	// USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound;										// Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// A stop bit //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// Sending and receiving mode //收发模式
	USART_Init(USART1, &USART_InitStructure);										// Initialize serial port 1 //初始化串口1

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(USART1, ENABLE);					   // Enable serial port 1 //使能串口1

	// 在初始化末尾添加缓冲区初始化
	RingBuffer_Init(&USART1_RxBuffer); // everloss//
}
/**************************************************************************
Function: Serial port 2 initialization
Input   : none
Output  : none
函数功能：串口2初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart2_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);  // Enable the gpio clock  //使能GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // Enable the Usart clock //使能USART时钟

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	  // 输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	  // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 高速50MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	  // 上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);			  // 初始化

	// UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	// Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	// Subpriority //子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	// Enable the IRQ channel //IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// Initialize the VIC register with the specified parameters
	// 根据指定的参数初始化VIC寄存器
	NVIC_Init(&NVIC_InitStructure);

	// USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound;										// Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No;								// Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// Sending and receiving mode //收发模式
	USART_Init(USART2, &USART_InitStructure);										// Initialize serial port 2 //初始化串口2

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(USART2, ENABLE);					   // Enable serial port 2 //使能串口2
}
/**************************************************************************
Function: Serial port 3 initialization
Input   : none
Output  : none
函数功能：串口3初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart3_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);  // Enable the gpio clock  //使能GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // Enable the Usart clock //使能USART时钟

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	  // 输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	  // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 高速50MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	  // 上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);			  // 初始化

	// UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	// Preempt priority //抢占优先级（数值越大优先级越低，建议6）
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	// Subpriority //子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	// Enable the IRQ channel //IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// Initialize the VIC register with the specified参数
	// 根据指定的参数初始化VIC寄存器
	NVIC_Init(&NVIC_InitStructure);

	// USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound;										// Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No;								// Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// Sending and receiving mode //收发模式
	USART_Init(USART3, &USART_InitStructure);										// Initialize serial port 3 //初始化串口3

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(USART3, ENABLE);					   // Enable serial port 3 //使能串口3
	// 在初始化末尾添加缓冲区初始化
	RingBuffer_Init(&USART3_RxBuffer); // everloss//
}
/**************************************************************************
Function: Serial port 5 initialization
Input   : none
Output  : none
函数功能：串口5初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart5_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// PC12 TX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // Enable the gpio clock  //使能GPIO时钟
														  // PD2 RX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); // Enable the gpio clock  //使能GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); // Enable the Usart clock //使能USART时钟

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	  // 输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	  // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 高速50MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	  // 上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);			  // 初始化

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	  // 输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	  // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 高速50MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	  // 上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);			  // 初始化

	// UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	// Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	// Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	// Enable the IRQ channel //IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// Initialize the VIC register with the specified parameters
	// 根据指定的参数初始化VIC寄存器
	NVIC_Init(&NVIC_InitStructure);

	// USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound;										// Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No;								// Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// Sending and receiving mode //收发模式
	USART_Init(UART5, &USART_InitStructure);										// Initialize serial port 5 //初始化串口5
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);									// Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(UART5, ENABLE);														// Enable serial port 5 //使能串口5
}
/**************************************************************************
Function: Serial port 6 initialization
Input   : bound - baud rate                              everloss与deepseek合力编写
Output  : none                                              usart6堂堂登场
函数功能：串口6初始化
入口参数：bound - 波特率
返 回 值：无
**************************************************************************/
void uart6_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// 1. 使能GPIOC和USART6时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	// 2. 配置GPIO复用功能
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); // TX: PC6
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); // RX: PC7

	// 3. 配置GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	  // 复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	  // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 高速50MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	  // 上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// 4. 配置USART6中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // 子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);

	// 5. 配置USART6参数
	USART_InitStructure.USART_BaudRate = bound;										// 波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式
	USART_Init(USART6, &USART_InitStructure);

	// 6. 使能接收中断
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

	// 7. 使能USART6
	USART_Cmd(USART6, ENABLE);
}
/**************************************************************************
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
函数功能：串口1接收中断
入口参数：无
返 回 值：无
**************************************************************************/
int USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		uint8_t data = USART_ReceiveData(USART1);

		// 只做最少的工作：存储数据和标记帧结束
		if (data == '\n')
		{
			USART1_RxBuffer.frame_ready = 1;
		}

		// 将数据放入环形缓冲区
		RingBuffer_Put(&USART1_RxBuffer, data);
	}
	return 0;
}

/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
// BT04-A蓝牙的AT指令反馈过滤函数.包含连接反馈与断开反馈
static uint8_t ATCommandFeedBack_BT04A(uint8_t recv)
{
#define DEBUG_BT04ACommand 0

	uint8_t isFilter = 0; // 是否允许该字符通过,1过滤,0不处理
	static uint8_t lastrecv = 0;
	static uint8_t filterIndex = 0;

	const char *BT04AConnect = "+CONNECTING<<XX:XX:XX:XX:XX:XX\r\n+CONNECTED\r\n";
	const char *BT04ADisConnect = "+DISC:SUCCESS\r\n+READY\r\n+PAIRABLE\r\n";
	enum
	{
		BT04A_NORMAL = 0,
		BT04A_CONNECTSTART,
		BT04A_DISCONNECTSTART,
	};

	static uint8_t statemachine = BT04A_NORMAL;

	// switch( statemachine )
	// {
	// 	case BT04A_NORMAL:
	// 		if( recv=='C'&&lastrecv=='+' )
	// 		{
	// 			statemachine = BT04A_CONNECTSTART;//接收到特征值,开始匹配
	// 			isFilter = 1;//过滤字符
	// 			filterIndex = 2; //2号开始索引
	// 		}
	// 		else if( recv=='D'&&lastrecv=='+' )
	// 		{
	// 			statemachine = BT04A_DISCONNECTSTART;//接收到特征值,开始匹配
	// 			isFilter = 1;//过滤字符
	// 			filterIndex = 2; //2号开始索引
	// 		}
	// 		break;
	// 	case BT04A_CONNECTSTART:
	// 		if( BT04AConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
	// 		{
	// 			isFilter = 1;//匹配连接字段,若完成匹配则过滤

	// 			#if 1== DEBUG_BT04ACommand
	// 			printf("yes:%c\r\n",recv);
	// 			#endif
	// 		}
	// 		else if( (filterIndex>=13&&filterIndex<=29) && \
	//    				  ((recv>='0'&&recv<='9')||(recv>='a'&&recv<='z')) )//进入到MAC地址匹配阶段.该阶段匹配 0~9 、a~z字段
	// 		{
	// 			isFilter = 1;//MAC地址过滤
	// 			#if 1== DEBUG_BT04ACommand
	// 			printf("yes:%c\r\n",recv);
	// 			#endif
	// 		}
	// 		else
	// 		{
	// 			//都不满足,允许字符通过.并退出过滤模式
	// 			statemachine = BT04A_NORMAL;
	// 			#if 1== DEBUG_BT04ACommand
	// 			printf("No:get:%c,but:%c\r\n",recv,BT04AConnect[filterIndex]);
	// 			#endif
	// 		}

	// 		//索引直至完成过滤列表
	// 		filterIndex++;
	// 		if( filterIndex == strlen(BT04AConnect) )
	// 		{
	// 			statemachine = BT04A_NORMAL;
	// 			#if 1== DEBUG_BT04ACommand
	// 			printf("filter con done!\r\n");
	// 			#endif
	// 		}
	// 		break;
	// 	case BT04A_DISCONNECTSTART:
	// 		if( BT04ADisConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
	// 		{
	// 			isFilter = 1;//匹配连接字段,若完成匹配则过滤
	// 			#if 1== DEBUG_BT04ACommand
	// 			printf("yes:%c\r\n",recv);
	// 			#endif
	// 		}
	// 		else
	// 		{
	// 			statemachine = BT04A_NORMAL;
	// 			#if 1== DEBUG_BT04ACommand
	// 			printf("No:get:%c,but:%c\r\n",recv,BT04ADisConnect[filterIndex]);
	// 			#endif
	// 		}

	// 		//索引直至完成过滤列表
	// 		filterIndex++;
	// 		if( filterIndex == strlen(BT04ADisConnect) )
	// 		{
	// 			statemachine = BT04A_NORMAL;
	// 			#if 1== DEBUG_BT04ACommand
	// 			printf("filter dis done!\r\n");
	// 			#endif
	// 		}
	// 		break;
	// }
	lastrecv = recv;

	return isFilter;
}

// JDY-33蓝牙AT指令集过滤
static uint8_t ATCommandFeedBack_JDY33(uint8_t recv)
{
#define DEBUG_JDY33Command 0

	uint8_t isFilter = 0; // 是否允许该字符通过,1过滤,0允许
	static uint8_t lastrecv = 0;
	static uint8_t filterIndex = 0;

	const char *JDY33_SPPConnect = "+CONNECTING<<XX:XX:XX:XX:XX:XX\r\nCONNECTED\r\r\n";
	const char *JDY33_BLEConnect = "CONNECTED\r\r\n";
	const char *JDY33_DisConnect = "+DISC:SUCCESS\r\r\n\0"; // 注意\0不会被统计字长,需要自行把字长+1
	enum
	{
		JDY33_NORMAL = 0,
		JDY33_SPPCONNECTSTART,
		JDY33_BLECONNECTSTART,
		JDY33_DISCONNECTSTART,
	};

	static uint8_t statemachine = JDY33_NORMAL;

	// switch( statemachine )
	// {
	// 	case JDY33_NORMAL:
	// 		if( recv=='C'&&lastrecv=='+' )
	// 		{
	// 			statemachine = JDY33_SPPCONNECTSTART;//接收到特征值,开始匹配
	// 			isFilter = 1;//过滤字符
	// 			filterIndex = 2; //2号开始索引
	// 		}
	// 		else if( recv=='O'&&lastrecv=='C' )
	// 		{
	// 			statemachine = JDY33_BLECONNECTSTART;//接收到特征值,开始匹配
	// 			isFilter = 1;//过滤字符
	// 			filterIndex = 2; //2号开始索引
	// 		}
	// 		else if( recv=='D'&&lastrecv=='+' )
	// 		{
	// 			statemachine = JDY33_DISCONNECTSTART;//接收到特征值,开始匹配
	// 			isFilter = 1;//过滤字符
	// 			filterIndex = 2; //2号开始索引
	// 		}
	// 		else if( recv=='C'&&lastrecv!='C' )//进入状态的判断优先,再到歧义的判断
	// 		{
	// 			//有关C的歧义,"+C"、"CO"不能通过. "?C也不允许通过",才能保证连接时不存在控制命令,若需要控制小车,需要连续的C
	// 			isFilter = 1;//禁止通过
	// 		}
	// 		break;
	// 	case JDY33_SPPCONNECTSTART:
	// 		if( JDY33_SPPConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
	// 		{
	// 			isFilter = 1;//匹配连接字段,若完成匹配则过滤

	// 			#if 1== DEBUG_JDY33Command
	// 			printf("yes:%c\r\n",recv);
	// 			#endif
	// 		}
	// 		else if( (filterIndex>=13&&filterIndex<=29) && \
	//    				  ((recv>='0'&&recv<='9')||(recv>='A'&&recv<='Z')) )//进入到MAC地址匹配阶段.该阶段匹配 0~9 、a~z字段
	// 		{
	// 			isFilter = 1;//MAC地址过滤
	// 			#if 1== DEBUG_JDY33Command
	// 			printf("yes:%c\r\n",recv);
	// 			#endif
	// 		}
	// 		else
	// 		{
	// 			//都不满足,允许字符通过.并退出过滤模式
	// 			statemachine = JDY33_NORMAL;
	// 			#if 1== DEBUG_JDY33Command
	// 			printf("SPP->No:get:%c,but:%c\r\n",recv,JDY33_SPPConnect[filterIndex]);
	// 			#endif
	// 		}

	// 		//索引直至完成过滤列表
	// 		filterIndex++;
	// 		if( filterIndex == strlen(JDY33_SPPConnect) )
	// 		{
	// 			statemachine = JDY33_NORMAL;
	// 			#if 1== DEBUG_JDY33Command
	// 			printf("SPP filter con done!\r\n");
	// 			#endif
	// 		}
	// 		break;
	// 	case JDY33_BLECONNECTSTART:
	// 		if( JDY33_BLEConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
	// 		{
	// 			isFilter = 1;//匹配连接字段,若完成匹配则过滤
	// 			#if 1== DEBUG_JDY33Command
	// 			printf("yes:%c\r\n",recv);
	// 			#endif
	// 		}
	// 		else
	// 		{
	// 			statemachine = JDY33_NORMAL;
	// 			#if 1== DEBUG_JDY33Command
	// 			printf("BLE->No:get:%c,but:%c\r\n",recv,JDY33_BLEConnect[filterIndex]);
	// 			#endif
	// 		}

	// 		//索引直至完成过滤列表
	// 		filterIndex++;
	// 		if( filterIndex == strlen(JDY33_BLEConnect) )
	// 		{
	// 			statemachine = JDY33_NORMAL;
	// 			#if 1== DEBUG_JDY33Command
	// 			printf("ble filter dis done!\r\n");
	// 			#endif
	// 		}
	// 		break;
	// 	case JDY33_DISCONNECTSTART:
	// 		if( JDY33_DisConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
	// 		{
	// 			isFilter = 1;//匹配连接字段,若完成匹配则过滤
	// 			#if 1== DEBUG_JDY33Command
	// 			printf("yes:%c\r\n",recv);
	// 			#endif
	// 		}
	// 		else
	// 		{
	// 			statemachine = JDY33_NORMAL;
	// 			#if 1== DEBUG_JDY33Command
	// 			printf("dis->No:get:%c,but:%c\r\n",recv,JDY33_DisConnect[filterIndex]);
	// 			#endif
	// 		}

	// 		//索引直至完成过滤列表
	// 		filterIndex++;
	// 		if( filterIndex == strlen(JDY33_DisConnect)+1 ) //+1为补充空字符'\0'
	// 		{
	// 			statemachine = JDY33_NORMAL;
	// 			#if 1== DEBUG_JDY33Command
	// 			printf("filter dis done!\r\n");
	// 			#endif
	// 		}
	// 		break;
	// }
	lastrecv = recv;

	return isFilter;
}

uint8_t FlashWriteFlag = 0;

int USART2_IRQHandler(void)
{
	int Usart_Receive;
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) // Check if data is received //判断是否接收到数据
	{
		static u8 Flag_PID, i, j, Receive[50], Last_Usart_Receive;
		static float Data;

		Usart_Receive = USART2->DR; // Read the data //读取数据

		// 过滤蓝牙的AT指令反馈
		uint8_t ATFilter = 0;
		ATFilter += ATCommandFeedBack_JDY33(Usart_Receive);
		ATFilter += ATCommandFeedBack_BT04A(Usart_Receive);
		if (ATFilter > 0)
			return 0; // 任意存在一个蓝牙的过滤信息,则过滤

		_System_Reset_(Usart_Receive);

		if (SysVal.Time_count < CONTROL_DELAY)
			// Data is not processed until 10 seconds after startup
			// 开机10秒前不处理数据
			return 0;

		if (Check == 0)
		{
			if (Usart_Receive == 0x41 && Last_Usart_Receive == 0x41 && APP_ON_Flag == 0)
				// 10 seconds after startup, press the forward button of APP to enter APP control mode
				// The APP controls the flag position 1 and the other flag position 0
				// 开机10秒之后，按下APP的前进键进入APP控制模式
				// APP控制标志位置1，其它标志位置0
				PS2_ON_Flag = 0, Remote_ON_Flag = 0, APP_ON_Flag = 1, CAN_ON_Flag = 0, Usart1_ON_Flag = 0, Usart5_ON_Flag = 0;
			Last_Usart_Receive = Usart_Receive;

			if (Usart_Receive == 0x4B)
				// Enter the APP steering control interface
				// 进入APP转向控制界面
				Turn_Flag = 1;
			else if (Usart_Receive == 0x49 || Usart_Receive == 0x4A)
				// Enter the APP direction control interface
				// 进入APP方向控制界面
				Turn_Flag = 0;

			if (Turn_Flag == 0)
			{
				// App rocker control interface command
				// APP摇杆控制界面命令
				if (Usart_Receive >= 0x41 && Usart_Receive <= 0x48)
				{
					Flag_Direction = Usart_Receive - 0x40;
				}
				else if (Usart_Receive <= 8)
				{
					Flag_Direction = Usart_Receive;
				}
				else
					Flag_Direction = 0;
			}
			else if (Turn_Flag == 1)
			{
				// APP steering control interface command
				// APP转向控制界面命令
				if (Usart_Receive == 0x43)
					Flag_Left = 0, Flag_Right = 1; // Right rotation //右自转
				else if (Usart_Receive == 0x47)
					Flag_Left = 1, Flag_Right = 0; // Left rotation  //左自转
				else
					Flag_Left = 0, Flag_Right = 0;
				if (Usart_Receive == 0x41 || Usart_Receive == 0x45)
					Flag_Direction = Usart_Receive - 0x40;
				else
					Flag_Direction = 0;
			}
			if (Usart_Receive == 0x58)
				RC_Velocity = RC_Velocity + 100; // Accelerate the keys, +100mm/s //加速按键，+100mm/s
			if (Usart_Receive == 0x59)
				RC_Velocity = RC_Velocity - 100; // Slow down buttons,   -100mm/s //减速按键，-100mm/s

			// The following is the communication with the APP debugging interface
			// 以下是与APP调试界面通讯
			if (Usart_Receive == 0x7B)
				Flag_PID = 1; // The start bit of the APP parameter instruction //APP参数指令起始位
			if (Usart_Receive == 0x7D)
				Flag_PID = 2; // The APP parameter instruction stops the bit    //APP参数指令停止位

			if (Usart_Receive == 'b')
				Allow_Recharge = !Allow_Recharge;

			if (Flag_PID == 1) // Collect data //采集数据
			{
				Receive[i] = Usart_Receive;
				i++;
			}
			if (Flag_PID == 2) // Analyze the data //分析数据
			{
				if (Receive[3] == 0x50)
					PID_Send = 1;
				else if (Receive[3] == 0x57)
					FlashWriteFlag = 1;
				else if (Receive[1] != 0x23)
				{
					for (j = i; j >= 4; j--)
					{
						Data += (Receive[j - 1] - 48) * pow(10, i - j);
					}
					switch (Receive[1])
					{
					case 0x30:
						RC_Velocity = Data;
						break;
					case 0x31:
						Velocity_KP = Data;
						break;
					case 0x32:
						Velocity_KI = Data;
						break;
					case 0x33:
						LineDiffParam = Data;
						break;
					case 0x34:
						break;
					case 0x35:
						break;
					case 0x36:
						break;
					case 0x37:
						break;
					case 0x38:
						break;
					}
				}
				// Relevant flag position is cleared
				// 相关标志位清零
				Flag_PID = 0;
				i = 0;
				j = 0;
				Data = 0;
				memset(Receive, 0, sizeof(u8) * 50); // Clear the array to zero//数组清零
			}
			if (RC_Velocity < 0)
				RC_Velocity = 0;
		}
		else if (Check == 1)
		{
			if (Usart_Receive == 0x41)
				uart2_send_flag = 1;
			else if (Usart_Receive == 0x43)
				uart2_send_flag = 2;
			else if (Usart_Receive == 0x45)
				uart2_send_flag = 3;
			else if (Usart_Receive == 0x47)
				uart2_send_flag = 4;
			else if (Usart_Receive == 0x5A)
				uart2_send_flag = 5;

			//			uart2_receive_message[app_count] = Usart_Receive;
			//			if(Usart_Receive=='#')
			//			{
			//				uart2_receive_message[app_count] = '\0';
			//				uart2_send_flag = 1;
			//			}
			//			app_count++;
		}
	}
	return 0;
}
/**************************************************************************
Function: Serial port 3 receives interrupted
Input   : none
Output  : none
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		uint8_t data = USART_ReceiveData(USART3);

		// 只做最少的工作：存储数据和标记帧结束
		if (data == '\n')
		{
			USART3_RxBuffer.frame_ready = 1;
		}

		// 将数据放入环形缓冲区
		RingBuffer_Put(&USART3_RxBuffer, data);
	}
	return 0;
}

/**************************************************************************
Function: Serial port 5 receives interrupted
Input   : none
Output  : none
函数功能：串口5接收中断
入口参数：无
返回  值：无
**************************************************************************/
int UART5_IRQHandler(void)
{
	static u8 Count = 0;
	u8 Usart_Receive;

	if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) // Check if data is received //判断是否接收到数据
	{
		Usart_Receive = USART_ReceiveData(UART5); // Read the data //读取数据
		if (SysVal.Time_count < CONTROL_DELAY)
			// Data is not processed until 10 seconds after startup
			// 开机10秒前不处理数据
			return 0;

		// Fill the array with serial data
		// 串口数据填入数组
		Receive_Data.buffer[Count] = Usart_Receive;

		// Ensure that the first data in the array is FRAME_HEADER
		// 确保数组第一个数据为FRAME_HEADER
		if (Usart_Receive == FRAME_HEADER || Count > 0)
			Count++;
		else
			Count = 0;

		if (Count == 11) // Verify the length of the packet //验证数据包的长度
		{
			Count = 0;								   // Prepare for the serial port data to be refill into the array //为串口数据重新填入数组做准备
			if (Receive_Data.buffer[10] == FRAME_TAIL) // Verify the frame tail of the packet //验证数据包的帧尾
			{
				// Data exclusionary or bit check calculation, mode 0 is sent data check
				// 数据异或位校验计算，模式0是发送数据校验
				if (Receive_Data.buffer[9] == Check_Sum(9, 0))
				{
					float Vz;
					// All modes flag position 0, USART3 control mode
					// 所有模式标志位置0，为Usart5控制模式
					PS2_ON_Flag = 0;
					Remote_ON_Flag = 0;
					APP_ON_Flag = 0;
					CAN_ON_Flag = 0;
					Usart5_ON_Flag = 0;
					command_lost_count = 0;
					// Calculate the target speed of three axis from serial data, unit m/s
					// 从串口数据求三轴目标速度， 单位m/s
					Move_X = XYZ_Target_Speed_transition(Receive_Data.buffer[3], Receive_Data.buffer[4]);
					Move_Y = XYZ_Target_Speed_transition(Receive_Data.buffer[5], Receive_Data.buffer[6]);
					Vz = XYZ_Target_Speed_transition(Receive_Data.buffer[7], Receive_Data.buffer[8]);
					if (Car_Mode == Akm_Car)
					{
						Move_Z = Vz_to_Akm_Angle(Move_X, Vz);
					}
					else
					{
						Move_Z = XYZ_Target_Speed_transition(Receive_Data.buffer[7], Receive_Data.buffer[8]);
					}
				}
			}
		}
	}
	return 0;
}

/**************************************************************************
Function: Serial port 6 receives interrupted
Input   : none
Output  : none                          everloss......
函数功能：串口6接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART6_IRQHandler(void)
{
	if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{
		u8 data = USART_ReceiveData(USART6);
		// 添加接收数据处理逻辑
		// 例如：存入接收缓冲区、协议解析等
	}
	return 0;
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来目标前进速度Vx、目标角速度Vz，转换为阿克曼小车的右前轮转角
入口参数：目标前进速度Vx、目标角速度Vz，单位：m/s，rad/s
返回  值：阿克曼小车的右前轮转角，单位：rad
**************************************************************************/
float Vz_to_Akm_Angle(float Vx, float Vz)
{
	float R, AngleR, Min_Turn_Radius;
	// float AngleL;

	// Ackermann car needs to set minimum turning radius
	// If the target speed requires a turn radius less than the minimum turn radius,
	// This will greatly improve the friction force of the car, which will seriously affect the control effect
	// 阿克曼小车需要设置最小转弯半径
	// 如果目标速度要求的转弯半径小于最小转弯半径，
	// 会导致小车运动摩擦力大大提高，严重影响控制效果
	Min_Turn_Radius = MINI_AKM_MIN_TURN_RADIUS;

	if (Vz != 0 && Vx != 0)
	{
		// If the target speed requires a turn radius less than the minimum turn radius
		// 如果目标速度要求的转弯半径小于最小转弯半径
		if (float_abs(Vx / Vz) <= Min_Turn_Radius)
		{
			// Reduce the target angular velocity and increase the turning radius to the minimum turning radius in conjunction with the forward speed
			// 降低目标角速度，配合前进速度，提高转弯半径到最小转弯半径
			if (Vz > 0)
				Vz = float_abs(Vx) / (Min_Turn_Radius);
			else
				Vz = -float_abs(Vx) / (Min_Turn_Radius);
		}
		R = Vx / Vz;
		// AngleL=atan(Axle_spacing/(R-0.5*Wheel_spacing));
		AngleR = atan(Axle_spacing / (R + 0.5f * Wheel_spacing));
	}
	else
	{
		AngleR = 0;
	}

	return AngleR;
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High, u8 Low)
{
	// Data conversion intermediate variable
	// 数据转换的中间变量
	short transition;

	// 将高8位和低8位整合成一个16位的short型数据
	// The high 8 and low 8 bits are integrated into a 16-bit short data
	transition = ((High << 8) + Low);
	return transition / 1000 + (transition % 1000) * 0.001; // Unit conversion, mm/s->m/s //单位转换, mm/s->m/s
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while ((USART1->SR & 0x40) == 0)
		;
}
/**************************************************************************
Function: Serial port 2 sends data
Input   : The data to send
Output  : none
函数功能：串口2发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while ((USART2->SR & 0x40) == 0)
		;
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : The data to send
Output  : none
函数功能：串口3发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while ((USART3->SR & 0x40) == 0)
		;
}

/**************************************************************************
Function: Serial port 5 sends data
Input   : The data to send
Output  : none
函数功能：串口5发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart5_send(u8 data)
{
	UART5->DR = data;
	while ((UART5->SR & 0x40) == 0)
		;
}
/**************************************************************************
Function: Serial port 6 sends data
Input   : The data to send
Output  : none                        　　everloss手动添加
函数功能：串口6发送数据                   上杉えりいが大好き
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart6_send(u8 data)
{
	USART6->DR = data;
	while ((USART6->SR & 0x40) == 0)
		; // 等待发送完成
}
/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
函数功能：计算要发送/接收的数据校验结果
入口参数：Count_Number：校验的前几位数；Mode：0-对接收数据进行校验，1-对发送数据进行校验
返回  值：校验结果
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number, unsigned char Mode)
{
	unsigned char check_sum = 0, k;

	// Validate the data to be sent
	// 对要发送的数据进行校验
	if (Mode == 1)
		for (k = 0; k < Count_Number; k++)
		{
			check_sum = check_sum ^ Send_Data.buffer[k];
		}

	// Verify the data received
	// 对接收到的数据进行校验
	if (Mode == 0)
		for (k = 0; k < Count_Number; k++)
		{
			check_sum = check_sum ^ Receive_Data.buffer[k];
		}
	return check_sum;
}

// 自动回充发送字节专用校验函数
u8 Check_Sum_AutoCharge(unsigned char Count_Number, unsigned char Mode)
{
	unsigned char check_sum = 0, k;

	// Validate the data to be sent
	// 对要发送的数据进行校验
	if (Mode == 1)
		for (k = 0; k < Count_Number; k++)
		{
			check_sum = check_sum ^ Send_AutoCharge_Data.buffer[k];
		}

	return check_sum;
}

// 蓝牙AT指令抓包，防止指令干扰到机器人正常的蓝牙通信
u8 AT_Command_Capture(u8 uart_recv)
{
	/*
	蓝牙链接时发送的字符，00:11:22:33:44:55为蓝牙的MAC地址
	+CONNECTING<<00:11:22:33:44:55\r\n
	+CONNECTED\r\n
	共44个字符

	蓝牙断开时发送的字符
	+DISC:SUCCESS\r\n
	+READY\r\n
	+PAIRABLE\r\n
	共34个字符
	\r -> 0x0D
	\n -> 0x0A
	*/

	static u8 pointer = 0; // 蓝牙接受时指针记录器
	static u8 bt_line = 0; // 表示现在在第几行
	static u8 disconnect = 0;
	static u8 connect = 0;

	// 断开连接
	static char *BlueTooth_Disconnect[3] = {"+DISC:SUCCESS\r\n", "+READY\r\n", "+PAIRABLE\r\n"};

	// 开始连接
	static char *BlueTooth_Connect[2] = {"+CONNECTING<<00:00:00:00:00:00\r\n", "+CONNECTED\r\n"};

	// 特殊标识符，开始警惕(使用时要-1)
	if (uart_recv == '+')
	{
		bt_line++, pointer = 0; // 收到‘+’，表示切换了行数
		disconnect++, connect++;
		return 1; // 抓包，禁止控制
	}

	if (bt_line != 0)
	{
		pointer++;

		// 开始追踪数据是否符合断开的特征，符合时全部屏蔽，不符合时取消屏蔽
		if (uart_recv == BlueTooth_Disconnect[bt_line - 1][pointer])
		{
			disconnect++;
			if (disconnect == 34)
				disconnect = 0, connect = 0, bt_line = 0, pointer = 0;
			return 1; // 抓包，禁止控制
		}

		// 追踪连接特征 (bt_line==1&&connect>=13)区段是蓝牙MAC地址，每一个蓝牙MAC地址都不相同，所以直接屏蔽过去
		else if (uart_recv == BlueTooth_Connect[bt_line - 1][pointer] || (bt_line == 1 && connect >= 13))
		{
			connect++;
			if (connect == 44)
				connect = 0, disconnect = 0, bt_line = 0, pointer = 0;
			return 1; // 抓包，禁止控制
		}

		// 在抓包期间收到其他命令，停止抓包
		else
		{
			disconnect = 0;
			connect = 0;
			bt_line = 0;
			pointer = 0;
			return 0; // 非禁止数据，可以控制
		}
	}

	return 0; // 非禁止数据，可以控制
}

// 软复位进BootLoader区域
void _System_Reset_(u8 uart_recv)
{
	static u8 res_buf[5];
	static u8 res_count = 0;

	res_buf[res_count] = uart_recv;

	if (uart_recv == 'r' || res_count > 0)
		res_count++;
	else
		res_count = 0;

	if (res_count == 5)
	{
		res_count = 0;
		// 接受到上位机请求的复位字符“reset”，执行软件复位
		if (res_buf[0] == 'r' && res_buf[1] == 'e' && res_buf[2] == 's' && res_buf[3] == 'e' && res_buf[4] == 't')
		{
			NVIC_SystemReset(); // 进行软件复位，复位后执行 BootLoader 程序
		}
	}
}
/**************************************************************************
激光函数移植...........部分在usart6_send上
**************************************************************************/
void parseCoordinates(const char *str, int *x1, int *y1, int *x2, int *y2,
					  int *x3, int *y3, int *x4, int *y4)
{
	// 使用 sscanf 一次性解析所有坐标
	int result = sscanf(str, "%d,%d;%d,%d;%d,%d;%d,%d",
						x1, y1, x2, y2, x3, y3, x4, y4);

	// 如果解析失败（返回值小于8），设置默认值
	if (result != 8)
	{
		*x1 = *y1 = *x2 = *y2 = *x3 = *y3 = *x4 = *y4 = -1;
	}
	turn_time = 0;
}

// 位置解析函数
void parsePositions(const char *str, float *x_car, float *y_car, float *x_enemy, float *y_enemy)
{
	// 使用 sscanf 解析两个坐标
	int result = sscanf(str, "%f,%f;%f,%f", x_enemy, y_enemy, x_car, y_car);
	// 如果解析失败（返回值小于4），设置默认值
}