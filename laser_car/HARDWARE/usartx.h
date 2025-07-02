#ifndef __USRATX_H
#define __USRATX_H 

#include "stdio.h"
#include "sys.h"
#include "system.h"
#include "laser_tracker.h"//������ֲ

#define DATA_STK_SIZE   512 
#define DATA_TASK_PRIO  5

#define FRAME_HEADER      0X7B //Frame_header //֡ͷ
#define FRAME_TAIL        0X7D //Frame_tail   //֡β
#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11

#define AutoCharge_HEADER      0X7C //Frame_header //֡ͷ
#define AutoCharge_TAIL        0X7F //Frame_tail   //֡β
#define AutoCharge_DATA_SIZE    8

/*****A structure for storing triaxial data of a gyroscope accelerometer*****/
/*****���ڴ�������Ǽ��ٶȼ��������ݵĽṹ��*********************************/
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes //2���ֽ�
	short Y_data; //2 bytes //2���ֽ�
	short Z_data; //2 bytes //2���ֽ�
}Mpu6050_Data;

/*******The structure of the serial port sending data************/
/*******���ڷ������ݵĽṹ��*************************************/
typedef struct _SEND_DATA_  
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1���ֽ�
		short X_speed;	            //2 bytes //2���ֽ�
		short Y_speed;              //2 bytes //2���ֽ�
		short Z_speed;              //2 bytes //2���ֽ�
		short Power_Voltage;        //2 bytes //2���ֽ�
		Mpu6050_Data Accelerometer; //6 bytes //6���ֽ�
		Mpu6050_Data Gyroscope;     //6 bytes //6���ֽ�	
		unsigned char Frame_Tail;   //1 bytes //1���ֽ�
	}Sensor_Str;
}SEND_DATA;

typedef struct _SEND_AutoCharge_DATA_  
{
	unsigned char buffer[AutoCharge_DATA_SIZE];
	struct _AutoCharge_Str_
	{
		unsigned char Frame_Header; //1 bytes //1���ֽ�
		short Charging_Current;	    //2 bytes //2���ֽ�
		unsigned char RED;          //1 bytes //1���ֽ�
		unsigned char Charging;     //1 bytes //1���ֽ�
		unsigned char yuliu;		//1 bytes //1���ֽ�
		unsigned char Frame_Tail;   //1 bytes //1���ֽ�
	}AutoCharge_Str;
}SEND_AutoCharge_DATA;

typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header; //1 bytes //1���ֽ�
		float X_speed;	            //4 bytes //4���ֽ�
		float Y_speed;              //4 bytes //4���ֽ�
		float Z_speed;              //4 bytes //4���ֽ�
		unsigned char Frame_Tail;   //1 bytes //1���ֽ�
	}Control_Str;
}RECEIVE_DATA;

void data_task(void *pvParameters);
void data_transition(void);
void USART1_SEND(void);
void USART3_SEND(void);
void USART3_Return(void);
void USART2_Return(void);
void USART5_SEND(void);
void USART6_SEND(void);//����

void CAN_SEND(void);
void uart1_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound);
void uart5_init(u32 bound);
void uart6_init(u32 bound);//����

int USART1_IRQHandler(void);
int USART2_IRQHandler(void);
int USART3_IRQHandler(void);
int UART5_IRQHandler(void);
int UART6_IRQHandler(void);//����

float Vz_to_Akm_Angle(float Vx, float Vz);
float XYZ_Target_Speed_transition(u8 High,u8 Low);
void usart1_send(u8 data);
void usart2_send(u8 data);
void usart3_send(u8 data);
void usart5_send(u8 data);
void usart6_send(u8 data);//����

u8 Check_Sum(unsigned char Count_Number,unsigned char Mode);
u8 Check_Sum_AutoCharge(unsigned char Count_Number,unsigned char Mode);
u8 AT_Command_Capture(u8 uart_recv);
void _System_Reset_(u8 uart_recv);

extern uint8_t FlashWriteFlag;

extern int x1, y1, x2, y2, x3, y3, x4, y4;
extern uint16_t turn_time;
extern float X_car,Y_car,X_enemy,Y_enemy;

// ���ӻ��λ������ṹ
#define RING_BUF_SIZE 1024
// �������������
extern volatile uint32_t usart1_overflow_count;
extern volatile uint32_t usart3_overflow_count;
// 定义目标信息结构体
typedef struct {
    char shape[20];          // 形状：circle/rectangle等
    char color[20];          // 颜色
    int center_x;            // 中心点X坐标
    int center_y;            // 中心点Y坐标
    float size;              // 尺寸（直径或边长）
    float distance;          // 距离（米）
    char size_unit[10];      // 尺寸单位：px/m
    int detected;            // 是否检测到目标（1=检测到，0=未检测到）
} ObjectInfo;

extern ObjectInfo info;

typedef struct {
	uint8_t buffer[RING_BUF_SIZE];
	volatile uint16_t head;
	volatile uint16_t tail;
	volatile uint8_t frame_ready;
} RingBuffer;

// �����ⲿ������
extern RingBuffer USART1_RxBuffer;
extern RingBuffer USART3_RxBuffer;

// ���Ӻ�������
void RingBuffer_Init(RingBuffer *rb);
uint8_t RingBuffer_Put(RingBuffer *rb, uint8_t data);
uint8_t RingBuffer_Get(RingBuffer *rb, uint8_t *data);
uint16_t RingBuffer_Available(RingBuffer *rb);
uint8_t RingBuffer_IsEmpty(RingBuffer *rb);
void USART1_ProcessFrame(void);
void USART3_ProcessFrame(void);


#endif

