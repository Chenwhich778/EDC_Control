#ifndef CAR_BSP_H
#define CAR_BSP_H


#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>
#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "./ICM45686/IMU/IMU.h"
#include "./Grayscale_Sensor/hardware_iic.h"


/*新添加头文件在这里！        everloss 倾情添加*/
#include "laser_tracker.h"  //servo_protocol.h在laser_tracker.h中无需再次添加
#include "usart.h"
#include "sensor.h"
#include "key.h"
#include "stdlib.h"

#define rKEY1 DL_GPIO_readPins(KEY1_PORT, KEY1_PB5_PIN)
/*********************************************usb_uart0*********************************************************/
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
void usb_uart0_IRQ_init(void);
/**************************************************************************************************************/

/**********************************************key1*************************************************************/
void key1_IRQ_init(void);
/**************************************************************************************************************/


/*********************************************oled*********************************************************/
#define OLED_CMD 0  // 写命令
#define OLED_DATA 1 // 写数据
#define OLED_MODE 0

#define u8 uint8_t
#define u32 uint32_t

//-----------------OLED端口定义----------------
// 要是SPI名字设的不一样在这改下就行,例如下面这样

#define OLED_CS_Clr() DL_GPIO_clearPins(OLED_CS_PORT,OLED_CS_PIN)
#define OLED_CS_Set() DL_GPIO_setPins(OLED_CS_PORT,OLED_CS_PIN)

#define OLED_RST_Clr() DL_GPIO_clearPins(OLED_RESET_PORT,OLED_RESET_PIN) 
#define OLED_RST_Set() DL_GPIO_setPins(OLED_RESET_PORT,OLED_RESET_PIN)

#define OLED_DC_Clr() DL_GPIO_clearPins(OLED_DC_PORT,OLED_DC_PIN)
#define OLED_DC_Set() DL_GPIO_setPins(OLED_DC_PORT,OLED_DC_PIN)

// #define OLED_SCLK_Clr() DL_GPIO_clearPins(GPIOB, GPIO_GRP_0_D0_PIN)
// #define OLED_SCLK_Set() DL_GPIO_setPins(GPIOB, GPIO_GRP_0_D0_PIN)

// #define OLED_SDIN_Clr() DL_GPIO_clearPins(GPIOB, GPIO_GRP_0_D1_PIN)
// #define OLED_SDIN_Set() DL_GPIO_setPins(GPIOB, GPIO_GRP_0_D1_PIN)
// OLED模式设置
// 0:4线串行模式
// 1:并行8080模式

#define SIZE 16
#define XLevelL 0x02
#define XLevelH 0x10
#define Max_Column 128
#define Max_Row 64
#define Brightness 0xFF
#define X_WIDTH 128
#define Y_WIDTH 64


// OLED控制用函数
void OLED_WR_Byte(u8 dat, u8 cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x, u8 y, u8 t);
void OLED_Fill(u8 x1, u8 y1, u8 x2, u8 y2, u8 dot);
void OLED_ShowChar(u8 x, u8 y, u8 chr);
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size2);
void OLED_ShowString(u8 x, u8 y, u8 *p);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x, u8 y, u8 no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[]);
/*************************************************************************************************************/


/*********************************************buzzer***********************************************************/
void set_buzzer_hz_duty(uint16_t hz,uint16_t duty);/*改变蜂鸣器占空比和频率*/
/**************************************************************************************************************/


/*********************************************rgb*************************************************************/
#define RGB_B(x) ((x) ? DL_GPIO_setPins(RGB_GB_B_PORT, RGB_GB_B_PIN) : DL_GPIO_clearPins(RGB_GB_B_PORT, RGB_GB_B_PIN))
#define RGB_G(x) ((x) ? DL_GPIO_setPins(RGB_GB_G_PORT, RGB_GB_G_PIN) : DL_GPIO_clearPins(RGB_GB_G_PORT, RGB_GB_G_PIN))
void set_rgb_duty(uint8_t r,uint8_t g,uint8_t b);
/**************************************************************************************************************/


/*********************************************vcc_adc**********************************************************/
void vcc_adc_IQR_init(void);
uint16_t get_vcc_adc_value(void);
/**************************************************************************************************************/
/********************************************motor*************************************************************/
#define MOTOR1_CTRL1(x)  ( (x) ? DL_GPIO_setPins(MOTOR1_PORT,MOTOR1_CTRL1_PIN) : DL_GPIO_clearPins(MOTOR1_PORT,MOTOR1_CTRL1_PIN) )
#define MOTOR1_CTRL2(x)  ( (x) ? DL_GPIO_setPins(MOTOR1_PORT,MOTOR1_CTRL2_PIN) : DL_GPIO_clearPins(MOTOR1_PORT,MOTOR1_CTRL2_PIN) )

#define MOTOR2_CTRL1(x)  ( (x) ? DL_GPIO_setPins(MOTOR2_CTRL3_PORT,MOTOR2_CTRL3_PIN) : DL_GPIO_clearPins(MOTOR2_CTRL3_PORT,MOTOR2_CTRL3_PIN) )
#define MOTOR2_CTRL2(x)  ( (x) ? DL_GPIO_setPins(MOTOR2_CTRL4_PORT,MOTOR2_CTRL4_PIN) : DL_GPIO_clearPins(MOTOR2_CTRL4_PORT,MOTOR2_CTRL4_PIN) )

#define MOTOR3_CTRL1(x)  ( (x) ? DL_GPIO_setPins(MOTOR3_CTRL5_PORT,MOTOR3_CTRL5_PIN) : DL_GPIO_clearPins(MOTOR3_CTRL5_PORT,MOTOR3_CTRL5_PIN) )
#define MOTOR3_CTRL2(x)  ( (x) ? DL_GPIO_setPins(MOTOR3_CTRL6_PORT,MOTOR3_CTRL6_PIN) : DL_GPIO_clearPins(MOTOR3_CTRL6_PORT,MOTOR3_CTRL6_PIN) )

#define MOTOR4_CTRL1(x)  ( (x) ? DL_GPIO_setPins(MOTOR4_CTRL7_PORT,MOTOR4_CTRL7_PIN) : DL_GPIO_clearPins(MOTOR4_CTRL7_PORT,MOTOR4_CTRL7_PIN) )
#define MOTOR4_CTRL2(x)  ( (x) ? DL_GPIO_setPins(MOTOR4_CTRL8_PORT,MOTOR4_CTRL8_PIN) : DL_GPIO_clearPins(MOTOR4_CTRL8_PORT,MOTOR4_CTRL8_PIN) )


#define rMOTOR_ENCA1 DL_GPIO_readPins(MOTOR1_PORT, MOTOR1_ENCA1_PIN)
#define rMOTOR_ENCB1 DL_GPIO_readPins(MOTOR1_PORT, MOTOR1_ENCB1_PIN)

#define rMOTOR_ENCA2 DL_GPIO_readPins(MOTOR2_ENCA2_PORT, MOTOR2_ENCA2_PIN)
#define rMOTOR_ENCB2 DL_GPIO_readPins(MOTOR2_ENCB2_PORT, MOTOR2_ENCB2_PIN)

#define rMOTOR_ENCA3 DL_GPIO_readPins(MOTOR3_ENCA3_PORT, MOTOR3_ENCA3_PIN)
#define rMOTOR_ENCB3 DL_GPIO_readPins(MOTOR3_ENCB3_PORT, MOTOR3_ENCB3_PIN)

#define rMOTOR_ENCA4 DL_GPIO_readPins(MOTOR4_ENCA4_PORT, MOTOR4_ENCA4_PIN)
#define rMOTOR_ENCB4 DL_GPIO_readPins(MOTOR4_ENCB4_PORT, MOTOR4_ENCB4_PIN)
void set_motor(int motor1,int motor2,int motor3,int motor4);
void motor_init(void);
uint32_t get_motor_enc_count(uint8_t swi);


#define SPI0_CS0(x) ((x) ? DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_8) : DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_8))
extern uint32_t nowtime;
void SPI0_reload(void);
/**************************************************************************************************************/

//YAREYARE EVERLOSS

extern int8_t EN;
extern int x;
extern int y;


void stop_all_motors(void);

void UART0_ProcessFrame(void);















#endif