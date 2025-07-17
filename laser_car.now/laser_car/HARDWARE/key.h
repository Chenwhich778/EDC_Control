#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#include "system.h"

void KEY_Init(void);
u8 click(void);
void Delay_ms(void);
u8 click_N_Double (u8 time);
u8 click_N_Double_MPU6050 (u8 time);
u8 Long_Press(void);

/*--------KEY control pin--------*/
#define KEY_PORT	GPIOE
#define KEY_PIN		GPIO_Pin_0
#define KEY			PEin(0) 
/*----------------------------------*/

u8 KEY_Scan(u16 Frequency,u16 filter_times);
enum {
	key_stateless,
	single_click,
	double_click,
	long_click
};

#endif 
