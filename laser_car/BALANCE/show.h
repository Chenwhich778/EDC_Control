#ifndef __SHOW_H
#define __SHOW_H
#include "sys.h"
#include "oled.h"
#include "system.h"
#include "bsp_gamepad.h"
#define SHOW_TASK_PRIO		3
#define SHOW_STK_SIZE 		512  

void show_task(void *pvParameters);
void oled_show(void);
void APP_Show(void);
void OLED_ShowCheckConfirming(void);
void OLED_ShowChecking(void);
void OLED_ShowCheckResult(void);

extern u8 oled_page,oled_refresh_flag;
#define OLED_MAX_Page 2 //OLED²Ëµ¥Ò³Êý

extern float base_vol;
extern float VolMean_Filter(float data);
extern int Buzzer_count;
#endif
