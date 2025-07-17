#ifndef __OLED_H
#define __OLED_H			  	 
#include "sys.h"
#include "system.h"

extern u8 OLED_GRAM[128][8];

//Oled port macro definition
//OLED端口宏定义
#define OLED_RST_Clr() PDout(12)=0   //RST
#define OLED_RST_Set() PDout(12)=1   //RST

#define OLED_RS_Clr()  PDout(11)=0   //DC
#define OLED_RS_Set()  PDout(11)=1   //DC

#define OLED_SCLK_Clr()  PDout(14)=0   //SCL
#define OLED_SCLK_Set()  PDout(14)=1   //SCL

#define OLED_SDIN_Clr()  PDout(13)=0   //SDA
#define OLED_SDIN_Set()  PDout(13)=1   //SDA
#define OLED_CMD  0	//Command //写命令
#define OLED_DATA 1	//Data //写数据

#define OLED_Height			64
#define OLED_Width			128

//Oled control function
//OLED控制用函数
void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   				   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p);
void OLED_ShowString16(u8 x,u8 y,const u8 *p);
void OLED_Refresh_Line(void);

#define CNSizeWidth  16
#define CNSizeHeight 16
//extern char Hzk16[][16];

void OLED_ShowCHinese(u8 x,u8 y,u8 *str);	
void OLED_ShowFont16(u8 x,u8 y,u8 *s);
void OLED_ShowFont12(u8 x,u8 y,u8 *s);
void OLED_ShowCHinese12(u8 x,u8 y,u8 *str);
//void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_Show_POT(void);
void OLED_DrawPoint_Shu(u8 x,u8 y,u8 t);

//void OLED_ShowCHinese(u8 x,u8 y,u8 no,u8 font_width,u8 font_height);	
void OLED_Set_Pos(unsigned char x, unsigned char y);
void oled_showfloat(const float needtoshow,u8 show_x,u8 show_y,u8 zs_num,u8 xs_num);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,const unsigned char BMP[]);

extern const unsigned char gImage_usb[420];

#endif  
	 
