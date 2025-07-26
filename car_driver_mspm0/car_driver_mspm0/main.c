/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* 此测试代码，小车上电后，OLED显示logo */
/* 接着小车开始不间断声光提示，代码在./car_bsp/ICM45686/timer_100us.c */
/* 数秒后，小车初始化完成，期间要求小车静止不动，屏幕开始显示编码器速度以及姿态数据 */
/* 根据需求，小车开始走直线 */

#include "ti_msp_dl_config.h"
#include "car_bsp/car_bsp.h"
#include "../car_bsp/OLED/oledfont_bmp.h"
// 定义位索引（0=最低位/最右侧，7=最高位/最左侧）
#define BIT_0 0
#define BIT_1 1
#define BIT_2 2
#define BIT_3 3
#define BIT_4 4
#define BIT_5 5
#define BIT_6 6
#define BIT_7 7

// 计算黑线加权位置（0=最左，6=最右，-1=无黑线）
int Get_Line_Position(uint8_t values[7]) {
    int sum = 0, count = 0;
    for (int i = 0; i < 7; i++) {
        if (values[i]) {
            sum += i;
            count++;
        }
    }
    return (count > 0) ? (sum * 10 / count) : -1; // 放大10倍提高精度
}




/* IMU数值转化为字符串 */
char syaw[7];
char spitch[7];
char sroll[7];

/* 编码器数值转化为字符串 */
char sM4_C[4];
char sM3_C[4];

/* IMU数值 */
float ypr[3]; // 上传yaw pitch roll的值

/* 记录上一时刻和当前时刻编码器数值 */
uint32_t a1,a2,b1,b2;

/* 记录感为灰度I2C返回数值 */
unsigned char Digtal;

/* 记录偏移数值 */
uint32_t speed_sum=0;

/* 记录静止时IMU_yaw数值 */
float yaw_f=0;

/* 设定小车启动默认电机数值 */
uint16_t speed[4]={0,0,0,0};

/* 将逻辑电平转化为数据，灰度中心离巡线越远，数值变化越大，变化范围-8~+8，有缺陷，但寻单个曲线足够 */
void update_speed_sum(uint8_t data, uint32_t* speed_sum) 
{
    // 清零初始值（根据需求可省略）
    *speed_sum = 0;
    
    // 逐位检测并更新值
    if (data & (1 << BIT_7)) *speed_sum += 8;  // 第7位=1时加8
    if (data & (1 << BIT_6)) *speed_sum += 4;  // 第6位=1时加4
    if (data & (1 << BIT_5)) *speed_sum += 2;  // 第5位=1时加2
    if (data & (1 << BIT_4)) *speed_sum += 1;  // 第4位=1时加1
    
    if (data & (1 << BIT_3)) *speed_sum -= 1;  // 第3位=1时减1
    if (data & (1 << BIT_2)) *speed_sum -= 2;  // 第2位=1时减2
    if (data & (1 << BIT_1)) *speed_sum -= 4;  // 第1位=1时减4
    if (data & (1 << BIT_0)) *speed_sum -= 8;  // 第0位=1时减8
}

int main(void)
{
    /* 初始化系统，由sysconfig配置 */
    SYSCFG_DL_init();

    /* 初始化调试串口 */
    usb_uart0_IRQ_init();
    delay_ms(100); // 等待部署

    /* 初始化OLED */
    OLED_Init();    

    /* OLED与IMU共用SPI，此板卡OLED使用完转IMU需要重新初始化SPI */
    SPI0_reload();

    /* IMU初始化，静止不动采集数据1s左右 */
	IMU_init();
    NVIC_EnableIRQ(TIMER_A0_100us_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_G12_1ms_INST_INT_IRQN);
    for (int i=0; i<50; i++) {
    IMU_getYawPitchRoll(ypr); 
    Servo_SetPosition(1, prime_x1 , 200);
    Servo_SetPosition(2, prime_y1 , 200);
    delay_ms(10);
    }

    /* 开始记录静止时yaw数值 */
    yaw_f=ypr[0];
    delay_ms(100); // 等待部署

    /* 按键外部触发中断使能 */
    key1_IRQ_init();

    /* 初始化ADC,电池电压采集 */
    vcc_adc_IQR_init();

    /* 开启PWM计数，sysconfig设置后可不需要 */
    DL_TimerG_startCounter(PWM_G0_INST);
    // set_buzzer_hz_duty(400,0);
    // set_rgb_duty(50,0,0);

    /* 初始化电机 */
    motor_init();

    /* 设置电机速度和方向 -1000 ~ -1 和 1~1000 */
    set_motor(speed[0],speed[1],speed[2],speed[3]);

    uint8_t sensor_values[7];
    while (1) 
    {
        Read_Grayscale(sensor_values);
        int pos = Get_Line_Position(sensor_values);


        a1=get_motor_enc_count(4);
        b1=get_motor_enc_count(3);
		IMU_getYawPitchRoll(ypr);
        OLED_Clear();
		OLED_DrawBMP(0, 0, 16, 2  , qishi);
		OLED_ShowString(16,0,"M3:");
        OLED_ShowString(72,0,"M4:");
		OLED_ShowString(0,2,"yaw  :");
		OLED_ShowString(0,4,"EN:");
		OLED_ShowString(0,6,"pos :");
		sprintf(syaw, "%.2f", ypr[0]);
		sprintf(spitch, "%d", EN);
		sprintf(sroll, "%d", pos);
        sprintf(sM4_C, "%d", a2-a1);
        sprintf(sM3_C, "%d", b2-b1);
        OLED_ShowString(96,0,sM3_C);
		OLED_ShowString(40,0,sM4_C);
		OLED_ShowString(54,2,syaw);
		OLED_ShowString(54,4,spitch);
		OLED_ShowString(54,6,sroll);
        SPI0_reload();
        a2=a1;
        b2=b1;

        /* 根据灰度数据行驶寻曲线 */
		//  Digtal=IIC_Get_Digtal();
		//  update_speed_sum(Digtal,&speed_sum);

        /* 根据IMU数据行驶走直线 */
        ypr[0]-=yaw_f;

        if(EN == 1)
        set_motor(100-ypr[0]*50,100+ypr[0]*50,200-ypr[0]*50,200+ypr[0]*50);
        else
        stop_all_motors();

        /* 轮询基本延时 */
        delay_ms(10);
    }
}
