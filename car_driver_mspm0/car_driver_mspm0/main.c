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
#include "math.h"

// 定义位索引（0=最低位/最右侧，7=最高位/最左侧）
#define BIT_0 0
#define BIT_1 1
#define BIT_2 2
#define BIT_3 3
#define BIT_4 4
#define BIT_5 5
#define BIT_6 6
#define BIT_7 7





/* IMU数值转化为字符串 */
char syaw[7];
char spitch[7];
char sroll[7];
char srol[7];

/* 编码器数值转化为字符串 */
char sM4_C[4];
char sM3_C[4];

/* IMU数值 */
float ypr[3]; // 上传yaw pitch roll的值
float adjust=0.0;
/* 记录上一时刻和当前时刻编码器数值 */
int a1,a2,b1,b2;

/* 记录感为灰度I2C返回数值 */
unsigned char Digtal;

/* 记录偏移数值 */
uint32_t speed_sum=0;

/* 记录静止时IMU_yaw数值 */
float yaw_f=0;
float pre_yaw=0.0;
float mibu=0.0;
char input='0';
float speed3_filter=0.0;
float speed4_filter=0.0;
/* 设定小车启动默认电机数值 */
uint16_t speed[4]={0,0,0,0};
/*小车pid*/
PID_Controller pid_left;
PID_Controller pid_right;

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
    int turn=1;
    int turn_flag=0;
    int turn_count=0;
    SYSCFG_DL_init();
    PID_Init(&pid_left, 100, 8, 0, 0.01, 1000);
    PID_Init(&pid_right, 5, 0.1, 0.01, 0.01, 1000);
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
    // NVIC_EnableIRQ(TIMER_G12_1ms_INST_INT_IRQN);
    for (int i=0; i <100; i++) {
    IMU_getYawPitchRoll(ypr); 
    Servo_SetPosition(1, prime_x1 , 500);
    Servo_SetPosition(2, prime_y1 , 500);
    delay_ms(10);
    }

    /* 开始记录静止时yaw数值 */
    yaw_f=ypr[0];
    
    delay_ms(150); // 等待部署

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
    // set_motor(speed[0],speed[1],speed[2],speed[3]);

    int8_t sensor_values[7];
    
    int base_speed = 170;    // 基础速度
    float kp = 30.0;         // 比例控制系数
    int weights[7] = {-1.5, -1, -0.5, 0, 0.5, 1, 1.5}; // 传感器权重（右负左正）
    float error = 0;
    float last_error = 0;
    float steer = 0;


    while (1) 
    {
        Read_Grayscale(sensor_values);
        UART0_ProcessFrame();
        
        yaw_f-=0.01478;
        a1=get_motor_enc_count(3);
        b1=get_motor_enc_count(4);
		IMU_getYawPitchRoll(ypr);
        ypr[0]-=yaw_f;
        float tmp=ypr[0]-pre_yaw;
        if(tmp<-200)
            mibu=+360;
        else if(tmp>200)
            mibu-=360;
        pre_yaw=ypr[0];
        ypr[0]+=mibu;
        // while(ypr[0]>180)
        // ypr[0]-=360;
        // while(ypr[0]<-180)
        // ypr[0]+=360;
        // if (EN!=1) {
        //     uint8_t keyboard=getKeyValue();
        //     input=get_keychar(keyboard);
        // }
        OLED_Clear();
		OLED_DrawBMP(0, 0, 16, 2  , qishi);
		OLED_ShowString(16,0,"M3:");
        OLED_ShowString(72,0,"M4:");
		OLED_ShowString(0,2,"yaw  :");
		OLED_ShowString(0,4,"EN:");
		OLED_ShowString(0,6,"x :");
        OLED_ShowString(64,6,"y :");
		sprintf(syaw, "%.2f", ypr[0]);
		sprintf(spitch, "%d  %d", turn_flag,turn_count);
		sprintf(sroll, "%d", x);
        sprintf(srol, "%d", y);
        
        float speed4=(b1-b2)*5*0.5+speed4_filter*0.5;
        float speed3=(a1-a2)*5*0.5+speed3_filter*0.5;
        speed4_filter=(b1-b2)*5;
        speed3_filter=(a1-a2)*5;
        
        sprintf(sM3_C, "%.1f", speed3);
        sprintf(sM4_C, "%.1f", speed4);
        OLED_ShowString(40,0,sM3_C);
		OLED_ShowString(96,0,sM4_C);
		OLED_ShowString(54,2,syaw);
		OLED_ShowString(54,4,spitch);
		OLED_ShowString(24,6,sroll);
        OLED_ShowString(96,6,srol);
        SPI0_reload();
        a2=a1;
        b2=b1;


printf("%.2f,%.2f\n",speed3,speed4);

        /* 根据灰度数据行驶寻曲线 */
		//  Digtal=IIC_Get_Digtal();
		//  update_speed_sum(Digtal,&speed_sum);

        /* 根据IMU数据行驶走直线 */
        
        adjust=-ypr[0]/360*4096;
        //  Servo_SetPosition(1,servo_x+adjust,1000);
        if(turn_flag==0){
         control_camera(x, y);
         Servo_SetPosition(1, servo_x+adjust, 1200);
        Servo_SetPosition(2, servo_y, 1000);

        }
        if(turn_flag==1){
         control_camera(x, y);
         Servo_SetPosition(1, servo_x+adjust, 1200);
         Servo_SetPosition(2, servo_y, 1000);

        }


// 灰度循迹算法 ======================================
        int line_position = 0;
        int sensor_count = 0;
        error = 0;
        
        // 1. 计算黑线位置（加权平均）
        for (int i = 0; i < 7; i++) {
            if (sensor_values[i] == 0) { // 检测到黑线
                error += weights[i];
                sensor_count++;
            }
        }
        
       if(sensor_count>=4&&turn==1)

       {
         turn_flag=1;
         turn=0;
         turn_count=-32;
         
       }


        //2. 处理不同检测情况
        if (sensor_count > 0) {
            error /= sensor_count; // 计算平均位置误差
        } else {
         //   未检测到黑线：使用上次误差或停止
            error = (last_error > 0) ? 3.0 : -3.0;
        }
        
        // 3. 比例控制计算转向量
        steer = kp * error;
        last_error = error; // 保存本次误差
        
        // 4. 计算电机速度 (差速转向)
        int left_speed  = base_speed - steer;
        int right_speed = base_speed + steer;
        
        // 5. 速度限制 (防止超限)
        left_speed = (left_speed < -500) ? -500 : (left_speed > 500 ? 500 : left_speed);
        right_speed = (right_speed < -500) ? -500 : (right_speed > 500 ? 500 : right_speed);
        if(EN == 1){
        // set_motor(100-ypr[0]*50,100+(ypr[0])*50,(200-ypr[0]*10)*0.5,(200+ypr[0]*10)*0.5);
       //此处加入灰度循迹代码
        // if (turn_count<0) {
        //     turn_count++;
        // }
        // if(turn_flag==0){
        // if(turn_count++>80)
        //   set_motor(0, 0, left_speed, right_speed);
        // else
        //  stop_all_motors();
        // }
        // else if(turn_flag==1&&turn_count>=0){
        //     set_motor(0, 0, 165, -165);
        //     turn_count++;
        //     if(turn_count>40)
        //     turn_flag=0;
        // }
        float output_left=PID_Compute(&pid_left, 100, speed3);
        float output_right=PID_Compute(&pid_right, 50, speed4);
        set_motor(1, 1, left_speed, 1);
        }
        else
         stop_all_motors();

        /* 轮询基本延时 */
          delay_ms(10);
    }
}
