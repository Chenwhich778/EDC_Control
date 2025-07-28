#include "../car_bsp.h"
extern float speed3;
extern float speed4;
uint32_t nowtime=0;
int a2=0;
int b2=0;
void TIMER_A0_100us_INST_IRQHandler(void)
{
    switch( DL_TimerA_getPendingInterrupt(TIMER_A0_100us_INST) )
    {
        case DL_TIMER_IIDX_ZERO://如果是0溢出中断
            nowtime++;
            if (nowtime%10==0) {
                int a1=get_motor_enc_count(4);
                int b1=get_motor_enc_count(3);
                speed3=a1-a2;
                speed4=b1-b2;
                b2=b1;
                a2=a1;
            }
            break;
        default://其他的定时器中断
            break;
    }
}

void TIMER_G12_1ms_INST_IRQHandler(void)
{
    switch( DL_TimerA_getPendingInterrupt(TIMER_G12_1ms_INST) )
    {
        case DL_TIMER_IIDX_ZERO://如果是0溢出中断
            // nowtime++;
            // if (nowtime%10000==0) {
            //     // set_buzzer_hz_duty(600,50);
            //     set_rgb_duty(80,1,0);
            // }
            // else if (nowtime%5000==0) {
            //     // set_buzzer_hz_duty(600,0);
            //     set_rgb_duty(0,0,0);
            // }
            break;
        default://其他的定时器中断
            break;
    }
}
