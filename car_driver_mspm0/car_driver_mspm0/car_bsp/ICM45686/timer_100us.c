#include "../car_bsp.h"

uint32_t nowtime=0;
void TIMER_A0_100us_INST_IRQHandler(void)
{
    switch( DL_TimerA_getPendingInterrupt(TIMER_A0_100us_INST) )
    {
        case DL_TIMER_IIDX_ZERO://如果是0溢出中断
            nowtime++;
            if (nowtime%10000==0) {
                // set_buzzer_hz_duty(600,50);
                set_rgb_duty(80,1,0);
            }
            else if (nowtime%5000==0) {
                // set_buzzer_hz_duty(600,0);
                set_rgb_duty(0,0,0);
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
            nowtime++;
            if (nowtime%10000==0) {
                // set_buzzer_hz_duty(600,50);
                set_rgb_duty(80,1,0);
            }
            else if (nowtime%5000==0) {
                // set_buzzer_hz_duty(600,0);
                set_rgb_duty(0,0,0);
            }
            break;
        default://其他的定时器中断
            break;
    }
}
