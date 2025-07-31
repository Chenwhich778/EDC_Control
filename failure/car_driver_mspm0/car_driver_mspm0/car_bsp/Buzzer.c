#include "car_bsp.h"


void set_buzzer_hz_duty(uint16_t hz,uint16_t duty)
{
    uint16_t count=400000/hz;
    TIMG0->COUNTERREGS.LOAD=count;
     DL_TimerG_setCaptureCompareValue(PWM_G0_INST,count*duty/100,GPIO_PWM_G0_C0_IDX);
}







