#include "car_bsp.h"


void set_rgb_duty(uint8_t r,uint8_t g,uint8_t b)
{
    uint16_t TIMG0_LOAD=TIMG0->COUNTERREGS.LOAD;
    DL_TimerG_setCaptureCompareValue(PWM_G0_INST,TIMG0_LOAD*r/100,GPIO_PWM_G0_C1_IDX);
    RGB_B(!b);
    RGB_G(!g);
}

