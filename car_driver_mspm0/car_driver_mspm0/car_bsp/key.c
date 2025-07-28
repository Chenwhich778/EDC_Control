#include "key.h"
#include "car_bsp/car_bsp.h"
const struct {
    GPIO_Regs* gpio;
    uint32_t pin;
} SENSOR_PINS[8] = {
    {GPIOB, KEY_H1_PIN}, 
    {GPIOB, KEY_H2_PIN},  
    {GPIOB, KEY_H3_PIN}, 
    {GPIOA, KEY_H4_PIN},  
    {GPIOB, KEY_V1_PIN}, 
    {GPIOB, KEY_V2_PIN},  
    {GPIOA, KEY_V3_PIN},   
    {GPIOB, KEY_V4_PIN}  
};
int getKeyValue(void)
{
    int i, j = 0;
    int key_value = 0;
    for (i = 0; i < 4; i++)
    {

        DL_GPIO_setPins(SENSOR_PINS[i].gpio, SENSOR_PINS[i].pin);
        DL_GPIO_clearPins(SENSOR_PINS[(i+1)%4].gpio, SENSOR_PINS[(i+1)%4].pin);
        DL_GPIO_clearPins(SENSOR_PINS[(i+2)%4].gpio, SENSOR_PINS[(i+2)%4].pin);
        DL_GPIO_clearPins(SENSOR_PINS[(i+3)%4].gpio, SENSOR_PINS[(i+3)%4].pin);

         delay_ms(3);

        for (j = 0; j < 4; j++)
        {
                if (DL_GPIO_readPins(SENSOR_PINS[j+4].gpio, SENSOR_PINS[j+4].pin) != 0){
                    key_value = j * 4 + i + 1;
                }
             delay_cycles(1000);
        }

         delay_cycles(1000);
    }

    return key_value; // 没有按下，返回0
}