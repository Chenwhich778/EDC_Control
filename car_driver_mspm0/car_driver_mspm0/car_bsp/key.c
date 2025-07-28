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
        delay_us(100);
        DL_GPIO_setPins(SENSOR_PINS[i].gpio, SENSOR_PINS[i].pin);
        DL_GPIO_clearPins(SENSOR_PINS[(i+1)%4].gpio, SENSOR_PINS[(i+1)%4].pin);
        DL_GPIO_clearPins(SENSOR_PINS[(i+2)%4].gpio, SENSOR_PINS[(i+2)%4].pin);
        DL_GPIO_clearPins(SENSOR_PINS[(i+3)%4].gpio, SENSOR_PINS[(i+3)%4].pin);

         delay_ms(3);

        for (j = 0; j < 4; j++)
        {
                if (DL_GPIO_readPins(SENSOR_PINS[j+4].gpio, SENSOR_PINS[j+4].pin) != 0){
                    key_value = i * 4 + j + 1;
                }
             delay_cycles(1000);
        }

         delay_cycles(1000);
    }

    return key_value; // 没有按下，返回0
}
char get_keychar(uint8_t keyboard){
    static char input='0';
    switch (keyboard) {
            case KEY_0:input='0';break;
            case KEY_1:input='1';break;
            case KEY_2:input='2';break;
            case KEY_3:input='3';break;
            case KEY_4:input='4';break;
            case KEY_5:input='5';break;
            case KEY_6:input='6';break;
            case KEY_7:input='7';break;
            case KEY_8:input='8';break;
            case KEY_A:input='A';break;
            case KEY_B:input='B';break;
            case KEY_C:input='C';break;
            case KEY_D:input='D';break;
            case KEY_xin:input='*';break;
            case KEY_jing:input='#';break;
            default:break;
        }
    return input;
}