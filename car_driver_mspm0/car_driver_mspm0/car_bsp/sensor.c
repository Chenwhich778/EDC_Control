#include "sensor.h"
#include "ti_msp_dl_config.h"

// 根据实际接线定义（BABBABA分布）
const struct {
    GPIO_Regs* gpio;
    uint32_t pin;
} SENSOR_PINS[7] = {
    {GPIOB, GPIO_SENSOR_PIN_0_PIN},  // 第1路：B6
    {GPIOA, GPIO_SENSOR_PIN_1_PIN},  // 第2路：A9
    {GPIOB, GPIO_SENSOR_PIN_2_PIN},  // 第3路：B11
    {GPIOB, GPIO_SENSOR_PIN_3_PIN},  // 第4路：B3
    {GPIOA, GPIO_SENSOR_PIN_4_PIN},  // 第5路：A16
    {GPIOB, GPIO_SENSOR_PIN_5_PIN},  // 第6路：B16
    {GPIOA, GPIO_SENSOR_PIN_6_PIN}   // 第7路：A2
};

void Read_Grayscale(int8_t values[7]) {
    for (int i = 0; i < 7; i++) {
        // 逐个读取不同端口的引脚
        values[i] = DL_GPIO_readPins(SENSOR_PINS[i].gpio, SENSOR_PINS[i].pin)?0:1;
    }
}