#include "sensor.h"
#include "ti_msp_dl_config.h"

// 根据实际接线定义（BABBABA分布）
const struct {
    GPIO_Regs* gpio;
    uint32_t pin;
} SENSOR_PINS[7] = {
    {GPIOB, DL_GPIO_PIN_0},  // 第1路：B6
    {GPIOA, DL_GPIO_PIN_1},  // 第2路：A9
    {GPIOB, DL_GPIO_PIN_2},  // 第3路：B11
    {GPIOB, DL_GPIO_PIN_3},  // 第4路：B3
    {GPIOA, DL_GPIO_PIN_4},  // 第5路：A16
    {GPIOB, DL_GPIO_PIN_5},  // 第6路：B16
    {GPIOA, DL_GPIO_PIN_6}   // 第7路：A2
};

void Read_Grayscale(uint8_t values[7]) {
    for (int i = 0; i < 7; i++) {
        // 逐个读取不同端口的引脚
        values[i] = DL_GPIO_readPins(SENSOR_PINS[i].gpio, SENSOR_PINS[i].pin) ? 0 : 1;
    }
}