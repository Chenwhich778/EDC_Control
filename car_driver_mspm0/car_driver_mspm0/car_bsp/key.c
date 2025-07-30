#include "key.h"
#include "car_bsp/car_bsp.h"
#include <string.h>  // 添加string.h用于memcpy

#define KEY_BUFFER_SIZE 64
#define DEBOUNCE_TIME_MS 20  // 消抖时间20ms

char key_buffer[KEY_BUFFER_SIZE];
uint16_t key_index = 0;
uint16_t count = 0;
bool load_flag = false;
extern uint16_t xx;

// 按键消抖相关变量
static uint32_t last_key_time = 0;
static uint8_t last_key_value = 0;
static bool key_stable = false;

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

// 带消抖的按键读取
int getKeyValue(void)
{
    int i, j = 0;
    int key_value = 0;
    uint32_t current_time = get_current_time_ms();
    
    for (i = 0; i < 4; i++) {
        delay_us(100);
        DL_GPIO_setPins(SENSOR_PINS[i].gpio, SENSOR_PINS[i].pin);
        DL_GPIO_clearPins(SENSOR_PINS[(i+1)%4].gpio, SENSOR_PINS[(i+1)%4].pin);
        DL_GPIO_clearPins(SENSOR_PINS[(i+2)%4].gpio, SENSOR_PINS[(i+2)%4].pin);
        DL_GPIO_clearPins(SENSOR_PINS[(i+3)%4].gpio, SENSOR_PINS[(i+3)%4].pin);

        delay_ms(3);

        for (j = 0; j < 4; j++) {
            if (DL_GPIO_readPins(SENSOR_PINS[j+4].gpio, SENSOR_PINS[j+4].pin) != 0) {
                int new_key = i * 4 + j + 1;
                
                // 消抖逻辑
                if (new_key != last_key_value) {
                    last_key_value = new_key;
                    last_key_time = current_time;
                    key_stable = false;
                } else if ((current_time - last_key_time) > DEBOUNCE_TIME_MS) {
                    if (!key_stable) {
                        key_stable = true;
                        key_value = new_key;
                    }
                }
            }
            delay_cycles(1000);
        }
        delay_cycles(1000);
    }

    return key_value; // 没有按下，返回0
}

char get_keychar(uint8_t keyboard) {
    static char input = '0';
    uint32_t current_time = get_current_time_ms();
    
    // 只有按键稳定时才处理
    if ((current_time - last_key_time) < DEBOUNCE_TIME_MS || !key_stable) {
        return 0;
    }

    switch (keyboard) {
        case KEY_0: input = '0'; break;
        case KEY_1: input = '1'; break;
        case KEY_2: input = '2'; break;
        case KEY_3: input = '3'; break;
        case KEY_4: input = '4'; break;
        case KEY_5: input = '5'; break;
        case KEY_6: input = '6'; break;
        case KEY_7: input = '7'; break;
        case KEY_8: input = '8'; break;
        case KEY_9: input = '9'; break;
        case KEY_A: input = 'A'; break;
        case KEY_B: input = 'B'; break;
        case KEY_C: input = 'C'; break;
        case KEY_D: input = 'D'; break;
        case KEY_xin: input = '*'; break;
        case KEY_jing: input = '#'; break;
        default: return 0;
    }

    // 边界检查
    if (key_index >= KEY_BUFFER_SIZE - 1) {
        key_index = 0;  // 缓冲区满时重置
    }

    if (input == 'D') { // delete
        if (key_index > 0) key_index--;
    }
    else if (input == 'B') { // go back
        if (key_index < KEY_BUFFER_SIZE - 1) key_index++;
    }
    else if (input == '#') {
        key_buffer[key_index] = '\0';  // 修正终止符
        count = key_index;
        key_index = 0;
        load_flag = true;
    }
    else {
        key_buffer[key_index++] = input;
    }

    return input;
}

void load_input(void) {
    if (!load_flag) return;
    
    if (count < 5) {
        load_flag = false;
        return;
    }

    // 验证格式
    if (key_buffer[2] != 'A' && key_buffer[2] != 'C') {
        load_flag = false;
        return;
    }

    uint16_t head_index = (key_buffer[0]-'0')*10 + (key_buffer[1]-'0');
    char tmp[32];
    
    // 安全复制
    uint16_t copy_len = (count-4) < 31 ? (count-4) : 31;
    memcpy(tmp, &key_buffer[4], copy_len);
    tmp[copy_len] = '\0';

    if (key_buffer[2] == 'A') {  // 整数
        int content_d = atoi(tmp);
        if (key_buffer[3] == 'C') {  // 负数
            content_d = -content_d;
        }
        xx = (uint16_t)content_d;
    }
    else if (key_buffer[2] == 'C') {  // 浮点数
        float content_f = atof(tmp);
        if (key_buffer[3] == 'C') {  // 负数
            content_f = -content_f;
        }
        // 这里可以添加浮点数处理逻辑
    }

    load_flag = false;
}