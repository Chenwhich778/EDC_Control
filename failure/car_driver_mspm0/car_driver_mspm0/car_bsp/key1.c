#include "car_bsp.h"

int8_t EN = -1;
// 按键消抖时间（通常10ms~50ms）
#define KEY_DEBOUNCE_TIME_MS 20

// 按键状态变量
volatile uint32_t last_key_press_time = 0;
volatile uint8_t key_pressed_flag = 0;

void key1_IRQ_init(void) {
  NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN); // 开启按键引脚中断
}

/*GROUP1_IRQHandler包含所有GPIO外部触发中断*/
int motor1_enc_count = 0;
int motor2_enc_count = 0;
int motor3_enc_count = 0;
int motor4_enc_count = 0;
void GROUP1_IRQHandler(void) // Group1的中断服务函数
{
    uint32_t current_time = get_current_time_ms();
    uint32_t gpioB = DL_GPIO_getEnabledInterruptStatus(
        GPIOB, KEY1_PB5_PIN | MOTOR1_ENCA1_PIN | MOTOR2_ENCA2_PIN);

    // 按键处理（带消抖）
    if (((gpioB & KEY1_PB5_PIN) == KEY1_PB5_PIN) && (rKEY1 == 0)) {
        // 只有当按键按下时间超过消抖时间，并且之前没有按下标志时才处理
        if ((current_time - last_key_press_time) > KEY_DEBOUNCE_TIME_MS) {
            if (!key_pressed_flag) {
                EN = -EN; // 执行按键操作
                key_pressed_flag = 1; // 设置按键已按下标志
            }
        }
        last_key_press_time = current_time;
    } else {
        // 按键释放时清除标志
        key_pressed_flag = 0;
    }

    
  if (((gpioB & MOTOR1_ENCA1_PIN) == MOTOR1_ENCA1_PIN) && rMOTOR_ENCA1 == 0) {
    if (rMOTOR_ENCB1) {
      motor1_enc_count++;
    } else
      motor1_enc_count--;
  }
  if (((gpioB & MOTOR2_ENCA2_PIN) == MOTOR2_ENCA2_PIN) && rMOTOR_ENCA2 == 0) {
    if (rMOTOR_ENCB2) {
      motor2_enc_count++;
    } else
      motor2_enc_count--;
  }
}
