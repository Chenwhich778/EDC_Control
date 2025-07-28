#include "car_bsp.h"

int8_t EN = -1;

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
  uint32_t gpioB = DL_GPIO_getEnabledInterruptStatus(
      GPIOB, KEY1_PB5_PIN | MOTOR1_ENCA1_PIN | MOTOR2_ENCA2_PIN);

    if (((gpioB & KEY1_PB5_PIN) == KEY1_PB5_PIN) && rKEY1 == 0) {
            EN = 1;
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
