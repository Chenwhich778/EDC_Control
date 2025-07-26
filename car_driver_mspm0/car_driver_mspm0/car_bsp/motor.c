
#include "car_bsp.h"

#define motor_load 1000


void motor_init(void)
{
    NVIC_EnableIRQ(TIMER_M4_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_M4_INST);

    NVIC_EnableIRQ(TIMER_M3_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_M3_INST);

    MOTOR1_CTRL1(0);
    MOTOR1_CTRL2(0);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_1_2_INST,0,GPIO_PWM_MOTOR_1_2_C0_IDX);

    MOTOR2_CTRL1(0);
    MOTOR2_CTRL2(0);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_1_2_INST,0,GPIO_PWM_MOTOR_1_2_C1_IDX);

    MOTOR3_CTRL1(0);
    MOTOR3_CTRL2(0);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_3_4_INST,0,GPIO_PWM_MOTOR_3_4_C0_IDX);

    MOTOR4_CTRL1(0);
    MOTOR4_CTRL2(0);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_3_4_INST,0,GPIO_PWM_MOTOR_3_4_C1_IDX);
}


void set_motor(int motor1,int motor2,int motor3,int motor4)
{
    uint8_t motor1_dir = (motor1 >= 0);
    uint8_t motor2_dir = (motor2 >= 0);
    uint8_t motor3_dir = (motor3 >= 0);
    uint8_t motor4_dir = (motor4 >= 0);

    MOTOR1_CTRL1(motor1_dir);
    MOTOR1_CTRL2(!motor1_dir);

    MOTOR2_CTRL1(motor2_dir);
    MOTOR2_CTRL2(!motor2_dir);

    MOTOR3_CTRL1(motor3_dir);
    MOTOR3_CTRL2(!motor3_dir);

    MOTOR4_CTRL1(motor4_dir);
    MOTOR4_CTRL2(!motor4_dir);

    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_1_2_INST,
        motor1_dir ? (motor_load - motor1) : (1000 + motor1),
        GPIO_PWM_MOTOR_1_2_C0_IDX);

    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_1_2_INST,
        motor2_dir ? (motor_load - motor2) : (1000 + motor2),
        GPIO_PWM_MOTOR_1_2_C1_IDX);

    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_3_4_INST,
        motor3_dir ? (motor_load - motor3) : (1000 + motor3),
        GPIO_PWM_MOTOR_3_4_C0_IDX);

    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_3_4_INST,
        motor4_dir ? (motor_load - motor4) : (1000 + motor4),
        GPIO_PWM_MOTOR_3_4_C1_IDX);
}




uint32_t M4_count=0;
uint32_t M3_count=0;
void TIMER_M4_INST_IRQHandler(void) 
{
  switch (DL_TimerA_getPendingInterrupt(TIMER_M4_INST)) 
  {
  case DL_TIMERA_IIDX_LOAD:
    if (!DL_GPIO_readPins(MOTOR4_ENCA4_PORT, MOTOR4_ENCA4_PIN)) 
    {
    // 反转
        M4_count++;
    } 
    else 
    {
    // 正转
        M4_count--;
    }
    break;
  default:
    break;
  }
}

void TIMER_M3_INST_IRQHandler(void) 
{
  switch (DL_TimerG_getPendingInterrupt(TIMER_M3_INST)) 
  {
  case DL_TIMERG_IIDX_LOAD:
    if (!DL_GPIO_readPins(MOTOR3_ENCB3_PORT, MOTOR3_ENCB3_PIN))
    {
    // 反转
        M3_count++;
    } 
    else 
    {
    // 正转
        M3_count--;
    }
    break;
  default:
    break;
  }
}

uint32_t get_motor_enc_count(uint8_t swi)
{
    switch (swi) {
        case 3: return M3_count;
        case 4: return M4_count;
        default :return 0;
    }
}



void stop_all_motors(void) {
    // 关闭所有方向控制信号
    MOTOR1_CTRL1(0); MOTOR1_CTRL2(0);
    MOTOR2_CTRL1(0); MOTOR2_CTRL2(0);
    MOTOR3_CTRL1(0); MOTOR3_CTRL2(0);
    MOTOR4_CTRL1(0); MOTOR4_CTRL2(0);
    
    // 设置所有PWM占空比为0
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_1_2_INST, 0, GPIO_PWM_MOTOR_1_2_C0_IDX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_1_2_INST, 0, GPIO_PWM_MOTOR_1_2_C1_IDX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_3_4_INST, 0, GPIO_PWM_MOTOR_3_4_C0_IDX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_3_4_INST, 0, GPIO_PWM_MOTOR_3_4_C1_IDX);
}