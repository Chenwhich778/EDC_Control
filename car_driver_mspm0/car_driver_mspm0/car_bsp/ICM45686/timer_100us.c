#include "../car_bsp.h"
uint32_t nowtime = 0;  // 100us计数
volatile uint32_t ms_count = 0; // ms计数
uint32_t g12time = 0;
uint32_t test = 0;
extern PID_Controller pid_left;
extern PID_Controller pid_right;
extern float left_target;
extern float right_target;
extern float speed4;
extern  float speed3;
extern uint8_t turn_flag;
void TIMER_A0_100us_INST_IRQHandler(void)
{
    switch( DL_TimerA_getPendingInterrupt(TIMER_A0_100us_INST) )
    {
        case DL_TIMER_IIDX_ZERO://如果是0溢出中断
        
            nowtime++;
            // 每10个100us就是1ms (1000us)
            if(nowtime % 10 == 0) {
                ms_count++;
            }
            
            if (nowtime%100==0) {
               if (EN == 1) {
            // 使用PID控制电机
            float output_left = PID_Compute(&pid_left, left_target, speed3);
            float output_right = PID_Compute(&pid_right, right_target, speed4);
            // float output_left = PID_Compute(&pid_left, 100, speed3);
            // float output_right = PID_Compute(&pid_right, 100, speed4);
            
            // 设置电机输出
            set_motor(1, 1, output_left, output_right);
            
            
        } else {
            stop_all_motors();
        }
            }
            break;
        default://其他的定时器中断
            break;
    }
}



uint32_t get_current_time_ms(void)
{
    return ms_count;
}