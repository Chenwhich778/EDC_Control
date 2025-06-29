
#include "stdint.h"
#include "angle.h"
/*struct---------------------------*/
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float Ts;           // 采样时间(s)
    float integral;      // 积分项
    float prev_error;    // 上次误差
    float max_output;    // 输出限幅
    float max_integral;  // 积分限幅
	  float prev_d;
} PID_Controller;
/*struct--------------------------------*/
#define SAMPLE_TIME 0.02
#define M_PI 3.14159265358979323846
/*变量-------------------------------------------*/

/*变量-----------------------------------------------*/

/*函数------------------------------------------------------------------------*/
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float Ts,float max);
float PID_Compute(PID_Controller *pid, float setpoint, float measurement);
void motor_control_in_TIM1();
//float PID_angle_Compute(PID_Controller *pid, float setpoint, float measurement);
/*函数------------------------------------------------------------------------*/