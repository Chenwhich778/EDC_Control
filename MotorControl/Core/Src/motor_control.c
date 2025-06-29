//rpm[0]为左轮，motor_left
//rpm[1]为右轮,motor_right

# include "motor_control.h"
# include "gpio.h"
# include "tim.h"
#include "stdlib.h"
#include "math.h"
/* USER CODE BEGIN ET */


/* PID初始化 --------------------------------------------------------*/
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float Ts,float max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Ts = Ts;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->max_output = max;
    pid->max_integral = pid->max_output * 0.5f; // 积分限幅为输出的50%
	  pid->prev_d=0.0f;
}
    
/* PID计算（带抗饱和和滤波）-----------------------------------------*/
float PID_Compute(PID_Controller *pid, float setpoint, float measurement) {
    // 计算误差
    float error = setpoint - measurement;
    // 比例项
    float P = pid->Kp * error;
    
    // 积分项（带限幅）
    pid->integral += error * pid->Ts;
    if(pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if(pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    float I = pid->Ki * pid->integral;
    
    // 微分项（带一阶低通滤波）
    float D = pid->Kd * (error - pid->prev_error) / pid->Ts;
    D = 0.2f * D + 0.8f * pid->prev_d; // 低通滤波系数
    pid->prev_d = D;
    pid->prev_error = error;
    
    // 计算输出
    float output = P + I + D;
    if(output<0.0){
			output=0.0;
		}
    // 输出限幅
    if(output > pid->max_output) output = pid->max_output;
    
    return output;
}

void motor_control_in_TIM1(){
	
}
