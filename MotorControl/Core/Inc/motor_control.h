
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

typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float Ts;           // 采样时间(s)
    float integral;      // 积分项
    float max_output;    // 输出限幅
    float max_integral;  // 积分限幅
	  float prev_d;
} PID_Correct;
/*struct--------------------------------*/
#define SAMPLE_TIME 0.01
/*变量-------------------------------------------*/
extern float duty[2];
extern int32_t current_total[2];
extern int32_t pre_total[2];
extern float rpm[2];
extern float pre_rpm[2];
extern float pwm_left;
extern float pwm_right;
extern uint8_t stop_flag;
extern uint32_t sensor_time;
extern uint32_t pre_sensor_time;
extern uint32_t un_sensor_time;
extern PID_Controller left_motor_pid;     // PID控制器实例
extern PID_Controller right_motor_pid;     // PID控制器实例
extern PID_Controller angle_pid;
extern PID_Correct correctl_pid;    //修正实例
extern PID_Correct correctr_pid;    //修正实例
extern float target_rpm[2];    // 目标转速 0为左轮B，1为右轮A
extern uint32_t pwm_max;       // PWM最大值对应100%占空比
extern float speed_time;//加速用时
extern float straight_error;
extern float pre_straight_error;
extern float correct[2];
extern float pre_correct[2];
extern uint8_t Direction[7];
extern uint8_t Pre_Direction[7];
/*变量-----------------------------------------------*/

/*函数------------------------------------------------------------------------*/
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float Ts,float max);
float PID_Compute(PID_Controller *pid, float setpoint, float measurement);
void PIDC_Init(PID_Correct *pid, float Kp, float Ki, float Kd, float Ts);
float PIDC_Compute(PID_Correct *pid, float rpm[],float pre_rpm[]);
void read_Direction_flag(uint8_t Direction[],uint8_t Pre_Direction[],uint8_t n);
void road_plan();
void motor_control_in_TIM1();
float PID_angle_Compute(PID_Controller *pid, float setpoint, float measurement);
/*函数------------------------------------------------------------------------*/