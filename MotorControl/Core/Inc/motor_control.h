
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
extern uint8_t mode;
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
extern float target_rpm[2];    // 目标转速 0为左轮B，1为右轮A
extern uint32_t pwm_max;       // PWM最大值对应100%占空比
extern float correct[2];
extern float pre_correct[2];
extern uint8_t Direction[7];
extern uint8_t Pre_Direction[7];
extern char openmv_state;
extern char pre_openmv_state;
extern char k210_state;
extern char k210_reserved_state;
extern uint8_t turning_flag;
extern uint8_t turning_tmp;
extern uint8_t switch_count;
extern uint8_t crossing_count;
extern uint8_t unload;
extern uint8_t load;
extern uint8_t judge;

// 转弯控制变量
extern int32_t initial_right_encoder;        // 右轮初始编码器值
extern int32_t target_pulse;           // 右轮目标脉冲数
extern float TURN_DISTANCE_CM;     // 右轮90度转动距离
extern float accel_distance;  // 加速阶段脉冲数
extern float decel_distance;  // 减速阶段脉冲数
extern float cruise_distance; // 巡航阶段脉冲数

extern uint8_t alarm_enable;//alarm-------------------*/
extern uint8_t beep_time;
extern uint32_t origin_count;
extern float PULSE_PER_REV[2];
/*变量-----------------------------------------------*/

/*函数------------------------------------------------------------------------*/
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float Ts,float max);
float PID_Compute(PID_Controller *pid, float setpoint, float measurement);
//void read_Direction_flag(uint8_t Direction[],uint8_t Pre_Direction[],uint8_t n);
void Set_Left_Direction(uint8_t flag);
void Set_Right_Direction(uint8_t flag);
void turn(float degrees);
void turn180();
float get_distance();
void receive_k210_state();
uint8_t judge_k210_state();
uint8_t if_ready();
void track();
void road_plan();
void motor_control_in_TIM1();
//float PID_angle_Compute(PID_Controller *pid, float setpoint, float measurement);
/*函数------------------------------------------------------------------------*/