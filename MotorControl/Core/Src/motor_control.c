//rpm[0]为左轮，motor_left
//rpm[1]为右轮,motor_right

# include "motor_control.h"
# include "gpio.h"
# include "tim.h"

/* USER CODE BEGIN ET */



float duty[2];//--------------------------转速相关*/
int32_t current_total[2];
int32_t pre_total[2];
float rpm[2];
float pre_rpm[2];

float pwm_left;
float pwm_right;

uint8_t stop_flag=1;//-------------------停车 */

uint32_t sensor_time;//-------------------灰度传感器感应时间*/
uint32_t pre_sensor_time;
uint32_t un_sensor_time;

PID_Controller left_motor_pid;     // PID控制器实例
PID_Controller right_motor_pid;     // PID控制器实例
PID_Controller angle_pid;
PID_Correct correctl_pid;    //修正实例
PID_Correct correctr_pid;    //修正实例
float target_rpm[2];    // 目标转速 0为左轮B，1为右轮A
uint32_t pwm_max = 1000;       // PWM最大值对应100%占空比

float straight_error=0.0f;
float pre_straight_error=0.0f;
float correct[2]={0.0};
float pre_correct[2]={0.0};
uint8_t Direction[7]={0};
uint8_t Pre_Direction[7]={0};

uint8_t Number;//识别的数字
uint8_t line_flag=0;//识别到停止线

/* USER CODE END ET */

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
    
    // 输出限幅
    if(output > pid->max_output) output = pid->max_output;
    else if(output < 0.0f) output = 0.0f;
    
    return output;
}

/* correct初始化 --------------------------------------------------------*/
void PIDC_Init(PID_Correct *pid, float Kp, float Ki, float Kd, float Ts) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Ts = Ts;
    pid->integral = 0.0f;
    pid->max_output = 200.0f;
    pid->max_integral = pid->max_output * 0.5f; // 积分限幅为输出的50%
	  pid->prev_d=0.0f;
}
/* correct计算（带抗饱和和滤波）-----------------------------------------*/
float PIDC_Compute(PID_Correct *pid, float rpm[],float pre_rpm[]) {
	  if((rpm[0]+pre_rpm[0]-2*target_rpm[0])<-2.0f||(rpm[0]+pre_rpm[0]-2*target_rpm[0])>2.0f)
			 return 0;
    //计算correct
		straight_error=(rpm[0]+pre_rpm[0])/2*pid->Ts-(rpm[1]+pre_rpm[1])/2*pid->Ts;
    
    // 比例项
    float P = pid->Kp * straight_error;
    
    // 积分项（带限幅）
    pid->integral += straight_error * pid->Ts;
    if(pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if(pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    float I = pid->Ki * pid->integral;
    
    // 微分项（带一阶低通滤波）
    float D = pid->Kd * (straight_error - pre_straight_error) / pid->Ts;
    D = 0.2f * D + 0.8f * pid->prev_d; // 低通滤波系数
    pid->prev_d = D;
    pre_straight_error = straight_error;
    
    // 计算输出
    float output = P + I + D;
    
    // 输出限幅
    if(output > pid->max_output) output = pid->max_output;
    else if(output < 0.0f) output = 0.0f;
    
    return output;
}


void read_Direction_flag(uint8_t Direction[],uint8_t Pre_Direction[],uint8_t n){
	for(int i=0;i<n;i++)
		  Pre_Direction[i]=Direction[i];
		if(HAL_GPIO_ReadPin(Direction_6_GPIO_Port,Direction_6_Pin)==GPIO_PIN_SET)//D_6
			Direction[6]=1;
		else
			Direction[6]=0;
		if(HAL_GPIO_ReadPin(Direction_5_GPIO_Port,Direction_5_Pin)==GPIO_PIN_SET)//5
			Direction[5]=1;
		else
			Direction[5]=0;
		if(HAL_GPIO_ReadPin(Direction_4_GPIO_Port,Direction_4_Pin)==GPIO_PIN_SET)//4
			Direction[4]=1;
		else
			Direction[4]=0;
		if(HAL_GPIO_ReadPin(Direction_3_GPIO_Port,Direction_3_Pin)==GPIO_PIN_SET)//3
			Direction[3]=1;
		else
			Direction[3]=0;
		if(HAL_GPIO_ReadPin(Direction_2_GPIO_Port,Direction_2_Pin)==GPIO_PIN_SET)//2
			Direction[2]=1;
		else
			Direction[2]=0;
		if(HAL_GPIO_ReadPin(Direction_1_GPIO_Port,Direction_1_Pin)==GPIO_PIN_SET)//1
			Direction[1]=1;
		else
			Direction[1]=0;
		if(HAL_GPIO_ReadPin(Direction_0_GPIO_Port,Direction_0_Pin)==GPIO_PIN_SET)//0
			Direction[0]=1;
		else
			Direction[0]=0;
}
/*路线规划------------------------------------------------*/
void road_plan(){
	



















}
/*路线规划-------------------------------------------------*/
void motor_control_in_TIM1(){
	 // 读取当前编码器计数值
		if((TIM3->CR1 & TIM_CR1_DIR)&&(TIM2->CR1 & TIM_CR1_DIR)){
			current_total[0]-=(65535-TIM3->CNT);
		  TIM3->CNT=65535;
			current_total[1]-=(65535-TIM2->CNT);
		  TIM2->CNT=65535;
		}
		else{
		  current_total[0]+=TIM3->CNT;
			TIM3->CNT=0;
			current_total[1]+=TIM2->CNT;
			TIM2->CNT=0;
		}
	// 计算增量（处理溢出）
    int32_t delta_left = current_total[0]-pre_total[0];
		int32_t delta_right = current_total[1]-pre_total[1];
	  
    // 计算转速（单位：RPM）
    // 假设编码器为4线正交编码，每转产生N个脉冲
    float PULSE_PER_REV[2];
		PULSE_PER_REV[0]=1500;
		PULSE_PER_REV[1]=1500;
			 // 根据实际编码器参数修改）
    
    rpm[0] = (delta_left / PULSE_PER_REV[0]) * (60.0f / SAMPLE_TIME);
		rpm[1] = (delta_right / PULSE_PER_REV[1]) * (60.0f / SAMPLE_TIME);
    
    // 更新上一次计数值
    pre_total[0] =current_total[0];
		pre_total[1] =current_total[1];
		
    //读取灰度传感器
		read_Direction_flag(Direction,Pre_Direction,7);
		uint8_t current_state = Direction[0] | Direction[1] | Direction[2] | Direction[3] | Direction[4] | Direction[5] | Direction[6];
    uint8_t previous_state = Pre_Direction[0] | Pre_Direction[1] | Pre_Direction[2] | Pre_Direction[3] | Pre_Direction[4] | Pre_Direction[5] | Pre_Direction[6];
    if(current_state==1){
			pre_sensor_time=sensor_time;
			sensor_time++;
			un_sensor_time=0;
		}
		else{
			pre_sensor_time=sensor_time;
			sensor_time=0;
			un_sensor_time++;
		}
		  
		// PID计算
	  if(stop_flag==0){
      pwm_left = PID_Compute(&left_motor_pid, (target_rpm[0]-correct[0]), rpm[0]);
		  pwm_right = PID_Compute(&right_motor_pid, (target_rpm[1]+correct[1]), rpm[1]);
		}
		else{
			pwm_left=0;
			pwm_right=0;
		}
		// 更新PWM输出
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, (uint32_t)pwm_left);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, (uint32_t)pwm_right);
		pre_rpm[0]=rpm[0];
		pre_rpm[1]=rpm[1];
}

//angle_pid计算
float PID_angle_Compute(PID_Controller *pid, float setpoint, float measurement) {
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
    
    return output;
}