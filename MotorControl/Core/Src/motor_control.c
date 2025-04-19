//rpm[0]为左轮，motor_left
//rpm[1]为右轮,motor_right

# include "motor_control.h"
# include "gpio.h"
# include "tim.h"
#include "stdlib.h"
#include "math.h"
/* USER CODE BEGIN ET */

uint8_t mode=0;

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
float target_rpm[2];    // 目标转速 0为左轮B，1为右轮A
uint32_t pwm_max = 1000;       // PWM最大值对应100%占空比
float correct[2]={0.0};
float pre_correct[2]={0.0};
uint8_t Direction[7]={0};
uint8_t Pre_Direction[7]={0};

char openmv_state='0';
char pre_openmv_state='0';
char k210_state='0';
char k210_resrved_state='1';
uint8_t turning_flag=0;
uint8_t turning_tmp=0;
uint8_t switch_count=0;
uint8_t unload=0;
uint8_t load=1;


// 左转控制变量
int32_t initial_right_encoder = 0;        // 右轮初始编码器值
int32_t target_pulse_right = 0;           // 右轮目标脉冲数
float TURN_DISTANCE_CM = 27.5f;     // 右轮90度转动距离
float accel_distance;  // 加速阶段脉冲数
float decel_distance;  // 减速阶段脉冲数
float cruise_distance; // 巡航阶段脉冲数
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
void Set_Left_Direction(uint8_t flag){
	if(flag==1){
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
	}
	if(flag==0){
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET);
	}
}

void Set_Right_Direction(uint8_t flag){
	if(flag==1){
		HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
	}
	if(flag==0){
		HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET);
	}
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
			
			output=-output;
			if(pid==&left_motor_pid)
			  Set_Left_Direction(0);
			else
				Set_Right_Direction(0);
		}
		else{
			if(pid==&left_motor_pid)
			  Set_Left_Direction(1);
			else
				Set_Right_Direction(1);
		}
    // 输出限幅
    if(output > pid->max_output) output = pid->max_output;
    
    return output;
}


void turn(float degrees){
    float pwm1 =  0 ;
    float pwm2 =  0 ;
    if ( degrees == 180.0) TURN_DISTANCE_CM = 27.5 * 2;
    
    if (turning_flag!=0) {
        // 1 初始化左转参数（仅首次触发时执行）
        if (target_pulse_right == 0) {
            // 停止左轮
            pwm1 = 0 ;
            
            // 计算右轮目标脉冲数（轮半径为3.4cm）
            float wheel_circumference = 2 * 3.1416f * 3.4f;
            float revolutions = TURN_DISTANCE_CM / wheel_circumference;
            target_pulse_right = (int32_t)(revolutions * 1500);
            
            // 记录初始编码器值
                    if (degrees == -90.0 ) initial_right_encoder = current_total[0];
              else initial_right_encoder = current_total[1];
            
            
            // 启动右轮
            pwm2 = 300.0f; // 右轮目标转速（RPM）
                    
                    // 初始化S曲线参数
            accel_distance = target_pulse_right * 0.5f;  // 前20%加速
            decel_distance = target_pulse_right * 0.5f; // 后20%减速
            cruise_distance = target_pulse_right * 0.2f; // 中间巡航
        }
             int32_t delta_pulse = 0 ;
         if (degrees == -90.0 ) delta_pulse = abs(current_total[0] - initial_right_encoder);
             else  delta_pulse = abs(current_total[1] - initial_right_encoder);
                
                // S曲线速度控制
        if (delta_pulse < accel_distance) {
            // 加速阶段：使用余弦曲线加速
            float progress = (float)delta_pulse / accel_distance;
            pwm2 = 500.0f * (0.5f - 0.5f * cosf(progress * M_PI));
                      // 确保最小PWM值
            if (pwm2 < 200.0f) pwm2 = 200.0f;
                    
        } 
        else if (delta_pulse > (accel_distance + cruise_distance)) {
            // 减速阶段：使用余弦曲线减速
            float progress = (float)(delta_pulse - (accel_distance + cruise_distance)) / decel_distance;
            pwm2 = 500.0f * (0.5f + 0.5f * cosf(progress * M_PI));
           // 确保最小PWM值
            if (pwm2 < 100.0f) pwm2 = 100.0f;
        }
        else {
            // 巡航阶段：保持最大速度
            pwm2 = 500.0f;
        }
                
        if (delta_pulse >= target_pulse_right) {
            // 停止右轮
            pwm2 = 0 ;
            // 重置状态
            target_pulse_right = 0;
					  turning_flag=0;
        }
    }
    
        if (degrees == 90.0 || degrees == 180.0)
        {
            pwm_left = pwm1 ;
            pwm_right = pwm2 ;
        }
        if (degrees == - 90.0)
        {
            pwm_left = pwm2 ;
            pwm_right = pwm1 ;
        }
        
}
/*void read_Direction_flag(uint8_t Direction[],uint8_t Pre_Direction[],uint8_t n){
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
}*/

//判断是否开始跑
uint8_t if_ready(){
	static uint8_t ready=0;
	if(k210_state>='1'&&k210_state<='8'&&unload==0&&load==0){
		ready=1;
		stop_flag=0;
	}
	if(openmv_state=='5'&&pre_openmv_state!='5'&&ready==1){
		switch_count++;
		if(switch_count==2){
				ready=0;
				k210_state='0';
				openmv_state='0';
				turning_flag=3;
				unload=1;
			  stop_flag=1;
		}
		else if(switch_count==4){
			if(turning_flag==0){
				ready=0;
				k210_state='0';
				openmv_state='0';
				unload=1;
				stop_flag=1;
			}
	  }
  }
	return ready;
}

//识别到十字后还要往前走一段
float get_more(uint8_t turning_tmp){
	static float more=0.0;
	if(turning_tmp==1)
	  more+=(rpm[0]+rpm[1]);
	else more=0;
	return more*SAMPLE_TIME*M_PI*3.2;
}
//寻红线
//1直行，2左转，3右转，4识别到十字
void track(){
	switch(openmv_state){
		case '1':{
			correct[0]=0.0;
			correct[1]=0.0;
			break;
		}
		case '2':{
			correct[0]=0.2*target_rpm[0];
			correct[1]=0.2*target_rpm[1];
			break;
		}
		case '3':{
			correct[0]=-0.2*target_rpm[0];
			correct[1]=-0.2*target_rpm[1];
			break;
		}
		ERROR:{
			correct[0]=0.0;
			correct[1]=0.0;
			break;
		}
	}
	
}
	

/*路线规划------------------------------------------------*/
//转弯后记得把turning_flag置零(1表示正在左转，2表示正在右转，3表示正在掉头）
void road_plan(){
	if(if_ready()==1){
		switch(mode){
			case 1:{
				if(switch_count==0||switch_count==1){//第一次经过虚线不停留
					if(openmv_state=='4'){
						turning_tmp=1;
					}
					if(turning_tmp==1&&get_more(turning_tmp)>=50.0){
						if(k210_resrved_state==1)
							turning_flag=1;
						else if(k210_resrved_state==2)
						  turning_flag=2;
						turning_tmp=0;
					}
					if(turning_flag==0)
						track();
					else {
						if(turning_flag==1)
						  turn(90.0);
						else if(turning_flag==2)
							turn(-90.0);
					}
				}
				else if(switch_count==2){//第二次识别虚线停在1号病房，ready后启动
					if(turning_flag==3){
						turn(180.0);
					}
					else{
						if(openmv_state=='4')
							turning_tmp=1;
						if(turning_tmp==1&&get_more(turning_tmp)>=50.0){
							if(k210_resrved_state==1)
							  turning_flag=2;
						  else if(k210_resrved_state==2)
						    turning_flag=1;
						  turning_tmp=0;
					  }
						if(turning_flag==0)
							track();
						else {
							if(turning_flag==1)
						    turn(90.0);
						  else if(turning_flag==2)
							  turn(-90.0);
						}
					}
				}
				else if(switch_count==3){
					if(turning_flag==3)
						turn(180.0);
					else {
						track();
						if(get_more(1)>=80.0){
						  turning_flag=3;
						  get_more(0);
					  }
					}
				}
				break;
			}
			case 2:{
				
				break;
			}
			case 3:{
				
				break;
			}
			ERROR :{
				
				break;
			}
		}
	}



















}
/*路线规划-------------------------------------------------*/
void motor_control_in_TIM1(){
	 // 读取当前编码器计数值
		if(TIM2->CR1&TIM_CR1_DIR){
			current_total[0]-=(65535-TIM2->CNT);
		  TIM2->CNT=65535;
		}
		else{
		  current_total[0]+=TIM2->CNT;
			TIM2->CNT=0;
		}
		if(TIM3->CR1&TIM_CR1_DIR){
			current_total[1]-=(65535-TIM3->CNT);
		  TIM3->CNT=65535;
		}
		else{
		  current_total[1]+=TIM3->CNT;
			TIM3->CNT=0;
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
		//read_Direction_flag(Direction,Pre_Direction,7);
		/*uint8_t current_state = Direction[0] | Direction[1] | Direction[2] | Direction[3] | Direction[4] | Direction[5] | Direction[6];
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
		}*/
		road_plan();
		// PID计算
		if(stop_flag==1){
			pwm_left=0;
			pwm_right=0;
		}
	  else if(turning_flag==0){
      pwm_left = PID_Compute(&left_motor_pid, (target_rpm[0]-correct[0]), rpm[0]);
		  pwm_right = PID_Compute(&right_motor_pid, (target_rpm[1]+correct[1]), rpm[1]);
		}
		// 更新PWM输出
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)pwm_left);
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