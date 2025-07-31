
# include "PID.h"
/* USER CODE BEGIN ET */


/* PID��ʼ�� --------------------------------------------------------*/
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float Ts,float max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Ts = Ts;
    pid->integral = 75.0f;
    pid->prev_error = 0.0f;
    pid->max_output = max;
    pid->max_integral = pid->max_output*0.5; // �����޷�Ϊ������50%
	pid->prev_d=0.0f;
}
    
/* PID���㣨�������ͺ��˲���-----------------------------------------*/
float PID_Compute(PID_Controller *pid, float setpoint, float measurement) {
    // ��������
    float error = setpoint - measurement;
    // ������
    float P = pid->Kp * error;
    
    // ��������޷���
    pid->integral += error * pid->Ts;
    if(pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if(pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    float I = pid->Ki * pid->integral;
    
    // ΢�����һ�׵�ͨ�˲���
    float D = pid->Kd * (error - pid->prev_error) / pid->Ts;
    D = 0.2f * D + 0.8f * pid->prev_d; // ��ͨ�˲�ϵ��
    pid->prev_d = D;
    pid->prev_error = error;
    
    // ��������
    float output = P + I + D;
    if(output<0.0){
			output=0.0;
		}
    // �����޷�
    if(output > pid->max_output) output = pid->max_output;

    
    if (output==0) {
        output=1;
    }
    return output;
}
