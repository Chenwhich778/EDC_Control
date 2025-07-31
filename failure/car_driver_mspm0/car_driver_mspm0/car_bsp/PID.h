
/*struct---------------------------*/
typedef struct {
    float Kp;           // ����ϵ��
    float Ki;           // ����ϵ��
    float Kd;           // ΢��ϵ��
    float Ts;           // ����ʱ��(s)
    float integral;      // ������
    float prev_error;    // �ϴ�����
    float max_output;    // �����޷�
    float max_integral;  // �����޷�
	float prev_d;
} PID_Controller;
/*struct--------------------------------*/
// #define SAMPLE_TIME 0.02
// #define M_PI 3.14159265358979323846
/*����-------------------------------------------*/

/*����-----------------------------------------------*/

/*����------------------------------------------------------------------------*/
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float Ts,float max);
float PID_Compute(PID_Controller *pid, float setpoint, float measurement);
//float PID_angle_Compute(PID_Controller *pid, float setpoint, float measurement);
/*����------------------------------------------------------------------------*/