#include "balance.h"
#include <math.h>
#include <stdbool.h>
#include <float.h>

// �������ĵ�
const float forbidden[3][2] = {{1.0, 2.0}, {2.0, 2.0}, {3.0, 2.0}};
const float forbidden_radius = 0.2f; // 20cm

// Ԥ����˾������� (����ʵ����������)
float PRESET_ENEMY_X = 2.9f; // ��һ��Ŀ���
float PRESET_ENEMY_Y = 1.0f;
float PRESET_ENEMY_X2 = 1.0f; // �ڶ���Ŀ���
float PRESET_ENEMY_Y2 = 3.0f;

//// Ԥ����˾������� (����ʵ����������)
//float PRESET_ENEMY_X = 1.9f;
//float PRESET_ENEMY_Y = 1.5f;

int test_num1 = 0; // ���Ա��������ڵ���

// �жϵ�(x, y)�Ƿ�����һ�����뾶��
bool is_in_forbidden(float x, float y)
{
	for (int i = 0; i < 3; ++i)
	{
		float dx = x - forbidden[i][0];
		float dy = y - forbidden[i][1];
		if (sqrtf(dx * dx + dy * dy) < forbidden_radius)
			return true;
	}
	return false;
}

// ·����ṹ
typedef struct
{
	float x;
	float y;
} Waypoint;

// ·���滮���
typedef struct
{
	Waypoint points[3]; // ���3��·����
	int count;			// ʵ��·��������
} Path;


#define SAFE_DIST 0.25f
#define SAFE_DIST_SQ (SAFE_DIST * SAFE_DIST)


// �����������ж��߶��Ƿ�����һ����Բ�ཻ���ϸ���ϣ�
bool is_segment_safe(float x1, float y1, float x2, float y2) {
	extern const float forbidden[3][2];
	extern const float forbidden_radius;
	for (int i = 0; i < 3; i++) {
		float cx = forbidden[i][0];
		float cy = forbidden[i][1];
		float dx = x2 - x1;
		float dy = y2 - y1;
		float fx = x1 - cx;
		float fy = y1 - cy;
		float a = dx * dx + dy * dy;
		float b = 2 * (fx * dx + fy * dy);
		float c = fx * fx + fy * fy - (forbidden_radius + 0.05f) * (forbidden_radius + 0.05f); // �Ӱ�ȫԣ��
		float discriminant = b * b - 4 * a * c;
		if (discriminant < 0) continue; // ���ཻ
		float sqrt_disc = sqrtf(discriminant);
		float t1 = (-b - sqrt_disc) / (2 * a);
		float t2 = (-b + sqrt_disc) / (2 * a);
		if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)) {
			return false; // �н��㣬��Խ����
		}
	}
	return true;
}


Path plan_path(float start_x, float start_y, float end_x, float end_y) {
	Path path = {.count = 0};
	const float candidates[4] = {0.5f, 1.5f, 2.5f, 3.5f}; // �̶���ѡ�м��

	// ����ֱ������·����ˮƽ->��ֱ��
	if (is_segment_safe(start_x, start_y, end_x, start_y) &&
		is_segment_safe(end_x, start_y, end_x, end_y)) {
		path.points[path.count++] = (Waypoint){end_x, start_y};
		path.points[path.count++] = (Waypoint){end_x, end_y};
		return path;
	}

	// ��������·����ˮƽ->��ֱ->ˮƽ��
	float best_mid_x = 0.0f;
	float min_distance = FLT_MAX;
	bool found_safe_path = false;

	// ���������������ȼ���ѡ��
	for (int i = 0; i < 4; i++) {
		float mid_x = candidates[i];
		float distance = fabsf(mid_x - end_x);

		// �������·���İ�ȫ��
		bool safe1 = is_segment_safe(start_x, start_y, mid_x, start_y);
		bool safe2 = is_segment_safe(mid_x, start_y, mid_x, end_y);
		bool safe3 = is_segment_safe(mid_x, end_y, end_x, end_y);

		if (safe1 && safe2 && safe3 && distance < min_distance) {
			best_mid_x = mid_x;
			min_distance = distance;
			found_safe_path = true;
		}
	}

	// �ҵ���ȫ·���򹹽�����·��
	if (found_safe_path) {
		path.points[path.count++] = (Waypoint){best_mid_x, start_y}; // ˮƽ�ƶ����м��
		path.points[path.count++] = (Waypoint){best_mid_x, end_y};   // ��ֱ�ƶ���Ŀ��߶�
		path.points[path.count++] = (Waypoint){end_x, end_y};       // ˮƽ�ƶ����յ�
	}

	return path; // ��δ�ҵ���ȫ·�������ؿ�·��
}



// ����Ƿ񵽴�Ŀ���
bool reached_target(float current_x, float current_y, float target_x, float target_y)
{
	const float TOLERANCE = 0.28f; // 10cm�ݲ�
	return fabsf(current_x - target_x) < TOLERANCE &&
		   fabsf(current_y - target_y) < TOLERANCE;
}


// int Time_count=0; //Time variable //��ʱ����

u32 Buzzer_count1 = 0;

// Robot mode is wrong to detect flag bits
// ������ģʽ�Ƿ��������־λ
int robot_mode_check_flag = 0;

short test_num;
u8 command_lost_count = 0; //

Encoder OriginalEncoder; // Encoder raw data //������ԭʼ����

//========== PWM���ʹ�ñ��� ==========//
u8 start_check_flag = 0; // ����Ƿ���Ҫ���PWM
u8 wait_clear_times = 0;
u8 start_clear = 0;		// ��ǿ�ʼ���PWM
u8 clear_done_once = 0; // �����ɱ�־λ
u16 clear_again_times = 0;
float debug_show_diff = 0;
void auto_pwm_clear(void);
volatile u8 clear_state = 0x00;
/*------------------------------------*/

uint32_t LineDiffParam = 50; // ��ƫϵ��  everloss......change

static uint8_t FlashParam_Save(uint8_t *flag)
{
	u8 check = 0;

	if (*flag == 1)
	{
		*flag = 0;

		Set_Pwm(0, 0, 0, 0, 0);

		check = 1;
		taskENTER_CRITICAL(); // ����FLash�����ٽ磬��֤���ݰ�ȫ

		int32_t buf[4] = {0};
		buf[0] = *((int32_t *)&RC_Velocity);
		buf[1] = *((int32_t *)&Velocity_KP);
		buf[2] = *((int32_t *)&Velocity_KI);
		buf[3] = LineDiffParam;
		check += Write_Flash((u32 *)buf, 4);

		taskEXIT_CRITICAL(); // �˳��ٽ�

		// ��ȫ��д��ɹ�,check==1
	}

	return check;
}

void FlashParam_Read(void)
{
	int read;
	read = Read_Flash(0); // ��ȡ�±�Ϊ0������
	if (read != 0xffffffff)
		RC_Velocity = *((float *)&read);

	read = Read_Flash(1); // ��ȡ�±�Ϊ1������
	if (read != 0xffffffff)
		Velocity_KP = *((float *)&read);

	read = Read_Flash(2); // ��ȡ�±�Ϊ2������
	if (read != 0xffffffff)
		Velocity_KI = *((float *)&read);

	read = Read_Flash(3);
	if (read != 0xffffffff)
		LineDiffParam = read;

	// �쳣�ٶ�����,����
	if (RC_Velocity < 0 || RC_Velocity > 10000)
		RC_Velocity = 500;

	// ��ƫϵ���쳣,����
	if (LineDiffParam > 100)
		LineDiffParam = 50;
}

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
�������ܣ��˶�ѧ��⣬��������Ŀ���ٶȼ��������Ŀ��ת��
��ڲ�����X��Y��Z�᷽���Ŀ���˶��ٶ�
����  ֵ����
**************************************************************************/
// �����ƫϵ��
static float wheelCoefficient(uint32_t diffparam, uint8_t isLeftWheel)
{
	if (1 == isLeftWheel) // ���־�ƫ,��Ӧ50~100��Ӧ1.0~1.2���ľ�ƫϵ��
	{
		if (diffparam >= 50)
			return 1.0f + 0.006f * (diffparam - 50);
	}
	else // ���־�ƫ,50~0��Ӧ1.0~1.2���ľ�ƫϵ��
	{
		if (diffparam <= 50)
			return 1.0f + 0.004f * (50 - diffparam);
	}

	return 1.0f; // ����������ʱ,Ĭ����1.
}

void Drive_Motor(float Vx, float Vy, float Vz)
{
	float amplitude = 3.5; // Wheel target speed limit //����Ŀ���ٶ��޷�

	Vx = target_limit_float(Vx, -amplitude, amplitude);
	Vy = target_limit_float(Vy, -amplitude, amplitude);
	Vz = target_limit_float(Vz, -amplitude, amplitude);

	// Speed smoothing is enabled when moving the omnidirectional trolley
	// ȫ���ƶ�С���ſ����ٶ�ƽ������
	if (Car_Mode == Mec_Car || Car_Mode == Omni_Car || Car_Mode == Mec_Car_V550)
	{
		if (Allow_Recharge == 0)
			Smooth_control(Vx, Vy, Vz); // Smoothing the input speed //�������ٶȽ���ƽ������
		else
			smooth_control.VX = Vx,
			smooth_control.VY = Vy,
			smooth_control.VZ = Vz;

		// Get the smoothed data
		// ��ȡƽ�������������
		Vx = smooth_control.VX;
		Vy = smooth_control.VY;
		Vz = smooth_control.VZ;
	}

	// ��ƫϵ������
	float LeftWheelDiff = wheelCoefficient(LineDiffParam, 1);
	float RightWheelDiff = wheelCoefficient(LineDiffParam, 0);

	// Mecanum wheel car
	// �����ķ��С��
	if (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550)
	{
		// Inverse kinematics //�˶�ѧ���
		MOTOR_A.Target = +Vy + Vx - Vz * (Axle_spacing + Wheel_spacing);
		MOTOR_B.Target = -Vy + Vx - Vz * (Axle_spacing + Wheel_spacing);
		MOTOR_C.Target = +Vy + Vx + Vz * (Axle_spacing + Wheel_spacing);
		MOTOR_D.Target = -Vy + Vx + Vz * (Axle_spacing + Wheel_spacing);

		// Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
		MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
		MOTOR_C.Target = target_limit_float(MOTOR_C.Target, -amplitude, amplitude);
		MOTOR_D.Target = target_limit_float(MOTOR_D.Target, -amplitude, amplitude);

		MOTOR_A.Target *= LeftWheelDiff;
		MOTOR_B.Target *= LeftWheelDiff;
		MOTOR_C.Target *= RightWheelDiff;
		MOTOR_D.Target *= RightWheelDiff;
	}

	// Omni car
	// ȫ����С��
	else if (Car_Mode == Omni_Car)
	{
		// Inverse kinematics //�˶�ѧ���
		MOTOR_A.Target = Vy + Omni_turn_radiaus * Vz;
		MOTOR_B.Target = -X_PARAMETER * Vx - Y_PARAMETER * Vy + Omni_turn_radiaus * Vz;
		MOTOR_C.Target = +X_PARAMETER * Vx - Y_PARAMETER * Vy + Omni_turn_radiaus * Vz;

		// Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
		MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
		MOTOR_C.Target = target_limit_float(MOTOR_C.Target, -amplitude, amplitude);
		MOTOR_D.Target = 0; // Out of use //û��ʹ�õ�

		MOTOR_B.Target *= LeftWheelDiff;
		MOTOR_C.Target *= RightWheelDiff;
	}

	// Ackermann structure car
	// ������С��
	else if (Car_Mode == Akm_Car)
	{
		// Ackerman car specific related variables //������С��ר����ر���
		float R, Ratio = 636.56, AngleR, Angle_Servo;

		// For Ackerman small car, Vz represents the front wheel steering Angle
		// ���ڰ�����С��Vz������ǰ��ת��Ƕ�
		AngleR = Vz;
		R = Axle_spacing / tan(AngleR) - 0.5f * Wheel_spacing;

		// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
		// ǰ��ת��Ƕ��޷�(�������ǰ��ת��Ƕ�)����λ��rad
		AngleR = target_limit_float(AngleR, -0.49f, 0.32f);

		// Inverse kinematics //�˶�ѧ���
		if (AngleR != 0)
		{
			MOTOR_A.Target = Vx * (R - 0.5f * Wheel_spacing) / R;
			MOTOR_B.Target = Vx * (R + 0.5f * Wheel_spacing) / R;
		}
		else
		{
			MOTOR_A.Target = Vx;
			MOTOR_B.Target = Vx;
		}
		// The PWM value of the servo controls the steering Angle of the front wheel
		// ���PWMֵ���������ǰ��ת��Ƕ�
		Angle_Servo = -0.628f * pow(AngleR, 3) + 1.269f * pow(AngleR, 2) - 1.772f * AngleR + 1.573f;
		Servo = SERVO_INIT + (Angle_Servo - 1.572f) * Ratio;

		// Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
		MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
		MOTOR_C.Target = 0;							// Out of use //û��ʹ�õ�
		MOTOR_D.Target = 0;							// Out of use //û��ʹ�õ�
		Servo = target_limit_int(Servo, 800, 2200); // Servo PWM value limit //���PWMֵ�޷�

		MOTOR_A.Target *= LeftWheelDiff;
		MOTOR_B.Target *= RightWheelDiff;
	}

	// Differential car
	// ����С��
	else if (Car_Mode == Diff_Car)
	{
		// Inverse kinematics //�˶�ѧ���
		MOTOR_A.Target = Vx - Vz * Wheel_spacing / 2.0f; // ��������ֵ�Ŀ���ٶ�
		MOTOR_B.Target = Vx + Vz * Wheel_spacing / 2.0f; // ��������ֵ�Ŀ���ٶ�

		// Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
		MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
		MOTOR_C.Target = 0; // Out of use //û��ʹ�õ�
		MOTOR_D.Target = 0; // Out of use //û��ʹ�õ�

		MOTOR_A.Target *= LeftWheelDiff;
		MOTOR_B.Target *= RightWheelDiff;
	}

	// FourWheel car
	// ������
	else if (Car_Mode == FourWheel_Car || Car_Mode == FourWheel_Car_V550)
	{
		// Inverse kinematics //�˶�ѧ���
		MOTOR_A.Target = Vx - Vz * (Wheel_spacing + Axle_spacing) / 2.0f; // ��������ֵ�Ŀ���ٶ�
		MOTOR_B.Target = Vx - Vz * (Wheel_spacing + Axle_spacing) / 2.0f; // ��������ֵ�Ŀ���ٶ�
		MOTOR_C.Target = Vx + Vz * (Wheel_spacing + Axle_spacing) / 2.0f; // ��������ֵ�Ŀ���ٶ�
		MOTOR_D.Target = Vx + Vz * (Wheel_spacing + Axle_spacing) / 2.0f; // ��������ֵ�Ŀ���ٶ�

		// Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
		MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
		MOTOR_C.Target = target_limit_float(MOTOR_C.Target, -amplitude, amplitude);
		MOTOR_D.Target = target_limit_float(MOTOR_D.Target, -amplitude, amplitude);

		MOTOR_A.Target *= LeftWheelDiff;
		MOTOR_B.Target *= LeftWheelDiff;
		MOTOR_C.Target *= RightWheelDiff;
		MOTOR_D.Target *= RightWheelDiff;
	}

	// Tank Car
	// �Ĵ���
	else if (Car_Mode == Tank_Car)
	{
		// Inverse kinematics //�˶�ѧ���
		MOTOR_A.Target = Vx - Vz * (Wheel_spacing) / 2.0f; // ��������ֵ�Ŀ���ٶ�
		MOTOR_B.Target = Vx + Vz * (Wheel_spacing) / 2.0f; // ��������ֵ�Ŀ���ٶ�

		// Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
		MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
		MOTOR_C.Target = 0; // Out of use //û��ʹ�õ�
		MOTOR_D.Target = 0; // Out of use //û��ʹ�õ�

		MOTOR_A.Target *= LeftWheelDiff;
		MOTOR_B.Target *= RightWheelDiff;
	}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
�������ܣ�FreeRTOS���񣬺����˶���������
��ڲ�������
����  ֵ����
**************************************************************************/


void Balance_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();

typedef enum
{
    STAGE_START,           // ��ʼ״̬
    STAGE_CROSS_Y2,        // Խ��y=2.3��
    STAGE_STABLE_WAIT,     // �ȶ��ȴ��׶�
    STAGE_CHASE,           // Ѱ��׷��(��һ����)
    STAGE_WAIT_AFTER_FIRST, // ��һ������ɺ�ĵȴ�
    STAGE_CHASE_SECOND,    // Ѱ��׷��(�ڶ�����)
    STAGE_STOP             // ����ֹͣ
} EnemyStage;



// ȫ��״̬����
static EnemyStage stage = STAGE_START;
static uint8_t autoNavigation = 1;
static Path enemy_path = {0};
static int current_waypoint = 0;
static float stable_x = 0.0f;
static float stable_y = 0.0f;
static uint32_t stable_start_tick = 0;
static uint32_t wait_start_tick = 0; // �ȴ���ʼʱ��

Car_Mode = Mec_Car;

	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));
		if (SysVal.Time_count < 3000)
			SysVal.Time_count++;
		Buzzer_count1++;
		Get_Velocity_Form_Encoder();

		if (autoNavigation && Check == 0 && EN == 1)
{
	float k = 0.5f; // ����ϵ���������ƶ��ٶ�
	switch (stage)
	{
	case STAGE_START:
		// ����Ҫ����λ�ã�ֱ�ӽ�����һ�׶�
		stage = STAGE_CROSS_Y2;
		break;

	case STAGE_CROSS_Y2:
		// ��y��������ǰ����ֱ��Խ��y=2.3����ȫ�߶ȣ�
		Move_X = k * (2.5f - Y_car); // ��y�������ƶ�
		if(Move_X < 0.5f && Move_X > -0.5f) // ��ֹ�ٶȹ�С
			Move_X = (Move_X > 0) ? 0.5f : -0.5f;
		Move_Y = 0;                  // x���򲻶�
		Move_Z = 0;
		test_num = 2; // ������Ա��q

		// �ﵽ��ȫ�߶Ⱥ�����ȶ��ȴ��׶�
		if (Y_car >= 2.5f)
		{
			stage = STAGE_STABLE_WAIT;
		}
		break;

	case STAGE_STABLE_WAIT:
		// ԭ��ֹͣ���ۼ�5�루��ȷ��ʱ��
		Move_X = 0;
		Move_Y = 0;
		Move_Z = 0;
		test_num = 5; // �ȶ��ȴ�״̬
		test_num1++;
		if (stable_start_tick == 0)
		{
			// ��һ�ν��룬��¼��ǰλ�ú���ʼʱ��
			stable_x = X_car;
			stable_y = Y_car;
			stable_start_tick = getSysTickCnt();
		}
		// �ȴ�5�루5000ms��
		if (getSysTickCnt() - stable_start_tick >= 1000)
		{
			// 5���ʹ���ȶ�ֵ����·���滮
			enemy_path = plan_path(stable_x, stable_y, PRESET_ENEMY_X, PRESET_ENEMY_Y);
			current_waypoint = 0;
			stage = STAGE_CHASE;
			stable_start_tick = 0; // ���ã���ֹ�´�����
		}
		break;

	case STAGE_CHASE:
	{
		if (current_waypoint >= enemy_path.count)
		{
			// ����·��������ɣ�����Ƿ񵽴����λ��
			if (reached_target(X_car, Y_car, PRESET_ENEMY_X, PRESET_ENEMY_Y))
			{
				stage = STAGE_WAIT_AFTER_FIRST; // ����ȴ�״̬
                    wait_start_tick = getSysTickCnt(); // ��¼�ȴ���ʼʱ��
			}
			break;
		}
		test_num = 3; //

		// ��ȡ��ǰĿ��㣨�������꣩
		Waypoint target = enemy_path.points[current_waypoint];
		
		// ���ݵ�ǰ·�������;����ƶ�����
		// ��һ��·���㣺ˮƽ�ƶ����ı�x���꣩
		if (current_waypoint == 0 || 
			(fabsf(target.y - Y_car) < 0.1f && fabsf(target.x - X_car) > 0.01f))
		{
			test_num = 31;
			// ˮƽ�ƶ���ֻ�ı�x���꣨ʹ��Move_Y���ƣ�
			Move_Y = -k * (target.x - X_car)/1.4; // ����x�᷽���ƶ�
			if (fabsf(Move_Y) < 0.2f) // ��ֹ�ٶȹ�С
				Move_Y = (Move_Y > 0) ? 0.2f : -0.2f;
			Move_X = 0; // y�᷽�򲻶�
			Move_Z = 0;
			
			// ����Ƿ񵽴�x��Ŀ��
			if (fabsf(X_car - target.x) < 0.05f) {
				current_waypoint++;
			}
		}
		// �ڶ���·���㣺��ֱ�ƶ����ı�y���꣩
		else if (current_waypoint == 1 || 
				 (fabsf(target.x - X_car) < 0.1f && fabsf(target.y - Y_car) > 0.01f))
		{
			test_num = 32;
			// ��ֱ�ƶ���ֻ�ı�y���꣨ʹ��Move_X���ƣ�
			Move_X = k * (target.y - Y_car)/1.7; // ����y�᷽���ƶ�
			if (fabsf(Move_X) < 0.2f) // ��ֹ�ٶȹ�С
				Move_X = (Move_X > 0) ? 0.15f : -0.15f;
			Move_Y = 0; // x�᷽�򲻶�
			Move_Z = 0;
			
			// ����Ƿ񵽴�y��Ŀ��
			if (fabsf(Y_car - target.y) < 0.05f) {
				current_waypoint++;
			}
		}
		// ������·���㣨�����Ҫ����ˮƽ�ƶ����ı�x���꣩
		else if (current_waypoint == 2)
		{
			test_num = 33;
			// ˮƽ�ƶ���ֻ�ı�x���꣨ʹ��Move_Y���ƣ�
			Move_Y = -k * (target.x - X_car)/1.5; // ����x�᷽���ƶ�
			if (fabsf(Move_Y) < 0.3f) // ��ֹ�ٶȹ�С
				Move_Y = (Move_Y > 0) ? 0.15f : -0.15f;
			Move_X = 0; // y�᷽�򲻶�
			Move_Z = 0;
			
			// ����Ƿ񵽴�x��Ŀ��
			if (fabsf(X_car - target.x) < 0.05f) {
				current_waypoint++;
			}
		}
		break;
	}
		
	 case STAGE_WAIT_AFTER_FIRST:
            Move_X = 0;
            Move_Y = 0;
            Move_Z = 0;
            test_num = 6; // �ȴ�״̬
            
            // �ȴ�20�루20000ms��
            if (getSysTickCnt() - wait_start_tick >= 8000)
            {
                // ʹ�õ�ǰλ�ù滮���ڶ������·��
                enemy_path = plan_path(X_car, Y_car, PRESET_ENEMY_X2, PRESET_ENEMY_Y2);
                current_waypoint = 0;
                stage = STAGE_CHASE_SECOND;
            }
            break;
						
						
						  case STAGE_CHASE_SECOND:
        {
            if (current_waypoint >= enemy_path.count)
            {
                // ����·��������ɣ�����Ƿ񵽴����λ��
                if (reached_target(X_car, Y_car, PRESET_ENEMY_X2, PRESET_ENEMY_Y2))
                {
                    stage = STAGE_STOP;
                }
                break;
            }
            test_num = 7; // ׷�ٵڶ�����
            
            // ��ȡ��ǰĿ��㣨�������꣩
            Waypoint target = enemy_path.points[current_waypoint];
            
            // ·��׷���߼�����STAGE_CHASE��ͬ��
            if (current_waypoint == 0 || 
                (fabsf(target.y - Y_car) < 0.1f && fabsf(target.x - X_car) > 0.01f))
            {
                test_num = 71;
                Move_Y = -k * (target.x - X_car)/1.8;
                if (fabsf(Move_Y) < 0.2f)
                    Move_Y = (Move_Y > 0) ? 0.2f : -0.2f;
                Move_X = 0;
                Move_Z = 0;
                
                if (fabsf(X_car - target.x) < 0.05f) {
                    current_waypoint++;
                }
            }
            else if (current_waypoint == 1 || 
                     (fabsf(target.x - X_car) < 0.1f && fabsf(target.y - Y_car) > 0.01f))
            {
                test_num = 72;
                Move_X = k * (target.y - Y_car)/1.6;
                if (fabsf(Move_X) < 0.2f)
                    Move_X = (Move_X > 0) ? 0.15f : -0.15f;
                Move_Y = 0;
                Move_Z = 0;
                
                if (fabsf(Y_car - target.y) < 0.05f) {
                    current_waypoint++;
                }
            }
            else if (current_waypoint == 2)
            {
                test_num = 73;
                Move_Y = -k * (target.x - X_car)/1.5;
                if (fabsf(Move_Y) < 0.3f)
                    Move_Y = (Move_Y > 0) ? 0.15f : -0.15f;
                Move_X = 0;
                Move_Z = 0;
                
                if (fabsf(X_car - target.x) < 0.05f) {
                    current_waypoint++;
                }
            }
            break;
        }
						
	case STAGE_STOP:
		test_num = 4; // ֹͣ״̬
		Move_X = 0;
		Move_Y = 0;
		Move_Z = 0;
		autoNavigation = 0;
		break;
	}
	if(fabsf(Move_X) >1.2) Move_X =(Move_X > 0) ? 1.2f : -1.2f;
	if(fabsf(Move_Y )>1.2) Move_Y =(Move_Y > 0) ? 1.2f : -1.2f;
	// �����������Ʊ�־
	APP_ON_Flag = 0;
	Remote_ON_Flag = 0;
	PS2_ON_Flag = 0;
}
//// ���˶�����ǰ����Ƕȱջ�
//float target_yaw = 0; // Ŀ��Ƕ�0��
//float current_yaw = imu.gyro.z; // ��IMU��ȡ��ǰƫ����
//float yaw_error = target_yaw - current_yaw;

//// ��һ���Ƕ���[-��, ��]
//while (yaw_error > PI) yaw_error -= 2 * PI;
//while (yaw_error < -PI) yaw_error += 2 * PI;

//// �Ƕȱջ�PID����
//float Kp_yaw = 0.5f; // ����ϵ��
//float Kd_yaw = 0.1f; // ΢��ϵ��
//static float last_yaw_error = 0;

//// ������ת������
//float compensation_Vz = Kp_yaw * yaw_error + Kd_yaw * (yaw_error - last_yaw_error);
//last_yaw_error = yaw_error;

//// �����������ӵ�ԭʼ��ת�ٶ�
//Move_Z += compensation_Vz;
			// ===== �������� =====

			// Click the user button to update the gyroscope zero
			// �����û������������������
			Key();

			if (Allow_Recharge == 1)
				if (Get_Charging_HardWare == 0)
					Allow_Recharge = 0, Find_Charging_HardWare();

			if (Check == 0) // If self-check mode is not enabled //���û�������Լ�ģʽ
			{
				//			command_lost_count++;
				//			if(command_lost_count>RATE_100_HZ && APP_ON_Flag==0 && Remote_ON_Flag==0 && PS2_ON_Flag==0)
				//				Move_X=0,Move_Y=0,Move_Z=0;
				if (Allow_Recharge == 1)
				{
					if (Get_Charging_HardWare == 1)
					{ // ���ڻس�װ��ʱ���Իس�װ����״̬���м��
						charger_check++;
						if (charger_check > RATE_100_HZ)
							charger_check = RATE_100_HZ + 1, Allow_Recharge = 0, RED_STATE = 0, Recharge_Red_Move_X = 0, Recharge_Red_Move_Y = 0, Recharge_Red_Move_Z = 0;
					}
					// ��������˵����س䣬ͬʱû�н��յ������źţ�����������λ���ĵĻس��������
					if (nav_walk == 1 && RED_STATE == 0)
						Drive_Motor(Recharge_UP_Move_X, 0, Recharge_UP_Move_Z);
					// ���յ��˺����źţ��������Իس�װ���Ļس��������
					else if (RED_STATE != 0)
						nav_walk = 0, Drive_Motor(Recharge_Red_Move_X, 0, Recharge_Red_Move_Z);
					// ��ֹû�к����ź�ʱС���˶�
					if (nav_walk == 0 && RED_STATE == 0)
						Drive_Motor(0, 0, 0);
				}
				else
				{
					if (APP_ON_Flag)
						Get_RC(); // Handle the APP remote commands //����APPң������
					else if (Remote_ON_Flag)
						Remote_Control(); // Handle model aircraft remote commands //������ģң������
					else if (PS2_ON_Flag)
						PS2_control(); // Handle PS2 controller commands //����PS2�ֱ���������

					// CAN, Usart 1, Usart 3, Uart5 control can directly get the three axis target speed,
					// without additional processing
					// CAN������1������3(ROS)������5����ֱ�ӵõ�����Ŀ���ٶȣ�������⴦��
					else
						Drive_Motor(Move_X, Move_Y, Move_Z);
				}

				// If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
				// and the software failure flag is 0
				// �����ص�ѹ�������쳣������ʹ�ܿ�����ON��λ����������ʧ�ܱ�־λΪ0
				if (Turn_Off(Voltage) == 0 || (Allow_Recharge && EN && !Flag_Stop))
				{
					// Speed closed-loop control to calculate the PWM value of each motor,
					// PWM represents the actual wheel speed
					// �ٶȱջ����Ƽ�������PWMֵ��PWM��������ʵ��ת��
					MOTOR_A.Motor_Pwm = Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
					MOTOR_B.Motor_Pwm = Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
					MOTOR_C.Motor_Pwm = Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
					MOTOR_D.Motor_Pwm = Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);

					Limit_Pwm(16700);

					// ����Ƿ���Ҫ���PWM���Զ�ִ������
					auto_pwm_clear();

					// Set different PWM control polarity according to different car models
					// ���ݲ�ͬС���ͺ����ò�ͬ��PWM���Ƽ���
					switch (Car_Mode)
					{
					case Mec_Car:
					case Mec_Car_V550:
						Set_Pwm(MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0);
						break; // Mecanum wheel car       //�����ķ��С��
					case Omni_Car:
						Set_Pwm(-MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0);
						break; // Omni car                //ȫ����С��
					case Akm_Car:
						Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo);
						break; // Ackermann structure car //������С��
					case Diff_Car:
						Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0);
						break; // Differential car        //���ֲ���С��
					case FourWheel_Car:
					case FourWheel_Car_V550:
						Set_Pwm(MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0);
						break; // FourWheel car           //������
					case Tank_Car:
						Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0);
						break; // Tank Car                //�Ĵ���
					}
				}
				// If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
				// ���Turn_Off(Voltage)����ֵΪ1������������С�������˶���PWMֵ����Ϊ0
				else
					Set_Pwm(0, 0, 0, 0, 0);

				// Flashд��
				if (1 == FlashParam_Save(&FlashWriteFlag))
				{
					Buzzer_count = 0;
				}
			}
			else // �û��Լ����
			{
				if (Proc_Flag == 3) // �Լ���
				{
					if (check_time_count_motor_forward > 0)
					{
						check_time_count_motor_forward--;
						Full_rotation = 16799;
					}
					else if (check_time_count_motor_retreat > 0)
					{
						check_time_count_motor_retreat--;
						Full_rotation = -16799;
					}

					switch (Car_Mode)
					{
					case Mec_Car:
					case Mec_Car_V550:
						Set_Pwm(Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0);
						break; // Mecanum wheel car       //�����ķ��С��
					case Omni_Car:
						Set_Pwm(-Full_rotation, Full_rotation, -Full_rotation, Full_rotation, 0);
						break; // Omni car                //ȫ����С��
					case Akm_Car:
						Set_Pwm(Full_rotation, Full_rotation, Full_rotation, Full_rotation, 0);
						break; // Ackermann structure car //������С��
					case Diff_Car:
						Set_Pwm(Full_rotation, Full_rotation, Full_rotation, Full_rotation, 0);
						break; // Differential car        //���ֲ���С��
					case FourWheel_Car:
					case FourWheel_Car_V550:
						Set_Pwm(Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0);
						break; // FourWheel car           //������
					case Tank_Car:
						Set_Pwm(Full_rotation, Full_rotation, Full_rotation, Full_rotation, 0);
						break; // Tank Car                //�Ĵ���
					}
					if (!(check_time_count_motor_retreat > 0) && !(check_time_count_motor_forward > 0))
					{
						Set_Pwm(0, 0, 0, 0, 0);
					}
				}
				if (Proc_Flag == 4)
					Set_Pwm(0, 0, 0, 0, 0);
				if (Proc_Flag == 6)
					TIM8_SERVO_Init(9999, 168 - 1); // ��·���
				if (Proc_Flag == 7)					// ���ƶ��
				{
					if (servo_direction[0] == 0 && Servo_Count[0] < 2500)
						Servo_Count[0] = Servo_Count[0] + 5;
					if (servo_direction[0] == 0 && Servo_Count[0] >= 2500)
						servo_direction[0] = 1;
					if (Servo_Count[0] > 500 && servo_direction[0] == 1)
						Servo_Count[0] = Servo_Count[0] - 5;
					if (Servo_Count[0] <= 500 && servo_direction[0] == 1)
						Servo_Count[0] = 500, servo_direction[0] = 2;
					TIM12->CCR2 = Servo_Count[0];
				}
				if (Proc_Flag == 8)
				{
					if (servo_direction[0] != 2)
						Servo_Count[0] = 500, TIM12->CCR2 = Servo_Count[0];
					if (servo_direction[1] == 0 && Servo_Count[1] < 2500)
						Servo_Count[1] = Servo_Count[1] + 5;
					if (servo_direction[1] == 0 && Servo_Count[1] >= 2500)
						servo_direction[1] = 1;
					if (Servo_Count[1] > 500 && servo_direction[1] == 1)
						Servo_Count[1] = Servo_Count[1] - 5;
					if (Servo_Count[1] <= 500 && servo_direction[1] == 1)
						Servo_Count[1] = 500, servo_direction[1] = 2;
					TIM12->CCR1 = Servo_Count[1];
				}
				if (Proc_Flag == 9)
				{
					if (servo_direction[1] != 2)
						Servo_Count[1] = 500, TIM12->CCR1 = Servo_Count[1];
					if (servo_direction[2] == 0 && Servo_Count[2] < 2500)
						Servo_Count[2] = Servo_Count[2] + 5;
					if (servo_direction[2] == 0 && Servo_Count[2] >= 2500)
						servo_direction[2] = 1;
					if (Servo_Count[2] > 500 && servo_direction[2] == 1)
						Servo_Count[2] = Servo_Count[2] - 5;
					if (Servo_Count[2] <= 500 && servo_direction[2] == 1)
						Servo_Count[2] = 500, servo_direction[2] = 2;
					TIM8->CCR4 = Servo_Count[2];
				}
				if (Proc_Flag == 10)
				{
					if (servo_direction[2] != 2)
						Servo_Count[2] = 500, TIM8->CCR4 = Servo_Count[2];
					if (servo_direction[3] == 0 && Servo_Count[3] < 2500)
						Servo_Count[3] = Servo_Count[3] + 5;
					if (servo_direction[3] == 0 && Servo_Count[3] >= 2500)
						servo_direction[3] = 1;
					if (Servo_Count[3] > 500 && servo_direction[3] == 1)
						Servo_Count[3] = Servo_Count[3] - 5;
					if (Servo_Count[3] <= 500 && servo_direction[3] == 1)
						Servo_Count[3] = 500, servo_direction[3] = 2;
					TIM8->CCR3 = Servo_Count[3];
				}
				if (Proc_Flag == 11)
				{
					if (servo_direction[3] != 2)
						Servo_Count[3] = 500, TIM8->CCR3 = Servo_Count[3];
					if (servo_direction[4] == 0 && Servo_Count[4] < 2500)
						Servo_Count[4] = Servo_Count[4] + 5;
					if (servo_direction[4] == 0 && Servo_Count[4] >= 2500)
						servo_direction[4] = 1;
					if (Servo_Count[4] > 500 && servo_direction[4] == 1)
						Servo_Count[4] = Servo_Count[4] - 5;
					if (Servo_Count[4] <= 500 && servo_direction[4] == 1)
						Servo_Count[4] = 500, servo_direction[4] = 2;
					TIM8->CCR2 = Servo_Count[4];
				}
				if (Proc_Flag == 12)
				{
					if (servo_direction[4] != 2)
						Servo_Count[4] = 500, TIM8->CCR2 = Servo_Count[4];
					if (servo_direction[5] == 0 && Servo_Count[5] < 2500)
						Servo_Count[5] = Servo_Count[5] + 5;
					if (servo_direction[5] == 0 && Servo_Count[5] >= 2500)
						servo_direction[5] = 1;
					if (Servo_Count[5] > 500 && servo_direction[5] == 1)
						Servo_Count[5] = Servo_Count[5] - 5;
					if (Servo_Count[5] <= 500 && servo_direction[5] == 1)
						Servo_Count[5] = 500, servo_direction[5] = 2;
					TIM8->CCR1 = Servo_Count[5];
				}

				if (Proc_Flag == 13) //
				{
					servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
					Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
					TIM8->CCR1 = Servo_Count[5];
					TIM8->CCR2 = Servo_Count[4];
					TIM8->CCR3 = Servo_Count[3];
					TIM8->CCR4 = Servo_Count[2];
					TIM12->CCR1 = Servo_Count[1];
					TIM12->CCR2 = Servo_Count[0];
				}
				if (Proc_Flag == 14) // ���������1s��һ��
				{
					if ((Buzzer_count1 / 100) % 2)
						Buzzer = 1;
					else
						Buzzer = 0;
				}
				if (Proc_Flag == 15)
					Buzzer = 0;
				//				if(Proc_Flag==17)																	//��APP����WHEELTEC
				//				{
				//					if(uart2_send_flag==1)
				//					{
				//						USART2_Return();
				//						uart2_send_flag = 0;
				//						app_count = 0;
				//					}
				//				}
				if (Proc_Flag == 19)
				{
					if (uart3_send_flag == 1)
					{
						USART3_Return();
						uart3_send_flag = 0;
						message_count = 0;
					}
				}
			}
		}
	}

/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
�������ܣ���ֵ��PWM�Ĵ��������Ƴ���ת���뷽��
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d, int servo)
{
	// Forward and reverse control of motor
	// �������ת����
	if (motor_a < 0)
		PWMA1 = 16799, PWMA2 = 16799 + motor_a;
	else
		PWMA2 = 16799, PWMA1 = 16799 - motor_a;

	// Forward and reverse control of motor
	// �������ת����
	if (motor_b < 0)
		PWMB1 = 16799, PWMB2 = 16799 + motor_b;
	else
		PWMB2 = 16799, PWMB1 = 16799 - motor_b;
	//  PWMB1=10000,PWMB2=5000;

	// Forward and reverse control of motor
	// �������ת����
	if (motor_c < 0)
		PWMC1 = 16799, PWMC2 = 16799 + motor_c;
	else
		PWMC2 = 16799, PWMC1 = 16799 - motor_c;

	// Forward and reverse control of motor
	// �������ת����
	if (motor_d < 0)
		PWMD1 = 16799, PWMD2 = 16799 + motor_d;
	else
		PWMD2 = 16799, PWMD1 = 16799 - motor_d;

	// Servo control
	// �������
	Servo_PWM = servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
�������ܣ�����PWMֵ
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Limit_Pwm(int amplitude)
{
	MOTOR_A.Motor_Pwm = target_limit_float(MOTOR_A.Motor_Pwm, -amplitude, amplitude);
	MOTOR_B.Motor_Pwm = target_limit_float(MOTOR_B.Motor_Pwm, -amplitude, amplitude);
	MOTOR_C.Motor_Pwm = target_limit_float(MOTOR_C.Motor_Pwm, -amplitude, amplitude);
	MOTOR_D.Motor_Pwm = target_limit_float(MOTOR_D.Motor_Pwm, -amplitude, amplitude);
}
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
�������ܣ��޷�����
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
float target_limit_float(float insert, float low, float high)
{
	if (insert < low)
		return low;
	else if (insert > high)
		return high;
	else
		return insert;
}
int target_limit_int(int insert, int low, int high)
{
	if (insert < low)
		return low;
	else if (insert > high)
		return high;
	else
		return insert;
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
�������ܣ�����ص�ѹ��ʹ�ܿ���״̬������ʧ�ܱ�־λ״̬
��ڲ�������ѹ
����  ֵ���Ƿ��������ƣ�1����������0����
**************************************************************************/
u8 Turn_Off(int voltage)
{
	u8 temp;
	if (voltage < 10 || EN == 0 || Flag_Stop == 1)
	{
		temp = 1;
		PWMA1 = 0;
		PWMA2 = 0;
		PWMB1 = 0;
		PWMB2 = 0;
		PWMC1 = 0;
		PWMC2 = 0;
		PWMD1 = 0;
		PWMD2 = 0;
	}
	else
		temp = 0;
	return temp;
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
�������ܣ������ֵ
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{
	u32 temp;
	if (a < 0)
		temp = -a;
	else
		temp = a;
	return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)

�������ܣ�����ʽPI������
��ڲ���������������ֵ(ʵ���ٶ�)��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)��������ƫ��
e(k-1)������һ�ε�ƫ��  �Դ�����
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A(float Encoder, float Target)
{
	static float Bias, Pwm, Last_bias;
	Bias = Target - Encoder; // Calculate the deviation //����ƫ��
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
	if (Pwm > 16700)
		Pwm = 16700;
	if (Pwm < -16700)
		Pwm = -16700;
	Last_bias = Bias; // Save the last deviation //������һ��ƫ��

	// ���PWM��־λ����λΪ1ʱ������Ҫ���PWM
	if (start_clear)
	{
		// PWM�𽥵ݼ��ķ�ʽ���������С�����ڵ���ͷŶ������΢�ƶ���Ӱ��
		if (Pwm > 0)
			Pwm--;
		if (Pwm < 0)
			Pwm++;

		// �������ϣ����Ǳ�־λ��4������ֱ���4��bit��ʾ
		if (Pwm < 2.0f && Pwm > -2.0f)
			Pwm = 0, clear_state |= 1 << 0;
		else
			clear_state &= ~(1 << 0);
	}

	return Pwm;
}
int Incremental_PI_B(float Encoder, float Target)
{
	static float Bias, Pwm, Last_bias;
	Bias = Target - Encoder; // Calculate the deviation //����ƫ��
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
	if (Pwm > 16700)
		Pwm = 16700;
	if (Pwm < -16700)
		Pwm = -16700;
	Last_bias = Bias; // Save the last deviation //������һ��ƫ��
	if (start_clear)
	{
		if (Pwm > 0)
			Pwm--;
		if (Pwm < 0)
			Pwm++;

		if (Pwm < 2.0f && Pwm > -2.0f)
			Pwm = 0, clear_state |= 1 << 1;
		else
			clear_state &= ~(1 << 1);
	}
	return Pwm;
}
int Incremental_PI_C(float Encoder, float Target)
{
	static float Bias, Pwm, Last_bias;
	Bias = Target - Encoder; // Calculate the deviation //����ƫ��
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
	if (Pwm > 16700)
		Pwm = 16700;
	if (Pwm < -16700)
		Pwm = -16700;
	Last_bias = Bias; // Save the last deviation //������һ��ƫ��

	if (Car_Mode == Diff_Car || Car_Mode == Akm_Car || Car_Mode == Tank_Car)
		Pwm = 0;
	if (start_clear)
	{
		if (Pwm > 0)
			Pwm--;
		if (Pwm < 0)
			Pwm++;

		if (Pwm < 2.0f && Pwm > -2.0f)
			Pwm = 0, clear_state |= 1 << 2;
		else
			clear_state &= ~(1 << 2);
	}
	return Pwm;
}
int Incremental_PI_D(float Encoder, float Target)
{
	static float Bias, Pwm, Last_bias;

	Bias = Target - Encoder; // Calculate the deviation //����ƫ��
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
	if (Pwm > 16700)
		Pwm = 16700;
	if (Pwm < -16700)
		Pwm = -16700;
	Last_bias = Bias; // Save the last deviation //������һ��ƫ��

	if (Car_Mode == Diff_Car || Car_Mode == Akm_Car || Car_Mode == Tank_Car || Car_Mode == Omni_Car)
		Pwm = 0;
	if (start_clear)
	{
		if (Pwm > 0)
			Pwm--;
		if (Pwm < 0)
			Pwm++;

		if (Pwm < 2.0f && Pwm > -2.0f)
			Pwm = 0, clear_state |= 1 << 3;
		else
			clear_state &= ~(1 << 3);

		// 4������������ϣ���ر��������
		if ((clear_state & 0xff) == 0x0f)
			start_clear = 0, clear_done_once = 1, clear_state = 0;
	}
	return Pwm;
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
�������ܣ���APPͨ������2���͹�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move = 1;
	if (Car_Mode == Mec_Car || Car_Mode == Omni_Car || Car_Mode == Mec_Car_V550) // The omnidirectional wheel moving trolley can move laterally //ȫ�����˶�С�����Խ��к����ƶ�
	{
		switch (Flag_Direction) // Handle direction control commands //���������������
		{
		case 1:
			Move_X = RC_Velocity;
			Move_Y = 0;
			Flag_Move = 1;
			break;
		case 2:
			Move_X = RC_Velocity;
			Move_Y = -RC_Velocity;
			Flag_Move = 1;
			break;
		case 3:
			Move_X = 0;
			Move_Y = -RC_Velocity;
			Flag_Move = 1;
			break;
		case 4:
			Move_X = -RC_Velocity;
			Move_Y = -RC_Velocity;
			Flag_Move = 1;
			break;
		case 5:
			Move_X = -RC_Velocity;
			Move_Y = 0;
			Flag_Move = 1;
			break;
		case 6:
			Move_X = -RC_Velocity;
			Move_Y = RC_Velocity;
			Flag_Move = 1;
			break;
		case 7:
			Move_X = 0;
			Move_Y = RC_Velocity;
			Flag_Move = 1;
			break;
		case 8:
			Move_X = RC_Velocity;
			Move_Y = RC_Velocity;
			Flag_Move = 1;
			break;
		default:
			Move_X = 0;
			Move_Y = 0;
			Flag_Move = 0;
			break;
		}
		if (Flag_Move == 0)
		{
			// If no direction control instruction is available, check the steering control status
			// ����޷������ָ����ת�����״̬
			if (Flag_Left == 1)
				Move_Z = PI / 2 * (RC_Velocity / 500); // left rotation  //����ת
			else if (Flag_Right == 1)
				Move_Z = -PI / 2 * (RC_Velocity / 500); // right rotation //����ת
			else
				Move_Z = 0; // stop           //ֹͣ
		}
	}
	else // Non-omnidirectional moving trolley //��ȫ���ƶ�С��
	{
		switch (Flag_Direction) // Handle direction control commands //���������������
		{
		case 1:
			Move_X = +RC_Velocity;
			Move_Z = 0;
			break;
		case 2:
			Move_X = +RC_Velocity;
			Move_Z = -PI / 2;
			break;
		case 3:
			Move_X = 0;
			Move_Z = -PI / 2;
			break;
		case 4:
			Move_X = -RC_Velocity;
			Move_Z = -PI / 2;
			break;
		case 5:
			Move_X = -RC_Velocity;
			Move_Z = 0;
			break;
		case 6:
			Move_X = -RC_Velocity;
			Move_Z = +PI / 2;
			break;
		case 7:
			Move_X = 0;
			Move_Z = +PI / 2;
			break;
		case 8:
			Move_X = +RC_Velocity;
			Move_Z = +PI / 2;
			break;
		default:
			Move_X = 0;
			Move_Z = 0;
			break;
		}
		if (Flag_Left == 1)
			Move_Z = PI / 2; // left rotation  //����ת
		else if (Flag_Right == 1)
			Move_Z = -PI / 2; // right rotation //����ת
	}

	// Z-axis data conversion //Z������ת��
	if (Car_Mode == Akm_Car)
	{
		// Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		// �������ṹС��ת��Ϊǰ��ת��Ƕ�
		Move_Z = Move_Z * 2 / 9;
	}
	else if (Car_Mode == Diff_Car || Car_Mode == Tank_Car || Car_Mode == FourWheel_Car || Car_Mode == FourWheel_Car_V550)
	{
		if (Move_X < 0)
			Move_Z = -Move_Z; // The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
		Move_Z = Move_Z * RC_Velocity / 500;
	}

	// Unit conversion, mm/s -> m/s
	// ��λת����mm/s -> m/s
	Move_X = Move_X / 1000;
	Move_Y = Move_Y / 1000;
	Move_Z = Move_Z;

	// Control target value is obtained and kinematics analysis is performed
	// �õ�����Ŀ��ֵ�������˶�ѧ����
	Drive_Motor(Move_X, Move_Y, Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
�������ܣ���PS2�ֱ�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
#include "xbox360_gamepad.h"
#include "WiredPS2_gamepad.h"
// xbox360��Ϸ�ֱ������ص�����
void Xbox360GamePad_KeyEvent_Callback(uint8_t keyid, GamePadKeyEventType_t event)
{
	// ����start����
	if (keyid == Xbox360KEY_Menu && event == GamePadKeyEvent_SINGLECLICK)
		GamePadInterface->StartFlag = 1;

	if (gamepad_brand == Xbox360)
	{
		// �ֱ��Ӽ���
		if (keyid == Xbox360KEY_LB && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK))
			RC_Velocity -= 50;
		else if (keyid == Xbox360KEY_RB && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK))
			RC_Velocity += 50;

		if (RC_Velocity < 0)
			RC_Velocity = 0;
	}
	else if (gamepad_brand == PS2_USB_Wiredless)
	{
		if (keyid == Xbox360KEY_LB && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK))
			RC_Velocity += 50;
		else if (keyid == Xbox360_PaddingBit && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK))
			RC_Velocity -= 50;
		if (RC_Velocity < 0)
			RC_Velocity = 0;
	}

	// �𶯼�����ȡ��
	if (keyid == Xbox360KEY_SELECT && event == GamePadKeyEvent_LONGCLICK)
	{
		if (GamePadInterface->Vib_EN)
		{
			GamePadInterface->SetVibration(0, 127);
			vTaskDelay(50);
			GamePadInterface->Vib_EN = !GamePadInterface->Vib_EN;
		}
		else
		{
			GamePadInterface->Vib_EN = !GamePadInterface->Vib_EN;
			vTaskDelay(50);
			GamePadInterface->SetVibration(0, 127);
		}
	}
}

// ����USB�ֱ��ص�����
void Wired_USB_PS2GamePad_KeyEvent_Callback(uint8_t keyid, GamePadKeyEventType_t event)
{
	// ����start����
	if (keyid == PS2KEY_START && event == GamePadKeyEvent_SINGLECLICK)
		GamePadInterface->StartFlag = 1;

	// �ֱ��Ӽ���
	else if (keyid == PS2KEY_L2 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK))
		RC_Velocity -= 50;
	else if (keyid == PS2KEY_L1 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK))
		RC_Velocity += 50;

	if (RC_Velocity < 0)
		RC_Velocity = 0;
}

// ����PS2�ֱ��ص�����,��USB��
void Classic_PS2GamePad_KeyEvent_Callback(uint8_t keyid, GamePadKeyEventType_t event)
{
	// ����start����
	if (keyid == PS2KEY_START && event == GamePadKeyEvent_SINGLECLICK)
		GamePadInterface->StartFlag = 1;

	// �ֱ��Ӽ���
	else if (keyid == PS2KEY_L2 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK))
		RC_Velocity -= 50;
	else if (keyid == PS2KEY_L1 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK))
		RC_Velocity += 50;

	if (RC_Velocity < 0)
		RC_Velocity = 0;
}

// �ֱ���ӳ�亯��
static uint8_t map_to_vib(float x)
{
	// ������뷶Χ�������� [0.2, 1.2] ��
	if (x < 0.1f)
		return 0;
	if (x > 1.2f)
		x = 1.2f;

	// ����ӳ��
	float result = 255.0f * (x - 0.1f) / 1.1f;

	// �������벢ת��Ϊ uint8_t
	return (uint8_t)(result + 0.5f);
}

void PS2_control(void)
{
	float LX = 127, LY = 127, RX = 127;
	float ThrottleTri = 255;

	// ǰ��ҡ��
	LY = GamePadInterface->LY - 127;

	// ���Һ���
	LX = 127 - GamePadInterface->LX;

	// ˳��ʱ��
	RX = 127 - GamePadInterface->RX;

	// ҡ��΢С���ȹ���
	if (fabs(LY) < 20)
		LY = 0;
	if (fabs(LX) < 20)
		LX = 0;
	if (fabs(RX) < 20)
		RX = 0;

	// ���xbox360�ֱ������Ϊģ����ʱ������ʹ�ð������
	if (gamepad_brand == Xbox360)
	{
		// ǰ��ҡ����ֵʱ,���ð����ֵ
		if ((int)LY == 0)
		{
			if (GamePadInterface->LT == 0 && GamePadInterface->RT != 0)
				ThrottleTri = GamePadInterface->RT, LY = 127;
			else if (GamePadInterface->LT != 0 && GamePadInterface->RT == 0)
				ThrottleTri = -GamePadInterface->LT, LY = 127;
			else
				ThrottleTri = 0;
		}
	}

	// ���usb�����ֱ�,�ڷ�ģ����ģʽ�µ�ҡ��ֵӳ��
	else if (gamepad_brand == PS2_USB_Wired || gamepad_brand == PS2_USB_WiredV2)
	{
		if (fabs(RX) < 0.0001f)
		{
			if (GamePadInterface->getKeyState(PS2KEY_4PINK))
				RX = 127;
			else if (GamePadInterface->getKeyState(PS2KEY_2RED))
				RX = -127;
		}
	}

	// Handle PS2 controller control commands
	// ��PS2�ֱ�����������д���

	Move_X = (LY / 127.0f) * RC_Velocity * (ThrottleTri / 255.0f);
	Move_Y = (LX / 127.0f) * RC_Velocity;
	Move_Z = (PI / 2) * (RX / 127.0f) * (RC_Velocity / 500.0f);

	//		Move_X=LX*RC_Velocity/128;
	//		Move_Y=LY*RC_Velocity/128;
	//		Move_Z=RY*(PI/2)/128;

	// Z-axis data conversion //Z������ת��
	if (Car_Mode == Mec_Car || Car_Mode == Omni_Car || Car_Mode == Mec_Car_V550)
	{
		Move_Z = Move_Z * RC_Velocity / 500;
	}
	else if (Car_Mode == Akm_Car)
	{
		// Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		// �������ṹС��ת��Ϊǰ��ת��Ƕ�
		Move_Z = Move_Z * 2 / 9;
	}
	else if (Car_Mode == Diff_Car || Car_Mode == Tank_Car || Car_Mode == FourWheel_Car || Car_Mode == FourWheel_Car_V550)
	{
		if (Move_X < 0)
			Move_Z = -Move_Z; // The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
		Move_Z = Move_Z * RC_Velocity / 500;
	}

	// Unit conversion, mm/s -> m/s
	// ��λת����mm/s -> m/s
	Move_X = Move_X / 1000;
	Move_Y = Move_Y / 1000;
	Move_Z = Move_Z;

	// Control target value is obtained and kinematics analysis is performed
	// �õ�����Ŀ��ֵ�������˶�ѧ����
	Drive_Motor(Move_X, Move_Y, Move_Z);

// ���ݼ��ٶȷ�Ӧ��������ﵽ�ֱ�
#include "bsp_gamepad.h"

	// Z�������ж���ǿ��
	float now_z = imu.accel.z / 1671.84f;
	static float last_z = 0;
	float strength = fabs(last_z - now_z);

	// ��ӳ�䵽�ֱ�
	if (strength > 0.1f && SysVal.Time_count > CONTROL_DELAY)
	{
		if (GamePadInterface->SetVibration != NULL)
			GamePadInterface->SetVibration(map_to_vib(strength), 0);
	}
	last_z = now_z;
}

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
�������ܣ��Ժ�ģң�ؿ���������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Remote_Control(void)
{
	// Data within 1 second after entering the model control mode will not be processed
	// �Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
	static u8 thrice = 100;
	int Threshold = 100; // Threshold to ignore small movements of the joystick //��ֵ������ҡ��С���ȶ���

	// limiter //�޷�
	int LX, LY, RY, RX, Remote_RCvelocity;
	Remoter_Ch1 = target_limit_int(Remoter_Ch1, 1000, 2000);
	Remoter_Ch2 = target_limit_int(Remoter_Ch2, 1000, 2000);
	Remoter_Ch3 = target_limit_int(Remoter_Ch3, 1000, 2000);
	Remoter_Ch4 = target_limit_int(Remoter_Ch4, 1000, 2000);

	// Front and back direction of left rocker. Control forward and backward.
	// ��ҡ��ǰ���򡣿���ǰ�����ˡ�
	LX = Remoter_Ch2 - 1500;

	// Left joystick left and right.Control left and right movement. Only the wheelie omnidirectional wheelie will use the channel.
	// Ackerman trolleys use this channel as a PWM output to control the steering gear
	// ��ҡ�����ҷ��򡣿��������ƶ�������ȫ���ֲŻ�ʹ�õ���ͨ����������С��ʹ�ø�ͨ����ΪPWM������ƶ��
	LY = Remoter_Ch4 - 1500;

	// Front and back direction of right rocker. Throttle/acceleration/deceleration.
	// ��ҡ��ǰ��������/�Ӽ��١�
	RX = Remoter_Ch3 - 1500;

	// Right stick left and right. To control the rotation.
	// ��ҡ�����ҷ��򡣿�����ת��
	RY = Remoter_Ch1 - 1500;

	if (LX > -Threshold && LX < Threshold)
		LX = 0;
	if (LY > -Threshold && LY < Threshold)
		LY = 0;
	if (RX > -Threshold && RX < Threshold)
		RX = 0;
	if (RY > -Threshold && RY < Threshold)
		RY = 0;

	// Throttle related //�������
	Remote_RCvelocity = RC_Velocity + RX;
	if (Remote_RCvelocity < 0)
		Remote_RCvelocity = 0;

	// The remote control command of model aircraft is processed
	// �Ժ�ģң�ؿ���������д���
	Move_X = LX * Remote_RCvelocity / 500;
	Move_Y = -LY * Remote_RCvelocity / 500;
	Move_Z = -RY * (PI / 2) / 500;

	// Z������ת��
	if (Car_Mode == Mec_Car || Car_Mode == Omni_Car || Car_Mode == Mec_Car_V550)
	{
		Move_Z = Move_Z * Remote_RCvelocity / 500;
	}
	else if (Car_Mode == Akm_Car)
	{
		// Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		// �������ṹС��ת��Ϊǰ��ת��Ƕ�
		Move_Z = Move_Z * 2 / 9;
	}
	else if (Car_Mode == Diff_Car || Car_Mode == Tank_Car || Car_Mode == FourWheel_Car || Car_Mode == FourWheel_Car_V550)
	{
		if (Move_X < 0)
			Move_Z = -Move_Z; // The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
		Move_Z = Move_Z * Remote_RCvelocity / 500;
	}

	// Unit conversion, mm/s -> m/s
	// ��λת����mm/s -> m/s
	Move_X = Move_X / 1000;
	Move_Y = Move_Y / 1000;
	Move_Z = Move_Z;

	// Data within 1 second after entering the model control mode will not be processed
	// �Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
	if (thrice > 0)
		Move_X = 0, Move_Z = 0, thrice--;

	// Control target value is obtained and kinematics analysis is performed
	// �õ�����Ŀ��ֵ�������˶�ѧ����
	Drive_Motor(Move_X, Move_Y, Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
�������ܣ������û������������������
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{
	u8 tmp;

	// ���������Ƶ��
	tmp = KEY_Scan(RATE_100_HZ, 0);
	if (Check == 0)
	{
		// ���� �� �ֱ�ͬʱ�������ߵ��°���������Զ��س�
		if (tmp == single_click)
		{

			PRESET_ENEMY_X = X_car + X_enemy;
			PRESET_ENEMY_Y = Y_car + Y_enemy;

			// Allow_Recharge = !Allow_Recharge;
			ImuData_copy(&imu.Deviation_gyro, &imu.gyro);
			ImuData_copy(&imu.Deviation_accel, &imu.accel);
		}

		// ˫�� �� �ֱ�ͬʱ�������ߵ�ҡ��,����������
		else if (tmp == double_click)
		{
			ImuData_copy(&imu.Deviation_gyro, &imu.gyro);
			ImuData_copy(&imu.Deviation_accel, &imu.accel);
		}

		// ���� �л�ҳ��
		else if (tmp == long_click)
		{
			oled_refresh_flag = 1;
			oled_page++;
			if (oled_page > OLED_MAX_Page - 1)
				oled_page = 0;
		}
	}
	else if (Check == 1)
	{
		if (tmp == single_click)
		{
			Proc_Flag++;
			if (Proc_Flag == 21)
			{
				Check = 0;
				Buzzer = 0;
				Proc_Flag = 0;
				check_time_count_motor_forward = 300;
				check_time_count_motor_retreat = 500;
				Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
				servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
				TIM_ITConfig(TIM8, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
			}
		}
		else if (tmp == double_click)
		{
			Check = 0;
			Buzzer = 0;
			Proc_Flag = 0;
			check_time_count_motor_forward = 300;
			check_time_count_motor_retreat = 500;
			Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 500;
			servo_direction[0] = servo_direction[1] = servo_direction[2] = servo_direction[3] = servo_direction[4] = servo_direction[5] = 0;
			TIM_ITConfig(TIM8, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
		}
	}
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
�������ܣ���ȡ��������ֵ�����㳵���ٶȣ���λm/s
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	// Retrieves the original data of the encoder
	// ��ȡ��������ԭʼ����
	float Encoder_A_pr, Encoder_B_pr, Encoder_C_pr, Encoder_D_pr;
	OriginalEncoder.A = Read_Encoder(2);
	OriginalEncoder.B = Read_Encoder(3);
	OriginalEncoder.C = Read_Encoder(4);
	OriginalEncoder.D = Read_Encoder(5);

	// �����ƫϵ��
	float LeftWheelDiff = wheelCoefficient(LineDiffParam, 1);
	float RightWheelDiff = wheelCoefficient(LineDiffParam, 0);

	// test_num=OriginalEncoder.B;

	// Decide the encoder numerical polarity according to different car models
	// ���ݲ�ͬС���ͺž�����������ֵ����
	switch (Car_Mode)
	{
	case Mec_Car:
	case Mec_Car_V550:
	case FourWheel_Car:
	case FourWheel_Car_V550:
		Encoder_A_pr = OriginalEncoder.A;
		Encoder_B_pr = OriginalEncoder.B;
		Encoder_C_pr = -OriginalEncoder.C;
		Encoder_D_pr = -OriginalEncoder.D;
		break;
	case Akm_Car:
	case Diff_Car:
	case Tank_Car:
		Encoder_A_pr = OriginalEncoder.A;
		Encoder_B_pr = -OriginalEncoder.B;
		Encoder_C_pr = OriginalEncoder.C;
		Encoder_D_pr = OriginalEncoder.D;
		break;
	case Omni_Car:
		Encoder_A_pr = -OriginalEncoder.A;
		Encoder_B_pr = -OriginalEncoder.B;
		Encoder_C_pr = -OriginalEncoder.C;
		Encoder_D_pr = -OriginalEncoder.D;
		break;
	}

	// The encoder converts the raw data to wheel speed in m/s
	// ������ԭʼ����ת��Ϊ�����ٶȣ���λm/s
	MOTOR_A.Encoder = Encoder_A_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
	MOTOR_B.Encoder = Encoder_B_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
	MOTOR_C.Encoder = Encoder_C_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
	MOTOR_D.Encoder = Encoder_D_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;

	if (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550 || Car_Mode == FourWheel_Car || Car_Mode == FourWheel_Car_V550)
	{
		MOTOR_A.Encoder /= LeftWheelDiff;
		MOTOR_B.Encoder /= LeftWheelDiff;
		MOTOR_C.Encoder /= RightWheelDiff;
		MOTOR_D.Encoder /= RightWheelDiff;
	}
	else if (Car_Mode == Diff_Car || Car_Mode == Tank_Car || Car_Mode == Akm_Car)
	{
		MOTOR_A.Encoder /= LeftWheelDiff;
		MOTOR_B.Encoder /= RightWheelDiff;
	}
	else if (Car_Mode == Omni_Car)
	{
		MOTOR_B.Encoder /= LeftWheelDiff;
		MOTOR_C.Encoder /= RightWheelDiff;
	}
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
�������ܣ�������Ŀ���ٶ���ƽ������
��ڲ���������Ŀ���ٶ�
����  ֵ����
**************************************************************************/
void Smooth_control(float vx, float vy, float vz)
{
	float step = 0.01;

	if (PS2_ON_Flag)
	{
		step = 0.05;
	}
	else
	{
		step = 0.01;
	}

	if (vx > 0)
		smooth_control.VX += step;
	else if (vx < 0)
		smooth_control.VX -= step;
	else if (vx == 0)
		smooth_control.VX = smooth_control.VX * 0.9f;

	if (vy > 0)
		smooth_control.VY += step;
	else if (vy < 0)
		smooth_control.VY -= step;
	else if (vy == 0)
		smooth_control.VY = smooth_control.VY * 0.9f;

	if (vz > 0)
		smooth_control.VZ += step;
	else if (vz < 0)
		smooth_control.VZ -= step;
+	else if (vz == 0)
		smooth_control.VZ = smooth_control.VZ * 0.9f;

	smooth_control.VX = target_limit_float(smooth_control.VX, -float_abs(vx), float_abs(vx));
	smooth_control.VY = target_limit_float(smooth_control.VY, -float_abs(vy), float_abs(vy));
	smooth_control.VZ = target_limit_float(smooth_control.VZ, -float_abs(vz), float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
�������ܣ����������ݼ������ֵ
��ڲ�����������
����  ֵ���������ľ���ֵ
**************************************************************************/
float float_abs(float insert)
{
	if (insert >= 0)
		return insert;
	else
		return -insert;
}

u32 int_abs(int a)
{
	u32 temp;
	if (a < 0)
		temp = -a;
	else
		temp = a;
	return temp;
}

/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in initialization error caused by the motor spinning.Out of service
Input   : none
Output  : none
�������ܣ���ֹ��λ��ѡ��ģʽ�����³�ʼ���������������ת����ֹͣʹ��
��ڲ�������
����  ֵ����
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error = 0;

	if (abs(MOTOR_A.Motor_Pwm) > 2500 || abs(MOTOR_B.Motor_Pwm) > 2500 || abs(MOTOR_C.Motor_Pwm) > 2500 || abs(MOTOR_D.Motor_Pwm) > 2500)
		error++;
	// If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
	// �������6�νӽ�����������ж�Ϊ�����ת���õ��ʧ��
	if (error > 6)
		EN = 0, Flag_Stop = 1, robot_mode_check_flag = 1;
}

// PWM��������
void auto_pwm_clear(void)
{
	// С����̬�����ж�
	float y_accle = (float)(imu.accel.y / 1671.84f); // Y����ٶ�ʵ��ֵ
	float z_accle = (float)(imu.accel.z / 1671.84f); // Z����ٶ�ʵ��ֵ
	float diff;

	// ����Y��Z���ٶ��ں�ֵ����ֵԽ�ӽ�9.8����ʾС����̬Խˮƽ
	if (y_accle > 0)
		diff = z_accle - y_accle;
	else
		diff = z_accle + y_accle;

	//	debug_show_diff = diff;

	// PWM�������
	if (MOTOR_A.Target != 0.0f || MOTOR_B.Target != 0.0f || MOTOR_C.Target != 0.0f || MOTOR_D.Target != 0.0f)
	{
		start_check_flag = 1; // �����Ҫ���PWM
		wait_clear_times = 0; // ��λ��ռ�ʱ
		start_clear = 0;	  // ��λ�����־

		// �˶�ʱб�¼������ݸ�λ
		clear_done_once = 0;
		clear_again_times = 0;
	}
	else // ��Ŀ���ٶ��ɷ�0��0ʱ����ʼ��ʱ 2.5 �룬��С������б��״̬�£����pwm
	{
		if (start_check_flag == 1)
		{
			wait_clear_times++;
			if (wait_clear_times >= 250)
			{
				// С����ˮƽ����ʱ�ű�����pwm����ֹС����б�����˶���������
				if (diff > 8.8f)
					start_clear = 1, clear_state = 0; // �������pwm
				else
					clear_done_once = 1; // С����б���ϣ������������

				start_check_flag = 0;
			}
		}
		else
		{
			wait_clear_times = 0;
		}
	}

	// �����������������Ƴ���Ϊ��pwm����һ����ֵ����10����ٴ����
	if (clear_done_once)
	{
		// С���ӽ���ˮƽ��ʱ����������������ֹС����б�����ﳵ
		if (diff > 8.8f)
		{
			// ��������pwm�ٴλ��ۣ��������
			if (int_abs(MOTOR_A.Motor_Pwm) > 300 || int_abs(MOTOR_B.Motor_Pwm) > 300 || int_abs(MOTOR_C.Motor_Pwm) > 300 || int_abs(MOTOR_D.Motor_Pwm) > 300)
			{
				clear_again_times++;
				if (clear_again_times > 1000)
				{
					clear_done_once = 0;
					start_clear = 1; // �������pwm
					clear_state = 0;
				}
			}
			else
			{
				clear_again_times = 0;
			}
		}
		else
		{
			clear_again_times = 0;
		}
	}
}
