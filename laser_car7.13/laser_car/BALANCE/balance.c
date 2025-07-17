#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#include "balance.h"
#include <math.h>
#include <stdbool.h>
#include <float.h>

// 禁区中心点
const float forbidden[3][2] = {{1.0, 2.0}, {2.0, 2.0}, {3.0, 2.0}};
const float forbidden_radius = 0.2f; // 20cm

// 预设敌人绝对坐标 (根据实际需求设置)
float PRESET_ENEMY_X = 2.0f;
float PRESET_ENEMY_Y = 1.5f;

int test_num1 = 0; // 测试变量，用于调试

// 判断点(x, y)是否在任一禁区半径内
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

// 路径点结构
typedef struct
{
	float x;
	float y;
} Waypoint;

// 路径规划结果
typedef struct
{
	Waypoint points[3]; // 最多3个路径点
	int count;			// 实际路径点数量
} Path;

// ========== 新增: 最佳攻击点算法 ========== //
#define MIN_ATTACK_DISTANCE 1.0f
#define MAX_ATTACK_DISTANCE 2.0f
#define MAP_MIN_X 0.0f
#define MAP_MAX_X 4.0f
#define MAP_MIN_Y 0.0f
#define MAP_MAX_Y 4.0f

// 计算两点间距离
static float calculate_distance(float x1, float y1, float x2, float y2)
{
	float dx = x2 - x1;
	float dy = y2 - y1;
	return sqrtf(dx * dx + dy * dy);
}

// 判断点是否在地图内
static bool is_in_map(float x, float y)
{
	return (x >= MAP_MIN_X + 0.1f && x <= MAP_MAX_X - 0.1f && y >= MAP_MIN_Y + 0.1f && y <= MAP_MAX_Y - 0.1f);
}

// 判断某个点是否在攻击范围内且安全
static bool is_valid_attack_point(float x, float y, float enemy_x, float enemy_y)
{
	float dist = calculate_distance(x, y, enemy_x, enemy_y);
	if (dist < MIN_ATTACK_DISTANCE || dist > MAX_ATTACK_DISTANCE)
		return false;
	if (is_in_forbidden(x, y))
		return false;
	if (!is_in_map(x, y))
		return false;
	return true;
}

// 寻找最佳攻击点
static Waypoint find_best_attack_point(float enemy_x, float enemy_y)
{
	Waypoint best_point = {0, 0};
	float best_score = -1000.0f;
	// 以8个方向为基准,在1-2米范围内取样点
	const float angles[8] = {0, M_PI / 4, M_PI / 2, 3 * M_PI / 4, M_PI, 5 * M_PI / 4, 3 * M_PI / 2, 7 * M_PI / 4};
	const float distances[3] = {1.2f, 1.5f, 1.8f};
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			float test_x = enemy_x + distances[j] * cosf(angles[i]);
			float test_y = enemy_y + distances[j] * sinf(angles[i]);
			if (!is_valid_attack_point(test_x, test_y, enemy_x, enemy_y))
				continue;
			// 计算攻击点评分
			float score = 0;
			// 距离评分 - 偏好1.5m
			float dist_score = 1.0f - fabsf(calculate_distance(test_x, test_y, enemy_x, enemy_y) - 1.5f) / 0.5f;
			// 位置评分 - 偏好侧面和背面(假设敌人朝y轴正方向)
			float angle_to_enemy = atan2f(test_y - enemy_y, test_x - enemy_x);
			float pos_score = -cosf(angle_to_enemy); // 背面为1，正面为-1
			// 安全评分 - 远离禁区
			float safety_score = 1.0f;
			for (int k = 0; k < 3; k++)
			{
				float dist_to_forbidden = calculate_distance(test_x, test_y, forbidden[k][0], forbidden[k][1]);
				float safe = (dist_to_forbidden - forbidden_radius) / 2.0f;
				if (safe < 0)
					safe = 0;
				if (safe > 1)
					safe = 1;
				safety_score *= safe;
			}
			// 离墙评分
			float wall_dist = fminf(fminf(test_x - MAP_MIN_X, MAP_MAX_X - test_x), fminf(test_y - MAP_MIN_Y, MAP_MAX_Y - test_y));
			float wall_score = wall_dist / 0.5f;
			if (wall_score > 1)
				wall_score = 1;
			if (wall_score < 0)
				wall_score = 0;
			// 综合评分
			score = dist_score * 0.25f + pos_score * 0.25f + safety_score * 0.3f + wall_score * 0.2f;
			if (score > best_score)
			{
				best_score = score;
				best_point.x = test_x;
				best_point.y = test_y;
			}
		}
	}
	return best_point;
}

#define SAFE_DIST 0.25f
#define SAFE_DIST_SQ (SAFE_DIST * SAFE_DIST)

// 辅助函数：判断线段是否与任一禁区圆相交（严格避障）
bool is_segment_safe(float x1, float y1, float x2, float y2)
{
	extern const float forbidden[3][2];
	extern const float forbidden_radius;
	for (int i = 0; i < 3; i++)
	{
		float cx = forbidden[i][0];
		float cy = forbidden[i][1];
		float dx = x2 - x1;
		float dy = y2 - y1;
		float fx = x1 - cx;
		float fy = y1 - cy;
		float a = dx * dx + dy * dy;
		float b = 2 * (fx * dx + fy * dy);
		float c = fx * fx + fy * fy - (forbidden_radius + 0.05f) * (forbidden_radius + 0.05f); // 加安全裕度
		float discriminant = b * b - 4 * a * c;
		if (discriminant < 0)
			continue; // 不相交
		float sqrt_disc = sqrtf(discriminant);
		float t1 = (-b - sqrt_disc) / (2 * a);
		float t2 = (-b + sqrt_disc) / (2 * a);
		if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1))
		{
			return false; // 有交点，穿越禁区
		}
	}
	return true;
}

Path plan_path(float start_x, float start_y, float end_x, float end_y)
{
	Path path = {.count = 0};
	const float candidates[4] = {0.5f, 1.5f, 2.5f, 3.5f}; // 固定候选中间点

	// 尝试直接两段路径（水平->垂直）
	if (is_segment_safe(start_x, start_y, end_x, start_y) &&
		is_segment_safe(end_x, start_y, end_x, end_y))
	{
		path.points[path.count++] = (Waypoint){end_x, start_y};
		path.points[path.count++] = (Waypoint){end_x, end_y};
		return path;
	}

	// 尝试三段路径（水平->垂直->水平）
	float best_mid_x = 0.0f;
	float min_distance = FLT_MAX;
	bool found_safe_path = false;

	// 按距离起点最近优先检查候选点
	for (int i = 0; i < 4; i++)
	{
		float mid_x = candidates[i];
		float distance = fabsf(mid_x - end_x);

		// 检查三段路径的安全性
		bool safe1 = is_segment_safe(start_x, start_y, mid_x, start_y);
		bool safe2 = is_segment_safe(mid_x, start_y, mid_x, end_y);
		bool safe3 = is_segment_safe(mid_x, end_y, end_x, end_y);

		if (safe1 && safe2 && safe3 && distance < min_distance)
		{
			best_mid_x = mid_x;
			min_distance = distance;
			found_safe_path = true;
		}
	}

	// 找到安全路径则构建三段路径
	if (found_safe_path)
	{
		path.points[path.count++] = (Waypoint){best_mid_x, start_y}; // 水平移动到中间点
		path.points[path.count++] = (Waypoint){best_mid_x, end_y};	 // 垂直移动到目标高度
		path.points[path.count++] = (Waypoint){end_x, end_y};		 // 水平移动到终点
	}

	return path; // 若未找到安全路径，返回空路径
}

// 检查是否到达目标点
bool reached_target(float current_x, float current_y, float target_x, float target_y)
{
	const float TOLERANCE = 0.1f; // 8cm容差
	return fabsf(current_x - target_x) < TOLERANCE &&
		   fabsf(current_y - target_y) < TOLERANCE;
}

// int Time_count=0; //Time variable //计时变量

u32 Buzzer_count1 = 0;

// Robot mode is wrong to detect flag bits
// 机器人模式是否出错检测标志位
int robot_mode_check_flag = 0;

short test_num;
u8 command_lost_count = 0; //

Encoder OriginalEncoder; // Encoder raw data //编码器原始数据

//========== PWM清除使用变量 ==========//
u8 start_check_flag = 0; // 标记是否需要清空PWM
u8 wait_clear_times = 0;
u8 start_clear = 0;		// 标记开始清除PWM
u8 clear_done_once = 0; // 清除完成标志位
u16 clear_again_times = 0;
float debug_show_diff = 0;
void auto_pwm_clear(void);
volatile u8 clear_state = 0x00;
/*------------------------------------*/

uint32_t LineDiffParam = 49.3; // 纠偏系数  everloss......change

static uint8_t FlashParam_Save(uint8_t *flag)
{
	u8 check = 0;

	if (*flag == 1)
	{
		*flag = 0;

		Set_Pwm(0, 0, 0, 0, 0);

		check = 1;
		taskENTER_CRITICAL(); // 操作FLash进入临界，保证数据安全

		int32_t buf[4] = {0};
		buf[0] = *((int32_t *)&RC_Velocity);
		buf[1] = *((int32_t *)&Velocity_KP);
		buf[2] = *((int32_t *)&Velocity_KI);
		buf[3] = LineDiffParam;
		check += Write_Flash((u32 *)buf, 4);

		taskEXIT_CRITICAL(); // 退出临界

		// 若全部写入成功,check==1
	}

	return check;
}

void FlashParam_Read(void)
{
	int read;
	read = Read_Flash(0); // 读取下标为0的数据
	if (read != 0xffffffff)
		RC_Velocity = *((float *)&read);

	read = Read_Flash(1); // 读取下标为1的数据
	if (read != 0xffffffff)
		Velocity_KP = *((float *)&read);

	read = Read_Flash(2); // 读取下标为2的数据
	if (read != 0xffffffff)
		Velocity_KI = *((float *)&read);

	read = Read_Flash(3);
	if (read != 0xffffffff)
		LineDiffParam = read;

	// 异常速度数据,过滤
	if (RC_Velocity < 0 || RC_Velocity > 10000)
		RC_Velocity = 500;

	// 纠偏系数异常,过滤
	if (LineDiffParam > 100)
		LineDiffParam = 50;
}

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
// 计算纠偏系数
static float wheelCoefficient(uint32_t diffparam, uint8_t isLeftWheel)
{
	if (1 == isLeftWheel) // 左轮纠偏,对应50~100对应1.0~1.2倍的纠偏系数
	{
		if (diffparam >= 50)
			return 1.0f + 0.004f * (diffparam - 50);
	}
	else // 右轮纠偏,50~0对应1.0~1.2倍的纠偏系数
	{
		if (diffparam <= 50)
			return 1.0f + 0.004f * (50 - diffparam);
	}

	return 1.0f; // 不满足条件时,默认是1.
}

void Drive_Motor(float Vx, float Vy, float Vz)
{
	float amplitude = 3.5; // Wheel target speed limit //车轮目标速度限幅

	Vx = target_limit_float(Vx, -amplitude, amplitude);
	Vy = target_limit_float(Vy, -amplitude, amplitude);
	Vz = target_limit_float(Vz, -amplitude, amplitude);

	// Speed smoothing is enabled when moving the omnidirectional trolley
	// 全向移动小车才开启速度平滑处理
	if (Car_Mode == Mec_Car || Car_Mode == Omni_Car || Car_Mode == Mec_Car_V550)
	{
		if (Allow_Recharge == 0)
			Smooth_control(Vx, Vy, Vz); // Smoothing the input speed //对输入速度进行平滑处理
		else
			smooth_control.VX = Vx,
			smooth_control.VY = Vy,
			smooth_control.VZ = Vz;

		// Get the smoothed data
		// 获取平滑处理后的数据
		Vx = smooth_control.VX;
		Vy = smooth_control.VY;
		Vz = smooth_control.VZ;
	}

	// 纠偏系数计算
	float LeftWheelDiff = wheelCoefficient(LineDiffParam, 1);
	float RightWheelDiff = wheelCoefficient(LineDiffParam, 0);

	// Mecanum wheel car
	// 麦克纳姆轮小车
	if (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550)
	{
		// Inverse kinematics //运动学逆解
		MOTOR_A.Target = +Vy + Vx - Vz * (Axle_spacing + Wheel_spacing);
		MOTOR_B.Target = -Vy + Vx - Vz * (Axle_spacing + Wheel_spacing);
		MOTOR_C.Target = +Vy + Vx + Vz * (Axle_spacing + Wheel_spacing);
		MOTOR_D.Target = -Vy + Vx + Vz * (Axle_spacing + Wheel_spacing);

		// Wheel (motor) target speed limit //车轮(电机)目标速度限幅
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
	// 全向轮小车
	else if (Car_Mode == Omni_Car)
	{
		// Inverse kinematics //运动学逆解
		MOTOR_A.Target = Vy + Omni_turn_radiaus * Vz;
		MOTOR_B.Target = -X_PARAMETER * Vx - Y_PARAMETER * Vy + Omni_turn_radiaus * Vz;
		MOTOR_C.Target = +X_PARAMETER * Vx - Y_PARAMETER * Vy + Omni_turn_radiaus * Vz;

		// Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
		MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
		MOTOR_C.Target = target_limit_float(MOTOR_C.Target, -amplitude, amplitude);
		MOTOR_D.Target = 0; // Out of use //没有使用到

		MOTOR_B.Target *= LeftWheelDiff;
		MOTOR_C.Target *= RightWheelDiff;
	}

	// Ackermann structure car
	// 阿克曼小车
	else if (Car_Mode == Akm_Car)
	{
		// Ackerman car specific related variables //阿克曼小车专用相关变量
		float R, Ratio = 636.56, AngleR, Angle_Servo;

		// For Ackerman small car, Vz represents the front wheel steering Angle
		// 对于阿克曼小车Vz代表右前轮转向角度
		AngleR = Vz;
		R = Axle_spacing / tan(AngleR) - 0.5f * Wheel_spacing;

		// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
		// 前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
		AngleR = target_limit_float(AngleR, -0.49f, 0.32f);

		// Inverse kinematics //运动学逆解
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
		// 舵机PWM值，舵机控制前轮转向角度
		Angle_Servo = -0.628f * pow(AngleR, 3) + 1.269f * pow(AngleR, 2) - 1.772f * AngleR + 1.573f;
		Servo = SERVO_INIT + (Angle_Servo - 1.572f) * Ratio;

		// Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
		MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
		MOTOR_C.Target = 0;							// Out of use //没有使用到
		MOTOR_D.Target = 0;							// Out of use //没有使用到
		Servo = target_limit_int(Servo, 800, 2200); // Servo PWM value limit //舵机PWM值限幅

		MOTOR_A.Target *= LeftWheelDiff;
		MOTOR_B.Target *= RightWheelDiff;
	}

	// Differential car
	// 差速小车
	else if (Car_Mode == Diff_Car)
	{
		// Inverse kinematics //运动学逆解
		MOTOR_A.Target = Vx - Vz * Wheel_spacing / 2.0f; // 计算出左轮的目标速度
		MOTOR_B.Target = Vx + Vz * Wheel_spacing / 2.0f; // 计算出右轮的目标速度

		// Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
		MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
		MOTOR_C.Target = 0; // Out of use //没有使用到
		MOTOR_D.Target = 0; // Out of use //没有使用到

		MOTOR_A.Target *= LeftWheelDiff;
		MOTOR_B.Target *= RightWheelDiff;
	}

	// FourWheel car
	// 四驱车
	else if (Car_Mode == FourWheel_Car || Car_Mode == FourWheel_Car_V550)
	{
		// Inverse kinematics //运动学逆解
		MOTOR_A.Target = Vx - Vz * (Wheel_spacing + Axle_spacing) / 2.0f; // 计算出左轮的目标速度
		MOTOR_B.Target = Vx - Vz * (Wheel_spacing + Axle_spacing) / 2.0f; // 计算出左轮的目标速度
		MOTOR_C.Target = Vx + Vz * (Wheel_spacing + Axle_spacing) / 2.0f; // 计算出右轮的目标速度
		MOTOR_D.Target = Vx + Vz * (Wheel_spacing + Axle_spacing) / 2.0f; // 计算出右轮的目标速度

		// Wheel (motor) target speed limit //车轮(电机)目标速度限幅
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
	// 履带车
	else if (Car_Mode == Tank_Car)
	{
		// Inverse kinematics //运动学逆解
		MOTOR_A.Target = Vx - Vz * (Wheel_spacing) / 2.0f; // 计算出左轮的目标速度
		MOTOR_B.Target = Vx + Vz * (Wheel_spacing) / 2.0f; // 计算出右轮的目标速度

		// Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
		MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
		MOTOR_C.Target = 0; // Out of use //没有使用到
		MOTOR_D.Target = 0; // Out of use //没有使用到

		MOTOR_A.Target *= LeftWheelDiff;
		MOTOR_B.Target *= RightWheelDiff;
	}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
void Balance_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();

	typedef enum
	{
		STAGE_START,	   // 初始状态
		STAGE_CROSS_Y2,	   // 越过y=2.3线
		STAGE_STABLE_WAIT, // 稳定等待阶段
		STAGE_CHASE,	   // 寻敌追击
		STAGE_STOP		   // 停止
	} EnemyStage;

	// 全局状态变量
	static EnemyStage stage = STAGE_START;
	static uint8_t autoNavigation = 0;
	static Path enemy_path = {0};
	static int current_waypoint = 0;
	static float stable_x = 0.0f;
	static float stable_y = 0.0f;
	static uint32_t stable_start_tick = 0;

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
			float k = 0.5f; // 控制系数，调节移动速度
			switch (stage)
			{
			case STAGE_START:
				// 根据外部传感器的相对坐标计算敌人绝对坐标

				// 进入下一阶段
				stage = STAGE_CROSS_Y2;
				break;

			case STAGE_CROSS_Y2:
				// 沿y轴正方向前进，直到越过y=2.3（安全高度）
				Move_X = k * (2.5f - Y_car);		 // 向y正方向移动
				if (Move_X < 0.5f && Move_X > -0.5f) // 防止速度过小
					Move_X = (Move_X > 0) ? 0.5f : -0.5f;
				Move_Y = 0; // x方向不动
				Move_Z = 0;
				test_num = 2; // 清除测试编号q

				// 达到安全高度后进入稳定等待阶段
				if (Y_car >= 2.0f)
				{
					stage = STAGE_STABLE_WAIT;
				}
				break;

			case STAGE_STABLE_WAIT:
				// 原地停止，累计5秒（精确计时）
				Move_X = 0;
				Move_Y = 0;
				Move_Z = 0;
				test_num = 5; // 稳定等待状态
				test_num1++;
				if (stable_start_tick == 0)
				{
					// 第一次进入，记录当前位置和起始时间
					stable_x = X_car;
					stable_y = Y_car;
					stable_start_tick = getSysTickCnt();
				}
				// 等待5秒（5000ms）
				if (getSysTickCnt() - stable_start_tick >= 2000)
				{
					// enemy_x, enemy_y 为相对坐标（单位：米），X_car, Y_car为自身绝对坐标
					PRESET_ENEMY_X = X_car + X_enemy;
					PRESET_ENEMY_Y = Y_car + Y_enemy;

					// 5秒后，计算最佳攻击点并进行路径规划
					Waypoint best_attack = find_best_attack_point(PRESET_ENEMY_X, PRESET_ENEMY_Y);
					enemy_path = plan_path(stable_x, stable_y, best_attack.x, best_attack.y);
					current_waypoint = 0;
					stage = STAGE_CHASE;
					stable_start_tick = 0; // 重置，防止下次误用
				}
				break;

			case STAGE_CHASE:
			{
				if (current_waypoint >= enemy_path.count)
				{
					// 所有路径点已完成，检查是否到达最佳攻击点
					Waypoint best_attack = find_best_attack_point(PRESET_ENEMY_X, PRESET_ENEMY_Y);
					if (reached_target(X_car, Y_car, best_attack.x, best_attack.y))
					{
						stage = STAGE_STOP;
					}
					break;
				}
				test_num = 3; //

				// 获取当前目标点（绝对坐标）
				Waypoint target = enemy_path.points[current_waypoint];

				// 根据当前路径点类型决定移动方向
				// 第一个路径点：水平移动（改变x坐标）
				if (current_waypoint == 0 ||
					(fabsf(target.y - Y_car) < 0.01f && fabsf(target.x - X_car) > 0.01f))
				{
					// 水平移动：只改变x坐标（使用Move_Y控制）
					Move_Y = -k * (target.x - X_car); // 控制x轴方向移动
					if (fabsf(Move_Y) < 0.2f)		  // 防止速度过小
						Move_Y = (Move_Y > 0) ? 0.2f : -0.2f;
					Move_X = 0; // y轴方向不动
					Move_Z = 0;

					// 检查是否到达x轴目标
					if (fabsf(X_car - target.x) < 0.15f)
					{
						current_waypoint++;
					}
				}
				// 第二个路径点：垂直移动（改变y坐标）
				else if (current_waypoint == 1 ||
						 (fabsf(target.x - X_car) < 0.01f && fabsf(target.y - Y_car) > 0.01f))
				{
					// 垂直移动：只改变y坐标（使用Move_X控制）
					Move_X = k * (target.y - Y_car); // 控制y轴方向移动
					if (fabsf(Move_X) < 0.2f)		 // 防止速度过小
						Move_X = (Move_X > 0) ? 0.2f : -0.2f;
					Move_Y = 0; // x轴方向不动
					Move_Z = 0;

					// 检查是否到达y轴目标
					if (fabsf(Y_car - target.y) < 0.15f)
					{
						current_waypoint++;
					}
				}
				// 第三个路径点（如果需要）：水平移动（改变x坐标）
				else if (current_waypoint == 2)
				{
					// 水平移动：只改变x坐标（使用Move_Y控制）
					Move_Y = -k * (target.x - X_car); // 控制x轴方向移动
					if (fabsf(Move_Y) < 0.2f)		  // 防止速度过小
						Move_Y = (Move_Y > 0) ? 0.2f : -0.2f;
					Move_X = 0; // y轴方向不动
					Move_Z = 0;

					// 检查是否到达x轴目标
					if (fabsf(X_car - target.x) < 0.15f)
					{
						current_waypoint++;
					}
				}
				break;
			}

			case STAGE_STOP:
				test_num = 4; // 停止状态
				Move_X = 0;
				Move_Y = 0;
				Move_Z = 0;
				autoNavigation = 0;
				break;
			}

			// 禁用其他控制标志
			APP_ON_Flag = 0;
			Remote_ON_Flag = 0;
			PS2_ON_Flag = 0;
		}
		// ===== 新增结束 =====

		// Click the user button to update the gyroscope zero
		// 单击用户按键更新陀螺仪零点
		Key();

		if (Allow_Recharge == 1)
			if (Get_Charging_HardWare == 0)
				Allow_Recharge = 0, Find_Charging_HardWare();

		if (Check == 0) // If self-check mode is not enabled //如果没有启动自检模式
		{
			//			command_lost_count++;
			//			if(command_lost_count>RATE_100_HZ && APP_ON_Flag==0 && Remote_ON_Flag==0 && PS2_ON_Flag==0)
			//				Move_X=0,Move_Y=0,Move_Z=0;
			if (Allow_Recharge == 1)
			{
				if (Get_Charging_HardWare == 1)
				{ // 存在回充装备时，对回充装备的状态进行检测
					charger_check++;
					if (charger_check > RATE_100_HZ)
						charger_check = RATE_100_HZ + 1, Allow_Recharge = 0, RED_STATE = 0, Recharge_Red_Move_X = 0, Recharge_Red_Move_Y = 0, Recharge_Red_Move_Z = 0;
				}
				// 如果开启了导航回充，同时没有接收到红外信号，接收来自上位机的的回充控制命令
				if (nav_walk == 1 && RED_STATE == 0)
					Drive_Motor(Recharge_UP_Move_X, 0, Recharge_UP_Move_Z);
				// 接收到了红外信号，接收来自回充装备的回充控制命令
				else if (RED_STATE != 0)
					nav_walk = 0, Drive_Motor(Recharge_Red_Move_X, 0, Recharge_Red_Move_Z);
				// 防止没有红外信号时小车运动
				if (nav_walk == 0 && RED_STATE == 0)
					Drive_Motor(0, 0, 0);
			}
			else
			{
				if (APP_ON_Flag)
					Get_RC(); // Handle the APP remote commands //处理APP遥控命令
				else if (Remote_ON_Flag)
					Remote_Control(); // Handle model aircraft remote commands //处理航模遥控命令
				else if (PS2_ON_Flag)
					PS2_control(); // Handle PS2 controller commands //处理PS2手柄控制命令

				// CAN, Usart 1, Usart 3, Uart5 control can directly get the three axis target speed,
				// without additional processing
				// CAN、串口1、串口3(ROS)、串口5控制直接得到三轴目标速度，无须额外处理
				else
					Drive_Motor(Move_X, Move_Y, Move_Z);
			}

			// If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
			// and the software failure flag is 0
			// 如果电池电压不存在异常，而且使能开关在ON档位，而且软件失能标志位为0
			if (Turn_Off(Voltage) == 0 || (Allow_Recharge && EN && !Flag_Stop))
			{
				// Speed closed-loop control to calculate the PWM value of each motor,
				// PWM represents the actual wheel speed
				// 速度闭环控制计算各电机PWM值，PWM代表车轮实际转速
				MOTOR_A.Motor_Pwm = Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
				MOTOR_B.Motor_Pwm = Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
				MOTOR_C.Motor_Pwm = Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
				MOTOR_D.Motor_Pwm = Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);

				Limit_Pwm(16700);

				// 检测是否需要清除PWM并自动执行清理
				auto_pwm_clear();

				// Set different PWM control polarity according to different car models
				// 根据不同小车型号设置不同的PWM控制极性
				switch (Car_Mode)
				{
				case Mec_Car:
				case Mec_Car_V550:
					Set_Pwm(MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0);
					break; // Mecanum wheel car       //麦克纳姆轮小车
				case Omni_Car:
					Set_Pwm(-MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0);
					break; // Omni car                //全向轮小车
				case Akm_Car:
					Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo);
					break; // Ackermann structure car //阿克曼小车
				case Diff_Car:
					Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0);
					break; // Differential car        //两轮差速小车
				case FourWheel_Car:
				case FourWheel_Car_V550:
					Set_Pwm(MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0);
					break; // FourWheel car           //四驱车
				case Tank_Car:
					Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0);
					break; // Tank Car                //履带车
				}
			}
			// If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
			// 如果Turn_Off(Voltage)返回值为1，不允许控制小车进行运动，PWM值设置为0
			else
				Set_Pwm(0, 0, 0, 0, 0);

			// Flash写入
			if (1 == FlashParam_Save(&FlashWriteFlag))
			{
				Buzzer_count = 0;
			}
		}
		else // 用户自检代码
		{
			if (Proc_Flag == 3) // 自检电机
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
					break; // Mecanum wheel car       //麦克纳姆轮小车
				case Omni_Car:
					Set_Pwm(-Full_rotation, Full_rotation, -Full_rotation, Full_rotation, 0);
					break; // Omni car                //全向轮小车
				case Akm_Car:
					Set_Pwm(Full_rotation, Full_rotation, Full_rotation, Full_rotation, 0);
					break; // Ackermann structure car //阿克曼小车
				case Diff_Car:
					Set_Pwm(Full_rotation, Full_rotation, Full_rotation, Full_rotation, 0);
					break; // Differential car        //两轮差速小车
				case FourWheel_Car:
				case FourWheel_Car_V550:
					Set_Pwm(Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0);
					break; // FourWheel car           //四驱车
				case Tank_Car:
					Set_Pwm(Full_rotation, Full_rotation, Full_rotation, Full_rotation, 0);
					break; // Tank Car                //履带车
				}
				if (!(check_time_count_motor_retreat > 0) && !(check_time_count_motor_forward > 0))
				{
					Set_Pwm(0, 0, 0, 0, 0);
				}
			}
			if (Proc_Flag == 4)
				Set_Pwm(0, 0, 0, 0, 0);
			if (Proc_Flag == 6)
				TIM8_SERVO_Init(9999, 168 - 1); // 六路舵机
			if (Proc_Flag == 7)					// 控制舵机
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
			if (Proc_Flag == 14) // 蜂鸣器间隔1s响一次
			{
				if ((Buzzer_count1 / 100) % 2)
					Buzzer = 1;
				else
					Buzzer = 0;
			}
			if (Proc_Flag == 15)
				Buzzer = 0;
			//				if(Proc_Flag==17)																	//向APP发送WHEELTEC
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
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d, int servo)
{
	// Forward and reverse control of motor
	// 电机正反转控制
	if (motor_a < 0)
		PWMA1 = 16799, PWMA2 = 16799 + motor_a;
	else
		PWMA2 = 16799, PWMA1 = 16799 - motor_a;

	// Forward and reverse control of motor
	// 电机正反转控制
	if (motor_b < 0)
		PWMB1 = 16799, PWMB2 = 16799 + motor_b;
	else
		PWMB2 = 16799, PWMB1 = 16799 - motor_b;
	//  PWMB1=10000,PWMB2=5000;

	// Forward and reverse control of motor
	// 电机正反转控制
	if (motor_c < 0)
		PWMC1 = 16799, PWMC2 = 16799 + motor_c;
	else
		PWMC2 = 16799, PWMC1 = 16799 - motor_c;

	// Forward and reverse control of motor
	// 电机正反转控制
	if (motor_d < 0)
		PWMD1 = 16799, PWMD2 = 16799 + motor_d;
	else
		PWMD2 = 16799, PWMD1 = 16799 - motor_d;

	// Servo control
	// 舵机控制
	Servo_PWM = servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值
入口参数：幅值
返回  值：无
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
函数功能：限幅函数
入口参数：幅值
返回  值：无
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
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
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
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
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
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A(float Encoder, float Target)
{
	static float Bias, Pwm, Last_bias;
	Bias = Target - Encoder; // Calculate the deviation //计算偏差
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
	if (Pwm > 16700)
		Pwm = 16700;
	if (Pwm < -16700)
		Pwm = -16700;
	Last_bias = Bias; // Save the last deviation //保存上一次偏差

	// 清除PWM标志位，该位为1时代表需要清除PWM
	if (start_clear)
	{
		// PWM逐渐递减的方式清除，减缓小车由于电机释放而造成轻微移动的影响
		if (Pwm > 0)
			Pwm--;
		if (Pwm < 0)
			Pwm++;

		// 若清除完毕，则标记标志位，4个电机分别用4个bit表示
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
	Bias = Target - Encoder; // Calculate the deviation //计算偏差
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
	if (Pwm > 16700)
		Pwm = 16700;
	if (Pwm < -16700)
		Pwm = -16700;
	Last_bias = Bias; // Save the last deviation //保存上一次偏差
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
	Bias = Target - Encoder; // Calculate the deviation //计算偏差
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
	if (Pwm > 16700)
		Pwm = 16700;
	if (Pwm < -16700)
		Pwm = -16700;
	Last_bias = Bias; // Save the last deviation //保存上一次偏差

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

	Bias = Target - Encoder; // Calculate the deviation //计算偏差
	Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
	if (Pwm > 16700)
		Pwm = 16700;
	if (Pwm < -16700)
		Pwm = -16700;
	Last_bias = Bias; // Save the last deviation //保存上一次偏差

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

		// 4个电机均清除完毕，则关闭清除任务
		if ((clear_state & 0xff) == 0x0f)
			start_clear = 0, clear_done_once = 1, clear_state = 0;
	}
	return Pwm;
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move = 1;
	if (Car_Mode == Mec_Car || Car_Mode == Omni_Car || Car_Mode == Mec_Car_V550) // The omnidirectional wheel moving trolley can move laterally //全向轮运动小车可以进行横向移动
	{
		switch (Flag_Direction) // Handle direction control commands //处理方向控制命令
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
			// 如果无方向控制指令，检查转向控制状态
			if (Flag_Left == 1)
				Move_Z = PI / 2 * (RC_Velocity / 500); // left rotation  //左自转
			else if (Flag_Right == 1)
				Move_Z = -PI / 2 * (RC_Velocity / 500); // right rotation //右自转
			else
				Move_Z = 0; // stop           //停止
		}
	}
	else // Non-omnidirectional moving trolley //非全向移动小车
	{
		switch (Flag_Direction) // Handle direction control commands //处理方向控制命令
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
			Move_Z = PI / 2; // left rotation  //左自转
		else if (Flag_Right == 1)
			Move_Z = -PI / 2; // right rotation //右自转
	}

	// Z-axis data conversion //Z轴数据转化
	if (Car_Mode == Akm_Car)
	{
		// Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		// 阿克曼结构小车转换为前轮转向角度
		Move_Z = Move_Z * 2 / 9;
	}
	else if (Car_Mode == Diff_Car || Car_Mode == Tank_Car || Car_Mode == FourWheel_Car || Car_Mode == FourWheel_Car_V550)
	{
		if (Move_X < 0)
			Move_Z = -Move_Z; // The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z = Move_Z * RC_Velocity / 500;
	}

	// Unit conversion, mm/s -> m/s
	// 单位转换，mm/s -> m/s
	Move_X = Move_X / 1000;
	Move_Y = Move_Y / 1000;
	Move_Z = Move_Z;

	// Control target value is obtained and kinematics analysis is performed
	// 得到控制目标值，进行运动学分析
	Drive_Motor(Move_X, Move_Y, Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
函数功能：对PS2手柄控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
#include "xbox360_gamepad.h"
#include "WiredPS2_gamepad.h"
// xbox360游戏手柄按键回调函数
void Xbox360GamePad_KeyEvent_Callback(uint8_t keyid, GamePadKeyEventType_t event)
{
	// 按下start按键
	if (keyid == Xbox360KEY_Menu && event == GamePadKeyEvent_SINGLECLICK)
		GamePadInterface->StartFlag = 1;

	if (gamepad_brand == Xbox360)
	{
		// 手柄加减速
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

	// 震动激活与取消
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

// 有线USB手柄回调函数
void Wired_USB_PS2GamePad_KeyEvent_Callback(uint8_t keyid, GamePadKeyEventType_t event)
{
	// 按下start按键
	if (keyid == PS2KEY_START && event == GamePadKeyEvent_SINGLECLICK)
		GamePadInterface->StartFlag = 1;

	// 手柄加减速
	else if (keyid == PS2KEY_L2 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK))
		RC_Velocity -= 50;
	else if (keyid == PS2KEY_L1 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK))
		RC_Velocity += 50;

	if (RC_Velocity < 0)
		RC_Velocity = 0;
}

// 经典PS2手柄回调函数,非USB款
void Classic_PS2GamePad_KeyEvent_Callback(uint8_t keyid, GamePadKeyEventType_t event)
{
	// 按下start按键
	if (keyid == PS2KEY_START && event == GamePadKeyEvent_SINGLECLICK)
		GamePadInterface->StartFlag = 1;

	// 手柄加减速
	else if (keyid == PS2KEY_L2 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK))
		RC_Velocity -= 50;
	else if (keyid == PS2KEY_L1 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK))
		RC_Velocity += 50;

	if (RC_Velocity < 0)
		RC_Velocity = 0;
}

// 手柄震动映射函数
static uint8_t map_to_vib(float x)
{
	// 检查输入范围，限制在 [0.2, 1.2] 内
	if (x < 0.1f)
		return 0;
	if (x > 1.2f)
		x = 1.2f;

	// 线性映射
	float result = 255.0f * (x - 0.1f) / 1.1f;

	// 四舍五入并转换为 uint8_t
	return (uint8_t)(result + 0.5f);
}

void PS2_control(void)
{
	float LX = 127, LY = 127, RX = 127;
	float ThrottleTri = 255;

	// 前进摇杆
	LY = GamePadInterface->LY - 127;

	// 左右横移
	LX = 127 - GamePadInterface->LX;

	// 顺逆时针
	RX = 127 - GamePadInterface->RX;

	// 摇杆微小幅度过滤
	if (fabs(LY) < 20)
		LY = 0;
	if (fabs(LX) < 20)
		LX = 0;
	if (fabs(RX) < 20)
		RX = 0;

	// 针对xbox360手柄，扳机为模拟量时，允许使用扳机控制
	if (gamepad_brand == Xbox360)
	{
		// 前进摇杆无值时,采用扳机的值
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

	// 针对usb有线手柄,在非模拟量模式下的摇杆值映射
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
	// 对PS2手柄控制命令进行处理

	Move_X = (LY / 127.0f) * RC_Velocity * (ThrottleTri / 255.0f);
	Move_Y = (LX / 127.0f) * RC_Velocity;
	Move_Z = (PI / 2) * (RX / 127.0f) * (RC_Velocity / 500.0f);

	//		Move_X=LX*RC_Velocity/128;
	//		Move_Y=LY*RC_Velocity/128;
	//		Move_Z=RY*(PI/2)/128;

	// Z-axis data conversion //Z轴数据转化
	if (Car_Mode == Mec_Car || Car_Mode == Omni_Car || Car_Mode == Mec_Car_V550)
	{
		Move_Z = Move_Z * RC_Velocity / 500;
	}
	else if (Car_Mode == Akm_Car)
	{
		// Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		// 阿克曼结构小车转换为前轮转向角度
		Move_Z = Move_Z * 2 / 9;
	}
	else if (Car_Mode == Diff_Car || Car_Mode == Tank_Car || Car_Mode == FourWheel_Car || Car_Mode == FourWheel_Car_V550)
	{
		if (Move_X < 0)
			Move_Z = -Move_Z; // The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z = Move_Z * RC_Velocity / 500;
	}

	// Unit conversion, mm/s -> m/s
	// 单位转换，mm/s -> m/s
	Move_X = Move_X / 1000;
	Move_Y = Move_Y / 1000;
	Move_Z = Move_Z;

	// Control target value is obtained and kinematics analysis is performed
	// 得到控制目标值，进行运动学分析
	Drive_Motor(Move_X, Move_Y, Move_Z);

// 根据加速度反应震动情况传达到手柄
#include "bsp_gamepad.h"

	// Z轴数据判断震动强度
	float now_z = imu.accel.z / 1671.84f;
	static float last_z = 0;
	float strength = fabs(last_z - now_z);

	// 震动映射到手柄
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
函数功能：对航模遥控控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control(void)
{
	// Data within 1 second after entering the model control mode will not be processed
	// 对进入航模控制模式后1秒内的数据不处理
	static u8 thrice = 100;
	int Threshold = 100; // Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作

	// limiter //限幅
	int LX, LY, RY, RX, Remote_RCvelocity;
	Remoter_Ch1 = target_limit_int(Remoter_Ch1, 1000, 2000);
	Remoter_Ch2 = target_limit_int(Remoter_Ch2, 1000, 2000);
	Remoter_Ch3 = target_limit_int(Remoter_Ch3, 1000, 2000);
	Remoter_Ch4 = target_limit_int(Remoter_Ch4, 1000, 2000);

	// Front and back direction of left rocker. Control forward and backward.
	// 左摇杆前后方向。控制前进后退。
	LX = Remoter_Ch2 - 1500;

	// Left joystick left and right.Control left and right movement. Only the wheelie omnidirectional wheelie will use the channel.
	// Ackerman trolleys use this channel as a PWM output to control the steering gear
	// 左摇杆左右方向。控制左右移动。麦轮全向轮才会使用到改通道。阿克曼小车使用该通道作为PWM输出控制舵机
	LY = Remoter_Ch4 - 1500;

	// Front and back direction of right rocker. Throttle/acceleration/deceleration.
	// 右摇杆前后方向。油门/加减速。
	RX = Remoter_Ch3 - 1500;

	// Right stick left and right. To control the rotation.
	// 右摇杆左右方向。控制自转。
	RY = Remoter_Ch1 - 1500;

	if (LX > -Threshold && LX < Threshold)
		LX = 0;
	if (LY > -Threshold && LY < Threshold)
		LY = 0;
	if (RX > -Threshold && RX < Threshold)
		RX = 0;
	if (RY > -Threshold && RY < Threshold)
		RY = 0;

	// Throttle related //油门相关
	Remote_RCvelocity = RC_Velocity + RX;
	if (Remote_RCvelocity < 0)
		Remote_RCvelocity = 0;

	// The remote control command of model aircraft is processed
	// 对航模遥控控制命令进行处理
	Move_X = LX * Remote_RCvelocity / 500;
	Move_Y = -LY * Remote_RCvelocity / 500;
	Move_Z = -RY * (PI / 2) / 500;

	// Z轴数据转化
	if (Car_Mode == Mec_Car || Car_Mode == Omni_Car || Car_Mode == Mec_Car_V550)
	{
		Move_Z = Move_Z * Remote_RCvelocity / 500;
	}
	else if (Car_Mode == Akm_Car)
	{
		// Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		// 阿克曼结构小车转换为前轮转向角度
		Move_Z = Move_Z * 2 / 9;
	}
	else if (Car_Mode == Diff_Car || Car_Mode == Tank_Car || Car_Mode == FourWheel_Car || Car_Mode == FourWheel_Car_V550)
	{
		if (Move_X < 0)
			Move_Z = -Move_Z; // The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z = Move_Z * Remote_RCvelocity / 500;
	}

	// Unit conversion, mm/s -> m/s
	// 单位转换，mm/s -> m/s
	Move_X = Move_X / 1000;
	Move_Y = Move_Y / 1000;
	Move_Z = Move_Z;

	// Data within 1 second after entering the model control mode will not be processed
	// 对进入航模控制模式后1秒内的数据不处理
	if (thrice > 0)
		Move_X = 0, Move_Z = 0, thrice--;

	// Control target value is obtained and kinematics analysis is performed
	// 得到控制目标值，进行运动学分析
	Drive_Motor(Move_X, Move_Y, Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
函数功能：单击用户按键更新陀螺仪零点
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{
	u8 tmp;

	// 传入任务的频率
	tmp = KEY_Scan(RATE_100_HZ, 0);
	if (Check == 0)
	{
		// 单击 或 手柄同时按下两边的下扳机，开启自动回充
		if (tmp == single_click)
		{

			PRESET_ENEMY_X = X_car + X_enemy;
			PRESET_ENEMY_Y = Y_car + Y_enemy;

			// Allow_Recharge = !Allow_Recharge;
			ImuData_copy(&imu.Deviation_gyro, &imu.gyro);
			ImuData_copy(&imu.Deviation_accel, &imu.accel);
		}

		// 双击 或 手柄同时按下两边的摇杆,更新陀螺仪
		else if (tmp == double_click)
		{
			ImuData_copy(&imu.Deviation_gyro, &imu.gyro);
			ImuData_copy(&imu.Deviation_accel, &imu.accel);
		}

		// 长按 切换页面
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
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	// Retrieves the original data of the encoder
	// 获取编码器的原始数据
	float Encoder_A_pr, Encoder_B_pr, Encoder_C_pr, Encoder_D_pr;
	OriginalEncoder.A = Read_Encoder(2);
	OriginalEncoder.B = Read_Encoder(3);
	OriginalEncoder.C = Read_Encoder(4);
	OriginalEncoder.D = Read_Encoder(5);

	// 计算纠偏系数
	float LeftWheelDiff = wheelCoefficient(LineDiffParam, 1);
	float RightWheelDiff = wheelCoefficient(LineDiffParam, 0);

	// test_num=OriginalEncoder.B;

	// Decide the encoder numerical polarity according to different car models
	// 根据不同小车型号决定编码器数值极性
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
	// 编码器原始数据转换为车轮速度，单位m/s
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
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
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
	else if (vz == 0)
		smooth_control.VZ = smooth_control.VZ * 0.9f;

	smooth_control.VX = target_limit_float(smooth_control.VX, -float_abs(vx), float_abs(vx));
	smooth_control.VY = target_limit_float(smooth_control.VY, -float_abs(vy), float_abs(vy));
	smooth_control.VZ = target_limit_float(smooth_control.VZ, -float_abs(vz), float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
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
函数功能：防止电位器选错模式，导致初始化出错引发电机乱转。已停止使用
入口参数：无
返回  值：无
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error = 0;

	if (abs(MOTOR_A.Motor_Pwm) > 2500 || abs(MOTOR_B.Motor_Pwm) > 2500 || abs(MOTOR_C.Motor_Pwm) > 2500 || abs(MOTOR_D.Motor_Pwm) > 2500)
		error++;
	// If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
	// 如果连续6次接近满幅输出，判断为电机乱转，让电机失能
	if (error > 6)
		EN = 0, Flag_Stop = 1, robot_mode_check_flag = 1;
}

// PWM消除函数
void auto_pwm_clear(void)
{
	// 小车姿态简易判断
	float y_accle = (float)(imu.accel.y / 1671.84f); // Y轴加速度实际值
	float z_accle = (float)(imu.accel.z / 1671.84f); // Z轴加速度实际值
	float diff;

	// 计算Y、Z加速度融合值，该值越接近9.8，表示小车姿态越水平
	if (y_accle > 0)
		diff = z_accle - y_accle;
	else
		diff = z_accle + y_accle;

	//	debug_show_diff = diff;

	// PWM消除检测
	if (MOTOR_A.Target != 0.0f || MOTOR_B.Target != 0.0f || MOTOR_C.Target != 0.0f || MOTOR_D.Target != 0.0f)
	{
		start_check_flag = 1; // 标记需要清空PWM
		wait_clear_times = 0; // 复位清空计时
		start_clear = 0;	  // 复位清除标志

		// 运动时斜坡检测的数据复位
		clear_done_once = 0;
		clear_again_times = 0;
	}
	else // 当目标速度由非0变0时，开始计时 2.5 秒，若小车不在斜坡状态下，清空pwm
	{
		if (start_check_flag == 1)
		{
			wait_clear_times++;
			if (wait_clear_times >= 250)
			{
				// 小车在水平面上时才标记清空pwm，防止小车在斜坡上运动出现溜坡
				if (diff > 8.8f)
					start_clear = 1, clear_state = 0; // 开启清除pwm
				else
					clear_done_once = 1; // 小车在斜坡上，标记已完成清除

				start_check_flag = 0;
			}
		}
		else
		{
			wait_clear_times = 0;
		}
	}

	// 完成了清除后，若出现推车行为，pwm积累一定数值后将在10秒后再次清空
	if (clear_done_once)
	{
		// 小车接近于水平面时才作积累消除，防止小车在斜坡上溜车
		if (diff > 8.8f)
		{
			// 完成清除后pwm再次积累，重新清除
			if (int_abs(MOTOR_A.Motor_Pwm) > 300 || int_abs(MOTOR_B.Motor_Pwm) > 300 || int_abs(MOTOR_C.Motor_Pwm) > 300 || int_abs(MOTOR_D.Motor_Pwm) > 300)
			{
				clear_again_times++;
				if (clear_again_times > 1000)
				{
					clear_done_once = 0;
					start_clear = 1; // 开启清除pwm
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
