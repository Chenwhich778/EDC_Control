#ifndef LASER_TRACKER_H
#define LASER_TRACKER_H


#include "servo_protocol.h"
#include "stdio.h"
#include "sys.h"
#include "system.h"


// PID����������

#define KP_x 0.05    // ����ϵ��1.25
#define KI_x 0.05  // ����ϵ��0.1
#define KD_x 0.01   // ΢��ϵ��0.2

#define KP_y 0.8    // ����ϵ��0.5
#define KI_y 0.04   // ����ϵ��0.04
#define KD_y 0.08   // ΢��ϵ��0.08


// �߽�����
#define MIN_X (center_x1+general_angle/360*4096-400)
#define MAX_X (center_x1+general_angle/360*4096+400)
#define MIN_Y (center_y1 - 400)
#define MAX_Y (center_y1 + 400)
#define MAX_INTER_X (MAX_X-MIN_X+1)/KI_x
#define MAX_INTER_Y (MAX_Y-MIN_Y+1)
#define MAX_FINE_ADJUST 2048  // 4096 * 0.5
//��������
extern float general_angle;

extern uint16_t servo[4];
extern uint8_t find_target;
extern uint8_t track_flag;
extern float search_delt;
extern uint16_t lose_time;
extern int laser_x;
extern int laser_y;


// ��������
void laser_track(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4);
#endif

