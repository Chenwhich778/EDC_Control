#ifndef __SERVO_PROTOCOL_H__
#define __SERVO_PROTOCOL_H__
#include "stdio.h"
#include "car_bsp.h"

#define BUF_SIZE 64
#define SERVO_TIMEOUT 100
#define GOAL_POSITION_ADDR 0x2A
#define READ_DATA 0x02
#define prime_x1 700
#define prime_y1 2250
#define max_servo 12000

extern char coord[BUF_SIZE];
extern volatile uint8_t coord_updated;

void Servo_SetPosition(uint8_t id, uint16_t position, uint16_t speed);

#endif

