#ifndef __SERVO_PROTOCOL_H__
#define __SERVO_PROTOCOL_H__

#include "stm32f4xx_hal.h"

#define BUF_SIZE 64
#define SERVO_TIMEOUT 100
#define GOAL_POSITION_ADDR 0x2A
#define READ_DATA 0x02

extern char coord[BUF_SIZE];
extern volatile uint8_t coord_updated;

void Servo_Init(UART_HandleTypeDef *huart);
void Servo_SetPosition(uint8_t id, uint16_t position, uint16_t speed);
uint16_t Servo_ReadPosition(uint8_t id);
void Servo_GetCoord(char *dest, uint16_t max_len);
void Servo_SetREDLaser(uint8_t id1, uint16_t pos1, uint16_t speed1, 
                            uint8_t id2, uint16_t pos2, uint16_t speed2);

#endif

