/**
 * @file         key.h
 * @details      用于声明key.c对外开放的函数
 * @author       zjs
 */
#ifndef _KEY_H_
#define _KEY_H_
/*
===========================
头文件包括
===========================
*/
#include "ti_msp_dl_config.h"
/*
===========================
函数声明
===========================
*/
#define KEY_1 1
#define KEY_2 2
#define KEY_3 3
#define KEY_A 4
#define KEY_4 5
#define KEY_5 6
#define KEY_6 7
#define KEY_B 8
#define KEY_7 9
#define KEY_8 10
#define KEY_9 11
#define KEY_C 12
#define KEY_xin 13
#define KEY_0 14
#define KEY_jing 15
#define KEY_D 16

extern char key_buffer[64];
extern uint16_t key_index;
extern uint16_t count;
extern bool load_flag;
/* 获取矩阵键盘键值 */
int getKeyValue(void);
char get_keychar(uint8_t keyboard);
void load_input(void);
#endif