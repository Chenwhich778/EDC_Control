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

/* 获取矩阵键盘键值 */
int getKeyValue(void);

#endif