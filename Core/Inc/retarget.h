/*
 * retarget.h
 *
 *  Created on: Sep 23, 2021
 *      Author: LENOVO
 */

#ifndef _RETARGET_H__
#define _RETARGET_H__

#include "stm32f3xx_hal.h"
#include <sys/stat.h>

void RetargetInit(UART_HandleTypeDef *huart);
int _isatty(int fd);
int _write(int fd, char* ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char* ptr, int len);
int _fstat(int fd, struct stat* st);

#endif //#ifndef _RETARGET_H__
