/**
  ******************************************************************************
  * @file    dbus.h
  * @brief   This file contains all the function prototypes for
  *          the dbus.c file
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DBUS_H__
#define __DBUS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define DBUS_BUFFER_SIZE 18
//#define DBUS_IWDG

void dbus_init(UART_HandleTypeDef* huart_ptr, DMA_HandleTypeDef* hdma_usart_rx_ptr);
void dbus_decode();
void dbus_callback();

typedef struct
{
	struct
	{
		uint16_t CH[4];
		uint8_t s1;
		uint8_t s2;
		uint16_t rolling_wheel;
	}channel;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t left_key;
		uint8_t right_key;
	}mouse;
	struct
	{
		uint8_t w;
		uint8_t s;
		uint8_t a;
		uint8_t d;
		uint8_t q;
		uint8_t e;
		uint8_t shift;
		uint8_t ctrl;
	}keyboard;
}dbus_struct;

extern dbus_struct dbus_ctrl_data;

#ifdef __cplusplus
}
#endif
#endif /*__ DBUS_H__ */

