/**
  ******************************************************************************
  * @file    dbus.c
	* @author  jingqiao
  * @brief   recevice and decode dbus.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dbus.h"
#include "usart.h"
#include "dma.h"
#ifdef DBUS_IWDG
#include "iwdg.h"
#endif

// 修正：显式初始化为0，兼容所有编译器
uint8_t dbus_buffer[DBUS_BUFFER_SIZE] = {0};
dbus_struct dbus_ctrl_data = {0};

DMA_HandleTypeDef* dbus_hdma_usart_rx_ptr;
UART_HandleTypeDef* dbus_huart_ptr;

void dbus_init(UART_HandleTypeDef* huart_ptr, DMA_HandleTypeDef* hdma_usart_rx_ptr)
{
	dbus_hdma_usart_rx_ptr = hdma_usart_rx_ptr;
	dbus_huart_ptr = huart_ptr;
	// 仅开启IDLE中断（删除冗余的RXNE中断）
	__HAL_UART_ENABLE_IT(dbus_huart_ptr, UART_IT_IDLE);
	HAL_UART_Receive_DMA(dbus_huart_ptr, dbus_buffer, DBUS_BUFFER_SIZE);
	dbus_ctrl_data.channel.CH[0] = 0x400;
	dbus_ctrl_data.channel.CH[1] = 0x400;
	dbus_ctrl_data.channel.CH[2] = 0x400;
	dbus_ctrl_data.channel.CH[3] = 0x400;
	dbus_ctrl_data.channel.rolling_wheel = 0x400;
}

void dbus_decode()
{
	// 修正：所有二进制数替换为十六进制，修复笔误0xb00000001
	dbus_ctrl_data.channel.CH[0] = dbus_buffer[0] | ((dbus_buffer[1] & 0x07) << 8);//[0]all, [1]high 3
	dbus_ctrl_data.channel.CH[1] = ((dbus_buffer[1] & 0xF8) >> 3) | ((dbus_buffer[2] & 0x3F) << 5);//[1]low 5, [2]high 6
	dbus_ctrl_data.channel.CH[2] = ((dbus_buffer[2] & 0xC0) >> 6) | ((dbus_buffer[3] & 0xFF) << 2) | ((dbus_buffer[4] & 0x01) << 10);//[2]low 2, [3]all, [4]high 1
	dbus_ctrl_data.channel.CH[3] = ((dbus_buffer[4] & 0xFE) >> 1) | ((dbus_buffer[5] & 0x0F) << 7);//[4]low 7, [5]high 4
	dbus_ctrl_data.channel.s1 = ((dbus_buffer[5] & 0xC0) >> 6);
	dbus_ctrl_data.channel.s2 = ((dbus_buffer[5] & 0x30) >> 4);
	dbus_ctrl_data.mouse.x = dbus_buffer[6] | (dbus_buffer[7] << 8);
	dbus_ctrl_data.mouse.y = dbus_buffer[8] | (dbus_buffer[9] << 8);
	dbus_ctrl_data.mouse.z = dbus_buffer[10] | (dbus_buffer[11] << 8);
	dbus_ctrl_data.mouse.left_key = dbus_buffer[12];
	dbus_ctrl_data.mouse.right_key = dbus_buffer[13];
	dbus_ctrl_data.keyboard.w = dbus_buffer[14] & 0x01;
	dbus_ctrl_data.keyboard.s = (dbus_buffer[14] & 0x02) >> 1;
	dbus_ctrl_data.keyboard.a = (dbus_buffer[14] & 0x04) >> 2;
	dbus_ctrl_data.keyboard.d = (dbus_buffer[14] & 0x08) >> 3;
	dbus_ctrl_data.keyboard.shift = (dbus_buffer[14] & 0x10) >> 4;
	dbus_ctrl_data.keyboard.ctrl = (dbus_buffer[14] & 0x20) >> 5;
	dbus_ctrl_data.keyboard.q = (dbus_buffer[14] & 0x40) >> 6;
	dbus_ctrl_data.keyboard.e = (dbus_buffer[14] & 0x80) >> 7;
	dbus_ctrl_data.channel.rolling_wheel = (dbus_buffer[17] << 8) | dbus_buffer[16];
}

void dbus_callback()
{
	if (__HAL_UART_GET_FLAG(dbus_huart_ptr, UART_FLAG_IDLE) != RESET)
	{
#ifdef DBUS_IWDG
		HAL_IWDG_Refresh(&hiwdg);
#endif
		__HAL_UART_CLEAR_IDLEFLAG(dbus_huart_ptr);
		HAL_UART_DMAStop(dbus_huart_ptr);
		if (__HAL_DMA_GET_COUNTER(dbus_hdma_usart_rx_ptr) == 0)
			dbus_decode();
		HAL_UART_Receive_DMA(dbus_huart_ptr, dbus_buffer, DBUS_BUFFER_SIZE);
	}
}
