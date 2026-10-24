//
// Created by 33140 on 2026/10/21.
//

//默认用usart1，懒得改了

#include "vofa.h"
#include "usart.h"
#include <string.h>

#define VOFA_MAX_COUNT 1024U

static uint8_t tx_buffer[VOFA_MAX_COUNT * sizeof(float) + 4U];
static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

const int tail1  = 0x7F800000, tail2 = 0x7F800000; //照片发送前导帧帧尾

// 返回1表示发送未就绪，返回0表示可以发送
static uint8_t VOFA_IsBusy(void)
{
    return huart1.gState != HAL_UART_STATE_READY;  //进usart1中断HAL会自己改
}


HAL_StatusTypeDef vofa_send_float(const float *data, uint16_t count) {
    if (data == NULL || count == 0 || count > VOFA_MAX_COUNT ||
        huart1.hdmatx == NULL) {
        return HAL_ERROR;
        }

    if (VOFA_IsBusy()) {return HAL_BUSY;}

    uint16_t size = (uint16_t)(count * sizeof(float));

    memcpy(tx_buffer, data, size);
    memcpy(tx_buffer + size, tail, sizeof(tail));

    return HAL_UART_Transmit_DMA(&huart1, tx_buffer,size + sizeof(tail));
}

void vofa_image_packet_config(vofa_image_header_t* header
,int id , int size , int  width , int height , enum ImgFormat format) {
    if (header == NULL) {return;}
    header->IMG_ID = id;
    header->IMG_WIDTH = width;
    header->IMG_HEIGHT = height;
    header->IMG_SIZE = size;
    header->IMG_FORMAT = format;
    header->tail1 = tail1;
    header->tail2 = tail2;
}

static uint8_t check_packet(vofa_image_packet_t* packet) {
    if (packet->header.tail1 != tail1 || packet->header.tail2 != tail2
    || packet->header.IMG_SIZE <= 0) {return 1;}
    return 0;
}

HAL_StatusTypeDef vofa_send_image(vofa_image_packet_t* packet) {
    if (packet == NULL ||  huart1.hdmatx == NULL) {return HAL_ERROR;}
    if (check_packet(packet)){return HAL_ERROR;}

    if (VOFA_IsBusy()) {return HAL_BUSY;}

    const size_t header_size = sizeof(packet->header);
    const size_t data_size = (size_t)packet->header.IMG_SIZE;
    const size_t full_size = header_size + data_size;
    if (full_size > sizeof(tx_buffer) || full_size > UINT16_MAX) {return HAL_ERROR;}

    memcpy(tx_buffer, &packet->header, header_size);
    memcpy(tx_buffer + header_size, packet->data,data_size);

    return HAL_UART_Transmit_DMA(&huart1, tx_buffer,(uint16_t)full_size);
}