//
// Created by 33140 on 2026/10/21.
//

#ifndef COMPLETE_HERO_CHASSIS_VOFA_H
#define COMPLETE_HERO_CHASSIS_VOFA_H

#include "stm32f4xx_hal.h"

//图片格式宏定义
enum ImgFormat {
 Format_Invalid,Format_Mono,Format_MonoLSB,Format_Indexed8,
 Format_RGB32,Format_ARGB32,Format_ARGB32_Premultiplied,Format_RGB16,Format_ARGB8565_Premultiplied,
 Format_RGB666,Format_ARGB6666_Premultiplied,Format_RGB555,Format_ARGB8555_Premultiplied,Format_RGB888,
 Format_RGB444,Format_ARGB4444_Premultiplied,Format_RGBX8888,Format_RGBA8888,Format_RGBA8888_Premultiplied,Format_BGR30,
 Format_A2BGR30_Premultiplied,Format_RGB30,Format_A2RGB30_Premultiplied,Format_Alpha8,Format_Grayscale8,

 // 以下格式发送时，IMG_WIDTH和IMG_HEIGHT不需要强制指定，设置为-1即可
 Format_BMP,Format_GIF,Format_JPG,Format_PNG,Format_PBM,Format_PGM,Format_PPM,Format_XBM,Format_XPM,Format_SVG,
};

//照片发送前导帧
typedef struct {
   int IMG_ID,       // 图片通道 ID
   IMG_SIZE,     // 图片数据字节数
   IMG_WIDTH,    // 图片宽度
   IMG_HEIGHT,   // 图片高度
   IMG_FORMAT,   // 图片格式
   tail1,
   tail2;
}vofa_image_header_t;

//照片发送包装帧
typedef struct {
    vofa_image_header_t header;
    uint8_t data[];
}vofa_image_packet_t;

/**
 * @brief 向vofa上位机用串口dma发送float类型数据
 * @param data 要发送的数据
 * @param count float 元素个数，范围为 1～1024。
* @return HAL_OK 已启动发送；HAL_BUSY 发送未就绪；
 *         HAL_ERROR 参数、配置或启动错误。
 */
HAL_StatusTypeDef vofa_send_float(const float *data, uint16_t count);

/**
 * @brief 照片发送前导帧构建
 * @param header 前导帧实例指针
 * @param id 图片通道编号
 * @param size 图片数据的字节数，不包含前导帧
 * @param width 图片宽度，单位是像素
 * @param height 图片高度，单位是像素
 * @param format 图片格式，取 enum ImgFormat 中的值。
 */
void vofa_image_packet_config(vofa_image_header_t* header ,  int id ,
            int size , int  width , int height, enum ImgFormat format);

/**
 * @brief vofa照片帧发送
 * @param packet packet 完整图片包指针，data 至少包含 IMG_SIZE 字节有效数据。
* @return HAL_OK 已启动发送；HAL_BUSY 发送未就绪；
 *         HAL_ERROR 参数、配置或启动错误。
 */
HAL_StatusTypeDef vofa_send_image(vofa_image_packet_t* packet);



#endif //COMPLETE_HERO_CHASSIS_VOFA_H
