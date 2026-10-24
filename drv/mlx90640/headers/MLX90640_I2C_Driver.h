/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef _MLX90640_I2C_Driver_H_
#define _MLX90640_I2C_Driver_H_

#include <stdint.h>
#include "MLX90640_API.h"

    /* 初始化 I2C 外设及 SDA、SCL 引脚。 */
    extern void MLX90640_I2CInit(void);

    /* 向通用呼叫地址 0x00 发送 0x06，复位同总线上支持该命令的设备；0 成功，非 0 失败。 */
    extern int MLX90640_I2CGeneralReset(void);

    /* 从 startAddress 开始读取 nMemAddressRead 个 16 位字到 data；slaveAddr 为未左移的 7 位设备地址。
     * 返回约定：0 成功，-1 通信 NACK或参数或通信失败 ；底层负责将接收字节正确组合成 16 位数据。 */
    extern int MLX90640_I2CRead(uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data);

    /* 向 writeAddress 写入一个 16 位 data 并读回校验；slaveAddr 为未左移的 7 位设备地址。
     * 返回约定：0 成功，-1 通信 NACK，-2 校验不一致。 */
    extern int MLX90640_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data);

    /* 调整 I2C 通信频率 ， 单位是khz；freq 的单位由底层实现约定，不是传感器图像刷新率。 */
    extern void MLX90640_I2CFreqSet(int freq);
#endif
