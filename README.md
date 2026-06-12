# complete_hero_chassis

基于 STM32F407 的 RoboMaster 英雄机器人底盘控制系统。

## 硬件平台

- **主控**: STM32F407VET6 (168MHz)
- **底盘电机**: 4× M3508 (CAN 总线，速度/位置双环 PID)
- **关节电机**: 4× DM4340 (MIT 模式力控)
- **IMU**: BMI088 (SPI 通信，Mahony 互补滤波 AHRS)
- **遥控**: DBUS 接收机 (UART + DMA)

## 系统架构

```
complete_hero_chassis/
├── Core/               # CubeMX 生成的 HAL 外设初始化
│   ├── Inc/            # 外设头文件 (CAN, SPI, UART, TIM, DMA, GPIO)
│   └── Src/            # 外设源文件 + main.c
├── app/                # 应用层
│   ├── control_dbus    # 遥控器数据解析 + 麦克纳姆轮解算
│   ├── dm_joint_ctrl   # 四条腿关节协调控制 (高度/Pitch/Roll)
│   └── imu_data        # Mahony 姿态解算 + 欧拉角转换
├── bsp/                # 板级支持包
│   ├── bsp_imu         # BMI088 驱动 (加速度 & 陀螺仪读取/校准)
│   └── drv_can         # CAN 滤波配置封装
├── drv/                # 电机驱动
│   ├── M3508           # M3508 速度/位置 PID 控制
│   ├── dm_motor        # DM4340 MIT 模式驱动 (位置/速度/力矩)
│   └── dbus            # DBUS 协议解析
└── Drivers/            # CMSIS + STM32F4 HAL 库
```

## 功能特性

- **麦克纳姆轮底盘**: 遥控器控制全向移动 (X/Y 平移 + 旋转)
- **云台跟随模式**: 根据 IMU 偏航角实现底盘随云台旋转
- **四腿姿态控制**: 高度/Pitch/Roll 三环并联 PID，合力分配到 4 条腿
- **抬腿状态机**: 前腿/后腿依次抬放 (`lift_f_leg` → `lift_b_leg` → `normal`)
- **IMU 姿态解算**: Mahony 互补滤波，500 次初始对准校准
- **CAN 双总线**: CAN1 接收 M3508 编码器，CAN2 控制 DM4340 + M3508

## TODO

- [ ] 功率控制（目前尚未实现）

## 控制参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kp_h` / `kd_h` | 40.0 / 1.0 | 高度 PD 控制 |
| `kp_pitch` / `kd_pitch` | 0.4 / 0.05 | 俯仰 PD 控制 |
| `kp_roll` / `kd_roll` | 0.4 / 0.02 | 横滚 PD 控制 |
| `tar_h` | 0.76m | 目标高度 |

## 开发环境

- **IDE**: Keil MDK-ARM v5
- **工具链**: ARM Compiler 5 / ARM Compiler 6
- **HAL 库**: STM32F4 HAL Driver
- **调试器**: J-Link / ST-Link (SWD)

## 快速开始

1. 用 Keil MDK-ARM 打开 `MDK-ARM/complete_hero_chassis.uvprojx`
2. 编译并下载到开发板
3. 确认 BMI088 上电自检通过（温度稳定在 40°C 左右）
4. 连接 DBUS 遥控器接收机
5. 上电后 500ms IMU 自校准完成即可操作

## License

MIT License
