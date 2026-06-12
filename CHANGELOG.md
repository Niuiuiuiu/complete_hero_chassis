# Changelog

## v0.2 — 功率计驱动 & 串口输出 (2026-06-12)

### 新增
- **功率计驱动** (`drv/power_meter.c/.h`)
  - 支持吴张扬 (WZY) 和欧阳云翔 (OUYANG) 两种功率计
  - CAN ID `0x212` 接收，解析电池电压/电流/底盘实时功率
  - 低通滤波 (`alpha=0.1`) 平滑功率数据
  - 支持设定功率上限、目标功率并回传

### 变更
- **`main.c`**: 主循环 USART6 输出从 IMU 姿态数据切换为底盘功率 `P_x1W_chassis`
  - `power_send[0]` = 底盘实时功率 (W)
  - 发送格式: 3×float + tail `{0x00,0x00,0x80,0x7f}` = 16 字节/帧
- **`drv_can.c`**: RxFifo1 回调新增 `POWERMETER_CAN_RX_ID` 分发
- **CAN 滤波掩码**: `0x7F0` → `0x700`（收紧单 ID 匹配精度）

### 清理
- `.gitignore` 新增排除编译产物目录 (`MDK-ARM/complete_hero_chassis/`, `MDK-ARM/RTE/`)

---

## v0.1 — 初始版本 (2026-06-12)

- STM32F407 英雄机器人底盘控制系统
- M3508 麦克纳姆轮底盘 (CAN 总线, 速度/位置双环 PID)
- DM4340 四腿关节 MIT 模式力控
- BMI088 IMU 姿态解算 (Mahony 互补滤波)
- DBUS 遥控接收, 云台跟随模式
- 抬腿状态机 (前/后腿依次抬放)
