# J4310-X-Head-F4

达妙（DM）电机驱动的 STM32F429 项目（M1/M2/M3 三电机）。

## 环境
- MCU：STM32F429IGT6
- 工具链：Keil MDK-ARM（工程位于 `MDK-ARM/`）
- 通信：经典 CAN1 @ 1Mbps，DM 电机 MIT 协议

## 目录结构
```
Core/        应用主逻辑（main.c 等）
BSP/         板级驱动（DM_Motor、LCD、KEY、Motion 等）
Drivers/     STM32 HAL/CMSIS 库
MDK-ARM/     Keil 工程与编译输出（输出内容已 gitignore）
```

## 功能
- 驱动 3 路 DM 电机（M1=0x01/0x11，M2=0x02/0x12，M3=0x03/0x13）
- KEY2 切换模式：正弦模式 / neck_anim 动画模式
- KEY3 使能 / 失能电机
- 屏幕显示各电机 pos/vel/kp/kd/扭矩/温度
- LED 指示：正弦亮红灯，动画亮绿灯，失能全灭

## 模式
| 模式 | 说明 |
|------|------|
| 正弦 | M1 基础正弦、M2 反向、M3 单独幅度 |
| 动画 | 按 `BSP/Motion/neck_anim` 序列约 30fps 播放 |
