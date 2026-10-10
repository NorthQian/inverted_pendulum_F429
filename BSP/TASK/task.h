#ifndef _TASK_H_
#define _TASK_H_

#include "main.h"
#include "pid.h"

#define POS_TARGET_STEP 0.05f   // KEY1/KEY2 每次按键的位置目标增量（rad）

void pendulum_pid_init(void);              // 串级PID初始化
void pendulum_pid_enable(uint8_t en);      // 启动/停止平衡控制
void pendulum_pid_toggle(void);            // 翻转运行状态（按键切换用）
void pendulum_pid_set_position(float pos); // 设置外环位置目标（连续坐标，rad）
void pendulum_pid_step_position(float delta); // 位置目标相对步进（按键加减）
float pendulum_angle_rad(uint16_t adc);    // ADC原始值 -> 相对竖直向上的角度（rad）
uint8_t pendulum_pid_get_state(void);      // 读取运行状态（0=停止, 1=运行）
void motor_pos_zero(void);                 // 将当前编码器位置映射为0（记录零点偏移）
float motor_get_pos(void);                 // 返回映射后的位置（rad）

extern float angle;                        // 摆杆角度偏差（counts），1ms中断更新
extern float motor_pos;                    // 映射后的电机位置（rad），1ms中断更新
extern pid_type_def pos_pid;               // 外环位置环（供 LCD 显示目标用）

#endif
