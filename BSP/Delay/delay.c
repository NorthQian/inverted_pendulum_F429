#include"delay.h"

extern  TIM_HandleTypeDef htim7;
#define DLY_TIM_Handle  (&htim7)

/**
  * @brief  提供微秒级延时函数
  * @note  需要打开一个定时器 并调整其计数为1MHZ频率 不用开中断
  */

void delay_us(uint16_t nus)
{
    __HAL_TIM_SET_COUNTER(DLY_TIM_Handle, 0);
    __HAL_TIM_ENABLE(DLY_TIM_Handle);

    while (__HAL_TIM_GET_COUNTER(DLY_TIM_Handle) < nus) {
    }

    __HAL_TIM_DISABLE(DLY_TIM_Handle);
}

