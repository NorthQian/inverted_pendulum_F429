#include "task.h"
#include "pid.h"
#include "dm_motor_drv.h"
#include "can.h"

extern motor_t motor[num];
extern void dm_motor_ctrl_send(hcan_t* hcan, motor_t *motor);


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
       
        dm_motor_ctrl_send(&hcan1, &motor[Motor1]);
    }
}