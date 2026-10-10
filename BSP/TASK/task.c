#include "task.h"
#include "pid.h"
#include "dm_motor_drv.h"
#include "can.h"

extern motor_t motor[num];
extern void dm_motor_ctrl_send(hcan_t *hcan, motor_t *motor);
extern void dm_motor_disable(hcan_t *hcan, motor_t *motor);

/* 摆杆角度传感器原始值（0~4095），由 ADC1 + DMA 后台循环采集 */
extern uint32_t ADC_Value;

/* ========================= 传感器标定 ========================= */
int16_t PENDULUM_CENTER_ANGLE = 3880;   // 摆杆竖直向上时的 ADC 原始值（实测）3859 二次3880
#define PENDULUM_ADC_MAX 4096.0f        // 12位ADC满量程（0~4095）
#define PENDULUM_SENSOR_SPAN 6.2831853f // 360°满量程 = 2π（回绕式电位器）
#define PENDULUM_FALL_ANGLE 1.5708f     // 倒杆判定：|角度|超过π/2（约±90°，偏过水平）即停机
#define PENDULUM_DIR 1.0f               // 电机转向系数，摆杆反方向失衡时改为 -1.0f

/* ========================= 串级PID参数（需整定） ========================= */
/* 内环·角度环（5ms）：输入单位 rad，输出 -> 电机速度 vel_set（rad/s） */
#define ANGLE_KP 40.2f // 纯KP使用67
#define ANGLE_KI 1.227f
#define ANGLE_KD 329.0f // 注意：模板的D项是"每采样差分"，量级随 1/5ms 放大
#define ANGLE_MAX_OUT 200.0f
#define ANGLE_MAX_IOUT 50.0f

/* 外环·位置环（50ms）：输入单位 rad，输出 -> 目标倾斜角（rad） */
#define POS_KP 0.005f
#define POS_KI 0.0f
#define POS_KD 0.00f
#define POS_MAX_OUT 0.5f // 允许的最大倾斜角（rad，约28°）
#define POS_MAX_IOUT 0.1f
/* =============================================================== */

pid_type_def angle_pid; // 内环：角度环（摆杆平衡）
pid_type_def pos_pid;   // 外环：位置环（横杆定位）
uint8_t run_state = 0;  // 0=停止, 1=运行
float motor_pos = 0;
float angle = 0;
uint16_t delay_position = 0;
uint8_t delay_position_register = 0;
/**
 * @brief ADC 原始值 -> 相对竖直向上的有符号角度（rad），竖直向上 = 0
 */
float pendulum_angle_rad(uint16_t adc)
{
    int32_t diff = (int32_t)adc - (int32_t)PENDULUM_CENTER_ANGLE; // 有符号角差

    /* 回绕处理：传感器是 360° 连续电位器，跨过 0/4095 边界时取最短角差 */
    if (diff > 2048)
        diff -= 4096;
    if (diff < -2048)
        diff += 4096;

    return (float)diff * (PENDULUM_SENSOR_SPAN / PENDULUM_ADC_MAX); // 单位 rad
}

/**
 * @brief 串级PID初始化（在 dm_motor_init 之后调用）
 */
void pendulum_pid_init(void)
{
    PID_init(&angle_pid, PID_POSITION, ANGLE_KP, ANGLE_KI, ANGLE_KD,
             ANGLE_MAX_OUT, ANGLE_MAX_IOUT, 0.9f); // 角度环D项滤波较轻，响应更快
    PID_init(&pos_pid, PID_POSITION, POS_KP, POS_KI, POS_KD,
             POS_MAX_OUT, POS_MAX_IOUT, 0.9f);

    angle_pid.set = 0.0f; // 内环目标 = 竖直向上（0 rad）
    pos_pid.set = 0.0f;   // 外环目标 = 横杆回到零位

    run_state = 0; // 上电先停止，避免电机乱动
}

/**
 * @brief 启动/停止串级PID（可用按键等触发）
 */
void pendulum_pid_enable(uint8_t en)
{
    run_state = en ? 1 : 0;
}

/**
 * @brief 翻转运行状态（按键每触发一次翻转一次）
 */
void pendulum_pid_toggle(void)
{
    run_state = run_state ? 0 : 1;
}

/**
 * @brief 设置外环位置目标（单位：rad，与 motor[Motor1].para.pos 一致）
 */
void pendulum_pid_set_position(float pos)
{
    pos_pid.set = pos;
}

/**
 * @brief 读取运行状态（0=停止, 1=运行）
 */
uint8_t pendulum_pid_get_state(void)
{
    return run_state;
}

static float motor_pos_offset = 0.0f; // 编码器零点偏移（rad）

/**
 * @brief 将当前编码器位置映射为0（记录零点偏移）
 */
void motor_pos_zero(void)
{
    motor_pos_offset = motor[Motor1].para.pos;
}

/**
 * @brief 返回映射后的位置（原始位置 - 零点偏移）
 */
float motor_get_pos(void)
{
    return motor[Motor1].para.pos - motor_pos_offset;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) // 1ms 进一次
    {
        static uint16_t cnt_angle = 0; // 角度环 5ms 分频
        static uint16_t cnt_pos = 0;   // 位置环 50ms 分频

        angle = pendulum_angle_rad((uint16_t)ADC_Value); // 有符号角度（rad）
        motor_pos = motor_get_pos();                     // 横杆位置（rad，已映射零点）

        /* 倒杆自动停机：偏过水平（|diff|>1024，即 -2048~-1024 或 1024~2048）即停机 */
        if (angle > PENDULUM_FALL_ANGLE || angle < -PENDULUM_FALL_ANGLE)
        {
            run_state = 0;
            delay_position = 0;
            delay_position_register = 0;
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        }

        if (run_state)
        {
            /* 内环·角度环，5ms 一次，输出作为电机速度给定 */
            if (++cnt_angle >= 2)
            {
                cnt_angle = 0;
                angle_pid.fdb = angle;
                angle_pid.out = PID_calc(&angle_pid);
                motor[Motor1].ctrl.vel_set = angle_pid.out;
                // motor[Motor1].ctrl.vel_set = 0.0f;   // 暂时不让电机动，避免倒杆
            }

            if (run_state == 1)
            {
                if (++delay_position >= 3000)
                {
                     delay_position = 3100;
                    delay_position_register = 1;
                }
                else
                {
                }
            }
            /* 外环·位置环，20ms 一次，输出作为目标倾斜角（串级） */
            if (++cnt_pos >= 8)
            {
                if (delay_position_register == 1)
                {

                    cnt_pos = 0;
                    pos_pid.fdb = motor_pos;
                    pos_pid.out = PID_calc(&pos_pid);
                    angle_pid.set = pos_pid.out + 0.0f; // 添加一个小的偏置，使摆杆更容易保持直立
                                                        // angle_pid.set =0.0f; // 直接让摆杆保持竖直，避免位置环干扰角度环
                    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
                }
                else
                {
                    angle_pid.set = 0.0f;
                    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
                }
            }
        }
        else
        {
            motor[Motor1].ctrl.vel_set = 0.0f; // 停机，电机速度给定清零
            PID_clear(&angle_pid);             // 停机，角度环PID清零
            // dm_motor_disable(&hcan1, &motor[Motor1]); // 停机，电机使能清零
        }
        dm_motor_ctrl_send(&hcan1, &motor[Motor1]);
    }
}
