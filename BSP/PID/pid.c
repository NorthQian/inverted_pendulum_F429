/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "math.h"

#define LimitMax(input, max) \
  {                          \
    if (input > max)         \
    {                        \
      input = max;           \
    }                        \
    else if (input < -max)   \
    {                        \
      input = -max;          \
    }                        \
  }



pid_type_def chassis_speed_x_pid;
pid_type_def chassis_speed_y_pid;
pid_type_def chassis_yaw_pid;

/**
 * @brief          pid struct data init
 * @param[out]     pid: PID�ṹ����ָ��
 * @param[in]      mode: PID_POSITION:��ͨPID
 *                 PID_DELTA: ���PID
 * @param[in]      Kp Ki Kd
 * @param[in]      max_out: pid������
 * @param[in]      max_iout: pid���������
 * @retval         none
 */
void PID_init(pid_type_def *pid, uint8_t mode, const float kp, const float ki, const float kd, float max_out,
              float max_iout, float alpha)
{
    if (pid == NULL) {
        return;
    }

    pid->mode = mode;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;

    pid->alpha = alpha;
    pid->fdb_prev = 0.0f;
    pid->fdb_buf[0] = pid->fdb_buf[1] = pid->fdb_buf[2] = 0.0f;
    pid->I_band = 0.0f;
}

/**
 * @brief          pid calculate
 * @param[out]     pid: PID struct data point
 * @param[in]      ref: feedback data
 * @param[in]      set: set point
 * @retval         pid out
 */
float PID_calc(pid_type_def *pid)
{
    if (pid == NULL) {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = pid->set - pid->fdb;

    if (pid->mode == PID_POSITION) {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];

        
        float raw_D = pid->Kd * (pid->fdb_prev - pid->fdb);
        pid->Dout = pid->alpha * pid->Dout + (1.0f - pid->alpha) * raw_D;
        pid->fdb_prev = pid->fdb;

        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    } else if (pid->mode == PID_DELTA) {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];

        
        float raw_D = pid->Kd * (-(pid->fdb_buf[0] - 2.0f * pid->fdb_buf[1] + pid->fdb_buf[2]));
        pid->Dout = pid->alpha * pid->Dout + (1.0f - pid->alpha) * raw_D;

       
        pid->fdb_buf[2] = pid->fdb_buf[1];
        pid->fdb_buf[1] = pid->fdb_buf[0];
        pid->fdb_buf[0] = pid->fdb;

        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }

    return pid->out;
}

/**
 * @brief          pid out clear
 * @param[out]     pid: PID struct data point
 * @retval         none
 */

void PID_clear(pid_type_def *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;

    pid->fdb_prev = 0.0f;
    pid->fdb_buf[0] = pid->fdb_buf[1] = pid->fdb_buf[2] = 0.0f;
}

/**
 * @brief          pid init with integral separation
 * @param[out]     pid: PID struct data point
 * @param[in]      mode: PID_POSITION / PID_DELTA
 * @param[in]      Kp Ki Kd
 * @param[in]      max_out: pid max out
 * @param[in]      max_iout: pid max iout
 * @param[in]      alpha: D term low-pass filter coefficient
 * @param[in]      I_band: integral separation threshold (|error| > I_band -> I disabled)
 * @retval         none
 */
void PID_init_ilimit(pid_type_def *pid, uint8_t mode, const float kp, const float ki, const float kd,
                     float max_out, float max_iout, float alpha, float I_band)
{
    if (pid == NULL) {
        return;
    }

    PID_init(pid, mode, kp, ki, kd, max_out, max_iout, alpha);
    pid->I_band = I_band;
}

/**
 * @brief          pid calculate with integral separation
 * @param[out]     pid: PID struct data point
 * @retval         pid out
 */
float PID_calc_ilimit(pid_type_def *pid)
{
    if (pid == NULL) {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = pid->set - pid->fdb;

    if (pid->mode == PID_POSITION) {
        pid->Pout = pid->Kp * pid->error[0];

        
        if (fabsf(pid->error[0]) > pid->I_band) {
            pid->Iout = 0.0f;
        } else {
            pid->Iout += pid->Ki * pid->error[0];
        }

        float raw_D = pid->Kd * (pid->fdb_prev - pid->fdb);
        pid->Dout = pid->alpha * pid->Dout + (1.0f - pid->alpha) * raw_D;
        pid->fdb_prev = pid->fdb;

        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    } else if (pid->mode == PID_DELTA) {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);

       
        if (fabsf(pid->error[0]) > pid->I_band) {
            pid->Iout = 0.0f;
        } else {
            pid->Iout = pid->Ki * pid->error[0];
        }

        float raw_D = pid->Kd * (-(pid->fdb_buf[0] - 2.0f * pid->fdb_buf[1] + pid->fdb_buf[2]));
        pid->Dout = pid->alpha * pid->Dout + (1.0f - pid->alpha) * raw_D;

        pid->fdb_buf[2] = pid->fdb_buf[1];
        pid->fdb_buf[1] = pid->fdb_buf[0];
        pid->fdb_buf[0] = pid->fdb;

        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }

    return pid->out;
}

/**
 * @brief    ����P����
 */
void PID_Set_P(pid_type_def *pid, float Kp)
{
    pid->Kp = Kp;
}
/**
 * @brief    ����I����
 */
void PID_Set_I(pid_type_def *pid, float Ki)
{
    pid->Ki = Ki;
}
/**
 * @brief    ����D����
 */
void PID_Set_D(pid_type_def *pid, float Kd)
{
    pid->Kd = Kd;
}
/**
 * @brief    ����Ŀ��ֵ
 */
void PID_Set_Target(pid_type_def *pid, float set)
{
    pid->set = set;
}
/**
 * @brief    ��ʵ��ֵд��PID��
 */
void PID_Set_Fact(pid_type_def *pid, float ref)
{
    pid->fdb = ref;
}
