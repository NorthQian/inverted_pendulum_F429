/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
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
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H


#include "main.h"
#include "stdlib.h"




enum PID_MODE {
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct {
    uint8_t mode;
    //PID ������
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������

    float set;
    float fdb;  //ʵ��ֵ

    float out;
    float Pout;
    float Iout;
    float Dout;
    float error[3]; //��� 0=��ǰ, 1=��һ��, 2=���ϴ�

    // ������ֵ��
    float fdb_prev;       // ��һ�η���ֵ��λ��ʽ�ã�
    float fdb_buf[3];     // ��������ֵ��ʷ 0=��ǰ, 1=��һ��, 2=���ϴΣ�����ʽ�ã�

    // ���˲�
    float alpha;          // �˲�ϵ�� (0~1, ����ֵ 0.9)

    // ���ַ���
    float I_band;         // ���ַ�����ֵ (|error| > I_band ʱ�رջ���)
} pid_type_def;





/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
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
              float max_iout, float alpha);

/**
  * @brief          pid calculate
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @note           �ڵ�����һ��֮ǰӦ�ô����µ��趨ֵ��ʵ��ֵ
  * @retval         pid���
  */
float PID_calc(pid_type_def *pid);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
  * @retval         none
  */
void PID_clear(pid_type_def *pid);


void PID_Set_P(pid_type_def *pid, float Kp);
void PID_Set_I(pid_type_def *pid, float Ki);
void PID_Set_D(pid_type_def *pid, float Kd);
void PID_Set_Target(pid_type_def *pid, float set);
void PID_Set_Fact(pid_type_def *pid, float ref);

// ���ַ��� PID
void PID_init_ilimit(pid_type_def *pid, uint8_t mode, const float kp, const float ki, const float kd,
                     float max_out, float max_iout, float alpha, float I_band);
float PID_calc_ilimit(pid_type_def *pid);

#endif
