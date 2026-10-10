#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"
#include "string.h"
#include "stdbool.h"

motor_t motor[num];


/* ===================== Motor1 位置回绕展开（多圈） =====================
 * para.pos 由主机 tmp.PMAX 解码（已改为 π），名义范围 (-π, +π)，跨边界回绕。
 * 把回绕值展开成连续（多圈）坐标：半周期 = PMAX，周期 = 2*PMAX。
 * 半周期直接从 tmp.PMAX 派生 —— 只有一个真值来源，改 PMAX 时展开自动跟随。
 * 若单帧位移恒小于半周期（不回绕），本函数退化为恒等直通，不破坏现状。
 */
#define MOTOR_POS_DELTA_MAX 1.0f   // 单帧可信位移上限（rad）：
                                   // VMAX/反馈率 = 50/1000 = 0.05，取 1.0 留足余量

static float   motor1_pos_cont   = 0.0f;   // 展开后的连续位置（rad，可多圈）
static float   motor1_pos_prev   = 0.0f;   // 上一帧的原始回绕值（rad）
static uint8_t motor1_pos_inited = 0;      // 首帧标志

/**
 * @brief 把一帧原始回绕位置展开到连续坐标（在 CAN 反馈回调中调用，一帧一次）
 * @param raw 本帧 motor[Motor1].para.pos（回绕值，rad）
 * @note  单帧位移超过半周期判为过零并补偿；补偿后仍过大判为丢帧/干扰，
 *        只重新锚定、不累加，避免一帧坏数据把连续坐标打飞。
 */
void dm_motor_pos_unwrap(float raw)
{
	const float half   = motor[Motor1].tmp.PMAX;   // 回绕半周期
	const float period = 2.0f * half;

	if (!motor1_pos_inited)
	{
		motor1_pos_prev = raw;
		motor1_pos_cont = raw;   // 首帧直接对齐，避免上电首拍跳变
		motor1_pos_inited = 1;
		return;
	}

	float delta = raw - motor1_pos_prev;

	/* 过零补偿（正/反向各一次判断） */
	if (delta >  half) delta -= period;
	else if (delta < -half) delta += period;

	/* 丢帧/干扰守卫：只重锚不累加 */
	if (delta > MOTOR_POS_DELTA_MAX || delta < -MOTOR_POS_DELTA_MAX)
	{
		motor1_pos_prev = raw;
		return;
	}

	motor1_pos_cont += delta;
	motor1_pos_prev = raw;
}

/** @brief 读取 Motor1 连续坐标（rad，可能为多圈的大数） */
float dm_motor_pos_cont(void)
{
	return motor1_pos_cont;
}

/** @brief 首帧是否已到（供 motor_pos_zero 判断能否取零） */
uint8_t dm_motor_pos_ready(void)
{
	return motor1_pos_inited;
}


/**
************************************************************************
* @brief:      	dm4310_motor_init: DM4310�����ʼ������
* @param:      	void
* @retval:     	void
* @details:    	��ʼ��1��DM4310�ͺŵĵ��������Ĭ�ϲ����Ϳ���ģʽ��
*               ����ID������ģʽ������ģʽ����Ϣ��
************************************************************************
**/
void dm_motor_init(void)
{
	// ��ʼ��Motor1��Motor2�ĵ���ṹ
	memset(&motor[Motor1], 0, sizeof(motor[Motor1]));
	memset(&motor[Motor2], 0, sizeof(motor[Motor2]));
	memset(&motor[Motor3], 0, sizeof(motor[Motor3]));
	memset(&motor[Motor4], 0, sizeof(motor[Motor4]));
	memset(&motor[Motor5], 0, sizeof(motor[Motor5]));
	memset(&motor[Motor6], 0, sizeof(motor[Motor6]));

	// ����Motor1�ĵ����Ϣ
	motor[Motor1].id = 0x01;
	motor[Motor1].mst_id = 0x11;
	motor[Motor1].tmp.read_flag = 1;
	motor[Motor1].ctrl.mode 	= mit_mode;
	motor[Motor1].ctrl.vel_set 	= 1.0f;
	motor[Motor1].ctrl.pos_set 	= 10.0f;
	motor[Motor1].ctrl.cur_set 	= 0.03f;
	motor[Motor1].ctrl.kd_set 	= 1.0f;
	motor[Motor1].tmp.PMAX		= 3.1415926f;  // 原 12.5f；使 para.pos 范围为 (-π, +π)
	motor[Motor1].tmp.VMAX		= 50.0f;
	motor[Motor1].tmp.TMAX		= 10.0f;

	// Motor2 config
	motor[Motor2].id = 0x02;
	motor[Motor2].mst_id = 0x12;
	motor[Motor2].tmp.read_flag = 1;
	motor[Motor2].ctrl.mode 	= mit_mode;
	motor[Motor2].ctrl.vel_set 	= 1.0f;
	motor[Motor2].ctrl.pos_set 	= 10.0f;
	motor[Motor2].ctrl.cur_set 	= 0.03f;
	motor[Motor2].ctrl.kd_set 	= 1.0f;
	motor[Motor2].tmp.PMAX		= 12.5f;
	motor[Motor2].tmp.VMAX		= 50.0f;
	motor[Motor2].tmp.TMAX		= 10.0f;

	// Motor3 config
	motor[Motor3].id = 0x03;
	motor[Motor3].mst_id = 0x13;
	motor[Motor3].tmp.read_flag = 1;
	motor[Motor3].ctrl.mode 	= mit_mode;
	motor[Motor3].ctrl.vel_set 	= 1.0f;
	motor[Motor3].ctrl.pos_set 	= 10.0f;
	motor[Motor3].ctrl.cur_set 	= 0.03f;
	motor[Motor3].ctrl.kd_set 	= 1.0f;
	motor[Motor3].tmp.PMAX		= 12.5f;
	motor[Motor3].tmp.VMAX		= 50.0f;
	motor[Motor3].tmp.TMAX		= 10.0f;
}
/**
************************************************************************
* @brief:      	read_all_motor_data: ��ȡ��������мĴ�����������Ϣ
* @param:      	motor_t����������ṹ��
* @retval:     	void
* @details:    	��η��Ͷ�ȡ����
************************************************************************
**/
void read_all_motor_data(motor_t *motor)
{
    switch (motor->tmp.read_flag)
    {
		case 1:	 read_motor_data(motor->id, 0);  break; // UV_Value
        case 2:	 read_motor_data(motor->id, 1);  break; // KT_Value
		case 3:  read_motor_data(motor->id, 2);  break; // OT_Value
        case 4:  read_motor_data(motor->id, 3);  break; // OC_Value
		case 5:	 read_motor_data(motor->id, 4);  break; // ACC
        case 6:	 read_motor_data(motor->id, 5);  break; // DEC
		case 7:  read_motor_data(motor->id, 6);  break; // MAX_SPD
        case 8:  read_motor_data(motor->id, 7);  break;// MSC_ID 
		case 9:  read_motor_data(motor->id, 8);  break;// ESC_ID
        case 10: read_motor_data(motor->id, 9);  break;// TIMEOUT 
		case 11: read_motor_data(motor->id, 10); break;// CTRL_MODE 
        case 12: read_motor_data(motor->id, 11); break;// Damp 
		case 13: read_motor_data(motor->id, 12); break;// Inertia
        case 14: read_motor_data(motor->id, 13); break;// Rsv1 
		case 15: read_motor_data(motor->id, 14); break;// sw_ver 
        case 16: read_motor_data(motor->id, 15); break;// Rsv2 
		case 17: read_motor_data(motor->id, 16); break;// NPP 
        case 18: read_motor_data(motor->id, 17); break;// Rs 
		case 19: read_motor_data(motor->id, 18); break;// Ls 
        case 20: read_motor_data(motor->id, 19); break;// Flux 
		case 21: read_motor_data(motor->id, 20); break;// Gr 
        case 22: read_motor_data(motor->id, 21); break;// PMAX 
		case 23: read_motor_data(motor->id, 22); break;// VMAX 
        case 24: read_motor_data(motor->id, 23); break;// TMAX 
		case 25: read_motor_data(motor->id, 24); break;// I_BW 
        case 26: read_motor_data(motor->id, 25); break;// KP_ASR 
		case 27: read_motor_data(motor->id, 26); break;// KI_ASR 
        case 28: read_motor_data(motor->id, 27); break;// KP_APR 
		case 29: read_motor_data(motor->id, 28); break;// KI_APR 
		case 30: read_motor_data(motor->id, 29); break;// OV_Value 
        case 31: read_motor_data(motor->id, 30); break;// GREF 
		case 32: read_motor_data(motor->id, 31); break;// Deta 
        case 33: read_motor_data(motor->id, 32); break;// V_BW 
		case 34: read_motor_data(motor->id, 33); break;// IQ_c1 
        case 35: read_motor_data(motor->id, 34); break;// VL_c1 
		case 36: read_motor_data(motor->id, 35); break;// can_br 
        case 37: read_motor_data(motor->id, 36); break;// sub_ver 
		case 38: read_motor_data(motor->id, 50); break;// u_off 
        case 39: read_motor_data(motor->id, 51); break;// v_off 
		case 40: read_motor_data(motor->id, 52); break;// k1 
        case 41: read_motor_data(motor->id, 53); break;// k2 
		case 42: read_motor_data(motor->id, 54); break;// m_off 
		case 43: read_motor_data(motor->id, 55); break;// dir 
		case 44: read_motor_data(motor->id, 80); break;// pm 
		case 45: read_motor_data(motor->id, 81); break;// xout 
    }
}
/**
************************************************************************
* @brief:      	receive_motor_data: ���յ�����ص�������Ϣ
* @param:      	motor_t����������ṹ��
* @param:      	data�����յ�����
* @retval:     	void
* @details:    	��ν��յ���ش��Ĳ�����Ϣ
************************************************************************
**/
void receive_motor_data(motor_t *motor, uint8_t *data)
{
	float_type_u y;
	
	if(data[2] == 0x33)
	{
		y.b_val[0] = data[4];
		y.b_val[1] = data[5];
		y.b_val[2] = data[6];
		y.b_val[3] = data[7];
		
		switch(data[3])
		{
			case  0: motor->tmp.UV_Value = y.f_val; motor->tmp.read_flag =  2; break;
			case  1: motor->tmp.KT_Value = y.f_val; motor->tmp.read_flag =  3; break;
			case  2: motor->tmp.OT_Value = y.f_val; motor->tmp.read_flag =  4; break;
			case  3: motor->tmp.OC_Value = y.f_val; motor->tmp.read_flag =  5; break;
			case  4: motor->tmp.ACC 	 = y.f_val; motor->tmp.read_flag =  6; break;
			case  5: motor->tmp.DEC 	 = y.f_val; motor->tmp.read_flag =  7; break;
			case  6: motor->tmp.MAX_SPD  = y.f_val; motor->tmp.read_flag =  8; break;
			case  7: motor->tmp.MST_ID   = y.u_val; motor->tmp.read_flag =  9; break;
			case  8: motor->tmp.ESC_ID   = y.u_val; motor->tmp.read_flag = 10; break;
			case  9: motor->tmp.TIMEOUT  = y.u_val; motor->tmp.read_flag = 11; break;
			case 10: motor->tmp.cmode    = y.u_val; motor->tmp.read_flag = 12; break;
			case 11: motor->tmp.Damp 	 = y.f_val; motor->tmp.read_flag = 13; break;
			case 12: motor->tmp.Inertia  = y.f_val; motor->tmp.read_flag = 14; break;
			case 13: motor->tmp.hw_ver   = y.u_val; motor->tmp.read_flag = 15; break;
			case 14: motor->tmp.sw_ver   = y.u_val; motor->tmp.read_flag = 16; break;
			case 15: motor->tmp.SN 	  	 = y.u_val; motor->tmp.read_flag = 17; break;
			case 16: motor->tmp.NPP 	 = y.u_val; motor->tmp.read_flag = 18; break;
			case 17: motor->tmp.Rs 	  	 = y.f_val; motor->tmp.read_flag = 19; break;
			case 18: motor->tmp.Ls 	  	 = y.f_val; motor->tmp.read_flag = 20; break;
			case 19: motor->tmp.Flux 	 = y.f_val; motor->tmp.read_flag = 21; break;
			case 20: motor->tmp.Gr 	  	 = y.f_val; motor->tmp.read_flag = 22; break;
			case 21: motor->tmp.PMAX 	 = y.f_val; motor->tmp.read_flag = 23; break;
			case 22: motor->tmp.VMAX 	 = y.f_val; motor->tmp.read_flag = 24; break;
			case 23: motor->tmp.TMAX 	 = y.f_val; motor->tmp.read_flag = 25; break;
			case 24: motor->tmp.I_BW 	 = y.f_val; motor->tmp.read_flag = 26; break;
			case 25: motor->tmp.KP_ASR   = y.f_val; motor->tmp.read_flag = 27; break;
			case 26: motor->tmp.KI_ASR   = y.f_val; motor->tmp.read_flag = 28; break;
			case 27: motor->tmp.KP_APR   = y.f_val; motor->tmp.read_flag = 29; break;
			case 28: motor->tmp.KI_APR   = y.f_val; motor->tmp.read_flag = 30; break;
			case 29: motor->tmp.OV_Value = y.f_val; motor->tmp.read_flag = 31; break;
			case 30: motor->tmp.GREF 	 = y.f_val; motor->tmp.read_flag = 32; break;
			case 31: motor->tmp.Deta 	 = y.f_val; motor->tmp.read_flag = 33; break;
			case 32: motor->tmp.V_BW 	 = y.f_val; motor->tmp.read_flag = 34; break;
			case 33: motor->tmp.IQ_cl 	 = y.f_val; motor->tmp.read_flag = 35; break;
			case 34: motor->tmp.VL_cl 	 = y.f_val; motor->tmp.read_flag = 36; break;
			case 35: motor->tmp.can_br   = y.u_val; motor->tmp.read_flag = 37; break;
			case 36: motor->tmp.sub_ver  = y.u_val; motor->tmp.read_flag = 38; break;
			case 50: motor->tmp.u_off 	 = y.f_val; motor->tmp.read_flag = 39; break;
			case 51: motor->tmp.v_off 	 = y.f_val; motor->tmp.read_flag = 40; break;
			case 52: motor->tmp.k1 		 = y.f_val; motor->tmp.read_flag = 41; break;
			case 53: motor->tmp.k2		 = y.f_val; motor->tmp.read_flag = 42; break;
			case 54: motor->tmp.m_off 	 = y.f_val; motor->tmp.read_flag = 43; break;
			case 55: motor->tmp.dir 	 = y.f_val; motor->tmp.read_flag = 44; break;
			case 80: motor->tmp.p_m 	 = y.f_val; motor->tmp.read_flag = 45; break;
			case 81: motor->tmp.x_out 	 = y.f_val; motor->tmp.read_flag = 0 ; break;
		}
	}
}

/**
************************************************************************
* @brief:      	fdcan1_rx_callback: CAN1���ջص�����
* @param:      	void
* @retval:     	void
* @details:    	����CAN1�����жϻص������ݽ��յ���ID�����ݣ�ִ����Ӧ�Ĵ�����
*               �����յ�IDΪ0ʱ������dm4310_fbdata��������Motor�ķ������ݡ�
************************************************************************
**/
void can1_rx_callback(void)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	canx_receive(&hcan1, &rec_id, rx_data);
	switch (rec_id)
	{
 		case 0x11: dm_motor_fbdata(&motor[Motor1], rx_data);
 	           dm_motor_pos_unwrap(motor[Motor1].para.pos);   // 紧跟解析之后展开
 	           break;
			case 0x12: dm_motor_fbdata(&motor[Motor2], rx_data); break;
			case 0x13: dm_motor_fbdata(&motor[Motor3], rx_data); break;
			case 0x7FF:	/* DM register-reply frame: data[0] carries the motor address */
				if (rx_data[0] == motor[Motor1].id)      receive_motor_data(&motor[Motor1], rx_data);
				else if (rx_data[0] == motor[Motor2].id) receive_motor_data(&motor[Motor2], rx_data);
				else if (rx_data[0] == motor[Motor3].id) receive_motor_data(&motor[Motor3], rx_data);
				break;
	}
}


