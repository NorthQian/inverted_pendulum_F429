#include "bsp_can.h"
/**
************************************************************************
* @brief:      	can_bsp_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN ʹ��
************************************************************************
**/
void bsp_can_init(void)
{ 
	can_filter_init();
	HAL_CAN_Start(&hcan1);   

	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);	
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN�˲�����ʼ��
************************************************************************
**/
void can_filter_init(void)
{
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
/**
************************************************************************
* @brief:      	canx_bsp_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hcan: CAN���
* @param:       id: 	CAN�豸ID
* @param:       data: ���͵�����
* @param:       len:  ���͵����ݳ���
* @retval:     	void
* @details:    	��������
************************************************************************
**/
uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{	
	CAN_TxHeaderTypeDef	tx_header;
	uint32_t tx_mailbox;
	
	tx_header.StdId = id;
	tx_header.ExtId = 0;
	tx_header.IDE   = 0;
	tx_header.RTR   = 0;
	tx_header.DLC   = len;
  /*�ҵ��յķ������䣬�����ݷ��ͳ�ȥ*/
	if(HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox) != HAL_OK) {
		if(HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox) != HAL_OK) {
			HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
		}
	}
	return 0;
}
/**
************************************************************************
* @brief:      	canx_bsp_receive(CAN_HandleTypeDef *hcan, uint8_t *buf)
* @param:       hcan: CAN���
* @param[out]:  rec_id: 	���յ����ݵ�CAN�豸ID
* @param:       buf���������ݻ���
* @retval:     	���յ����ݳ���
* @details:    	��������
************************************************************************
**/
uint8_t canx_receive(hcan_t *hcan, uint16_t *rec_id, uint8_t *buf)
{	
	CAN_RxHeaderTypeDef rx_header;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, buf) == HAL_OK)
	{
		*rec_id = rx_header.StdId;
		return rx_header.DLC; //�������ݳ���
	}
	else
		return 0;
}
/**
************************************************************************
* @brief:      	HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
* @param:       hfdcan��FDCAN���
* @param:       RxFifo0ITs���жϱ�־λ
* @retval:     	void
* @details:    	HAL���CAN�жϻص�����
************************************************************************
**/
void HAL_CAN_RxFifo0MsgPendingCallback(hcan_t *hcan)
{
	if(hcan == &hcan1) {
		can1_rx_callback();
	}
}
/**
************************************************************************
* @brief:      	can1_rx_callback(void)
* @param:       void
* @retval:     	void
* @details:    	���û����õĽ���������
************************************************************************
**/
__weak void can1_rx_callback(void)
{

}
/**
************************************************************************
* @brief:      	can2_rx_callback(void)
* @param:       void
* @retval:     	void
* @details:    	���û����õĽ���������
************************************************************************
**/
__weak void can2_rx_callback(void)
{

}


