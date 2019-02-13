/*
 * can.c
 *
 *  Created on: Feb 12, 2019
 *      Author: Matt Flanagan
 */
#include "can.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CanRxMsgTypeDef rx;
	CAN_RxHeaderTypeDef header;
	HAL_CAN_GetRxMessage(hcan, 0, &header, rx.Data);
	rx.DLC = header.DLC;
	rx.StdId = header.StdId;
	xQueueSendFromISR(q_rx_dcan, &rx, NULL);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CanRxMsgTypeDef rx;
	CAN_RxHeaderTypeDef header;
	HAL_CAN_GetRxMessage(hcan, 1, &header, rx.Data);
	rx.DLC = header.DLC;
	rx.StdId = header.StdId;
	xQueueSendFromISR(q_rx_dcan, &rx, NULL);
}

void DCANFilterConfig()
{
	  CAN_FilterTypeDef FilterConf;
	  FilterConf.FilterIdHigh =         0x400 << 5; // 2 num
	  FilterConf.FilterIdLow =          0x401 << 5; // 0
	  FilterConf.FilterMaskIdHigh =     0x7ff;       // 3
	  FilterConf.FilterMaskIdLow =      0x7fe;       // 1
	  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
	  FilterConf.FilterBank = 0;
	  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
	  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
	  FilterConf.FilterActivation = ENABLE;
	  HAL_CAN_ConfigFilter(&hcan1, &FilterConf);
}

void VCANFilterConfig()
{


	  CAN_FilterTypeDef FilterConf;
	  FilterConf.FilterIdHigh =         0x501 << 5; // 2 num
	  FilterConf.FilterIdLow =          0x201 << 5; // 0
	  FilterConf.FilterMaskIdHigh =     0x7ff;       // 3
	  FilterConf.FilterMaskIdLow =      0x7ff;       // 1
	  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO1;
	  FilterConf.FilterBank = 1;
	  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
	  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
	  FilterConf.FilterActivation = ENABLE;
	  HAL_CAN_ConfigFilter(&hcan2, &FilterConf);
}

void taskTX_DCAN()
{
	CanTxMsgTypeDef tx;

	for (;;)
	{
		//check if this task is triggered
		if (xQueuePeek(q_tx_dcan, &tx, portMAX_DELAY) == pdTRUE)
		{
			xQueueReceive(q_tx_dcan, &tx, portMAX_DELAY);  //actually take item out of queue
			CAN_TxHeaderTypeDef header;
			header.DLC = tx.DLC;
			header.IDE = tx.IDE;
			header.RTR = tx.RTR;
			header.StdId = tx.StdId;
			header.TransmitGlobalTime = DISABLE;
			uint32_t mailbox;
			while (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)); // while mailboxes not free
			HAL_CAN_AddTxMessage(&hcan1, &header, tx.Data, &mailbox);
		}
	}
}

void taskTX_VCAN()
{
	CanTxMsgTypeDef tx;

	for (;;)
	{
		//check if this task is triggered
		if (xQueuePeek(q_tx_vcan, &tx, portMAX_DELAY) == pdTRUE)
		{
			xQueueReceive(q_tx_vcan, &tx, portMAX_DELAY);  //actually take item out of queue
			CAN_TxHeaderTypeDef header;
			header.DLC = tx.DLC;
			header.IDE = tx.IDE;
			header.RTR = tx.RTR;
			header.StdId = tx.StdId;
			header.TransmitGlobalTime = DISABLE;
			uint32_t mailbox;
			while (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)); // while mailboxes not free
			HAL_CAN_AddTxMessage(&hcan2, &header, tx.Data, &mailbox);
		}
	}
}

void taskRXCANProcess()
{

	CanRxMsgTypeDef rx;  //CanRxMsgTypeDef to be received on the queue
	while (1)
	{
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		//if there is a CanRxMsgTypeDef in the queue, pop it, and store in rx
		if (xQueueReceive(q_rx_dcan, &rx, portMAX_DELAY) == pdTRUE)
		{
			//A CAN message has been recieved
			//check what kind of message we received
			switch (rx.StdId)
			{
			}
		}
	}
}
