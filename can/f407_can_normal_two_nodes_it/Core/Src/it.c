#include "main.h"
#include "it.h"

void SysTick_Handler(void){
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}



// irq handler name from startup file
extern TIM_HandleTypeDef htim6;

void TIM6_DAC_IRQHandler(void){
	// check source of interrupt, acknowledge and call the specific callback
	// in our case HAL_TIM_PeriodElapsedCallback must be implemented
	HAL_TIM_IRQHandler(&htim6);
	}


void EXTI0_IRQHandler(void){
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	}


/*
 *  4 IRQs per CAN controller (8 total).
 * 	we need to implement the specific callbacks of interest in our
 * 	application, by overriding the weak functions specified in
 *	HAL_CAN_IRQHandler

// tx  callbacks
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan);

// rxfifo0 callbacks
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan);

// rxfifo1 callbacks
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan);

// status change and error callbacks
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
*/

extern CAN_HandleTypeDef hcan1;

void  CAN1_TX_IRQHandler(void){
	HAL_CAN_IRQHandler(&hcan1);
	}


void  CAN1_RX0_IRQHandler(void){
	HAL_CAN_IRQHandler(&hcan1);
	}


void CAN1_RX1_IRQHandler(void){
	HAL_CAN_IRQHandler(&hcan1);
	}


void CAN1_SCE_IRQHandler(void){
	HAL_CAN_IRQHandler(&hcan1);
	}
