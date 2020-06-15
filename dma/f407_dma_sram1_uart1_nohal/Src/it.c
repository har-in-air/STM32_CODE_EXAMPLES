/*
 * it.c
 *
 *  Created on: Jun 13, 2020
 *      Author: hari
 */
#include "stm32f407xx.h"


// look for names of irq handlers in startup .s file


// PA0 button press ISR
void EXTI0_IRQHandler(void) {
	// clear the pending bit in the PR register by writing 1
	EXTI_TypeDef* pEXTI = EXTI;
	if (pEXTI->PR & EXTI_PR_PR0_Msk) {
		pEXTI->PR |= EXTI_PR_PR0;
		}
	// send UART1 DMA transmit request to DMA2 controller
	USART_TypeDef* pUART1 = USART1;
	pUART1->CR3 |= USART_CR3_DMAT;
	}


extern void dma2_txCompleteCallback();

void DMA2_Stream7_IRQHandler(void){
	DMA_TypeDef* pDMA = DMA2;
	// DMA stream 7 status flags are in the high status register HISR
	// clear the bits by writing a 1  to the flag clear register HIFCR
	// half transfer complete
	if (pDMA->HISR & DMA_HISR_HTIF7){
		pDMA->HIFCR |= DMA_HIFCR_CHTIF7;
		}
	else
	// full transfer complete
	if (pDMA->HISR & DMA_HISR_TCIF7){
		pDMA->HIFCR |= DMA_HIFCR_CTCIF7;
		// reset the stream for another transfer
		dma2_txCompleteCallback();
		}
	else
	// transfer error
	if (pDMA->HISR & DMA_HISR_TEIF7){
		pDMA->HIFCR |= DMA_HIFCR_CTEIF7;
		}
	else
	// direct mode error
	if (pDMA->HISR & DMA_HISR_DMEIF7){
		pDMA->HIFCR |= DMA_HIFCR_CDMEIF7;
		}
	else
	// fifo overrun/underrun error
	if (pDMA->HISR & DMA_HISR_FEIF7){
		pDMA->HIFCR |= DMA_HIFCR_CFEIF7;
		}
	}
