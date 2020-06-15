// DevEBox STM32F407VGT6 dev board
// Demo SRAM1 to UART1 DMA transfer without using HAL libraries
// Create new project in CubeIDE with only automatic startup code generation, this
// will also create the syscalls.c and sysmem.c files in /Src folder.
// In the /Inc folder copy all the relevant CMSIS header files and stm32f407xx.h from
// an existing CubeMX HAL generated project.
// In the /Src folder, create new main.c and it.c files.

#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"
#include "it.h"

void button_init(void);
void uart1_init(void);
void dma2_init(void);
void print_uart1(char* pMsg);
void dma2_interrupt_config(void);
void dma2_enable(void);
void dma2_txCompleteCallback(void);

char DMASourceData[] = "string in SRAM1\r\n";

int main(void){
	// power-on/reset clock defaults : HSI oscillator, sysclk, hclk, pclk1, pclk2 = 16MHz
	button_init();
	uart1_init();

	// use for testing UART1 functionality
	//print_uart1("Hello world\r\n");

	dma2_init();
	dma2_interrupt_config();
	dma2_enable();

	while(1){
		}
	return 0;
	}


// use this function to test uart1 transmit functionality before
// coding and testing DMA2->UART1 transfer
void print_uart1(char* pMsg) {
	USART_TypeDef* pUART1 = USART1;
	int numChars = strlen(pMsg);
	for (int inx = 0; inx < numChars; inx++){
		// wait for TX empty
		while ( !(pUART1->SR &  USART_SR_TXE) );
		pUART1->DR = pMsg[inx];
		}
	}


// button K1 on board connects PA0 to VCC, with no external pull-down resistor
// we want to generate an interrupt on pressing the button
// In the IRQ handler we will enable the DMA2->UART1 data transfer
void button_init(void) {
	// enable clock to GPIOA peripheral
	RCC_TypeDef *pRCC = RCC;
	pRCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN_Msk;

	// Set PA0 pin mode = input (0 : input, 1: output, 2: alternate function, 3:analog)
	GPIO_TypeDef *pGPIOA = GPIOA;
	pGPIOA->MODER &= ~GPIO_MODER_MODER0_Msk;

	// enable PA0 internal pulldown ( 0 : none, 1 : pullup, 2 : pulldown, 3 : rsvd)
	pGPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;
	pGPIOA->PUPDR |= (0x2UL << GPIO_PUPDR_PUPD0_Pos);

	// enable the interrupt for pin 0 ( 0 : masked, 1 : enabled)
	EXTI_TypeDef* pEXTI = EXTI;
	pEXTI->IMR |= EXTI_IMR_MR0_Msk;

	// enable clock to SYSCFG block
	pRCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// select PA0 for EXTI0 (select from PA0,PB0,PC0...)
	SYSCFG_TypeDef* pSYSCFG = SYSCFG;
	pSYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_Msk;
	pSYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;

	// enable rising-edge interrupt on  pin 0 ( 0 : disabled, 1 : enabled )
	// make sure you do this AFTER selecting PA0 for EXTI0 to avoid spurious interrupts
	pEXTI->RTSR |= EXTI_RTSR_TR0_Msk;

	// enable IRQn in NVIC (core_cm4.h)
	NVIC_EnableIRQ(EXTI0_IRQn);
	}


void uart1_init(void){
	// enable clock to USART1 peripheral
	RCC_TypeDef *pRCC = RCC;
	pRCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	// enable clock to GPIOA peripheral
	pRCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN_Msk;

	// Set PA9 (TX) pin mode = alternate function (0 : input, 1: output, 2: alternate function, 3:analog)
	GPIO_TypeDef *pGPIOA = GPIOA;
	pGPIOA->MODER &= ~GPIO_MODER_MODER9_Msk;
	pGPIOA->MODER |= (2UL << GPIO_MODER_MODER9_Pos);
	// choose pin alternate function AF7 as per datasheet
	// PA9 is in AFRH (AFR[1])
	pGPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL9_Msk;
	pGPIOA->AFR[1] |= (7UL << GPIO_AFRH_AFSEL9_Pos);
	// PA9 speed is high
	pGPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED9_Msk;
	pGPIOA->OSPEEDR |= (2UL << GPIO_OSPEEDR_OSPEED9_Pos);
	// PA9 no pullup or pulldown ( 0 : none, 1 : pullup, 2 : pulldown, 3 : rsvd)
	pGPIOA->PUPDR &= ~GPIO_PUPDR_PUPD9_Msk;

	// Set PA10 (RX) pin mode = alternate function (0 : input, 1: output, 2: alternate function, 3:analog)
	pGPIOA->MODER &= ~GPIO_MODER_MODER10_Msk;
	pGPIOA->MODER |= (2UL << GPIO_MODER_MODER10_Pos);
	// choose pin alternate function AF7 as per datasheet
	// PA10 is in AFRH (AFR[1])
	pGPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL10_Msk;
	pGPIOA->AFR[1] |= (7UL << GPIO_AFRH_AFSEL10_Pos);
	// PA10 speed is high
	pGPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED10_Msk;
	pGPIOA->OSPEEDR |= (2UL << GPIO_OSPEEDR_OSPEED10_Pos);
	// PA10 no pullup or pulldown ( 0 : none, 1 : pullup, 2 : pulldown, 3 : rsvd)
	pGPIOA->PUPDR &= ~GPIO_PUPDR_PUPD10_Msk;

	// configure baudrate = 115200
	//  need 8.6875 at 16MHz as per ref manual
	// fractional part with 4bit resolution = 0.6875*16 = 11 = 0xB
	// so we need to program 0x8B in the fractional divisor reg
	USART_TypeDef* pUART1 = USART1;
	pUART1->BRR = 0x8B;

	// configure data width, stop bits, parity
	// power on reset values are ok : 1startbit, 8 bits, even parity, 1 stop bit

	// enable TX engine
	pUART1->CR1 |= USART_CR1_TE;
	// enable uart
	pUART1->CR1 |= USART_CR1_UE;
	}


void dma2_init(void) {
	// enable clock to DMA2 peripheral
	// DMA2 slave port for programming registers is on AHB1 bus
	RCC_TypeDef *pRCC = RCC;
	pRCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	// note : stream must be disabled before configuring

	// identify stream and channel for  DMA2 -> USART1_TX transfer
	// stream 7 channel 4 as per ref manual
	DMA_Stream_TypeDef* pStream = DMA2_Stream7;

	// sw can only configure stream and fifo when the EN bit is cleared
    pStream->CR &= ~DMA_SxCR_EN_Msk;

    // connect channel 4 to stream 7
    pStream->CR &= ~DMA_SxCR_CHSEL_Msk;
    pStream->CR |= (4UL << DMA_SxCR_CHSEL_Pos);

	// set source address in SRAM1
	pStream->M0AR = (uint32_t) DMASourceData;

	// set destination address = UART1 DR
	USART_TypeDef* pUART1 = USART1;
	pStream->PAR = (uint32_t) &(pUART1->DR);

	// set number of data items to send
	uint32_t len = sizeof(DMASourceData);
	pStream->NDTR = len;

	// set direction of transfer = M2P (0 : P2M, 1 : M2P, 2 : M2M, 3 : rsvd)
	pStream->CR &= ~DMA_SxCR_DIR_Msk;
	pStream->CR |= (1UL << DMA_SxCR_DIR_Pos);

	// set memory data size  = byte (0 : byte, 1 : halfword, 2 : word)
	pStream->CR &= ~DMA_SxCR_MSIZE_Msk;

	// set peripheral data size  = byte (0 : byte, 1 : halfword, 2 : word)
	pStream->CR &= ~DMA_SxCR_PSIZE_Msk;

	// set memory increment mode = increment ( 0 : fixed, 1 : increment)
	pStream->CR &= ~DMA_SxCR_MINC_Msk;
	pStream->CR |= (1UL << DMA_SxCR_MINC_Pos);

	// we are writing to a fixed register UART1->DR
	// set peripheral increment mode = fixed ( 0 : fixed, 1 : increment)
	pStream->CR &= ~DMA_SxCR_PINC_Msk;

	// set mode = fifo (0 : direct mode, 1 : fifo )
	pStream->FCR &= ~DMA_SxFCR_DMDIS_Msk;
	pStream->FCR |= (1UL << DMA_SxFCR_DMDIS_Pos);

	// set fifo threshold = full (0: 1/4, 1: 1/2, 2: 3/4, 3: Full)
	pStream->FCR &= ~DMA_SxFCR_FTH_Msk;
	pStream->FCR |= (3UL << DMA_SxFCR_FTH_Pos);

	// set normal mode : (0 : normal mode, 1 : circular mode)
    pStream->CR &= ~DMA_SxCR_CIRC_Msk;

	// set memory burst transfer = single (0 : single, 1: burst4, 2:burst8, 3:burst16)
    pStream->CR &= ~DMA_SxCR_MBURST_Msk;

    // set peripheral burst transfer = single (0 : single, 1: burst4, 2:burst8, 3:burst16)
    pStream->CR &= ~DMA_SxCR_PBURST_Msk;

	// set stream priority = low (0: low, 1:medium, 2:high, 3:very high)
    pStream->CR &= ~DMA_SxCR_PL_Msk;

	}


void dma2_enable(void) {
	DMA_Stream_TypeDef* pStream = DMA2_Stream7;
	// enable stream, this bit will be cleared by HW on end of transfer, AHB xfer error etc.
    pStream->CR |= DMA_SxCR_EN;
	}


void dma2_txCompleteCallback(void) {
	//DMA_Stream_TypeDef* pStream = DMA2_Stream7;
	// sw can only configure stream and fifo when the EN bit is cleared
    //pStream->CR &= ~DMA_SxCR_EN_Msk;
	// reset the data transfer length
	//pStream->NDTR = sizeof(DMASourceData);
	// disable the peripheral DMA request
	USART_TypeDef* pUART1 = USART1;
	pUART1->CR3 &= ~USART_CR3_DMAT;
	// enable the DMA transfer
	dma2_enable();
	// ready for another button triggered UART1 DMA transfer request
	}


void dma2_interrupt_config(void){
	DMA_Stream_TypeDef* pStream = DMA2_Stream7;
	// half-transfer complete HTIE
	pStream->CR |= DMA_SxCR_HTIE;
	// transfer complete TCIE
	pStream->CR |= DMA_SxCR_TCIE;
	// transfer error TEIE
	pStream->CR |= DMA_SxCR_TEIE;
	// direct mode error DMEIE
	pStream->CR |= DMA_SxCR_DMEIE;
	// fifo overrun/underrun FEIE
	pStream->FCR |= DMA_SxFCR_FEIE;

	// enable irq for DMA2 stream 7 in NVIC (core-cm4.h)
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);
	}

