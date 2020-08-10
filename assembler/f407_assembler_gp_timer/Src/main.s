// SysTick Cortex M peripheral : general purpose timer

.cpu	cortex-m4
.syntax	unified
.thumb

.equ RCC_BASE,    			0x40023800

.equ AHB1ENR_OFFSET,        0x30
.equ RCC_AHB1ENR,           (RCC_BASE + AHB1ENR_OFFSET)

.equ APB1ENR_OFFSET,        0x40
.equ RCC_APB1ENR,           (RCC_BASE + APB1ENR_OFFSET)

// ----------- TIM2 on APB1 ---------------


.equ APB1_TIM2_EN,   	(1<<0)

.equ TIM2_BASE, 			0x40000000

.equ TIM2_CR1_OFFSET,		0x00
.equ TIM2_CR1, 				(TIM2_BASE + TIM2_CR1_OFFSET)

.equ TIM2_SR_OFFSET,		0x10
.equ TIM2_SR, 				(TIM2_BASE + TIM2_SR_OFFSET)

.equ TIM2_CNT_OFFSET,		0x24
.equ TIM2_CNT, 				(TIM2_BASE + TIM2_CNT_OFFSET)

.equ TIM2_PSC_OFFSET,		0x28
.equ TIM2_PSC,				(TIM2_BASE + TIM2_PSC_OFFSET)

.equ TIM2_ARR_OFFSET,		0x2C
.equ TIM2_ARR, 				(TIM2_BASE + TIM2_ARR_OFFSET)


.equ TIM2_SR_UIF, 			(1<<0)

// timer update freq = (CLK_IN / PSC) / ARR, here CLK_IN = HSI = 16MHz
.equ TIM2_PSC_CNF,			(1600-1) 	// counter clock = 10000Hz
.equ TIM2_ARR_CNF,			(10000-1)	// one second

.equ TIM2_CNT_CNF,			0 // clear counter
.equ TIM2_CR1_CNF,			(1<<0) // enable counter

//------------------- GPIOA registers ---------------------
.equ GPIOA_BASE,            0x40020000

.equ GPIOA_MODER_OFFSET,    0x00
.equ GPIOA_MODER,           (GPIOA_BASE + GPIOA_MODER_OFFSET)

.equ GPIOA_AFRL_OFFSET, 	0x20
.equ GPIOA_AFRL,           	(GPIOA_BASE + GPIOA_AFRL_OFFSET)

.equ GPIOA_BSRR_OFFSET,     0x18
.equ GPIOA_BSRR,            (GPIOA_BASE + GPIOA_BSRR_OFFSET)


//------------------ GPIOA settings ----------------------------
// enable peripheral clock to GPIOA in AHB1ENR
.equ AHB1_GPIOA_EN,				(1<<0)

// configure PA2 (TX) and PA3 (RX) in USART2 mode
// 1. Mode setting (00 input, 01 output, 10 alternate function, 11 analog)
// PA2: MODER2 (5,4) = 10
// PA3: MODER3 (7,6) = 10
.equ MODE23_AF,		((1<<5)|(1<<7))

// PA2 and PA3 set for USART2 function
.equ GPIOA_AFRL_SET, 		((0x7<<8)|(0x7<<12))

// on board LED connected to PA1
// MODER : 2bit field needs to be 00 for gpio input, 01 for gpio output, 10 for alternate, 11 for analog
.equ MODER1_OUT,               (1<<2)

// LED on PA1 is active low
// BSRR : write 1 to upper 16bits to reset, write 1 to lower 16bits to set
.equ BSRR_1_RESET,            (1<<17)
.equ BSRR_1_SET,              (1<<1)

.equ LED_OFF,                  BSRR_1_SET
.equ LED_ON,                   BSRR_1_RESET


.equ DELAY_10MS, 160000

.section .text
.type    	main, %function
.global		main


main:
        BL  GPIOInit
        BL	TIM2Init
		BL	LEDBlink


GPIOInit:
        // RCC->AHB1ENR |= GPIOA_EN

        // r0 = RCC_AHB1ENR register address
        LDR     R0, =RCC_AHB1ENR
        // r1 = [RCC_AHB1_ENR], i.e. register content
        LDR     R1, [R0]
        // R1 |=  GPIOA peripheral clock enable bit
        ORR     R1, #AHB1_GPIOA_EN
        // [RCC_AHB1_ENR] = r1
        STR     R1, [R0]
		// Configure pin PA1 connected to onboard LED
        // GPIOA->MODER |= MODER1_OUT
        LDR     R0, =GPIOA_MODER
        LDR     R1, [R0]
        ORR     R1, #MODER1_OUT
        STR     R1, [R0]
		// LED off
        LDR     R2, =GPIOA_BSRR
        MOV     R1, #LED_OFF
        STR     R1, [R2]
        BX      LR


TIM2Init:
		// enable clock to TIM2
        LDR     R0, =RCC_APB1ENR
        LDR     R1, [R0]
        ORR     R1, #APB1_TIM2_EN
        STR		R1, [R0]

		// prescaler configuration
        LDR     R0, =TIM2_PSC
        // 16bit move
        MOVW	R1, #TIM2_PSC_CNF
        STR		R1, [R0]

		// autoreload  configuration
        LDR     R0, =TIM2_ARR
        // 16bit move
        MOVW	R1, #TIM2_ARR_CNF
        STR		R1, [R0]

		// clear the counter
		LDR		R0, =TIM2_CNT
		MOV		R1, #TIM2_CNT_CNF
		STR		R1, [R0]

		// enable the counter
		LDR		R0, =TIM2_CR1
		MOV		R1, #TIM2_CR1_CNF
		STR		R1, [R0]

		BX		LR

__wait:
		LDR		R1, =TIM2_SR
		// while UIF flag is not set, loop
lp1:	LDR		R2, [R1]
		AND		R2, #TIM2_SR_UIF
		CMP		R2, #0
		BEQ		lp1
		// UIF is set, clear UIF (unlike systick, where update flag is cleared on read)
		LDR		R3, [R1]
		BIC		R3, R3, #1 // clear UIF (bit 0)
		STR		R3, [R1]
		BX		LR

LEDBlink:
        LDR     R4, =GPIOA_BSRR
        MOV     R1, #LED_ON
        STR     R1, [R4]
       	BL		__wait

       	MOV		R1, #LED_OFF
       	STR		R1, [R4]
       	BL		__wait

		B		LEDBlink

.end



