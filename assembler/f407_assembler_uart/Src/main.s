// receive via USART2 @115200baud, assuming
// power on reset clock defaults (HSI, 16MHz)

.cpu	cortex-m4
.syntax	unified
.thumb

.equ RCC_BASE,    			0x40023800

.equ AHB1ENR_OFFSET,        0x30
.equ RCC_AHB1ENR,           (RCC_BASE + AHB1ENR_OFFSET)

.equ APB1ENR_OFFSET,        0x40
.equ RCC_APB1ENR,           (RCC_BASE + APB1ENR_OFFSET)

//-------------USART2 registers ---------------------------

.equ USART2_BASE,           0x40004400

.equ USART_SR_OFFSET,		0x00
.equ USART2_SR,				(USART2_BASE + USART_SR_OFFSET)

.equ USART_DR_OFFSET, 		0x04
.equ USART2_DR,   			(USART2_BASE + USART_DR_OFFSET)

.equ USART_BRR_OFFSET, 		0x08
.equ USART2_BRR, 			(USART2_BASE + USART_BRR_OFFSET)

.equ USART_CR1_OFFSET, 		0x0C
.equ USART2_CR1, 			(USART2_BASE + USART_CR1_OFFSET)

.equ USART_CR2_OFFSET, 		0x10
.equ USART2_CR2,   			(USART2_BASE + USART_CR2_OFFSET)

.equ USART_CR3_OFFSET, 		0x14
.equ USART2_CR3,   			(USART2_BASE + USART_CR3_OFFSET)


//--------------- USART2 register settings ----------------
// enable peripheral clock to USART2 in APB1ENR
.equ USART2_EN,	(1<<17)

// BRR
// we want 115200baud @ HSI clock of 16MHz
// divisor = 16000000/(8*2*115200) = 8.68
// 8 = 0x8
// .68*16 = 10.8 ~= 11 = B
.equ BRR_CNF,  	0x008B

// CR1,2,3
.equ CR1_CNF, 	((1<<3)|(1<<2)) // enable TX and RX
.equ CR2_CNF, 	0x0000 // 1 stop bit
.equ CR3_CNF, 	0x0000 // no flow control
.equ CR1_EN,    (1<<13) // usart enable

// SR
.equ TXE_FLAG,	(1<<7) // transmit buffer empty flag
.equ RXNE_FLAG,	(1<<5) // receive buffer not empty flag


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
.equ GPIOA_EN,				(1<<0)

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


.equ CR, 	0x0D
.equ LF, 	0x0A
.equ BS, 	0x08
.equ ESC,	0x1B
.equ SPC,	0x20
.equ DEL,	0x7F

.equ ONE_SEC, 333333

.section .text
.type    	main, %function
.global		main


main:
        BL  HWInit
Loop:
		BL	USART2ReadChar
		//BL  USART2WriteChar
		BL  LEDBlink
        B	Loop



HWInit:
        // RCC->AHB1ENR |= GPIOA_EN

        // r0 = RCC_AHB1ENR register address
        LDR     R0, =RCC_AHB1ENR
        // r1 = [RCC_AHB1_ENR], i.e. register content
        LDR     R1, [R0]
        // R1 |=  GPIOA peripheral clock enable bit
        ORR     R1, #GPIOA_EN
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


        // RCC->APB1ENR |= USART2_EN
        // r0 = RCC_APB1ENR register address
        LDR     R0, =RCC_APB1ENR
        // r1 = [RCC_APB1_ENR]
        LDR     R1, [R0]
        // R1 |=  USART2 peripheral clock enable bit
        ORR     R1, #USART2_EN
        // [RCC_APB1_ENR] = r1
        STR     R1, [R0]

		// Alternate function USART2 for PA2 and PA3
		// GPIOA->AFR[0] |= GPIOA_AFRL_SET
        LDR     R0, =GPIOA_AFRL
        LDR     R1, [R0]
        ORR     R1, #GPIOA_AFRL_SET
        STR     R1, [R0]

		// Configure pin PA2, PA3 as gpio : alternate function mode
        // GPIOA->MODER |= MODE23_AF
        LDR     R0, =GPIOA_MODER
        LDR     R1, [R0]
        ORR     R1, #MODE23_AF
        STR     R1, [R0]

        // normally you want to disable uart before configuring brr etc !
        // UART2->BRR = BRR_CNF
        LDR     R0, =USART2_BRR
        // movw as value is larger than 1 byte
        MOVW    R1, #BRR_CNF
        STR     R1, [R0]

        // UART2->CR1 = CR1_CNF
        LDR     R0, =USART2_CR1
        // mov as value is 1 byte
        MOV     R1, #CR1_CNF
        STR     R1, [R0]

        // UART2->CR2 = CR2_CNF
        LDR     R0, =USART2_CR2
        // mov as value is 1 byte
        MOV     R1, #CR2_CNF
        STR     R1, [R0]

        // UART2->CR3 = CR3_CNF
        LDR     R0, =USART2_CR3
        // mov as value is 1 byte
        MOV     R1, #CR3_CNF
        STR     R1, [R0]

        // UART2->CR1 |= CR1_EN
        LDR     R0, =USART2_CR1
        LDR     R1, [R0]
        ORR     R1, #CR1_EN
        STR     R1, [R0]
        BX      LR


USART2WriteChar:
		// char to transmit is in R0
		LDR		R1, =USART2_SR
		// spin while TX buffer is not empty
oloop:
		LDR		R2, [R1]
		AND		R2, #TXE_FLAG
		CMP		R2, #0
		BEQ		oloop
		// write to DR
		LDR		R2, =USART2_DR
		STR		R0, [R2]
		BX 		LR


USART2ReadChar:
		LDR		R1, =USART2_SR
iloop:    // wait until RXNE is set
		LDR		R2, [R1]
		AND 	R2, #RXNE_FLAG
		CMP 	R2, #0
		BEQ		iloop
		LDR		R3, =USART2_DR
		//  read incoming char into R0
		LDR		R0,[R3]
		BX		LR

LEDBlink:
		MOV		R3, R0
		CMP		R3, #0x0031 // check if '1' was sent
		BEQ		pt0
		BX		LR
pt0:
        LDR     R2, =GPIOA_BSRR
        MOV     R1, #LED_ON
        STR     R1, [R2]

        //LDR		R3, =ONE_SEC
        BL		Delay

        LDR     R2, =GPIOA_BSRR
        MOV     R1, #LED_OFF
        STR     R1, [R2]

        //LDR		R3, =ONE_SEC
        //BL		Delay
	    BX  	LR

#if 0
LEDSet:
		MOV		R3, R0
		CMP		R3, #0x0031 // turn on LED if '1' was sent
		BEQ		LEDOn
		CMP		R3, #0x0032 // turn off LED if '2' was sent
		BEQ		LEDOff
		BX		LR

LEDOn:
        LDR     R2, =GPIOA_BSRR
        MOV     R1, #LED_ON
        STR     R1, [R2]
		BX 		LR

LEDOff:
        LDR     R2, =GPIOA_BSRR
        MOV     R1, #LED_OFF
        STR     R1, [R2]
      	BX		LR

Delay:
        // update flags after instruction 'S' suffix
        // r3 = r3-1
        SUBS	R3,R3,#1
        // branch if z flag is non-zero
        BNE		Delay
        // return from subroutine
        BX      LR
#endif

Delay:
    ldr     r2, =ONE_SEC          // load DELAY value
1:  subs    r2, r2, #1          // count down to 0 = wait
    bne     1b
    bx      lr

.end



