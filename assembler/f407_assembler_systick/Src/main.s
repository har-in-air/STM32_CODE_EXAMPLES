// SysTick Cortex M peripheral : 24bit down counter with autoreload

.cpu	cortex-m4
.syntax	unified
.thumb

.equ RCC_BASE,    			0x40023800

.equ AHB1ENR_OFFSET,        0x30
.equ RCC_AHB1ENR,           (RCC_BASE + AHB1ENR_OFFSET)

.equ APB1ENR_OFFSET,        0x40
.equ RCC_APB1ENR,           (RCC_BASE + APB1ENR_OFFSET)

// ----------- SysTick is a core peripheral ---------------

.equ SYST_CSR,		0xE000E010	// control and status
.equ SYST_RVR,		0xE000E014	// reload value
.equ SYST_CVR,		0xE000E018	// current value
.equ SYST_CALIB,	0xE000E01C	// calibration value

.equ CSR_COUNTFLAG, (1<<16) // 1 if timer counted down to 0 since last read
.equ CSR_CLKSRC, 	(1<<2) 	// 0 = external, 1= processor clock
.equ CSR_TICKINT, 	(1<<1) 	// 1 = assert SysTick interrupt on counter=zero
.equ CSR_ENABLE, 	(1<<0) 	// enable down counter


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


.equ DELAY_10MS, 160000

.section .text
.type    	main, %function
.global		main


main:
        BL  GPIOInit
		BL	LEDBlink


GPIOInit:
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
        BX      LR


SysTickWait:
		// R0 has the number of clock cycles to wait

		// first disable the counter
        LDR     R1, =SYST_CSR
        MOV		R2, #0
        STR		R2, [R1]

		// load R0-1 into the reload value register
        LDR     R1, =SYST_RVR
        SUB		R0,	#1
        STR		R0, [R1]

		// clear the counter
		// writing any value clears both the counter and CSR_COUNTFLAG
		LDR		R1, =SYST_CVR
		MOV		R0, #0
		STR		R0, [R1]

		// config for use of processor clock, and enable the counter
        LDR     R1, =SYST_CSR
        MOV		R0, #(CSR_CLKSRC | CSR_ENABLE)
        STR		R0, [R1]

lp1:    // spin until the COUNTFLAG is set (counter reaches 0)
        LDR		R1, =SYST_CSR
        LDR		R3, [R1] // R3  has the control and status bits
        ANDS	R3, R3, #CSR_COUNTFLAG // terminate if countflag is set
		BEQ		lp1 // if AND result is zero, loop
		BX		LR


SysTickWait10ms:
		// R0 has the count of 10mS ticks to wait
		PUSH	{R4, LR}
		MOVS	R4, R0 // R4 = R0
		BEQ		done // if zero argument, no delay
lp2:
		LDR		R0, =DELAY_10MS
		BL		SysTickWait
		SUBS	R4, #1 // decrement the count of ticks
		BHI		lp2 // branch if higher than zero
done:
		POP		{R4, LR}
		BX		LR


LEDBlink:
        LDR     R2, =GPIOA_BSRR
        MOV     R1, #LED_ON
        STR     R1, [R2]

        MOV		R0, #100 // 100*10mS = one second delay
        BL		SysTickWait10ms

        LDR     R2, =GPIOA_BSRR
        MOV     R1, #LED_OFF
        STR     R1, [R2]

        MOV		R0, #100 // 100*10mS = one second delay
        BL		SysTickWait10ms

		B		LEDBlink


.end



