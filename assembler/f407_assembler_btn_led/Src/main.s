
// monitor button on PA0 connected to VCC
// when it is pressed, turn on LED connected to PA1
// else turn off LED

.cpu	cortex-m4
.syntax	unified

.equ RCC_BASE,    			0x40023800
.equ AHB1ENR_OFFSET,           0x30
.equ RCC_AHB1ENR,              (RCC_BASE + AHB1ENR_OFFSET)

// enable peripheral clock to GPIOA
.equ GPIOA_EN,					(1<<0)

.equ GPIOA_BASE,               0x40020000
.equ GPIOA_MODER_OFFSET,       0x00
.equ GPIOA_MODER,              (GPIOA_BASE + GPIOA_MODER_OFFSET)

.equ GPIOA_IDR_OFFSET,			0x10
.equ GPIOA_IDR,				    (GPIOA_BASE + GPIOA_IDR_OFFSET)

.equ GPIOA_PUPDR_OFFSET,		0x0C
.equ GPIOA_PUPDR,             (GPIOA_BASE + GPIOA_PUPDR_OFFSET)

.equ GPIOA_BSRR_OFFSET,       0x18
.equ GPIOA_BSRR,               (GPIOA_BASE + GPIOA_BSRR_OFFSET)

// onboard button connected to PA0, so needs to be input mode
// MODER : 2bit field needs to be 00 for gpio input, 01 for gpio output, 10 for alternate, 11 for analog
// we will AND the register to clear the bits for gpio input
.equ MODER0_IN,                0xFFFFFFFC

// on board LED connected to PA1
// MODER : 2bit field needs to be 00 for gpio input, 01 for gpio output, 10 for alternate, 11 for analog
.equ MODER1_OUT,               (1<<2)

// LED on PA1 is active low
// BSRR : write 1 to upper 16bits to reset, write 1 to lower 16bits to set
.equ BSRR_1_RESET,            (1<<17)
.equ BSRR_1_SET,              (1<<1)

.equ LED_OFF,                  BSRR_1_SET
.equ LED_ON,                   BSRR_1_RESET

// PA0 is the button, active high
.equ BTN_ON,      0x0001
.equ BTN_OFF,     0x0000
.equ BTN_PIN,     0x0001

// PUPDR : 2bit field  00 no pullup/pullown, 01 pullup, 10 pulldown, 11 reserved
// we want PA0 pulled down as button K1 on board connects PA0 to VCC
.equ PUPDR0_PULLDOWN,         (1<<1)

.section .text
.globl	main


main:
        BL  GPIOA_Init

Loop:
		BL 	GetInput
        // [ro] = (GPIOA_IDR & BTN_ON)
        CMP R0, #BTN_ON
        BEQ LedOn
        CMP R0, #BTN_OFF
        BEQ LedOff

        B   Loop

LedOn:
        LDR     R2, =GPIOA_BSRR
        MOV     R1, #LED_ON
        STR     R1, [R2]
        B       Loop

LedOff:
        LDR     R2, =GPIOA_BSRR
        MOV     R1, #LED_OFF
        STR     R1, [R2]
        B       Loop

GetInput:
        LDR     R1, =GPIOA_IDR
        LDR     R0, [R1]
        AND     R0, R0, #BTN_PIN
        BX      LR


GPIOA_Init:
        // RCC->AHB1ENR |= GPIOA_EN

        // r0 = RCC_AHB1ENR register address
        LDR     R0, =RCC_AHB1ENR
        // r1 = [RCC_AHB1_ENR], i.e. register content
        LDR     R1, [R0]
        // set the GPIOA peripheral clock enable bit
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

        // configure pin PA0 connected to onboard button K1
        // GPIOA->MODER &= 0xFFFFFFFC
        LDR     R0, =GPIOA_MODER
        LDR     R1, [R0]
        AND     R1, #MODER0_IN
        STR     R1, [R0]

        // enable pulldown for PA0 button
        LDR     R0, =GPIOA_PUPDR
        LDR     R1, [R0]
        ORR     R1, #PUPDR0_PULLDOWN
        STR     R1, [R0]

        BX      LR


.end



