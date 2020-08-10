// adc driver

.cpu	cortex-m4
.syntax	unified
.thumb

.equ RCC_BASE,    			0x40023800

.equ AHB1ENR_OFFSET,        0x30
.equ RCC_AHB1ENR,           (RCC_BASE + AHB1ENR_OFFSET)

.equ APB1ENR_OFFSET,        0x40
.equ RCC_APB1ENR,           (RCC_BASE + APB1ENR_OFFSET)

.equ APB2ENR_OFFSET,        0x44
.equ RCC_APB2ENR,           (RCC_BASE + APB2ENR_OFFSET)

//--------------------- ADC ---------------------------

.equ ADC1_BASE, 			0x40012000

.equ APB2_ADC1_EN,			(1<<8)


.equ ADC1_SR_OFFSET, 		0x00
.equ ADC1_SR, 				(ADC1_BASE + ADC1_SR_OFFSET)

.equ ADC1_SR_EOC,			(1<<1)


.equ ADC1_CR2_OFFSET, 		0x08
.equ ADC1_CR2, 				(ADC1_BASE + ADC1_CR2_OFFSET)

.equ ADC1_CR2_EN,			(1<<0)  // adc enable (1) / disable(0)
.equ ADC1_CR2_SWSTART,		(1<<30) // software start conversion
.equ ADC1_CR2_SWTRIG,		0 // software triggered conversion


.equ ADC1_SQR1_OFFSET, 		0x2C
.equ ADC1_SQR1,				(ADC1_BASE + ADC1_SQR1_OFFSET)

.equ ADC1_SQR1_CNF,			(0<<20) // conversion sequence length = 1

.equ ADC1_SQR3_OFFSET, 		0x34
.equ ADC1_SQR3,				(ADC1_BASE + ADC1_SQR3_OFFSET)

.equ ADC1_SQR3_CNF,			(4<<0) // conversion sequence SQ1 = channel 4 (PA4)

.equ ADC1_DR_OFFSET, 		0x4C
.equ ADC1_DR, 				(ADC1_BASE + ADC1_DR_OFFSET)


//------------------- GPIOA registers ---------------------

.equ GPIOA_BASE,            0x40020000

.equ GPIOA_MODER_OFFSET,    0x00
.equ GPIOA_MODER,           (GPIOA_BASE + GPIOA_MODER_OFFSET)

.equ GPIOA_AFRL_OFFSET, 	0x20
.equ GPIOA_AFRL,           	(GPIOA_BASE + GPIOA_AFRL_OFFSET)

.equ GPIOA_BSRR_OFFSET,     0x18
.equ GPIOA_BSRR,            (GPIOA_BASE + GPIOA_BSRR_OFFSET)


// 12bit ADC, max = 4095
// turn on LED if sample > threshold
.equ SAMPLE_THRESHOLD,		3000

//------------------ GPIOA settings ----------------------------

// enable peripheral clock to GPIOA in AHB1ENR
.equ GPIOA_EN,				(1<<0)


// MODER 2bit field = 00 gpio input, 01 gpio output, 10 alternate func, 11 analog

// PA1: MODER1 (3,2) = 01 = output connected to onboard LED
.equ MODER1_OUT,            (1<<2)

// PA4: MODER4 (9,8) = 11 = analog adc channel 4
.equ MODER4_ANA,			(3 << 8)


// LED on PA1 is active low
// BSRR : write 1 to upper 16bits to reset, write 1 to lower 16bits to set
.equ BSRR_1_RESET,            (1<<17)
.equ BSRR_1_SET,              (1<<1)

.equ LED_OFF,                  BSRR_1_SET
.equ LED_ON,                   BSRR_1_RESET

.section .text
.global		GPIOInit
.global		ADC1Init
.global		ADC1Read
.global		LEDControl

GPIOInit:
        // RCC->AHB1ENR |= GPIOA_EN
        // R0 = RCC_AHB1ENR register address
        LDR     R0, =RCC_AHB1ENR
        // R1 = [RCC_AHB1_ENR], i.e. register content
        LDR     R1, [R0]
        // R1 |=  GPIOA peripheral clock enable
        ORR     R1, #GPIOA_EN
        // [RCC_AHB1_ENR] = R1
        STR     R1, [R0]

		// Configure pin PA1 as output connected to onboard LED
		// Configure pin PA4 as analog connected to ADC channel 4
        // GPIOA->MODER |= (MODER1_OUT | MODE4_ANA)
        LDR     R0, =GPIOA_MODER
        LDR     R1, [R0]
        ORR     R1, #(MODER1_OUT | MODER4_ANA)
        STR     R1, [R0]

		// LED off
        LDR     R2, =GPIOA_BSRR
        MOV     R1, #LED_OFF
        STR     R1, [R2]
		BX		LR


ADC1Init:
        LDR     R0, =RCC_APB2ENR
        // R1 = [RCC_APB2_ENR]
        LDR     R1, [R0]
        // R1 |=  ADC1 clock enable
        ORR     R1, #APB2_ADC1_EN
        // [RCC_APB2_ENR] = R1
        STR     R1, [R0]

		// configure software trigger
        LDR     R0, =ADC1_CR2
        LDR		R1, [R0]
        ORR     R1, #ADC1_CR2_SWTRIG
        STR     R1, [R0]

		// seq 1 is ADC chan 4
        LDR     R0, =ADC1_SQR3
        MOV     R1, #ADC1_SQR3_CNF
        STR     R1, [R0]

		// num seq conversions = 1
        LDR     R0, =ADC1_SQR1
        MOV     R1, #ADC1_SQR1_CNF
        STR     R1, [R0]

		// enable ADC
        LDR     R0, =ADC1_CR2
        LDR		R1, [R0]
        ORR     R1, #ADC1_CR2_EN
        STR     R1, [R0]
        BX		LR


ADC1Read:
        LDR     R0, =ADC1_CR2
        LDR		R1, [R0]
        ORR     R1, #ADC1_CR2_SWSTART
        STR     R1, [R0]

		//wait for conversion complete
        LDR     R0, =ADC1_SR
lp1:    LDR		R1, [R0]
		AND		R1,  #ADC1_SR_EOC
		CMP		R1, #0
		BEQ		lp1

		// complete, return sample in R0
		LDR		R2, = ADC1_DR
		LDR		R0, [R2]
		BX		LR


LEDControl: // R0 contains the ADC Sample
		LDR		R1, =SAMPLE_THRESHOLD
		CMP		R0, R1
		BGT		LEDOn  // > threshold, turn on LED
		BLT		LEDOff // < threshold turn off LED
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


.end



