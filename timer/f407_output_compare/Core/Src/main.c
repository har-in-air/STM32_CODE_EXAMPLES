// DevEBox STM32F407VGT6 development board
// Timer Output Compare 
// Generate square waves at 500Hz, 1khz, 2khz and 4khz on TIM2 output channels

#include "main.h"
#include "stdio.h"
#include "string.h"

TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void GPIO_Init(void);
static void TIM2_Init(void);
static void UART1_Init(void);
void Error_Handler(void);


#define printSz(sz)		   HAL_UART_Transmit(&huart1, (uint8_t*)sz, strlen(sz), HAL_MAX_DELAY)


int main(void){
    HAL_Init();

    SystemClock_Config();
    GPIO_Init();
    UART1_Init();


    char msg[100];

    #if 1
    sprintf(msg, "\r\nOutput Compare : \r\nSYSCLK : %ld\r\n", HAL_RCC_GetSysClockFreq());
    printSz(msg);

    sprintf(msg, "HCLK : %ld\r\n", HAL_RCC_GetHCLKFreq());
    printSz(msg);

    sprintf(msg, "PCLK1 : %ld\r\n", HAL_RCC_GetPCLK1Freq());
    printSz(msg);

    sprintf(msg, "PCLK2 : %ld\r\n", HAL_RCC_GetPCLK2Freq());
    printSz(msg);
    #endif


    TIM2_Init();
    TIM2->SR = 0; // clear status register to avoid spurious interrupts before we're ready
    if (HAL_OK != HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1)) {
        Error_Handler();
        }
    if (HAL_OK != HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2)) {
        Error_Handler();
        }
    if (HAL_OK != HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3)) {
        Error_Handler();
        }
    if (HAL_OK != HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4)) {
        Error_Handler();
        }

    while (1) {
        }
    }



void SystemClock_Config(void){
    // for high frequency 100MHz operation, we need to set voltage regulator scale 1
    // (reset value is scale 2)
    // enable clock for power controller
    __HAL_RCC_PWR_CLK_ENABLE();
    // set regulator voltage scale
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // if using HSI with PLL we need to set the calibration value, else it will
    // be zeroed out, which is not the power on / reset default value
    // =  RCC_HSICALIBRATION_DEFAULT = 16 which is
    // guaranteed to give 1% accuracy in HSI clock frequency @ 25C.
    // Worst case 4% error in mcu datasheet operating temperature range
    // You can measure the HSI frequency and modify by  default +/- increment
    // = 0.5%  =  +/-80kHz

    // when using active clock oscillator input at OSC_IN, OSC_OUT is NC
    //osc_init.HSEState = RCC_HSE_BYPASS;
    // other osc_init fields are not applicable (and zeroed out by memset)
    RCC_OscInitTypeDef oscConfig = {0};
    oscConfig.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    oscConfig.HSEState = RCC_HSE_ON;
    oscConfig.LSEState = RCC_LSE_ON;
    //oscConfig.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    // if using PLLCLK as clk_init.SYSCLKSource instead of HSE or HSI, do these steps
    // PLLCLK = ((fHSE/M)*N)/P
    // for PLLCLK = 100MHz with crystal of 8MHz, we use prescaler M = 8,
    // multiplier N = 200, post divider P = 2
    oscConfig.PLL.PLLState = RCC_PLL_ON;
    oscConfig.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscConfig.PLL.PLLM = 8;
    oscConfig.PLL.PLLN = 200;
    oscConfig.PLL.PLLP = RCC_PLLP_DIV2;
    oscConfig.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&oscConfig) != HAL_OK)  {
        Error_Handler();
        }

	// we want HCLK = 100MHz, PCLK1 = 25MHz, PCLK2 = 50MHz
	// HCLK is the main CPU clock, also used for AHB interface. It can be gated when the CPU is sleeping
	// FCLK is synchronous to HCLK but is not gated when the CPU goes to sleep, so that it can wake
	// in case of interrupt.
	// HCLK and FCLK are derived from SYSCLK by the AHB Clk divider
    RCC_ClkInitTypeDef clkConfig = {0};
    clkConfig.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clkConfig.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clkConfig.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clkConfig.APB1CLKDivider = RCC_HCLK_DIV4;
    clkConfig.APB2CLKDivider = RCC_HCLK_DIV2;
    // flash wait states depend on HCLK and supply voltage
    // for 2.7 - 3.6 V
    // 0 < HCLK < 30MHz, WS = 0
    // 30 to 60 => WS = 1
    // 60 to 90 => WS = 2
    // for 100MHz  WS = 3
    if (HAL_RCC_ClockConfig(&clkConfig, FLASH_LATENCY_3) != HAL_OK)  {
        Error_Handler();
        }

    // reconfigure SysTick because HAL layer needs a 1ms period (1000Hz) tick timer to work
    // initializes periodic timer and interrupt
    // specify tick source HCLK or HCLKDIV8
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    // specify numTicks between interrupts using this tick source, for 1000Hz tick frequency
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0); // highest priority
    }

// we want to generate 4 square waves at 500Hz, 1kHz, 2kHz and 4kHz
uint32_t pulse1_val = 25000;
uint32_t pulse2_val = 12500;
uint32_t pulse3_val = 6250;
uint32_t pulse4_val = 3125;


static void TIM2_Init(void){
    // base address of peripheral
    // TIM2 is on APB1 (see stm32f407xx.h_
    htim2.Instance = TIM2; 
    // TIM_CLK = APB1 Timer Clock (see cubemx clock tree) = PCLK1*2
    // TIM_CNT_CLK = TIM_CLK /(1 + Prescaler)

    // 16bit (0-65535) value written into TIM2_PSC
    htim2.Init.Prescaler = 1; // divide by 2
    // for input capture, .CounterMode has only one relevant option count up
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    // TIM2 32bit counter, we need to count up to the max value 0XFFFF FFFF
    htim2.Init.Period = 0xFFFFFFFF;

    // when output capture is used, need to use TIM_OC_Init instead of TIM_Base_Init
    if (HAL_TIM_OC_Init(&htim2) != HAL_OK)  {
        Error_Handler();
        }
    // now we need to configure the output channel
    // only set fields required for output compare mode
    // make sure all other fields are zeroed out !
    TIM_OC_InitTypeDef ocConfig = {0};
    // Using output toggle mode
    // Counter reaches CCR1, output channel is toggled and isr called. In Isr we
    // set the next CCR1 value, and the process repeats
    ocConfig.OCMode = TIM_OCMODE_TOGGLE;
    ocConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    // we want a square wave at 500Hz, so we need to toggle at 1kHz
    // TIM_CNT_CLK = 25MHz
    // we want first toggle at 25MHz/1kHz ticks = 25000
    ocConfig.Pulse = pulse1_val;
    // this function will call HAL_TIM_OC_MspInit which we need to define
    if (HAL_TIM_OC_ConfigChannel(&htim2, &ocConfig, TIM_CHANNEL_1) != HAL_OK)  {
        Error_Handler();
        }
    // square wave 1000Hz, so we need to toggle at 2kHz
    //  1000Hz : we want first toggle at 25MHz/2kHz ticks = 12500
    ocConfig.Pulse = pulse2_val;
    if (HAL_TIM_OC_ConfigChannel(&htim2, &ocConfig, TIM_CHANNEL_2) != HAL_OK)  {
        Error_Handler();
        }
    // 2000Hz : we want first toggle at 25MHz/4kHz ticks = 6250
    ocConfig.Pulse = pulse3_val;
    if (HAL_TIM_OC_ConfigChannel(&htim2, &ocConfig, TIM_CHANNEL_3) != HAL_OK)  {
        Error_Handler();
        }
    // 4000Hz : we want first toggle at 25MHz/8kHz ticks = 3250
    ocConfig.Pulse = pulse4_val;
    if (HAL_TIM_OC_ConfigChannel(&htim2, &ocConfig, TIM_CHANNEL_4) != HAL_OK)  {
        Error_Handler();
        }
    }


// output compare, this is the callback we are interested in
// global variable to avoid stack push/pop overhead
uint32_t ccr;
// set the new CCR1..4 values for the next toggle
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
		ccr = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, ccr + pulse1_val);
		}
	else
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		ccr = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, ccr + pulse2_val);
		}
	else
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
		ccr = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, ccr + pulse3_val);
		}
	else
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
		ccr = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, ccr + pulse4_val);
		}
	}


static void GPIO_Init(void){
    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure GPIO pin Output Level for board builtin LED (which is active LOW) on PA1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure GPIO pin : PA0  board built in button (also wakeup pin)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure GPIO pin : PA1  board builtin LED
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }



static void UART1_Init(void) {
	// base address of peripherals available as macros in stm32f411xe.h
	// e.g. #define USART1              ((USART_TypeDef *) USART1_BASE)
	huart1.Instance = USART1;
	// high level parameter initialization using the Init structure
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	// high level parameter initialization and create handle
	if (HAL_OK != HAL_UART_Init(&huart1)) {
		Error_Handler();
		}
	// low level initialization in msp.c : HAL_UART_Msp_Init(), which is called by HAL_UART_Init()
	}


void Error_Handler(void){
    // Toggle the board LED rapidly
	int counter = 0;
	while (1) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
		while (counter < 0x3FFFF) {counter++;}
		counter = 0;
		}
    }


