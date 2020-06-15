// DevEBox STM32F407VGT6 development board
// Generate a periodic signal using a basic timer peripheral e.g. TIM6

#include "main.h"
#include "stdio.h"
#include "string.h"

TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void GPIO_Init(void);
static void TIM6_Init(void);
static void UART1_Init(void);
void Error_Handler(void);


//#define TIM_BASE_POLL
#define TIM_BASE_INT

int main(void){
    HAL_Init();

    SystemClock_Config();
    GPIO_Init();
    TIM6_Init();
    UART1_Init();

    char msg[100];

    sprintf(msg, "SYSCLK : %ld\r\n", HAL_RCC_GetSysClockFreq());
    HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

    sprintf(msg, "HCLK : %ld\r\n", HAL_RCC_GetHCLKFreq());
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    sprintf(msg, "PCLK1 : %ld\r\n", HAL_RCC_GetPCLK1Freq());
    HAL_UART_Transmit(&huart1,(uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

    sprintf(msg, "PCLK2 : %ld\r\n", HAL_RCC_GetPCLK2Freq());
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

#ifdef TIM_BASE_POLL
    // basic timer in polling mode, 100mS led toggle
    HAL_TIM_Base_Start(&htim6); // enable the upcounter ( TIM6->CR1.en )
    // waste of processor resource if there is other work to be done
    while(1) {
        // wait until UIF flag (update event) is set in the TIM6 status register TIM6_SR
        while (!(TIM6->SR & TIM_SR_UIF));
        // we need to clear the flag
        TIM6->SR = 0;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
        }
#endif

#ifdef TIM_BASE_INT
    // basic timer in interrupt mode, 100mS led toggle
    TIM6->SR = 0;  // clear status register to avoid spurious interupts
    HAL_TIM_Base_Start_IT(&htim6); // enable the upcounter ( TIM6->CR1.en )
    // note : even if processor is in sleep mode, counter continues to work
    // so in interrupt mode, we can put the processor to sleep and wake up on the
    // update event interrupt.
    // on update event, ISR called and then the PeriodElapsedCallback.
    // led is toggled in the callback. Not 100% timing accurate as
    // we have delays due to ISR and gpio toggle code execution
    // Use output compare mode for precise small periods - the timer hw will 
    // change the gpio pin state
    while(1) {
        }
#endif

    }


#ifdef TIM_BASE_INT
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim_base){
    if	(htim_base->Instance==TIM6)  {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
        }
    }
#endif    


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
    //osc_init.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    // when using active clock oscillator input at OSC_IN, OSC_OUT is NC
    //osc_init.HSEState = RCC_HSE_BYPASS;
    // Zero out the struct to ensure fields not required for basic timer operation 
    RCC_OscInitTypeDef oscInit = {0};
    oscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscInit.HSEState = RCC_HSE_ON;
    // if using PLLCLK as clk_init.SYSCLKSource instead of HSE or HSI, do these steps
    // PLLCLK = ((fHSE/M)*N)/P
    // for PLLCLK = 100MHz with crystal of 8MHz, we use prescaler M = 8,
    // multiplier N = 200, post divider P = 2
    oscInit.PLL.PLLState = RCC_PLL_ON;
    oscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscInit.PLL.PLLM = 8;
    oscInit.PLL.PLLN = 200;
    oscInit.PLL.PLLP = RCC_PLLP_DIV2;
    oscInit.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&oscInit) != HAL_OK)  {
        Error_Handler();
        }
    // we want  HCLK = 100MHz, PCLK1 = 25MHz, PCLK2 = 50MHz
    // HCLK is the main CPU clock, also used for AHB interface. It can be gated when the CPU is sleeping
    // (WFI for example)
    // FCLK is synchronous to HCLK but is not gated when the CPU goes to sleep, so that it can awake
    // in case of interrupt.
    // HCLK and FCLK are derived from SYSCLK by the AHB divider
    RCC_ClkInitTypeDef clkInit = {0};
    clkInit.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                          |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clkInit.APB1CLKDivider = RCC_HCLK_DIV4;
    clkInit.APB2CLKDivider = RCC_HCLK_DIV2;
    // flash wait states depend on HCLK and supply voltage
    // for 2.7 - 3.6 V
    // 0 < HCLK < 30MHz, WS = 0
    // 30 to 60 => WS = 1
    // 60 to 90 => WS = 2
    // for 100MHz  WS = 3
    if (HAL_RCC_ClockConfig(&clkInit, FLASH_LATENCY_3) != HAL_OK)  {
        Error_Handler();
        }
    // reconfigure SysTick (because HAL layer needs a 1ms period to work)
    // initializes periodic timer and interrupt
    // specify tick source HCLK or HCLKDIV8
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    // specify numTicks between interrupts using this tick source, for 1000Hz tick frequency
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    }



static void TIM6_Init(void){
    htim6.Instance = TIM6; // base address of peripheral
    // configuring for basic timer operation
    // TIM6 in Memory and Bus architecture in ref manual is on APB1
    // TIM_CLK = APB1 Timer Clock (see cubemx clock tree) = 50MHz
    // TIM_CNT_CLK = TIM_CLK /(1 + Prescaler)
    // 16bit (0-65535) value written into TIM6_PSC
    htim6.Init.Prescaler = 100-1;
    // for basic timer, .CounterMode has only one relevant option count up
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    // 16bit (0-65535) value written into AutoReloadRegister TIM6_ARR at next update event
    // if 0, timer will not start even if enabled
    // for 100mS period (10Hz), we need to count up to (50MHz/100)/10Hz = 50000, ok < 65535
    htim6.Init.Period = 50000-1;
    // this will call TIM_Base_MspInit where we need to do the low level initialization
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)  {
        Error_Handler();
        }
    }



static void GPIO_Init(void){
    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // Configure GPIO pin Output Level for board builtin LED on PA1, which is active LOW
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

	// low level initialization
	// enable peripheral clock, mux pins, enable peripheral int in NVIC and set priority
	// this is done in HAL_UART_Msp_Init() called by HAL_Uart_Init()
	}


void Error_Handler(void){
	int counter = 0;
	while (1) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
		while (counter < 0x3FFFF) {counter++;}
		counter = 0;
		}
    }



