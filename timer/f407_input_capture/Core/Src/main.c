// DevEBox STM32F407VGT6 development board
// Timer Input Compare 
// Measure the time period of an external periodic waveform output using gen purpose time TIM2

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

// External waveform signal is generated from the LSE 32768Hz crystal oscillator via the MCO1 pin
void MCO1_LSEConfig(void);


#define printSz(sz)		   HAL_UART_Transmit(&huart1, (uint8_t*)sz, strlen(sz), HAL_MAX_DELAY)

volatile uint32_t capture[2] = {0};
volatile int eventCount = 0;
volatile int isCaptureDone = 0;

int main(void){
    HAL_Init();

    SystemClock_Config();
    GPIO_Init();
    UART1_Init();

    char msg[100];

    #if 1
    sprintf(msg, "\r\nSYSCLK : %ld\r\n", HAL_RCC_GetSysClockFreq());
    printSz(msg);

    sprintf(msg, "HCLK : %ld\r\n", HAL_RCC_GetHCLKFreq());
    printSz(msg);

    sprintf(msg, "PCLK1 : %ld\r\n", HAL_RCC_GetPCLK1Freq());
    printSz(msg);

    sprintf(msg, "PCLK2 : %ld\r\n", HAL_RCC_GetPCLK2Freq());
    printSz(msg);
    #endif

    // connect MCO1 LSE clock output (PA8)  to TIM2_CH1 input capture pin (PA5) with a jumper wire
    MCO1_LSEConfig();
    // input capture used to measure time period (frequency) of external signals
    // Limitation due to the time taken to process ISR and callback (a few uS @ 100MHz)
    // so we can't measure period of high frequency waveforms

    TIM2_Init();
    TIM2->SR = 0; // clear status register to avoid spurious interrupts before we're ready
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

    uint32_t difference;
    while (1) {
        if (isCaptureDone == 1) {
            if (capture[1] > capture[0]) {
                difference = capture[1] - capture[0];
                }
            else {
                difference = capture[1] + (0xFFFFFFFF - capture[0]);
                }
            // look at cubemx clock tree, timer clock on APB1 is 2x PCLK1
            double tim2_CounterFreq = (HAL_RCC_GetPCLK1Freq() * 2.0) / (htim2.Init.Prescaler+1);
            //double signalPeriod = difference / tim2_CounterFreq; // (difference * counter_clock_period)
            double signalFreq = tim2_CounterFreq / (difference > 0 ? difference : 1);  // 1 / signalPeriod
            // to print floats we need to go to C/C++ Build->Settings-> GCC Linker -> Misc -> Linker Flags
            // and add -u_printf_float
            sprintf(msg, "Input Capture Frequency = %d Hz\r\n", (int)signalFreq);
            printSz(msg);
            isCaptureDone = 0;
            }
        }
    }



void MCO1_LSEConfig(void) {
	// drive LSE on output MCO1 pin PA8 without divider
	// no need to configure the Gpio AF mode, this function does it
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCOSOURCE_LSE, RCC_MCODIV_1);
	}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
	if (isCaptureDone == 0) {
		if (eventCount == 0) {
			capture[0] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
			eventCount++;
			}
		else
		if (eventCount == 1) {
			capture[1] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
			eventCount = 0;
			isCaptureDone = 1;
			}
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
    // we want  HCLK = 100MHz, PCLK1 = 25MHz, PCLK2 = 50MHz
    // HCLK is the main CPU clock, also used for AHB interface. It can be gated when the CPU is sleeping
    // (WFI for example)
    // FCLK is synchronous to HCLK but is not gated when the CPU goes to sleep, so that it can awake
    // in case of interrupt.
    // HCLK and FCLK are derived from SYSCLK by the AHB divider
    RCC_ClkInitTypeDef clkConfig = {0};
    clkConfig.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                          |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clkConfig.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clkConfig.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clkConfig.APB1CLKDivider = RCC_HCLK_DIV4;
    clkConfig.APB2CLKDivider = RCC_HCLK_DIV2;
    // flash wait states depend on HCLK and supply voltage
    // for 2.7 - 3.6 V and  0 < HCLK < 30MHz, WS = 0
    // 30 to 60 => WS = 1
    // 60 to 90 => WS = 2
    // for 100MHz  WS = 3
    if (HAL_RCC_ClockConfig(&clkConfig, FLASH_LATENCY_3) != HAL_OK)  {
        Error_Handler();
        }
    // reconfigure SysTick (HAL layer needs a 1ms period to work)
    // initializes periodic timer and interrupt
    // specify tick source HCLK or HCLKDIV8
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    // specify numTicks between interrupts using this tick source, for 1000Hz tick frequency
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
    }


static void TIM2_Init(void){
    htim2.Instance = TIM2; // base address of peripheral
    // configuring for basic timer operation
    // TIM2 is on APB1 (see stm32f407xx.h_
    // TIM_CLK = APB1 Timer Clock (see cubemx clock tree) = PCLK1*2
    // TIM_CNT_CLK = TIM_CLK /(1 + Prescaler)

    // 16bit (0-65535) value written into TIM2_PSC
    htim2.Init.Prescaler = 0; // divide by 1
    // for input capture, .CounterMode has only one relevant option count up
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    // TIM2 32bit counter, we need to count up to the max value 0XFFFF FFFF
    // Lowest frequency that can be measured is TIM_CNT_CLK / 0xFFFF FFFF
    // Anything lower and the counter will overflow without detection
    htim2.Init.Period = 0xFFFFFFFF;
    // when input capture is used, need to use TIM_IC_Init instead of TIM_Base_Init
    if (HAL_TIM_IC_Init(&htim2) != HAL_OK)  {
        Error_Handler();
        }
    // now we need to configure the input channel
    // ensure the struct is zeroed out, because we only set the fields used for input compare
    TIM_IC_InitTypeDef config = {0};
    config.ICPolarity = TIM_ICPOLARITY_RISING;
    // capture direct mode ie. input channel
    config.ICSelection = TIM_ICSELECTION_DIRECTTI;
    // no scaler
    config.ICPrescaler = TIM_ICPSC_DIV1;
    // no edge noise filter
    config.ICFilter = 0;
    // CC1S field : want CC1 channel configured as input, IC1 mapped to TI1
    // IC1F field : noise filter, 0 to 15, 0 => no filter, 
    // filter defines how many consecutive events must happen for validation of change
    // this function will call HAL_TIM_IC_MspInit which we need to define
    if (HAL_TIM_IC_ConfigChannel(&htim2, &config, TIM_CHANNEL_1) != HAL_OK)  {
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


