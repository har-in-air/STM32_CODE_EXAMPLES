// DevEBox STM32F407VGT6 development board
// Demo : generation of events (not interrupts) and use of WFE
// 1. Use peripheral interrupt as an event (applicable to all peripherals)
//	set SEVONPEND bit to 1
//	disable peripheral interrupt in NVIC
//	make peripheral issue an interrupt
// 2. Some peripherals can generate events
//  Look at EXTI block diagram in reference manual for specific processor
//   lower part pulse generator is the event generator
//   23 signals can generate events.
//   16 are from gpio port Px0 ... Px15 external interrupts
//   7 signals from PVD (1), USB (2), RTC (4)
// This demo uses peripheral events option 2, i.e. EXTI0 to generate an event
// Use WFE instruction to put the CPU to sleep. Wake up on PA0 button press
// to transmit a message from uart1, returns to main thread, again calls WFE
// to sleep.
// Note that any interrupt will wake up the CPU so we need to suspend the
// SysTick timer, prior to calling wfi. Note also that the first call to WFE
// before the while loop is because the Systick timer would have generated
// an event already, so wfe won't do anything other than clear the event bit.
// This is another difference from WFI. WFE is conditional, i.e. it will put
// the CPU to sleep only if the event bit is not set. If it is already set on
// the call to WFE,  WFE will just clear it and the CPU continues to run.
// also wfi wakeup causes transfer to ISR and then back to normal thread,
// while wfe wakeup just executes the next instruction after wfe.

// -------- further power consumption reduction tips ---------
// disable clock to unneeded peripherals
// reduce cpu clock frequency
// reduce voltage - use voltage scaling 2 or 3 instead of 1, if clock frequency is low.
// On enabling clock to a gpio port, all the pins will be in digital input mode which
// will drain some current. So keep unused pins in analog mode - see gpio_analogConfig.
// Uart : increase baudrate to reduce total transmission time, if rx not required
// initialize in mode TX not TXRX, and gate clock to uart only when Cpu is not
// sleeping - this is just a one-time initialization -  see hal_uart_msp_init.

#include "main.h"
#include "stdio.h"
#include "string.h"

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART1_Init(void);
void Error_Handler(void);
static void GPIO_AnalogConfig(void);


#define printSz(sz)		   HAL_UART_Transmit(&huart1, (uint8_t*)sz, strlen(sz), HAL_MAX_DELAY)


int main(void){
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    GPIO_AnalogConfig(); // save power by putting unused pins in GPIOA in analog mode
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

    // Stop Systick Timer else it will generate events
    HAL_SuspendTick();
    // first WFE call will not send the cpu to sleep because the event
    // bit would already have been set due to the systick timer running.
    // But it will clear the event bit, and subsequent events are only
    // coming from the button press.
	__WFE(); // uses inline assembly to call wfe thumb instruction

    char msgToSend[] = "Woke up from EXTI0 event!\r\n";


    while (1) {
    	__WFE();

    	// CPU resumes here on event generation
    	// do your work and go back to sleep on next WFE call
    	if (HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t*)msgToSend, strlen(msgToSend), HAL_MAX_DELAY)) {
    		Error_Handler();
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
    // and osc_init.HSEState = RCC_HSE_BYPASS;
    // other osc_init fields are not applicable (and zeroed out)
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




static void GPIO_Init(void){
    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // we cannot disable clocks to GPIOA when CPU is asleep because we need PA0 active to wake up
    //__HAL_RCC_GPIOA_CLK_SLEEP_DISABLE();

    // Configure GPIO pin Output Level for board builtin LED (which is active LOW) on PA1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    GPIO_InitTypeDef gpioConfig = {0};
    // Configure GPIO pin : PA0  board built in button as external event pin
    // Button connects PA0 to VCC
     gpioConfig.Pin = GPIO_PIN_0;
     // NOTE : we are generating an event, not an interrupt !!
     gpioConfig.Mode = GPIO_MODE_EVT_RISING;
     gpioConfig.Pull = GPIO_PULLDOWN;
     HAL_GPIO_Init(GPIOA, &gpioConfig);

     // Configure GPIO pin : PA1 board led
     gpioConfig.Pin = GPIO_PIN_1;
     gpioConfig.Mode = GPIO_MODE_OUTPUT_PP;
     gpioConfig.Pull = GPIO_NOPULL;
     gpioConfig.Speed = GPIO_SPEED_FREQ_LOW;
     HAL_GPIO_Init(GPIOA, &gpioConfig);

     // no interrupt generation required
     // EXTI interrupt init
     //HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
     //HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    }


static void GPIO_AnalogConfig(void) {
    GPIO_InitTypeDef gpioConfig = {0};
    // PA0, PA1 used for gpio, PA9/PA10 used for UART1, PA13/PA14 used for SWD
    // keep all unused gpio pins in analog mode
    gpioConfig.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
    		GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_11 |
			GPIO_PIN_12  | GPIO_PIN_15;

    gpioConfig.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &gpioConfig);
	}


static void UART1_Init(void) {
	// base address of peripherals available as macros in stm32f411xe.h
	// e.g. #define USART1              ((USART_TypeDef *) USART1_BASE)
	huart1.Instance = USART1;

	// use high baudrate to reduce total transmission time, this will
	// reduce power consumption
	huart1.Init.BaudRate = 921600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	// using TX mode only to save power
	//huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.Mode = UART_MODE_TX;
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


