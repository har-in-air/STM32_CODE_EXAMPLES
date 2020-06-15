// DevEBox STM32F407VGT6 development board
// Demo Standby with backup SRAM
// Check on reset whether it is a reset on wakeup event from standby mode
// If so clear the flags, then check backup SRAM to see if the previously written string
// is retained in memory.
// To ensure backup SRAM retains memory, we need to enable the backup SRAM regulator
// before entering standby.
// To wake up from standby press button K1 (PA0 configured to generate  WKUP_PIN_1 event)
// To test SRAM memory retention, make sure you wait a minute or so after
// the Cpu goes into standby before waking it up


#include "main.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART1_Init(void);
void Error_Handler(void);
void printMsg(char* format, ...);
void checkBkpSRAM(void);

// base address of backup sram in stm32f407xx.h : 0x40024000
uint8_t* pBkpSRAM = (uint8_t *) BKPSRAM_BASE;

int main(void){
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    UART1_Init();

    // enable clock to backup SRAM
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
    // enable clock to PWR block
    __HAL_RCC_PWR_CLK_ENABLE();
    // backup SRAM is write protected, enable write access
    HAL_PWR_EnableBkUpAccess();

    if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) {
    	// this is a wake up from standby
    	// hw set these flags, sw must clear them
    	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
    	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    	printMsg("Woke up from standby, check backup SRAM data\r\n");
    	checkBkpSRAM();
    	}
    else {
    	printMsg("Normal reset, check backup SRAM data\r\n");
    	checkBkpSRAM();

    	printMsg("Writing string 'Hi' to backup SRAM\r\n");
		*pBkpSRAM = 'H';
		*(pBkpSRAM+1) = 'i';

        // add breakpoint here, check backup SRAM memory with Memory Browser window @ 0x40024000
    	}

    int counter = 5;
    while (counter) {
    	printMsg("Going to Standby mode in %d seconds\r\n", counter);
    	counter--;
        HAL_Delay(1000);
    	}

    // set PA0 as wake up pin
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    // enable backup voltage regulator to preserve backup SRAM data in standby mode
    // if you comment this out, backup SRAM data will be lost on going to standby.
    // ( wait a minute before waking up the CPU when testing this)
    HAL_PWREx_EnableBkUpReg();

    printMsg("Went to Standby mode, press button K1 (PA0 = WKUP_PIN_1) to wake up\r\n");
    HAL_PWR_EnterSTANDBYMode();

    // on exit from standby (WKUP_PIN_1 event), CPU will go into system reset
    // and SB and WU flags  will be set in the PWR status register

    while (1) {
        }
    }

void checkBkpSRAM(void){
	uint8_t c1 =  *pBkpSRAM;
	uint8_t c2 =  *(pBkpSRAM+1);
	if (c1 == 'H' && c2 == 'i') {
		printMsg("Backup SRAM data was retained\r\n");
		}
	else {
		printMsg("Backup SRAM data lost\r\n");
		}
	}

void printMsg(char* format, ...) {
	char sz[100];
	va_list args;
	va_start(args, format);
	vsprintf(sz, format, args);
	HAL_UART_Transmit(&huart1, (uint8_t *)sz, strlen(sz), HAL_MAX_DELAY);
	va_end(args);
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
    // Configure GPIO pin Output Level for board builtin LED (which is active LOW) on PA1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    GPIO_InitTypeDef gpioConfig = {0};
    // Configure GPIO pin : PA0  board built in button K1 to generate WKUP_PIN_1 event
    // button connects PA0 to VCC, so it needs a pulldown
    gpioConfig.Pin = GPIO_PIN_0;
    gpioConfig.Mode = GPIO_MODE_EVT_RISING;
    gpioConfig.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &gpioConfig);
    // Configure GPIO pin : PA1  board builtin LED
    gpioConfig.Pin = GPIO_PIN_1;
    gpioConfig.Mode = GPIO_MODE_OUTPUT_PP;
    gpioConfig.Pull = GPIO_NOPULL;
    gpioConfig.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpioConfig);
    }



static void UART1_Init(void) {
	// base address of peripherals available as macros in stm32f411xe.h
	// e.g. #define USART1              ((USART_TypeDef *) USART1_BASE)
	huart1.Instance = USART1;

	// use high baudrate to reduce total transmission time, this will
	// reduce power consumption
	huart1.Init.BaudRate = 115200;
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
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
        HAL_Delay(50);
        }
    }


