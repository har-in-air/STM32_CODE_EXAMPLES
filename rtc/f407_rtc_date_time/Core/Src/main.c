// DevEBox STM32F407VGT6 development board
// Demo RTC Calendar (date/time) configuration and read
// Also demonstrate that RTC is not affected by going to standby
// mode (Standby mode does not affect LSE, RTC continues to get clock
// and RTC is in backup domain)

#include "main.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"

UART_HandleTypeDef huart1;
RTC_HandleTypeDef  hrtc;

void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART1_Init(void);
static void RTC_Init(void);
void RTC_CalendarConfig(void);
void Error_Handler(void);
void printMsg(char* format, ...);

void printDateTime(void);


#define TEST_STANDBY

int main(void){
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    UART1_Init();
    RTC_Init();

    if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) {
    	// this is a wake up from standby
    	// hw set these flags, sw must clear them
    	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
    	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    	printMsg("\r\nWoke up from standby, check date-time\r\n");
        printDateTime();
    	}
    else {
    	printMsg("\r\nNormal reset, check date-time");
    	printDateTime();
    	}

    // uncomment this if you want to set the date and time
    //RTC_CalendarConfig();

#ifdef TEST_STANDBY
    int counter = 10;
    printMsg("\r\nPress reset during countdown, for date-time after normal reset\r\n");
    printMsg("Let it go to standby, for date-time after standby\r\n");

    while (counter) {
    	printMsg("Standby mode in %d seconds\r\n", counter);
    	counter--;
        HAL_Delay(1000);
    	}
    // set PA0 as wake up pin
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

    printMsg("Went to standby, press button K1 (PA0) to wake up\r\n");
    HAL_PWR_EnterSTANDBYMode();
    // on exit from standby (WKUP_PIN_1 event), CPU will go into system reset
    // and SB and WU flags  will be set in the PWR status register
#else
    // test calendar read on pressing K1 button
    printMsg("Press button K1 for current date and time\r\n");
    while (1) {
    	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
    		HAL_Delay(50); // debounce
    		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
    			printDateTime();
    			}
    		HAL_Delay(1000);
    		}
        }
#endif
    }


void printMsg(char* format, ...) {
	char sz[100];
	va_list args;
	va_start(args, format);
	vsprintf(sz, format, args);
	HAL_UART_Transmit(&huart1, (uint8_t *)sz, strlen(sz), HAL_MAX_DELAY);
	va_end(args);
	}


static void RTC_Init(void) {
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	// set divisors for generating 1second clock from LSE = 32768Hz
	hrtc.Init.AsynchPrediv = 127; // this is actually default
	hrtc.Init.SynchPrediv = 255; // this is actually default
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE; // not using output pins
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_LOW; // dont care
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;  // dont care

	if (HAL_OK != HAL_RTC_Init(&hrtc) ) {
		Error_Handler();
		}
	}


void RTC_CalendarConfig(void){
	// set calendar to 2:43:24 PM, 14th june 2020 Sunday
	RTC_TimeTypeDef sTime;
	sTime.Hours = 14;
	sTime.Minutes = 43;
	sTime.Seconds = 24;
	sTime.TimeFormat = RTC_HOURFORMAT_24;
	if (HAL_OK != HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN)) {
		Error_Handler();
		}

	RTC_DateTypeDef sDate;
	sDate.Date = 14;
	sDate.Month = RTC_MONTH_JUNE;
	sDate.Year = 20; // 2020
	sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
	if (HAL_OK != HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN)) {
		Error_Handler();
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

    // Configure GPIO pin Output Level for board builtin LED (which is active LOW) on PA1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

    GPIO_InitTypeDef gpioConfig = {0};
    // Configure GPIO pin : PA0  board built in button K1 as default input (Wakeup)
    // Button connects PA0 to VCC
     gpioConfig.Pin = GPIO_PIN_0;
#ifdef TEST_STANDBY
     // PA0 button press should generate WKP_PIN_1 event
     gpioConfig.Mode = GPIO_MODE_EVT_RISING;
#else
     // PA0 button press normal input
     gpioConfig.Mode = GPIO_MODE_INPUT;
#endif
     gpioConfig.Pull = GPIO_PULLDOWN;
     HAL_GPIO_Init(GPIOA, &gpioConfig);

     // Configure GPIO pin : PA1 board led
     gpioConfig.Pin = GPIO_PIN_1;
     gpioConfig.Mode = GPIO_MODE_OUTPUT_PP;
     gpioConfig.Pull = GPIO_NOPULL;
     gpioConfig.Speed = GPIO_SPEED_FREQ_LOW;
     HAL_GPIO_Init(GPIOA, &gpioConfig);
    }

char* getWeekDay(uint8_t num) {
	char* szWeekDay[] = {"Monday","Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
	return szWeekDay[num-1];
	}

void printDateTime(void){
	RTC_TimeTypeDef sTime;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	RTC_DateTypeDef sDate;
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	printMsg("\r\nTime : %02d:%02d:%02d\r\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
	printMsg("Date : 20%02d-%02d-%02d %s\r\n", sDate.Year, sDate.Month, sDate.Date, getWeekDay(sDate.WeekDay));
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


