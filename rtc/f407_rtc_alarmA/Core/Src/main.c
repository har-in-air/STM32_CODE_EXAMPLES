// DevEBox STM32F407VGT6 development board
// Demo RTC Alarm feature
// Alarm every 45minutes 9 seconds.
// Alarm every day at 12:00:15
// Alarm every Sunday at 12:00:05
// Configure the RTC calendar and alarm in button press isr

#include "main.h"


UART_HandleTypeDef huart1;
RTC_HandleTypeDef  hrtc;

void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART1_Init(void);
static void RTC_Init(void);
void RTC_CalendarConfig(void);
void Error_Handler(void);
void RTC_AlarmConfig(void);

void printDateTime(void);
char* getWeekDay(uint8_t num);


int main(void){
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    UART1_Init();
    RTC_Init();

    while (1){

    	}
    return 0;
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
	sTime.Hours = 12;
	sTime.Minutes = 45;
	sTime.Seconds = 0;
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

void HAL_GPIO_EXTI_Callback(uint16_t gpioPin) {
    RTC_CalendarConfig();
	RTC_TimeTypeDef rtcTime;
	HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
	RTC_DateTypeDef rtcDate;
	HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);

	printMsg("Current time : %02d:%02d:%02d\r\n", rtcTime.Hours, rtcTime.Minutes, rtcTime.Seconds);
	printMsg("Current date : 20%02d-%02d-%02d [%s]\r\n", rtcDate.Year, rtcDate.Month, rtcDate.Date, getWeekDay(rtcDate.WeekDay));

	RTC_AlarmConfig();
	}


void RTC_AlarmConfig(void) {
	RTC_AlarmTypeDef configAlarmA = {0};

	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
	configAlarmA.Alarm = RTC_ALARM_A;
#if 0
	// every 45 minutes and 09 seconds generate Alarm event and interrupt
	configAlarmA.AlarmTime.Minutes = 45;
	configAlarmA.AlarmTime.Seconds = 9;
	// not interested in hours, date or weekday, mask them
	configAlarmA.AlarmMask = RTC_ALARMMASK_HOURS | RTC_ALARMMASK_DATEWEEKDAY;
	configAlarmA.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
#endif
#if 0
	// @ 12:45:15 every day generate Alarm event and interrupt
	configAlarmA.AlarmTime.Hours = 12;
	configAlarmA.AlarmTime.Minutes = 45;
	configAlarmA.AlarmTime.Seconds = 15;
	// not interested in the specific date or weekday
	configAlarmA.AlarmMask =  RTC_ALARMMASK_DATEWEEKDAY;
	configAlarmA.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
#endif
#if 1
	// @ 12:45:05 every Sunday generate Alarm event and interrupt
	configAlarmA.AlarmTime.Hours = 12;
	configAlarmA.AlarmTime.Minutes = 45;
	configAlarmA.AlarmTime.Seconds = 05;
	configAlarmA.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
	configAlarmA.AlarmDateWeekDay = RTC_WEEKDAY_SUNDAY;
	configAlarmA.AlarmMask = RTC_ALARMMASK_NONE;
	configAlarmA.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
#endif


	if (HAL_OK != HAL_RTC_SetAlarm_IT(&hrtc, &configAlarmA, RTC_FORMAT_BIN)) {
		Error_Handler();
		}
	printMsg("Alarm set\r\n");
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
     // PA0 button press should generate interrupt
     gpioConfig.Mode = GPIO_MODE_IT_RISING;
     gpioConfig.Pull = GPIO_PULLDOWN;
     HAL_GPIO_Init(GPIOA, &gpioConfig);

     // Configure GPIO pin : PA1 board led
     gpioConfig.Pin = GPIO_PIN_1;
     gpioConfig.Mode = GPIO_MODE_OUTPUT_PP;
     gpioConfig.Pull = GPIO_NOPULL;
     gpioConfig.Speed = GPIO_SPEED_FREQ_LOW;
     HAL_GPIO_Init(GPIOA, &gpioConfig);

     // EXTI interrupt init
     HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
     HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	}

char* getWeekDay(uint8_t num) {
	char* szWeekDay[] = {"Monday","Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
	return szWeekDay[num-1];
	}

void printDateTime(void){
	// getTime locks the values in the shadow registers until getDate is called
	// so even if only interested in time, call getDate after getTime
	RTC_TimeTypeDef sTime;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	RTC_DateTypeDef sDate;
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	printMsg("\r\nTime : %02d:%02d:%02d\r\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
	printMsg("Date : 20%02d-%02d-%02d %s\r\n", sDate.Year, sDate.Month, sDate.Date, getWeekDay(sDate.WeekDay));
	}


void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef* hrtc){
	// Show alarm event by pulsing the onboard LED
	// note if  you use HAL_Delay in the isr (not good idea), the systick irq must be
	// higher priority than the rtc alarm irq priority !!
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // active low on
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	printMsg("Alarm triggered\r\n");
	RTC_TimeTypeDef rtcTime;
	HAL_RTC_GetTime(hrtc, &rtcTime, RTC_FORMAT_BIN);
	// dummy call to get date after get time, to unlock the shadow registers
	RTC_DateTypeDef sDate;
	HAL_RTC_GetDate(hrtc, &sDate, RTC_FORMAT_BIN);
	printMsg("Current time : %02d:%02d:%02d\r\n", rtcTime.Hours, rtcTime.Minutes, rtcTime.Seconds);
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
	int counter = 0;
	while (1) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
		while (counter < 0x3FFFF) {counter++;}
		counter = 0;
		}
    }


