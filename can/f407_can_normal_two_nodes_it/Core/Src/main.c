// DevEBox STM32F407VGT6 development board
// Demo CAN1 application in normal mode with two nodes N1,N2 using MCP2551 transceivers.
// Project is compiled identically for node N1 and node N2. You pick which board is supposed to be N1
// and press a button. On pressing the button, the board become N1 and starts periodically sending
// data and remote frames.
//
// N1 sends a data frame every second with the payload consisting of 1 byte encoding LED # (0 to 3)
// N2 receives LED# data frame and turns on only the specific LED #.
// N1 also sends a remote frame every 2 seconds requesting 2bytes of data.
// N2 responds to the remote frame with a data frame containing an incrementing counter as the
// requested 2bytes of data.
// All CAN tx and rx transactions are implemented in interrupt+callbacks mode.


#include "main.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"


UART_HandleTypeDef huart1;
CAN_HandleTypeDef  hcan1;
TIM_HandleTypeDef htim6;

void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART1_Init(void);
static void CAN1_Init(void);
static void TIM6_Init(void);

void CAN1_RX(void);
void CAN_Filter_Config(void);
void SendLEDCtrlMessage(uint32_t stdID);
void SendRemoteResponse(uint32_t stdID);
void SendRemoteRequest(uint32_t stdID);
void LedControl(uint8_t ctrl);

void Error_Handler(void);


int main(void){
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    UART1_Init();
    TIM6_Init();

    printMsg("CAN1 normal mode with two nodes N1, N2 using interrupt\r\n");

    // bxCan on reset is in sleep mode
    // move to init mode by resetting bxCAN sleep bit sleep.inrq.ack
    // and then configure parameters. In init mode, bxCAN does not
    // monitor RX line. Note we are initializing in loopback mode
    CAN1_Init();

    // configure acceptance filtering before moving to normal mode
    CAN_Filter_Config();

    // enable interrupt-generating events of interest to our application
    if (HAL_OK != HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)) {
    	Error_Handler();
    	}

    // move bxCAN from init to normal mode
    if (HAL_OK != HAL_CAN_Start(&hcan1)) {
    	Error_Handler();
    	}

    // at this point both boards are in identical state. If you press PA0 button on one board,
    // it starts a periodic timer and becomes N1, the other becomes N2
    while (1) {
        }
    return 0;
	}


// N1 : 1 second periodic timer callback
uint32_t  TimerCounter = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim_base){
    if (TIM6 == htim_base->Instance)  {
    	// toggle N1 onboard LED every second
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
        // every second, N1 sends data frame for LED control
        SendLEDCtrlMessage(0x65D);
        // every 4 seconds, N1 sends a remote request frame
        if (++TimerCounter >= 2) {
        	TimerCounter = 0;
        	SendRemoteRequest(0x651);
        	}
        }
    }


// Pressing PA0 starts the periodic timer, this board becomes Node N1
void HAL_GPIO_EXTI_Callback(uint16_t gpioPin) {
	if (GPIO_PIN_0 == gpioPin ) {
		// on update event, ISR called and then the PeriodElapsedCallback.
		TIM6->SR = 0;  // clear status register to avoid spurious interupts
		HAL_TIM_Base_Start_IT(&htim6);
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

// use bittiming.can-wiki.info for bit timing configuration
// CAN1 is on APB1 bus, so clk is PCLK1 = 25MHz
// 1 bit has Nq time quanta (depending on the prescaler), divided into 4 segments
// We have to allocate Nq across the 4 segments.
// As per website higher Nq is better (16 recommended for CAN FD).
// The yellow highlighted rows in the table are optimal working options.
// sync seg (always 1 quantum)
// propagation
// phase 1  (sampling happens between phase 1 and phase 2
// phase 2
// @ 25MHz PCLK1 we cannot do 1Mbps, max is 500kbps with Nq=10, optimal is 125kbps with Nq=16
// we are setting up for 500Kbps
// sync:1q +  (prop + ph1):8q + ph2:1q = 10q
void CAN1_Init(void){
	hcan1.Instance = CAN1;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	// from website for 25MHz clk and 500kbps
	hcan1.Init.Prescaler = 5;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;

    // automatic retransmission on transmit errors
    hcan1.Init.AutoRetransmission = ENABLE;
    // automatically recover from bus-off state when in normal mode.
    // auto-recovery needs to see 128 sequences of 11 recessive bits on CANRX line before
    // becoming bus active again. If disabled, sw has to take bxCAN to init
    // and then normal mode again.
    hcan1.Init.AutoBusOff = ENABLE;
    // overrun the oldest received message if fifo is full
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    // transmission priority depends on message identifier, not chronological order
    hcan1.Init.TransmitFifoPriority = DISABLE ;
    hcan1.Init.TimeTriggeredMode = DISABLE;

    if (HAL_OK != HAL_CAN_Init(&hcan1)){
    	Error_Handler();
    	}
	}


// First test with accepting all frames so we accept both the 0x65D and 0x651 frames.
// accept all frames => Filter Mask = 0x0000 0000 and Filter ID is don't care (0x0000 0000).
// You will see Node 2 decoding the 0x65D frame and controlling the LEDs as well as responding to
// the 0x651 remote frame.
//
// If we set FilterMask to 0x01C0 0000 and Filter ID to 0x0000 0000, then the 0x65D frame is rejected,
// the 0x651 frame is accepted. Now you will then see Node 2 only responding to the 0x651 remote frame.

void CAN_Filter_Config(void){
	CAN_FilterTypeDef fltrConfig;

	fltrConfig.FilterBank = 0;
    fltrConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	fltrConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	fltrConfig.FilterIdHigh = 0x0000;
    fltrConfig.FilterIdLow = 0x0000;

	fltrConfig.FilterMaskIdHigh = 0x0000;      // accept all frames
	// fltrConfig.FilterMaskIdHigh = 0x01C0;   // filter out the 0x65D frame

	fltrConfig.FilterMaskIdLow = 0x00000;
	fltrConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	// slave CAN2 not used
	// fltrConfig.SlaveStartFilterBank = x
	fltrConfig.FilterActivation = ENABLE;

	if (HAL_OK != HAL_CAN_ConfigFilter(&hcan1, &fltrConfig)){
		Error_Handler();
		}
	}


void LedControl(uint8_t ctrl){
	switch (ctrl) {
	case 0 :
	default :
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // LED # 0
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
		break;
	case 1 :
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); // LED # 1
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
		break;
	case 2 :
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // LED # 2
	    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
		break;
	case 3 :
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET); // LED # 3
		break;
	}
}

// called from timer6 period elapsed callback (every second)
// sends a data frame containing LED# (0 .. 3) in the data payload (1 byte)

uint8_t LedNumber = 0;

void SendLEDCtrlMessage(uint32_t stdID){
	CAN_TxHeaderTypeDef txhdr;

	uint8_t msg; // 1byte enough to encode LED#
	txhdr.DLC = 1; // 1byte
	txhdr.StdId = stdID; // 11 bit value
	txhdr.IDE = CAN_ID_STD; // standard frame, not extended
	txhdr.RTR = CAN_RTR_DATA; // data frame, not remote(request) frame
	// ignore transmit global time, only used for time triggered messages

	msg = LedNumber;
	if (++LedNumber >= 4) {
		LedNumber = 0;
		}

	// this selects an empty mailbox, sets TXRQ bit and returns the mailbox used
	uint32_t TxMailBox;
	if (HAL_OK != HAL_CAN_AddTxMessage(&hcan1, &txhdr, &msg, &TxMailBox)) {
		Error_Handler();
		}
	// tx callback will be triggered on complete
	}


void SendRemoteRequest(uint32_t stdID){
	CAN_TxHeaderTypeDef txhdr;

	uint8_t msg; // no meaning for remote request
	txhdr.DLC = 2; // want two byte response
	txhdr.StdId = stdID; // Remote/request message identifier
	txhdr.IDE = CAN_ID_STD; // standard frame, not extended
	txhdr.RTR = CAN_RTR_REMOTE; // remote/request frame

	// this selects an empty mailbox, sets TXRQ bit and returns the mailbox used
	uint32_t TxMailBox;
	if (HAL_OK != HAL_CAN_AddTxMessage(&hcan1, &txhdr, &msg, &TxMailBox)) {
		Error_Handler();
		}
	// tx callback will be triggered on complete
	}


uint16_t ResponseCounter = 0;

void SendRemoteResponse(uint32_t stdID){
	CAN_TxHeaderTypeDef txhdr;

	uint8_t response[2]; // 2byte response required for RTR 0x651
	txhdr.DLC = sizeof(response);
	txhdr.StdId = stdID; // reply to RTR  has to have the same message identifier
	txhdr.IDE = CAN_ID_STD; // standard frame, not extended
	txhdr.RTR = CAN_RTR_DATA; // data frame, not remote(request) frame

	// response is just a counter incremented each time we get the RTR 0x651
	response[0] = (uint8_t)((ResponseCounter>>8) & 0xff);
	response[1] = (uint8_t)(ResponseCounter & 0xff);
	ResponseCounter++;

	// this selects an empty mailbox, sets TXRQ bit and returns the mailbox used
	uint32_t TxMailBox;
	if (HAL_OK != HAL_CAN_AddTxMessage(&hcan1, &txhdr, response, &TxMailBox)) {
		Error_Handler();
		}
	// tx callback will be triggered on complete
	}


// callback is in isr context, don't want large structs as local variables
CAN_RxHeaderTypeDef rxhdr;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	// maximum data payload in Standard frame is 8 bytes
	uint8_t rxmsg[8] = {0};
	// get a can frame data payload from rxfifo 0 into the receive data buffer
	if (HAL_OK != HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxhdr, rxmsg)) {
		Error_Handler();
		}
	if (rxhdr.StdId == 0x65D && rxhdr.RTR == CAN_RTR_DATA){
		// N2 : received data frame to control the LEDs
		LedControl(rxmsg[0]);
		printMsg("N2: received 0x65D DATA frame LED[%x]\r\n", rxmsg[0]);
		}
	else
	if (rxhdr.StdId == 0x651 && rxhdr.RTR == CAN_RTR_REMOTE){
		// N2 : received remote request 0x651
		SendRemoteResponse(rxhdr.StdId);
		printMsg("N2: received RTR frame 0x651\r\n");
		return;
		}
	else
	if (rxhdr.StdId == 0x651 && rxhdr.RTR == CAN_RTR_DATA){
		// N1 : received reply to remote request 0x651
		printMsg("N1: dataframe response received for 0x651 RTR [%04X]\r\n", (((uint16_t)rxmsg[0])<<8) | (uint16_t)rxmsg[1]);
		}
	}


// CAN transmit scheduler decides which mailbbox the tx message is placed in.
// so we need to monitor all 3 callbacks to see where our message was
// transmitted from
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
	printMsg("Message transmitted from tx mailbox 0\r\n");
	}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
	printMsg("Message transmitted from tx mailbox 1\r\n");
	}


void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
	printMsg("Message transmitted from tx mailbox 2\r\n");
	}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
	printMsg("CAN1 Error!\r\n");
	Error_Handler();
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


static void TIM6_Init(void){
    htim6.Instance = TIM6; // base address of peripheral
    // configuring for basic timer operation
    // TIM6 in Memory and Bus architecture in ref manual is on APB1
    // TIM_CLK = APB1 Timer Clock (see cubemx clock tree) = 50MHz
    // TIM_CNT_CLK = TIM_CLK /(1 + Prescaler)
    // 16bit (0-65535) value written into TIM6_PSC
    htim6.Init.Prescaler = 5000-1;
    // for basic timer, .CounterMode has only one relevant option count up
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    // 16bit (0-65535) value written into AutoReloadRegister TIM6_ARR at next update event
    // if 0, timer will not start even if enabled
    // for 1Sec period (1Hz), we need to count up to (50MHz/5000)/1Hz = 10000,  < 65535 => OK
    htim6.Init.Period = 10000-1;
    // this will call TIM_Base_MspInit where we need to do the low level initialization
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)  {
        Error_Handler();
        }
    }

static void GPIO_Init(void){
    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    // Configure GPIO pin Output Level for board builtin LED (which is active LOW) on PA1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);

    GPIO_InitTypeDef gpioConfig = {0};
    // Configure GPIO pin : PA0  board built in button K1 as external interrupt
    // Button connects PA0 to VCC
     gpioConfig.Pin = GPIO_PIN_0;
     gpioConfig.Mode = GPIO_MODE_IT_RISING;
     gpioConfig.Pull = GPIO_PULLDOWN;
     HAL_GPIO_Init(GPIOA, &gpioConfig);

     // EXTI interrupt init
     HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
     HAL_NVIC_EnableIRQ(EXTI0_IRQn);

     // Configure GPIO pin : PA1 board led
     gpioConfig.Pin = GPIO_PIN_1;
     gpioConfig.Mode = GPIO_MODE_OUTPUT_PP;
     gpioConfig.Pull = GPIO_NOPULL;
     gpioConfig.Speed = GPIO_SPEED_FREQ_LOW;
     HAL_GPIO_Init(GPIOA, &gpioConfig);

     // Configure GPIO pin : PB10, PB11, PB12, PE15 as outputs ( LEDs )
     gpioConfig.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 ;
     gpioConfig.Mode = GPIO_MODE_OUTPUT_PP;
     gpioConfig.Pull = GPIO_NOPULL;
     gpioConfig.Speed = GPIO_SPEED_FREQ_LOW;
     HAL_GPIO_Init(GPIOB, &gpioConfig);

     gpioConfig.Pin = GPIO_PIN_15;
     gpioConfig.Mode = GPIO_MODE_OUTPUT_PP;
     gpioConfig.Pull = GPIO_NOPULL;
     gpioConfig.Speed = GPIO_SPEED_FREQ_LOW;
     HAL_GPIO_Init(GPIOE, &gpioConfig);
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


