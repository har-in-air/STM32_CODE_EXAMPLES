// DevEBox STM32F407VGT6 development board
// Demo CAN1 transmit @ 500kbps with bxCAN internal loopback mode enabled.
// This internally routes the tx output back to rx. CAN1_RX pin signal
// is ignored. You can monitor the transmission at CAN1_TX pin with a logic analyzer.
// Since the CAN controller cannot ACK its own transmissions, the transmission ends with a NAK.
// This is an error and should result in retransmission if Autoretransmit is enabled.
// But in internal loopback mode, even if auto-retransmit is enabled, no retransmit
// on NAK. So you will see only one packet transmitted.

#include "main.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"

UART_HandleTypeDef huart1;
CAN_HandleTypeDef  hcan1;

void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART1_Init(void);
static void CAN1_Init(void);

void CAN1_Tx(void);
void CAN1_RX(void);
void CAN_Filter_Config(void);

void Error_Handler(void);
void printMsg(char* format, ...);


int main(void){
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    UART1_Init();

    printMsg("CAN1 internal Loopback test\r\n");

    // bxCan on reset is in sleep mode
    // move to init mode by resetting bxCAN sleep bit sleep.inrq.ack
    // and then configure parameters
    CAN1_Init();

    // configure acceptance filtering before moving to normal mode
    CAN_Filter_Config();

    // move bxCAN from init to normal mode
    if (HAL_OK != HAL_CAN_Start(&hcan1)) {
    	Error_Handler();
    	}

    // testing loopback mode without a transceiver
    // PA12 (CAN1_TX) N.C.
    // PA11 (CAN1_RX) pulled up to 3V3 with 3K3 resistor

    // testing loopback mode with transceiver
    // connect PA12 CAN1_TX to module TX, PA11 CAN1_RX to module RX
    // ensure 120 ohm termination on transceiver CANH-CANL lines.

    // check CAN1_TX line with logic analyzer @20Ms/s and CAN analyzer plugin enabled
    // confirm only one packet transmission, and it ends with NAK.
    // confirm bitrate 500Kbps by measuring SOF bit pulse width, it should be 2uS

    CAN1_Tx();

    CAN1_RX();
    while (1) {
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
	hcan1.Init.Mode = CAN_MODE_LOOPBACK;
	// from website for 25MHz clk and 500kbps
	hcan1.Init.Prescaler = 5;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;

    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE; // overrun the oldest
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE ; // transmission priority depends on message identifier, not chronological order
    hcan1.Init.TimeTriggeredMode = DISABLE;

    if (HAL_OK != HAL_CAN_Init(&hcan1)){
    	Error_Handler();
    	}
	}


// 3 tx mailboxes are available. The transmission scheduler decides the transmission order
// based on message identifier priority or chronological order depending on configuration.
// After loading the packet with message identifier and data payload, set TXRQ bit and
// the mailbox goes to pending state.
// While in pending state, it can be aborted.
// If in pending state and the highest priority mailbox it goes to scheduled for transmission state
// Can still be aborted.
// If scheduled and  the CAN bus is idle (11 recessive states), transmit attempt.
// Successful transmission of mailbox indicated by RQCP and TXOK bits in CAN_TSR, and
// mailbox is now empty, available for another packet.
// Failure indicated by ALST bit if arbitration lost, and/or TERR bit if transmission error.
// If automatic retransmission is enabled, mailbox goes back to scheduled state.
// In meantime, if another mailbox is created with higher priority, current mailbox can be
// pushed back to pending state.

void CAN1_Tx(void){
	CAN_TxHeaderTypeDef txhdr;

	uint8_t msg[5] = {'h','e','l', 'l', 'o'};
	txhdr.DLC = sizeof(msg); // sending 'hello'
	txhdr.StdId = 1629; // arbitrary 11 bit value ( 0x65D)
	txhdr.IDE = CAN_ID_STD; // standard frame, not extended
	txhdr.RTR = CAN_RTR_DATA; // data frame, not remote(request) frame
	// ignore transmit global time, only used for time triggered messages

	// this selects an empty mailbox, sets TXRQ bit and returns the mailbox used
	uint32_t TxMailBox;
	if (HAL_OK != HAL_CAN_AddTxMessage(&hcan1, &txhdr, msg, &TxMailBox)) {
		Error_Handler();
		}

	// poll for tx complete
	while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailBox));
	printMsg("Message Transmitted\r\n");
	}


void CAN_Filter_Config(void){
	CAN_FilterTypeDef fltrConfig;
	fltrConfig.FilterBank = 0;
    fltrConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	fltrConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	fltrConfig.FilterIdHigh = 0x0000;
    fltrConfig.FilterIdLow = 0x0000;
	fltrConfig.FilterMaskIdHigh = 0x0000;
	fltrConfig.FilterMaskIdLow = 0x00000;
	fltrConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	// slave CAN2 not used
	// fltrConfig.SlaveStartFilterBank =
	fltrConfig.FilterActivation = ENABLE;

	if (HAL_OK != HAL_CAN_ConfigFilter(&hcan1, &fltrConfig)){
		Error_Handler();
		}
	}

// set up acceptance filtering, direct accepted messages into a specified receive fifo (0 or 1)
// the receive fifo can hold up to 3 messages.
// check if any messages in the fifo, then retrieve the message payload into
// destination buffer
void CAN1_RX(void){
	CAN_RxHeaderTypeDef rxhdr;
	uint8_t rxmsg[5] = {0};
	// wait until rXfifo has a message, returns value from 0 (empty) to 3 (full)
	while (!HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0));

	// get a can frame data payload from rxfifo into the destination data buffer
	if (HAL_OK != HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxhdr, rxmsg)) {
		Error_Handler();
		}
	printMsg("Message received : %s\r\n", rxmsg);
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
     // PA0 button press normal input
     gpioConfig.Mode = GPIO_MODE_INPUT;
     gpioConfig.Pull = GPIO_PULLDOWN;
     HAL_GPIO_Init(GPIOA, &gpioConfig);

     // Configure GPIO pin : PA1 board led
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
	int counter = 0;
	while (1) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
		while (counter < 0x3FFFF) {counter++;}
		counter = 0;
		}
    }


