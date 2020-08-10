/* USER CODE BEGIN Header */
// Demonstrating  bootloader project suitable for bootloader + application
// Flash Sector 0 and 1 allocated to bootloader (starting at 0x8000000, total 32kBytes)
// User application is loaded at start of Flash Sector 2 (0x08008000)
// 1. In user application project,
// a. Edit  STM32F407VGTX_FLASH.ld, and change FLASH ORIGIN to 0x8008000
// and reduce LENGTH by 32K to 992K
//
// MEMORY
// {
//   CCMRAM    (xrw)    : ORIGIN = 0x10000000,   LENGTH = 64K
//   RAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 128K
//   FLASH    (rx)    : ORIGIN = 0x8008000,   LENGTH = 992K
// }
//
// b. Edit  system_stm32f4xx.c, set the vector table offset to 0x8000
//
// #define VECT_TAB_OFFSET  0x8000
//
// 2. Use STM32CubeProgrammer with STLink to erase all of flash
// 3. Use STM32CubeIDE to flash the user application project
// 4. Verify with STM32CubeProgrammer that addresses below 0x8008000 are still erased
// 5. Use STM32CubeIDE to flash this bootloader project
//
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/


/* USER CODE BEGIN 0 */

// version 1.0 encoded as 0x10

#define BLDR_VERSION 0x10

// user application starts at Sector 2, sector 0 and 1 are 16kBytes each
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000

#define DBG_UART 	huart1
#define BLDR_UART 	huart2

#define BLDR_FAILURE 0
#define BLDR_SUCCESS 1

#define BLDR_ACK 	0xA5
#define BLDR_NACK	0x7F


#define BL_GET_VER 				0x51
#define BL_GET_HELP				0x52
#define BL_GET_CID				0x53
#define BL_GET_RDP_STATUS		0x54
#define BL_GOTO_ADDR			0x55
#define BL_FLASH_ERASE			0x56
#define BL_MEM_WRITE			0x57
#define BL_EN_RW_PROTECT		0x58
#define BL_MEM_READ				0x59
#define BL_READ_SECTOR_PSTATUS	0x5A
#define BL_OTP_READ			 	0x5B
#define BL_DISABLE_RWPROTECT	0x5C


#define ADDR_VALID 			0x00
#define ADDR_INVALID 		0x01

#define INVALID_SECTOR 		0x04

// STM32F407VGT6
// note 64K of CCM RAM is only for data, so we can't jump there
#define SRAM1_SIZE          112*1024
#define SRAM1_END           (SRAM1_BASE + SRAM1_SIZE)
#define SRAM2_SIZE          16*1024
#define SRAM2_END           (SRAM2_BASE + SRAM2_SIZE)
#define FLASH_SIZE          1024*1024
#define BKPSRAM_SIZE        4*1024
#define BKPSRAM_END         (BKPSRAM_BASE + BKPSRAM_SIZE)

void bldr_CmdGetVersion(uint8_t* pBuf);
void bldr_CmdGetHelp(uint8_t* pBuf);
void bldr_CmdGetCID(uint8_t* pBuf);
void bldr_CmdGetRDPStatus(uint8_t* pBuf);
void bldr_CmdGotoAddress(uint8_t* pBuf);
void bldr_CmdFlashErase(uint8_t* pBuf);
void bldr_CmdMemWrite(uint8_t* pBuf);
void bldr_CmdMemRead(uint8_t* pBuf);
void bldr_CmdReadSectorPStatus(uint8_t* pBuf);
void bldr_CmdEnRWProtect(uint8_t* pBuf);
void bldr_CmdReadOTP(uint8_t* pBuf);
void bldr_CmdDisableRWProtect(uint8_t* pBuf);

void bldr_SendACK(uint8_t cmd, uint8_t followLength);
void bldr_SendNACK(void);
int  bldr_CheckCRC(uint8_t* pBuf, uint32_t numBytes, uint32_t crcHost);
void bldr_UartWriteData(uint8_t* pBuf, int numBytes);
uint8_t bldr_VerifyAddress(uint32_t go_address);
uint8_t bldr_FlashEraseSectors(uint8_t sectorIndex , uint8_t numSectors);
uint8_t bldr_MemWrite(uint8_t *pBuffer, uint32_t address, uint32_t length);
uint16_t bldr_ReadOBRWProtectionStatus(void);
uint8_t bldr_ConfigureSectorRWProtection(uint8_t sectorDetails, uint8_t protectionMode, uint8_t disable);


void bldr_HandleCommand(void);
void bldr_JumpUserApp(uint32_t address);

void printMsg(char* format, ...) {
	char sz[100];
	va_list args;
	va_start(args, format);
	vsprintf(sz, format, args);
	HAL_UART_Transmit(&DBG_UART, (uint8_t *)sz, strlen(sz), HAL_MAX_DELAY);
	va_end(args);
	}

uint8_t BldrRxBuffer[200];

uint8_t BldrSupportedCommands[] = {
BL_GET_VER,
BL_GET_HELP,
BL_GET_CID,
BL_GET_RDP_STATUS,
BL_GOTO_ADDR,
BL_FLASH_ERASE,
BL_MEM_WRITE,
BL_EN_RW_PROTECT,
BL_MEM_READ,
BL_READ_SECTOR_PSTATUS,
BL_OTP_READ,
BL_DISABLE_RWPROTECT
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
	  printMsg("Bootloader button pressed\r\n");
	  bldr_HandleCommand();
  	  }
  else {
	  printMsg("Bootloader button not pressed\r\n");

	  // user app is loaded at start of sector 2
	  bldr_JumpUserApp(FLASH_SECTOR2_BASE_ADDRESS);
  	  }
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  printMsg("hello from uart1\r\n");
	  HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void bldr_HandleCommand(void){
	uint8_t rcvLength = 0;
	while (1) {
		memset(BldrRxBuffer, 0, 200);
		// read and decode the host commands
		HAL_UART_Receive(&BLDR_UART, BldrRxBuffer, 1, HAL_MAX_DELAY);
		rcvLength = BldrRxBuffer[0];
		HAL_UART_Receive(&BLDR_UART, &BldrRxBuffer[1], rcvLength, HAL_MAX_DELAY);
		switch (BldrRxBuffer[1]){
		case BL_GET_VER:
			bldr_CmdGetVersion(BldrRxBuffer);
			break;
		case BL_GET_HELP:
			bldr_CmdGetHelp(BldrRxBuffer);
			break;
		case BL_GET_CID:
			bldr_CmdGetCID(BldrRxBuffer);
			break;
		case BL_GET_RDP_STATUS:
			bldr_CmdGetRDPStatus(BldrRxBuffer);
			break;
		case BL_GOTO_ADDR:
			bldr_CmdGotoAddress(BldrRxBuffer);
			break;
		case BL_FLASH_ERASE:
			bldr_CmdFlashErase(BldrRxBuffer);
			break;
		case BL_MEM_WRITE:
			bldr_CmdMemWrite(BldrRxBuffer);
			break;
		case BL_EN_RW_PROTECT:
			bldr_CmdEnRWProtect(BldrRxBuffer);
			break;
		case BL_MEM_READ:
			bldr_CmdMemRead(BldrRxBuffer);
			break;
		case BL_READ_SECTOR_PSTATUS:
			bldr_CmdReadSectorPStatus(BldrRxBuffer);
			break;
		case BL_OTP_READ:
			bldr_CmdReadOTP(BldrRxBuffer);
			break;
		case BL_DISABLE_RWPROTECT:
			bldr_CmdDisableRWProtect(BldrRxBuffer);
			break;
		default :
			printMsg("BLDR : Invalid command received : RcvLen %d, CMD %#x\r\n", BldrRxBuffer[0],BldrRxBuffer[1]);
			break;
			}
		}
	}



void bldr_CmdFlashErase(uint8_t* pBuf){
	uint8_t eraseStatus = 0x00;
	printMsg("BLDR : FlashErase\r\n");
	uint32_t cmdLength = pBuf[0]+1;
	uint32_t crcHost = *((uint32_t*) (pBuf + cmdLength - 4));
	if (BLDR_SUCCESS == bldr_CheckCRC(&pBuf[0], cmdLength-4, crcHost)){
		printMsg("BLDR : CRC OK\r\n");
		bldr_SendACK(pBuf[0], 1);
        printMsg("BLDR : sectorIndex : %d  numSectors: %d\n", pBuf[2], pBuf[3]);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0); // turn on onboard LED
        eraseStatus = bldr_FlashEraseSectors(pBuf[2] , pBuf[3]);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1); // turn off onboard LED
        printMsg("BLDR: flash erase status: %#x\n", eraseStatus);
        bldr_UartWriteData(&eraseStatus,1);
		}
	else {
		printMsg("BLDR : CRC error\r\n");
		bldr_SendNACK();
		}
	}


void bldr_CmdMemWrite(uint8_t* pBuf){
	uint8_t writeStatus = 0x00;
	uint8_t payloadLength = pBuf[6];
	uint32_t address = *((uint32_t *) ( &pBuf[2]) );

	printMsg("BLDR : MemWrite\r\n");
	uint32_t cmdLength = pBuf[0]+1;
	uint32_t crcHost = *((uint32_t*) (pBuf + cmdLength - 4));
	if (BLDR_SUCCESS == bldr_CheckCRC(&pBuf[0], cmdLength-4, crcHost)){
		printMsg("BLDR : CRC OK\r\n");
		bldr_SendACK(pBuf[0], 1);
		printMsg("BLDR : address %#x\r\n", address);
		if (bldr_VerifyAddress(address) == ADDR_VALID){
			printMsg("BLDR : valid address\r\n");
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0); // turn on LED
			writeStatus = bldr_MemWrite(&pBuf[7], address, payloadLength);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1); // turn off LED
			}
		else {
			printMsg("BLDR : invalid address\r\n");
			writeStatus = ADDR_INVALID;
			}
		bldr_UartWriteData(&writeStatus, 1);
		}
	else {
		printMsg("BLDR : CRC error\r\n");
		bldr_SendNACK();
		}
	}


void bldr_CmdMemRead(uint8_t* pBuf){
	printMsg("BLDR : MemRead\r\n");
	uint32_t cmdLength = pBuf[0]+1;
	uint32_t crcHost = *((uint32_t*) (pBuf + cmdLength - 4));
	if (BLDR_SUCCESS == bldr_CheckCRC(&pBuf[0], cmdLength-4, crcHost)){
		printMsg("BLDR : CRC OK\r\n");
		bldr_SendACK(pBuf[0], 1);

		}
	else {
		printMsg("BLDR : CRC error\r\n");
		bldr_SendNACK();
		}
	}


void bldr_CmdReadSectorPStatus(uint8_t* pBuf){
	uint16_t status;
	printMsg("BLDR : ReadSectorPStatus\r\n");
	uint32_t cmdLength = pBuf[0]+1;
	uint32_t crcHost = *((uint32_t*) (pBuf + cmdLength - 4));
	if (BLDR_SUCCESS == bldr_CheckCRC(&pBuf[0], cmdLength-4, crcHost)){
		printMsg("BLDR : CRC OK\r\n");
		bldr_SendACK(pBuf[0], 2);
		status = bldr_ReadOBRWProtectionStatus();
		printMsg("BLDR : nWRP status %#x\r\n", status);
		bldr_UartWriteData((uint8_t*)&status, 2);
		}
	else {
		printMsg("BLDR : CRC error\r\n");
		bldr_SendNACK();
		}
	}


void bldr_CmdGetVersion(uint8_t* pBuf){
	uint8_t version;
	printMsg("BLDR : GetVersion\r\n");
	uint32_t cmdLength = pBuf[0]+1;
	uint32_t crcHost = *((uint32_t*) (pBuf + cmdLength - 4));
	if (BLDR_SUCCESS == bldr_CheckCRC(&pBuf[0], cmdLength-4, crcHost)){
		printMsg("BLDR : CRC OK\r\n");
		bldr_SendACK(pBuf[0], 1);
		version = (uint8_t) BLDR_VERSION;
		printMsg("BLDR : Version %#x\r\n", version);
		bldr_UartWriteData(&version, 1);
		}
	else {
		printMsg("BLDR : CRC error\r\n");
		bldr_SendNACK();
		}
	}


void bldr_CmdGetHelp(uint8_t* pBuf){
	printMsg("BLDR : GetHelp\r\n");
	uint32_t cmdLength = pBuf[0]+1;
	uint32_t crcHost = *((uint32_t*) (pBuf + cmdLength - 4));
	if (BLDR_SUCCESS == bldr_CheckCRC(&pBuf[0], cmdLength-4, crcHost)){
		printMsg("BLDR : CRC OK\r\n");
		bldr_SendACK(pBuf[0], sizeof(BldrSupportedCommands));
		bldr_UartWriteData(BldrSupportedCommands, sizeof(BldrSupportedCommands));
		}
	else {
		printMsg("BLDR : CRC error\r\n");
		bldr_SendNACK();
		}
	}


void bldr_CmdGetCID(uint8_t* pBuf){
	printMsg("BLDR : GetCID\r\n");
	uint32_t cmdLength = pBuf[0]+1;
	uint32_t crcHost = *((uint32_t*) (pBuf + cmdLength - 4));
	if (BLDR_SUCCESS == bldr_CheckCRC(&pBuf[0], cmdLength-4, crcHost)){
		printMsg("BLDR : CRC OK\r\n");
		bldr_SendACK(pBuf[0], 2);
		// die revision and part #, bits 11:0 are part#, 0x413 for STM32F407VGT6
		uint16_t cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
		printMsg("BLDR : CID %#x\r\n", cid);
		bldr_UartWriteData((uint8_t*)&cid, 2);
		}
	else {
		printMsg("BLDR : CRC error\r\n");
		bldr_SendNACK();
		}
	}

// 16 option bytes
// 0x1FFFC000, bits 15:8 RDP field = read protection for flash memory
// 0xAA : level 0, no protection
// 0xCC : level 2, chip protection. Boot from RAM/system ROM bootloader disabled.
// 			JTAG/SWD and all debug features disabled, user option bytes cannot be changed.
//          Application booting from flash can access flash (read/write/erase)
//          *** This is irreversible ****
// others : Level 1, No access (read, erase or program) of flash memory, while
// debug feature is connected, or while booting from RAM or system ROM bootloader.
// when booting from flash memory, application code can access flash
// If Level 1 is active and level 0 is requested, mass chip erase is performed.

void bldr_CmdGetRDPStatus(uint8_t* pBuf){
	printMsg("BLDR : GetRDPStatus\r\n");
	uint32_t cmdLength = pBuf[0]+1;
	uint32_t crcHost = *((uint32_t*) (pBuf + cmdLength - 4));
	if (BLDR_SUCCESS == bldr_CheckCRC(&pBuf[0], cmdLength-4, crcHost)){
		printMsg("BLDR : CRC OK\r\n");
		bldr_SendACK(pBuf[0], 1);
		FLASH_OBProgramInitTypeDef obHandle;
		HAL_FLASHEx_OBGetConfig(&obHandle);
		uint8_t rdp = (uint8_t)obHandle.RDPLevel;
		//volatile uint32_t* pOptBytes = (volatile uint32_t*)0x1FFFC000;
		//uint8_t rdp = (uint8_t) ((*pOptBytes) >> 8);
		bldr_UartWriteData(&rdp, 1);
		}
	else {
		printMsg("BLDR : CRC error\r\n");
		bldr_SendNACK();
		}
	}


// to test jumping to a valid address, browse the memory starting at 0x08008000
// assuming we've already programmed the user application there,
// the first word is the MSP, second is the application reset handler address
// e.g. 0x080088BD. Enter this at the host application goto address prompt

void bldr_CmdGotoAddress(uint8_t* pBuf){
    uint32_t address = 0;
    uint8_t flag;

	printMsg("BLDR : GotoAddress\r\n");
	uint32_t cmdLength = pBuf[0]+1;
	uint32_t crcHost = *((uint32_t*) (pBuf + cmdLength - 4));
	if (BLDR_SUCCESS == bldr_CheckCRC(&pBuf[0], cmdLength-4, crcHost)){
		printMsg("BLDR : CRC OK\r\n");
		bldr_SendACK(pBuf[0], 1);
        address = *((uint32_t *)&pBuf[2] );
        printMsg("BLDR : Goto address %#x\r\n", address);
        if( bldr_VerifyAddress(address) == ADDR_VALID ){
             // tell host that address is fine
        	 flag = ADDR_VALID;
             bldr_UartWriteData(&flag, 1);
#if 0
             // host must ensure that valid code is present at goto address
             // ensure address is a Thumb instruction address
             address |= 1;
             void (*jump)(void) = (void *)address;
             printMsg("BLDR : jumping to go address\r\n");
             jump();
#else
             bldr_JumpUserApp(address);
#endif
 			}
        else{
             printMsg("BLDR : Goto address invalid\r\n");
             flag = ADDR_INVALID;
             bldr_UartWriteData(&flag, 1);
 			}
		}
	else {
		printMsg("BLDR : CRC error\r\n");
		bldr_SendNACK();
		}
	}


void bldr_CmdEnRWProtect(uint8_t* pBuf){
	uint8_t status = 0;
	printMsg("BLDR : EndRWProtect\r\n");
	uint32_t cmdLength = pBuf[0]+1;
	uint32_t crcHost = *((uint32_t*) (pBuf + cmdLength - 4));
	if (BLDR_SUCCESS == bldr_CheckCRC(&pBuf[0], cmdLength-4, crcHost)){
		printMsg("BLDR : CRC OK\r\n");
		bldr_SendACK(pBuf[0], 1);
		status = bldr_ConfigureSectorRWProtection(pBuf[2], pBuf[3], 0);
		bldr_UartWriteData(&status, 1);
		}
	else {
		printMsg("BLDR : CRC error\r\n");
		bldr_SendNACK();
		}
	}


void bldr_CmdReadOTP(uint8_t* pBuf){
	printMsg("BLDR : ReadOTP\r\n");
	uint32_t cmdLength = pBuf[0]+1;
	uint32_t crcHost = *((uint32_t*) (pBuf + cmdLength - 4));
	if (BLDR_SUCCESS == bldr_CheckCRC(&pBuf[0], cmdLength-4, crcHost)){
		printMsg("BLDR : CRC OK\r\n");
		bldr_SendACK(pBuf[0], 1);

		}
	else {
		printMsg("BLDR : CRC error\r\n");
		bldr_SendNACK();
		}
	}

void bldr_CmdDisableRWProtect(uint8_t* pBuf){
	uint8_t status = 0;
	printMsg("BLDR : DisableRWProtect\r\n");
	uint32_t cmdLength = pBuf[0]+1;
	uint32_t crcHost = *((uint32_t*) (pBuf + cmdLength - 4));
	if (BLDR_SUCCESS == bldr_CheckCRC(&pBuf[0], cmdLength-4, crcHost)){
		printMsg("BLDR : CRC OK\r\n");
		bldr_SendACK(pBuf[0], 1);
		status = bldr_ConfigureSectorRWProtection(0,0,1);
		printMsg("BLDR : config RW Protect status %#x\r\n", status);
		bldr_UartWriteData(&status, 1);
		}
	else {
		printMsg("BLDR : CRC error\r\n");
		bldr_SendNACK();
		}
	}


void bldr_JumpUserApp(uint32_t address){
	// pointer to hold the address of the reset handler for the user application
	void (*appResetHandler)(void);
	printMsg("BLDR : jump to user application\r\n");

	// first word at address has the user application programmed MSP
	uint32_t msp_val = *((volatile uint32_t*) address);
	printMsg("BLDR : MSP Sector 2 value = %#x\r\n", msp_val);
	// CMSIS macro
	__set_MSP(msp_val);

	// second word has the user application Reset_Handler address
	uint32_t resetHandlerAddress = *(volatile uint32_t*)(address+4);
	appResetHandler = (void*)resetHandlerAddress;
	printMsg("User application reset handler address = %#x\r\n", resetHandlerAddress);

	// jump to user application reset handler
	appResetHandler();
	}


uint8_t bldr_VerifyAddress(uint32_t go_address){
	// system memory ? yes
	// sram1 memory ?  yes
	// sram2 memory ? yes
	// backup sram memory ? yes
	// peripheral memory ? possible, but no
	// external memory ? yes.

	if ( go_address >= SRAM1_BASE && go_address <= SRAM1_END){
		return ADDR_VALID;
		}
	else if ( go_address >= SRAM2_BASE && go_address <= SRAM2_END){
		return ADDR_VALID;
		}
	else if ( go_address >= FLASH_BASE && go_address <= FLASH_END)	{
		return ADDR_VALID;
		}
	else if ( go_address >= BKPSRAM_BASE && go_address <= BKPSRAM_END)	{
		return ADDR_VALID;
		}
	else
		return ADDR_INVALID;
	}



uint8_t bldr_FlashEraseSectors(uint8_t sectorIndex , uint8_t numSectors){
    // STM32F407 has 8 flash sectors
	// numSectors has to be in the range of 0 to 7
	// if sectorIndex = 0xff , do mass erase
	FLASH_EraseInitTypeDef hFlashErase;
	uint32_t sectorError;
	HAL_StatusTypeDef status;

	if( numSectors > 8 ){
		return INVALID_SECTOR;
		}

	if( (sectorIndex == 0xff ) || (sectorIndex <= 7) )	{
		if(sectorIndex == (uint8_t) 0xff)		{
			hFlashErase.TypeErase = FLASH_TYPEERASE_MASSERASE;
			}
		else{
		    // limit the number of sectors to be erased
			uint8_t remainingSectors = 8 - sectorIndex;
            if( numSectors > remainingSectors) {
            	numSectors = remainingSectors;
            	}
            hFlashErase.TypeErase = FLASH_TYPEERASE_SECTORS;
            hFlashErase.Sector = sectorIndex; // start sector
            hFlashErase.NbSectors = numSectors;
			}
		hFlashErase.Banks = FLASH_BANK_1;
		HAL_FLASH_Unlock();
		hFlashErase.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // 2.7v- 3.6v
		status = (uint8_t) HAL_FLASHEx_Erase(&hFlashErase, &sectorError);
		HAL_FLASH_Lock();
		return status;
		}
	return INVALID_SECTOR;
	}


// This function writes the contents of pBuf to  "address"
uint8_t bldr_MemWrite(uint8_t *pBuf, uint32_t address, uint32_t length){
    uint8_t status;
    if (address >= FLASH_BASE && address <= FLASH_END) {
    	HAL_FLASH_Unlock();
    	for(uint32_t inx = 0 ; inx < length ; inx++)    {
    		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address+inx, pBuf[inx] );
    		}
    	HAL_FLASH_Lock();
    	}
    else {
    	volatile uint8_t* ramAddress = (uint8_t*) address;
    	while (length--){
    		*ramAddress++ = *pBuf++;
    		}
    	status = HAL_OK;
    	}
    return status;
	}




// sector details : sector numbers are bit position, 1 = protection, 0 = no protection
// protection mode : 1 = write protection (note : read protection is not available on STM32F407)

uint8_t bldr_ConfigureSectorRWProtection(uint8_t sectorDetails, uint8_t protectionMode, uint8_t disable){
	if ( disable )	{
		HAL_FLASH_OB_Unlock();
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		// clear the write protection for all sectors
		FLASH->OPTCR |= (0xFF << 16);
		//Set the option start bit (OPTSTRT)
		FLASH->OPTCR |= ( 1 << 1);
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		HAL_FLASH_OB_Lock();
		return 0;
		}

	if(protectionMode > 0)  {
	   // write protect the sectors encoded in sectorDetails
		HAL_FLASH_OB_Unlock();
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		// write protection is active low, so invert the sectorDetails byte
		FLASH->OPTCR &= ~ (((uint32_t)sectorDetails) << 16);
		//Set the option start bit (OPTSTRT)
		FLASH->OPTCR |= ( 1 << 1);
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		HAL_FLASH_OB_Lock();
		}
	return 0;
	}


uint16_t bldr_ReadOBRWProtectionStatus(void){
	FLASH_OBProgramInitTypeDef OBInit;
	HAL_FLASH_OB_Unlock();
	HAL_FLASHEx_OBGetConfig(&OBInit);
	HAL_FLASH_Lock();
	return (uint16_t)OBInit.WRPSector;
	}

void bldr_UartWriteData(uint8_t* pBuf, int numBytes){
	HAL_UART_Transmit(&BLDR_UART, pBuf, numBytes, HAL_MAX_DELAY);
	}


int bldr_CheckCRC(uint8_t* pBuf, uint32_t numBytes, uint32_t crcHost){
	uint32_t uwCRCValue = 0xFFFFFFFF;
	for (uint32_t inx = 0; inx < numBytes; inx++) {
		uint32_t udata = (uint32_t) pBuf[inx];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &udata, 1);
		}
	__HAL_CRC_DR_RESET(&hcrc);
	return (uwCRCValue == crcHost) ?  BLDR_SUCCESS : BLDR_FAILURE;
	}

void bldr_SendACK(uint8_t cmd, uint8_t followLength) {
	uint8_t ack[2];
	ack[0] = BLDR_ACK;
	ack[1] = followLength;
	HAL_UART_Transmit(&BLDR_UART, ack, 2, HAL_MAX_DELAY);
	}

void bldr_SendNACK(void) {
	uint8_t nack = BLDR_NACK;
	HAL_UART_Transmit(&BLDR_UART, &nack, 1, HAL_MAX_DELAY);
	}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	int counter = 0;
	while (1) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
		while (counter < 0x3FFFF) {counter++;}
		counter = 0;
		}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
