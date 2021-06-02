#include "util/misc.h"
#include "i2c.h"

#define CLEAR_ADDR_FLAG() 				{uint32_t tmp = I2C1->SR1; tmp = I2C1->SR2; (void)tmp;}

#define I2C_DUTYCYCLE_2                 0x00000000U
#define I2C_DUTYCYCLE_16_9              I2C_CCR_DUTY

#define I2C_ADDRESSINGMODE_7BIT         0x00004000U
#define I2C_ADDRESSINGMODE_10BIT        (I2C_OAR1_ADDMODE | 0x00004000U)

#define I2C_DUALADDRESS_DISABLE        	0x00000000U
#define I2C_DUALADDRESS_ENABLE         	I2C_OAR2_ENDUAL

#define I2C_GENERALCALL_DISABLE        	0x00000000U
#define I2C_GENERALCALL_ENABLE         	I2C_CR1_ENGC

#define I2C_NOSTRETCH_DISABLE          	0x00000000U
#define I2C_NOSTRETCH_ENABLE           	I2C_CR1_NOSTRETCH

#define I2C_CCR_CALCULATION(pclk, speed, coeff)     (((((pclk) - 1U)/((speed) * (coeff))) + 1U) & I2C_CCR_CCR)
#define I2C_RISE_TIME(freqrange, speed)            	(((speed) <= 100000U) ? ((freqrange) + 1U) : ((((freqrange) * 300U) / 1000U) + 1U))
#define I2C_SPEED_STANDARD(pclk, speed)            	((I2C_CCR_CALCULATION((pclk), (speed), 2U) < 4U)? 4U:I2C_CCR_CALCULATION((pclk), (speed), 2U))
#define I2C_SPEED_FAST(pclk, speed, dutycycle) 		(((dutycycle) == I2C_DUTYCYCLE_2)? I2C_CCR_CALCULATION((pclk), (speed), 3U) : (I2C_CCR_CALCULATION((pclk), (speed), 25U) | I2C_DUTYCYCLE_16_9))

#define I2C_CLOCK_FREQ_HZ 				400000U // fast clock 400kHz

static void i2c_request_read(uint8_t slave_addr,  uint8_t mem_addr);
static void i2c_request_write(uint8_t slave_addr, uint8_t mem_addr);

// This is a skeletonized version of the HAL I2C api :
// 1. specific to STM32F407 I2C1 with SCL = PB6, SDA = PB7
// 2. clock frequency 400kHz
// 3. minimal interface : init, read_buffer and write_buffer
// 4. polled, no timeouts

void i2c_init() {
	// Enable clock to GPIOB
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	// Configure PB6 and PB7 as I2C SCL and SDA (AF4)
	MODIFY_REG(GPIOB->AFR[0],
		GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7,
		_VAL2FLD(GPIO_AFRL_AFSEL6, 4) | _VAL2FLD(GPIO_AFRL_AFSEL7, 4)
		);
	// Configure I2C pins to work in alternate function mode = 2 (0= input, 1= output).
	MODIFY_REG(GPIOB->MODER,
		GPIO_MODER_MODER6 | GPIO_MODER_MODER7,
		_VAL2FLD(GPIO_MODER_MODER6, 2) | _VAL2FLD(GPIO_MODER_MODER7, 2)
		);
	// Enable internal weak pullups
	MODIFY_REG(GPIOB->PUPDR,
		GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7,
		_VAL2FLD(GPIO_PUPDR_PUPD6, 1) | _VAL2FLD(GPIO_PUPDR_PUPD7, 1)
		);
	// Set pins to open drain
	SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT6);
	SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT7);


	// enable clock to I2C1 peripheral before accessing I2C1 registers
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	// disable I2C1 before initialization
	CLEAR_BIT(I2C1->CR1, I2C_CR1_PE);
	// software reset of I2C peripheral
	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;
	// need APB1 clock frequency
	uint32_t pclk1 = get_pclk1_freq();
	uint32_t pclk1_mhz = get_pclk1_freq()/1000000;
	// Frequency range
	MODIFY_REG(I2C1->CR2, I2C_CR2_FREQ, pclk1_mhz);
	// Rise Time
	MODIFY_REG(I2C1->TRISE, I2C_TRISE_TRISE, I2C_RISE_TIME(pclk1_mhz, I2C_CLOCK_FREQ_HZ));
	// Speed 400kHz
	MODIFY_REG(I2C1->CCR, (I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR), I2C_SPEED_FAST(pclk1, I2C_CLOCK_FREQ_HZ, I2C_DUTYCYCLE_2));
	// Generalcall and NoStretch mode
	MODIFY_REG(I2C1->CR1, (I2C_CR1_ENGC | I2C_CR1_NOSTRETCH), (I2C_GENERALCALL_DISABLE | I2C_NOSTRETCH_DISABLE));
	// Own Address1 and addressing mode
	MODIFY_REG(I2C1->OAR1, (I2C_OAR1_ADDMODE | I2C_OAR1_ADD8_9 | I2C_OAR1_ADD1_7 | I2C_OAR1_ADD0), (I2C_ADDRESSINGMODE_7BIT | 0));
	// Dual mode and Own Address2
	MODIFY_REG(I2C1->OAR2, (I2C_OAR2_ENDUAL | I2C_OAR2_ADD2), (I2C_DUALADDRESS_DISABLE | 0));
	// enable I2C1
	SET_BIT(I2C1->CR1, I2C_CR1_PE);
	}


static void i2c_request_write(uint8_t slave_addr, uint8_t mem_addr){
	// Generate Start
	SET_BIT(I2C1->CR1, I2C_CR1_START);
	// Wait until SB flag is set
	while (!(I2C1->SR1 & I2C_SR1_SB));
	// Send slave address (write)
	I2C1->DR = slave_addr;
	// Wait until ADDR flag is set
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	CLEAR_ADDR_FLAG();
	// Wait until TXE flag is set
	while (!(I2C1->SR1 & I2C_SR1_TXE));
	// Send Memory Address
	I2C1->DR = mem_addr;
	// Wait until TXE flag is set
	while (!(I2C1->SR1 & I2C_SR1_TXE));
	}


static void i2c_request_read(uint8_t slave_addr,  uint8_t mem_addr){
	// Enable Acknowledge
	SET_BIT(I2C1->CR1, I2C_CR1_ACK);
	// Generate Start
	SET_BIT(I2C1->CR1, I2C_CR1_START);
	// Wait until SB flag is set
	while (!(I2C1->SR1 & I2C_SR1_SB));
	// Send slave address (write)
	I2C1->DR = slave_addr;
	// Wait until ADDR flag is set
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	CLEAR_ADDR_FLAG();
	// Wait until TXE flag is set
	while (!(I2C1->SR1 & I2C_SR1_TXE));
	//Send Memory Address
	I2C1->DR = mem_addr;
	// Wait until TXE flag is set
	while (!(I2C1->SR1 & I2C_SR1_TXE));
	// Generate Restart
	SET_BIT(I2C1->CR1, I2C_CR1_START);
	// Wait until SB flag is set
	while (!(I2C1->SR1 & I2C_SR1_SB));
	// Send slave address (read)
	I2C1->DR = slave_addr | 0x01;
	// Wait until ADDR flag is set
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	}


void i2c_write(uint8_t slave_addr, uint8_t mem_addr,  uint8_t *pData, int num_bytes) {
    // Wait until BUSY flag is reset
    while (I2C1->SR2 & I2C_SR2_BUSY);
    // Disable Pos
    CLEAR_BIT(I2C1->CR1, I2C_CR1_POS);
    int count = num_bytes;
    // Send Slave Address and Memory Address
    i2c_request_write(slave_addr, mem_addr);
	while (count)    {
		// Wait until TXE flag is set
		while (!(I2C1->SR1 & I2C_SR1_TXE));
		I2C1->DR = *pData;
		pData++;
		count--;
		if ((I2C1->SR1 & I2C_SR1_BTF) && count)      {
			I2C1->DR = *pData;
			pData++;
			count--;
			}
		}
    // Wait until BTF flag is set
    while(!(I2C1->SR1 & I2C_SR1_BTF));
    // Generate Stop
    SET_BIT(I2C1->CR1, I2C_CR1_STOP);
	}


void i2c_read(uint8_t slave_addr, uint8_t mem_addr, uint8_t *pData, int num_bytes){
    // Wait until BUSY flag is reset
    while (I2C1->SR2 & I2C_SR2_BUSY);
    // Disable Pos
    CLEAR_BIT(I2C1->CR1, I2C_CR1_POS);
    int count = num_bytes;
    // Send Slave Address and Memory Address
    i2c_request_read(slave_addr, mem_addr);
    if (count == 1){
		// Disable Acknowledge
		CLEAR_BIT(I2C1->CR1, I2C_CR1_ACK);
		CLEAR_ADDR_FLAG();
		// Generate Stop
		SET_BIT(I2C1->CR1, I2C_CR1_STOP);
		}
    else
    if (count == 2)   {
		// Disable Acknowledge
		CLEAR_BIT(I2C1->CR1, I2C_CR1_ACK);
		// Enable Pos
		SET_BIT(I2C1->CR1, I2C_CR1_POS);
		CLEAR_ADDR_FLAG();
		}
    else {
		CLEAR_ADDR_FLAG();
    	}
    while (count > 0) {
    	if (count <= 3)  {
    		// One byte
    		if (count == 1) {
    			// Wait until RXNE flag is set
    			while(!(I2C1->SR1 & I2C_SR1_RXNE));
    			*pData = (uint8_t)I2C1->DR;
    			pData++;
    			count--;
        		}
    		// Two bytes
    		else
    		if (count == 2) {
    			// Wait until BTF flag is set
    			while(!(I2C1->SR1 & I2C_SR1_BTF));
    			// Generate Stop
    			SET_BIT(I2C1->CR1, I2C_CR1_STOP);
    			*pData = (uint8_t)I2C1->DR;
    			pData++;
    			count--;
    			*pData = (uint8_t)I2C1->DR;
    			pData++;
    			count--;
    			}
    		// 3 Last bytes
    		else {
    			// Wait until BTF flag is set
    			while(!(I2C1->SR1 & I2C_SR1_BTF));
    			// Disable Acknowledge
    			CLEAR_BIT(I2C1->CR1, I2C_CR1_ACK);
    			*pData = (uint8_t)I2C1->DR;
    			pData++;
    			count--;
    			while(!(I2C1->SR1 & I2C_SR1_BTF));
    			SET_BIT(I2C1->CR1, I2C_CR1_STOP);
    			*pData = (uint8_t)I2C1->DR;
    			pData++;
    			count--;
    			*pData = (uint8_t)I2C1->DR;
    			pData++;
    			count--;
    			}
    		}
      else  {
    	  // Wait until RXNE flag is set
          while(!(I2C1->SR1 & I2C_SR1_RXNE));
          // Read data from DR
          *pData = (uint8_t)I2C1->DR;
          pData++;
          count--;
          if (I2C1->SR1 & I2C_SR1_BTF) {
        	  *pData = (uint8_t)I2C1->DR;
        	  pData++;
        	  count--;
          	  }
      	}
    } // while (count > 0)
}

