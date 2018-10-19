#include "stm32_mini.h"
	
DMA_InitTypeDef dma_struct;
uint8_t *global_data_num_pointer;
uint8_t global_data_num;

void mini_led_init(void)
{
	GPIO_InitTypeDef gpio_struct;
	
	GPIO_RCC_CMD(LED_RCC, ENABLE);
	
	gpio_struct.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_struct.GPIO_Speed = GPIO_Speed_2MHz;
	gpio_struct.GPIO_Pin = LED_D4 | LED_D5;
	GPIO_Init(LED_GPIO, &gpio_struct);
}

void mini_pb_config(void)
{
	// struct define
	GPIO_InitTypeDef gpio_struct;
	EXTI_InitTypeDef exti_struct;
	NVIC_InitTypeDef nvic_struct;
	
	// enable clock
	// enable AFIO!!!
	GPIO_RCC_CMD(AFIO_RCC, ENABLE);
	GPIO_RCC_CMD(PB_K1_RCC, ENABLE);
	GPIO_RCC_CMD(PB_K2_RCC, ENABLE);
	
	// GPIO initialization
	gpio_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio_struct.GPIO_Pin = PB_K1;
	GPIO_Init(PB_K1_GPIO, &gpio_struct);
	
	GPIO_EXTILineConfig(PB_K1_PORTSOURCE, PB_K1_PINSOURCE);
	
	gpio_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio_struct.GPIO_Pin = PB_K2;
	GPIO_Init(PB_K2_GPIO, &gpio_struct);
	
	GPIO_EXTILineConfig(PB_K2_PORTSOURCE, PB_K2_PINSOURCE);
	
	// EXTI initialization
	exti_struct.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_struct.EXTI_Trigger = EXTI_Trigger_Rising;
	exti_struct.EXTI_Line = PB_K1_EXTILINE;
	exti_struct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_struct);
	
	exti_struct.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_struct.EXTI_Trigger = EXTI_Trigger_Rising;
	exti_struct.EXTI_Line = PB_K2_EXTILINE;
	exti_struct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_struct);
	
	// NVIC initialization
	// set priority group
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	nvic_struct.NVIC_IRQChannel = PB_K1_IRQ;
	nvic_struct.NVIC_IRQChannelPreemptionPriority = PB_PREPRIO;
	nvic_struct.NVIC_IRQChannelSubPriority = PB_SUBPRIO;
	nvic_struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_struct);
	
	nvic_struct.NVIC_IRQChannel = PB_K2_IRQ;
	nvic_struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_struct);
}

void mini_uart_config(void)
{
	// structure define
	USART_InitTypeDef usart_struct;
	GPIO_InitTypeDef gpio_struct;
	NVIC_InitTypeDef nvic_struct;
	
	// clock enable
	DEBUG_UART_RCC_CMD(DEBUG_UART_RCC, ENABLE);
	GPIO_RCC_CMD(AFIO_RCC, ENABLE);
	GPIO_RCC_CMD(DEBUG_UART_GPIO_RCC, ENABLE);
	
	// GPIO initialization
	gpio_struct.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_struct.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_struct.GPIO_Pin = DEBUG_UART_TX;
	GPIO_Init(DEBUG_UART_GPIO, &gpio_struct);
	
	gpio_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio_struct.GPIO_Pin = DEBUG_UART_RX;
	GPIO_Init(DEBUG_UART_GPIO, &gpio_struct);
	
	// NVIC	initialization
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	nvic_struct.NVIC_IRQChannel = DEBUG_UART_IRQ;
	nvic_struct.NVIC_IRQChannelPreemptionPriority = DEBUG_UART_PRIPRIO;
	nvic_struct.NVIC_IRQChannelSubPriority = DEBUG_UART_SUBPRIO;
	nvic_struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_struct);
	
	// USART initialization
	usart_struct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	usart_struct.USART_Parity = USART_Parity_No;
	usart_struct.USART_StopBits = USART_StopBits_1;
	usart_struct.USART_WordLength = USART_WordLength_8b;
	usart_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_struct.USART_BaudRate = DEBUG_UART_BAUDRATE;
	USART_Init(DEBUG_UART, &usart_struct);
	
	USART_Cmd(DEBUG_UART, ENABLE);
	
	USART_ITConfig(DEBUG_UART, USART_IT_RXNE, ENABLE);
}

void mini_i2c_init(void)
{
	// structure define
	I2C_InitTypeDef i2c_struct;
	GPIO_InitTypeDef gpio_struct;
	NVIC_InitTypeDef nvic_struct;
	
	// clock enable
	USER_I2C_RCC_CMD(USER_I2C_RCC, ENABLE);
	GPIO_RCC_CMD(AFIO_RCC, ENABLE);
	GPIO_RCC_CMD(USER_I2C_GPIO_RCC, ENABLE);
	USER_I2C_DMA_RCC_CMD(USER_I2C_DMA_RCC, ENABLE);
	
	// GPIO initialization
	gpio_struct.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio_struct.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_struct.GPIO_Pin = USER_I2C_SCL | USER_I2C_SDA;
	GPIO_Init(USER_I2C_GPIO, &gpio_struct);
	
	// I2C DMA NVIC initialization
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	nvic_struct.NVIC_IRQChannel = USER_I2C_DMA_TX_IRQ;
	nvic_struct.NVIC_IRQChannelPreemptionPriority = USER_I2C_DMA_PREPRIO;
	nvic_struct.NVIC_IRQChannelSubPriority = USER_I2C_DMA_SUBPRIO;
	nvic_struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_struct);
	
	nvic_struct.NVIC_IRQChannel = USER_I2C_DMA_RX_IRQ;
	NVIC_Init(&nvic_struct);
	
	// I2C DMA
	DMA_DeInit(USER_I2C_DMA_CH_TX);
	dma_struct.DMA_PeripheralBaseAddr = USER_I2C_DR_ADDR;
	dma_struct.DMA_MemoryBaseAddr = 0;			// will be reconfigure
	dma_struct.DMA_DIR = DMA_DIR_PeripheralDST;	// will be reconfigure
	dma_struct.DMA_BufferSize = 0xFFFF;			// will be reconfigure
	dma_struct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_struct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma_struct.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	dma_struct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma_struct.DMA_Mode = DMA_Mode_Normal;
	dma_struct.DMA_Priority = DMA_Priority_VeryHigh;
	dma_struct.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(USER_I2C_DMA_CH_TX, &dma_struct);
	
	DMA_DeInit(USER_I2C_DMA_CH_RX);
	DMA_Init(USER_I2C_DMA_CH_RX, &dma_struct);
	
	DMA_ITConfig(USER_I2C_DMA_CH_TX, DMA_IT_TC, ENABLE);
	DMA_ITConfig(USER_I2C_DMA_CH_RX, DMA_IT_TC, ENABLE);
	
	// I2C initialization
	i2c_struct.I2C_Mode = I2C_Mode_I2C;
	i2c_struct.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c_struct.I2C_OwnAddress1 = USER_I2C_OWN_ADDR;
	i2c_struct.I2C_Ack = I2C_Ack_Enable;
	i2c_struct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2c_struct.I2C_ClockSpeed = USER_I2C_SPEED;
	I2C_Init(USER_I2C, &i2c_struct);
	
	I2C_Cmd(USER_I2C, ENABLE);
	
	I2C_DMACmd(USER_I2C, ENABLE);
}

void mini_i2c_dma_config(uint32_t mem_addr, uint32_t buffer_size, uint32_t direction)
{
	dma_struct.DMA_MemoryBaseAddr = mem_addr;
	dma_struct.DMA_DIR = direction;
	dma_struct.DMA_BufferSize = buffer_size;
	if(direction == USER_I2C_DMA_DIR_TX)
	{
		DMA_Init(USER_I2C_DMA_CH_TX, &dma_struct);
	}
	else
	{
		DMA_Init(USER_I2C_DMA_CH_RX, &dma_struct);
	}
}

void mini_i2c_byte_write(uint8_t word_addr, uint8_t byte_data)
{
	// start
	I2C_GenerateSTART(USER_I2C, ENABLE);
	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_MODE_SELECT) == RESET);
	
	// device address
	I2C_Send7bitAddress(USER_I2C, EEPROM_ADDR, I2C_Direction_Transmitter);
	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == RESET);
	
	// EEPROM address
	I2C_SendData(USER_I2C, word_addr);
	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == RESET);
	
	I2C_SendData(USER_I2C, byte_data);
	while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_BTF) == RESET);
	
	I2C_GenerateSTOP(USER_I2C, ENABLE);
}

// AT24C02 accept 8-byte page write
void mini_i2c_page_write(uint8_t *page_data, uint8_t word_addr, uint8_t *data_num_pointer)
{
	global_data_num_pointer = data_num_pointer;
	
	// S
	I2C_GenerateSTART(USER_I2C, ENABLE);
	// EV5: SB=1, cleared by reading SR1 register followed by writing DR register with Address
	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_MODE_SELECT) == RESET);
	
	// Address
	I2C_Send7bitAddress(USER_I2C, EEPROM_ADDR, I2C_Direction_Transmitter);
	// EV6: ADDR=1, cleared by reading SR1 register followed by reading SR2
	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == RESET);
	
	// word address
	I2C_SendData(USER_I2C, word_addr);
	
//	for(uint8_t i=0;i<8;i++)
//	{
//		// EV8: TxE=1, shift register not empty, data regiter empty, cleared by writing DR register
//		while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING) == RESET);
//		I2C_SendData(USER_I2C, page_data[i]);
//	}
//	
//	// EV8_2: TxE=1, BTF=1, Program Stop request. TxE and BTF are cleared by hardware by the Stop condition
//	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == RESET);
//	I2C_GenerateSTOP(USER_I2C, ENABLE);

	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == RESET);
	
	mini_i2c_dma_config((uint32_t)page_data, *data_num_pointer, USER_I2C_DMA_DIR_TX);
	
	DMA_Cmd(USER_I2C_DMA_CH_TX, ENABLE);
}

void mini_i2c_write(uint8_t *data, uint8_t word_addr, uint16_t data_num)
{
	uint8_t num_page = 0, num_single = 0, count = 0, addr = 0;
	
	addr = word_addr % EEPROM_PAGESIZE;
	count = EEPROM_PAGESIZE - addr;
	num_page = data_num / EEPROM_PAGESIZE;
	num_single = data_num % EEPROM_PAGESIZE;
	
	// if word_addr is EEPROM_PAGESIZE aligned
	if(addr == 0)
	{
		// if data_num < EEPROM_PAGESIZE
		if(num_page == 0)
		{
			global_data_num = num_single;			
			mini_i2c_page_write(data, word_addr, (uint8_t*)(&global_data_num));			
			while(global_data_num > 0);			
			mini_i2c_wait_standby();
		}
		// if data_num > EEPROM_PAGESIZE
		else
		{
			while(num_page--)
			{
				global_data_num = EEPROM_PAGESIZE;				
				mini_i2c_page_write(data, word_addr, &global_data_num);				
				while(global_data_num > 0);				
				mini_i2c_wait_standby();
				
				data += EEPROM_PAGESIZE;
				word_addr += EEPROM_PAGESIZE;
			}
			if(num_single != 0)
			{
				global_data_num = num_single;				
				mini_i2c_page_write(data, word_addr, (uint8_t*)(&global_data_num));				
				while(global_data_num > 0);				
				mini_i2c_wait_standby();
			}
		}
	}
	else
	{
		// if data_num < EEPROM_PAGESIZE
		if(num_page == 0)
		{
			// if the number of data to be written is more than 
			// the remaining space in the current page
			if(data_num > count)
			{
				// first part
				global_data_num = count;				
				mini_i2c_page_write(data, word_addr, (uint8_t*)(&global_data_num));				
				while(global_data_num > 0);				
				mini_i2c_wait_standby();
				
				// second part
				global_data_num = data_num - count;				
				mini_i2c_page_write(data+count, word_addr+count, (uint8_t*)(&global_data_num));				
				while(global_data_num > 0);				
				mini_i2c_wait_standby();
			}
			else
			{
				global_data_num = num_single;				
				mini_i2c_page_write(data, word_addr, (uint8_t*)(&global_data_num));				
				while(global_data_num > 0);				
				mini_i2c_wait_standby();
			}
		}
		// data_num > EEPROM_PAGESIZE
		else
		{
			data_num -= count;
			num_page = data_num / EEPROM_PAGESIZE;
			num_single = data_num % EEPROM_PAGESIZE;
			
			if(count != 0)
			{
				global_data_num = count;				
				mini_i2c_page_write(data, word_addr, &global_data_num);
				while(global_data_num > 0);
				mini_i2c_wait_standby();
				
				data += count;
				word_addr += count;
			}
			while(num_page--)
			{
				global_data_num = EEPROM_PAGESIZE;
				mini_i2c_page_write(data, word_addr, (uint8_t*)(&global_data_num));
				while(global_data_num > 0);
				mini_i2c_wait_standby();
				
				data += EEPROM_PAGESIZE;
				word_addr += EEPROM_PAGESIZE;
			}
			if(num_single != 0)
			{
				global_data_num = num_single;
				mini_i2c_page_write(data, word_addr, (uint8_t*)(&global_data_num));
				while(global_data_num > 0);
				mini_i2c_wait_standby();
			}
		}
	}
}

// Mathod 2: I2C is used with polling
uint8_t mini_i2c_random_read(uint8_t word_addr)
{
	// start
	I2C_GenerateSTART(USER_I2C, ENABLE);
	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_MODE_SELECT) == RESET);
	
	// device address
	I2C_Send7bitAddress(USER_I2C, EEPROM_ADDR, I2C_Direction_Transmitter);
	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == RESET);
	
	// register address
	I2C_SendData(USER_I2C, word_addr);
	while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_BTF) == RESET);
		
	// restart
	I2C_GenerateSTART(USER_I2C, ENABLE);
	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_MODE_SELECT) == RESET);
	
	// device address
	I2C_Send7bitAddress(USER_I2C, EEPROM_ADDR, I2C_Direction_Receiver);
	while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_ADDR) == RESET);
	
	// EV6_3
	// program ACK = 0
	I2C_AcknowledgeConfig(USER_I2C, DISABLE);
	// clear ADDR by reading SR1 register followed by reading SR2 register
	USER_I2C->SR2;
	// program STOP = 1 just after ADDR is cleared
	I2C_GenerateSTOP(USER_I2C, ENABLE);
	
	while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_RXNE) == RESET);
	uint8_t data = I2C_ReceiveData(USER_I2C);
	
	I2C_AcknowledgeConfig(USER_I2C, ENABLE);
	
	return data;
}

void mini_i2c_sequential_read(uint8_t word_addr, uint8_t *data, uint32_t nbyte)
{
	// start
	I2C_GenerateSTART(USER_I2C, ENABLE);
	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_MODE_SELECT) == RESET);
	
	// device address
	I2C_Send7bitAddress(USER_I2C, EEPROM_ADDR, I2C_Direction_Transmitter);
	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == RESET);
	
	// register address
	I2C_SendData(USER_I2C, word_addr);
	while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_BTF) == RESET);
		
	// restart
	I2C_GenerateSTART(USER_I2C, ENABLE);
	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_MODE_SELECT) == RESET);
	
	// device address
	I2C_Send7bitAddress(USER_I2C, EEPROM_ADDR, I2C_Direction_Receiver);
	while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_ADDR) == RESET);
	
	if(nbyte == 1)
	{
		// EV6_3
		I2C_AcknowledgeConfig(USER_I2C, DISABLE);
		USER_I2C->SR2;
		I2C_GenerateSTOP(USER_I2C, ENABLE);
		
		// EV7
		while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_RXNE) == RESET);
		*data = I2C_ReceiveData(USER_I2C);
		
		I2C_AcknowledgeConfig(USER_I2C, ENABLE);		
	}
	else if(nbyte == 2)
	{
		// set pos flag
		I2C_NACKPositionConfig(USER_I2C, I2C_NACKPosition_Next);
		
		// EV6
		USER_I2C->SR2;
		// EV6_1
		I2C_AcknowledgeConfig(USER_I2C, DISABLE);
		
		// EV7_3
		while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_BTF) == RESET);
		I2C_GenerateSTOP(USER_I2C, ENABLE);
		*data = I2C_ReceiveData(USER_I2C);
		*(data+1) = I2C_ReceiveData(USER_I2C);
		
		I2C_AcknowledgeConfig(USER_I2C, ENABLE);
	}
	else
	{
		// EV6
		USER_I2C->SR2;
		
		for(uint32_t i=0;i<nbyte-3;i++)
		{
			// EV7, using BTF instea of RXNE, refering Discovering STM32 Microcontroller
			while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_BTF) == RESET);
			*(data+i) = I2C_ReceiveData(USER_I2C);
		}
		
		// EV7_2
		while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_BTF) == RESET);
		I2C_AcknowledgeConfig(USER_I2C, DISABLE);
		*(data+nbyte-3) = I2C_ReceiveData(USER_I2C);
		I2C_GenerateSTOP(USER_I2C, ENABLE);
		*(data+nbyte-2) = I2C_ReceiveData(USER_I2C);
		
		// EV7
		while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_RXNE) == RESET);
		*(data+nbyte-1) = I2C_ReceiveData(USER_I2C);
		
		I2C_AcknowledgeConfig(USER_I2C, ENABLE);
	}
}

// method 1
void mini_i2c_read(uint8_t *data, uint8_t word_addr, uint8_t data_num)
{
	I2C_GenerateSTART(USER_I2C, ENABLE);
	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_MODE_SELECT) == RESET);
	
	I2C_Send7bitAddress(USER_I2C, EEPROM_ADDR, I2C_Direction_Transmitter);
	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == RESET);
	
	I2C_SendData(USER_I2C, word_addr);
	while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_BTF) == RESET);
	
	I2C_GenerateSTART(USER_I2C, ENABLE);
	while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_MODE_SELECT) == RESET);
	
	I2C_Send7bitAddress(USER_I2C, EEPROM_ADDR, I2C_Direction_Receiver);
	
	if(data_num == 1)
	{
		while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_ADDR) == RESET);
		
		I2C_AcknowledgeConfig(USER_I2C, DISABLE);
		USER_I2C->SR2;
		I2C_GenerateSTOP(USER_I2C, ENABLE);
		
		while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_RXNE) == RESET);
		
		*data = I2C_ReceiveData(USER_I2C);
		
		while(USER_I2C->CR1 & I2C_CR1_STOP);
		
		I2C_AcknowledgeConfig(USER_I2C, ENABLE);
	}
	else
	{
		while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == RESET);
		
		mini_i2c_dma_config((uint32_t)data, data_num, USER_I2C_DMA_DIR_RX);
		
		// Inform the DMA that the next End Of Transfer Signal will be the last one
		I2C_DMALastTransferCmd(USER_I2C, ENABLE);
		
		DMA_Cmd(USER_I2C_DMA_CH_RX, ENABLE);
	}
}

// after write operation, EEPROM need some time
void mini_i2c_wait_standby(void)
{
	uint8_t busy = 1;
	uint16_t tmp_sr;
	
	while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_BUSY));
	
	while(busy)
	{
		I2C_GenerateSTART(USER_I2C, ENABLE);
		while(I2C_CheckEvent(USER_I2C, I2C_EVENT_MASTER_MODE_SELECT) == RESET);
		
		I2C_Send7bitAddress(USER_I2C, EEPROM_ADDR, I2C_Direction_Transmitter);
		tmp_sr = USER_I2C->SR1;
		
		// check if the EEPROM responded
		do
		{
			tmp_sr = USER_I2C->SR1;
		}
		while((tmp_sr & (I2C_SR1_ADDR | I2C_SR1_AF)) == 0);
		
		if(tmp_sr & I2C_SR1_ADDR)
		{
			USER_I2C->SR2;
			
			I2C_GenerateSTOP(USER_I2C, ENABLE);
			
			busy = 0;
		}
		else
		{
			I2C_ClearFlag(USER_I2C, I2C_FLAG_AF);
		}
	}
}

int fputc(int ch, FILE* f)
{
	while(USART_GetFlagStatus(DEBUG_UART, USART_FLAG_TXE) == RESET);
	
	USART_SendData(DEBUG_UART, ch);
	
	return ch;
}

