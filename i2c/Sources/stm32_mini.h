#ifndef __STM32_MINI_H
#define __STM32_MINI_H

#include "stm32f10x.h"
#include <stdio.h>

#define GPIO_RCC_CMD						RCC_APB2PeriphClockCmd

#define GPIO_ToggleBits(GPIOx, GPIO_Pin)	GPIOx->ODR ^= GPIO_Pin

#define AFIO_RCC						RCC_APB2Periph_AFIO

#define LED_GPIO						GPIOC
#define LED_RCC							RCC_APB2Periph_GPIOC
#define LED_D4							GPIO_Pin_2
#define LED_D5							GPIO_Pin_3
#define LED_D4_ON						GPIO_ResetBits(LED_GPIO, LED_D4)
#define LED_D4_OFF						GPIO_SetBits(LED_GPIO, LED_D4)
#define LED_D4_TOGGLE					GPIO_ToggleBits(LED_GPIO, LED_D4)
#define LED_D5_ON						GPIO_ResetBits(LED_GPIO, LED_D5)
#define LED_D5_OFF						GPIO_SetBits(LED_GPIO, LED_D5)
#define LED_D5_TOGGLE					GPIO_ToggleBits(LED_GPIO, LED_D5)

#define PB_K1_GPIO						GPIOA
#define PB_K1_RCC						RCC_APB2Periph_GPIOA
#define PB_K1							GPIO_Pin_0
#define PB_K1_PORTSOURCE				GPIO_PortSourceGPIOA
#define PB_K1_PINSOURCE					GPIO_PinSource0
#define PB_K1_EXTILINE					EXTI_Line0
#define PB_K1_IRQ						EXTI0_IRQn
#define PB_K1_IRQ_HANDLER				EXTI0_IRQHandler
#define PB_K2_GPIO						GPIOC
#define PB_K2_RCC						RCC_APB2Periph_GPIOC
#define PB_K2							GPIO_Pin_13
#define PB_K2_PORTSOURCE				GPIO_PortSourceGPIOC
#define PB_K2_PINSOURCE					GPIO_PinSource13
#define PB_K2_EXTILINE					EXTI_Line13
#define PB_K2_IRQ						EXTI15_10_IRQn
#define PB_K2_IRQ_HANDLER				EXTI15_10_IRQHandler
#define PB_PREPRIO						3
#define PB_SUBPRIO						3

#define DEBUG_UART						USART1
#define DEBUG_UART_RCC					RCC_APB2Periph_USART1
#define DEBUG_UART_RCC_CMD				RCC_APB2PeriphClockCmd
#define DEBUG_UART_GPIO					GPIOA
#define DEBUG_UART_GPIO_RCC				RCC_APB2Periph_GPIOA
#define DEBUG_UART_TX					GPIO_Pin_9
#define DEBUG_UART_RX					GPIO_Pin_10
#define DEBUG_UART_BAUDRATE				115200
#define DEBUG_UART_IRQ					USART1_IRQn
#define DEBUG_UART_IRQ_HANDLER			USART1_IRQHandler
#define DEBUG_UART_PRIPRIO				2
#define DEBUG_UART_SUBPRIO				2

#define USER_I2C						I2C1
#define USER_I2C_RCC					RCC_APB1Periph_I2C1
#define USER_I2C_RCC_CMD				RCC_APB1PeriphClockCmd
#define USER_I2C_GPIO					GPIOB
#define USER_I2C_GPIO_RCC				RCC_APB2Periph_GPIOB
#define USER_I2C_SCL					GPIO_Pin_6
#define USER_I2C_SDA					GPIO_Pin_7
#define USER_I2C_SPEED					100000
#define USER_I2C_OWN_ADDR				0x24
#define USER_I2C_EV_IRQ					I2C1_EV_IRQn
#define USER_I2C_EV_IRQ_HANDLER			I2C1_EV_IRQHandler
#define USER_I2C_DMA					DMA1
#define USER_I2C_DMA_RCC				RCC_AHBPeriph_DMA1
#define USER_I2C_DMA_RCC_CMD			RCC_AHBPeriphClockCmd
#define USER_I2C_DMA_CH_TX				DMA1_Channel6
#define USER_I2C_DMA_CH_RX				DMA1_Channel7
#define USER_I2C_DMA_TX_IRQ				DMA1_Channel6_IRQn
#define USER_I2C_DMA_RX_IRQ				DMA1_Channel7_IRQn
#define USER_I2C_DMA_TX_IRQ_HANDLER		DMA1_Channel6_IRQHandler
#define USER_I2C_DMA_RX_IRQ_HANDLER		DMA1_Channel7_IRQHandler
#define USER_I2C_DMA_PREPRIO			0
#define USER_I2C_DMA_SUBPRIO			0
#define USER_I2C_DR_ADDR				((uint32_t)0x40005410)
#define USER_I2C_DMA_DIR_TX				DMA_DIR_PeripheralDST
#define USER_I2C_DMA_DIR_RX				DMA_DIR_PeripheralSRC
#define USER_I2C_DMA_IT_TX_TC			DMA1_IT_TC6
#define USER_I2C_DMA_IT_TX_GL			DMA1_IT_GL6
#define USER_I2C_DMA_IT_RX_TC			DMA1_IT_TC7
#define USER_I2C_DMA_IT_RX_GL			DMA1_IT_GL7

#define EEPROM_ADDR						0xA0
#define EEPROM_PAGESIZE					8

void mini_led_init(void);
void mini_pb_config(void);
void mini_uart_config(void);
void mini_i2c_init(void);
void mini_i2c_byte_write(uint8_t word_addr, uint8_t byte_data);
void mini_i2c_page_write(uint8_t *page_data, uint8_t word_addr, uint8_t *data_num_pointer);
void mini_i2c_write(uint8_t *data, uint8_t word_addr, uint16_t data_num);
uint8_t mini_i2c_random_read(uint8_t word_addr);
void mini_i2c_sequential_read(uint8_t word_addr, uint8_t *data, uint32_t nbyte);
void mini_i2c_read(uint8_t *data, uint8_t word_addr, uint8_t data_num);
void mini_i2c_wait_standby(void);
void mini_i2c_dma_config(uint32_t mem_addr, uint32_t buffer_size, uint32_t direction);

#endif
