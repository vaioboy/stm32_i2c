/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t i2c_write_data[];
extern uint8_t i2c_rx_data[255];
extern uint8_t *global_data_num_pointer;
extern uint8_t global_data_num;
extern uint8_t test_num;
extern uint8_t test_addr;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void PB_K1_IRQ_HANDLER(void)
{
	if(EXTI_GetITStatus(PB_K1_EXTILINE) != RESET)
	{
		EXTI_ClearITPendingBit(PB_K1_EXTILINE);
		
//		mini_i2c_sequential_read(0x00, i2c_rx_data, 255);
		mini_i2c_read(i2c_rx_data, test_addr, test_num);
		LED_D4_TOGGLE;
	}
}

void PB_K2_IRQ_HANDLER(void)
{
	if(EXTI_GetITStatus(PB_K2_EXTILINE) != RESET)
	{
		EXTI_ClearITPendingBit(PB_K2_EXTILINE);
		
		mini_i2c_write(i2c_write_data, test_addr, test_num);
//		test_num = 8;
//		mini_i2c_page_write(i2c_write_data, 0, &test_num);
		LED_D5_TOGGLE;
	}
}

void DEBUG_UART_IRQ_HANDLER(void)
{
	if(USART_GetITStatus(DEBUG_UART, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(DEBUG_UART, USART_IT_RXNE);
		
		uint16_t data = USART_ReceiveData(DEBUG_UART);
		USART_SendData(DEBUG_UART, data);
	}
}

void USER_I2C_DMA_TX_IRQ_HANDLER(void)
{
	if(DMA_GetITStatus(USER_I2C_DMA_IT_TX_TC) != RESET)
	{
		// GL is used to clear all flag
		DMA_ClearITPendingBit(USER_I2C_DMA_IT_TX_GL);
		
		DMA_Cmd(USER_I2C_DMA_CH_TX, DISABLE);
		while(I2C_GetFlagStatus(USER_I2C, I2C_FLAG_BTF) == RESET);
		
		I2C_GenerateSTOP(USER_I2C, ENABLE);
		
		USER_I2C->SR1;
		USER_I2C->SR2;
		
		*global_data_num_pointer = 0;
	}
}

void USER_I2C_DMA_RX_IRQ_HANDLER(void)
{
	if(DMA_GetITStatus(USER_I2C_DMA_IT_RX_TC) != RESET)
	{
		DMA_ClearITPendingBit(USER_I2C_DMA_IT_RX_GL);
		
		I2C_GenerateSTOP(USER_I2C, ENABLE);
		
		DMA_Cmd(USER_I2C_DMA_CH_RX, DISABLE);
	}
}
/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
