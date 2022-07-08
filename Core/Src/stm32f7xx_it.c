/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "hw.h"        //Added by GG 10/6/21
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern bool bPfcLoadEnable;    //added by GG 10/6/21
extern uint32_t uPfcLoadEnTimeDB;  //added by GG 10/6/21

extern bool bPowerFailWarn;   //added by GG 10/6/21
extern uint32_t uPowerFailWarnTimeDB;  //added by GG 10/6/21

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc3;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern DMA_HandleTypeDef hdma_sdmmc2_tx;
extern DMA_HandleTypeDef hdma_sdmmc2_rx;
extern SD_HandleTypeDef hsd2;
extern DMA_HandleTypeDef hdma_tim6_up;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart8;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim6_up);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles I2C2 event interrupt.
  */
void I2C2_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_EV_IRQn 0 */

  /* USER CODE END I2C2_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_EV_IRQn 1 */

  /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	uint32_t i = 0;    //added by GG 10/06/21
	if(__HAL_GPIO_EXTI_GET_FLAG(PFC_POWERFAILWARNNING_N_Pin))
	{
		bPowerFailWarn = true;
		uPowerFailWarnTimeDB = Get_SysTick();
	}
	if(__HAL_GPIO_EXTI_GET_FLAG(PFC_LoadEN_N_Pin))
	{
		for (i=0; i<1000000; i++) { }  //stupid delay to avoid using HAL_delay inside interupt GG 10/05/21
                                       //we must delay to de-bounce signal here, not in the main! GG
		if (! HAL_GPIO_ReadPin(GPIOF, PFC_LoadEN_N_Pin))  //if signal is still low then set flag
		{
			SetPfcLoadEnable( true );
			HAL_GPIO_WritePin(GPIOD, LED_Y_Pin, true);  //added for troubleshoot purposes GG 100521
		}
		//bPfcLoadEnable = true;
	    //uPfcLoadEnTimeDB = Get_SysTick();
	}

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */
  USER_UART_IRQHandler(&debugUart);
  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_sdmmc2_tx);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream4 global interrupt.
  */
void DMA2_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream4_IRQn 0 */

  /* USER CODE END DMA2_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream4_IRQn 1 */

  /* USER CODE END DMA2_Stream4_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream5 global interrupt.
  */
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_sdmmc2_rx);
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/**
  * @brief This function handles I2C3 event interrupt.
  */
void I2C3_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_EV_IRQn 0 */

  /* USER CODE END I2C3_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_EV_IRQn 1 */

  /* USER CODE END I2C3_EV_IRQn 1 */
}

/**
  * @brief This function handles UART8 global interrupt.
  */
void UART8_IRQHandler(void)
{
  /* USER CODE BEGIN UART8_IRQn 0 */

  /* USER CODE END UART8_IRQn 0 */
  HAL_UART_IRQHandler(&huart8);
  /* USER CODE BEGIN UART8_IRQn 1 */
  USER_UART_IRQHandler(&guiUart);
  /* USER CODE END UART8_IRQn 1 */
}

/**
  * @brief This function handles SDMMC2 global interrupt.
  */
void SDMMC2_IRQHandler(void)
{
  /* USER CODE BEGIN SDMMC2_IRQn 0 */

  /* USER CODE END SDMMC2_IRQn 0 */
  HAL_SD_IRQHandler(&hsd2);
  /* USER CODE BEGIN SDMMC2_IRQn 1 */

  /* USER CODE END SDMMC2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(UART5 == huart->Instance)
    {
        if(RESET != __HAL_UART_GET_FLAG(&debugUart, UART_FLAG_IDLE))   // Determine whether idle interrupt is set
        {
            __HAL_UART_CLEAR_IDLEFLAG(&debugUart);                     // Clear idle interrupt (otherwise it will continue to enter interrupt)
            USER_UART_IDLECallback(&debugUart);                             // Call UART idle interrupt handler
        }
    }

    if(UART8 == huart->Instance)
    {
        if(RESET != __HAL_UART_GET_FLAG(&guiUart, UART_FLAG_IDLE))   // Determine whether idle interrupt is set
        {
            __HAL_UART_CLEAR_IDLEFLAG(&guiUart);                     // Clear idle interrupt (otherwise it will continue to enter interrupt)
            USER_UART8_IDLECallback(&guiUart);                             // Call UART idle interrupt handler
        }
    }
}

//Declare external variables for Debug UART
extern uint8_t uartReceiveBuff[UART_RX_DMA_BUFFER_SIZE];
extern char commandBuf[MAX_COMMAND_LENGTH];
extern bool commandReady;
char commandBufTemp[MAX_COMMAND_LENGTH];

const int8_t endOfCommandChar = '\r';
uint16_t unprocessedCommandInputLength = 0;
uint16_t uartRxXferCount;

void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	if( commandReady != false )
	{
		// Previous command has not been processed
		// Do not start assembling next command until previous command has been processed
		return;
	}

	// Get number of bytes received
	uartRxXferCount = UART_RX_DMA_BUFFER_SIZE - huart->RxXferCount;

	// Stop this UART reception
	HAL_UART_AbortReceive_IT(&debugUart);

    // Calculate the length of the received data
    char * endOfCommandPos = strchr((char *)uartReceiveBuff, endOfCommandChar);
    uint16_t rxMessageLength = uartRxXferCount;
    if( endOfCommandPos != NULL )
    {
    	rxMessageLength = (uint8_t *)endOfCommandPos - uartReceiveBuff + 1;
    }

    // Check that total command is not too long
    if (MAX_COMMAND_LENGTH <  (unprocessedCommandInputLength + rxMessageLength) )
    {
    	// Command length exceeds maximum length -- discard current command and return
    	unprocessedCommandInputLength = 0;

    	// Clear Receive and temp command Buffers
    	memset(uartReceiveBuff, 0, MAX_COMMAND_LENGTH);
    	memset(commandBufTemp, '\0', MAX_COMMAND_LENGTH);

    	// Restart to start IT transmission of UART_RX_DMA_BUFFER_SIZE bytes of data at a time
    	HAL_UART_Receive_IT(&debugUart, (uint8_t*)uartReceiveBuff, UART_RX_DMA_BUFFER_SIZE);
    	return;
    }

    // If end of command character is in rx buffer, command is complete and ready for processing
    if( endOfCommandPos != NULL )
    {
    	// Command terminator found
    	// Copy rx buff to command buffer without end of command terminator
    	if( unprocessedCommandInputLength > 0 )
    	{
    		// Concatenate rx buffer with unprocessed command input
    		strncat( commandBufTemp, (char *)uartReceiveBuff, rxMessageLength );

    		// Copy completed message in temp command buffer to command buffer for processing in background
        	strncpy( commandBuf, commandBufTemp, strlen(commandBufTemp) - 1);
    	}
    	else // No unprocessed command input, copy rx input directly to command buffer for processing in background
    	{
    		strncpy( commandBuf, (char *)uartReceiveBuff, rxMessageLength - 1);
    	}

    	// Save any remaining rx input beyond the current command to temp buffer for processing when next rx input is received
    	if( rxMessageLength < uartRxXferCount )
    	{
    		strncpy( commandBufTemp, (char *)(uartReceiveBuff + rxMessageLength), (uartRxXferCount - rxMessageLength) );

    		// Keep track of received unprocessed command size
    		unprocessedCommandInputLength = uartRxXferCount - rxMessageLength;
    	}
    	else // No remaining rx input, clear temp buffer
    	{
    		memset(commandBufTemp, '\0', (unprocessedCommandInputLength + rxMessageLength));

    		// Set unprocessed command length to 0
    		unprocessedCommandInputLength = 0;
    	}

    	// Set flag for command processing by background task since full command has been received
    	commandReady = true;
    }
    else
    {
    	// Command terminator not found
    	// Copy rx buff to temp command buffer for later processing since no terminator was found in rx buff
    	strncat( commandBufTemp, (char *)uartReceiveBuff, rxMessageLength);

    	// Keep track of received unprocessed command size
    	unprocessedCommandInputLength += rxMessageLength;
    }

    // Zero UART Receive Buffer
    memset(uartReceiveBuff, 0, rxMessageLength);

    // Restart IT reception of UART_RX_DMA_BUFFER_SIZE bytes of data at a time
    HAL_UART_Receive_IT(&debugUart, (uint8_t*)uartReceiveBuff, UART_RX_DMA_BUFFER_SIZE);
}

//Declare external variables for GUI UART
extern uint8_t guiUartReceiveBuff[UART_RX_DMA_BUFFER_SIZE];
extern char guiCommandBuf[MAX_COMMAND_LENGTH];
extern bool guiCommandReady;
char guiCommandBufTemp[MAX_COMMAND_LENGTH];
uint16_t guiUnprocessedCommandInputLength = 0;
uint16_t guiUartRxXferCount;

void USER_UART8_IDLECallback(UART_HandleTypeDef *huart)
{
	if( guiCommandReady != false )
	{
		// Previous command has not been processed
		// Do not start assembling next command until previous command has been processed
		return;
	}

	// Get number of bytes received
	guiUartRxXferCount = UART_RX_DMA_BUFFER_SIZE - huart->RxXferCount;

	// Stop this UART reception
	HAL_UART_AbortReceive_IT(&guiUart);

    // Calculate the length of the received data
    char * endOfCommandPos = strchr((char *)guiUartReceiveBuff, endOfCommandChar);
    uint16_t rxMessageLength = guiUartRxXferCount;
    if( endOfCommandPos != NULL )
    {
    	rxMessageLength = (uint8_t *)endOfCommandPos - guiUartReceiveBuff + 1;
    }

    // Check that total command is not too long
    if (MAX_COMMAND_LENGTH <  (guiUnprocessedCommandInputLength + rxMessageLength) )
    {
    	// Command length exceeds maximum length -- discard current command and return
    	guiUnprocessedCommandInputLength = 0;

    	// Clear Receive and temp command Buffers
    	memset(guiUartReceiveBuff, 0, MAX_COMMAND_LENGTH);
    	memset(guiCommandBufTemp, '\0', MAX_COMMAND_LENGTH);

    	// Restart to start IT transmission of UART_RX_DMA_BUFFER_SIZE bytes of data at a time
    	HAL_UART_Receive_IT(&guiUart, (uint8_t*)guiUartReceiveBuff, UART_RX_DMA_BUFFER_SIZE);
    	return;
    }

    // If end of command character is in rx buffer, command is complete and ready for processing
    if( endOfCommandPos != NULL )
    {
    	// Command terminator found
    	// Copy rx buff to command buffer without end of command terminator
    	if( guiUnprocessedCommandInputLength > 0 )
    	{
    		// Concatenate rx buffer with unprocessed command input
    		strncat( guiCommandBufTemp, (char *)guiUartReceiveBuff, rxMessageLength );

    		// Copy completed message in temp command buffer to command buffer for processing in background
        	strncpy( guiCommandBuf, guiCommandBufTemp, strlen(guiCommandBufTemp) - 1);
    	}
    	else // No unprocessed command input, copy rx input directly to command buffer for processing in background
    	{
    		strncpy( guiCommandBuf, (char *)guiUartReceiveBuff, rxMessageLength - 1);
    	}

    	// Save any remaining rx input beyond the current command to temp buffer for processing when next rx input is received
    	if( rxMessageLength < guiUartRxXferCount )
    	{
    		strncpy( guiCommandBufTemp, (char *)(guiUartReceiveBuff + rxMessageLength), (guiUartRxXferCount - rxMessageLength) );

    		// Keep track of received unprocessed command size
    		guiUnprocessedCommandInputLength = guiUartRxXferCount - rxMessageLength;
    	}
    	else // No remaining rx input, clear temp buffer
    	{
    		memset(guiCommandBufTemp, '\0', (guiUnprocessedCommandInputLength + rxMessageLength));

    		// Set unprocessed command length to 0
    		guiUnprocessedCommandInputLength = 0;
    	}

    	// Set flag for command processing by background task since full command has been received
    	guiCommandReady = true;
    }
    else
    {
    	// Command terminator not found
    	// Copy rx buff to temp command buffer for later processing since no terminator was found in rx buff
    	strncat( guiCommandBufTemp, (char *)guiUartReceiveBuff, rxMessageLength);

    	// Keep track of received unprocessed command size
    	guiUnprocessedCommandInputLength += rxMessageLength;
    }

    // Zero UART Receive Buffer
    memset(guiUartReceiveBuff, 0, rxMessageLength);

    // Restart IT reception of UART_RX_DMA_BUFFER_SIZE bytes of data at a time
    HAL_UART_Receive_IT(&guiUart, (uint8_t*)guiUartReceiveBuff, UART_RX_DMA_BUFFER_SIZE);
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
