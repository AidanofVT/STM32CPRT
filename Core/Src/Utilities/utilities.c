#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include "main.h"

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart8;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim14;

uint8_t uartTxQueue[UART_TX_QUEUE_SIZE][UART_TX_QUEUE_MAX_MSG_SIZE];
uint16_t uartTxQueueReadIndex = 0;
uint16_t uartTxQueueWriteIndex = 0;

uint8_t printBuf[UART_TX_QUEUE_MAX_MSG_SIZE];
uint8_t txUartBuf[UART_TX_QUEUE_MAX_MSG_SIZE];
uint8_t txUartErrorBuf[UART_TX_QUEUE_MAX_MSG_SIZE];

uint32_t maxUartTxQueueWaitTimeMs = 0;
uint16_t maxTxUartQueueDepth = 0;
uint16_t curTxUartQueueDepth = 0;

uint8_t AtTxBuf[UART_TX_QUEUE_MAX_MSG_SIZE];

uint8_t guiPrintBuf[UART_TX_QUEUE_MAX_MSG_SIZE];

/*-----------------------------------------------------------------------------
 * Function:    InitUartTxQueue
 *
 * Description: Initialize Tx UART queue read and write indexes to 0.
 *
 * Parameters:  None
 *
 * Returns:     None
 *---------------------------------------------------------------------------*/
void InitUartTxQueue()
{
	uartTxQueueReadIndex = 0;
	uartTxQueueWriteIndex = 0;
	memset(&uartTxQueue[0], 0, UART_TX_QUEUE_SIZE * UART_TX_QUEUE_MAX_MSG_SIZE);
}

/*-----------------------------------------------------------------------------
 * Function:    IsTxUartQueueEmpty
 *
 * Description: Determines if Tx UART queue is empty.
 *
 * Parameters:  None
 *
 * Returns:     true if empty, false if not empty
 *---------------------------------------------------------------------------*/
bool IsTxUartQueueEmpty( )
{
	int16_t queueIndexDiff = uartTxQueueWriteIndex - uartTxQueueReadIndex;
	curTxUartQueueDepth = queueIndexDiff;

	if( queueIndexDiff == 0 )
	{
		// Tx UART queue is empty
		return true;
	}

	if( queueIndexDiff < 0 )
	{
		queueIndexDiff += UART_TX_QUEUE_SIZE;
	}

	// Keep track of maximum Tx UART queue depth
	if( queueIndexDiff > maxTxUartQueueDepth )
	{
		maxTxUartQueueDepth = queueIndexDiff;
	}

	// Tx UART queue is not empty
	return false;
}

/*-----------------------------------------------------------------------------
 * Function:    IsTxUartQueueFull
 *
 * Description: Determines if Tx UART queue is full.
 *
 * Parameters:  None
 *
 * Returns:     true if full, false if not full
 *---------------------------------------------------------------------------*/
bool IsTxUartQueueFull( )
{
	int16_t queueIndexDiff = uartTxQueueWriteIndex - uartTxQueueReadIndex;

	if( queueIndexDiff < 0 )
    {
		queueIndexDiff += UART_TX_QUEUE_SIZE;
	}

	if( queueIndexDiff == (UART_TX_QUEUE_SIZE - 1) )
	{
		// Tx UART queue is full
		return true;
	}

	// Tx UART queue is not full
	return false;
}

/*-----------------------------------------------------------------------------
 * Function:    ReadTxUartQueueAndTransmit
 *
 * Description: If the Tx UART queue is not empty, the queue entry pointed to
 *              by the queue read index is read and transmitted to the Ts UART
 *              with DMA and the queue read index is incremented
 *
 * Parameters:  None
 *
 * Returns:     None
 *---------------------------------------------------------------------------*/
void ReadTxUartQueueAndTransmit()
{
	if( IsTxUartQueueEmpty() != false )
	{
		// Tx UART queue is empty, nothing to read and transmit
		return;
	}

	// Read data from Tx UART queue into buffer
	sprintf((char*)txUartBuf, (char *)&uartTxQueue[uartTxQueueReadIndex]);

	// Transmit buffer data from Tx UART queue to UART via IT
	if( HAL_UART_Transmit_IT(&debugUart, txUartBuf, strlen((char *)txUartBuf) ) != HAL_OK )
	{
		sprintf((char*)txUartBuf, "UART Transmit ERROR in ReadTxUartQueueAndTransmit\n");
		HAL_UART_Transmit(&debugUart, txUartBuf, strlen((char*)txUartBuf), 0x200);
	}

	// Clear read queue location
	memset(&uartTxQueue[uartTxQueueReadIndex], 0, strlen((char *)&uartTxQueue[uartTxQueueReadIndex]));

	// Increment Tx UART queue read index
	uartTxQueueReadIndex++;
	if( uartTxQueueReadIndex >= UART_TX_QUEUE_SIZE )
	{
		// Read index was at maximum, set to 0
		uartTxQueueReadIndex =  0;
	}
}

/*-----------------------------------------------------------------------------
 * Function:    WriteTxUartQueue
 *
 * Description: When the Tx UART queue is not full, the queue entry pointed to
 *              by the queue write index is written with the data in the passed
 *              buffer and the queue write index is incremented
 *
 * Parameters:  buffer - pointer to data to be written to Tx UART queue
 *
 * Returns:     None
 *---------------------------------------------------------------------------*/
void WriteTxUartQueue(uint8_t * buffer )
{
	uint32_t startWaitTimeMs = GetTimeMs();

	while( IsTxUartQueueFull() != false )
	{
		// Wait for Tx UART queue not to be full
		uint32_t waitTimeMs = DiffTimeFromStartTimeMs( startWaitTimeMs );
		if ( waitTimeMs > 20 ) // 20 ms
		{
			sprintf((char*)txUartErrorBuf, "UART Tx queue full for 20 ms\r\n");
			HAL_UART_Transmit(&debugUart, txUartErrorBuf, strlen((char*)txUartErrorBuf), 0x200);
			return; // Wait too long for Tx UART queue not to be full
		}

		// Keep track of max wait time for Tx UART Queue not to be full
		if( waitTimeMs > maxUartTxQueueWaitTimeMs )
		{
			maxUartTxQueueWaitTimeMs = waitTimeMs;
		}
	}

	// Write buffer data to Tx UART queue
	strncpy((char *)&uartTxQueue[uartTxQueueWriteIndex], (char *)buffer, strlen((char *)buffer));

	// Increment UART Tx queue write index
	uartTxQueueWriteIndex++;
	if( uartTxQueueWriteIndex >= UART_TX_QUEUE_SIZE )
	{
		// Write index was at maximum, set to 0
		uartTxQueueWriteIndex =  0;
	}

	return;
}


/*-----------------------------------------------------------------------------
 * Function:    UartPrintf
 *
 * Description: Converts string to a char buffer and writes buffer
 *              to Tx UART queue and reads Tx UART queue is UART
 *              is ready. Appends a prompt.
 *
 * Parameters:  format - string that contains text to be printed
 *
 * Returns:     None
 *---------------------------------------------------------------------------*/
void UartPrintf(const char * format, ... )
{
	if( (GetNoAsyncPrintsFlag( ) != false) || (IsStopDebugPring() == true) )
	{
		return;
	}

	va_list ap;
	int n;

	// Put print string in buffer
	va_start(ap, format);
	// Max number of bytes is UART_TX_QUEUE_MAX_MSG_SIZE-2 to allow for append of "$ " prompt
	n = vsnprintf ((char*)printBuf, UART_TX_QUEUE_MAX_MSG_SIZE - 2, format, ap);
	va_end(ap);

	// Append command prompt to print string
	strcat((char *)printBuf,"$ ");

	if ( n == -1 )
	{
		printBuf[UART_TX_QUEUE_MAX_MSG_SIZE-1] = '\0';
	}

	// Write buffer to Tx UART queue
	WriteTxUartQueue( printBuf );

	// If UART is ready, read data from TX UART queue and transmit to UART,
	// else return and HAL_UART_TxCpltCallback will call ReadTxUArtQueue
	// after current UART transmission is complete
	if(debugUart.gState == HAL_UART_STATE_READY)
	{
		ReadTxUartQueueAndTransmit();
	}

	return;
}

/*-----------------------------------------------------------------------------
 * Function:    UartPrintfWithoutPrompt
 *
 * Description: Converts string to a char buffer and writes buffer
 *              to Tx UART queue and reads Tx UART queue is UART
 *              is ready. Does not append a prompt.
 *
 * Parameters:  format - string that contains text to be printed
 *
 * Returns:     None
 *---------------------------------------------------------------------------*/
void UartPrintfWithoutPrompt(const char * format, ... )
{
	if( GetNoAsyncPrintsFlag( ) != false )
	{
		return;
	}

	va_list ap;
	int n;

	// Put print string in buffer
	va_start(ap, format);
	n = vsnprintf ((char*)printBuf, UART_TX_QUEUE_MAX_MSG_SIZE, format, ap);
	va_end(ap);

	// If string is empty, nothing to print so return
	if( n == 0 )
	{
		return;
	}

	if ( n == -1 )
	{
		printBuf[UART_TX_QUEUE_MAX_MSG_SIZE-1] = '\0';
	}

	// Write buffer to Tx UART queue
	WriteTxUartQueue( printBuf );

	// If UART is ready, read data from TX UART queue and transmit to UART,
	// else return and HAL_UART_TxCpltCallback will call ReadTxUArtQueue
	// after current UART transmission is complete
	if(debugUart.gState == HAL_UART_STATE_READY)
	{
		ReadTxUartQueueAndTransmit();
	}

	return;
}

void GuiUartPrintf(const char * format, ... )
{
	va_list ap;
	int n;

	// Put print string in buffer
	va_start(ap, format);
	n = vsnprintf ((char*)guiPrintBuf, UART_TX_QUEUE_MAX_MSG_SIZE, format, ap);
	va_end(ap);

	// If string is empty, nothing to print so return
	if( n == 0 )
	{
		return;
	}

	if ( n == -1 )
	{
		guiPrintBuf[UART_TX_QUEUE_MAX_MSG_SIZE-1] = '\0';
	}

	// If UART is ready transmit to UART,

	if(guiUart.gState == HAL_UART_STATE_READY)
	{
		// Transmit buffer data from Tx UART queue to UART via IT
		if( HAL_UART_Transmit_IT(&guiUart, guiPrintBuf, n ) != HAL_OK )
		{
			sprintf((char*)guiPrintBuf, "UART Transmit ERROR in ReadTxUartQueueAndTransmit\n");
			HAL_UART_Transmit(&guiUart, guiPrintBuf, strlen((char*)guiPrintBuf), 0x200);
		}
	}

	return;
}
/*-----------------------------------------------------------------------------
 * Function:    PmUartPrintf
 *
 * Description: Converts string to a char buffer and writes buffer
 *              to Tx UART queue and reads Tx UART queue is UART
 *              is ready. Appends a prompt.
 *
 * Parameters:  format - string that contains text to be printed
 *
 * Returns:     None
 *---------------------------------------------------------------------------*/
void PmUartPrintf(const char * format, ... )
{
	va_list ap;
	int n;

	// Put print string in buffer
	va_start(ap, format);
	// Max number of bytes is UART_TX_QUEUE_MAX_MSG_SIZE-2 to allow for append of "$ " prompt
	n = vsnprintf ((char*)printBuf, UART_TX_QUEUE_MAX_MSG_SIZE - 2, format, ap);
	va_end(ap);

	// Append command prompt to print string
	strcat((char *)printBuf,"$ ");

	if ( n == -1 )
	{
		printBuf[UART_TX_QUEUE_MAX_MSG_SIZE-1] = '\0';
	}

	// Write buffer to Tx UART queue
	WriteTxUartQueue( printBuf );

	// If UART is ready, read data from TX UART queue and transmit to UART,
	// else return and HAL_UART_TxCpltCallback will call ReadTxUArtQueue
	// after current UART transmission is complete
	if(debugUart.gState == HAL_UART_STATE_READY)
	{
		ReadTxUartQueueAndTransmit();
	}

	return;
}

/*-----------------------------------------------------------------------------
 * Function:    PmUartPrintfWithoutPrompt
 *
 * Description: Converts string to a char buffer and writes buffer
 *              to Tx UART queue and reads Tx UART queue is UART
 *              is ready. Does not append a prompt.
 *
 * Parameters:  format - string that contains text to be printed
 *
 * Returns:     None
 *---------------------------------------------------------------------------*/
void PmUartPrintfWithoutPrompt(const char * format, ... )
{
	va_list ap;
	int n;

	// Put print string in buffer
	va_start(ap, format);
	n = vsnprintf ((char*)printBuf, UART_TX_QUEUE_MAX_MSG_SIZE, format, ap);
	va_end(ap);

	// If string is empty, nothing to print so return
	if( n == 0 )
	{
		return;
	}

	if ( n == -1 )
	{
		printBuf[UART_TX_QUEUE_MAX_MSG_SIZE-1] = '\0';
	}

	// Write buffer to Tx UART queue
	WriteTxUartQueue( printBuf );

	// If UART is ready, read data from TX UART queue and transmit to UART,
	// else return and HAL_UART_TxCpltCallback will call ReadTxUArtQueue
	// after current UART transmission is complete
	if(debugUart.gState == HAL_UART_STATE_READY)
	{
		ReadTxUartQueueAndTransmit();
	}

	return;
}

// Returns a millisecond count based on the microsecond timer
// For use in callbacks where HAL_GetTick() might not increment due to interrupt priority
// e.g. where UartPrintf is called in a callback
uint32_t GetTimeMs(void)
{
	return __HAL_TIM_GetCounter(&htim5)/1000;
}

// Calculate current difference in time in ms from startTimeMs
uint32_t DiffTimeFromStartTimeMs( uint32_t startTimeMs )
{
	int64_t diffTimeMs = (__HAL_TIM_GetCounter(&htim5)/1000) - startTimeMs;

	if( diffTimeMs < 0 )
	{
		diffTimeMs += 0xFFFFFFFF;
	}

	uint32_t diffTimeMsUint32 = diffTimeMs;

	return diffTimeMsUint32;
}

//Provides a tick value in millisecond.
uint32_t Get_SysTick(void)
{
	return HAL_GetTick();
}
