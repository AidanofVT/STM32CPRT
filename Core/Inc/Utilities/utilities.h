
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _UTILITIES_H
#define _UTILITIES_H

#include <stdbool.h>
#include "stm32f7xx_hal.h"

#define UART_TX_QUEUE_SIZE 32
#define UART_TX_QUEUE_MAX_MSG_SIZE 256

#define ARRAY_LEN(_x)               (sizeof(_x) / sizeof(_x[0]))
#define ARRAY_SIZE(a)               (sizeof(a) / sizeof((a)[0]))

#define SEC_TO_TICK(_x)             ((_x) * 1000)
#define TICK_TO_SEC(_x)             ((_x) / 1000)

/*
#define uInt8         char
#define uInt16        short
#define uInt32        int
#define int32         int
#define WORD          short
#define DWORD         int
#define BYTE          char
*/

typedef enum
{
	PRINT_DISABLE_ALL = 0,

	PRINT_ENABLE_PRESS,
	PRINT_DISABLE_PRESS,

	PRINT_ENABLE_PFC,
	PRINT_DISABLE_PFC,

	PRINT_ILLEGAL_COMMAND
} PRINT_COMMAND;

#define MIN_PRINT_PERIOD_MS 5 // Minimum time between prints.
#define MIN_NUM_5MS_COUNTS (MIN_PRINT_PERIOD_MS / 5 ) // Minimum number of 5ms periods between prints allowed.

typedef enum
{
	PRINT_PRESS	  = 0,
	PRINT_PFC     = 1,

	PRINT_GROUPS
} PRINT_CODE;

typedef union
{
	struct
	{
		unsigned int
		printPress	: 1,
		printPfc    : 1,

		spare		: 30;
	} bits;

	unsigned int PrintEnableWord;
} PrintEnables;

uint32_t GetTimeMs(void);
uint32_t DiffTimeFromStartTimeMs( uint32_t startTimeMs );
uint32_t Get_SysTick(void);
void ftoa(float n, char* res, int afterpoint);

void UartPrintf(const char * format, ... );
void UartPrintfWithoutPrompt(const char * format, ... );
void PmUartPrintf(const char * format, ... );
void PmUartPrintfWithoutPrompt(const char * format, ... );
bool GetNoAsyncPrintsFlag( );
bool IsStopDebugPring();
void ReadTxUartQueueAndTransmit();
void InitUartTxQueue();
void PrintManagerInit();
int  PrintManager();

void GuiUartPrintf(const char * format, ... );

#endif /* _UTILITIES_H */
