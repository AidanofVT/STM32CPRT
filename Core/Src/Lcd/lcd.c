#include <lcd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_i2c.h"
#include "main.h"

#define  LCD_OFF_DELAY_MS    25
#define  LCD_ON_DELAY_MS     125

extern I2C_HandleTypeDef hi2c2;


typedef enum
{
	LCD_STATE_RESET_OFF,
	LCD_STATE_RESET_OFF_DELAY,
	LCD_STATE_RESET_ON,
	LCD_STATE_RESET_ON_DELAY,
} LCD_STATE_RESET;

typedef struct
{
	LCD_LINE  CurrentLine;
	uint8_t   CurrentCursor;
	uint32_t  TimeLastDisplay;
} CONTEXT;

static CONTEXT c;

uint8_t lcdTxQueue[LCD_TX_QUEUE_SIZE][MAX_NUM_CHARS_PER_LCD_LINE+1];
uint8_t lcdNumCharsTxQueue[LCD_TX_QUEUE_SIZE];
uint16_t lcdTxQueueReadIndex = 0;
uint16_t lcdTxQueueWriteIndex = 0;

uint32_t maxLcdTxQueueWaitTimeMs = 0;
uint16_t maxTxLcdQueueDepth = 0;
uint16_t curTxLcdQueueDepth = 0;

bool clearScreenWait = false;
uint8_t lcdTxBuf[MAX_NUM_CHARS_PER_LCD_LINE+1];

static void LCD_ClearScreen(void);
static void LCD_BlinkCursorOnOff(bool flag);


bool GetClearScreenWait( )
{
	return clearScreenWait;
}

void SetClearScreenWait( bool clearScreenWaitFlag )
{
	clearScreenWait = clearScreenWaitFlag;
}

/*-----------------------------------------------------------------------------
 * Function:    IsTxLcdQueueEmpty
 *
 * Description: Determines if Tx LCD queue is empty.
 *
 * Parameters:  None
 *
 * Returns:     true if empty, false if not empty
 *---------------------------------------------------------------------------*/
bool IsTxLcdQueueEmpty( )
{
	int16_t queueIndexDiff = lcdTxQueueWriteIndex - lcdTxQueueReadIndex;
	curTxLcdQueueDepth = queueIndexDiff;

	if( queueIndexDiff == 0 )
	{
		// Tx LCD queue is empty
		return true;
	}

	if( queueIndexDiff < 0 )
	{
		queueIndexDiff += LCD_TX_QUEUE_SIZE;
	}

	// Keep track of maximum Tx LCD queue depth
	if( queueIndexDiff > maxTxLcdQueueDepth )
	{
		maxTxLcdQueueDepth = queueIndexDiff;
	}

	// Tx LCD queue is not empty
	return false;
}

/*-----------------------------------------------------------------------------
 * Function:    IsTxLcdQueueFull
 *
 * Description: Determines if Tx LCD queue is full.
 *
 * Parameters:  None
 *
 * Returns:     true if full, false if not full
 *---------------------------------------------------------------------------*/
bool IsTxLcdQueueFull( )
{
	int16_t queueIndexDiff = lcdTxQueueWriteIndex - lcdTxQueueReadIndex;

	if( queueIndexDiff < 0 )
    {
		queueIndexDiff += LCD_TX_QUEUE_SIZE;
	}

	if( queueIndexDiff == (LCD_TX_QUEUE_SIZE - 1) )
	{
		// Tx LCD queue is full
		return true;
	}

	// Tx LCD queue is not full
	return false;
}

/*-----------------------------------------------------------------------------
 * Function:    ReadTxLcdQueueAndTransmit
 *
 * Description: If the Tx LCD queue is not empty, the queue entry pointed to
 *              by the queue read index is read and transmitted to the LCD I2C
 *              with IT and the queue read index is incremented
 *
 * Parameters:  None
 *
 * Returns:     None
 *---------------------------------------------------------------------------*/
void ReadTxLcdQueueAndTransmit()
{
	static bool suppresLcdTransmitError = false;

	if( IsTxLcdQueueEmpty() != false )
	{
		// Tx LCD queue is empty, nothing to read and transmit
		return;
	}

	if( clearScreenWait != false )
	{
		// Wait until clear screen wait time has elapsed before writing LCD line
		return;
	}

	if(HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
	{
		// LCD I2C bus is not ready yet
		UartPrintf("LCD I2C bus is NOT READY\n");
		return;
	}

	memset( lcdTxBuf, ' ', MAX_NUM_CHARS_PER_LCD_LINE+1 );

	// Read number of characters to write from LCD queue to LCD I2C
	uint8_t numChars = lcdNumCharsTxQueue[lcdTxQueueReadIndex];
	memcpy( lcdTxBuf, lcdTxQueue[lcdTxQueueReadIndex], numChars );

	memset(lcdTxQueue[lcdTxQueueReadIndex], ' ', MAX_NUM_CHARS_PER_LCD_LINE+1);

	if( lcdTxBuf[1] == 0x51 ) // Clear screen command, start wait to allow clear screen to finish before next write to LCD
	{
		clearScreenWait = true;
	}

	// Increment Tx LCD queue read index
	lcdTxQueueReadIndex++;
	if( lcdTxQueueReadIndex >= LCD_TX_QUEUE_SIZE )
	{
		// Read index was at maximum, set to 0
		lcdTxQueueReadIndex =  0;
	}

    // UartPrintf("LCD Write %2d chars: %s\n", numChars, lcdTxBuf);
	// Transmit data from Tx LCD queue to I2C via IT
    HAL_StatusTypeDef err = HAL_I2C_Master_Transmit_IT(&hi2c2, LCD_ADD, lcdTxBuf, numChars );
	if( err != HAL_OK )
	{
		if( suppresLcdTransmitError == false )
		{
			UartPrintf("LCD I2C IT Transmit ERROR in ReadTxLcdQueueAndTransmit %x\n", err);
			suppresLcdTransmitError = true;
		}
	}
	else
	{
		suppresLcdTransmitError = false;
	}
}

/*-----------------------------------------------------------------------------
 * Function:    WriteTxLcdQueue
 *
 * Description: When the Tx LCD queue is not full, the queue entry pointed to
 *              by the queue write index is written with the data in the passed
 *              buffer and the queue write index is incremented
 *
 * Parameters:  buffer - pointer to data to be written to Tx LCD queue
 *
 * Returns:     None
 *---------------------------------------------------------------------------*/
void WriteTxLcdQueue(uint8_t * buffer, uint8_t numChars )
{
	uint32_t startWaitTime = GetTimeMs();

	while( IsTxLcdQueueFull() != false )
	{
		// Wait for Tx LCD queue not to be full
		uint32_t waitTime = DiffTimeFromStartTimeMs(startWaitTime);
		if ( waitTime > 10 ) // 10 ms
		{
			// Comment out print so debug print won't get cluttered if no display is connected
			//UartPrintf("LCD I2C IT Tx queue full for 10 ms\n");
			return; // Wait too long for Tx I2C LCD queue not to be full
		}

		// Keep track of max wait time for Tx LCD Queue not to be full
		if( waitTime > maxLcdTxQueueWaitTimeMs )
		{
			maxLcdTxQueueWaitTimeMs = waitTime;
		}
	}

	if( numChars > MAX_NUM_CHARS_PER_LCD_LINE )
	{
		 UartPrintf("Number of LCD Line chars to write is %d. Truncating to %d  chars\n", numChars, MAX_NUM_CHARS_PER_LCD_LINE);
		numChars = MAX_NUM_CHARS_PER_LCD_LINE;
	}

	// Write buffer data to Tx LCD queue
	memcpy((char *)&lcdTxQueue[lcdTxQueueWriteIndex], (char *)buffer, numChars);
	lcdNumCharsTxQueue[lcdTxQueueWriteIndex] = numChars;

	// Increment LCD Tx queue write index
	lcdTxQueueWriteIndex++;
	if( lcdTxQueueWriteIndex >= LCD_TX_QUEUE_SIZE )
	{
		// Write index was at maximum, set to 0
		lcdTxQueueWriteIndex =  0;
	}

	// If I2C2 is ready, read data from TX LCD queue and write to LCD,
	// else return and HAL_I2C_MemTxCpltCallback will call ReadTxLcdQueueAndTransmit
	// after current I2C2 transmission is complete
	if( HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY )
	{
		ReadTxLcdQueueAndTransmit();
	}

	return;
}


static void LCD_ClearScreen(void)
{
	uint8_t clear_screen[2] = {0xFE, 0x51};
	WriteTxLcdQueue( clear_screen, 2 );
}

static void LCD_BlinkCursorOnOff(bool flag)
{
	uint8_t blink_cursor[2] = {0xFE, 0x00};
	blink_cursor[1] = (flag) ? 0x4B : 0x4C;
	WriteTxLcdQueue( blink_cursor, 2 );
}

void LCD_MoveCursor(LCD_LINE LineNum, uint8_t Position)
{
    uint8_t set_cursor[3] = {0xFE, 0x45, 0x00};
	//Line 1 0x00 0x13
    //Line 2 0x40 0x53
    //Line 3 0x14 0x27
    //Line 4 0x54 0x67
	//Position 0x00 to 0x13
	
	set_cursor[2] = (LineNum == LCD_LINE_4) ? 0x54 :
	                (LineNum == LCD_LINE_3) ? 0x14 :
	                (LineNum == LCD_LINE_2) ? 0x40 : 0x00;
	set_cursor[2] += Position;
	WriteTxLcdQueue( set_cursor, 3 );
}

static void LCD_SetBacklight( )
{
	uint8_t set_backlight[3] = {0xFE, 0x53, 0x08};
	WriteTxLcdQueue( set_backlight, 3 );
}

uint8_t LCD_Write(bool bFresh, LCD_LINE Line, uint8_t Position, char *msg, uint8_t NumChar)
{	
	uint8_t ret = 0;
	
	if(bFresh)
	{
	   LCD_ClearScreen();
	}
	
	LCD_MoveCursor(Line, Position);
	WriteTxLcdQueue( (uint8_t *)msg, NumChar );

	return ret;
}

LCD* LCD_Init(void)
{
	char msg[32];
	int len;


	lcdTxQueueReadIndex = 0;
	lcdTxQueueWriteIndex = 0;

	UartPrintf("Init LCD\n");

	HAL_Delay(10);
	LCD_SetBacklight( );

	len = snprintf(msg, sizeof(msg), "CPRT %s", GetSwVersionString( CURRENT_SW_VERSION_ID ));
	LCD_Write(true, LCD_LINE_1, 0, msg, len );

	LCD_BlinkCursorOnOff(false);

    return (LCD *)&c;
}

void LCD_Power_Onoff(bool bOnOff)
{
	if(bOnOff == true)
	{
		HAL_GPIO_WritePin(DISP_POWER_EN_GPIO_Port, DISP_POWER_EN_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(DISP_POWER_EN_GPIO_Port, DISP_POWER_EN_Pin, GPIO_PIN_RESET);
	}
}

void LCD_Power_Reset()
{
	HAL_GPIO_WritePin(DISP_POWER_EN_GPIO_Port, DISP_POWER_EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(DISP_POWER_EN_GPIO_Port, DISP_POWER_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(2);
}
