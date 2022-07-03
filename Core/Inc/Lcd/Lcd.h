
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_H
#define __LCD_H

#include "stdint.h"
#include "stdbool.h"

#define LCD_ADD                    (0x50)
#define LCD_TX_QUEUE_SIZE          (16)
#define MAX_NUM_CHARS_PER_LCD_LINE (20)

typedef enum
{
	LCD_LINE_1,
	LCD_LINE_2,
	LCD_LINE_3,
	LCD_LINE_4,
	LCD_LINE_COUNT,
} LCD_LINE;


typedef void LCD;
LCD*    LCD_Init(void);
uint8_t LCD_Write(bool bFresh, LCD_LINE Line, uint8_t Position, char *msg, uint8_t NumChar);
void    ReadTxLcdQueueAndTransmit();
bool    GetClearScreenWait( );
void    SetClearScreenWait( bool clearScreenWaitFlag );
void    WriteTxLcdQueue(uint8_t * buffer, uint8_t numChars );
void    LCD_MoveCursor(LCD_LINE LineNum, uint8_t Position);
void    LCD_Hardware_Manager();
void    LCD_Power_Onoff(bool bOnOff);

#endif /* __LCD_H */
