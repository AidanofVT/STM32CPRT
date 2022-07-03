
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_LED_LED_H_
#define INC_LED_LED_H_

#include "stdint.h"

typedef enum
{
	LED_SET_OFF,
	LED_SET_ON,
	LED_SET_BLINK,
} LED_SET;

typedef enum
{
	LED_NONE,
	LED_STM_RED,
	LED_STM_GREEN,
	LED_STM_YELLOW,
	LED_STM_BAT_25,
	LED_STM_BAT_50,
	LED_STM_BAT_75,
	LED_STM_BAT_100,
	LED_MAX_START,
	LED_MAX_PAUSE,

	LED_COUNT,
} LED_ID;

void    LED_Set(LED_ID eLedID, LED_SET eLedSet, uint16_t wtOnInMs, uint16_t wtOffInMs);
void    LED_Run(void);

#endif /* INC_LED_LED_H_ */
