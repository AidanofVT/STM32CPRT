
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DISPLAY_H
#define __DISPLAY_H

#include <Buzzer.h>
#include <Led.h>
#include "stdbool.h"

typedef enum
{
    DISPLAY_ID_NONE,
	DISPLAY_ID_STANDBY,
	DISPLAY_ID_MUTE,
	DISPLAY_ID_START,
	DISPLAY_ID_MENU,
	DISPLAY_ID_PAUSE,
	DISPLAY_ID_LEFT,
	DISPLAY_ID_UP,
	DISPLAY_ID_DOWN,
	DISPLAY_ID_SEL,
	DISPLAY_ID_RIGHT,


    DISPLAY_ID_COUNT,

} DISPLAY_ID;

typedef enum
{
	SYSTEM_STATE_INITIALIZE,
	SYSTEM_STATE_POST,
	SYSTEM_STATE_STANDBY,
	SYSTEM_STATE_RUN,
	SYSTEM_STATE_COUNT,
} SYSTEM_STATE;

typedef struct
{
	LED_ID      ledId;
	uint16_t    ledOnTimeMs;
	uint16_t    ledOffTimeMs;
	BUZZER_TONE_TYPE buzzerTone;
	uint16_t    buzzOnTimeMs;
	uint16_t    buzzOffTimeMs;
	uint16_t    buzzWaitBetweenBeepMs;
	uint16_t    buzzWaitNextSeqMs;
} ALARM_INFO;

typedef enum
{
	DISPLAY_STATE_DISABLE,
	DISPLAY_STATE_IDLE,
	DISPLAY_STATE_ACTIVE,
	DISPLAY_STATE_ALARM,
	DISPLAY_STATE_COUNT,
} DISPLAY_STATE;

void Display_Init();
void Display_Manager();
void Display_Request_Refresh();

#endif /* __DISPLAY_H */
