

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SWITCHES_H
#define __SWITCHES_H

#include "stdbool.h"

#define MAX7370_ADR               (0x70)
#define MUTE_S1_KEYCODE           (0x00)
#define START_S2_KEYCODE          (0x08)
#define MENU_S3_KEYCODE           (0x10)
#define PAUSE_S4_KEYCODE          (0x01)
#define LEFT_S5_KEYCODE           (0x09)
#define UP_S6_KEYCODE             (0x11)
#define DOWN_S7_KEYCODE           (0x02)
#define SEL_S8_KEYCODE            (0x0A)
#define RIGHT_S9_KEYCODE          (0x12)


typedef enum
{
    SWITCH_ID_NONE,
	SWITCH_ID_MUTE,
	SWITCH_ID_START,
	SWITCH_ID_MENU,
	SWITCH_ID_PAUSE,
	SWITCH_ID_LEFT,
	SWITCH_ID_UP,
	SWITCH_ID_DOWN,
	SWITCH_ID_SEL,
	SWITCH_ID_RIGHT,

    SWITCH_ID_COUNT,

} SWITCH_ID;


typedef enum
{
    KEY_PRESS,
    KEY_HOLD_4_S,
	KEY_HOLD_2_S,
	KEY_FAIL,

} KEY_STATUS;

typedef void SWITCH;

SWITCH* Switches_Init(void);
bool Switches_Scan(void);
void Switches_Get_Status( SWITCH_ID * eSwitchId, KEY_STATUS * eKeyStatus );
void PressSwitch( SWITCH_ID switchId, KEY_STATUS keyStatus );
void Switches_Power_Onoff(bool bOnOff);
void LED_Start_On();
void LED_Start_Off();
void LED_Pause_On();
void LED_Pause_Off();
#endif /* __SWITCHES_H */
