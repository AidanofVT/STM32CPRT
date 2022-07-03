#include <lcd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_rtc.h"
#include "main.h"
#include "Display.h"
#include "Model.h"
#include "Switches.h"

#define SETTING_DISPLAY_IN_SEC (SEC_TO_TICK(20))

static void Display_Page(DISPLAY_ID eId);


static DISPLAY_ID prevDisplayId = DISPLAY_ID_NONE;
static SYSTEM_STATE systemState = SYSTEM_STATE_POST;

static bool bRequestDisplayRefresh = false;
static uint32_t screenUpdateWaitTimeSeconds = SETTING_DISPLAY_IN_SEC;

typedef struct
{
	uint32_t      TimeLastSwitchPress;
	bool          bSettingMonitorDisplay;
	uint32_t      TimeLastPageToggle;
	DISPLAY_STATE eDisplayState;
} CONTEXT;

static CONTEXT c;
static CONTEXT* gstDisplayManager;

void SetSystemState( SYSTEM_STATE state )
{
	systemState = state;
	UartPrintf("System State: %d\n", state);
}

void Display_Init(void)
{
	//Init switches
	Switches_Init();

    //Init LCD
	LCD_Init();

	//Init buzzer
	BUZZER_Init();

	c.bSettingMonitorDisplay = false;  //using for toggle Idle displays
    c.TimeLastSwitchPress    = 0;
    c.TimeLastPageToggle     = 0;
    c.eDisplayState          = DISPLAY_STATE_IDLE;

    SetSystemState( SYSTEM_STATE_INITIALIZE ); //To allow ATtiny alarm check
    //standbyStartTimeMinute = GetMinuteCount();
    //previousMinuteCount = standbyStartTimeMinute;
    //previousMinuteCountSysTick = Get_SysTick();
    prevDisplayId = DISPLAY_ID_STANDBY;

    gstDisplayManager = &c;

}

SYSTEM_STATE GetSystemState( )
{
    return systemState;
}

void Display_Manager(void)
{
	if( Switches_Scan() == true)
	{
		// Switch Press
		gstDisplayManager->TimeLastSwitchPress = Get_SysTick();

		SWITCH_ID eSwitchId;
		KEY_STATUS eKeyStatus;
		DISPLAY_ID displayId;

		Switches_Get_Status(&eSwitchId, &eKeyStatus);

		switch(eSwitchId)
		{
		    case SWITCH_ID_MUTE:
		    	displayId = DISPLAY_ID_MUTE;
			break;

			case SWITCH_ID_START:
			{
				displayId = DISPLAY_ID_START;
				LED_Start_On();
			}
			break;

			case SWITCH_ID_MENU:
				displayId = DISPLAY_ID_MENU;
			break;

			case SWITCH_ID_PAUSE:
			{
				displayId = DISPLAY_ID_PAUSE;
				LED_Pause_On();
			}
			break;

			case SWITCH_ID_LEFT:
				displayId = DISPLAY_ID_LEFT;
			break;

			case SWITCH_ID_UP:
				displayId = DISPLAY_ID_UP;
			break;

			case SWITCH_ID_DOWN:
				displayId = DISPLAY_ID_DOWN;
			break;

			case SWITCH_ID_SEL:
			{
				displayId = DISPLAY_ID_SEL;
				LED_Pause_Off();
				LED_Start_Off();
			}
			break;

			case SWITCH_ID_RIGHT:
				displayId = DISPLAY_ID_RIGHT;
			break;

			case SWITCH_ID_NONE:
			case SWITCH_ID_COUNT:
			default:
				break;
		}

		Display_Page(displayId);
		prevDisplayId = displayId;
	}


	// If screen timeout occurred, go to display idle state
	if( gstDisplayManager->TimeLastSwitchPress &&
		((Get_SysTick() - gstDisplayManager->TimeLastSwitchPress) >= screenUpdateWaitTimeSeconds) )
	{
		gstDisplayManager->TimeLastSwitchPress = 0;
		Display_Page(DISPLAY_ID_STANDBY);
		prevDisplayId = DISPLAY_ID_STANDBY;
	}

    //Running LED
    LED_Run();

    //Running Buzzer
    BUZZER_Run();

    //Refresh Display after LCD toggle
    if(bRequestDisplayRefresh == true)
    {
    	bRequestDisplayRefresh = false;
    	Display_Page( prevDisplayId );
    }

}

static void Display_Standby_Page(void)
{
	char msg[32];
	int len;

	// Line 1
	LCD_Write(true,   LCD_LINE_1, 0, "   CPRT IDLE PAGE   ", 20);

	// Line 2
	len = snprintf(msg, sizeof(msg), "  SW REV: %s", GetSwVersionString( CURRENT_SW_VERSION_ID));
	LCD_Write(false, LCD_LINE_2, 0, msg, len );

	len = snprintf(msg, sizeof(msg), "%s %s", gstModel.adate,  gstModel.atime);

	// Line 4 Display Date and Time from RTC
	LCD_Write(false, LCD_LINE_4, 0, msg, len );

}

static void Display_Mute_Page(void)
{
	// Line 1
	LCD_Write(true,   LCD_LINE_1, 0, "     BUTTON ACTIVE  ", 20);

	// Line 2
	LCD_Write(false, LCD_LINE_2, 0,  "BUTTON MUTE PRESSED ", 20);
}

static void Display_Start_Page(void)
{
	// Line 1
	LCD_Write(true,   LCD_LINE_1, 0, "     BUTTON ACTIVE  ", 20);

	// Line 2
	LCD_Write(false, LCD_LINE_2, 0,  "BUTTON START PRESSED", 20);
}

static void Display_Menu_Page(void)
{
	// Line 1
	LCD_Write(true,   LCD_LINE_1, 0, "     BUTTON ACTIVE  ", 20);

	// Line 2
	LCD_Write(false, LCD_LINE_2, 0,  "BUTTON MENU PRESSED ", 20);
}

static void Display_Pause_Page(void)
{
	// Line 1
	LCD_Write(true,   LCD_LINE_1, 0, "     BUTTON ACTIVE  ", 20);

	// Line 2
	LCD_Write(false, LCD_LINE_2, 0,  "BUTTON PAUSE PRESSED", 20);
}

static void Display_Left_Page(void)
{
	// Line 1
	LCD_Write(true,   LCD_LINE_1, 0, "     BUTTON ACTIVE  ", 20);

	// Line 2
	LCD_Write(false, LCD_LINE_2, 0,  "BUTTON LEFT PRESSED ", 20);
}

static void Display_Up_Page(void)
{
	// Line 1
	LCD_Write(true,   LCD_LINE_1, 0, "     BUTTON ACTIVE  ", 20);

	// Line 2
	LCD_Write(false, LCD_LINE_2, 0,  "  BUTTON UP PRESSED ", 20);
}

static void Display_Down_Page(void)
{
	// Line 1
	LCD_Write(true,   LCD_LINE_1, 0, "     BUTTON ACTIVE  ", 20);

	// Line 2
	LCD_Write(false, LCD_LINE_2, 0,  "BUTTON DOWN PRESSED ", 20);
}

static void Display_Sel_Page(void)
{
	// Line 1
	LCD_Write(true,   LCD_LINE_1, 0, "     BUTTON ACTIVE  ", 20);

	// Line 2
	LCD_Write(false, LCD_LINE_2, 0,  "BUTTON SEL PRESSED  ", 20);
}

static void Display_Right_Page(void)
{
	// Line 1
	LCD_Write(true,   LCD_LINE_1, 0, "     BUTTON ACTIVE  ", 20);

	// Line 2
	LCD_Write(false, LCD_LINE_2, 0,  "BUTTON RIGHT PRESSED", 20);
}

static void (*dispF[])(void) =
{
		NULL,						// DISPLAY_ID_NONE
		Display_Standby_Page,		// DISPLAY_ID_STANDBY
		Display_Mute_Page,
		Display_Start_Page,
		Display_Menu_Page,
		Display_Pause_Page,
		Display_Left_Page,
		Display_Up_Page,
		Display_Down_Page,
		Display_Sel_Page,
		Display_Right_Page,

};

static void Display_Page(DISPLAY_ID eId)
{
	if ( (eId > DISPLAY_ID_NONE) && (eId < DISPLAY_ID_COUNT) )
	{
		dispF[eId]();
	}
}

void Display_Update_TimeLastSwitchPress(void)
{
	gstDisplayManager->TimeLastSwitchPress = Get_SysTick();
}

void Display_Request_Refresh()
{
    //delayWd100ms( 1 );
	bRequestDisplayRefresh = true;
}
