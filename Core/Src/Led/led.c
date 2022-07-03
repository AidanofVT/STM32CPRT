/*
 * led.c
 *
 *  Created on: Jun 16, 2020
 */
#include <led.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "Switches.h"

typedef void (*FpLedOn) (void);
typedef void (*FpLedOff) (void);

typedef enum
{
	BLINK_STATUS_DISABLE,
	BLINK_STATUS_ON,
	BLINK_STATUS_OFF
} BLINK_STATUS;

typedef struct
{
	LED_ID             eId;
	bool               bOn;
	bool               bOff;
	bool               bBlink;
    uint32_t           wLastTimeOn;
    uint32_t           wLastTimeOff;
    uint16_t           wtimeOn;
    uint16_t           wtimeOff;
    BLINK_STATUS       eBlinkStatus;
    GPIO_TypeDef*      ptrPort;
    uint16_t           pin;
    FpLedOn            pFunctionLedOn;
    FpLedOff           pFunctionLedOff;
}LED_ITEM_t;


LED_ITEM_t astLedSet[] =
{                       //        on    off  blink   LastTimeOn  LastTimeOff wtimeOn wtimeOff   eBlinkStaus
	[LED_NONE       ] = { LED_NONE       , false, false,  false,   0,          0,         0,        0,     BLINK_STATUS_DISABLE ,                     NULL,                0 ,  NULL                , NULL                      },
    [LED_STM_RED    ] = { LED_STM_RED    , false, false,  false,   0,          0,         0,        0,     BLINK_STATUS_OFF     ,   LED_R_GPIO_Port       ,   LED_R_Pin      ,  NULL                , NULL                      },
	[LED_STM_GREEN  ] = { LED_STM_GREEN  , false, false,  false,   0,          0,         0,        0,     BLINK_STATUS_OFF     ,   LED_G_GPIO_Port       ,   LED_G_Pin      ,  NULL                , NULL                      },
    [LED_STM_YELLOW ] = { LED_STM_YELLOW , false, false,  false,   0,          0,         0,        0,     BLINK_STATUS_OFF     ,   LED_Y_GPIO_Port       ,   LED_Y_Pin      ,  NULL                , NULL                      },
    [LED_STM_BAT_25 ] = { LED_STM_BAT_25 , false, false,  false,   0,          0,         0,        0,     BLINK_STATUS_OFF     ,   LED_BAT25_GPIO_Port   ,   LED_BAT25_Pin  ,  NULL                , NULL                      },
    [LED_STM_BAT_50 ] = { LED_STM_BAT_50 , false, false,  false,   0,          0,         0,        0,     BLINK_STATUS_OFF     ,   LED_BAT50_GPIO_Port   ,   LED_BAT50_Pin  ,  NULL                , NULL                      },
    [LED_STM_BAT_75 ] = { LED_STM_BAT_75 , false, false,  false,   0,          0,         0,        0,     BLINK_STATUS_OFF     ,   LED_BAT75_GPIO_Port   ,   LED_BAT75_Pin  ,  NULL                , NULL                      },
    [LED_STM_BAT_100] = { LED_STM_BAT_100, false, false,  false,   0,          0,         0,        0,     BLINK_STATUS_OFF     ,   LED_BAT100_GPIO_Port  ,   LED_BAT100_Pin ,  NULL                , NULL                      },
    [LED_MAX_START  ] = { LED_MAX_START  , false, false,  false,   0,          0,         0,        0,     BLINK_STATUS_OFF     ,                     NULL,                0 , (FpLedOn)LED_Start_On, (FpLedOff)LED_Start_Off   },
    [LED_MAX_PAUSE  ] = { LED_MAX_PAUSE  , false, false,  false,   0,          0,         0,        0,     BLINK_STATUS_OFF     ,                     NULL,                0 , (FpLedOn)LED_Pause_On, (FpLedOff)LED_Pause_Off   },
};

void LED_Set(LED_ID eLedID, LED_SET eLedSet, uint16_t wtOnInMs, uint16_t wtOffInMs)
{
	if( eLedID == LED_COUNT )
	{
		int  index;

		for(index = 0; index < ARRAY_LEN(astLedSet); index++)
		{
			switch(eLedSet)
			{
				case LED_SET_OFF:
				{
					astLedSet[index].bOff   = true;
					astLedSet[index].bBlink = false;
					//UartPrintf("Set Off:%d\n", index);
				}
				break;

				case LED_SET_ON:
				{
					astLedSet[index].bOn = true;
					astLedSet[index].bBlink = false;
					//UartPrintf("Set On:%d\n", index);
				}
				break;

				case LED_SET_BLINK:
				{

					if((astLedSet[index].eBlinkStatus != BLINK_STATUS_DISABLE) &&
					   (astLedSet[index].bBlink == false))
					{
						//UartPrintf("Blink:%d On:%d Off:%d\n", index, wtOnInMs, wtOffInMs);
						astLedSet[index].bBlink       = true;
						astLedSet[index].wtimeOn      = wtOnInMs;
						astLedSet[index].wtimeOff     = wtOffInMs;
						astLedSet[index].wLastTimeOn  = Get_SysTick();
						astLedSet[index].wLastTimeOff = 0;
						astLedSet[index].eBlinkStatus = BLINK_STATUS_ON;
					}
				}
				break;

				default:
				break;
			}
		}
	}
	else
	{
		switch(eLedSet)
		{
			case LED_SET_OFF:
			{
				astLedSet[eLedID].bOff   = true;
				astLedSet[eLedID].bBlink = false;
				//UartPrintf("Set Off:%d\n", eLedID);
			}
			break;

			case LED_SET_ON:
			{
				astLedSet[eLedID].bOn = true;
				astLedSet[eLedID].bBlink = false;
				//UartPrintf("Set On:%d\n", eLedID);
			}
			break;

			case LED_SET_BLINK:
			{

				if((astLedSet[eLedID].eBlinkStatus != BLINK_STATUS_DISABLE) &&
				   (astLedSet[eLedID].bBlink == false))
				{
					//UartPrintf("Blink:%d On:%d Off:%d\n", eLedID, wtOnInMs, wtOffInMs);
					astLedSet[eLedID].bBlink       = true;
					astLedSet[eLedID].wtimeOn      = wtOnInMs;
					astLedSet[eLedID].wtimeOff     = wtOffInMs;
					astLedSet[eLedID].wLastTimeOn  = Get_SysTick();
					astLedSet[eLedID].wLastTimeOff = 0;
					astLedSet[eLedID].eBlinkStatus = BLINK_STATUS_ON;
				}
			}
			break;

			default:
			break;
		}
	}
}

void LED_Run(void)
{
	int  index;

	//OFF Handle the on LED, one at the time
	for(index = 0; index < ARRAY_LEN(astLedSet); index++)
	{
		if(astLedSet[index].bOff == true)
		{
			astLedSet[index].bOff = false;

			if( astLedSet[index].ptrPort != NULL )
		    {
				//If the LEDs controlling from STM32 toggle the GPIO
                HAL_GPIO_WritePin(astLedSet[index].ptrPort, astLedSet[index].pin, GPIO_PIN_RESET);
			}
			else if( astLedSet[index].pFunctionLedOff != NULL )
			{
				//Otherwise call the call back function
				astLedSet[index].pFunctionLedOff();
			}
		}
	}

	//ON Handle the on LED, one at the time
	for(index = 0; index < ARRAY_LEN(astLedSet); index++)
	{
		if(astLedSet[index].bOn == true)
		{
            astLedSet[index].bOn = false;

			if( astLedSet[index].ptrPort != NULL )
		    {
				//If the LEDs controlling from STM32 toggle the GPIO
                HAL_GPIO_WritePin(astLedSet[index].ptrPort, astLedSet[index].pin, GPIO_PIN_SET);
			}
			else if( astLedSet[index].pFunctionLedOff != NULL )
			{
				//Otherwise call the call back function
				astLedSet[index].pFunctionLedOn();
			}
		}
	}

	//BLINK Handle the blink LED for all the LEDs that have this feature available
	for(index = 0; index < ARRAY_LEN(astLedSet); index++)
	{
		if((astLedSet[index].eBlinkStatus != BLINK_STATUS_DISABLE) &&
		   (astLedSet[index].bBlink == true))
		{
			switch(astLedSet[index].eBlinkStatus)
			{
				case  BLINK_STATUS_ON:
				{
					if( (Get_SysTick() - astLedSet[index].wLastTimeOn) >= astLedSet[index].wtimeOn)
					{
						//On time is expired turn off the LED
						astLedSet[index].wLastTimeOff = Get_SysTick(); //save the start of off time
						astLedSet[index].eBlinkStatus = BLINK_STATUS_OFF;

						if( astLedSet[index].ptrPort != NULL )
					    {
							//If the LEDs controlling from STM32 toggle the GPIO
			                HAL_GPIO_WritePin(astLedSet[index].ptrPort, astLedSet[index].pin, GPIO_PIN_RESET);
						}
						else if( astLedSet[index].pFunctionLedOff != NULL )
						{
							//Otherwise call the call back function
							astLedSet[index].pFunctionLedOff();
						}

					}
				}
				break;

				case  BLINK_STATUS_OFF:
				{
					if( (Get_SysTick() - astLedSet[index].wLastTimeOff) >= astLedSet[index].wtimeOff)
					{
						//Off time is expired turn on the LED
						astLedSet[index].wLastTimeOn  = Get_SysTick(); //save the start of On time
						astLedSet[index].eBlinkStatus = BLINK_STATUS_ON;

						if( astLedSet[index].ptrPort != NULL )
					    {
							//If the LEDs controlling from STM32 toggle the GPIO
			                HAL_GPIO_WritePin(astLedSet[index].ptrPort, astLedSet[index].pin, GPIO_PIN_SET);
						}
						else if( astLedSet[index].pFunctionLedOff != NULL )
						{
							//Otherwise call the call back function
							astLedSet[index].pFunctionLedOn();
						}
					}
				}
				break;

				default:
				break;
			}
		}
	}

}


