/*
 * buzzer.c
 *
 *  Created on: Jun 16, 2020
 */
#include <Buzzer.h>
#include <stdlib.h>
#include "stdbool.h"
#include "stm32f7xx_hal.h"
#include "main.h"
#include "Model.h"

extern TIM_HandleTypeDef htim2;


#define BUZZER_RUN_TIMER_PERIOD_MS    (5)
#define BUZZER_50_DUTY                (125)


typedef struct {
	BuzzerState_t     eBuzzerState;
	BUZZER_TONE_TYPE  eBuzzerType;
	uint8_t           wNumBeeps1;
	uint8_t           wNumBeeps2;
    uint16_t          wOnDuration5MsCounts;
    uint16_t          wOffDuration5MsCounts;
    uint16_t          wWaitBetweenBeepDuration5MsCounts;
    uint16_t          wWaitNextSequenceDuration5MsCounts;
} CONTEXT;

static CONTEXT c;
static CONTEXT *gstBuzzer;

/*
 * BUZZER_Request function to set up the buzzer
 *
 * eBuzzerType = BUZZER_TONE_MED/BUZZER_TONE_HIGH,
 * BUZZER_TONE_MED pattern is BBB wait and repeat,
 * BUZZER_TONE_HIGH pattern is BBB<space>BB wait and repeat
 *
 * wOnTimeMs Buzzer time in milliseconds from multiple of 5ms
 *
 * wOffTimeMs Buzzer time in milliseconds from multiple of 5ms
 *
 * wWaitBetweenBeepMs time wait for space from High priority alarms in milliseconds X numbers of 5ms
 *
 * wWaitNextSequenceMs time wait for next sequence start
 */

void BUZZER_Request(BUZZER_TONE_TYPE eBuzzerType, uint16_t wOnTimeMs, uint16_t wOffTimeMs,
		            uint16_t wWaitBetweenBeepMs, uint16_t wWaitNextSequenceMs)
{
	gstBuzzer->wOnDuration5MsCounts               = (wOnTimeMs / BUZZER_RUN_TIMER_PERIOD_MS);
	gstBuzzer->wOffDuration5MsCounts              = (wOffTimeMs / BUZZER_RUN_TIMER_PERIOD_MS);
	gstBuzzer->wWaitBetweenBeepDuration5MsCounts  = (wWaitBetweenBeepMs / BUZZER_RUN_TIMER_PERIOD_MS);
	gstBuzzer->wWaitNextSequenceDuration5MsCounts = (wWaitNextSequenceMs / BUZZER_RUN_TIMER_PERIOD_MS);
	gstBuzzer->eBuzzerType                        = eBuzzerType;

	switch(eBuzzerType)
	{
	    case BUZZER_TONE_OFF:
	    {
	    	gstBuzzer->eBuzzerState = eBuzzerState_Inactive;
	    	htim2.Instance->CCR4   = 0;
	    	gstBuzzer->wNumBeeps1   = 0;
	    	gstBuzzer->wNumBeeps2   = 0;
	    }
		break;

	    case BUZZER_TONE_MED:
	    {
	    	gstBuzzer->eBuzzerState = eBuzzerState_Initiate;
	    	gstBuzzer->wNumBeeps1   = 3;
	    	gstBuzzer->wNumBeeps2   = 0;
	    }
		break;

	    case BUZZER_TONE_HIGH:
	    {
	    	gstBuzzer->eBuzzerState = eBuzzerState_Initiate;
	    	gstBuzzer->wNumBeeps1   = 3;
	    	gstBuzzer->wNumBeeps2   = 2;
	    }
	    break;

	    default:
	    	gstBuzzer->eBuzzerState = eBuzzerState_Inactive;
		break;
	}
}


void BUZZER_Run(void)
{
	static uint16_t wWaitCounts = 0;
	static uint8_t  wBeepCounts = 0;

	switch(gstBuzzer->eBuzzerState)
	{
		case eBuzzerState_Inactive:
		break;

		//Set up timer for buzzer on then go to next
		//state to wait for the on time expire
		case eBuzzerState_Initiate:
			if( gstBuzzer->wNumBeeps1 && gstBuzzer->wOnDuration5MsCounts )
			{
				//get the time Buzzer need to be ON
				wWaitCounts             = gstBuzzer->wOnDuration5MsCounts;
				wBeepCounts             = gstBuzzer->wNumBeeps1;
				gstBuzzer->eBuzzerState = eBuzzerState_BeepOn;
				BUZZER_EnableSound( true );
			}
			else {
				gstBuzzer->eBuzzerState = eBuzzerState_Inactive;
			}
		break;

		case eBuzzerState_BeepOn:
			if( --wWaitCounts == 0 )
			{

				BUZZER_EnableSound( false );
				wBeepCounts--;

				if( gstBuzzer->wOffDuration5MsCounts ) {
					gstBuzzer->eBuzzerState = eBuzzerState_BeepOff;
					wWaitCounts = gstBuzzer->wOffDuration5MsCounts;
				} else {
					gstBuzzer->eBuzzerState = eBuzzerState_Inactive;
				}
			}
		break;

		case eBuzzerState_BeepOff:
			if( --wWaitCounts == 0 )
			{
				if( wBeepCounts == 0 )
				{
					BUZZER_EnableSound( false );
					// No more beeps to do, from 1st sequence
					if(gstBuzzer->eBuzzerType == BUZZER_TONE_MED)
					{
						//If Med Prior only 3 bbb is needed, go to wait for next cycle
						gstBuzzer->eBuzzerState = eBuzzerState_PauseBetweenBuzzer;
						wWaitCounts = gstBuzzer->wWaitNextSequenceDuration5MsCounts;
						//UartPrintf("Med Prior End Go to wait next Cycle state %d waitCnt %d\n", gstBuzzer->eBuzzerState, wWaitCounts);
					}
					else
					{
						gstBuzzer->eBuzzerState = eBuzzerState_BeepSpace;
						wWaitCounts = gstBuzzer->wWaitBetweenBeepDuration5MsCounts;
					}

				} else {
					gstBuzzer->eBuzzerState = eBuzzerState_BeepOn;
					wWaitCounts = gstBuzzer->wOffDuration5MsCounts;
				    BUZZER_EnableSound( true );
				}
			}
		break;

		case eBuzzerState_BeepSpace:
			if( --wWaitCounts == 0 )
			{
				if( gstBuzzer->wNumBeeps2 && gstBuzzer->wOnDuration5MsCounts )
				{
					//get the time Buzzer need to be ON
					wWaitCounts             = gstBuzzer->wOnDuration5MsCounts;
					wBeepCounts             = gstBuzzer->wNumBeeps2;
					gstBuzzer->eBuzzerState = eBuzzerState_BeepOn2;
					BUZZER_EnableSound( true );
				}
				else
				{
					gstBuzzer->eBuzzerState = eBuzzerState_Inactive;
				}
			}
		break;

		case eBuzzerState_BeepOn2:
			if( --wWaitCounts == 0 )
			{
				BUZZER_EnableSound( false );
				wBeepCounts--;

				if( gstBuzzer->wOffDuration5MsCounts ) {
					gstBuzzer->eBuzzerState = eBuzzerState_BeepOff2;
					wWaitCounts = gstBuzzer->wOffDuration5MsCounts;
				} else {
					gstBuzzer->eBuzzerState = eBuzzerState_Inactive;
				}
			}
		break;

		case eBuzzerState_BeepOff2:
			if( --wWaitCounts == 0 )
			{
				if( wBeepCounts == 0 )
				{
					BUZZER_EnableSound( false );
					// No more beeps to do, from 2 sequence.  Go to wait for next cycle
					gstBuzzer->eBuzzerState = eBuzzerState_PauseBetweenBuzzer;
					wWaitCounts = gstBuzzer->wWaitNextSequenceDuration5MsCounts;
				}
				else
				{
					gstBuzzer->eBuzzerState = eBuzzerState_BeepOn2;
					wWaitCounts = gstBuzzer->wOffDuration5MsCounts;
					BUZZER_EnableSound( true );
				}
			}
		break;

		case eBuzzerState_PauseBetweenBuzzer:
			if( --wWaitCounts == 0 )
			{
				//Wait for next cycle kick in
				gstBuzzer->eBuzzerState = eBuzzerState_Initiate;
			}
		break;

		default:
			gstBuzzer->eBuzzerState = eBuzzerState_Inactive;
		break;
	}
}

void BUZZER_Init(void)
{
	c.eBuzzerState = eBuzzerState_Inactive;
	gstBuzzer = &c;
}

void BUZZER_EnableSound(bool bOnOff)
{
	if(bOnOff)
	{
		//255 = 100%, 0 = 0%, should not set to 100% since it is all high
		htim2.Instance->CCR4 = BUZZER_50_DUTY;
	}
	else
	{
		//0 is buzzer off value
		htim2.Instance->CCR4 = 0;
	}
}



