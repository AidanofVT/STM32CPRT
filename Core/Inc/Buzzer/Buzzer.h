/*
 * buzzer.h
 *
 *  Created on: Jun 16, 2020
 */

#ifndef INC_BUZZER_BUZZER_H_
#define INC_BUZZER_BUZZER_H_

#include "stdint.h"
#include "stdbool.h"

typedef enum {
   BUZZER_TONE_OFF,
   BUZZER_TONE_MED,
   BUZZER_TONE_HIGH,

} BUZZER_TONE_TYPE;

typedef enum {
    eBuzzerState_Inactive = 0,
	eBuzzerState_Initiate,
	eBuzzerState_BeepOn,
	eBuzzerState_BeepOff,
	eBuzzerState_BeepSpace,
	eBuzzerState_BeepOn2,
	eBuzzerState_BeepOff2,
	eBuzzerState_PauseBetweenBuzzer,
} BuzzerState_t;

void    BUZZER_Init(void);
void    BUZZER_Run(void);
void    BUZZER_EnableSound(bool bOnOff);
void    BUZZER_Request(BUZZER_TONE_TYPE eBuzzerType, uint16_t wOnTimeMs, uint16_t wOffTimeMs,
		               uint16_t wWaitBetweenBeepMs, uint16_t wWaitNextSequenceMs);

#endif /* INC_BUZZER_BUZZER_H_ */
