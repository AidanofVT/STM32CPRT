/*
 * Model.h
 *
 */

#ifndef MODEL_MODEL_H_
#define MODEL_MODEL_H_
#include "stdbool.h"

typedef struct
{
	bool    bSystemEnterRunMode;
	bool    bSystemEnterStanbyMode;
	bool    bLCDResetRequest;
	bool    bLCDRefreshRequest;
	bool    bKeyCodeReady;
	bool    bDisplayTime;
	bool    bPfcPowerWarnFail;
	bool    bPfcLoadEnable;
	bool    bMembranePress;
	bool    bPressureData;
	uint8_t gKeyCode;
	char    atime[10];
	char    adate[10];
	uint16_t dac_value;

} MODEL;

extern MODEL gstModel;

void Model_Init(void);

#endif /* MODEL_MODEL_H_ */

