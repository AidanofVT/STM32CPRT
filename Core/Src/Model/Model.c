/*
 * Model.c
 *
 */

#include <string.h>
#include "main.h"
#include "Model.h"

//Main model collection of all global parameters
MODEL gstModel;

void Model_Init(void)
{
    gstModel.bKeyCodeReady     = false;
    gstModel.bLCDResetRequest  = false;
    gstModel.dac_value         = 2048; //1.5V
    gstModel.bDisplayTime      = false;
    gstModel.bPfcLoadEnable    = false;
    gstModel.bPfcPowerWarnFail = false;
    gstModel.bMembranePress    = false;
}
