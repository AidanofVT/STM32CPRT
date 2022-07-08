/*
 * cprtfunctions.c
 *
 *  Created on: August 25, 2021
 *      Author: GiovanniG
 */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : cprtfunctions.c
  * @brief          : Main CPRT Functions
  * @Author			: Giovann`i Gonzalez
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "main.h"
#include "Model.h"
#include "utilities.h"
#include "hw.h"
#include "lcd.h"

extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim14;

uint32_t g_currentDACVal = 0;

/*                                                                          _____
SynchOut();		Generates a single pulse 100mS duration  High-Low-High ____|     |____  */
void SynchOut(bool status)
{
	HAL_GPIO_WritePin(GPIOB, SYNC_OUT_Pin, status);
}

/*
  * @brief VALVE Control Functions
  * @param None
  * @retval None
  */
void valve_control(uint8_t valvename, bool status)
{
	switch(valvename){
		case 1 :
			HAL_GPIO_WritePin(GPIOG, VALVE_1A_CTRL_Pin, status);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOG, VALVE_1B_CTRL_Pin, status);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOG, VALVE_2A_CTRL_Pin, status);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOG, VALVE_2B_CTRL_Pin, status);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOG, VALVE_3A_CTRL_Pin, status);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOG, VALVE_3B_CTRL_Pin, status);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOG, VALVE_4A_CTRL_Pin, status);
			break;
		case 8:
			HAL_GPIO_WritePin(GPIOG, VALVE_4B_CTRL_Pin, status);
			break;
	}
}

void Valve_1A_On()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_1A_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
}

void Valve_1A_Off()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_1A_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
}

void Valve_1B_On()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_1B_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

void Valve_1B_Off()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_1B_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

void Valve_2A_On()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_2A_CTRL_Pin, GPIO_PIN_SET);
}

void Valve_2A_Off()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_2A_CTRL_Pin, GPIO_PIN_RESET);
}

void Valve_2B_On()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_2B_CTRL_Pin, GPIO_PIN_SET);
}

void Valve_2B_Off()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_2B_CTRL_Pin, GPIO_PIN_RESET);
}

void Valve_3A_On()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_3A_CTRL_Pin, GPIO_PIN_SET);
}

void Valve_3A_Off()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_3A_CTRL_Pin, GPIO_PIN_RESET);
}

void Valve_3B_On()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_3B_CTRL_Pin, GPIO_PIN_SET);
}

void Valve_3B_Off()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_3B_CTRL_Pin, GPIO_PIN_RESET);
}

void Valve_4A_On()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_4A_CTRL_Pin, GPIO_PIN_SET);
}

void Valve_4A_Off()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_4A_CTRL_Pin, GPIO_PIN_RESET);
}

void Valve_4B_On()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_4B_CTRL_Pin, GPIO_PIN_SET);
}

void Valve_4B_Off()
{
	HAL_GPIO_WritePin(GPIOG, VALVE_4B_CTRL_Pin, GPIO_PIN_RESET);
}


// Battery LED Control
// Syntaxis example:  batled(LedBat25, on/off)

void batleds_control(uint8_t batled, bool status)
{
	switch(batled){
	case 1 :
			HAL_GPIO_WritePin(GPIOD, LED_BAT25_Pin, status);
			break;
	case 2:
			HAL_GPIO_WritePin(GPIOD, LED_BAT50_Pin, status);
			break;
	case 3:
			HAL_GPIO_WritePin(GPIOD, LED_BAT75_Pin, status);
			break;
	case 4:
			HAL_GPIO_WritePin(GPIOD, LED_BAT100_Pin, status);
			break;

	}

}

// Alarm LED Control
// Syntaxis example:  alarmled yellow on/off
void alarmleds_control(uint8_t alarmled, bool status)
{
	switch(alarmled)
	{
	    case 1 :
		HAL_GPIO_WritePin(GPIOD, LED_Y_Pin, status);
		break;

	    case 2:
		HAL_GPIO_WritePin(GPIOD, LED_G_Pin, status);
		break;

	    case 3:
		HAL_GPIO_WritePin(GPIOD, LED_R_Pin, status);
		break;
	}
}

void compressor_control(bool status)
{
	HAL_GPIO_WritePin(GPIOE, COMPRESSOR_CTRL_Pin, status);
}

void Compressor_Control_On()
{
	HAL_GPIO_WritePin(GPIOE, COMPRESSOR_CTRL_Pin, GPIO_PIN_SET);
}

void Compressor_Control_Off()
{
	HAL_GPIO_WritePin(GPIOE, COMPRESSOR_CTRL_Pin, GPIO_PIN_RESET);
}

void MembranePS_control(bool status)
{
	HAL_GPIO_WritePin(GPIOG, MEMB_POWER_EN_Pin, status);
}

// Tested OK 092121 GG
void displayPS_control(bool status)
{
	HAL_GPIO_WritePin(GPIOA, DISP_POWER_EN_Pin, status);
}

// Tested OK 090721 GG
void powerpcb_control(bool status)
{
	HAL_GPIO_WritePin(GPIOE, PWRRELAY_CTRL_Pin, status);  //PE6
}

// Tested OK 090721 GG
void PF_control(bool status)
{
	HAL_GPIO_WritePin(GPIOE, PF_EN_Pin, status);  //PE11
}

/*
void output_control(bool status)
{
	HAL_GPIO_WritePin(GPIOE, PF_EN_Pin, status);  //PE11
}*/

// Tested OK 090721 GG
void PFC_RLY_control(bool status)
{
	HAL_GPIO_WritePin(PFC_RLY_EN_GPIO_Port, PFC_RLY_EN_Pin, status);  //PE7
}


/*
//Blower Control
//----------------------------------------
//BlowerDAC(n); 		n is a floating point value 0-100%
//For 12-bit DAC = 2^12 = 4096
//Vout = (Vref x D) / 2^N = (3.3 x 1000)/4096 = 0.806V */

void BlowerDAC(uint32_t speed)
{
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, speed);
}

void BlowerControl(uint16_t dacval)
{
	uint32_t var =0;
	float value = 0.0;
	g_currentDACVal = dacval;

	//Warning
	//These while loops with delays inside are mean! they halt all program
	//execution, use ONLY for hardware debugging and fix for firmware production

	while(var < dacval )   //(value < dacval )
	{
		var = value*(0xFFF + 1)/3.3;
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, var);
		value += 0.05;
		HAL_Delay(40); //this controls the speed of ramping up
	}
}

void BlowerOff()
{
	//uint32_t var;
	uint32_t value = g_currentDACVal;

    //Warning
	//These while loops with delays inside are mean! they halt all program
	//execution, use ONLY for hardware debugging and fix for firmware production

	while(value  > 50  )
	{
		value = value - 50;
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value);
		g_currentDACVal = value;
		HAL_Delay(40); //this controls the speed of ramping up
	}
}


void Set_Fan_ONOFF(bool OnOff)
{
	if(OnOff)
	{
		//UartPrintf("Fan On\n");
		htim14.Instance->CCR1 = 2500; //64% duty cycle
	}
	else
	{
		//UartPrintf("Fan Off\n");
		htim14.Instance->CCR1 = 0;
	}
}

void FanOnOff( uint8_t OnOff )
{
    if(OnOff)
    {
        Set_Fan_ONOFF( true );
    }
    else
    {
    	Set_Fan_ONOFF( false );
    }
	UartPrintf("Fan %s\n", (OnOff == 1) ? "ON" : "OFF");
}

void cprtsystem_init()
{
	//Power board is OFF
	powerpcb_control(false);

	//Blower Relay is disable
	PFC_RLY_control(false);

	//DAC is set to zero
	BlowerDAC(0);

	//PFC_Module is disable
	PF_control(false);

	//Set chassis fan off
	//HAL_GPIO_WritePin(GPIOF, FAN_CTRL_PWM_TIM14_Pin, GPIO_PIN_RESET);
	//Please use the API to control the GPIO
	Set_Fan_ONOFF(false);
}


void cprtPowerOnSequence()
{
	char msg[20];
	int len = 0;
	uint32_t i = 0;

	//1. Apply Power (24V) to Power PCBA (/PWR_Ctrl signal is active, low)
	powerpcb_control(true);

	sprintf(msg, "Powerpcb=ON");
	len =  strlen(msg);
	LCD_Write(true, LCD_LINE_1, 0, msg, len);
	//HAL_Delay(3000);
	for (i=0; i<10000000; i++) { }  //stupid delay to avoid using HAL_delay GG 10/05/21

	//2. Enable PFC Module with /PF_Enable signal
	PF_control(true);

	sprintf(msg, "PFCctrl=ON");
	len =  strlen(msg);
	LCD_Write(true, LCD_LINE_2, 0, msg, len);

	//3. Wait until /LD_EN signal is active (low)
	//HAL_Delay(3000);  // As per PFC module specs Ton can be 2.5 Sec - 4 Sec

	for (i=0; i<30000000; i++) { }  //stupid delay to avoid using HAL_delay GG 10/05/21

	//Evaluate if LD-ENABLE pin from PFC is active

	if (GetPfcLoadEnable())
	{
		ClearPfcLoadEnable();
		HAL_GPIO_WritePin(GPIOD, LED_Y_Pin, false);  //added for troubleshoot purposes GG 100521

		sprintf(msg, "/LDEnab detected!");
		len = snprintf(msg, sizeof(msg), "%s", msg);
		LCD_Write(true, LCD_LINE_3, 0, msg, len-1);

		HAL_Delay(1000); //small additional delay
		//4. Close K2 relay to power Blower on
		PFC_RLY_control(true);

		sprintf(msg, "PFC_rly=closed");
		len =  strlen(msg);
		LCD_Write(true, LCD_LINE_4, 0, msg, len);
		HAL_Delay(1000); //small additional delay
	}
	else
	{
		sprintf(msg, "PFCLdEN not detected");
		len =  strlen(msg);
		LCD_Write(true, LCD_LINE_3, 0, msg, len);
		HAL_Delay(2000); //small additional delay
	}
}


void cprtPowerOffSequence()
{
	char msg[20];
	int len = 0;

	//1. Open K2 relay to power Blower off
	PFC_RLY_control(false);

	sprintf(msg, "PFC_rly=open");
	len =  strlen(msg);
	LCD_Write(true, LCD_LINE_1, 0, msg, len);
	HAL_Delay(2000);

	//2. Disable PFC Module with /PF_Enable signal
	PF_control(false);

	sprintf(msg, "PFCctrl=OFF");
	len =  strlen(msg);
	LCD_Write(true, LCD_LINE_2, 0, msg, len);
	HAL_Delay(2000);

	//3. Cut Power (24V) to Power PCBA (/PWR_Ctrl signal is inactive, high)
	powerpcb_control(false);

	sprintf(msg, "Powerpcb=OFF");
	len =  strlen(msg);
	LCD_Write(true, LCD_LINE_3, 0, msg, len);
	HAL_Delay(2000);
}


void ClearPfcPowerFailWarn()
{
	gstModel.bPfcPowerWarnFail = false;
}

void ClearPfcLoadEnable()
{
	gstModel.bPfcLoadEnable = false;
}

bool GetPfcPowerFailWarn()
{
	return gstModel.bPfcPowerWarnFail;
}

bool GetPfcLoadEnable()
{
	return gstModel.bPfcLoadEnable;
}

void SetPfcPowerFailWarn(bool flag)
{
	gstModel.bPfcPowerWarnFail = flag;
}

void SetPfcLoadEnable(bool flag)
{
	gstModel.bPfcLoadEnable = flag;
}


/*

//E-Stop Activated  TESTED OK
//---------------------------------------
//Pin PE10 GPIO Input, Active High, (High = EStop activated)
//Assign an Interrupt, ISR TBD
// Done: The estopflag = true is set every time the interrupt on PE10 is detected (low->High).

Chassis/Cabinet Temperature
---------------------------------------
Get_ADCCabTemp(n);	Returns a buffer of n samples where n can be 1 to 1000  Sampling rate 1000 Samp/Sec
SetFans(n);		n is fan speed in 0% to 100%  (FAN_CTRL is PWM)

SDCard
------------------------------------------
WritteSD();
ReadSD();


Interface with CombiMaster
------------------------------------------
Inputs
Get_ADCPFCCurrent(n);	Returns a buffer of n samples where n can be 1 to 1000  Sampling rate 1000 Samp/Sec
Get_ADCPFCTemp(n);	Returns a buffer of n samples where n can be 1 to 1000  Sampling rate 1000 Samp/Sec
Get_ADCPFCVout(n);	Returns a buffer of n samples where n can be 1 to 1000  Sampling rate 1000 Samp/Sec
Interrupt for /PFCPowerFail GPIO  (Input)
Interrupt for /PFCLoadEn GPIO     (Input)

Outputs:
_PFCEn(status);		status is boolean ON / OFF  This output is active Low
_PFCRlyEn(status); 	status is boolean ON / OFF  This output is active Low

Digital Communication with CombiMaster is TBD, we need information about the protocol and commands

USB Interface
------------------------------------------
Not a priority now but at some point it must be tested  */



