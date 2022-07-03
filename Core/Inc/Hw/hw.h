/*
 * cprtfunctions.h
 *
 *  Created on: Aug 25, 2021
 *      Author: GiovanniG
 */

#ifndef INC_CPRTFUNCTIONS_H_
#define INC_CPRTFUNCTIONS_H_

typedef enum
{
	valve1A,
	valve1B,
	valve2A,
	valve2B,
	valve3A,
	valve3B,
	valve4A,
	valve4B,
} VALVE_NUMBER;

void valve_control(uint8_t valvename, bool status);
void compressor_control(bool status);
void MembranePS_control(bool status);
void displayPS_control(bool status);
void powerpcb_control(bool status);
void PF_control(bool status);
void PFC_RLY_control(bool status);
void chassisFan_control(bool status);
void BlowerDAC(uint32_t speed);
void BlowerControl(uint16_t dacval);
//void BlowerControl(bool status);
void SynchOut(bool status);
void GetADC(uint32_t numsamples);
void GetPS1(uint32_t numsamples);
void alarmleds_control(uint8_t alarmled, bool status);
void batleds_control(uint8_t batled, bool status);
void FanOnOff( uint8_t OnOff );
void Set_Fan_ONOFF(bool OnOff);
void cprtPowerOffSequence();
void cprtPowerOnSequence();
void BlowerOff();
void ClearPfcPowerFailWarn();
void ClearPfcLoadEnable();
bool GetPfcPowerFailWarn();
bool GetPfcLoadEnable();
void SetPfcPowerFailWarn(bool flag);
void SetPfcLoadEnable(bool flag);
void cprtsystem_init(); //added by Gio 10/5/21

void Valve_1A_On();
void Valve_1A_Off();
void Valve_1B_On();
void Valve_1B_Off();
void Valve_2A_On();
void Valve_2A_Off();
void Valve_2B_On();
void Valve_2B_Off();
void Valve_3A_On();
void Valve_3A_Off();
void Valve_3B_On();
void Valve_3B_Off();
void Valve_4A_On();
void Valve_4A_Off();
void Valve_4B_On();
void Valve_4B_Off();
void Compressor_Control_On();
void Compressor_Control_Off();

#endif /* INC_CPRTFUNCTIONS_H_ */
