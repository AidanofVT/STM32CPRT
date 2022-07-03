#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include "main.h"
#include "interp.h"
#include "eeprom.h"
#include "Led.h"
#include "Buzzer.h"
#include "Model.h"
#include "Adc.h"
#include "hw.h"


extern uint32_t maxUartTxQueueWaitTimeMs;
extern uint16_t maxTxUartQueueDepth;
extern uint16_t curTxUartQueueDepth;
extern TIM_HandleTypeDef htim5;

volatile extern bool wdi_flag;


static uint8_t headerPrinted = 0;
bool noAsyncPrints           = false;
bool bStopDebugPrint         = false;

bool GetNoAsyncPrintsFlag( )
{
	return noAsyncPrints;
}

bool IsStopDebugPring()
{
	return bStopDebugPrint;
}

PrintEnables printEnables;
uint16_t PrintTimes[PRINT_GROUPS];

void PrintManagerInit()
{
	printEnables.PrintEnableWord = 0;
	PrintTimes[PRINT_PRESS]      = 200;

	return;
}

int PrintManager()
{
	static unsigned int pmCount = 0;

	if ( printEnables.PrintEnableWord )
	{
		//check if there is any request to print Item for example this is the request to show
		//the pressure sensor

		if (  printEnables.bits.printPress  )
		{
			if ( !(pmCount % PrintTimes[PRINT_PRESS]) )
			{
				if ( !headerPrinted )
				{
					PmUartPrintfWithoutPrompt("PSen1 mmHg       PSen2   mmHg     PSen3   mmHg    PSen4  mmHg\n");
					headerPrinted = 1;
				}

				//UartPrintfWithoutPrompt("%4.3f     %4.3f     %4.3f     %4.3f\n",
				//		                   GetPressureSensor1Value(), GetPressureSensor2Value(),
				//						   GetPressureSensor3Value(), GetPressureSensor4Value() );

				UartPrintfWithoutPrompt("%9.4f        %9.4f        %9.4f        %9.4f\n",
						               GetCalcMmhgSensorPressure1( ), GetCalcMmhgSensorPressure2( ),
						               GetCalcMmhgSensorPressure3( ), GetCalcMmhgSensorPressure4( ) );
			}
		}
		else if (  printEnables.bits.printPfc  )
		{
			if ( !(pmCount % PrintTimes[PRINT_PFC]) )
			{
				if ( !headerPrinted )
				{
					PmUartPrintfWithoutPrompt("PfcCurrentRawAdc      PfcVoutRawAdc     PfcTempMonRawAdc\n");
					headerPrinted = 1;
				}

				PmUartPrintfWithoutPrompt("%d       %d       %d\n", GetPfcCurrentMonRawAdc(), GetPfcVOutRawAdc(), GetPfcTempMonRawAdc() );
			}
		}

	}

	pmCount++;

	return 0;
}

void SetPrintTimePeriods( PRINT_CODE pc, uint16_t tp )
{
	noAsyncPrints = false;

	if ( pc >= PRINT_GROUPS )
	{
		UartPrintf( "Illegal Print Group for Setting Time Period \n");
		return;
	}

	if ( tp == 0 )
	{
		headerPrinted = 0;

		switch ( pc )
		{
			case PRINT_PRESS:
			    printEnables.bits.printPress = 0;
			break;

			case PRINT_PFC:
			    printEnables.bits.printPfc = 0;
			break;

			default:
			printEnables.PrintEnableWord = 0;
			break;
		}

		return;
	}

    if ( tp >= MIN_PRINT_PERIOD_MS )
	{
		PrintTimes[pc] = tp / 5;  // Print Times in 5ms ticks

		if( tp < 20 )
		{
			noAsyncPrints = true;
		}
	}
	else
	{
		printEnables.PrintEnableWord = 0;
		UartPrintfWithoutPrompt( "Print Period is too short \n");
		UartPrintf( "Print time must be 0 (turn print group off) or >= %d \n", MIN_PRINT_PERIOD_MS );
	}

	return;
}


int SetPMprints( int argc, char** argv )
{
	PRINT_CODE printCode;

    if ( !strcmp(argv[1], "press") )
	{
		printCode = PRINT_PRESS;
		printEnables.bits.printPress = 1;
	}
	else if( !strcmp(argv[1], "pfc") )
	{
		printCode = PRINT_PFC;
		printEnables.bits.printPfc = 1;
	}
	else
	{
		UartPrintf( "Illegal Print Code Used \n" );
		return -1;
	}

	if ( argc > 2 )
	{
		uint16_t printTimePeriodMs;
		printTimePeriodMs = atoi( (const char*)argv[2] );
		SetPrintTimePeriods( printCode, printTimePeriodMs );
	}
	else
	{
		UartPrintfWithoutPrompt( "No Print Period specified -- defaulting to 1 second\n");
		SetPrintTimePeriods( printCode, 200 );
	}

	return 0;
}

int SetAllLedBlink( int argc, char** argv )
{
	UartPrintf("Received Blink all LEDs Command, TimeOn TimeOff should be multiple of 5ms\n");
	LED_Set( LED_COUNT, LED_SET_BLINK, 50, 50);
	return 0;
}

int SetAllLedOn( int argc, char** argv )
{
	UartPrintf("Received Turn all LEDs On Command\n");
	LED_Set( LED_COUNT, LED_SET_ON, 0, 0);
	return 0;
}

int SetAllLedOff( int argc, char** argv )
{
	UartPrintf("Received Turn all LEDs Off Command\n");
	LED_Set( LED_COUNT, LED_SET_OFF, 0, 0);
	return 0;
}

int InitLog( int argc, char** argv )
{
	InitializeLog( );

	UartPrintf("");

    return 0;
}

int GetMfgDate( int argc, char** argv )
{
	GetMfgDateFromEeprom( );

	return 0;
}

int SetMfgDate( int argc, char** argv )
{
	if( sizeof(argv[1]) <= 16 )
	{
		StoreMfgDateInEeprom( argv[1] );
	}
	else
	{
		UartPrintf("Mfg Date: %s is too long\n", argv[1]);
	}

	UartPrintf("Set Mfg Date:%s\n", argv[1]);

	return 0;
}

int GetSerialNum( int argc, char** argv )
{
	GetSerialNumberFromEeprom( );

	return 0;
}

int SetSerialNum( int argc, char** argv )
{
	uint32_t serialNum = strtol(argv[1], NULL, 10);

	StoreSerialNumberInEeprom( serialNum );

	UartPrintf("");

	return 0;
}

int SetMediumAlarm( int argc, char** argv )
{
	UartPrintf("Med Alarm On\n");
	BUZZER_Request( BUZZER_TONE_MED, 300, 300, 0, 10000 );
	return 0;
}

int SetHightAlarm( int argc, char** argv )
{
	UartPrintf("High Alarm On\n");
	BUZZER_Request( BUZZER_TONE_HIGH, 250, 250, 300, 5000 );
	return 0;
}

int SetAlarmOff( int argc, char** argv )
{
	UartPrintf("Alarm Off\n");
	BUZZER_Request( BUZZER_TONE_OFF, 0, 0, 0, 0 );
	return 0;
}

int SetDacOutput( int argc, char** argv )
{

	float fV = strtof(argv[1], NULL);

	if(fV < 0.0 || fV > 3.3)
	{
		UartPrintfWithoutPrompt("Bad DAC value: range allows from 0.0 to 3.3\n");
	}
	else
	{
	   UartPrintfWithoutPrompt("Set DAC value to %4.2f Volt\n", fV);
	   Set_DAC_Voltage(fV);
	}
	return 0;
}

int GetDacOutput( int argc, char** argv )
{
	UartPrintfWithoutPrompt("What is DAC setting?\n");
	Get_DAC_Voltage();
	return 0;
}

int SetRTCTime( int argc, char** argv )
{
	uint8_t hr  = (uint8_t)strtol(argv[1], NULL, 10);
	uint8_t min = (uint8_t)strtol(argv[2], NULL, 10);
	uint8_t sec = (uint8_t)strtol(argv[3], NULL, 10);
	Set_Time( hr, min, sec);
	return 0;
}

int SetRTCDate( int argc, char** argv )
{
	uint8_t day   = (uint8_t)strtol(argv[1], NULL, 10);
	uint8_t month = (uint8_t)strtol(argv[2], NULL, 10);
	uint8_t date  = (uint8_t)strtol(argv[3], NULL, 10);
	uint8_t year  = (uint8_t)strtol(argv[4], NULL, 10);
	Set_Date( day, month, date, year);
	return 0;
}

int DisplayTime( int argc, char** argv )
{
	uint8_t onoff   = (uint8_t)strtol(argv[1], NULL, 10);

	if(onoff)
	{
		gstModel.bDisplayTime = true;
	}
	else
	{
		gstModel.bDisplayTime = false;
	}
	return 0;
}

int GetAdc1Values( int argc, char** argv )
{
	uint16_t numReads = (uint16_t)strtol(argv[1], NULL, 10);

	if ( numReads > 50000 )
	{
		UartPrintf( "Print Count Limit of 50000 \r\n" );
		return -1;
	}

	PrintAdc1ChannelsSpecifiedNumberOfTimes( numReads );

    return 0;
}

int GetAdc3Values( int argc, char** argv )
{
	uint16_t numReads = (uint16_t)strtol(argv[1], NULL, 10);

	if ( numReads > 50000 )
	{
		UartPrintf( "Print Count Limit of 50000 \r\n" );
		return -1;
	}

	PrintAdc3ChannelsSpecifiedNumberOfTimes( numReads );

    return 0;
}

int GetAdc1OneTime( int argc, char** argv )
{
	PrintAdc1Channels( );

    return 0;
}

int GetAdc3OneTime( int argc, char** argv )
{
	PrintAdc3Channels( );

    return 0;
}

int ValveControl( int argc, char** argv )
{
	uint8_t valvenumber;
	bool status;

	if ( !strcmp(argv[2], "1") )
	{
		status = true;
	}

	if ( !strcmp(argv[2], "0") )
	{
		status = false;
	}

	valvenumber = atoi( (const char*)argv[1] );

	int istatus = atoi( (const char*)argv[2] );

	if ( valvenumber > 8 )
	{
		UartPrintf( "Valve Number must be < 8\n" );
		return -1;
	}

	if ( istatus > 1 )
	{
		UartPrintf( "Valve Enable Param must be 1 to enable or 0 to disable \n" );
		return -1;
	}

	valve_control(valvenumber, status);
	UartPrintf("Valve %d Control Status %d\n", valvenumber, status);
	return 0;
}

int SetFanOnOff( int argc, char** argv )
{
	uint8_t onoff = (uint16_t)strtol(argv[1], NULL, 10);

	FanOnOff( onoff );

	return 0;
}

int StopDebugPrinting( int argc, char** argv )
{
	uint8_t onoff = (uint16_t)strtol(argv[1], NULL, 10);

	bStopDebugPrint = onoff;

	return 0;
}

int BatLedControl( int argc, char** argv )
{
	uint8_t ledbat;
	bool status;

	if ( !strcmp(argv[2], "1") )
	{
		status = true;
	}
	if ( !strcmp(argv[2], "0") )
	{
		status = false;
	}

	ledbat = atoi( (const char*)argv[1] );

	int istatus = atoi( (const char*)argv[2] );

	if ( istatus > 1 )
	{
		UartPrintf( "Battery LED Enable Param must be 1 to turn on or 0 to turn off \n" );
		return -1;
	}

	batleds_control(ledbat, status);

	UartPrintf("Led Battery %d Control Status %d\n", ledbat, status);
	return 0;
}


int LedAlarmControl ( int argc, char** argv )
{
	uint8_t alarmled;
	bool status;

	if ( !strcmp(argv[2], "1") )
	{
		status = true;
	}
	if ( !strcmp(argv[2], "0") )
	{
		status = false;
	}

	alarmled = atoi( (const char*)argv[1] );

	int istatus = atoi( (const char*)argv[2] );

	if ( istatus > 1 )
	{
		UartPrintf( "Alarm LED Enable Param must be 1 to turn on or 0 to turn off \n" );
		return -1;
	}

	alarmleds_control(alarmled, status);

	UartPrintf("Alarm Led %d Control Status %d\n", alarmled, status);
	return 0;
}


int CompressorControl( int argc, char** argv )
{
	uint8_t compressorEn;

	compressorEn = atoi( (const char*)argv[1] );
	if ( compressorEn > 1 )
	{
		UartPrintf( "Compressor Enable Param must be 1 to enable or 0 to disable \n" );
		return -1;
	}
	compressor_control(compressorEn);
	UartPrintf( "Compressor Power set to %d\n", compressorEn );
	return 0;
}

int MembranePSControl( int argc, char** argv )
{
	uint8_t membranepsEn;

	membranepsEn = atoi( (const char*)argv[1] );
	if ( membranepsEn > 1 )
	{
		UartPrintf( "Membrane PS Enable Param must be 1 to enable or 0 to disable \n" );
		return -1;
	}

	MembranePS_control(membranepsEn);
	UartPrintf( "Membrane Power Supply set to %d\n", membranepsEn );
	return 0;
}

int DisplayPSControl( int argc, char** argv )
{
	uint8_t displaypsEn;

	displaypsEn = atoi( (const char*)argv[1] );
	if ( displaypsEn > 1 )
	{
		UartPrintf( "Display Power Control Param must be 1 to enable or 0 to disable \n" );
		return -1;
	}

	displayPS_control(displaypsEn);
	UartPrintf( "Display Power Control set to %d\n", displaypsEn );
	return 0;
}


int SyncOutControl ( int argc, char** argv )
{
	uint8_t syncOutEn;

		syncOutEn = atoi( (const char*)argv[1] );
		if ( syncOutEn > 1 )
		{
			UartPrintf( "Synch Output Enable Param must be 1 to enable or 0 to disable \n" );
			return -1;
		}

		SynchOut(syncOutEn);
		UartPrintf("Sync Output Control Status %d\n", syncOutEn);
		return 0;
}

int pcbpowerrelayControl( int argc, char** argv )
{
	uint8_t powerpcbEn;

	powerpcbEn = atoi( (const char*)argv[1] );
	if ( powerpcbEn > 1 )
	{
		UartPrintf( "Power PCB Relay Enable Param must be 1 to enable or 0 to disable \n" );
		return -1;
	}

	powerpcb_control(powerpcbEn);
	UartPrintf("Power PCB Relay Control Status %d\n", powerpcbEn);
	return 0;
}


int chassisfanControl( int argc, char** argv )
{
	uint8_t fanEn;

	fanEn = atoi( (const char*)argv[1] );
	if ( fanEn> 1 )
	{
		UartPrintf( "Chassis Fan Enable Param must be 1 to enable or 0 to disable \n" );
		return -1;
	}

	chassisFan_control(fanEn);
	UartPrintf("Chassis FAN has been set to %d\n", fanEn);
	return 0;
}


int pfControl( int argc, char** argv )
{
	uint8_t pfEn;

	pfEn = atoi( (const char*)argv[1] );
	if ( pfEn> 1 )
	{
		UartPrintf( "Power Factor Module Enable Param must be 1 to enable or 0 to disable \n" );
		return -1;
	}

	PF_control(pfEn);
	UartPrintf("Power Factor Module Enable Status is %d\n", pfEn);
	return 0;
}


int wdiControl( int argc, char** argv )
{
	uint8_t status;

	status = atoi( (const char*)argv[1] );
	if ( status > 1 )
	{
		UartPrintf( "Watchdog Timer Input Param must be 1 to enable or 0 to disable \n" );
		return -1;
	}

	wdi_flag = status;
	UartPrintf("Watchdog Timer Input set to %d\n", status);
	return 0;
}

int pfcRlyControl( int argc, char** argv )
{
	uint8_t pfcRlyEn;

	pfcRlyEn = atoi( (const char*)argv[1] );
	if ( pfcRlyEn > 1 )
	{
		UartPrintf( "Power Factor Relay Enable Param must be 1 to enable or 0 to disable \n" );
		return -1;
	}

	PFC_RLY_control(pfcRlyEn);
	UartPrintf("Power Factor Relay Enable Status is %d\n", pfcRlyEn);
	return 0;
}


int BlowerSpeedCtrl ( int argc, char** argv )
{
	uint16_t speed;

	speed = atoi( argv[1] );   //speed = atoi( (const char*)argv[1] );
	if ( speed > 4095 )
	{
		UartPrintf( "Blower max speed Param must be less than 4096\n" );
		return -1;
	}
	UartPrintf("Blower Speed Control will be set to  %u\n", speed);
	BlowerDAC(speed);

	return 0;
}

int BlowerCtrl ( int argc, char** argv )
{
	uint16_t dacval;

	dacval = atoi( (const char*)argv[1] );
	if ( dacval > 4095 )
	{
		UartPrintf( "Blower DAC Value must be 0 to 4095\n" );
		return -1;
	}

	BlowerControl(dacval);
	UartPrintf("Blower Control Status set to %d\n", dacval);
	return 0;

}

int Bloweroff ( int argc, char** argv )
{
	BlowerOff();
	return 0;
}

int CprtseqON ( int argc, char** argv )
{
	cprtPowerOnSequence();
	return 0;
}

int CprtseqOFF ( int argc, char** argv )
{
	cprtPowerOffSequence();
	return 0;
}


void RegisterAllCommands()
{
    InterpRegister( "initlog",         InitLog,            1, 1 );
    InterpRegister( "setmfgdate",      SetMfgDate,         2, 2 );
    InterpRegister( "getmfgdate",      GetMfgDate,         1, 1 );
    InterpRegister( "setserialnum",    SetSerialNum,       2, 2 );
    InterpRegister( "getserialnum",    GetSerialNum,       1, 1 );
    InterpRegister( "ledallblk",       SetAllLedBlink,     1, 1 );
    InterpRegister( "ledallon",        SetAllLedOn,        1, 1 );
    InterpRegister( "ledalloff",       SetAllLedOff,       1, 1 );

    InterpRegister( "alarmmed",        SetMediumAlarm,     1, 1 );
    InterpRegister( "alarmhigh",       SetHightAlarm,      1, 1 );
    InterpRegister( "alarmoff",        SetAlarmOff,        1, 1 );

    InterpRegister( "setdac",         SetDacOutput,        2, 2 );
    InterpRegister( "getdac",         GetDacOutput,        1, 1 );

    InterpRegister( "settime",         SetRTCTime,          4, 4 );
    InterpRegister( "setdate",         SetRTCDate,          5, 5 );

    InterpRegister( "disptime",        DisplayTime,         2, 2 );
    InterpRegister( "getadc1",         GetAdc1Values,       2, 2 );
    InterpRegister( "getadc3",         GetAdc3Values,       2, 2 );
    InterpRegister( "adc1",            GetAdc1OneTime,      1, 1 );
    InterpRegister( "adc3",            GetAdc3OneTime,      1, 1 );
    InterpRegister( "setfan",         SetFanOnOff,          2, 2 );
    InterpRegister( "setprt",  	  	  SetPMprints,          2, 3 );

    //commands from Cprtfunctions code
    InterpRegister( "valvectrl",      ValveControl,           3, 3 );
	InterpRegister( "ledbatctrl",     BatLedControl,          3, 3 );
	InterpRegister( "ledalarmctrl",   LedAlarmControl,        3, 3 );
	InterpRegister( "compressorctrl", CompressorControl,      2, 2 );
	InterpRegister( "membranepsctrl", MembranePSControl,      2, 2 );
	InterpRegister( "displaypsctrl",  DisplayPSControl,       2, 2 );
	InterpRegister( "syncoutctrl",    SyncOutControl,         2, 2 );
	InterpRegister( "powerpcbctrl",   pcbpowerrelayControl,   2, 2 );
	InterpRegister( "pfctrl",   	  pfControl,              2, 2 );
	InterpRegister( "wdictrl",   	  wdiControl,             2, 2 );
	InterpRegister( "pfcrlyctrl",  	  pfcRlyControl,          2, 2 );
	InterpRegister( "blowerspeed",    BlowerSpeedCtrl,        2, 2 );
	InterpRegister( "blowerctrl",     BlowerCtrl,             2, 2 );
	InterpRegister( "bloweroff",      Bloweroff,              1, 1 );
	InterpRegister( "cprtseqon", 	  CprtseqON,			  1, 1 );
	InterpRegister( "cprtseqoff", 	  CprtseqOFF,			  1, 1 );
	InterpRegister( "stopdbgprt",     StopDebugPrinting,      2, 2 );

    UartPrintf("All Interpreter Commands Registered\n");
}

