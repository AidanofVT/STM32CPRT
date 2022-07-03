#include <stdbool.h>
#include "main.h"
#include "utilities.h"
#include "Adc.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

static bool adc1DmaComplete = false;
static bool adc3DmaComplete = false;

static const float Adc3_3VIndexToMiliVolts   = (3.3f / 4096.0f) * 1000;
static const float Power24VIndexToVolts      = (6.77266f / 1000.0f);
static const float Adc3V3IndexToVolts        = (0.887975f / 1000.0f);
static const float Adc5VIndexToVolts         = (1.343101f / 1000.0f);
static const float PfcCurrMonIndexToAmps     = (0.805861f * 2.0f / 1000.0f);  //Modified by GG 100621
static const float PfcVOutIndexToVolts       = (0.805861f * 92.592f * 2.0f / 1000.0f);  //Modified by GG 100621  10.8mv -->1V, including Resistor divider
static const float PfcTempMonIndexToCentig   = (0.805861f * 1.50f / 10.0f);   //added GG 10/5/21  10K||20K divider, multiplied by 100

#define NOMINAL_ZERO_ADC_COUNT              (1862)
#define NOMINAL_PRESSURE_CONVERSION_FACTOR  (0.041077f)
#define CMH2O_TO_MMHG                       (0.735559)

uint16_t pAdcZero       = NOMINAL_ZERO_ADC_COUNT;
float pConversionFactor = NOMINAL_PRESSURE_CONVERSION_FACTOR;

static uint32_t adc1Values[NUM_ADC1_CHANNELS];
static uint32_t adc3Values[NUM_ADC3_CHANNELS];

static bool startupAdc1 = true;
static bool startupAdc3 = true;

/* ADC1 Channels */

void Adc1Init( )
{
	startupAdc1 = true;
	HAL_ADC_Start_DMA(&hadc1, adc1Values, NUM_ADC1_CHANNELS);
}

void StartAdc1Dma( )
{
	// Get fresh ADC1 values
	HAL_ADC_Stop_DMA(&hadc1);
	SetAdc1CompleteFlag( false );
	HAL_ADC_Start_DMA(&hadc1, adc1Values, NUM_ADC1_CHANNELS);
}

void SetAdc1CompleteFlag( bool complete )
{
	adc1DmaComplete = complete;
}

bool GetAdc1CompleteFlag( )
{
	return adc1DmaComplete;
}

/* ADC1 Channels Raw ADC counts*/

uint32_t GetPressureSensor3RawAdc()
{
	return adc1Values[ADC1_CHANNEL_PressureSensor3];
}

uint32_t GetPressureSensor1RawAdc()
{
	return adc1Values[ADC1_CHANNEL_PressureSensor1];
}

uint32_t GetPressureSensor2RawAdc()
{
	return adc1Values[ADC1_CHANNEL_PressureSensor2];
}

uint32_t GetPressureSensor4RawAdc()
{
	return adc1Values[ADC1_CHANNEL_PressureSensor4];
}

uint32_t Get12VRawAdc()
{
	return adc1Values[ADC1_CHANNEL_12V];
}

uint32_t Get5VRefRawAdc()
{
	return adc1Values[ADC1_CHANNEL_5VREF];
}


uint32_t GetCabTempRawAdc()
{
	return adc1Values[ADC1_CHANNEL_CabTemp];
}


/* ADC1 Channels Values*/

float GetPressureSensor3Value()
{
	return adc1Values[ADC1_CHANNEL_PressureSensor3] * Adc3_3VIndexToMiliVolts;
}

float GetPressureSensor1Value()
{
	return adc1Values[ADC1_CHANNEL_PressureSensor1] * Adc3_3VIndexToMiliVolts;
}

float GetPressureSensor2Value()
{
	return adc1Values[ADC1_CHANNEL_PressureSensor2] * Adc3_3VIndexToMiliVolts;
}

float GetPressureSensor4Value()
{
	return adc1Values[ADC1_CHANNEL_PressureSensor4] * Adc3_3VIndexToMiliVolts;
}

/* Convert Volt to (PSI) or mmHg  for PS2, PS3, PS4
 * Pressure(psi) = (Vout – 0.5) /0.8;
 * Pressure(mmHg) = (Vout - 0.5) /0.8 * 51.7149
 */

float GetCalcMmhgSensorPressure2( )
{
    return 	((GetPressureSensor2Value() / 1000.0) - 0.5) / 0.8 * 51.7149;
}

float GetCalcMmhgSensorPressure3( )
{
    return 	((GetPressureSensor3Value() / 1000.0) - 0.5) / 0.8 * 51.7149;
}

float GetCalcMmhgSensorPressure4( )
{
    return 	((GetPressureSensor4Value() / 1000.0) - 0.5) / 0.8 * 51.7149;
}

float GetCalcMmhgSensorPressure1( )
{
	uint16_t pAdcRaw = adc1Values[ADC1_CHANNEL_PressureSensor1];

	return ((pAdcRaw - pAdcZero) * pConversionFactor) * 1.0197162f * CMH2O_TO_MMHG;
}


float Get12vValue()
{
	return adc1Values[ADC1_CHANNEL_12V] * Adc3_3VIndexToMiliVolts;
}

float Get5VRefValue()
{
	return adc1Values[ADC1_CHANNEL_5VREF] * Adc3_3VIndexToMiliVolts;
}

float GetCabTempValue()
{
	return adc1Values[ADC1_CHANNEL_CabTemp] * Adc3_3VIndexToMiliVolts;
}


/* ADC3 Channels */

void Adc3Init( )
{
	startupAdc3 = true;
	HAL_ADC_Start_DMA(&hadc3, adc3Values, NUM_ADC3_CHANNELS);
}

void StartAdc3Dma( )
{
	// Get fresh ADC3 values
	HAL_ADC_Stop_DMA(&hadc3);
	SetAdc3CompleteFlag( false );
	HAL_ADC_Start_DMA(&hadc3, adc3Values, NUM_ADC3_CHANNELS);
}

void SetAdc3CompleteFlag( bool complete )
{
	adc3DmaComplete = complete;
}

bool GetAdc3CompleteFlag( )
{
	return adc3DmaComplete;
}

/* ADC3 Channels Raw ADC counts*/

uint32_t GetMainPcbaCurrentRawAdc()
{
	return adc3Values[ADC3_CHANNEL_MainPcba_Current];
}

uint32_t GetPower24VoltageRawAdc()
{
	return adc3Values[ADC3_CHANNEL_Power_Voltage];
}

uint32_t Get3V3RawAdc()
{
	return adc3Values[ADC3_CHANNEL_ADC_3V3];
}

uint32_t Get5VoltRawAdc()
{
	return adc3Values[ADC3_CHANNEL_ADC_5V];
}

uint32_t GetPfcCurrentMonRawAdc()
{
	return adc3Values[ADC3_CHANNEL_PFCCurrMon];
}

uint32_t GetBuzzerRawAdc()
{
	return adc3Values[ADC3_CHANNEL_BUZ_ADC];
}

uint32_t GetPfcVOutRawAdc()
{
	return adc3Values[ADC3_CHANNEL_ADC_PFCVOMon];
}

uint32_t GetPfcTempMonRawAdc()
{
	return adc3Values[ADC3_CHANNEL_PFCTempMon];
}

/* ADC3 Channels Values*/

float GetMainPcbaCurrentValue()
{
	return adc3Values[ADC3_CHANNEL_MainPcba_Current] * Adc3_3VIndexToMiliVolts;
}

float GetPower24VoltageValue()
{
	return adc3Values[ADC3_CHANNEL_Power_Voltage] * Power24VIndexToVolts;
}

float Get3V3Value()
{
	return adc3Values[ADC3_CHANNEL_ADC_3V3] * Adc3V3IndexToVolts;
}

float Get5VoltValue()
{
	return adc3Values[ADC3_CHANNEL_ADC_5V] * Adc5VIndexToVolts;
}

float GetPfcCurrentMonValue()
{
	return (adc3Values[ADC3_CHANNEL_PFCCurrMon] * PfcCurrMonIndexToAmps);
}

float GetBuzzerValue()
{
	return adc3Values[ADC3_CHANNEL_BUZ_ADC] * Adc3_3VIndexToMiliVolts;
}

float GetPfcVOutValue()
{
	return adc3Values[ADC3_CHANNEL_ADC_PFCVOMon] * PfcVOutIndexToVolts;
}

float GetPfcTempMonValue()
{
	return adc3Values[ADC3_CHANNEL_PFCTempMon] * PfcTempMonIndexToCentig;  //Modified GG 10/5/21
}

void ReadAdc1Channels()
{
	// Get fresh ADC values for print out
	HAL_ADC_Stop_DMA(&hadc1);

	adc1DmaComplete = false;

	HAL_ADC_Start_DMA(&hadc1, adc1Values, NUM_ADC1_CHANNELS);

	// Ensure that DMA is complete before reading ADC 1 values
	uint32_t startWaitTime = HAL_GetTick();

	while(adc1DmaComplete == false)
	{
		if ( (HAL_GetTick() - startWaitTime) > 3 ) // 3 ms
		{
			UartPrintf("ADC1 DMA not complete for 3 ms\n");
			return; // Wait too long for ADC1 DMA to complete
		}
	}
}

void PrintAdc1Channels( )
{
	// Get fresh ADC values for print out
	HAL_ADC_Stop_DMA(&hadc1);

	adc1DmaComplete = false;

	HAL_ADC_Start_DMA(&hadc1, adc1Values, NUM_ADC1_CHANNELS);

	// Ensure that DMA is complete before reading ADC 1 values
	uint32_t startWaitTime = HAL_GetTick();

	while(adc1DmaComplete == false)
	{
		if ( (HAL_GetTick() - startWaitTime) > 3 ) // 3 ms
		{
			UartPrintf("ADC1 DMA not complete for 3 ms\n");
			return; // Wait too long for ADC1 DMA to complete
		}
	}

	HAL_ADC_Stop_DMA(&hadc1);

	//UartPrintf("%d  %d  %d  %d  %d  %d  %d\n",
	//			GetPressureSensor1RawAdc(), GetPressureSensor2RawAdc(), GetPressureSensor3RawAdc(), GetPressureSensor4RawAdc(),
	//		    Get12VRawAdc(), Get5VRefRawAdc(), GetCabTempRawAdc());

	// Read ADC1 values for pressure sensors
	float pressureSensor3Print = GetPressureSensor3Value();
	float pressureSensor1Print = GetPressureSensor1Value();
	float pressureSensor2Print = GetPressureSensor2Value();
	float twelveVoltPrint      = Get12vValue();
	float fiveVoltRefPrint     = Get5VRefValue();
	float pressureSensor4Print = GetPressureSensor4Value();
	float cabTempPrint         = GetCabTempValue();

	UartPrintf("\nPSensor1:  %4.3f mV\n"
			   "PSensor2:  %4.3f mV\n"
			   "PSensor3:  %4.3f mV\n"
			   "PSensor4:  %4.3f mV\n"
			   "12 Volts:  %4.3f mV\n"
			   "5V Ref:    %4.3f mV\n"
			   "CabTemp:   %4.3f mV\n",
			   pressureSensor1Print, pressureSensor2Print, pressureSensor3Print, pressureSensor4Print,
			   twelveVoltPrint, fiveVoltRefPrint, cabTempPrint);

	adc1DmaComplete = false;

	HAL_ADC_Start_DMA(&hadc1, adc1Values, NUM_ADC1_CHANNELS);
}

void PrintAdc1ChannelsSpecifiedNumberOfTimes( uint16_t numReads )
{
	// Get fresh ADC values for print out
	HAL_ADC_Stop_DMA(&hadc1);

	adc1DmaComplete = false;

	HAL_ADC_Start_DMA(&hadc1, adc1Values, NUM_ADC1_CHANNELS);

	UartPrintfWithoutPrompt("  PSensor1  PSensor2  PSensor3  PSensor4  12Volts  5VoltsRef  CabTemp\n");

	for( int i = 0; i<numReads; i++ )
	{
		// Ensure that DMA is complete before reading ADC 1 values
		uint32_t startWaitTime = HAL_GetTick();

		while(adc1DmaComplete == false)
		{
			if ( (HAL_GetTick() - startWaitTime) > 3 ) // 3 ms
			{
				UartPrintf("ADC1 DMA not complete for 3 ms\n");
				return; // Wait too long for ADC1 DMA to complete
			}
		}

		HAL_ADC_Stop_DMA(&hadc1);

		// Read ADC1 values for pressure sensors
		float pressureSensor3Print = GetPressureSensor3Value();
		float pressureSensor1Print = GetPressureSensor1Value();
		float pressureSensor2Print = GetPressureSensor2Value();
		float twelveVoltPrint      = Get12vValue();
		float fiveVoltRefPrint     = Get5VRefValue();
		float pressureSensor4Print = GetPressureSensor4Value();
		float cabTempPrint         = GetCabTempValue();

		adc1DmaComplete = false;

		HAL_ADC_Start_DMA(&hadc1, adc1Values, NUM_ADC1_CHANNELS);

		UartPrintfWithoutPrompt("%4.3f   %4.3f   %4.3f   %4.3f   %4.3f   %4.3f   %4.3f\n",
				               pressureSensor1Print, pressureSensor2Print, pressureSensor3Print, pressureSensor4Print,
							   twelveVoltPrint, fiveVoltRefPrint, cabTempPrint);

		HAL_Delay(5);
	}

	UartPrintf("");
}

void ReadAdc3Channels()
{
	// Get fresh ADC values for print out
	HAL_ADC_Stop_DMA(&hadc3);

	adc3DmaComplete = false;

	HAL_ADC_Start_DMA(&hadc3, adc3Values, NUM_ADC3_CHANNELS);

	// Ensure that DMA is complete before reading ADC 1 values
	uint32_t startWaitTime = HAL_GetTick();

	while(adc3DmaComplete == false)
	{
		if ( (HAL_GetTick() - startWaitTime) > 3 ) // 3 ms
		{
			UartPrintf("ADC3 DMA not complete for 3 ms\n");
			return; // Wait too long for ADC1 DMA to complete
		}
	}
}

void PrintAdc3Channels( )
{
	// Get fresh ADC values for print out
	HAL_ADC_Stop_DMA(&hadc3);

	adc3DmaComplete = false;

	HAL_ADC_Start_DMA(&hadc3, adc3Values, NUM_ADC3_CHANNELS);

	// Ensure that DMA is complete before reading ADC 1 values
	uint32_t startWaitTime = HAL_GetTick();

	while(adc3DmaComplete == false)
	{
		if ( (HAL_GetTick() - startWaitTime) > 3 ) // 3 ms
		{
			UartPrintf("ADC3 DMA not complete for 3 ms\n");
			return; // Wait too long for ADC3 DMA to complete
		}
	}

	HAL_ADC_Stop_DMA(&hadc3);

	// Read ADC3 values
	float mainPcbaCurrentPrint = GetMainPcbaCurrentValue();
	float power24VPrint        = GetPower24VoltageValue();
	float threethreeVoltsPrint = Get3V3Value();
	float fiveVoltsPrint       = Get5VoltValue();
	float pfcCurrentMonPrint   = GetPfcCurrentMonValue();
	float buzzerPrint          = GetBuzzerValue();
	float pfcVoutPrint         = GetPfcVOutValue();
    float pfcTempMonPrint      = GetPfcTempMonValue();

	UartPrintf("\nMain Current:  %4.3f mA\n"
			   "Power 24V:    %4.3f V\n"
			   "3.3 Volts:    %4.3f V\n"
			   "5 Volts:      %4.3f V\n"
			   "PFC Current:  %4.3f A\n"
			   "Buzzer:       %4.3f mV\n"
			   "PFC Vout:     %4.3f V\n"
			   "PFC Temp:     %4.3f mV\n",
			   mainPcbaCurrentPrint, power24VPrint, threethreeVoltsPrint, fiveVoltsPrint,
			   pfcCurrentMonPrint, buzzerPrint, pfcVoutPrint, pfcTempMonPrint);

	adc3DmaComplete = false;

	HAL_ADC_Start_DMA(&hadc3, adc3Values, NUM_ADC3_CHANNELS);
}

void PrintAdc3ChannelsSpecifiedNumberOfTimes( uint16_t numReads )
{
	// Get fresh ADC values for print out
	HAL_ADC_Stop_DMA(&hadc3);

	adc3DmaComplete = false;

	HAL_ADC_Start_DMA(&hadc3, adc3Values, NUM_ADC3_CHANNELS);

	UartPrintfWithoutPrompt("  MainCurrent    Power24V    3.3V    5V    PFCCurrent    Buzzer    PFCVout    PFCTemp\n");

	for( int i = 0; i<numReads; i++ )
	{
		// Ensure that DMA is complete before reading ADC 1 values
		uint32_t startWaitTime = HAL_GetTick();

		while(adc3DmaComplete == false)
		{
			if ( (HAL_GetTick() - startWaitTime) > 3 ) // 3 ms
			{
				UartPrintf("ADC3 DMA not complete for 3 ms\n");
				return; // Wait too long for ADC1 DMA to complete
			}
		}

		HAL_ADC_Stop_DMA(&hadc3);

		// Read ADC3 values
		float mainPcbaCurrentPrint = GetMainPcbaCurrentValue();
		float power24VPrint        = GetPower24VoltageValue();
		float threethreeVoltsPrint = Get3V3Value();
		float fiveVoltsPrint       = Get5VoltValue();
		float pfcCurrentMonPrint   = GetPfcCurrentMonValue();
		float buzzerPrint          = GetBuzzerValue();
		float pfcVoutPrint         = GetPfcVOutValue();
	    float pfcTempMonPrint      = GetPfcTempMonValue();


		adc3DmaComplete = false;

		HAL_ADC_Start_DMA(&hadc3, adc3Values, NUM_ADC3_CHANNELS);

		UartPrintfWithoutPrompt("%4.3f    %4.3f    %4.3f    %4.3f    %4.3f    %4.3f   %4.3f    %4.3f\n",
				                mainPcbaCurrentPrint, power24VPrint, threethreeVoltsPrint, fiveVoltsPrint,
							    pfcCurrentMonPrint, buzzerPrint, pfcVoutPrint, pfcTempMonPrint);

		HAL_Delay(5);
	}

	UartPrintf("");
}

