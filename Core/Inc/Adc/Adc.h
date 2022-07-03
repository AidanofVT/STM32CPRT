

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VOLTAGE_H
#define __VOLTAGE_H


typedef enum {
	ADC1_CHANNEL_PressureSensor3,  //PA5
	ADC1_CHANNEL_PressureSensor1,  //PA6
	ADC1_CHANNEL_PressureSensor2,  //PA7
	ADC1_CHANNEL_12V,              //PB0
	ADC1_CHANNEL_5VREF,            //PB1
	ADC1_CHANNEL_PressureSensor4,  //PC4
	ADC1_CHANNEL_CabTemp,          //PC5

	NUM_ADC1_CHANNELS
} ADC1_CHANNELS;


typedef enum {
	ADC3_CHANNEL_MainPcba_Current,  //PA0
	ADC3_CHANNEL_Power_Voltage,     //PA1
	ADC3_CHANNEL_ADC_3V3,           //PA2
	ADC3_CHANNEL_ADC_5V,            //PA3
	ADC3_CHANNEL_PFCCurrMon,        //PF3
	ADC3_CHANNEL_BUZ_ADC,           //PC0
	ADC3_CHANNEL_ADC_PFCVOMon,      //PC1
	ADC3_CHANNEL_PFCTempMon,        //PF4

	NUM_ADC3_CHANNELS
} ADC3_CHANNELS;

#define NUM_VALUES_TO_AVERAGE       (8)

void Adc1Init( );
void SetAdc1CompleteFlag( bool complete );
bool GetAdc1CompleteFlag( );
void StartAdc1Dma( );
void PrintAdc1Channels( );
void PrintAdc1ChannelsSpecifiedNumberOfTimes( uint16_t numReads );
void ReadAdc1Channels();

void Adc3Init( );
void SetAdc3CompleteFlag( bool complete );
bool GetAdc3CompleteFlag( );
void StartAdc3Dma( );
void PrintAdc3Channels( );
void PrintAdc3ChannelsSpecifiedNumberOfTimes( uint16_t numReads );
void ReadAdc3Channels();

float GetPressureSensor1Value();
float GetPressureSensor2Value();
float GetPressureSensor3Value();
float GetPressureSensor4Value();

float GetCalcMmhgSensorPressure1( );
float GetCalcMmhgSensorPressure2( );
float GetCalcMmhgSensorPressure3( );
float GetCalcMmhgSensorPressure4( );

uint32_t GetMainPcbaCurrentRawAdc();
uint32_t GetPower24VoltageRawAdc();
uint32_t Get3V3RawAdc();
uint32_t Get5VoltRawAdc();
uint32_t GetPfcCurrentMonRawAdc();
uint32_t GetBuzzerRawAdc();
uint32_t GetPfcVOutRawAdc();
uint32_t GetPfcTempMonRawAdc();


#endif /* __VOLTAGE_H */
