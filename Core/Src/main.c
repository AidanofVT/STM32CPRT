/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <lcd.h>
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "interp.h"
#include "Display.h"
#include "Buzzer.h"
#include "Model.h"
#include "Switches.h"
#include "eeprom.h"
#include "Adc.h"
#include "hw.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIGNAL_DEBOUNCE_30MS     (30)
uint32_t blower_duty_cycle[11] = { 0, 100, 620, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4095 };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd2;
DMA_HandleTypeDef hdma_sdmmc2_tx;
DMA_HandleTypeDef hdma_sdmmc2_rx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;

/* USER CODE BEGIN PV */
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */

// **** Must also add new corresponding swVersionId in main.h ****
const char * swVersionStrings[] = {
		"Unknown",
		"1.0.0",
		"1.0.1",
		"1.0.2",
		"1.0.3",
		"1.0.4",
		"1.0.5",
		"1.0.6",
		"1.0.7",
		"1.0.8",
		"1.0.9",
		"1.1.0",
};

uint8_t uartReceiveBuff[UART_RX_DMA_BUFFER_SIZE];   // Define the RX UART array
char commandBuf[MAX_COMMAND_LENGTH];                // Define command buffer array
bool commandReady = false;                          // Initialize commandReady flag for command from UART to false

uint8_t guiUartReceiveBuff[UART_RX_DMA_BUFFER_SIZE];   // Define the RX UART array for GUI communication
char guiCommandBuf[MAX_COMMAND_LENGTH];                // Define GUI's command buffer array
bool guiCommandReady = false;                          // Initialize Gui's commandReady flag for command from UART to false

volatile bool wdi_flag                            = false;
uint32_t minuteCntr                               = 0;
static bool fiveMsTick                            = false;
static bool oneMinuteTick                         = false;
static uint16_t fiveMsHalfMillisecondCnt          = 0;
static bool oneSecondTick                         = false;
static uint32_t oneMinuteHalfMillisecondCnt       = 0;
static uint16_t oneSecondHalfMillisecondCnt       = 0;
static uint16_t clearScreenHalfMillisecondCnt     = 0;

static uint16_t eepromWaitHalfMillisecondCnt      = 0;

bool bPfcLoadEnable                        = false;   //static
uint32_t uPfcLoadEnTimeDB                  = 0;       //static

bool bPowerFailWarn                        = false;   //static
uint32_t uPowerFailWarnTimeDB              = 0;       //static



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SDMMC2_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM14_Init(void);
static void MX_UART5_Init(void);
static void MX_UART7_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM7_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_UART8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool IsPfcLoadEnLow()
{
	//PfcLoadEnanable signal low return TRUE else return FALSE
	return ( HAL_GPIO_ReadPin(PFC_LoadEN_N_GPIO_Port, PFC_LoadEN_N_Pin) == GPIO_PIN_RESET) ? true : false;
}

bool IsPfcPowerFailWarnLow()
{
	//PFC_POWERFAILWARNNING_N signal low return TRUE else return FALSE
	return ( HAL_GPIO_ReadPin(PFC_POWERFAILWARNNING_N_GPIO_Port, PFC_POWERFAILWARNNING_N_Pin) == GPIO_PIN_RESET) ? true : false;
}

void Refresh_Hardware_WDT()
{
	HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
}

void delayWd100ms( uint8_t numOf100msTics )
{
	uint8_t i;

	// Allow a maximum delay of 2 sec.
	if ( numOf100msTics > 20 )
	{
		numOf100msTics = 20;
		UartPrintf( "ERROR - Requesting illegal WD delay over 2 sec \n" );
	}

	for ( i = 0; i < numOf100msTics; i++ )
	{
		HAL_Delay( 100 );
	    //HAL_IWDG_Refresh( &hiwdg );  // Feed the Internal dog.
	    Refresh_Hardware_WDT();		 // Feed the External dog.
	}

	return;
}

void CommandProcessing( )
{
    // Check if command from UART1 is ready to process
	if( commandReady != false )
	{
		// Process command
		if( strlen(commandBuf) > 0 )
		{

			if( InterpRun( commandBuf ) != INTERP_SUCCESS )
			{
			    UartPrintf("Error Executing Received Command\r\n");
			}
		}
		else // Empty command received, do not attempt to process
		{
		    UartPrintf("");
		}

		// Command processed, clear command buffer and command ready flag
		memset(commandBuf, 0, strlen(commandBuf));
		commandReady = false;
	}
}

void GuiCommandProcessing( )
{
    // Check if command from UART1 is ready to process
	if( guiCommandReady != false )
	{
		// Process command
		if( strlen(guiCommandBuf) > 0 )
		{
			uint32_t length = strlen(guiCommandBuf);

			UartPrintf("Received:%d bytes %s\n", length, guiCommandBuf);

			uint32_t index = 0;

            while(length != 0)
            {
            	char inchar = guiCommandBuf[index];

				switch(inchar)
				{
					case 'A':
						UartPrintf("Thoracic Vest 1 On\n");
						Valve_1A_On();
					break;

					case 'B':
						UartPrintf("Thoracic Vest 1 Off\n");
						Valve_1A_Off();
					break;

					case 'C':
						UartPrintf("Thoracic Vest 2 On\n");
						Valve_1B_On();
					break;

					case 'D':
						UartPrintf("Thoracic Vest 2 Off\n");
						Valve_1B_Off();
					break;

					case 'E':
						UartPrintf("Abd Vest 1 On\n");
						Valve_2A_On();
					break;

					case 'F':
						UartPrintf("Abd Vest 1 Off\n");
						Valve_2A_Off();
					break;

					case 'M':
						UartPrintf("Abd Vest 2 On\n");
						Valve_2B_On();
					break;

					case 'N':
						UartPrintf("Abd Vest 2 Off\n");
						Valve_2B_Off();
					break;

					case 'G':
						UartPrintf("Vac Div 1 On\n");
						Valve_3A_On();
					break;

					case 'H':
						UartPrintf("Vac Div 1 Off\n");
						Valve_3A_Off();
					break;

					case 'I':
						UartPrintf("Vac Div 2 On\n");
						Valve_3B_On();
					break;

					case 'J':
						UartPrintf("Vac Div 2 Off\n");
						Valve_3B_Off();
					break;

					case 'K':
						UartPrintf("Vent 1 On\n");
						Valve_4A_On();
					break;

					case 'L':
						UartPrintf("Vent 1 Off\n");
						Valve_4A_Off();
					break;

					case 'S':
					{
						//process speed setting S+speed+T
						//parse for number
                        char msg[8];

                        //advance index
                        uint8_t duty_cycle_index = 0;

                        index++; //SxT where x = 0 to 10
                        char nextchar = guiCommandBuf[index];

                        while (nextchar != 'T' && nextchar != '\0')
                        {
                        	msg[duty_cycle_index] = nextchar;
                        	duty_cycle_index++;
                        	index++;
                        	nextchar = guiCommandBuf[index];
                        }

                        msg[duty_cycle_index] = '\0'; //complete string

                        uint8_t dc_index = atoi( msg );

						if(dc_index >= 0 && dc_index <= 10)
						{
							uint32_t duty_cycle = blower_duty_cycle[dc_index];

							UartPrintf("Speed:%d DAC:%d\r\n", dc_index, duty_cycle);

							BlowerDAC( duty_cycle );

							powerpcb_control(1);

							PF_control(1);

							PFC_RLY_control(1);
						}

					}
					break;

					case 'P':
						//turn on pilot compressor P2
						UartPrintf("Compressor Contro1 On\n");
						Compressor_Control_On();
					break;

					case 'Q':
						//turn off pilot compressor P2
						UartPrintf("Compressor Contro1 Off\n");
						Compressor_Control_Off();
					break;

					case 'W':
						//subscribe for 4-pressure sensors
						UartPrintf("Subscribe for Pressure Statuses\n");
						gstModel.bPressureData = true;
					break;

					case 'Z':
						//unsubscribe for 4-pressure sensors
						UartPrintf("Un-Subscribe for Pressure Statuses\n");
						gstModel.bPressureData = false;
					break;

					default:
					break;
				}

				length--;
				index++;
            }
		}

		// Command processed, clear command buffer and command ready flag
		memset(guiCommandBuf, 0, strlen(guiCommandBuf));
		guiCommandReady = false;
	}
}

void Set_DAC_Voltage(float fV)
{
	//only allow from 0.0 to 3.3V
	if( (fV >= 0.0) && (fV <= 3.3) )
	{
		gstModel.dac_value = (uint16_t)(fV * 4095 / 3.3);

		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,  gstModel.dac_value);

		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	}
}

void  Get_DAC_Voltage()
{
	UartPrintfWithoutPrompt("DAC is set at: %2.1f\n", (gstModel.dac_value * 3.3)/4095.0);
}

void Set_Time(uint8_t hr, uint8_t min, uint8_t sec)
{
  RTC_TimeTypeDef sTime;

  /** Initialize RTC and set the Time and Date  */
  sTime.Hours   = hr;
  sTime.Minutes = min;
  sTime.Seconds = sec;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0xDEADBEEF); // backup register
}

void Set_Date(uint8_t day, uint8_t month, uint8_t date, uint8_t year)
{
  RTC_DateTypeDef sDate;

  sDate.WeekDay = day; //RTC_WEEKDAY_TUESDAY;
  sDate.Month   = month; //RTC_MONTH_SEPTEMBER;
  sDate.Date    = date;   //28;
  sDate.Year    = year;   //21;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0xDEADBEEF); // backup register
}

void Get_Time()
{
	 RTC_DateTypeDef gDate;
	 RTC_TimeTypeDef gTime;

	 /* Get the RTC current Time */
	 HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);

	 /* Get the RTC current Date */
	 HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

	 /* Display time Format: hh:mm:ss */
	 sprintf((char *)&gstModel.atime,"%02d:%02d:%02d", gTime.Hours, gTime.Minutes, gTime.Seconds);

	 /* Display date Format: mm-dd-yy */
	 sprintf((char *)&gstModel.adate,"%02d-%02d-%2d", gDate.Month, gDate.Date, 2000 + gDate.Year);
}

int Monitor()
{
	ReadAdc1Channels();
	ReadAdc3Channels();
	return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SDMMC2_SD_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM14_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */

  wdi_flag = true;  //Enables feeding EWDT

  Adc1Init();
  //  Adc3Init();
  HAL_ADC_Start(&hadc1);

  // Starts the DAC
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);   //Added by Gio  10/6/21

  //Added by Gio 10/5/21
  cprtsystem_init();
  UartPrintf("CPRT System Initialization Started\n");

  //UartPrintf("Initialization Started\n");
  GetMinuteCounterFromEeprom( );
  delayWd100ms( 1 );
  GetEventCounterFromEeprom( );
  delayWd100ms( 1 );
  GetMfgDateFromEeprom( );
  delayWd100ms( 1 );
  GetSerialNumberFromEeprom( );
  delayWd100ms( 1 );

  //Added the checking to do the initlog
  uint32_t minuteCount = GetMinuteCount();

  if(minuteCount >= 0x025DFBE0) // TODO: value check coming from data showed 27579 x 1440, need to confirm this value
  {
      InitializeLog( );
  }

  delayWd100ms( 5 );

  /*Debug Uart using UART5*/
  memset(commandBuf, 0, MAX_COMMAND_LENGTH);
  __HAL_UART_ENABLE_IT(&debugUart, UART_IT_IDLE);  // Enable UART idle interrupt
  __HAL_UART_ENABLE_IT(&debugUart, UART_IT_TC);    // Enable UART Tx Complete interrupt
  HAL_UART_Receive_IT(&debugUart, uartReceiveBuff, UART_RX_DMA_BUFFER_SIZE ); // Set up the IT UART transmission to move the received
                                                                                // data of serial port to uartReceiveBuff,
                                                                                // UART_RX_DMA_BUFFER_SIZE bytes at a time

  /*GUI Uart suing UART8 */
  memset(guiCommandBuf, 0, MAX_COMMAND_LENGTH);
  __HAL_UART_ENABLE_IT(&guiUart, UART_IT_IDLE);  // Enable UART idle interrupt
  //__HAL_UART_ENABLE_IT(&guiUart, UART_IT_TC);    // Enable UART Tx Complete interrupt
  HAL_UART_Receive_IT(&guiUart, guiUartReceiveBuff, UART_RX_DMA_BUFFER_SIZE ); // Set up the IT UART transmission to move the received
                                                                                // data of serial port to uartReceiveBuff,
                                                                                // UART_RX_DMA_BUFFER_SIZE bytes at a time


   /*
   * Timers tables
   * TIM2:  generate 3.9 KHz pwm for Buzzer
   * TIM5:  generate 1us timer
   * TIM7:  generate 0.5ms to use for 5ms main loop
   * TIM14: generate 255Hz pwm for Fan
   */

  //Start internal 1us timer
  HAL_TIM_Base_Start(&htim5);                // Start timer TIM5 for 1us timer

  HAL_TIM_Base_Start_IT(&htim7);             // Start timer TIM7 for half millisecond timer
                                             // TIM7 is driven by APB1 clock of 108 MHz reload is set Prescale = 108, ARR= 499+1
                                             // for a 0.5 ms period (1/(10^6/500) * 10^3 = 0.5)

  //Start pwm for buzzer
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  //Start pwm for fan-ctrl
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

  InitEepromTxQueue();

  InitUartTxQueue();     // Initialize queue for transmitting data to UART

  UartPrintf("");

  InterpInit();          // Initialize command interpreter

  RegisterAllCommands(); // Register commands with interpreter

  PrintManagerInit();

  if( HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) !=  0xDEADBEEF )
  {
	  Set_Time( 23, 15, 0 );
	  Set_Date( RTC_WEEKDAY_TUESDAY, RTC_MONTH_SEPTEMBER, 28, 21 );
  }

  HAL_Delay(500);

  FRESULT res;                                          /* FatFs function common result code */
  uint32_t byteswritten;                                /* File write/read counts */
  uint8_t wtext[] = "Text line added at power up.\n";     /* File write buffer */

  /*##-2- Register the file system object to the FatFs module ##############*/
  if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
  {
      /* FatFs Initialization Error */
	  Error_Handler();
  }
  else
  {
	  UartPrintf("File system Mounted Success!!!\r\n");
	  UartPrintf("Create, Open and Write text line to the file!!!\r\n");

	  /*##-4- Create and Open a new text file object with write access #####*/
	  res = f_open(&MyFile, "STM32.TXT", FA_OPEN_APPEND | FA_WRITE);

	  if( res != FR_OK)
	  {
	      /* 'STM32.TXT' file Open for write Error */
		  UartPrintf("File created error!!!\r\n");
		  Error_Handler();
	  }
	  else
	  {
		  /*##-5- Write data to the text file ################################*/
		  res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);

		  if((byteswritten == 0) || (res != FR_OK))
	      {
			  /* 'STM32.TXT' file Write or EOF Error */
			  UartPrintf("Card Write Error!!!\r\n");
			  Error_Handler();
		  }
		  else
		  {
			  /*##-6- Close the open text file #################################*/
			  UartPrintf("Card Write Success and Close The File!!!\r\n");
			  f_close(&MyFile);
		  }
	   }
  }

  Model_Init();

  Get_Time(); //get time from RTC to update the Model time and date

  Display_Init();

  Set_Fan_ONOFF( true );
  
  //Set_DAC_Voltage(1.8);  //GG 10/05/21 No need to Initialized DAC here

  UartPrintf("Initialization Complete -- Version: %s %s\n", swVersionStrings[CURRENT_SW_VERSION_ID], VERSION_DETAILS);
  UartPrintf("Minutes of In Services: %d\n", minuteCount);
  
  fiveMsTick    = false; // Clear 5 ms tick right before entering while loop to ensure a full 5 ms between the first two
  oneSecondTick = false;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if( fiveMsTick == true ) // 5 ms processing
	  {
	      fiveMsTick = false;

	      Monitor();

	      Display_Manager();

	      PrintManager();
	  }

	  if( oneSecondTick == true)
	  {
		  oneSecondTick = false;
		  Get_Time();
		  Display_Request_Refresh();

		  if(gstModel.bDisplayTime == true)
		  {
			  UartPrintf("%s\n", gstModel.atime);
			  UartPrintf("%s\n", gstModel.adate);
		  }

		  if(gstModel.bPressureData == true)
		  {
			  //send back Pressure data from 4 sensors
			  GuiUartPrintf("%9.4f,%9.4f,%9.4f, %9.4f\n",
						     GetCalcMmhgSensorPressure1( ), GetCalcMmhgSensorPressure2( ),
						     GetCalcMmhgSensorPressure3( ), GetCalcMmhgSensorPressure4( ) );
		  }
	  }


	  if(bPfcLoadEnable == true)
	  {
		  //Debounce PFC Load Enable signal for 30ms
		  if( uPfcLoadEnTimeDB && (Get_SysTick() - uPfcLoadEnTimeDB >= SIGNAL_DEBOUNCE_30MS))
		  {
			  if(IsPfcLoadEnLow() == true)
			  {
				  SetPfcLoadEnable( true );
				  HAL_GPIO_WritePin(GPIOD, LED_Y_Pin, true);  //added for troubleshoot purposes GG 100521
			  }

			  uPfcLoadEnTimeDB = 0;
			  bPfcLoadEnable   = false;
		  }
	  }

	  if(bPowerFailWarn == true)
	  {
		  //Debounce Power Failure Warning signal for 30ms
		  if( uPowerFailWarnTimeDB && (Get_SysTick() - uPowerFailWarnTimeDB >= SIGNAL_DEBOUNCE_30MS))
		  {
			  if(IsPfcPowerFailWarnLow() == true)
			  {
				  SetPfcPowerFailWarn( true );
			  }

			  uPowerFailWarnTimeDB = 0;
			  bPowerFailWarn   = false;
		  }
	  }

	  // Feed the dog.
	  //HAL_IWDG_Refresh( &hiwdg );
	  
	  Refresh_Hardware_WDT();

	  //Debug command process
  	  CommandProcessing();

  	  //Gui Command process
  	  GuiCommandProcessing();
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 108;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_UART7|RCC_PERIPHCLK_UART8
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_I2C3
                              |RCC_PERIPHCLK_SDMMC2|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart8ClockSelection = RCC_UART8CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.Sdmmc2ClockSelection = RCC_SDMMC2CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 8;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_0;
  AnalogWDGConfig.ITMode = DISABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc3, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */
  Switches_Power_Onoff(true);
  LCD_Power_Onoff(true);
  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x306034D1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x20404768;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
#if(0)
  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */
#endif
  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
#if(0)
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 21;
  sTime.Minutes = 33;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_SEPTEMBER;
  sDate.Date = 28;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
#endif
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDMMC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC2_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC2_Init 0 */

  /* USER CODE END SDMMC2_Init 0 */

  /* USER CODE BEGIN SDMMC2_Init 1 */

  /* USER CODE END SDMMC2_Init 1 */
  hsd2.Instance = SDMMC2;
  hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd2.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd2.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd2.Init.ClockDiv = 2;
  /* USER CODE BEGIN SDMMC2_Init 2 */

  /* USER CODE END SDMMC2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 108;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 108;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xffffffff;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 108;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 499;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 108;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 3920;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, COMPRESSOR_CTRL_Pin|PWRRELAY_CTRL_Pin|PFC_RLY_EN_Pin|PF_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, ModBUS_Dir_Pin|WDI_Pin|TP34_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WIFI_RST_N_GPIO_Port, WIFI_RST_N_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, VALVE_1A_CTRL_Pin|VALVE_1B_CTRL_Pin|VALVE_2A_CTRL_Pin|VALVE_2B_CTRL_Pin
                          |ONBOARD_LED_Pin|VALVE_3A_CTRL_Pin|VALVE_3B_CTRL_Pin|VALVE_4A_CTRL_Pin
                          |VALVE_4B_CTRL_Pin|MEMB_POWER_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SYNC_OUT_Pin|TP32_Pin|TP33_Pin|TP36_Pin
                          |TP29_Pin|TP30_Pin|TP31_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_BAT100_Pin|LED_BAT75_Pin|LED_Y_Pin|LED_G_Pin
                          |LED_R_Pin|LED_BAT50_Pin|LED_BAT25_Pin|TP26_Pin
                          |TP28_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DISP_POWER_EN_Pin|WIFI_SPI3_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : N_MEMBRANE_INT_Pin MEMBRANESW_IN_Pin */
  GPIO_InitStruct.Pin = N_MEMBRANE_INT_Pin|MEMBRANESW_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : COMPRESSOR_CTRL_Pin PWRRELAY_CTRL_Pin PFC_RLY_EN_Pin PF_EN_Pin */
  GPIO_InitStruct.Pin = COMPRESSOR_CTRL_Pin|PWRRELAY_CTRL_Pin|PFC_RLY_EN_Pin|PF_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ModBUS_Dir_Pin WDI_Pin TP34_Pin */
  GPIO_InitStruct.Pin = ModBUS_Dir_Pin|WDI_Pin|TP34_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : WIFI_RST_N_Pin */
  GPIO_InitStruct.Pin = WIFI_RST_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(WIFI_RST_N_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PFC_LoadEN_N_Pin */
  GPIO_InitStruct.Pin = PFC_LoadEN_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PFC_LoadEN_N_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : VALVE_1A_CTRL_Pin VALVE_1B_CTRL_Pin VALVE_2A_CTRL_Pin VALVE_2B_CTRL_Pin
                           ONBOARD_LED_Pin VALVE_3A_CTRL_Pin VALVE_3B_CTRL_Pin VALVE_4A_CTRL_Pin
                           VALVE_4B_CTRL_Pin */
  GPIO_InitStruct.Pin = VALVE_1A_CTRL_Pin|VALVE_1B_CTRL_Pin|VALVE_2A_CTRL_Pin|VALVE_2B_CTRL_Pin
                          |ONBOARD_LED_Pin|VALVE_3A_CTRL_Pin|VALVE_3B_CTRL_Pin|VALVE_4A_CTRL_Pin
                          |VALVE_4B_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : ESTOP_ACTIVE_Pin */
  GPIO_InitStruct.Pin = ESTOP_ACTIVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ESTOP_ACTIVE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PFC_POWERFAILWARNNING_N_Pin */
  GPIO_InitStruct.Pin = PFC_POWERFAILWARNNING_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PFC_POWERFAILWARNNING_N_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SYNC_OUT_Pin TP32_Pin TP33_Pin TP36_Pin
                           TP29_Pin TP30_Pin TP31_Pin */
  GPIO_InitStruct.Pin = SYNC_OUT_Pin|TP32_Pin|TP33_Pin|TP36_Pin
                          |TP29_Pin|TP30_Pin|TP31_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_BAT100_Pin LED_BAT75_Pin LED_Y_Pin LED_G_Pin
                           LED_R_Pin LED_BAT50_Pin LED_BAT25_Pin TP26_Pin
                           TP28_Pin */
  GPIO_InitStruct.Pin = LED_BAT100_Pin|LED_BAT75_Pin|LED_Y_Pin|LED_G_Pin
                          |LED_R_Pin|LED_BAT50_Pin|LED_BAT25_Pin|TP26_Pin
                          |TP28_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DISP_POWER_EN_Pin WIFI_SPI3_EN_Pin */
  GPIO_InitStruct.Pin = DISP_POWER_EN_Pin|WIFI_SPI3_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DETECT_N_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_DETECT_N_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMB_POWER_EN_Pin */
  GPIO_InitStruct.Pin = MEMB_POWER_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MEMB_POWER_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SYNC_IN_N_Pin */
  GPIO_InitStruct.Pin = SYNC_IN_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SYNC_IN_N_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Counter reached end of period
	if (TIM7 == htim->Instance ) // Determine whether counter was TIM7 half millisecond counter
	{
		//Here is a frequency of 500uS
		if (wdi_flag)
		{
		   //Feeding the external watchdog timer here
			HAL_GPIO_TogglePin(GPIOF, WDI_Pin);  //This is PF13 on STM32 MCU
		}

		if(++fiveMsHalfMillisecondCnt >= 10)
		{
			fiveMsHalfMillisecondCnt = 0;
			fiveMsTick = true;
		}

		if(++oneSecondHalfMillisecondCnt >= 2000)
		{
			oneSecondHalfMillisecondCnt = 0;
			oneSecondTick = true;

		}
		
		if(++oneMinuteHalfMillisecondCnt >= 120000)
		{
			oneMinuteHalfMillisecondCnt = 0;
			oneMinuteTick = true;
			minuteCntr++;

			StoreMinuteCntrInEeprom( minuteCntr );
		}
		
		if( GetEepromWriteWait() != false )
		{
			eepromWaitHalfMillisecondCnt++;
			if(eepromWaitHalfMillisecondCnt >= 28)
			{
				SetEepromWriteWait( false );
				eepromWaitHalfMillisecondCnt = 0;
				ReadTxEepromQueueAndWriteEeprom();
			}
		}

		if( GetClearScreenWait() != false )
		{
			clearScreenHalfMillisecondCnt++;
			if(clearScreenHalfMillisecondCnt >= 6)
			{
				SetClearScreenWait( false );
				clearScreenHalfMillisecondCnt = 0;
				if(gstModel.bLCDResetRequest == false)
				{
				    ReadTxLcdQueueAndTransmit();
				}
			}
		}
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == N_MEMBRANE_INT_Pin)
	{
		UartPrintf("Membrane Pressed trigger\n");
		gstModel.bMembranePress = true;
	}
	else if(GPIO_Pin == PFC_LoadEN_N_Pin)
	{
		if(bPfcLoadEnable == false)
		{
		    UartPrintf("Load Enable trigger\n");
		    bPfcLoadEnable = true;
		    uPfcLoadEnTimeDB = Get_SysTick();
		}
	}
	else if(GPIO_Pin == PFC_POWERFAILWARNNING_N_Pin)
	{
		if(bPowerFailWarn == false)
		{
		    UartPrintf("Power Fail Warn trigger\n");
		    bPowerFailWarn = true;
		    uPowerFailWarnTimeDB = Get_SysTick();
		}
	}
	else
	{
		__NOP();
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
	// Reception of I2C data is complete
	if(I2C3 == hi2c->Instance)    // Determine whether complete is from I2C 3 EEPROM
	{
        // Check for EEPROM log read processing
		ProcessEepromReadComplete();
	}
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
	// Transmit of I2C data is complete
	if(I2C3 == hi2c->Instance)    // Determine whether complete is from I2C 3 EEPROM
	{
		// Read next entry in Tx EEPROM queue, if any, and write EEPROM
		ReadTxEepromQueueAndWriteEeprom();
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
	// Receive of I2C data is complete
	if(I2C2 == hi2c->Instance)
	{
		//STMF7xx has no Device Address in the i2c instance
		//so we can't check for device address here
		//however, we are lucky that this bus only has LCD and keypad which is
		//the only one device on the bus we can read back the status
		//hence the RxCplt callback assume is from keypad read back
	    gstModel.bKeyCodeReady = true;

	    if(gstModel.bLCDResetRequest == false)
	    {
	       ReadTxLcdQueueAndTransmit();
	    }
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
	// Transmit of I2C data is complete

	if(I2C2 == hi2c->Instance)    // Determine whether complete is from I2C 2 LCD
	{
		if(gstModel.bLCDResetRequest == false)
		{
			ReadTxLcdQueueAndTransmit();
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
	if(UART5 == huart->Instance)
	{
		// Transmit of Debug UART data is complete
		// Read next entry in print Tx UART queue, if any, and transmit out Debug UART
		ReadTxUartQueueAndTransmit();
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(ADC1 == hadc->Instance)
	{
		SetAdc1CompleteFlag( true );
	}
	else if(ADC3 == hadc->Instance)
	{
		SetAdc3CompleteFlag( true );
	}
}

uint32_t GetMinuteCount( )
{
	return minuteCntr;
}
void SetMinuteCount( uint32_t minuteCount )
{
	minuteCntr = minuteCount;
}

const char * GetSwVersionString( uint32_t swVersionNumId )
{
	return swVersionStrings[swVersionNumId];
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	uint8_t buf[128];
	sprintf((char*)buf, "Error_Handler called\r\n");
	HAL_UART_Transmit(&debugUart, buf, strlen((char*)buf), 0x200);

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
