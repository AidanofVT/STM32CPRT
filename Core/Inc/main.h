/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "utilities.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// **** Must also add new corresponding string in swVersionStrings in main.c ****
typedef enum
{
	SW_VERSION_UNKNOWN = 0,
	SW_VERSION_1_0_0,
	SW_VERSION_1_0_1,
	SW_VERSION_1_0_2,
	SW_VERSION_1_0_3,
	SW_VERSION_1_0_4,
	SW_VERSION_1_0_5,
	SW_VERSION_1_0_6,
	SW_VERSION_1_0_7,
	SW_VERSION_1_0_8,
	SW_VERSION_1_0_9,
	SW_VERSION_1_1_0,

	SW_VERSION_NUM_END,

} swVersionId;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//#define NO_uSD_MODE

uint32_t GetMinuteCount( );
void SetMinuteCount( uint32_t minuteCount );
void delayWd100ms( uint8_t numOf100msTics );
void feedInternalDogOnly();
const char * GetSwVersionString( uint32_t swVersionNumId );
void Set_DAC_Voltage(float fV);
void Get_DAC_Voltage();
void Set_Time(uint8_t hr, uint8_t min, uint8_t sec);
void Set_Date(uint8_t day, uint8_t month, uint8_t date, uint8_t year);
void Get_Time();
bool IsPfcLoadEnLow();
bool IsPfcPowerFailWarnLow();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define N_MEMBRANE_INT_Pin GPIO_PIN_2
#define N_MEMBRANE_INT_GPIO_Port GPIOE
#define N_MEMBRANE_INT_EXTI_IRQn EXTI2_IRQn
#define MEMBRANESW_IN_Pin GPIO_PIN_4
#define MEMBRANESW_IN_GPIO_Port GPIOE
#define COMPRESSOR_CTRL_Pin GPIO_PIN_5
#define COMPRESSOR_CTRL_GPIO_Port GPIOE
#define PWRRELAY_CTRL_Pin GPIO_PIN_6
#define PWRRELAY_CTRL_GPIO_Port GPIOE
#define UI_I2C2_SDA_Pin GPIO_PIN_0
#define UI_I2C2_SDA_GPIO_Port GPIOF
#define UI_I2C2_SCL_Pin GPIO_PIN_1
#define UI_I2C2_SCL_GPIO_Port GPIOF
#define FPCCurrMon_ADC3_IN9_Pin GPIO_PIN_3
#define FPCCurrMon_ADC3_IN9_GPIO_Port GPIOF
#define FPCTempMon_ADC3_IN14_Pin GPIO_PIN_4
#define FPCTempMon_ADC3_IN14_GPIO_Port GPIOF
#define ModBUS_Dir_Pin GPIO_PIN_5
#define ModBUS_Dir_GPIO_Port GPIOF
#define UART7_RX_Pin GPIO_PIN_6
#define UART7_RX_GPIO_Port GPIOF
#define UART7_TX_Pin GPIO_PIN_7
#define UART7_TX_GPIO_Port GPIOF
#define FAN_CTRL_PWM_TIM14_Pin GPIO_PIN_9
#define FAN_CTRL_PWM_TIM14_GPIO_Port GPIOF
#define BUZ_ADC3_IN10_Pin GPIO_PIN_0
#define BUZ_ADC3_IN10_GPIO_Port GPIOC
#define ADC_PFCVOMon_ADC3_11_Pin GPIO_PIN_1
#define ADC_PFCVOMon_ADC3_11_GPIO_Port GPIOC
#define WIFI_RST_N_Pin GPIO_PIN_2
#define WIFI_RST_N_GPIO_Port GPIOC
#define MainPcba_Current_ADC3_IN0_Pin GPIO_PIN_0
#define MainPcba_Current_ADC3_IN0_GPIO_Port GPIOA
#define Power_Voltage_ADC3_IN1_Pin GPIO_PIN_1
#define Power_Voltage_ADC3_IN1_GPIO_Port GPIOA
#define ADC_3V3_ADC3_IN2_Pin GPIO_PIN_2
#define ADC_3V3_ADC3_IN2_GPIO_Port GPIOA
#define ADC_5V_ADC3_IN3_Pin GPIO_PIN_3
#define ADC_5V_ADC3_IN3_GPIO_Port GPIOA
#define PressureSensor3_ADC1_IN5_Pin GPIO_PIN_5
#define PressureSensor3_ADC1_IN5_GPIO_Port GPIOA
#define PressureSensor1_ADC1_IN6_Pin GPIO_PIN_6
#define PressureSensor1_ADC1_IN6_GPIO_Port GPIOA
#define PressureSensor2_ADC1_IN7_Pin GPIO_PIN_7
#define PressureSensor2_ADC1_IN7_GPIO_Port GPIOA
#define PressureSensor4_ADC1_IN14_Pin GPIO_PIN_4
#define PressureSensor4_ADC1_IN14_GPIO_Port GPIOC
#define CABTEMP_ADC1_IN15_Pin GPIO_PIN_5
#define CABTEMP_ADC1_IN15_GPIO_Port GPIOC
#define ADC_12V_ADC1_IN8_Pin GPIO_PIN_0
#define ADC_12V_ADC1_IN8_GPIO_Port GPIOB
#define ADC_5VREF_ADC1_IN9_Pin GPIO_PIN_1
#define ADC_5VREF_ADC1_IN9_GPIO_Port GPIOB
#define WIFI_SPI3_MOSI_Pin GPIO_PIN_2
#define WIFI_SPI3_MOSI_GPIO_Port GPIOB
#define PFC_LoadEN_N_Pin GPIO_PIN_11
#define PFC_LoadEN_N_GPIO_Port GPIOF
#define PFC_LoadEN_N_EXTI_IRQn EXTI15_10_IRQn
#define WDI_Pin GPIO_PIN_13
#define WDI_GPIO_Port GPIOF
#define TP34_Pin GPIO_PIN_14
#define TP34_GPIO_Port GPIOF
#define VALVE_1A_CTRL_Pin GPIO_PIN_0
#define VALVE_1A_CTRL_GPIO_Port GPIOG
#define VALVE_1B_CTRL_Pin GPIO_PIN_1
#define VALVE_1B_CTRL_GPIO_Port GPIOG
#define PFC_RLY_EN_Pin GPIO_PIN_7
#define PFC_RLY_EN_GPIO_Port GPIOE
#define ESTOP_ACTIVE_Pin GPIO_PIN_10
#define ESTOP_ACTIVE_GPIO_Port GPIOE
#define ESTOP_ACTIVE_EXTI_IRQn EXTI15_10_IRQn
#define PF_EN_Pin GPIO_PIN_11
#define PF_EN_GPIO_Port GPIOE
#define PFC_POWERFAILWARNNING_N_Pin GPIO_PIN_14
#define PFC_POWERFAILWARNNING_N_GPIO_Port GPIOE
#define PFC_POWERFAILWARNNING_N_EXTI_IRQn EXTI15_10_IRQn
#define SYNC_OUT_Pin GPIO_PIN_10
#define SYNC_OUT_GPIO_Port GPIOB
#define BZR_PWM_Pin GPIO_PIN_11
#define BZR_PWM_GPIO_Port GPIOB
#define TP32_Pin GPIO_PIN_12
#define TP32_GPIO_Port GPIOB
#define TP33_Pin GPIO_PIN_14
#define TP33_GPIO_Port GPIOB
#define TP36_Pin GPIO_PIN_15
#define TP36_GPIO_Port GPIOB
#define LED_BAT100_Pin GPIO_PIN_9
#define LED_BAT100_GPIO_Port GPIOD
#define LED_BAT75_Pin GPIO_PIN_10
#define LED_BAT75_GPIO_Port GPIOD
#define LED_Y_Pin GPIO_PIN_11
#define LED_Y_GPIO_Port GPIOD
#define LED_G_Pin GPIO_PIN_12
#define LED_G_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_13
#define LED_R_GPIO_Port GPIOD
#define LED_BAT50_Pin GPIO_PIN_14
#define LED_BAT50_GPIO_Port GPIOD
#define LED_BAT25_Pin GPIO_PIN_15
#define LED_BAT25_GPIO_Port GPIOD
#define VALVE_2A_CTRL_Pin GPIO_PIN_2
#define VALVE_2A_CTRL_GPIO_Port GPIOG
#define VALVE_2B_CTRL_Pin GPIO_PIN_3
#define VALVE_2B_CTRL_GPIO_Port GPIOG
#define ONBOARD_LED_Pin GPIO_PIN_4
#define ONBOARD_LED_GPIO_Port GPIOG
#define VALVE_3A_CTRL_Pin GPIO_PIN_5
#define VALVE_3A_CTRL_GPIO_Port GPIOG
#define VALVE_3B_CTRL_Pin GPIO_PIN_6
#define VALVE_3B_CTRL_GPIO_Port GPIOG
#define VALVE_4A_CTRL_Pin GPIO_PIN_7
#define VALVE_4A_CTRL_GPIO_Port GPIOG
#define VALVE_4B_CTRL_Pin GPIO_PIN_8
#define VALVE_4B_CTRL_GPIO_Port GPIOG
#define I2C3_SDA_EEPROM_Pin GPIO_PIN_9
#define I2C3_SDA_EEPROM_GPIO_Port GPIOC
#define I2C3_SCL_EEPROM_Pin GPIO_PIN_8
#define I2C3_SCL_EEPROM_GPIO_Port GPIOA
#define VBUS_SENSE_Pin GPIO_PIN_9
#define VBUS_SENSE_GPIO_Port GPIOA
#define DISP_POWER_EN_Pin GPIO_PIN_10
#define DISP_POWER_EN_GPIO_Port GPIOA
#define USB_D_N_Pin GPIO_PIN_11
#define USB_D_N_GPIO_Port GPIOA
#define USB_D_P_Pin GPIO_PIN_12
#define USB_D_P_GPIO_Port GPIOA
#define WIFI_SPI3_EN_Pin GPIO_PIN_15
#define WIFI_SPI3_EN_GPIO_Port GPIOA
#define WIFI_SPI3_SCK_Pin GPIO_PIN_10
#define WIFI_SPI3_SCK_GPIO_Port GPIOC
#define WIFI_SPI3_MISO_Pin GPIO_PIN_11
#define WIFI_SPI3_MISO_GPIO_Port GPIOC
#define DEBUG_UART5_TX_Pin GPIO_PIN_12
#define DEBUG_UART5_TX_GPIO_Port GPIOC
#define DEBUG_UART5_RX_Pin GPIO_PIN_2
#define DEBUG_UART5_RX_GPIO_Port GPIOD
#define TP26_Pin GPIO_PIN_4
#define TP26_GPIO_Port GPIOD
#define TP28_Pin GPIO_PIN_5
#define TP28_GPIO_Port GPIOD
#define SDIO_SCK_Pin GPIO_PIN_6
#define SDIO_SCK_GPIO_Port GPIOD
#define SDIO_CMD_Pin GPIO_PIN_7
#define SDIO_CMD_GPIO_Port GPIOD
#define SDIO_D0_Pin GPIO_PIN_9
#define SDIO_D0_GPIO_Port GPIOG
#define SDIO_D1_Pin GPIO_PIN_10
#define SDIO_D1_GPIO_Port GPIOG
#define SDIO_D2_Pin GPIO_PIN_11
#define SDIO_D2_GPIO_Port GPIOG
#define SDIO_D3_Pin GPIO_PIN_12
#define SDIO_D3_GPIO_Port GPIOG
#define SD_DETECT_N_Pin GPIO_PIN_13
#define SD_DETECT_N_GPIO_Port GPIOG
#define MEMB_POWER_EN_Pin GPIO_PIN_14
#define MEMB_POWER_EN_GPIO_Port GPIOG
#define TP24_Pin GPIO_PIN_3
#define TP24_GPIO_Port GPIOB
#define TP25_Pin GPIO_PIN_4
#define TP25_GPIO_Port GPIOB
#define TP27_Pin GPIO_PIN_5
#define TP27_GPIO_Port GPIOB
#define TP29_Pin GPIO_PIN_6
#define TP29_GPIO_Port GPIOB
#define TP30_Pin GPIO_PIN_7
#define TP30_GPIO_Port GPIOB
#define TP31_Pin GPIO_PIN_8
#define TP31_GPIO_Port GPIOB
#define SYNC_IN_N_Pin GPIO_PIN_9
#define SYNC_IN_N_GPIO_Port GPIOB
#define GUI_UART8_RX_Pin GPIO_PIN_0
#define GUI_UART8_RX_GPIO_Port GPIOE
#define GUI_UART8_TX_Pin GPIO_PIN_1
#define GUI_UART8_TX_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define CURRENT_SW_VERSION_ID (SW_VERSION_NUM_END-1)
#define VERSION_DETAILS "(Added power on commands for Blower -- 12 Jan 2022)"

#define MAX_COMMAND_LENGTH 510
#define UART_RX_DMA_BUFFER_SIZE MAX_COMMAND_LENGTH + 2

#define debugUart (huart5)
#define guiUart   (huart8)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
