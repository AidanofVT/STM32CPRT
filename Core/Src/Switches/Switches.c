#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "Model.h"
#include "Switches.h"
#include "Display.h"

#define MAX_TIME_INT_SIGNAL_LOW_MS       (500)

extern I2C_HandleTypeDef hi2c2;


extern bool bPfcLoadEnable;
extern uint32_t uPfcLoadEnTimeDB;

extern bool bPowerFailWarn;
extern uint32_t uPowerFailWarnTimeDB;

//=====================Membrane Switches==========================================================================

#define KEY_FIFO_REG                   (0x00) //Read only 0x3F Read FIFO keyscan data out
#define CFG_REG                        (0x01) //R/W 0x0B Configuration Power-down, key-release enable, autowake, and I2C timeout enable
#define DEBOUNCE_KEY_REG               (0x02) //R/W 0xFF debounce time setting
#define INTERRUPT_KS_REG               (0x03) //R/W 0x00 Interrupt Key-switch interrupt and INT frequency setting
#define KEY_REPEAT_REG                 (0x05) //R/W 0x00 Key repeat Delay and frequency for key repeat
#define SLEEP_REG                      (0x06) //R/W 0x07 Sleep Idle time to autosleep
#define KEY_SWITCH_SIZE_REG            (0x30) //R/W 0xFF Key-switch size Keyscan switch array size
#define LED_DRIVER_EN_REG              (0x31) //R/W 0x00 LED driver enable LED driver enable register
#define ROW70_EN_REG                   (0x32) //R/W 0xFF GPI enable GPI enable for ROW7�ROW0
#define COL70_EN_REG                   (0x33) //R/W 0xFF GPI enable GPI enable for COL7�COL0
#define ROW70_DIR_REG                  (0x34) //R/W 0x00 GPIO direction 1 GPIO input/output control register 1 for ROW7�ROW0
#define COL70_DIR_REG                  (0x35) //R/W 0x00 GPIO direction 2 GPIO input/output control register 2 for COL7�COL0
#define ROW70_PUSH_PULL_REG            (0x36) //R/W 0xFF GPO output mode 1 GPO open-drain/push-pull output setting for ROW7�ROW0
#define COL70_PUSH_PULL_REG            (0x37) //R/W 0x0F GPO output mode 2GPO open-drain/push-pull output setting for COL7�COL0
#define SUPPLY_VOLT_ROW70_REG      	   (0x38) //R/W 0x00 GPIO supply voltage 1	GPIO voltages supplied by VCC or VLA for ROW7�ROW0
#define SUPPLY_VOLT_COL70_REG      	   (0x39) //R/W 0x00 GPIO supply voltage 2	GPIO voltages supplied by VCC or VLA for COL7�COL0
#define DEBOUNCED_VAL_ROW70_REG        (0x3A) //R/W 0xFF GPIO values 1 Debounced input or output values of ROW7�ROW0
#define DEBOUNCED_VAL_COL70_REG        (0x3B) //R/W 0xFF GPIO values 2 Debounced input or output values of COL7�COL0
#define LEVEL_SHIFTER_REG      		   (0x3C) //R/W 0x00 GPIO levelshifter enable GPIO direct level-shifter pair enable
#define GLOBAL_CFG_REG      		   (0x40) //R/W 0x00 GPIO global configuration GPIO global enable, GPIO reset, LED fade enable
#define GLOBAL_DEBOUNCE_ROW70_REG      (0x42) //R/W 0x00 GPIO debounce ROW7�ROW0 debounce time setting
#define COL70_CC_SETTING_REG           (0x43) //R/W 0xC0 LED constantcurrent setting COL7�COL4 constant-current output setting
#define COMMON_PWM_REG		           (0x45) //R/W 0x00 Common PWM Common PWM duty-cycle setting
#define I2C_TIMEOUT_REG		           (0x48) //Read only 0x00 I2C timeout flag I2C timeout since last POR
#define LED_R_PWM_REG      			   (0x50) //R/W 0x00 COL4 PWM ratio COL4 individual duty-cycle setting
#define LED_G_PWM_REG      			   (0x51) //R/W 0x00 COL5 PWM ratio COL5 individual duty-cycle setting
#define LED_B_PWM_REG      			   (0x52) //R/W 0x00 COL6 PWM ratio COL6 individual duty-cycle setting
#define COL7_PWM_REG                   (0x53) //R/W 0x00 COL7 PWM ratio COL7 individual duty-cycle setting
#define LED_R_CFG_REG    	           (0x54) //R/W 0x00 COL4 LED configuration COL4 interrupt, PWM mode control, and blinkperiod settings
#define LED_G_CFG_REG      			   (0x55) //R/W 0x00 COL5 LED configuration COL5 interrupt, PWM mode control, and blinkperiod settings
#define LED_B_CFG_REG      			   (0x56) //R/W 0x00 COL6 LED configuration COL6 interrupt, PWM mode control, and blinkperiod settings
#define COL7_CFG_REG		           (0x57) //R/W 0x00 COL7 LED configuration COL7 interrupt, PWM mode control, and blinkperiod settings
#define ROW70_INT_MASK_REG             (0x58) //R/W 0xFF Interrupt mask 1 Interrupt mask for ROW7�ROW0
#define COL70_INT_MASK_REG	           (0x59) //R/W 0xFF Interrupt	mask 2 Interrupt mask for COL7�COL0
#define ROW70_TRIGGER_MODE_REG         (0x5A) //R/W 0x00 GPI trigger mode 1 GPI edge-triggered detection setting for ROW7�ROW0
#define COL70_TRIGGER_MODE_REG         (0x5B) //R/W 0x00 GPI trigger mode 2 GPI edge-triggered detection setting for COL7�COL0


#define STANBY_SWITCH_PRESS_1_SEC      (SEC_TO_TICK(1))
#define KEY_HOLD_2_SEC                 (SEC_TO_TICK(2))
#define KEY_HOLD_4_SEC                 (SEC_TO_TICK(4))
#define KEY_HOLD_20_SEC                (SEC_TO_TICK(20))
#define KEY_NUMBER_MASK                (0x3F)
#define KEY_REPEAT_LAST_DATA           (0x3E)
#define KEY_REPEAT_MORE_DATA           (0x7E)


#define KEY_RELEASE                    (0x40)
#define KEY_NOW                        (gstModel.gKeyCode & KEY_NUMBER_MASK)

typedef struct
{
	KEY_STATUS eKeyStatus;
	SWITCH_ID  eKeyID;
	uint8_t    uKeyCode;
	uint32_t   uLastActiveTime;
	uint32_t   uFirstRepeatTime;
} KEY_ACTIVE;

typedef struct
{
	uint8_t    uKeyCode;
} KEY_CODE_t;

KEY_CODE_t astKeyTranslate[] =
{
		[ SWITCH_ID_NONE  ]   = { 0xFF  },
	    [ SWITCH_ID_MUTE  ]   = { MUTE_S1_KEYCODE  },
		[ SWITCH_ID_START ]   = { START_S2_KEYCODE },
	    [ SWITCH_ID_MENU  ]   = { MENU_S3_KEYCODE  },
	    [ SWITCH_ID_PAUSE ]   = { PAUSE_S4_KEYCODE },
	    [ SWITCH_ID_LEFT  ]   = { LEFT_S5_KEYCODE  },
	    [ SWITCH_ID_UP    ]   = { UP_S6_KEYCODE    },
	    [ SWITCH_ID_DOWN  ]   = { DOWN_S7_KEYCODE  },
	    [ SWITCH_ID_SEL   ]   = { SEL_S8_KEYCODE   },
	    [ SWITCH_ID_RIGHT ]   = { RIGHT_S9_KEYCODE },
};

typedef struct
{
	KEY_ACTIVE        stKeyActive;
} CONTEXT;

static CONTEXT c;
static CONTEXT *gstSwitch;
static bool keyboardFail            = false;
uint32_t    wMAX7370LastRead;

//New Membrane implementation
uint8_t send_data[2] = { DEBOUNCED_VAL_COL70_REG, 0x70 };
static uint8_t led_start_pause   = 0x70;

static void MAX7370_write(uint8_t *pData, uint16_t Size);
static void MAX7370_init();
static void MAX7370_read();

//=============================Switches Membrane=========================================================

static void MAX7370_write(uint8_t *pData, uint16_t Size)
{
	 HAL_StatusTypeDef ret;

	 ret = HAL_I2C_Master_Transmit(&hi2c2, MAX7370_ADR, pData, Size, HAL_MAX_DELAY);

	 if(ret != HAL_OK)
	 {
		 UartPrintf("MAX7370 Setup write err %x\n", ret);
	 }
}

static bool isMembraneIntStillLow()
{
	bool bStatus;
	bStatus = HAL_GPIO_ReadPin(N_MEMBRANE_INT_GPIO_Port, N_MEMBRANE_INT_Pin) ? false : true;
	return bStatus;
}

static void MAX7370_read(void)
{
	  HAL_StatusTypeDef ret;

	  uint8_t cmd = KEY_FIFO_REG;

      ret = HAL_I2C_Master_Transmit(&hi2c2, MAX7370_ADR, &cmd, 1, HAL_MAX_DELAY);

	  if(ret == HAL_OK)
	  {
		 ret = HAL_I2C_Master_Receive_IT(&hi2c2, MAX7370_ADR, &gstModel.gKeyCode, 1);

		 if(ret != HAL_OK)
		 {
			 UartPrintf("MAX7370 RCIT Read err %x\n", ret);
		 }
	  }
	  else
	  {
		  UartPrintf("MAX7370 TX Read err  %x\n", ret);
	  }
}

static void MAX7370_init()
{
	uint8_t data[2];

	data[0] = GLOBAL_CFG_REG;
    data[1] = 0x10;                  // Normal Op: GPIO,PWM,CCS
    MAX7370_write((uint8_t *)&data, 2);

    data[0] = CFG_REG;
    data[1] = 0x89;                  // Key-switch operation mode, key-release en, timout disable
    MAX7370_write((uint8_t *)&data, 2);

	data[0] = KEY_REPEAT_REG;
	data[1] = 0xFF;                  // Key-switch auto-repeat enable Autorepeat frequency is 32 debounce cycles, Autorepeat delay is 128 debounce cycles
	MAX7370_write((uint8_t *)&data, 2);

    data[0] = KEY_SWITCH_SIZE_REG;
    data[1] = 0x33;                  // Keypad size: ROW 2:0, COL 2:0
    MAX7370_write((uint8_t *)&data, 2);

    //------ROW/COL signal characteristics
    //ROW7:0 default as PP
    //COL7:4 default as OP, COL3:0 default as PP

    //.......ROW/COL Direction for Keypad Operation
    //ROW7:0 default all Inputs

    data[0] = COL70_DIR_REG;
    data[1] = 0x30;                       // COL5:4 = out; COL3:0 = in; 0=in; 1=out; default=0x00
    MAX7370_write((uint8_t *)&data, 2);

    //............ Col 7:4 Constant Current Setting
    data[0] = COL70_CC_SETTING_REG;
    data[1] = 0xC1;                       // COL 7:4 0xC0=20mA; 0xC1=10mA
    MAX7370_write((uint8_t *)&data, 2);

    //Key-switches and Interrupt set up
    data[0] = DEBOUNCE_KEY_REG;
    data[1] = 0x0A;           // 20ms press debounce time
    MAX7370_write((uint8_t *)&data, 2);

    data[0] = INTERRUPT_KS_REG;
    data[1] = 0x01;           // /INT assert every debounce cycle
    MAX7370_write((uint8_t *)&data, 2);

    MAX7370_write((uint8_t *)&send_data, 2);//off all
}


SWITCH* Switches_Init(void)
{
	UartPrintf("Init MAX7370\n");

	MAX7370_init();

	gstSwitch = &c;

	return (SWITCH*) &c;
}

SWITCH_ID KeyCodeTranslation(uint8_t KeyCode)
{
	int i;

	for(i = 1; i < ARRAY_LEN(astKeyTranslate); i++)
	{
		if(KeyCode == astKeyTranslate[i].uKeyCode)
		{
			return i;
		}
	}

    return SWITCH_ID_NONE;
}

bool Switches_Scan(void)
{
	bool bSwitchPress = false;

	if( keyboardFail != false )
	{
		return bSwitchPress;
	}

	KEY_ACTIVE *pstKeyActive = &(gstSwitch->stKeyActive);

	//Serve the interrupt;
	if(gstModel.bMembranePress == true)
	{
		gstModel.bMembranePress = false;
		MAX7370_read();
		wMAX7370LastRead = GetTimeMs();
	}

	//Serve the KeyCode read back
	if(gstModel.bKeyCodeReady == true)
	{
		gstModel.bKeyCodeReady = false;

		//Check to see if key interrupt from press or release action
		if( (gstModel.gKeyCode & KEY_RELEASE) != KEY_RELEASE)
		{
			//Key press
			pstKeyActive->uKeyCode        = KEY_NOW; //save keycode

			//detection of the key stuck for more than 20 seconds
			if(KEY_REPEAT_LAST_DATA ==  pstKeyActive->uKeyCode)
			{
				if(pstKeyActive->uFirstRepeatTime == 0)
				{
				    pstKeyActive->uFirstRepeatTime = Get_SysTick(); //time stamp of the first repeat no data event
				}
				else
				{
					if( (Get_SysTick() - pstKeyActive->uFirstRepeatTime) >= KEY_HOLD_20_SEC)
					{
						//KEY_FAIL
						pstKeyActive->uFirstRepeatTime = 0;
						pstKeyActive->eKeyStatus = KEY_FAIL;
						keyboardFail = true;
						UartPrintf("Switch held too Long, Disable Switch Scan\n");
					}
				}
			}
			else
			{
				pstKeyActive->uLastActiveTime  = Get_SysTick(); //store the key press
				pstKeyActive->uFirstRepeatTime = 0;
			}
		}
		else
		{
			//Key release, only validate Keycode with previously press
			uint8_t uKeyCodeNow = KEY_NOW;

			if(pstKeyActive->uLastActiveTime && (uKeyCodeNow == pstKeyActive->uKeyCode))
			{
				if( (Get_SysTick() - pstKeyActive->uLastActiveTime) >= KEY_HOLD_4_SEC)
				{
					//KEY_HOLD_4_SEC
					pstKeyActive->eKeyStatus = KEY_HOLD_4_S;
				}
				else if( (Get_SysTick() - pstKeyActive->uLastActiveTime) >= KEY_HOLD_2_SEC)
				{
					//KEY_HOLD_2_SEC
					pstKeyActive->eKeyStatus = KEY_HOLD_2_S;
				}
				else
				{
					pstKeyActive->eKeyStatus = KEY_PRESS;
				}

				pstKeyActive->eKeyID = KeyCodeTranslation(pstKeyActive->uKeyCode); //translation from KeyCode to enum ID
				pstKeyActive->uLastActiveTime = 0; //clean up temp holder
				if( keyboardFail  == false )
				{
					bSwitchPress = true;
					UartPrintf("Key Active: %d\n", pstKeyActive->eKeyID);

					//If LED reset key detect reset the power for LCD
					//if(SWITCH_ID_LED_RESET == pstKeyActive->eKeyID)
					//{
					//	 gstModel.bLCDResetRequest = true;
					//}

				}
			}
		}
	}

	//Fail safe check to see the interrupt line still low, do another read
	//to make sure the fifo is clear
	if(isMembraneIntStillLow() == true)
	{
		if((GetTimeMs() - wMAX7370LastRead) >= MAX_TIME_INT_SIGNAL_LOW_MS)
		{
		    MAX7370_read();
		}
	}

	return bSwitchPress;

}

void Switches_Get_Status( SWITCH_ID * eSwitchId, KEY_STATUS * eKeyStatus )
{
	KEY_ACTIVE *pstKeyActive 	= &(gstSwitch->stKeyActive);
	*eSwitchId        			= pstKeyActive->eKeyID;
	*eKeyStatus 				= pstKeyActive->eKeyStatus;
}


void Switches_Power_Onoff(bool bOnOff)
{
	if(bOnOff == true)
	{
		HAL_GPIO_WritePin(MEMB_POWER_EN_GPIO_Port, MEMB_POWER_EN_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(MEMB_POWER_EN_GPIO_Port, MEMB_POWER_EN_Pin, GPIO_PIN_RESET);
	}
}

void LED_Start_On()
{
	uint8_t data[2];
    data[0] = DEBOUNCED_VAL_COL70_REG;
    led_start_pause &= ~(1<<4); //COL4 = 0
    data[1] = led_start_pause;
    MAX7370_write((uint8_t *)&data, 2);
}

void LED_Start_Off()
{
	uint8_t data[2];
    data[0] = DEBOUNCED_VAL_COL70_REG;
    led_start_pause |= (1<<4); //COL4 = 1
    data[1] = led_start_pause;
    MAX7370_write((uint8_t *)&data, 2);
}

void LED_Pause_On()
{
	uint8_t data[2];
    data[0] = DEBOUNCED_VAL_COL70_REG;
    led_start_pause &= ~(1<<5); //COL4 = 0
    data[1] = led_start_pause;
    MAX7370_write((uint8_t *)&data, 2);
}

void LED_Pause_Off()
{
	uint8_t data[2];
    data[0] = DEBOUNCED_VAL_COL70_REG;
    led_start_pause |= (1<<5); //COL4 = 1
    data[1] = led_start_pause;
    MAX7370_write((uint8_t *)&data, 2);
}
