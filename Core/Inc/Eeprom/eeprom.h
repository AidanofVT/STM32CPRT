

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

#define EEPROM_DEVICE_ADDRESS 0xA0
// const uint8_t readControlCode = 0xA1;

#define LOG_MAX_ADDR     (65535)                                    // 64 Kbyte EEPROM device
#define EEPROM_PAGE_SIZE   (128)                                    // 128 bytes x 512 pages = 65536 bytes

//
// The first EEPROM page (0x0 -  0x7F), 128 bytes, is for configuration constants and other values that have fixed locations
// The log, which wraps, starts at location 0x80 and ends at location 0xFFFF -- 65,408 bytes -- 4,088 16-byte log entries
//
#define PINSP_OFFSET_ADRS                                      (0)  //       2 bytes - 0x0000
#define PEXP_OFFSET_ADRS          (PINSP_OFFSET_ADRS         +  2)  //       2 bytes - 0x0002
#define PINSP_GAIN_ADRS           (PEXP_OFFSET_ADRS          +  2)  //       4 bytes - 0x0004
#define PEXP_GAIN_ADRS            (PINSP_GAIN_ADRS           +  4)  //       4 bytes - 0x0008
#define CAL_CONSTANTS_CRC_ADRS    (PEXP_GAIN_ADRS            +  4)  //       4 bytes - 0x000C
#define PROGRAM_CRC_ADRS          (CAL_CONSTANTS_CRC_ADRS    +  4)  //       4 bytes - 0x0010
#define MFG_DATE_ADRS             (PROGRAM_CRC_ADRS          +  4)  //      16 bytes - 0x0014
#define SERIAL_NUMBER_ADRS        (MFG_DATE_ADRS             + 16)  //       4 bytes - 0x0024
#define NIV_WORD_ADRS             (SERIAL_NUMBER_ADRS        +  4)  //       4 bytes - 0x0028
#define HUMAN_USE_WORD_ADRS       (NIV_WORD_ADRS             +  4)  //       4 bytes - 0x002C

#define START_UNUSED_BYTES_ADRS   (HUMAN_USE_WORD_ADRS       +  4)  //      69 bytes - 0x0030

#define EVENT_CNTR_ADDRESS        (START_UNUSED_BYTES_ADRS   + 69)  //       2 bytes - 0x0075 - Previous two byte locations before SUPERCAP_BYTE_ADDRESS
#define SUPERCAP_BYTE_ADDRESS     (EVENT_CNTR_ADDRESS        +  2)  //       1 byte  - 0x0077 - Previous byte location before WATCHDOG_BYTE_ADDRESS
#define WATCHDOG_BYTE_ADDRESS     (SUPERCAP_BYTE_ADDRESS     +  1)  //       1 byte  - 0x0078 - Previous byte location before LOG_WRAPPED_FLAG_ADDRESS
#define LOG_WRAPPED_FLAG_ADDRESS  (WATCHDOG_BYTE_ADDRESS     +  1)  //       1 byte  - 0x0079 - Previous byte location before MINUTE_CNTR_ADDRESS
#define MINUTE_CNTR_ADDRESS       (LOG_WRAPPED_FLAG_ADDRESS  +  1)  //       4 bytes - 0x007A - Previous four byte locations before LOG_WRITE_POINTER_ADDRESS
#define LOG_WRITE_POINTER_ADDRESS (MINUTE_CNTR_ADDRESS       +  4)  //       2 bytes - 0x007E - Last two byte locations of Page 0

#define LOG_START_ADDR            (LOG_WRITE_POINTER_ADDRESS +  2)  //  65,408 bytes - 0x0080 - Start log at first address of Page 1

#define EEPROM_TX_QUEUE_SIZE         (32)
#define MAX_NUM_LOG_BYTES_PER_READ  (512)                           // Limit a single EEPROM read length to one page (512 bytes)

typedef struct __attribute__((__packed__)) LOG_INFO {
	uint32_t minuteCount;
	union {
		float floatValue;
		uint32_t uint32Value;
		uint8_t byteValues[4];
	} value;
	uint8_t code;
	uint16_t eventCount;
	uint16_t logWritePointer;
	uint8_t spare[3];
} logInfo;

#define LOG_EVENT_SIZE (sizeof(logInfo))

typedef struct {
	uint16_t address;
	uint8_t numBytes;
	uint8_t data[LOG_EVENT_SIZE];
} eepromQueueInfo;

typedef struct __attribute__((__packed__)) CAL_CONSTANTS {
	int16_t pInspOffset;
	int16_t pExpOffset;
	union {
		float floatValue;
		int32_t int32Value;
		uint8_t byteValues[4];
	} pInspGain;
	union {
		float floatValue;
		int32_t int32Value;
		uint8_t byteValues[4];
	} pExpGain;
	uint32_t crc32;
} calConstants;

void InitializeLog( );
void InitEepromTxQueue();
void GetLogWritePointerFromEeprom( );
void StoreLogWritePointerInEeprom( uint16_t logWritePointer );
void ReadTxEepromQueueAndWriteEeprom( );
void StoreMinuteCntrInEeprom( uint32_t minuteCounter );
void StoreEventCntrInEeprom( uint16_t eventCounter );
void GetMinuteCounterFromEeprom( );
void GetEventCounterFromEeprom( );
uint16_t GetEventCount( );
uint16_t GetLogWritePointer( );
void SetEventCount( uint16_t eventCount );
void TestEepromRead( uint16_t address, uint16_t numBytes );
void TestEepromWrite( uint16_t address, uint8_t * data, uint16_t numBytes );
void WriteLogEvent( uint8_t * logEvent );
void DumpEepromLog( );
void ProcessEepromReadComplete( );
bool GetEepromWriteWait( );
void SetEepromWriteWait( bool eepromWriteWaitFlag );
void StorePinspOffsetInEeprom( int16_t pInspOffset );
void StorePexpOffsetInEeprom( int16_t pExpOffset );
void StorePinspGainInEeprom( float pInspGain );
void StorePexpGainInEeprom( float pExpGain );
void ReadCalConstants( );
void GetCalConstants( calConstants * calConsts );
void StoreCalConstantsCrcInEeprom( );
void DisplayLogData( );
int8_t GetCalConstantsCrcGood( );
void StoreWatchdogByteInEeprom( uint8_t watchdogByte );
uint8_t GetWatchdogByteFromEprom( );
void StoreSupercapByteInEeprom( uint8_t supercapByte );
uint8_t GetSupercapByteFromEprom( );
void SetDisplayLogDataFlag( bool enable );
bool GetDisplayLogDataFlag( );
bool GetDisplayEepromTestFlag( );
void SetDisplayEepromTestFlag( bool enable );
void DisplayEepromTestRead( );
void StoreProgramCrcInEeprom( uint32_t programCrc );
void SetProgramCrcInEeprom( );
uint32_t GetProgramCrcFromEeprom( );
void StoreMfgDateInEeprom( char * mfgDate );
void GetMfgDateFromEeprom( );
void StoreSerialNumberInEeprom( uint32_t serialNum );
void GetSerialNumberFromEeprom( );
uint32_t GetNivWord( );
uint32_t GetHumanUseWord( );
void GetNivWordFromEeprom( );
void GetHumanUseWordFromEeprom( );
void StoreNivWordInEeprom( uint32_t nivWord );
void StoreHumanUseWordInEeprom( uint32_t humanUseWord );


#endif /* __EEPROM_H */
