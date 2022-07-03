
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "eeprom.h"



extern I2C_HandleTypeDef hi2c3;

static calConstants calibrationConstants;

static char manufactureDate[16] = "09/29/1970";
static uint32_t serialNumber = 0;

static eepromQueueInfo eepromTxQueue[EEPROM_TX_QUEUE_SIZE];
static uint16_t eepromTxQueueReadIndex = 0;
static uint16_t eepromTxQueueWriteIndex = 0;

static uint32_t maxEepromTxQueueWaitTimeMs = 0;
static uint16_t maxTxEepromQueueDepth = 0;
static uint16_t curTxEepromQueueDepth = 0;

static uint16_t eepromLogWritePtr;

static uint32_t calcCalConstCrc;
static int8_t calConstantsCrcGood = -1;
static uint32_t minuteCounter;
static uint32_t nivWordValue;
static uint32_t humanUseWordValue;
static uint16_t eventCounter;
static uint8_t logWrappedFlag = 0;

static uint8_t logData[MAX_NUM_LOG_BYTES_PER_READ];
static uint16_t numLogBytesToRead = MAX_NUM_LOG_BYTES_PER_READ;
static uint16_t logReadPointer;

static uint8_t readDataBuf[MAX_NUM_LOG_BYTES_PER_READ];
static uint16_t numTestReadBytes = 0;
static uint16_t testReadAddress = 0;

static bool getLogWritePtr = false;
static bool getMinuteCounter = false;
static bool getEventCounter = false;
static bool getNivWord = false;
static bool getHumanUseWord = false;
static bool getProgramCrc = false;
static bool eepromWriteWait = false;
static bool displayLogData = false;
static bool displayEepromTest = false;
static bool getWatchdogByte = false;
static bool getSupercapByte = false;
static bool testRead = false;
static bool getCalConstants = false;
static bool dumpEepromFlag = false;
static bool initiateLogRead = false;
static bool startLogDump = false;
static bool logReadComplete = false;
static bool getMfgDate = false;
static bool getSerialNumber = false;


bool GetDisplayLogDataFlag( )
{
	return displayLogData;
}

void SetDisplayLogDataFlag( bool enable )
{
	displayLogData = enable;
}

bool GetDisplayEepromTestFlag( )
{
	return displayEepromTest;
}

void SetDisplayEepromTestFlag( bool enable )
{
	displayEepromTest = enable;
}

bool GetEepromWriteWait( )
{
	return eepromWriteWait;
}

void SetEepromWriteWait( bool eepromWriteWaitFlag )
{
	eepromWriteWait = eepromWriteWaitFlag;
}

void GetCalConstants( calConstants * calConsts )
{
    calConsts->pInspOffset = calibrationConstants.pInspOffset;
    calConsts->pExpOffset = calibrationConstants.pExpOffset;
    calConsts->pInspGain = calibrationConstants.pInspGain;
    calConsts->pExpGain = calibrationConstants.pExpGain;
}

uint16_t GetEventCount( )
{
	return eventCounter;
}

uint16_t GetLogWritePointer( )
{
	return eepromLogWritePtr;
}

uint32_t GetNivWord( )
{
	return nivWordValue;
}

uint32_t GetHumanUseWord( )
{
	return humanUseWordValue;
}

/*-----------------------------------------------------------------------------
 * Function:    ReadEeprom
 *
 * Description: Read data bytes from EEPROM.
 *
 * Parameters: dataAddress - start address in EEPROM from where bytes are to be read
 *             readData - pointer to data where bytes to be read from the EEPROM are stored
 *             numBytesToRead - number of bytes to read from EEPROM
 *
 * Returns:     None
 *---------------------------------------------------------------------------*/
void ReadEeprom( uint16_t dataAddress, uint8_t * readData, int16_t numBytesToRead )
{
	if( HAL_I2C_Mem_Read_IT( &hi2c3, EEPROM_DEVICE_ADDRESS, dataAddress, I2C_MEMADD_SIZE_16BIT, readData, numBytesToRead )
			!= HAL_OK)
	{
		UartPrintf("I2C3 EEPROM read failed.\n");
	}
}


//
// Read once at initialization
//
void GetLogWritePointerFromEeprom( )
{
	getLogWritePtr = true;

	ReadEeprom( LOG_WRITE_POINTER_ADDRESS, (uint8_t *)&eepromLogWritePtr, 2 );
}

void GetMinuteCounterFromEeprom( )
{
	getMinuteCounter = true;

	ReadEeprom( MINUTE_CNTR_ADDRESS, (uint8_t *)&minuteCounter, 4 );
}

void GetEventCounterFromEeprom( )
{
	getEventCounter = true;

	ReadEeprom( EVENT_CNTR_ADDRESS, (uint8_t *)&eventCounter, 2 );
}

void GetNivWordFromEeprom( )
{
	getNivWord = true;

	ReadEeprom( NIV_WORD_ADRS, (uint8_t *)&nivWordValue, 4 );
}

void GetHumanUseWordFromEeprom( )
{
	getHumanUseWord = true;

	ReadEeprom( HUMAN_USE_WORD_ADRS, (uint8_t *)&humanUseWordValue, 4 );
}

uint32_t GetProgramCrcFromEeprom( )
{
	uint32_t storedProgramCrc = 0;

	getProgramCrc = true;

    // Function is only called during initialization so 12 ms delay is okay
	// This delay is to ensure that all other EEPROM accesses have completed
    HAL_Delay(12);

	uint32_t startWaitTime = GetTimeMs();
	while (HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY)
	{
		if ( DiffTimeFromStartTimeMs(startWaitTime) > 3 ) // 3 ms
		{
			UartPrintf("I2C3 EEPROM transfer not complete for 3 ms\n");
			return 1; // Wait too long for I2C3 EEPROM transfer to complete
		}
	}

	ReadEeprom( PROGRAM_CRC_ADRS, (uint8_t *)&storedProgramCrc, 4 );

	startWaitTime = GetTimeMs();
	while( getProgramCrc != false )
	{
		// Wait until program CRC is returned from I2C
		if ( DiffTimeFromStartTimeMs(startWaitTime) > 3 ) // 3 ms
		{
			UartPrintf("I2C3 EEPROM transfer not complete for 3 ms\n");
			return 1; // Wait too long for I2C3 EEPROM transfer to complete
		}
	}

	UartPrintf("Stored Program CRC: 0x%08x\n", storedProgramCrc);

	return storedProgramCrc;
}

uint8_t GetWatchdogByteFromEprom( )
{
	uint8_t watchdogByte = 0;

    getWatchdogByte = true;

    // Function is only called during initialization so 12 ms delay is okay
    // This delay is to ensure that all other EEPROM accesses have completed
    HAL_Delay(12);

	uint32_t startWaitTime = GetTimeMs();
	while (HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY)
	{
		if ( DiffTimeFromStartTimeMs(startWaitTime) > 3 ) // 3 ms
		{
			UartPrintf("I2C3 EEPROM transfer not complete for 3 ms\n");
			return 1; // Wait too long for I2C3 EEPROM transfer to complete
		}
	}

	ReadEeprom( WATCHDOG_BYTE_ADDRESS, &watchdogByte, 1 );

	startWaitTime = GetTimeMs();
	while( getWatchdogByte != false )
	{
		// Wait until watchdog byte is returned from I2C
		if ( DiffTimeFromStartTimeMs(startWaitTime) > 3 ) // 3 ms
		{
			UartPrintf("I2C3 EEPROM transfer not complete for 3 ms\n");
			return 1; // Wait too long for I2C3 EEPROM transfer to complete
		}
	}

	return watchdogByte;
}

uint8_t GetSupercapByteFromEprom( )
{
	uint8_t supercapByte = 0;

    getSupercapByte = true;

    // Function is only called during manufacturing test so 12 ms delay is okay
    // This delay is to ensure that all other EEPROM accesses have completed
    HAL_Delay(12);

	uint32_t startWaitTime = GetTimeMs();
	while (HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY)
	{
		if ( DiffTimeFromStartTimeMs(startWaitTime) > 3 ) // 3 ms
		{
			UartPrintf("I2C3 EEPROM transfer not complete for 3 ms\n");
			return 1; // Wait too long for I2C3 EEPROM transfer to complete
		}
	}

	ReadEeprom( SUPERCAP_BYTE_ADDRESS, &supercapByte, 1 );

	startWaitTime = GetTimeMs();
	while( getSupercapByte != false )
	{
		// Wait until supercap byte is returned from I2C
		if ( DiffTimeFromStartTimeMs(startWaitTime) > 3 ) // 3 ms
		{
			UartPrintf("I2C3 EEPROM transfer not complete for 3 ms\n");
			return 1; // Wait too long for I2C3 EEPROM transfer to complete
		}
	}

	return supercapByte;
}

/*-----------------------------------------------------------------------------
 * Function:    InitEepromTxQueue
 *
 * Description: Initialize Tx EEPROM queue read and write indexes to 0.
 *
 * Parameters:  None
 *
 * Returns:     None
 *---------------------------------------------------------------------------*/
void InitEepromTxQueue()
{
	eepromTxQueueReadIndex = 0;
	eepromTxQueueWriteIndex = 0;

	GetLogWritePointerFromEeprom( );

	UartPrintf("Size of logInfo: %d\n", LOG_EVENT_SIZE);
}

/*-----------------------------------------------------------------------------
 * Function:    IsTxEepromQueueEmpty
 *
 * Description: Determines if Tx EEPROM queue is empty.
 *
 * Parameters:  None
 *
 * Returns:     true if empty, false if not empty
 *---------------------------------------------------------------------------*/
bool IsTxEepromQueueEmpty( )
{
	int16_t queueIndexDiff = eepromTxQueueWriteIndex - eepromTxQueueReadIndex;
	curTxEepromQueueDepth = queueIndexDiff;

	if( queueIndexDiff == 0 )
	{
		// Tx EEPROM queue is empty
		return true;
	}

	if( queueIndexDiff < 0 )
	{
		queueIndexDiff += EEPROM_TX_QUEUE_SIZE;
	}

	// Keep track of maximum Tx EEPROM queue depth
	if( queueIndexDiff > maxTxEepromQueueDepth )
	{
		maxTxEepromQueueDepth = queueIndexDiff;
	}

	// Tx EEPROM queue is not empty
	return false;
}

/*-----------------------------------------------------------------------------
 * Function:    IsTxEepromQueueFull
 *
 * Description: Determines if Tx EEPROM queue is full.
 *
 * Parameters:  None
 *
 * Returns:     true if full, false if not full
 *---------------------------------------------------------------------------*/
bool IsTxEepromQueueFull( )
{
	int16_t queueIndexDiff = eepromTxQueueWriteIndex - eepromTxQueueReadIndex;

	if( queueIndexDiff < 0 )
    {
		queueIndexDiff += EEPROM_TX_QUEUE_SIZE;
	}

	if( queueIndexDiff == (EEPROM_TX_QUEUE_SIZE - 1) )
	{
		// Tx EEPROM queue is full
		return true;
	}

	// Tx EEPROM queue is not full
	return false;
}

// NOTE:
//
// EEPROM Page write operations are limited to writing bytes within a single physical page (128 bytes),
// regardless of the number of bytes actually being written to the EEPROM.
// Physical page boundaries start at addresses that are integer multiples of the page buffer size
// (or ‘page size’, e.g. 0, 128, 256, etc. ) and end at addresses that are integer
// multiples of [page size - 1] (127, 255, etc.). If a Page Write command attempts
// to write across a physical page boundary, the result is that the data wraps around to the
// beginning of the current page (overwriting data previously stored there), instead of
// being written to the next page. It is, therefore, necessary for the application software to
// prevent page write operations that would attempt to cross a page boundary.
//
/*-----------------------------------------------------------------------------
 * Function:    WriteEeprom
 *
 * Description: Write data bytes to EEPROM.
 *
 * Parameters: dataAddress - start address in EEPROM where bytes are to be written
 *             writeData - pointer to data bytes to be written to the EEPROM
 *             numBytesToWrite - number of bytes to write to EEPROM
 *
 * Returns:     None
 *---------------------------------------------------------------------------*/
uint8_t WriteEeprom( uint16_t dataAddress, uint8_t * writeData, uint16_t numBytesToWrite )
{
	uint16_t numBytesRemaining = numBytesToWrite;
	uint16_t numBytesWritten = 0;

	uint8_t pageStartAddress = dataAddress % EEPROM_PAGE_SIZE;
	uint8_t numBytesToEndOfPage = EEPROM_PAGE_SIZE - pageStartAddress;

	while( numBytesRemaining > 0 )
	{
		// Wait for I2C3 to be ready
		uint32_t startWaitTime = GetTimeMs();

		while( eepromWriteWait != false )
		{
			// Wait until EEPROM write wait time has elapsed before writing EEPROM
			// UartPrintf("I2C EEPROM wait time has not elasped\n");
			if ( DiffTimeFromStartTimeMs(startWaitTime) > 10 ) // 10 ms
			{
				UartPrintf("I2C3 EEPROM transfer not complete for 10 ms\n");
				return 1; // Wait too long for I2C3 EEPROM transfer to complete
			}
		}

		while (HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY)
		{
			if ( DiffTimeFromStartTimeMs(startWaitTime) > 3 ) // 3 ms
			{
				UartPrintf("I2C3 EEPROM transfer not complete for 3 ms\n");
				return 1; // Wait too long for I2C3 EEPROM transfer to complete
			}
		}

		if( numBytesRemaining <= numBytesToEndOfPage )
		{
			// All remaining bytes will fit in page
			eepromWriteWait = true;
			if( HAL_I2C_Mem_Write_IT( &hi2c3, EEPROM_DEVICE_ADDRESS, dataAddress + numBytesWritten, I2C_MEMADD_SIZE_16BIT,
					&writeData[numBytesWritten], numBytesRemaining ) != HAL_OK )
			{
				UartPrintf("I2C3 EEPROM write to signal page failed.\n");
				return 1;
			}
			numBytesWritten = numBytesRemaining;
		}
		else // Number of bytes to write requires multiple pages
		{
			eepromWriteWait = true;
			if( HAL_I2C_Mem_Write_IT( &hi2c3, EEPROM_DEVICE_ADDRESS, dataAddress + numBytesWritten,
					I2C_MEMADD_SIZE_16BIT, &writeData[numBytesWritten], numBytesToEndOfPage ) != HAL_OK )
			{
				UartPrintf("I2C3 EEPROM write to multiple pages failed.\n");
				return 1;
			}
			numBytesWritten = numBytesToEndOfPage;
		}

		// Update number of bytes remaining to be written to EEPROM
		numBytesRemaining -= numBytesWritten;

		// After write to initial page, subsequent writes start at the beginning of the next page
		// and the full page is available
		numBytesToEndOfPage = EEPROM_PAGE_SIZE;
	}

	return 0;
}

/*-----------------------------------------------------------------------------
 * Function:    WriteTxEepromQueue
 *
 * Description: When the Tx EEPROM queue is not full, the queue entry pointed to
 *              by the queue write index is written with that address. the number
 *              of bytes to be written and the data in the passed buffer and the
 *              queue write index is incremented
 *
 * Parameters:  buffer - pointer to data to be written to Tx UART queue
 *
 * Returns:     None
 *---------------------------------------------------------------------------*/
int WriteTxEepromQueue(uint16_t address, uint8_t * writeData, uint8_t numBytes )
{
	uint32_t startWaitTime = GetTimeMs();

	while( IsTxEepromQueueFull() != false )
	{
		// Wait for Tx EEPROM queue not to be full
		uint32_t waitTime = DiffTimeFromStartTimeMs(startWaitTime);
		if ( waitTime > 10 ) // 10 ms
		{
			UartPrintf("\r\nI2C IT EEPROM queue full for 10 ms\r\n");
			return -1; // Wait too long for Tx EEPEROM queue not to be full
		}

		// Keep track of max wait time for Tx UART Queue not to be full
		if( waitTime > maxEepromTxQueueWaitTimeMs )
		{
			maxEepromTxQueueWaitTimeMs = waitTime;
		}
	}

	// Write info to Tx EEPROM queue
	eepromTxQueue[eepromTxQueueWriteIndex].address = address;
	eepromTxQueue[eepromTxQueueWriteIndex].numBytes = numBytes;
	for( int i=0; i<numBytes; i++ )
	{
		eepromTxQueue[eepromTxQueueWriteIndex].data[i] = *writeData++;
	}

	// Increment EEPROM Tx queue write index
	eepromTxQueueWriteIndex++;
	if( eepromTxQueueWriteIndex >= EEPROM_TX_QUEUE_SIZE )
	{
		// Write index was at maximum, set to 0
		eepromTxQueueWriteIndex =  0;
	}

	// If I2C3 is ready, read data from TX EEPROM queue and write to EEPROM,
	// else return and HAL_I2C_MemTxCpltCallback will call ReadTxEepromQueueAndWriteEeprom
	// after current I2C3 transmission is complete
	if(HAL_I2C_GetState(&hi2c3) == HAL_I2C_STATE_READY)
	{
		ReadTxEepromQueueAndWriteEeprom();
	}

	return 0;
}

/*-----------------------------------------------------------------------------
 * Function:    ReadTxEepromQueueAndWriteEeprom
 *
 * Description: If the Tx EEPROM queue is not empty, the queue entry pointed to
 *              by the queue read index is read and written to the EEPROM
 *              with IT and the queue read index is incremented
 *
 * Parameters:  None
 *
 * Returns:     None
 *---------------------------------------------------------------------------*/
void ReadTxEepromQueueAndWriteEeprom()
{
	if( IsTxEepromQueueEmpty() != false )
	{
		// Tx Eeprom queue is empty, nothing to read and transmit
		// UartPrintf("I2C EEPROM Tx queue is empty\n");
		return;
	}

	if( eepromWriteWait != false )
	{
		// Wait until EEPROM write wait time has elapsed before writing EEPROM
		// UartPrintf("I2C EEPROM wait time has not elasped\n");
		return;
	}

	// Read data from Tx EEPROM queue
    uint16_t dataAddress = eepromTxQueue[eepromTxQueueReadIndex].address;
    uint8_t numBytesToWrite = eepromTxQueue[eepromTxQueueReadIndex].numBytes;
    uint8_t * writeData = &(eepromTxQueue[eepromTxQueueReadIndex].data[0]);

	// Increment Tx EEPROM queue read index
	eepromTxQueueReadIndex++;
	if( eepromTxQueueReadIndex >= EEPROM_TX_QUEUE_SIZE )
	{
		// Read index was at maximum, set to 0
		eepromTxQueueReadIndex =  0;
	}

	// Write data to EEPROM
	if( WriteEeprom( dataAddress, writeData, numBytesToWrite ) != 0 )
	{
		UartPrintf("I2C IT Transmit ERROR in ReadTxEepromQueueAndTransmit\n");
	}
}

void StorePinspOffsetInEeprom( int16_t pInspOffset )
{
	WriteTxEepromQueue(PINSP_OFFSET_ADRS, (uint8_t *)&pInspOffset, 2 );
}

void StorePexpOffsetInEeprom( int16_t pExpOffset )
{
	WriteTxEepromQueue(PEXP_OFFSET_ADRS, (uint8_t *)&pExpOffset, 2 );
}

void StorePinspGainInEeprom( float pInspGain )
{
	WriteTxEepromQueue(PINSP_GAIN_ADRS, (uint8_t *)&pInspGain, 4 );
}

void StorePexpGainInEeprom( float pExpGain )
{
	WriteTxEepromQueue(PEXP_GAIN_ADRS, (uint8_t *)&pExpGain, 4 );
}

void StoreLogWritePointerInEeprom( uint16_t logWritePointer )
{
	WriteTxEepromQueue(LOG_WRITE_POINTER_ADDRESS, (uint8_t *)&logWritePointer, 2 );
}

void StoreMinuteCntrInEeprom( uint32_t minuteCount )
{
	WriteTxEepromQueue( MINUTE_CNTR_ADDRESS, (uint8_t *)&minuteCount, 4 );
}

void StoreEventCntrInEeprom( uint16_t eventCount )
{
	WriteTxEepromQueue( EVENT_CNTR_ADDRESS, (uint8_t *)&eventCount, 2 );
}

void StoreWatchdogByteInEeprom( uint8_t watchdogByte )
{
	WriteTxEepromQueue(WATCHDOG_BYTE_ADDRESS, (uint8_t *)&watchdogByte, 1 );
}

void StoreSupercapByteInEeprom( uint8_t supercapByte )
{
	WriteTxEepromQueue(SUPERCAP_BYTE_ADDRESS, (uint8_t *)&supercapByte, 1 );
}

void StoreProgramCrcInEeprom( uint32_t programCrc )
{
	WriteTxEepromQueue(PROGRAM_CRC_ADRS, (uint8_t *)&programCrc, 4 );
}

void StoreNivWordInEeprom( uint32_t nivWord )
{
	WriteTxEepromQueue(NIV_WORD_ADRS, (uint8_t *)&nivWord, 4 );
}

void StoreHumanUseWordInEeprom( uint32_t humanUseWord )
{
	WriteTxEepromQueue(HUMAN_USE_WORD_ADRS, (uint8_t *)&humanUseWord, 4 );
}

void StoreMfgDateInEeprom( char * mfgDate )
{
	WriteTxEepromQueue(MFG_DATE_ADRS, (uint8_t *)mfgDate, 16 );
}

void StoreSerialNumberInEeprom( uint32_t serialNum )
{
	WriteTxEepromQueue(SERIAL_NUMBER_ADRS, (uint8_t *)&serialNum, 4 );
}

void SetEventCount( uint16_t eventCount )
{
	eventCounter = eventCount;
}

//
// Set at manufacturing time
//
void InitializeLog( )
{
	// Set EEPROM log write pointer to log start address
	eepromLogWritePtr = LOG_START_ADDR;
	StoreLogWritePointerInEeprom( eepromLogWritePtr );

	// Set minute counter to 0
	uint32_t minuteCounter = 0;
	SetMinuteCount( minuteCounter );
	StoreMinuteCntrInEeprom( minuteCounter );

	// Set event counter to 0
	uint16_t eventCount = 0;
	SetEventCount( eventCount );
	StoreEventCntrInEeprom( eventCount );

	// Clear log wrapped flag
	logWrappedFlag = 0;
	WriteTxEepromQueue( LOG_WRAPPED_FLAG_ADDRESS, &logWrappedFlag, 1);
}

void WriteLogEvent( uint8_t * logEvent )
{
	if( WriteTxEepromQueue(eepromLogWritePtr, logEvent, LOG_EVENT_SIZE ) != 0 )
	{
		UartPrintf("Unable to write event to EEPROM log because EEPROM queue is full\n");
		return;
	}

	eventCounter++;
	StoreEventCntrInEeprom( eventCounter );

	uint32_t logWritePtr = eepromLogWritePtr + LOG_EVENT_SIZE;

	// Reset log write pointer location if beyond maximum
	if( logWritePtr > LOG_MAX_ADDR )
	{
		eepromLogWritePtr = LOG_START_ADDR;

		// Set log wrapped flag
		logWrappedFlag = 1;
		WriteTxEepromQueue( LOG_WRAPPED_FLAG_ADDRESS, &logWrappedFlag, 1);
	}
	else
	{
		eepromLogWritePtr = logWritePtr;
	}

	StoreLogWritePointerInEeprom( eepromLogWritePtr );
}

void TestEepromWrite( uint16_t address, uint8_t * data, uint16_t numBytes )
{
	WriteTxEepromQueue(address, data, numBytes );
}

void TestEepromRead( uint16_t address, uint16_t numBytes )
{
    testRead = true;
    testReadAddress = address;
    numTestReadBytes = numBytes;

    ReadEeprom( address, readDataBuf, numBytes );
}

void ReadCalConstants( )
{
	getCalConstants = true;

	ReadEeprom( PINSP_OFFSET_ADRS, (uint8_t *)&calibrationConstants, sizeof(calConstants) );
}

void GetMfgDateFromEeprom( )
{
	getMfgDate = true;

	ReadEeprom( MFG_DATE_ADRS, (uint8_t *)&manufactureDate, 16 );
}

void GetSerialNumberFromEeprom( )
{
	getSerialNumber = true;

	ReadEeprom( SERIAL_NUMBER_ADRS, (uint8_t *)&serialNumber, 4 );
}

/* CRC-32C (iSCSI) polynomial in reversed bit order. */
#define POLY 0x82f63b78

uint32_t crc32c( uint32_t crc, uint8_t * buf, size_t len )
{
    crc = ~crc;

    while (len--)
    {
        crc ^= *buf++;

        for( int i=0; i<8; i++ )
        {
        	crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        }
    }

    return ~crc;
}

uint32_t CalcCalConstantsCrc( )
{
	calcCalConstCrc = crc32c( 0, (uint8_t *)&calibrationConstants, sizeof(calConstants) - 4 );
	return calcCalConstCrc;
}

void StoreCalConstantsCrcInEeprom( )
{
	uint32_t crc = CalcCalConstantsCrc();

	WriteTxEepromQueue(CAL_CONSTANTS_CRC_ADRS, (uint8_t *)&crc, 4 );
}

void SetProgramCrcInEeprom( )
{
#if(0)
	uint32_t programCrc = CalcProgramCrc();

	StoreProgramCrcInEeprom( programCrc );
#endif
}

void ReadFromUpperPartOfEeprom( )
{
	uint16_t bytesToEndOfLog = LOG_MAX_ADDR - logReadPointer;

	numLogBytesToRead = (bytesToEndOfLog > MAX_NUM_LOG_BYTES_PER_READ) ? MAX_NUM_LOG_BYTES_PER_READ : bytesToEndOfLog;

	if( numLogBytesToRead == 0 )
	{
		UartPrintf("Number of bytes to read from upper part of log is 0\n");
		return;
	}

	ReadEeprom( logReadPointer, logData, numLogBytesToRead );

	uint32_t logReadPtr = logReadPointer + numLogBytesToRead;

	if( logReadPtr >= LOG_MAX_ADDR )
	{
		logReadPointer = LOG_START_ADDR;
	}
	else
	{
		logReadPointer = logReadPtr;
	}
}

void ReadFromLowerPartOfEeprom( )
{
	uint16_t bytesToWritePointer = eepromLogWritePtr - logReadPointer;

	numLogBytesToRead = (bytesToWritePointer > MAX_NUM_LOG_BYTES_PER_READ) ? MAX_NUM_LOG_BYTES_PER_READ : bytesToWritePointer;

	if( numLogBytesToRead == 0 )
	{
		UartPrintf("Number of bytes to read from lower part of log is 0\n");
		return;
	}

	ReadEeprom( logReadPointer, logData, numLogBytesToRead );

	uint32_t logReadPtr = logReadPointer + numLogBytesToRead;

	if( logReadPtr >= eepromLogWritePtr )
	{
		logReadComplete = true;
	}
	else
	{
		logReadPointer = logReadPtr;
	}
}

//static uint32_t prevEventTime = 0;
void DisplayLogData( )
{
#if(0)
	logInfo *logEvent;
	uint8_t numBytesPerEvent = sizeof(logInfo);
	uint16_t numLogEvents = numLogBytesToRead/numBytesPerEvent;

	// Display portion of log that has been read
    for( int i=0; i<numLogEvents; i++ )
    {
    	logEvent = (logInfo *)&logData[i*numBytesPerEvent];
    	uint32_t eventTime = logEvent->minuteCount;
    	uint8_t code = logEvent->code;
    	uint16_t eventCount = logEvent->eventCount;
    	uint16_t logWritePointer = logEvent->logWritePointer;

    	if( startLogDump != false )
    	{
    		prevEventTime = eventTime;
    		startLogDump = false;
    	}

    	if( eventTime < prevEventTime )
    	{
    		UartPrintfWithoutPrompt("**** WARNING: Event Minute Count is smaller than previous Event Minute Count ****\n");
    	}

    	prevEventTime = eventTime;

    	char name[MAX_EVENT_FIELD_DISPLAY_LENGTH];
    	GetAlarmOrEventName( code, (char *)&name );

    	if(( code == ALARM_BIOT_POST ) || ( code == ALARM_LIFE_EXCEEDED ) ||
    			( code == ALARM_VENTILATION_STOPPED ) || ( code == ALARM_KEYBOARD_FAILED ) ||
				( code == EVENT_BIOT_WATCHDOG_RESET ) || ( code == EVENT_POST_INT_WATCHDOG_NOT_WORKING ) ||
				( code == EVENT_POST_EXT_WATCHDOG_NOT_WORKING ) || ( code == EVENT_RESET_SOURCE) ||
				( code == EVENT_ENTERED_STANDBY_MODE ) || ( code == EVENT_ENTERED_RUN_MODE_PCV ) || (code == EVENT_ENTERED_RUN_MODE_PSV ) ||
				( code == EVENT_ENTERED_RUN_MODE_SIMV ) || ( code == EVENT_SOFTWARE_VERSION) ||
				( code == EVENT_SST_1_FAILED ) || ( code == EVENT_SST_2_FAILED ) ||( code == EVENT_SST_3_FAILED ))
    	{
    		uint32_t value = logEvent->value.uint32Value;
    		bool mfgFlag = false;

    		if( code == ALARM_BIOT_POST )
    		{
    			if( value == 0 )
    			{
    				strcpy( name, "POST/BIOT CLEARED");
    			}
    			else if( value < EVENT_POST_PASSED )
    			{
    				strcpy( name, "POST FAILED");
    			}
    			else
    			{
    				strcpy( name, "BIOT FAILED");
    			}
    		}
    		else if( code == EVENT_SOFTWARE_VERSION )
    		{
    		    mfgFlag = ((value & MFG_TEST_BUILD_ID) == MFG_TEST_BUILD_ID) ? true : false;
    			uint32_t swVersion = (value & ~MFG_TEST_BUILD_ID);

    			if( swVersion <= CURRENT_SW_VERSION_ID )
    			{
    				strcat( name, GetSwVersionString( swVersion ) );
    				if( mfgFlag != false )
    				{
    					strcat( name, "M" );
    				}
    			}
    			else
    			{
    				strcat( name, "Unknown" );
    			}
    		}
    		else if( code == EVENT_RESET_SOURCE )
    		{
    			if( value <= RESET_CAUSE_BROWNOUT_RESET )
    			{
    				strcat( name, ResetCauseString( value ) );
    			}
    			else
    			{
    				strcat( name, "UNKNOWN" );
    			}
    		}

    		if( code == EVENT_SOFTWARE_VERSION )
    		{
    			UartPrintfWithoutPrompt("Event Count: %4u   LogWrPtr: 0x%04x   Minute Count: %5d   Event: %-35s   Value: %8x\n", eventCount, logWritePointer, eventTime, name, value);
    		}
    		else
    		{
    			UartPrintfWithoutPrompt("Event Count: %4u   LogWrPtr: 0x%04x   Minute Count: %5d   Event: %-35s   Value: %8ld\n", eventCount, logWritePointer, eventTime, name, value);
    		}
    	}
    	else if(( code == EVENT_SST_PRESSURE_PASS ) || ( code == EVENT_SST_PRESSURE_FAIL ) || ( code == EVENT_SST_1_PASSED ) ||
    			( code == EVENT_SST_2_PASSED ) || ( code == EVENT_SST_3_PASSED ) || ( code == EVENT_POST_PASSED)  ||
				( code == EVENT_NEW_PATIENT ) || ( code == EVENT_SAME_PATIENT ) || ( code == EVENT_AUDIO_PAUSED ) || ( code == EVENT_ALARM_RESET ))
    	{
    		UartPrintfWithoutPrompt("Event Count: %4u   LogWrPtr: 0x%04x   Minute Count: %5d   Event: %-35s\n", eventCount, logWritePointer, eventTime, name);
    	}
    	else if(( code == EVENT_POST_SW_CRC_BAD ) || ( code == EVENT_POST_RAM_TEST_FAIL ))
    	{
    		uint32_t value = logEvent->value.uint32Value;

    		UartPrintfWithoutPrompt("Event Count: %4u   LogWrPtr: 0x%04x   Minute Count: %5d   Event: %-35s   Value: 0x%08x\n", eventCount, logWritePointer, eventTime, name, value);
    	}
    	else
    	{
        	float value = logEvent->value.floatValue;

        	UartPrintfWithoutPrompt("Event Count: %4u   LogWrPtr: 0x%04x   Minute Count: %5d   Event: %-35s   Value: %8.2f\n", eventCount, logWritePointer, eventTime, name, value);
    	}
    }

    // End log read process if last portion of log has been displayed
    if( logReadComplete != false )
    {
    	logReadComplete = false;
    	UartPrintf("");
    	return;
    }

    // Read next portion of Log
	if( logReadPointer > eepromLogWritePtr )
	{
		// Wrap condition - read from upper portion of EEPROM
		ReadFromUpperPartOfEeprom();
	}
	else
	{
		// Read from lower portion of EEPROM
		ReadFromLowerPartOfEeprom();
	}
#endif
}

int8_t GetCalConstantsCrcGood( )
{
	// -1 is not calculated, 0 is good, and 1 is bad
	return calConstantsCrcGood;
}

void DisplayEepromTestRead( )
{
	UartPrintfWithoutPrompt("\n");

    for( int i=0; i<numTestReadBytes; i++ )
    {
    	UartPrintfWithoutPrompt("EEPROM Address:  0x%04x  Data: 0x%02x\n", testReadAddress+i, readDataBuf[i]);
    }

    UartPrintf("");
}

void ProcessEepromReadComplete( )
{
	if( testRead != false )
	{
		testRead = false;

		displayEepromTest = true;

	    return;
	}

	if( getLogWritePtr != false )
	{
		getLogWritePtr = false;

		UartPrintf("Log Write Ptr: 0x%04x\n", eepromLogWritePtr);

		return;
	}

	if( getMinuteCounter != false )
	{
		getMinuteCounter = false;

		SetMinuteCount( minuteCounter );

		UartPrintf("Minute Counter: 0x%04x\n", minuteCounter);

		return;
	}

	if( getEventCounter != false )
	{
		getEventCounter = false;

		SetEventCount( eventCounter );

		UartPrintf("Event Counter: %4u\n", eventCounter);

		return;
	}

	if( getNivWord != false )
	{
		getNivWord = false;

		UartPrintf("Niv Word: 0x%08x\n", nivWordValue);

		return;
	}

	if( getHumanUseWord != false )
	{
		getHumanUseWord = false;

		UartPrintf("Human Use Word: 0x%08x\n", humanUseWordValue);

		return;
	}

	if( getProgramCrc != false )
	{
		getProgramCrc = false;

		return;
	}

	if( getCalConstants != false )
	{
		getCalConstants = false;

		CalcCalConstantsCrc();

		if( calcCalConstCrc != calibrationConstants.crc32 )
		{
			UartPrintf("Calibration Constants Bad CRC\n");
			calConstantsCrcGood = 1;
		}
		else
		{
			UartPrintf("Calibration Constants Good CRC\n");
			calConstantsCrcGood = 0;
		}

		UartPrintf("PinspOffset: %d  PexpOffset: %d   PinspGain: %f  PexpGain: %f  CRC: 0x%08x\n", calibrationConstants.pInspOffset, calibrationConstants.pExpOffset,
				calibrationConstants.pInspGain.floatValue, calibrationConstants.pExpGain.floatValue, calibrationConstants.crc32);
		return;
	}

	if( getMfgDate != false )
	{
		getMfgDate = false;

		UartPrintf("Manufacture Date: %s\n", manufactureDate);

		return;
	}

	if( getSerialNumber != false )
	{
		getSerialNumber = false;

		UartPrintf("Serial Number: %u\n", serialNumber);

		return;
	}

	if( getWatchdogByte != false )
	{
		getWatchdogByte = false;

		return;
	}

	if( getSupercapByte != false )
	{
		getSupercapByte = false;

		return;
	}

	if( dumpEepromFlag != false )
	{
		if( logReadComplete != false )
		{
			// Log read is complete
			// Set flag to display final portion of log that has been read
			dumpEepromFlag = false;
			displayLogData = true;
		}
		else if( initiateLogRead != false )
		{
			initiateLogRead = false;

			// Read first portion of log
			if(( logWrappedFlag == 0 ) && ( eepromLogWritePtr == LOG_START_ADDR ))
			{
				dumpEepromFlag = false;
				displayLogData = false;
				UartPrintf("Log is empty.\n");
			}
			else if(( logWrappedFlag == 1 ) && ( eepromLogWritePtr != LOG_START_ADDR ))
			{
				// Log wrapped flag is set, dump from log write pointer to end of log, and then from log start to log write pointer - 1
				logReadPointer = eepromLogWritePtr;
				ReadFromUpperPartOfEeprom();
			}
			else
			{
				// Log wrapped flag is not set, dump from log start to log write pointer - 1
				logReadPointer = LOG_START_ADDR;
				ReadFromLowerPartOfEeprom();
			}
		}
		else
		{
			// Set flag to display portion of log that has been read
			displayLogData = true;
		}
	}
}

void DumpEepromLog( )
{
	// Set Dump EEPROM log flag and initiate log read flag
    dumpEepromFlag = true;
    initiateLogRead = true;
    startLogDump = true;

    // Read log wrapped flag
    ReadEeprom( LOG_WRAPPED_FLAG_ADDRESS, &logWrappedFlag, 1 );
    // logWrappedFlag = 1;
    // ProcessEepromReadComplete( );


    UartPrintfWithoutPrompt("SW REV: %s\n", GetSwVersionString( CURRENT_SW_VERSION_ID ));
    UartPrintfWithoutPrompt("SN: %u\n", serialNumber);
    UartPrintfWithoutPrompt("Date of Manufacture: %s\n", manufactureDate);

    uint32_t minuteCount = GetMinuteCount();
	uint16_t days = minuteCount/1440;
	uint16_t hours = (minuteCount%1440)/60;
	uint16_t minutes = (minuteCount%1440)%60;
	UartPrintfWithoutPrompt("Total OP Time: %2dDay %2dHr %2dMin\n", days, hours, minutes);
}



