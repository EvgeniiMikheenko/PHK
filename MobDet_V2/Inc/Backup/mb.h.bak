//==============================================================================
//
//==============================================================================

#ifndef _MB_H_
#define _MB_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
//#include <mb_slave.h>

//#include <modbus_config.h>

#define MAX_REPEATE_TRANSMIT_COUNT 			5
#define MODBUS_REGS_IOCOUNT_MAX				125
// 											(addr  	+ func  + startAddr + count + (MODBUS_REGS_IOCOUNT_MAX * 2))
#define MOBUS_MAX_PACKET_SIZE				(  1 	+   1 	+    2 		+   2 	+ (MODBUS_REGS_IOCOUNT_MAX * 2))

#define MB_DEVICE_ADDRESS_BROADCAST			0
#define MB_DEVICE_ADDRESS_MIN				1
#define MB_DEVICE_ADDRESS_MAX				247

#define MB_PACKET_SIZE_MIN					4

#define MB_PACKET_ADDR_INDEX				0
#define MB_PACKET_FUNC_CODE_INDEX			1
#define MB_PACKET_ERROR_CODE_INDEX			2

#define MB_ERROR_PACKET_SIZE				5

enum {
	  MB_CMD_READ_COILS 				= 0x01 // Чтение статуса выходов
	, MB_CMD_READ_DINPUTS 				= 0x02 // Чтение состояния дискретных входов
	, MB_CMD_READ_HOLDING_REGS 			= 0x03 // Чтение содержимого регистров
	, MB_CMD_READ_INPUT_REGS 			= 0x04 // Чтение содержимого входных регистров
	, MB_CMD_WRITE_SINGLE_COIL 			= 0x05 // Уствновка единичного выхода в ON или OFF
	, MB_CMD_WRITE_SINGLE_REG 			= 0x06 // Запись в единичный регистр
	, MB_CMD_READ_EXCEPTION_STATUS		= 0x07 // Чтение статуса устройства
	, MB_CMD_WRITE_MULTI_COILS 			= 0x0F // Установка множества выходов в ON или OFF
	, MB_CMD_WRITE_MULTI_REGS 			= 0x10 // Запись в несколько регистров
	, MB_CMD_READ_FILE_RECORD			= 0x14 // Чтение файла
	, MB_CMD_WRITE_FILE_RECORD			= 0x15 // Запись файла
	, MB_CMD_READ_DEVICE_ID				= 0x2B // Чтение идентификатора устройства
};

typedef enum {
	  MB_EXC_NONE						= 0x00
	, MB_EXC_ILLEGAL_FUNCTION 			= 0x01
	, MB_EXC_ILLEGAL_DATA_ADDR			= 0x02
	, MB_EXC_ILLEGAL_DATA_VALUE			= 0x03
	, MB_EXC_SERVER_DEVICE_FAILURE		= 0x04
	, MB_EXC_ACK						= 0x05
	, MB_EXC_SERVER_DEVICE_BUSY			= 0x06
	, MB_EXC_MEMORY_PARITY_ERR			= 0x08
	, MB_EXC_GATEWAY_PATH_UNAVAILABLE	= 0x0A
	, MB_EXC_GATEWAY_TARGET_DEV_FAILED	= 0x0B
} MbExceptionCodes_t;

typedef enum {
	  MB_ERR_NONE
	, MB_ERR_INVALID_DATA
	, MB_ERR_NOINIT
	, MB_ERR_INVALID_FUNCTION_PTR
	, MB_ERR_SERIAL_INIT
	, MB_ERR_INVALID_DEVICE_ADDR
	, MB_ERR_TIMER_INIT
	, MB_ERR_PACKET_SIZE_MIN
	, MB_ERR_PACKET_CRC
	, MB_ERR_RX_BUF_OVF
} MbErrorCodes_t;

typedef enum {
	  MB_EVT_NONE
	, MB_EVT_PARSE
	, MB_EVT_TX
	, MB_EVT_ADDR_INVALID
} MBEvent_t;


#define USM_LOCATION_DESCRIPTION_STRING_SIZE			(64)
#define USM_CONFIG_DUMMY1_COUNT							(16)
#define USM_DUMMY1_COUNT								(64)
#define USM_DUMMY2_COUNT								(64)
#define USM_DUMMY3_COUNT								(64)
#define USM_DUMMY4_COUNT								(64)
#define USM_HW_VERSION_COUNT							(4)
#define USM_MD5_REGISTERS_COUNT							(8)


	
#pragma pack(push, 1)








typedef struct {
	uint16_t Flags_write;
	uint16_t var_res_level;		//hv_val
	uint16_t shunt_time;	//freq
	uint16_t shunt_tresh;	// hv_offset
	uint16_t pos_tresh;	//pos_tresh
	uint16_t neg_tresh;	//neg_tresh
	uint16_t ref_null;
	uint16_t pos_Tresh_Danger;		
	uint16_t neg_Tresh_Danger;

	uint16_t crc;
	
	uint16_t dummy1[20];
	uint16_t write_reg;	
	uint16_t Sensor_voltage;
	
	short Temperature;
	uint16_t Humidity;
	
	short test1;
	short test2;
	
	
	uint32_t FwVer;
	uint32_t HwVer[ USM_HW_VERSION_COUNT ];
	uint16_t MD5[ USM_MD5_REGISTERS_COUNT ];
	//float VccSysVoltage;
	uint16_t Dummy[90];
	
} mb_Regs;






#pragma pack(pop)


// Указатель на функцию события модбас
typedef void (*pxMbEvent)( MBEvent_t event );

// Указатели на функции работы с UART
typedef bool (*pxMBSerialPortInit)( uint32_t baudrate, uint8_t stopBits, uint8_t parity );
typedef bool (*pxMBSerialPortEnable)( bool rxEn, bool txEn );

// Указатели на функции работы с таймером
typedef bool (*pxMBTimerInit) ( uint32_t us );
typedef bool (*pxMBTimerEneble)( );
typedef bool (*pxMBTimerDisable)( );
typedef bool (*pxMBTimerReset)( );

// Указатели на функции модбас команд
typedef MbExceptionCodes_t (*pxMbReadCoils)( uint8_t *lpDstBuf, uint16_t startAddr, uint16_t count, uint16_t *lpResultByteCount );
typedef MbExceptionCodes_t (*pxMbReadDescreteInputs)( uint8_t *lpDstBuf, uint16_t startAddr, uint16_t count, uint16_t *lpResultByteCount );
typedef MbExceptionCodes_t (*pxMbReadRegisters)( uint8_t *lpDstBuf, uint16_t startAddr, uint16_t count );
typedef MbExceptionCodes_t (*pxMbReadInputRegisters)( uint8_t *lpDstBuf, uint16_t startAddr, uint16_t count );
typedef MbExceptionCodes_t (*pxMbWriteRegisters)( uint16_t startAddr, uint16_t count, uint8_t *lpSrcBuf );
typedef MbExceptionCodes_t (*pxMbReadExcStatus)( uint8_t *lpDstBuf );
typedef MbExceptionCodes_t (*pxMbWriteMultiCoils)( uint16_t startAddr, uint16_t count, uint8_t *lpSrcBuf );
typedef MbExceptionCodes_t (*pxMbReadFileRecord)( uint16_t fileNumber, uint16_t startAddr, uint16_t count, uint8_t *lpDstBuf );
typedef MbExceptionCodes_t (*pxMbWriteFileRecord)( uint16_t fileNumber, uint16_t startAddr, uint16_t count, uint8_t *lpSrcBuf );

#endif // _MB_H_
