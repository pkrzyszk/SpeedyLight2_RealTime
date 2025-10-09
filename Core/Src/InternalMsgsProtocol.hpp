#pragma once
/**
 * \file
 *
 * \brief Structures and enums for system queue messages.
 *
 * Created: 18/06/2018 08:40:56
 * Author: piotr.krzyszkowski
 */



#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <assert.h>

 /* Kernel includes. */
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

//#include "mc_hal_can/CanMsgProtocol.h"

#define DATA_POINTER_SEMAPHORE_INDEX 0
#define DATA_POINTER_DATA_INDEX 1

#define MAX_NUMBER_OF_TASKS 20
#define SUPER_QUEUE_LENGTH	100
#define GENERIC_QUEUE_LENGTH	20
#define GENERIC_QUEUE_ITEM_SIZE ((sizeof(sMsgStruct)))
#define DATA_POINTER_SEMAPHORE_INDEX 0
#define DATA_POINTER_DATA_INDEX 1
#define DEFAULTSTACKSIZE 1024
#define MAX_NUMBER_OF_TASKS 20

//Bit Masks
#define SONDE_MODE_MASK        0x03
#define SONDE_FREQUENCY_MASK   0x0C
#define SONDE_CAMERA_MASK      0x10

//pack it nicely to avoid problems with casting pointers
// lets pack structures so the variables follow each other
// so we have no issues when casting pointers
#pragma pack(1) //COMPILER_PACK_SET(1)

#define BIT0       0x01
#define BIT1       0x02
#define BIT2       0x04
#define BIT3       0x08
#define BIT4       0x10
#define BIT5       0x20
#define BIT6       0x40
#define BIT7       0x80

#define PSU_MAX 100.0

#define SPEED_MAX 1.25

#define BROKEN_PT100 750


/**
* \brief Enum for indexing system message Addresses
*/
typedef enum _MsgTypes
{
	MT_DO_NOT_USE,
	MT_STATE,
	MT_MTR_STATUS,
	MT_MTR_CONTROL,
	MT_DISTANCE,
	MT_TEMPERATURE,
	MT_PSU_STATUS,
	MT_PSU_CONTROL,
	MT_KNOB,
	MT_BUTTON,
	MT_RESET,
	MT_REED_SWITCH,
	MT_TIME_CONTROL,
	MT_TIME_STATUS,
	MT_UVHEAD_PRESSURE,
	MT_UVHEAD_HUMIDITY,
	MT_MODBUS_STATE,
	MT_METRIC_STATE,
	MT_RESET_DISTANCE,
	MT_PRINT_TASKS_STATS,
	/* Don't add after this line */
	MT_NUM_TYPES
}eMsgTypes;

typedef enum
{
	SM_IDLE,
	SM_RUNNING
}eSM_State;

/*typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;*/

typedef enum
{
	MODS_OK,
	MODS_HAL_ERROR,
	MODS_HAL_BUSY,
	MODS_HAL_TIMEOUT,
	MODS_CRC_ERROR
}eMODS_State;

typedef enum
{
	DE_OFF,
	DE_ON
}eDebug_State;

typedef enum
{
	PSU_OFF,
	PSU_ON,
	PSU_NO_CHANGE
}ePSU_State;

typedef enum
{
	TIME_OFF,
	TIME_ON,
}eTIME_Control;

typedef enum
{
	BOTH_OFF,
	FRONT_ON,
	BACK_ON,
	BOTH_ON,
	REED_MAX_STATES,
}eReedSwitch_State;

typedef enum
{
	TEMP_NORMAL,
	TEMP_HIGH,
	TEMP_CRITICAL,
	TEMP_ERROR
}eTemp_State;

typedef enum
{
	R_SYSTEM= 0,

}eReset;


typedef enum _eMotorCmds
{
	MTR_STOPPED = 0,
	MTR_MOVING,
	MTR_NO_CHANGE,
	MTR_BLOCKED,

}eMotorStatus;

/*typedef enum _eMotorDir
{
	MTRD_BEST = 0,
	MTRD_CLOCK,
	MTRD_ANTICLOCK,
}eMotorDir;*/

typedef struct
{
	eMotorStatus status;
	double speed;
	double modifier = 1.0;
} sMsgMotorStatus;

typedef struct
{
	eMotorStatus status;
	double speed;
	double modifier;
	double change;
	bool resetDistance;
} sMsgMotorControl;

typedef struct
{
	double distance;
} sMsgDistance;

typedef struct
{
	ePSU_State state;
	double setting;
} sMsgPSUControl;

typedef struct
{
	eReedSwitch_State state;
	double raw;
} sMsgReedSwitchStatus;

typedef struct
{
	ePSU_State state;
	double percent;
} sMsgPSUStatus;

typedef struct
{
	eSM_State state;
	eTemp_State temperature;
	bool debugOn;
} sMsgState;

typedef struct
{
	eMODS_State state;
} sMsgModbusState;

typedef struct
{
	double temperature;
	double temperatureVolatile;
} sMsgTmpr;

typedef struct
{
	double humidity;
} sMsgHumidity;


typedef struct
{
	eReset value;
} sMsgReset;

typedef struct
{
	int32_t absValue;
	int32_t change;
} sMsgKnob;

typedef struct
{
	eTIME_Control Control;
} sMsgTimeCtrl;

typedef struct
{
	eTIME_Control OnOff;
	int32_t Seconds;
	uint32_t Start = 0;
} sMsgTimeStatus;

typedef struct
{
	bool Metric;
} sMsgMetricStatus;

typedef struct
{
	bool Reset;
} sMsgResetDistance;

typedef enum
{
	BS_PRESSED,
	BS_RELEASED,
	BS_HOLD_1S,
	BS_HOLD_5S
}eButtonState;

typedef enum
{
	BN_KNOB,
	BN_USER,
	BN_METRIC,
}eButtonType;

typedef struct
{
	eButtonType type;
	eButtonState value;
	bool active;
} sMsgButtn;

typedef struct
{
	int uvHeadPressure;
} sMsgUvHeadPressure;



#define SERIAL_NUMBER_SIZE 8 //as per PT factory PC camera software 7, +1 for null

typedef struct
{
	char 	SerialNumber[SERIAL_NUMBER_SIZE];
}sMsgSerialNumber;

#define MESSAGE_PAYLOAD_SIZE 28

typedef struct
{
	TickType_t timeStamp;
	eMsgTypes type;
	uint8_t  data[MESSAGE_PAYLOAD_SIZE];
	uint8_t* pData;
}sMsgStruct;


#pragma pack() //COMPILER_PACK_RESET()
