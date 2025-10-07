#include "ModBus.hpp"

#include "main.h"
#include "cmsis_os.h"
#include "../Dispatcher/dispatcher.hpp"
//#include "usart.h"



static const int speedDir = -1;

union FloatToBytes {
  float value;
  uint8_t bytes[sizeof(float)];
};

uint16_t Modbus_CalculateCRC(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void Modbus_SendRequest(uint8_t slaveAddress, uint8_t functionCode, uint16_t startAddress, uint16_t quantity) {
    uint8_t request[8];
    request[0] = slaveAddress;
    request[1] = functionCode;
    request[2] = (startAddress >> 8) & 0xFF;
    request[3] = startAddress & 0xFF;
    request[4] = (quantity >> 8) & 0xFF;
    request[5] = quantity & 0xFF;
    uint16_t crc = Modbus_CalculateCRC(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;

	//HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart2, request, 8, HAL_MAX_DELAY);
	//HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_RESET);
}


void Modbus_SendSetRequest(uint8_t slaveAddress, uint8_t functionCode, uint16_t startAddress, uint16_t quantity, uint8_t* value) {
    uint8_t request[13];
    int i = 0;
    request[i++] = slaveAddress;
    request[i++] = functionCode;
    request[i++] = (startAddress >> 8) & 0xFF;
    request[i++] = startAddress & 0xFF;
    request[i++] = (quantity >> 8) & 0xFF;
    request[i++] = quantity & 0xFF;
    request[i++] = quantity*2;//byte count
    request[i++] = value[0];
    request[i++] = value[1];
    request[i++] = value[2];
	request[i++] = value[3];
    uint16_t crc = Modbus_CalculateCRC(request, i);
    request[i++] = crc & 0xFF;
    request[i++] = (crc >> 8) & 0xFF;

	//HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart2, request, i, HAL_MAX_DELAY);
	//HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_RESET);
}


//static Dispatcher* Dispatcher = Dispatcher::getDispatcher();

#define UV_HEAD_ADDRESS 1
#define PULLING_UNIT_ADDRESS 97

//UV head
//function 3 input registers
#define MODBUS_TEMPERATURE_REG 0x20 //float
#define MODBUS_PRESSURE_REG 0x24 //uint16_t
#define MODBUS_REEDS_REG 0x28//uint16_t
#define MODBUS_TEMP2_REG 0x22//float
#define MODBUS_HUMIDITY_REG 0x26//float
//coil
#define MODBUS_RESET_REEDS_REG 0x20 //function 5


//pulling unit
#define MODBUS_DISTANCE_REG			0x20 //function 4 //float
#define MODBUS_RESET_DISTANCE_REG	0x22 //function 5
#define MODBUS_SET_SPEED_REG		0x26 //function 16 //float





void ReportError(eMODS_State error)
{
	Dispatcher* Dispatcher = Dispatcher::getDispatcher();
	sMsgModbusState Status = {MODS_OK};
	Status.state = error;
	Dispatcher->DispatcherPostMsgByCopy(MT_MODBUS_STATE, &Status, sizeof(Status));
}


int Modbus_ProcessResponse(uint8_t *request, uint16_t length, uint8_t *payload) {
    // Calculate CRC of the received request
    uint16_t receivedCRC = (request[length - 1] << 8) | request[length - 2];
    uint16_t calculatedCRC = Modbus_CalculateCRC(request, length - 2);

    // Check if the CRC is valid
    if (receivedCRC != calculatedCRC) {
        // Invalid CRC, ignore the request
    	//vTaskDelay(10);
    	ReportError(MODS_CRC_ERROR);
        return 0;
    }

    uint8_t slaveAddress = request[0];
    uint8_t functionCode = request[1];
    uint16_t quantityBytes = request[2];
    int i = 0;

    // Check if the request is for this slave
    if (slaveAddress != UV_HEAD_ADDRESS and slaveAddress != PULLING_UNIT_ADDRESS) {
        return 0 ; // Not for this slave
    }

    // Check if the function code is supported
    if (functionCode != 3 and functionCode != 4 and functionCode != 5 and functionCode != 0x10) {
        return 0; // Unsupported function code
    }

    // extract payload
    int j = 0;
    if(3 + quantityBytes < length)
    {
    	for(j = 0; j < quantityBytes; j++)
    	{
    		payload[i++] = request[3+j];
    	}
    }
    return j;
}

#define RESPONSE_DELAY 10 //was 100
int MODBUS_receive(int numberOfBytes, uint8_t* payloadbuffer)
{

    // Buffer to store the received request
    uint8_t request[50];
    uint32_t size = 0;
    uint32_t sizeTotal = 0;;

    HAL_StatusTypeDef RETvAL = HAL_UART_Receive(&huart2, request, numberOfBytes, RESPONSE_DELAY);
    //SerialSend("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n\r", request[0], request[1], request[2], request[3], request[4], request[5], request[6], request[7]);

	// Receive Modbus request simple draft assumes frame has 8 bytes
	if( RETvAL == HAL_OK)
	{
		return Modbus_ProcessResponse(request, numberOfBytes, payloadbuffer);
	}
	else
	{
		uint8_t temp = 0;
		while(HAL_UART_Receive(&huart2, &temp, 1, 0) == HAL_OK);
		ReportError((eMODS_State)RETvAL);
		return 0;
	}
}


void checkTemp(Dispatcher* Dispatcher)
{
	uint8_t buff[20] = {0};
	int RegisterLen = 2;
	Modbus_SendRequest(UV_HEAD_ADDRESS,4,MODBUS_TEMPERATURE_REG,RegisterLen);
	//len 5 + register has two bytes
	int received = MODBUS_receive(5 + RegisterLen*2, buff);

	sMsgTmpr MsgTmpr = {0};

	//byte to float conversion
	if(received > 0)
	{
		FloatToBytes temp;
		/*for(int i = 0; i < 4; i++)
		{
			//temp.bytes[i] = buff[3-i];
			temp.bytes[i] = buff[i];
		}*/
		//2143 1032

		temp.bytes[0] = buff[1];
		temp.bytes[1] = buff[0];
		temp.bytes[2] = buff[3];
		temp.bytes[3] = buff[2];


		MsgTmpr.temperature = temp.value;

		Dispatcher->DispatcherPostMsgByCopy(MT_TEMPERATURE, &MsgTmpr, sizeof(MsgTmpr));
	}


}

void checkDistance(Dispatcher* Dispatcher)
{
	uint8_t buff[20] = {0};
	int RegisterLen = 2;
	Modbus_SendRequest(PULLING_UNIT_ADDRESS,4,MODBUS_DISTANCE_REG,RegisterLen);
	//len 5 + register has two bytes
	int received = MODBUS_receive(5 + RegisterLen*2, buff);

	sMsgDistance MsgDist = {0};


	//byte to float conversion
	if(received > 0)
	{
		FloatToBytes temp;
		/*for(int i = 0; i < 4; i++)
		{
			//temp.bytes[i] = buff[3-i];
			temp.bytes[i] = buff[i];
		}*/
		//2143 1032

		temp.bytes[0] = buff[1];
		temp.bytes[1] = buff[0];
		temp.bytes[2] = buff[3];
		temp.bytes[3] = buff[2];

		 //SerialSend("0x%02x 0x%02x 0x%02x 0x%02x\n\r", buff[0], buff[1], buff[2], buff[3]);
		if(abs(temp.value) > 0.001)
		{
			MsgDist.distance = speedDir * temp.value;
		}
		else
		{
			MsgDist.distance = temp.value;
		}

		Dispatcher->DispatcherPostMsgByCopy(MT_DISTANCE, &MsgDist, sizeof(MsgDist));
	}


}

void setSpeed(Dispatcher* Dispatcher, float setSpeed)
{
	uint8_t buff[20] = {0};
	FloatToBytes speed = {0};
	speed.value = setSpeed;
	uint16_t RegisterLen = 2;

	FloatToBytes temp;

	temp.bytes[0] = speed.bytes[1];
	temp.bytes[1] = speed.bytes[0];
	temp.bytes[2] = speed.bytes[3];
	temp.bytes[3] = speed.bytes[2];

	Modbus_SendSetRequest(PULLING_UNIT_ADDRESS,0x10,MODBUS_SET_SPEED_REG,RegisterLen, temp.bytes);
	//volatile int received = MODBUS_receive(8, buff); //ignore for now just testing
	uint8_t tempByte = 0;
	while(HAL_UART_Receive(&huart2, &tempByte, 1, 20) == HAL_OK);

}

void resetDistance(Dispatcher* Dispatcher)
{
	uint8_t buff[20] = {0};
	uint16_t RegisterLen = 0xFF00;

	//volatile int received = MODBUS_receive(8, buff); //ignore for now just testing

	int repeatCounter = 3;
	int received = 0;
	while((received == 0) and (repeatCounter > 0))
	{
		Modbus_SendRequest(PULLING_UNIT_ADDRESS,5,MODBUS_RESET_DISTANCE_REG,RegisterLen);
		received = MODBUS_receive(8, buff);
		repeatCounter--;
		vTaskDelay(1);
	}
}

void resetReeds(Dispatcher* Dispatcher)
{
	uint8_t buff[20] = {0};
	uint16_t RegisterLen = 0xFF00;
	Modbus_SendRequest(UV_HEAD_ADDRESS,5,MODBUS_RESET_REEDS_REG,RegisterLen);
	//volatile int received = MODBUS_receive(8, buff); //ignore for now just testing
	uint8_t tempByte = 0;
	while(HAL_UART_Receive(&huart2, &tempByte, 1, 20) == HAL_OK);
	//todo check if response received


}

void checkHumidity(Dispatcher* Dispatcher)
{
	uint8_t buff[20] = {0};
	int RegisterLen = 2;
	Modbus_SendRequest(UV_HEAD_ADDRESS,4,MODBUS_HUMIDITY_REG,RegisterLen);
	//len 5 + register has two bytes
	int received = MODBUS_receive(5 + RegisterLen*2, buff);

	sMsgHumidity MsgHum = {0};

	//byte to float conversion
	if(received > 0)
	{
		FloatToBytes temp;
		/*for(int i = 0; i < 4; i++)
		{
			temp.bytes[i] = buff[3-i];
		}*/

		temp.bytes[0] = buff[1];
		temp.bytes[1] = buff[0];
		temp.bytes[3] = buff[2];
		temp.bytes[2] = buff[3];


		MsgHum.humidity = temp.value;

		Dispatcher->DispatcherPostMsgByCopy(MT_UVHEAD_HUMIDITY, &MsgHum, sizeof(MsgHum));
	}


}

void checkPressure(Dispatcher* Dispatcher)
{
	uint8_t buff[20] = {0};
	int RegisterLen = 1;
	Modbus_SendRequest(UV_HEAD_ADDRESS,4,MODBUS_PRESSURE_REG,RegisterLen);
	int received = MODBUS_receive(5 + RegisterLen*2, buff);

	sMsgUvHeadPressure MsgPressure = {0};

	if(received > 0)
	{
		MsgPressure.uvHeadPressure = buff[1];

		Dispatcher->DispatcherPostMsgByCopy(MT_UVHEAD_PRESSURE, &MsgPressure, sizeof(MsgPressure));
	}
}

void checkReeds(Dispatcher* Dispatcher)
{
	uint8_t buff[20] = {0};
	int RegisterLen = 1;
	Modbus_SendRequest(UV_HEAD_ADDRESS,4,MODBUS_REEDS_REG,RegisterLen);
	int received = MODBUS_receive(5 + RegisterLen*2, buff);

	sMsgReedSwitchStatus ReedSwitchStatus;
	ReedSwitchStatus.state = BOTH_OFF;

	if(received > 0)
	{

		if(buff[0] > 0 && buff[1] > 0)
		{
			ReedSwitchStatus.state = BOTH_ON;
		}
		else if(buff[0] > 0)
		{
			ReedSwitchStatus.state = FRONT_ON;
		}
		else if(buff[1] > 0)
		{
			ReedSwitchStatus.state = BACK_ON;
		}
		else
		{
			ReedSwitchStatus.state = BOTH_OFF;
		}

		Dispatcher->DispatcherPostMsgByCopy(MT_REED_SWITCH, &ReedSwitchStatus, sizeof(ReedSwitchStatus));
		//reset after reading
		vTaskDelay(1);
	    resetReeds(Dispatcher);
	}


}

void task_ModBus(void *pvParameters)
{
    Dispatcher_Task *TaskPtr = (Dispatcher_Task*) pvParameters;
	sMsgStruct Msg = {};
	Dispatcher* Dispatcher = Dispatcher::getDispatcher();

	//sMsgPSUStatus MSgPSU = {state, UVLED->GetDuty()};
	//Dispatcher->DispatcherPostMsgByCopy(MT_PSU_STATUS, &MSgPSU, sizeof(MSgPSU));
	uint8_t buff[20] = {0};
	int received = 0;
#define DELAY_MOD 5

	for (;;)
	{
		//vTaskDelay(300);
		sMsgMotorControl Control;
		sMsgResetDistance ResetDistance;
		bool ctrl = false;
		bool dist = false;

		//make sure we act only on the last messaqge
		while(xQueueReceive(TaskPtr->GetQueueHandle(), &Msg, 0) == pdPASS)
		{
			switch (Msg.type) {
				case MT_MTR_CONTROL:
				{
					Control = *(sMsgMotorControl*)Msg.data;
					ctrl = true;
					break;
				}
				case MT_RESET_DISTANCE:
				{
					ResetDistance = *(sMsgResetDistance*)Msg.data;
					dist = true;
					break;
				}
				default:
					break;
			}
		}

		if(ctrl)
		{
			if(Control.status == MTR_STOPPED)
			{
				setSpeed(Dispatcher, 0);
			}
			else if (Control.status == MTR_MOVING)
			{
				//sMsgMotorControl *Control = (sMsgMotorControl*)Msg.data;

				setSpeed(Dispatcher, speedDir * (Control.speed * Control.modifier));
			}


			if(Control.resetDistance)
			{
				resetDistance(Dispatcher);
			}
			ctrl = false;
		}

		if(dist)
		{
			resetDistance(Dispatcher);
			dist = false;
		}

		vTaskDelay(DELAY_MOD);
		checkDistance(Dispatcher);

		vTaskDelay(DELAY_MOD);
		checkHumidity(Dispatcher);

		vTaskDelay(DELAY_MOD);
		checkReeds(Dispatcher);

		vTaskDelay(DELAY_MOD);
		checkPressure(Dispatcher);

		vTaskDelay(DELAY_MOD);
		checkTemp(Dispatcher);

		vTaskDelay(500);
		HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
	}


}

void task_ModBus_Init(uint8_t taskIndex)
{
	Dispatcher* Dispatcher = Dispatcher::getDispatcher();
	Dispatcher->dispatcherSubscribe(MT_MTR_CONTROL, taskIndex);
	Dispatcher->dispatcherSubscribe(MT_RESET_DISTANCE, taskIndex);

}
