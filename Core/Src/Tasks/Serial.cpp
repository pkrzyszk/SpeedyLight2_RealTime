#include "Serial.hpp"

#include "main.h"
#include "cmsis_os.h"
#include <cstdarg>
#include <cstring>
#include <cstdio>
extern "C" {
#include "task.h"
}
//#include "usart.h"

#include "../Dispatcher/dispatcher.hpp"


//static Dispatcher* Dispatcher = Dispatcher::getDispatcher();

void handleState(eSM_State state)
{
    switch (state)
    {
        case SM_IDLE:
            SerialSend("System is idle\n\r");
            break;
        case SM_RUNNING:
            SerialSend("System is running\n\r");
            break;
        /*case SM_TEMP_HIGH:
            SerialSend("System temperature is high\n\r");
            break;
        case SM_TEMP_CRITICAL:
            SerialSend("System temperature is critical!\n\r");
            break;*/
        default:
            SerialSend("Unknown state\n\r");
            break;
    }
}
void handleTempState(eTemp_State state)
{
	switch (state)
	{
		case TEMP_NORMAL:
			SerialSend("System temperature is normal\n\r");
			break;
		case TEMP_HIGH:
			SerialSend("System temperature is high\n\r");
			break;
		case TEMP_CRITICAL:
			SerialSend("System temperature is critical!\n\r");
			break;
		default:
			SerialSend("UNKNOWN_STATE\n\r");
			break;
	}
}

void printTaskStats(void)
{
	char statsBuffer[512];
	//vTaskDelay(15000);
	vTaskGetRunTimeStats(statsBuffer);
	SerialSend(statsBuffer);
}


void HandleMODSState(eMODS_State state)
{
	switch (state)
	{
	    case MODS_OK:
	        SerialSend("MODS OK.\n\r");
	        break;

	    case MODS_HAL_ERROR:
	        SerialSend("MODS HAL Error occurred.\n\r");
	        break;

	    case MODS_HAL_BUSY:
	        SerialSend("MODS HAL is busy.\n\r");
	        break;

	    case MODS_HAL_TIMEOUT:
	        SerialSend("MODS HAL Timeout occurred.\n\r");
	        break;

	    case MODS_CRC_ERROR:
	        SerialSend("MODS CRC Error occurred.\n\r");
	        break;

	    default:
	        SerialSend("Unknown MODS state.\n\r");
	        break;
	}

}


void task_Serial(void *pvParameters)
{
    Dispatcher_Task *TaskPtr = (Dispatcher_Task*) pvParameters;
	sMsgStruct Msg = {};

	SerialSend("*** Sewertronics 2 %.2f 2***\n\r", 2.0);
	printf("*** Sewertronics 2 %.2f 2***\n\r", 2.0);

	for (;;)
	{
		if (xQueueReceive(TaskPtr->GetQueueHandle(), &Msg, 1000))
		{

			switch (Msg.type) {
				case MT_MTR_STATUS:
				{
					sMsgMotorStatus *Status = (sMsgMotorStatus*)Msg.data;
					SerialSend("MtrStat Speed: %.2f Mod: %.2f Stat %d\n\r", Status->speed, Status->modifier, Status->status);
					break;
				}
				case MT_MTR_CONTROL:
				{
					sMsgMotorStatus *Control = (sMsgMotorStatus*)Msg.data;
					SerialSend("MtrCtrl Speed: %.2f Mod: %.2f Stat %d\n\r", Control->speed, Control->modifier, Control->status);
					printf("MtrCtrl Speed: %.2f Mod: %.2f Stat %d\n\r", Control->speed, Control->modifier, Control->status);
					break;
				}
				case MT_DISTANCE:
				{
					sMsgDistance *Status = (sMsgDistance*)Msg.data;
					SerialSend("Distance: %.2f m/min\n\r", Status->distance);
					break;
				}
				case MT_TEMPERATURE:
				{
					sMsgTmpr *Status = (sMsgTmpr*)Msg.data;
					SerialSend("Temp: %.2f ��C\n\r", Status->temperature);
					break;
				}
				case MT_UVHEAD_HUMIDITY:
				{
					sMsgHumidity *Status = (sMsgHumidity*)Msg.data;
					SerialSend("Hum: %.2f ��C\n\r", Status->humidity);
					break;
				}
				case MT_KNOB:
				{
					sMsgKnob *Status = (sMsgKnob*)Msg.data;
					SerialSend("Knob: %ld %ld\n\r", Status->absValue, Status->change);
					break;
				}
				case MT_PSU_STATUS:
				{
					sMsgPSUStatus *Status = (sMsgPSUStatus*)Msg.data;
					SerialSend("PSU Status %d %.2f\n\r", Status->state, Status->percent);
					break;
				}
				case MT_UVHEAD_PRESSURE:
				{
					sMsgUvHeadPressure *Status = (sMsgUvHeadPressure*)Msg.data;
					SerialSend("Pressure %d \n\r", Status->uvHeadPressure);
					break;
				}
				case MT_REED_SWITCH:
				{
					char Lookup[REED_MAX_STATES][10] = {"off", "front", "back", "all"};
					sMsgReedSwitchStatus *ReedSwitchStatus = (sMsgReedSwitchStatus*)Msg.data;
					//sMsgReedSwitchStatus ReedSwitchStatus;
					if(ReedSwitchStatus->state < REED_MAX_STATES)
					{
						SerialSend("Reeds %s \n\r", Lookup[ReedSwitchStatus->state]);
					}
					break;
				}
				case MT_PSU_CONTROL:
				{
					sMsgPSUControl *Status = (sMsgPSUControl*)Msg.data;
					SerialSend("PSU Control %d %.2f\n\r", Status->state, Status->setting);
					break;
				}
				case MT_BUTTON:
				{
					sMsgButtn *Status = (sMsgButtn*)Msg.data;
					switch (Status->value) {
					    case BS_PRESSED:
					        SerialSend("Button pressed\n\r");
					        break;
					    case BS_RELEASED:
					        SerialSend("Button released\n\r");
					        break;
					    case BS_HOLD_1S:
					        SerialSend("Button BS_HOLD_1S\n\r");
					        break;
					    default:
					        // Handle other cases if necessary
					        break;
					}
					break;
				}

				case MT_STATE:
				{
					sMsgState *Status = (sMsgState*)Msg.data;
					handleState(Status->state);
					handleTempState(Status->temperature);
					break;
				}

				case MT_MODBUS_STATE:
				{
					sMsgModbusState *Status = (sMsgModbusState*)Msg.data;
					HandleMODSState(Status->state);
					break;
				}

				case MT_PRINT_TASKS_STATS:
				{
					printTaskStats();
					break;
				}
				case MT_RESET_DISTANCE:
				{
					SerialSend("Reset distance\n\r");
					break;
				}


				default:
				{
					SerialSend("####### BAD INTERNAL MSG #######");
					break;
				}
		}
		}
		//printTaskStats();
//		sMsgSondeSet MsgSondeSet = {};
//		Dispatcher->DispatcherPostMsgByCopy(SL_CAN_RX_CHANNEL,SL_LED_BLUE,MT_BLUE_SET,&MsgSondeSet,sizeof(sMsgSondeSet));
		//vTaskDelay(100);
	}

}

void task_Serial_Init(uint8_t taskIndex)
{
	Dispatcher* Dispatcher = Dispatcher::getDispatcher();
	/*Dispatcher->dispatcherSubscribe(MT_MODBUS_STATE, taskIndex);

	Dispatcher->dispatcherSubscribe(MT_UVHEAD_PRESSURE, taskIndex);
	Dispatcher->dispatcherSubscribe(MT_TEMPERATURE, taskIndex);
	Dispatcher->dispatcherSubscribe(MT_UVHEAD_HUMIDITY, taskIndex);
	Dispatcher->dispatcherSubscribe(MT_REED_SWITCH, taskIndex);
	Dispatcher->dispatcherSubscribe(MT_DISTANCE, taskIndex);
	Dispatcher->dispatcherSubscribe(MT_PRINT_TASKS_STATS, taskIndex);


	Dispatcher->dispatcherSubscribe(MT_MTR_CONTROL, taskIndex);
	Dispatcher->dispatcherSubscribe(MT_PSU_CONTROL, taskIndex);
	Dispatcher->dispatcherSubscribe(MT_RESET_DISTANCE, taskIndex);*/

	/*Dispatcher->dispatcherSubscribe(MT_MTR_STATUS, taskIndex);


	Dispatcher->dispatcherSubscribe(MT_PSU_STATUS, taskIndex);
	Dispatcher->dispatcherSubscribe(MT_PSU_CONTROL, taskIndex);


	Dispatcher->dispatcherSubscribe(MT_STATE, taskIndex);*/
	//Dispatcher->dispatcherSubscribe(MT_TEMPERATURE, taskIndex);

	//Dispatcher->dispatcherSubscribe(MT_BUTTON, taskIndex);*/
	//Dispatcher->dispatcherSubscribe(MT_KNOB, taskIndex);



}

void SerialSendChar(const char *Text)
{
	HAL_UART_Transmit(&huart3, (const uint8_t *)Text, strlen((char*)Text), HAL_MAX_DELAY);
}

void SerialSend(const char *Text, ...)
{
	uint8_t buf[512];
	va_list Params;
	va_start(Params, Text);
	vsprintf((char*)buf, Text, Params);
	HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), HAL_MAX_DELAY);
	va_end(Params);
}
