#include "DummyTask.hpp"
//#include "../Dispatcher/dispatcher.hpp"


#include "main.h"



void task_DummyTask(void *pvParameters)
{
    Dispatcher_Task *TaskPtr = (Dispatcher_Task*) pvParameters;
	sMsgStruct Msg = {};
	Dispatcher* dispatcher = Dispatcher::getDispatcher();
	sMsgTmpr MsgTmpr = {65};


	for (;;)
	{
		if (xQueueReceive(TaskPtr->GetQueueHandle(), &Msg, 100))
		{

		}
		MsgTmpr.temperature += 1;
		if(MsgTmpr.temperature > 80)
		{
			MsgTmpr.temperature = 65;
		}
		//dispatcher->DispatcherPostMsgByCopy(MT_TEMPERATURE, &MsgTmpr, sizeof(MsgTmpr));
	}

}

void task_DummyTask_Init(uint8_t taskIndex)
{

}
