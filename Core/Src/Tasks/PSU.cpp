#include "PSU.hpp"

#include "main.h"
#include "cmsis_os.h"
#include "PWM.hpp"



using namespace STM32_drivers;

#include "Serial.hpp"


//static Dispatcher* Dispatcher = Dispatcher::getDispatcher();

void task_PSU(void *pvParameters)
{
    Dispatcher_Task *TaskPtr = (Dispatcher_Task*) pvParameters;
	sMsgStruct Msg = {};
	Dispatcher* Dispatcher = Dispatcher::getDispatcher();

	double MaxDuty = PSU_MAX;

	PWM *UVLED = new PWM(htim3, TIM_CHANNEL_4, 0, MaxDuty);
	UVLED->SetDuty(0);
	ePSU_State state = PSU_OFF;

	sMsgPSUStatus MSgPSU = {state, UVLED->GetDuty()};
	Dispatcher->DispatcherPostMsgByCopy(MT_PSU_STATUS, &MSgPSU, sizeof(MSgPSU));

	for (;;)
	{
		if (xQueueReceive(TaskPtr->GetQueueHandle(), &Msg, portMAX_DELAY))
		{
			switch (Msg.type) {
				case MT_PSU_CONTROL:
				{
					sMsgPSUControl *Control = (sMsgPSUControl*)Msg.data;
					UVLED->SetDuty(Control->setting);
					break;
				}
				default:
					break;
			}
			sMsgPSUStatus MSgPSU = {state, UVLED->GetDuty()};
			Dispatcher->DispatcherPostMsgByCopy(MT_PSU_STATUS, &MSgPSU, sizeof(MSgPSU));
		}
	}


}

void task_PSU_Init(uint8_t taskIndex)
{
	Dispatcher* Dispatcher = Dispatcher::getDispatcher();
	Dispatcher->dispatcherSubscribe(MT_PSU_CONTROL, taskIndex);
}
