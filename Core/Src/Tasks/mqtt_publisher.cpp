/*
 * mqtt_publisher.c
 *
 *  Created on: Sep 17, 2025
 *      Author: p.krzyszkowski
 */


#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>
#include "mqtt_publisher.h"
#include "stdbool.h"
#include "tcpip.h"
#include "../Dispatcher/dispatcher.hpp"
#include <vector>

static int mqttConnected = 0;
static bool mqttSubscribed = false;

// MQTT connection callback
void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT connection accepted!\n");
        mqttConnected = 1;


    } else {
        printf("!@#$   MQTT connection failed: %d   !@#$\n", status);
        mqttConnected = 0;
        mqttSubscribed = false;
    }
}

void mqtt_subscribe_callback(void *arg, err_t err);


class MqttTopicHandler {
public:
    struct TopicEntry {
        const char* topic;
        int id;
        const char* description;
    };

    MqttTopicHandler() : topicID(0) {
        topics = {
            { "hw/pulling/1/speed/1",    1, "Speed topic received!" },
            { "hw/CU/1/psu/1",           2, "UV topic received!" },
            { "hw/CU/1/stats/1",         3, "Stats topic received!" },
            { "hw/pulling/1/resetDistance/1", 4, "Distance topic received!" }
        };
    }

    void handleIncoming(const char* topic, uint32_t tot_len)
    {
        for (const auto& entry : topics) {
            if (strcmp(topic, entry.topic) == 0)
            {
                printf(entry.description);
                topicID = entry.id;
                return;
            }
        }

        printf("Unknown topic received.\n");
        topicID = 0;
    }

    void handleDataFragment(const uint8_t* data, uint16_t len, uint8_t flags)
    {
        printf("Received data fragment: %.*s for topic %d\n", len, data, topicID);
        constexpr int bufferSize = 30;
        char tmp[bufferSize] = {0};
        //char tmp[2500] = {0};


        Dispatcher* dispatcher = Dispatcher::getDispatcher();
        //leave one char for null
        int lenToCpy = len < (bufferSize-1) ? len : (bufferSize-1);
        memcpy(tmp, data, lenToCpy);
        if (topicID == 1) {
        	//data[len] = '\0';
            float speed = std::atof(reinterpret_cast<const char*>(tmp));
            sMsgMotorStatus control = { MTR_MOVING, 0, 1 };
            control.speed = speed;
            dispatcher->DispatcherPostMsgByCopy(MT_MTR_CONTROL, &control, sizeof(control));
        }
        else if (topicID == 2) {
            float value = std::atof(reinterpret_cast<const char*>(tmp));
            sMsgPSUControl control = { PSU_OFF, 0 };
            control.setting = value*100;
            dispatcher->DispatcherPostMsgByCopy(MT_PSU_CONTROL, &control, sizeof(control));
        }
        else if (topicID == 3) {
            float value = std::atof(reinterpret_cast<const char*>(tmp)); // opcjonalne
            int dummy;
            dispatcher->DispatcherPostMsgByCopy(MT_PRINT_TASKS_STATS, &dummy, sizeof(dummy));
        }
        else if (topicID == 4) {
            sMsgResetDistance reset = { true };
            dispatcher->DispatcherPostMsgByCopy(MT_RESET_DISTANCE, &reset, sizeof(reset));
        }

        if (flags & MQTT_DATA_FLAG_LAST) {
            printf("End of message.\n");
        }
    }


    err_t subscribeAll(mqtt_client_t* client)
    {
		err_t result = ERR_OK;
		for (const auto& entry : topics)
		{
			result |= mqtt_sub_unsub(client, entry.topic, 0, mqtt_subscribe_callback, (void*)entry.topic, 1);
		}
		return result;
	}


private:
    std::vector<TopicEntry> topics;
    int topicID;
};


MqttTopicHandler mqttHandler;

// Called when a new publish message starts arriving
void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    printf("Incoming message on topic: %s, total length: %lu\n", topic, (unsigned long)tot_len);
    mqttHandler.handleIncoming(topic, tot_len);
}

err_t
simple_mqtt_publish(mqtt_client_t *client, const char *topic, const void *payload, u16_t payload_length, int connected)
{
	if(connected == 1)
	{
		LOCK_TCPIP_CORE();
		err_t ret = mqtt_publish(client, topic, payload, payload_length, 0, 0, NULL, NULL);
		UNLOCK_TCPIP_CORE();
		return ret;
	}
	else
	{
		return ERR_CONN;
	}
}

// Called for each fragment of the payload
void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	printf("Received data fragment: %.*s\n", len, data);
	mqttHandler.handleDataFragment(data, len, flags);

    if (flags & MQTT_DATA_FLAG_LAST) {
        printf("End of message.\n");
    }
}



// Callback function for subscription response
void mqtt_subscribe_callback(void *arg, err_t err)
{
	if (err == ERR_OK) {
		printf("Subscribed successfully to topic: %s\n", (char *)arg);
	} else {
		printf("Failed to subscribe to topic: %s, error: %d\n", (char *)arg, err);
	}
}



const char* msgTopicLookup[MT_NUM_TYPES] = { NULL };

void initMsgTopicLookup(void)
{
    msgTopicLookup[MT_UVHEAD_PRESSURE]   = "hw/head/1/pressure/1";
    msgTopicLookup[MT_DISTANCE]          = "hw/pulling/1/distance/1";
    msgTopicLookup[MT_TEMPERATURE]       = "hw/head/1/temperature/1";
    msgTopicLookup[MT_MODBUS_STATE]      = "hw/CU/1/modbus/1";
    msgTopicLookup[MT_REED_SWITCH]       = "hw/head/1/reed/1";
    msgTopicLookup[MT_UVHEAD_HUMIDITY]   = "hw/head/1/humidity/1";
}



// FreeRTOS task to run MQTT client
void task_MQTT(void *pvParameters)
{
	printf("Started vMQTTTask task\n");
    Dispatcher_Task *TaskPtr = (Dispatcher_Task*) pvParameters;
	sMsgStruct Msg = {};
	//osDelay(20000);

    mqtt_client_t *client = mqtt_client_new();
    if (!client) {
        printf("Failed to create MQTT client\n");
        vTaskDelete(NULL);
        return;
    }
    else
    {
    	printf("MQTT client created\n");
    }

    ip_addr_t broker_ip;
    //IP4_ADDR(&broker_ip, 5, 196, 78, 28); // Replace with your broker IP
    //IP4_ADDR(&broker_ip, 192, 168, 101, 2); //home
    IP4_ADDR(&broker_ip, 10, 74, 90, 17); //work
    struct mqtt_connect_client_info_t ci = {
        .client_id = "stm32_freertos_client",
        .client_user = "user",
        .client_pass = "password",
        .keep_alive = 60,
        .will_topic = NULL,
        .will_msg = NULL,
        .will_qos = 0,
        .will_retain = 0
    };

    /*struct mqtt_connect_client_info_t ci = {
        .client_id = "stm32_freertos_client",
        .client_user = NULL,
        .client_pass = NULL,
        .keep_alive = 60,
        .will_topic = NULL,
        .will_msg = NULL,
        .will_qos = 0,
        .will_retain = 0
    };*/

    //mqtt_tcp_err_cb: TCP error callback: error -14, arg: 0x20078154



	initMsgTopicLookup();


    // Task can optionally wait here or do other work
    while (1) {
    	HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
        //vTaskDelay(10);
        if (xQueueReceive(TaskPtr->GetQueueHandle(), &Msg, 1000))
		{
        	char str[30] = "err";
        	const char *topic = msgTopicLookup[Msg.type];
			switch (Msg.type) {
				case MT_UVHEAD_PRESSURE:
				{
					sMsgUvHeadPressure *Status = (sMsgUvHeadPressure*)Msg.data;
					sprintf(str, "%.2f", Status->uvHeadPressure/100.0);

					break;
				}
				case MT_DISTANCE:
				{
					sMsgDistance *Status = (sMsgDistance*)Msg.data;
					sprintf(str, "%.2f", Status->distance);

					break;
				}
				case MT_TEMPERATURE:
				{
     				sMsgTmpr *Status = (sMsgTmpr*)Msg.data;
					sprintf(str, "%.2f", Status->temperature);
					break;
				}
				case MT_MODBUS_STATE:
				{
					sMsgModbusState *Status = (sMsgModbusState*)Msg.data;
					static int errCounter= 0;
					errCounter++;
					sprintf(str, "%d", errCounter);
					break;
				}
				case MT_REED_SWITCH:
				{
					sMsgReedSwitchStatus *ReedSwitchStatus = (sMsgReedSwitchStatus*)Msg.data;
					sprintf(str, "%d", (int)ReedSwitchStatus->state);
					break;
				}
				case MT_UVHEAD_HUMIDITY:
				{
					sMsgHumidity *Status = (sMsgHumidity*)Msg.data;
					sprintf(str, "%.2f", Status->humidity);
					break;
				}
				default:
					break;
			}
			simple_mqtt_publish(client, topic, str, strlen(str), mqttConnected);
		}

        static int counter = 0;
        if(mqttConnected == 1)
        {
			const char *topicTemp = "head/1/temp/1";
			const char *topicPres = "head/1/pressure/1";
			const char *message = "Hello from Piotr!";

			char str[30];
			sprintf(str, "%d", counter++);

			//mqtt_publish(client, topic, message, strlen(message), 0, 0, NULL, NULL);
			/*LOCK_TCPIP_CORE();
			mqtt_publish(client, topicTemp, str, strlen(str), 0, 0, NULL, NULL);
			mqtt_publish(client, topicPres, str, strlen(str), 0, 0, NULL, NULL);
			UNLOCK_TCPIP_CORE();*/

			if(mqttSubscribed == false)
			{

				// Call the subscribe function
				printf("Subscribe to mqtt speed1\n");
				mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);
				LOCK_TCPIP_CORE();
				err_t result = mqttHandler.subscribeAll(client);
				UNLOCK_TCPIP_CORE();

				if (result != ERR_OK)
				{
					printf("mqtt_sub_unsub() failed with error: %d\n", result);
				}
				else
				{
					mqttSubscribed = true;
				}
			}
        }
        else if(mqttConnected == 0)
        {
        	printf("################### connecting MQTT in 10s ####################\n");
        	vTaskDelay(pdMS_TO_TICKS(10000));
        	mqttConnected = 2;
        	LOCK_TCPIP_CORE();
        	err_t err = mqtt_client_connect(client, &broker_ip, MQTT_PORT, mqtt_connection_cb, NULL, &ci);
        	UNLOCK_TCPIP_CORE();
			if (err != ERR_OK) {
				printf("MQTT connect failed: %d\n", err);
				mqttConnected = 0;
				//mqtt_client_free(client);
				//vTaskDelete(NULL);
				//return;
			}
			else
			{
				printf("MQTT connection request send\n");
			}
        }

    }
}

// Call this from main or another init function
/*void start_mqtt_task(void) {
    xTaskCreate(vMQTTTask, "MQTTTask", 1024, NULL, tskIDLE_PRIORITY, NULL);
}*/

void task_Mqtt_Init(uint8_t taskIndex)
{
	Dispatcher* Dispatcher = Dispatcher::getDispatcher();
	Dispatcher->dispatcherSubscribe(MT_TEMPERATURE, taskIndex);
	Dispatcher->dispatcherSubscribe(MT_DISTANCE, taskIndex);
	Dispatcher->dispatcherSubscribe(MT_UVHEAD_PRESSURE, taskIndex);
	Dispatcher->dispatcherSubscribe(MT_UVHEAD_HUMIDITY, taskIndex);
	Dispatcher->dispatcherSubscribe(MT_MODBUS_STATE, taskIndex);
	Dispatcher->dispatcherSubscribe(MT_REED_SWITCH, taskIndex);

	//Dispatcher->dispatcherSubscribe(MT_MTR_CONTROL, taskIndex);
	//Dispatcher->dispatcherSubscribe(MT_RESET_DISTANCE, taskIndex);

}


