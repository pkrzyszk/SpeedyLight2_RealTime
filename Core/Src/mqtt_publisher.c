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

static bool mqttConnected = false;

// MQTT connection callback
void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT connection accepted!\n");
        mqttConnected = true;

    } else {
        printf("!@#$   MQTT connection failed: %d   !@#$\n", status);
        mqttConnected = false;
    }
}

// FreeRTOS task to run MQTT client
void vMQTTTask(void *pvParameters)
{
	printf("Started vMQTTTask task\n");
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
    IP4_ADDR(&broker_ip, 192, 168, 101, 24);
    /*struct mqtt_connect_client_info_t ci = {
        .client_id = "stm32_freertos_client",
        .client_user = "user",
        .client_pass = "password",
        .keep_alive = 60,
        .will_topic = NULL,
        .will_msg = NULL,
        .will_qos = 0,
        .will_retain = 0
    };*/

    struct mqtt_connect_client_info_t ci = {
        .client_id = "stm32_freertos_client",
        .client_user = NULL,
        .client_pass = NULL,
        .keep_alive = 60,
        .will_topic = NULL,
        .will_msg = NULL,
        .will_qos = 0,
        .will_retain = 0
    };

    //mqtt_tcp_err_cb: TCP error callback: error -14, arg: 0x20078154

    // Task can optionally wait here or do other work
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000)); // Delay to simulate periodic work
        static int counter = 0;
        if(mqttConnected)
        {
			const char *topicTemp = "temp1";
			const char *topicPres = "pressure1";
			const char *message = "Hello from Piotr!";

			char str[30];
			sprintf(str, "%d", counter++);

			//mqtt_publish(client, topic, message, strlen(message), 0, 0, NULL, NULL);
			mqtt_publish(client, topicTemp, str, strlen(str), 0, 0, NULL, NULL);
			mqtt_publish(client, topicPres, str, strlen(str), 0, 0, NULL, NULL);
        }
        else
        {
        	printf("################### connecting MQTT ####################\n");

        	err_t err = mqtt_client_connect(client, &broker_ip, MQTT_PORT, mqtt_connection_cb, NULL, &ci);
			if (err != ERR_OK) {
				printf("MQTT connect failed: %d\n", err);
				mqtt_client_free(client);
				vTaskDelete(NULL);
				return;
			}
			else
			{
				printf("MQTT connection request send\n");
			}
        }

    }
}

// Call this from main or another init function
void start_mqtt_task(void) {
    xTaskCreate(vMQTTTask, "MQTTTask", 1024, NULL, tskIDLE_PRIORITY, NULL);
}


