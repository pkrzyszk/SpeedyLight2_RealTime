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

// MQTT connection callback
void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT connected!\n");

        const char *topic = "test/topic";
        const char *message = "Hello from FreeRTOS!";
        mqtt_publish(client, topic, message, strlen(message), 0, 0, NULL, NULL);
    } else {
        printf("MQTT connection failed: %d\n", status);
    }
}

// FreeRTOS task to run MQTT client
void vMQTTTask(void *pvParameters)
{
	printf("Started vMQTTTask task\n");
	osDelay(20000);
    mqtt_client_t *client = mqtt_client_new();
    if (!client) {
        printf("Failed to create MQTT client\n");
        vTaskDelete(NULL);
        return;
    }

    ip_addr_t broker_ip;
    IP4_ADDR(&broker_ip, 192, 168, 1, 100); // Replace with your broker IP
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

    err_t err = mqtt_client_connect(client, &broker_ip, MQTT_PORT, mqtt_connection_cb, NULL, &ci);
    if (err != ERR_OK) {
        printf("MQTT connect failed: %d\n", err);
        mqtt_client_free(client);
        vTaskDelete(NULL);
        return;
    }

    // Task can optionally wait here or do other work
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to simulate periodic work
    }
}

// Call this from main or another init function
void start_mqtt_task(void) {
    xTaskCreate(vMQTTTask, "MQTTTask", 1024, NULL, tskIDLE_PRIORITY, NULL);
}


