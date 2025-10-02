/*
 * mqtt_publisher.h
 *
 *  Created on: Sep 17, 2025
 *      Author: p.krzyszkowski
 */

#ifndef INC_MQTT_PUBLISHER_H_
#define INC_MQTT_PUBLISHER_H_


//void start_mqtt_task(void);
void task_Mqtt_Init(uint8_t taskIndex);
void task_MQTT(void *pvParameters);


#endif /* INC_MQTT_PUBLISHER_H_ */
