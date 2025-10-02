#pragma once

#include <stdlib.h>
#include "../Dispatcher/dispatcher.hpp"

void task_Serial(void *pvParameters);
void task_Serial_Init(uint8_t taskIndex);

void SerialSend(const char *Text, ...);
void SerialSendChar(const char *Text);
