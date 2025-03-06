#include <Arduino.h>
#include <FreeRTOS.h>

#include "EEG_channel.h"

void task_read_ADC(void *parameter);