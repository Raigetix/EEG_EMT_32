#include <Arduino.h>
#include <FreeRTOS.h>
#include "EEG_channel.h"

#define MAX_DAC_VALUE 4095
#define MIN_DAC_VALUE 1000

void task_write_DAC(void *pvParameters);