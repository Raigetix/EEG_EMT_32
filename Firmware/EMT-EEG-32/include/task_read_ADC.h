#ifndef _TASK_ADC_H
#define _TASK_ADC_H

#include <Arduino.h>
#include <FreeRTOS.h>

#include "EEG_channel.h"

void task_read_ADC(void *parameter);

extern AD777x adc;

#endif

