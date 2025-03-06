#include <Arduino.h>
#include "EEG_channel.h"

#include "task_read_ADC.h"
#include "task_write_DAC.h"

#define DEFAULT_STACK_SIZE 2048
#define CORE_1 1
#define CORE_0 0

AD7771 adc(&SPI, 7);
DAC7678 dac;

EEG_channel EEG_channel1(&adc, &dac, 1);
/*EEG_channel EEG_channel2(&adc, &dac, 2);
EEG_channel EEG_channel3(&adc, &dac, 3);
EEG_channel EEG_channel4(&adc, &dac, 4);
EEG_channel EEG_channel5(&adc, &dac, 5);
EEG_channel EEG_channel6(&adc, &dac, 6);
EEG_channel EEG_channel7(&adc, &dac, 7); */

// put function declarations here:
int myFunction(int, int);

void setup() {

  adc.begin(
    AD7771_INT_REF,
    AD7771_HIGH_RES,
    AD7771_DCLK_DIV_1, 
    AD7771_DISABLE, 
    AD7771_GAIN_1
  );
  EEG_channels.push_back(EEG_channel1);

  xTaskCreatePinnedToCore(task_read_ADC, "read_ADC", DEFAULT_STACK_SIZE * 2, NULL, 1, NULL, CORE_1);
  xTaskCreatePinnedToCore(task_write_DAC, "write_DAC", DEFAULT_STACK_SIZE * 2, NULL, 1, NULL, CORE_0);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}