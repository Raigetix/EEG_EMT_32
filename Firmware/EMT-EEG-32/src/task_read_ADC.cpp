#include "task_read_ADC.h"

void task_read_ADC(void *parameter){
    while(true){
        adc.read_all_channels();
        for(auto &channel : EEG_channels){
            if(channel.isEnable()){
                uint32_t value = channel.get_ADC_Channel_Value();
                Serial.print("Channel ");
                Serial.print(channel.getChannel());
                Serial.print(": ");
                Serial.println(value);
            }
        }
        delay(10);
    }
}