#include "task_write_DAC.h"


void task_write_DAC(void *pvParameters){
    while (true){
        for(auto channel : EEG_channels){
            if (channel.isEnable()){
                channel.set_DAC_Channel_Value(random(MIN_DAC_VALUE, MAX_DAC_VALUE));
            }
        }
        delay(10);
    }
}