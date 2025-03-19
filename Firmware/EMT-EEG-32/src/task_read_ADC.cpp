#include "task_read_ADC.h"

void task_read_ADC(void *parameter){
    uint32_t adc_raw_buff[NUM_CHANNELS] = { 0x0 };
    uint32_t timeout;
    uint32_t sample_index = 0;
    int32_t ret;
    while(true){

    
        /* Start conversion by setting the ADC to SD conversion mode */
        adc.set_spi_operation_mode(AD777x::SpiOpMode::AD777x_SD_CONV);
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