#ifndef CHANNEL_H
#define CHANNEL_H

#include <vector>
#include <Arduino.h>
#include "AD777x.h"
#include "DAC7678.h"

class EEG_channel{

    private:
        AD777x* adc;
        DAC7678* dac;

        uint8_t channel;

        AD777x::Channel ADC_Channel;
        DAC7678::Channel DAC_Channel;

        bool enable;

    public:
        EEG_channel(AD777x* adc, DAC7678* dac, uint8_t channel);
        ~EEG_channel();

        uint32_t get_ADC_Channel_Value();
        void set_DAC_Channel_Value(uint32_t value);

        uint8_t getChannel();

        bool isEnable();
        void setEnable(bool enable);
};

extern std::vector<EEG_channel> EEG_channels;

#endif