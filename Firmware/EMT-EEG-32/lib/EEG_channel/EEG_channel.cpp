#include "EEG_channel.h"

EEG_channel::EEG_channel(AD777x* adc, DAC7678* dac, uint8_t channel){
    this->adc = adc;
    this->dac = dac;
    this->channel = channel;

    this->enable = true;

    switch (this->channel){

    case 0:
        ADC_Channel = AD777x::Channel::AD777x_CH0;
        DAC_Channel = DAC7678::Channel::A;
        break;

    case 1:
        ADC_Channel = AD777x::Channel::AD777x_CH1;
        DAC_Channel = DAC7678::Channel::B;
        break;

    case 2:
        ADC_Channel = AD777x::Channel::AD777x_CH2;
        DAC_Channel = DAC7678::Channel::C;
        break;

    case 3:
        ADC_Channel = AD777x::Channel::AD777x_CH3;
        DAC_Channel = DAC7678::Channel::D;
        break;

    case 4:
        ADC_Channel = AD777x::Channel::AD777x_CH4;
        DAC_Channel = DAC7678::Channel::E;
        break;

    case 5:
        ADC_Channel = AD777x::Channel::AD777x_CH5;
        DAC_Channel = DAC7678::Channel::F;
        break;
    
    case 6:
        ADC_Channel = AD777x::Channel::AD777x_CH6;
        DAC_Channel = DAC7678::Channel::G;
        break;

    case 7:
        ADC_Channel = AD777x::Channel::AD777x_CH7;
        DAC_Channel = DAC7678::Channel::H;
        break;

    
    default:
        break;
    }
}

EEG_channel::~EEG_channel(){
    delete adc;
    delete dac;
}

uint32_t EEG_channel::get_ADC_Channel_Value(){
    return this->adc->read_channel(this->ADC_Channel);
}

void EEG_channel::set_DAC_Channel_Value(uint32_t value){
    dac->setDAC(DAC_Channel, value);
}

bool EEG_channel::isEnable(){
    return this->enable;
}

void EEG_channel::setEnable(bool enable){
    this->enable = enable;
}

uint8_t EEG_channel::getChannel(){
    return this->channel;
}

std::vector<EEG_channel> EEG_channels;