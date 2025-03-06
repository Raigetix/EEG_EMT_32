#include "EEG_channel.h"

EEG_channel::EEG_channel(AD7771* adc, DAC7678* dac, uint8_t channel){
    this->adc = adc;
    this->dac = dac;
    this->channel = channel;

    this->enable = true;

    switch (this->channel){
    case 1:
        ADC_Channel = AD7771_CH0;
        DAC_Channel = 0b0000;
        break;
    case 2:
        ADC_Channel = AD7771_CH1;
        DAC_Channel = 0b0001;
        break;
    
    case 3:
        ADC_Channel = AD7771_CH2;
        DAC_Channel = 0b0010;
        break;
    
    case 4:
        ADC_Channel = AD7771_CH3;
        DAC_Channel = 0b0011;
        break;
    
    case 5:
        ADC_Channel = AD7771_CH4;
        DAC_Channel = 0b0100;
        break;
    
    case 6:
        ADC_Channel = AD7771_CH5;
        DAC_Channel = 0b0101;
        break;
    
    case 7:
        ADC_Channel = AD7771_CH6;
        DAC_Channel = 0b0110;
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
    uint16_t value;
    adc->doSingleSarConversion(AD7771_REF1P_REF1N, &value);
    return value;
}

void EEG_channel::set_DAC_Channel_Value(uint32_t value){
    dac->setDAC(DAC_Channel, value);
}

void EEG_channel::config_ADC_Channel(ADC_config config){
    adc->setReferenceType(config.reference);
    adc->setDclkDivider(config.clk_div);
    adc->setDecimationRate(config.resolution_mode, config.filter);
    adc->setGain((ad7771_ch)ADC_Channel, config.channel_gain);
}

void EEG_channel::config_DAC_Channel(DAC_config config){
    dac->setReferenceMode((DAC7678::ReferenceMode) config.referenceMode);
    dac->chooseUpdateMode((DAC7678::UpdateMode)config.updateMode);
    dac->setPDMode((DAC7678::PDMode)config.powerMode, DAC_Channel);
    dac->setToStaticMode();
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