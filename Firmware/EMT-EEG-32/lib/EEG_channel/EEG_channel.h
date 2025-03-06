#ifndef CHANNEL_H
#define CHANNEL_H

#include <vector>
#include <Arduino.h>
#include "AD7771.h"
#include "DAC7678.h"


typedef struct {
    uint8_t reference;
    uint8_t resolution_mode;
    uint8_t clk_div; 
    uint8_t filter; 
    ad7771_gain channel_gain;
} ADC_config;

typedef struct {
    uint8_t controlMode;
    uint8_t powerMode;
    uint8_t spreedMode;
    uint8_t referenceMode;
    uint8_t updateMode;
} DAC_config;


class EEG_channel{

    private:
        AD7771* adc;
        DAC7678* dac;

        uint8_t channel;

        uint8_t ADC_Channel;
        uint8_t DAC_Channel;

        bool enable;

    public:
        EEG_channel(AD7771* adc, DAC7678* dac, uint8_t channel);
        ~EEG_channel();

        uint32_t get_ADC_Channel_Value();
        void set_DAC_Channel_Value(uint32_t value);

        void config_ADC_Channel(ADC_config config);
        void config_DAC_Channel(DAC_config config);

        uint8_t getChannel();

        bool isEnable();
        void setEnable(bool enable);

/*         int32_t setChannelState(ad7771_state state);
        ad7771_state getChannelState();
        int32_t setGain(ad7771_gain gain);
        ad7771_gain getGain();
        int32_t setOffsetCorrection(uint32_t offset);
        uint32_t getOffsetCorrection();
        int32_t setGainCorrection(uint32_t gain);
        uint32_t getGainCorrection();
        int32_t doSingleSarConversion(ad7771_sar_mux mux, uint16_t* sar_code);
        int32_t setDacValue(uint16_t dac_value);
        uint16_t getDacValue();
        int32_t setDacState(dac7678_state state);
        dac7678_state getDacState();
        int32_t setDacPowerMode(uint8_t mode);
        uint8_t getDacPowerMode();
        int32_t setDacGain(dac7678_gain gain);
        dac7678_gain getDacGain();
        int32_t setDacReferenceType(uint8_t type);
        uint8_t getDacReferenceType();
        int32_t setDacOutputMode(uint8_t mode);
        uint8_t getDacOutputMode();
        int32_t setDacLoadMode(uint8_t mode);
        uint8_t getDacLoadMode();
        int32_t setDacLoadValue(uint8_t value);
        uint8_t getDacLoadValue();
        int32_t setDacLoadState(dac7678_load_state state);
        dac7678_load_state getDacLoadState();
        int32_t setDacLoadPowerMode(uint8_t mode);
        uint8_t getDacLoadPowerMode();
        int32_t setDacLoadGain(dac7678_gain gain);
        dac7678_gain getDacLoadGain();
        int32_t setDacLoadReferenceType(uint8_t type);
        uint8_t getDacLoadReferenceType();
        int32_t setDacLoadOutputMode(uint8_t mode);
        uint8_t getDacLoadOutputMode();
        int32_t setDacLoadLoadMode(uint8_t mode);
        uint8_t getDacLoadLoadMode();
        int32_t setDacLoadLoadValue(uint8_t value); */
};

extern std::vector<EEG_channel> EEG_channels;

#endif