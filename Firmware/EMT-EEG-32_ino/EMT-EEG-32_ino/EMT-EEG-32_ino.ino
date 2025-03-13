
#include <SPI.h>
#include "EEG_channel.h"

#include "task_read_ADC.h"
#include "task_write_DAC.h"

#define DEFAULT_STACK_SIZE 2048
#define CORE_1 1
#define CORE_0 0

SPIClass* p_spi = &SPI;

AD777x::Config config{
  .spi_host = p_spi,
  .sclk_pin = 15,
  .miso_pin = 14,
  .mosi_pin = 13,
  .cs_pin = 16,
  .drdy_pin = 27,
  .reset_pin = 7,
  .mode0_pin = 32,
  .mode1_pin = 33,
  .mode2_pin = 25,
  .mode3_pin = 26,
  .dclk0_pin = 27,
  .dclk1_pin = 14,
  .dclk2_pin = 12,
  .sync_in_pin = 13,
  .convst_sar_pin = 15,
  .ctrl_mode = AD777x::CtrlMode::AD777x_SPI_CTRL,
  .spi_crc_en = AD777x::State::AD777x_DISABLE,
  .state = {AD777x::State::AD777x_ENABLE, AD777x::State::AD777x_ENABLE, AD777x::State::AD777x_ENABLE, AD777x::State::AD777x_ENABLE,
            AD777x::State::AD777x_ENABLE, AD777x::State::AD777x_ENABLE, AD777x::State::AD777x_ENABLE, AD777x::State::AD777x_ENABLE},
  .dec_rate_int = 128,
  .dec_rate_dec = 0,
  .sinc5_state = AD777x::Sinc5State::AD777x_ENABLE
};

AD777x adc;
DAC7678 dac(DAC7678_DEFAULT_ADDRESS, Wire);

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
  Serial.begin(115200);
  delay(1000);
  //pinMode(config.cs_pin, OUTPUT);
	//digitalWrite(config.cs_pin, HIGH); 
  //SPI.begin(/* config.sclk_pin */15, /* config.miso_pin */14, /* config.mosi_pin */13);
  Serial.println("Por lo menos inicio?");
  Serial.printf("Address de SPI: %d", config.spi_host);
  adc.initialize(config);

  EEG_channels.push_back(EEG_channel1);

  xTaskCreatePinnedToCore(task_read_ADC, "read_ADC", DEFAULT_STACK_SIZE * 2, NULL, 1, NULL, CORE_1);
  //xTaskCreatePinnedToCore(task_write_DAC, "write_DAC", DEFAULT_STACK_SIZE * 2, NULL, 1, NULL, CORE_0);
}

void loop() {
  // put your main code here, to run repeatedly:
}
