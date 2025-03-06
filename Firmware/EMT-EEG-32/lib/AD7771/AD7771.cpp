#include "AD7771.h"

const uint8_t pin_mode_options[16][4] = {
	/*GAIN_1, GAIN_2, GAIN_4, GAIN_8 */
	{0x03,	0xFF,	0x07,	0xFF},	// DEC_RATE_128, HIGH_RES, EXT_REF
	{0x0A,	0xFF,	0xFF,	0xFF},	// DEC_RATE_128, HIGH_RES, INT_REF
	{0x0D,	0xFF,	0xFF,	0xFF},	// DEC_RATE_128, LOW_PWR, EXT_REF
	{0x0E,	0xFF,	0xFF,	0xFF},	// DEC_RATE_128, LOW_PWR, INT_REF
	{0x02,	0x04,	0x06,	0xFF},	// DEC_RATE_256, HIGH_RES, EXT_REF
	{0x09,	0xFF,	0xFF,	0xFF},	// DEC_RATE_256, HIGH_RES, INT_REF
	{0x0C,	0xFF,	0xFF,	0xFF},	// DEC_RATE_256, LOW_PWR, EXT_REF
	{0x0F,	0xFF,	0xFF,	0xFF},	// DEC_RATE_256, LOW_PWR, INT_REF
	{0x01,	0xFF,	0x05,	0xFF},	// DEC_RATE_512, HIGH_RES, EXT_REF
	{0x08,	0xFF,	0xFF,	0xFF},	// DEC_RATE_512, HIGH_RES, INT_REF
	{0x08,	0xFF,	0xFF,	0xFF},	// DEC_RATE_512, LOW_PWR, EXT_REF
	{0xFF,	0xFF,	0xFF,	0xFF},	// DEC_RATE_512, LOW_PWR, INT_REF
	{0x00,	0xFF,	0xFF,	0xFF},	// DEC_RATE_1024, HIGH_RES, EXT_REF
	{0xFF,	0xFF,	0xFF,	0xFF},	// DEC_RATE_1024, HIGH_RES, INT_REF
	{0xFF,	0xFF,	0xFF,	0xFF},	// DEC_RATE_1024, LOW_PWR, EXT_REF
	{0xFF,	0xFF,	0xFF,	0xFF},	// DEC_RATE_1024, LOW_PWR, INT_REF
};

// Implementation of AD7771 methods
AD7771::AD7771(SPIClass* spi, uint8_t reset_pin) {
    this->spi           = spi;
    this->reset_pin     = reset_pin;
    this->sync_in       = sync_in;
    this->spi_crc_en    = AD7771_ENABLE;

    this->dec_rate_int  = 0x0000;
    this->dec_rate_dec  = 0x0000;
    this->ref_type      = AD7771_INT_REF;
    this->pwr_mode      = AD7771_HIGH_RES;
    this->dclk_div      = AD7771_DCLK_DIV_1;

    this->ref_buf_op_mode[0] = AD7771_REF_BUF_ENABLED;
    this->ref_buf_op_mode[1] = AD7771_REF_BUF_ENABLED;

    this->sinc5_state = AD7771_DISABLE;

    for (int i = 0; i < 8; ++i) {
        state[i] = AD7771_ENABLE;
        gain[i] = AD7771_GAIN_1;
    }
}

AD7771::~AD7771() {
    // Cleanup resources if needed
}

int32_t AD7771::begin( uint8_t reference = AD7771_INT_REF, uint8_t resolution_mode = AD7771_HIGH_RES, uint8_t clk_div = AD7771_DCLK_DIV_1, uint8_t filter = AD7771_DISABLE, ad7771_gain channel_gain = AD7771_GAIN_1) {
    if(spi_init == false){
        spi_init = true;
        this->spi->begin(AD7771_SCK, AD7771_MISO, AD7771_MOSI, AD7771_CS);

        this->dec_rate_int  = 0x0000;
        this->dec_rate_dec  = 0x0000;
        this->ref_type      = reference;
        this->pwr_mode      = resolution_mode;
        this->dclk_div      = clk_div;

        this->ref_buf_op_mode[0] = AD7771_REF_BUF_ENABLED;
        this->ref_buf_op_mode[1] = AD7771_REF_BUF_ENABLED;

        this->sinc5_state = filter;

        for (int i = 0; i < 8; ++i) {
            state[i] = AD7771_ENABLE;
            gain[i] = channel_gain;
        }

        for (int i = 0; i <= AD7771_REG_SRC_UPDATE; i++)
            spi_int_reg_read(i, &this->cached_reg_val[i]);

        digitalWrite(reset_pin, LOW);
        delay(10);      
        digitalWrite(reset_pin, HIGH);
        delay(10);
    }
    return 0; // Assume success
}

int32_t AD7771::setSpiOperationMode(ad7771_spi_op_mode mode) {

    int32_t ret;
	uint8_t cfg_2;
	uint8_t cfg_3;

	switch (mode) {
	case AD7771_SD_CONV:
		cfg_2 = 0;
		cfg_3 = AD7771_SPI_SLAVE_MODE_EN;
		break;
	case AD7771_SAR_CONV:
		cfg_2 = AD7771_SAR_DIAG_MODE_EN;
		cfg_3 = 0;
		break;
	default:	// AD7771_INT_REG
		cfg_2 = 0;
		cfg_3 = 0;
	}
	ret = spi_int_reg_write_mask(
					    AD7771_REG_GENERAL_USER_CONFIG_2,
					    AD7771_SAR_DIAG_MODE_EN,
					    cfg_2);
	ret |= spi_int_reg_write_mask(
					     AD7771_REG_GENERAL_USER_CONFIG_3,
					     AD7771_SPI_SLAVE_MODE_EN,
					     cfg_3);
	spi_op_mode = mode;

	return ret;
}

ad7771_spi_op_mode AD7771::getSpiOperationMode() {
    return spi_op_mode;
}

int32_t AD7771::setChannelState(ad7771_ch channel, ad7771_state state) {

    int32_t ret;

	ret = spi_int_reg_write_mask(
					    AD7771_REG_CH_DISABLE,
					    AD7771_CH_DISABLE(0x1),
					    AD7771_CH_DISABLE(state));
	this->state[channel] = state;

	return ret;
}

ad7771_state AD7771::getChannelState(ad7771_ch channel){
    return this->state[channel];
}

int32_t AD7771::setGain(ad7771_ch channel, ad7771_gain gain) {

	int32_t ret;

	if (ctrl_mode == AD7771_PIN_CTRL) {
		if (channel <= AD7771_CH3) {
			this->gain[AD7771_CH0] = gain;
			this->gain[AD7771_CH1] = gain;
			this->gain[AD7771_CH2] = gain;
			this->gain[AD7771_CH3] = gain;
		} else {
			this->gain[AD7771_CH4] = gain;
			this->gain[AD7771_CH5] = gain;
			this->gain[AD7771_CH6] = gain;
			this->gain[AD7771_CH7] = gain;
		}
		ret = do_update_mode_pins();
	} else {
		this->gain[channel] = gain;
		ret = spi_int_reg_write_mask(
						    AD7771_REG_CH_CONFIG(channel),
						    AD7771_CH_GAIN(0x3),
						    AD7771_CH_GAIN(gain));
	}

	return ret;
}

ad7771_gain AD7771::getGain(ad7771_ch channel) {
    return this->gain[channel];
}

int32_t AD7771::setDecimationRate(uint16_t int_val, uint16_t dec_val) {
    
    int32_t ret;
	uint8_t msb;
	uint8_t lsb;

	if (ctrl_mode == AD7771_PIN_CTRL) {
		switch (int_val) {
		case 128:
			break;
		case 256:
			break;
		case 512:
			break;
		case 1024:
			break;
		default:
			printf("%s: This setting can't be set in PIN control mode.\n",
			       __func__);
			return FAILURE;
		}
		dec_rate_int = int_val;
		dec_rate_int = dec_val;
		ret = do_update_mode_pins();
	} else {
		msb = (int_val & 0x0F00) >> 8;
		lsb = (int_val & 0x00FF) >> 0;
		ret = spi_int_reg_write(
					       AD7771_REG_SRC_N_MSB,
					       msb);
		ret |= spi_int_reg_write(
						AD7771_REG_SRC_N_LSB,
						lsb);
		dec_val = (dec_val * 65536) / 1000;
		msb = (dec_val & 0xFF00) >> 8;
		lsb = (dec_val & 0x00FF) >> 0;
		ret |= spi_int_reg_write(
						AD7771_REG_SRC_IF_MSB,
						msb);
		ret |= spi_int_reg_write(
						AD7771_REG_SRC_IF_LSB,
						lsb);
		dec_rate_int = int_val;
		dec_rate_int = dec_val;
	}

	return ret;

    dec_rate_int = int_val;
    dec_rate_dec = dec_val;
    return 0;
}

int32_t AD7771::getDecimationRate() {
    return ((dec_rate_int << 16) + dec_rate_dec);
}

int32_t AD7771::setPowerMode(uint8_t pwr_mode) {
	int32_t ret;

	ret = spi_int_reg_write_mask(
					    AD7771_REG_GENERAL_USER_CONFIG_1,
					    AD7771_MOD_POWERMODE,
					    pwr_mode ? AD7771_MOD_POWERMODE : 0);
	this->pwr_mode = pwr_mode;

	return ret;
}

int32_t AD7771::getPowerMode() {
    return this->pwr_mode;
}

int32_t AD7771::setReferenceType(uint8_t type) {
    int32_t ret;

	ret = spi_int_reg_write_mask(
					    AD7771_REG_GENERAL_USER_CONFIG_1,
					    AD7771_PDB_REFOUT_BUF,
					    type ? AD7771_PDB_REFOUT_BUF : 0);
	this->ref_type = type;

	return ret;
}

int32_t AD7771::getReferenceType() {
    return this->ref_type;
}

int32_t AD7771::setDclkDivider(uint8_t divider) {
    int32_t ret;

    if (ctrl_mode == AD7771_PIN_CTRL) {
        digitalWrite(dclk_pins[0], ((divider & 0x01) >> 0));
        digitalWrite(dclk_pins[1], ((divider & 0x02) >> 1));
        digitalWrite(dclk_pins[2], ((divider & 0x04) >> 2));
	} else {
		ret = spi_int_reg_write_mask(
						    AD7771_REG_CH_DISABLE,
						    AD7771_DCLK_CLK_DIV(0x3),
						    AD7771_DCLK_CLK_DIV(divider));
	}
	this->dclk_div = divider;

	return ret;
}

int32_t AD7771::getDclkDivider() {
    return this->dclk_div;
}

int32_t AD7771::setSyncOffset(ad7771_ch channel, uint8_t offset) {
	int32_t ret;

	if (this->ctrl_mode == AD7771_PIN_CTRL) {
		printf("%s: This feature is not available in PIN control mode.\n",
		       __func__);
		return FAILURE;
	}

	ret = spi_int_reg_write(
				       AD7771_REG_CH_SYNC_OFFSET(channel),
				       offset);
	this->sync_offset[channel] = offset;

	return ret;
}

int32_t AD7771::getSyncOffset(ad7771_ch channel) {

    if (this->ctrl_mode == AD7771_PIN_CTRL) {
		printf("%s: This feature is not available in PIN control mode.\n",
		       __func__);
		return FAILURE;
	}

	return this->sync_offset[channel];
}

int32_t AD7771::setOffsetCorrection(ad7771_ch channel, uint32_t offset) {
	int32_t ret;
	uint8_t upper_byte;
	uint8_t mid_byte;
	uint8_t lower_byte;

	if (this->ctrl_mode == AD7771_PIN_CTRL) {
		printf("%s: This feature is not available in PIN control mode.\n",
		       __func__);
		return FAILURE;
	}

	upper_byte = (offset & 0xFF0000) >> 16;
	mid_byte = (offset & 0x00FF00) >> 8;
	lower_byte = (offset & 0x0000FF) >> 0;
	ret = spi_int_reg_write(
				       AD7771_REG_CH_OFFSET_UPPER_BYTE(channel),
				       upper_byte);
	ret |= spi_int_reg_write(
					AD7771_REG_CH_OFFSET_MID_BYTE(channel),
					mid_byte);
	ret |= spi_int_reg_write(
					AD7771_REG_CH_OFFSET_LOWER_BYTE(channel),
					lower_byte);
	this->offset_corr[channel] = offset;

	return ret;
}

int32_t AD7771::getOffsetCorrection(ad7771_ch channel) {
	if (this->ctrl_mode == AD7771_PIN_CTRL) {
		printf("%s: This feature is not available in PIN control mode.\n",
		       __func__);
		return FAILURE;
	}
    return this->offset_corr[channel];
}

int32_t AD7771::setGainCorrection(ad7771_ch channel, uint32_t gain) {

	int32_t ret;
	uint8_t upper_byte;
	uint8_t mid_byte;
	uint8_t lower_byte;

	if (this->ctrl_mode == AD7771_PIN_CTRL) {
		printf("%s: This feature is not available in PIN control mode.\n",
		       __func__);
		return FAILURE;
	}

	gain &= 0xFFFFFF;
	upper_byte = (gain & 0xff0000) >> 16;
	mid_byte = (gain & 0x00ff00) >> 8;
	lower_byte = (gain & 0x0000ff) >> 0;
	ret = spi_int_reg_write(
				       AD7771_REG_CH_GAIN_UPPER_BYTE(channel),
				       upper_byte);
	ret |= spi_int_reg_write(
					AD7771_REG_CH_GAIN_MID_BYTE(channel),
					mid_byte);
	ret |= spi_int_reg_write(
					AD7771_REG_CH_GAIN_LOWER_BYTE(channel),
					lower_byte);
	this->gain_corr[channel] = gain;

	return ret;

    if (channel >= 8) return -1;
    gain_corr[channel] = gain;
    return 0;
}

int32_t AD7771::getGainCorrection(ad7771_ch channel) {
	if (this->ctrl_mode == AD7771_PIN_CTRL) {
		printf("%s: This feature is not available in PIN control mode.\n",
		       __func__);
		return FAILURE;
	}
	return this->gain_corr[channel];
}

int32_t AD7771::setSarConfiguration(ad7771_state state, ad7771_sar_mux mux) {
	int32_t ret;

	ret = spi_int_reg_write_mask(
					    AD7771_REG_GENERAL_USER_CONFIG_1,
					    AD7771_PDB_SAR,
					    (state == AD7771_ENABLE) ?
					    AD7771_PDB_SAR : 0);
	ret |= spi_int_reg_write(
					AD7771_REG_GLOBAL_MUX_CONFIG,
					AD7771_GLOBAL_MUX_CTRL(mux));
	this->sar_state = state;
	this->sar_mux = mux;

	return ret;
}

int32_t AD7771::getSarConfiguration() {
    return ((this->sar_state << 16) + this->sar_mux);
}

int32_t AD7771::doSingleSarConversion(ad7771_sar_mux mux, uint16_t* sar_code) {
	ad7771_spi_op_mode restore_spi_op_mode;
	ad7771_state restore_sar_state;
	int32_t ret;

	restore_spi_op_mode = this->spi_op_mode;
	restore_sar_state = this->sar_state;
	ret = setSarConfiguration(AD7771_ENABLE, mux);
	ret |= setSpiOperationMode(AD7771_SAR_CONV);
	digitalWrite(this->convst_sar, LOW);
	delay(10);	// Acquisition Time = min 500 ns
	digitalWrite(this->convst_sar, HIGH);
	delay(10);	// Conversion Time = max 3.4 us
	spi_sar_read_code(mux, sar_code);
	ret |= setSarConfiguration(restore_sar_state, mux);
	ret |= setSpiOperationMode(restore_spi_op_mode);

	return ret;
}

int32_t AD7771::doSpiSoftReset() {
	uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	int32_t ret;

	/* Keeping the SDI pin high during 64 consecutives clocks generates a
	   software reset */
	ret = spi_write_and_read(buf, 8);

	return ret;
}

int32_t AD7771::spi_write_and_read(uint8_t *data,uint16_t bytes_number){
    //digitalWrite(this->cs, LOW);

    this->spi->beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    this->spi->transfer(data, bytes_number);
    this->spi->endTransaction();

    //digitalWrite(desc->chip_select, HIGH);

    return SUCCESS;
}

/**
 * Compute CRC8 checksum.
 * @param data - The data buffer.
 * @param data_size - The size of the data buffer.
 * @return CRC8 checksum.
 */
uint8_t AD7771::compute_crc8(uint8_t *data, uint8_t data_size)
{
	uint8_t i;
	uint8_t crc = 0;

	while (data_size) {
		for (i = 0x80; i != 0; i >>= 1) {
			if (((crc & 0x80) != 0) != ((*data & i) != 0)) {
				crc <<= 1;
				crc ^= 0x07;
			} else
				crc <<= 1;
		}
		data++;
		data_size--;
	}

	return crc;
}

/**
 * SPI internal register read from device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t AD7771::spi_int_reg_read(uint8_t reg_addr,uint8_t *reg_data)
{
	uint8_t buf[3];
	uint8_t buf_size = 2;
	uint8_t crc;
	int32_t ret;

	buf[0] = 0x80 | (reg_addr & 0x7F);
	buf[1] = 0x00;
	buf[2] = 0x00;
	if (this->spi_crc_en == AD7771_ENABLE)
		buf_size = 3;
	ret = spi_write_and_read(buf, buf_size);

	*reg_data = buf[1];
	if (this->spi_crc_en == AD7771_ENABLE) {
		buf[0] = 0x80 | (reg_addr & 0x7F);
		crc = compute_crc8(&buf[0], 2);
		if (crc != buf[2]) {
			printf("%s: CRC Error.\n", __func__);
			ret = ERR_INVALID_CRC;
		}
	}

	return ret;
}

/**
 * SPI internal register write to device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t AD7771::spi_int_reg_write(uint8_t reg_addr,uint8_t reg_data)
{
	uint8_t buf[3];
	uint8_t buf_size = 2;
	int32_t ret;

	buf[0] = 0x00 | (reg_addr & 0x7F);
	buf[1] = reg_data;
	if (spi_crc_en == AD7771_ENABLE) {
		buf[2] = compute_crc8(&buf[0], 2);
		buf_size = 3;
	}
	ret = spi_write_and_read( buf, buf_size);
	cached_reg_val[reg_addr] = reg_data;

	return ret;
}


/**
 * SPI internal register read from device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t AD7771::spi_int_reg_read_mask(uint8_t reg_addr,uint8_t mask,uint8_t *data)
{
	uint8_t reg_data;
	int32_t ret;

	ret = spi_int_reg_read(reg_addr, &reg_data);
	*data = (reg_data & mask);

	return ret;
}

/**
 * SPI internal register write to device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t AD7771::spi_int_reg_write_mask(uint8_t reg_addr,uint8_t mask,uint8_t data)
{
	uint8_t reg_data;
	int32_t ret;

	reg_data = cached_reg_val[reg_addr];
	reg_data &= ~mask;
	reg_data |= data;
	ret = spi_int_reg_write(reg_addr, reg_data);

	return ret;
}

/**
 * SPI SAR conversion code read.
 * @param dev - The device structure.
 * @param mux_next_conv - The SAR mux input configuration for the next
 *			  conversion.
 *			  Accepted values: AD7771_AUXAINP_AUXAINN
 *					   AD7771_DVBE_AVSSX
 *					   AD7771_REF1P_REF1N
 *					   AD7771_REF2P_REF2N
 *					   AD7771_REF_OUT_AVSSX
 *					   AD7771_VCM_AVSSX
 *					   AD7771_AREG1CAP_AVSSX_ATT
 *					   AD7771_AREG2CAP_AVSSX_ATT
 *					   AD7771_DREGCAP_DGND_ATT
 *					   AD7771_AVDD1A_AVSSX_ATT
 *					   AD7771_AVDD1B_AVSSX_ATT
 *					   AD7771_AVDD2A_AVSSX_ATT
 *					   AD7771_AVDD2B_AVSSX_ATT
 *					   AD7771_IOVDD_DGND_ATT
 *					   AD7771_AVDD4_AVSSX
 *					   AD7771_DGND_AVSS1A_ATT
 *					   AD7771_DGND_AVSS1B_ATT
 *					   AD7771_DGND_AVSSX_ATT
 *					   AD7771_AVDD4_AVSSX_ATT
 *					   AD7771_REF1P_AVSSX
 *					   AD7771_REF2P_AVSSX
 *					   AD7771_AVSSX_AVDD4_ATT
 * @param sar_code - SAR conversion code.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t AD7771::spi_sar_read_code(ad7771_sar_mux mux_next_conv,uint16_t *sar_code)
{
	uint8_t buf[3];
	uint8_t buf_size = 2;
	uint8_t crc;
	int32_t ret;

	buf[0] = 0x00 | (AD7771_REG_GLOBAL_MUX_CONFIG & 0x7F);
	buf[1] = AD7771_GLOBAL_MUX_CTRL(mux_next_conv);
	if (spi_crc_en == AD7771_ENABLE) {
		buf[2] = compute_crc8(&buf[0], 2);
		buf_size = 3;
	}
	ret = spi_write_and_read( buf, buf_size);
	cached_reg_val[AD7771_REG_GLOBAL_MUX_CONFIG] =
		AD7771_GLOBAL_MUX_CTRL(mux_next_conv);
	buf[0] = buf[0] & 0x0F;
	*sar_code = (buf[0] << 8) | buf[1];
	if (spi_crc_en == AD7771_ENABLE) {
		crc = compute_crc8(&buf[0], 2);
		if (crc != buf[2]) {
			printf("%s: CRC Error.\n", __func__);
			ret = FAILURE;
		}
	}

	return ret;
}

/**
 * Update the state of the MODEx pins according to the settings specified in
 * the device structure.
 * @param dev - The device structure.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t AD7771::do_update_mode_pins()
{
	int32_t ret;
	uint8_t option_index;
	uint8_t mode;

	if (!(gain[AD7771_CH0] == gain[AD7771_CH1] &&
	      gain[AD7771_CH1] == gain[AD7771_CH2] &&
	      gain[AD7771_CH2] == gain[AD7771_CH3] &&
	      gain[AD7771_CH3] == AD7771_GAIN_1))
		goto error;

	if (!(gain[AD7771_CH4] == gain[AD7771_CH5] &&
	      gain[AD7771_CH5] == gain[AD7771_CH6] &&
	      gain[AD7771_CH6] == gain[AD7771_CH7]))
		goto error;

	switch (dec_rate_int) {
	case 128:
		option_index = 0;
		break;
	case 256:
		option_index = 4;
		break;
	case 512:
		option_index = 8;
		break;
	case 1024:
		option_index = 12;
		break;
	default:
		goto error;
	}

	if (pwr_mode == AD7771_HIGH_RES)
		if (ref_type == AD7771_EXT_REF)
			mode = pin_mode_options[option_index + 0][gain[AD7771_CH4]];
		else
			mode = pin_mode_options[option_index + 1][gain[AD7771_CH4]];
	else if (ref_type == AD7771_EXT_REF)
		mode = pin_mode_options[option_index + 2][gain[AD7771_CH4]];
	else
		mode = pin_mode_options[option_index + 3][gain[AD7771_CH4]];

	if (mode == 0xFF)
		goto error;

    ret = SUCCESS;
	digitalWrite(mode_pins[0],((mode & 0x01) >> 0));
	digitalWrite(mode_pins[1],((mode & 0x02) >> 1));
	digitalWrite(mode_pins[2],((mode & 0x04) >> 2));
	digitalWrite(mode_pins[3],((mode & 0x08) >> 3));

	/* All the pins that define the AD7771 configuration mode are re-evaluated
	 * every time SYNC_IN pin is pulsed. */
	digitalWrite(sync_in, LOW);
	delay(10);
	digitalWrite(sync_in, HIGH);

	return ret;

error:
	printf("%s: This setting can't be set in PIN control mode.\n",
	       __func__);
	return FAILURE;
}
