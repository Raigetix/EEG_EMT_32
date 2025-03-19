#include "AD777x.h"

AD777x::AD777x(const Config& cfg) : config(cfg) {
	if (cfg.ctrl_mode == AD777x::CtrlMode::AD777x_SPI_CTRL) {
		// Configurar SPI
		this->p_spi = config.spi_host;
		pinMode(this->config.cs_pin, OUTPUT);
		digitalWrite(this->config.cs_pin, HIGH);  // Desactivamos el dispositivo SPI
		
		//p_spi->begin(config.sclk_pin, config.miso_pin, config.mosi_pin, config.cs_pin);
	}
  	else if(cfg.ctrl_mode == AD777x::CtrlMode::AD777x_PIN_CTRL){
		// Configurar pines GPIO
		gpio_mode0 = static_cast<gpio_num_t>(config.mode0_pin);
		gpio_mode1 = static_cast<gpio_num_t>(config.mode1_pin);
		gpio_mode2 = static_cast<gpio_num_t>(config.mode2_pin);
		gpio_mode3 = static_cast<gpio_num_t>(config.mode3_pin);
		gpio_dclk0 = static_cast<gpio_num_t>(config.dclk0_pin);
		gpio_dclk1 = static_cast<gpio_num_t>(config.dclk1_pin);
		gpio_dclk2 = static_cast<gpio_num_t>(config.dclk2_pin);
		gpio_sync_in = static_cast<gpio_num_t>(config.sync_in_pin);
		gpio_convst_sar = static_cast<gpio_num_t>(config.convst_sar_pin);
	}
	gpio_reset = static_cast<gpio_num_t>(config.reset_pin);
}

AD777x::~AD777x() {
}

bool AD777x::initialize() {
	reset_device();
	setup_defaults();
	return true;
}

void AD777x::reset_device() {
	gpio_set_level(gpio_reset, 0);
	vTaskDelay(pdMS_TO_TICKS(10));
	gpio_set_level(gpio_reset, 1);
	vTaskDelay(pdMS_TO_TICKS(100));
}

void AD777x::setup_defaults() {
	// Configurar referencia
	set_reference_type(config.ref_type);
	
	// Configurar ganancia para todos los canales
	for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
		set_gain(static_cast<Channel>(ch), config.gain[ch]);
	}
	
	// Configurar decimación
	set_decimation_rate(config.dec_rate_int, config.dec_rate_dec);
	
	// Configurar modo de potencia
	set_power_mode(config.pwr_mode);
	
	// Configurar divisor de DCLK
	set_dclk_div(config.dclk_div);
	
	// Configurar filtro SINC5
	set_sinc5_filter_state(config.sinc5_state);
}

void AD777x::write_register(uint8_t reg, uint8_t value) {
/* 	spi_transaction_t t = {};
	t.flags = SPI_TRANS_USE_RXDATA;
	t.length = 16;
	t.tx_buffer = &reg;
	t.rx_buffer = nullptr;
	
	uint8_t tx_data[2] = {reg, value};
	t.tx_buffer = tx_data;
	
	spi_device_polling_transmit(spi, &t); */
	digitalWrite(this->config.cs_pin, LOW);  	// Activamos el dispositivo SPI
    this->p_spi->transfer(reg);          		// Enviamos el registro
    this->p_spi->transfer(value);        		// Enviamos el valor
    digitalWrite(this->config.cs_pin, HIGH);	// Desactivamos el dispositivo SPI
}

uint8_t AD777x::read_register(uint8_t reg) {
/* 	spi_transaction_t t = {};
	uint8_t rx_data[2];
	t.length = 16;
	t.rx_buffer = rx_data;
	t.tx_buffer = &reg;
	
	spi_device_polling_transmit(spi, &t);
	return rx_data[1]; */
    digitalWrite(this->config.cs_pin, LOW);			// Activamos el dispositivo SPI
    this->p_spi->transfer(reg);						// Enviamos el registro
    uint8_t value = this->p_spi->transfer(0x00);	// Leemos el valor
    digitalWrite(this->config.cs_pin, HIGH);		// Desactivamos el dispositivo SPI
    return value;
}

void AD777x::wait_for_data_ready() {
	while (gpio_get_level(static_cast<gpio_num_t>(config.drdy_pin))) {
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

void AD777x::read_all_channels() {

	uint8_t buff[32] = { 0 };
	uint8_t byte_index;
	uint8_t ch_id;
	int32_t ret;
	int32_t ret;

	/* Read the SPI data */
	ret = spi_write_and_read(buff,sizeof(buff));
	if (ret) {
		return;
	}

	/* Decode the bytes into ADC Raw data */
	for (ch_id = 0; ch_id < NUM_CHANNELS; ch_id++) {
		byte_index = (DATA_BYTES * ch_id) + (ch_id + 1);
		this->ch_values[ch_id] = (buff[byte_index] << 16) | (buff[byte_index + 1] << 8) | (buff[byte_index + 2]);
	}
}

uint32_t AD777x::read_channel(Channel ch){
	return this->ch_values[(int)ch];
}

void AD777x::set_gain(Channel channel, Gain gain) {
/* 	uint8_t reg = static_cast<uint8_t>(channel) + 0x00; // Registro de configuración del canal
	uint8_t value = static_cast<uint8_t>(gain) << 6;    // Ganancia en bits 6-7 */

	if (this->config.ctrl_mode == CtrlMode::AD777x_PIN_CTRL) {
		if (channel <= Channel::AD777x_CH3) {
			this->config.gain[(int)Channel::AD777x_CH0] = gain;
			this->config.gain[(int)Channel::AD777x_CH1] = gain;
			this->config.gain[(int)Channel::AD777x_CH2] = gain;
			this->config.gain[(int)Channel::AD777x_CH3] = gain;
		} else {
			this->config.gain[(int)Channel::AD777x_CH4] = gain;
			this->config.gain[(int)Channel::AD777x_CH5] = gain;
			this->config.gain[(int)Channel::AD777x_CH6] = gain;
			this->config.gain[(int)Channel::AD777x_CH7] = gain;
		}
		//ret = ad7779_do_update_mode_pins(dev);
	} else {
		this->config.gain[(int)channel] = gain;
		spi_int_reg_write_mask(AD777x_REG_CH_CONFIG((int)channel),AD777x_CH_GAIN(0x3),AD777x_CH_GAIN((int)gain));
	}

	/* write_register(reg, value); */
}

void AD777x::set_decimation_rate(uint16_t int_val, uint16_t dec_val) {
	int32_t ret;
	uint8_t msb;
	uint8_t lsb;
	if (this->config.ctrl_mode == CtrlMode::AD777x_PIN_CTRL) {
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
			//return -1;
		}
		this->config.dec_rate_int = int_val;
		this->config.dec_rate_int = dec_val;
		//ret = ad7779_do_update_mode_pins(dev);
	} else {
		msb = (int_val & 0x0F00) >> 8;
		lsb = (int_val & 0x00FF) >> 0;
		ret = spi_int_reg_write(AD777x_REG_SRC_N_MSB,msb);
		ret |= spi_int_reg_write(AD777x_REG_SRC_N_LSB,lsb);
		dec_val = (dec_val * 65536) / 1000;
		msb = (dec_val & 0xFF00) >> 8;
		lsb = (dec_val & 0x00FF) >> 0;
		ret |= spi_int_reg_write(AD777x_REG_SRC_IF_MSB,msb);
		ret |= spi_int_reg_write(AD777x_REG_SRC_IF_LSB,lsb);
		this->config.dec_rate_int = int_val;
		this->config.dec_rate_int = dec_val;
	}

/* 	write_register(0x60, (int_val >> 8) & 0xFF); // MSB
	write_register(0x61, int_val & 0xFF);        // LSB
	write_register(0x62, (dec_val >> 8) & 0xFF); // MSB
	write_register(0x63, dec_val & 0xFF);        // LSB */
}

void AD777x::set_power_mode(PowerMode mode) {
	int32_t ret = spi_int_reg_write_mask(AD777x_REG_GENERAL_USER_CONFIG_1,AD777x_MOD_POWERMODE, (int)mode ? AD777x_MOD_POWERMODE : 0);
	this->config.pwr_mode = mode;

	//uint8_t value = (mode == PowerMode::AD777x_HIGH_RES) ? 0x40 : 0x00; // Bit 6
	//write_register(0x11, value);
}

void AD777x::set_reference_type(RefType ref_type) {

	int32_t ret;
	if (ref_type == RefType::AD777x_INT_REF) 
		ret = spi_int_reg_write_mask(AD777x_REG_GENERAL_USER_CONFIG_1,AD777x_PDB_REFOUT_BUF,AD777x_PDB_REFOUT_BUF);
	else
		ret = spi_int_reg_write_mask(AD777x_REG_GENERAL_USER_CONFIG_1, AD777x_PDB_REFOUT_BUF, 0);
	
 	if (ret)
		return;

	ret = spi_int_reg_write_mask(AD777x_REG_ADC_MUX_CONFIG, AD777x_REF_MUX_CTRL(0x3), AD777x_REF_MUX_CTRL((int)ref_type));
	if (ret)
		return;

	this->config.ref_type = ref_type;

/* 	uint8_t value = static_cast<uint8_t>(ref_type) << 6; // Bits 6-7
	write_register(0x15, value); */
}

void AD777x::set_dclk_div(DclkDiv div) {
	int32_t ret;
	if (this->config.ctrl_mode == CtrlMode::AD777x_PIN_CTRL) {
/* 		ret = no_os_gpio_set_value(dev->gpio_dclk0,
					   ((div & 0x01) >> 0));
		ret |= no_os_gpio_set_value(dev->gpio_dclk1,
					    ((div & 0x02) >> 1));
		ret |= no_os_gpio_set_value(dev->gpio_dclk2,
					    ((div & 0x04) >> 2)); */
	} else {
		ret = spi_int_reg_write_mask(AD777x_REG_DOUT_FORMAT,AD777x_DCLK_CLK_DIV(0x7),AD777x_DCLK_CLK_DIV((int)div));
	}
	this->config.dclk_div = div;


/* 	uint8_t value = static_cast<uint8_t>(div) << 1; // Bits 1-3
	write_register(0x14, value); */
}

void AD777x::set_sinc5_filter_state(Sinc5State state) {
	int32_t ret;
	if (this->config.ctrl_mode == CtrlMode::AD777x_PIN_CTRL) {
		printf("%s: This feature is not available in PIN control mode.\n",
		       __func__);
		return;
	}

	ret = spi_int_reg_write_mask(AD777x_REG_GENERAL_USER_CONFIG_2,AD777x_FILTER_MODE,(state == Sinc5State::AD777x_ENABLE) ?AD777x_FILTER_MODE : 0);
	this->config.sinc5_state = state;

	uint8_t value = (state == Sinc5State::AD777x_ENABLE) ? 0x40 : 0x00; // Bit 6
	write_register(0x12, value);
}

// Métodos SAR ADC
int32_t AD777x::set_sar_configuration(State state, SarMux mux) {

	int32_t ret;

	ret = spi_int_reg_write_mask(AD777x_REG_GENERAL_USER_CONFIG_1,AD777x_PDB_SAR,(state == State::AD777x_ENABLE) ? AD777x_PDB_SAR : 0);
	ret |= spi_int_reg_write(AD777x_REG_GLOBAL_MUX_CONFIG,AD777x_GLOBAL_MUX_CTRL((int)mux));
 	this->config.sar_state = state;
	this->config.sar_mux = mux;

/*     uint8_t value = (state == State::AD777x_ENABLE) ? 0x08 : 0x00; // Bit 3
    value |= static_cast<uint8_t>(mux) << 3; // Bits 4-7
    write_register(0x16, value);
    return 0; */
}

int32_t AD777x::set_spi_operation_mode(SpiOpMode mode) {

	int32_t ret;
	uint8_t cfg_2;
	uint8_t cfg_3;

	switch (mode) {
	case SpiOpMode::AD777x_SD_CONV:
		cfg_2 = 0;
		cfg_3 = AD777x_SPI_SLAVE_MODE_EN;
		break;
	case SpiOpMode::AD777x_SAR_CONV:
		cfg_2 = AD777x_SAR_DIAG_MODE_EN;
		cfg_3 = 0;
		break;
	default:	// AD7779_INT_REG
		cfg_2 = 0;
		cfg_3 = 0;
	}
	ret = spi_int_reg_write_mask(AD777x_REG_GENERAL_USER_CONFIG_2,AD777x_SAR_DIAG_MODE_EN,cfg_2);
	ret |= spi_int_reg_write_mask(AD777x_REG_GENERAL_USER_CONFIG_3,AD777x_SPI_SLAVE_MODE_EN,cfg_3);
	this->config.spi_op_mode = mode;

    uint8_t value = static_cast<uint8_t>(mode);
    write_register(0x13, value);
    return 0;
}

int32_t AD777x::spi_sar_read_code(SarMux mux_next_conv, uint16_t* sar_code) {
/*     uint8_t buf[3];
    buf[0] = 0x00 | (0x16 & 0x7F); // Registro GLOBAL_MUX_CONFIG
    buf[1] = static_cast<uint8_t>(mux_next_conv) << 3; // Configura el mux
    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = buf;
    t.rx_buffer = buf;
    spi_device_polling_transmit(spi, &t);
    *sar_code = (buf[1] << 8) | buf[2];
    return 0; */

	uint8_t buf[3]; // Buffer para almacenar los datos enviados y recibidos

    // Configurar el registro GLOBAL_MUX_CONFIG y el mux
    buf[0] = 0x00 | (0x16 & 0x7F); // Dirección del registro GLOBAL_MUX_CONFIG
    buf[1] = static_cast<uint8_t>(mux_next_conv) << 3; // Configurar el mux

    // Activar el dispositivo SPI (poner el pin CS en LOW)
    digitalWrite(this->config.cs_pin, LOW);

    // Enviar y recibir datos
    buf[1] = this->p_spi->transfer(buf[0]); // Enviar la dirección del registro y leer el primer byte
    buf[2] = this->p_spi->transfer(buf[1]); // Enviar el valor del mux y leer el segundo byte

    // Desactivar el dispositivo SPI (poner el pin CS en HIGH)
    digitalWrite(this->config.cs_pin, HIGH);

    // Decodificar el código SAR
    *sar_code = (buf[1] << 8) | buf[2];

    return 0; // Retornar éxito
}

int32_t AD777x::do_single_sar_conversion(SarMux mux, uint16_t* sar_code) {
    SpiOpMode restore_spi_op_mode = SpiOpMode::AD777x_INT_REG;
    State restore_sar_state = State::AD777x_DISABLE;

    // Guardar el modo SPI actual
    restore_spi_op_mode = static_cast<SpiOpMode>(read_register(0x13) & 0x03);

    // Configurar el SAR ADC
    set_sar_configuration(State::AD777x_ENABLE, mux);
    set_spi_operation_mode(SpiOpMode::AD777x_SAR_CONV);

    // Iniciar la conversión
    digitalWrite(this->config.convst_sar_pin, LOW);
    delay(1); // Tiempo de adquisición
    digitalWrite(this->config.convst_sar_pin, HIGH);
    delay(1); // Tiempo de conversión

    // Leer el resultado
    spi_sar_read_code(mux, sar_code);

    // Restaurar el modo SPI y el estado del SAR ADC
    set_spi_operation_mode(restore_spi_op_mode);
    set_sar_configuration(restore_sar_state, mux);

    return 0;
}