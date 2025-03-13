#ifndef AD777x_H
#define AD777x_H

#include <vector>
#include <Arduino.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <SPI.h>

#define SUCCESS		0
#define FAILURE		-1
#define ERR_INVALID_CRC -2
#define ERR_INVALID_GPIO -3

#define	SPI_CPHA	0x01
#define	SPI_CPOL	0x02

#define AD777x_MISO 22
#define AD777x_MOSI 21
#define AD777x_SCK  15
#define AD777x_CS   16

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD777x_REG_CH_CONFIG(ch)			(0x00 + (ch))		// Channel Configuration
#define AD777x_REG_CH_DISABLE				0x08				// Disable clocks to ADC channel
#define AD777x_REG_CH_SYNC_OFFSET(ch)		(0x09 + (ch))		// Channel SYNC Offset
#define AD777x_REG_GENERAL_USER_CONFIG_1	0x11				// General User Config 1
#define AD777x_REG_GENERAL_USER_CONFIG_2	0x12				// General User Config 2
#define AD777x_REG_GENERAL_USER_CONFIG_3	0x13				// General User Config 3
#define AD777x_REG_DOUT_FORMAT				0x14				// Data out format
#define AD777x_REG_ADC_MUX_CONFIG			0x15				// Main ADC meter and reference Mux control
#define AD777x_REG_GLOBAL_MUX_CONFIG		0x16				// Global diagnostics mux
#define AD777x_REG_GPIO_CONFIG				0x17				// GPIO config
#define AD777x_REG_GPIO_DATA				0x18				// GPIO Data
#define AD777x_REG_BUFFER_CONFIG_1			0x19				// Buffer Config 1
#define AD777x_REG_BUFFER_CONFIG_2			0x1A				// Buffer Config 2
#define AD777x_REG_CH_OFFSET_UPPER_BYTE(ch)	(0x1C + (ch) * 6)	// Channel offset upper byte
#define AD777x_REG_CH_OFFSET_MID_BYTE(ch)	(0x1D + (ch) * 6)	// Channel offset middle byte
#define AD777x_REG_CH_OFFSET_LOWER_BYTE(ch)	(0x1E + (ch) * 6)	// Channel offset lower byte
#define AD777x_REG_CH_GAIN_UPPER_BYTE(ch)	(0x1F + (ch) * 6)	// Channel gain upper byte
#define AD777x_REG_CH_GAIN_MID_BYTE(ch)		(0x20 + (ch) * 6)	// Channel gain middle byte
#define AD777x_REG_CH_GAIN_LOWER_BYTE(ch)	(0x21 + (ch) * 6)	// Channel gain lower byte
#define AD777x_REG_CH_ERR_REG(ch)			(0x4C + (ch))		// Channel Status Register
#define AD777x_REG_CH0_1_SAT_ERR			0x54				// Channel 0/1 DSP errors
#define AD777x_REG_CH2_3_SAT_ERR			0x55				// Channel 2/3 DSP errors
#define AD777x_REG_CH4_5_SAT_ERR			0x56				// Channel 4/5 DSP errors
#define AD777x_REG_CH6_7_SAT_ERR			0x57				// Channel 6/7 DSP errors
#define AD777x_REG_CHX_ERR_REG_EN			0x58				// Channel 0-7 Error Reg Enable
#define AD777x_REG_GEN_ERR_REG_1			0x59				// General Errors Register 1
#define AD777x_REG_GEN_ERR_REG_1_EN			0x5A				// General Errors Register 1 Enable
#define AD777x_REG_GEN_ERR_REG_2			0x5B				// General Errors Register 2
#define AD777x_REG_GEN_ERR_REG_2_EN			0x5C				// General Errors Register 2 Enable
#define AD777x_REG_STATUS_REG_1				0x5D				// Error Status Register 1
#define AD777x_REG_STATUS_REG_2				0x5E				// Error Status Register 2
#define AD777x_REG_STATUS_REG_3				0x5F				// Error Status Register 3
#define AD777x_REG_SRC_N_MSB				0x60				// Decimation Rate (N) MSB
#define AD777x_REG_SRC_N_LSB				0x61				// Decimation Rate (N) LSB
#define AD777x_REG_SRC_IF_MSB				0x62				// Decimation Rate (IF) MSB
#define AD777x_REG_SRC_IF_LSB				0x63				// Decimation Rate (IF) LSB
#define AD777x_REG_SRC_UPDATE				0x64				// SRC load source and load update

/* AD777x_REG_CHx_CONFIG */
#define AD777x_CH_GAIN(x)					(((x) & 0x3) << 6)
#define AD777x_CH_RX						(1 << 4)

/* AD777x_REG_CH_DISABLE */
#define AD777x_CH_DISABLE(x)				(1 << (x))

/* AD777x_REG_GENERAL_USER_CONFIG_1 */
#define AD777x_ALL_CH_DIS_MCLK_EN			(1 << 7)
#define AD777x_MOD_POWERMODE				(1 << 6)
#define AD777x_PDB_VCM						(1 << 5)
#define AD777x_PDB_REFOUT_BUF				(1 << 4)
#define AD777x_PDB_SAR						(1 << 3)
#define AD777x_PDB_RC_OSC					(1 << 2)
#define AD777x_SOFT_RESET(x)				(((x) & 0x3) << 0)

/* AD777x_REG_GENERAL_USER_CONFIG_2 */
#define AD777x_FILTER_MODE					(1 << 6)
#define AD777x_SAR_DIAG_MODE_EN				(1 << 5)
#define AD777x_SDO_DRIVE_STR(x)				(((x) & 0x3) << 3)
#define AD777x_DOUT_DRIVE_STR(x)			(((x) & 0x3) << 1)
#define AD777x_SPI_SYNC						(1 << 0)

/* AD777x_REG_GENERAL_USER_CONFIG_3 */
#define AD777x_CONVST_DEGLITCH_DIS(x)		(((x) & 0x3) << 6)
#define AD777x_SPI_SLAVE_MODE_EN			(1 << 4)
#define AD777x_CLK_QUAL_DIS					(1 << 0)

/* AD777x_REG_DOUT_FORMAT */
#define AD777x_DOUT_FORMAT(x)				(((x) & 0x3) << 6)
#define AD777x_DOUT_HEADER_FORMAT			(1 << 5)
#define AD777x_DCLK_CLK_DIV(x)				(((x) & 0x3) << 1)

/* AD777x_REG_GLOBAL_MUX_CONFIG */
#define AD777x_GLOBAL_MUX_CTRL(x)			(((x) & 0x1F) << 3)

/* AD777x_REG_BUFFER_CONFIG_1 */
#define AD777x_REF_BUF_POS_EN				(1 << 4)
#define AD777x_REF_BUF_NEG_EN				(1 << 3)

/* AD777x_REG_BUFFER_CONFIG_2 */
#define AD777x_REFBUFP_PREQ					(1 << 7)
#define AD777x_REFBUFN_PREQ					(1 << 6)
#define AD777x_PDB_ALDO1_OVRDRV				(1 << 2)
#define AD777x_PDB_ALDO2_OVRDRV				(1 << 1)
#define AD777x_PDB_DLDO_OVRDRV				(1 << 0)

/* AD777x_REG_GEN_ERR_REG_1_EN */
#define AD777x_MEMMAP_CRC_TEST_EN			(1 << 5)
#define AD777x_ROM_CRC_TEST_EN				(1 << 4)
#define AD777x_SPI_CLK_COUNT_TEST_EN		(1 << 3)
#define AD777x_SPI_INVALID_READ_TEST_EN		(1 << 2)
#define AD777x_SPI_INVALID_WRITE_TEST_EN	(1 << 1)
#define AD777x_SPI_CRC_TEST_EN				(1 << 0)

#define AD777x_CRC8_POLY					0x07

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
class AD777x {
	public:
		// Enums y defines basados en ad7779.h
		enum class CtrlMode {
			AD777x_PIN_CTRL,
			AD777x_SPI_CTRL
		};
	
		enum class SpiOpMode {
			AD777x_INT_REG,
			AD777x_SD_CONV,
			AD777x_SAR_CONV
		};
	
		enum class Channel {
			AD777x_CH0,
			AD777x_CH1,
			AD777x_CH2,
			AD777x_CH3,
			AD777x_CH4,
			AD777x_CH5,
			AD777x_CH6,
			AD777x_CH7
		};
	
		enum class State {
			AD777x_ENABLE,
			AD777x_DISABLE
		};
	
		enum class Gain {
			AD777x_GAIN_1,
			AD777x_GAIN_2,
			AD777x_GAIN_4,
			AD777x_GAIN_8
		};
	
		enum class PowerMode {
			AD777x_LOW_PWR,
			AD777x_HIGH_RES
		};
	
		enum class RefType {
			AD777x_EXT_REF,
			AD777x_INT_REF,
			AD777x_EXT_SUPPLY,
			AD777x_EXT_REF_INV
		};
	
		enum class DclkDiv {
			AD777x_DIV_1,
			AD777x_DIV_2,
			AD777x_DIV_4,
			AD777x_DIV_8,
			AD777x_DIV_16,
			AD777x_DIV_32,
			AD777x_DIV_64,
			AD777x_DIV_128
		};
	
		enum class Sinc5State {
			AD777x_ENABLE,
			AD777x_DISABLE
		};

		enum class SarMux{
			AD777x_AUXAINP_AUXAINN,
			AD777x_DVBE_AVSSX,
			AD777x_REF1P_REF1N,
			AD777x_REF2P_REF2N,
			AD777x_REF_OUT_AVSSX,
			AD777x_VCM_AVSSX,
			AD777x_AREG1CAP_AVSSX_ATT,
			AD777x_AREG2CAP_AVSSX_ATT,
			AD777x_DREGCAP_DGND_ATT,
			AD777x_AVDD1A_AVSSX_ATT,
			AD777x_AVDD1B_AVSSX_ATT,
			AD777x_AVDD2A_AVSSX_ATT,
			AD777x_AVDD2B_AVSSX_ATT,
			AD777x_IOVDD_DGND_ATT,
			AD777x_AVDD4_AVSSX,
			AD777x_DGND_AVSS1A_ATT,
			AD777x_DGND_AVSS1B_ATT,
			AD777x_DGND_AVSSX_ATT,
			AD777x_AVDD4_AVSSX_ATT,
			AD777x_REF1P_AVSSX,
			AD777x_REF2P_AVSSX,
			AD777x_AVSSX_AVDD4_ATT,
		};

	
		// Configuración básica del dispositivo
		struct Config {
			SPIClass* spi_host;
			int sclk_pin;
			int miso_pin;
			int mosi_pin;
			int cs_pin;
			int drdy_pin;
			int reset_pin;
			int mode0_pin;
			int mode1_pin;
			int mode2_pin;
			int mode3_pin;
			int dclk0_pin;
			int dclk1_pin;
			int dclk2_pin;
			int sync_in_pin;
			int convst_sar_pin;
			CtrlMode ctrl_mode;
			State spi_crc_en;
			State state[8];
			Gain gain[8];
			uint16_t dec_rate_int;
			uint16_t dec_rate_dec;
			RefType ref_type;
			PowerMode pwr_mode;
			DclkDiv dclk_div;
			uint8_t sync_offset[8];
			uint32_t offset_corr[8];
			uint32_t gain_corr[8];
			Sinc5State sinc5_state;
		};
	
		AD777x(const Config& config);
		~AD777x();
		
		bool initialize();
		void read_all_channels();
		uint32_t read_channel(Channel ch);
		void set_gain(Channel channel, Gain gain);
		void set_decimation_rate(uint16_t int_val, uint16_t dec_val);
		void set_power_mode(PowerMode mode);
		void set_reference_type(RefType ref_type);
		void set_dclk_div(DclkDiv div);
		void set_sinc5_filter_state(Sinc5State state);
		// Métodos SAR ADC
		int32_t do_single_sar_conversion(SarMux mux, uint16_t* sar_code);
		int32_t set_sar_configuration(State state, SarMux mux);
		int32_t set_spi_operation_mode(SpiOpMode mode);
		int32_t spi_sar_read_code(SarMux mux_next_conv, uint16_t* sar_code);

	
	private:
		Config config;
		SPIClass* p_spi;
		gpio_num_t gpio_reset;
		gpio_num_t gpio_mode0;
		gpio_num_t gpio_mode1;
		gpio_num_t gpio_mode2;
		gpio_num_t gpio_mode3;
		gpio_num_t gpio_dclk0;
		gpio_num_t gpio_dclk1;
		gpio_num_t gpio_dclk2;
		gpio_num_t gpio_sync_in;
		gpio_num_t gpio_convst_sar;

		uint32_t ch_values[8];
		
		void write_register(uint8_t reg, uint8_t value);
		uint8_t read_register(uint8_t reg);
		void setup_defaults();
		void reset_device();
		void wait_for_data_ready();
		
		// Constantes del AD777x
		static constexpr uint8_t NUM_CHANNELS = 8;
		static constexpr uint8_t DATA_BYTES = 3;
	};
	



#endif // AD777x_H
