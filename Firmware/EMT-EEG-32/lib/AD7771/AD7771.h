#ifndef AD7771_H
#define AD7771_H

#include <Arduino.h>
#include <SPI.h>

#define SUCCESS		0
#define FAILURE		-1
#define ERR_INVALID_CRC -2
#define ERR_INVALID_GPIO -3

#define	SPI_CPHA	0x01
#define	SPI_CPOL	0x02

#define AD7771_MISO 22
#define AD7771_MOSI 21
#define AD7771_SCK  15
#define AD7771_CS   16

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD7771_REG_CH_CONFIG(ch)			(0x00 + (ch))		// Channel Configuration
#define AD7771_REG_CH_DISABLE				0x08				// Disable clocks to ADC channel
#define AD7771_REG_CH_SYNC_OFFSET(ch)		(0x09 + (ch))		// Channel SYNC Offset
#define AD7771_REG_GENERAL_USER_CONFIG_1	0x11				// General User Config 1
#define AD7771_REG_GENERAL_USER_CONFIG_2	0x12				// General User Config 2
#define AD7771_REG_GENERAL_USER_CONFIG_3	0x13				// General User Config 3
#define AD7771_REG_DOUT_FORMAT				0x14				// Data out format
#define AD7771_REG_ADC_MUX_CONFIG			0x15				// Main ADC meter and reference Mux control
#define AD7771_REG_GLOBAL_MUX_CONFIG		0x16				// Global diagnostics mux
#define AD7771_REG_GPIO_CONFIG				0x17				// GPIO config
#define AD7771_REG_GPIO_DATA				0x18				// GPIO Data
#define AD7771_REG_BUFFER_CONFIG_1			0x19				// Buffer Config 1
#define AD7771_REG_BUFFER_CONFIG_2			0x1A				// Buffer Config 2
#define AD7771_REG_CH_OFFSET_UPPER_BYTE(ch)	(0x1C + (ch) * 6)	// Channel offset upper byte
#define AD7771_REG_CH_OFFSET_MID_BYTE(ch)	(0x1D + (ch) * 6)	// Channel offset middle byte
#define AD7771_REG_CH_OFFSET_LOWER_BYTE(ch)	(0x1E + (ch) * 6)	// Channel offset lower byte
#define AD7771_REG_CH_GAIN_UPPER_BYTE(ch)	(0x1F + (ch) * 6)	// Channel gain upper byte
#define AD7771_REG_CH_GAIN_MID_BYTE(ch)		(0x20 + (ch) * 6)	// Channel gain middle byte
#define AD7771_REG_CH_GAIN_LOWER_BYTE(ch)	(0x21 + (ch) * 6)	// Channel gain lower byte
#define AD7771_REG_CH_ERR_REG(ch)			(0x4C + (ch))		// Channel Status Register
#define AD7771_REG_CH0_1_SAT_ERR			0x54				// Channel 0/1 DSP errors
#define AD7771_REG_CH2_3_SAT_ERR			0x55				// Channel 2/3 DSP errors
#define AD7771_REG_CH4_5_SAT_ERR			0x56				// Channel 4/5 DSP errors
#define AD7771_REG_CH6_7_SAT_ERR			0x57				// Channel 6/7 DSP errors
#define AD7771_REG_CHX_ERR_REG_EN			0x58				// Channel 0-7 Error Reg Enable
#define AD7771_REG_GEN_ERR_REG_1			0x59				// General Errors Register 1
#define AD7771_REG_GEN_ERR_REG_1_EN			0x5A				// General Errors Register 1 Enable
#define AD7771_REG_GEN_ERR_REG_2			0x5B				// General Errors Register 2
#define AD7771_REG_GEN_ERR_REG_2_EN			0x5C				// General Errors Register 2 Enable
#define AD7771_REG_STATUS_REG_1				0x5D				// Error Status Register 1
#define AD7771_REG_STATUS_REG_2				0x5E				// Error Status Register 2
#define AD7771_REG_STATUS_REG_3				0x5F				// Error Status Register 3
#define AD7771_REG_SRC_N_MSB				0x60				// Decimation Rate (N) MSB
#define AD7771_REG_SRC_N_LSB				0x61				// Decimation Rate (N) LSB
#define AD7771_REG_SRC_IF_MSB				0x62				// Decimation Rate (IF) MSB
#define AD7771_REG_SRC_IF_LSB				0x63				// Decimation Rate (IF) LSB
#define AD7771_REG_SRC_UPDATE				0x64				// SRC load source and load update

/* AD7771_REG_CHx_CONFIG */
#define AD7771_CH_GAIN(x)					(((x) & 0x3) << 6)
#define AD7771_CH_RX						(1 << 4)

/* AD7771_REG_CH_DISABLE */
#define AD7771_CH_DISABLE(x)				(1 << (x))

/* AD7771_REG_GENERAL_USER_CONFIG_1 */
#define AD7771_ALL_CH_DIS_MCLK_EN			(1 << 7)
#define AD7771_MOD_POWERMODE				(1 << 6)
#define AD7771_PDB_VCM						(1 << 5)
#define AD7771_PDB_REFOUT_BUF				(1 << 4)
#define AD7771_PDB_SAR						(1 << 3)
#define AD7771_PDB_RC_OSC					(1 << 2)
#define AD7771_SOFT_RESET(x)				(((x) & 0x3) << 0)

/* AD7771_REG_GENERAL_USER_CONFIG_2 */
#define AD7771_FILTER_MODE					(1 << 6)
#define AD7771_SAR_DIAG_MODE_EN				(1 << 5)
#define AD7771_SDO_DRIVE_STR(x)				(((x) & 0x3) << 3)
#define AD7771_DOUT_DRIVE_STR(x)			(((x) & 0x3) << 1)
#define AD7771_SPI_SYNC						(1 << 0)

/* AD7771_REG_GENERAL_USER_CONFIG_3 */
#define AD7771_CONVST_DEGLITCH_DIS(x)		(((x) & 0x3) << 6)
#define AD7771_SPI_SLAVE_MODE_EN			(1 << 4)
#define AD7771_CLK_QUAL_DIS					(1 << 0)

/* AD7771_REG_DOUT_FORMAT */
#define AD7771_DOUT_FORMAT(x)				(((x) & 0x3) << 6)
#define AD7771_DOUT_HEADER_FORMAT			(1 << 5)
#define AD7771_DCLK_CLK_DIV(x)				(((x) & 0x3) << 1)

/* AD7771_REG_GLOBAL_MUX_CONFIG */
#define AD7771_GLOBAL_MUX_CTRL(x)			(((x) & 0x1F) << 3)

/* AD7771_REG_BUFFER_CONFIG_1 */
#define AD7771_REF_BUF_POS_EN				(1 << 4)
#define AD7771_REF_BUF_NEG_EN				(1 << 3)

/* AD7771_REG_BUFFER_CONFIG_2 */
#define AD7771_REFBUFP_PREQ					(1 << 7)
#define AD7771_REFBUFN_PREQ					(1 << 6)
#define AD7771_PDB_ALDO1_OVRDRV				(1 << 2)
#define AD7771_PDB_ALDO2_OVRDRV				(1 << 1)
#define AD7771_PDB_DLDO_OVRDRV				(1 << 0)

/* AD7771_REG_GEN_ERR_REG_1_EN */
#define AD7771_MEMMAP_CRC_TEST_EN			(1 << 5)
#define AD7771_ROM_CRC_TEST_EN				(1 << 4)
#define AD7771_SPI_CLK_COUNT_TEST_EN		(1 << 3)
#define AD7771_SPI_INVALID_READ_TEST_EN		(1 << 2)
#define AD7771_SPI_INVALID_WRITE_TEST_EN	(1 << 1)
#define AD7771_SPI_CRC_TEST_EN				(1 << 0)

#define AD7771_CRC8_POLY					0x07

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
typedef enum {
	AD7771_ENABLE,
	AD7771_DISABLE,
}ad7771_state;

typedef enum {
	AD7771_PIN_CTRL,
	AD7771_SPI_CTRL,
} ad7771_ctrl_mode;

typedef enum  {
	AD7771_INT_REG,
	AD7771_SD_CONV,
	AD7771_SAR_CONV,
}ad7771_spi_op_mode;

typedef enum {
	AD7771_CH0,
	AD7771_CH1,
	AD7771_CH2,
	AD7771_CH3,
	AD7771_CH4,
	AD7771_CH5,
	AD7771_CH6,
	AD7771_CH7,
}ad7771_ch;

typedef enum {
	AD7771_AUXAINP_AUXAINN,
	AD7771_DVBE_AVSSX,
	AD7771_REF1P_REF1N,
	AD7771_REF2P_REF2N,
	AD7771_REF_OUT_AVSSX,
	AD7771_VCM_AVSSX,
	AD7771_AREG1CAP_AVSSX_ATT,
	AD7771_AREG2CAP_AVSSX_ATT,
	AD7771_DREGCAP_DGND_ATT,
	AD7771_AVDD1A_AVSSX_ATT,
	AD7771_AVDD1B_AVSSX_ATT,
	AD7771_AVDD2A_AVSSX_ATT,
	AD7771_AVDD2B_AVSSX_ATT,
	AD7771_IOVDD_DGND_ATT,
	AD7771_AVDD4_AVSSX,
	AD7771_DGND_AVSS1A_ATT,
	AD7771_DGND_AVSS1B_ATT,
	AD7771_DGND_AVSSX_ATT,
	AD7771_AVDD4_AVSSX_ATT,
	AD7771_REF1P_AVSSX,
	AD7771_REF2P_AVSSX,
	AD7771_AVSSX_AVDD4_ATT,
}ad7771_sar_mux;

enum {
	AD7771_REF_BUF_ENABLED,
	AD7771_REF_BUF_PRECHARGED,
	AD7771_REF_BUF_DISABLED,
};

enum {
	AD7771_REFX_P,
	AD7771_REFX_N,
};

enum {
	AD7771_HIGH_RES,
	AD7771_LOW_PWR,
};

enum {
	AD7771_EXT_REF,
	AD7771_INT_REF
};


enum {
	AD7771_DCLK_DIV_1,
	AD7771_DCLK_DIV_2,
	AD7771_DCLK_DIV_4,
	AD7771_DCLK_DIV_8,
	AD7771_DCLK_DIV_16,
	AD7771_DCLK_DIV_32,
	AD7771_DCLK_DIV_64,
	AD7771_DCLK_DIV_128,
};

typedef enum {
	AD7771_GAIN_1,
	AD7771_GAIN_2,
	AD7771_GAIN_4,
	AD7771_GAIN_8,
}ad7771_gain;

enum spi_mode {
	SPI_MODE_0 = (0 | 0),
	SPI_MODE_1 = (0 | SPI_CPHA),
	SPI_MODE_2 = (SPI_CPOL | 0),
	SPI_MODE_3 = (SPI_CPOL | SPI_CPHA)
};

struct spi_init_param {
	uint32_t	    max_speed_hz;
	uint8_t		    chip_select;
	enum spi_mode	mode;
	void		    *extra;
};

struct spi_desc {
	uint32_t        max_speed_hz;
	uint8_t	        chip_select;
	enum spi_mode	mode;
	void	    	*extra;
};

class AD7771 {
public:
    // Constructor and Destructor
    AD7771(SPIClass* spi, uint8_t reset_pin);
    ~AD7771();

    // Initialization
    int32_t begin(
        uint8_t reference,
        uint8_t resolution_mode,
        uint8_t clk_div,
        uint8_t filter,
        ad7771_gain channel_gain
    );

    // Device control methods
    int32_t setSpiOperationMode(ad7771_spi_op_mode mode);
    ad7771_spi_op_mode getSpiOperationMode();

    int32_t setChannelState(ad7771_ch channel, ad7771_state state);
    ad7771_state getChannelState(ad7771_ch channel);

    int32_t setGain(ad7771_ch channel, ad7771_gain gain);
    ad7771_gain getGain(ad7771_ch channel);

    int32_t setDecimationRate(uint16_t int_val, uint16_t dec_val);
    int32_t getDecimationRate();

    int32_t setPowerMode(uint8_t mode);
    int32_t getPowerMode();

    int32_t setReferenceType(uint8_t type);
    int32_t getReferenceType();

    int32_t	setDclkDivider(uint8_t divider);
    int32_t getDclkDivider();

    int32_t setSyncOffset(ad7771_ch channel, uint8_t offset);
    int32_t getSyncOffset(ad7771_ch channel);

    int32_t setOffsetCorrection(ad7771_ch channel, uint32_t offset);
    int32_t getOffsetCorrection(ad7771_ch channel);

    int32_t setGainCorrection(ad7771_ch channel, uint32_t gain);
    int32_t getGainCorrection(ad7771_ch channel);

    int32_t setSarConfiguration(ad7771_state state, ad7771_sar_mux mux);
    int32_t getSarConfiguration();

    int32_t doSingleSarConversion(ad7771_sar_mux mux, uint16_t* sar_code);

    int32_t doSpiSoftReset();

private:
    bool spi_init = false;
    SPIClass* spi;
    uint8_t reset_pin;
    uint8_t mode_pins[4];
    uint8_t dclk_pins[3];
    uint8_t sync_in;
    uint8_t convst_sar;

	/* Device Settings */
    uint8_t     			ctrl_mode,dclk_div,ref_buf_op_mode[2],sar_mux,sinc5_state,cached_reg_val[0x65];
	ad7771_state			sar_state;
    uint8_t     			spi_crc_en;
    ad7771_spi_op_mode		spi_op_mode;
    ad7771_state			state[8];
    ad7771_gain     		gain[8];
    uint16_t    			dec_rate_int;
    uint16_t    			dec_rate_dec;
    uint8_t     			ref_type;
    uint8_t     			pwr_mode;
    uint8_t     			sync_offset[8];
    uint32_t    			offset_corr[8];
    uint32_t    			gain_corr[8];

	/* Private methods */
	int32_t spi_write_and_read(uint8_t *data,uint16_t bytes_number);
	uint8_t compute_crc8(uint8_t *data,uint8_t data_size);
	int32_t spi_int_reg_read(uint8_t reg_addr, uint8_t *reg_data);
	int32_t spi_int_reg_write(uint8_t reg_addr,uint8_t reg_data);
	int32_t spi_int_reg_read_mask(uint8_t reg_addr,uint8_t mask,uint8_t *data);
	int32_t spi_int_reg_write_mask(uint8_t reg_addr,uint8_t mask,uint8_t data);
	int32_t spi_sar_read_code(ad7771_sar_mux mux_next_conv,uint16_t *sar_code);
	int32_t do_update_mode_pins();
};


#endif // AD7771_H
