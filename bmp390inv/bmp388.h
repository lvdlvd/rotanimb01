#pragma once

// 388 and 390 are almost identical

enum {
	// Barometer registers
	BMP388_CHIP_ID		   = 0x0,
	BMP388_CHIP_ID_VAL	   = 0x50,
	BMP390_CHIP_ID_VAL	   = 0x60,	// deeper fifo and an extra bit in INT_CTL
	BMP390_REV_ID		   = 0x1,	// the BM388 doesn't have this
	BMP388_ERR_REG		   = 0x2,
	BMP388_STATUS		   = 0x3,
	BMP388_STATUS_DRDY_CMD = 0x10,
	BMP388_STATUS_DRDY_P   = 0x20,
	BMP388_STATUS_DRDY_T   = 0x40,

	BMP388_P_XLSB		 = 0x4,
	BMP388_P_LSB		 = 0x5,
	BMP388_P_MSB		 = 0x6,
	BMP388_T_XLSB		 = 0x7,
	BMP388_T_LSB		 = 0x8,
	BMP388_T_MSB		 = 0x9,
	BMP388_SENSORTIME_0	 = 0x0C,
	BMP388_SENSORTIME_1	 = 0x0D,
	BMP388_SENSORTIME_2	 = 0x0E,
	BMP388_SENSORTIME_3	 = 0x0F,  // reserved on the BM390, only 24 bits
	BMP388_EVENT		 = 0x10,
	BMP388_INT_STATUS	 = 0x11,
	BMP388_FIFO_LENGTH_0 = 0x12,
	BMP388_FIFO_LENGTH_1 = 0x13,
	BMP388_FIFO_DATA	 = 0x14,
	BMP388_FIFO_WTM_0	 = 0x15,
	BMP388_FIFO_WTM_1	 = 0x16,
	BMP388_FIFO_CONFIG_1 = 0x17,
	BMP388_FIFO_CONFIG_2 = 0x18,
	BMP388_INT_CTRL		 = 0x19,
	BMP388_IF_CONF		 = 0x1A,
	BMP388_PWR_CTRL		 = 0x1B,
	BMP388_OSR			 = 0x1C,
	BMP388_ODR			 = 0x1D,
	BMP388_CONFIG		 = 0x1F,
	BMP388_NVM_PARAMS	 = 0x31,  // 21 registers 0x31..0x45 inclusive [49...69]
	BMP388_CMD			 = 0x7E,

};

struct LinearisationParameters {
	int32_t T[4];	// 0 unused
	int32_t P[12];	// 0 unused
};

void bmp_decodeLinearisationParameters(struct LinearisationParameters *p, uint8_t *bufp);

// output in milli degrees C and milli(!)pascal
void bmp_linearize(struct LinearisationParameters *par, uint32_t traw, uint32_t praw, int32_t *t_mdegc, int32_t *p_mpa);
