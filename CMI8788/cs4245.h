//
//  cs4245.h
//  PCIAudioDriver
//
//  Created by Gagan on 2022-12-06.
//  Copyright © 2022 CMedia. All rights reserved.
//

#ifndef cs4245_h
#define cs4245_h
/* SPDX-License-Identifier: GPL-2.0 */
#define CS4245_CHIP_ID        0x01
#define CS4245_POWER_CTRL    0x02
#define CS4245_DAC_CTRL_1    0x03
#define CS4245_ADC_CTRL        0x04
#define CS4245_MCLK_FREQ    0x05
#define CS4245_SIGNAL_SEL    0x06
#define CS4245_PGA_B_CTRL    0x07
#define CS4245_PGA_A_CTRL    0x08
#define CS4245_ANALOG_IN    0x09
#define CS4245_DAC_A_CTRL    0x0a
#define CS4245_DAC_B_CTRL    0x0b
#define CS4245_DAC_CTRL_2    0x0c
#define CS4245_INT_STATUS    0x0d
#define CS4245_INT_MASK        0x0e
#define CS4245_INT_MODE_MSB    0x0f
#define CS4245_INT_MODE_LSB    0x10

/* Chip ID */
#define CS4245_CHIP_PART_MASK    0xf0
#define CS4245_CHIP_REV_MASK    0x0f

/* Power Control */
#define CS4245_FREEZE        0x80
#define CS4245_PDN_MIC        0x08
#define CS4245_PDN_ADC        0x04
#define CS4245_PDN_DAC        0x02
#define CS4245_PDN        0x01

/* DAC Control */
#define CS4245_DAC_FM_MASK    0xc0
#define CS4245_DAC_FM_SINGLE    0x00
#define CS4245_DAC_FM_DOUBLE    0x40
#define CS4245_DAC_FM_QUAD    0x80
#define CS4245_DAC_DIF_MASK    0x30
#define CS4245_DAC_DIF_LJUST    0x00
#define CS4245_DAC_DIF_I2S    0x10
#define CS4245_DAC_DIF_RJUST_16    0x20
#define CS4245_DAC_DIF_RJUST_24    0x30
#define CS4245_RESERVED_1    0x08
#define CS4245_MUTE_DAC        0x04
#define CS4245_DEEMPH        0x02
#define CS4245_DAC_MASTER    0x01

/* ADC Control */
#define CS4245_ADC_FM_MASK    0xc0
#define CS4245_ADC_FM_SINGLE    0x00
#define CS4245_ADC_FM_DOUBLE    0x40
#define CS4245_ADC_FM_QUAD    0x80
#define CS4245_ADC_DIF_MASK    0x10
#define CS4245_ADC_DIF_LJUST    0x00
#define CS4245_ADC_DIF_I2S    0x10
#define CS4245_MUTE_ADC        0x04
#define CS4245_HPF_FREEZE    0x02
#define CS4245_ADC_MASTER    0x01

/* MCLK Frequency */
#define CS4245_MCLK1_MASK    0x70
#define CS4245_MCLK1_SHIFT    4
#define CS4245_MCLK2_MASK    0x07
#define CS4245_MCLK2_SHIFT    0
#define CS4245_MCLK_1        0
#define CS4245_MCLK_1_5        1
#define CS4245_MCLK_2        2
#define CS4245_MCLK_3        3
#define CS4245_MCLK_4        4

/* Signal Selection */
#define CS4245_A_OUT_SEL_MASK    0x60
#define CS4245_A_OUT_SEL_HIZ    0x00
#define CS4245_A_OUT_SEL_DAC    0x20
#define CS4245_A_OUT_SEL_PGA    0x40
#define CS4245_LOOP        0x02
#define CS4245_ASYNCH        0x01

/* Channel B/A PGA Control */
#define CS4245_PGA_GAIN_MASK    0x3f

/* ADC Input Control */
#define CS4245_PGA_SOFT        0x10
#define CS4245_PGA_ZERO        0x08
#define CS4245_SEL_MASK        0x07
#define CS4245_SEL_MIC        0x00
#define CS4245_SEL_INPUT_1    0x01
#define CS4245_SEL_INPUT_2    0x02
#define CS4245_SEL_INPUT_3    0x03
#define CS4245_SEL_INPUT_4    0x04
#define CS4245_SEL_INPUT_5    0x05
#define CS4245_SEL_INPUT_6    0x06

/* DAC Channel A/B Volume Control */
#define CS4245_VOL_MASK        0xff

/* DAC Control 2 */
#define CS4245_DAC_SOFT        0x80
#define CS4245_DAC_ZERO        0x40
#define CS4245_INVERT_DAC    0x20
#define CS4245_INT_ACTIVE_HIGH    0x01

/* Interrupt Status/Mask/Mode */
#define CS4245_ADC_CLK_ERR    0x08
#define CS4245_DAC_CLK_ERR    0x04
#define CS4245_ADC_OVFL        0x02
#define CS4245_ADC_UNDRFL    0x01

#define CS4245_SPI_ADDRESS_S    (0x9e << 16)
#define CS4245_SPI_WRITE_S    (0 << 16)

#define CS4245_SPI_ADDRESS    0x9e
#define CS4245_SPI_WRITE    0
#define CS4245_SPI_READ        1

#endif /* cs4245_h */
