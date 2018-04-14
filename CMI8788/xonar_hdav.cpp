/*
 File:SamplePCIAudioEngine.cpp
 
 Contains:
 
 Version:1.0.0
 
 Copyright:Copyright ) 1997-2000 by Apple Computer, Inc., All Rights Reserved.
 
 Disclaimer:IMPORTANT:  This Apple software is supplied to you by Apple Computer, Inc.
 ("Apple") in consideration of your agreement to the following terms, and your use,
 installation, modification or redistribution of this Apple software constitutes acceptance
 of these terms.  If you do not agree with these terms, please do not use, install, modify or
 redistribute this Apple software.
 
 In consideration of your agreement to abide by the following terms, and subject
 to these terms, Apple grants you a personal, non-exclusive license, under Apple's
 copyrights in this original Apple software (the "Apple Software"), to use, reproduce,
 modify and redistribute the Apple Software, with or without modifications, in source and/or
 binary forms; provided that if you redistribute the Apple Software in its entirety
 and without modifications, you must retain this notice and the following text
 and disclaimers in all such redistributions of the Apple Software.  Neither the
 name, trademarks, service marks or logos of Apple Computer, Inc. may be used to
 endorse or promote products derived from the Apple Software without specific prior
 written permission from Apple.  Except as expressly stated in this notice, no
 other rights or licenses, express or implied, are granted by Apple herein,
 including but not limited to any patent rights that may be infringed by your derivative
 works or by other works in which the Apple Software may be incorporated.
 
 The Apple Software is provided by Apple on an "AS IS" basis.  APPLE MAKES NO WARRANTIES,
 EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT,
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE APPLE SOFTWARE
 OR ITS USE AND OPERATION ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL APPLE
 BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE,
 REPRODUCTION, MODIFICATION AND/OR DISTRIBUTION OF THE APPLE SOFTWARE, HOWEVER CAUSED
 AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE), STRICT
 LIABILITY OR OTHERWISE, EVEN IF APPLE HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 */


//#include "ak4396.h"
//#include "wm8785.h"
#include <libkern/OSByteOrder.h>
#include <sys/errno.h>
#include <i386/limits.h>
#include </usr/include//libkern/OSAtomic.h>


#include <IOKit/IOLib.h>
#include <IOKit/IOFilterInterruptEventSource.h>
//#include <architecture/i386/pio.h>
#include "xonar_hdav.hpp"
#include "pcm1796.h"
#include "cm9780.h"
#include "cs2000.h"
#include "ac97.h"
#define INITIAL_SAMPLE_RATE	44100
#define NUM_SAMPLE_FRAMES	16384
#define NUM_CHANNELS		2
#define BIT_DEPTH			16


#define super IOAudioEngine

OSDefineMetaClassAndStructors(SamplePCIAudioEngine, IOAudioEngine)



//static void ak4396_write(struct oxygen *chip, unsigned int codec,
//                         UInt8 reg, UInt8 value)
//{
//    /* maps ALSA channel pair number to SPI output */
//    static const UInt8 codec_spi_map[4] = {
//        0, 1, 2, 4
//    };
//    struct generic_data *data = (struct generic_data*)chip->model_data;
//    
//                      oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER |
//                     OXYGEN_SPI_DATA_LENGTH_2 |
//                     OXYGEN_SPI_CLOCK_160 |
//                     (codec_spi_map[codec] << OXYGEN_SPI_CODEC_SHIFT) |
//                     OXYGEN_SPI_CEN_LATCH_CLOCK_HI,
//                     AK4396_WRITE | (reg << 8) | value);
//    data->ak4396_regs[codec][reg] = value;
//}
//
//static void ak4396_write_cached(struct oxygen *chip, unsigned int codec,
//                                UInt8 reg, UInt8 value)
//{
//    struct generic_data *data = (struct generic_data*)chip->model_data;
//    
//    if (value != data->ak4396_regs[codec][reg])
//        ak4396_write(chip, codec, reg, value);
//}
//
//static void wm8785_write(struct oxygen *chip, UInt8 reg, unsigned int value)
//{
//    struct generic_data *data = (struct generic_data*)chip->model_data;
//    
//    oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER |
//                     OXYGEN_SPI_DATA_LENGTH_2 |
//                     OXYGEN_SPI_CLOCK_160 |
//                     (3 << OXYGEN_SPI_CODEC_SHIFT) |
//                     OXYGEN_SPI_CEN_LATCH_CLOCK_LO,
//                     (reg << 9) | value);
//    if (reg < ARRAY_SIZE(data->wm8785_regs))
//        data->wm8785_regs[reg] = value;
//}
//
//
//static void ak4396_registers_init(struct oxygen *chip)
//{
//    struct generic_data *data = (struct generic_data *)chip->model_data;
//    unsigned int i;
//    
//    for (i = 0; i < data->dacs; ++i) {
//        ak4396_write(chip, i, AK4396_CONTROL_1,
//                     AK4396_DIF_24_MSB | AK4396_RSTN);
//        ak4396_write(chip, i, AK4396_CONTROL_2,
//                     data->ak4396_regs[0][AK4396_CONTROL_2]);
//        ak4396_write(chip, i, AK4396_CONTROL_3,
//                     AK4396_PCM);
//        ak4396_write(chip, i, AK4396_LCH_ATT,
//                     chip->dac_volume[i * 2]);
//        ak4396_write(chip, i, AK4396_RCH_ATT,
//                     chip->dac_volume[i * 2 + 1]);
//    }
//}
//
//static void ak4396_init(struct oxygen *chip)
//{
//    struct generic_data *data =(struct generic_data*) chip->model_data;
//    
//    data->dacs = chip->model.dac_channels_pcm / 2;
//    data->ak4396_regs[0][AK4396_CONTROL_2] =
//    AK4396_SMUTE | AK4396_DEM_OFF | AK4396_DFS_NORMAL;
//    ak4396_registers_init(chip);
//  //  snd_component_add(chip->card, "AK4396");
//}
//
//
//static void wm8785_registers_init(struct oxygen *chip)
//{
//    struct generic_data *data = (struct generic_data*)chip->model_data;
//    
//    wm8785_write(chip, WM8785_R7, 0);
//    wm8785_write(chip, WM8785_R0, data->wm8785_regs[0]);
//    wm8785_write(chip, WM8785_R2, data->wm8785_regs[2]);
//}
//
//static void wm8785_init(struct oxygen *chip)
//{
//    struct generic_data *data = (struct generic_data*)chip->model_data;
//    
//    data->wm8785_regs[0] =
//    WM8785_MCR_SLAVE | WM8785_OSR_SINGLE | WM8785_FORMAT_LJUST;
//    data->wm8785_regs[2] = WM8785_HPFR | WM8785_HPFL;
//    wm8785_registers_init(chip);
//   // snd_component_add(chip->card, "WM8785");
//}
//
//void oxygen_spdif_input_bits_changed(struct work_struct *work)
//{
//    struct oxygen *chip;
//    UInt32 reg;
//    OSSpinLock spinny;
//    /*
//     * This function gets called when there is new activity on the SPDIF
//     * input, or when we lose lock on the input signal, or when the rate
//     * changes.
//     */
//    IODelay(1000);
//    OSSpinLockLock(&chip->reg_lock);
//    reg = oxygen_read32(chip, OXYGEN_SPDIF_CONTROL);
//    if ((reg & (OXYGEN_SPDIF_SENSE_STATUS |
//                OXYGEN_SPDIF_LOCK_STATUS))
//        == OXYGEN_SPDIF_SENSE_STATUS) {
//        /*
//         * If we detect activity on the SPDIF input but cannot lock to
//         * a signal, the clock bit is likely to be wrong.
//         */
//        reg ^= OXYGEN_SPDIF_IN_CLOCK_MASK;
//        oxygen_write32(chip, OXYGEN_SPDIF_CONTROL, reg);
//        OSSpinLockUnlock(&chip->reg_lock);
//        IODelay(1000);
//        OSSpinLockLock(&chip->reg_lock);
//        reg = oxygen_read32(chip, OXYGEN_SPDIF_CONTROL);
//        if ((reg & (OXYGEN_SPDIF_SENSE_STATUS |
//                    OXYGEN_SPDIF_LOCK_STATUS))
//            == OXYGEN_SPDIF_SENSE_STATUS) {
//            /* nothing detected with either clock; give up */
//            if ((reg & OXYGEN_SPDIF_IN_CLOCK_MASK)
//                == OXYGEN_SPDIF_IN_CLOCK_192) {
//                /*
//                 * Reset clock to <= 96 kHz because this is
//                 * more likely to be received next time.
//                 */
//                reg &= ~OXYGEN_SPDIF_IN_CLOCK_MASK;
//                reg |= OXYGEN_SPDIF_IN_CLOCK_96;
//                oxygen_write32(chip, OXYGEN_SPDIF_CONTROL, reg);
//            }
//        }
//    }
//    OSSpinLockUnlock(&chip->reg_lock);
//    
//    if (chip->controls[CONTROL_SPDIF_INPUT_BITS]) {
//        OSSpinLockLock(&chip->reg_lock);
//        chip->interrupt_mask |= OXYGEN_INT_SPDIF_IN_DETECT;
//        oxygen_write16(chip, OXYGEN_INTERRUPT_MASK,
//                       chip->interrupt_mask);
//        OSSpinLockUnlock(&chip->reg_lock);
//        
//        /*
//         * We don't actually know that any channel status bits have
//         * changed, but let's send a notification just to be sure.
//         */
//        //    snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
//        //                  &chip->controls[CONTROL_SPDIF_INPUT_BITS]->id);
//    }
//}




#define GPIO_D2X_EXT_POWER	0x0020
#define GPIO_D2_ALT		0x0080
#define GPIO_D2_OUTPUT_ENABLE	0x0100

#define GPI_EXT_POWER		0x01
#define GPIO_INPUT_ROUTE	0x0100

#define GPIO_HDAV_OUTPUT_ENABLE	0x0001
#define GPIO_HDAV_MAGIC		0x00c0

#define GPIO_DB_MASK		0x0030
#define GPIO_DB_H6		0x0000

#define GPIO_ST_OUTPUT_ENABLE	0x0001
#define GPIO_ST_HP_REAR		0x0002
#define GPIO_ST_MAGIC		0x0040
#define GPIO_ST_HP		0x0080

#define GPIO_XENSE_OUTPUT_ENABLE	(0x0001 | 0x0010 | 0x0020)
#define GPIO_XENSE_SPEAKERS		0x0080

#define I2C_DEVICE_PCM1796(i)	(0x98 + ((i) << 1))	/* 10011, ii, /W=0 */
#define I2C_DEVICE_CS2000	0x9c			/* 100111, 0, /W=0 */

#define PCM1796_REG_BASE	16


void xonar_enable_output(struct oxygen *chip)
{
    struct xonar_generic *data =(struct xonar_generic*)   chip->model_data;
    
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, data->output_enable_bit);
    IODelay(data->anti_pop_delay*1000);
    oxygen_set_bits16(chip, OXYGEN_GPIO_DATA, data->output_enable_bit);
}

void xonar_disable_output(struct oxygen *chip)
{
    struct xonar_generic *data = (struct xonar_generic*)  chip->model_data;
    
    oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, data->output_enable_bit);
}

static void xonar_ext_power_gpio_changed(struct oxygen *chip)
{
    struct xonar_generic *data = (struct xonar_generic*) chip->model_data;
    UInt8 has_power;
    
    has_power = !!(oxygen_read8(chip, data->ext_power_reg)
                   & data->ext_power_bit);
    if (has_power != data->has_power) {
        data->has_power = has_power;
        if (has_power) {
            IOLog("power restored\n");
        } else {
            IOLog("Hey! Don't unplug the power cable!\n");
            /* TODO: stop PCMs */
        }
    }
}

void xonar_init_ext_power(struct oxygen *chip)
{
    struct xonar_generic *data = (struct xonar_generic*) chip->model_data;
    
    oxygen_set_bits8(chip, data->ext_power_int_reg,
                     data->ext_power_bit);
    chip->interrupt_mask |= OXYGEN_INT_GPIO;
    chip->model.gpio_changed = xonar_ext_power_gpio_changed;
    data->has_power = !!(oxygen_read8(chip, data->ext_power_reg)
                         & data->ext_power_bit);
}

void xonar_init_cs53x1(struct oxygen *chip)
{
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_CS53x1_M_MASK);
    oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                          GPIO_CS53x1_M_SINGLE, GPIO_CS53x1_M_MASK);
}
/*
void xonar_set_cs53x1_params(struct oxygen *chip,
                             struct snd_pcm_hw_params *params)
{
    unsigned int value;
    
    if (params_rate(params) <= 54000)
        value = GPIO_CS53x1_M_SINGLE;
    else if (params_rate(params) <= 108000)
        value = GPIO_CS53x1_M_DOUBLE;
    else
        value = GPIO_CS53x1_M_QUAD;
    oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                          value, GPIO_CS53x1_M_MASK);
}

int xonar_gpio_bit_switch_get(struct snd_kcontrol *ctl,
                              struct snd_ctl_elem_value *value)
{
    struct oxygen *chip = ctl->private_data;
    UInt16 bit = ctl->private_value;
    bool invert = ctl->private_value & XONAR_GPIO_BIT_INVERT;
    
    value->value.integer.value[0] =
    !!(oxygen_read16(chip, OXYGEN_GPIO_DATA) & bit) ^ invert;
    return 0;
}

int xonar_gpio_bit_switch_put(struct snd_kcontrol *ctl,
                              struct snd_ctl_elem_value *value)
{
    struct oxygen *chip = ctl->private_data;
    UInt16 bit = ctl->private_value;
    bool invert = ctl->private_value & XONAR_GPIO_BIT_INVERT;
    UInt16 old_bits, new_bits;
    int changed;
    
    spin_lock_irq(&chip->reg_lock);
    old_bits = oxygen_read16(chip, OXYGEN_GPIO_DATA);
    if (!!value->value.integer.value[0] ^ invert)
        new_bits = old_bits | bit;
    else
        new_bits = old_bits & ~bit;
    changed = new_bits != old_bits;
    if (changed)
        oxygen_write16(chip, OXYGEN_GPIO_DATA, new_bits);
    spin_unlock_irq(&chip->reg_lock);
    return changed;
}
*/

void _write_uart(struct oxygen *chip, unsigned int port, UInt8 data)
{
    if (oxygen_read8(chip, OXYGEN_MPU401 + 1) & MPU401_TX_FULL)
        IODelay(1e3);
    oxygen_write8(chip, OXYGEN_MPU401 + port, data);
}

void oxygen_reset_uart(struct oxygen *chip)
{
    _write_uart(chip, 1, MPU401_RESET);
    IODelay(1e3); /* wait for ACK */
    _write_uart(chip, 1, MPU401_ENTER_UART);
}
//EXPORT_SYMBOL(oxygen_reset_uart);

void oxygen_write_uart(struct oxygen *chip, UInt8 data)
{
    _write_uart(chip, 0, data);
}
//EXPORT_SYMBOL(oxygen_write_uart);
static void hdmi_write_command(struct oxygen *chip, UInt8 command,
                               unsigned int count, const UInt8 *params)
{
    unsigned int i;
    UInt8 checksum;
    
    oxygen_write_uart(chip, 0xfb);
    oxygen_write_uart(chip, 0xef);
    oxygen_write_uart(chip, command);
    oxygen_write_uart(chip, count);
    for (i = 0; i < count; ++i)
        oxygen_write_uart(chip, params[i]);
    checksum = 0xfb + 0xef + command + count;
    for (i = 0; i < count; ++i)
        checksum += params[i];
    oxygen_write_uart(chip, checksum);
}

static void xonar_hdmi_init_commands(struct oxygen *chip,
                                     struct xonar_hdmi *hdmi)
{
    UInt8 param;
    
  //  oxygen_reset_uart(chip);
    param = 0;
    hdmi_write_command(chip, 0x61, 1, &param);
    param = 1;
    hdmi_write_command(chip, 0x74, 1, &param);
    hdmi_write_command(chip, 0x54, 5, hdmi->params);
}

void xonar_hdmi_init(struct oxygen *chip, struct xonar_hdmi *hdmi)
{
    hdmi->params[1] = IEC958_AES3_CON_FS_48000;
    hdmi->params[4] = 1;
    xonar_hdmi_init_commands(chip, hdmi);
}

void xonar_hdmi_cleanup(struct oxygen *chip)
{
    UInt8 param = 0;
    
    hdmi_write_command(chip, 0x74, 1, &param);
}

void xonar_hdmi_resume(struct oxygen *chip, struct xonar_hdmi *hdmi)
{
    xonar_hdmi_init_commands(chip, hdmi);
}
/*
void xonar_hdmi_pcm_hardware_filter(unsigned int channel,
                                    struct snd_pcm_hardware *hardware)
{
    if (channel == PCM_MULTICH) {
        hardware->rates = SNDRV_PCM_RATE_44100 |
        SNDRV_PCM_RATE_48000 |
        SNDRV_PCM_RATE_96000 |
        SNDRV_PCM_RATE_192000;
        hardware->rate_min = 44100;
    }
}

void xonar_set_hdmi_params(struct oxygen *chip, struct xonar_hdmi *hdmi,
                           struct snd_pcm_hw_params *params)
{
    hdmi->params[0] = 0; // 1 = non-audio
    switch (params_rate(params)) {
        case 44100:
            hdmi->params[1] = IEC958_AES3_CON_FS_44100;
            break;
        case 48000:
            hdmi->params[1] = IEC958_AES3_CON_FS_48000;
            break;
        default: // 96000
            hdmi->params[1] = IEC958_AES3_CON_FS_96000;
            break;
        case 192000:
            hdmi->params[1] = IEC958_AES3_CON_FS_192000;
            break;
    }
    hdmi->params[2] = params_channels(params) / 2 - 1;
    if (params_format(params) == SNDRV_PCM_FORMAT_S16_LE)
        hdmi->params[3] = 0;
    else
        hdmi->params[3] = 0xc0;
    hdmi->params[4] = 1; // ?
    hdmi_write_command(chip, 0x54, 5, hdmi->params);
}
*/

void xonar_hdmi_uart_input(struct oxygen *chip)
{
    if (chip->uart_input_count >= 2 &&
        chip->uart_input[chip->uart_input_count - 2] == 'O' &&
        chip->uart_input[chip->uart_input_count - 1] == 'K') {
        IOLog("message from HDMI chip received:\n");
        //print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
         //                    chip->uart_input, chip->uart_input_count);
        chip->uart_input_count = 0;
    }
}





static inline void pcm1796_write_spi(struct oxygen *chip, unsigned int codec,
                                     UInt8 reg, UInt8 value)
{
    /* maps ALSA channel pair number to SPI output */
    static const UInt8 codec_map[4] = {
        0, 1, 2, 4
    };
  /*
    oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER  |
                     OXYGEN_SPI_DATA_LENGTH_2 |
                     OXYGEN_SPI_CLOCK_160 |
                     (codec_map[codec] << OXYGEN_SPI_CODEC_SHIFT) |
                     OXYGEN_SPI_CEN_LATCH_CLOCK_HI,
                     (reg << 8) | value); */
}

static inline void pcm1796_write_i2c(struct oxygen *chip, unsigned int codec,
                                     UInt8 reg, UInt8 value)
{
  //  oxygen_write_i2c(chip, I2C_DEVICE_PCM1796(codec), reg, value);
}

static void pcm1796_write(struct oxygen *chip, unsigned int codec,
                          UInt8 reg, UInt8 value)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*)chip->model_data;
    
    if ((chip->model.function_flags & OXYGEN_FUNCTION_2WIRE_SPI_MASK) ==
        OXYGEN_FUNCTION_SPI)
        pcm1796_write_spi(chip, codec, reg, value);
    else
        pcm1796_write_i2c(chip, codec, reg, value);
    if ((unsigned int)(reg - PCM1796_REG_BASE)
        < ARRAY_SIZE(data->pcm1796_regs[codec]))
        data->pcm1796_regs[codec][reg - PCM1796_REG_BASE] = value;
}

static void pcm1796_write_cached(struct oxygen *chip, unsigned int codec,
                                 UInt8 reg, UInt8 value)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*)chip->model_data;
    
    if (value != data->pcm1796_regs[codec][reg - PCM1796_REG_BASE])
        pcm1796_write(chip, codec, reg, value);
}

static void cs2000_write(struct oxygen *chip, UInt8 reg, UInt8 value)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*)chip->model_data;
    
   // oxygen_write_i2c(chip, I2C_DEVICE_CS2000, reg, value);
    data->cs2000_regs[reg] = value;
}

static void cs2000_write_cached(struct oxygen *chip, UInt8 reg, UInt8 value)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*)chip->model_data;
    
    if (value != data->cs2000_regs[reg])
        cs2000_write(chip, reg, value);
}

static void pcm1796_registers_init(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*)chip->model_data;
    unsigned int i;
    SInt8 gain_offset;
    
    IODelay(1000);
    gain_offset = data->hp_active ? data->hp_gain_offset : 0;
    for (i = 0; i < data->dacs; ++i) {
        /* set ATLD before ATL/ATR */
        pcm1796_write(chip, i, 18,
                      data->pcm1796_regs[0][18 - PCM1796_REG_BASE]);
        pcm1796_write(chip, i, 16, chip->dac_volume[i * 2]
                      + gain_offset);
        pcm1796_write(chip, i, 17, chip->dac_volume[i * 2 + 1]
                      + gain_offset);
        pcm1796_write(chip, i, 19,
                      data->pcm1796_regs[0][19 - PCM1796_REG_BASE]);
        pcm1796_write(chip, i, 20,
                      data->pcm1796_regs[0][20 - PCM1796_REG_BASE]);
        pcm1796_write(chip, i, 21, 0);
        gain_offset = 0;
    }
}

static void pcm1796_init(struct oxygen *chip)
{
    struct xonar_pcm179x *data =(struct xonar_pcm179x*) chip->model_data;
    
    data->pcm1796_regs[0][18 - PCM1796_REG_BASE] =
    PCM1796_DMF_DISABLED | PCM1796_FMT_24_I2S | PCM1796_ATLD;
    if (!data->broken_i2c)
        data->pcm1796_regs[0][18 - PCM1796_REG_BASE] |= PCM1796_MUTE;
    data->pcm1796_regs[0][19 - PCM1796_REG_BASE] =
    PCM1796_FLT_SHARP | PCM1796_ATS_1;
    data->pcm1796_regs[0][20 - PCM1796_REG_BASE] =
    data->h6 ? PCM1796_OS_64 : PCM1796_OS_128;
    pcm1796_registers_init(chip);
    data->current_rate = 48000;
}

static void xonar_d2_init(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
    
    data->generic.anti_pop_delay = 300;
    data->generic.output_enable_bit = GPIO_D2_OUTPUT_ENABLE;
    data->dacs = 4;
    
    pcm1796_init(chip);
    
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_D2_ALT);
    oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_D2_ALT);
    
    oxygen_ac97_set_bits(chip, 0, CM9780_JACK, CM9780_FMIC2MIC);
    
    xonar_init_cs53x1(chip);
    xonar_enable_output(chip);
    
  //  snd_component_add(chip->card, "PCM1796");
   // snd_component_add(chip->card, "CS5381");
}

static void xonar_d2x_init(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
    
    data->generic.ext_power_reg = OXYGEN_GPIO_DATA;
    data->generic.ext_power_int_reg = OXYGEN_GPIO_INTERRUPT_MASK;
    data->generic.ext_power_bit = GPIO_D2X_EXT_POWER;
    oxygen_clear_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_D2X_EXT_POWER);
    xonar_init_ext_power(chip);
    xonar_d2_init(chip);
}

static void xonar_hdav_init(struct oxygen *chip)
{
    struct xonar_hdav *data = (struct xonar_hdav*) chip->model_data;
    
    oxygen_write16(chip, OXYGEN_2WIRE_BUS_STATUS,
                   OXYGEN_2WIRE_LENGTH_8 |
                   OXYGEN_2WIRE_INTERRUPT_MASK |
                   OXYGEN_2WIRE_SPEED_STANDARD);
    
    data->pcm179x.generic.anti_pop_delay = 100;
    data->pcm179x.generic.output_enable_bit = GPIO_HDAV_OUTPUT_ENABLE;
    data->pcm179x.generic.ext_power_reg = OXYGEN_GPI_DATA;
    data->pcm179x.generic.ext_power_int_reg = OXYGEN_GPI_INTERRUPT_MASK;
    data->pcm179x.generic.ext_power_bit = GPI_EXT_POWER;
    data->pcm179x.dacs = chip->model.dac_channels_mixer / 2;
    data->pcm179x.h6 = chip->model.dac_channels_mixer > 2;
    
    pcm1796_init(chip);
    
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL,
                      GPIO_HDAV_MAGIC | GPIO_INPUT_ROUTE);
    oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_INPUT_ROUTE);
    
    xonar_init_cs53x1(chip);
    xonar_init_ext_power(chip);
    xonar_hdmi_init(chip, &data->hdmi);
    xonar_enable_output(chip);
    
   // snd_component_add(chip->card, "PCM1796");
   // snd_component_add(chip->card, "CS5381");
}

static void xonar_st_init_i2c(struct oxygen *chip)
{
    oxygen_write16(chip, OXYGEN_2WIRE_BUS_STATUS,
                   OXYGEN_2WIRE_LENGTH_8 |
                   OXYGEN_2WIRE_INTERRUPT_MASK |
                   OXYGEN_2WIRE_SPEED_STANDARD);
}

static void xonar_st_init_common(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
    
    data->generic.output_enable_bit = GPIO_ST_OUTPUT_ENABLE;
    data->dacs = chip->model.dac_channels_mixer / 2;
    data->h6 = chip->model.dac_channels_mixer > 2;
    data->hp_gain_offset = 2*-18;
    
    pcm1796_init(chip);
    
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL,
                      GPIO_INPUT_ROUTE | GPIO_ST_HP_REAR |
                      GPIO_ST_MAGIC | GPIO_ST_HP);
    oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA,
                        GPIO_INPUT_ROUTE | GPIO_ST_HP_REAR | GPIO_ST_HP);
    
    xonar_init_cs53x1(chip);
    xonar_enable_output(chip);
    
 //   snd_component_add(chip->card, "PCM1792A");
  //  snd_component_add(chip->card, "CS5381");
}

static void cs2000_registers_init(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
    
    cs2000_write(chip, CS2000_GLOBAL_CFG, CS2000_FREEZE);
    cs2000_write(chip, CS2000_DEV_CTRL, 0);
    cs2000_write(chip, CS2000_DEV_CFG_1,
                 CS2000_R_MOD_SEL_1 |
                 (0 << CS2000_R_SEL_SHIFT) |
                 CS2000_AUX_OUT_SRC_REF_CLK |
                 CS2000_EN_DEV_CFG_1);
    cs2000_write(chip, CS2000_DEV_CFG_2,
                 (0 << CS2000_LOCK_CLK_SHIFT) |
                 CS2000_FRAC_N_SRC_STATIC);
    cs2000_write(chip, CS2000_RATIO_0 + 0, 0x00); /* 1.0 */
    cs2000_write(chip, CS2000_RATIO_0 + 1, 0x10);
    cs2000_write(chip, CS2000_RATIO_0 + 2, 0x00);
    cs2000_write(chip, CS2000_RATIO_0 + 3, 0x00);
    cs2000_write(chip, CS2000_FUN_CFG_1,
                 data->cs2000_regs[CS2000_FUN_CFG_1]);
    cs2000_write(chip, CS2000_FUN_CFG_2, 0);
    cs2000_write(chip, CS2000_GLOBAL_CFG, CS2000_EN_DEV_CFG_2);
    IODelay(3*1000); /* PLL lock delay */
}

static void xonar_st_init(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*)chip->model_data;
    
    data->generic.anti_pop_delay = 100;
    data->h6 = chip->model.dac_channels_mixer > 2;
    data->has_cs2000 = 1;
    data->cs2000_regs[CS2000_FUN_CFG_1] = CS2000_REF_CLK_DIV_1;
    data->broken_i2c = true;
    
    oxygen_write16(chip, OXYGEN_I2S_A_FORMAT,
                   OXYGEN_RATE_48000 |
                   OXYGEN_I2S_FORMAT_I2S |
                   OXYGEN_I2S_MCLK(data->h6 ? MCLK_256 : MCLK_512) |
                   OXYGEN_I2S_BITS_16 |
                   OXYGEN_I2S_MASTER |
                   OXYGEN_I2S_BCLK_64);
    
    xonar_st_init_i2c(chip);
    cs2000_registers_init(chip);
    xonar_st_init_common(chip);
    
  //  snd_component_add(chip->card, "CS2000");
}

static void xonar_stx_init(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*)chip->model_data;
    
    xonar_st_init_i2c(chip);
    data->generic.anti_pop_delay = 800;
    data->generic.ext_power_reg = OXYGEN_GPI_DATA;
    data->generic.ext_power_int_reg = OXYGEN_GPI_INTERRUPT_MASK;
    data->generic.ext_power_bit = GPI_EXT_POWER;
    xonar_init_ext_power(chip);
    xonar_st_init_common(chip);
}

static void xonar_xense_init(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*)chip->model_data;
    
    data->generic.ext_power_reg = OXYGEN_GPI_DATA;
    data->generic.ext_power_int_reg = OXYGEN_GPI_INTERRUPT_MASK;
    data->generic.ext_power_bit = GPI_EXT_POWER;
    xonar_init_ext_power(chip);
    
    data->generic.anti_pop_delay = 100;
    data->has_cs2000 = 1;
    data->cs2000_regs[CS2000_FUN_CFG_1] = CS2000_REF_CLK_DIV_1;
    
    oxygen_write16(chip, OXYGEN_I2S_A_FORMAT,
                   OXYGEN_RATE_48000 |
                   OXYGEN_I2S_FORMAT_I2S |
                   OXYGEN_I2S_MCLK(MCLK_512) |
                   OXYGEN_I2S_BITS_16 |
                   OXYGEN_I2S_MASTER |
                   OXYGEN_I2S_BCLK_64);
    
    xonar_st_init_i2c(chip);
    cs2000_registers_init(chip);
    
    data->generic.output_enable_bit = GPIO_XENSE_OUTPUT_ENABLE;
    data->dacs = 1;
    data->hp_gain_offset = 2*-18;
    
    pcm1796_init(chip);
    
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL,
                      GPIO_INPUT_ROUTE | GPIO_ST_HP_REAR |
                      GPIO_ST_MAGIC | GPIO_XENSE_SPEAKERS);
    oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA,
                        GPIO_INPUT_ROUTE | GPIO_ST_HP_REAR |
                        GPIO_XENSE_SPEAKERS);
    
    xonar_init_cs53x1(chip);
    xonar_enable_output(chip);
    
 //   snd_component_add(chip->card, "PCM1796");
 //   snd_component_add(chip->card, "CS5381");
  //  snd_component_add(chip->card, "CS2000");
}

static void xonar_d2_cleanup(struct oxygen *chip)
{
    xonar_disable_output(chip);
}

static void xonar_hdav_cleanup(struct oxygen *chip)
{
    xonar_hdmi_cleanup(chip);
    xonar_disable_output(chip);
    IODelay(2);
}

static void xonar_st_cleanup(struct oxygen *chip)
{
    xonar_disable_output(chip);
}

static void xonar_d2_suspend(struct oxygen *chip)
{
    xonar_d2_cleanup(chip);
}

static void xonar_hdav_suspend(struct oxygen *chip)
{
    xonar_hdav_cleanup(chip);
}

static void xonar_st_suspend(struct oxygen *chip)
{
    xonar_st_cleanup(chip);
}

static void xonar_d2_resume(struct oxygen *chip)
{
    pcm1796_registers_init(chip);
    xonar_enable_output(chip);
}

static void xonar_hdav_resume(struct oxygen *chip)
{
    struct xonar_hdav *data = (struct xonar_hdav*) chip->model_data;
    
    pcm1796_registers_init(chip);
    xonar_hdmi_resume(chip, &data->hdmi);
    xonar_enable_output(chip);
}

static void xonar_stx_resume(struct oxygen *chip)
{
    pcm1796_registers_init(chip);
    xonar_enable_output(chip);
}

static void xonar_st_resume(struct oxygen *chip)
{
    cs2000_registers_init(chip);
    xonar_stx_resume(chip);
}

static void update_pcm1796_oversampling(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
    unsigned int i;
    UInt8 reg;
    
    if (data->current_rate <= 48000 && !data->h6)
        reg = PCM1796_OS_128;
    else
        reg = PCM1796_OS_64;
    for (i = 0; i < data->dacs; ++i)
        pcm1796_write_cached(chip, i, 20, reg);
}
/*
static void set_pcm1796_params(struct oxygen *chip,
                               struct snd_pcm_hw_params *params)
{
    struct xonar_pcm179x *data = (struct xonar_hdav*) chip->model_data;
    
    msleep(1);
    data->current_rate = params_rate(params);
    update_pcm1796_oversampling(chip);
}*/

static void update_pcm1796_volume(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
    unsigned int i;
    SInt8 gain_offset;
    
    gain_offset = data->hp_active ? data->hp_gain_offset : 0;
    for (i = 0; i < data->dacs; ++i) {
        pcm1796_write_cached(chip, i, 16, chip->dac_volume[i * 2]
                             + gain_offset);
        pcm1796_write_cached(chip, i, 17, chip->dac_volume[i * 2 + 1]
                             + gain_offset);
        gain_offset = 0;
    }
}

static void update_pcm1796_mute(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
    unsigned int i;
    UInt8 value;
    
    value = PCM1796_DMF_DISABLED | PCM1796_FMT_24_I2S | PCM1796_ATLD;
    if (chip->dac_mute)
        value |= PCM1796_MUTE;
    for (i = 0; i < data->dacs; ++i)
        pcm1796_write_cached(chip, i, 18, value);
}

static void update_cs2000_rate(struct oxygen *chip, unsigned int rate)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
    UInt8 rate_mclk, reg;
    
    switch (rate) {
        case 32000:
        case 64000:
            rate_mclk = OXYGEN_RATE_32000;
            break;
        case 44100:
        case 88200:
        case 176400:
            rate_mclk = OXYGEN_RATE_44100;
            break;
        default:
        case 48000:
        case 96000:
        case 192000:
            rate_mclk = OXYGEN_RATE_48000;
            break;
    }
    
    if (rate <= 96000 && (rate > 48000 || data->h6)) {
        rate_mclk |= OXYGEN_I2S_MCLK(MCLK_256);
        reg = CS2000_REF_CLK_DIV_1;
    } else {
        rate_mclk |= OXYGEN_I2S_MCLK(MCLK_512);
        reg = CS2000_REF_CLK_DIV_2;
    }
    
    oxygen_write16_masked(chip, OXYGEN_I2S_A_FORMAT, rate_mclk,
                          OXYGEN_I2S_RATE_MASK | OXYGEN_I2S_MCLK_MASK);
    cs2000_write_cached(chip, CS2000_FUN_CFG_1, reg);
    IODelay(3*1000); /* PLL lock delay */
}
/*
static void set_st_params(struct oxygen *chip,
                          struct snd_pcm_hw_params *params)
{
    update_cs2000_rate(chip, params_rate(params));
    set_pcm1796_params(chip, params);
}

static void set_hdav_params(struct oxygen *chip,
                            struct snd_pcm_hw_params *params)
{
    struct xonar_hdav *data = (struct xonar_hdav*) chip->model_data;
    
    set_pcm1796_params(chip, params);
    xonar_set_hdmi_params(chip, &data->hdmi, params);
}
*/
//static const struct snd_kcontrol_new alt_switch = {
//    .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//    .name = "Analog Loopback Switch",
//    .info = snd_ctl_boolean_mono_info,
//    .get = xonar_gpio_bit_switch_get,
//    .put = xonar_gpio_bit_switch_put,
//    .private_value = GPIO_D2_ALT,
//};

//static int rolloff_info(struct snd_kcontrol *ctl,
//                        struct snd_ctl_elem_info *info)
//{
//    static const char *const names[2] = {
//        "Sharp Roll-off", "Slow Roll-off"
//    };
//    
//    return snd_ctl_enum_info(info, 1, 2, names);
//}
//
//static int rolloff_get(struct snd_kcontrol *ctl,
//                       struct snd_ctl_elem_value *value)
//{
//    struct oxygen *chip = ctl->private_data;
//    struct xonar_pcm179x *data = chip->model_data;
//    
//    value->value.enumerated.item[0] =
//    (data->pcm1796_regs[0][19 - PCM1796_REG_BASE] &
//     PCM1796_FLT_MASK) != PCM1796_FLT_SHARP;
//    return 0;
//}
//
//static int rolloff_put(struct snd_kcontrol *ctl,
//                       struct snd_ctl_elem_value *value)
//{
//    struct oxygen *chip = ctl->private_data;
//    struct xonar_pcm179x *data = chip->model_data;
//    unsigned int i;
//    int changed;
//    UInt8 reg;
//    
//    mutex_lock(&chip->mutex);
//    reg = data->pcm1796_regs[0][19 - PCM1796_REG_BASE];
//    reg &= ~PCM1796_FLT_MASK;
//    if (!value->value.enumerated.item[0])
//        reg |= PCM1796_FLT_SHARP;
//    else
//        reg |= PCM1796_FLT_SLOW;
//    changed = reg != data->pcm1796_regs[0][19 - PCM1796_REG_BASE];
//    if (changed) {
//        for (i = 0; i < data->dacs; ++i)
//            pcm1796_write(chip, i, 19, reg);
//    }
//    mutex_unlock(&chip->mutex);
//    return changed;
//}
//
//static const struct snd_kcontrol_new rolloff_control = {
//    .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//    .name = "DAC Filter Playback Enum",
//    .info = rolloff_info,
//    .get = rolloff_get,
//    .put = rolloff_put,
//};
//
//static const struct snd_kcontrol_new hdav_hdmi_control = {
//    .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//    .name = "HDMI Playback Switch",
//    .info = snd_ctl_boolean_mono_info,
//    .get = xonar_gpio_bit_switch_get,
//    .put = xonar_gpio_bit_switch_put,
//    .private_value = GPIO_HDAV_OUTPUT_ENABLE | XONAR_GPIO_BIT_INVERT,
//};
//
//static int st_output_switch_info(struct snd_kcontrol *ctl,
//                                 struct snd_ctl_elem_info *info)
//{
//    static const char *const names[3] = {
//        "Speakers", "Headphones", "FP Headphones"
//    };
//    
//    return snd_ctl_enum_info(info, 1, 3, names);
//}
//
//static int st_output_switch_get(struct snd_kcontrol *ctl,
//                                struct snd_ctl_elem_value *value)
//{
//    struct oxygen *chip = ctl->private_data;
//    UInt16 gpio;
//    
//    gpio = oxygen_read16(chip, OXYGEN_GPIO_DATA);
//    if (!(gpio & GPIO_ST_HP))
//        value->value.enumerated.item[0] = 0;
//    else if (gpio & GPIO_ST_HP_REAR)
//        value->value.enumerated.item[0] = 1;
//    else
//        value->value.enumerated.item[0] = 2;
//    return 0;
//}
//
//
//static int st_output_switch_put(struct snd_kcontrol *ctl,
//                                struct snd_ctl_elem_value *value)
//{
//    struct oxygen *chip = ctl->private_data;
//    struct xonar_pcm179x *data = chip->model_data;
//    UInt16 gpio_old, gpio;
//    
//    mutex_lock(&chip->mutex);
//    gpio_old = oxygen_read16(chip, OXYGEN_GPIO_DATA);
//    gpio = gpio_old;
//    switch (value->value.enumerated.item[0]) {
//        case 0:
//            gpio &= ~(GPIO_ST_HP | GPIO_ST_HP_REAR);
//            break;
//        case 1:
//            gpio |= GPIO_ST_HP | GPIO_ST_HP_REAR;
//            break;
//        case 2:
//            gpio = (gpio | GPIO_ST_HP) & ~GPIO_ST_HP_REAR;
//            break;
//    }
//    oxygen_write16(chip, OXYGEN_GPIO_DATA, gpio);
//    data->hp_active = gpio & GPIO_ST_HP;
//    update_pcm1796_volume(chip);
//    mutex_unlock(&chip->mutex);
//    return gpio != gpio_old;
//}
//
//static int st_hp_volume_offset_info(struct snd_kcontrol *ctl,
//                                    struct snd_ctl_elem_info *info)
//{
//    static const char *const names[4] = {
//        "< 32 ohms", "32-64 ohms", "64-300 ohms", "300-600 ohms"
//    };
//    
//    return snd_ctl_enum_info(info, 1, 4, names);
//}
//
//static int st_hp_volume_offset_get(struct snd_kcontrol *ctl,
//                                   struct snd_ctl_elem_value *value)
//{
//    struct oxygen *chip = ctl->private_data;
//    struct xonar_pcm179x *data = chip->model_data;
//    
//    mutex_lock(&chip->mutex);
//    if (data->hp_gain_offset < 2*-12)
//        value->value.enumerated.item[0] = 0;
//    else if (data->hp_gain_offset < 2*-6)
//        value->value.enumerated.item[0] = 1;
//    else if (data->hp_gain_offset < 0)
//        value->value.enumerated.item[0] = 2;
//    else
//        value->value.enumerated.item[0] = 3;
//    mutex_unlock(&chip->mutex);
//    return 0;
//}
//
//
//static int st_hp_volume_offset_put(struct snd_kcontrol *ctl,
//                                   struct snd_ctl_elem_value *value)
//{
//    static const s8 offsets[] = { 2*-18, 2*-12, 2*-6, 0 };
//    struct oxygen *chip = ctl->private_data;
//    struct xonar_pcm179x *data = chip->model_data;
//    s8 offset;
//    int changed;
//    
//    if (value->value.enumerated.item[0] > 3)
//        return -EINVAL;
//    offset = offsets[value->value.enumerated.item[0]];
//    mutex_lock(&chip->mutex);
//    changed = offset != data->hp_gain_offset;
//    if (changed) {
//        data->hp_gain_offset = offset;
//        update_pcm1796_volume(chip);
//    }
//    mutex_unlock(&chip->mutex);
//    return changed;
//}
//
////static const struct snd_kcontrol_new st_controls[] = {
////    {
////        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
////        .name = "Analog Output",
////        .info = st_output_switch_info,
////        .get = st_output_switch_get,
////        .put = st_output_switch_put,
////    },
////    {
////        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
////        .name = "Headphones Impedance Playback Enum",
////        .info = st_hp_volume_offset_info,
////        .get = st_hp_volume_offset_get,
////        .put = st_hp_volume_offset_put,
////    },
////};
//
//static int xense_output_switch_get(struct snd_kcontrol *ctl,
//                                   struct snd_ctl_elem_value *value)
//{
//    struct oxygen *chip = ctl->private_data;
//    UInt16 gpio;
//    
//    gpio = oxygen_read16(chip, OXYGEN_GPIO_DATA);
//    if (gpio & GPIO_XENSE_SPEAKERS)
//        value->value.enumerated.item[0] = 0;
//    else if (!(gpio & GPIO_XENSE_SPEAKERS) && (gpio & GPIO_ST_HP_REAR))
//        value->value.enumerated.item[0] = 1;
//    else
//        value->value.enumerated.item[0] = 2;
//    return 0;
//}
//
//static int xense_output_switch_put(struct snd_kcontrol *ctl,
//                                   struct snd_ctl_elem_value *value)
//{
//    struct oxygen *chip = ctl->private_data;
//    struct xonar_pcm179x *data = chip->model_data;
//    UInt16 gpio_old, gpio;
//    
//    mutex_lock(&chip->mutex);
//    gpio_old = oxygen_read16(chip, OXYGEN_GPIO_DATA);
//    gpio = gpio_old;
//    switch (value->value.enumerated.item[0]) {
//        case 0:
//            gpio |= GPIO_XENSE_SPEAKERS | GPIO_ST_HP_REAR;
//            break;
//        case 1:
//            gpio = (gpio | GPIO_ST_HP_REAR) & ~GPIO_XENSE_SPEAKERS;
//            break;
//        case 2:
//            gpio &= ~(GPIO_XENSE_SPEAKERS | GPIO_ST_HP_REAR);
//            break;
//    }
//    oxygen_write16(chip, OXYGEN_GPIO_DATA, gpio);
//    data->hp_active = !(gpio & GPIO_XENSE_SPEAKERS);
//    update_pcm1796_volume(chip);
//    mutex_unlock(&chip->mutex);
//    return gpio != gpio_old;
//}

//static const struct snd_kcontrol_new xense_controls[] = {
//    {
//        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//        .name = "Analog Output",
//        .info = st_output_switch_info,
//        .get = xense_output_switch_get,
//        .put = xense_output_switch_put,
//    },
//    {
//        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//        .name = "Headphones Impedance Playback Enum",
//        .info = st_hp_volume_offset_info,
//        .get = st_hp_volume_offset_get,
//        .put = st_hp_volume_offset_put,
//    },
//};

static void xonar_line_mic_ac97_switch(struct oxygen *chip,
                                       unsigned int reg, unsigned int mute)
{
    if (reg == AC97_LINE) {
        OSSpinLockLock(&chip->reg_lock);
        oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                              mute ? GPIO_INPUT_ROUTE : 0,
                              GPIO_INPUT_ROUTE);
        OSSpinLockUnlock(&chip->reg_lock);
    }
}
//
//static const DECLARE_TLV_DB_SCALE(pcm1796_db_scale, -6000, 50, 0);
//
//static int xonar_d2_control_filter(struct snd_kcontrol_new *template)
//{
//    if (!strncmp(template->name, "CD Capture ", 11))
//    /* CD in is actually connected to the video in pin */
//        template->private_value ^= AC97_CD ^ AC97_VIDEO;
//    return 0;
//}
//
//static int xonar_st_h6_control_filter(struct snd_kcontrol_new *template)
//{
//    if (!strncmp(template->name, "Master Playback ", 16))
//    /* no volume/mute, as I²C to the third DAC does not work */
//        return 1;
//    return 0;
//}

static int add_pcm1796_controls(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*)chip->model_data;
    int err;
    
    /*if (!data->broken_i2c) {
        err = snd_ctl_add(chip->card,
                          snd_ctl_new1(&rolloff_control, chip));
        if (err < 0)
            return err;
    }*/
    return 0;
}

static int xonar_d2_mixer_init(struct oxygen *chip)
{
    int err;
    
 //   err = snd_ctl_add(chip->card, snd_ctl_new1(&alt_switch, chip));
    if (err < 0)
        return err;
    err = add_pcm1796_controls(chip);
    if (err < 0)
        return err;
    return 0;
}

static int xonar_hdav_mixer_init(struct oxygen *chip)
{
    int err;
    
  //  err = snd_ctl_add(chip->card, snd_ctl_new1(&hdav_hdmi_control, chip));
    if (err < 0)
        return err;
    err = add_pcm1796_controls(chip);
    if (err < 0)
        return err;
    return 0;
}



bool SamplePCIAudioEngine::init(struct oxygen *chip)
{
    bool result = false;
    
    IOLog("SamplePCIAudioEngine[%p]::init(%p)\n", this, chip);
    
    if (!chip) {
        goto Done;
    }
    
    if (!super::init(NULL)) {
        goto Done;
    }
  //  ak4396_init(chip);
  //  wm8785_init(chip);
    deviceRegisters = (struct xonar_hdav*)chip->model_data;
    
    result = true;
    
Done:
    
    return result;
}

bool SamplePCIAudioEngine::initHardware(IOService *provider)
{
    bool result = false;
    IOAudioSampleRate initialSampleRate;
    IOAudioStream *audioStream;
    IOWorkLoop *workLoop;
    
    IOLog("SamplePCIAudioEngine[%p]::initHardware(%p)\n", this, provider);
    
    if (!super::initHardware(provider)) {
        goto Done;
    }
    
    // Setup the initial sample rate for the audio engine
    initialSampleRate.whole = INITIAL_SAMPLE_RATE;
    initialSampleRate.fraction = 0;
    
    setDescription("Sample PCI Audio Engine");
    
    setSampleRate(&initialSampleRate);
    
    // Set the number of sample frames in each buffer
    setNumSampleFramesPerBuffer(NUM_SAMPLE_FRAMES);
    
    workLoop = getWorkLoop();
    if (!workLoop) {
        goto Done;
    }
    
    // Create an interrupt event source through which to receive interrupt callbacks
    // In this case, we only want to do work at primary interrupt time, so
    // we create an IOFilterInterruptEventSource which makes a filtering call
    // from the primary interrupt interrupt who's purpose is to determine if
    // our secondary interrupt handler is to be called.  In our case, we
    // can do the work in the filter routine and then return false to
    // indicate that we do not want our secondary handler called
    interruptEventSource = IOFilterInterruptEventSource::filterInterruptEventSource(this,
                                                                                    SamplePCIAudioEngine::interruptHandler,
                                                                                    SamplePCIAudioEngine::interruptFilter,
                                                                                    audioDevice->getProvider());
    if (!interruptEventSource) {
        goto Done;
    }
    
    // In order to allow the interrupts to be received, the interrupt event source must be
    // added to the IOWorkLoop
    // Additionally, interrupts will not be firing until the interrupt event source is
    // enabled by calling interruptEventSource->enable() - this probably doesn't need to
    // be done until performAudioEngineStart() is called, and can probably be disabled
    // when performAudioEngineStop() is called and the audio engine is no longer running
    // Although this really depends on the specific hardware
    workLoop->addEventSource(interruptEventSource);
    
    // Allocate our input and output buffers - a real driver will likely need to allocate its buffers
    // differently
    outputBuffer = (SInt16 *)IOMalloc(BUFFER_SIZE);
    if (!outputBuffer) {
        goto Done;
    }
    
    inputBuffer = (SInt16 *)IOMalloc(BUFFER_SIZE);
    if (!inputBuffer) {
        goto Done;
    }
    
    // Create an IOAudioStream for each buffer and add it to this audio engine
    audioStream = createNewAudioStream(kIOAudioStreamDirectionOutput, outputBuffer, BUFFER_SIZE);
    if (!audioStream) {
        goto Done;
    }
    
    addAudioStream(audioStream);
    audioStream->release();
    
    audioStream = createNewAudioStream(kIOAudioStreamDirectionInput, inputBuffer, BUFFER_SIZE);
    if (!audioStream) {
        goto Done;
    }
    
    addAudioStream(audioStream);
    audioStream->release();
    
    result = true;
    
Done:
    
    return result;
}

void SamplePCIAudioEngine::free()
{
    IOLog("SamplePCIAudioEngine[%p]::free()\n", this);
    
    // We need to free our resources when we're going away
    
    if (interruptEventSource) {
        interruptEventSource->release();
        interruptEventSource = NULL;
    }
    
    if (outputBuffer) {
        IOFree(outputBuffer, BUFFER_SIZE);
        outputBuffer = NULL;
    }
    
    if (inputBuffer) {
        IOFree(inputBuffer, BUFFER_SIZE);
        inputBuffer = NULL;
    }
    
    super::free();
}

IOAudioStream *SamplePCIAudioEngine::createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize)
{
    IOAudioStream *audioStream;
    
    // For this sample device, we are only creating a single format and allowing 44.1KHz and 48KHz
    audioStream = new IOAudioStream;
    if (audioStream) {
        if (!audioStream->initWithAudioEngine(this, direction, 1)) {
            audioStream->release();
        } else {
            IOAudioSampleRate rate;
            IOAudioStreamFormat format = {
                2,												// num channels
                kIOAudioStreamSampleFormatLinearPCM,			// sample format
                kIOAudioStreamNumericRepresentationSignedInt,	// numeric format
                BIT_DEPTH,										// bit depth
                BIT_DEPTH,										// bit width
                kIOAudioStreamAlignmentHighByte,				// high byte aligned - unused because bit depth == bit width
                kIOAudioStreamByteOrderBigEndian,				// big endian
                true,											// format is mixable
                0												// driver-defined tag - unused by this driver
            };
            
            // As part of creating a new IOAudioStream, its sample buffer needs to be set
            // It will automatically create a mix buffer should it be needed
            audioStream->setSampleBuffer(sampleBuffer, sampleBufferSize);
            
            // This device only allows a single format and a choice of 2 different sample rates
            rate.fraction = 0;
            rate.whole = 44100;
            audioStream->addAvailableFormat(&format, &rate, &rate);
            rate.whole = 48000;
            audioStream->addAvailableFormat(&format, &rate, &rate);
            
            // Finally, the IOAudioStream's current format needs to be indicated
            audioStream->setFormat(&format);
        }
    }
    
    return audioStream;
}

void SamplePCIAudioEngine::stop(IOService *provider)
{
    IOLog("SamplePCIAudioEngine[%p]::stop(%p)\n", this, provider);
    
    // When our device is being stopped and torn down, we should go ahead and remove
    // the interrupt event source from the IOWorkLoop
    // Additionally, we'll go ahead and release the interrupt event source since it isn't
    // needed any more
    if (interruptEventSource) {
        IOWorkLoop *wl;
        
        wl = getWorkLoop();
        if (wl) {
            wl->removeEventSource(interruptEventSource);
        }
        
        interruptEventSource->release();
        interruptEventSource = NULL;
    }
    
    // Add code to shut down hardware (beyond what is needed to simply stop the audio engine)
    // There may be nothing needed here
    
    super::stop(provider);
}

IOReturn SamplePCIAudioEngine::performAudioEngineStart()
{
    IOLog("SamplePCIAudioEngine[%p]::performAudioEngineStart()\n", this);
    
    // The interruptEventSource needs to be enabled to allow interrupts to start firing
    assert(interruptEventSource);
    interruptEventSource->enable();
    
    // When performAudioEngineStart() gets called, the audio engine should be started from the beginning
    // of the sample buffer.  Because it is starting on the first sample, a new timestamp is needed
    // to indicate when that sample is being read from/written to.  The function takeTimeStamp()
    // is provided to do that automatically with the current time.
    // By default takeTimeStamp() will increment the current loop count in addition to taking the current
    // timestamp.  Since we are starting a new audio engine run, and not looping, we don't want the loop count
    // to be incremented.  To accomplish that, false is passed to takeTimeStamp().
    takeTimeStamp(false);
    
    // Add audio - I/O start code here
    
#error performAudioEngineStart() - driver will not work until audio engine start code is added
    
    return kIOReturnSuccess;
}

IOReturn SamplePCIAudioEngine::performAudioEngineStop()
{
    IOLog("SamplePCIAudioEngine[%p]::performAudioEngineStop()\n", this);
    
    // Assuming we don't need interrupts after stopping the audio engine, we can disable them here
    assert(interruptEventSource);
    interruptEventSource->disable();
    
    // Add audio - I/O stop code here
    
#error performAudioEngineStop() - driver will not work until audio engine stop code is added
    
    return kIOReturnSuccess;
}

UInt32 SamplePCIAudioEngine::getCurrentSampleFrame()
{
    IOLog("SamplePCIAudioEngine[%p]::getCurrentSampleFrame()\n", this);
    
    // In order for the erase process to run properly, this function must return the current location of
    // the audio engine - basically a sample counter
    // It doesn't need to be exact, but if it is inexact, it should err towards being before the current location
    // rather than after the current location.  The erase head will erase up to, but not including the sample
    // frame returned by this function.  If it is too large a value, sound data that hasn't been played will be
    // erased.
    
#error getCurrentSampleFrame() - driver will not work until correct sample frame is returned
    
    // Change to return the real value
    return 0;
}

IOReturn SamplePCIAudioEngine::performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate)
{
    IOLog("SamplePCIAudioEngine[%p]::peformFormatChange(%p, %p, %p)\n", this, audioStream, newFormat, newSampleRate);
    
    // Since we only allow one format, we only need to be concerned with sample rate changes
    // In this case, we only allow 2 sample rates - 44100 & 48000, so those are the only ones
    // that we check for
    if (newSampleRate) {
        switch (newSampleRate->whole) {
            case 44100:
                IOLog("/t-> 44.1kHz selected\n");
                
                // Add code to switch hardware to 44.1khz
                break;
            case 48000:
                IOLog("/t-> 48kHz selected\n");
                
                // Add code to switch hardware to 48kHz
                break;
            default:
                // This should not be possible since we only specified 44100 and 48000 as valid sample rates
                IOLog("/t Internal Error - unknown sample rate selected.\n");
                break;
        }
    }
    
    return kIOReturnSuccess;
}


void SamplePCIAudioEngine::interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count)
{
    // Since our interrupt filter always returns false, this function will never be called
    // If the filter returned true, this function would be called on the IOWorkLoop
    return;
}

bool SamplePCIAudioEngine::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source)
{
    SamplePCIAudioEngine *audioEngine = OSDynamicCast(SamplePCIAudioEngine, owner);
    
    // We've cast the audio engine from the owner which we passed in when we created the interrupt
    // event source
    if (audioEngine) {
        // Then, filterInterrupt() is called on the specified audio engine
        audioEngine->filterInterrupt(source->getIntIndex());
    }
    
    return false;
}

void SamplePCIAudioEngine::filterInterrupt(int index)
{
    // In the case of our simple device, we only get interrupts when the audio engine loops to the
    // beginning of the buffer.  When that happens, we need to take a timestamp and increment
    // the loop count.  The function takeTimeStamp() does both of those for us.  Additionally,
    // if a different timestamp is to be used (other than the current time), it can be passed
    // in to takeTimeStamp()
    takeTimeStamp();
}

