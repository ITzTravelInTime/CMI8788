/*
 File:XonarAudioEngine.cpp
 
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
#include </usr/include/libkern/OSAtomic.h>


#include <IOKit/IOLib.h>
#include <IOKit/IOFilterInterruptEventSource.h>
//#include <architecture/i386/pio.h>
#include "XonarAudioEngine.hpp"
#include "pcm1796.h"
#include "cm9780.h"
#include "cs2000.h"
#include "ac97.h"
#define INITIAL_SAMPLE_RATE	44100
#define NUM_SAMPLE_FRAMES	16384
#define NUM_CHANNELS		2
#define BIT_DEPTH			16


#define super IOAudioEngine

OSDefineMetaClassAndStructors(XonarAudioEngine, IOAudioEngine)



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


void XonarAudioEngine::oxygen_interrupt(OSObject *owner, IOInterruptEventSource *src, int dummy, void *dev_id)
{
    struct oxygen *chip = (struct oxygen*)dev_id;
    unsigned int status, clear, elapsed_streams, i;
    
    status = oxygen_read16(chip, OXYGEN_INTERRUPT_STATUS);
//    if (!status)
//        return IRQ_NONE;
    
    OSSpinLockLock(&chip->reg_lock);
    
    clear = status & (OXYGEN_CHANNEL_A |
                      OXYGEN_CHANNEL_B |
                      OXYGEN_CHANNEL_C |
                      OXYGEN_CHANNEL_SPDIF |
                      OXYGEN_CHANNEL_MULTICH |
                      OXYGEN_CHANNEL_AC97 |
                      OXYGEN_INT_SPDIF_IN_DETECT |
                      OXYGEN_INT_GPIO |
                      OXYGEN_INT_AC97);
    if (clear) {
        if (clear & OXYGEN_INT_SPDIF_IN_DETECT)
            chip->interrupt_mask &= ~OXYGEN_INT_SPDIF_IN_DETECT;
        oxygen_write16(chip, OXYGEN_INTERRUPT_MASK,
                       chip->interrupt_mask & ~clear);
        oxygen_write16(chip, OXYGEN_INTERRUPT_MASK,
                       chip->interrupt_mask);
    }
    
    elapsed_streams = status & chip->pcm_running;
    
    OSSpinLockUnlock(&chip->reg_lock);
    
    for (i = 0; i < PCM_COUNT; ++i)
        if ((elapsed_streams & (1 << i)) && chip->streams[i])
   //         snd_pcm_period_elapsed(chip->streams[i]);
    
    if (status & OXYGEN_INT_SPDIF_IN_DETECT) {
        OSSpinLockLock(&chip->reg_lock);
        i = oxygen_read32(chip, OXYGEN_SPDIF_CONTROL);
        if (i & (OXYGEN_SPDIF_SENSE_INT | OXYGEN_SPDIF_LOCK_INT |
                 OXYGEN_SPDIF_RATE_INT)) {
            /* write the interrupt bit(s) to clear */
            oxygen_write32(chip, OXYGEN_SPDIF_CONTROL, i);
        //    schedule_work(&chip->spdif_input_bits_work);
        }
        OSSpinLockUnlock(&chip->reg_lock);
    }
    
    if (status & OXYGEN_INT_GPIO)
        //schedule_work(&chip->gpio_work);
    
    if (status & OXYGEN_INT_MIDI) {
       // if (chip->midi)
       //     _snd_mpu401_uart_interrupt(0, chip->midi->private_data);
       // else
            oxygen_read_uart(chip);
    }
    
    //if (status & OXYGEN_INT_AC97)
       // wake_up(&chip->ac97_waitqueue);
    
    //return IRQ_HANDLED;
}




void XonarAudioEngine::xonar_enable_output(struct oxygen *chip)
{
    struct xonar_generic *data =(struct xonar_generic*)   chip->model_data;
    
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, data->output_enable_bit);
    IODelay(data->anti_pop_delay*1000);
    oxygen_set_bits16(chip, OXYGEN_GPIO_DATA, data->output_enable_bit);
}

void XonarAudioEngine::xonar_disable_output(struct oxygen *chip)
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

void XonarAudioEngine::xonar_init_ext_power(struct oxygen *chip)
{
    struct xonar_generic *data = (struct xonar_generic*) chip->model_data;
    
    oxygen_set_bits8(chip, data->ext_power_int_reg,
                     data->ext_power_bit);
    chip->interrupt_mask |= OXYGEN_INT_GPIO;
   // chip->model.gpio_changed = xonar_ext_power_gpio_changed;
    data->has_power = !!(oxygen_read8(chip, data->ext_power_reg)
                         & data->ext_power_bit);
}

void XonarAudioEngine::xonar_init_cs53x1(struct oxygen *chip)
{
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_CS53x1_M_MASK);
    oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                          GPIO_CS53x1_M_SINGLE, GPIO_CS53x1_M_MASK);
}

void XonarAudioEngine::xonar_set_cs53x1_params(struct oxygen *chip)
{
    unsigned int value;
    
    if (this->getSampleRate()->whole <= 54000)
        value = GPIO_CS53x1_M_SINGLE;
    else if (this->getSampleRate()->whole <= 108000)
        value = GPIO_CS53x1_M_DOUBLE;
    else
        value = GPIO_CS53x1_M_QUAD;
    oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                          value, GPIO_CS53x1_M_MASK);
}
/*
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

static inline int oxygen_uart_input_ready(struct oxygen *chip)
{
    return !(oxygen_read8(chip, OXYGEN_MPU401 + 1) & MPU401_RX_EMPTY);
}

static void _write_uart(struct oxygen *chip, unsigned int port, UInt8 data)
{
    if (oxygen_read8(chip, OXYGEN_MPU401 + 1) & MPU401_TX_FULL)
        IODelay(1e3);
    oxygen_write8(chip, OXYGEN_MPU401 + port, data);
}

void XonarAudioEngine::oxygen_reset_uart(struct oxygen *chip)
{
    _write_uart(chip, 1, MPU401_RESET);
    IODelay(1e3); /* wait for ACK */
    _write_uart(chip, 1, MPU401_ENTER_UART);
}
//EXPORT_SYMBOL(oxygen_reset_uart);

void XonarAudioEngine::oxygen_write_uart(struct oxygen *chip, UInt8 data)
{
    _write_uart(chip, 0, data);
}
//EXPORT_SYMBOL(oxygen_write_uart);

static int oxygen_wait_spi(struct oxygen *chip)
{
    unsigned int count;
    
    /*
     * Higher timeout to be sure: 200 us;
     * actual transaction should not need more than 40 us.
     */
    for (count = 50; count > 0; count--) {
        IODelay(4);
        if ((oxygen_read8(chip, OXYGEN_SPI_CONTROL) &
             OXYGEN_SPI_BUSY) == 0)
            return 0;
    }
    dev_err(chip->card->dev, "oxygen: SPI wait timeout\n");
    return -EIO;
}

void XonarAudioEngine::oxygen_read_uart(struct oxygen *chip)
{
    /* no idea if there's an OSX equivalent for this. will look into it later.
    if (unlikely(!oxygen_uart_input_ready(chip))) {
        // no data, but read it anyway to clear the interrupt
        oxygen_read8(chip, OXYGEN_MPU401);
        return;
    }*/
    do {
        UInt8 data = oxygen_read8(chip, OXYGEN_MPU401);
        if (data == MPU401_ACK)
            continue;
        if (chip->uart_input_count >= ARRAY_SIZE(chip->uart_input))
            chip->uart_input_count = 0;
        chip->uart_input[chip->uart_input_count++] = data;
    } while (oxygen_uart_input_ready(chip));
    if (chip->model.uart_input)
        chip->model.uart_input(chip);
}

int oxygen_write_spi(struct oxygen *chip, UInt8 control, unsigned int data)
{
    /*
     * We need to wait AFTER initiating the SPI transaction,
     * otherwise read operations will not work.
     */
    oxygen_write8(chip, OXYGEN_SPI_DATA1, data);
    oxygen_write8(chip, OXYGEN_SPI_DATA2, data >> 8);
    if (control & OXYGEN_SPI_DATA_LENGTH_3)
        oxygen_write8(chip, OXYGEN_SPI_DATA3, data >> 16);
    oxygen_write8(chip, OXYGEN_SPI_CONTROL, control);
    return oxygen_wait_spi(chip);
}
//EXPORT_SYMBOL(oxygen_write_spi);



static inline void pcm1796_write_spi(struct oxygen *chip, unsigned int codec,
                                     UInt8 reg, UInt8 value)
{
    /* maps ALSA channel pair number to SPI output */
    static const UInt8 codec_map[4] = {
        0, 1, 2, 4
    };
  
    oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER  |
                     OXYGEN_SPI_DATA_LENGTH_2 |
                     OXYGEN_SPI_CLOCK_160 |
                     (codec_map[codec] << OXYGEN_SPI_CODEC_SHIFT) |
                     OXYGEN_SPI_CEN_LATCH_CLOCK_HI,
                     (reg << 8) | value); 
}



void oxygen_write_i2c(struct oxygen *chip, UInt8 device, UInt8 map, UInt8 data)
{
    /* should not need more than about 300 us */
    IODelay(1000);
    
    oxygen_write8(chip, OXYGEN_2WIRE_MAP, map);
    oxygen_write8(chip, OXYGEN_2WIRE_DATA, data);
    oxygen_write8(chip, OXYGEN_2WIRE_CONTROL,
                  device | OXYGEN_2WIRE_DIR_WRITE);
}
//EXPORT_SYMBOL(oxygen_write_i2c);

static inline void pcm1796_write_i2c(struct oxygen *chip, unsigned int codec,
                                     UInt8 reg, UInt8 value)
{
    oxygen_write_i2c(chip, I2C_DEVICE_PCM1796(codec), reg, value);
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
    
    oxygen_write_i2c(chip, I2C_DEVICE_CS2000, reg, value);
    data->cs2000_regs[reg] = value;
}

static void cs2000_write_cached(struct oxygen *chip, UInt8 reg, UInt8 value)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*)chip->model_data;
    
    if (value != data->cs2000_regs[reg])
        cs2000_write(chip, reg, value);
}

void XonarAudioEngine::pcm1796_registers_init(struct oxygen *chip)
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

void XonarAudioEngine::pcm1796_init(struct oxygen *chip)
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
    data->current_rate->whole = 48000;
}

void XonarAudioEngine::cs2000_registers_init(struct oxygen *chip)
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


void XonarAudioEngine::update_pcm1796_oversampling(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
    unsigned int i;
    UInt8 reg;
    
    if (data->current_rate->whole <= 48000 && !data->h6)
        reg = PCM1796_OS_128;
    else
        reg = PCM1796_OS_64;
    for (i = 0; i < data->dacs; ++i)
        pcm1796_write_cached(chip, i, 20, reg);
}

void XonarAudioEngine::set_pcm1796_params(struct oxygen *chip, XonarAudioEngine *instance)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
    
    IODelay(1*1000);
    //hopefully OSX getSampleRate() for IOAudioEngine objects has
    // similar behaviour to linux's params_rate...
    data->current_rate = (IOAudioSampleRate *) instance->getSampleRate();
   // data->current_rate = params_rate(params);
    update_pcm1796_oversampling(chip);
}

void XonarAudioEngine::update_pcm1796_volume(struct oxygen *chip)
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

void XonarAudioEngine::update_pcm1796_mute(struct oxygen *chip)
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


void XonarAudioEngine::update_cs2000_rate(struct oxygen *chip, unsigned int rate)
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
//}

void XonarAudioEngine::xonar_line_mic_ac97_switch(struct oxygen *chip,
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


int XonarAudioEngine::add_pcm1796_controls(struct oxygen *chip)
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

bool XonarAudioEngine::init(struct oxygen *chip, int model)
{
    bool result = false;
    
    IOLog("XonarAudioEngine[%p]::init(%p)\n", this, chip);
    
    if (!chip) {
        goto Done;
    }
    
    if (!super::init(NULL)) {
        goto Done;
    }
  //  ak4396_init(chip);
  //  wm8785_init(chip);
//    if(model == HDAV_MODEL)
//        struct xonar_hdav *deviceRegisters = (struct xonar_hdav*)chip->model_data;
//    else
//        struct xonar_generic *deviceRegisters = (struct xonar_generic*)chip->model_data;
    // the below aren't correct. have to bridge the workqueue calls to IOWorkLoop
    queue_init(&chip->ac97_waitqueue);
    chip->mutex = OS_SPINLOCK_INIT;
    
    result = true;
    
Done:
    
    return result;
}

bool XonarAudioEngine::initHardware(IOService *provider)
{
    bool result = false;
    IOAudioSampleRate initialSampleRate;
    IOAudioStream *audioStream;
   // IOWorkLoop *workLoop;
    
    IOLog("XonarAudioEngine[%p]::initHardware(%p)\n", this, provider);
    
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
    interruptEventSource = IOInterruptEventSource::interruptEventSource(this,
                                                                                    (IOInterruptEventAction)&oxygen_interrupt,
                                                                                                                                                                       audioDevice->getProvider());
    workLoop->addEventSource(interruptEventSource);
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

void XonarAudioEngine::free()
{
    IOLog("XonarAudioEngine[%p]::free()\n", this);
    
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

IOAudioStream *XonarAudioEngine::createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize)
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

void XonarAudioEngine::stop(IOService *provider)
{
    IOLog("XonarAudioEngine[%p]::stop(%p)\n", this, provider);
    
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

IOReturn XonarAudioEngine::performAudioEngineStart()
{
    IOLog("XonarAudioEngine[%p]::performAudioEngineStart()\n", this);
    
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

IOReturn XonarAudioEngine::performAudioEngineStop()
{
    IOLog("XonarAudioEngine[%p]::performAudioEngineStop()\n", this);
    
    // Assuming we don't need interrupts after stopping the audio engine, we can disable them here
    assert(interruptEventSource);
    interruptEventSource->disable();
    
    // Add audio - I/O stop code here
    
#error performAudioEngineStop() - driver will not work until audio engine stop code is added
    
    return kIOReturnSuccess;
}

UInt32 XonarAudioEngine::getCurrentSampleFrame()
{
    IOLog("XonarAudioEngine[%p]::getCurrentSampleFrame()\n", this);
    
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

IOReturn XonarAudioEngine::performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate)
{
    IOLog("XonarAudioEngine[%p]::peformFormatChange(%p, %p, %p)\n", this, audioStream, newFormat, newSampleRate);
    
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


void XonarAudioEngine::interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count)
{
    // Since our interrupt filter always returns false, this function will never be called
    // If the filter returned true, this function would be called on the IOWorkLoop
    return;
}

bool XonarAudioEngine::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source)
{
    XonarAudioEngine *audioEngine = OSDynamicCast(XonarAudioEngine, owner);
    
    // We've cast the audio engine from the owner which we passed in when we created the interrupt
    // event source
    if (audioEngine) {
        // Then, filterInterrupt() is called on the specified audio engine
        audioEngine->filterInterrupt(source->getIntIndex());
    }
    
    return false;
}

void XonarAudioEngine::filterInterrupt(int index)
{
    // In the case of our simple device, we only get interrupts when the audio engine loops to the
    // beginning of the buffer.  When that happens, we need to take a timestamp and increment
    // the loop count.  The function takeTimeStamp() does both of those for us.  Additionally,
    // if a different timestamp is to be used (other than the current time), it can be passed
    // in to takeTimeStamp()
    takeTimeStamp();
}

