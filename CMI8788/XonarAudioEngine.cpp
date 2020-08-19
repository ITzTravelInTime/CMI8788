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


#include "ak4396.h"
//#include "wm8785.h"

//#include <libkern/locks.h>
//#include <architecture/i386/pio.h>
#include "XonarAudioEngine.hpp"
#include "pcm1796.h"
#include "cm9780.h"
#include "cs2000.h"
#include "cs4398.h"
#include "cs4362a.h"
#include "ac97.h"

#include <IOKit/IOLib.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#define INITIAL_SAMPLE_RATE	44100
#define NUM_SAMPLE_FRAMES	16384
#define NUM_CHANNELS		2
#define BIT_DEPTH			16

#include "hexdumpfn.c"
#define super IOAudioEngine

OSDefineMetaClassAndStructors(XonarAudioEngine, IOAudioEngine)

//TODO: Fix division by zero
static inline unsigned long msecs_to_jiffies(const unsigned int m)
{
    return (m + (MSEC_PER_SEC / HZ) - 1) / (MSEC_PER_SEC / HZ);
}

//static kern_return_t check_ac97_status(struct oxygen *chip)
//{
//    UInt8 statusbits = 0;
//    //chip->ac97_maskval = maskval;
//    statusbits |= oxygen_read8(chip, OXYGEN_AC97_INTERRUPT_STATUS);
//    statusbits = statusbits & chip->ac97_maskval;
//    return statusbits;
//}

void XonarAudioEngine::xonar_enable_output(struct oxygen *chip)
{
    struct xonar_generic *data =(struct xonar_generic*) chip->model_data;
    
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
            kprintf("power restored\n");
        } else {
            kprintf("Hey! Don't unplug the power cable!\n");
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
    chip->model.gpio_changed = xonar_ext_power_gpio_changed;
    data->has_power = !!(oxygen_read8(chip, data->ext_power_reg)
                         & data->ext_power_bit);
}

void XonarAudioEngine::xonar_init_cs53x1(struct oxygen *chip)
{
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_CS53x1_M_MASK);
    oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                          GPIO_CS53x1_M_SINGLE, GPIO_CS53x1_M_MASK);
}

void XonarAudioEngine::xonar_set_cs53x1_params(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    unsigned int value;
    
    if (audioEngine->getSampleRate()->whole <= 54000)
        value = GPIO_CS53x1_M_SINGLE;
    else if (audioEngine->getSampleRate()->whole <= 108000)
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
    kprintf("oxygen: SPI wait timeout\n");
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

int XonarAudioEngine::oxygen_write_spi(struct oxygen *chip, UInt8 control, unsigned int data)
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



inline void XonarAudioEngine::pcm1796_write_spi(struct oxygen *chip, unsigned int codec,
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



void XonarAudioEngine::oxygen_write_i2c(struct oxygen *chip, UInt8 device, UInt8 map, UInt8 data)
{
    /* should not need more than about 300 us */
    IODelay(1000);
    
    oxygen_write8(chip, OXYGEN_2WIRE_MAP, map);
    oxygen_write8(chip, OXYGEN_2WIRE_DATA, data);
    oxygen_write8(chip, OXYGEN_2WIRE_CONTROL,
                  device | OXYGEN_2WIRE_DIR_WRITE);
}

inline void XonarAudioEngine::pcm1796_write_i2c(struct oxygen *chip, unsigned int codec,
                                                UInt8 reg, UInt8 value)
{
    oxygen_write_i2c(chip, I2C_DEVICE_PCM1796(codec), reg, value);
}

void XonarAudioEngine::pcm1796_write(struct oxygen *chip, unsigned int codec,
                                     UInt8 reg, UInt8 value)
{
    struct xonar_pcm179x *data;
    if(chip->card_model == HDAV_MODEL)
        data = (struct xonar_pcm179x*) &((struct xonar_hdav*) chip->model_data)->pcm179x;
    else
        data = (struct xonar_pcm179x*) chip->model_data;
    
    if ((chip->model.function_flags & OXYGEN_FUNCTION_2WIRE_SPI_MASK) ==
        OXYGEN_FUNCTION_SPI) {
        pcm1796_write_spi(chip, codec, reg, value);
    }
    else
        pcm1796_write_i2c(chip, codec, reg, value);

    if ((unsigned int)(reg - PCM1796_REG_BASE)
        < ARRAY_SIZE(data->pcm1796_regs[codec])) {
        data->pcm1796_regs[codec][reg - PCM1796_REG_BASE] = value;
    }
}

void XonarAudioEngine::pcm1796_write_cached(struct oxygen *chip, unsigned int codec,
                                            UInt8 reg, UInt8 value)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
    
    if (value != data->pcm1796_regs[codec][reg - PCM1796_REG_BASE])
        pcm1796_write(chip, codec, reg, value);
}

void XonarAudioEngine::pcm1796_registers_init(struct oxygen *chip)
{
    struct xonar_pcm179x *data;
    if(chip->card_model == HDAV_MODEL)
        data = (struct xonar_pcm179x*) &((struct xonar_hdav*) chip->model_data)->pcm179x;
    else
        data = (struct xonar_pcm179x*) chip->model_data;
    
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
    struct xonar_pcm179x *data;
    if(chip->card_model == HDAV_MODEL)
        data = (struct xonar_pcm179x*) &((struct xonar_hdav*) chip->model_data)->pcm179x;
    else
        data = (struct xonar_pcm179x*) chip->model_data;
    
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

void XonarAudioEngine::cs2000_write(struct oxygen *chip, UInt8 reg, UInt8 value)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
    
    oxygen_write_i2c(chip, I2C_DEVICE_CS2000, reg, value);
    data->cs2000_regs[reg] = value;
}

void XonarAudioEngine::cs2000_write_cached(struct oxygen *chip, UInt8 reg, UInt8 value)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
    
    if (value != data->cs2000_regs[reg])
        cs2000_write(chip, reg, value);
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

void XonarAudioEngine::update_pcm1796_volume(struct oxygen *chip, XonarAudioEngine *audioEngine)
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

void XonarAudioEngine::update_pcm1796_mute(struct oxygen *chip, XonarAudioEngine *audioEngine)
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
//    IOLockLock(&chip->mutex);
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
//    IOLockUnlock(&chip->mutex);
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
 
static int oxygen_ac97_wait(struct oxygen *chip, unsigned int mask)
{
    UInt8 status = 0;
    wait_result_t retval;
    /*
     * Reading the status register also clears the bits, so we have to save
     * the read bits in status.
     */
    //chip->ac97_maskval = mask;
    IOLockLock(chip->ac97_mutex);
    while( ({status |= oxygen_read8(chip, OXYGEN_AC97_INTERRUPT_STATUS);
        status & mask;}) ) {
    //pthread_cond_timedwait(&chip->ac97_condition,&chip->ac97_mutex,&chip->ac97_timeout);
            retval = IOLockSleepDeadline(chip->ac97_mutex, &chip->ac97_statusbits, 1e6, THREAD_UNINT);
        if(!retval)
            kprintf("ac97_thread was awakened!\n");
        else if(retval == 1)
            kprintf("ac97_thread timed out\n");
        else if (retval == -1)
            kprintf ("ac97_thread waiting...\n");
            
    }
    IOLockUnlock(chip->ac97_mutex);
    //thread_block(THREAD_CONTINUE_NULL);
    /*
     * Check even after a timeout because this function should not require
     * the AC'97 interrupt to be enabled.
     */
    status |= oxygen_read8(chip, OXYGEN_AC97_INTERRUPT_STATUS);
    return status & mask ? 0 : -EIO;
}


void oxygen_write_ac97(struct oxygen *chip, unsigned int codec,
                       unsigned int index, UInt16 data)
{
    unsigned int count, succeeded;
    UInt32 reg;
    
    reg = data;
    reg |= index << OXYGEN_AC97_REG_ADDR_SHIFT;
    reg |= OXYGEN_AC97_REG_DIR_WRITE;
    reg |= codec << OXYGEN_AC97_REG_CODEC_SHIFT;
    succeeded = 0;
    for (count = 5; count > 0; --count) {
        IODelay(5);
        oxygen_write32(chip, OXYGEN_AC97_REGS, reg);
        /* require two "completed" writes, just to be sure */
        if (oxygen_ac97_wait(chip, OXYGEN_AC97_INT_WRITE_DONE) >= 0 &&
            ++succeeded >= 2) {
            chip->saved_ac97_registers[codec][index / 2] = data;
            return;
        }
    }
    kprintf("AC'97 write timeout\n");
}
//EXPORT_SYMBOL(oxygen_write_ac97);

UInt16 oxygen_read_ac97(struct oxygen *chip, unsigned int codec,
                        unsigned int index)
{
    unsigned int count;
    unsigned int last_read = UINT_MAX;
    UInt32 reg;
    
    reg = index << OXYGEN_AC97_REG_ADDR_SHIFT;
    reg |= OXYGEN_AC97_REG_DIR_READ;
    reg |= codec << OXYGEN_AC97_REG_CODEC_SHIFT;
    for (count = 5; count > 0; --count) {
        IODelay(5);
        oxygen_write32(chip, OXYGEN_AC97_REGS, reg);
        IODelay(10);
        if (oxygen_ac97_wait(chip, OXYGEN_AC97_INT_READ_DONE) >= 0) {
            UInt16 value = oxygen_read16(chip, OXYGEN_AC97_REGS);
            /* we require two consecutive reads of the same value */
            if (value == last_read)
                return value;
            last_read = value;
            /*
             * Invert the register value bits to make sure that two
             * consecutive unsuccessful reads do not return the same
             * value.
             */
            reg ^= 0xffff;
        }
    }
    kprintf("AC'97 read timeout on codec %u\n", codec);
    return 0;
}

void oxygen_write_ac97_masked(struct oxygen *chip, unsigned int codec,
                              unsigned int index, UInt16 data, UInt16 mask)
{
    UInt16 value = oxygen_read_ac97(chip, codec, index);
    value &= ~mask;
    value |= data & mask;
    oxygen_write_ac97(chip, codec, index, value);
}



static inline void oxygen_ac97_set_bits(struct oxygen *chip, unsigned int codec,
                                        unsigned int index, UInt16 value)
{
    oxygen_write_ac97_masked(chip, codec, index, value, value);
}

static inline void oxygen_ac97_clear_bits(struct oxygen *chip,
                                          unsigned int codec,
                                          unsigned int index, UInt16 value)
{
    oxygen_write_ac97_masked(chip, codec, index, 0, value);
}



void XonarAudioEngine::xonar_line_mic_ac97_switch(struct oxygen *chip,
                                                  unsigned int reg, unsigned int mute)
{
    if (reg == AC97_LINE) {
        IOSimpleLockLock(chip->reg_lock);
        oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                              mute ? GPIO_INPUT_ROUTE : 0,
                              GPIO_INPUT_ROUTE);
        IOSimpleLockUnlock(chip->reg_lock);
    }
}


int XonarAudioEngine::add_pcm1796_controls(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
    int err;
    
    /*if (!data->broken_i2c) {
     err = snd_ctl_add(chip->card,
     snd_ctl_new1(&rolloff_control, chip));
     if (err < 0)
     return err;
     }*/
    return 0;
}


void XonarAudioEngine::hdmi_write_command(struct oxygen *chip, UInt8 command,
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

void XonarAudioEngine::xonar_hdmi_init_commands(struct oxygen *chip,
                                                struct xonar_hdmi *hdmi)
{
    UInt8 param;
    
    oxygen_reset_uart(chip);
    param = 0;
    hdmi_write_command(chip, 0x61, 1, &param);
    param = 1;
    hdmi_write_command(chip, 0x74, 1, &param);
    hdmi_write_command(chip, 0x54, 5, hdmi->params);
}


void XonarAudioEngine::xonar_hdmi_init(struct oxygen *chip, struct xonar_hdmi *hdmi)
{
    hdmi->params[1] = IEC958_AES3_CON_FS_48000;
    hdmi->params[4] = 1;
    xonar_hdmi_init_commands(chip, hdmi);
}

void XonarAudioEngine::xonar_hdmi_cleanup(struct oxygen *chip)
{
    UInt8 param = 0;
    
    hdmi_write_command(chip, 0x74, 1, &param);
}

void XonarAudioEngine::xonar_hdmi_resume(struct oxygen *chip, struct xonar_hdmi *hdmi)
{
    xonar_hdmi_init_commands(chip, hdmi);
}

void XonarAudioEngine::xonar_set_hdmi_params(struct oxygen *chip, struct xonar_hdmi *hdmi)
{
    hdmi->params[0] = 0; // 1 = non-audio
    switch (getSampleRate()->whole) {
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
    //Linux call:
    //hdmi->params[2] = params_channels(params) / 2 - 1;
    //Mac Call:
    hdmi->params[2] = inputs[0]->maxNumChannels / 2 - 1;
    //^ this is wrong because it should be NumChannels, not MaxNum
    //however since IOAudioStream calls are deprecated as of 10.10,
    //i'm going to use this is a placeholder/semi-correct call.
    
    //Linux call:
    //if (params_format(params) == SNDRV_PCM_FORMAT_S16_LE)
    //Mac Call:
    if(inputs[0]->format.fSampleFormat == SNDRV_PCM_FORMAT_S16_LE)
        hdmi->params[3] = 0;
    else
        hdmi->params[3] = 0xc0;
    hdmi->params[4] = 1; // ?
    hdmi_write_command(chip, 0x54, 5, hdmi->params);
}


void XonarAudioEngine::xonar_hdmi_uart_input(struct oxygen *chip)
{
    if (chip->uart_input_count >= 2 &&
        chip->uart_input[chip->uart_input_count - 2] == 'O' &&
        chip->uart_input[chip->uart_input_count - 1] == 'K') {
        kprintf("message from HDMI chip received:\n");
        print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
                             chip->uart_input, chip->uart_input_count);
        chip->uart_input_count = 0;
    }
}


bool XonarAudioEngine::init(struct oxygen *chip, int model)
{
    /*this function can be looked at as oxygen_pci_probe, with
     *the differences being:
     1. there are no model detection function pointers
     -this means we have to "hard code" the model instantiation
     code using a simple if statement.
     2. after the registers for the requested model are set,
     the code directly after is the content of oxygen_init
     3. still figuring out how to add the gpio/spdifwork to the IOWorkLoopQueue
     -theoretically, we may only need a single IOWorkLoop instance that
     belongs to either the PCIAudioDevice/XonarAudioEngine class, and therefore
     may only need to add {gpio,spdif_input_bits}_changed to this single workloop
     */
    bool result = false;
    
    kprintf("XonarAudioEngine::init\n");
    
    if (!chip) {
        goto Done;
    }
    
    if (!super::init(NULL)) {
        goto Done;
    }
    //compare the subdevice id with known models'
    if(model == HDAV_MODEL) {
        //code from get_xonar_pcm179x_model portions (case 0x8314)
        /*these are the relevant attributes from the xonar_model_hdav struct.
         *some of the function pointers in the xonar_model struct may not be needed
         *as OSX has its own way of instantiating volume controls/mixers. however,
         *the code comprising the functions which the struct is pointing to, may still be
         *very useful (and indeed:necessary)
         */
        chip->model.update_dac_volume = this->update_pcm1796_volume;
        chip->model.update_dac_mute = this->update_pcm1796_mute;
        chip->model.model_data_size = sizeof(struct xonar_hdav);
        chip->model.device_config = PLAYBACK_0_TO_I2S |
        PLAYBACK_1_TO_SPDIF |
        CAPTURE_0_FROM_I2S_2 |
        CAPTURE_1_FROM_SPDIF;
        chip->model.dac_channels_pcm = 8;
        chip->model.dac_channels_mixer = 2;
        chip->model.dac_volume_min = 255 - 2*60;
        chip->model.dac_volume_max = 255;
        chip->model.misc_flags = OXYGEN_MISC_MIDI;
        chip->model.function_flags = OXYGEN_FUNCTION_2WIRE;
        chip->model.dac_mclks = OXYGEN_MCLKS(512, 128, 128);
        chip->model.adc_mclks = OXYGEN_MCLKS(256, 128, 128);
        chip->model.dac_i2s_format = OXYGEN_I2S_FORMAT_I2S;
        chip->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
        chip->model.ac97_switch = this->xonar_line_mic_ac97_switch;
        oxygen_clear_bits16(chip,OXYGEN_GPIO_CONTROL,GPIO_DB_MASK);
        switch (oxygen_read16(chip, OXYGEN_GPIO_DATA) & GPIO_DB_MASK) {
            default:
                chip->model.shortname = "Xonar HDAV1.3";
                break;
            case GPIO_DB_H6:
                chip->model.shortname = "Xonar HDAV1.3+H6";
                chip->model.dac_channels_mixer = 8;
                chip->model.dac_mclks = OXYGEN_MCLKS(256, 128, 128);
                break;
        }
        chip->model.uart_input = xonar_hdmi_uart_input;
        
        
    }
    else if (model == ST_MODEL || model == STX_MODEL ||
             model == STX2_MODEL || model == XENSE_MODEL) {
        chip->model.model_data_size = sizeof(struct xonar_pcm179x);
        chip->model.device_config = PLAYBACK_0_TO_I2S |
        PLAYBACK_1_TO_SPDIF |
        CAPTURE_0_FROM_I2S_2 |
        CAPTURE_1_FROM_SPDIF |
        AC97_FMIC_SWITCH;
        chip->model.dac_channels_pcm = 2;
        chip->model.dac_channels_mixer = 2;
        chip->model.dac_volume_min = 255 - 2*60;
        chip->model.dac_volume_max = 255;
        chip->model.function_flags = OXYGEN_FUNCTION_2WIRE;
        chip->model.dac_mclks = OXYGEN_MCLKS(512, 128, 128);
        chip->model.adc_mclks = OXYGEN_MCLKS(256, 128, 128);
        chip->model.dac_i2s_format = OXYGEN_I2S_FORMAT_I2S;
        chip->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
        chip->model.ac97_switch = this->xonar_line_mic_ac97_switch;
        
        //0x835d
        if(model == ST_MODEL) {
            oxygen_clear_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_DB_MASK);
            switch (oxygen_read16(chip, OXYGEN_GPIO_DATA) & GPIO_DB_MASK) {
                default:
                    chip->model.shortname = "Xonar ST";
                    break;
                case GPIO_DB_H6:
                    chip->model.shortname = "Xonar ST+H6";
                    //still have to figure out all the controls; commenting this
                    //out for now.
                    //chip->model.control_filter = xonar_st_h6_control_filter;
                    chip->model.dac_channels_pcm = 8;
                    chip->model.dac_channels_mixer = 8;
                    chip->model.dac_volume_min = 255;
                    chip->model.dac_mclks = OXYGEN_MCLKS(256, 128, 128);
                    break;
            }
        }
        //end 0x835d
        
        //0x835c
        else if(model == STX_MODEL) {
            chip->model.shortname = "Xonar STX";
            // not sure if we'll need the two lines below */
            /*chip->model.resume = xonar_stx_resume;
             chip->model.set_dac_params = set_pcm1796_params;*/
        }
        //end 0x835c
        
        //0x85f4
        else if(model == STX2_MODEL) {
            oxygen_clear_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_DB_MASK);
            switch (oxygen_read16(chip, OXYGEN_GPIO_DATA) & GPIO_DB_MASK) {
                default:
                    chip->model.shortname = "Xonar STX II";
                    break;
                case GPIO_DB_H6:
                    chip->model.shortname = "Xonar STX II+H6";
                    chip->model.dac_channels_pcm = 8;
                    chip->model.dac_channels_mixer = 8;
                    chip->model.dac_mclks = OXYGEN_MCLKS(256, 128, 128);
                    break;
            }
            //similar to what i stated in 0x835c: not sure if we'll need these
            //yet (if we do, they'll be set in the STAudioengineclass file, not here)
            /*chip->model.resume = xonar_stx_resume;
             chip->model.set_dac_params = set_pcm1796_params;*/
        }
        //end 0x85f4
        
        //0x8428
        else if(model == XENSE_MODEL) {
            chip->model.shortname = "Xonar Xense";
            //not sure if we'll need the line below yet
            //chip->model.mixer_init = xonar_xense_mixer_init;
            
        }
        //end 0x8428
        
    }
    else if (model == D2_MODEL || model == D2X_MODEL) {
        chip->model.model_data_size = sizeof(struct xonar_pcm179x);
        chip->model.device_config = PLAYBACK_0_TO_I2S |
        PLAYBACK_1_TO_SPDIF |
        CAPTURE_0_FROM_I2S_2 |
        CAPTURE_1_FROM_SPDIF |
        MIDI_OUTPUT |
        MIDI_INPUT |
        AC97_CD_INPUT;
        chip->model.dac_channels_pcm = 8;
        chip->model.dac_channels_mixer = 8;
        chip->model.dac_volume_min = 255 - 2*60;
        chip->model.dac_volume_max = 255;
        chip->model.misc_flags = OXYGEN_MISC_MIDI;
        chip->model.function_flags = OXYGEN_FUNCTION_SPI |
        OXYGEN_FUNCTION_ENABLE_SPI_4_5;
        chip->model.dac_mclks = OXYGEN_MCLKS(512, 128, 128);
        chip->model.adc_mclks = OXYGEN_MCLKS(256, 128, 128);
        chip->model.dac_i2s_format = OXYGEN_I2S_FORMAT_I2S;
        chip->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
        
        //0x8269
        if(model == D2_MODEL)
            chip->model.shortname = "Xonar D2";
        //end 0x8269
        
        //0x82b7
        else if (model == D2X_MODEL)
            chip->model.shortname = "Xonar D2X";
        //end 0x82b7
        
    }
    else if (model == XONAR_GENERIC){ // generic
        chip->model.model_data_size = sizeof(struct generic_data);
        chip->model.device_config = PLAYBACK_0_TO_I2S |
        PLAYBACK_1_TO_SPDIF |
        PLAYBACK_2_TO_AC97_1 |
        CAPTURE_0_FROM_I2S_1 |
        CAPTURE_1_FROM_SPDIF |
        CAPTURE_2_FROM_AC97_1 |
        AC97_CD_INPUT;
        chip->model.dac_channels_pcm = 8;
        chip->model.dac_channels_mixer = 8;
        chip->model.dac_volume_min = 0;
        chip->model.dac_volume_max = 255;
        chip->model.function_flags = OXYGEN_FUNCTION_SPI |
        OXYGEN_FUNCTION_ENABLE_SPI_4_5;
        chip->model.dac_mclks = OXYGEN_MCLKS(256, 128, 128);
        chip->model.adc_mclks = OXYGEN_MCLKS(256, 256, 128);
        chip->model.dac_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
        chip->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
        
    }
    else if (model == DS_MODEL || model == DSX_MODEL ||
             model == HDAV_SLIM) {
        if(model == DS_MODEL || model == DSX_MODEL){
            /*for all submodels, this manual assignment
             * mimics chip->model = <struct model name>
             */
            chip->model.model_data_size = sizeof(struct xonar_wm87x6);
            chip->model.device_config = PLAYBACK_0_TO_I2S |
            PLAYBACK_1_TO_SPDIF |
            CAPTURE_0_FROM_I2S_1 |
            CAPTURE_1_FROM_SPDIF;
            chip->model.dac_channels_pcm = 8;
            chip->model.dac_channels_mixer = 8;
            chip->model.dac_volume_min = 255 - 2*60;
            chip->model.dac_volume_max = 255;
            chip->model.function_flags = OXYGEN_FUNCTION_SPI;
            chip->model.dac_mclks = OXYGEN_MCLKS(256, 256, 128);
            chip->model.adc_mclks = OXYGEN_MCLKS(256, 256, 128);
            chip->model.dac_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
            chip->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
            /*end chip->model struct assignment */
        }
        
        else if (model == HDAV_SLIM) {
            chip->model.model_data_size = sizeof(struct xonar_wm87x6);
            chip->model.device_config = PLAYBACK_0_TO_I2S |
            PLAYBACK_1_TO_SPDIF |
            CAPTURE_0_FROM_I2S_1 |
            CAPTURE_1_FROM_SPDIF;
            chip->model.dac_channels_pcm = 8;
            chip->model.dac_channels_mixer = 2;
            chip->model.dac_volume_min = 255 - 2*60;
            chip->model.dac_volume_max = 255;
            chip->model.function_flags = OXYGEN_FUNCTION_2WIRE;
            chip->model.dac_mclks = OXYGEN_MCLKS(256, 256, 128);
            chip->model.adc_mclks = OXYGEN_MCLKS(256, 256, 128);
            chip->model.dac_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
            chip->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
        }
    }
    else {
        chip->model.model_data_size = sizeof(struct xonar_cs43xx);
        chip->model.device_config = PLAYBACK_0_TO_I2S |
        PLAYBACK_1_TO_SPDIF |
        CAPTURE_0_FROM_I2S_2 |
        CAPTURE_1_FROM_SPDIF |
        AC97_FMIC_SWITCH;
        chip->model.dac_channels_pcm = 8;
        chip->model.dac_channels_mixer = 8;
        chip->model.dac_volume_min = 127 - 60;
        chip->model.dac_volume_max = 127;
        chip->model.function_flags = OXYGEN_FUNCTION_2WIRE;
        chip->model.dac_mclks = OXYGEN_MCLKS(256, 128, 128);
        chip->model.adc_mclks = OXYGEN_MCLKS(256, 128, 128);
        chip->model.dac_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
        chip->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
        
    }
  
      
    //begin oxygen_init
    unsigned int i;
    
    chip->dac_routing = 1;
    for (i = 0; i < 8; ++i)
        chip->dac_volume[i] = chip->model.dac_volume_min;
    chip->dac_mute = 1;
    chip->spdif_playback_enable = 1;
    chip->spdif_bits = OXYGEN_SPDIF_C | OXYGEN_SPDIF_ORIGINAL |
    (IEC958_AES1_CON_PCM_CODER << OXYGEN_SPDIF_CATEGORY_SHIFT);
    chip->spdif_pcm_bits = chip->spdif_bits;

    if (!(oxygen_read8(chip, OXYGEN_REVISION) & OXYGEN_REVISION_2))
        oxygen_set_bits8(chip, OXYGEN_MISC,
                         OXYGEN_MISC_PCI_MEM_W_1_CLOCK);
    
    i = oxygen_read16(chip, OXYGEN_AC97_CONTROL);
    chip->has_ac97_0 = (i & OXYGEN_AC97_CODEC_0) != 0;
    chip->has_ac97_1 = (i & OXYGEN_AC97_CODEC_1) != 0;
    
    oxygen_write8_masked(chip, OXYGEN_FUNCTION,
                         OXYGEN_FUNCTION_RESET_CODEC |
                         chip->model.function_flags,
                         OXYGEN_FUNCTION_RESET_CODEC |
                         OXYGEN_FUNCTION_2WIRE_SPI_MASK |
                         OXYGEN_FUNCTION_ENABLE_SPI_4_5);
    oxygen_write8(chip, OXYGEN_DMA_STATUS, 0);
    oxygen_write8(chip, OXYGEN_DMA_PAUSE, 0);
    oxygen_write8(chip, OXYGEN_PLAY_CHANNELS,
                  OXYGEN_PLAY_CHANNELS_2 |
                  OXYGEN_DMA_A_BURST_8 |
                  OXYGEN_DMA_MULTICH_BURST_8);
    oxygen_write16(chip, OXYGEN_INTERRUPT_MASK, 0);
    oxygen_write8_masked(chip, OXYGEN_MISC,
                         chip->model.misc_flags,
                         OXYGEN_MISC_WRITE_PCI_SUBID |
                         OXYGEN_MISC_REC_C_FROM_SPDIF |
                         OXYGEN_MISC_REC_B_FROM_AC97 |
                         OXYGEN_MISC_REC_A_FROM_MULTICH |
                         OXYGEN_MISC_MIDI);
    oxygen_write8(chip, OXYGEN_REC_FORMAT,
                  (OXYGEN_FORMAT_16 << OXYGEN_REC_FORMAT_A_SHIFT) |
                  (OXYGEN_FORMAT_16 << OXYGEN_REC_FORMAT_B_SHIFT) |
                  (OXYGEN_FORMAT_16 << OXYGEN_REC_FORMAT_C_SHIFT));
    oxygen_write8(chip, OXYGEN_PLAY_FORMAT,
                  (OXYGEN_FORMAT_16 << OXYGEN_SPDIF_FORMAT_SHIFT) |
                  (OXYGEN_FORMAT_16 << OXYGEN_MULTICH_FORMAT_SHIFT));
    oxygen_write8(chip, OXYGEN_REC_CHANNELS, OXYGEN_REC_CHANNELS_2_2_2);
    oxygen_write16(chip, OXYGEN_I2S_MULTICH_FORMAT,
                   OXYGEN_RATE_48000 |
                   chip->model.dac_i2s_format |
                   OXYGEN_I2S_MCLK(chip->model.dac_mclks) |
                   OXYGEN_I2S_BITS_16 |
                   OXYGEN_I2S_MASTER |
                   OXYGEN_I2S_BCLK_64);
    if (chip->model.device_config & CAPTURE_0_FROM_I2S_1)
        oxygen_write16(chip, OXYGEN_I2S_A_FORMAT,
                       OXYGEN_RATE_48000 |
                       chip->model.adc_i2s_format |
                       OXYGEN_I2S_MCLK(chip->model.adc_mclks) |
                       OXYGEN_I2S_BITS_16 |
                       OXYGEN_I2S_MASTER |
                       OXYGEN_I2S_BCLK_64);
    else
        oxygen_write16(chip, OXYGEN_I2S_A_FORMAT,
                       OXYGEN_I2S_MASTER |
                       OXYGEN_I2S_MUTE_MCLK);
    if (chip->model.device_config & (CAPTURE_0_FROM_I2S_2 |
                                     CAPTURE_2_FROM_I2S_2))
        oxygen_write16(chip, OXYGEN_I2S_B_FORMAT,
                       OXYGEN_RATE_48000 |
                       chip->model.adc_i2s_format |
                       OXYGEN_I2S_MCLK(chip->model.adc_mclks) |
                       OXYGEN_I2S_BITS_16 |
                       OXYGEN_I2S_MASTER |
                       OXYGEN_I2S_BCLK_64);
    else
        oxygen_write16(chip, OXYGEN_I2S_B_FORMAT,
                       OXYGEN_I2S_MASTER |
                       OXYGEN_I2S_MUTE_MCLK);
    if (chip->model.device_config & CAPTURE_3_FROM_I2S_3)
        oxygen_write16(chip, OXYGEN_I2S_C_FORMAT,
                       OXYGEN_RATE_48000 |
                       chip->model.adc_i2s_format |
                       OXYGEN_I2S_MCLK(chip->model.adc_mclks) |
                       OXYGEN_I2S_BITS_16 |
                       OXYGEN_I2S_MASTER |
                       OXYGEN_I2S_BCLK_64);
    else
        oxygen_write16(chip, OXYGEN_I2S_C_FORMAT,
                       OXYGEN_I2S_MASTER |
                       OXYGEN_I2S_MUTE_MCLK);
    oxygen_clear_bits32(chip, OXYGEN_SPDIF_CONTROL,
                        OXYGEN_SPDIF_OUT_ENABLE |
                        OXYGEN_SPDIF_LOOPBACK);
    if (chip->model.device_config & CAPTURE_1_FROM_SPDIF)
        oxygen_write32_masked(chip, OXYGEN_SPDIF_CONTROL,
                              OXYGEN_SPDIF_SENSE_MASK |
                              OXYGEN_SPDIF_LOCK_MASK |
                              OXYGEN_SPDIF_RATE_MASK |
                              OXYGEN_SPDIF_LOCK_PAR |
                              OXYGEN_SPDIF_IN_CLOCK_96,
                              OXYGEN_SPDIF_SENSE_MASK |
                              OXYGEN_SPDIF_LOCK_MASK |
                              OXYGEN_SPDIF_RATE_MASK |
                              OXYGEN_SPDIF_SENSE_PAR |
                              OXYGEN_SPDIF_LOCK_PAR |
                              OXYGEN_SPDIF_IN_CLOCK_MASK);
    else
        oxygen_clear_bits32(chip, OXYGEN_SPDIF_CONTROL,
                            OXYGEN_SPDIF_SENSE_MASK |
                            OXYGEN_SPDIF_LOCK_MASK |
                            OXYGEN_SPDIF_RATE_MASK);
    oxygen_write32(chip, OXYGEN_SPDIF_OUTPUT_BITS, chip->spdif_bits);
    oxygen_write16(chip, OXYGEN_2WIRE_BUS_STATUS,
                   OXYGEN_2WIRE_LENGTH_8 |
                   OXYGEN_2WIRE_INTERRUPT_MASK |
                   OXYGEN_2WIRE_SPEED_STANDARD);
    oxygen_clear_bits8(chip, OXYGEN_MPU401_CONTROL, OXYGEN_MPU401_LOOPBACK);
    oxygen_write8(chip, OXYGEN_GPI_INTERRUPT_MASK, 0);
    oxygen_write16(chip, OXYGEN_GPIO_INTERRUPT_MASK, 0);
    oxygen_write16(chip, OXYGEN_PLAY_ROUTING,
                   OXYGEN_PLAY_MULTICH_I2S_DAC |
                   OXYGEN_PLAY_SPDIF_SPDIF |
                   (0 << OXYGEN_PLAY_DAC0_SOURCE_SHIFT) |
                   (1 << OXYGEN_PLAY_DAC1_SOURCE_SHIFT) |
                   (2 << OXYGEN_PLAY_DAC2_SOURCE_SHIFT) |
                   (3 << OXYGEN_PLAY_DAC3_SOURCE_SHIFT));
    oxygen_write8(chip, OXYGEN_REC_ROUTING,
                  OXYGEN_REC_A_ROUTE_I2S_ADC_1 |
                  OXYGEN_REC_B_ROUTE_I2S_ADC_2 |
                  OXYGEN_REC_C_ROUTE_SPDIF);
    oxygen_write8(chip, OXYGEN_ADC_MONITOR, 0);
    oxygen_write8(chip, OXYGEN_A_MONITOR_ROUTING,
                  (0 << OXYGEN_A_MONITOR_ROUTE_0_SHIFT) |
                  (1 << OXYGEN_A_MONITOR_ROUTE_1_SHIFT) |
                  (2 << OXYGEN_A_MONITOR_ROUTE_2_SHIFT) |
                  (3 << OXYGEN_A_MONITOR_ROUTE_3_SHIFT));
    
    if (chip->has_ac97_0 | chip->has_ac97_1)
        oxygen_write8(chip, OXYGEN_AC97_INTERRUPT_MASK,
                      OXYGEN_AC97_INT_READ_DONE |
                      OXYGEN_AC97_INT_WRITE_DONE);
    else
        oxygen_write8(chip, OXYGEN_AC97_INTERRUPT_MASK, 0);
    oxygen_write32(chip, OXYGEN_AC97_OUT_CONFIG, 0);
    oxygen_write32(chip, OXYGEN_AC97_IN_CONFIG, 0);
    if (!(chip->has_ac97_0 | chip->has_ac97_1))
        oxygen_set_bits16(chip, OXYGEN_AC97_CONTROL,
                          OXYGEN_AC97_CLOCK_DISABLE);


    if (!chip->has_ac97_0) {
        oxygen_set_bits16(chip, OXYGEN_AC97_CONTROL,
                          OXYGEN_AC97_NO_CODEC_0);
    } else {
        oxygen_write_ac97(chip, 0, AC97_RESET, 0);
        IODelay(1000);
        oxygen_ac97_set_bits(chip, 0, CM9780_GPIO_SETUP,
                             CM9780_GPIO0IO | CM9780_GPIO1IO);
        oxygen_ac97_set_bits(chip, 0, CM9780_MIXER,
                             CM9780_BSTSEL | CM9780_STRO_MIC |
                             CM9780_MIX2FR | CM9780_PCBSW);
        oxygen_ac97_set_bits(chip, 0, CM9780_JACK,
                             CM9780_RSOE | CM9780_CBOE |
                             CM9780_SSOE | CM9780_FROE |
                             CM9780_MIC2MIC | CM9780_LI2LI);
        oxygen_write_ac97(chip, 0, AC97_MASTER, 0x0000);
        oxygen_write_ac97(chip, 0, AC97_PC_BEEP, 0x8000);
        oxygen_write_ac97(chip, 0, AC97_MIC, 0x8808);
        oxygen_write_ac97(chip, 0, AC97_LINE, 0x0808);
        oxygen_write_ac97(chip, 0, AC97_CD, 0x8808);
        oxygen_write_ac97(chip, 0, AC97_VIDEO, 0x8808);
        oxygen_write_ac97(chip, 0, AC97_AUX, 0x8808);
        oxygen_write_ac97(chip, 0, AC97_REC_GAIN, 0x8000);
        oxygen_write_ac97(chip, 0, AC97_CENTER_LFE_MASTER, 0x8080);
        oxygen_write_ac97(chip, 0, AC97_SURROUND_MASTER, 0x8080);
        oxygen_ac97_clear_bits(chip, 0, CM9780_GPIO_STATUS,
                               CM9780_GPO0);
        /* power down unused ADCs and DACs */
        oxygen_ac97_set_bits(chip, 0, AC97_POWERDOWN,
                             AC97_PD_PR0 | AC97_PD_PR1);
        oxygen_ac97_set_bits(chip, 0, AC97_EXTENDED_STATUS,
                             AC97_EA_PRI | AC97_EA_PRJ | AC97_EA_PRK);
    }
    if (chip->has_ac97_1) {
        oxygen_set_bits32(chip, OXYGEN_AC97_OUT_CONFIG,
                          OXYGEN_AC97_CODEC1_SLOT3 |
                          OXYGEN_AC97_CODEC1_SLOT4);
        oxygen_write_ac97(chip, 1, AC97_RESET, 0);
        IODelay(1000);
        oxygen_write_ac97(chip, 1, AC97_MASTER, 0x0000);
        oxygen_write_ac97(chip, 1, AC97_HEADPHONE, 0x8000);
        oxygen_write_ac97(chip, 1, AC97_PC_BEEP, 0x8000);
        oxygen_write_ac97(chip, 1, AC97_MIC, 0x8808);
        oxygen_write_ac97(chip, 1, AC97_LINE, 0x8808);
        oxygen_write_ac97(chip, 1, AC97_CD, 0x8808);
        oxygen_write_ac97(chip, 1, AC97_VIDEO, 0x8808);
        oxygen_write_ac97(chip, 1, AC97_AUX, 0x8808);
        oxygen_write_ac97(chip, 1, AC97_PCM, 0x0808);
        oxygen_write_ac97(chip, 1, AC97_REC_SEL, 0x0000);
        oxygen_write_ac97(chip, 1, AC97_REC_GAIN, 0x0000);
        oxygen_ac97_set_bits(chip, 1, 0x6a, 0x0040);
    }
    // end oxygen_init
    chip->card_model = model;
    dev_id = chip;
    //save ptr to oxygen struct from PCIDriver into private class var dev_id for interrupthandler
    chipData = (struct oxygen*) dev_id;
    result = true;
    
Done:
    
    return result;
}

bool XonarAudioEngine::initHardware(IOService *provider)
{
    /* Comments by Broly:
     * card has been initialised at this point as follows:
     * 1. after . */
    
    bool result = false;
    IOAudioStream *audioStream;
    /*
     IOAudioSampleRate initialSampleRate;
     IOWorkLoop *workLoop;
     */
    
    kprintf("XonarAudioEngine::initHardware()\n");
    
    if (!super::initHardware(provider)) {
        goto Done;
    }
    
    // ] our input and output buffers - a real driver will likely need to allocate its buffers
    // differently
    outputBuffer = (SInt16 *)IOMalloc(DEFAULT_BUFFER_BYTES);
    if (!outputBuffer) {
        goto Done;
    }
    
    inputBuffer = (SInt16 *)IOMalloc(DEFAULT_BUFFER_BYTES);
    if (!inputBuffer) {
        goto Done;
    }
    
    
    //begin oxygen_pcm_init
    struct snd_pcm *pcm;
    int outs, ins;
    int err;
    
    outs = !!(chipData->model.device_config & PLAYBACK_0_TO_I2S);
    ins = !!(chipData->model.device_config & (CAPTURE_0_FROM_I2S_1 |
                                              CAPTURE_0_FROM_I2S_2));
    if (outs | ins) {
        //err = snd_pcm_new(chip->card, "Multichannel",
        //                  0, outs, ins, &pcm);
        if (err < 0)
            return err;
        if (outs) {
            // add multich_ops fns
            audioStream = createAudioStream(kIOAudioStreamDirectionOutput, outputBuffer, DEFAULT_BUFFER_BYTES);
            if (!audioStream) {
                goto Done;
            }

        }
        if (chipData->model.device_config & CAPTURE_0_FROM_I2S_1) {
            //add rec_a_ops fns
            audioStream = createAudioStream(kIOAudioStreamDirectionInput, inputBuffer, DEFAULT_BUFFER_BYTES);
            if (!audioStream) {
                goto Done;
            }
            
        }
        else if (chipData->model.device_config & CAPTURE_0_FROM_I2S_2) {
            //add rec_b_ops fns
            audioStream = createAudioStream(kIOAudioStreamDirectionInput, inputBuffer, DEFAULT_BUFFER_BYTES);
            if (!audioStream) {
                goto Done;
            }

        }
        audioStream->setName("MultiChannel");
        addAudioStream(audioStream);
        audioStream->release();
        //        pcm->private_data = chip;
        //        strcpy(pcm->name, "Multichannel");
        //        if (outs)
        //            snd_pcm_lib_preallocate_pages(pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream,
        //                                          SNDRV_DMA_TYPE_DEV,
        //                                          snd_dma_pci_data(chip->pci),
        //                                          DEFAULT_BUFFER_BYTES_MULTICH,
        //                                          BUFFER_BYTES_MAX_MULTICH);
        //        if (ins)
        //            snd_pcm_lib_preallocate_pages(pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream,
        //                                          SNDRV_DMA_TYPE_DEV,
        //                                          snd_dma_pci_data(chip->pci),
        //                                          DEFAULT_BUFFER_BYTES,
        //                                          BUFFER_BYTES_MAX);
    }
    
    outs = !!(chipData->model.device_config & PLAYBACK_1_TO_SPDIF);
    ins = !!(chipData->model.device_config & CAPTURE_1_FROM_SPDIF);
    if (outs | ins) {
        //err = snd_pcm_new(chip->card, "Digital", 1, outs, ins, &pcm);
        if (err < 0)
            return err;
        if (outs) {
            //add spdif_ops fns
            audioStream = createAudioStream(kIOAudioStreamDirectionOutput, outputBuffer, DEFAULT_BUFFER_BYTES);
            if (!audioStream) {
                goto Done;
            }

        }
        if (ins) {
            //add rc_c_ops fns
            audioStream = createAudioStream(kIOAudioStreamDirectionInput, inputBuffer, DEFAULT_BUFFER_BYTES);
            if (!audioStream) {
                goto Done;
            }

        }
        audioStream->setName("Digital");
        addAudioStream(audioStream);
        audioStream->release();
        //        pcm->private_data = chip;
        //        strcpy(pcm->name, "Digital");
        //        snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
        //                                              snd_dma_pci_data(chipData->pci),
        //                                              DEFAULT_BUFFER_BYTES,
        //                                              BUFFER_BYTES_MAX);
    }
    
    if (chipData->has_ac97_1) {
        outs = !!(chipData->model.device_config & PLAYBACK_2_TO_AC97_1);
        ins = !!(chipData->model.device_config & CAPTURE_2_FROM_AC97_1);
    } else {
        outs = 0;
        ins = !!(chipData->model.device_config & CAPTURE_2_FROM_I2S_2);
    }
    
    if (outs | ins) {
        //        err = snd_pcm_new(chip->card, outs ? "AC97" : "Analog2",
        //                          2, outs, ins, &pcm);
        if (err < 0)
            return err;
        if (outs) {
            //add ac97_ops fns
            audioStream = createAudioStream(kIOAudioStreamDirectionOutput, outputBuffer, DEFAULT_BUFFER_BYTES);
            if (!audioStream) {
                goto Done;
            }
            audioStream->setName(outs ? "AC97" : "Analog2");
            addAudioStream(audioStream);
            audioStream->release();
            oxygen_write8_masked(chipData, OXYGEN_REC_ROUTING,
                                 OXYGEN_REC_B_ROUTE_AC97_1,
                                 OXYGEN_REC_B_ROUTE_MASK);
        }
        if (ins) {
            //add rec_b_ops fns
            audioStream = createAudioStream(kIOAudioStreamDirectionInput, inputBuffer, DEFAULT_BUFFER_BYTES);
            if (!audioStream) {
                goto Done;
            }
            audioStream->setName(outs ? "Front Panel" : "Analog 2");
            addAudioStream(audioStream);
            audioStream->release();
        }
        //        pcm->private_data = chip;
        
        ins = !!(chipData->model.device_config & CAPTURE_3_FROM_I2S_3);
        if (ins) {
            //            err = snd_pcm_new(chip->card,  3, 0, ins, &pcm);
            if (err < 0)
                return err;
            //add rec_c fns
            audioStream = createAudioStream(kIOAudioStreamDirectionInput, inputBuffer, DEFAULT_BUFFER_BYTES);
            if (!audioStream) {
                goto Done;
            }
            audioStream->setName("Analog 3");
            addAudioStream(audioStream);
            audioStream->release();
            oxygen_write8_masked(chipData, OXYGEN_REC_ROUTING,
                                 OXYGEN_REC_C_ROUTE_I2S_ADC_3,
                                 OXYGEN_REC_C_ROUTE_MASK);
            //            pcm->private_data = chip;
            //            strcpy(pcm->name, "Analog 3");
            //            snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
            //                                                  snd_dma_pci_data(chip->pci),
            //                                                  DEFAULT_BUFFER_BYTES,
            //                                                  BUFFER_BYTES_MAX);
        }
    }
    
    
     // Setup the initial sample rate for the audio engine
     initialSampleRate.whole = INITIAL_SAMPLE_RATE;
     initialSampleRate.fraction = 0;
     
     //setDescription("Sample PCI Audio Engine");
     
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
    
     interruptEventSource_main = IOFilterInterruptEventSource::filterInterruptEventSource(this,
     XonarAudioEngine::interruptHandler,
     XonarAudioEngine::interruptFilter,                                                                                                                                                                       audioDevice->getProvider());
     
     if (!interruptEventSource_main) { //|| !gpioEventSource || !spdifEventSource) { <- (see comment in header file)
     goto Done;
     }
    
    result = true;
    
Done:
    
    return result;
    
}


void XonarAudioEngine::free()
{
    kprintf("XonarAudioEngine::free()\n");
    
    // We need to free our resources when we're going away
    
    if (interruptEventSource_main) {
        interruptEventSource_main->release();
        interruptEventSource_main = NULL;
    }
    
    if (outputBuffer) {
        IOFree(outputBuffer, DEFAULT_BUFFER_BYTES);
        outputBuffer = NULL;
    }
    
    if (inputBuffer) {
        IOFree(inputBuffer, DEFAULT_BUFFER_BYTES);
        inputBuffer = NULL;
    }
    super::free();
}

IOReturn XonarAudioEngine::clipOutputSamples(const void *mixBuf,
                                             
                                             void *sampleBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames,
                                             
                                             const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream)

{
    
    UInt32 sampleIndex, maxSampleIndex;
    float *floatMixBuf;
    SInt16 *outputBuf;
    
    
    
    floatMixBuf = (float *)mixBuf;
    outputBuf = (SInt16 *)sampleBuf;
    
    
    
    maxSampleIndex = (firstSampleFrame + numSampleFrames)*streamFormat->fNumChannels;
    
    
    
    for (sampleIndex = (firstSampleFrame * streamFormat->fNumChannels);
         sampleIndex < maxSampleIndex; sampleIndex++)  {
        
        float inSample;
        inSample = floatMixBuf[sampleIndex];
        const static float divisor = ( 1.0 / 32768 );
        
        // Note: A softer clipping operation could be done here
        if (inSample > (1.0 - divisor)) {
            inSample = 1.0 - divisor;
        } else if (inSample < -1.0) {
            inSample = -1.0;
        }
        
        outputBuf[sampleIndex] = (SInt16) (inSample * 32768.0);
        
    }
    
    return kIOReturnSuccess;
    
}

IOReturn XonarAudioEngine::convertInputSamples(const void *sampleBuf,
                                               void *destBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames,
                                               const IOAudioStreamFormat *streamFormat, IOAudioStream
                                               *audioStream)
{
    UInt32 numSamplesLeft;
    float *floatDestBuf;
    SInt16 *inputBuf;
    
    // Note: Source is offset by firstSampleFrame
    inputBuf = &(((SInt16 *)sampleBuf)[firstSampleFrame *
                                       streamFormat->fNumChannels]);
    
    // Note: Destination is not.
    floatDestBuf = (float *)destBuf;
    
    numSamplesLeft = numSampleFrames * streamFormat->fNumChannels;
    
    const static float divisor = ( 1.0 / 32768 );
    while (numSamplesLeft > 0) {
        SInt16 inputSample;
        inputSample = *inputBuf;
        
        if (inputSample >= 0) {
            *floatDestBuf = inputSample * divisor;
        }
        
        ++inputBuf;
        ++floatDestBuf;
        --numSamplesLeft;
    }
    
    return kIOReturnSuccess;
}

IOAudioStream *XonarAudioEngine::createAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize)
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
                kIOAudioStreamByteOrderLittleEndian,				// little endian
                true,											// format is mixable
                0												// driver-defined tag - unused by this driver
            };
            // As part of creating a new IOAudioStream, its sample buffer needs to be set
            // It will automatically create a mix buffer should it be needed
            audioStream->setSampleBuffer(sampleBuffer, sampleBufferSize);
            
            // This device only allows a single format and a choice of 2 different sample rates
            rate.fraction = 0;
            //the below is copying the .rates entry of oxygen_stereo_hardware in oxygen_pcm.c...
            rate.whole = 32000;
            audioStream->addAvailableFormat(&format, &rate, &rate);
            rate.whole = 44100;
            audioStream->addAvailableFormat(&format, &rate, &rate);
            rate.whole = 48000;
            audioStream->addAvailableFormat(&format, &rate, &rate);
            rate.whole = 64000;
            audioStream->addAvailableFormat(&format, &rate, &rate);
            rate.whole = 88200;
            audioStream->addAvailableFormat(&format, &rate, &rate);
            rate.whole = 96000;
            audioStream->addAvailableFormat(&format, &rate, &rate);
            rate.whole = 176400;
            audioStream->addAvailableFormat(&format, &rate, &rate);
            rate.whole = 192000;
            audioStream->addAvailableFormat(&format, &rate, &rate);
            
            // Finally, the IOAudioStream's current format needs to be indicated
            audioStream->setFormat(&format);
        }
    }
    
    return audioStream;
}


void XonarAudioEngine::stop(IOService *provider)
{
    kprintf("XonarAudioEngine::stop()\n");
    
    // When our device is being stopped and torn down, we should go ahead and remove
    // the interrupt event source from the IOWorkLoop
    // Additionally, we'll go ahead and release the interrupt event source since it isn't
    // needed any more
    if (interruptEventSource_main) {
        IOWorkLoop *wl;
        
        wl = getWorkLoop();
        if (wl) {
            wl->removeEventSource(interruptEventSource_main);
        }
        
        interruptEventSource_main->release();
        interruptEventSource_main = NULL;
    }
    
    // Add code to shut down hardware (beyond what is needed to simply stop the audio engine)
    // There may be nothing needed here
    
    super::stop(provider);
}

IOReturn XonarAudioEngine::performAudioEngineStart()
{
    kprintf("XonarAudioEngine::performAudioEngineStart()\n");
    
    // The interruptEventSource needs to be enabled to allow interrupts to start firing
    assert(interruptEventSource);
    interruptEventSource_main->enable();
    
    // When performAudioEngineStart() gets called, the audio engine should be started from the beginning
    // of the sample buffer.  Because it is starting on the first sample, a new timestamp is needed
    // to indicate when that sample is being read from/written to.  The function takeTimeStamp()
    // is provided to do that automatically with the current time.
    // By default takeTimeStamp() will increment the current loop count in addition to taking the current
    // timestamp.  Since we are starting a new audio engine run, and not looping, we don't want the loop count
    // to be incremented.  To accomplish that, false is passed to takeTimeStamp().
    
    /* ^^ this seems like where we'd need something like snd_pcm_period_elapsed.
     * providing first conditional from alsa main interrupt handler (oxygen_interrupt, oxygen_lib.c line 100-101):
     */
    
     // for(i = 0; i < PCM_COUNT; ++i) <- //we don't need the loop here as each IOAudioStream instance should do this
    //      if ((elapsed_streams & (1 << i)) && chip->streams[i])
    //          snd_pcm_period_elapsed(chip->streams[i]);
     
     /*
     * need a way to update the frame pointers at the beginning of each interrupt so that the ensuing
     * operations are applied to a fresh chunk of data (process per-chunk i guess).
     */
    
    takeTimeStamp(false);
    
    // Add audio - I/O start code here
    
    //#error performAudioEngineStart() - driver will not work until audio engine start code is added
    
    return kIOReturnSuccess;
}

IOReturn XonarAudioEngine::performAudioEngineStop()
{
    kprintf("XonarAudioEngine::performAudioEngineStop()\n");
    
    // Assuming we don't need interrupts after stopping the audio engine, we can disable them here
    assert(interruptEventSource_main);
    interruptEventSource_main->disable();
    
    // Add audio - I/O stop code here
    
    //#error performAudioEngineStop() - driver will not work until audio engine stop code is added
    
    return kIOReturnSuccess;
}

UInt32 XonarAudioEngine::getCurrentSampleFrame()
{
    kprintf("XonarAudioEngine::getCurrentSampleFrame()\n");
    
    // In order for the erase process to run properly, this function must return the current location of
    // the audio engine - basically a sample counter
    // It doesn't need to be exact, but if it is inexact, it should err towards being before the current location
    // rather than after the current location.  The erase head will erase up to, but not including the sample
    // frame returned by this function.  If it is too large a value, sound data that hasn't been played will be
    // erased.
    
    //#error getCurrentSampleFrame() - driver will not work until correct sample frame is returned
    
    // Change to return the real value
    return 0;
}

IOReturn XonarAudioEngine::performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate)
{
    //printf("XonarAudioEngine[%p]::peformFormatChange(%p, %p, %p)\n", this, audioStream, newFormat, newSampleRate);
    
    // Since we only allow one format, we only need to be concerned with sample rate changes
    // In this case, we only allow 2 sample rates - 44100 & 48000, so those are the only ones
    // that we check for
    if (newSampleRate) {
        switch (newSampleRate->whole) {
            case 44100:
                kprintf("/t-> 44.1kHz selected\n");
                
                // Add code to switch hardware to 44.1khz
                break;
            case 48000:
                kprintf("/t-> 48kHz selected\n");
                
                // Add code to switch hardware to 48kHz
                break;
            default:
                // This should not be possible since we only specified 44100 and 48000 as valid sample rates
                kprintf("/t Internal Error - unknown sample rate selected.\n");
                break;
        }
    }
    
    return kIOReturnSuccess;
}

void XonarAudioEngine::oxygen_gpio_changed(struct oxygen* chip)
{
    //struct oxygen *chip; = (struct oxygen*) this->dev_id;
    if (chip->model.gpio_changed)
        chip->model.gpio_changed(chip);
}


//
void XonarAudioEngine::oxygen_spdif_input_bits_changed(struct oxygen* chip)
{
    //struct oxygen *chip;// = (struct oxygen*) dev_id;
    UInt32 reg;
    /*
     * This function gets called when there is new activity on the SPDIF
     * input, or when we lose lock on the input signal, or when the rate
     * changes.
     */
    IODelay(1000);
    IOSimpleLockLock(chip->reg_lock);
    reg = oxygen_read32(chip, OXYGEN_SPDIF_CONTROL);
    if ((reg & (OXYGEN_SPDIF_SENSE_STATUS |
                OXYGEN_SPDIF_LOCK_STATUS))
        == OXYGEN_SPDIF_SENSE_STATUS) {
        /*
         * If we detect activity on the SPDIF input but cannot lock to
         * a signal, the clock bit is likely to be wrong.
         */
        reg ^= OXYGEN_SPDIF_IN_CLOCK_MASK;
        oxygen_write32(chip, OXYGEN_SPDIF_CONTROL, reg);
        IOSimpleLockUnlock(chip->reg_lock);
        IODelay(1000);
        IOSimpleLockLock(chip->reg_lock);
        reg = oxygen_read32(chip, OXYGEN_SPDIF_CONTROL);
        if ((reg & (OXYGEN_SPDIF_SENSE_STATUS |
                    OXYGEN_SPDIF_LOCK_STATUS))
            == OXYGEN_SPDIF_SENSE_STATUS) {
            /* nothing detected with either clock; give up */
            if ((reg & OXYGEN_SPDIF_IN_CLOCK_MASK)
                == OXYGEN_SPDIF_IN_CLOCK_192) {
                /*
                 * Reset clock to <= 96 kHz because this is
                 * more likely to be received next time.
                 */
                reg &= ~OXYGEN_SPDIF_IN_CLOCK_MASK;
                reg |= OXYGEN_SPDIF_IN_CLOCK_96;
                oxygen_write32(chip, OXYGEN_SPDIF_CONTROL, reg);
            }
        }
    }
    IOSimpleLockUnlock(chip->reg_lock);
    
     if (chip->controls[CONTROL_SPDIF_INPUT_BITS]) {
     IOSimpleLockLock(chip->reg_lock);
     chip->interrupt_mask |= OXYGEN_INT_SPDIF_IN_DETECT;
     oxygen_write16(chip, OXYGEN_INTERRUPT_MASK,
     chip->interrupt_mask);
     IOSimpleLockUnlock(chip->reg_lock);
     
     
     // We don't actually know that any channel status bits have
     // changed, but let's send a notification just to be sure.
     //
     //    snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
     //                  &chip->controls[CONTROL_SPDIF_INPUT_BITS]->id);
     }
     
}





void XonarAudioEngine::interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count)
{
    // Since our interrupt filter always returns false, this function will never be called
    // If the filter returned true, this function would be called on the IOWorkLoop
    return;
}
bool XonarAudioEngine::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *src)
{
    XonarAudioEngine *callingInstance = OSDynamicCast(XonarAudioEngine, owner);
    struct oxygen *chip = callingInstance->chipData;
    unsigned int status, clear, elapsed_streams, i;
    
    status = oxygen_read16(chip, OXYGEN_INTERRUPT_STATUS);
    if (!status)
        return false;
    
    IOSimpleLockLock(chip->reg_lock);
    
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
    
    IOSimpleLockUnlock(chip->reg_lock);
    /* Not sure if we need this loop since OSX handles PCM very differently
     *from Linux/ALSA. commenting out for now since there is no equivalent of
     *snd_pcm_period_elapsed (updates the position and tells us if we've run over
     *edit: seems like we'll need the elapsed_streams/snd_pcm_period collapsed to
     * go in
     */
    for (i = 0; i < PCM_COUNT; ++i)
        //if ((elapsed_streams & (1 << i)) && chip->streams[i])
        //         snd_pcm_period_elapsed(chip->streams[i]);
        
        if (status & OXYGEN_INT_SPDIF_IN_DETECT) {
            IOSimpleLockLock(chip->reg_lock);
            i = oxygen_read32(chip, OXYGEN_SPDIF_CONTROL);
            if (i & (OXYGEN_SPDIF_SENSE_INT | OXYGEN_SPDIF_LOCK_INT |
                     OXYGEN_SPDIF_RATE_INT)) {
                /* write the interrupt bit(s) to clear */
                oxygen_write32(chip, OXYGEN_SPDIF_CONTROL, i);
                //Linux Call below:
                //    schedule_work(&chip->spdif_input_bits_work);
                //Experimental OSX-equivalent call below...
                /* Conceptually i *think* this function is the IOWorkLoop context,
                 * so by using the dynamic cast (thanks osxbook), i hope to recover
                 * the calling (single) class, and then use its member functions to perform
                 * the work. in this case, checking the spdif bits*/
                callingInstance->workLoop->runAction((Action)callingInstance->oxygen_spdif_input_bits_changed, callingInstance, callingInstance->dev_id);
            }
            IOSimpleLockUnlock(chip->reg_lock);
        }
    
    if (status & OXYGEN_INT_GPIO)
        //Linux call below:
        //schedule_work(&chip->gpio_work);
        //Experimental OSX-equivalent (see note above):
        callingInstance->workLoop->runAction((Action)callingInstance->oxygen_gpio_changed, callingInstance, callingInstance->dev_id);
    /* Commentary on substituting schedule_work with runAction:
     * This *seems* to make sense, no? runAction runs in a single-threaded context w/in the handler.
     * for OSX it seems we don't even need work_queues because:
     * 1. each work_queue instance (gpio_work, spdif_input_bits_work) are used within their associated
     *    functions (gpio_work,spdif_input_bits_changed) to cast the data back to (struct oxygen *)
     *      -since this OSX driver is OOP, we have access to (struct oxygen*) via dev_id and so
     *       we don't need pass such structures to the handling functions as we have access.
     * 2. each of these workqueues seem to schedule work only within the handler, meaning
     *    we should be able to get away with runAction, since there is no other point in the entire
     *    driver where we schedule_work for either the gpio or spdif bits.
     ****
     * i could be totally wrong, but i hope i'm not. runAction would be a superior abstraction if it
     * can replace our need to instantiate "sub"workqueues whose work is added/handled inside the main
     * interrupt handler....
     * i hope it does what we need...
     */
    if (status & OXYGEN_INT_MIDI) {
        // if (chip->midi)
        //     _snd_mpu401_uart_interrupt(0, chip->midi->private_data);
        // else
        oxygen_read_uart(chip);
    }
    
    if (status & OXYGEN_INT_AC97) {
                chip->ac97_statusbits = 0;
                IOLockWakeup(chip->ac97_mutex, &chip->ac97_statusbits, true);
                //thread_wakeup_prim((event_t)chip->ac97_statusbits,0,THREAD_AWAKENED);
    }
    
    return true;
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

