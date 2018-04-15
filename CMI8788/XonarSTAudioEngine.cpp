/*
 File:XonarSTAudioEngine.cpp
 
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
#include "XonarSTAudioEngine.hpp"

#include "pcm1796.h"
#include "cm9780.h"
#include "cs2000.h"
#include "ac97.h"
#define INITIAL_SAMPLE_RATE	44100
#define NUM_SAMPLE_FRAMES	16384
#define NUM_CHANNELS		2
#define BIT_DEPTH			16


#define super IOAudioEngine

//OSDefineMetaClassAndStructors(XonarSTAudioEngine, IOAudioEngine)


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




/*

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
*/

void XonarSTAudioEngine::xonar_st_init_i2c(struct oxygen *chip)
{
    oxygen_write16(chip, OXYGEN_2WIRE_BUS_STATUS,
                   OXYGEN_2WIRE_LENGTH_8 |
                   OXYGEN_2WIRE_INTERRUPT_MASK |
                   OXYGEN_2WIRE_SPEED_STANDARD);
}


void XonarSTAudioEngine::xonar_st_init_common(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
    
    data->generic.output_enable_bit = GPIO_ST_OUTPUT_ENABLE;
    data->dacs = chip->model.dac_channels_mixer / 2;
    data->h6 = chip->model.dac_channels_mixer > 2;
    data->hp_gain_offset = 2*-18;
 
    XonarAudioEngine::pcm1796_init(chip);
    
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL,
                      GPIO_INPUT_ROUTE | GPIO_ST_HP_REAR |
                      GPIO_ST_MAGIC | GPIO_ST_HP);
    oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA,
                        GPIO_INPUT_ROUTE | GPIO_ST_HP_REAR | GPIO_ST_HP);
    
    //still need to figure out how to instantiate the device
    //when calling the constructor for this object, but definitely
    // we can't call init from this function (needs to be other way around).
    //init(chip);
    XonarAudioEngine::xonar_enable_output(chip);
    
 //   snd_component_add(chip->card, "PCM1792A");
  //  snd_component_add(chip->card, "CS5381");
}

void XonarSTAudioEngine::xonar_st_init(struct oxygen *chip)
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
    XonarAudioEngine::cs2000_registers_init(chip);
    xonar_st_init_common(chip);
    
  //  snd_component_add(chip->card, "CS2000");
}

void XonarSTAudioEngine::xonar_stx_init(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*)chip->model_data;
    
    xonar_st_init_i2c(chip);
    data->generic.anti_pop_delay = 800;
    data->generic.ext_power_reg = OXYGEN_GPI_DATA;
    data->generic.ext_power_int_reg = OXYGEN_GPI_INTERRUPT_MASK;
    data->generic.ext_power_bit = GPI_EXT_POWER;
    XonarAudioEngine::xonar_init_ext_power(chip);
    xonar_st_init_common(chip);
}
/*
void xonar_xense_init(struct oxygen *chip)
{
    struct xonar_pcm179x *data = (struct xonar_pcm179x*)chip->model_data;
    
    data->generic.ext_power_reg = OXYGEN_GPI_DATA;
    data->generic.ext_power_int_reg = OXYGEN_GPI_INTERRUPT_MASK;
    data->generic.ext_power_bit = GPI_EXT_POWER;
    XonarAudioEngine::xonar_init_ext_power(chip);
    
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
XonarAudioEngine:xonar_disable_output(chip);
}
*/
void XonarSTAudioEngine::xonar_st_cleanup(struct oxygen *chip)
{
    XonarAudioEngine::xonar_disable_output(chip);
}
/*
static void xonar_d2_suspend(struct oxygen *chip)
{
    xonar_d2_cleanup(chip);
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
*/
void XonarSTAudioEngine::xonar_stx_resume(struct oxygen *chip)
{
    XonarAudioEngine::pcm1796_registers_init(chip);
    XonarAudioEngine::xonar_enable_output(chip);
}

void XonarSTAudioEngine::xonar_st_resume(struct oxygen *chip)
{
    XonarAudioEngine::cs2000_registers_init(chip);
    xonar_stx_resume(chip);
}

void XonarSTAudioEngine::set_st_params(struct oxygen *chip,
                          XonarAudioEngine *instance)
{
    XonarAudioEngine::update_cs2000_rate(chip, instance->getSampleRate()->whole);
    //original call also sends params struct. need to stay on top of this
    //with the IOAudioStream/Engine classes. will figure that out after
    //the skeleton OOP setup is finished.
    //Linux Call:
    //set_pcm1796_params(chip, params);
    //Mac Call:
    XonarAudioEngine::set_pcm1796_params(chip, instance);
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

//static int xonar_d2_mixer_init(struct oxygen *chip)
//{
//    int err;
//    
// //   err = snd_ctl_add(chip->card, snd_ctl_new1(&alt_switch, chip));
//    if (err < 0)
//        return err;
//    err = add_pcm1796_controls(chip);
//    if (err < 0)
//        return err;
//    return 0;
//}


bool XonarSTAudioEngine::init(XonarAudioEngine *engine, struct oxygen *chip)
{
    bool result = false;
    
    IOLog("XonarSTAudioEngine[%p]::init(%p)\n", this, chip);
    
    if (!chip) {
        goto Done;
    }
    
    if (!super::init(NULL)) {
        goto Done;
    }
  //  ak4396_init(chip);
  //  wm8785_init(chip);
    deviceRegisters = (struct xonar_hdav*)chip->model_data;
 
    // the below aren't correct. have to bridge the workqueue calls to IOWorkLoop
    queue_init(&chip->ac97_waitqueue);
    chip->mutex = OS_SPINLOCK_INIT;
    //set the pointer to XonarAudioEngine.
    this->engineInstance = engine;
    
    result = true;
    
Done:
    
    return result;
}

bool XonarSTAudioEngine::initHardware(IOService *provider)
{
    bool result = false;
    IOAudioSampleRate initialSampleRate;
    IOAudioStream *audioStream;
    IOWorkLoop *workLoop;
    
    IOLog("XonarSTAudioEngine[%p]::initHardware(%p)\n", this, provider);
    
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
                                                                                    XonarSTAudioEngine::interruptHandler,
                                                                                    XonarSTAudioEngine::interruptFilter,
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

void XonarSTAudioEngine::free()
{
    IOLog("XonarSTAudioEngine[%p]::free()\n", this);
    
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

IOAudioStream *XonarSTAudioEngine::createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize)
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

void XonarSTAudioEngine::stop(IOService *provider)
{
    IOLog("XonarSTAudioEngine[%p]::stop(%p)\n", this, provider);
    
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

IOReturn XonarSTAudioEngine::performAudioEngineStart()
{
    IOLog("XonarSTAudioEngine[%p]::performAudioEngineStart()\n", this);
    
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

IOReturn XonarSTAudioEngine::performAudioEngineStop()
{
    IOLog("XonarSTAudioEngine[%p]::performAudioEngineStop()\n", this);
    
    // Assuming we don't need interrupts after stopping the audio engine, we can disable them here
    assert(interruptEventSource);
    interruptEventSource->disable();
    
    // Add audio - I/O stop code here
    
#error performAudioEngineStop() - driver will not work until audio engine stop code is added
    
    return kIOReturnSuccess;
}

UInt32 XonarSTAudioEngine::getCurrentSampleFrame()
{
    IOLog("XonarSTAudioEngine[%p]::getCurrentSampleFrame()\n", this);
    
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

IOReturn XonarSTAudioEngine::performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate)
{
    IOLog("XonarSTAudioEngine[%p]::peformFormatChange(%p, %p, %p)\n", this, audioStream, newFormat, newSampleRate);
    
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


void XonarSTAudioEngine::interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count)
{
    // Since our interrupt filter always returns false, this function will never be called
    // If the filter returned true, this function would be called on the IOWorkLoop
    return;
}

bool XonarSTAudioEngine::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source)
{
    XonarSTAudioEngine *audioEngine = OSDynamicCast(XonarSTAudioEngine, owner);
    
    // We've cast the audio engine from the owner which we passed in when we created the interrupt
    // event source
    if (audioEngine) {
        // Then, filterInterrupt() is called on the specified audio engine
        audioEngine->filterInterrupt(source->getIntIndex());
    }
    
    return false;
}

void XonarSTAudioEngine::filterInterrupt(int index)
{
    // In the case of our simple device, we only get interrupts when the audio engine loops to the
    // beginning of the buffer.  When that happens, we need to take a timestamp and increment
    // the loop count.  The function takeTimeStamp() does both of those for us.  Additionally,
    // if a different timestamp is to be used (other than the current time), it can be passed
    // in to takeTimeStamp()
    takeTimeStamp();
}
