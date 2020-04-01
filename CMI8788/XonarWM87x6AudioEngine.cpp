#include "cm9780.h"
#include "cs4398.h"
#include "cs4362a.h"
#include "ac97.h"
#include "XonarWM87x6AudioEngine.hpp"


#include <IOKit/IOLib.h>
#include <IOKit/IOFilterInterruptEventSource.h>

#define INITIAL_SAMPLE_RATE	44100
#define NUM_SAMPLE_FRAMES	16384
#define NUM_CHANNELS		2
#define BIT_DEPTH			16


#define super IOAudioEngine

OSDefineMetaClassAndStructors(XonarWM87x6AudioEngine, IOAudioEngine)


/*
 * card driver for models with WM8776/WM8766 DACs (Xonar DS/HDAV1.3 Slim)
 *
 * Copyright (c) Clemens Ladisch <clemens@ladisch.de>
 *
 *
 *  This driver is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License, version 2.
 *
 *  This driver is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this driver; if not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Xonar DS
 * --------
 *
 * CMI8788:
 *
 *   SPI 0 -> WM8766 (surround, center/LFE, back)
 *   SPI 1 -> WM8776 (front, input)
 *
 *   GPIO 4 <- headphone detect, 0 = plugged
 *   GPIO 6 -> route input jack to mic-in (0) or line-in (1)
 *   GPIO 7 -> enable output to front L/R speaker channels
 *   GPIO 8 -> enable output to other speaker channels and front panel headphone
 *
 * WM8776:
 *
 *   input 1 <- line
 *   input 2 <- mic
 *   input 3 <- front mic
 *   input 4 <- aux
 */

/*
 * Xonar HDAV1.3 Slim
 * ------------------
 *
 * CMI8788:
 *
 *   I²C <-> WM8776 (addr 0011010)
 *
 *   GPIO 0  -> disable HDMI output
 *   GPIO 1  -> enable HP output
 *   GPIO 6  -> firmware EEPROM I²C clock
 *   GPIO 7 <-> firmware EEPROM I²C data
 *
 *   UART <-> HDMI controller
 *
 * WM8776:
 *
 *   input 1 <- mic
 *   input 2 <- aux
 */

#include "wm8776.h"
#include "wm8766.h"

#define GPIO_DS_HP_DETECT	0x0010
#define GPIO_DS_INPUT_ROUTE	0x0040
#define GPIO_DS_OUTPUT_FRONTLR	0x0080
#define GPIO_DS_OUTPUT_ENABLE	0x0100

#define GPIO_SLIM_HDMI_DISABLE	0x0001
#define GPIO_SLIM_OUTPUT_ENABLE	0x0002
#define GPIO_SLIM_FIRMWARE_CLK	0x0040
#define GPIO_SLIM_FIRMWARE_DATA	0x0080

#define I2C_DEVICE_WM8776	0x34	/* 001101, 0, /W=0 */

#define LC_CONTROL_LIMITER	0x40000000
#define LC_CONTROL_ALC		0x20000000


void XonarWM87x6AudioEngine::wm8776_write_spi(struct oxygen *chip,
                             unsigned int reg, unsigned int value, XonarAudioEngine *audioEngine)
{
    audioEngine->oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER |
                     OXYGEN_SPI_DATA_LENGTH_2 |
                     OXYGEN_SPI_CLOCK_160 |
                     (1 << OXYGEN_SPI_CODEC_SHIFT) |
                     OXYGEN_SPI_CEN_LATCH_CLOCK_LO,
                     (reg << 9) | value);
}

void XonarWM87x6AudioEngine::wm8776_write_i2c(struct oxygen *chip,
                             unsigned int reg, unsigned int value, XonarAudioEngine *audioEngine)
{
    audioEngine->oxygen_write_i2c(chip, I2C_DEVICE_WM8776,
                     (reg << 1) | (value >> 8), value);
}

void XonarWM87x6AudioEngine::wm8776_write(struct oxygen *chip,
                         unsigned int reg, unsigned int value, XonarAudioEngine *audioEngine)
{
    struct xonar_wm87x6 *data = (struct xonar_wm87x6*) &chip->model_data;
    
    if ((chip->model.function_flags & OXYGEN_FUNCTION_2WIRE_SPI_MASK) ==
        OXYGEN_FUNCTION_SPI)
        wm8776_write_spi(chip, reg, value,audioEngine);
    else
        wm8776_write_i2c(chip, reg, value,audioEngine);
    if (reg < ARRAY_SIZE(data->wm8776_regs)) {
        if (reg >= WM8776_HPLVOL && reg <= WM8776_DACMASTER)
            value &= ~WM8776_UPDATE;
        data->wm8776_regs[reg] = value;
    }
}

void XonarWM87x6AudioEngine::wm8776_write_cached(struct oxygen *chip,
                                unsigned int reg, unsigned int value, XonarAudioEngine *audioEngine)
{
    struct xonar_wm87x6 *data = (struct xonar_wm87x6*) &chip->model_data;
    
    if (reg >= ARRAY_SIZE(data->wm8776_regs) ||
        value != data->wm8776_regs[reg])
        wm8776_write(chip, reg, value,audioEngine);
}

void XonarWM87x6AudioEngine::wm8766_write(struct oxygen *chip,
                         unsigned int reg, unsigned int value, XonarAudioEngine *audioEngine)
{
    struct xonar_wm87x6 *data = (struct xonar_wm87x6*) &chip->model_data;
    
    audioEngine->oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER |
                     OXYGEN_SPI_DATA_LENGTH_2 |
                     OXYGEN_SPI_CLOCK_160 |
                     (0 << OXYGEN_SPI_CODEC_SHIFT) |
                     OXYGEN_SPI_CEN_LATCH_CLOCK_LO,
                     (reg << 9) | value);
    if (reg < ARRAY_SIZE(data->wm8766_regs)) {
        if ((reg >= WM8766_LDA1 && reg <= WM8766_RDA1) ||
            (reg >= WM8766_LDA2 && reg <= WM8766_MASTDA))
            value &= ~WM8766_UPDATE;
        data->wm8766_regs[reg] = value;
    }
}

void XonarWM87x6AudioEngine::wm8766_write_cached(struct oxygen *chip,
                                unsigned int reg, unsigned int value, XonarAudioEngine *audioEngine)
{
    struct xonar_wm87x6 *data = (struct xonar_wm87x6*) &chip->model_data;
    
    if (reg >= ARRAY_SIZE(data->wm8766_regs) ||
        value != data->wm8766_regs[reg])
        wm8766_write(chip, reg, value, audioEngine);
}

void XonarWM87x6AudioEngine::wm8776_registers_init(struct oxygen *chip,XonarAudioEngine *audioEngine)
{
    struct xonar_wm87x6 *data = (struct xonar_wm87x6*) &chip->model_data;
    
    wm8776_write(chip, WM8776_RESET, 0,audioEngine);
    wm8776_write(chip, WM8776_PHASESWAP, WM8776_PH_MASK,audioEngine);
    wm8776_write(chip, WM8776_DACCTRL1, WM8776_DZCEN |
                 WM8776_PL_LEFT_LEFT | WM8776_PL_RIGHT_RIGHT, audioEngine);
    wm8776_write(chip, WM8776_DACMUTE, chip->dac_mute ? WM8776_DMUTE : 0, audioEngine);
    wm8776_write(chip, WM8776_DACIFCTRL,
                 WM8776_DACFMT_LJUST | WM8776_DACWL_24, audioEngine);
    wm8776_write(chip, WM8776_ADCIFCTRL,
                 data->wm8776_regs[WM8776_ADCIFCTRL], audioEngine);
    wm8776_write(chip, WM8776_MSTRCTRL, data->wm8776_regs[WM8776_MSTRCTRL], audioEngine);
    wm8776_write(chip, WM8776_PWRDOWN, data->wm8776_regs[WM8776_PWRDOWN],audioEngine);
    wm8776_write(chip, WM8776_HPLVOL, data->wm8776_regs[WM8776_HPLVOL], audioEngine);
    wm8776_write(chip, WM8776_HPRVOL, data->wm8776_regs[WM8776_HPRVOL] |
                 WM8776_UPDATE, audioEngine);
    wm8776_write(chip, WM8776_ADCLVOL, data->wm8776_regs[WM8776_ADCLVOL], audioEngine);
    wm8776_write(chip, WM8776_ADCRVOL, data->wm8776_regs[WM8776_ADCRVOL], audioEngine);
    wm8776_write(chip, WM8776_ADCMUX, data->wm8776_regs[WM8776_ADCMUX], audioEngine);
    wm8776_write(chip, WM8776_DACLVOL, chip->dac_volume[0],audioEngine);
    wm8776_write(chip, WM8776_DACRVOL, chip->dac_volume[1] | WM8776_UPDATE, audioEngine);
}

void XonarWM87x6AudioEngine::wm8766_registers_init(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    struct xonar_wm87x6 *data = (struct xonar_wm87x6*) &chip->model_data;
    
    wm8766_write(chip, WM8766_RESET, 0, audioEngine);
    wm8766_write(chip, WM8766_DAC_CTRL, data->wm8766_regs[WM8766_DAC_CTRL], audioEngine);
    wm8766_write(chip, WM8766_INT_CTRL, WM8766_FMT_LJUST | WM8766_IWL_24, audioEngine);
    wm8766_write(chip, WM8766_DAC_CTRL2,
                 WM8766_ZCD | (chip->dac_mute ? WM8766_DMUTE_MASK : 0), audioEngine);
    wm8766_write(chip, WM8766_LDA1, chip->dac_volume[2], audioEngine);
    wm8766_write(chip, WM8766_RDA1, chip->dac_volume[3], audioEngine);
    wm8766_write(chip, WM8766_LDA2, chip->dac_volume[4], audioEngine);
    wm8766_write(chip, WM8766_RDA2, chip->dac_volume[5], audioEngine);
    wm8766_write(chip, WM8766_LDA3, chip->dac_volume[6], audioEngine);
    wm8766_write(chip, WM8766_RDA3, chip->dac_volume[7] | WM8766_UPDATE, audioEngine);
}

void XonarWM87x6AudioEngine::wm8776_init(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    struct xonar_wm87x6 *data = (struct xonar_wm87x6*) &chip->model_data;
    
    data->wm8776_regs[WM8776_HPLVOL] = (0x79 - 60) | WM8776_HPZCEN;
    data->wm8776_regs[WM8776_HPRVOL] = (0x79 - 60) | WM8776_HPZCEN;
    data->wm8776_regs[WM8776_ADCIFCTRL] =
    WM8776_ADCFMT_LJUST | WM8776_ADCWL_24 | WM8776_ADCMCLK;
    data->wm8776_regs[WM8776_MSTRCTRL] =
    WM8776_ADCRATE_256 | WM8776_DACRATE_256;
    data->wm8776_regs[WM8776_PWRDOWN] = WM8776_HPPD;
    data->wm8776_regs[WM8776_ADCLVOL] = 0xa5 | WM8776_ZCA;
    data->wm8776_regs[WM8776_ADCRVOL] = 0xa5 | WM8776_ZCA;
    data->wm8776_regs[WM8776_ADCMUX] = 0x001;
    wm8776_registers_init(chip, audioEngine);
}

void XonarWM87x6AudioEngine::wm8766_init(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    struct xonar_wm87x6 *data = (struct xonar_wm87x6*) &chip->model_data;
    
    data->wm8766_regs[WM8766_DAC_CTRL] =
    WM8766_PL_LEFT_LEFT | WM8766_PL_RIGHT_RIGHT;
    wm8766_registers_init(chip,audioEngine);
}

void XonarWM87x6AudioEngine::xonar_ds_handle_hp_jack(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    struct xonar_wm87x6 *data = (struct xonar_wm87x6*) &chip->model_data;
    bool hp_plugged;
    unsigned int reg;
    
    IOLockLock(chip->mutex);
    
    hp_plugged = !(oxygen_read16(chip, OXYGEN_GPIO_DATA) &
                   GPIO_DS_HP_DETECT);
    
    oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                          hp_plugged ? 0 : GPIO_DS_OUTPUT_FRONTLR,
                          GPIO_DS_OUTPUT_FRONTLR);
    
    reg = data->wm8766_regs[WM8766_DAC_CTRL] & ~WM8766_MUTEALL;
    if (hp_plugged)
        reg |= WM8766_MUTEALL;
    wm8766_write_cached(chip, WM8766_DAC_CTRL, reg, audioEngine);
    
   // snd_jack_report(data->hp_jack, hp_plugged ? SND_JACK_HEADPHONE : 0);
    IOLockUnlock(chip->mutex);
}


void XonarWM87x6AudioEngine::xonar_ds_cleanup(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    audioEngine->xonar_disable_output(chip);
    wm8776_write(chip, WM8776_RESET, 0, audioEngine);
}

void XonarWM87x6AudioEngine::xonar_hdav_slim_cleanup(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    audioEngine->xonar_hdmi_cleanup(chip);
    audioEngine->xonar_disable_output(chip);
    wm8776_write(chip, WM8776_RESET, 0, audioEngine);
    IODelay(2);
}

void XonarWM87x6AudioEngine::xonar_ds_suspend(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    xonar_ds_cleanup(chip, audioEngine);
}

void XonarWM87x6AudioEngine::xonar_hdav_slim_suspend(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    xonar_hdav_slim_cleanup(chip, audioEngine);
}

void XonarWM87x6AudioEngine::xonar_ds_resume(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    wm8776_registers_init(chip, audioEngine);
    wm8766_registers_init(chip, audioEngine);
    audioEngine->xonar_enable_output(chip);
    //xonar_ds_handle_hp_jack(chip);
}

void XonarWM87x6AudioEngine::xonar_hdav_slim_resume(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    struct xonar_wm87x6 *data = (struct xonar_wm87x6*) &chip->model_data;
    
    wm8776_registers_init(chip, audioEngine);
    audioEngine->xonar_hdmi_resume(chip, &data->hdmi);
    audioEngine->xonar_enable_output(chip);
}

void XonarWM87x6AudioEngine::set_wm87x6_dac_params(struct oxygen *chip,
                                  XonarAudioEngine *audioEngine)
{
}

void XonarWM87x6AudioEngine::set_wm8776_adc_params(struct oxygen *chip,
                                  XonarAudioEngine *audioEngine)
{
    UInt16 reg;
    
    reg = WM8776_ADCRATE_256 | WM8776_DACRATE_256;
    if (audioEngine->getSampleRate()->whole > 48000)
        reg |= WM8776_ADCOSR;
    wm8776_write_cached(chip, WM8776_MSTRCTRL, reg, audioEngine);
}

void XonarWM87x6AudioEngine::set_hdav_slim_dac_params(struct oxygen *chip,
                                     XonarAudioEngine *audioEngine)
{
    struct xonar_wm87x6 *data = (struct xonar_wm87x6*) &chip->model_data;
    
    audioEngine->xonar_set_hdmi_params(chip, &data->hdmi);
}

void XonarWM87x6AudioEngine::update_wm8776_volume(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    struct xonar_wm87x6 *data = (struct xonar_wm87x6*) &chip->model_data;
    UInt8 to_change;
    
    if (chip->dac_volume[0] == chip->dac_volume[1]) {
        if (chip->dac_volume[0] != data->wm8776_regs[WM8776_DACLVOL] ||
            chip->dac_volume[1] != data->wm8776_regs[WM8776_DACRVOL]) {
            wm8776_write(chip, WM8776_DACMASTER,
                         chip->dac_volume[0] | WM8776_UPDATE,audioEngine);
            data->wm8776_regs[WM8776_DACLVOL] = chip->dac_volume[0];
            data->wm8776_regs[WM8776_DACRVOL] = chip->dac_volume[0];
        }
    } else {
        to_change = (chip->dac_volume[0] !=
                     data->wm8776_regs[WM8776_DACLVOL]) << 0;
        to_change |= (chip->dac_volume[1] !=
                      data->wm8776_regs[WM8776_DACLVOL]) << 1;
        if (to_change & 1)
            wm8776_write(chip, WM8776_DACLVOL, chip->dac_volume[0] |
                         ((to_change & 2) ? 0 : WM8776_UPDATE),audioEngine);
        if (to_change & 2)
            wm8776_write(chip, WM8776_DACRVOL,
                         chip->dac_volume[1] | WM8776_UPDATE,audioEngine);
    }
}

void XonarWM87x6AudioEngine::update_wm87x6_volume(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    static const UInt8 wm8766_regs[6] = {
        WM8766_LDA1, WM8766_RDA1,
        WM8766_LDA2, WM8766_RDA2,
        WM8766_LDA3, WM8766_RDA3,
    };
    struct xonar_wm87x6 *data = (struct xonar_wm87x6*) &chip->model_data;
    unsigned int i;
    UInt8 to_change;
    
    update_wm8776_volume(chip, audioEngine);
    if (chip->dac_volume[2] == chip->dac_volume[3] &&
        chip->dac_volume[2] == chip->dac_volume[4] &&
        chip->dac_volume[2] == chip->dac_volume[5] &&
        chip->dac_volume[2] == chip->dac_volume[6] &&
        chip->dac_volume[2] == chip->dac_volume[7]) {
        to_change = 0;
        for (i = 0; i < 6; ++i)
            if (chip->dac_volume[2] !=
                data->wm8766_regs[wm8766_regs[i]])
                to_change = 1;
        if (to_change) {
            wm8766_write(chip, WM8766_MASTDA,
                         chip->dac_volume[2] | WM8766_UPDATE, audioEngine);
            for (i = 0; i < 6; ++i)
                data->wm8766_regs[wm8766_regs[i]] =
                chip->dac_volume[2];
        }
    } else {
        to_change = 0;
        for (i = 0; i < 6; ++i)
            to_change |= (chip->dac_volume[2 + i] !=
                          data->wm8766_regs[wm8766_regs[i]]) << i;
        for (i = 0; i < 6; ++i)
            if (to_change & (1 << i))
                wm8766_write(chip, wm8766_regs[i],
                             chip->dac_volume[2 + i] |
                             ((to_change & (0x3e << i))
                              ? 0 : WM8766_UPDATE), audioEngine);
    }
}

void XonarWM87x6AudioEngine::update_wm8776_mute(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    wm8776_write_cached(chip, WM8776_DACMUTE,
                        chip->dac_mute ? WM8776_DMUTE : 0, audioEngine);
}

void XonarWM87x6AudioEngine::update_wm87x6_mute(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    update_wm8776_mute(chip, audioEngine);
    wm8766_write_cached(chip, WM8766_DAC_CTRL2, WM8766_ZCD |
                        (chip->dac_mute ? WM8766_DMUTE_MASK : 0), audioEngine);
}


void XonarWM87x6AudioEngine::xonar_ds_gpio_changed(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
    xonar_ds_handle_hp_jack(chip, audioEngine);
}

bool XonarWM87x6AudioEngine::init(XonarAudioEngine *audioEngine, struct oxygen *chip, UInt16 model)
{
    //begin APPUL portion of sampleaudioengine::init
    bool result = false;
    
    printf("SamplePCIAudioEngine[%p]::init(%p)\n", this, chip);
    chip->model_data = IOMalloc(chip->model.model_data_size);
    deviceRegisters = (struct xonar_wm87x6 *) &chip->model_data;
    if (!chip) {
        goto Done;
    }
    
    if (!super::init(NULL)) {
        goto Done;
    }
    
    
    
    if(model == DS_MODEL || model == DSX_MODEL){
        /*for all submodels, this manual assignment
        * mimics chip->model = <struct model name> 
         */
        chip->model.cleanup = xonar_ds_cleanup;
        chip->model.suspend = xonar_ds_suspend;
        chip->model.resume = xonar_ds_resume;
        //chip->model.pcm_hardware_filter = wm8776_adc_hardware_filter;
        chip->model.set_dac_params = set_wm87x6_dac_params;
        chip->model.set_adc_params = set_wm8776_adc_params;
        chip->model.update_dac_volume = update_wm87x6_volume;
        chip->model.update_dac_mute = update_wm87x6_mute;
        //chip->model.update_center_lfe_mix = update_wm8766_center_lfe_mix;
        //chip->model.gpio_changed = xonar_ds_gpio_changed;
        //chip->model.dump_registers = dump_wm87x6_registers;
        //chip->model.dac_tlv = wm87x6_dac_db_scale;
        
        /* begin ds_init */
        deviceRegisters->generic.anti_pop_delay = 300;
        deviceRegisters->generic.output_enable_bit = GPIO_DS_OUTPUT_ENABLE;
        
        wm8776_init(chip, audioEngine);
        wm8766_init(chip, audioEngine);
        
        oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL,
                          GPIO_DS_INPUT_ROUTE | GPIO_DS_OUTPUT_FRONTLR);
        oxygen_clear_bits16(chip, OXYGEN_GPIO_CONTROL,
                            GPIO_DS_HP_DETECT);
        oxygen_set_bits16(chip, OXYGEN_GPIO_DATA, GPIO_DS_INPUT_ROUTE);
        oxygen_set_bits16(chip, OXYGEN_GPIO_INTERRUPT_MASK, GPIO_DS_HP_DETECT);
        chip->interrupt_mask |= OXYGEN_INT_GPIO;
        
        audioEngine->xonar_enable_output(chip);
        /*end xonar_ds_init*/

    }
    else if (model == HDAV_SLIM) {
        //.init = xonar_hdav_slim_init,
        //chip->model.mixer_init = xonar_hdav_slim_mixer_init;
        chip->model.cleanup = xonar_hdav_slim_cleanup;
        chip->model.suspend = xonar_hdav_slim_suspend;
        chip->model.resume = xonar_hdav_slim_resume;
        //chip->model.pcm_hardware_filter = xonar_hdav_slim_hardware_filter;
        chip->model.set_dac_params = set_hdav_slim_dac_params;
        chip->model.set_adc_params = set_wm8776_adc_params;
        chip->model.update_dac_volume = update_wm8776_volume;
        chip->model.update_dac_mute = update_wm8776_mute;
        chip->model.uart_input = audioEngine->xonar_hdmi_uart_input;
        //chip->model.dump_registers = dump_wm8776_registers;
        //chip->model.dac_tlv = wm87x6_dac_db_scale;
        
        /* begin hdav_slim_init */
        struct xonar_wm87x6 *data = (struct xonar_wm87x6*) &chip->model_data;
        
        data->generic.anti_pop_delay = 300;
        data->generic.output_enable_bit = GPIO_SLIM_OUTPUT_ENABLE;
        
        wm8776_init(chip, audioEngine);
        
        oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL,
                          GPIO_SLIM_HDMI_DISABLE |
                          GPIO_SLIM_FIRMWARE_CLK |
                          GPIO_SLIM_FIRMWARE_DATA);
        
        audioEngine->xonar_hdmi_init(chip, &data->hdmi);
        audioEngine->xonar_enable_output(chip);
        
        //snd_component_add(chip->card, "WM8776");

        /*end hdav_slim_init */
    }
    else
        goto Done;

    //end APPUL portion of sampleaudioengine::init
    
    
    
    //set registers/engine and finish APPUL sampleaudioengine init
    this->engineInstance = audioEngine;
    result = true;
    
Done:
    
    return result;
}


bool XonarWM87x6AudioEngine::initHardware(IOService *provider)
{
    bool result = false;
    IOAudioSampleRate initialSampleRate;
    IOAudioStream *audioStream;
    IOWorkLoop *workLoop;
    
    printf("XonarWM87x6AudioEngine[%p]::initHardware(%p)\n", this, provider);
    
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
    
    workLoop = engineInstance->getWorkLoop();
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
                                                                                    engineInstance->interruptHandler,
                                                                                    engineInstance->interruptFilter,
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
    outputBuffer = (SInt16 *)IOMalloc(DEFAULT_BUFFER_BYTES);
    if (!outputBuffer) {
        goto Done;
    }
    
    inputBuffer = (SInt16 *)IOMalloc(DEFAULT_BUFFER_BYTES);
    if (!inputBuffer) {
        goto Done;
    }
    
    // Create an IOAudioStream for each buffer and add it to this audio engine
    audioStream = createNewAudioStream(kIOAudioStreamDirectionOutput, outputBuffer, DEFAULT_BUFFER_BYTES);
    if (!audioStream) {
        goto Done;
    }
    
    addAudioStream(audioStream);
    audioStream->release();
    
    audioStream = createNewAudioStream(kIOAudioStreamDirectionInput, inputBuffer, DEFAULT_BUFFER_BYTES);
    if (!audioStream) {
        goto Done;
    }
    
    addAudioStream(audioStream);
    audioStream->release();
    
    result = true;
    
Done:
    
    return result;
}

void XonarWM87x6AudioEngine::free()
{
    printf("XonarWM87x6AudioEngine[%p]::free()\n", this);
    
    // We need to free our resources when we're going away
    
    if (interruptEventSource) {
        interruptEventSource->release();
        interruptEventSource = NULL;
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

IOAudioStream *XonarWM87x6AudioEngine::createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize)
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

void XonarWM87x6AudioEngine::stop(IOService *provider)
{
    printf("XonarWM87x6AudioEngine[%p]::stop(%p)\n", this, provider);
    
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

IOReturn XonarWM87x6AudioEngine::performAudioEngineStart()
{
    printf("XonarWM87x6AudioEngine[%p]::performAudioEngineStart()\n", this);
    
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
    
    //#error performAudioEngineStart() - driver will not work until audio engine start code is added
    
    return kIOReturnSuccess;
}

IOReturn XonarWM87x6AudioEngine::performAudioEngineStop()
{
    printf("XonarWM87x6AudioEngine[%p]::performAudioEngineStop()\n", this);
    
    // Assuming we don't need interrupts after stopping the audio engine, we can disable them here
    assert(interruptEventSource);
    interruptEventSource->disable();
    
    // Add audio - I/O stop code here
    
    //#error performAudioEngineStop() - driver will not work until audio engine stop code is added
    
    return kIOReturnSuccess;
}

UInt32 XonarWM87x6AudioEngine::getCurrentSampleFrame()
{
    printf("XonarWM87x6AudioEngine[%p]::getCurrentSampleFrame()\n", this);
    
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

IOReturn XonarWM87x6AudioEngine::performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate)
{
    printf("XonarWM87x6AudioEngine[%p]::peformFormatChange(%p, %p, %p)\n", this, audioStream, newFormat, newSampleRate);
    
    // Since we only allow one format, we only need to be concerned with sample rate changes
    // In this case, we only allow 2 sample rates - 44100 & 48000, so those are the only ones
    // that we check for
    if (newSampleRate) {
        switch (newSampleRate->whole) {
            case 44100:
                printf("/t-> 44.1kHz selected\n");
                
                // Add code to switch hardware to 44.1khz
                break;
            case 48000:
                printf("/t-> 48kHz selected\n");
                
                // Add code to switch hardware to 48kHz
                break;
            default:
                // This should not be possible since we only specified 44100 and 48000 as valid sample rates
                printf("/t Internal Error - unknown sample rate selected.\n");
                break;
        }
    }
    
    return kIOReturnSuccess;
}


void XonarWM87x6AudioEngine::interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count)
{
    // Since our interrupt filter always returns false, this function will never be called
    // If the filter returned true, this function would be called on the IOWorkLoop
    return;
}

bool XonarWM87x6AudioEngine::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source)
{
    XonarWM87x6AudioEngine *audioEngine = OSDynamicCast(XonarWM87x6AudioEngine, owner);
    
    // We've cast the audio engine from the owner which we passed in when we created the interrupt
    // event source
    if (audioEngine) {
        // Then, filterInterrupt() is called on the specified audio engine
        audioEngine->filterInterrupt(source->getIntIndex());
    }
    
    return false;
}

void XonarWM87x6AudioEngine::filterInterrupt(int index)
{
    // In the case of our simple device, we only get interrupts when the audio engine loops to the
    // beginning of the buffer.  When that happens, we need to take a timestamp and increment
    // the loop count.  The function takeTimeStamp() does both of those for us.  Additionally,
    // if a different timestamp is to be used (other than the current time), it can be passed
    // in to takeTimeStamp()
    takeTimeStamp();
}


/*
 static void wm8776_adc_hardware_filter(unsigned int channel,
 struct snd_pcm_hardware *hardware)
 {
 if (channel == PCM_A) {
 hardware->rates = SNDRV_PCM_RATE_32000 |
 SNDRV_PCM_RATE_44100 |
 SNDRV_PCM_RATE_48000 |
 SNDRV_PCM_RATE_64000 |
 SNDRV_PCM_RATE_88200 |
 SNDRV_PCM_RATE_96000;
 hardware->rate_max = 96000;
 }
 }
 
 static void xonar_hdav_slim_hardware_filter(unsigned int channel,
 struct snd_pcm_hardware *hardware)
 {
 wm8776_adc_hardware_filter(channel, hardware);
 xonar_hdmi_pcm_hardware_filter(channel, hardware);
 }
 */
//static void update_wm8766_center_lfe_mix(struct oxygen *chip, bool mixed)
//{
//    struct xonar_wm87x6 *data = chip->model_data;
//    unsigned int reg;
//
//    /*
//     * The WM8766 can mix left and right channels, but this setting
//     * applies to all three stereo pairs.
//     */
//    reg = data->wm8766_regs[WM8766_DAC_CTRL] &
//    ~(WM8766_PL_LEFT_MASK | WM8766_PL_RIGHT_MASK);
//    if (mixed)
//        reg |= WM8766_PL_LEFT_LRMIX | WM8766_PL_RIGHT_LRMIX;
//    else
//        reg |= WM8766_PL_LEFT_LEFT | WM8766_PL_RIGHT_RIGHT;
//    wm8766_write_cached(chip, WM8766_DAC_CTRL, reg);
//}

/*
 static int wm8776_bit_switch_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_wm87x6 *data = chip->model_data;
 u16 bit = ctl->private_value & 0xffff;
 unsigned int reg_index = (ctl->private_value >> 16) & 0xff;
 bool invert = (ctl->private_value >> 24) & 1;
 
 value->value.integer.value[0] =
 ((data->wm8776_regs[reg_index] & bit) != 0) ^ invert;
 return 0;
 }
 
 static int wm8776_bit_switch_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_wm87x6 *data = chip->model_data;
 u16 bit = ctl->private_value & 0xffff;
 u16 reg_value;
 unsigned int reg_index = (ctl->private_value >> 16) & 0xff;
 bool invert = (ctl->private_value >> 24) & 1;
 int changed;
 
 IOLockLock(&chip->mutex);
 reg_value = data->wm8776_regs[reg_index] & ~bit;
 if (value->value.integer.value[0] ^ invert)
 reg_value |= bit;
 changed = reg_value != data->wm8776_regs[reg_index];
 if (changed)
 wm8776_write(chip, reg_index, reg_value);
 IOLockUnlock(&chip->mutex);
 return changed;
 }
 
 static int wm8776_field_enum_info(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_info *info)
 {
 static const char *const hld[16] = {
 "0 ms", "2.67 ms", "5.33 ms", "10.6 ms",
 "21.3 ms", "42.7 ms", "85.3 ms", "171 ms",
 "341 ms", "683 ms", "1.37 s", "2.73 s",
 "5.46 s", "10.9 s", "21.8 s", "43.7 s",
 };
 static const char *const atk_lim[11] = {
 "0.25 ms", "0.5 ms", "1 ms", "2 ms",
 "4 ms", "8 ms", "16 ms", "32 ms",
 "64 ms", "128 ms", "256 ms",
 };
 static const char *const atk_alc[11] = {
 "8.40 ms", "16.8 ms", "33.6 ms", "67.2 ms",
 "134 ms", "269 ms", "538 ms", "1.08 s",
 "2.15 s", "4.3 s", "8.6 s",
 };
 static const char *const dcy_lim[11] = {
 "1.2 ms", "2.4 ms", "4.8 ms", "9.6 ms",
 "19.2 ms", "38.4 ms", "76.8 ms", "154 ms",
 "307 ms", "614 ms", "1.23 s",
 };
 static const char *const dcy_alc[11] = {
 "33.5 ms", "67.0 ms", "134 ms", "268 ms",
 "536 ms", "1.07 s", "2.14 s", "4.29 s",
 "8.58 s", "17.2 s", "34.3 s",
 };
 static const char *const tranwin[8] = {
 "0 us", "62.5 us", "125 us", "250 us",
 "500 us", "1 ms", "2 ms", "4 ms",
 };
 u8 max;
 const char *const *names;
 
 max = (ctl->private_value >> 12) & 0xf;
 switch ((ctl->private_value >> 24) & 0x1f) {
 case WM8776_ALCCTRL2:
 names = hld;
 break;
 case WM8776_ALCCTRL3:
 if (((ctl->private_value >> 20) & 0xf) == 0) {
 if (ctl->private_value & LC_CONTROL_LIMITER)
 names = atk_lim;
 else
 names = atk_alc;
 } else {
 if (ctl->private_value & LC_CONTROL_LIMITER)
 names = dcy_lim;
 else
 names = dcy_alc;
 }
 break;
 case WM8776_LIMITER:
 names = tranwin;
 break;
 default:
 return -ENXIO;
 }
 return snd_ctl_enum_info(info, 1, max + 1, names);
 }
 
 static int wm8776_field_volume_info(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_info *info)
 {
 info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
 info->count = 1;
 info->value.integer.min = (ctl->private_value >> 8) & 0xf;
 info->value.integer.max = (ctl->private_value >> 12) & 0xf;
 return 0;
 }
 
 static void wm8776_field_set_from_ctl(struct snd_kcontrol *ctl)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_wm87x6 *data = chip->model_data;
 unsigned int value, reg_index, mode;
 u8 min, max, shift;
 u16 mask, reg_value;
 bool invert;
 
 if ((data->wm8776_regs[WM8776_ALCCTRL1] & WM8776_LCSEL_MASK) ==
 WM8776_LCSEL_LIMITER)
 mode = LC_CONTROL_LIMITER;
 else
 mode = LC_CONTROL_ALC;
 if (!(ctl->private_value & mode))
 return;
 
 value = ctl->private_value & 0xf;
 min = (ctl->private_value >> 8) & 0xf;
 max = (ctl->private_value >> 12) & 0xf;
 mask = (ctl->private_value >> 16) & 0xf;
 shift = (ctl->private_value >> 20) & 0xf;
 reg_index = (ctl->private_value >> 24) & 0x1f;
 invert = (ctl->private_value >> 29) & 0x1;
 
 if (invert)
 value = max - (value - min);
 reg_value = data->wm8776_regs[reg_index];
 reg_value &= ~(mask << shift);
 reg_value |= value << shift;
 wm8776_write_cached(chip, reg_index, reg_value);
 }
 
 static int wm8776_field_set(struct snd_kcontrol *ctl, unsigned int value)
 {
 struct oxygen *chip = ctl->private_data;
 u8 min, max;
 int changed;
 
 min = (ctl->private_value >> 8) & 0xf;
 max = (ctl->private_value >> 12) & 0xf;
 if (value < min || value > max)
 return -EINVAL;
 IOLockLock(&chip->mutex);
 changed = value != (ctl->private_value & 0xf);
 if (changed) {
 ctl->private_value = (ctl->private_value & ~0xf) | value;
 wm8776_field_set_from_ctl(ctl);
 }
 IOLockUnlock(&chip->mutex);
 return changed;
 }
 
 static int wm8776_field_enum_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 value->value.enumerated.item[0] = ctl->private_value & 0xf;
 return 0;
 }
 
 static int wm8776_field_volume_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 value->value.integer.value[0] = ctl->private_value & 0xf;
 return 0;
 }
 
 static int wm8776_field_enum_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 return wm8776_field_set(ctl, value->value.enumerated.item[0]);
 }
 
 static int wm8776_field_volume_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 return wm8776_field_set(ctl, value->value.integer.value[0]);
 }
 
 static int wm8776_hp_vol_info(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_info *info)
 {
 info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
 info->count = 2;
 info->value.integer.min = 0x79 - 60;
 info->value.integer.max = 0x7f;
 return 0;
 }
 
 static int wm8776_hp_vol_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_wm87x6 *data = chip->model_data;
 
 IOLockLock(&chip->mutex);
 value->value.integer.value[0] =
 data->wm8776_regs[WM8776_HPLVOL] & WM8776_HPATT_MASK;
 value->value.integer.value[1] =
 data->wm8776_regs[WM8776_HPRVOL] & WM8776_HPATT_MASK;
 IOLockUnlock(&chip->mutex);
 return 0;
 }
 
 static int wm8776_hp_vol_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_wm87x6 *data = chip->model_data;
 u8 to_update;
 
 IOLockLock(&chip->mutex);
 to_update = (value->value.integer.value[0] !=
 (data->wm8776_regs[WM8776_HPLVOL] & WM8776_HPATT_MASK))
 << 0;
 to_update |= (value->value.integer.value[1] !=
 (data->wm8776_regs[WM8776_HPRVOL] & WM8776_HPATT_MASK))
 << 1;
 if (value->value.integer.value[0] == value->value.integer.value[1]) {
 if (to_update) {
 wm8776_write(chip, WM8776_HPMASTER,
 value->value.integer.value[0] |
 WM8776_HPZCEN | WM8776_UPDATE);
 data->wm8776_regs[WM8776_HPLVOL] =
 value->value.integer.value[0] | WM8776_HPZCEN;
 data->wm8776_regs[WM8776_HPRVOL] =
 value->value.integer.value[0] | WM8776_HPZCEN;
 }
 } else {
 if (to_update & 1)
 wm8776_write(chip, WM8776_HPLVOL,
 value->value.integer.value[0] |
 WM8776_HPZCEN |
 ((to_update & 2) ? 0 : WM8776_UPDATE));
 if (to_update & 2)
 wm8776_write(chip, WM8776_HPRVOL,
 value->value.integer.value[1] |
 WM8776_HPZCEN | WM8776_UPDATE);
 }
 IOLockUnlock(&chip->mutex);
 return to_update != 0;
 }
 
 static int wm8776_input_mux_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_wm87x6 *data = chip->model_data;
 unsigned int mux_bit = ctl->private_value;
 
 value->value.integer.value[0] =
 !!(data->wm8776_regs[WM8776_ADCMUX] & mux_bit);
 return 0;
 }
 
 static int wm8776_input_mux_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_wm87x6 *data = chip->model_data;
 struct snd_kcontrol *other_ctl;
 unsigned int mux_bit = ctl->private_value;
 u16 reg;
 int changed;
 
 IOLockLock(&chip->mutex);
 reg = data->wm8776_regs[WM8776_ADCMUX];
 if (value->value.integer.value[0]) {
 reg |= mux_bit;
 // line-in and mic-in are exclusive
 mux_bit ^= 3;
 if (reg & mux_bit) {
 reg &= ~mux_bit;
 if (mux_bit == 1)
 other_ctl = data->line_adcmux_control;
 else
 other_ctl = data->mic_adcmux_control;
 snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
 &other_ctl->id);
 }
 } else
 reg &= ~mux_bit;
 changed = reg != data->wm8776_regs[WM8776_ADCMUX];
 if (changed) {
 oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
 reg & 1 ? GPIO_DS_INPUT_ROUTE : 0,
 GPIO_DS_INPUT_ROUTE);
 wm8776_write(chip, WM8776_ADCMUX, reg);
 }
 IOLockUnlock(&chip->mutex);
 return changed;
 }
 
 static int wm8776_input_vol_info(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_info *info)
 {
 info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
 info->count = 2;
 info->value.integer.min = 0xa5;
 info->value.integer.max = 0xff;
 return 0;
 }
 
 static int wm8776_input_vol_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_wm87x6 *data = chip->model_data;
 
 IOLockLock(&chip->mutex);
 value->value.integer.value[0] =
 data->wm8776_regs[WM8776_ADCLVOL] & WM8776_AGMASK;
 value->value.integer.value[1] =
 data->wm8776_regs[WM8776_ADCRVOL] & WM8776_AGMASK;
 IOLockUnlock(&chip->mutex);
 return 0;
 }
 
 static int wm8776_input_vol_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_wm87x6 *data = chip->model_data;
 int changed = 0;
 
 IOLockLock(&chip->mutex);
 changed = (value->value.integer.value[0] !=
 (data->wm8776_regs[WM8776_ADCLVOL] & WM8776_AGMASK)) ||
 (value->value.integer.value[1] !=
 (data->wm8776_regs[WM8776_ADCRVOL] & WM8776_AGMASK));
 wm8776_write_cached(chip, WM8776_ADCLVOL,
 value->value.integer.value[0] | WM8776_ZCA);
 wm8776_write_cached(chip, WM8776_ADCRVOL,
 value->value.integer.value[1] | WM8776_ZCA);
 IOLockUnlock(&chip->mutex);
 return changed;
 }
 
 static int wm8776_level_control_info(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_info *info)
 {
 static const char *const names[3] = {
 "None", "Peak Limiter", "Automatic Level Control"
 };
 
 return snd_ctl_enum_info(info, 1, 3, names);
 }
 
 static int wm8776_level_control_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_wm87x6 *data = chip->model_data;
 
 if (!(data->wm8776_regs[WM8776_ALCCTRL2] & WM8776_LCEN))
 value->value.enumerated.item[0] = 0;
 else if ((data->wm8776_regs[WM8776_ALCCTRL1] & WM8776_LCSEL_MASK) ==
 WM8776_LCSEL_LIMITER)
 value->value.enumerated.item[0] = 1;
 else
 value->value.enumerated.item[0] = 2;
 return 0;
 }
 
 static void activate_control(struct oxygen *chip,
 struct snd_kcontrol *ctl, unsigned int mode)
 {
 unsigned int access;
 
 if (ctl->private_value & mode)
 access = 0;
 else
 access = SNDRV_CTL_ELEM_ACCESS_INACTIVE;
 if ((ctl->vd[0].access & SNDRV_CTL_ELEM_ACCESS_INACTIVE) != access) {
 ctl->vd[0].access ^= SNDRV_CTL_ELEM_ACCESS_INACTIVE;
 snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_INFO, &ctl->id);
 }
 }
 
 static int wm8776_level_control_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_wm87x6 *data = chip->model_data;
 unsigned int mode = 0, i;
 u16 ctrl1, ctrl2;
 int changed;
 
 if (value->value.enumerated.item[0] >= 3)
 return -EINVAL;
 IOLockLock(&chip->mutex);
 changed = value->value.enumerated.item[0] != ctl->private_value;
 if (changed) {
 ctl->private_value = value->value.enumerated.item[0];
 ctrl1 = data->wm8776_regs[WM8776_ALCCTRL1];
 ctrl2 = data->wm8776_regs[WM8776_ALCCTRL2];
 switch (value->value.enumerated.item[0]) {
 default:
 wm8776_write_cached(chip, WM8776_ALCCTRL2,
 ctrl2 & ~WM8776_LCEN);
 break;
 case 1:
 wm8776_write_cached(chip, WM8776_ALCCTRL1,
 (ctrl1 & ~WM8776_LCSEL_MASK) |
 WM8776_LCSEL_LIMITER);
 wm8776_write_cached(chip, WM8776_ALCCTRL2,
 ctrl2 | WM8776_LCEN);
 mode = LC_CONTROL_LIMITER;
 break;
 case 2:
 wm8776_write_cached(chip, WM8776_ALCCTRL1,
 (ctrl1 & ~WM8776_LCSEL_MASK) |
 WM8776_LCSEL_ALC_STEREO);
 wm8776_write_cached(chip, WM8776_ALCCTRL2,
 ctrl2 | WM8776_LCEN);
 mode = LC_CONTROL_ALC;
 break;
 }
 for (i = 0; i < ARRAY_SIZE(data->lc_controls); ++i)
 activate_control(chip, data->lc_controls[i], mode);
 }
 IOLockUnlock(&chip->mutex);
 return changed;
 }
 
 static int hpf_info(struct snd_kcontrol *ctl, struct snd_ctl_elem_info *info)
 {
 static const char *const names[2] = {
 "None", "High-pass Filter"
 };
 
 return snd_ctl_enum_info(info, 1, 2, names);
 }
 
 static int hpf_get(struct snd_kcontrol *ctl, struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_wm87x6 *data = chip->model_data;
 
 value->value.enumerated.item[0] =
 !(data->wm8776_regs[WM8776_ADCIFCTRL] & WM8776_ADCHPD);
 return 0;
 }
 
 static int hpf_put(struct snd_kcontrol *ctl, struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_wm87x6 *data = chip->model_data;
 unsigned int reg;
 int changed;
 
 IOLockLock(&chip->mutex);
 reg = data->wm8776_regs[WM8776_ADCIFCTRL] & ~WM8776_ADCHPD;
 if (!value->value.enumerated.item[0])
 reg |= WM8776_ADCHPD;
 changed = reg != data->wm8776_regs[WM8776_ADCIFCTRL];
 if (changed)
 wm8776_write(chip, WM8776_ADCIFCTRL, reg);
 IOLockUnlock(&chip->mutex);
 return changed;
 }
 
 #define WM8776_BIT_SWITCH(xname, reg, bit, invert, flags) { \
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
 .name = xname, \
 .info = snd_ctl_boolean_mono_info, \
 .get = wm8776_bit_switch_get, \
 .put = wm8776_bit_switch_put, \
 .private_value = ((reg) << 16) | (bit) | ((invert) << 24) | (flags), \
 }
 #define _WM8776_FIELD_CTL(xname, reg, shift, initval, min, max, mask, flags) \
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
 .name = xname, \
 .private_value = (initval) | ((min) << 8) | ((max) << 12) | \
 ((mask) << 16) | ((shift) << 20) | ((reg) << 24) | (flags)
 #define WM8776_FIELD_CTL_ENUM(xname, reg, shift, init, min, max, mask, flags) {\
 _WM8776_FIELD_CTL(xname " Capture Enum", \
 reg, shift, init, min, max, mask, flags), \
 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE | \
 SNDRV_CTL_ELEM_ACCESS_INACTIVE, \
 .info = wm8776_field_enum_info, \
 .get = wm8776_field_enum_get, \
 .put = wm8776_field_enum_put, \
 }
 #define WM8776_FIELD_CTL_VOLUME(a, b, c, d, e, f, g, h, tlv_p) { \
 _WM8776_FIELD_CTL(a " Capture Volume", b, c, d, e, f, g, h), \
 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE | \
 SNDRV_CTL_ELEM_ACCESS_INACTIVE | \
 SNDRV_CTL_ELEM_ACCESS_TLV_READ, \
 .info = wm8776_field_volume_info, \
 .get = wm8776_field_volume_get, \
 .put = wm8776_field_volume_put, \
 .tlv = { .p = tlv_p }, \
 }
 
 static const DECLARE_TLV_DB_SCALE(wm87x6_dac_db_scale, -6000, 50, 0);
 static const DECLARE_TLV_DB_SCALE(wm8776_adc_db_scale, -2100, 50, 0);
 static const DECLARE_TLV_DB_SCALE(wm8776_hp_db_scale, -6000, 100, 0);
 static const DECLARE_TLV_DB_SCALE(wm8776_lct_db_scale, -1600, 100, 0);
 static const DECLARE_TLV_DB_SCALE(wm8776_maxgain_db_scale, 0, 400, 0);
 static const DECLARE_TLV_DB_SCALE(wm8776_ngth_db_scale, -7800, 600, 0);
 static const DECLARE_TLV_DB_SCALE(wm8776_maxatten_lim_db_scale, -1200, 100, 0);
 static const DECLARE_TLV_DB_SCALE(wm8776_maxatten_alc_db_scale, -2100, 400, 0);
 
 static const struct snd_kcontrol_new ds_controls[] = {
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Headphone Playback Volume",
 .info = wm8776_hp_vol_info,
 .get = wm8776_hp_vol_get,
 .put = wm8776_hp_vol_put,
 .tlv = { .p = wm8776_hp_db_scale },
 },
 WM8776_BIT_SWITCH("Headphone Playback Switch",
 WM8776_PWRDOWN, WM8776_HPPD, 1, 0),
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Input Capture Volume",
 .info = wm8776_input_vol_info,
 .get = wm8776_input_vol_get,
 .put = wm8776_input_vol_put,
 .tlv = { .p = wm8776_adc_db_scale },
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Line Capture Switch",
 .info = snd_ctl_boolean_mono_info,
 .get = wm8776_input_mux_get,
 .put = wm8776_input_mux_put,
 .private_value = 1 << 0,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Mic Capture Switch",
 .info = snd_ctl_boolean_mono_info,
 .get = wm8776_input_mux_get,
 .put = wm8776_input_mux_put,
 .private_value = 1 << 1,
 },
 WM8776_BIT_SWITCH("Front Mic Capture Switch",
 WM8776_ADCMUX, 1 << 2, 0, 0),
 WM8776_BIT_SWITCH("Aux Capture Switch",
 WM8776_ADCMUX, 1 << 3, 0, 0),
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "ADC Filter Capture Enum",
 .info = hpf_info,
 .get = hpf_get,
 .put = hpf_put,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Level Control Capture Enum",
 .info = wm8776_level_control_info,
 .get = wm8776_level_control_get,
 .put = wm8776_level_control_put,
 .private_value = 0,
 },
 };
 static const struct snd_kcontrol_new hdav_slim_controls[] = {
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "HDMI Playback Switch",
 .info = snd_ctl_boolean_mono_info,
 .get = xonar_gpio_bit_switch_get,
 .put = xonar_gpio_bit_switch_put,
 .private_value = GPIO_SLIM_HDMI_DISABLE | XONAR_GPIO_BIT_INVERT,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Headphone Playback Volume",
 .info = wm8776_hp_vol_info,
 .get = wm8776_hp_vol_get,
 .put = wm8776_hp_vol_put,
 .tlv = { .p = wm8776_hp_db_scale },
 },
 WM8776_BIT_SWITCH("Headphone Playback Switch",
 WM8776_PWRDOWN, WM8776_HPPD, 1, 0),
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Input Capture Volume",
 .info = wm8776_input_vol_info,
 .get = wm8776_input_vol_get,
 .put = wm8776_input_vol_put,
 .tlv = { .p = wm8776_adc_db_scale },
 },
 WM8776_BIT_SWITCH("Mic Capture Switch",
 WM8776_ADCMUX, 1 << 0, 0, 0),
 WM8776_BIT_SWITCH("Aux Capture Switch",
 WM8776_ADCMUX, 1 << 1, 0, 0),
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "ADC Filter Capture Enum",
 .info = hpf_info,
 .get = hpf_get,
 .put = hpf_put,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Level Control Capture Enum",
 .info = wm8776_level_control_info,
 .get = wm8776_level_control_get,
 .put = wm8776_level_control_put,
 .private_value = 0,
 },
 };
 static const struct snd_kcontrol_new lc_controls[] = {
 WM8776_FIELD_CTL_VOLUME("Limiter Threshold",
 WM8776_ALCCTRL1, 0, 11, 0, 15, 0xf,
 LC_CONTROL_LIMITER, wm8776_lct_db_scale),
 WM8776_FIELD_CTL_ENUM("Limiter Attack Time",
 WM8776_ALCCTRL3, 0, 2, 0, 10, 0xf,
 LC_CONTROL_LIMITER),
 WM8776_FIELD_CTL_ENUM("Limiter Decay Time",
 WM8776_ALCCTRL3, 4, 3, 0, 10, 0xf,
 LC_CONTROL_LIMITER),
 WM8776_FIELD_CTL_ENUM("Limiter Transient Window",
 WM8776_LIMITER, 4, 2, 0, 7, 0x7,
 LC_CONTROL_LIMITER),
 WM8776_FIELD_CTL_VOLUME("Limiter Maximum Attenuation",
 WM8776_LIMITER, 0, 6, 3, 12, 0xf,
 LC_CONTROL_LIMITER,
 wm8776_maxatten_lim_db_scale),
 WM8776_FIELD_CTL_VOLUME("ALC Target Level",
 WM8776_ALCCTRL1, 0, 11, 0, 15, 0xf,
 LC_CONTROL_ALC, wm8776_lct_db_scale),
 WM8776_FIELD_CTL_ENUM("ALC Attack Time",
 WM8776_ALCCTRL3, 0, 2, 0, 10, 0xf,
 LC_CONTROL_ALC),
 WM8776_FIELD_CTL_ENUM("ALC Decay Time",
 WM8776_ALCCTRL3, 4, 3, 0, 10, 0xf,
 LC_CONTROL_ALC),
 WM8776_FIELD_CTL_VOLUME("ALC Maximum Gain",
 WM8776_ALCCTRL1, 4, 7, 1, 7, 0x7,
 LC_CONTROL_ALC, wm8776_maxgain_db_scale),
 WM8776_FIELD_CTL_VOLUME("ALC Maximum Attenuation",
 WM8776_LIMITER, 0, 10, 10, 15, 0xf,
 LC_CONTROL_ALC, wm8776_maxatten_alc_db_scale),
 WM8776_FIELD_CTL_ENUM("ALC Hold Time",
 WM8776_ALCCTRL2, 0, 0, 0, 15, 0xf,
 LC_CONTROL_ALC),
 WM8776_BIT_SWITCH("Noise Gate Capture Switch",
 WM8776_NOISEGATE, WM8776_NGAT, 0,
 LC_CONTROL_ALC),
 WM8776_FIELD_CTL_VOLUME("Noise Gate Threshold",
 WM8776_NOISEGATE, 2, 0, 0, 7, 0x7,
 LC_CONTROL_ALC, wm8776_ngth_db_scale),
 };
 
 static int add_lc_controls(struct oxygen *chip)
 {
 struct xonar_wm87x6 *data = (struct xonar_wm87x6*) &chip->model_data;
 unsigned int i;
 //struct snd_kcontrol *ctl;
 int err;
 
 BUILD_BUG_ON(ARRAY_SIZE(lc_controls) != ARRAY_SIZE(data->lc_controls));
 for (i = 0; i < ARRAY_SIZE(lc_controls); ++i) {
 ctl = snd_ctl_new1(&lc_controls[i], chip);
 if (!ctl)
 return -ENOMEM;
 err = snd_ctl_add(chip->card, ctl);
 if (err < 0)
 return err;
 data->lc_controls[i] = ctl;
 }
 return 0;
 }
 
 static int xonar_ds_mixer_init(struct oxygen *chip)
 {
 struct xonar_wm87x6 *data = chip->model_data;
 unsigned int i;
 struct snd_kcontrol *ctl;
 int err;
 
 for (i = 0; i < ARRAY_SIZE(ds_controls); ++i) {
 ctl = snd_ctl_new1(&ds_controls[i], chip);
 if (!ctl)
 return -ENOMEM;
 err = snd_ctl_add(chip->card, ctl);
 if (err < 0)
 return err;
 if (!strcmp(ctl->id.name, "Line Capture Switch"))
 data->line_adcmux_control = ctl;
 else if (!strcmp(ctl->id.name, "Mic Capture Switch"))
 data->mic_adcmux_control = ctl;
 }
 if (!data->line_adcmux_control || !data->mic_adcmux_control)
 return -ENXIO;
 
 return add_lc_controls(chip);
 }
 
 static int xonar_hdav_slim_mixer_init(struct oxygen *chip)
 {
 unsigned int i;
 struct snd_kcontrol *ctl;
 int err;
 
 for (i = 0; i < ARRAY_SIZE(hdav_slim_controls); ++i) {
 ctl = snd_ctl_new1(&hdav_slim_controls[i], chip);
 if (!ctl)
 return -ENOMEM;
 err = snd_ctl_add(chip->card, ctl);
 if (err < 0)
 return err;
 }
 
 return add_lc_controls(chip);
 }
 
 static void dump_wm8776_registers(struct oxygen *chip,
 struct snd_info_buffer *buffer)
 {
 struct xonar_wm87x6 *data = chip->model_data;
 unsigned int i;
 
 snd_iprintf(buffer, "\nWM8776:\n00:");
 for (i = 0; i < 0x10; ++i)
 snd_iprintf(buffer, " %03x", data->wm8776_regs[i]);
 snd_iprintf(buffer, "\n10:");
 for (i = 0x10; i < 0x17; ++i)
 snd_iprintf(buffer, " %03x", data->wm8776_regs[i]);
 snd_iprintf(buffer, "\n");
 }
 
 static void dump_wm87x6_registers(struct oxygen *chip,
 struct snd_info_buffer *buffer)
 {
 struct xonar_wm87x6 *data = chip->model_data;
 unsigned int i;
 
 dump_wm8776_registers(chip, buffer);
 snd_iprintf(buffer, "\nWM8766:\n00:");
 for (i = 0; i < 0x10; ++i)
 snd_iprintf(buffer, " %03x", data->wm8766_regs[i]);
 snd_iprintf(buffer, "\n");
 }
 
 */




