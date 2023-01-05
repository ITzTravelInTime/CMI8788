#include <IOKit/IOFilterInterruptEventSource.h>
#include "XonarGenericAudioEngine.hpp"

#include "oxygen.h"
#include "alsa.h"
#include "ak4396.h"
#include "wm8785.h"

#define INITIAL_SAMPLE_RATE	44100
#define NUM_SAMPLE_FRAMES	16384
#define NUM_CHANNELS		2
#define BIT_DEPTH			16


//#include <architecture/i386/pio.h>

/*
 * C-Media CMI8788 driver for C-Media's reference design and similar models
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
 *  along with this driver; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

/*
 * CMI8788:
 *
 *   SPI 0 -> 1st AK4396 (front)
 *   SPI 1 -> 2nd AK4396 (surround)
 *   SPI 2 -> 3rd AK4396 (center/LFE)
 *   SPI 3 -> WM8785
 *   SPI 4 -> 4th AK4396 (back)
 *
 *   GPIO 0 -> DFS0 of AK5385
 *   GPIO 1 -> DFS1 of AK5385
 *
 * X-Meridian models:
 *   GPIO 4 -> enable extension S/PDIF input
 *   GPIO 6 -> enable on-board S/PDIF input
 *
 * Claro models:
 *   GPIO 6 -> S/PDIF from optical (0) or coaxial (1) input
 *   GPIO 8 -> enable headphone amplifier
 *
 * CM9780:
 *
 *   LINE_OUT -> input of ADC
 *
 *   AUX_IN <- aux
 *   CD_IN  <- CD
 *   MIC_IN <- mic
 *
 *   GPO 0 -> route line-in (0) or AC97 output (1) to ADC input
 */

#define super IOAudioEngine


OSDefineMetaClassAndStructors(XonarGenericAudioEngine, IOAudioEngine)



//static const struct pci_device_id oxygen_ids[] = {
//    /* C-Media's reference design */
//    { OXYGEN_PCI_SUBID(0x10b0, 0x0216), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x10b0, 0x0217), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x10b0, 0x0218), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x10b0, 0x0219), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x13f6, 0x0001), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x13f6, 0x0010), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x13f6, 0x8788), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x147a, 0xa017), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x1a58, 0x0910), .driver_data = MODEL_CMEDIA_REF },
//    /* Asus Xonar DG */
//    { OXYGEN_PCI_SUBID(0x1043, 0x8467), .driver_data = MODEL_XONAR_DG },
//    /* Asus Xonar DGX */
//    { OXYGEN_PCI_SUBID(0x1043, 0x8521), .driver_data = MODEL_XONAR_DGX },
//    /* PCI 2.0 HD Audio */
//    { OXYGEN_PCI_SUBID(0x13f6, 0x8782), .driver_data = MODEL_2CH_OUTPUT },
//    /* Kuroutoshikou CMI8787-HG2PCI */
//    { OXYGEN_PCI_SUBID(0x13f6, 0xffff), .driver_data = MODEL_HG2PCI },
//    /* TempoTec HiFier Fantasia */
//    { OXYGEN_PCI_SUBID(0x14c3, 0x1710), .driver_data = MODEL_FANTASIA },
//    /* TempoTec HiFier Serenade */
//    { OXYGEN_PCI_SUBID(0x14c3, 0x1711), .driver_data = MODEL_SERENADE },
//    /* AuzenTech X-Meridian */
//    { OXYGEN_PCI_SUBID(0x415a, 0x5431), .driver_data = MODEL_MERIDIAN },
//    /* AuzenTech X-Meridian 2G */
//    { OXYGEN_PCI_SUBID(0x5431, 0x017a), .driver_data = MODEL_MERIDIAN_2G },
//    /* HT-Omega Claro */
//    { OXYGEN_PCI_SUBID(0x7284, 0x9761), .driver_data = MODEL_CLARO },
//    /* HT-Omega Claro halo */
//    { OXYGEN_PCI_SUBID(0x7284, 0x9781), .driver_data = MODEL_CLARO_HALO },
//    { }
//};

static void dump_ak4396_registers(struct oxygen *chip,
                                  struct snd_info_buffer *buffer)
{
        struct generic_data *data = (struct generic_data*)chip->model_data;
        unsigned int dac, i;
        
        for (dac = 0; dac < data->dacs; ++dac) {
                snd_iprintf(buffer, "\nAK4396 %u:", dac + 1);
                for (i = 0; i < 5; ++i)
                        snd_iprintf(buffer, " %02x", data->ak4396_regs[dac][i]);
        }
        snd_iprintf(buffer, "\n");
}

static void dump_wm8785_registers(struct oxygen *chip,
                                  struct snd_info_buffer *buffer)
{
        struct generic_data *data = (struct generic_data*) chip->model_data;
        unsigned int i;
        
        snd_iprintf(buffer, "\nWM8785:");
        for (i = 0; i < 3; ++i)
                snd_iprintf(buffer, " %03x", data->wm8785_regs[i]);
        snd_iprintf(buffer, "\n");
}

static void dump_oxygen_registers(struct oxygen *chip,
                                  struct snd_info_buffer *buffer)
{
        dump_ak4396_registers(chip, buffer);
        dump_wm8785_registers(chip, buffer);
}



int cs4245_read_spi(struct oxygen *chip, UInt8 addr)
{
        struct dg *data = (struct dg*) chip->model_data;
        int ret;
        
        ret = oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER |
                               OXYGEN_SPI_DATA_LENGTH_2 |
                               OXYGEN_SPI_CEN_LATCH_CLOCK_HI |
                               OXYGEN_SPI_CLOCK_1280 | (0 << OXYGEN_SPI_CODEC_SHIFT),
                               ((CS4245_SPI_ADDRESS | CS4245_SPI_WRITE) << 8) | addr);
        if (ret < 0)
                return ret;
        
        ret = oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER |
                               OXYGEN_SPI_DATA_LENGTH_2 |
                               OXYGEN_SPI_CEN_LATCH_CLOCK_HI |
                               OXYGEN_SPI_CLOCK_1280 | (0 << OXYGEN_SPI_CODEC_SHIFT),
                               (CS4245_SPI_ADDRESS | CS4245_SPI_READ) << 8);
        if (ret < 0)
                return ret;
        
        data->cs4245_shadow[addr] = oxygen_read8(chip, OXYGEN_SPI_DATA1);
        
        return 0;
}


static int output_select_apply(struct oxygen *chip)
{
        struct dg *data = (struct dg*) chip->model_data;
        
        data->cs4245_shadow[CS4245_SIGNAL_SEL] &= ~CS4245_A_OUT_SEL_MASK;
        if (data->output_sel == PLAYBACK_DST_HP) {
                /* mute FP (aux output) amplifier, switch rear jack to CS4245 */
                oxygen_set_bits8(chip, OXYGEN_GPIO_DATA, GPIO_HP_REAR);
        } else if (data->output_sel == PLAYBACK_DST_HP_FP) {
                /*
                 * Unmute FP amplifier, switch rear jack to CS4361;
                 * I2S channels 2,3,4 should be inactive.
                 */
                oxygen_clear_bits8(chip, OXYGEN_GPIO_DATA, GPIO_HP_REAR);
                data->cs4245_shadow[CS4245_SIGNAL_SEL] |= CS4245_A_OUT_SEL_DAC;
        } else {
                /*
                 * 2.0, 4.0, 5.1: switch to CS4361, mute FP amp.,
                 * and change playback routing.
                 */
                oxygen_clear_bits8(chip, OXYGEN_GPIO_DATA, GPIO_HP_REAR);
        }
        return cs4245_write_spi(chip, CS4245_SIGNAL_SEL);
}


int XonarGenericAudioEngine::output_select_put(struct snd_kcontrol *ctl,
                                               struct snd_ctl_elem_value *value)
{
        struct oxygen *chip = (struct oxygen*) ctl->private_data;
        struct dg *data = (struct dg*) chip->model_data;
        unsigned int newval = value->value.enumerated.item[0];
        int changed = 0;
        int ret;
        
        IOLockLock(chip->mutex);
        if (data->output_sel != newval) {
                data->output_sel = newval;
                ret = output_select_apply(chip);
                changed = ret >= 0 ? 1 : ret;
                engineInstance->oxygen_update_dac_routing(chip);
        }
        IOLockUnlock(chip->mutex);
        
        return changed;
}

#define INPUT_VOLUME(xname, index) { \
.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
.name = xname, \
.access = SNDRV_CTL_ELEM_ACCESS_READWRITE | \
SNDRV_CTL_ELEM_ACCESS_TLV_READ, \
.info = input_vol_info, \
.get = input_vol_get, \
.put = input_vol_put, \
.tlv = { .p = pga_db_scale }, \
.private_value = index, \
}
static const DECLARE_TLV_DB_MINMAX(hp_db_scale, -12550, 0);
static const DECLARE_TLV_DB_MINMAX(pga_db_scale, -1200, 1200);
/*static const struct snd_kcontrol_new dg_controls[] = {
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Analog Output Playback Enum",
 .info = output_select_info,
 .get = output_select_get,
 .put = output_select_put,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Headphone Playback Volume",
 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
 SNDRV_CTL_ELEM_ACCESS_TLV_READ,
 .info = hp_stereo_volume_info,
 .get = hp_stereo_volume_get,
 .put = hp_stereo_volume_put,
 .tlv = { .p = hp_db_scale, },
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Headphone Playback Switch",
 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
 .info = snd_ctl_boolean_mono_info,
 .get = hp_mute_get,
 .put = hp_mute_put,
 },
 INPUT_VOLUME("Mic Capture Volume", CAPTURE_SRC_MIC),
 INPUT_VOLUME("Front Mic Capture Volume", CAPTURE_SRC_FP_MIC),
 INPUT_VOLUME("Line Capture Volume", CAPTURE_SRC_LINE),
 INPUT_VOLUME("Aux Capture Volume", CAPTURE_SRC_AUX),
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Capture Source",
 .info = input_sel_info,
 .get = input_sel_get,
 .put = input_sel_put,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "ADC High-pass Filter Capture Enum",
 .info = hpf_info,
 .get = hpf_get,
 .put = hpf_put,
 },
 };
 */

static int dg_control_filter(struct snd_kcontrol_new *_template)
{
        if (!strncmp(_template->name, "Master Playback ", 16))
                return 1;
        return 0;
}

static int dg_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *engineInstance)
{
        unsigned int i;
        int err;
        
        output_select_apply(chip);
        input_source_apply(chip);
        engineInstance->oxygen_update_dac_routing(chip);
        
        //    for (i = 0; i < ARRAY_SIZE(dg_controls); ++i) {
        //GOTTA FIX TSHIS
        //        err = snd_ctl_add(chip->card,
        //                  snd_ctl_new1(&dg_controls[i], chip));
        //        if (err < 0)
        //            return err;
        //   }
        
        return 0;
}



int cs4245_shadow_control(struct oxygen *chip, enum cs4245_shadow_operation op)
{
        struct dg *data = (struct dg*) chip->model_data;
        unsigned char addr;
        int ret;
        
        for (addr = 1; addr < ARRAY_SIZE(data->cs4245_shadow); addr++) {
                ret = (op == CS4245_SAVE_TO_SHADOW ?
                       cs4245_read_spi(chip, addr) :
                       cs4245_write_spi(chip, addr));
                if (ret < 0)
                        return ret;
        }
        return 0;
}

void set_cs4245_dac_params(struct oxygen *chip, XonarAudioEngine *engineInstance,
                           IOAudioStream *currentStream)
{
        struct dg *data = (struct dg*) chip->model_data;
        unsigned char dac_ctrl;
        unsigned char mclk_freq;
        
        dac_ctrl = data->cs4245_shadow[CS4245_DAC_CTRL_1] & ~CS4245_DAC_FM_MASK;
        mclk_freq = data->cs4245_shadow[CS4245_MCLK_FREQ] & ~CS4245_MCLK1_MASK;
        if (engineInstance->getSampleRate()->whole <= 50000) {
                dac_ctrl |= CS4245_DAC_FM_SINGLE;
                mclk_freq |= CS4245_MCLK_1 << CS4245_MCLK1_SHIFT;
        } else if (engineInstance->getSampleRate()->whole <= 100000) {
                dac_ctrl |= CS4245_DAC_FM_DOUBLE;
                mclk_freq |= CS4245_MCLK_1 << CS4245_MCLK1_SHIFT;
        } else {
                dac_ctrl |= CS4245_DAC_FM_QUAD;
                mclk_freq |= CS4245_MCLK_2 << CS4245_MCLK1_SHIFT;
        }
        data->cs4245_shadow[CS4245_DAC_CTRL_1] = dac_ctrl;
        data->cs4245_shadow[CS4245_MCLK_FREQ] = mclk_freq;
        cs4245_write_spi(chip, CS4245_DAC_CTRL_1);
        cs4245_write_spi(chip, CS4245_MCLK_FREQ);
}

void set_cs4245_adc_params(struct oxygen *chip,
                           XonarAudioEngine *engineInstance)
{
        struct dg *data = (struct dg*) chip->model_data;
        unsigned char adc_ctrl;
        unsigned char mclk_freq;
        
        adc_ctrl = data->cs4245_shadow[CS4245_ADC_CTRL] & ~CS4245_ADC_FM_MASK;
        mclk_freq = data->cs4245_shadow[CS4245_MCLK_FREQ] & ~CS4245_MCLK2_MASK;
        if (engineInstance->getSampleRate()->whole <= 50000) {
                adc_ctrl |= CS4245_ADC_FM_SINGLE;
                mclk_freq |= CS4245_MCLK_1 << CS4245_MCLK2_SHIFT;
        } else if (engineInstance->getSampleRate()->whole <= 100000) {
                adc_ctrl |= CS4245_ADC_FM_DOUBLE;
                mclk_freq |= CS4245_MCLK_1 << CS4245_MCLK2_SHIFT;
        } else {
                adc_ctrl |= CS4245_ADC_FM_QUAD;
                mclk_freq |= CS4245_MCLK_2 << CS4245_MCLK2_SHIFT;
        }
        data->cs4245_shadow[CS4245_ADC_CTRL] = adc_ctrl;
        data->cs4245_shadow[CS4245_MCLK_FREQ] = mclk_freq;
        cs4245_write_spi(chip, CS4245_ADC_CTRL);
        cs4245_write_spi(chip, CS4245_MCLK_FREQ);
}

static inline unsigned int shift_bits(unsigned int value,
                                      unsigned int shift_from,
                                      unsigned int shift_to,
                                      unsigned int mask)
{
        if (shift_from < shift_to)
                return (value << (shift_to - shift_from)) & mask;
        else
                return (value >> (shift_from - shift_to)) & mask;
}

unsigned int adjust_dg_dac_routing(struct oxygen *chip,
                                   unsigned int play_routing)
{
        struct dg *data = (struct dg*) chip->model_data;
        
        switch (data->output_sel) {
                case PLAYBACK_DST_HP:
                case PLAYBACK_DST_HP_FP:
                        oxygen_write8_masked(chip, OXYGEN_PLAY_ROUTING,
                                             OXYGEN_PLAY_MUTE23 | OXYGEN_PLAY_MUTE45 |
                                             OXYGEN_PLAY_MUTE67, OXYGEN_PLAY_MUTE_MASK);
                        break;
                case PLAYBACK_DST_MULTICH:
                        oxygen_write8_masked(chip, OXYGEN_PLAY_ROUTING,
                                             OXYGEN_PLAY_MUTE01, OXYGEN_PLAY_MUTE_MASK);
                        break;
        }
        return (play_routing & OXYGEN_PLAY_DAC0_SOURCE_MASK) |
        shift_bits(play_routing,
                   OXYGEN_PLAY_DAC2_SOURCE_SHIFT,
                   OXYGEN_PLAY_DAC1_SOURCE_SHIFT,
                   OXYGEN_PLAY_DAC1_SOURCE_MASK) |
        shift_bits(play_routing,
                   OXYGEN_PLAY_DAC1_SOURCE_SHIFT,
                   OXYGEN_PLAY_DAC2_SOURCE_SHIFT,
                   OXYGEN_PLAY_DAC2_SOURCE_MASK) |
        shift_bits(play_routing,
                   OXYGEN_PLAY_DAC0_SOURCE_SHIFT,
                   OXYGEN_PLAY_DAC3_SOURCE_SHIFT,
                   OXYGEN_PLAY_DAC3_SOURCE_MASK);
}

void dump_cs4245_registers(struct oxygen *chip,
                           struct snd_info_buffer *buffer)
{
        struct dg *data = (struct dg*) chip->model_data;
        unsigned int addr;
        
        snd_iprintf(buffer, "\nCS4245:");
        cs4245_read_spi(chip, CS4245_INT_STATUS);
        for (addr = 1; addr < ARRAY_SIZE(data->cs4245_shadow); addr++)
                snd_iprintf(buffer, " %02x", data->cs4245_shadow[addr]);
        snd_iprintf(buffer, "\n");
}

static void cs4245_init(struct oxygen *chip)
{
        struct dg *data = (struct dg*) chip->model_data;
        
        /* save the initial state: codec version, registers */
        cs4245_shadow_control(chip, CS4245_SAVE_TO_SHADOW);
        
        /*
         * Power up the CODEC internals, enable soft ramp & zero cross, work in
         * async. mode, enable aux output from DAC. Invert DAC output as in the
         * Windows driver.
         */
        data->cs4245_shadow[CS4245_POWER_CTRL] = 0;
        data->cs4245_shadow[CS4245_SIGNAL_SEL] =
        CS4245_A_OUT_SEL_DAC | CS4245_ASYNCH;
        data->cs4245_shadow[CS4245_DAC_CTRL_1] =
        CS4245_DAC_FM_SINGLE | CS4245_DAC_DIF_LJUST;
        data->cs4245_shadow[CS4245_DAC_CTRL_2] =
        CS4245_DAC_SOFT | CS4245_DAC_ZERO | CS4245_INVERT_DAC;
        data->cs4245_shadow[CS4245_ADC_CTRL] =
        CS4245_ADC_FM_SINGLE | CS4245_ADC_DIF_LJUST;
        data->cs4245_shadow[CS4245_ANALOG_IN] =
        CS4245_PGA_SOFT | CS4245_PGA_ZERO;
        data->cs4245_shadow[CS4245_PGA_B_CTRL] = 0;
        data->cs4245_shadow[CS4245_PGA_A_CTRL] = 0;
        data->cs4245_shadow[CS4245_DAC_A_CTRL] = 8;
        data->cs4245_shadow[CS4245_DAC_B_CTRL] = 8;
        
        cs4245_shadow_control(chip, CS4245_LOAD_FROM_SHADOW);
        //GOTTA FIX THIS
        //snd_component_add(chip->card, "CS4245");
}




void dg_init(struct oxygen *chip)
{
        struct dg *data = (struct dg*) chip->model_data;
        
        data->output_sel = PLAYBACK_DST_HP_FP;
        data->input_sel = CAPTURE_SRC_MIC;
        
        cs4245_init(chip);
        oxygen_write16(chip, OXYGEN_GPIO_CONTROL,
                       GPIO_OUTPUT_ENABLE | GPIO_HP_REAR | GPIO_INPUT_ROUTE);
        /* anti-pop delay, wait some time before enabling the output */
        IOSleep(2500);
        oxygen_write16(chip, OXYGEN_GPIO_DATA,
                       GPIO_OUTPUT_ENABLE | GPIO_INPUT_ROUTE);
}

void dg_cleanup(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_OUTPUT_ENABLE);
}

void dg_suspend(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        dg_cleanup(chip, engineInstance);
}

void dg_resume(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        cs4245_shadow_control(chip, CS4245_LOAD_FROM_SHADOW);
        IOSleep(2500);
        oxygen_set_bits16(chip, OXYGEN_GPIO_DATA, GPIO_OUTPUT_ENABLE);
}

void XonarGenericAudioEngine::wm8785_write(struct oxygen *chip, UInt8 reg, unsigned int value)
{
        
        struct generic_data *data = (struct generic_data*) chip->model_data;
        
        oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER |
                         OXYGEN_SPI_DATA_LENGTH_2 |
                         OXYGEN_SPI_CLOCK_160 |
                         (3 << OXYGEN_SPI_CODEC_SHIFT) |
                         OXYGEN_SPI_CEN_LATCH_CLOCK_LO,
                         (reg << 9) | value);
        if (reg < ARRAY_SIZE(data->wm8785_regs))
                data->wm8785_regs[reg] = value;
}


void XonarGenericAudioEngine::ak4396_write(struct oxygen *chip, unsigned int codec,
                                           UInt8 reg, UInt8 value)
{
        /* maps ALSA channel pair number to SPI output */
        static const UInt8 codec_spi_map[4] = {
                0, 1, 2, 4
        };
        struct generic_data *data = (struct generic_data*) chip->model_data;
        
        oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER |
                         OXYGEN_SPI_DATA_LENGTH_2 |
                         OXYGEN_SPI_CLOCK_160 |
                         (codec_spi_map[codec] << OXYGEN_SPI_CODEC_SHIFT) |
                         OXYGEN_SPI_CEN_LATCH_CLOCK_HI,
                         AK4396_WRITE | (reg << 8) | value);
        data->ak4396_regs[codec][reg] = value;
}


void XonarGenericAudioEngine::ak4396_write_cached(struct oxygen *chip, unsigned int codec,
                                                  UInt8 reg, UInt8 value)
{
        struct generic_data *data = (struct generic_data*) chip->model_data;
        
        if (value != data->ak4396_regs[codec][reg])
                ak4396_write(chip, codec, reg, value);
}


void XonarGenericAudioEngine::ak4396_registers_init(struct oxygen *chip)
{
        struct generic_data *data = (struct generic_data*) chip->model_data;
        unsigned int i;
        
        for (i = 0; i < data->dacs; ++i) {
                ak4396_write(chip, i, AK4396_CONTROL_1,
                             AK4396_DIF_24_MSB | AK4396_RSTN);
                ak4396_write(chip, i, AK4396_CONTROL_2,
                             data->ak4396_regs[0][AK4396_CONTROL_2]);
                ak4396_write(chip, i, AK4396_CONTROL_3,
                             AK4396_PCM);
                ak4396_write(chip, i, AK4396_LCH_ATT,
                             chip->dac_volume[i * 2]);
                ak4396_write(chip, i, AK4396_RCH_ATT,
                             chip->dac_volume[i * 2 + 1]);
        }
}

void XonarGenericAudioEngine::ak4396_init(struct oxygen *chip)
{
        struct generic_data *data = (struct generic_data*) chip->model_data;
        
        data->dacs = chip->model.dac_channels_pcm / 2;
        data->ak4396_regs[0][AK4396_CONTROL_2] =
        AK4396_SMUTE | AK4396_DEM_OFF | AK4396_DFS_NORMAL;
        ak4396_registers_init(chip);
        // snd_component_add(chip->card, "AK4396");
}

void XonarGenericAudioEngine::ak5385_init(struct oxygen *chip)
{
        oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_AK5385_DFS_MASK);
        oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_AK5385_DFS_MASK);
        //  snd_component_add(chip->card, "AK5385");
}

void XonarGenericAudioEngine::wm8785_registers_init(struct oxygen *chip)
{
        struct generic_data *data = (struct generic_data*) chip->model_data;
        
        wm8785_write(chip, WM8785_R7, 0);
        wm8785_write(chip, WM8785_R0, data->wm8785_regs[0]);
        wm8785_write(chip, WM8785_R2, data->wm8785_regs[2]);
}

void XonarGenericAudioEngine::wm8785_init(struct oxygen *chip)
{
        struct generic_data *data = (struct generic_data*) chip->model_data;
        
        data->wm8785_regs[0] =
        WM8785_MCR_SLAVE | WM8785_OSR_SINGLE | WM8785_FORMAT_LJUST;
        data->wm8785_regs[2] = WM8785_HPFR | WM8785_HPFL;
        wm8785_registers_init(chip);
        // snd_component_add(chip->card, "WM8785");
}


void XonarGenericAudioEngine::generic_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        ak4396_init(chip);
        wm8785_init(chip);
}

void XonarGenericAudioEngine::meridian_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL,
                          GPIO_MERIDIAN_DIG_MASK);
        oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                              GPIO_MERIDIAN_DIG_BOARD, GPIO_MERIDIAN_DIG_MASK);
        ak4396_init(chip);
        ak5385_init(chip);
}

void XonarGenericAudioEngine::claro_enable_hp(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        IODelay(300);
        oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_CLARO_HP);
        oxygen_set_bits16(chip, OXYGEN_GPIO_DATA, GPIO_CLARO_HP);
}

void XonarGenericAudioEngine::claro_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_CLARO_DIG_COAX);
        oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_CLARO_DIG_COAX);
        ak4396_init(chip);
        wm8785_init(chip);
        claro_enable_hp(chip, engineInstance);
}

void XonarGenericAudioEngine::claro_halo_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_CLARO_DIG_COAX);
        oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_CLARO_DIG_COAX);
        ak4396_init(chip);
        ak5385_init(chip);
        claro_enable_hp(chip, engineInstance);
}

void XonarGenericAudioEngine::fantasia_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        ak4396_init(chip);
        //snd_component_add(chip->card, "CS5340");
}

void XonarGenericAudioEngine::stereo_output_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        ak4396_init(chip);
}
/*
 static const struct snd_kcontrol_new rolloff_control = {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "DAC Filter Playback Enum",
 .info = rolloff_info,
 .get = rolloff_get,
 .put = rolloff_put,
 };
 */
int XonarGenericAudioEngine::generic_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *engineInstance)
{
        
        
        return 0; //GOTTA FIX THIS
        //snd_ctl_add(chip->card, snd_ctl_new1(&rolloff_control, chip));
}

int XonarGenericAudioEngine::generic_wm8785_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *engineInstance)
{
        int err;
        
        err = generic_mixer_init(chip, dev, engineInstance);
        if (err < 0)
                return err;
        //GOTTA FIX THIS
        //err = snd_ctl_add(chip->card, snd_ctl_new1(&hpf_control, chip));
        //if (err < 0)
        //return err;
        return 0;
}

int XonarGenericAudioEngine::meridian_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *engineInstance)
{
        int err;
        
        err = generic_mixer_init(chip, dev, engineInstance);
        if (err < 0)
                return err;
        //GOTTA FIX THIS
        // err = snd_ctl_add(chip->card,
        // snd_ctl_new1(&meridian_dig_source_control, chip));
        // if (err < 0)
        // return err;
        return 0;
}

int XonarGenericAudioEngine::claro_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *engineInstance)
{
        int err;
        
        err = generic_wm8785_mixer_init(chip, dev, engineInstance);
        if (err < 0)
                return err;
        //GOTTA FIX THIS
        // err = snd_ctl_add(chip->card,
        // snd_ctl_new1(&claro_dig_source_control, chip));
        // if (err < 0)
        // return err;
        return 0;
}

int XonarGenericAudioEngine::claro_halo_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *engineInstance)
{
        int err;
        
        err = generic_mixer_init(chip, dev, engineInstance);
        if (err < 0)
                return err;
        //GOTTA FIX THIS
        //     err = snd_ctl_add(chip->card,
        // snd_ctl_new1(&claro_dig_source_control, chip));
        // if (err < 0)
        // return err;
        return 0;
}



void XonarGenericAudioEngine::generic_cleanup(struct oxygen *chip, XonarAudioEngine *audioEngineInstance)
{
}

void XonarGenericAudioEngine::claro_disable_hp(struct oxygen *chip)
{
        oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_CLARO_HP);
}

void XonarGenericAudioEngine::claro_cleanup(struct oxygen *chip,
                                            XonarAudioEngine *audioEngineInstance)
{
        claro_disable_hp(chip);
}

void XonarGenericAudioEngine::claro_suspend(struct oxygen *chip,
                                            XonarAudioEngine *audioEngineInstance)
{
        claro_disable_hp(chip);
}

void XonarGenericAudioEngine::generic_resume(struct oxygen *chip,
                                             XonarAudioEngine *audioEngineInstance)
{
        ak4396_registers_init(chip);
        wm8785_registers_init(chip);
}

void XonarGenericAudioEngine::meridian_resume(struct oxygen *chip,
                                              XonarAudioEngine *audioEngineInstance)
{
        ak4396_registers_init(chip);
}

void XonarGenericAudioEngine::claro_resume(struct oxygen *chip,
                                           XonarAudioEngine *audioEngineInstance)
{
        ak4396_registers_init(chip);
        claro_enable_hp(chip, audioEngineInstance);
}

void XonarGenericAudioEngine::stereo_resume(struct oxygen *chip,
                                            XonarAudioEngine *audioEngineInstance)
{
        ak4396_registers_init(chip);
}

void XonarGenericAudioEngine::set_ak4396_params(struct oxygen *chip, XonarAudioEngine *engineInstance,
                                                IOAudioStream *currentStream)
{
        struct generic_data *data = (struct generic_data*) chip->model_data;
        unsigned int i;
        UInt8 value;
        
        value = data->ak4396_regs[0][AK4396_CONTROL_2] & ~AK4396_DFS_MASK;
        if (engineInstance->getSampleRate()->whole <= 54000)
                value |= AK4396_DFS_NORMAL;
        else if (engineInstance->getSampleRate()->whole <= 108000)
                value |= AK4396_DFS_DOUBLE;
        else
                value |= AK4396_DFS_QUAD;
        
        IODelay(1); /* wait for the new MCLK to become stable */
        
        if (value != data->ak4396_regs[0][AK4396_CONTROL_2]) {
                for (i = 0; i < data->dacs; ++i) {
                        ak4396_write(chip, i, AK4396_CONTROL_1,
                                     AK4396_DIF_24_MSB);
                        ak4396_write(chip, i, AK4396_CONTROL_2, value);
                        ak4396_write(chip, i, AK4396_CONTROL_1,
                                     AK4396_DIF_24_MSB | AK4396_RSTN);
                }
        }
}

static void set_no_params(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
}

void XonarGenericAudioEngine::update_ak4396_volume(struct oxygen *chip)
{
        struct generic_data *data = (struct generic_data*) chip->model_data;
        unsigned int i;
        
        for (i = 0; i < data->dacs; ++i) {
                ak4396_write_cached(chip, i, AK4396_LCH_ATT,
                                    chip->dac_volume[i * 2]);
                ak4396_write_cached(chip, i, AK4396_RCH_ATT,
                                    chip->dac_volume[i * 2 + 1]);
        }
}

void XonarGenericAudioEngine::update_ak4396_mute(struct oxygen *chip)
{
        struct generic_data *data = (struct generic_data*) chip->model_data;
        unsigned int i;
        UInt8 value;
        
        value = data->ak4396_regs[0][AK4396_CONTROL_2] & ~AK4396_SMUTE;
        if (chip->dac_mute)
                value |= AK4396_SMUTE;
        for (i = 0; i < data->dacs; ++i)
                ak4396_write_cached(chip, i, AK4396_CONTROL_2, value);
}

void XonarGenericAudioEngine::set_wm8785_params(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        struct generic_data *data = (struct generic_data*) chip->model_data;
        unsigned int value;
        
        value = WM8785_MCR_SLAVE | WM8785_FORMAT_LJUST;
        if (engineInstance->getSampleRate()->whole <= 48000)
                value |= WM8785_OSR_SINGLE;
        else if (engineInstance->getSampleRate()->whole <= 96000)
                value |= WM8785_OSR_DOUBLE;
        else
                value |= WM8785_OSR_QUAD;
        if (value != data->wm8785_regs[0]) {
                wm8785_write(chip, WM8785_R7, 0);
                wm8785_write(chip, WM8785_R0, value);
                wm8785_write(chip, WM8785_R2, data->wm8785_regs[2]);
        }
}

void XonarGenericAudioEngine::set_ak5385_params(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        unsigned int value;
        
        if (engineInstance->getSampleRate()->whole <= 54000)
                value = GPIO_AK5385_DFS_NORMAL;
        else if (engineInstance->getSampleRate()->whole <= 108000)
                value = GPIO_AK5385_DFS_DOUBLE;
        else
                value = GPIO_AK5385_DFS_QUAD;
        oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                              value, GPIO_AK5385_DFS_MASK);
}

static const char *get_oxygen_model(int id)
{
        static const char *const names[] = {
                [MODEL_MERIDIAN]        = "AuzenTech X-Meridian",
                [MODEL_MERIDIAN_2G]        = "AuzenTech X-Meridian 2G",
                [MODEL_CLARO]                = "HT-Omega Claro",
                [MODEL_CLARO_HALO]        = "HT-Omega Claro halo",
                [MODEL_FANTASIA]        = "TempoTec HiFier Fantasia",
                [MODEL_SERENADE]        = "TempoTec HiFier Serenade",
                [MODEL_HG2PCI]                = "CMI8787-HG2PCI",
                [MODEL_XONAR_DG]        = "Xonar DG",
                [MODEL_XONAR_DGX]       = "Xonar DGX",
        };
        
        return names[id];
}


bool XonarGenericAudioEngine::init(XonarAudioEngine *audioEngine, struct oxygen *chip, UInt16 model)
{
        //begin APPUL portion of sampleaudioengine::init
        bool result = false;
        
        printf("XonarGenericAudioEngine[%p]::init(%p)\n", this, chip);
        
        if (!chip) {
                goto Done;
        }
        
        if (!audioEngine->init(chip, model)) {
                goto Done;
        }
        data_size = chip->model.model_data_size;
        deviceRegisters = (struct xonar_generic*) chip->model_data;
        engineInstance = audioEngine;
        chip->model.init = generic_init;
        //end APPUL portion of sampleaudioengine::init
        
        /* assign remaining values for oxygen_generic struct
         * (majority of assignments are in the 'else' of
         * XonarAudioEngine.cpp
         */
        
        chip->model.shortname = "C-Media CMI8788";
        chip->model.mixer_init = generic_wm8785_mixer_init;
        chip->model.cleanup = generic_cleanup;
        chip->model.resume = generic_resume;
        /* still not sure about these ALSA dac/adc_{params,volume,mute}
         * fields. i am leaving them commented out as a reference when
         * looking at how to incorporate the equivalent using IOAudiostream
         */
        chip->model.update_dac_volume = update_ak4396_volume;
        chip->model.update_dac_mute = update_ak4396_mute;
        
        chip->model.set_dac_params = set_ak4396_params;
        chip->model.set_adc_params = set_wm8785_params;
        chip->model.dump_registers = dump_oxygen_registers;
        //.dac_tlv = ak4396_db_scale,
        
        
        switch (model) {
                case MODEL_MERIDIAN:
                case MODEL_MERIDIAN_2G:
                        chip->model.init = meridian_init;
                        chip->model.mixer_init = meridian_mixer_init;
                        chip->model.resume = meridian_resume;
                        chip->model.set_adc_params = set_ak5385_params;
                        chip->model.dump_registers = dump_ak4396_registers;
                        chip->model.device_config = PLAYBACK_0_TO_I2S |
                        PLAYBACK_1_TO_SPDIF |
                        CAPTURE_0_FROM_I2S_2 |
                        CAPTURE_1_FROM_SPDIF;
                        if (model == MODEL_MERIDIAN)
                                chip->model.device_config |= AC97_CD_INPUT;
                        break;
                case MODEL_CLARO:
                        chip->model.init = claro_init;
                        chip->model.mixer_init = claro_mixer_init;
                        chip->model.cleanup = claro_cleanup;
                        chip->model.suspend = claro_suspend;
                        chip->model.resume = claro_resume;
                        break;
                case MODEL_CLARO_HALO:
                        chip->model.init = claro_halo_init;
                        chip->model.mixer_init = claro_halo_mixer_init;
                        chip->model.cleanup = claro_cleanup;
                        chip->model.suspend = claro_suspend;
                        chip->model.resume = claro_resume;
                        chip->model.set_adc_params = set_ak5385_params;
                        chip->model.dump_registers = dump_ak4396_registers;
                        chip->model.device_config = PLAYBACK_0_TO_I2S |
                        PLAYBACK_1_TO_SPDIF |
                        CAPTURE_0_FROM_I2S_2 |
                        CAPTURE_1_FROM_SPDIF;
                        break;
                case MODEL_FANTASIA:
                case MODEL_SERENADE:
                case MODEL_2CH_OUTPUT:
                case MODEL_HG2PCI:
                        chip->model.shortname = "C-Media CMI8787";
                        //chip->model.chip = "CMI8787";
                        if (model == MODEL_FANTASIA)
                                chip->model.init = fantasia_init;
                        else
                                chip->model.init = stereo_output_init;
                        chip->model.resume = stereo_resume;
                        chip->model.mixer_init = generic_mixer_init;
                        chip->model.set_adc_params = set_no_params;
                        chip->model.dump_registers = dump_ak4396_registers;
                        chip->model.device_config = PLAYBACK_0_TO_I2S |
                        PLAYBACK_1_TO_SPDIF;
                        if (model == MODEL_FANTASIA) {
                                chip->model.device_config |= CAPTURE_0_FROM_I2S_1;
                                chip->model.adc_mclks = OXYGEN_MCLKS(256, 128, 128);
                        }
                        chip->model.dac_channels_pcm = 2;
                        chip->model.dac_channels_mixer = 2;
                        break;
                case MODEL_XONAR_DG:
                case MODEL_XONAR_DGX:
                        chip->model.control_filter = dg_control_filter;
                        chip->model.shortname = "Xonar DG/X";
                        chip->model.mixer_init = dg_mixer_init;
                        chip->model.cleanup = dg_cleanup;
                        chip->model.suspend = dg_suspend;
                        chip->model.resume = dg_resume;
                        chip->model.set_dac_params = set_cs4245_dac_params;
                        chip->model.set_adc_params = set_cs4245_adc_params;
                        chip->model.adjust_dac_routing = adjust_dg_dac_routing;
                        chip->model.dump_registers = dump_cs4245_registers;
                        chip->model.model_data_size = sizeof(struct dg);
                        chip->model.device_config = PLAYBACK_0_TO_I2S |
                        PLAYBACK_1_TO_SPDIF |
                        CAPTURE_0_FROM_I2S_1 |
                        CAPTURE_1_FROM_SPDIF;
                        chip->model.dac_channels_pcm = 6;
                        chip->model.dac_channels_mixer = 0;
                        chip->model.function_flags = OXYGEN_FUNCTION_SPI;
                        chip->model.dac_mclks = OXYGEN_MCLKS(256, 128, 128);
                        chip->model.adc_mclks = OXYGEN_MCLKS(256, 128, 128);
                        chip->model.dac_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
                        chip->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
                        
                        break;
                default:
                        goto Done;
        }
        if (model == MODEL_MERIDIAN ||
            model == MODEL_MERIDIAN_2G ||
            model == MODEL_CLARO_HALO) {
                chip->model.misc_flags = OXYGEN_MISC_MIDI;
                chip->model.device_config |= MIDI_OUTPUT | MIDI_INPUT;
        }
        chip->model.shortname = get_oxygen_model(model);
        chip->model.init(chip, engineInstance);
        audioEngine->setDescription(chip->model.shortname);
        
        result = true;
        
Done:
        
        return result;
}



void XonarGenericAudioEngine::free()
{
        printf("XonarGenericAudioEngine[%p]::free()\n", this);
        
        // We need to free our resources when we're going away
        // the main engine will take care of anything it allocates.
        // the submodel shouldn't free anything it doesn't allocate.
        /*
        if(deviceRegisters) {
                IOFree(deviceRegisters, data_size);
                deviceRegisters = NULL;
        }
        
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
         */
        super::free();
}


void XonarGenericAudioEngine::stop(IOService *provider)
{
        printf("XonarGenericAudioEngine[%p]::stop(%p)\n", this, provider);
        
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
        engineInstance->stop(provider);
}


UInt32 XonarGenericAudioEngine::getCurrentSampleFrame()
{
        kprintf("XonarGenericAudioEngine::getCurrentSampleFrame()\n");
        
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


static int meridian_dig_source_info(struct snd_kcontrol *ctl,
                                    struct snd_ctl_elem_info *info)
{
        static const char *const names[2] = { "On-board", "Extension" };
        
        return snd_ctl_enum_info(info, 1, 2, names);
}

static int claro_dig_source_info(struct snd_kcontrol *ctl,
                                 struct snd_ctl_elem_info *info)
{
        static const char *const names[2] = { "Optical", "Coaxial" };
        
        return snd_ctl_enum_info(info, 1, 2, names);
}

static int meridian_dig_source_get(struct snd_kcontrol *ctl,
                                   struct snd_ctl_elem_value *value)
{
        struct oxygen *chip = (struct oxygen*) ctl->private_data;
        
        value->value.enumerated.item[0] =
        !!(oxygen_read16(chip, OXYGEN_GPIO_DATA) &
           GPIO_MERIDIAN_DIG_EXT);
        return 0;
}

static int claro_dig_source_get(struct snd_kcontrol *ctl,
                                struct snd_ctl_elem_value *value)
{
        struct oxygen *chip = (struct oxygen*) ctl->private_data;
        
        value->value.enumerated.item[0] =
        !!(oxygen_read16(chip, OXYGEN_GPIO_DATA) &
           GPIO_CLARO_DIG_COAX);
        return 0;
}

static int meridian_dig_source_put(struct snd_kcontrol *ctl,
                                   struct snd_ctl_elem_value *value)
{
        struct oxygen *chip = (struct oxygen*) ctl->private_data;
        UInt16 old_reg, new_reg;
        int changed;
        
        IOLockLock(chip->mutex);
        old_reg = oxygen_read16(chip, OXYGEN_GPIO_DATA);
        new_reg = old_reg & ~GPIO_MERIDIAN_DIG_MASK;
        if (value->value.enumerated.item[0] == 0)
                new_reg |= GPIO_MERIDIAN_DIG_BOARD;
        else
                new_reg |= GPIO_MERIDIAN_DIG_EXT;
        changed = new_reg != old_reg;
        if (changed)
                oxygen_write16(chip, OXYGEN_GPIO_DATA, new_reg);
        IOLockUnlock(chip->mutex);
        //return changed;
        return kIOReturnSuccess;

}

static int claro_dig_source_put(struct snd_kcontrol *ctl,
                                struct snd_ctl_elem_value *value)
{
        struct oxygen *chip = (struct oxygen*) ctl->private_data;
        UInt16 old_reg, new_reg;
        int changed;
        
        IOLockLock(chip->mutex);
        old_reg = oxygen_read16(chip, OXYGEN_GPIO_DATA);
        new_reg = old_reg & ~GPIO_CLARO_DIG_COAX;
        if (value->value.enumerated.item[0])
                new_reg |= GPIO_CLARO_DIG_COAX;
        changed = new_reg != old_reg;
        if (changed)
                oxygen_write16(chip, OXYGEN_GPIO_DATA, new_reg);
        IOLockUnlock(chip->mutex);
        //return changed;
        return kIOReturnSuccess;

}

static const struct snd_kcontrol_new meridian_dig_source_control = {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name = "IEC958 Source Capture Enum",
        .info = meridian_dig_source_info,
        .get = meridian_dig_source_get,
        .put = meridian_dig_source_put,
};

static const struct snd_kcontrol_new claro_dig_source_control = {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name = "IEC958 Source Capture Enum",
        .info = claro_dig_source_info,
        .get = claro_dig_source_get,
        .put = claro_dig_source_put,
};


static const DECLARE_TLV_DB_LINEAR(ak4396_db_scale, TLV_DB_GAIN_MUTE, 0);




