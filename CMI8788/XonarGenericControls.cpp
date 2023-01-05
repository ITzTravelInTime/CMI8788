#include "PCIAudioDevice.hpp"
#include "XonarGenericAudioEngine.hpp"

#include "ak4396.h"
#include "mpu401.h"

#include "oxygen.h"
#include "alsa.h"



static int rolloff_info(struct snd_kcontrol *ctl,
                        struct snd_ctl_elem_info *info)
{
        static const char *const names[2] = {
                "Sharp Roll-off", "Slow Roll-off"
        };
        
        return snd_ctl_enum_info(info, 1, 2, names);
}

static int rolloff_get(struct snd_kcontrol *ctl,
                       struct snd_ctl_elem_value *value)
{
        struct oxygen *chip = (struct oxygen *) ctl->private_data;
        struct generic_data *data = (struct generic_data *) chip->model_data;
        
        value->value.enumerated.item[0] =
        (data->ak4396_regs[0][AK4396_CONTROL_2] & AK4396_SLOW) != 0;
        return 0;
}

IOReturn PCIAudioDevice::rolloff_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
        struct oxygen *chip = (struct oxygen *) engine->chipData;
        struct generic_data *data = (struct generic_data *) chip->model_data;
        unsigned int i;
        int changed;
        UInt8 reg;
        
        IOLockLock(chip->mutex);
        reg = data->ak4396_regs[0][AK4396_CONTROL_2];
        if (newValue)
                reg |= AK4396_SLOW;
        else
                reg &= ~AK4396_SLOW;
        changed = reg != data->ak4396_regs[0][AK4396_CONTROL_2];
        if (changed) {
                // for (i = 0; i < data->dacs; ++i)
                //ak4396_write(chip, i, AK4396_CONTROL_2, reg);
        }
        IOLockLock(chip->mutex);
        //return changed;
        return kIOReturnSuccess;

}


/* CS4245 Headphone Channels A&B Volume Control */

static int hp_stereo_volume_info(struct snd_kcontrol *ctl,
                                 struct snd_ctl_elem_info *info)
{
        info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
        info->count = 2;
        info->value.integer.min = 0;
        info->value.integer.max = 255;
        return 0;
}

static int hp_stereo_volume_get(struct snd_kcontrol *ctl,
                                struct snd_ctl_elem_value *val)
{
        struct oxygen *chip = (struct oxygen*) ctl->private_data;
        struct dg *data = (struct dg*) chip->model_data;
        unsigned int tmp;
        
        IOLockLock(chip->mutex);
        tmp = (~data->cs4245_shadow[CS4245_DAC_A_CTRL]) & 255;
        val->value.integer.value[0] = tmp;
        tmp = (~data->cs4245_shadow[CS4245_DAC_B_CTRL]) & 255;
        val->value.integer.value[1] = tmp;
        IOLockUnlock(chip->mutex);
        return 0;
}

IOReturn PCIAudioDevice::hp_stereo_volume_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        struct dg *data = (struct dg*) chip->model_data;
        int ret;
        int changed = 0;
        short *value = (SInt16*) &newValue;
        long new1 = value[0];
        long new2 = value[1];
        
        if ((new1 > 255) || (new1 < 0) || (new2 > 255) || (new2 < 0))
                return -EINVAL;
        
        IOLockLock(chip->mutex);
        if ((data->cs4245_shadow[CS4245_DAC_A_CTRL] != ~new1) ||
            (data->cs4245_shadow[CS4245_DAC_B_CTRL] != ~new2)) {
                data->cs4245_shadow[CS4245_DAC_A_CTRL] = ~new1;
                data->cs4245_shadow[CS4245_DAC_B_CTRL] = ~new2;
                ret = cs4245_write_spi(chip, CS4245_DAC_A_CTRL);
                if (ret >= 0)
                        ret = cs4245_write_spi(chip, CS4245_DAC_B_CTRL);
                changed = ret >= 0 ? 1 : ret;
        }
        IOLockUnlock(chip->mutex);
        
        //return changed;
        return kIOReturnSuccess;

}

/* Headphone Mute */

static int hp_mute_get(struct snd_kcontrol *ctl,
                       struct snd_ctl_elem_value *val)
{
        struct oxygen *chip = (struct oxygen*) ctl->private_data;
        struct dg *data = (struct dg*) chip->model_data;
        
        IOLockLock(chip->mutex);
        val->value.integer.value[0] =
        !(data->cs4245_shadow[CS4245_DAC_CTRL_1] & CS4245_MUTE_DAC);
        IOLockUnlock(chip->mutex);
        return 0;
}

IOReturn PCIAudioDevice::hp_mute_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        struct dg *data = (struct dg*) chip->model_data;
        int ret;
        int changed;
        
        if (newValue > 1)
                return -EINVAL;
        IOLockLock(chip->mutex);
        data->cs4245_shadow[CS4245_DAC_CTRL_1] &= ~CS4245_MUTE_DAC;
        data->cs4245_shadow[CS4245_DAC_CTRL_1] |=
        (~newValue << 2) & CS4245_MUTE_DAC;
        ret = cs4245_write_spi(chip, CS4245_DAC_CTRL_1);
        changed = ret >= 0 ? 1 : ret;
        IOLockUnlock(chip->mutex);
        //return changed;
        return kIOReturnSuccess;

}

/* capture volume for all sources */

static int input_volume_apply(struct oxygen *chip, char left, char right)
{
        struct dg *data = (struct dg*) chip->model_data;
        int ret;
        
        data->cs4245_shadow[CS4245_PGA_A_CTRL] = left;
        data->cs4245_shadow[CS4245_PGA_B_CTRL] = right;
        ret = cs4245_write_spi(chip, CS4245_PGA_A_CTRL);
        if (ret < 0)
                return ret;
        return cs4245_write_spi(chip, CS4245_PGA_B_CTRL);
}

static int input_vol_info(struct snd_kcontrol *ctl,
                          struct snd_ctl_elem_info *info)
{
        info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
        info->count = 2;
        info->value.integer.min = 2 * -12;
        info->value.integer.max = 2 * 12;
        return 0;
}

static int input_vol_get(struct snd_kcontrol *ctl,
                         struct snd_ctl_elem_value *value)
{
        struct oxygen *chip = (struct oxygen*) ctl->private_data;
        struct dg *data = (struct dg*) chip->model_data;
        unsigned int idx = ctl->private_value;
        
        IOLockLock(chip->mutex);
        value->value.integer.value[0] = data->input_vol[idx][0];
        value->value.integer.value[1] = data->input_vol[idx][1];
        IOLockUnlock(chip->mutex);
        return 0;
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


IOReturn PCIAudioDevice::DGInputMICVolChangeHandler(IOService *target, IOAudioControl *volumeControl, const void *oldData, UInt32 oldDataSize, const void *newData, UInt32 newDataSize )
{
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;

        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->input_vol_put(volumeControl, audioDevice->accessibleEngineInstance, oldData, oldDataSize, newData, newDataSize, CAPTURE_SRC_MIC);
        }
        
        return result;
}

IOReturn PCIAudioDevice::DGInputFPMicVolChangeHandler(IOService *target, IOAudioControl *volumeControl, const void *oldData, UInt32 oldDataSize, const void *newData, UInt32 newDataSize )
{
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;

        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->input_vol_put(volumeControl, audioDevice->accessibleEngineInstance, oldData, oldDataSize, newData, newDataSize, CAPTURE_SRC_FP_MIC);
        }
        
        return result;
}

IOReturn PCIAudioDevice::DGInputLineVolChangeHandler(IOService *target, IOAudioControl *volumeControl, const void *oldData, UInt32 oldDataSize, const void *newData, UInt32 newDataSize )
{
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;

        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->input_vol_put(volumeControl, audioDevice->accessibleEngineInstance, oldData, oldDataSize, newData, newDataSize, CAPTURE_SRC_LINE);
        }
        
        return result;
}

IOReturn PCIAudioDevice::DGInputAuxVolChangeHandler(IOService *target, IOAudioControl *volumeControl, const void *oldData, UInt32 oldDataSize, const void *newData, UInt32 newDataSize )
{
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;

        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->input_vol_put(volumeControl, audioDevice->accessibleEngineInstance, oldData, oldDataSize, newData, newDataSize, CAPTURE_SRC_AUX);
        }
        
        return result;
}

IOReturn PCIAudioDevice::input_vol_put(IOAudioControl *volumeControl, XonarAudioEngine *engine, const void *oldData, UInt32 oldDataSize, const void *newData, UInt32 newDataSize, int private_value)
{
        //this is kinda tricky because we only get to pass a 32-bit value.
        //since it's a limited range, lets split the 32 bit integer into two 16 bit.
        char *value = (char *) newData;
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        struct dg *data = (struct dg*) chip->model_data;
        unsigned int idx = private_value;
        int changed = 0;
        int ret = 0;
        
        if (value[0] < 2 * -12 ||
            value[0] > 2 * 12 ||
            value[1] < 2 * -12 ||
            value[1] > 2 * 12)
                return -EINVAL;
        IOLockLock(chip->mutex);
        changed = data->input_vol[idx][0] != value[0] ||
        data->input_vol[idx][1] != value[1];
        if (changed) {
                data->input_vol[idx][0] = value[0];
                data->input_vol[idx][1] = value[1];
                if (idx == data->input_sel) {
                        ret = input_volume_apply(chip,
                                                 data->input_vol[idx][0],
                                                 data->input_vol[idx][1]);
                }
                changed = ret >= 0 ? 1 : ret;
        }
        IOLockUnlock(chip->mutex);
        //return changed;
        return kIOReturnSuccess;

}

static int input_sel_info(struct snd_kcontrol *ctl,
                          struct snd_ctl_elem_info *info)
{
        static const char *const names[4] = {
                "Mic", "Front Mic", "Line", "Aux"
        };
        
        return snd_ctl_enum_info(info, 1, 4, names);
}

static int input_sel_get(struct snd_kcontrol *ctl,
                         struct snd_ctl_elem_value *value)
{
        struct oxygen *chip = (struct oxygen*) ctl->private_data;
        struct dg *data = (struct dg*) chip->model_data;
        
        IOLockLock(chip->mutex);
        value->value.enumerated.item[0] = data->input_sel;
        IOLockUnlock(chip->mutex);
        return 0;
}

IOReturn PCIAudioDevice::input_sel_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        struct dg *data = (struct dg*) chip->model_data;
        int changed;
        int ret;
        
        if (newValue > 3)
                return -EINVAL;
        
        IOLockLock(chip->mutex);
        changed = newValue != data->input_sel;
        if (changed) {
                data->input_sel = newValue;
                
                ret = input_source_apply(chip);
                if (ret >= 0)
                        ret = input_volume_apply(chip,
                                                 data->input_vol[data->input_sel][0],
                                                 data->input_vol[data->input_sel][1]);
                changed = ret >= 0 ? 1 : ret;
        }
        IOLockUnlock(chip->mutex);
        //return changed;
        return kIOReturnSuccess;

}

/* ADC high-pass filter */

static int hpf_info(struct snd_kcontrol *ctl, struct snd_ctl_elem_info *info)
{
        static const char *const names[2] = { "Active", "Frozen" };
        
        return snd_ctl_enum_info(info, 1, 2, names);
}

static int hpf_get(struct snd_kcontrol *ctl, struct snd_ctl_elem_value *value)
{
        struct oxygen *chip = (struct oxygen*) ctl->private_data;
        struct dg *data = (struct dg*) chip->model_data;
        
        value->value.enumerated.item[0] =
        !!(data->cs4245_shadow[CS4245_ADC_CTRL] & CS4245_HPF_FREEZE);
        return 0;
}

IOReturn PCIAudioDevice::hpf_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        struct dg *data = (struct dg*) chip->model_data;
        UInt8 reg;
        int changed;
        
        IOLockLock(chip->mutex);
        reg = data->cs4245_shadow[CS4245_ADC_CTRL] & ~CS4245_HPF_FREEZE;
        if (newValue)
                reg |= CS4245_HPF_FREEZE;
        changed = reg != data->cs4245_shadow[CS4245_ADC_CTRL];
        if (changed) {
                data->cs4245_shadow[CS4245_ADC_CTRL] = reg;
                cs4245_write_spi(chip, CS4245_ADC_CTRL);
        }
        IOLockUnlock(chip->mutex);
        //return changed;
        return kIOReturnSuccess;

}
static int output_select_info(struct snd_kcontrol *ctl,
                              struct snd_ctl_elem_info *info)
{
        static const char *const names[3] = {
                "Stereo Headphones",
                "Stereo Headphones FP",
                "Multichannel",
        };
        
        return snd_ctl_enum_info(info, 1, 3, names);
}

static int output_select_get(struct snd_kcontrol *ctl,
                             struct snd_ctl_elem_value *value)
{
        struct oxygen *chip = (struct oxygen*) ctl->private_data;
        struct dg *data = (struct dg*) chip->model_data;
        
        IOLockLock(chip->mutex);
        value->value.enumerated.item[0] = data->output_sel;
        IOLockUnlock(chip->mutex);
        return 0;
}
