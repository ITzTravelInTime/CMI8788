////
////  XonarAC97Controls.cpp
////  PCIAudioDriver
////
////  Created by Gagan on 2023-01-02.
////  Copyright © 2023 CMedia. All rights reserved.
////
//#include "PCIAudioDevice.hpp"
//#include "XonarAudioEngine.hpp"
//#include "oxygen.h"
//#include "alsa.h"
//#include "cm9780.h"
//#include "ac97.h"
//
//#include <IOKit/pci/IOPCIDevice.h>
//#include <IOKit/audio/IOAudioLevelControl.h>
//#include <IOKit/audio/IOAudioToggleControl.h>
//#include <IOKit/audio/IOAudioPort.h>
//#include <IOKit/audio/IOAudioDefines.h>
//
//
//static int monitor_volume_info(struct snd_kcontrol *ctl,
//                               struct snd_ctl_elem_info *info)
//{
//        info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
//        info->count = 1;
//        info->value.integer.min = 0;
//        info->value.integer.max = 1;
//        return 0;
//}
//
//static int monitor_get(struct snd_kcontrol *ctl,
//                       struct snd_ctl_elem_value *value)
//{
//        struct oxygen *chip = (struct oxygen*) ctl->private_data;
//        UInt8 bit = ctl->private_value;
//        int invert = ctl->private_value & (1 << 8);
//        
//        value->value.integer.value[0] =
//        !!invert ^ !!(oxygen_read8(chip, OXYGEN_ADC_MONITOR) & bit);
//        return 0;
//}
//
//static int monitor_put(struct snd_kcontrol *ctl,
//                       struct snd_ctl_elem_value *value)
//{
//        struct oxygen *chip = (struct oxygen*) ctl->private_data;
//        UInt8 bit = ctl->private_value;
//        int invert = ctl->private_value & (1 << 8);
//        UInt8 oldreg, newreg;
//        int changed;
//        
//        IOSimpleLockLock(chip->reg_lock);
//        oldreg = oxygen_read8(chip, OXYGEN_ADC_MONITOR);
//        if ((!!value->value.integer.value[0] ^ !!invert) != 0)
//                newreg = oldreg | bit;
//        else
//                newreg = oldreg & ~bit;
//        changed = newreg != oldreg;
//        if (changed)
//                oxygen_write8(chip, OXYGEN_ADC_MONITOR, newreg);
//        IOSimpleLockUnlock(chip->reg_lock);
//        //return changed;
//        return kIOReturnSuccess;
//        
//}
//
//static int ac97_switch_get(struct snd_kcontrol *ctl,
//                           struct snd_ctl_elem_value *value)
//{
//        struct oxygen *chip = (struct oxygen*) ctl->private_data;
//        unsigned int codec = (ctl->private_value >> 24) & 1;
//        unsigned int index = ctl->private_value & 0xff;
//        unsigned int bitnr = (ctl->private_value >> 8) & 0xff;
//        int invert = ctl->private_value & (1 << 16);
//        UInt16 reg;
//        
//        IOLockLock(chip->mutex);
//        reg = oxygen_read_ac97(chip, codec, index);
//        IOLockUnlock(chip->mutex);
//        if (!(reg & (1 << bitnr)) ^ !invert)
//                value->value.integer.value[0] = 1;
//        else
//                value->value.integer.value[0] = 0;
//        return 0;
//}
//
//
//static int mic_fmic_source_info(struct snd_kcontrol *ctl,
//                                struct snd_ctl_elem_info *info)
//{
//        static const char *const names[] = { "Mic Jack", "Front Panel" };
//        
//        return snd_ctl_enum_info(info, 1, 2, names);
//}
//
//static int mic_fmic_source_get(struct snd_kcontrol *ctl,
//                               struct snd_ctl_elem_value *value)
//{
//        struct oxygen *chip = (struct oxygen*) ctl->private_data;
//        
//        IOLockLock(chip->mutex);
//        value->value.enumerated.item[0] =
//        !!(oxygen_read_ac97(chip, 0, CM9780_JACK) & CM9780_FMIC2MIC);
//        IOLockUnlock(chip->mutex);
//        return 0;
//}
//
//static int mic_fmic_source_put(struct snd_kcontrol *ctl,
//                               struct snd_ctl_elem_value *value)
//{
//        struct oxygen *chip = (struct oxygen*) ctl->private_data;
//        UInt16 oldreg, newreg;
//        int change;
//        
//        IOLockLock(chip->mutex);
//        oldreg = oxygen_read_ac97(chip, 0, CM9780_JACK);
//        if (value->value.enumerated.item[0])
//                newreg = oldreg | CM9780_FMIC2MIC;
//        else
//                newreg = oldreg & ~CM9780_FMIC2MIC;
//        change = newreg != oldreg;
//        if (change)
//                oxygen_write_ac97(chip, 0, CM9780_JACK, newreg);
//        IOLockUnlock(chip->mutex);
//        return change;
//}
//
//static int ac97_fp_rec_volume_info(struct snd_kcontrol *ctl,
//                                   struct snd_ctl_elem_info *info)
//{
//        info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
//        info->count = 2;
//        info->value.integer.min = 0;
//        info->value.integer.max = 7;
//        return 0;
//}
//
//static int ac97_fp_rec_volume_get(struct snd_kcontrol *ctl,
//                                  struct snd_ctl_elem_value *value)
//{
//        struct oxygen *chip = (struct oxygen*) ctl->private_data;
//        UInt16 reg;
//        
//        IOLockLock(chip->mutex);
//        reg = oxygen_read_ac97(chip, 1, AC97_REC_GAIN);
//        IOLockUnlock(chip->mutex);
//        value->value.integer.value[0] = reg & 7;
//        value->value.integer.value[1] = (reg >> 8) & 7;
//        return 0;
//}
//
//static int ac97_fp_rec_volume_put(struct snd_kcontrol *ctl,
//                                  struct snd_ctl_elem_value *value)
//{
//        struct oxygen *chip = (struct oxygen*) ctl->private_data;
//        UInt16 oldreg, newreg;
//        int change;
//        
//        IOLockLock(chip->mutex);
//        oldreg = oxygen_read_ac97(chip, 1, AC97_REC_GAIN);
//        newreg = oldreg & ~0x0707;
//        newreg = newreg | (value->value.integer.value[0] & 7);
//        newreg = newreg | ((value->value.integer.value[0] & 7) << 8);
//        change = newreg != oldreg;
//        if (change)
//                oxygen_write_ac97(chip, 1, AC97_REC_GAIN, newreg);
//        IOLockUnlock(chip->mutex);
//        return change;
//}
//
//
//
//static DECLARE_TLV_DB_SCALE(monitor_db_scale, -600, 600, 0);
//static DECLARE_TLV_DB_SCALE(ac97_db_scale, -3450, 150, 0);
//static DECLARE_TLV_DB_SCALE(ac97_rec_db_scale, 0, 150, 0);
//
//
//static const struct {
//        unsigned int pcm_dev;
//        struct snd_kcontrol_new controls[2];
//} monitor_controls[] = {
//        {
//                .pcm_dev = CAPTURE_0_FROM_I2S_1,
//                .controls = {
//                        {
//                                .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//                                .name = "Analog Input Monitor Playback Switch",
//                                .info = snd_ctl_boolean_mono_info,
//                                .get = monitor_get,
//                                .put = monitor_put,
//                                .private_value = OXYGEN_ADC_MONITOR_A,
//                        },
//                        {
//                                .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//                                .name = "Analog Input Monitor Playback Volume",
//                                .access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
//                                SNDRV_CTL_ELEM_ACCESS_TLV_READ,
//                                .info = monitor_volume_info,
//                                .get = monitor_get,
//                                .put = monitor_put,
//                                .private_value = OXYGEN_ADC_MONITOR_A_HALF_VOL
//                                | (1 << 8),
//                                .tlv = { .p = monitor_db_scale, },
//                        },
//                },
//        },
//        {
//                .pcm_dev = CAPTURE_0_FROM_I2S_2,
//                .controls = {
//                        {
//                                .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//                                .name = "Analog Input Monitor Playback Switch",
//                                .info = snd_ctl_boolean_mono_info,
//                                .get = monitor_get,
//                                .put = monitor_put,
//                                .private_value = OXYGEN_ADC_MONITOR_B,
//                        },
//                        {
//                                .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//                                .name = "Analog Input Monitor Playback Volume",
//                                .access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
//                                SNDRV_CTL_ELEM_ACCESS_TLV_READ,
//                                .info = monitor_volume_info,
//                                .get = monitor_get,
//                                .put = monitor_put,
//                                .private_value = OXYGEN_ADC_MONITOR_B_HALF_VOL
//                                | (1 << 8),
//                                .tlv = { .p = monitor_db_scale, },
//                        },
//                },
//        },
//        {
//                .pcm_dev = CAPTURE_2_FROM_I2S_2,
//                .controls = {
//                        {
//                                .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//                                .name = "Analog Input Monitor Playback Switch",
//                                .index = 1,
//                                .info = snd_ctl_boolean_mono_info,
//                                .get = monitor_get,
//                                .put = monitor_put,
//                                .private_value = OXYGEN_ADC_MONITOR_B,
//                        },
//                        {
//                                .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//                                .name = "Analog Input Monitor Playback Volume",
//                                .index = 1,
//                                .access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
//                                SNDRV_CTL_ELEM_ACCESS_TLV_READ,
//                                .info = monitor_volume_info,
//                                .get = monitor_get,
//                                .put = monitor_put,
//                                .private_value = OXYGEN_ADC_MONITOR_B_HALF_VOL
//                                | (1 << 8),
//                                .tlv = { .p = monitor_db_scale, },
//                        },
//                },
//        },
//        {
//                .pcm_dev = CAPTURE_3_FROM_I2S_3,
//                .controls = {
//                        {
//                                .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//                                .name = "Analog Input Monitor Playback Switch",
//                                .index = 2,
//                                .info = snd_ctl_boolean_mono_info,
//                                .get = monitor_get,
//                                .put = monitor_put,
//                                .private_value = OXYGEN_ADC_MONITOR_C,
//                        },
//                        {
//                                .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//                                .name = "Analog Input Monitor Playback Volume",
//                                .index = 2,
//                                .access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
//                                SNDRV_CTL_ELEM_ACCESS_TLV_READ,
//                                .info = monitor_volume_info,
//                                .get = monitor_get,
//                                .put = monitor_put,
//                                .private_value = OXYGEN_ADC_MONITOR_C_HALF_VOL
//                                | (1 << 8),
//                                .tlv = { .p = monitor_db_scale, },
//                        },
//                },
//        },
//        {
//                .pcm_dev = CAPTURE_1_FROM_SPDIF,
//                .controls = {
//                        {
//                                .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//                                .name = "Digital Input Monitor Playback Switch",
//                                .info = snd_ctl_boolean_mono_info,
//                                .get = monitor_get,
//                                .put = monitor_put,
//                                .private_value = OXYGEN_ADC_MONITOR_C,
//                        },
//                        {
//                                .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//                                .name = "Digital Input Monitor Playback Volume",
//                                .access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
//                                SNDRV_CTL_ELEM_ACCESS_TLV_READ,
//                                .info = monitor_volume_info,
//                                .get = monitor_get,
//                                .put = monitor_put,
//                                .private_value = OXYGEN_ADC_MONITOR_C_HALF_VOL
//                                | (1 << 8),
//                                .tlv = { .p = monitor_db_scale, },
//                        },
//                },
//        },
//};
//
//
//static void mute_ac97_ctl(struct oxygen *chip, unsigned int control)
//{
//        unsigned int priv_idx;
//        UInt16 value;
//        
//        if (!chip->controls[control])
//                return;
//        
//        //have to fix the line below
//        //priv_idx = chip->controls[control]->private_value & 0xff;
//        value = oxygen_read_ac97(chip, 0, priv_idx);
//        if (!(value & 0x8000)) {
//                oxygen_write_ac97(chip, 0, priv_idx, value | 0x8000);
//                if (chip->model.ac97_switch)
//                        chip->model.ac97_switch(chip, priv_idx, 0x8000);
//                //sendChangeNotification seems like what we're going to have to use to notify about the change.
//                //snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
//                //                 &chip->controls[control]->id);
//        }
//}
//
//static int ac97_switch_put(struct snd_kcontrol *ctl,
//                           struct snd_ctl_elem_value *value)
//{
//        struct oxygen *chip = (struct oxygen*) ctl->private_data;
//        unsigned int codec = (ctl->private_value >> 24) & 1;
//        unsigned int index = ctl->private_value & 0xff;
//        unsigned int bitnr = (ctl->private_value >> 8) & 0xff;
//        int invert = ctl->private_value & (1 << 16);
//        UInt16 oldreg, newreg;
//        int change;
//        
//        IOLockLock(chip->mutex);
//        oldreg = oxygen_read_ac97(chip, codec, index);
//        newreg = oldreg;
//        if (!value->value.integer.value[0] ^ !invert)
//                newreg |= 1 << bitnr;
//        else
//                newreg &= ~(1 << bitnr);
//        change = newreg != oldreg;
//        if (change) {
//                oxygen_write_ac97(chip, codec, index, newreg);
//                if (codec == 0 && chip->model.ac97_switch)
//                        chip->model.ac97_switch(chip, index, newreg & 0x8000);
//                if (index == AC97_LINE) {
//                        oxygen_write_ac97_masked(chip, 0, CM9780_GPIO_STATUS,
//                                                 newreg & 0x8000 ?
//                                                 CM9780_GPO0 : 0, CM9780_GPO0);
//                        if (!(newreg & 0x8000)) {
//                                mute_ac97_ctl(chip, CONTROL_MIC_CAPTURE_SWITCH);
//                                mute_ac97_ctl(chip, CONTROL_CD_CAPTURE_SWITCH);
//                                mute_ac97_ctl(chip, CONTROL_AUX_CAPTURE_SWITCH);
//                        }
//                } else if ((index == AC97_MIC || index == AC97_CD ||
//                            index == AC97_VIDEO || index == AC97_AUX) &&
//                           bitnr == 15 && !(newreg & 0x8000)) {
//                        mute_ac97_ctl(chip, CONTROL_LINE_CAPTURE_SWITCH);
//                        oxygen_write_ac97_masked(chip, 0, CM9780_GPIO_STATUS,
//                                                 CM9780_GPO0, CM9780_GPO0);
//                }
//        }
//        IOLockUnlock(chip->mutex);
//        return change;
//}
//
//static int ac97_volume_info(struct snd_kcontrol *ctl,
//                            struct snd_ctl_elem_info *info)
//{
//        int stereo = (ctl->private_value >> 16) & 1;
//        
//        info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
//        info->count = stereo ? 2 : 1;
//        info->value.integer.min = 0;
//        info->value.integer.max = 0x1f;
//        return 0;
//}
//
//static int ac97_volume_get(struct snd_kcontrol *ctl,
//                           struct snd_ctl_elem_value *value)
//{
//        struct oxygen *chip = (struct oxygen*) ctl->private_data;
//        unsigned int codec = (ctl->private_value >> 24) & 1;
//        int stereo = (ctl->private_value >> 16) & 1;
//        unsigned int index = ctl->private_value & 0xff;
//        UInt16 reg;
//        
//        IOLockLock(chip->mutex);
//        reg = oxygen_read_ac97(chip, codec, index);
//        IOLockUnlock(chip->mutex);
//        if (!stereo) {
//                value->value.integer.value[0] = 31 - (reg & 0x1f);
//        } else {
//                value->value.integer.value[0] = 31 - ((reg >> 8) & 0x1f);
//                value->value.integer.value[1] = 31 - (reg & 0x1f);
//        }
//        return 0;
//}
//
//static int ac97_volume_put(struct snd_kcontrol *ctl,
//                           struct snd_ctl_elem_value *value)
//{
//        struct oxygen *chip = (struct oxygen*) ctl->private_data;
//        unsigned int codec = (ctl->private_value >> 24) & 1;
//        int stereo = (ctl->private_value >> 16) & 1;
//        unsigned int index = ctl->private_value & 0xff;
//        UInt16 oldreg, newreg;
//        int change;
//        
//        IOLockLock(chip->mutex);
//        oldreg = oxygen_read_ac97(chip, codec, index);
//        if (!stereo) {
//                newreg = oldreg & ~0x1f;
//                newreg |= 31 - (value->value.integer.value[0] & 0x1f);
//        } else {
//                newreg = oldreg & ~0x1f1f;
//                newreg |= (31 - (value->value.integer.value[0] & 0x1f)) << 8;
//                newreg |= 31 - (value->value.integer.value[1] & 0x1f);
//        }
//        change = newreg != oldreg;
//        if (change)
//                oxygen_write_ac97(chip, codec, index, newreg);
//        IOLockUnlock(chip->mutex);
//        return change;
//}
//
//#define AC97_SWITCH(xname, codec, index, bitnr, invert) { \
//.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
//.name = xname, \
//.info = snd_ctl_boolean_mono_info, \
//.get = ac97_switch_get, \
//.put = ac97_switch_put, \
//.private_value = ((codec) << 24) | ((invert) << 16) | \
//((bitnr) << 8) | (index), \
//}
//#define AC97_VOLUME(xname, codec, index, stereo) { \
//.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
//.name = xname, \
//.access = SNDRV_CTL_ELEM_ACCESS_READWRITE | \
//SNDRV_CTL_ELEM_ACCESS_TLV_READ, \
//.info = ac97_volume_info, \
//.get = ac97_volume_get, \
//.put = ac97_volume_put, \
//.tlv = { .p = ac97_db_scale, }, \
//.private_value = ((codec) << 24) | ((stereo) << 16) | (index), \
//}
//static const struct snd_kcontrol_new ac97_controls[] = {
//        AC97_VOLUME("Mic Capture Volume", 0, AC97_MIC, 0),
//        AC97_SWITCH("Mic Capture Switch", 0, AC97_MIC, 15, 1),
//        AC97_SWITCH("Mic Boost (+20dB)", 0, AC97_MIC, 6, 0),
//        {
//                .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//                .name = "Mic Source Capture Enum",
//                .info = mic_fmic_source_info,
//                .get = mic_fmic_source_get,
//                .put = mic_fmic_source_put,
//        },
//        AC97_SWITCH("Line Capture Switch", 0, AC97_LINE, 15, 1),
//        AC97_VOLUME("CD Capture Volume", 0, AC97_CD, 1),
//        AC97_SWITCH("CD Capture Switch", 0, AC97_CD, 15, 1),
//        AC97_VOLUME("Aux Capture Volume", 0, AC97_AUX, 1),
//        AC97_SWITCH("Aux Capture Switch", 0, AC97_AUX, 15, 1),
//};
//
//static const struct snd_kcontrol_new ac97_fp_controls[] = {
//        AC97_VOLUME("Front Panel Playback Volume", 1, AC97_HEADPHONE, 1),
//        AC97_SWITCH("Front Panel Playback Switch", 1, AC97_HEADPHONE, 15, 1),
//        {
//                .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//                .name = "Front Panel Capture Volume",
//                .access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
//                SNDRV_CTL_ELEM_ACCESS_TLV_READ,
//                .info = ac97_fp_rec_volume_info,
//                .get = ac97_fp_rec_volume_get,
//                .put = ac97_fp_rec_volume_put,
//                .tlv = { .p = ac97_rec_db_scale, },
//        },
//        AC97_SWITCH("Front Panel Capture Switch", 1, AC97_REC_GAIN, 15, 1),
//};
//
