/* APPUL insists they'll clean up our mixer/control mess, so let them.
 static void oxygen_any_ctl_free(struct snd_kcontrol *ctl)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 unsigned int i;
 
 // I'm too lazy to write a function for each control :-)
 for (i = 0; i < ARRAY_SIZE(chip->controls); ++i)
 chip->controls[i] = NULL;
 }
 */


/*
 
 static void oxygen_to_iec958(UInt32 bits, struct snd_ctl_elem_value *value)
 {
 value->value.iec958.status[0] =
 bits & (OXYGEN_SPDIF_NONAUDIO | OXYGEN_SPDIF_C |
 OXYGEN_SPDIF_PREEMPHASIS);
 value->value.iec958.status[1] = //category and original
 bits >> OXYGEN_SPDIF_CATEGORY_SHIFT;
 }
 
 static int add_controls(struct oxygen *chip,
 const struct snd_kcontrol_new controls[],
 unsigned int count)
 {
 static const char *const known_ctl_names[CONTROL_COUNT] = {
 [CONTROL_SPDIF_PCM] =
 SNDRV_CTL_NAME_IEC958("", PLAYBACK, PCM_STREAM),
 [CONTROL_SPDIF_INPUT_BITS] =
 SNDRV_CTL_NAME_IEC958("", CAPTURE, DEFAULT),
 [CONTROL_MIC_CAPTURE_SWITCH] = "Mic Capture Switch",
 [CONTROL_LINE_CAPTURE_SWITCH] = "Line Capture Switch",
 [CONTROL_CD_CAPTURE_SWITCH] = "CD Capture Switch",
 [CONTROL_AUX_CAPTURE_SWITCH] = "Aux Capture Switch",
 };
 unsigned int i, j;
 struct snd_kcontrol_new _template;
 struct snd_kcontrol *ctl;
 int err;
 
 for (i = 0; i < count; ++i) {
 _template = controls[i];
 if (chip->model.control_filter) {
 err = chip->model.control_filter(&_template);
 if (err < 0)
 return err;
 if (err == 1)
 continue;
 }
 if (!strcmp(_template.name, "Stereo Upmixing") &&
 chip->model.dac_channels_pcm == 2)
 continue;
 if (!strcmp(_template.name, "Mic Source Capture Enum") &&
 !(chip->model.device_config & AC97_FMIC_SWITCH))
 continue;
 if (!strncmp(_template.name, "CD Capture ", 11) &&
 !(chip->model.device_config & AC97_CD_INPUT))
 continue;
 if (!strcmp(_template.name, "Master Playback Volume") &&
 chip->model.dac_tlv) {
 _template.tlv.p = chip->model.dac_tlv;
 _template.access |= SNDRV_CTL_ELEM_ACCESS_TLV_READ;
 }
 
 //gonna have to fix this line below too
 //ctl = snd_ctl_new1(&_template, chip);
 //if (!ctl)
 //    return -ENOMEM;
 //and this
 //err = snd_ctl_add(chip->card, ctl);
 if (err < 0)
 return err;
 for (j = 0; j < CONTROL_COUNT; ++j)
 if (!strcmp(ctl->id.name, known_ctl_names[j])) {
 //and this
 //chip->controls[j] = ctl;
 ctl->private_free = oxygen_any_ctl_free;
 }
 }
 return 0;
 }
 
 
 
 static int dac_volume_info(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_info *info)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 
 info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
 info->count = chip->model.dac_channels_mixer;
 info->value.integer.min = chip->model.dac_volume_min;
 info->value.integer.max = chip->model.dac_volume_max;
 return 0;
 }
 
 static int dac_volume_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 unsigned int i;
 
 IOLockLock(chip->mutex);
 for (i = 0; i < chip->model.dac_channels_mixer; ++i)
 value->value.integer.value[i] = chip->dac_volume[i];
 IOLockUnlock(chip->mutex);
 return 0;
 }
 
 
 static int dac_volume_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 unsigned int i;
 int changed;
 
 changed = 0;
 IOLockLock(chip->mutex);
 for (i = 0; i < chip->model.dac_channels_mixer; ++i)
 if (value->value.integer.value[i] != chip->dac_volume[i]) {
 chip->dac_volume[i] = value->value.integer.value[i];
 changed = 1;
 }
 //        if (changed)
 //                chip->model.update_dac_volume(chip);
 IOLockUnlock(chip->mutex);
 return changed;
 }
 
 static int dac_mute_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 int changed;
 
 IOLockLock(chip->mutex);
 changed = (!value->value.integer.value[0]) != chip->dac_mute;
 if (changed) {
 chip->dac_mute = !value->value.integer.value[0];
 //       chip->model.update_dac_mute(chip);
 }
 IOLockUnlock(chip->mutex);
 return changed;
 }
 
 
 static int dac_mute_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 
 IOLockLock(chip->mutex);
 value->value.integer.value[0] = !chip->dac_mute;
 IOLockUnlock(chip->mutex);
 return 0;
 }
 
 int XonarAudioEngine::upmix_info(struct snd_kcontrol *ctl, struct snd_ctl_elem_info *info)
 {
 static const char *const names[5] = {
 "Front",
 "Front+Surround",
 "Front+Surround+Back",
 "Front+Surround+Center/LFE",
 "Front+Surround+Center/LFE+Back",
 };
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 unsigned int count = upmix_item_count(chip);
 return snd_ctl_enum_info(info, 1, count, names);
 }
 
 
 static int upmix_put(struct snd_kcontrol *ctl, struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 unsigned int count = upmix_item_count(chip);
 int changed;
 
 if (value->value.enumerated.item[0] >= count)
 return -EINVAL;
 IOLockLock(chip->mutex);
 changed = value->value.enumerated.item[0] != chip->dac_routing;
 if (changed) {
 chip->dac_routing = value->value.enumerated.item[0];
 oxygen_update_dac_routing(chip);
 }
 IOLockUnlock(chip->mutex);
 return changed;
 }
 static int upmix_get(struct snd_kcontrol *ctl, struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 
 IOLockLock(chip->mutex);
 value->value.enumerated.item[0] = chip->dac_routing;
 IOLockUnlock(chip->mutex);
 return 0;
 }
 
 
 */



/*
 static const struct snd_kcontrol_new controls[] = {
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Master Playback Volume",
 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
 .info = dac_volume_info,
 .get = dac_volume_get,
 .put = dac_volume_put,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Master Playback Switch",
 .info = snd_ctl_boolean_mono_info,
 .get = dac_mute_get,
 .put = dac_mute_put,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Stereo Upmixing",
 .info = upmix_info,
 .get = upmix_get,
 .put = upmix_put,
 },
 };
 */

/*
 static const struct snd_kcontrol_new spdif_output_controls[] = {
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = SNDRV_CTL_NAME_IEC958("", PLAYBACK, SWITCH),
 .info = snd_ctl_boolean_mono_info,
 .get = spdif_switch_get,
 .put = spdif_switch_put,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
 .device = 1,
 .name = SNDRV_CTL_NAME_IEC958("", PLAYBACK, DEFAULT),
 .info = spdif_info,
 .get = spdif_default_get,
 .put = spdif_default_put,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
 .device = 1,
 .name = SNDRV_CTL_NAME_IEC958("", PLAYBACK, CON_MASK),
 .access = SNDRV_CTL_ELEM_ACCESS_READ,
 .info = spdif_info,
 .get = spdif_mask_get,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
 .device = 1,
 .name = SNDRV_CTL_NAME_IEC958("", PLAYBACK, PCM_STREAM),
 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
 SNDRV_CTL_ELEM_ACCESS_INACTIVE,
 .info = spdif_info,
 .get = spdif_pcm_get,
 .put = spdif_pcm_put,
 },
 };
 */

/*
 
 static int spdif_switch_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 
 IOLockLock(chip->mutex);
 value->value.integer.value[0] = chip->spdif_playback_enable;
 IOLockUnlock(chip->mutex);
 return 0;
 }
 
 
 static int spdif_switch_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 int changed;
 
 IOLockLock(chip->mutex);
 changed = value->value.integer.value[0] != chip->spdif_playback_enable;
 if (changed) {
 chip->spdif_playback_enable = !!value->value.integer.value[0];
 IOSimpleLockLock(chip->reg_lock);
 oxygen_update_spdif_source(chip);
 IOSimpleLockUnlock(chip->reg_lock);
 }
 IOLockUnlock(chip->mutex);
 return changed;
 }
 
 
 
 
 static int spdif_default_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 
 IOLockLock(chip->mutex);
 oxygen_to_iec958(chip->spdif_bits, value);
 IOLockUnlock(chip->mutex);
 return 0;
 }
 
 static int spdif_default_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 UInt32 new_bits;
 int changed;
 
 new_bits = iec958_to_oxygen(value);
 IOLockLock(chip->mutex);
 changed = new_bits != chip->spdif_bits;
 if (changed) {
 chip->spdif_bits = new_bits;
 if (!(chip->pcm_active & (1 << PCM_SPDIF)))
 write_spdif_bits(chip, new_bits);
 }
 IOLockUnlock(chip->mutex);
 return changed;
 }
 static int spdif_mask_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 value->value.iec958.status[0] = IEC958_AES0_NONAUDIO |
 IEC958_AES0_CON_NOT_COPYRIGHT | IEC958_AES0_CON_EMPHASIS;
 value->value.iec958.status[1] =
 IEC958_AES1_CON_CATEGORY | IEC958_AES1_CON_ORIGINAL;
 return 0;
 }
 
 static int spdif_pcm_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 
 IOLockLock(chip->mutex);
 oxygen_to_iec958(chip->spdif_pcm_bits, value);
 IOLockUnlock(chip->mutex);
 return 0;
 }
 
 static int spdif_pcm_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 UInt32 new_bits;
 int changed;
 
 // new_bits = iec958_to_oxygen(value);
 IOLockLock(chip->mutex);
 changed = new_bits != chip->spdif_pcm_bits;
 if (changed) {
 chip->spdif_pcm_bits = new_bits;
 if (chip->pcm_active & (1 << PCM_SPDIF))
 write_spdif_bits(chip, new_bits);
 }
 IOLockUnlock(chip->mutex);
 return changed;
 }
 
 
 static const struct snd_kcontrol_new spdif_input_controls[] = {
 {
 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
 .device = 1,
 .name = SNDRV_CTL_NAME_IEC958("", CAPTURE, MASK),
 .access = SNDRV_CTL_ELEM_ACCESS_READ,
 .info = spdif_info,
 .get = spdif_input_mask_get,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
 .device = 1,
 .name = SNDRV_CTL_NAME_IEC958("", CAPTURE, DEFAULT),
 .access = SNDRV_CTL_ELEM_ACCESS_READ,
 .info = spdif_info,
 .get = spdif_input_default_get,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = SNDRV_CTL_NAME_IEC958("Loopback ", NONE, SWITCH),
 .info = snd_ctl_boolean_mono_info,
 .get = spdif_bit_switch_get,
 .put = spdif_bit_switch_put,
 .private_value = OXYGEN_SPDIF_LOOPBACK,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = SNDRV_CTL_NAME_IEC958("Validity Check ",CAPTURE,SWITCH),
 .info = snd_ctl_boolean_mono_info,
 .get = spdif_bit_switch_get,
 .put = spdif_bit_switch_put,
 .private_value = OXYGEN_SPDIF_SPDVALID,
 },
 };
 
 
 
 static int spdif_info(struct snd_kcontrol *ctl, struct snd_ctl_elem_info *info)
 {
 info->type = SNDRV_CTL_ELEM_TYPE_IEC958;
 info->count = 1;
 return 0;
 }
 
 
 
 static int spdif_input_mask_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 value->value.iec958.status[0] = 0xff;
 value->value.iec958.status[1] = 0xff;
 value->value.iec958.status[2] = 0xff;
 value->value.iec958.status[3] = 0xff;
 return 0;
 }
 
 static int spdif_input_default_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 UInt32 bits;
 
 bits = oxygen_read32(chip, OXYGEN_SPDIF_INPUT_BITS);
 value->value.iec958.status[0] = bits;
 value->value.iec958.status[1] = bits >> 8;
 value->value.iec958.status[2] = bits >> 16;
 value->value.iec958.status[3] = bits >> 24;
 return 0;
 }
 
 static int spdif_bit_switch_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 UInt32 bit = ctl->private_value;
 
 value->value.integer.value[0] =
 !!(oxygen_read32(chip, OXYGEN_SPDIF_CONTROL) & bit);
 return 0;
 }
 
 static int spdif_bit_switch_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 UInt32 bit = ctl->private_value;
 UInt32 oldreg, newreg;
 int changed;
 
 IOSimpleLockLock(chip->reg_lock);
 oldreg = oxygen_read32(chip, OXYGEN_SPDIF_CONTROL);
 if (value->value.integer.value[0])
 newreg = oldreg | bit;
 else
 newreg = oldreg & ~bit;
 changed = newreg != oldreg;
 if (changed)
 oxygen_write32(chip, OXYGEN_SPDIF_CONTROL, newreg);
 IOSimpleLockUnlock(chip->reg_lock);
 return changed;
 }
 
 
 //we will probably never use oxygen_close because the audioengine will clear it all up when its done
 
 int oxygen_close(IOAudioStream *substream)
 {
 struct oxygen *chip = ((XonarAudioEngine *) substream->audioEngine)->chipData;//snd_pcm_substream_chip(substream);
 unsigned int channel = substream->getFormat()->fDriverTag; //getStartingChannelID();
 
 IOLockLock(chip->mutex);
 chip->pcm_active &= ~(1 << channel);
 if (channel == PCM_SPDIF) {
 //                chip->controls[CONTROL_SPDIF_PCM]->vd[0].access |=
 //                        SNDRV_CTL_ELEM_ACCESS_INACTIVE;
 //                snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_VALUE |
 //                               SNDRV_CTL_EVENT_MASK_INFO,
 //                               &chip->controls[CONTROL_SPDIF_PCM]->id);
 }
 if (channel == PCM_SPDIF || channel == PCM_MULTICH)
 oxygen_update_spdif_source(chip);
 IOLockUnlock(chip->mutex);
 
 chip->streams[channel] = NULL;
 return 0;
 }
 
 */
