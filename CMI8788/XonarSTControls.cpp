#include "PCIAudioDevice.hpp"
#include "XonarSTAudioEngine.hpp"
#include "oxygen.h"
#include "alsa.h"

#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/audio/IOAudioLevelControl.h>
#include <IOKit/audio/IOAudioToggleControl.h>
#include <IOKit/audio/IOAudioPort.h>
#include <IOKit/audio/IOAudioDefines.h>

/*
 static const struct snd_kcontrol_new alt_switch = {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Analog Loopback Switch",
 .info = snd_ctl_boolean_mono_info,
 .get = XonarAudioEngine::xonar_gpio_bit_switch_get,
 .put = XonarAudioEngine::xonar_gpio_bit_switch_put,
 .private_value = GPIO_D2_ALT,
 };
 
 static const struct snd_kcontrol_new hdav_hdmi_control = {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "HDMI Playback Switch",
 .info = snd_ctl_boolean_mono_info,
 .get = XonarAudioEngine::xonar_gpio_bit_switch_get,
 .put = XonarAudioEngine::xonar_gpio_bit_switch_put,
 .private_value = GPIO_HDAV_OUTPUT_ENABLE | XONAR_GPIO_BIT_INVERT,
 };
 
 
 
 
 int st_output_switch_info(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_info *info)
 {
 static const char *const names[3] = {
 "Speakers", "Headphones", "FP Headphones"
 };
 
 return snd_ctl_enum_info(info, 1, 3, names);
 }
 
 int st_output_switch_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 UInt16 gpio;
 
 gpio = oxygen_read16(chip, OXYGEN_GPIO_DATA);
 if (!(gpio & GPIO_ST_HP))
 value->value.enumerated.item[0] = 0;
 else if (gpio & GPIO_ST_HP_REAR)
 value->value.enumerated.item[0] = 1;
 else
 value->value.enumerated.item[0] = 2;
 return 0;
 }
 
 
 static int st_hp_volume_offset_info(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_info *info)
 {
 static const char *const names[4] = {
 "< 32 ohms", "32-64 ohms", "64-300 ohms", "300-600 ohms"
 };
 
 return snd_ctl_enum_info(info, 1, 4, names);
 }
 
 static int st_hp_volume_offset_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = (struct oxygen*) ctl->private_data;
 struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
 
 IOLockLock(chip->mutex);
 if (data->hp_gain_offset < 2*-12)
 value->value.enumerated.item[0] = 0;
 else if (data->hp_gain_offset < 2*-6)
 value->value.enumerated.item[0] = 1;
 else if (data->hp_gain_offset < 0)
 value->value.enumerated.item[0] = 2;
 else
 value->value.enumerated.item[0] = 3;
 IOLockUnlock(chip->mutex);
 return 0;
 }
 */
IOReturn PCIAudioDevice::SThpVolumeOffsetChangeHandler(IOService *target, IOAudioControl *volumeControl, int oldValue, int newValue)
{
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;
        
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->st_hp_volume_offset_put(volumeControl, audioDevice->accessibleEngineInstance, oldValue, newValue);
        }
        
        return result;
}


IOReturn PCIAudioDevice::st_hp_volume_offset_put(IOAudioControl *volumeControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
        static const SInt8 offsets[] = { 2*-18, 2*-12, 2*-6, 0 };
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        
        SInt8 offset;
        int changed;
        
        if (newValue > 3)
                return -EINVAL;
        offset = offsets[newValue];
        IOLockLock(chip->mutex);
        changed = offset != data->hp_gain_offset;
        if (changed) {
                data->hp_gain_offset = offset;
                engine->update_pcm1796_volume(chip);
        }
        IOLockUnlock(chip->mutex);
        //return changed;
        return kIOReturnSuccess;

}




static int xense_output_switch_get(struct snd_kcontrol *ctl,
                                   struct snd_ctl_elem_value *value)
{
        struct oxygen *chip = (struct oxygen*) ctl->private_data;
        UInt16 gpio;
        
        gpio = oxygen_read16(chip, OXYGEN_GPIO_DATA);
        if (gpio & GPIO_XENSE_SPEAKERS)
                value->value.enumerated.item[0] = 0;
        else if (!(gpio & GPIO_XENSE_SPEAKERS) && (gpio & GPIO_ST_HP_REAR))
                value->value.enumerated.item[0] = 1;
        else
                value->value.enumerated.item[0] = 2;
        return 0;
}

IOReturn PCIAudioDevice::XenseOutputChangeHandler(IOService *target, IOAudioControl *volumeControl, int oldValue, int newValue)
{
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;
        
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->xense_output_switch_put(volumeControl, audioDevice->accessibleEngineInstance, oldValue, newValue);
        }
        
        return result;
}



IOReturn PCIAudioDevice::xense_output_switch_put(IOAudioControl *volumeControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        UInt16 gpio_old, gpio;
        
        IOLockLock(chip->mutex);
        gpio_old = oxygen_read16(chip, OXYGEN_GPIO_DATA);
        gpio = gpio_old;
        switch (newValue) {
                case 0:
                        gpio |= GPIO_XENSE_SPEAKERS | GPIO_ST_HP_REAR;
                        break;
                case 1:
                        gpio = (gpio | GPIO_ST_HP_REAR) & ~GPIO_XENSE_SPEAKERS;
                        break;
                case 2:
                        gpio &= ~(GPIO_XENSE_SPEAKERS | GPIO_ST_HP_REAR);
                        break;
        }
        oxygen_write16(chip, OXYGEN_GPIO_DATA, gpio);
        data->hp_active = !(gpio & GPIO_XENSE_SPEAKERS);
        engine->update_pcm1796_volume(chip);
        IOLockUnlock(chip->mutex);
        return gpio != gpio_old;
}

IOReturn PCIAudioDevice::STOutputChangeHandler(IOService *target, IOAudioControl *volumeControl, int oldValue, int newValue)
{
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;
        
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->st_output_switch_put(volumeControl, audioDevice->accessibleEngineInstance, oldValue, newValue);
        }
        
        return result;
}


IOReturn PCIAudioDevice::st_output_switch_put(IOAudioControl *volumeControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
        IOLog("PCIAudioDevice[%p]::st_output_switch_put(%p, %d, %d)\n", this, volumeControl, oldValue, newValue);
        
        if (volumeControl) {
                IOLog("\t-> Channel %d\n", volumeControl->getChannelID());
        }
        
        // Add hardware volume code change
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        UInt16 gpio_old, gpio;
        
        IOLockLock(chip->mutex);
        gpio_old = oxygen_read16(chip, OXYGEN_GPIO_DATA);
        gpio = gpio_old;
        switch (newValue) {
                case 0:
                        gpio &= ~(GPIO_ST_HP | GPIO_ST_HP_REAR);
                        break;
                case 1:
                        gpio |= GPIO_ST_HP | GPIO_ST_HP_REAR;
                        break;
                case 2:
                        gpio = (gpio | GPIO_ST_HP) & ~GPIO_ST_HP_REAR;
                        break;
        }
        oxygen_write16(chip, OXYGEN_GPIO_DATA, gpio);
        data->hp_active = gpio & GPIO_ST_HP;
        engine->update_pcm1796_volume(chip);
        IOLockUnlock(chip->mutex);
        return gpio != gpio_old;
        
        //return kIOReturnSuccess;
}

//IOReturn PCIAudioDevice::outputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, int oldValue, int newValue)
//{
//    IOReturn result = kIOReturnBadArgument;
//    SamplePCIAudioDevice *audioDevice;
//
//    audioDevice = (SamplePCIAudioDevice *)target;
//    if (audioDevice) {
//        result = audioDevice->outputMuteChanged(muteControl, oldValue, newValue);
//    }
//
//    return result;
//}
//
//IOReturn PCIAudioDevice::outputMuteChanged(IOAudioControl *muteControl, int oldValue, int newValue)
//{
//    IOLog("SamplePCIAudioDevice[%p]::outputMuteChanged(%p, %d, %d)\n", this, muteControl, oldValue, newValue);
//
//    // Add output mute code here
//
//    return kIOReturnSuccess;
//}

