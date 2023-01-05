//
//  XonarSPDIFHandlers.cpp
//  PCIAudioDriver
//
//  Created by Gagan on 2022-12-12.
//  Copyright © 2022 CMedia. All rights reserved.
//

#include "PCIAudioDevice.hpp"
#include "XonarAudioEngine.hpp"
#include "XonarSPDIFAudioEngine.hpp"
#include "oxygen.h"
#include "alsa.h"

#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/audio/IOAudioLevelControl.h>
#include <IOKit/audio/IOAudioToggleControl.h>
#include <IOKit/audio/IOAudioPort.h>
#include <IOKit/audio/IOAudioDefines.h>


int PCIAudioDevice::createSPDIFoutputControl(IOAudioPort *spdifOutputPort) {
        IOAudioControl *spdifOutputControl;
        spdifOutputPort = IOAudioPort::withAttributes(kIOAudioPortTypeOutput, "SPDIF Output", kIOAudioOutputPortSubTypeSPDIF);
#if DEBUG
        kprintf("PCIAudioDevice::%-32s BEGIN\n", __func__);
#endif
        kprintf("PCIAudioDevice::%-32s creating SPDIF Output Control\n", " ");
        spdifOutputControl = IOAudioToggleControl::create(false,    // initial state - unmuted
                                                          kIOAudioControlChannelNumberInactive,    // Affects all channels
                                                          kIOAudioControlChannelNameAll,
                                                          0,        // control ID - driver-defined
                                                          kIOAudioToggleControlSubTypePhantomPower,
                                                          kIOAudioControlUsageOutput);
        if (!spdifOutputControl) {
                goto Done;
        }
        
        spdifOutputControl->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)spdifOutputDefaultChangeHandler, this);
        spdifOutputControl->setName("SPDIF Out Playback Default");
        accessibleEngineInstance->spdifEngine->addDefaultAudioControl(spdifOutputControl);
        spdifOutputPort->addAudioControl(spdifOutputControl);
        spdifOutputControl->release();
        
        spdifOutputControl = IOAudioToggleControl::create(false,    // initial state - unmuted
                                                          kIOAudioControlChannelNumberInactive,    // Affects all channels
                                                          kIOAudioControlChannelNameAll,
                                                          0,        // control ID - driver-defined
                                                          kIOAudioToggleControlSubTypePhantomPower,
                                                          kIOAudioControlUsageOutput);
        
        if (!spdifOutputControl) {
                goto Done;
        }
        spdifOutputControl->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)spdifOutputSwitchChangeHandler, this);
        spdifOutputControl->setName("SPDIF Out Playback Toggle Switch");
        accessibleEngineInstance->spdifEngine->addDefaultAudioControl(spdifOutputControl);
        spdifOutputPort->addAudioControl(spdifOutputControl);
        spdifOutputControl->release();
        
        //time to add "new" stuff, finally!@
        //i think it's OK to set this as a generic toggle. this may need tweaking
        spdifOutputControl = IOAudioToggleControl::create(false,    // initial state - unmuted
                                                          kIOAudioControlChannelNumberInactive,    // Affects all channels
                                                          kIOAudioControlChannelNameAll,
                                                          0,        // control ID - driver-defined
                                                          kIOAudioToggleControlSubTypePhantomPower,
                                                          kIOAudioControlUsageOutput);
        if (!spdifOutputControl) {
                goto Done;
        }
        spdifOutputControl->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)spdifOutputPCMChangeHandler, this);
        spdifOutputControl->setName("SPDIF Out Playback PCM");
        accessibleEngineInstance->spdifEngine->addDefaultAudioControl(spdifOutputControl);
        spdifOutputPort->addAudioControl(spdifOutputControl);
        attachAudioPort(spdifOutputPort, accessibleEngineInstance->spdifEngine, NULL);
        spdifOutputControl->release();
        spdifOutputPort->release();
        
        return 0;
        
Done:
        return -1;
        
}

int PCIAudioDevice::createSPDIFinputControl(IOAudioPort *spdifInputPort) {
        IOAudioControl *spdifInputControl;
        spdifInputPort = IOAudioPort::withAttributes(kIOAudioPortTypeInput, "SPDIF Input", kIOAudioInputPortSubTypeSPDIF);
#if DEBUG
        kprintf("PCIAudioDevice::%-32s BEGIN\n", __func__);
#endif
        kprintf("PCIAudioDevice::%-32s creating SPDIF Input Control\n", " ");
        spdifInputControl = IOAudioToggleControl::create(false,    // initial state - unmuted
                                                         kIOAudioControlChannelNumberInactive,    // Affects all channels
                                                         kIOAudioControlChannelNameAll,
                                                         0,        // control ID - driver-defined
                                                         kIOAudioToggleControlSubTypePhantomPower,
                                                         kIOAudioControlUsageInput);
        if (!spdifInputControl) {
                goto Done;
        }
        
        spdifInputControl->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)spdifInputDefaultChangeHandler, this);
        spdifInputControl->setName("SPDIF Input Capture Default");
        accessibleEngineInstance->spdifEngine->addDefaultAudioControl(spdifInputControl);
        spdifInputPort->addAudioControl(spdifInputControl);
        spdifInputControl->release();
        
        spdifInputControl = IOAudioToggleControl::create(false,    // initial state - unmuted
                                                         kIOAudioControlChannelNumberInactive,    // Affects all channels
                                                         kIOAudioControlChannelNameAll,
                                                         0,        // control ID - driver-defined
                                                         kIOAudioToggleControlSubTypePhantomPower,
                                                         kIOAudioControlUsageInput);
        
        if (!spdifInputControl) {
                goto Done;
        }
        spdifInputControl->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)spdifInputLoopbackSwitchChangeHandler, this);
        spdifInputControl->setName("SPDIF In Loopback Switch");
        accessibleEngineInstance->spdifEngine->addDefaultAudioControl(spdifInputControl);
        spdifInputPort->addAudioControl(spdifInputControl);
        spdifInputControl->release();
        
        //time to add "new" stuff, finally!@
        //i think it's OK to set this as a generic toggle. this may need tweaking
        spdifInputControl = IOAudioToggleControl::create(false,    // initial state - unmuted
                                                         kIOAudioControlChannelNumberInactive,    // Affects all channels
                                                         kIOAudioControlChannelNameAll,
                                                         0,        // control ID - driver-defined
                                                         kIOAudioToggleControlSubTypePhantomPower,
                                                         kIOAudioControlUsageInput);
        if (!spdifInputControl) {
                goto Done;
        }
        spdifInputControl->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)spdifInputValiditySwitchChangeHandler, this);
        spdifInputControl->setName("SPDIF In Validity Check Capture Switch");
        accessibleEngineInstance->spdifEngine->addDefaultAudioControl(spdifInputControl);
        spdifInputPort->addAudioControl(spdifInputControl);
        attachAudioPort(spdifInputPort, accessibleEngineInstance->spdifEngine, spdifInputControl);
        spdifInputControl->release();
        spdifInputPort->release();
#if DEBUG
        kprintf("PCIAudioDevice::%-32s END\n", __func__);
#endif
        return 0;
        
Done:
        return -1;
        
}

static void write_spdif_bits(struct oxygen *chip, UInt32 bits)
{
        oxygen_write32_masked(chip, OXYGEN_SPDIF_OUTPUT_BITS, bits,
                              OXYGEN_SPDIF_NONAUDIO |
                              OXYGEN_SPDIF_C |
                              OXYGEN_SPDIF_PREEMPHASIS |
                              OXYGEN_SPDIF_CATEGORY_MASK |
                              OXYGEN_SPDIF_ORIGINAL |
                              OXYGEN_SPDIF_V);
}

static UInt32 iec958_to_oxygen(int value)
{
        // the CMI8788 doc says bits 0-6, 9-16 are relevant
        // and 24-26 refer to the sampling rate
        //gonna treat the value for SPDIF passed by the handler as a 4 byte array
        // since clemens only uses the first 2 and we don't touch it aside from the SPDIF ops, this should work.
        UInt32 bits;
        UInt8 status[4];
        memcpy(&status, &value, sizeof(value));
        bits = status[0] &
        (OXYGEN_SPDIF_NONAUDIO | OXYGEN_SPDIF_C |
         OXYGEN_SPDIF_PREEMPHASIS);
        bits |= status[1] << OXYGEN_SPDIF_CATEGORY_SHIFT;
        if (bits & OXYGEN_SPDIF_NONAUDIO)
                bits |= OXYGEN_SPDIF_V;
        return bits;
}


IOReturn PCIAudioDevice::spdifOutputDefaultChangeHandler(IOService *target, IOAudioControl *volumeControl, int oldValue, int newValue)
{
#if DEBUG
        printf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;
        
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->spdif_default_put(volumeControl, audioDevice->accessibleEngineInstance, oldValue, newValue);
        }
        
        return result;
}

IOReturn PCIAudioDevice::spdif_default_put(IOAudioControl *volumeControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
#if DEBUG
        printf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        if (volumeControl) {
                printf("\t-> Channel %d\n", volumeControl->getChannelID());
        }
        
        
        // Add hardware volume code change
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        UInt32 new_bits;
        int changed;
        
        new_bits = iec958_to_oxygen(newValue);
        IOLockLock(chip->mutex);
        changed = new_bits != chip->spdif_bits;
        if (changed) {
                chip->spdif_bits = new_bits;
                if (!(chip->pcm_active & (1 << PCM_SPDIF)))
                        write_spdif_bits(chip, new_bits);
        }
        IOLockUnlock(chip->mutex);
        //return changed;
        return kIOReturnSuccess;
}

IOReturn PCIAudioDevice::spdifOutputSwitchChangeHandler(IOService *target, IOAudioControl *muteControl, int oldValue, int newValue)
{
#if DEBUG
        printf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;
        
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->spdif_switch_put(muteControl, audioDevice->accessibleEngineInstance, oldValue, newValue);
        }
        
        return result;
}

IOReturn PCIAudioDevice::spdif_switch_get(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
#if DEBUG
        printf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        
        IOLockLock(chip->mutex);
        newValue = chip->spdif_playback_enable;
        IOLockUnlock(chip->mutex);
        return 0;
}


IOReturn PCIAudioDevice::spdif_switch_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
#if DEBUG
        printf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        int changed;
        
        IOLockLock(chip->mutex);
        changed = newValue != chip->spdif_playback_enable;
        if (changed) {
                chip->spdif_playback_enable = !!newValue;
                IOSimpleLockLock(chip->reg_lock);
                engine->spdifEngine->oxygen_update_spdif_source(chip);
                IOSimpleLockUnlock(chip->reg_lock);
        }
        IOLockUnlock(chip->mutex);
        //return changed;
        return kIOReturnSuccess;
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

IOReturn PCIAudioDevice::spdifOutputPCMChangeHandler(IOService *target, IOAudioControl *muteControl, int oldValue, int newValue)
{
#if DEBUG
        printf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;
        
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->spdif_pcm_put(muteControl, audioDevice->accessibleEngineInstance, oldValue, newValue);
        }
        
        return result;
}

IOReturn PCIAudioDevice::spdif_pcm_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
#if DEBUG
        printf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        UInt32 new_bits;
        int changed;
        
        new_bits = iec958_to_oxygen(newValue);
        IOLockLock(chip->mutex);
        changed = new_bits != chip->spdif_pcm_bits;
        if (changed) {
                chip->spdif_pcm_bits = new_bits;
                if (chip->pcm_active & (1 << PCM_SPDIF))
                        write_spdif_bits(chip, new_bits);
        }
        IOLockUnlock(chip->mutex);
        //return changed;
        return kIOReturnSuccess;
}

IOReturn PCIAudioDevice::spdifInputDefaultChangeHandler(IOService *target, IOAudioControl *muteControl, int oldValue, int newValue)
{
#if DEBUG
        printf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;
        
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->spdif_input_default_get(muteControl, audioDevice->accessibleEngineInstance, oldValue, newValue);
        }
        
        return result;
}

IOReturn PCIAudioDevice::spdif_input_default_get(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
#if DEBUG
        printf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        UInt32 bits;
        
        UInt8 status[4];
        memcpy(&status, &newValue, sizeof(newValue));
        
        
        bits = oxygen_read32(chip, OXYGEN_SPDIF_INPUT_BITS);
        status[0] = bits;
        status[1] = bits >> 8;
        status[2] = bits >> 16;
        status[3] = bits >> 24;
        // return 0;
        
        
        return kIOReturnSuccess;
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

IOReturn PCIAudioDevice::spdifInputLoopbackSwitchChangeHandler(IOService *target, IOAudioControl *muteControl, int oldValue, int newValue)
{
#if DEBUG
        printf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;
        
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->spdif_bit_switch_put(muteControl, audioDevice->accessibleEngineInstance, oldValue, newValue, OXYGEN_SPDIF_LOOPBACK);
        }
        
        return result;
}

IOReturn PCIAudioDevice::spdif_bit_switch_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue, int private_value)
{
#if DEBUG
        printf("PCIAudioDevice::%-32s(%d, %d)\n",__func__, oldValue, newValue);
#endif
        
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        
        UInt32 bit = private_value;
        UInt32 oldreg, newreg;
        int changed;
        
        IOSimpleLockLock(chip->reg_lock);
        oldreg = oxygen_read32(chip, OXYGEN_SPDIF_CONTROL);
        if (newValue)
                newreg = oldreg | bit;
        else
                newreg = oldreg & ~bit;
        changed = newreg != oldreg;
        if (changed)
                oxygen_write32(chip, OXYGEN_SPDIF_CONTROL, newreg);
        IOSimpleLockUnlock(chip->reg_lock);
        //return changed;
        return kIOReturnSuccess;
}


IOReturn PCIAudioDevice::spdifInputValiditySwitchChangeHandler(IOService *target, IOAudioControl *muteControl, int oldValue, int newValue)
{
#if DEBUG
        printf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;
        
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->spdif_bit_switch_put(muteControl, audioDevice->accessibleEngineInstance, oldValue, newValue, OXYGEN_SPDIF_SPDVALID);
        }
        
        return result;
}



IOReturn PCIAudioDevice::spdif_bit_switch_get(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue, int private_value)
{
#if DEBUG
        printf("PCIAudioDevice::%-32s(%d, %d, %d)\n", __func__, oldValue, newValue, private_value);
#endif
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        UInt32 bit = private_value;
        
        newValue = !!(oxygen_read32(chip, OXYGEN_SPDIF_CONTROL) & bit);
        
        return kIOReturnSuccess;
        
}


IOReturn PCIAudioDevice::gainChangeHandler(IOService *target, IOAudioControl *gainControl, int oldValue, int newValue)
{
        
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;
        
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->gainChanged(gainControl, oldValue, newValue);
        }
        
        return result;
}

IOReturn PCIAudioDevice::gainChanged(IOAudioControl *gainControl, int oldValue, int newValue)
{
#if DEBUG
        kprintf("PCIAudioDevice::%-32s(%p, %d, %d)\n", __func__,gainControl, oldValue, newValue);
#endif
        //put in dac_volume_put
        if (gainControl) {
                //printf("\t-> Channel %ld\n", gainControl->getChannelID());
        }
        
        // Add hardware gain change code here
        
        return kIOReturnSuccess;
}

IOReturn PCIAudioDevice::inputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, int oldValue, int newValue)
{
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;
        
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->inputMuteChanged(muteControl, oldValue, newValue);
        }
        
        return result;
}

IOReturn PCIAudioDevice::inputMuteChanged(IOAudioControl *muteControl, int oldValue, int newValue)
{
        //printf("SamplePCIAudioDevice[%p]::inputMuteChanged(%p, %d, %d)\n", this, muteControl, oldValue, newValue);
        
        // Add input mute change code here
        
        return kIOReturnSuccess;
}

