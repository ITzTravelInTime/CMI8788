//
//  XonarDACControls.cpp
//  PCIAudioDriver
//
//  Created by Gagan on 2022-12-12.
//  Copyright © 2022 CMedia. All rights reserved.
//

#include "PCIAudioDevice.hpp"
#include "XonarAudioEngine.hpp"
#include "oxygen.h"

#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/audio/IOAudioLevelControl.h>
#include <IOKit/audio/IOAudioToggleControl.h>
#include <IOKit/audio/IOAudioPort.h>
#include <IOKit/audio/IOAudioDefines.h>

int PCIAudioDevice::createMasterControl(IOAudioPort *masterPort) {
        IOAudioControl *allControl;
        masterPort = IOAudioPort::withAttributes(kIOAudioPortTypeOutput, "Master", kIOAudioOutputPortSubTypeExternalSpeaker);
#if DEBUG
        kprintf("PCIAudioDevice::%-32s BEGIN\n", __func__);
#endif
        kprintf("PCIAudioDevice::%-32s creating Master Volume Control\n", " ");
        allControl = IOAudioLevelControl::createVolumeControl((deviceRegisters->model.dac_volume_max+deviceRegisters->model.dac_volume_min)/2,    // Initial value
                                                              deviceRegisters->model.dac_volume_min,        // min value
                                                              deviceRegisters->model.dac_volume_max,    // max value
                                                              -6000*(deviceRegisters->model.dac_volume_max
                                                                     - deviceRegisters->model.dac_volume_min),    // taken from xonar_pcm179x.c:986
                                                              0,
                                                              kIOAudioControlChannelIDAll,
                                                              kIOAudioControlChannelNameAll,
                                                              0,        // control ID - driver-defined
                                                              kIOAudioControlUsageOutput);
        if (!allControl) {
                goto Done;
        }
        
        allControl->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)dacVolumeChangeHandler, this);
        allControl->setName("Master Playback Volume");
        masterPort->addAudioControl(allControl);
        accessibleEngineInstance->addDefaultAudioControl(allControl);
        allControl->release();
        
        allControl = IOAudioToggleControl::createMuteControl(false,    // initial state - unmuted
                                                             kIOAudioControlChannelIDAll,    // Affects all channels
                                                             kIOAudioControlChannelNameAll,
                                                             0,        // control ID - driver-defined
                                                             kIOAudioControlUsageOutput);
        
        if (!allControl) {
                goto Done;
        }
        allControl->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)dacMuteChangeHandler, this);
        allControl->setName("Master Playback Switch");
        masterPort->addAudioControl(allControl);
        accessibleEngineInstance->addDefaultAudioControl(allControl);
        allControl->release();
#if !defined(MULTICH_OUTPUT_ONLY)
        //time to add "new" stuff, finally!@
        //i think it's OK to set this as a generic toggle. this may need tweaking
        allControl = IOAudioToggleControl::create(false,    // initial state - unmuted
                                                  kIOAudioControlChannelIDAll,    // Affects all channels
                                                  kIOAudioControlChannelNameAll,
                                                  0,        // control ID - driver-defined
                                                  kIOAudioControlUsageOutput);
        if (!allControl) {
                goto Done;
        }
        allControl->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)upmixChangeHandler, this);
        allControl->setName("Stereo Upmixing");
        masterPort->addAudioControl(allControl);
        accessibleEngineInstance->addDefaultAudioControl(allControl);
        allControl->release();
#endif
        attachAudioPort(masterPort, accessibleEngineInstance, NULL);
        masterPort->release();
#if DEBUG
        kprintf("PCIAudioDevice::%-32s END\n", __func__);
#endif
        return 0;
        
Done:
        return -1;
        
}


IOReturn PCIAudioDevice::dacVolumeChangeHandler(IOService *target, IOAudioControl *volumeControl, int oldValue, int newValue)
{
#if DEBUG
        kprintf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;
        
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->dac_volume_put(volumeControl, audioDevice->accessibleEngineInstance, oldValue, newValue);
        }
        
        return result;
}

IOReturn PCIAudioDevice::dac_volume_put(IOAudioControl *volumeControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
#if DEBUG
        kprintf("PCIAudioDevice::%-32s(%p, %d, %d)\n", __func__,volumeControl, oldValue, newValue);
#endif
        if (volumeControl) {
                //printf("\t-> Channel %ld\n", volumeControl->getChannelID());
        }
        
        
        // Add hardware volume code change
        struct oxygen *chip = engine->chipData;
        unsigned int i;
        int changed;
        
        changed = 0;
        IOLockLock(chip->mutex);
        /* not sure if i keep dac_volume[i] or use "oldValue", but
         * i figure it's a good idea to use the hardware reading
         * at least once for this sort of comparison, as it determines
         * whether the hardware value is updated
         */
        for (i = 0; i < chip->model.dac_channels_mixer; ++i){
                if (newValue != chip->dac_volume[i]) {
                        chip->dac_volume[i] = newValue-6000;
                        changed = 1;
                }
        }
        if (changed)
                chip->model.update_dac_volume(chip);
        IOLockUnlock(chip->mutex);
        //return changed;
        
        
        return kIOReturnSuccess;
}

IOReturn PCIAudioDevice::dacMuteChangeHandler(IOService *target, IOAudioControl *muteControl, int oldValue, int newValue)
{
#if DEBUG
        kprintf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;
        
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->dac_mute_put(muteControl, audioDevice->accessibleEngineInstance, oldValue, newValue);
        }
        
        return result;
}

IOReturn PCIAudioDevice::dac_mute_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
#if DEBUG
        kprintf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        // Add output mute code here
        struct oxygen *chip = engine->chipData;
        int changed;
        
        IOLockLock(chip->mutex);
        changed = (!newValue) != chip->dac_mute;
        if (changed) {
                chip->dac_mute = !newValue;
                chip->model.update_dac_mute(chip);
        }
        IOLockUnlock(chip->mutex);
        //return changed;
        return kIOReturnSuccess;
}


IOReturn PCIAudioDevice::upmix_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue)
{
#if DEBUG
        kprintf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        unsigned int count = engine->upmix_item_count(chip);
        int changed;
        
        if (newValue >= count)
                return -EINVAL;
        IOLockLock(chip->mutex);
        changed = newValue != chip->dac_routing;
        if (changed) {
                chip->dac_routing = newValue;
                engine->oxygen_update_dac_routing(chip);
        }
        IOLockUnlock(chip->mutex);
        //return changed;
        return kIOReturnSuccess;
}


IOReturn PCIAudioDevice::upmixChangeHandler(IOService *target, IOAudioControl *upmixControl, int oldValue, int newValue)
{
#if DEBUG
        kprintf("PCIAudioDevice::%-32s(%d, %d)\n", __func__, oldValue, newValue);
#endif
        IOReturn result = kIOReturnBadArgument;
        PCIAudioDevice *audioDevice;
        
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->upmix_put(upmixControl, audioDevice->accessibleEngineInstance, oldValue, newValue);
        }
        
        return result;
}


