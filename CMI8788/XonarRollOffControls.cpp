//
//  XonarRollOffControls.cpp
//  PCIAudioDriver
//
//  Created by Gagan on 2022-12-17.
//  Copyright © 2022 CMedia. All rights reserved.
//

#include "PCIAudioDevice.hpp"
#include "XonarAudioEngine.hpp"
#include "oxygen.h"
#include "alsa.h"

#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/audio/IOAudioLevelControl.h>
#include <IOKit/audio/IOAudioToggleControl.h>
#include <IOKit/audio/IOAudioPort.h>
#include <IOKit/audio/IOAudioDefines.h>


IOReturn PCIAudioDevice::RollOffSelectHandler(IOService *target, IOAudioControl *SelectorControl,
                                                  const void *oldData, UInt32 oldDataSize, const void* newData, UInt32 newDataSize)
{
#if DEBUG
        kprintf("PCIAudioDevice::%-32s START\n", __func__);
#endif
        IOReturn result = kIOReturnBadArgument;
        char *newValue = (char *)newData; // there's the new value plus the private value, so doing this reduces code
        char *oldValue = (char *)oldData;
        PCIAudioDevice *audioDevice;
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("PCIAudioDevice::%-32s oldValue:%-15s oldValueSize:%d\n"
                "PCIAudioDevice::%-32s newValue:%-15s newValueSize:%d\n",
                " ", oldValue, oldDataSize,
                " ", newValue, newDataSize);
#endif
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->accessibleEngineInstance->rolloff_put(SelectorControl, audioDevice->accessibleEngineInstance, oldData, oldDataSize, newData, newDataSize);
        }
#if DEBUG
        kprintf("PCIAudioDevice::%-32s END\n", __func__);
#endif
        return result;
}

