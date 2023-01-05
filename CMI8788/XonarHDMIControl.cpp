//
//  XonarHDMIControl.cpp
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


IOReturn PCIAudioDevice::gpioBitSwitchHandler(IOService *target, IOAudioControl *ToggleControl,
                                                  const void *oldData, UInt32 oldDataSize, const void* newData, UInt32 newDataSize)
{
#if DEBUG
        kprintf("PCIAudioDevice::%-32s START\n", __func__);
#endif
        IOReturn result = kIOReturnBadArgument;
        int *newValue = (int *)newData; // there's the new value plus the private value, so doing this reduces code
        int *oldValue = (int *)oldData;
        PCIAudioDevice *audioDevice;
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("PCIAudioDevice::%-32s oldValue[0]:%d%-5s oldValue[1]:%d%-5s oldValueSize:%d\n"
                "PCIAudioDevice::%-32s newValue[0]:%d%-5s newValue[1]:%d%-5s newValueSize:%d\n",
                " ", oldValue[0], " ", oldValue[1], " ", oldDataSize,
                " ", newValue[0], " ", newValue[1], " ", newDataSize);
#endif
        audioDevice = (PCIAudioDevice *)target;
        if (audioDevice) {
                result = audioDevice->accessibleEngineInstance->xonar_gpio_bit_switch_put(ToggleControl, audioDevice->accessibleEngineInstance, oldValue[0], newValue[0], newValue[1]);
        }
#if DEBUG
        kprintf("PCIAudioDevice::%-32s valueChanged:%d\n", " ", result);
        kprintf("PCIAudioDevice::%-32s END\n", __func__);
#endif
        return kIOReturnSuccess;
}

