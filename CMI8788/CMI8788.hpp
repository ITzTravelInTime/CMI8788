/*
 File:SamplePCIAudioDevice.h
 
 Contains:
 
 Version:1.0.0
 
 Copyright:Copyright ) 1997-2000 by Apple Computer, Inc., All Rights Reserved.
 
 Disclaimer:IMPORTANT:  This Apple software is supplied to you by Apple Computer, Inc.
 ("Apple") in consideration of your agreement to the following terms, and your use,
 installation, modification or redistribution of this Apple software constitutes acceptance
 of these terms.  If you do not agree with these terms, please do not use, install, modify or
 redistribute this Apple software.
 
 In consideration of your agreement to abide by the following terms, and subject
 to these terms, Apple grants you a personal, non-exclusive license, under Apple's
 copyrights in this original Apple software (the "Apple Software"), to use, reproduce,
 modify and redistribute the Apple Software, with or without modifications, in source and/or
 binary forms; provided that if you redistribute the Apple Software in its entirety
 and without modifications, you must retain this notice and the following text
 and disclaimers in all such redistributions of the Apple Software.  Neither the
 name, trademarks, service marks or logos of Apple Computer, Inc. may be used to
 endorse or promote products derived from the Apple Software without specific prior
 written permission from Apple.  Except as expressly stated in this notice, no
 other rights or licenses, express or implied, are granted by Apple herein,
 including but not limited to any patent rights that may be infringed by your derivative
 works or by other works in which the Apple Software may be incorporated.
 
 The Apple Software is provided by Apple on an "AS IS" basis.  APPLE MAKES NO WARRANTIES,
 EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT,
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE APPLE SOFTWARE
 OR ITS USE AND OPERATION ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL APPLE
 BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE,
 REPRODUCTION, MODIFICATION AND/OR DISTRIBUTION OF THE APPLE SOFTWARE, HOWEVER CAUSED
 AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE), STRICT
 LIABILITY OR OTHERWISE, EVEN IF APPLE HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 */


#ifndef _SAMPLEPCIAUDIODEVICE_H
#define _SAMPLEPCIAUDIODEVICE_H

#include <IOKit/audio/IOAudioDevice.h>
#include <libkern/OSAtomic.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOLocks.h>
#include <sys/types.h>
#include <mach/semaphore.h>
#include <mach/task.h>

#define OXYGEN_IO_SIZE  0x100


typedef struct SamplePCIAudioDeviceRegisters {
    UInt32	reg1;
    UInt32	reg2;
    UInt32	reg3;
    UInt32	reg4;
} SamplePCIAudioDeviceRegisters;


struct generic_data {
    unsigned int dacs;
    UInt8 ak4396_regs[4][5];
    UInt16 wm8785_regs[3];
};


struct oxygen_model {
    
    const unsigned int *dac_tlv;
    size_t model_data_size;
    unsigned int device_config;
    UInt8 dac_channels_pcm;
    UInt8 dac_channels_mixer;
    UInt8 dac_volume_min;
    UInt8 dac_volume_max;
    UInt8 misc_flags;
    UInt8 function_flags;
    UInt8 dac_mclks;
    UInt8 adc_mclks;
    UInt16 dac_i2s_format;
    UInt16 adc_i2s_format;
    
};

struct oxygen {
    unsigned long addr;
    OSSpinLock reg_lock;
    IOLock *mutex;
    struct snd_card *card;
    struct pci_dev *pci;
    struct snd_rawmidi *midi;
    int irq;
    void *model_data;
    unsigned int interrupt_mask;
    UInt8 dac_volume[8];
    UInt8 dac_mute;
    UInt8 pcm_active;
    UInt8 pcm_running;
    UInt8 dac_routing;
    UInt8 spdif_playback_enable;
    UInt8 has_ac97_0;
    UInt8 has_ac97_1;
    UInt32 spdif_bits;
    UInt32 spdif_pcm_bits;
 //   struct snd_pcm_substream *streams[PCM_COUNT];
 //   struct snd_kcontrol *controls[CONTROL_COUNT];
    IOWorkLoop spdif_input_bits_work;
    IOWorkLoop gpio_work;
    queue_head_t ac97_waitqueue;
    union {// have to swap these ... remember.
        UInt8 _8[OXYGEN_IO_SIZE];
        SInt16 _16[OXYGEN_IO_SIZE / 2];
        SInt32 _32[OXYGEN_IO_SIZE / 4];
    } saved_registers;
    UInt16 saved_ac97_registers[2][0x40];
    unsigned int uart_input_count;
    UInt8 uart_input[32];
    struct oxygen_model model;
};



class IOPCIDevice;
class IOMemoryMap;

#define PCIAudioDevice com_CMedia_CMI8788_PCIAudioDevice

class PCIAudioDevice : public IOAudioDevice
{
    friend class SampleAudioEngine;
    
    OSDeclareDefaultStructors(PCIAudioDevice)
    
    IOPCIDevice					*pciDevice;
    IOMemoryMap					*deviceMap;
    
    struct oxygen               *deviceRegisters;
    
    virtual bool initHardware(IOService *provider);
    virtual bool createAudioEngine();
    virtual void free();
    
    static IOReturn volumeChangeHandler(IOService *target, IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn volumeChanged(IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue);
    
    static IOReturn outputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn outputMuteChanged(IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);
    
    static IOReturn gainChangeHandler(IOService *target, IOAudioControl *gainControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn gainChanged(IOAudioControl *gainControl, SInt32 oldValue, SInt32 newValue);
    
    static IOReturn inputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn inputMuteChanged(IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);
};

#endif /* _SAMPLEPCIAUDIODEVICE_H */