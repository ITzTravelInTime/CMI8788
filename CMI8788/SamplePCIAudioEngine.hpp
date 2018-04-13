/*
 File:SamplePCIAudioEngine.h
 
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


#ifndef _SAMPLEPCIAUDIOENGINE_H
#define _SAMPLEPCIAUDIOENGINE_H

#include <IOKit/audio/IOAudioEngine.h>

#include "CMI8788.hpp"

#define SamplePCIAudioEngine com_MyCompany_driver_SamplePCIAudioEngine

class IOFilterInterruptEventSource;
class IOInterruptEventSource;

class SamplePCIAudioEngine : public IOAudioEngine
{
    OSDeclareDefaultStructors(SamplePCIAudioEngine)
    
    struct generic_data                   *deviceRegisters;
    
    SInt16							*outputBuffer;
    SInt16							*inputBuffer;
    
    IOFilterInterruptEventSource	*interruptEventSource;
    
public:
    
    
    int oxygen_pci_probe(struct pci_dev *pci, int index, char *id,
                         struct module *owner,
                         const struct pci_device_id *ids,
                         int (*get_model)(struct oxygen *chip,
                                          const struct pci_device_id *id
                                          )
                         );
    void oxygen_pci_remove(struct pci_dev *pci);
#ifdef CONFIG_PM_SLEEP
    extern const struct dev_pm_ops oxygen_pci_pm;
#endif
    void oxygen_pci_shutdown(struct pci_dev *pci);
    
    /* oxygen_mixer.c */
    
    int oxygen_mixer_init(struct oxygen *chip);
    void oxygen_update_dac_routing(struct oxygen *chip);
    void oxygen_update_spdif_source(struct oxygen *chip);
    
    /* oxygen_pcm.c */
    
    int oxygen_pcm_init(struct oxygen *chip);
    
    /* oxygen_io.c */
    
    UInt8 oxygen_read8(struct oxygen *chip, unsigned int reg);
    UInt16 oxygen_read16(struct oxygen *chip, unsigned int reg);
    UInt32 oxygen_read32(struct oxygen *chip, unsigned int reg);
    void oxygen_write8(struct oxygen *chip, unsigned int reg, UInt8 value);
    void oxygen_write16(struct oxygen *chip, unsigned int reg, UInt16 value);
    void oxygen_write32(struct oxygen *chip, unsigned int reg, UInt32 value);
    void oxygen_write8_masked(struct oxygen *chip, unsigned int reg,
                              UInt8 value, UInt8 mask);
    void oxygen_write16_masked(struct oxygen *chip, unsigned int reg,
                               UInt16 value, UInt16 mask);
    void oxygen_write32_masked(struct oxygen *chip, unsigned int reg,
                               UInt32 value, UInt32 mask);
    
    UInt16 oxygen_read_ac97(struct oxygen *chip, unsigned int codec,
                         unsigned int index);
    void oxygen_write_ac97(struct oxygen *chip, unsigned int codec,
                           unsigned int index, UInt16 data);
    void oxygen_write_ac97_masked(struct oxygen *chip, unsigned int codec,
                                  unsigned int index, UInt16 data, UInt16 mask);
    
    int oxygen_write_spi(struct oxygen *chip, UInt8 control, unsigned int data);
    void oxygen_write_i2c(struct oxygen *chip, UInt8 device, UInt8 map, UInt8 data);
    
    void oxygen_reset_uart(struct oxygen *chip);
    void oxygen_write_uart(struct oxygen *chip, UInt8 data);
    
    UInt16 oxygen_read_eeprom(struct oxygen *chip, unsigned int index);
    void oxygen_write_eeprom(struct oxygen *chip, unsigned int index, UInt16 value);
    
        
    virtual bool init(struct oxygen *regs);
    virtual void free();
    
    virtual bool initHardware(IOService *provider);
    virtual void stop(IOService *provider);
    
    virtual IOAudioStream *createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize);
    
    virtual IOReturn performAudioEngineStart();
    virtual IOReturn performAudioEngineStop();
    
    virtual UInt32 getCurrentSampleFrame();
    
    virtual IOReturn performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate);
    
    virtual IOReturn clipOutputSamples(const void *mixBuf, void *sampleBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
    virtual IOReturn convertInputSamples(const void *sampleBuf, void *destBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
    
    static void interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count);
    static bool interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source);
    virtual void filterInterrupt(int index);
};

#endif /* _SAMPLEPCIAUDIOENGINE_H */
