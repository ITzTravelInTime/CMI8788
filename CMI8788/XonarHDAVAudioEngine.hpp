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


#ifndef _XonarHDAVAUDIOENGINE_H
#define _XonarHDAVAUDIOENGINE_H

#include <IOKit/audio/IOAudioEngine.h>

#include "XonarAudioEngine.hpp"

#define XonarHDAVAudioEngine com_CMedia_CMI8788_XonarHDAVAudioEngine

class XonarAudioEngine;
class XonarHDAVAudioEngine : public IOAudioEngine
{
    friend class XonarSTAudioEngine;
    OSDeclareDefaultStructors(XonarHDAVAudioEngine)
    
    struct xonar_hdav                   *deviceRegisters;
    //right now i've created 4 since there are 4 I2S input buffers
    // however, i am not sure how to incorporate them yet,
    // as i have to (probably) create an ioaudiostream for each
    // and then add the attributes.
    IOAudioStream                   *inputs[4];
    SInt16							*outputBuffer;
    SInt16							*inputBuffer;
    
    IOFilterInterruptEventSource	*interruptEventSource;
    XonarAudioEngine                *engineInstance;
public:
    
    
    
        
    virtual bool init(XonarAudioEngine *engine, struct oxygen *regs);
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
//    int oxygen_write_spi(struct oxygen *chip, UInt8 control, unsigned int data);
//    void xonar_ext_power_gpio_changed(struct oxygen *chip);
//    static void xonar_enable_output(struct oxygen *chip);
//    static void xonar_disable_output(struct oxygen *chip);
//    static void xonar_init_ext_power(struct oxygen *chip);
//    void xonar_set_cs53x1_params(struct oxygen *chip);
//    void xonar_init_cs53x1(struct oxygen *chip);

    static void xonar_hdav_init(struct oxygen *chip);
    static void xonar_hdav_resume(struct oxygen *chip);
    
    
//    static void pcm1796_init(struct oxygen *chip);
//    static void pcm1796_registers_init(struct oxygen *chip);
//    static void update_pcm1796_mute(struct oxygen *chip);
//    static void update_pcm1796_oversampling(struct oxygen *chip);
//    static void set_pcm1796_params(struct oxygen *chip, XonarHDAVAudioEngine *instance);
//    static void update_pcm1796_volume(struct oxygen *chip);
//
//    
//    int xonar_gpio_bit_switch_get(struct snd_kcontrol *ctl,
//                                  struct snd_ctl_elem_value *value);
//    int xonar_gpio_bit_switch_put(struct snd_kcontrol *ctl,
//                                  struct snd_ctl_elem_value *value);
    /* model-specific card drivers */
    
//    int get_xonar_pcm179x_model(struct oxygen *chip,
//                                const struct pci_device_id *id);
//    int get_xonar_cs43xx_model(struct oxygen *chip,
//                               const struct pci_device_id *id);
//    int get_xonar_wm87x6_model(struct oxygen *chip,
//                               const struct pci_device_id *id);
//    static void cs2000_registers_init(struct oxygen *chip);
//    static void update_cs2000_rate(struct oxygen *chip, unsigned int rate);
    static void xonar_hdav_cleanup(struct oxygen *chip);
    /* HDMI helper functions */
    static void hdmi_write_command(struct oxygen *chip, UInt8 command,
                                   unsigned int count, const UInt8 *params);
    
    static void xonar_hdmi_init_commands(struct oxygen *chip,
                                         struct xonar_hdmi *hdmi);

    
    static void xonar_hdmi_init(struct oxygen *chip, struct xonar_hdmi *data);
    static void xonar_hdmi_cleanup(struct oxygen *chip);
    static void xonar_hdmi_resume(struct oxygen *chip, struct xonar_hdmi *hdmi);
   // void xonar_hdmi_pcm_hardware_filter(unsigned int channel,
     //                                   struct snd_pcm_hardware *hardware);
    static void set_hdav_params(struct oxygen *chip, XonarHDAVAudioEngine *instance);
    static void xonar_set_hdmi_params(struct oxygen *chip, struct xonar_hdmi *hdmi);
    static void xonar_hdmi_uart_input(struct oxygen *chip);
    
    //void _write_uart(struct oxygen *chip, unsigned int port, UInt8 data);
    //void oxygen_reset_uart(struct oxygen *chip);
    //void oxygen_write_uart(struct oxygen *chip, UInt8 data);
};

#endif /* _SAMPLEPCIAUDIOENGINE_H */