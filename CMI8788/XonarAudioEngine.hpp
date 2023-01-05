/*
 File:SamplePCIAudioEngine.h
 
 Contains:
 
 Version:1.0.0
 
 Copyright:Copyright ) 1997-2000 by APPUL Computer, Inc., All Rights Reserved.
 
 Disclaimer:IMPORTANT:  This APPUL software is supplied to you by APPUL Computer, Inc.
 ("APPUL") in consideration of your agreement to the following terms, and your use,
 installation, modification or redistribution of this APPUL software constitutes acceptance
 of these terms.  If you do not agree with these terms, please do not use, install, modify or
 redistribute this APPUL software.
 
 In consideration of your agreement to abide by the following terms, and subject
 to these terms, APPUL grants you a personal, non-exclusive license, under APPUL's
 copyrights in this original APPUL software (the "APPUL Software"), to use, reproduce,
 modify and redistribute the APPUL Software, with or without modifications, in source and/or
 binary forms; provided that if you redistribute the APPUL Software in its entirety
 and without modifications, you must retain this notice and the following text
 and disclaimers in all such redistributions of the APPUL Software.  Neither the
 name, trademarks, service marks or logos of APPUL Computer, Inc. may be used to
 endorse or promote products derived from the APPUL Software without specific prior
 written permission from APPUL.  Except as expressly stated in this notice, no
 other rights or licenses, express or implied, are granted by APPUL herein,
 including but not limited to any patent rights that may be infringed by your derivative
 works or by other works in which the APPUL Software may be incorporated.
 
 The APPUL Software is provided by APPUL on an "AS IS" basis.  APPUL MAKES NO WARRANTIES,
 EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT,
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE APPUL SOFTWARE
 OR ITS USE AND OPERATION ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL APPUL
 BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE,
 REPRODUCTION, MODIFICATION AND/OR DISTRIBUTION OF THE APPUL SOFTWARE, HOWEVER CAUSED
 AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE), STRICT
 LIABILITY OR OTHERWISE, EVEN IF APPUL HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 */


#ifndef _XONARAUDIOENGINE_H
#define _XONARAUDIOENGINE_H

#include <IOKit/audio/IOAudioEngine.h>

#define XonarAudioEngine com_CMedia_CMI8788_XonarAudioEngine

#ifdef DEBUG
//do it like RALINK and introduce debug levels.
//set debuglevel to 1 unless it's specified via CFLAGS ("-DDEBUGLEVEL=<value>")
#ifndef DEBUGLEVEL
#define DEBUGLEVEL 1
#endif
#endif

#ifndef _IOKIT_IOFILTERINTERRUPTEVENTSOURCE_H
class IOFilterInterruptEventSource;
class IOInterruptEventSource;
#endif

//have to add this here if we want to use it in a prototype
#define PCIAudioDevice com_CMedia_CMI8788_PCIAudioDevice
class PCIAudioDevice;

#define XonarAC97AudioEngine com_CMedia_CMI8788_XonarAC97AudioEngine
class XonarAC97AudioEngine;

#define XonarSPDIFAudioEngine com_CMedia_CMI8788_XonarSPDIFAudioEngine
class XonarSPDIFAudioEngine;

typedef struct _buffers {
        IOBufferMemoryDescriptor *substreams[2];
} IOAudioStreamBuffers;


struct IOStream {
        IOBufferMemoryDescriptor *buffer;
        IOAudioStream *stream;
};

#define PCM_COUNT 6
class XonarAudioEngine : public IOAudioEngine
{
        OSDeclareDefaultStructors(XonarAudioEngine)
        friend class XonarSPDIFAudioEngine;
        friend class XonarAC97AudioEngine;
        friend class PCIAudioDevice;
        //each card in the family uses a different composition of
        //DMA engines. there are six possible DMA engines, each with
        //input and output. cards can use any (or all) of them, but
        //Clemens has said the most one card uses is three. anyways,
        //you get the idea
        IOAudioStreamBuffers                sampleBuffers[PCM_COUNT];
        IOStream                       *inputStream;
        IOStream                       *inputStream2;
        IOStream                       *outputStream;
        XonarSPDIFAudioEngine               *spdifEngine;
        XonarAC97AudioEngine                *ac97Engine;
        unsigned int                             channel_mask;
        bool                                multiInput;
        //UInt32							*outputBuffer;
        //UInt32							*inputBuffer;
        IOAudioSampleRate               initialSampleRate;
        //IOWorkLoop                      *workLoop;
        IOFilterInterruptEventSource	*interruptEventSource_main;
        //IOFilterInterruptEventSource	*gpioEventSource; unused (if runAction works in the interrupthandler, which is all we need)
        //IOFilterInterruptEventSource	*spdifEventSource; unused (see above line)
        //need this for the interrupt handler, as the filterInterrupt OS call doesn't allow us to pass parameters.
        void                            *dev_id;
        
        char                            *Description;
       // virtual IOPhysicalAddress getSampleBufferAddress(unsigned int channel, const IOAudioStreamDirection direction);
        
        int oxygen_prepare(unsigned int channel);
        void oxygen_spdif_hw_free(void);
        void oxygen_hw_free(unsigned int channel);
        int oxygen_hw_params(IOAudioStream *substream, int formatChange, UInt32 source,
                             const IOAudioStreamFormat *newFormat, const IOAudioStreamFormatExtension *newFormatEx);
        int oxygen_rec_a_hw_params(IOAudioStream *substream, int formatChange,
                                   const IOAudioStreamFormat *newFormat, const IOAudioStreamFormatExtension *newFormatEx,
                                   const IOAudioSampleRate *newSampleRate);
        int oxygen_rec_b_hw_params(IOAudioStream *substream, int formatChange,
                                   const IOAudioStreamFormat *newFormat, const IOAudioStreamFormatExtension *newFormatEx,
                                   const IOAudioSampleRate *newSampleRate);
        int oxygen_rec_c_hw_params(IOAudioStream *substream, int formatChange,
                                   const IOAudioStreamFormat *newFormat, const IOAudioStreamFormatExtension *newFormatEx,
                                   const IOAudioSampleRate *newSampleRate);
        int oxygen_multich_hw_params(IOAudioStream *substream, int formatChange,
                                     const IOAudioStreamFormat *newFormat, const IOAudioStreamFormatExtension *newFormatEx,
                                     const IOAudioSampleRate *newSampleRate);
        int oxygen_open(IOAudioStream *stream, unsigned int channel);
//        IOBufferMemoryDescriptor *streamBuffer(unsigned int channel, const IOAudioStreamDirection direction);
        
public:
        struct oxygen* chipData;// = (struct oxygen*) dev_id;
        
        virtual bool init(struct oxygen *regs, int model);
        virtual void free();
        
        virtual bool initHardware(IOService *provider);
        virtual void stop(IOService *provider);
        
        virtual IOStream *createAudioStream(IOAudioStreamDirection direction, /*void *sampleBuffer,*/
                                                 UInt32 sampleBufferSize, uint source);
        
        virtual IOReturn performAudioEngineStart();
        virtual IOReturn performAudioEngineStop();
        
        virtual UInt32 getCurrentSampleFrame();
        
        virtual IOReturn performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat,
                                             const IOAudioSampleRate *newSampleRate);
        virtual IOReturn clipOutputSamples(const void *mixBuf, void *sampleBuf, UInt32 firstSampleFrame,
                                           UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat,
                                           IOAudioStream *audioStream);
        virtual IOReturn convertInputSamples(const void *sampleBuf, void *destBuf, UInt32 firstSampleFrame,
                                             UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat,
                                             IOAudioStream *audioStream);
        
        
        static void interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count);
        static bool interruptFilter(OSObject *owner, IOFilterInterruptEventSource *src);
        virtual void filterInterrupt(int index);
        virtual void  filterStreams(IOAudioStream *stream, unsigned int channel,
                                   UInt32 filter);
        
        static void xonar_enable_output(struct oxygen *chip);
        static void xonar_disable_output(struct oxygen *chip);
        static void xonar_init_ext_power(struct oxygen *chip);
        static void xonar_init_cs53x1(struct oxygen *chip);
        static void xonar_set_cs53x1_params(struct oxygen *chip, XonarAudioEngine *instance);
        static void xonar_line_mic_ac97_switch(struct oxygen *chip, unsigned int reg,
                                               unsigned int mute);
        static void oxygen_reset_uart(struct oxygen *chip);
        static void oxygen_read_uart(struct oxygen *chip);
        static void oxygen_write_uart(struct oxygen *chip, UInt8 data);
        static void oxygen_spdif_input_bits_changed(struct oxygen *chip);
        //static void oxygen_update_spdif_source(struct oxygen *chip);
        
        static void oxygen_gpio_changed(struct oxygen* chip, XonarAudioEngine *engineInstance);
        
        int  add_pcm1796_controls(struct oxygen *chip, PCIAudioDevice *dev);
        static void pcm1796_init(struct oxygen *chip);
        static void pcm1796_registers_init(struct oxygen *chip);
        static void update_pcm1796_mute(struct oxygen *chip);
        static void update_pcm1796_oversampling(struct oxygen *chip);
        static void set_pcm1796_params(struct oxygen *chip, XonarAudioEngine *instance,
                                       IOAudioStream *currentStream);
        static void update_pcm1796_volume(struct oxygen *chip);
        static inline void pcm1796_write_spi(struct oxygen *chip, unsigned int codec,
                                             UInt8 reg, UInt8 value);
        static void pcm1796_write(struct oxygen *chip, unsigned int codec,
                                  UInt8 reg, UInt8 value);
        static void pcm1796_write_cached(struct oxygen *chip, unsigned int codec,
                                         UInt8 reg, UInt8 value);
        static inline void pcm1796_write_i2c(struct oxygen *chip, unsigned int codec,
                                             UInt8 reg, UInt8 value);
        
        
//        UInt16 oxygen_read_ac97(struct oxygen *chip, unsigned int codec,
//                                unsigned int index);
        //void oxygen_write_ac97(struct oxygen *chip, unsigned int codec,
        //                       unsigned int index, UInt16 data);
        void oxygen_restore_ac97(struct oxygen *chip, unsigned int codec);
        
        
        static int xonar_gpio_bit_switch_get(IOAudioControl *volumeControl, XonarAudioEngine *engine,
                                             int oldValue, int newValue, int private_value);
        static int xonar_gpio_bit_switch_put(IOAudioControl *volumeControl, XonarAudioEngine *engine,
                                             int oldValue, int newValue, int private_value);
        
        IOReturn gpioBitSwitchHandler(IOService *target, IOAudioControl *ToggleControl,
                                      void *oldData, UInt32 oldDataSize, void* newData, UInt32 newDataSize);
        static void cs2000_registers_init(struct oxygen *chip);
        static void update_cs2000_rate(struct oxygen *chip, unsigned int rate);
        static void cs2000_write(struct oxygen *chip, UInt8 reg, UInt8 value);
        static void cs2000_write_cached(struct oxygen *chip, UInt8 reg, UInt8 value);
        
        
        /* HDMI helper functions */
        static void hdmi_write_command(struct oxygen *chip, UInt8 command,
                                       unsigned int count, const UInt8 *params);
        static void xonar_hdmi_init_commands(struct oxygen *chip,
                                             struct xonar_hdmi *hdmi);
        static void xonar_hdmi_uart_input(struct oxygen *chip);
        void xonar_hdmi_init(struct oxygen *chip, struct xonar_hdmi *data);
        void xonar_hdmi_cleanup(struct oxygen *chip);
        void xonar_hdmi_resume(struct oxygen *chip, struct xonar_hdmi *hdmi);
        static void xonar_hdmi_pcm_hardware_filter(unsigned int channel,
                                                   IOAudioStream *hardware);
        void xonar_set_hdmi_params(struct oxygen *chip, struct xonar_hdmi *hdmi,
                                   IOAudioStream *currentStream);
        
        static void xonar_hdav_slim_hardware_filter(unsigned int channel,
                                                    IOAudioStream *hardware);
        static void wm8776_adc_hardware_filter(unsigned int channel,
                                               IOAudioStream *hardware);
        //generic
        
        static int rolloff_put(IOAudioControl *SelectorControl, XonarAudioEngine *engine,
                               const void *oldData, UInt32 oldDataSize, const void* newData, UInt32 newDataSize);
        
        
        
        //virtual int createControls(struct snd_kcontrol_new *_template);
        //    virtual int oxygen_mixer_init(void);
        virtual void oxygen_update_dac_routing(struct oxygen *chip);
        static unsigned int upmix_item_count(struct oxygen *chip);
        int oxygen_trigger();
        static unsigned int oxygen_format(const IOAudioStreamFormat *format);
        static unsigned int oxygen_rate(const IOAudioSampleRate *sampleRate);

        
};

void oxygen_write_ac97_masked(struct oxygen *chip, unsigned int codec,
                              unsigned int index, UInt16 data, UInt16 mask);

static inline void oxygen_ac97_set_bits(struct oxygen *chip, unsigned int codec,
                                        unsigned int index, UInt16 value)
{
        oxygen_write_ac97_masked(chip, codec, index, value, value);
}

static inline void oxygen_ac97_clear_bits(struct oxygen *chip,
                                          unsigned int codec,
                                          unsigned int index, UInt16 value)
{
        oxygen_write_ac97_masked(chip, codec, index, 0, value);
}

#endif /* _SAMPLEPCIAUDIOENGINE_H */
