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


#ifndef _XONARAUDIOENGINE_H
#define _XONARAUDIOENGINE_H
#import <IOKit/audio/IOAudioEngine.h>

#import "PCIAudioDevice.hpp"

#define XonarAudioEngine com_CMedia_CMI8788_XonarAudioEngine
#define XonarHDAVAudioEngine com_CMedia_CMI8788_XonarHDAVAudioEngine

struct xonar_generic {
    unsigned int anti_pop_delay;
    UInt16 output_enable_bit;
    UInt8 ext_power_reg;
    UInt8 ext_power_int_reg;
    UInt8 ext_power_bit;
    UInt8 has_power;
};

struct xonar_hdmi {
    UInt8 params[5];
};

struct xonar_pcm179x {
    struct xonar_generic generic;
    unsigned int dacs;
    UInt8 pcm1796_regs[4][5];
    IOAudioSampleRate *current_rate;
    bool h6;
    bool hp_active;
    SInt8 hp_gain_offset;
    bool has_cs2000;
    UInt8 cs2000_regs[0x1f];
    bool broken_i2c;
};

struct xonar_hdav {
    struct xonar_pcm179x pcm179x;
    struct xonar_hdmi hdmi;
};


struct xonar_cs43xx {
    struct xonar_generic generic;
    UInt8 cs4398_regs[8];
    UInt8 cs4362a_regs[15];
};



struct xonar_wm87x6 {
    struct xonar_generic generic;
    UInt16 wm8776_regs[0x17];
    UInt16 wm8766_regs[0x10];
    //struct snd_kcontrol *line_adcmux_control;
    //struct snd_kcontrol *mic_adcmux_control;
    //struct snd_kcontrol *lc_controls[13];
    //struct snd_jack *hp_jack;
    struct xonar_hdmi hdmi;
};


#define GPIO_CS53x1_M_MASK          0x000c
#define GPIO_CS53x1_M_SINGLE        0x0000
#define GPIO_CS53x1_M_DOUBLE        0x0004
#define GPIO_CS53x1_M_QUAD          0x0008
#define XONAR_GPIO_BIT_INVERT       (1 << 16)
#define GPIO_D2X_EXT_POWER          0x0020
#define GPIO_D2_ALT                 0x0080
#define GPIO_D2_OUTPUT_ENABLE       0x0100

#define GPI_EXT_POWER               0x01
#define GPIO_INPUT_ROUTE            0x0100

#define GPIO_HDAV_OUTPUT_ENABLE     0x0001
#define GPIO_HDAV_MAGIC             0x00c0

#define GPIO_DB_MASK                0x0030
#define GPIO_DB_H6                  0x0000

#define GPIO_ST_OUTPUT_ENABLE       0x0001
#define GPIO_ST_HP_REAR             0x0002
#define GPIO_ST_MAGIC               0x0040
#define GPIO_ST_HP                  0x0080

#define GPIO_XENSE_OUTPUT_ENABLE	(0x0001 | 0x0010 | 0x0020)
#define GPIO_XENSE_SPEAKERS         0x0080

#define I2C_DEVICE_PCM1796(i)       (0x98 + ((i) << 1))	/* 10011, ii, /W=0 */
#define I2C_DEVICE_CS2000           0x9c			/* 100111, 0, /W=0 */

#define I2C_DEVICE_CS4398           0x9e	/* 10011, AD1=1, AD0=1, /W=0 */
#define I2C_DEVICE_CS4362A          0x30	/* 001100, AD0=0, /W=0 */



#define PCM1796_REG_BASE            16

#define HDAV_MODEL                  0x8314
#define ST_MODEL                    0x835d
#define STX_MODEL                   0x835c
#define STX2_MODEL                  0x85f4
#define D2_MODEL                    0x8269
#define D2X_MODEL                   0x82b7
#define XENSE_MODEL                 0x8428
#define XONAR_GENERIC 7

#define D1_MODEL                    0x834F
#define CS4XX_MODEL                 0x8275
#define DX_MODEL                    0x8327

#define DS_MODEL                    0x838e
#define DSX_MODEL                   0x8522
#define HDAV_SLIM                   0x835e

//objective C does not use the bitwise operator
// so i am simply declaring SNDRV_PCM_FORMAT as-is
//Linux Def below for reference:
//typedef int __bitwise snd_pcm_format_t;
//#define	SNDRV_PCM_FORMAT_S16_LE	((__force snd_pcm_format_t) 2)
//OSX Def (may be wrong, but may not be [not sure about significance of bitwise here]):
#define SNDRV_PCM_FORMAT_S16_LE 2

class IOFilterInterruptEventSource;
class IOInterruptEventSource;
class PCIAudioDevice;

class XonarAudioEngine : public IOAudioEngine
{
    friend class XonarSTAudioEngine;
    friend class XonarHDAVAudioEngine;
    friend class PCIAudioDevice;
    OSDeclareDefaultStructors(XonarAudioEngine)
    
    //right now i've created 4 since there are 4 I2S input buffers
    // however, i am not sure how to incorporate them yet,
    // as i have to (probably) create an ioaudiostream for each
    // and then add the attributes.
    IOAudioStream                   *inputs[4];
    SInt16							*outputBuffer;
    SInt16							*inputBuffer;
    IOWorkLoop                      *workLoop;
    IOFilterInterruptEventSource	*interruptEventSource_main;
    //IOFilterInterruptEventSource	*gpioEventSource; unused (if runAction works in the interrupthandler, which is all we need)
    //IOFilterInterruptEventSource	*spdifEventSource; unused (see above line)
    //need this for the interrupt handler, as the filterInterrupt OS call doesn't allow us to pass parameters.
    void                            *dev_id;
    
public:
    struct oxygen* chipData;// = (struct oxygen*) dev_id;
    
    
    
    virtual bool init(struct oxygen *regs, int model);
    virtual void free();
    
    virtual bool initHardware(IOService *provider);
    virtual void stop(IOService *provider);
    
    virtual IOAudioStream *createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer,
                                                UInt32 sampleBufferSize);
    
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
    static void oxygen_write_i2c(struct oxygen *chip, UInt8 device, UInt8 map, UInt8 data);
    static int oxygen_write_spi(struct oxygen *chip, UInt8 control, unsigned int data);
    static void oxygen_gpio_changed(struct oxygen* chip);
    
    static int  add_pcm1796_controls(struct oxygen *chip);
    static void pcm1796_init(struct oxygen *chip);
    static void pcm1796_registers_init(struct oxygen *chip);
    static void update_pcm1796_mute(struct oxygen *chip, XonarAudioEngine *instance);
    static void update_pcm1796_oversampling(struct oxygen *chip);
    static void set_pcm1796_params(struct oxygen *chip, XonarAudioEngine *instance);
    static void update_pcm1796_volume(struct oxygen *chip, XonarAudioEngine *instance);
    static inline void pcm1796_write_spi(struct oxygen *chip, unsigned int codec,
                                         UInt8 reg, UInt8 value);
    static void pcm1796_write(struct oxygen *chip, unsigned int codec,
                              UInt8 reg, UInt8 value);
    static void pcm1796_write_cached(struct oxygen *chip, unsigned int codec,
                                     UInt8 reg, UInt8 value);
    static inline void pcm1796_write_i2c(struct oxygen *chip, unsigned int codec,
                                         UInt8 reg, UInt8 value);
    
    
    UInt16 oxygen_read_ac97(struct oxygen *chip, unsigned int codec,
                            unsigned int index);
    //void oxygen_write_ac97(struct oxygen *chip, unsigned int codec,
    //                       unsigned int index, UInt16 data);
    void oxygen_write_ac97_masked(struct oxygen *chip, unsigned int codec,
                                  unsigned int index, UInt16 data, UInt16 mask);
    
    
    int xonar_gpio_bit_switch_get(struct snd_kcontrol *ctl,
                                  struct snd_ctl_elem_value *value);
    int xonar_gpio_bit_switch_put(struct snd_kcontrol *ctl,
                                  struct snd_ctl_elem_value *value);

    
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
    // void xonar_hdmi_pcm_hardware_filter(unsigned int channel,
    //                                   struct snd_pcm_hardware *hardware);
    void xonar_set_hdmi_params(struct oxygen *chip, struct xonar_hdmi *hdmi);
    
    
    //generic
    
    
    
};

#endif /* _SAMPLEPCIAUDIOENGINE_H */
