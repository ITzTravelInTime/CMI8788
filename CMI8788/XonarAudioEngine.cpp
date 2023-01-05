/*
 File:XonarAudioEngine.cpp
 
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



#include <IOKit/IOFilterInterruptEventSource.h>
#include "PCIAudioDevice.hpp"
#include "XonarAudioEngine.hpp"
#include "XonarSPDIFAudioEngine.hpp"
#include "XonarAC97AudioEngine.hpp"
#include "oxygen.h"
#include "alsa.h"
#include "ak4396.h"
#include "ac97.h"
#include "pcm1796.h"
#include "cm9780.h"
#include "cs2000.h"
#include "cs4398.h"
#include "cs4362a.h"

#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/audio/IOAudioPort.h>
#include <IOKit/audio/IOAudioSelectorControl.h>
#include <IOKit/audio/IOAudioDefines.h>

#define INITIAL_SAMPLE_RATE	44100
#define NUM_SAMPLE_FRAMES	16384
#define NUM_CHANNELS		2
#define BIT_DEPTH			16

#include "hexdumpfn.c"
#define super IOAudioEngine

OSDefineMetaClassAndStructors(XonarAudioEngine, IOAudioEngine)

#if DEBUG && DEBUGLEVEL > 2

static const char *formatSampleFormat(UInt32 numericRep) {
        
        switch(numericRep) {
                case kIOAudioStreamSampleFormatLinearPCM:
                        return "lpcm";
                case kIOAudioStreamSampleFormatIEEEFloat:
                        return "ieee";
                case kIOAudioStreamSampleFormatALaw:
                        return "alaw";
                case kIOAudioStreamSampleFormatMuLaw:
                        return "ulaw";
                case kIOAudioStreamSampleFormatMPEG:
                        return  "mpeg";
                case kIOAudioStreamSampleFormatAC3:
                        return  "ac-3";
                case kIOAudioStreamSampleFormat1937AC3:
                        return  "cac3";
                case kIOAudioStreamSampleFormat1937MPEG1:
                        return  "mpg1";
                case kIOAudioStreamSampleFormat1937MPEG2:
                        return  "mpg2";
                case kIOAudioStreamSampleFormatTimeCode:
                        return  "time";
                default:
                        return "";
                        
        }
}
// only include these functions if we want easily-interpretable output for the format
static const char *formatRep(UInt32 numericRep){
        switch (numericRep) {
                case kIOAudioStreamNumericRepresentationSignedInt:
                        return "sint";
                case kIOAudioStreamNumericRepresentationUnsignedInt:
                        return "uint";
                case kIOAudioStreamNumericRepresentationIEEE754Float:
                        return "flot";
                default:
                        return " ";
        };
        
}
static const char *formatAlignment(UInt8 numericRep) {
        switch (numericRep) {
                case kIOAudioStreamAlignmentLowByte:
                        return "LowByte";
                case kIOAudioStreamAlignmentHighByte:
                        return "HighByte";
                default:
                        return " ";
        }
}
static const char *formatByteOrder(UInt8 numericRep){
        switch (numericRep ){
                case kIOAudioStreamByteOrderBigEndian:
                        return "Eric Lindros (\"The Big E\")";
                case kIOAudioStreamByteOrderLittleEndian:
                        return "Li'l PennE";
                default:
                        return " ";
                        
        }
        
}

const char *getStringStatus(long stateVal) {
        
        switch(stateVal) {
                        
                case kIOAudioEngineStopped:
                        return "Stopped";
                case kIOAudioEngineRunning:
                        return "Running";
                case kIOAudioEnginePaused:
                        return "Paused";
                case kIOAudioEngineResumed:
                        return "Resumed";
                default:
                        return "NULL";
        }
}
#endif
const IOAudioStreamFormat ac97_format = {
        2,                                                // num channels
        kIOAudioStreamSampleFormatLinearPCM,            // sample format
        kIOAudioStreamNumericRepresentationSignedInt,    // numeric format
        16,                                        // bit depth
        16,                                        // bit width
        kIOAudioStreamAlignmentLowByte,                // high byte aligned - unused because bit depth == bit width
        kIOAudioStreamByteOrderLittleEndian,                // little endian
        true,                                            // format is mixable
        PCM_AC97                                                // driver-defined tag - unused by this driver
};
const IOAudioSampleRate ac97_sampleRate = {48000, 0};

const IOAudioStreamFormat multich_format = {
        8,
        kIOAudioStreamSampleFormatLinearPCM,
        kIOAudioStreamNumericRepresentationSignedInt,
        16,
        16,
        kIOAudioStreamAlignmentLowByte,                // high byte aligned - unused because bit depth == bit width
        kIOAudioStreamByteOrderLittleEndian,                // little endian
        true,                                            // format is mixable
        PCM_MULTICH                                                // driver-defined tag - unused by this driver
};
const IOAudioStreamFormat oxygen_hardware[] = {
        {
                2,
                kIOAudioStreamSampleFormatLinearPCM,
                kIOAudioStreamNumericRepresentationSignedInt,
                16,
                16,
                kIOAudioStreamAlignmentLowByte,                // high byte aligned - unused because bit depth == bit width
                kIOAudioStreamByteOrderLittleEndian,                // little endian
                true,                                            // format is mixable
                PCM_A                                                // driver-defined tag
        }, //PCM_A
        {
                2,
                kIOAudioStreamSampleFormatLinearPCM,
                kIOAudioStreamNumericRepresentationSignedInt,
                16,
                16,
                kIOAudioStreamAlignmentLowByte,                // high byte aligned - unused because bit depth == bit width
                kIOAudioStreamByteOrderLittleEndian,                // little endian
                true,                                            // format is mixable
                PCM_B                                                // driver-defined tag
        }, //PCM_B
        {
                2,
                kIOAudioStreamSampleFormatLinearPCM,
                kIOAudioStreamNumericRepresentationSignedInt,
                16,
                16,
                kIOAudioStreamAlignmentLowByte,                // high byte aligned - unused because bit depth == bit width
                kIOAudioStreamByteOrderLittleEndian,                // little endian
                true,                                            // format is mixable
                PCM_C                                                // driver-defined tag
        }, //PCM_C
        {
                2,
                kIOAudioStreamSampleFormatLinearPCM,
                kIOAudioStreamNumericRepresentationSignedInt,
                16,
                16,
                kIOAudioStreamAlignmentLowByte,                // high byte aligned - unused because bit depth == bit width
                kIOAudioStreamByteOrderLittleEndian,                // little endian
                true,                                            // format is mixable
                PCM_SPDIF                                                // driver-defined tag
        }, //PCM_SPDIF
        multich_format, //PCM_MULTICH
        ac97_format
};


const IOAudioSampleRate validRates[] = {
        {32000,0},
        {44100,0},
        {48000,0},
        {64000,0},
        {88200,0},
        {96000,0},
        {176400,0},
        {192000,0}
};

const unsigned int validRatesbits[] = {
        SNDRV_PCM_RATE_32000,
        SNDRV_PCM_RATE_44100,
        SNDRV_PCM_RATE_48000,
        SNDRV_PCM_RATE_64000,
        SNDRV_PCM_RATE_88200,
        SNDRV_PCM_RATE_96000,
        SNDRV_PCM_RATE_176400,
        SNDRV_PCM_RATE_192000
};



//filterStreams is only called on non AC-97 streams so we know 32 they support 32 bit
void XonarAudioEngine::filterStreams(IOAudioStream *stream, unsigned int channel,
                                     UInt32 filter) {
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#endif
        stream->clearAvailableFormats(); // only way we can filter is to remove and re-add
        //copy voodoohda's 'template objects' approach to make some comparisons easier
        IOAudioStreamFormat format = {
                2,                                        // number of channels
                0,                                              // sample format (to be filled in)
                kIOAudioStreamNumericRepresentationSignedInt,    // numeric format
                0,                                                // bit depth (to be filled in)
                0,                                                // bit width (to be filled in)
                kIOAudioStreamAlignmentLowByte,                    // low byte aligned
                kIOAudioStreamByteOrderLittleEndian,            // little endian
                true,                                            // format is mixable
                channel                                                // driver-defined tag
        };
        
        IOAudioStreamFormatExtension formatEx = {
                kFormatExtensionCurrentVersion,                    // version
                0,                                              // flags
                0,                                                // frames per packet (to be filled in)
                0                                                // bytes per packet (to be filled in)
        };
        
        int loopCount = 0; //PCM_MULTICH needs 4 times as many formats because of the 2 channel step increments,

        if(channel != PCM_MULTICH) {
                loopCount = 1; // 2
                stream->maxNumChannels = 2;
        }
        else {
                loopCount  = 4; // 2 4 6 8
                stream->maxNumChannels = 8;
        }
        while(loopCount > 0) {
                for(int i=0; i<ARRAY_SIZE(validRates); i++) {
                        if(validRatesbits[i] & filter)
                                continue;
                        memcpy(&format, &oxygen_hardware[channel], sizeof(IOAudioStreamFormat));
                        
                        format.fNumChannels = 2*loopCount;
                        
                        //ALSA wiki says (https://www.alsa-project.org/wiki/FramesPeriods):
                        //> > The "frame" represents the unit, 1 frame = # channels x sample_bytes.
                        //> > In your case, 1 frame corresponds to 2 channels x 16 bits = 4 bytes.
                        //> >
                        //> > The periods is the number of periods in a ring-buffer.  In OSS, called
                        //> > as "fragments".
                        //> > So,
                        //> >  - buffer_size = period_size * periods
                        //> >  - period_bytes = period_size * bytes_per_frame
                        //> >  - bytes_per_frame = channels * bytes_per_sample
                        
                        //the best we can do to mimic the period and buffer byte constraints is to set
                        //the stream buffer to 32 bytes.
                        /* err = snd_pcm_hw_constraint_step(runtime, 0,
                         SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);
                         if (err < 0)
                         return err; */
                        
                        //SNDRV_PCM_HW_PARAM_PERIOD_BYTES = # of bytes between interrupts
                        //(https://elixir.bootlin.com/linux/latest/source/include/uapi/sound/asound.h#L384)
                        //so 32 bytes in a period
                        formatEx.fBytesPerPacket=32;
                        
                        /* err = snd_pcm_hw_constraint_step(runtime, 0,
                         SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
                         if (err < 0)
                         return err;
                         //SNDRV_PCM_HW_PARAM_BUFFER_BYTES = size of buffer in bytes
                         */
                        //if there's 32 bytes per packet then 1 frame must be 32 bytes since we
                        //can carry 8 channels each with 4 bytes (???)
                        formatEx.fFramesPerPacket=1;
                        
                        // ^ weird thing is clemens sets PERIOD_BYTES_MIN to 64:
                        //   https://elixir.bootlin.com/linux/v4.14.298/source/sound/pci/oxygen/oxygen_pcm.c#L35
                        //   so the formatEx parameters, will very likely need to be tweaked
                        
                        //It seems by setting the maxNumChannels in IOAudioStream, we are okay because
                        //the IOAudioStreamFormat is still 2, so the below should be satisfied.
#if DEBUG && (DEBUGLEVEL > 2)
                        
                        kprintf("XonarAudioEngine::%-30s adding format with the following attributes:\n"
                                "XonarAudioEngine::%-30s sampleRate->whole: %d\t sampleRate->fraction: %d\n"
                                "XonarAudioEngine::%-30s numChannels:%d\t sampleFormat:%s\t NumericRepresentation:%s\n"
                                "XonarAudioEngine::%-30s bitDepth:(%d, 24)\t bitWidth:(%d, 32)\t streamAlignment:%s\n"
                                "XonarAudioEngine::%-30s streamByteOrder:%s, mixable:%s, driverTag:%d\n"
                                "XonarAudioEngine::%-30s \n",
                                " ",
                                " ", validRates[i].whole, validRates[i].fraction,
                                " ", format.fNumChannels, formatSampleFormat(format.fSampleFormat), formatRep(format.fNumericRepresentation),
                                " ", format.fBitDepth, format.fBitWidth, formatAlignment(format.fAlignment),
                                " ", formatByteOrder(format.fByteOrder), format.fIsMixable ? "true" : "false", format.fDriverTag,
                                " ");
#endif
                        stream->addAvailableFormat(&format, &formatEx, &validRates[i], &validRates[i]); //runtime->hw = *oxygen_hardware[channel];
                        //i think we have to add the format twice, once as 16 bit and the second time as 32 bit.
                        //msbits constraint says 24 msbits for 32 bit format, so use formatEx struct.
                        
                        
                        //set fBitDepth=24 and fBitWidth=32 for 32 bit formats to mimic code below
                        //(Assuming that's VoodooHDA is doing here:
                        // https://sourceforge.net/p/voodoohda/code/HEAD/tree/tranc/VoodooHDAEngine.cpp#l527)
                        // ^ copy the struct, don't change the original.
                        format.fBitDepth = 24;
                        format.fBitWidth = 32;
                        /*if(runtime->hw.formats & SNDRV_PCM_FMTBIT_S32_LE) {
                         err = snd_pcm_hw_constraint_msbits(runtime, 0, 32, 24);
                         if (err < 0)
                         return err;
                         }*/
                        
                        
                        
                        /*
                         if (stream->maxNumChannels > 2) {
                         err = snd_pcm_hw_constraint_step(runtime, 0,
                         SNDRV_PCM_HW_PARAM_CHANNELS,
                         2);
                         if (err < 0)
                         return err;
                         }
                         */
                        
                        stream->addAvailableFormat(&format, &formatEx, &validRates[i], &validRates[i]); //runtime->hw = *oxygen_hardware[channel];
                }
                loopCount--;
        }
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        
}
int XonarAudioEngine::oxygen_open(IOAudioStream *stream,
                       unsigned int channel)
{
        //struct oxygen *chip = ((XonarAudioEngine *) stream->audioEngine)->chipData;//= snd_pcm_substream_chip(substream);
        //struct snd_pcm_runtime *runtime = substream->runtime;
        int err;
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#if DEBUGLEVEL > 2
        kprintf("XonarAudioEngine::%-30s adding formats+streams for channel %d\n", " ", channel);
#endif
#endif
        //runtime->private_data = (void *)(uintptr_t)channel;
        if (channel == PCM_B && chipData->has_ac97_1 &&
            (chipData->model.device_config & CAPTURE_2_FROM_AC97_1))
                stream->addAvailableFormat(&ac97_format, &ac97_sampleRate, &ac97_sampleRate);  // runtime->hw =  oxygen_ac97_hardware;
        else
                filterStreams(stream, channel, 0);
        switch (channel) {
                case PCM_C:
                        if (chipData->model.device_config & CAPTURE_1_FROM_SPDIF) {
                                filterStreams(stream, channel, (SNDRV_PCM_RATE_32000
                                                                | SNDRV_PCM_RATE_64000));
                                //                    runtime->hw.rates &= ~(SNDRV_PCM_RATE_32000 |
                                //                                           SNDRV_PCM_RATE_64000);
                                //                    runtime->hw.rate_min = 44100;
                        }
                        
                        /* fall through */
                case PCM_A:
                case PCM_B:
                        //runtime->hw.fifo_size = 0;
                        break;
                case PCM_MULTICH:
                        stream->maxNumChannels = chipData->model.dac_channels_pcm;
                        break;
        }
        if (chipData->model.pcm_hardware_filter) {
                chipData->model.pcm_hardware_filter(channel, stream);
        }
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s formats added without issue.\n", " ");
#endif
        // dunno WTF this call translates to for IOAUdio
        // snd_pcm_set_sync(substream);
        // chip->streams[channel] = stream;
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s calling IOLock and checking spdif bits\n", " ");
#endif
        IOLockLock(chipData->mutex);
        chipData->pcm_active |= 1 << channel;
        if (channel == PCM_SPDIF) {
                chipData->spdif_pcm_bits = chipData->spdif_bits;
                //                chip->controls[CONTROL_SPDIF_PCM]->vd[0].access &=
                //                        ~SNDRV_CTL_ELEM_ACCESS_INACTIVE;
                //                snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_VALUE |
                //                               SNDRV_CTL_EVENT_MASK_INFO,
                //                               &chip->controls[CONTROL_SPDIF_PCM]->id);
        }
        IOLockUnlock(chipData->mutex);
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        return 0;
}


unsigned int XonarAudioEngine::oxygen_format(const IOAudioStreamFormat *format)
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s bitWidth %d, byteOrder %d\n", __func__,
                format->fBitWidth, format->fByteOrder);
#endif
        //srsly hope this is right -_-
        if (format->fBitWidth == 32 &&
            format->fByteOrder == kIOAudioStreamByteOrderLittleEndian
            /*params_format(hw_params) == SNDRV_PCM_FORMAT_S32_LE*/)
                return OXYGEN_FORMAT_24;
        else
                return OXYGEN_FORMAT_16;
}

unsigned int  XonarAudioEngine::oxygen_rate(const IOAudioSampleRate *sampleRate)
{
        switch (sampleRate->whole) {
                case 32000:
                        return OXYGEN_RATE_32000;
                case 44100:
                        return OXYGEN_RATE_44100;
                default: /* 48000 */
                        return OXYGEN_RATE_48000;
                case 64000:
                        return OXYGEN_RATE_64000;
                case 88200:
                        return OXYGEN_RATE_88200;
                case 96000:
                        return OXYGEN_RATE_96000;
                case 176400:
                        return OXYGEN_RATE_176400;
                case 192000:
                        return OXYGEN_RATE_192000;
        }
}

static unsigned int oxygen_i2s_bits(const IOAudioStreamFormat *format)
{
        if (format->fBitWidth == 32 &&
            format->fByteOrder == kIOAudioStreamByteOrderLittleEndian
            /*params_format(hw_params) == SNDRV_PCM_FORMAT_S32_LE*/ )
                return OXYGEN_I2S_BITS_24;
        else
                return OXYGEN_I2S_BITS_16;
}

static unsigned int oxygen_play_channels(const IOAudioStreamFormat *format)
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s format->fNumChannels %d\n", __func__, format->fNumChannels);
#endif
        //man i hope i'm doing this right
        switch (format->fNumChannels) {
                default: /* 2 */
                        return OXYGEN_PLAY_CHANNELS_2;
                case 4:
                        return OXYGEN_PLAY_CHANNELS_4;
                case 6:
                        return OXYGEN_PLAY_CHANNELS_6;
                case 8:
                        return OXYGEN_PLAY_CHANNELS_8;
        }
}


void XonarAudioEngine::xonar_enable_output(struct oxygen *chip)
{
        struct xonar_generic *data =(struct xonar_generic*) chip->model_data;
        
        oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, data->output_enable_bit);
        IODelay(data->anti_pop_delay*1000);
        oxygen_set_bits16(chip, OXYGEN_GPIO_DATA, data->output_enable_bit);
}

void XonarAudioEngine::xonar_disable_output(struct oxygen *chip)
{
        struct xonar_generic *data = (struct xonar_generic*)  chip->model_data;
        
        oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, data->output_enable_bit);
}

static void xonar_ext_power_gpio_changed(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        struct xonar_generic *data = (struct xonar_generic*) chip->model_data;
        UInt8 has_power;
        
        has_power = !!(oxygen_read8(chip, data->ext_power_reg)
                       & data->ext_power_bit);
        if (has_power != data->has_power) {
                data->has_power = has_power;
                if (has_power) {
                        kprintf("power restored\n");
                } else {
                        kprintf("Hey! Don't unplug the power cable!\n");
                        /* TODO: stop PCMs */
                }
        }
}

void XonarAudioEngine::xonar_init_ext_power(struct oxygen *chip)
{
        struct xonar_generic *data = (struct xonar_generic*) chip->model_data;
        
        oxygen_set_bits8(chip, data->ext_power_int_reg,
                         data->ext_power_bit);
        chip->interrupt_mask |= OXYGEN_INT_GPIO;
        chip->model.gpio_changed = xonar_ext_power_gpio_changed;
        data->has_power = !!(oxygen_read8(chip, data->ext_power_reg)
                             & data->ext_power_bit);
}

void XonarAudioEngine::xonar_init_cs53x1(struct oxygen *chip)
{
        oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_CS53x1_M_MASK);
        oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                              GPIO_CS53x1_M_SINGLE, GPIO_CS53x1_M_MASK);
}

void XonarAudioEngine::xonar_set_cs53x1_params(struct oxygen *chip, XonarAudioEngine *audioEngine)
{
        unsigned int value;
        
        if (audioEngine->getSampleRate()->whole <= 54000)
                value = GPIO_CS53x1_M_SINGLE;
        else if (audioEngine->getSampleRate()->whole <= 108000)
                value = GPIO_CS53x1_M_DOUBLE;
        else
                value = GPIO_CS53x1_M_QUAD;
        oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                              value, GPIO_CS53x1_M_MASK);
}


int XonarAudioEngine::xonar_gpio_bit_switch_get(IOAudioControl *volumeControl, XonarAudioEngine *engine,
                                                int oldValue, int newValue, int private_value)
{
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        UInt16 bit = private_value;
        bool invert = private_value & XONAR_GPIO_BIT_INVERT;
        
        newValue =
        !!(oxygen_read16(chip, OXYGEN_GPIO_DATA) & bit) ^ invert;
        return 0;
}


int XonarAudioEngine::xonar_gpio_bit_switch_put(IOAudioControl *volumeControl, XonarAudioEngine *engine,
                                                int oldValue, int newValue, int private_value)
{
        struct oxygen *chip = (struct oxygen*) engine->chipData;
        UInt16 bit = private_value;
        bool invert = private_value & XONAR_GPIO_BIT_INVERT;
        UInt16 old_bits, new_bits;
        int changed;
        
        IOSimpleLockLock(chip->reg_lock);
        old_bits = oxygen_read16(chip, OXYGEN_GPIO_DATA);
        if (!!newValue ^ invert)
                new_bits = old_bits | bit;
        else
                new_bits = old_bits & ~bit;
        changed = new_bits != old_bits;
        if (changed)
                oxygen_write16(chip, OXYGEN_GPIO_DATA, new_bits);
        IOSimpleLockUnlock(chip->reg_lock);
        //return changed;
        return kIOReturnSuccess;
}


static inline int oxygen_uart_input_ready(struct oxygen *chip)
{
        return !(oxygen_read8(chip, OXYGEN_MPU401 + 1) & MPU401_RX_EMPTY);
}

static void _write_uart(struct oxygen *chip, unsigned int port, UInt8 data)
{
        if (oxygen_read8(chip, OXYGEN_MPU401 + 1) & MPU401_TX_FULL)
                IODelay(1e3);
        oxygen_write8(chip, OXYGEN_MPU401 + port, data);
}

void XonarAudioEngine::oxygen_reset_uart(struct oxygen *chip)
{
        _write_uart(chip, 1, MPU401_RESET);
        IODelay(1e3); /* wait for ACK */
        _write_uart(chip, 1, MPU401_ENTER_UART);
}
//EXPORT_SYMBOL(oxygen_reset_uart);

void XonarAudioEngine::oxygen_write_uart(struct oxygen *chip, UInt8 data)
{
        _write_uart(chip, 0, data);
}
//EXPORT_SYMBOL(oxygen_write_uart);


void XonarAudioEngine::oxygen_read_uart(struct oxygen *chip)
{
        if (unlikely(!oxygen_uart_input_ready(chip))) {
                // no data, but read it anyway to clear the interrupt
                oxygen_read8(chip, OXYGEN_MPU401);
                return;
        }
        do {
                UInt8 data = oxygen_read8(chip, OXYGEN_MPU401);
                if (data == MPU401_ACK)
                        continue;
                if (chip->uart_input_count >= ARRAY_SIZE(chip->uart_input))
                        chip->uart_input_count = 0;
                chip->uart_input[chip->uart_input_count++] = data;
        } while (oxygen_uart_input_ready(chip));
        if (chip->model.uart_input)
                chip->model.uart_input(chip);
}



void XonarAudioEngine::cs2000_write(struct oxygen *chip, UInt8 reg, UInt8 value)
{
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        
        oxygen_write_i2c(chip, I2C_DEVICE_CS2000, reg, value);
        data->cs2000_regs[reg] = value;
}

void XonarAudioEngine::cs2000_write_cached(struct oxygen *chip, UInt8 reg, UInt8 value)
{
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        
        if (value != data->cs2000_regs[reg])
                cs2000_write(chip, reg, value);
}

void XonarAudioEngine::cs2000_registers_init(struct oxygen *chip)
{
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        
        cs2000_write(chip, CS2000_GLOBAL_CFG, CS2000_FREEZE);
        cs2000_write(chip, CS2000_DEV_CTRL, 0);
        cs2000_write(chip, CS2000_DEV_CFG_1,
                     CS2000_R_MOD_SEL_1 |
                     (0 << CS2000_R_SEL_SHIFT) |
                     CS2000_AUX_OUT_SRC_REF_CLK |
                     CS2000_EN_DEV_CFG_1);
        cs2000_write(chip, CS2000_DEV_CFG_2,
                     (0 << CS2000_LOCK_CLK_SHIFT) |
                     CS2000_FRAC_N_SRC_STATIC);
        cs2000_write(chip, CS2000_RATIO_0 + 0, 0x00); /* 1.0 */
        cs2000_write(chip, CS2000_RATIO_0 + 1, 0x10);
        cs2000_write(chip, CS2000_RATIO_0 + 2, 0x00);
        cs2000_write(chip, CS2000_RATIO_0 + 3, 0x00);
        cs2000_write(chip, CS2000_FUN_CFG_1,
                     data->cs2000_regs[CS2000_FUN_CFG_1]);
        cs2000_write(chip, CS2000_FUN_CFG_2, 0);
        cs2000_write(chip, CS2000_GLOBAL_CFG, CS2000_EN_DEV_CFG_2);
        IODelay(3*1000); /* PLL lock delay */
}


inline void XonarAudioEngine::pcm1796_write_spi(struct oxygen *chip, unsigned int codec,
                                                UInt8 reg, UInt8 value)
{
        /* maps ALSA channel pair number to SPI output */
        static const UInt8 codec_map[4] = {
                0, 1, 2, 4
        };
        
        oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER  |
                         OXYGEN_SPI_DATA_LENGTH_2 |
                         OXYGEN_SPI_CLOCK_160 |
                         (codec_map[codec] << OXYGEN_SPI_CODEC_SHIFT) |
                         OXYGEN_SPI_CEN_LATCH_CLOCK_HI,
                         (reg << 8) | value);
}




inline void XonarAudioEngine::pcm1796_write_i2c(struct oxygen *chip, unsigned int codec,
                                                UInt8 reg, UInt8 value)
{
        oxygen_write_i2c(chip, I2C_DEVICE_PCM1796(codec), reg, value);
}

void XonarAudioEngine::pcm1796_write(struct oxygen *chip, unsigned int codec,
                                     UInt8 reg, UInt8 value)
{
        struct xonar_pcm179x *data;
        if(chip->card_model == MODEL_HDAV)
                data = (struct xonar_pcm179x*) &((struct xonar_hdav*) chip->model_data)->pcm179x;
        else
                data = (struct xonar_pcm179x*) chip->model_data;
        
        if ((chip->model.function_flags & OXYGEN_FUNCTION_2WIRE_SPI_MASK) ==
            OXYGEN_FUNCTION_SPI) {
                pcm1796_write_spi(chip, codec, reg, value);
        }
        else
                pcm1796_write_i2c(chip, codec, reg, value);
        
        if ((unsigned int)(reg - PCM1796_REG_BASE)
            < ARRAY_SIZE(data->pcm1796_regs[codec])) {
                data->pcm1796_regs[codec][reg - PCM1796_REG_BASE] = value;
        }
}

void XonarAudioEngine::pcm1796_write_cached(struct oxygen *chip, unsigned int codec,
                                            UInt8 reg, UInt8 value)
{
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        
        if (value != data->pcm1796_regs[codec][reg - PCM1796_REG_BASE])
                pcm1796_write(chip, codec, reg, value);
}

void XonarAudioEngine::pcm1796_registers_init(struct oxygen *chip)
{
        struct xonar_pcm179x *data;
        if(chip->card_model == MODEL_HDAV)
                data = (struct xonar_pcm179x*) &((struct xonar_hdav*) chip->model_data)->pcm179x;
        else
                data = (struct xonar_pcm179x*) chip->model_data;
        
        unsigned int i;
        SInt8 gain_offset;
        
        IODelay(1000);
        gain_offset = data->hp_active ? data->hp_gain_offset : 0;
        for (i = 0; i < data->dacs; ++i) {
                /* set ATLD before ATL/ATR */
                pcm1796_write(chip, i, 18,
                              data->pcm1796_regs[0][18 - PCM1796_REG_BASE]);
                pcm1796_write(chip, i, 16, chip->dac_volume[i * 2]
                              + gain_offset);
                pcm1796_write(chip, i, 17, chip->dac_volume[i * 2 + 1]
                              + gain_offset);
                pcm1796_write(chip, i, 19,
                              data->pcm1796_regs[0][19 - PCM1796_REG_BASE]);
                pcm1796_write(chip, i, 20,
                              data->pcm1796_regs[0][20 - PCM1796_REG_BASE]);
                pcm1796_write(chip, i, 21, 0);
                gain_offset = 0;
        }
}

void XonarAudioEngine::pcm1796_init(struct oxygen *chip)
{
        struct xonar_pcm179x *data;
        if(chip->card_model == MODEL_HDAV)
                data = (struct xonar_pcm179x*) &((struct xonar_hdav*) chip->model_data)->pcm179x;
        else
                data = (struct xonar_pcm179x*) chip->model_data;
        
        data->pcm1796_regs[0][18 - PCM1796_REG_BASE] =
        PCM1796_DMF_DISABLED | PCM1796_FMT_24_I2S | PCM1796_ATLD;
        if (!data->broken_i2c)
                data->pcm1796_regs[0][18 - PCM1796_REG_BASE] |= PCM1796_MUTE;
        data->pcm1796_regs[0][19 - PCM1796_REG_BASE] =
        PCM1796_FLT_SHARP | PCM1796_ATS_1;
        data->pcm1796_regs[0][20 - PCM1796_REG_BASE] =
        data->h6 ? PCM1796_OS_64 : PCM1796_OS_128;
        pcm1796_registers_init(chip);
        data->current_rate.whole = 48000;
}

void XonarAudioEngine::update_pcm1796_oversampling(struct oxygen *chip)
{
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        unsigned int i;
        UInt8 reg;
        
        if (data->current_rate.whole <= 48000 && !data->h6)
                reg = PCM1796_OS_128;
        else
                reg = PCM1796_OS_64;
        for (i = 0; i < data->dacs; ++i)
                pcm1796_write_cached(chip, i, 20, reg);
}

void XonarAudioEngine::set_pcm1796_params(struct oxygen *chip, XonarAudioEngine *instance,
                                          IOAudioStream *currentStream)
{
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        
        IODelay(1*1000);
        //hopefully OSX getSampleRate() for IOAudioEngine objects has
        // similar behaviour to linux's params_rate...
        data->current_rate.whole = instance->getSampleRate()->whole;
        data->current_rate.fraction = instance->getSampleRate()->fraction;
        // data->current_rate = params_rate(params);
        update_pcm1796_oversampling(chip);
}

void XonarAudioEngine::update_pcm1796_volume(struct oxygen *chip)
{
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        unsigned int i;
        SInt8 gain_offset;
        
        gain_offset = data->hp_active ? data->hp_gain_offset : 0;
        for (i = 0; i < data->dacs; ++i) {
                pcm1796_write_cached(chip, i, 16, chip->dac_volume[i * 2]
                                     + gain_offset);
                pcm1796_write_cached(chip, i, 17, chip->dac_volume[i * 2 + 1]
                                     + gain_offset);
                gain_offset = 0;
        }
}

void XonarAudioEngine::update_pcm1796_mute(struct oxygen *chip)
{
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        unsigned int i;
        UInt8 value;
        
        value = PCM1796_DMF_DISABLED | PCM1796_FMT_24_I2S | PCM1796_ATLD;
        if (chip->dac_mute)
                value |= PCM1796_MUTE;
        for (i = 0; i < data->dacs; ++i)
                pcm1796_write_cached(chip, i, 18, value);
}


void XonarAudioEngine::update_cs2000_rate(struct oxygen *chip, unsigned int rate)
{
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        UInt8 rate_mclk, reg;
        
        switch (rate) {
                case 32000:
                case 64000:
                        rate_mclk = OXYGEN_RATE_32000;
                        break;
                case 44100:
                case 88200:
                case 176400:
                        rate_mclk = OXYGEN_RATE_44100;
                        break;
                default:
                case 48000:
                case 96000:
                case 192000:
                        rate_mclk = OXYGEN_RATE_48000;
                        break;
        }
        
        if (rate <= 96000 && (rate > 48000 || data->h6)) {
                rate_mclk |= OXYGEN_I2S_MCLK(MCLK_256);
                reg = CS2000_REF_CLK_DIV_1;
        } else {
                rate_mclk |= OXYGEN_I2S_MCLK(MCLK_512);
                reg = CS2000_REF_CLK_DIV_2;
        }
        
        oxygen_write16_masked(chip, OXYGEN_I2S_A_FORMAT, rate_mclk,
                              OXYGEN_I2S_RATE_MASK | OXYGEN_I2S_MCLK_MASK);
        cs2000_write_cached(chip, CS2000_FUN_CFG_1, reg);
        IODelay(3*1000); /* PLL lock delay */
}

/*
 static const struct snd_kcontrol_new alt_switch = {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Analog Loopback Switch",
 .info = snd_ctl_boolean_mono_info,
 .get = XonarAudioEngine::xonar_gpio_bit_switch_get,
 .put = XonarAudioEngine::xonar_gpio_bit_switch_put,
 .private_value = GPIO_D2_ALT,
 };
 */
static int rolloff_info(struct snd_kcontrol *ctl,
                        struct snd_ctl_elem_info *info)
{
        static const char *const names[2] = {
                "Sharp Roll-off", "Slow Roll-off"
        };
        
        return snd_ctl_enum_info(info, 1, 2, names);
}

static int rolloff_get(struct snd_kcontrol *ctl,
                       struct snd_ctl_elem_value *value)
{
        struct oxygen *chip = (struct oxygen*) ctl->private_data;
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        
        value->value.enumerated.item[0] =
        (data->pcm1796_regs[0][19 - PCM1796_REG_BASE] &
         PCM1796_FLT_MASK) != PCM1796_FLT_SHARP;
        return 0;
}

int XonarAudioEngine::rolloff_put(IOAudioControl *SelectorControl, XonarAudioEngine *engine,
                                  const void *oldData, UInt32 oldDataSize, const void* newData, UInt32 newDataSize)
{
        struct oxygen *chip = engine->chipData;
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        char *newString = (char *) &newData;
        unsigned int i;
        int changed;
        UInt8 reg;
        
        IOLockLock(chip->mutex);
        reg = data->pcm1796_regs[0][19 - PCM1796_REG_BASE];
        reg &= ~PCM1796_FLT_MASK;
        if (strcmp(newString, "Sharp Roll-off"))
                reg |= PCM1796_FLT_SHARP;
        else
                reg |= PCM1796_FLT_SLOW;
        changed = reg != data->pcm1796_regs[0][19 - PCM1796_REG_BASE];
        if (changed) {
                for (i = 0; i < data->dacs; ++i)
                        pcm1796_write(chip, i, 19, reg);
        }
        IOLockUnlock(chip->mutex);
        //return changed;
        return kIOReturnSuccess;
        
}

//static const struct snd_kcontrol_new rolloff_control = {
//        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
//        .name = "DAC Filter Playback Enum",
//        .info = rolloff_info,
//        .get = rolloff_get,
//        .put = XonarAudioEngine::rolloff_put,
//};

/*
 static inline unsigned long msecs_to_jiffies(const unsigned int m)
 {
 return (m + (MSEC_PER_SEC / HZ) - 1) / (MSEC_PER_SEC / HZ);
 }
 */

static int oxygen_ac97_wait(struct oxygen *chip, unsigned int mask)
{
        UInt8 status = 0;
        wait_result_t retval;
        UInt32 condition;
        /*
         * Reading the status register also clears the bits, so we have to save
         * the read bits in status.
         */
        IOLockLock(chip->ac97_mutex);
        //while() {
        retval = IOLockSleepDeadline(chip->ac97_mutex, &(condition = ({status |= oxygen_read8(chip, OXYGEN_AC97_INTERRUPT_STATUS);
                status & mask;})), 1e8, THREAD_UNINT);
        //                        if(!retval)
        //                                kprintf("ac97_thread was awakened!\n");
        //                        else if(retval == 1)
        //                                kprintf("ac97_thread timed out\n");
        //                        else if (retval == -1)
        //                                kprintf ("ac97_thread waiting...\n");
        
        //}
        IOLockUnlock(chip->ac97_mutex);
        /*
         * Check even after a timeout because this function should not require
         * the AC'97 interrupt to be enabled.
         */
        status |= oxygen_read8(chip, OXYGEN_AC97_INTERRUPT_STATUS);
        return status & mask ? 0 : -EIO;
}


static inline void oxygen_write_ac97(struct oxygen *chip, unsigned int codec,
                                     unsigned int index, UInt16 data)
{
        unsigned int count, succeeded;
        UInt32 reg;
        
        reg = data;
        reg |= index << OXYGEN_AC97_REG_ADDR_SHIFT;
        reg |= OXYGEN_AC97_REG_DIR_WRITE;
        reg |= codec << OXYGEN_AC97_REG_CODEC_SHIFT;
        succeeded = 0;
        for (count = 5; count > 0; --count) {
                IODelay(5);
                oxygen_write32(chip, OXYGEN_AC97_REGS, reg);
                /* require two "completed" writes, just to be sure */
                if (oxygen_ac97_wait(chip, OXYGEN_AC97_INT_WRITE_DONE) >= 0 &&
                    ++succeeded >= 2) {
                        chip->saved_ac97_registers[codec][index / 2] = data;
                        return;
                }
        }
        kprintf("AC'97 write timeout\n");
}

UInt16 oxygen_read_ac97(struct oxygen *chip, unsigned int codec,
                        unsigned int index)
{
        unsigned int count;
        unsigned int last_read = UINT_MAX;
        UInt32 reg;
        
        reg = index << OXYGEN_AC97_REG_ADDR_SHIFT;
        reg |= OXYGEN_AC97_REG_DIR_READ;
        reg |= codec << OXYGEN_AC97_REG_CODEC_SHIFT;
        for (count = 5; count > 0; --count) {
                IODelay(5);
                oxygen_write32(chip, OXYGEN_AC97_REGS, reg);
                IODelay(10);
                if (oxygen_ac97_wait(chip, OXYGEN_AC97_INT_READ_DONE) >= 0) {
                        UInt16 value = oxygen_read16(chip, OXYGEN_AC97_REGS);
                        /* we require two consecutive reads of the same value */
                        if (value == last_read)
                                return value;
                        last_read = value;
                        /*
                         * Invert the register value bits to make sure that two
                         * consecutive unsuccessful reads do not return the same
                         * value.
                         */
                        reg ^= 0xffff;
                }
        }
        kprintf("AC'97 read timeout on codec %u\n", codec);
        return 0;
}

void oxygen_write_ac97_masked(struct oxygen *chip, unsigned int codec,
                              unsigned int index, UInt16 data, UInt16 mask)
{
        UInt16 value = oxygen_read_ac97(chip, codec, index);
        value &= ~mask;
        value |= data & mask;
        oxygen_write_ac97(chip, codec, index, value);
}



static const UInt32 ac97_registers_to_restore[2][0x40 / 32] = {
        { 0x18284fa2, 0x03060000 },
        { 0x00007fa6, 0x00200000 }
};

void XonarAudioEngine::oxygen_restore_ac97(struct oxygen *chip, unsigned int codec)
{
        unsigned int i;
        
        oxygen_write_ac97(chip, codec, AC97_RESET, 0);
        IOSleep(1);
        for (i = 1; i < 0x40; ++i)
                if (is_bit_set(ac97_registers_to_restore[codec], i))
                        oxygen_write_ac97(chip, codec, i * 2,
                                          chip->saved_ac97_registers[codec][i]);
}

void XonarAudioEngine::xonar_line_mic_ac97_switch(struct oxygen *chip,
                                                  unsigned int reg, unsigned int mute)
{
        if (reg == AC97_LINE) {
                IOSimpleLockLock(chip->reg_lock);
                oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                                      mute ? GPIO_INPUT_ROUTE : 0,
                                      GPIO_INPUT_ROUTE);
                IOSimpleLockUnlock(chip->reg_lock);
        }
}


int XonarAudioEngine::add_pcm1796_controls(struct oxygen *chip, PCIAudioDevice *dev)
{
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        int err;
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#endif
        if (!data->broken_i2c) {
                
                IOAudioPort *PCM1796Port;
                IOAudioSelectorControl *RollOffSelector;
                PCM1796Port = IOAudioPort::withAttributes(kIOAudioPortTypeProcessing, "PCM1796");
                
                kprintf("XonarAudioEngine::%-30s creating DAC Filter Selector\n", " ");
                RollOffSelector = IOAudioSelectorControl::create(false,    // initial state - unmuted
                                                                 kIOAudioControlChannelIDAll,    // Affects all channels
                                                                 kIOAudioControlChannelNameAll,
                                                                 0,        // control ID - driver-defined
                                                                 kIOAudioControlUsageOutput);
                if (!RollOffSelector) {
                        goto Done;
                }
                
                RollOffSelector->setValueChangeHandler((IOAudioControl::DataValueChangeHandler) dev->RollOffSelectHandler,
                                                       dev);
                RollOffSelector->setName("DAC Filter Playback Enum");
                PCM1796Port->addAudioControl(RollOffSelector);
                dev->attachAudioPort(PCM1796Port, this, NULL);
                RollOffSelector->release();
                PCM1796Port->release();
                
                
        }
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        return kIOReturnSuccess;
Done:
        return kIOReturnError;
}


void XonarAudioEngine::hdmi_write_command(struct oxygen *chip, UInt8 command,
                                          unsigned int count, const UInt8 *params)
{
        unsigned int i;
        UInt8 checksum;
        
        oxygen_write_uart(chip, 0xfb);
        oxygen_write_uart(chip, 0xef);
        oxygen_write_uart(chip, command);
        oxygen_write_uart(chip, count);
        for (i = 0; i < count; ++i)
                oxygen_write_uart(chip, params[i]);
        checksum = 0xfb + 0xef + command + count;
        for (i = 0; i < count; ++i)
                checksum += params[i];
        oxygen_write_uart(chip, checksum);
}

void XonarAudioEngine::xonar_hdmi_init_commands(struct oxygen *chip,
                                                struct xonar_hdmi *hdmi)
{
        UInt8 param;
        
        oxygen_reset_uart(chip);
        param = 0;
        hdmi_write_command(chip, 0x61, 1, &param);
        param = 1;
        hdmi_write_command(chip, 0x74, 1, &param);
        hdmi_write_command(chip, 0x54, 5, hdmi->params);
}


void XonarAudioEngine::xonar_hdmi_init(struct oxygen *chip, struct xonar_hdmi *hdmi)
{
        hdmi->params[1] = IEC958_AES3_CON_FS_48000;
        hdmi->params[4] = 1;
        xonar_hdmi_init_commands(chip, hdmi);
}

void XonarAudioEngine::xonar_hdmi_cleanup(struct oxygen *chip)
{
        UInt8 param = 0;
        
        hdmi_write_command(chip, 0x74, 1, &param);
}

void XonarAudioEngine::xonar_hdmi_resume(struct oxygen *chip, struct xonar_hdmi *hdmi)
{
        xonar_hdmi_init_commands(chip, hdmi);
}

void XonarAudioEngine::xonar_set_hdmi_params(struct oxygen *chip, struct xonar_hdmi *hdmi,
                                             IOAudioStream *currentStream)
{
        hdmi->params[0] = 0; // 1 = non-audio
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#endif
        switch (getSampleRate()->whole) {
                case 44100:
                        hdmi->params[1] = IEC958_AES3_CON_FS_44100;
                        break;
                case 48000:
                        hdmi->params[1] = IEC958_AES3_CON_FS_48000;
                        break;
                default: // 96000
                        hdmi->params[1] = IEC958_AES3_CON_FS_96000;
                        break;
                case 192000:
                        hdmi->params[1] = IEC958_AES3_CON_FS_192000;
                        break;
        }
        //Linux call:
        //hdmi->params[2] = params_channels(params) / 2 - 1;
        //Mac Call:
        hdmi->params[2] = currentStream->getFormat()->fNumChannels / 2 - 1;
        //^ this is wrong because it should be NumChannels, not MaxNum
        //however since IOAudioStream calls are deprecated as of 10.10,
        //i'm going to use this is a placeholder/semi-correct call.
        
        //Linux call:
        //if (params_format(params) == SNDRV_PCM_FORMAT_S16_LE)
        //Mac Call:
        if(currentStream->getFormat()->fSampleFormat == kIOAudioStreamSampleFormatLinearPCM
           && currentStream->getFormat()->fBitWidth == 16
           && currentStream->getFormat()->fByteOrder == kIOAudioStreamByteOrderLittleEndian)
                hdmi->params[3] = 0;
        else
                hdmi->params[3] = 0xc0;
        hdmi->params[4] = 1; // ?
        hdmi_write_command(chip, 0x54, 5, hdmi->params);
#if DEBUG
        kprintf("XonarAUdioEngine::%-30s END\n", __func__);
#endif
}


void XonarAudioEngine::xonar_hdmi_uart_input(struct oxygen *chip)
{
        if (chip->uart_input_count >= 2 &&
            chip->uart_input[chip->uart_input_count - 2] == 'O' &&
            chip->uart_input[chip->uart_input_count - 1] == 'K') {
                kprintf("message from HDMI chip received:\n");
                print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
                                     chip->uart_input, chip->uart_input_count);
                chip->uart_input_count = 0;
        }
}


void XonarAudioEngine::xonar_hdmi_pcm_hardware_filter(unsigned int channel,
                                                      IOAudioStream *stream)
{
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#endif
        if (channel == PCM_MULTICH) {
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s hardware filter for PCM_MULTICH, clearing & re-adding streams!\n"
                        "XonarAudioEngine::%-30s ",
                        " ",
                        " ");
#endif
                
                //                hardware->rates = SNDRV_PCM_RATE_44100 |
                //                                  SNDRV_PCM_RATE_48000 |
                //                                  SNDRV_PCM_RATE_96000 |
                //                                  SNDRV_PCM_RATE_192000;
                //                hardware->rate_min = 44100;
                ((XonarAudioEngine*)stream->audioEngine)->filterStreams(stream, channel, (SNDRV_PCM_RATE_32000 |
                                                SNDRV_PCM_RATE_64000 |
                                                SNDRV_PCM_RATE_88200 |
                                                SNDRV_PCM_RATE_176400 ));
        }
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif

}

void XonarAudioEngine::wm8776_adc_hardware_filter(unsigned int channel,
                                                  IOAudioStream *stream)
{
        if (channel == PCM_A) {
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s hardware filter for PCM_A, clearing & re-adding streams!\n",
                        " ");
#endif
                //         hardware->rates = SNDRV_PCM_RATE_32000 |
                //         SNDRV_PCM_RATE_44100 |
                //         SNDRV_PCM_RATE_48000 |
                //         SNDRV_PCM_RATE_64000 |
                //         SNDRV_PCM_RATE_88200 |
                //         SNDRV_PCM_RATE_96000;
                // hardware->rate_max = 96000;
                ((XonarAudioEngine*)stream->audioEngine)->filterStreams(stream, channel, (SNDRV_PCM_RATE_176400 |
                                                                                            SNDRV_PCM_RATE_192000));
        }
}

void XonarAudioEngine::xonar_hdav_slim_hardware_filter(unsigned int channel,
                                                       IOAudioStream *stream)
{
        wm8776_adc_hardware_filter(channel, stream);
        ((XonarAudioEngine*)stream->audioEngine)->xonar_hdmi_pcm_hardware_filter(channel, stream);
}

static int xonar_d2_control_filter(struct snd_kcontrol_new *_template)
{
        if (!strncmp(_template->name, "CD Capture ", 11))
                // CD in is actually connected to the video in pin
                _template->private_value ^= AC97_CD ^ AC97_VIDEO;
        return 0;
}
int xonar_st_h6_control_filter(struct snd_kcontrol_new *_template)
{
        if (!strncmp(_template->name, "Master Playback ", 16))
                // no volume/mute, as I²C to the third DAC does not work
                return 1;
        return 0;
}

bool XonarAudioEngine::init(struct oxygen *chip, int model)
{
        /*this function can be looked at as oxygen_pci_probe, with
         *the differences being:
         1. there are no model detection function pointers
         -this means we have to "hard code" the model instantiation
         code using a simple if statement.
         2. after the registers for the requested model are set,
         the code directly after is the content of oxygen_init
         3. still figuring out how to add the gpio/spdifwork to the IOWorkLoopQueue
         -theoretically, we may only need a single IOWorkLoop instance that
         belongs to either the PCIAudioDevice/XonarAudioEngine class, and therefore
         may only need to add {gpio,spdif_input_bits}_changed to this single workloop
         */
        bool result = false;
        
        kprintf("XonarAudioEngine::init\n");
        
        if (!chip) {
                goto Done;
        }
        
        if (!super::init(NULL)) {
                goto Done;
        }
        //compare the subdevice id with known models'
        switch(model) {
                        
                        
                case MODEL_HDAV:
                        //code from get_xonar_pcm179x_model portions (case 0x8314)
                        /*these are the relevant attributes from the xonar_model_hdav struct.
                         *some of the function pointers in the xonar_model struct may not be needed
                         *as OSX has its own way of instantiating volume controls/mixers. however,
                         *the code comprising the functions which the struct is pointing to, may still be
                         *very useful (and indeed:necessary)
                         */
                        chip->model.update_dac_volume = this->update_pcm1796_volume;
                        chip->model.update_dac_mute = this->update_pcm1796_mute;
                        chip->model.model_data_size = sizeof(struct xonar_hdav);
                        chip->model.device_config = PLAYBACK_0_TO_I2S |
                        PLAYBACK_1_TO_SPDIF |
                        CAPTURE_0_FROM_I2S_2 |
                        CAPTURE_1_FROM_SPDIF;
                        chip->model.dac_channels_pcm = 8;
                        chip->model.dac_channels_mixer = 2;
                        chip->model.dac_volume_min = 255 - 2*60;
                        chip->model.dac_volume_max = 255;
                        chip->model.misc_flags = OXYGEN_MISC_MIDI;
                        chip->model.function_flags = OXYGEN_FUNCTION_2WIRE;
                        chip->model.dac_mclks = OXYGEN_MCLKS(512, 128, 128);
                        chip->model.adc_mclks = OXYGEN_MCLKS(256, 128, 128);
                        chip->model.dac_i2s_format = OXYGEN_I2S_FORMAT_I2S;
                        chip->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
                        chip->model.ac97_switch = this->xonar_line_mic_ac97_switch;
                        oxygen_clear_bits16(chip,OXYGEN_GPIO_CONTROL,GPIO_DB_MASK);
                        switch (oxygen_read16(chip, OXYGEN_GPIO_DATA) & GPIO_DB_MASK) {
                                default:
                                        chip->model.shortname = "Xonar HDAV1.3";
                                        break;
                                case GPIO_DB_H6:
                                        chip->model.shortname = "Xonar HDAV1.3+H6";
                                        chip->model.dac_channels_mixer = 8;
                                        chip->model.dac_mclks = OXYGEN_MCLKS(256, 128, 128);
                                        break;
                        }
                        chip->model.uart_input = xonar_hdmi_uart_input;
                        
                        break;
                case MODEL_ST:
                case MODEL_STX:
                case MODEL_STX2:
                case MODEL_XENSE:
                        chip->model.model_data_size = sizeof(struct xonar_pcm179x);
                        chip->model.device_config = PLAYBACK_0_TO_I2S |
                        PLAYBACK_1_TO_SPDIF |
                        CAPTURE_0_FROM_I2S_2 |
                        CAPTURE_1_FROM_SPDIF |
                        AC97_FMIC_SWITCH;
                        chip->model.dac_channels_pcm = 2;
                        chip->model.dac_channels_mixer = 2;
                        chip->model.dac_volume_min = 255 - 2*60;
                        chip->model.dac_volume_max = 255;
                        chip->model.function_flags = OXYGEN_FUNCTION_2WIRE;
                        chip->model.dac_mclks = OXYGEN_MCLKS(512, 128, 128);
                        chip->model.adc_mclks = OXYGEN_MCLKS(256, 128, 128);
                        chip->model.dac_i2s_format = OXYGEN_I2S_FORMAT_I2S;
                        chip->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
                        chip->model.ac97_switch = this->xonar_line_mic_ac97_switch;
                        
                        //0x835d
                        if(model == MODEL_ST) {
                                oxygen_clear_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_DB_MASK);
                                switch (oxygen_read16(chip, OXYGEN_GPIO_DATA) & GPIO_DB_MASK) {
                                        default:
                                                chip->model.shortname = "Xonar ST";
                                                break;
                                        case GPIO_DB_H6:
                                                chip->model.shortname = "Xonar ST+H6";
                                                //still have to figure out all the controls; commenting this
                                                //out for now.
                                                //chip->model.control_filter = xonar_st_h6_control_filter;
                                                chip->model.dac_channels_pcm = 8;
                                                chip->model.dac_channels_mixer = 8;
                                                chip->model.dac_volume_min = 255;
                                                chip->model.dac_mclks = OXYGEN_MCLKS(256, 128, 128);
                                                break;
                                }
                        }
                        //end 0x835d
                        
                        //0x835c
                        else if(model == MODEL_STX) {
                                chip->model.shortname = "Xonar STX";
                                // not sure if we'll need the two lines below */
                                chip->model.set_dac_params = set_pcm1796_params;
                        }
                        //end 0x835c
                        
                        //0x85f4
                        else if(model == MODEL_STX2) {
                                chip->model.set_dac_params = set_pcm1796_params;
                                oxygen_clear_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_DB_MASK);
                                switch (oxygen_read16(chip, OXYGEN_GPIO_DATA) & GPIO_DB_MASK) {
                                        default:
                                                chip->model.shortname = "Xonar STX II";
                                                break;
                                        case GPIO_DB_H6:
                                                chip->model.shortname = "Xonar STX II+H6";
                                                chip->model.dac_channels_pcm = 8;
                                                chip->model.dac_channels_mixer = 8;
                                                chip->model.dac_mclks = OXYGEN_MCLKS(256, 128, 128);
                                                break;
                                }
                                
                        }
                        //end 0x85f4
                        
                        //0x8428
                        else if(model == MODEL_XENSE) {
                                chip->model.shortname = "Xonar Xense";
                                //not sure if we'll need the line below yet
                        }
                        //end 0x8428
                        break;
                        
                case MODEL_D2:
                case MODEL_D2X:
                        chip->model.model_data_size = sizeof(struct xonar_pcm179x);
                        chip->model.device_config = PLAYBACK_0_TO_I2S |
                        PLAYBACK_1_TO_SPDIF |
                        CAPTURE_0_FROM_I2S_2 |
                        CAPTURE_1_FROM_SPDIF |
                        MIDI_OUTPUT |
                        MIDI_INPUT |
                        AC97_CD_INPUT;
                        chip->model.dac_channels_pcm = 8;
                        chip->model.dac_channels_mixer = 8;
                        chip->model.dac_volume_min = 255 - 2*60;
                        chip->model.dac_volume_max = 255;
                        chip->model.misc_flags = OXYGEN_MISC_MIDI;
                        chip->model.function_flags = OXYGEN_FUNCTION_SPI |
                        OXYGEN_FUNCTION_ENABLE_SPI_4_5;
                        chip->model.dac_mclks = OXYGEN_MCLKS(512, 128, 128);
                        chip->model.adc_mclks = OXYGEN_MCLKS(256, 128, 128);
                        chip->model.dac_i2s_format = OXYGEN_I2S_FORMAT_I2S;
                        chip->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
                        chip->model.control_filter = xonar_d2_control_filter;
                        //0x8269
                        if(model == MODEL_D2)
                                chip->model.shortname = "Xonar D2";
                        //end 0x8269
                        
                        //0x82b7
                        else if (model == MODEL_D2X)
                                chip->model.shortname = "Xonar D2X";
                        //end 0x82b7
                        break;
                        
                case MODEL_DS:
                case MODEL_DSX:
                case HDAV_SLIM:
                        if(model == MODEL_DS || model == MODEL_DSX){
                                /*for all submodels, this manual assignment
                                 * mimics chip->model = <struct model name>
                                 */
                                chip->model.model_data_size = sizeof(struct xonar_wm87x6);
                                chip->model.device_config = PLAYBACK_0_TO_I2S |
                                PLAYBACK_1_TO_SPDIF |
                                CAPTURE_0_FROM_I2S_1 |
                                CAPTURE_1_FROM_SPDIF;
                                chip->model.dac_channels_pcm = 8;
                                chip->model.dac_channels_mixer = 8;
                                chip->model.dac_volume_min = 255 - 2*60;
                                chip->model.dac_volume_max = 255;
                                chip->model.function_flags = OXYGEN_FUNCTION_SPI;
                                chip->model.dac_mclks = OXYGEN_MCLKS(256, 256, 128);
                                chip->model.adc_mclks = OXYGEN_MCLKS(256, 256, 128);
                                chip->model.dac_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
                                chip->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
                                /*end chip->model struct assignment */
                                if(model == MODEL_DS)
                                        chip->model.shortname =  "Xonar DS";
                                else if(model == MODEL_DSX)
                                        chip->model.shortname =  "Xonar DSX";
                        }
                        else if (model == HDAV_SLIM) {
                                chip->model.model_data_size = sizeof(struct xonar_wm87x6);
                                chip->model.device_config = PLAYBACK_0_TO_I2S |
                                PLAYBACK_1_TO_SPDIF |
                                CAPTURE_0_FROM_I2S_1 |
                                CAPTURE_1_FROM_SPDIF;
                                chip->model.dac_channels_pcm = 8;
                                chip->model.dac_channels_mixer = 2;
                                chip->model.dac_volume_min = 255 - 2*60;
                                chip->model.dac_volume_max = 255;
                                chip->model.function_flags = OXYGEN_FUNCTION_2WIRE;
                                chip->model.dac_mclks = OXYGEN_MCLKS(256, 256, 128);
                                chip->model.adc_mclks = OXYGEN_MCLKS(256, 256, 128);
                                chip->model.dac_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
                                chip->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
                                chip->model.shortname = "Xonar HDAV1.3 Slim";
                        }
                        break;
                case MODEL_CS43XX:
                case MODEL_DX:
                case MODEL_D1:
                        chip->model.model_data_size = sizeof(struct xonar_cs43xx);
                        chip->model.device_config = PLAYBACK_0_TO_I2S |
                        PLAYBACK_1_TO_SPDIF |
                        CAPTURE_0_FROM_I2S_2 |
                        CAPTURE_1_FROM_SPDIF |
                        AC97_FMIC_SWITCH;
                        chip->model.dac_channels_pcm = 8;
                        chip->model.dac_channels_mixer = 8;
                        chip->model.dac_volume_min = 127 - 60;
                        chip->model.dac_volume_max = 127;
                        chip->model.function_flags = OXYGEN_FUNCTION_2WIRE;
                        chip->model.dac_mclks = OXYGEN_MCLKS(256, 128, 128);
                        chip->model.adc_mclks = OXYGEN_MCLKS(256, 128, 128);
                        chip->model.dac_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
                        chip->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
                        
                        if(model == MODEL_CS43XX || model == MODEL_DX)
                                chip->model.shortname = "Xonar DX";
                        else if (model == MODEL_D1)
                                chip->model.shortname = "Xonar D1";
                        break;
                        
                default:  // generic
                        chip->model.model_data_size = sizeof(struct generic_data);
                        chip->model.device_config = PLAYBACK_0_TO_I2S |
                        PLAYBACK_1_TO_SPDIF |
                        PLAYBACK_2_TO_AC97_1 |
                        CAPTURE_0_FROM_I2S_1 |
                        CAPTURE_1_FROM_SPDIF |
                        CAPTURE_2_FROM_AC97_1 |
                        AC97_CD_INPUT;
                        chip->model.dac_channels_pcm = 8;
                        chip->model.dac_channels_mixer = 8;
                        chip->model.dac_volume_min = 0;
                        chip->model.dac_volume_max = 255;
                        chip->model.function_flags = OXYGEN_FUNCTION_SPI |
                        OXYGEN_FUNCTION_ENABLE_SPI_4_5;
                        chip->model.dac_mclks = OXYGEN_MCLKS(256, 128, 128);
                        chip->model.adc_mclks = OXYGEN_MCLKS(256, 256, 128);
                        chip->model.dac_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
                        chip->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
                        chip->model.shortname = "C-Media CMI8788";
                        
                        
        }
        //begin oxygen_init
        unsigned int i;
        
        chip->dac_routing = 1;
        for (i = 0; i < 8; ++i)
                chip->dac_volume[i] = chip->model.dac_volume_min;
        chip->dac_mute = 1;
        chip->spdif_playback_enable = 1;
        chip->spdif_bits = OXYGEN_SPDIF_C | OXYGEN_SPDIF_ORIGINAL |
        (IEC958_AES1_CON_PCM_CODER << OXYGEN_SPDIF_CATEGORY_SHIFT);
        chip->spdif_pcm_bits = chip->spdif_bits;
        
        if (!(oxygen_read8(chip, OXYGEN_REVISION) & OXYGEN_REVISION_2))
                oxygen_set_bits8(chip, OXYGEN_MISC,
                                 OXYGEN_MISC_PCI_MEM_W_1_CLOCK);
        
        i = oxygen_read16(chip, OXYGEN_AC97_CONTROL);
        chip->has_ac97_0 = (i & OXYGEN_AC97_CODEC_0) != 0;
        chip->has_ac97_1 = (i & OXYGEN_AC97_CODEC_1) != 0;
        
        oxygen_write8_masked(chip, OXYGEN_FUNCTION,
                             OXYGEN_FUNCTION_RESET_CODEC |
                             chip->model.function_flags,
                             OXYGEN_FUNCTION_RESET_CODEC |
                             OXYGEN_FUNCTION_2WIRE_SPI_MASK |
                             OXYGEN_FUNCTION_ENABLE_SPI_4_5);
        oxygen_write8(chip, OXYGEN_DMA_STATUS, 0);
        oxygen_write8(chip, OXYGEN_DMA_PAUSE, 0);
        oxygen_write8(chip, OXYGEN_PLAY_CHANNELS,
                      OXYGEN_PLAY_CHANNELS_2 |
                      OXYGEN_DMA_A_BURST_8 |
                      OXYGEN_DMA_MULTICH_BURST_8);
        oxygen_write16(chip, OXYGEN_INTERRUPT_MASK, 0);
        oxygen_write8_masked(chip, OXYGEN_MISC,
                             chip->model.misc_flags,
                             OXYGEN_MISC_WRITE_PCI_SUBID |
                             OXYGEN_MISC_REC_C_FROM_SPDIF |
                             OXYGEN_MISC_REC_B_FROM_AC97 |
                             OXYGEN_MISC_REC_A_FROM_MULTICH |
                             OXYGEN_MISC_MIDI);
        oxygen_write8(chip, OXYGEN_REC_FORMAT,
                      (OXYGEN_FORMAT_16 << OXYGEN_REC_FORMAT_A_SHIFT) |
                      (OXYGEN_FORMAT_16 << OXYGEN_REC_FORMAT_B_SHIFT) |
                      (OXYGEN_FORMAT_16 << OXYGEN_REC_FORMAT_C_SHIFT));
        oxygen_write8(chip, OXYGEN_PLAY_FORMAT,
                      (OXYGEN_FORMAT_16 << OXYGEN_SPDIF_FORMAT_SHIFT) |
                      (OXYGEN_FORMAT_16 << OXYGEN_MULTICH_FORMAT_SHIFT));
        oxygen_write8(chip, OXYGEN_REC_CHANNELS, OXYGEN_REC_CHANNELS_2_2_2);
        oxygen_write16(chip, OXYGEN_I2S_MULTICH_FORMAT,
                       OXYGEN_RATE_48000 |
                       chip->model.dac_i2s_format |
                       OXYGEN_I2S_MCLK(chip->model.dac_mclks) |
                       OXYGEN_I2S_BITS_16 |
                       OXYGEN_I2S_MASTER |
                       OXYGEN_I2S_BCLK_64);
        if (chip->model.device_config & CAPTURE_0_FROM_I2S_1)
                oxygen_write16(chip, OXYGEN_I2S_A_FORMAT,
                               OXYGEN_RATE_48000 |
                               chip->model.adc_i2s_format |
                               OXYGEN_I2S_MCLK(chip->model.adc_mclks) |
                               OXYGEN_I2S_BITS_16 |
                               OXYGEN_I2S_MASTER |
                               OXYGEN_I2S_BCLK_64);
        else
                oxygen_write16(chip, OXYGEN_I2S_A_FORMAT,
                               OXYGEN_I2S_MASTER |
                               OXYGEN_I2S_MUTE_MCLK);
        if (chip->model.device_config & (CAPTURE_0_FROM_I2S_2 |
                                         CAPTURE_2_FROM_I2S_2))
                oxygen_write16(chip, OXYGEN_I2S_B_FORMAT,
                               OXYGEN_RATE_48000 |
                               chip->model.adc_i2s_format |
                               OXYGEN_I2S_MCLK(chip->model.adc_mclks) |
                               OXYGEN_I2S_BITS_16 |
                               OXYGEN_I2S_MASTER |
                               OXYGEN_I2S_BCLK_64);
        else
                oxygen_write16(chip, OXYGEN_I2S_B_FORMAT,
                               OXYGEN_I2S_MASTER |
                               OXYGEN_I2S_MUTE_MCLK);
        if (chip->model.device_config & CAPTURE_3_FROM_I2S_3)
                oxygen_write16(chip, OXYGEN_I2S_C_FORMAT,
                               OXYGEN_RATE_48000 |
                               chip->model.adc_i2s_format |
                               OXYGEN_I2S_MCLK(chip->model.adc_mclks) |
                               OXYGEN_I2S_BITS_16 |
                               OXYGEN_I2S_MASTER |
                               OXYGEN_I2S_BCLK_64);
        else
                oxygen_write16(chip, OXYGEN_I2S_C_FORMAT,
                               OXYGEN_I2S_MASTER |
                               OXYGEN_I2S_MUTE_MCLK);
        oxygen_clear_bits32(chip, OXYGEN_SPDIF_CONTROL,
                            OXYGEN_SPDIF_OUT_ENABLE |
                            OXYGEN_SPDIF_LOOPBACK);
        if (chip->model.device_config & CAPTURE_1_FROM_SPDIF)
                oxygen_write32_masked(chip, OXYGEN_SPDIF_CONTROL,
                                      OXYGEN_SPDIF_SENSE_MASK |
                                      OXYGEN_SPDIF_LOCK_MASK |
                                      OXYGEN_SPDIF_RATE_MASK |
                                      OXYGEN_SPDIF_LOCK_PAR |
                                      OXYGEN_SPDIF_IN_CLOCK_96,
                                      OXYGEN_SPDIF_SENSE_MASK |
                                      OXYGEN_SPDIF_LOCK_MASK |
                                      OXYGEN_SPDIF_RATE_MASK |
                                      OXYGEN_SPDIF_SENSE_PAR |
                                      OXYGEN_SPDIF_LOCK_PAR |
                                      OXYGEN_SPDIF_IN_CLOCK_MASK);
        else
                oxygen_clear_bits32(chip, OXYGEN_SPDIF_CONTROL,
                                    OXYGEN_SPDIF_SENSE_MASK |
                                    OXYGEN_SPDIF_LOCK_MASK |
                                    OXYGEN_SPDIF_RATE_MASK);
        oxygen_write32(chip, OXYGEN_SPDIF_OUTPUT_BITS, chip->spdif_bits);
        oxygen_write16(chip, OXYGEN_2WIRE_BUS_STATUS,
                       OXYGEN_2WIRE_LENGTH_8 |
                       OXYGEN_2WIRE_INTERRUPT_MASK |
                       OXYGEN_2WIRE_SPEED_STANDARD);
        oxygen_clear_bits8(chip, OXYGEN_MPU401_CONTROL, OXYGEN_MPU401_LOOPBACK);
        oxygen_write8(chip, OXYGEN_GPI_INTERRUPT_MASK, 0);
        oxygen_write16(chip, OXYGEN_GPIO_INTERRUPT_MASK, 0);
        oxygen_write16(chip, OXYGEN_PLAY_ROUTING,
                       OXYGEN_PLAY_MULTICH_I2S_DAC |
                       OXYGEN_PLAY_SPDIF_SPDIF |
                       (0 << OXYGEN_PLAY_DAC0_SOURCE_SHIFT) |
                       (1 << OXYGEN_PLAY_DAC1_SOURCE_SHIFT) |
                       (2 << OXYGEN_PLAY_DAC2_SOURCE_SHIFT) |
                       (3 << OXYGEN_PLAY_DAC3_SOURCE_SHIFT));
        oxygen_write8(chip, OXYGEN_REC_ROUTING,
                      OXYGEN_REC_A_ROUTE_I2S_ADC_1 |
                      OXYGEN_REC_B_ROUTE_I2S_ADC_2 |
                      OXYGEN_REC_C_ROUTE_SPDIF);
        oxygen_write8(chip, OXYGEN_ADC_MONITOR, 0);
        oxygen_write8(chip, OXYGEN_A_MONITOR_ROUTING,
                      (0 << OXYGEN_A_MONITOR_ROUTE_0_SHIFT) |
                      (1 << OXYGEN_A_MONITOR_ROUTE_1_SHIFT) |
                      (2 << OXYGEN_A_MONITOR_ROUTE_2_SHIFT) |
                      (3 << OXYGEN_A_MONITOR_ROUTE_3_SHIFT));
        
        if (chip->has_ac97_0 | chip->has_ac97_1)
                oxygen_write8(chip, OXYGEN_AC97_INTERRUPT_MASK,
                              OXYGEN_AC97_INT_READ_DONE |
                              OXYGEN_AC97_INT_WRITE_DONE);
        else
                oxygen_write8(chip, OXYGEN_AC97_INTERRUPT_MASK, 0);
        oxygen_write32(chip, OXYGEN_AC97_OUT_CONFIG, 0);
        oxygen_write32(chip, OXYGEN_AC97_IN_CONFIG, 0);
        if (!(chip->has_ac97_0 | chip->has_ac97_1))
                oxygen_set_bits16(chip, OXYGEN_AC97_CONTROL,
                                  OXYGEN_AC97_CLOCK_DISABLE);
        
        
        if (!chip->has_ac97_0) {
                oxygen_set_bits16(chip, OXYGEN_AC97_CONTROL,
                                  OXYGEN_AC97_NO_CODEC_0);
        } else {
                oxygen_write_ac97(chip, 0, AC97_RESET, 0);
                IODelay(1000);
                oxygen_ac97_set_bits(chip, 0, CM9780_GPIO_SETUP,
                                     CM9780_GPIO0IO | CM9780_GPIO1IO);
                oxygen_ac97_set_bits(chip, 0, CM9780_MIXER,
                                     CM9780_BSTSEL | CM9780_STRO_MIC |
                                     CM9780_MIX2FR | CM9780_PCBSW);
                oxygen_ac97_set_bits(chip, 0, CM9780_JACK,
                                     CM9780_RSOE | CM9780_CBOE |
                                     CM9780_SSOE | CM9780_FROE |
                                     CM9780_MIC2MIC | CM9780_LI2LI);
                oxygen_write_ac97(chip, 0, AC97_MASTER, 0x0000);
                oxygen_write_ac97(chip, 0, AC97_PC_BEEP, 0x8000);
                oxygen_write_ac97(chip, 0, AC97_MIC, 0x8808);
                oxygen_write_ac97(chip, 0, AC97_LINE, 0x0808);
                oxygen_write_ac97(chip, 0, AC97_CD, 0x8808);
                oxygen_write_ac97(chip, 0, AC97_VIDEO, 0x8808);
                oxygen_write_ac97(chip, 0, AC97_AUX, 0x8808);
                oxygen_write_ac97(chip, 0, AC97_REC_GAIN, 0x8000);
                oxygen_write_ac97(chip, 0, AC97_CENTER_LFE_MASTER, 0x8080);
                oxygen_write_ac97(chip, 0, AC97_SURROUND_MASTER, 0x8080);
                oxygen_ac97_clear_bits(chip, 0, CM9780_GPIO_STATUS,
                                       CM9780_GPO0);
                /* power down unused ADCs and DACs */
                oxygen_ac97_set_bits(chip, 0, AC97_POWERDOWN,
                                     AC97_PD_PR0 | AC97_PD_PR1);
                oxygen_ac97_set_bits(chip, 0, AC97_EXTENDED_STATUS,
                                     AC97_EA_PRI | AC97_EA_PRJ | AC97_EA_PRK);
        }
        if (chip->has_ac97_1) {
                oxygen_set_bits32(chip, OXYGEN_AC97_OUT_CONFIG,
                                  OXYGEN_AC97_CODEC1_SLOT3 |
                                  OXYGEN_AC97_CODEC1_SLOT4);
                oxygen_write_ac97(chip, 1, AC97_RESET, 0);
                IODelay(1000);
                oxygen_write_ac97(chip, 1, AC97_MASTER, 0x0000);
                oxygen_write_ac97(chip, 1, AC97_HEADPHONE, 0x8000);
                oxygen_write_ac97(chip, 1, AC97_PC_BEEP, 0x8000);
                oxygen_write_ac97(chip, 1, AC97_MIC, 0x8808);
                oxygen_write_ac97(chip, 1, AC97_LINE, 0x8808);
                oxygen_write_ac97(chip, 1, AC97_CD, 0x8808);
                oxygen_write_ac97(chip, 1, AC97_VIDEO, 0x8808);
                oxygen_write_ac97(chip, 1, AC97_AUX, 0x8808);
                oxygen_write_ac97(chip, 1, AC97_PCM, 0x0808);
                oxygen_write_ac97(chip, 1, AC97_REC_SEL, 0x0000);
                oxygen_write_ac97(chip, 1, AC97_REC_GAIN, 0x0000);
                oxygen_ac97_set_bits(chip, 1, 0x6a, 0x0040);
        }
        // end oxygen_init
        chip->card_model = model;
        chip->model_data = IOMalloc(chip->model.model_data_size);
        bzero(chip->model_data, chip->model.model_data_size);
        dev_id = chip;
        //save ptr to oxygen struct from PCIDriver into private class var dev_id for interrupthandler
        chipData = (struct oxygen*) dev_id;
        result = true;
        
Done:
        
        return result;
}

bool XonarAudioEngine::initHardware(IOService *provider)
{
        spdifEngine = new XonarSPDIFAudioEngine;
        ac97Engine = new XonarAC97AudioEngine;
        inputStream = NULL;
        outputStream = NULL;
        inputStream2 = NULL;
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#endif
        bool result = false;
        channel_mask = 0; //zero it out to be safe.
        /* Comments by Broly:
         *  After XonarAudioEngine::init has been called, we have completed:
         1. oxygen_pci_probe
         2. oxygen_init
         
         now we must complete oxygen_pcm_init
         */
        
        
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s oxygen_pcm_init()\n", " ");
#endif
        if (!super::initHardware(provider)) {
                goto Done;
        }
    
    // Setup the initial sample rate for the audio engine
    //initialSampleRate.whole = INITIAL_SAMPLE_RATE;
    //initialSampleRate.fraction = 0;
    //setDescription("Sample PCI Audio Engine");
#if DEBUG && (DEBUGLEVEL > 2)
    kprintf("XonarAudioEngine::%-30s setting samplerate\n", " ");
#endif
    setSampleRate(&ac97_sampleRate);
#if DEBUG && (DEBUGLEVEL > 2)
    kprintf("XonarAudioEngine::%-30s setting number of frames per buffer\n", " ");
#endif
    // Set the number of sample frames in each buffer
    setNumSampleFramesPerBuffer(NUM_SAMPLE_FRAMES);
    
    setDescription("Xonar Audio Engine (MAIN)");
    
    // Create an interrupt event source through which to receive interrupt callbacks
    // In this case, we only want to do work at primary interrupt time, so
    // we create an IOFilterInterruptEventSource which makes a filtering call
    // from the primary interrupt interrupt who's purpose is to determine if
    // our secondary interrupt handler is to be called.  In our case, we
    // can do the work in the filter routine and then return false to
    // indicate that we do not want our secondary handler called
    
    interruptEventSource_main = IOFilterInterruptEventSource::filterInterruptEventSource(this,
                                                                                         XonarAudioEngine::interruptHandler,
                                                                                         XonarAudioEngine::interruptFilter,
                                                                                         audioDevice->getProvider());
    
    if (!interruptEventSource_main) { //|| !gpioEventSource || !spdifEventSource) { <- (see comment in header file)
        goto Done;
    }
    
#if DEBUG && (DEBUGLEVEL > 2)
    kprintf("XonarAudioEngine::%-30s adding eventsource\n", " ");
#endif
    
    // the interruptEventSource needs to be enabled here, else IRQ sharing doesn't work
    // this 'bug' was found with the help of hydrasworld / hydra
    interruptEventSource_main->enable();
    
    // In order to allow the interrupts to be received, the interrupt event source must be
    // added to the IOWorkLoop
    // Additionally, interrupts will not be firing until the interrupt event source is
    // enabled by calling interruptEventSource->enable() - this probably doesn't need to
    // be done until performAudioEngineStart() is called, and can probably be disabled
    // when performAudioEngineStop() is called and the audio engine is no longer running
    // Although this really depends on the specific hardware
    workLoop->addEventSource(interruptEventSource_main);
        
        //begin oxygen_pcm_init
        
        //each audioStream is going to be attached to this engine, so XonarAudioEngine should
        //behave similar to the ALSA *pcm object declared below
        //struct snd_pcm *pcm;
        int outs, ins;
        int err;
        
        outs = !!(chipData->model.device_config & PLAYBACK_0_TO_I2S);
        ins = !!(chipData->model.device_config & (CAPTURE_0_FROM_I2S_1 |
                                                  CAPTURE_0_FROM_I2S_2));
        if (outs | ins) {
                //err = snd_pcm_new(chip->card, "Multichannel",
                //                  0, outs, ins, &pcm);
                //if (err < 0)
                //    return err;
                if (outs) {
                        // add multich_ops fns
                        outputStream = createAudioStream(kIOAudioStreamDirectionOutput, DEFAULT_BUFFER_BYTES_MULTICH, PCM_MULTICH);
                        if (!outputStream->stream) {
#if DEBUG
                                kprintf("XonarAudioEngine::%-30s Creation of PCM_MULTICH stream failed!\n", __func__);
#endif
                                goto Done;
                        }
                        outputStream->stream->setName("MultiChannel OUT");
                        addAudioStream(outputStream->stream);
                        outputStream->stream->release();
                        
                }
                if (chipData->model.device_config & CAPTURE_0_FROM_I2S_1) {
                        //add rec_a_ops fns
                        inputStream = createAudioStream(kIOAudioStreamDirectionInput, DEFAULT_BUFFER_BYTES, PCM_A);
                        
                        if (!inputStream->stream) {
#if DEBUG
                                kprintf("XonarAudioEngine::%-30s Creation of REC_A stream failed!\n", __func__);
#endif
                                goto Done;
                        }
                        inputStream->stream->setName("MultiChannel IN");
                        addAudioStream(inputStream->stream);
                        inputStream->stream->release();
                }
                else if (chipData->model.device_config & CAPTURE_0_FROM_I2S_2) {
                        //add rec_b_ops fns
                        inputStream = createAudioStream(kIOAudioStreamDirectionInput, DEFAULT_BUFFER_BYTES, PCM_B);
                        
                        if (!inputStream->stream) {
#if DEBUG
                                kprintf("XonarAudioEngine::%-30s Creation of REC_B stream failed!\n", __func__);
#endif
                                goto Done;
                        }
                        inputStream->stream->setName("MultiChannel IN");
                        addAudioStream(inputStream->stream);
                        inputStream->stream->release();
                }
                
                // with the OOP approach of APPUL's IOAudio, we always have a way to access the
                // chipdata, meaning we don't need to use alsa's private_data
                //        pcm->private_data = chip;
                //        strcpy(pcm->name, "Multichannel");
                
                //        if (outs)
                //            snd_pcm_lib_preallocate_pages(pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream,
                //                                          SNDRV_DMA_TYPE_DEV,
                //                                          snd_dma_pci_data(chip->pci),
                //                                          DEFAULT_BUFFER_BYTES_MULTICH,
                //                                          BUFFER_BYTES_MAX_MULTICH);
                
                //         if (ins)
                //            snd_pcm_lib_preallocate_pages(pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream,
                //                                          SNDRV_DMA_TYPE_DEV,
                //                                          snd_dma_pci_data(chip->pci),
                //                                          DEFAULT_BUFFER_BYTES,
                //                                          BUFFER_BYTES_MAX);
                
                
        }
#if !defined(MULTICH_OUTPUT_ONLY)
                spdifEngine->init(this);
                if(((PCIAudioDevice*) provider)->activateAudioEngine(spdifEngine)) {
                        kprintf("XonarAudioEngine::%-30s SPDIF engine creation failed.\n", __func__);
                }
                else {
                        spdifEngine->release();
                        kprintf("XonarAudioEngine::%-30s SPDIF engine successfully created.\n", __func__);
                }
                ac97Engine->init(this);
                if(((PCIAudioDevice*) provider)->activateAudioEngine(ac97Engine)) {
                        kprintf("XonarAudioEngine::%-30s AC97 engine creation failed.\n", __func__);
                }
                else{
                        ac97Engine->release();
                        kprintf("XonarAudioEngine::%-30s AC97 engine successfully created.\n", __func__);

                }

        ins = !!(chipData->model.device_config & CAPTURE_3_FROM_I2S_3);
        if (ins) {
                //err = snd_pcm_new(chip->card,  3, 0, ins, &pcm);
                //if (err < 0)
                //    return err;
                //add rec_c fns
                inputStream2 = createAudioStream(kIOAudioStreamDirectionInput, DEFAULT_BUFFER_BYTES, PCM_C);
                
                oxygen_write8_masked(chipData, OXYGEN_REC_ROUTING,
                                     OXYGEN_REC_C_ROUTE_I2S_ADC_3,
                                     OXYGEN_REC_C_ROUTE_MASK);
                if (!inputStream2->stream) {
#if DEBUG
                        kprintf("XonarAudioEngine::%-30s Creation of REC_C stream failed!\n", __func__);
#endif
                        goto Done;
                }
                inputStream2->stream->setName("Analog 3");
                addAudioStream(inputStream2->stream);
                //inputStream2->stream->release();
                
                //            pcm->private_data = chip;
                //            strcpy(pcm->name, "Analog 3");
                //            snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
                //                                                  snd_dma_pci_data(chip->pci),
                //                                                  DEFAULT_BUFFER_BYTES,
                //                                                  BUFFER_BYTES_MAX);
        }
#endif
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s oxygen_pcm_init completed, initialising mixer\n", " ");
#endif
        //end oxygen_pcm_init
        
        /* The remaining portions of oxygen_pci_probe focus on initialising PCM and the mixer.
         * from what i can gather, these portions of the init from the Linux Driver are handled
         * radically differently from OSX, and so this is where OSX-specific/new code will need to
         * handle these differences, where some of the code in oxygen_mixer.c/oxygen_pcm.c may be
         * used for the aforementioned purposes
         */
        
        //((PCIAudioDevice*)provider)->oxygen_mixer_init();
        //oxygen_proc_init(chipData);
#if !defined(MULTICH_OUTPUT_ONLY)
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s before SPDIF detect lock\n", " ");
#endif
        IOSimpleLockLock(chipData->reg_lock);
        if (chipData->model.device_config & CAPTURE_1_FROM_SPDIF)
                chipData->interrupt_mask |= OXYGEN_INT_SPDIF_IN_DETECT;
        if (chipData->has_ac97_0 | chipData->has_ac97_1)
                chipData->interrupt_mask |= OXYGEN_INT_AC97;
        oxygen_write16(chipData, OXYGEN_INTERRUPT_MASK, chipData->interrupt_mask);
        IOSimpleLockUnlock(chipData->reg_lock);
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s after SPDIF detect lock\n", " ");
#endif
#endif
    
        result = true;
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
Done:
        return result;
        
}


void XonarAudioEngine::free()
{
    if (interruptEventSource_main) {
        interruptEventSource_main->disable();
        interruptEventSource_main->release();
        interruptEventSource_main = NULL;
    }
    
#if DEBUG
        kprintf("XonarAudioEngine::%-30s\n", __func__);
#endif
        // We need to free our resources when we're going away
        
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s freeing chipData->model_data\n", " ");
#endif
        if(chipData->model_data)
                IOFree(chipData->model_data, chipData->model.model_data_size);
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s releasing interruptEventSource_main\n"," ");
#endif
        if (interruptEventSource_main) {
                interruptEventSource_main->release();
                interruptEventSource_main = NULL;
        }
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s freeing substream buffers\n"," ");
#endif
//        for(int i=0;i < PCM_COUNT; i++) {
        if(outputStream) {
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s freeing output stream\n", " ");
#endif
                outputStream->buffer->IOGeneralMemoryDescriptor::free();
        }
        if(inputStream) {
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s freeing input stream 1\n", " ");
#endif
                if(inputStream->buffer)
                        inputStream->buffer->IOGeneralMemoryDescriptor::free();
        }
        if(inputStream2) {
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s freeing input stream 2\n", " ");
#endif
                if(inputStream2->buffer)
                        inputStream2->buffer->IOGeneralMemoryDescriptor::free();
                
                //                if (sampleBuffers[i].substreams[0] != NULL)
                //                        sampleBuffers[i].substreams[0]->IOGeneralMemoryDescriptor::free();
                //                if (sampleBuffers[i].substreams[1] != NULL)
                //                        sampleBuffers[i].substreams[1]->IOGeneralMemoryDescriptor::free();
        }
        super::free();
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
}

IOStream *XonarAudioEngine::createAudioStream(IOAudioStreamDirection direction, UInt32 sampleBufferSize, uint source)
{
        IOStream *newStream = new IOStream;
        //when calling createaudiostream, we are "in" oxygen_pcm_init.
        //this function emulates snd_pcm_new, because each function is creating a stream.
        newStream->stream = new IOAudioStream;
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#endif
        //create the buffers.
    if(direction == kIOAudioStreamDirectionInput)
        newStream->buffer = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, kIODirectionIn | kIOMemoryPhysicallyContiguous, sampleBufferSize, allocation_mask);
    else
        newStream->buffer = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, kIODirectionOut | kIOMemoryPhysicallyContiguous, sampleBufferSize, allocation_mask);
        

#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s buffer creation COMPLETE, calling oxygen_open, direction %d, sample buff address: 0x%08llx\n",
                " ", direction, newStream->buffer->getPhysicalAddress());
#endif
        // call oxygen_open
        newStream->stream->initWithAudioEngine(this, direction, 1);
        oxygen_open(newStream->stream, source);
        
        // call oxygen_*_hw_params
        //        if(source == PCM_A)
        //                oxygen_rec_a_hw_params(audioStream, 0, NULL, NULL, NULL);
        //        else if(source == PCM_B)
        //                oxygen_rec_b_hw_params(audioStream, 0, NULL, NULL, NULL);
        //        else if(source == PCM_C)
        //                oxygen_rec_c_hw_params(audioStream, 0, NULL, NULL, NULL);
        //        else if(source == PCM_MULTICH)
        //                oxygen_multich_hw_params(audioStream, 0, NULL, NULL, NULL);
        //        else if(source == PCM_AC97)
        //                oxygen_hw_params(audioStream, 0, PCM_AC97, NULL, NULL);
        //        else if(source == PCM_SPDIF)
        //                spdifEngine->oxygen_spdif_hw_params(audioStream, 0, NULL, NULL, NULL);
        
        //per alsa documentation (https://www.kernel.org/doc/html/latest/sound/kernel-api/writing-an-alsa-driver.html#operators)
        //hw_params is only called when all of the info about the stream is known
        //however it seems like we can get away with it
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s setting sample buffer\n", " ");
#endif
        newStream->stream->setSampleBuffer(newStream->buffer->getBytesNoCopy(), newStream->buffer->getCapacity());
        
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s setting sample format\n", " ");
#endif
        //we shouldn't call the driver because the SPDIF and AC97
        //need the data from this call before they can do anything else.
        newStream->stream->setFormat(&oxygen_hardware[source], false);
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        //we use a mask variable to know what streams we have to stop
        channel_mask |= (1 << source);
        return newStream;
}


void XonarAudioEngine::stop(IOService *provider)
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#endif
        // When our device is being stopped and torn down, we should go ahead and remove
        // the interrupt event source from the IOWorkLoop
        // Additionally, we'll go ahead and release the interrupt event source since it isn't
        // needed any more
        if (interruptEventSource_main) {
                IOWorkLoop *wl;
                
                wl = getWorkLoop();
                if (wl) {
                        wl->removeEventSource(interruptEventSource_main);
                }
                
                interruptEventSource_main->release();
                interruptEventSource_main = NULL;
        }
        
        // Add code to shut down hardware (beyond what is needed to simply stop the audio engine)
        // There may be nothing needed here
        
        super::stop(provider);
//#if DEBUG
//        kprintf("XonarAudioEngine::%-30s trying to stop SPDIF engine (if it exists)\n", __func__);
//        spdifEngine->stop(provider);
//#endif
//#if DEBUG
//        kprintf("XonarAudioEngine::%-30s trying to stop AC97 engine (if it exists)\n", __func__);
//        spdifEngine->stop(provider);
//#endif
#if DEBUG
#if DEBUGLEVEL > 1
        kprintf("XonarAudioEngine::%-30s try to release submodel here..\n", __func__);
#endif
        ((PCIAudioDevice *)audioDevice)->submodelInstance->release();
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
}

IOReturn XonarAudioEngine::performAudioEngineStart()
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#endif
        int ret = kIOReturnError;
        
        
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s getting workloop\n", " ");
#endif
        workLoop = getWorkLoop();
        if (!workLoop) {
                goto Done;
        }
        
        //we need to call prepare here, not when format changes are performed.
        for(int i=0; i<PCM_COUNT; i++)
                if((1 << i) & channel_mask)
                        oxygen_prepare(i);
        
    
        
        // When performAudioEngineStart() gets called, the audio engine should be started from the beginning
        // of the sample buffer.  Because it is starting on the first sample, a new timestamp is needed
        // to indicate when that sample is being read from/written to.  The function takeTimeStamp()
        // is provided to do that automatically with the current time.
        // By default takeTimeStamp() will increment the current loop count in addition to taking the current
        // timestamp.  Since we are starting a new audio engine run, and not looping, we don't want the loop count
        // to be incremented.  To accomplish that, false is passed to takeTimeStamp().
        
        /* ^^ this seems like where we'd need something like snd_pcm_period_elapsed.
         * providing first conditional from alsa main interrupt handler (oxygen_interrupt, oxygen_lib.c line 100-101):
         */
        // for(i = 0; i < PCM_COUNT; ++i) <- //we don't need the loop here as each
        //                                   // IOAudioStream instance should do this
        //      if ((elapsed_streams & (1 << i)) && chip->streams[i])
        //          snd_pcm_period_elapsed(chip->streams[i]);
        //..
        /*
         * need a way to update the frame pointers at the beginning of each interrupt so that the ensuing
         * operations are applied to a fresh chunk of data (process per-chunk i guess).
         */
        takeTimeStamp(false);
        
        // Add audio - I/O start code here
        
        //#error performAudioEngineStart() - driver will not work until audio engine start code is added
        
        ret = kIOReturnSuccess;
        //didn't realise we had to set the state on our own....
        //OOOOK APPUL, kind of stupid not to set it after
        //this function returns successfully... almost kind of
        //... full <you know what>
        //setState(kIOAudioEngineRunning);
        oxygen_trigger();
#if DEBUG
        //((PCIAudioDevice*) audioDevice)->pciDevice->extendedConfigWrite16(OXYGEN_INTERRUPT_STATUS, 1);
    
    oxygen_write16(chipData, OXYGEN_INTERRUPT_STATUS, 1);
    
    kprintf("XonarAudioEngine::%-30s pcm_running    0x%08x, DMA_STATUS 0x%08x\n"
                "XonarAudioEngine::%-30s INTERRUPT_MASK 0x%08x, INTERRUPT_STATUS 0x%08x\n",
                " ", chipData->pcm_running, oxygen_read8(chipData, OXYGEN_DMA_STATUS),
                " ", oxygen_read16(chipData, OXYGEN_INTERRUPT_MASK), oxygen_read16(chipData, OXYGEN_INTERRUPT_STATUS));
        
#endif
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
Done:
        return ret;
}

IOReturn XonarAudioEngine::performAudioEngineStop()
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#endif
        
        for(int i=0; i<PCM_COUNT; i++) {
                if((1 << i) & channel_mask) {
#if DEBUG && (DEBUGLEVEL > 2)
                        kprintf("XonarAudioEngine::%-30s stopping interrupts on channel %d\n", __func__, i);
#endif
                        if(i == PCM_SPDIF)
                                oxygen_spdif_hw_free();
                        oxygen_hw_free(i);
                }
        }
    
        //setState(kIOAudioEngineStopped);
        
        // Assuming we don't need interrupts after stopping the audio engine, we can disable them here
        //        assert(interruptEventSource_main);
        //        interruptEventSource_main->disable();
        // Add audio - I/O stop code here
        
        //#error performAudioEngineStop() - driver will not work until audio engine stop code is added
    
    oxygen_write16(chipData, OXYGEN_INTERRUPT_STATUS, 0);
    
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        return kIOReturnSuccess;
}

UInt32 XonarAudioEngine::getCurrentSampleFrame()
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s ", __func__);
#endif
        // In order for the erase process to run properly, this function must return the current location of
        // the audio engine - basically a sample counter
        // It doesn't need to be exact, but if it is inexact, it should err towards being before the current location
        // rather than after the current location.  The erase head will erase up to, but not including the sample
        // frame returned by this function.  If it is too large a value, sound data that hasn't been played will be
        // erased.
        
        
        //apparently we don't care at all about the inputs. so, this engine only needs to stay on top
        // of the MULTICH output stream.
        UInt32 curr_addr;
        curr_addr = oxygen_read32(chipData, channel_base_registers[PCM_MULTICH]);
        
#if DEBUG
    kprintf("curr_addr is 0x%llx\n", curr_addr - outputStream->buffer->getPhysicalAddress());
#endif
        
        return curr_addr - outputStream->buffer->getPhysicalAddress();
        
}


IOReturn XonarAudioEngine::performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate)
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#endif
        /* it seems we would have to call hw_params every time the formats change. calling it once is not enough.
         so i guess calling hw_params will act as the switch statement and go through (and set) the format
         accordingly */
        
        //since this function is called when we either set or change a format,
        //we must use the tag of the newFormat because if we're setting a format
        //for the first time, the audioStream will give us an fDriverTag of 0, which is
        // obviously wrong
        int source = newFormat->fDriverTag; //getStartingChannelID();
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s format's stream ID is %d\n", " ", source);
#endif
        //clean out whatever settings may be on the card first.
        
        //ideally we are going to tune this formatExtension struct if this ever works
        //right now we need to pump out values that will create sound.
        IOAudioStreamFormatExtension formatExDefault;
        formatExDefault.fBytesPerPacket = 32;
        formatExDefault.fFramesPerPacket = 1;
        
        //clear the device before updating the format
        if(source == PCM_A)
                oxygen_rec_a_hw_params(audioStream, 1, newFormat, &formatExDefault, newSampleRate);
        else if(source == PCM_B)
                oxygen_rec_b_hw_params(audioStream, 1, newFormat, &formatExDefault, newSampleRate);
        else if(source == PCM_C)
                oxygen_rec_c_hw_params(audioStream, 1, newFormat, &formatExDefault, newSampleRate);
        else if(source == PCM_MULTICH)
                oxygen_multich_hw_params(audioStream, 1, newFormat, &formatExDefault, newSampleRate);
        //        else if(source == PCM_AC97)
        //                oxygen_hw_params(audioStream, 1, PCM_AC97, newFormat, &formatExDefault);
        //        else if(source == PCM_SPDIF)
        //                oxygen_spdif_hw_params(audioStream, 1, newFormat, &formatExDefault, newSampleRate);
        
        //    if (newSampleRate) {
        //        switch (newSampleRate->whole) {
        //            case 44100:
        //                kprintf("/t-> 44.1kHz selected\n");
        //
        //                // Add code to switch hardware to 44.1khz
        //                break;
        //            case 48000:
        //                kprintf("/t-> 48kHz selected\n");
        //
        //                // Add code to switch hardware to 48kHz
        //                break;
        //            default:
        //                // This should not be possible since we only specified 44100 and 48000 as valid sample rates
        //                kprintf("/t Internal Error - unknown sample rate selected.\n");
        //                break;
        //        }
        //    }
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        return kIOReturnSuccess;
}

void XonarAudioEngine::oxygen_gpio_changed(struct oxygen* chip, XonarAudioEngine *engineInstance)
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#endif
        if (chip->model.gpio_changed)
                chip->model.gpio_changed(chip, engineInstance);
        
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
}



void XonarAudioEngine::oxygen_spdif_input_bits_changed(struct oxygen* chip)
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s START\n", __func__);
#endif
        UInt32 reg;
        /*
         * This function gets called when there is new activity on the SPDIF
         * input, or when we lose lock on the input signal, or when the rate
         * changes.
         */
        IODelay(1000);
        IOSimpleLockLock(chip->reg_lock);
        reg = oxygen_read32(chip, OXYGEN_SPDIF_CONTROL);
        if ((reg & (OXYGEN_SPDIF_SENSE_STATUS |
                    OXYGEN_SPDIF_LOCK_STATUS))
            == OXYGEN_SPDIF_SENSE_STATUS) {
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s OXYGEN_SPDIF_{SENSE,LOCK}_STATUS is true\n", __func__);
#endif
                /*
                 * If we detect activity on the SPDIF input but cannot lock to
                 * a signal, the clock bit is likely to be wrong.
                 */
                reg ^= OXYGEN_SPDIF_IN_CLOCK_MASK;
                oxygen_write32(chip, OXYGEN_SPDIF_CONTROL, reg);
                IOSimpleLockUnlock(chip->reg_lock);
                IODelay(1000);
                IOSimpleLockLock(chip->reg_lock);
                reg = oxygen_read32(chip, OXYGEN_SPDIF_CONTROL);
                if ((reg & (OXYGEN_SPDIF_SENSE_STATUS |
                            OXYGEN_SPDIF_LOCK_STATUS))
                    == OXYGEN_SPDIF_SENSE_STATUS) {
                        /* nothing detected with either clock; give up */
                        if ((reg & OXYGEN_SPDIF_IN_CLOCK_MASK)
                            == OXYGEN_SPDIF_IN_CLOCK_192) {
                                /*
                                 * Reset clock to <= 96 kHz because this is
                                 * more likely to be received next time.
                                 */
                                reg &= ~OXYGEN_SPDIF_IN_CLOCK_MASK;
                                reg |= OXYGEN_SPDIF_IN_CLOCK_96;
                                oxygen_write32(chip, OXYGEN_SPDIF_CONTROL, reg);
                        }
                }
        }
        IOSimpleLockUnlock(chip->reg_lock);
        
        if (chip->controls[CONTROL_SPDIF_INPUT_BITS]) {
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s chip->controls[CONTROL_SPDIF_INPUT_BITS] is true\n", __func__);
#endif
                IOSimpleLockLock(chip->reg_lock);
                chip->interrupt_mask |= OXYGEN_INT_SPDIF_IN_DETECT;
                oxygen_write16(chip, OXYGEN_INTERRUPT_MASK,
                               chip->interrupt_mask);
                IOSimpleLockUnlock(chip->reg_lock);
                
                
                // We don't actually know that any channel status bits have
                // changed, but let's send a notification just to be sure.
                //
                //    snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
                //                  &chip->controls[CONTROL_SPDIF_INPUT_BITS]->id);
        }
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
}





void XonarAudioEngine::interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count)
{
        // Since our interrupt filter always returns false, this function will never be called
        // If the filter returned true, this function would be called on the IOWorkLoop
        return;
}
bool XonarAudioEngine::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *src)
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#endif
    
    //it's better to avoid doing interrupt handleing here, let's just use this function for what it's made to do.
    
        XonarAudioEngine *callingInstance = OSDynamicCast(XonarAudioEngine, owner);
    
        if (callingInstance)
            callingInstance->filterInterrupt(src->getIntIndex());
        
        return false;
}


void XonarAudioEngine::filterInterrupt(int index)
{
        // In the case of our simple device, we only get interrupts when the audio engine loops to the
        // beginning of the buffer.  When that happens, we need to take a timestamp and increment
        // the loop count.  The function takeTimeStamp() does both of those for us.  Additionally,
        // if a different timestamp is to be used (other than the current time), it can be passed
        // in to takeTimeStamp()
    
    struct oxygen *chip = chipData;
    unsigned int status, clear, elapsed_streams, i;
    
    status = oxygen_read16(chip, OXYGEN_INTERRUPT_STATUS);
#if DEBUG
    kprintf("XonarAudioEngine::%-30s status:%s\n", __func__, status? "true": "false" );
#endif
    if (!status)
        return;
    
    IOSimpleLockLock(chip->reg_lock);
    
    clear = status & (OXYGEN_CHANNEL_A |
                      OXYGEN_CHANNEL_B |
                      OXYGEN_CHANNEL_C |
                      OXYGEN_CHANNEL_SPDIF |
                      OXYGEN_CHANNEL_MULTICH |
                      OXYGEN_CHANNEL_AC97 |
                      OXYGEN_INT_SPDIF_IN_DETECT |
                      OXYGEN_INT_GPIO |
                      OXYGEN_INT_AC97);
    if (clear) {
        if (clear & OXYGEN_INT_SPDIF_IN_DETECT)
            chip->interrupt_mask &= ~OXYGEN_INT_SPDIF_IN_DETECT;
        oxygen_write16(chip, OXYGEN_INTERRUPT_MASK,
                       chip->interrupt_mask & ~clear);
        oxygen_write16(chip, OXYGEN_INTERRUPT_MASK,
                       chip->interrupt_mask);
    }
    
    elapsed_streams = status & chip->pcm_running;
#if DEBUG
    kprintf("XonarAudioEngine::%-30s status is 0x%08x,"
            " and pcm_running is 0x%08x\n", __func__, status, chip->pcm_running );
#endif
    IOSimpleLockUnlock(chip->reg_lock);
    /* Not sure if we need this loop since OSX handles PCM very differently
     *from Linux/ALSA. commenting out for now since there is no equivalent of
     *snd_pcm_period_elapsed (updates the position and tells us if we've run over
     *edit: seems like we'll need the elapsed_streams/snd_pcm_period collapsed to
     * go in
     */
    // for (i = 0; i < PCM_COUNT; ++i)
    //       if ((elapsed_streams & (1 << i)) && chip->streams[i])
    //         snd_pcm_period_elapsed(chip->streams[i]);
    
    if (status & OXYGEN_INT_SPDIF_IN_DETECT) {
        IOSimpleLockLock(chip->reg_lock);
        i = oxygen_read32(chip, OXYGEN_SPDIF_CONTROL);
        if (i & (OXYGEN_SPDIF_SENSE_INT | OXYGEN_SPDIF_LOCK_INT |
                 OXYGEN_SPDIF_RATE_INT)) {
            /* write the interrupt bit(s) to clear */
            oxygen_write32(chip, OXYGEN_SPDIF_CONTROL, i);
            //Linux Call below:
            //    schedule_work(&chip->spdif_input_bits_work);
            //Experimental OSX-equivalent call below...
            /* Conceptually i *think* this function is the IOWorkLoop context,
             * so by using the dynamic cast (thanks osxbook), i hope to recover
             * the calling (single) class, and then use its member functions to perform
             * the work. in this case, checking the spdif bits*/
#if DEBUG
            kprintf("XonarAudioEngine::%-30s status & OXYGEN_INT_SPDIF_IN true. calling oxygen_spdif_input_bits_changed (or so i hope)\n", __func__);
#endif
            workLoop->runAction((Action)oxygen_spdif_input_bits_changed, this, dev_id);
            
        }
        IOSimpleLockUnlock(chip->reg_lock);
    }
    
    if (status & OXYGEN_INT_GPIO) {
#if DEBUG
        kprintf("XonarAudioEngine::%-30s status & OXYGEN_INT_GPIO true. calling oxygen_gpio_changed (or so i hope)\n", __func__);
#endif
        //Linux call below:
        //schedule_work(&chip->gpio_work);
        //Experimental OSX-equivalent (see note above):
        workLoop->runAction((Action)oxygen_gpio_changed, this, dev_id);
    }
    /* Commentary on substituting schedule_work with runAction:
     * This *seems* to make sense, no? runAction runs in a single-threaded context w/in the handler.
     * for OSX it seems we don't even need work_queues because:
     * 1. each work_queue instance (gpio_work, spdif_input_bits_work) are used within their associated
     *    functions (gpio_work,spdif_input_bits_changed) to cast the data back to (struct oxygen *)
     *      -since this OSX driver is OOP, we have access to (struct oxygen*) via dev_id and so
     *       we don't need pass such structures to the handling functions as we have access.
     * 2. each of these workqueues seem to schedule work only within the handler, meaning
     *    we should be able to get away with runAction, since there is no other point in the entire
     *    driver where we schedule_work for either the gpio or spdif bits.
     ****
     * i could be totally wrong, but i hope i'm not. runAction would be a superior abstraction if it
     * can replace our need to instantiate "sub"workqueues whose work is added/handled inside the main
     * interrupt handler....
     * i hope it does what we need...
     */
    if (status & OXYGEN_INT_MIDI) {
#if DEBUG
        kprintf("XonarAudioEngine::%-30s status & OXYGEN_INT_MIDI true. calling oxygen_read_uart (or so i hope)\n", __func__);
#endif
        //before i even think about the read/write functions and worrying about shuttling data
        //back and forth with the hardware, i need to find the best way to handle the interrupt
        //with APPUL's api
        //if (chip->midi)
        //     MIDIGetDriverIORunLoop();
        //     _snd_mpuint401_t_uart_interrupt(0, chip->midi->private_data);
        //  else
        oxygen_read_uart(chip);
    }
    
    if (status & OXYGEN_INT_AC97) {
#if DEBUG
        kprintf("XonarAudioEngine::%-30s status & OXYGEN_INT_AC97 true. checking ac97_statusbits (or so i hope)\n", __func__);
#endif
        chip->ac97_statusbits = 0;
        IOLockWakeup(chip->ac97_mutex, &chip->ac97_statusbits, true);
        //thread_wakeup_prim((event_t)chip->ac97_statusbits,0,THREAD_AWAKENED);
    }
    
        takeTimeStamp();
}




unsigned int XonarAudioEngine::upmix_item_count(struct oxygen *chip)
{
        if (chip->model.dac_channels_pcm < 8)
                return 2;
        else if (chip->model.update_center_lfe_mix)
                return 5;
        else
                return 3;
}

void XonarAudioEngine::oxygen_update_dac_routing(struct oxygen *chip)
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#endif
        /* DAC 0: front, DAC 1: surround, DAC 2: center/LFE, DAC 3: back */
        static const unsigned int reg_values[5] = {
                /* stereo -> front */
                (0 << OXYGEN_PLAY_DAC0_SOURCE_SHIFT) |
                (1 << OXYGEN_PLAY_DAC1_SOURCE_SHIFT) |
                (2 << OXYGEN_PLAY_DAC2_SOURCE_SHIFT) |
                (3 << OXYGEN_PLAY_DAC3_SOURCE_SHIFT),
                /* stereo -> front+surround */
                (0 << OXYGEN_PLAY_DAC0_SOURCE_SHIFT) |
                (0 << OXYGEN_PLAY_DAC1_SOURCE_SHIFT) |
                (2 << OXYGEN_PLAY_DAC2_SOURCE_SHIFT) |
                (3 << OXYGEN_PLAY_DAC3_SOURCE_SHIFT),
                /* stereo -> front+surround+back */
                (0 << OXYGEN_PLAY_DAC0_SOURCE_SHIFT) |
                (0 << OXYGEN_PLAY_DAC1_SOURCE_SHIFT) |
                (2 << OXYGEN_PLAY_DAC2_SOURCE_SHIFT) |
                (0 << OXYGEN_PLAY_DAC3_SOURCE_SHIFT),
                /* stereo -> front+surround+center/LFE */
                (0 << OXYGEN_PLAY_DAC0_SOURCE_SHIFT) |
                (0 << OXYGEN_PLAY_DAC1_SOURCE_SHIFT) |
                (0 << OXYGEN_PLAY_DAC2_SOURCE_SHIFT) |
                (3 << OXYGEN_PLAY_DAC3_SOURCE_SHIFT),
                /* stereo -> front+surround+center/LFE+back */
                (0 << OXYGEN_PLAY_DAC0_SOURCE_SHIFT) |
                (0 << OXYGEN_PLAY_DAC1_SOURCE_SHIFT) |
                (0 << OXYGEN_PLAY_DAC2_SOURCE_SHIFT) |
                (0 << OXYGEN_PLAY_DAC3_SOURCE_SHIFT),
        };
        UInt8 channels;
        unsigned int reg_value;
        channels = oxygen_read8(chip, OXYGEN_PLAY_CHANNELS) &
        OXYGEN_PLAY_CHANNELS_MASK;
        if (channels == OXYGEN_PLAY_CHANNELS_2)
                reg_value = reg_values[chip->dac_routing];
        
        else if (channels == OXYGEN_PLAY_CHANNELS_8)
        /* in 7.1 mode, "rear" channels go to the "back" jack */
                reg_value = (0 << OXYGEN_PLAY_DAC0_SOURCE_SHIFT) |
                (3 << OXYGEN_PLAY_DAC1_SOURCE_SHIFT) |
                (2 << OXYGEN_PLAY_DAC2_SOURCE_SHIFT) |
                (1 << OXYGEN_PLAY_DAC3_SOURCE_SHIFT);
        
        else
                reg_value = (0 << OXYGEN_PLAY_DAC0_SOURCE_SHIFT) |
                (1 << OXYGEN_PLAY_DAC1_SOURCE_SHIFT) |
                (2 << OXYGEN_PLAY_DAC2_SOURCE_SHIFT) |
                (3 << OXYGEN_PLAY_DAC3_SOURCE_SHIFT);
        
        if (chip->model.adjust_dac_routing) {
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s before adjust_dac_routing\n", " ");
#endif
                reg_value = chip->model.adjust_dac_routing(chip, reg_value);
        }
        oxygen_write16_masked(chip, OXYGEN_PLAY_ROUTING, reg_value,
                              OXYGEN_PLAY_DAC0_SOURCE_MASK |
                              OXYGEN_PLAY_DAC1_SOURCE_MASK |
                              OXYGEN_PLAY_DAC2_SOURCE_MASK |
                              OXYGEN_PLAY_DAC3_SOURCE_MASK);
        if (chip->model.update_center_lfe_mix) {
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s before update_center_lfe_mix\n", " ");
#endif
                chip->model.update_center_lfe_mix(chip, chip->dac_routing > 2);
        }
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        
}


int XonarAudioEngine::oxygen_hw_params(IOAudioStream *substream, int formatChange, UInt32 channel,
                                       const IOAudioStreamFormat *newFormat, const IOAudioStreamFormatExtension *newFormatEx)
{
        
        
        const IOAudioStreamFormatExtension *formatEx;
        int direction = substream->getDirection();
        
        UInt32 address;
        if(direction)  {
                if(channel == PCM_B)
                        address = inputStream->buffer->getPhysicalAddress();
                else
                        address = inputStream2->buffer->getPhysicalAddress();
        }
        else
                address = outputStream->buffer->getPhysicalAddress();
        
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#if DEBUGLEVEL > 2
        kprintf("XonarAudioEngine::%-30s before assignment of channel and formatEx\n", " ");
#endif
#endif
        if(formatChange){
                //       channel = newFormat->fDriverTag;
                formatEx = newFormatEx;
        }
        else {
                //     channel = substream->getFormat()->fDriverTag;
                formatEx = substream->getFormatExtension();
        }


        //kprintf("XonarAudioEngine::%-30s try to print the address anyways, even though it probably will cause a crash: %llx\n", __func__,
        //        getSampleBufferAddress(channel, substream->getDirection()) );
        
        //memory should have already been allocated.
        //err = snd_pcm_lib_malloc_pages(substream,
        //                               substream->getSampleBufferSize());
        
        
        //no matter what, since clemens sets SNDRV_PCM_HW_PARAM_PERIOD_BYTES to 32 in oxygen_open,
        //then params_period_bytes(hw_params) will always be 32. since we have no such function
        //to pull the period size to make it look clean, just use member of audiostreamformat
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s before buffer read\n", " ");
#endif
        oxygen_write32(chipData, channel_base_registers[channel],
                       address);
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s after buffer read\n", " ");
        kprintf("XonarAudioEngine::%-30s channel is %d, substream->getSampleBufferSize %d, formatEx->fBytesPerPacket %d\n",
                " ", channel, substream->getSampleBufferSize(), formatEx->fBytesPerPacket);
        kprintf("XonarAudioEngine::%-30s channel_base_registers[channel] = 0x%08x, address retrieved: 0x%08x\n",
                " ", channel_base_registers[channel], oxygen_read32(chipData, channel_base_registers[channel]));
#endif
        
        if (channel == PCM_MULTICH) {
                oxygen_write32(chipData, OXYGEN_DMA_MULTICH_COUNT,
                               substream->getSampleBufferSize() / 4 - 1);
                oxygen_write32(chipData, OXYGEN_DMA_MULTICH_TCOUNT,
                               formatEx->fBytesPerPacket / 4 - 1);
        } else {
                oxygen_write16(chipData, channel_base_registers[channel] + 4,
                               substream->getSampleBufferSize() / 4 - 1);
                oxygen_write16(chipData, channel_base_registers[channel] + 6,
                               formatEx->fBytesPerPacket / 4 - 1);
        }
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        return 0;
}

static UInt16 get_mclk(struct oxygen *chip, unsigned int channel,
                       const IOAudioSampleRate *newSampleRate)
{
        unsigned int mclks, shift, setRate;
        
        setRate = newSampleRate->whole;
        
        if (channel == PCM_MULTICH)
                mclks = chip->model.dac_mclks;
        else
                mclks = chip->model.adc_mclks;
        
        if (setRate <= 48000)
                shift = 0;
        else if (setRate <= 96000)
                shift = 2;
        else
                shift = 4;
        
        return OXYGEN_I2S_MCLK(mclks >> shift);
}


int XonarAudioEngine::oxygen_rec_a_hw_params(IOAudioStream *substream, int formatChange,
                                             const IOAudioStreamFormat *newFormat, const IOAudioStreamFormatExtension *newFormatEx,
                                             const IOAudioSampleRate *newSampleRate)
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#if DEBUGLEVEL > 2
        kprintf("XonarAudioEngine::%-30s substream->getFormat()->fDriverTag:%d%-6s" "substream->getFormat():%p,\n"
                "XonarAudioEngine::%-30s engine->getSampleRate()->whole:%d%-10s" "engine->getSampleRate()->fraction:%d\n",
                " ", substream->getFormat()->fDriverTag, " ", substream->getFormat(),
                " ", getSampleRate()->whole, " ", getSampleRate()->fraction);
#endif
#endif
        if(formatChange){
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s newFormat->fDriverTag:%d\n", " ", newFormat->fDriverTag);
#endif
                if(!newSampleRate)
                        // if it's null, it's probs cause we called performFormatChange
                        // upon setting the sample format for the first time. so we
                        // use ol' reliable, aka AC NINE SEVEN
                        newSampleRate = &ac97_sampleRate;
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s newSampleRate->whole:%d%-10s fraction:%d\n", " ",
                        newSampleRate->whole, " ", newSampleRate->fraction);
#endif
        }
        
        int err;
        
        err = oxygen_hw_params(substream, formatChange, PCM_A, newFormat, newFormatEx);
        
        if (err < 0)
                return err;
        
        IOSimpleLockLock(chipData->reg_lock);
        oxygen_write8_masked(chipData, OXYGEN_REC_FORMAT,
                             oxygen_format(formatChange ? newFormat : substream->getFormat()) << OXYGEN_REC_FORMAT_A_SHIFT,
                             OXYGEN_REC_FORMAT_A_MASK);
        oxygen_write16_masked(chipData, OXYGEN_I2S_A_FORMAT,
                              oxygen_rate(formatChange ? newSampleRate
                                          : getSampleRate())  |
                              chipData->model.adc_i2s_format |
                              get_mclk(chipData, PCM_A, formatChange ? newSampleRate :
                                       getSampleRate()) |
                              oxygen_i2s_bits(formatChange ? newFormat : substream->getFormat()),
                              OXYGEN_I2S_RATE_MASK |
                              OXYGEN_I2S_FORMAT_MASK |
                              OXYGEN_I2S_MCLK_MASK |
                              OXYGEN_I2S_BITS_MASK);
        IOSimpleLockUnlock(chipData->reg_lock);
        
        IOLockLock(chipData->mutex);
        chipData->model.set_adc_params(chipData, this);
        IOLockUnlock(chipData->mutex);
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        
        return 0;
}

int XonarAudioEngine::oxygen_rec_b_hw_params(IOAudioStream *substream, int formatChange,
                                             const IOAudioStreamFormat *newFormat, const IOAudioStreamFormatExtension *newFormatEx,
                                             const IOAudioSampleRate *newSampleRate)
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#if DEBUGLEVEL > 2
        kprintf("XonarAudioEngine::%-30s substream->getFormat()->fDriverTag:%d%-6s" "substream->getFormat():%p,\n"
                "XonarAudioEngine::%-30s engine->getSampleRate()->whole:%d%-10s" "engine->getSampleRate()->fraction:%d\n",
                " ", substream->getFormat()->fDriverTag, " ", substream->getFormat(),
                " ", getSampleRate()->whole, " ", getSampleRate()->fraction);
#endif
#endif
        if(formatChange){
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s newFormat->fDriverTag:%d\n", " ", newFormat->fDriverTag);
#endif
                if(!newSampleRate)
                        // if it's null, it's probs cause we called performFormatChange
                        // upon setting the sample format for the first time. so we
                        // use ol' reliable, aka AC NINE SEVEN
                        newSampleRate = &ac97_sampleRate;
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s newSampleRate->whole:%d%-10s fraction:%d\n", " ",
                        newSampleRate->whole, " ", newSampleRate->fraction);
#endif
        }
        
        int is_ac97;
        int err;
        
        err = oxygen_hw_params(substream, formatChange, PCM_B, newFormat, newFormatEx);
        if (err < 0)
                return err;
        
        is_ac97 = chipData->has_ac97_1 &&
        (chipData->model.device_config & CAPTURE_2_FROM_AC97_1);
        
        IOSimpleLockLock(chipData->reg_lock);
        oxygen_write8_masked(chipData, OXYGEN_REC_FORMAT,
                             oxygen_format(formatChange ? newFormat : substream->getFormat()) << OXYGEN_REC_FORMAT_B_SHIFT,
                             OXYGEN_REC_FORMAT_B_MASK);
        if (!is_ac97)
                oxygen_write16_masked(chipData, OXYGEN_I2S_B_FORMAT,
                                      oxygen_rate(newFormat ? newSampleRate :
                                                  getSampleRate()) |
                                      chipData->model.adc_i2s_format |
                                      get_mclk(chipData, PCM_B, formatChange ? newSampleRate :
                                               getSampleRate()) |
                                      oxygen_i2s_bits(formatChange ? newFormat : substream->getFormat()),
                                      OXYGEN_I2S_RATE_MASK |
                                      OXYGEN_I2S_FORMAT_MASK |
                                      OXYGEN_I2S_MCLK_MASK |
                                      OXYGEN_I2S_BITS_MASK);
        IOSimpleLockUnlock(chipData->reg_lock);
        
        if (!is_ac97) {
                IOLockLock(chipData->mutex);
                chipData->model.set_adc_params(chipData, this);
                IOLockUnlock(chipData->mutex);
        }
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        
        return 0;
}

int XonarAudioEngine::oxygen_rec_c_hw_params(IOAudioStream *substream, int formatChange,
                                             const IOAudioStreamFormat *newFormat, const IOAudioStreamFormatExtension *newFormatEx,
                                             const IOAudioSampleRate *newSampleRate)
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#if DEBUGLEVEL > 2
        kprintf("XonarAudioEngine::%-30s substream->getFormat()->fDriverTag:%d%-6s" "substream->getFormat():%p,\n"
                "XonarAudioEngine::%-30s engine->getSampleRate()->whole:%d%-10s" "engine->getSampleRate()->fraction:%d\n",
                " ", substream->getFormat()->fDriverTag, " ", substream->getFormat(),
                " ", getSampleRate()->whole, " ", getSampleRate()->fraction);
#endif
#endif
        if(formatChange){
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s newFormat->fDriverTag:%d\n", " ", newFormat->fDriverTag);
#endif
                if(!newSampleRate)
                        // if it's null, it's probs cause we called performFormatChange
                        // upon setting the sample format for the first time. so we
                        // use ol' reliable, aka AC NINE SEVEN
                        newSampleRate = &ac97_sampleRate;
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s newSampleRate->whole:%d%-10s fraction:%d\n", " ",
                        newSampleRate->whole, " ", newSampleRate->fraction);
#endif
        }
        
        
        bool is_spdif;
        int err;
        
        err = oxygen_hw_params(substream, formatChange, PCM_C, newFormat, newFormatEx);
        if (err < 0)
                return err;
        
        is_spdif = chipData->model.device_config & CAPTURE_1_FROM_SPDIF;
        
        IOSimpleLockLock(chipData->reg_lock);
        oxygen_write8_masked(chipData, OXYGEN_REC_FORMAT,
                             oxygen_format(formatChange ? newFormat : substream->getFormat()) << OXYGEN_REC_FORMAT_C_SHIFT,
                             OXYGEN_REC_FORMAT_C_MASK);
        if (!is_spdif)
                oxygen_write16_masked(chipData, OXYGEN_I2S_C_FORMAT,
                                      oxygen_rate(newFormat ? newSampleRate :
                                                  getSampleRate()) |
                                      chipData->model.adc_i2s_format |
                                      get_mclk(chipData, PCM_B, formatChange ? newSampleRate :
                                               getSampleRate()) |
                                      oxygen_i2s_bits(formatChange ? newFormat : substream->getFormat()),
                                      OXYGEN_I2S_RATE_MASK |
                                      OXYGEN_I2S_FORMAT_MASK |
                                      OXYGEN_I2S_MCLK_MASK |
                                      OXYGEN_I2S_BITS_MASK);
        IOSimpleLockUnlock(chipData->reg_lock);
        
        if (!is_spdif) {
                IOLockLock(chipData->mutex);
                chipData->model.set_adc_params(chipData, this);
                IOLockUnlock(chipData->mutex);
        }
        
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        return 0;
}


int XonarAudioEngine::oxygen_multich_hw_params(IOAudioStream *substream, int formatChange,
                                               const IOAudioStreamFormat *newFormat, const IOAudioStreamFormatExtension *newFormatEx,
                                               const IOAudioSampleRate *newSampleRate)
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN\n", __func__);
#if DEBUGLEVEL > 2
        kprintf("XonarAudioEngine::%-30s substream->getFormat()->fDriverTag:%d%-6s" "substream->getFormat():%p,\n"
                "XonarAudioEngine::%-30s engine->getSampleRate()->whole:%d%-10s" "engine->getSampleRate()->fraction:%d\n",
                " ", substream->getFormat()->fDriverTag, " ", substream->getFormat(),
                " ", getSampleRate()->whole, " ", getSampleRate()->fraction);
#endif
#endif
        if(formatChange){
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s newFormat->fDriverTag:%d\n", " ", newFormat->fDriverTag);
#endif
                if(!newSampleRate)
                        // if it's null, it's probs cause we called performFormatChange
                        // upon setting the sample format for the first time. so we
                        // use ol' reliable, aka AC NINE SEVEN
                        newSampleRate = &ac97_sampleRate;
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarAudioEngine::%-30s newSampleRate->whole:%d%-10s fraction:%d\n", " ",
                        newSampleRate->whole, " ", newSampleRate->fraction);
#endif
        }
        
        int err;
        err = oxygen_hw_params(substream, formatChange, PCM_MULTICH, newFormat, newFormatEx);
        if (err < 0)
                return err;
        
        IOLockLock(chipData->mutex);
        IOSimpleLockLock(chipData->reg_lock);
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s before play_channels write\n", " ");
#endif
        oxygen_write8_masked(chipData, OXYGEN_PLAY_CHANNELS,
                             oxygen_play_channels(formatChange ? newFormat : substream->getFormat()),
                             OXYGEN_PLAY_CHANNELS_MASK);
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s before play_format write\n", " ");
#endif
        oxygen_write8_masked(chipData, OXYGEN_PLAY_FORMAT,
                             oxygen_format(formatChange ? newFormat : substream->getFormat()) << OXYGEN_MULTICH_FORMAT_SHIFT,
                             OXYGEN_MULTICH_FORMAT_MASK);
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s before i2s_multich_format write\n", " ");
#endif
        oxygen_write16_masked(chipData, OXYGEN_I2S_MULTICH_FORMAT,
                              oxygen_rate(newFormat ? newSampleRate :
                                          getSampleRate()) |
                              chipData->model.dac_i2s_format |
                              get_mclk(chipData, PCM_MULTICH, formatChange ? newSampleRate :
                                       getSampleRate()) |
                              oxygen_i2s_bits(formatChange ? newFormat : substream->getFormat()),
                              OXYGEN_I2S_RATE_MASK |
                              OXYGEN_I2S_FORMAT_MASK |
                              OXYGEN_I2S_MCLK_MASK |
                              OXYGEN_I2S_BITS_MASK);
        
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s before update_spdif_source write\n", " ");
#endif
        spdifEngine->oxygen_update_spdif_source(chipData);
        IOSimpleLockUnlock(chipData->reg_lock);
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s before set_dac_params\n", " ");
#endif
        chipData->model.set_dac_params(chipData, this, substream);
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarAudioEngine::%-30s before update_dac_routing\n", " ");
#endif
        oxygen_update_dac_routing(chipData);
        IOLockUnlock(chipData->mutex);
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        return 0;
}

void XonarAudioEngine::oxygen_hw_free(unsigned int channel)
{
        //struct oxygen *chip = ((XonarAudioEngine *)substream->audioEngine)->chipData; //snd_pcm_substream_chip(substream);
        //unsigned int channel = substream->getFormat()->fDriverTag;//oxygen_substream_channel(substream);
        unsigned int channel_mask = 1 << channel;
        
        IOSimpleLockLock(chipData->reg_lock);
        chipData->interrupt_mask &= ~channel_mask;
        oxygen_write16(chipData, OXYGEN_INTERRUPT_MASK, chipData->interrupt_mask);
        
        oxygen_set_bits8(chipData, OXYGEN_DMA_FLUSH, channel_mask);
        oxygen_clear_bits8(chipData, OXYGEN_DMA_FLUSH, channel_mask);
        IOSimpleLockUnlock(chipData->reg_lock);
        //((XonarAudioEngine*)substream->audioEngine)->free();
        //    return snd_pcm_lib_free_pages(substream);
}

int XonarAudioEngine::oxygen_trigger () {
        //        //i think the trigger function should go here
        //        struct oxygen *chip = snd_pcm_substream_chip(substream);
        //        struct snd_pcm_substream *s;
        long cmd = (long) state;
        int pausing;
#if DEBUG
        kprintf("XonarAudioEngine:%-30s enginestatus is %s\n", __func__, getStringStatus(state));
#endif
        switch (cmd) {
                case kIOAudioEngineResumed:
                case kIOAudioEngineRunning:
                        pausing = 0;
                        break;
                case kIOAudioEngineStopped:
                case kIOAudioEnginePaused:
                        pausing = 1;
                        break;
                default:
                        return -EINVAL;
                        
                        //                case SNDRV_PCM_TRIGGER_STOP:
                        //                case SNDRV_PCM_TRIGGER_START:
                        //                case SNDRV_PCM_TRIGGER_SUSPEND:
                        //                        pausing = 0;
                        //                        break;
                        //                case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
                        //                case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
                        //                        pausing = 1;
                        //                        break;
                        //                default:
                        //                        return -EINVAL;
        }
        //
        //        snd_pcm_group_for_each_entry(s, substream) {
        //                if (snd_pcm_substream_chip(s) == chip) {
        for(int i=0; i< PCM_COUNT; i++) {
                if((1 << i) & channel_mask) {
#if DEBUG
                        kprintf("XonarAudioEngine::%-30s adding stream %d to the pcm_running mask\n",
                                __func__, i);
#endif
                        unsigned int mask = 0;
                        mask |= 1 << i;
                        //                        snd_pcm_trigger_done(s, substream);
                        //                }
                        //        }
                        
                        //i am sure some of the wrapping from SNDRV_PCM_TRIGGER_* will require
                        //tweaking to match kIOEngineStatus stuff.
                        
                        //really, what we want is pcm_running to have the right mask by the time
                        //we call the interrupt handler.
                        IOSimpleLockLock(chipData->reg_lock);
                        if (!pausing) {
                                if (cmd == kIOAudioEngineRunning) //SNDRV_PCM_TRIGGER_START
                                        chipData->pcm_running |= mask;
                                else
                                        chipData->pcm_running &= ~mask;
                                oxygen_write8(chipData, OXYGEN_DMA_STATUS, chipData->pcm_running);
                        } else {
                                if (cmd == kIOAudioEnginePaused) //SNDRV_PCM_TRIGGER_PAUSE_PUSH)
                                        oxygen_set_bits8(chipData, OXYGEN_DMA_PAUSE, mask);
                                else
                                        oxygen_clear_bits8(chipData, OXYGEN_DMA_PAUSE, mask);
                        }
                        IOSimpleLockUnlock(chipData->reg_lock);
                        
                }
        }
        
        return 0;
        //}
        
        
        
}
void XonarAudioEngine::oxygen_spdif_hw_free()
{
        //struct oxygen *chip = ((XonarAudioEngine *)substream->audioEngine)->chipData; //snd_pcm_substream_chip(substream);
        
        IOSimpleLockLock(chipData->reg_lock);
        oxygen_clear_bits32(chipData, OXYGEN_SPDIF_CONTROL,
                            OXYGEN_SPDIF_OUT_ENABLE);
        IOSimpleLockUnlock(chipData->reg_lock);
        return oxygen_hw_free(PCM_SPDIF);
}

int XonarAudioEngine::oxygen_prepare(unsigned int channel)
{
#if DEBUG
        kprintf("XonarAudioEngine::%-30s BEGIN (channel %d)\n", __func__, channel);
#endif
        
        
        unsigned int channel_mask = 1 << channel;
        
        IOSimpleLockLock(chipData->reg_lock);
        oxygen_set_bits8(chipData, OXYGEN_DMA_FLUSH, channel_mask);
        oxygen_clear_bits8(chipData, OXYGEN_DMA_FLUSH, channel_mask);
        
        //gonna have to figure out the conditional equivalent to the line below.
        /*    if (substream->runtime->no_period_wakeup)
         chip->interrupt_mask &= ~channel_mask;
         else*/
        chipData->interrupt_mask |= channel_mask;
        oxygen_write16(chipData, OXYGEN_INTERRUPT_MASK, chipData->interrupt_mask);
        IOSimpleLockUnlock(chipData->reg_lock);
#if DEBUG
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        
        return 0;
}
