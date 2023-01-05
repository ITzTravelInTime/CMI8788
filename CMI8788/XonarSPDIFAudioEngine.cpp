/*
 File:XonarSPDIFAudioEngine.cpp
 
 Contains:
 
 Version:1.0.0
 
 Copyright:Copyright ) 1997-2010 by Apple Computer, Inc., All Rights Reserved.
 
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
#include <IOKit/IOLib.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/audio/IOAudioBlitterLibDispatch.h>


#include "PCIAudioDevice.hpp"
#include "XonarAudioEngine.hpp"
#include "XonarSPDIFAudioEngine.hpp"
#include "oxygen.h"
#include "oxygen_regs.h"
#include "alsa.h"

#define INITIAL_SAMPLE_RATE	48000
#define NUM_SAMPLE_FRAMES	16384
#define NUM_CHANNELS		2
#define BIT_DEPTH			16

#define BUFFER_SIZE			(NUM_SAMPLE_FRAMES * NUM_CHANNELS * BIT_DEPTH / 8)

#define super IOAudioEngine

OSDefineMetaClassAndStructors(XonarSPDIFAudioEngine, IOAudioEngine)

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

bool XonarSPDIFAudioEngine::init(XonarAudioEngine *instance)
{
        bool result = false;
        inputStream = NULL;
        outputStream = NULL;
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s BEGIN\n", __func__);
#endif
        if (!instance) {
                goto Done;
        }
        
        if (!super::init(NULL)) {
                goto Done;
        }
        
        engineInstance = instance;
        //deviceRegisters = regs;
        
        result = true;
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s END\n", __func__);
#endif
        
Done:
        
        return result;
}

bool XonarSPDIFAudioEngine::initHardware(IOService *provider)
{
        bool result = false;
        IOAudioSampleRate initialSampleRate;
        IOWorkLoop *workLoop;
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s BEGIN\n", __func__);
#endif
        if (!super::initHardware(provider)) {
                goto Done;
        }
        int ins, outs;
        //can't activate an audio engine inside another one. so we assign it in the main device.
        //engineInstance = ((PCIAudioDevice*)provider)->accessibleEngineInstance;
        outs = !!(engineInstance->chipData->model.device_config & PLAYBACK_1_TO_SPDIF);
        ins = !!(engineInstance->chipData->model.device_config & CAPTURE_1_FROM_SPDIF);
        if (outs | ins) {
                //err = snd_pcm_new(chip->card, "Digital", 1, outs, ins, &pcm);
                //if (err < 0)
                //    return err;
                if (outs) {
                        //add spdif_ops fns
                        outputStream = createAudioStream(kIOAudioStreamDirectionOutput, DEFAULT_BUFFER_BYTES);
                        if (!outputStream->stream) {
                                goto Done;
                        }
                        outputStream->stream->setName("SPDIF OUT");
                        addAudioStream(outputStream->stream);
                        outputStream->stream->release();
                }
                if (ins) {
                        //add rc_c_ops fns
                        inputStream = createAudioStream(kIOAudioStreamDirectionInput, DEFAULT_BUFFER_BYTES);
                        if (!inputStream->stream) {
                                goto Done;
                        }
                        inputStream->stream->setName("SPDIF IN");
                        addAudioStream(inputStream->stream);
                        inputStream->stream->release();
                }
                
                //        pcm->private_data = chip;
                //        strcpy(pcm->name, "Digital");
                //        snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
                //                                              snd_dma_pci_data(chipData->pci),
                //                                              DEFAULT_BUFFER_BYTES,
                //                                              BUFFER_BYTES_MAX);
                
                
        }
        
        // Setup the initial sample rate for the audio engine
        initialSampleRate.whole = INITIAL_SAMPLE_RATE;
        initialSampleRate.fraction = 0;
        //
        setDescription("Xonar SPDIF Audio Engine");
        //
        setSampleRate(&initialSampleRate);
        //
        //    // Set the number of sample frames in each buffer
        setNumSampleFramesPerBuffer(NUM_SAMPLE_FRAMES);
        //
        //                workLoop = getWorkLoop();
        //                if (!workLoop) {
        //                        goto Done;
        //                }
        
        // Create an interrupt event source through which to receive interrupt callbacks
        // In this case, we only want to do work at primary interrupt time, so
        // we create an IOFilterInterruptEventSource which makes a filtering call
        // from the primary interrupt interrupt who's purpose is to determine if
        // our secondary interrupt handler is to be called.  In our case, we
        // can do the work in the filter routine and then return false to
        // indicate that we do not want our secondary handler called
        //                interruptEventSource = IOFilterInterruptEventSource::filterInterruptEventSource(this,
        //                                                                                                interruptHandler,
        //                                                                                                interruptFilter,
        //                                                                                                audioDevice->getProvider());
        //                if (!interruptEventSource) {
        //                        goto Done;
        //           }
        
        // In order to allow the interrupts to be received, the interrupt event source must be
        // added to the IOWorkLoop
        // Additionally, interrupts will not be firing until the interrupt event source is
        // enabled by calling interruptEventSource->enable() - this probably doesn't need to
        // be done until performAudioEngineStart() is called, and can probably be disabled
        // when performAudioEngineStop() is called and the audio engine is no longer running
        // Although this really depends on the specific hardware
        //           workLoop->addEventSource(interruptEventSource);
        
        
        
        result = true;
Done:
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s END\n", __func__);
#endif
        return result;
}

IOStream *XonarSPDIFAudioEngine::createAudioStream(IOAudioStreamDirection direction, UInt32 sampleBufferSize)
{
        //when calling createaudiostream, we are "in" oxygen_pcm_init.
        //this function emulates snd_pcm_new, because each function is creating a stream.
        IOStream *newStream = new IOStream;
        newStream->stream = new IOAudioStream;
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s BEGIN\n", __func__);
#endif
        //create the buffers.
        if(direction == kIOAudioStreamDirectionInput)
                newStream->buffer = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionIn | kIOMemoryPhysicallyContiguous, sampleBufferSize);
        else
                newStream->buffer = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionOut | kIOMemoryPhysicallyContiguous, sampleBufferSize);
        
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarSPDIFAudioEngine::%-25s buffer creation COMPLETE, calling oxygen_open, direction %d, sample buff address: 0x%08llx\n",
                " ", direction, newStream->buffer->getPhysicalAddress());
#endif
        // call oxygen_open
        newStream->stream->initWithAudioEngine(this, direction, 1);
        engineInstance->oxygen_open(newStream->stream, direction ? PCM_C : PCM_SPDIF);
        
        // call oxygen_*_hw_params
        //        if(direction == kIOAudioStreamDirectionInput) //PCM_C
        //                engineInstance->oxygen_rec_c_hw_params(audioStream, 0, NULL, NULL, NULL);
        //        else // SPDIF_HW_PARAMS
        //                oxygen_spdif_hw_params(audioStream, 0, NULL, NULL, NULL);
        //        else if(source == PCM_AC97)
        //                oxygen_hw_params(audioStream, 0, PCM_AC97, NULL, NULL);
        //        else if(source == PCM_SPDIF)
        //                spdifEngine->oxygen_spdif_hw_params(audioStream, 0, NULL, NULL, NULL);
        
        //per alsa documentation (https://www.kernel.org/doc/html/latest/sound/kernel-api/writing-an-alsa-driver.html#operators)
        //hw_params is only called when all of the info about the stream is known
        //however it seems like we can get away with it
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarSPDIFAudioEngine::%-25s setting sample buffer\n", " ");
#endif
        newStream->stream->setSampleBuffer(newStream->buffer->getBytesNoCopy(), newStream->buffer->getCapacity());
        
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarSPDIFAudioEngine::%-25s setting sample format\n", " ");
#endif
        //we shouldn't call the driver because the SPDIF and AC97
        //need the data from this call before they can do anything else.
        newStream->stream->setFormat(&oxygen_hardware[direction ? PCM_C : PCM_SPDIF], false);
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s END\n", __func__);
#endif
        //we use a mask variable to know what streams we have to stop
        engineInstance->channel_mask |= (1 << (direction? PCM_C : PCM_SPDIF));
        return newStream;
}


void XonarSPDIFAudioEngine::free()
{
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s BEGIN\n", __func__);
#endif
        // We need to free our resources when we're going away
        // just the event source. thassit.
        if (interruptEventSource) {
                interruptEventSource->release();
                interruptEventSource = NULL;
        }
        
        //master engine takes care of ALL cleanup
        if (inputStream) {
                if(inputStream->buffer)
                inputStream->buffer->IOGeneralMemoryDescriptor::free();
                
        }
        
        if (outputStream->buffer) {
                if(outputStream->buffer)
                outputStream->buffer->IOGeneralMemoryDescriptor::free();
        }
        
        super::free();
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s END\n", __func__);
#endif
}

void XonarSPDIFAudioEngine::stop(IOService *provider)
{
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s BEGIN\n", __func__);
#endif
        // When our device is being stopped and torn down, we should go ahead and remove
        // the interrupt event source from the IOWorkLoop
        // Additionally, we'll go ahead and release the interrupt event source since it isn't
        // needed any more
        if (interruptEventSource) {
                IOWorkLoop *wl;
                
                wl = getWorkLoop();
                if (wl) {
                        wl->removeEventSource(interruptEventSource);
                }
                
                interruptEventSource->release();
                interruptEventSource = NULL;
        }
        
        // Add code to shut down hardware (beyond what is needed to simply stop the audio engine)
        // There may be nothing needed here
        
        super::stop(provider);
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s END\n", __func__);
#endif
}

static unsigned int oxygen_spdif_rate(unsigned int oxygen_rate)
{
        switch (oxygen_rate) {
                case OXYGEN_RATE_32000:
                        return IEC958_AES3_CON_FS_32000 << OXYGEN_SPDIF_CS_RATE_SHIFT;
                case OXYGEN_RATE_44100:
                        return IEC958_AES3_CON_FS_44100 << OXYGEN_SPDIF_CS_RATE_SHIFT;
                default: /* OXYGEN_RATE_48000 */
                        return IEC958_AES3_CON_FS_48000 << OXYGEN_SPDIF_CS_RATE_SHIFT;
                case OXYGEN_RATE_64000:
                        return 0xb << OXYGEN_SPDIF_CS_RATE_SHIFT;
                case OXYGEN_RATE_88200:
                        return IEC958_AES3_CON_FS_88200 << OXYGEN_SPDIF_CS_RATE_SHIFT;
                case OXYGEN_RATE_96000:
                        return IEC958_AES3_CON_FS_96000 << OXYGEN_SPDIF_CS_RATE_SHIFT;
                case OXYGEN_RATE_176400:
                        return IEC958_AES3_CON_FS_176400 << OXYGEN_SPDIF_CS_RATE_SHIFT;
                case OXYGEN_RATE_192000:
                        return IEC958_AES3_CON_FS_192000 << OXYGEN_SPDIF_CS_RATE_SHIFT;
        }
}


void XonarSPDIFAudioEngine::oxygen_update_spdif_source(struct oxygen *chip)
{
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s START\n", __func__);
#endif
        UInt32 old_control, new_control;
        UInt16 old_routing, new_routing;
        unsigned int oxygen_rate;
        
        old_control = oxygen_read32(chip, OXYGEN_SPDIF_CONTROL);
        old_routing = oxygen_read16(chip, OXYGEN_PLAY_ROUTING);
        if (chip->pcm_active & (1 << PCM_SPDIF)) {
                new_control = old_control | OXYGEN_SPDIF_OUT_ENABLE;
                new_routing = (old_routing & ~OXYGEN_PLAY_SPDIF_MASK)
                | OXYGEN_PLAY_SPDIF_SPDIF;
                oxygen_rate = (old_control >> OXYGEN_SPDIF_OUT_RATE_SHIFT)
                & OXYGEN_I2S_RATE_MASK;
                /* S/PDIF rate was already set by the caller */
        } else if ((chip->pcm_active & (1 << PCM_MULTICH)) &&
                   chip->spdif_playback_enable) {
                new_routing = (old_routing & ~OXYGEN_PLAY_SPDIF_MASK)
                | OXYGEN_PLAY_SPDIF_MULTICH_01;
                oxygen_rate = oxygen_read16(chip, OXYGEN_I2S_MULTICH_FORMAT)
                & OXYGEN_I2S_RATE_MASK;
                new_control = (old_control & ~OXYGEN_SPDIF_OUT_RATE_MASK) |
                (oxygen_rate << OXYGEN_SPDIF_OUT_RATE_SHIFT) |
                OXYGEN_SPDIF_OUT_ENABLE;
        } else {
                new_control = old_control & ~OXYGEN_SPDIF_OUT_ENABLE;
                new_routing = old_routing;
                oxygen_rate = OXYGEN_RATE_44100;
        }
        if (old_routing != new_routing) {
                oxygen_write32(chip, OXYGEN_SPDIF_CONTROL,
                               new_control & ~OXYGEN_SPDIF_OUT_ENABLE);
                oxygen_write16(chip, OXYGEN_PLAY_ROUTING, new_routing);
        }
        if (new_control & OXYGEN_SPDIF_OUT_ENABLE)
                oxygen_write32(chip, OXYGEN_SPDIF_OUTPUT_BITS,
                               oxygen_spdif_rate(oxygen_rate) |
                               ((chip->pcm_active & (1 << PCM_SPDIF)) ?
                                chip->spdif_pcm_bits : chip->spdif_bits));
        oxygen_write32(chip, OXYGEN_SPDIF_CONTROL, new_control);
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s END\n", __func__);
#endif
}





int XonarSPDIFAudioEngine::oxygen_spdif_hw_params(IOAudioStream *substream, int formatChange,
                                                  const IOAudioStreamFormat *newFormat, const IOAudioStreamFormatExtension *newFormatEx,
                                                  const IOAudioSampleRate *newSampleRate)
{
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s BEGIN\n", __func__);
#if DEBUGLEVEL > 2
        kprintf("XonarSPDIFAudioEngine::%-25s substream->getFormat()->fDriverTag:%d%-6s" "substream->getFormat():%p,\n"
                "XonarSPDIFAudioEngine::%-25s engine->getSampleRate()->whole:%d%-10s" "engine->getSampleRate()->fraction:%d\n",
                " ", substream->getFormat()->fDriverTag, " ", substream->getFormat(),
                " ", getSampleRate()->whole, " ", getSampleRate()->fraction);
#endif
#endif
        if(formatChange){
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarSPDIFAudioEngine::%-25s newFormat->fDriverTag:%d\n", " ", newFormat->fDriverTag);
#endif
                if(!newSampleRate)
                        // if it's null, it's probs cause we called performFormatChange
                        // upon setting the sample format for the first time. so we
                        // use ol' reliable, aka AC NINE SEVEN
                        newSampleRate = &ac97_sampleRate;
#if DEBUG && (DEBUGLEVEL > 2)
                kprintf("XonarSPDIFAudioEngine::%-25s newSampleRate->whole:%d%-10s fraction:%d\n", " ",
                        newSampleRate->whole, " ", newSampleRate->fraction);
#endif
        }
        
        int err;
        
        //the only thing i can think of right now is to send the stream with its associated format,
        // and use that to set the hardware parameters. that is what we want to do, right? set the hardware up
        // for the format description of the current stream (????)
        err = engineInstance->oxygen_hw_params(substream, formatChange, PCM_SPDIF, newFormat, newFormatEx);
        
        if (err < 0)
                return err;
        
        IOLockLock(engineInstance->chipData->mutex);
        IOSimpleLockLock(engineInstance->chipData->reg_lock);
        oxygen_clear_bits32(engineInstance->chipData, OXYGEN_SPDIF_CONTROL,
                            OXYGEN_SPDIF_OUT_ENABLE);
        oxygen_write8_masked(engineInstance->chipData, OXYGEN_PLAY_FORMAT,
                             engineInstance->oxygen_format(formatChange ? newFormat : substream->getFormat()) << OXYGEN_SPDIF_FORMAT_SHIFT,
                             OXYGEN_SPDIF_FORMAT_MASK);
        oxygen_write32_masked(engineInstance->chipData, OXYGEN_SPDIF_CONTROL,
                              engineInstance->oxygen_rate(newFormat ? newSampleRate :
                                                          getSampleRate()) << OXYGEN_SPDIF_OUT_RATE_SHIFT,
                              OXYGEN_SPDIF_OUT_RATE_MASK);
        oxygen_update_spdif_source(engineInstance->chipData);
        IOSimpleLockUnlock(engineInstance->chipData->reg_lock);
        IOLockUnlock(engineInstance->chipData->mutex);
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s END\n", __func__);
#endif
        
        return 0;
}


IOReturn XonarSPDIFAudioEngine::performAudioEngineStart()
{
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s BEGIN\n", __func__);
#endif
        // The interruptEventSource needs to be enabled to allow interrupts to start firing
        assert(interruptEventSource);
        interruptEventSource->enable();
        
        // When performAudioEngineStart() gets called, the audio engine should be started from the beginning
        // of the sample buffer.  Because it is starting on the first sample, a new timestamp is needed
        // to indicate when that sample is being read from/written to.  The function takeTimeStamp()
        // is provided to do that automatically with the current time.
        // By default takeTimeStamp() will increment the current loop count in addition to taking the current
        // timestamp.  Since we are starting a new audio engine run, and not looping, we don't want the loop count
        // to be incremented.  To accomplish that, false is passed to takeTimeStamp().
        takeTimeStamp(false);
        
        // Add audio - I/O start code here
        //it's already started, bitch.
        //#error performAudioEngineStart() - driver will not work until audio engine start code is added
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s END\n", __func__);
#endif
        return kIOReturnSuccess;
}

IOReturn XonarSPDIFAudioEngine::performAudioEngineStop()
{
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s BEGIN\n", __func__);
#endif
        // Assuming we don't need interrupts after stopping the audio engine, we can disable them here
        assert(interruptEventSource);
        interruptEventSource->disable();
        
        // Add audio - I/O stop code here
        // all taken care of by XonarAudioEngine's stop
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s END\n", __func__);
#endif
        
        return kIOReturnSuccess;
}

UInt32 XonarSPDIFAudioEngine::getCurrentSampleFrame()
{
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s BEGIN\n", __func__);
#endif
        // In order for the erase process to run properly, this function must return the current location of
        // the audio engine - basically a sample counter
        // It doesn't need to be exact, but if it is inexact, it should err towards being before the current location
        // rather than after the current location.  The erase head will erase up to, but not including the sample
        // frame returned by this function.  If it is too large a value, sound data that hasn't been played will be
        // erased.
        
        UInt32 curr_addr;
        
        /* no spinlock, this read should be atomic */
        curr_addr = oxygen_read32(engineInstance->chipData, channel_base_registers[PCM_SPDIF]);
        return curr_addr - outputStream->buffer->getPhysicalAddress();
}

IOReturn XonarSPDIFAudioEngine::performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate)
{
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s BEGIN\n", __func__);
#endif
        // Since we only allow one format, we only need to be concerned with sample rate changes
        // In this case, we only allow 2 sample rates - 44100 & 48000, so those are the only ones
        // that we check for
        IOAudioStreamFormatExtension formatExDefault;
        formatExDefault.fBytesPerPacket = 32;
        formatExDefault.fFramesPerPacket = 1;
        if (newSampleRate) {
                
                if(audioStream->getDirection() == kIOAudioStreamDirectionInput) //input uses REC_C hw_params
                        engineInstance->oxygen_rec_c_hw_params(audioStream, 1, newFormat, &formatExDefault, newSampleRate);
                else // output stream uses spdif_hw_params
                        oxygen_spdif_hw_params(audioStream, 1, newFormat, &formatExDefault, newSampleRate);
                
                //                switch (newSampleRate->whole) {
                //                        case 44100:
                //                                IOLog("/t-> 44.1kHz selected\n");
                //
                //                                // Add code to switch hardware to 44.1khz
                //                                break;
                //                        case 48000:
                //                                IOLog("/t-> 48kHz selected\n");
                //
                //                                // Add code to switch hardware to 48kHz
                //                                break;
                //                        default:
                //                                // This should not be possible since we only specified 44100 and 48000 as valid sample rates
                //                                IOLog("/t Internal Error - unknown sample rate selected.\n");
                //                                break;
                //                }
        }
#if DEBUG
        kprintf("XonarSPDIFAudioEngine::%-25s END\n", __func__);
#endif
        return kIOReturnSuccess;
}

//void XonarSPDIFAudioEngine::interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count)
//{
//    // Since our interrupt filter always returns false, this function will never be called
//    // If the filter returned true, this function would be called on the IOWorkLoop
//    return;
//}
//
//bool XonarSPDIFAudioEngine::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source)
//{
//    XonarSPDIFAudioEngine *audioEngine = OSDynamicCast(XonarSPDIFAudioEngine, owner);
//    
//    // We've cast the audio engine from the owner which we passed in when we created the interrupt
//    // event source
//    if (audioEngine) {
//        // Then, filterInterrupt() is called on the specified audio engine
//        audioEngine->filterInterrupt(source->getIntIndex());
//    }
//    
//    return false;
//}
//
//void XonarSPDIFAudioEngine::filterInterrupt(int index)
//{
//    // In the case of our simple device, we only get interrupts when the audio engine loops to the
//    // beginning of the buffer.  When that happens, we need to take a timestamp and increment
//    // the loop count.  The function takeTimeStamp() does both of those for us.  Additionally,
//    // if a different timestamp is to be used (other than the current time), it can be passed
//    // in to takeTimeStamp()
//    takeTimeStamp();
//}


