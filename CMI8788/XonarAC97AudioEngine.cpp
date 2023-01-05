/*
 File:XonarAC97AudioEngine.cpp
 
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
#include <IOKit/audio/IOAudioEngine.h>
#include <IOKit/audio/IOAudioBlitterLibDispatch.h>


#include "XonarAC97AudioEngine.hpp"
#include "PCIAudioDevice.hpp"
#include "XonarAudioEngine.hpp"
#include "oxygen.h"
#include "oxygen_regs.h"
#define super IOAudioEngine

OSDefineMetaClassAndStructors(XonarAC97AudioEngine, IOAudioEngine)


#define INITIAL_SAMPLE_RATE        48000
#define NUM_SAMPLE_FRAMES        16384
#define NUM_CHANNELS                2
#define BIT_DEPTH                        16

#define BUFFER_SIZE                        (NUM_SAMPLE_FRAMES * NUM_CHANNELS * BIT_DEPTH / 8)

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


bool XonarAC97AudioEngine::init(XonarAudioEngine *instance)
{
        bool result = false;
        inputStream = NULL;
        outputStream = NULL;
        kprintf("XonarAC97AudioEngine::%-26s BEGIN\n", __func__);
        
        if (!instance) {
                goto Done;
        }
        
        if (!super::init(NULL)) {
                goto Done;
        }
        
        engineInstance = instance;
        
        result = true;
        
Done:
        
        return result;
}

bool XonarAC97AudioEngine::initHardware(IOService *provider)
{
        bool result = false;
        IOAudioSampleRate initialSampleRate;
        //    IOAudioStream *audioStream;
        IOWorkLoop *workLoop;
#if DEBUG
        kprintf("XonarAC97AudioEngine::%-26s BEGIN\n", __func__);
#endif
        if (!super::initHardware(provider)) {
                goto Done;
        }
        int ins, outs;
        //can't activate an audio engine inside another one. so we assign it in the main device.
        //engineInstance = ((PCIAudioDevice*)provider)->accessibleEngineInstance;
        
        if (engineInstance->chipData->has_ac97_1) {
                outs = !!(engineInstance->chipData->model.device_config & PLAYBACK_2_TO_AC97_1);
                ins = !!(engineInstance->chipData->model.device_config & CAPTURE_2_FROM_AC97_1);
        } else {
                outs = 0;
                ins = !!(engineInstance->chipData->model.device_config & CAPTURE_2_FROM_I2S_2);
        }
        
        if (outs | ins) {
                //err = snd_pcm_new(chip->card, outs ? "AC97" : "Analog2",
                //                          2, outs, ins, &pcm);
                //if (err < 0)
                //    return err;
                if (outs) {
                        //add ac97_ops fns
                        outputStream = createAudioStream(kIOAudioStreamDirectionOutput, DEFAULT_BUFFER_BYTES);
                        if (!outputStream->stream) {
                                goto Done;
                        }
                        oxygen_write8_masked(engineInstance->chipData, OXYGEN_REC_ROUTING,
                                             OXYGEN_REC_B_ROUTE_AC97_1,
                                             OXYGEN_REC_B_ROUTE_MASK);
                        
                        outputStream->stream->setName(outs ? "AC97" : "Analog2");
                        addAudioStream(outputStream->stream);
                        outputStream->stream->release();
                }
                if (ins) {
                        //add rec_b_ops fns
                        inputStream = createAudioStream(kIOAudioStreamDirectionInput, DEFAULT_BUFFER_BYTES);
                        if (!inputStream->stream) {
                                goto Done;
                        }
                        //            snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
                        //                                                  snd_dma_pci_data(chip->pci),
                        //                                                  DEFAULT_BUFFER_BYTES,
                        //                                                  BUFFER_BYTES_MAX);
                        
                        inputStream->stream->setName(outs ? "Front Panel" : "Analog 2");
                        addAudioStream(inputStream->stream);
                        inputStream->stream->release();
                }
        }
        // Setup the initial sample rate for the audio engine
        initialSampleRate.whole = INITIAL_SAMPLE_RATE;
        initialSampleRate.fraction = 0;
        
        setDescription("Xonar AC97/Analog-Out Engine");
        
        setSampleRate(&initialSampleRate);
        
        // Set the number of sample frames in each buffer
        setNumSampleFramesPerBuffer(NUM_SAMPLE_FRAMES);
        
        //    workLoop = getWorkLoop();
        //    if (!workLoop) {
        //        goto Done;
        //    }
        
        // Create an interrupt event source through which to receive interrupt callbacks
        // In this case, we only want to do work at primary interrupt time, so
        // we create an IOFilterInterruptEventSource which makes a filtering call
        // from the primary interrupt interrupt who's purpose is to determine if
        // our secondary interrupt handler is to be called.  In our case, we
        // can do the work in the filter routine and then return false to
        // indicate that we do not want our secondary handler called
        //    interruptEventSource = IOFilterInterruptEventSource::filterInterruptEventSource(this,
        //                                    interruptHandler,
        //                                    interruptFilter,
        //                                    audioDevice->getProvider());
        //    if (!interruptEventSource) {
        //        goto Done;
        //    }
        
        // In order to allow the interrupts to be received, the interrupt event source must be
        // added to the IOWorkLoop
        // Additionally, interrupts will not be firing until the interrupt event source is
        // enabled by calling interruptEventSource->enable() - this probably doesn't need to
        // be done until performAudioEngineStart() is called, and can probably be disabled
        // when performAudioEngineStop() is called and the audio engine is no longer running
        // Although this really depends on the specific hardware
        //    workLoop->addEventSource(interruptEventSource);
        
        result = true;
Done:
#if DEBUG
        kprintf("XonarAC97AudioEngine::%-26s END\n", __func__);
#endif
        return result;
}

IOStream *XonarAC97AudioEngine::createAudioStream(IOAudioStreamDirection direction, UInt32 sampleBufferSize)
{
        //when calling createaudiostream, we are "in" oxygen_pcm_init.
        //this function emulates snd_pcm_new, because each function is creating a stream.
        IOStream *newStream = new IOStream;
        newStream->stream = new IOAudioStream;
#if DEBUG
        kprintf("XonarAC97AudioEngine::%-26s BEGIN\n", __func__);
#endif
        //create the buffers.
        if(direction == kIOAudioStreamDirectionInput)
                newStream->buffer = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionIn | kIOMemoryPhysicallyContiguous, sampleBufferSize);
        else
                newStream->buffer = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionOut | kIOMemoryPhysicallyContiguous, sampleBufferSize);
        
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAC97AudioEngine::%-26s buffer creation COMPLETE, calling oxygen_open, direction %d, sample buff address: 0x%08llx\n",
                " ", direction, newStream->buffer->getPhysicalAddress());
#endif
        // call oxygen_open
        newStream->stream->initWithAudioEngine(this, direction, 1);
        engineInstance->oxygen_open(newStream->stream, direction ? PCM_B : PCM_AC97);
        
        // call oxygen_*_hw_params
        //        if(direction == kIOAudioStreamDirectionInput) //PCM_B
        //                engineInstance->oxygen_rec_b_hw_params(audioStream, 0, NULL, NULL, NULL);
        //        else // SPDIF_HW_PARAMS
        //                engineInstance->oxygen_hw_params(audioStream, 0, PCM_AC97, NULL, NULL);
        //        else if(source == PCM_AC97)
        //                oxygen_hw_params(audioStream, 0, PCM_AC97, NULL, NULL);
        //        else if(source == PCM_SPDIF)
        //                spdifEngine->oxygen_spdif_hw_params(audioStream, 0, NULL, NULL, NULL);
        
        //per alsa documentation (https://www.kernel.org/doc/html/latest/sound/kernel-api/writing-an-alsa-driver.html#operators)
        //hw_params is only called when all of the info about the stream is known
        //however it seems like we can get away with it
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAC97AudioEngine::%-26s setting sample buffer\n", " ");
#endif
        newStream->stream->setSampleBuffer(newStream->buffer->getBytesNoCopy(), newStream->buffer->getCapacity());
        
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAC97AudioEngine::%-26s setting sample format\n", " ");
#endif
        //we shouldn't call the driver because the SPDIF and AC97
        //need the data from this call before they can do anything else.
        newStream->stream->setFormat(&oxygen_hardware[direction ? PCM_B : PCM_AC97], false);
#if DEBUG
        kprintf("XonarAC97AudioEngine::%-26s END\n", __func__);
#endif
        //we use a mask variable to know what streams we have to stop
        engineInstance->channel_mask |= (1 << (direction? PCM_B: PCM_AC97));
        return newStream;
}


void XonarAC97AudioEngine::free()
{
#if DEBUG
        kprintf("XonarAC97AudioEngine::%-26s BEGIN\n", __func__);
#endif
        if (inputStream) {
                if(inputStream->buffer)
                        inputStream->buffer->IOGeneralMemoryDescriptor::free();
                
        }
        
        if (outputStream) {
                if(outputStream->buffer)
                        outputStream->buffer->IOGeneralMemoryDescriptor::free();
        }
        // We need to free our resources when we're going away
        //
        //    if (interruptEventSource) {
        //        interruptEventSource->release();
        //        interruptEventSource = NULL;
        //    }
        super::free();
#if DEBUG
#endif
}


void XonarAC97AudioEngine::stop(IOService *provider)
{
#if DEBUG
        kprintf("XonarAC97AudioEngine::%-26s BEGIN\n", __func__);
#endif
        // When our device is being stopped and torn down, we should go ahead and remove
        // the interrupt event source from the IOWorkLoop
        // Additionally, we'll go ahead and release the interrupt event source since it isn't
        // needed any more
        //    if (interruptEventSource) {
        //        IOWorkLoop *wl;
        //
        //        wl = getWorkLoop();
        //        if (wl) {
        //            wl->removeEventSource(interruptEventSource);
        //        }
        //
        //        interruptEventSource->release();
        //        interruptEventSource = NULL;
        //    }
        
        // Add code to shut down hardware (beyond what is needed to simply stop the audio engine)
        // There may be nothing needed here
        
        super::stop(provider);
#if DEBUG
        kprintf("XonarAC97AudioEngine::%-26s END\n", __func__);
        
#endif
}

IOReturn XonarAC97AudioEngine::performAudioEngineStart()
{
#if DEBUG
        kprintf("XonarAC97AudioEngine::%-26s BEGIN\n", __func__);
#endif
        // The interruptEventSource needs to be enabled to allow interrupts to start firing
        //    assert(interruptEventSource);
        //    interruptEventSource->enable();
        
        // When performAudioEngineStart() gets called, the audio engine should be started from the beginning
        // of the sample buffer.  Because it is starting on the first sample, a new timestamp is needed
        // to indicate when that sample is being read from/written to.  The function takeTimeStamp()
        // is provided to do that automatically with the current time.
        // By default takeTimeStamp() will increment the current loop count in addition to taking the current
        // timestamp.  Since we are starting a new audio engine run, and not looping, we don't want the loop count
        // to be incremented.  To accomplish that, false is passed to takeTimeStamp().
        takeTimeStamp(false);
        
        // Add audio - I/O start code here
        
        //#error performAudioEngineStart() - driver will not work until audio engine start code is added
#if DEBUG
        kprintf("XonarAC97AudioEngine::%-26s END\n", __func__);
#endif
        return kIOReturnSuccess;
}

IOReturn XonarAC97AudioEngine::performAudioEngineStop()
{
#if DEBUG
        kprintf("XonarAC97AudioEngine::%-26s BEGIN\n", __func__);
#endif
        // Assuming we don't need interrupts after stopping the audio engine, we can disable them here
        assert(interruptEventSource);
        interruptEventSource->disable();
        
        // Add audio - I/O stop code here
        
        //#error performAudioEngineStop() - driver will not work until audio engine stop code is added
#if DEBUG
        kprintf("XonarAC97AudioEngine::%-26s END\n", __func__);
#endif
        return kIOReturnSuccess;
}

UInt32 XonarAC97AudioEngine::getCurrentSampleFrame()
{
#if DEBUG
        kprintf("XonarAC97AudioEngine::%-26s BEGIN\n", __func__);
#endif
        // In order for the erase process to run properly, this function must return the current location of
        // the audio engine - basically a sample counter
        // It doesn't need to be exact, but if it is inexact, it should err towards being before the current location
        // rather than after the current location.  The erase head will erase up to, but not including the sample
        // frame returned by this function.  If it is too large a value, sound data that hasn't been played will be
        // erased.
        UInt32 curr_addr;
        
        /* no spinlock, this read should be atomic */
        curr_addr = oxygen_read32(engineInstance->chipData, channel_base_registers[PCM_AC97]);
        return curr_addr - outputStream->buffer->getPhysicalAddress();
}

IOReturn XonarAC97AudioEngine::performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate)
{
#if DEBUG
        kprintf("XonarAC97AudioEngine::%-26s BEGIN\n", __func__);
#endif
        // Since we only allow one format, we only need to be concerned with sample rate changes
        // In this case, we only allow 2 sample rates - 44100 & 48000, so those are the only ones
        // that we check for
        IOAudioStreamFormatExtension formatExDefault;
        formatExDefault.fBytesPerPacket = 32;
        formatExDefault.fFramesPerPacket = 1;
        if (newSampleRate) {
                
                if(audioStream->getDirection() == kIOAudioStreamDirectionInput) // AC_97 input uses REC_B
                        engineInstance->oxygen_rec_b_hw_params(audioStream, 1, newFormat, &formatExDefault, newSampleRate);
                else //AC_97 OUTPUT uses hw_params with the PCM_AC97 tag.
                        engineInstance->oxygen_hw_params(audioStream, 1, newFormat->fDriverTag, newFormat, &formatExDefault);
                //        switch (newSampleRate->whole) {
                //            case 44100:
                //                IOLog("/t-> 44.1kHz selected\n");
                //
                //                // Add code to switch hardware to 44.1khz
                //                break;
                //            case 48000:
                //                IOLog("/t-> 48kHz selected\n");
                //
                //                // Add code to switch hardware to 48kHz
                //                break;
                //            default:
                //                // This should not be possible since we only specified 44100 and 48000 as valid sample rates
                //                IOLog("/t Internal Error - unknown sample rate selected.\n");
                //                break;
                //        }
        }
#if DEBUG
        kprintf("XonarAC97AudioEngine::%-26s END\n", __func__);
#endif
        return kIOReturnSuccess;
}


//void XonarAC97AudioEngine::interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count)
//{
//    // Since our interrupt filter always returns false, this function will never be called
//    // If the filter returned true, this function would be called on the IOWorkLoop
//    return;
//}
//
//bool XonarAC97AudioEngine::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source)
//{
//    XonarAC97AudioEngine *audioEngine = OSDynamicCast(XonarAC97AudioEngine, owner);
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
//void XonarAC97AudioEngine::filterInterrupt(int index)
//{
//    // In the case of our simple device, we only get interrupts when the audio engine loops to the
//    // beginning of the buffer.  When that happens, we need to take a timestamp and increment
//    // the loop count.  The function takeTimeStamp() does both of those for us.  Additionally,
//    // if a different timestamp is to be used (other than the current time), it can be passed
//    // in to takeTimeStamp()
//    takeTimeStamp();
//}

