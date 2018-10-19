/*
 File:XonarHDAVAudioEngine.cpp
 
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

#import <libkern/OSByteOrder.h>
#import <sys/errno.h>
#import <i386/limits.h>
#import </usr/include/libkern/OSAtomic.h>
#import </usr/include/string.h>

#import <IOKit/IOLib.h>
#import <IOKit/IOFilterInterruptEventSource.h>
#import "XonarHDAVAudioEngine.hpp"
#import "pcm1796.h"
#import "cm9780.h"
#import "cs2000.h"
#import "ac97.h"

#define INITIAL_SAMPLE_RATE	44100
#define NUM_SAMPLE_FRAMES	16384
#define NUM_CHANNELS		2
#define BIT_DEPTH			16


#define super IOAudioEngine

OSDefineMetaClassAndStructors(XonarHDAVAudioEngine, IOAudioEngine)



void XonarHDAVAudioEngine::set_hdav_params(struct oxygen *chip,XonarAudioEngine *engineInstance)
{
    struct xonar_hdav *data = (struct xonar_hdav*) chip->model_data;
    
    engineInstance->set_pcm1796_params(chip, engineInstance);
    engineInstance->xonar_set_hdmi_params(chip, &data->hdmi);
}


void XonarHDAVAudioEngine::xonar_hdav_cleanup(struct oxygen *chip,XonarAudioEngine *engineInstance)
{
    engineInstance->xonar_hdmi_cleanup(chip);
    engineInstance->xonar_disable_output(chip);
    IODelay(2);
}

void XonarHDAVAudioEngine::xonar_hdav_resume(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    struct xonar_hdav *data = (struct xonar_hdav*) chip->model_data;
    
    engineInstance->pcm1796_registers_init(chip);
    engineInstance->xonar_hdmi_resume(chip, &data->hdmi);
    engineInstance->xonar_enable_output(chip);
}

int XonarHDAVAudioEngine::xonar_hdav_mixer_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    int err;
    
    //  err = snd_ctl_add(chip->card, snd_ctl_new1(&hdav_hdmi_control, chip));
    if (err < 0)
        return err;
    err = engineInstance->add_pcm1796_controls(chip);
    if (err < 0)
        return err;
    return 0;
}


bool XonarHDAVAudioEngine::init(XonarAudioEngine *engine, struct oxygen *chip)
{
    
    /* sample driver init code (from SamplePCIAudioEngine.cpp's ::init) */
    bool result = false;
    
    printf("XonarHDAVAudioEngine[%p]::init(%p)\n", this, chip);
    
    if (!chip) {
        goto Done;
    }
    
    if (!engine->init(chip, HDAV_MODEL)) {
        goto Done;
    }
    /* end sample driver template */
    chip->model_data = IOMalloc(chip->model.model_data_size);
    deviceRegisters = (struct xonar_hdav*) &chip->model_data;
    this->engineInstance = engine;
    
    /* begin ALSA xonar_hdav_init */
    oxygen_write16(chip, OXYGEN_2WIRE_BUS_STATUS,
                   OXYGEN_2WIRE_LENGTH_8 |
                   OXYGEN_2WIRE_INTERRUPT_MASK |
                   OXYGEN_2WIRE_SPEED_STANDARD);
    
    deviceRegisters->pcm179x.generic.anti_pop_delay = 100;
    deviceRegisters->pcm179x.generic.output_enable_bit = GPIO_HDAV_OUTPUT_ENABLE;
    deviceRegisters->pcm179x.generic.ext_power_reg = OXYGEN_GPI_DATA;
    deviceRegisters->pcm179x.generic.ext_power_int_reg = OXYGEN_GPI_INTERRUPT_MASK;
    deviceRegisters->pcm179x.generic.ext_power_bit = GPI_EXT_POWER;
    deviceRegisters->pcm179x.dacs = chip->model.dac_channels_mixer / 2;
    deviceRegisters->pcm179x.h6 = chip->model.dac_channels_mixer > 2;
    // assign fn ptr uart_input to xonar_hdmi_uart_input
    chip->model.resume = xonar_hdav_resume;
    chip->model.cleanup = xonar_hdav_cleanup;
    
    engine->pcm1796_init(chip);
    
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL,
                      GPIO_HDAV_MAGIC | GPIO_INPUT_ROUTE);
    oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_INPUT_ROUTE);
    engine->xonar_init_cs53x1(chip);
    engine->xonar_init_ext_power(chip);
    engine->xonar_hdmi_init(chip, &deviceRegisters->hdmi);
    engine->xonar_enable_output(chip);
    
    
    
    /* end hdav_init, begin last bit of SamplePCIAudioEngine.cpp's init
     */
    
    result = true;
    
    goto Done;
    /* last bits of alsa's oxygen_init */
    // snd_component_add(chip->card, "PCM1796");
    // snd_component_add(chip->card, "CS5381");
    /* begin last bits of APPUL's samplepciaudioengine::init */
Done:
    return result;
    
    
    
}

bool XonarHDAVAudioEngine::initHardware(IOService *provider)
{
    bool result = false;
    IOAudioSampleRate initialSampleRate;
    IOAudioStream *audioStream;
    IOWorkLoop *workLoop;
    
    printf("XonarHDAVAudioEngine[%p]::initHardware(%p)\n", this, provider);
    
    if (!super::initHardware(provider)) {
        goto Done;
    }
    
    // Setup the initial sample rate for the audio engine
    initialSampleRate.whole = INITIAL_SAMPLE_RATE;
    initialSampleRate.fraction = 0;
    
    setDescription("Sample PCI Audio Engine");
    
    setSampleRate(&initialSampleRate);
    
    // Set the number of sample frames in each buffer
    setNumSampleFramesPerBuffer(NUM_SAMPLE_FRAMES);
    
    workLoop = engineInstance->getWorkLoop();
    if (!workLoop) {
        goto Done;
    }
    
    // Create an interrupt event source through which to receive interrupt callbacks
    // In this case, we only want to do work at primary interrupt time, so
    // we create an IOFilterInterruptEventSource which makes a filtering call
    // from the primary interrupt interrupt who's purpose is to determine if
    // our secondary interrupt handler is to be called.  In our case, we
    // can do the work in the filter routine and then return false to
    // indicate that we do not want our secondary handler called
    interruptEventSource = IOFilterInterruptEventSource::filterInterruptEventSource(this,
                                                                                    engineInstance->interruptHandler,
                                                                                    engineInstance->interruptFilter,
                                                                                    audioDevice->getProvider());
    if (!interruptEventSource) {
        goto Done;
    }
    
    // In order to allow the interrupts to be received, the interrupt event source must be
    // added to the IOWorkLoop
    // Additionally, interrupts will not be firing until the interrupt event source is
    // enabled by calling interruptEventSource->enable() - this probably doesn't need to
    // be done until performAudioEngineStart() is called, and can probably be disabled
    // when performAudioEngineStop() is called and the audio engine is no longer running
    // Although this really depends on the specific hardware
    workLoop->addEventSource(interruptEventSource);
    
    // Allocate our input and output buffers - a real driver will likely need to allocate its buffers
    // differently
    outputBuffer = (SInt16 *)IOMalloc(DEFAULT_BUFFER_BYTES);
    if (!outputBuffer) {
        goto Done;
    }
    
    inputBuffer = (SInt16 *)IOMalloc(DEFAULT_BUFFER_BYTES);
    if (!inputBuffer) {
        goto Done;
    }
    
    // Create an IOAudioStream for each buffer and add it to this audio engine
    audioStream = createNewAudioStream(kIOAudioStreamDirectionOutput, outputBuffer, DEFAULT_BUFFER_BYTES);
    if (!audioStream) {
        goto Done;
    }
    
    addAudioStream(audioStream);
    audioStream->release();
    
    audioStream = createNewAudioStream(kIOAudioStreamDirectionInput, inputBuffer, DEFAULT_BUFFER_BYTES);
    if (!audioStream) {
        goto Done;
    }
    
    addAudioStream(audioStream);
    audioStream->release();
    
    result = true;
    
Done:
    
    return result;
}

void XonarHDAVAudioEngine::free()
{
    printf("XonarHDAVAudioEngine[%p]::free()\n", this);
    
    // We need to free our resources when we're going away
    
    if (interruptEventSource) {
        interruptEventSource->release();
        interruptEventSource = NULL;
    }
    
    if (outputBuffer) {
        IOFree(outputBuffer, DEFAULT_BUFFER_BYTES);
        outputBuffer = NULL;
    }
    
    if (inputBuffer) {
        IOFree(inputBuffer, DEFAULT_BUFFER_BYTES);
        inputBuffer = NULL;
    }
    super::free();
}

IOAudioStream *XonarHDAVAudioEngine::createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize)
{
    IOAudioStream *audioStream;
    
    // For this sample device, we are only creating a single format and allowing 44.1KHz and 48KHz
    audioStream = new IOAudioStream;
    if (audioStream) {
        if (!audioStream->initWithAudioEngine(this, direction, 1)) {
            audioStream->release();
        } else {
            IOAudioSampleRate rate;
            IOAudioStreamFormat format = {
                2,												// num channels
                kIOAudioStreamSampleFormatLinearPCM,			// sample format
                kIOAudioStreamNumericRepresentationSignedInt,	// numeric format
                BIT_DEPTH,										// bit depth
                BIT_DEPTH,										// bit width
                kIOAudioStreamAlignmentHighByte,				// high byte aligned - unused because bit depth == bit width
                kIOAudioStreamByteOrderBigEndian,				// big endian
                true,											// format is mixable
                0												// driver-defined tag - unused by this driver
            };
            
            // As part of creating a new IOAudioStream, its sample buffer needs to be set
            // It will automatically create a mix buffer should it be needed
            audioStream->setSampleBuffer(sampleBuffer, sampleBufferSize);
            
            // This device only allows a single format and a choice of 2 different sample rates
            rate.fraction = 0;
            rate.whole = 44100;
            audioStream->addAvailableFormat(&format, &rate, &rate);
            rate.whole = 48000;
            audioStream->addAvailableFormat(&format, &rate, &rate);
            
            // Finally, the IOAudioStream's current format needs to be indicated
            audioStream->setFormat(&format);
        }
    }
    
    return audioStream;
}

void XonarHDAVAudioEngine::stop(IOService *provider)
{
    printf("XonarHDAVAudioEngine[%p]::stop(%p)\n", this, provider);
    
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
}

IOReturn XonarHDAVAudioEngine::performAudioEngineStart()
{
    printf("XonarHDAVAudioEngine[%p]::performAudioEngineStart()\n", this);
    
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
    
    //#error performAudioEngineStart() - driver will not work until audio engine start code is added
    
    return kIOReturnSuccess;
}

IOReturn XonarHDAVAudioEngine::performAudioEngineStop()
{
    printf("XonarHDAVAudioEngine[%p]::performAudioEngineStop()\n", this);
    
    // Assuming we don't need interrupts after stopping the audio engine, we can disable them here
    assert(interruptEventSource);
    interruptEventSource->disable();
    
    // Add audio - I/O stop code here
    
    //#error performAudioEngineStop() - driver will not work until audio engine stop code is added
    
    return kIOReturnSuccess;
}

UInt32 XonarHDAVAudioEngine::getCurrentSampleFrame()
{
    printf("XonarHDAVAudioEngine[%p]::getCurrentSampleFrame()\n", this);
    
    // In order for the erase process to run properly, this function must return the current location of
    // the audio engine - basically a sample counter
    // It doesn't need to be exact, but if it is inexact, it should err towards being before the current location
    // rather than after the current location.  The erase head will erase up to, but not including the sample
    // frame returned by this function.  If it is too large a value, sound data that hasn't been played will be
    // erased.
    
    //#error getCurrentSampleFrame() - driver will not work until correct sample frame is returned
    
    // Change to return the real value
    return 0;
}

IOReturn XonarHDAVAudioEngine::performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate)
{
    printf("XonarHDAVAudioEngine[%p]::peformFormatChange(%p, %p, %p)\n", this, audioStream, newFormat, newSampleRate);
    
    // Since we only allow one format, we only need to be concerned with sample rate changes
    // In this case, we only allow 2 sample rates - 44100 & 48000, so those are the only ones
    // that we check for
    if (newSampleRate) {
        switch (newSampleRate->whole) {
            case 44100:
                printf("/t-> 44.1kHz selected\n");
                
                // Add code to switch hardware to 44.1khz
                break;
            case 48000:
                printf("/t-> 48kHz selected\n");
                
                // Add code to switch hardware to 48kHz
                break;
            default:
                // This should not be possible since we only specified 44100 and 48000 as valid sample rates
                printf("/t Internal Error - unknown sample rate selected.\n");
                break;
        }
    }
    
    return kIOReturnSuccess;
}


void XonarHDAVAudioEngine::interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count)
{
    // Since our interrupt filter always returns false, this function will never be called
    // If the filter returned true, this function would be called on the IOWorkLoop
    return;
}

bool XonarHDAVAudioEngine::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source)
{
    XonarHDAVAudioEngine *audioEngine = OSDynamicCast(XonarHDAVAudioEngine, owner);
    
    // We've cast the audio engine from the owner which we passed in when we created the interrupt
    // event source
    if (audioEngine) {
        // Then, filterInterrupt() is called on the specified audio engine
        audioEngine->filterInterrupt(source->getIntIndex());
    }
    
    return false;
}

void XonarHDAVAudioEngine::filterInterrupt(int index)
{
    // In the case of our simple device, we only get interrupts when the audio engine loops to the
    // beginning of the buffer.  When that happens, we need to take a timestamp and increment
    // the loop count.  The function takeTimeStamp() does both of those for us.  Additionally,
    // if a different timestamp is to be used (other than the current time), it can be passed
    // in to takeTimeStamp()
    takeTimeStamp();
}

