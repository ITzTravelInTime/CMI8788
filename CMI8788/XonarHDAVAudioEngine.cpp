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


//#include "ak4396.h"
//#include "wm8785.h"
#include <libkern/OSByteOrder.h>
#include <sys/errno.h>
#include <i386/limits.h>
#include </usr/include/libkern/OSAtomic.h>


#include <IOKit/IOLib.h>
#include <IOKit/IOFilterInterruptEventSource.h>
//#include <architecture/i386/pio.h>
#include "XonarHDAVAudioEngine.hpp"
#include "pcm1796.h"
#include "cm9780.h"
#include "cs2000.h"
#include "ac97.h"
#define INITIAL_SAMPLE_RATE	44100
#define NUM_SAMPLE_FRAMES	16384
#define NUM_CHANNELS		2
#define BIT_DEPTH			16


#define super IOAudioEngine

OSDefineMetaClassAndStructors(XonarHDAVAudioEngine, IOAudioEngine)



void XonarHDAVAudioEngine::hdmi_write_command(struct oxygen *chip, UInt8 command,
                                              unsigned int count, const UInt8 *params)
{
    unsigned int i;
    UInt8 checksum;
    
    this->engineInstance->oxygen_write_uart(chip, 0xfb);
    this->engineInstance->oxygen_write_uart(chip, 0xef);
    this->engineInstance->oxygen_write_uart(chip, command);
    this->engineInstance->oxygen_write_uart(chip, count);
    for (i = 0; i < count; ++i)
        this->engineInstance->oxygen_write_uart(chip, params[i]);
    checksum = 0xfb + 0xef + command + count;
    for (i = 0; i < count; ++i)
        checksum += params[i];
    this->engineInstance->oxygen_write_uart(chip, checksum);
}

void XonarHDAVAudioEngine::xonar_hdmi_init_commands(struct oxygen *chip,
                                                    struct xonar_hdmi *hdmi)
{
    UInt8 param;
    
    //  oxygen_reset_uart(chip);
    param = 0;
    hdmi_write_command(chip, 0x61, 1, &param);
    param = 1;
    hdmi_write_command(chip, 0x74, 1, &param);
    hdmi_write_command(chip, 0x54, 5, hdmi->params);
}


void XonarHDAVAudioEngine::xonar_hdmi_init(struct oxygen *chip, struct xonar_hdmi *hdmi)
{
    hdmi->params[1] = IEC958_AES3_CON_FS_48000;
    hdmi->params[4] = 1;
    xonar_hdmi_init_commands(chip, hdmi);
}

void XonarHDAVAudioEngine::xonar_hdmi_cleanup(struct oxygen *chip)
{
    UInt8 param = 0;
    
    hdmi_write_command(chip, 0x74, 1, &param);
}

void XonarHDAVAudioEngine::xonar_hdmi_resume(struct oxygen *chip, struct xonar_hdmi *hdmi)
{
    xonar_hdmi_init_commands(chip, hdmi);
}
/*
 void xonar_hdmi_pcm_hardware_filter(unsigned int channel,
 struct snd_pcm_hardware *hardware)
 {
 if (channel == PCM_MULTICH) {
 hardware->rates = SNDRV_PCM_RATE_44100 |
 SNDRV_PCM_RATE_48000 |
 SNDRV_PCM_RATE_96000 |
 SNDRV_PCM_RATE_192000;
 hardware->rate_min = 44100;
 }
 }
 */
void XonarHDAVAudioEngine::xonar_set_hdmi_params(struct oxygen *chip, struct xonar_hdmi *hdmi)
{
    hdmi->params[0] = 0; // 1 = non-audio
    switch (this->engineInstance->getSampleRate()->whole) {
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
    hdmi->params[2] = this->engineInstance->inputs[0]->maxNumChannels / 2 - 1;
    //^ this is wrong because it should be NumChannels, not MaxNum
    //however since IOAudioStream calls are deprecated as of 10.10,
    //i'm going to use this is a placeholder/semi-correct call.
    
    //Linux call:
    //if (params_format(params) == SNDRV_PCM_FORMAT_S16_LE)
    //Mac Call:
    if(this->engineInstance->inputs[0]->format.fSampleFormat == SNDRV_PCM_FORMAT_S16_LE)
        hdmi->params[3] = 0;
    else
        hdmi->params[3] = 0xc0;
    hdmi->params[4] = 1; // ?
    hdmi_write_command(chip, 0x54, 5, hdmi->params);
}

void XonarHDAVAudioEngine::set_hdav_params(struct oxygen *chip)
{
    // struct xonar_hdav *data = (struct xonar_hdav*) chip->model_data;
    
    this->engineInstance->set_pcm1796_params(chip, this->engineInstance);
    xonar_set_hdmi_params(chip, &deviceRegisters->hdmi);
}


static void xonar_hdmi_uart_input(struct oxygen *chip)
{
    if (chip->uart_input_count >= 2 &&
        chip->uart_input[chip->uart_input_count - 2] == 'O' &&
        chip->uart_input[chip->uart_input_count - 1] == 'K') {
        IOLog("message from HDMI chip received:\n");
        //print_hex_dump_bytes("", DUMP_PREFIX_OFFSET,
        //                    chip->uart_input, chip->uart_input_count);
        chip->uart_input_count = 0;
    }
}



bool XonarHDAVAudioEngine::init(XonarAudioEngine *engine, struct oxygen *chip)
{
    
    bool result = false;
    
    IOLog("XonarHDAVAudioEngine[%p]::init(%p)\n", this, chip);
    
    if (!chip) {
        goto Done;
    }
    
    if (!super::init(NULL)) {
        goto Done;
    }
    //  ak4396_init(chip);
    //  wm8785_init(chip);
    
    chip->model_data = IOMalloc(chip->model.model_data_size);
    deviceRegisters = (struct xonar_hdav*)chip->model_data;
    this->engineInstance = engine;

    xonar_hdav_init(chip);
    
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
    //assign fn ptr uart_input to xonar_hdmi_uart_input
    chip->model.uart_input = xonar_hdmi_uart_input;
    
    this->engineInstance->pcm1796_init(chip);
    
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL,
                      GPIO_HDAV_MAGIC | GPIO_INPUT_ROUTE);
    oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_INPUT_ROUTE);
    
    this->engineInstance->xonar_init_cs53x1(chip);
    this->engineInstance->xonar_init_ext_power(chip);
    xonar_hdmi_init(chip, &deviceRegisters->hdmi);
    this->engineInstance->xonar_enable_output(chip);
    result = true;
    
    goto Done;
    // snd_component_add(chip->card, "PCM1796");
    // snd_component_add(chip->card, "CS5381");
    
Done:
    return result;
    
    
    
}


void XonarHDAVAudioEngine::xonar_hdav_cleanup(struct oxygen *chip)
{
    xonar_hdmi_cleanup(chip);
    this->engineInstance->xonar_disable_output(chip);
    IODelay(2);
}

void XonarHDAVAudioEngine::xonar_hdav_resume(struct oxygen *chip)
{
    //struct xonar_hdav *data = (struct xonar_hdav*) chip->model_data;
    
    this->engineInstance->pcm1796_registers_init(chip);
    xonar_hdmi_resume(chip, &deviceRegisters->hdmi);
    this->engineInstance->xonar_enable_output(chip);
}

int XonarHDAVAudioEngine::xonar_hdav_mixer_init(struct oxygen *chip)
{
    int err;
    
    //  err = snd_ctl_add(chip->card, snd_ctl_new1(&hdav_hdmi_control, chip));
    if (err < 0)
        return err;
    err = this->engineInstance->add_pcm1796_controls(chip);
    if (err < 0)
        return err;
    return 0;
}



//bool XonarHDAVAudioEngine::init(XonarAudioEngine *engine, struct oxygen *chip)
//{
//    bool result = false;
//
//    IOLog("XonarHDAVAudioEngine[%p]::init(%p)\n", this, chip);
//
//    if (!chip) {
//        goto Done;
//    }
//
//    if (!super::init(NULL)) {
//        goto Done;
//    }
//  //  ak4396_init(chip);
//  //  wm8785_init(chip);
//    deviceRegisters = (struct xonar_hdav*)chip->model_data;
//
//    // the below aren't correct. have to bridge the workqueue calls to IOWorkLoop
//    queue_init(&chip->ac97_waitqueue);
//    chip->mutex = OS_SPINLOCK_INIT;
//    this->engineInstance = engine;
//    xonar_hdav_init(chip);
//    result = true;
//
//Done:
//
//    return result;
//}

bool XonarHDAVAudioEngine::initHardware(IOService *provider)
{
    bool result = false;
    IOAudioSampleRate initialSampleRate;
    IOAudioStream *audioStream;
    IOWorkLoop *workLoop;
    
    IOLog("XonarHDAVAudioEngine[%p]::initHardware(%p)\n", this, provider);
    
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
    
    workLoop = getWorkLoop();
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
                                                                                    XonarHDAVAudioEngine::interruptHandler,
                                                                                    XonarHDAVAudioEngine::interruptFilter,
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
    IOLog("XonarHDAVAudioEngine[%p]::free()\n", this);
    
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
    IOLog("XonarHDAVAudioEngine[%p]::stop(%p)\n", this, provider);
    
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
    IOLog("XonarHDAVAudioEngine[%p]::performAudioEngineStart()\n", this);
    
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
    IOLog("XonarHDAVAudioEngine[%p]::performAudioEngineStop()\n", this);
    
    // Assuming we don't need interrupts after stopping the audio engine, we can disable them here
    assert(interruptEventSource);
    interruptEventSource->disable();
    
    // Add audio - I/O stop code here
    
//#error performAudioEngineStop() - driver will not work until audio engine stop code is added
    
    return kIOReturnSuccess;
}

UInt32 XonarHDAVAudioEngine::getCurrentSampleFrame()
{
    IOLog("XonarHDAVAudioEngine[%p]::getCurrentSampleFrame()\n", this);
    
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
    IOLog("XonarHDAVAudioEngine[%p]::peformFormatChange(%p, %p, %p)\n", this, audioStream, newFormat, newSampleRate);
    
    // Since we only allow one format, we only need to be concerned with sample rate changes
    // In this case, we only allow 2 sample rates - 44100 & 48000, so those are the only ones
    // that we check for
    if (newSampleRate) {
        switch (newSampleRate->whole) {
            case 44100:
                IOLog("/t-> 44.1kHz selected\n");
                
                // Add code to switch hardware to 44.1khz
                break;
            case 48000:
                IOLog("/t-> 48kHz selected\n");
                
                // Add code to switch hardware to 48kHz
                break;
            default:
                // This should not be possible since we only specified 44100 and 48000 as valid sample rates
                IOLog("/t Internal Error - unknown sample rate selected.\n");
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

