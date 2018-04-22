/*
 File:XonarD2XAudioEngine.cpp
 
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
#include "XonarD2XAudioEngine.hpp"

#include "pcm1796.h"
#include "cm9780.h"
#include "cs2000.h"
#include "ac97.h"
#define INITIAL_SAMPLE_RATE	44100
#define NUM_SAMPLE_FRAMES	16384
#define NUM_CHANNELS		2
#define BIT_DEPTH			16


#define super IOAudioEngine

OSDefineMetaClassAndStructors(XonarD2XAudioEngine, IOAudioEngine)


void XonarD2XAudioEngine::xonar_d2_cleanup(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    engineInstance->xonar_disable_output(chip);
}

void XonarD2XAudioEngine::xonar_d2_suspend(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    xonar_d2_cleanup(chip,engineInstance);
}


void XonarD2XAudioEngine::xonar_d2_resume(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    engineInstance->pcm1796_registers_init(chip);
    engineInstance->xonar_enable_output(chip);
}

/*
 static int xense_output_switch_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 UInt16 gpio;
 
 gpio = oxygen_read16(chip, OXYGEN_GPIO_DATA);
 if (gpio & GPIO_XENSE_SPEAKERS)
 value->value.enumerated.item[0] = 0;
 else if (!(gpio & GPIO_XENSE_SPEAKERS) && (gpio & GPIO_ST_HP_REAR))
 value->value.enumerated.item[0] = 1;
 else
 value->value.enumerated.item[0] = 2;
 return 0;
 }
 
 static int xense_output_switch_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_pcm179x *data = chip->model_data;
 UInt16 gpio_old, gpio;
 
 mutex_lock(&chip->mutex);
 gpio_old = oxygen_read16(chip, OXYGEN_GPIO_DATA);
 gpio = gpio_old;
 switch (value->value.enumerated.item[0]) {
 case 0:
 gpio |= GPIO_XENSE_SPEAKERS | GPIO_ST_HP_REAR;
 break;
 case 1:
 gpio = (gpio | GPIO_ST_HP_REAR) & ~GPIO_XENSE_SPEAKERS;
 break;
 case 2:
 gpio &= ~(GPIO_XENSE_SPEAKERS | GPIO_ST_HP_REAR);
 break;
 }
 oxygen_write16(chip, OXYGEN_GPIO_DATA, gpio);
 data->hp_active = !(gpio & GPIO_XENSE_SPEAKERS);
 update_pcm1796_volume(chip);
 mutex_unlock(&chip->mutex);
 return gpio != gpio_old;
 }
 
 static const struct snd_kcontrol_new xense_controls[] = {
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Analog Output",
 .info = st_output_switch_info,
 .get = xense_output_switch_get,
 .put = xense_output_switch_put,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Headphones Impedance Playback Enum",
 .info = st_hp_volume_offset_info,
 .get = st_hp_volume_offset_get,
 .put = st_hp_volume_offset_put,
 },
 };
 
 
 
 
 static int xonar_d2_control_filter(struct snd_kcontrol_new *template)
 {
 if (!strncmp(template->name, "CD Capture ", 11))
 // CD in is actually connected to the video in pin
 template->private_value ^= AC97_CD ^ AC97_VIDEO;
 return 0;
 }
 */


int XonarD2XAudioEngine::xonar_d2_mixer_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    int err;
    
    //   err = snd_ctl_add(chip->card, snd_ctl_new1(&alt_switch, chip));
    if (err < 0)
        return err;
    err = engineInstance->add_pcm1796_controls(chip);
    if (err < 0)
        return err;
    return 0;
}

void XonarD2XAudioEngine::xonar_d2_init(struct oxygen *chip, XonarAudioEngine *engineInstance) {
    
    struct xonar_pcm179x *data = (struct xonar_pcm179x*)chip->model_data;
    data->generic.anti_pop_delay = 300;
    data->generic.output_enable_bit = GPIO_D2_OUTPUT_ENABLE;
    data->dacs = 4;
    
    engineInstance->pcm1796_init(chip);
    
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_D2_ALT);
    oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_D2_ALT);
    
    //oxygen_ac97_set_bits(chip, 0, CM9780_JACK, CM9780_FMIC2MIC);
    
    engineInstance->xonar_init_cs53x1(chip);
    engineInstance->xonar_enable_output(chip);
    
    //  snd_component_add(chip->card, "PCM1796");
    // snd_component_add(chip->card, "CS5381");
}

bool XonarD2XAudioEngine::init(XonarAudioEngine *engine, struct oxygen *chip, uint8_t submodel)
{
    bool result = false;
    struct xonar_pcm179x *data = (struct xonar_pcm179x*)chip->model_data;
    IOLog("XonarD2XAudioEngine[%p]::init(%p)\n", this, chip);
    
    if (!chip) {
        goto Done;
    }
    
    if (!super::init(NULL)) {
        goto Done;
    }
    
    if(submodel == D2_MODEL)
        xonar_d2_init(chip, engineInstance);
    if(submodel == D2X_MODEL) {
        data->generic.ext_power_reg = OXYGEN_GPIO_DATA;
        data->generic.ext_power_int_reg = OXYGEN_GPIO_INTERRUPT_MASK;
        data->generic.ext_power_bit = GPIO_D2X_EXT_POWER;
        oxygen_clear_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_D2X_EXT_POWER);
        engineInstance->xonar_init_ext_power(chip);
        xonar_d2_init(chip, engineInstance);
    }
    if(submodel == XENSE_MODEL) {
        data->generic.ext_power_reg = OXYGEN_GPI_DATA;
        data->generic.ext_power_int_reg = OXYGEN_GPI_INTERRUPT_MASK;
        data->generic.ext_power_bit = GPI_EXT_POWER;
        this->engineInstance = engineInstance;
        this->engineInstance->xonar_init_ext_power(chip);
        
        data->generic.anti_pop_delay = 100;
        data->has_cs2000 = 1;
        data->cs2000_regs[CS2000_FUN_CFG_1] = CS2000_REF_CLK_DIV_1;
        
        oxygen_write16(chip, OXYGEN_I2S_A_FORMAT,
                       OXYGEN_RATE_48000 |
                       OXYGEN_I2S_FORMAT_I2S |
                       OXYGEN_I2S_MCLK(MCLK_512) |
                       OXYGEN_I2S_BITS_16 |
                       OXYGEN_I2S_MASTER |
                       OXYGEN_I2S_BCLK_64);
        
        this->STengineInstance->xonar_st_init_i2c(chip,engineInstance);
        this->engineInstance->cs2000_registers_init(chip);
        
        data->generic.output_enable_bit = GPIO_XENSE_OUTPUT_ENABLE;
        data->dacs = 1;
        data->hp_gain_offset = 2*-18;
        
        this->engineInstance->pcm1796_init(chip);
        
        oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL,
                          GPIO_INPUT_ROUTE | GPIO_ST_HP_REAR |
                          GPIO_ST_MAGIC | GPIO_XENSE_SPEAKERS);
        oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA,
                            GPIO_INPUT_ROUTE | GPIO_ST_HP_REAR |
                            GPIO_XENSE_SPEAKERS);
        
        this->engineInstance->xonar_init_cs53x1(chip);
        this->engineInstance->xonar_enable_output(chip);
        
        //   snd_component_add(chip->card, "PCM1796");
        //   snd_component_add(chip->card, "CS5381");
        //  snd_component_add(chip->card, "CS2000");
    }
    
    //  ak4396_init(chip);
    //  wm8785_init(chip);
    deviceRegisters = (struct xonar_pcm179x*)chip->model_data;
    
    //set the pointer to XonarAudioEngine.
    this->engineInstance = engine;
    
    result = true;
    
Done:
    
    return result;
}

bool XonarD2XAudioEngine::initHardware(IOService *provider)
{
    bool result = false;
    IOAudioSampleRate initialSampleRate;
    IOAudioStream *audioStream;
    IOWorkLoop *workLoop;
    
    IOLog("XonarD2XAudioEngine[%p]::initHardware(%p)\n", this, provider);
    
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
                                                                                    XonarD2XAudioEngine::interruptHandler,
                                                                                    XonarD2XAudioEngine::interruptFilter,
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

void XonarD2XAudioEngine::free()
{
    IOLog("XonarD2XAudioEngine[%p]::free()\n", this);
    
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

IOAudioStream *XonarD2XAudioEngine::createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize)
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

void XonarD2XAudioEngine::stop(IOService *provider)
{
    IOLog("XonarD2XAudioEngine[%p]::stop(%p)\n", this, provider);
    
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

IOReturn XonarD2XAudioEngine::performAudioEngineStart()
{
    IOLog("XonarD2XAudioEngine[%p]::performAudioEngineStart()\n", this);
    
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

IOReturn XonarD2XAudioEngine::performAudioEngineStop()
{
    IOLog("XonarD2XAudioEngine[%p]::performAudioEngineStop()\n", this);
    
    // Assuming we don't need interrupts after stopping the audio engine, we can disable them here
    assert(interruptEventSource);
    interruptEventSource->disable();
    
    // Add audio - I/O stop code here
    
//#error performAudioEngineStop() - driver will not work until audio engine stop code is added
    
    return kIOReturnSuccess;
}

UInt32 XonarD2XAudioEngine::getCurrentSampleFrame()
{
    IOLog("XonarD2XAudioEngine[%p]::getCurrentSampleFrame()\n", this);
    
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

IOReturn XonarD2XAudioEngine::performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate)
{
    IOLog("XonarD2XAudioEngine[%p]::peformFormatChange(%p, %p, %p)\n", this, audioStream, newFormat, newSampleRate);
    
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


void XonarD2XAudioEngine::interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count)
{
    // Since our interrupt filter always returns false, this function will never be called
    // If the filter returned true, this function would be called on the IOWorkLoop
    return;
}

bool XonarD2XAudioEngine::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source)
{
    XonarD2XAudioEngine *audioEngine = OSDynamicCast(XonarD2XAudioEngine, owner);
    
    // We've cast the audio engine from the owner which we passed in when we created the interrupt
    // event source
    if (audioEngine) {
        // Then, filterInterrupt() is called on the specified audio engine
        audioEngine->filterInterrupt(source->getIntIndex());
    }
    
    return false;
}

void XonarD2XAudioEngine::filterInterrupt(int index)
{
    // In the case of our simple device, we only get interrupts when the audio engine loops to the
    // beginning of the buffer.  When that happens, we need to take a timestamp and increment
    // the loop count.  The function takeTimeStamp() does both of those for us.  Additionally,
    // if a different timestamp is to be used (other than the current time), it can be passed
    // in to takeTimeStamp()
    takeTimeStamp();
}

