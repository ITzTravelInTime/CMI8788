#import <libkern/OSByteOrder.h>
#import <sys/errno.h>
#import <i386/limits.h>
#import </usr/include/libkern/OSAtomic.h>
#import <IOKit/IOLib.h>
#import <IOKit/IOFilterInterruptEventSource.h>
#import "XonarCS43XXAudioEngine.hpp"
#import "cm9780.h"
#import "cs4398.h"
#import "cs4362a.h"
#import "ac97.h"

#define INITIAL_SAMPLE_RATE	44100
#define NUM_SAMPLE_FRAMES	16384
#define NUM_CHANNELS		2
#define BIT_DEPTH			16


#define super IOAudioEngine

OSDefineMetaClassAndStructors(XonarCS43XXAudioEngine, IOAudioEngine)


/*
 * card driver for models with CS4398/CS4362A DACs (Xonar D1/DX)
 *
 * Copyright (c) Clemens Ladisch <clemens@ladisch.de>
 *
 *
 *  This driver is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License, version 2.
 *
 *  This driver is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this driver; if not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Xonar D1/DX
 * -----------
 *
 * CMI8788:
 *
 *   I²C <-> CS4398 (addr 1001111) (front)
 *       <-> CS4362A (addr 0011000) (surround, center/LFE, back)
 *
 *   GPI 0 <- external power present (DX only)
 *
 *   GPIO 0 -> enable output to speakers
 *   GPIO 1 -> route output to front panel
 *   GPIO 2 -> M0 of CS5361
 *   GPIO 3 -> M1 of CS5361
 *   GPIO 6 -> ?
 *   GPIO 7 -> ?
 *   GPIO 8 -> route input jack to line-in (0) or mic-in (1)
 *
 * CM9780:
 *
 *   LINE_OUT -> input of ADC
 *
 *   AUX_IN  <- aux
 *   MIC_IN  <- mic
 *   FMIC_IN <- front mic
 *
 *   GPO 0 -> route line-in (0) or AC97 output (1) to CS5361 input
 */




void XonarCS43XXAudioEngine::cs4398_write(struct oxygen *chip, UInt8 reg, UInt8 value, XonarAudioEngine *engineInstance)
{
    struct xonar_cs43xx *data = (struct xonar_cs43xx *)chip->model_data;
    
    engineInstance->oxygen_write_i2c(chip, I2C_DEVICE_CS4398, reg, value);
    if (reg < ARRAY_SIZE(data->cs4398_regs))
        data->cs4398_regs[reg] = value;
}

void XonarCS43XXAudioEngine::cs4398_write_cached(struct oxygen *chip, UInt8 reg, UInt8 value, XonarAudioEngine *engineInstance)
{
    struct xonar_cs43xx *data = (struct xonar_cs43xx *)chip->model_data;
    
    if (value != data->cs4398_regs[reg])
        cs4398_write(chip, reg, value, engineInstance);
}

void XonarCS43XXAudioEngine::cs4362a_write(struct oxygen *chip, UInt8 reg, UInt8 value, XonarAudioEngine *engineInstance)
{
    struct xonar_cs43xx *data = (struct xonar_cs43xx *)chip->model_data;
    
    engineInstance->oxygen_write_i2c(chip, I2C_DEVICE_CS4362A, reg, value);
    if (reg < ARRAY_SIZE(data->cs4362a_regs))
        data->cs4362a_regs[reg] = value;
}

void XonarCS43XXAudioEngine::cs4362a_write_cached(struct oxygen *chip, UInt8 reg, UInt8 value, XonarAudioEngine *engineInstance)
{
    struct xonar_cs43xx *data = (struct xonar_cs43xx *)chip->model_data;
    
    if (value != data->cs4362a_regs[reg])
        cs4362a_write(chip, reg, value,engineInstance);
}


void XonarCS43XXAudioEngine::update_cs4362a_volumes(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    unsigned int i;
    UInt8 mute;
    
    mute = chip->dac_mute ? CS4362A_MUTE : 0;
    for (i = 0; i < 6; ++i)
        cs4362a_write_cached(chip, 7 + i + i / 2,
                             (127 - chip->dac_volume[2 + i]) | mute,engineInstance);
}

void XonarCS43XXAudioEngine::update_cs43xx_volume(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    cs4398_write_cached(chip, 5, (127 - chip->dac_volume[0]) * 2,engineInstance);
    cs4398_write_cached(chip, 6, (127 - chip->dac_volume[1]) * 2, engineInstance);
    update_cs4362a_volumes(chip, engineInstance);
}

void XonarCS43XXAudioEngine::update_cs43xx_mute(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    UInt8 reg;
    
    reg = CS4398_MUTEP_LOW | CS4398_PAMUTE;
    if (chip->dac_mute)
        reg |= CS4398_MUTE_B | CS4398_MUTE_A;
    cs4398_write_cached(chip, 4, reg, engineInstance);
    update_cs4362a_volumes(chip, engineInstance);
}



void XonarCS43XXAudioEngine::cs43xx_registers_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    struct xonar_cs43xx *data = (struct xonar_cs43xx *)chip->model_data;
    unsigned int i;
    
    /* set CPEN (control port mode) and power down */
    cs4398_write(chip, 8, CS4398_CPEN | CS4398_PDN, engineInstance);
    cs4362a_write(chip, 0x01, CS4362A_PDN | CS4362A_CPEN, engineInstance);
    /* configure */
    cs4398_write(chip, 2, data->cs4398_regs[2], engineInstance);
    cs4398_write(chip, 3, CS4398_ATAPI_B_R | CS4398_ATAPI_A_L, engineInstance);
    cs4398_write(chip, 4, data->cs4398_regs[4], engineInstance);
    cs4398_write(chip, 5, data->cs4398_regs[5], engineInstance);
    cs4398_write(chip, 6, data->cs4398_regs[6], engineInstance);
    cs4398_write(chip, 7, data->cs4398_regs[7], engineInstance);
    cs4362a_write(chip, 0x02, CS4362A_DIF_LJUST, engineInstance);
    cs4362a_write(chip, 0x03, CS4362A_MUTEC_6 | CS4362A_AMUTE |
                  CS4362A_RMP_UP | CS4362A_ZERO_CROSS | CS4362A_SOFT_RAMP, engineInstance);
    cs4362a_write(chip, 0x04, data->cs4362a_regs[0x04], engineInstance);
    cs4362a_write(chip, 0x05, 0, engineInstance);
    for (i = 6; i <= 14; ++i)
        cs4362a_write(chip, i, data->cs4362a_regs[i], engineInstance);
    /* clear power down */
    cs4398_write(chip, 8, CS4398_CPEN, engineInstance);
    cs4362a_write(chip, 0x01, CS4362A_CPEN, engineInstance);
}


void XonarCS43XXAudioEngine::xonar_d1_cleanup(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    engineInstance->xonar_disable_output(chip);
    cs4362a_write(chip, 0x01, CS4362A_PDN | CS4362A_CPEN, engineInstance);
    oxygen_clear_bits8(chip, OXYGEN_FUNCTION, OXYGEN_FUNCTION_RESET_CODEC);
}

void XonarCS43XXAudioEngine::xonar_d1_suspend(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    xonar_d1_cleanup(chip, engineInstance);
}

void XonarCS43XXAudioEngine::xonar_d1_resume(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    oxygen_set_bits8(chip, OXYGEN_FUNCTION, OXYGEN_FUNCTION_RESET_CODEC);
    IODelay(1);
    cs43xx_registers_init(chip, engineInstance);
    engineInstance->xonar_enable_output(chip);
}

void XonarCS43XXAudioEngine::set_cs43xx_params(struct oxygen *chip,
                                               XonarAudioEngine *engineInstance)
{
    struct xonar_cs43xx *data = (struct xonar_cs43xx *)chip->model_data;
    
    UInt8 cs4398_fm, cs4362a_fm;
    
    if (engineInstance->getSampleRate()->whole <= 50000) {
        cs4398_fm = CS4398_FM_SINGLE;
        cs4362a_fm = CS4362A_FM_SINGLE;
    } else if (engineInstance->getSampleRate()->whole <= 100000) {
        cs4398_fm = CS4398_FM_DOUBLE;
        cs4362a_fm = CS4362A_FM_DOUBLE;
    } else {
        cs4398_fm = CS4398_FM_QUAD;
        cs4362a_fm = CS4362A_FM_QUAD;
    }
    cs4398_fm |= CS4398_DEM_NONE | CS4398_DIF_LJUST;
    cs4398_write_cached(chip, 2, cs4398_fm, engineInstance);
    cs4362a_fm |= data->cs4362a_regs[6] & ~CS4362A_FM_MASK;
    cs4362a_write_cached(chip, 6, cs4362a_fm, engineInstance);
    cs4362a_write_cached(chip, 12, cs4362a_fm, engineInstance);
    cs4362a_fm &= CS4362A_FM_MASK;
    cs4362a_fm |= data->cs4362a_regs[9] & ~CS4362A_FM_MASK;
    cs4362a_write_cached(chip, 9, cs4362a_fm, engineInstance);
}

void XonarCS43XXAudioEngine::xonar_d1_line_mic_ac97_switch(struct oxygen *chip,
                                                           unsigned int reg, unsigned int mute)
{
    if (reg == AC97_LINE) {
        OSSpinLockTry(&chip->reg_lock);
        oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                              mute ? GPIO_D1_INPUT_ROUTE : 0,
                              GPIO_D1_INPUT_ROUTE);
        OSSpinLockUnlock(&chip->reg_lock);
    }
}


bool XonarCS43XXAudioEngine::init(XonarAudioEngine *audioEngine, struct oxygen *chip, UInt16 model)
{
    //begin APPUL portion of sampleaudioengine::init
    bool result = false;
    
    IOLog("SamplePCIAudioEngine[%p]::init(%p)\n", this, chip);
    chip->model_data = IOMalloc(chip->model.model_data_size);
    struct xonar_cs43xx *data = (struct xonar_cs43xx *) chip->model_data;
    if (!chip) {
        goto Done;
    }
    
    if (!super::init(NULL)) {
        goto Done;
    }
    
    
    //end APPUL portion of sampleaudioengine::init
    
    /* assign remaining values for xonar_cs43xx struct
     * (majority of assignments are in the 'else' of
     * XonarAudioEngine.cpp
     */
    
    chip->model.set_dac_params = set_cs43xx_params;
    chip->model.set_adc_params = audioEngine->xonar_set_cs53x1_params;
    chip->model.update_dac_volume = update_cs43xx_volume;
    chip->model.update_dac_mute = update_cs43xx_mute;
    // chip->model.update_center_lfe_mix = update_cs43xx_center_lfe_mix;
    chip->model.ac97_switch = xonar_d1_line_mic_ac97_switch;
    // chip->model.dump_registers = dump_d1_registers;
    // chip->model.dac_tlv = cs4362a_db_scale;
    
    //begin D1/DX init.
    
    if(model == DX_MODEL || model == CS4XX_MODEL || model== D1_MODEL) {
        if(model == DX_MODEL) {
            //DX model is the same as D1, so just set the three params
            data->generic.ext_power_reg = OXYGEN_GPI_DATA;
            data->generic.ext_power_int_reg = OXYGEN_GPI_INTERRUPT_MASK;
            data->generic.ext_power_bit = GPI_EXT_POWER;
            engineInstance->xonar_init_ext_power(chip);
        }
        
        //d1_init is required for all CS4XXX cards.

        data->generic.anti_pop_delay = 800;
        data->generic.output_enable_bit = GPIO_D1_OUTPUT_ENABLE;
        data->cs4398_regs[2] =
        CS4398_FM_SINGLE | CS4398_DEM_NONE | CS4398_DIF_LJUST;
        data->cs4398_regs[4] = CS4398_MUTEP_LOW |
        CS4398_MUTE_B | CS4398_MUTE_A | CS4398_PAMUTE;
        data->cs4398_regs[5] = 60 * 2;
        data->cs4398_regs[6] = 60 * 2;
        data->cs4398_regs[7] = CS4398_RMP_DN | CS4398_RMP_UP |
        CS4398_ZERO_CROSS | CS4398_SOFT_RAMP;
        data->cs4362a_regs[4] = CS4362A_RMP_DN | CS4362A_DEM_NONE;
        data->cs4362a_regs[6] = CS4362A_FM_SINGLE |
        CS4362A_ATAPI_B_R | CS4362A_ATAPI_A_L;
        data->cs4362a_regs[7] = 60 | CS4362A_MUTE;
        data->cs4362a_regs[8] = 60 | CS4362A_MUTE;
        data->cs4362a_regs[9] = data->cs4362a_regs[6];
        data->cs4362a_regs[10] = 60 | CS4362A_MUTE;
        data->cs4362a_regs[11] = 60 | CS4362A_MUTE;
        data->cs4362a_regs[12] = data->cs4362a_regs[6];
        data->cs4362a_regs[13] = 60 | CS4362A_MUTE;
        data->cs4362a_regs[14] = 60 | CS4362A_MUTE;
        
        oxygen_write16(chip, OXYGEN_2WIRE_BUS_STATUS,
                       OXYGEN_2WIRE_LENGTH_8 |
                       OXYGEN_2WIRE_INTERRUPT_MASK |
                       OXYGEN_2WIRE_SPEED_FAST);
        
        cs43xx_registers_init(chip, engineInstance);
        
        oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL,
                          GPIO_D1_FRONT_PANEL |
                          GPIO_D1_MAGIC |
                          GPIO_D1_INPUT_ROUTE);
        oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA,
                            GPIO_D1_FRONT_PANEL | GPIO_D1_INPUT_ROUTE);
        
        engineInstance->xonar_init_cs53x1(chip);
        engineInstance->xonar_enable_output(chip);
    }
    else
        goto Done;
    
    //set registers/engine and finish APPUL sampleaudioengine init
    deviceRegisters = (struct xonar_cs43xx*) data;
    this->engineInstance = audioEngine;
    result = true;
    
Done:
    
    return result;
}


bool XonarCS43XXAudioEngine::initHardware(IOService *provider)
{
    bool result = false;
    IOAudioSampleRate initialSampleRate;
    IOAudioStream *audioStream;
    IOWorkLoop *workLoop;
    
    IOLog("XonarCS43XXAudioEngine[%p]::initHardware(%p)\n", this, provider);
    
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
                                                                                    XonarCS43XXAudioEngine::interruptHandler,
                                                                                    XonarCS43XXAudioEngine::interruptFilter,
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

void XonarCS43XXAudioEngine::free()
{
    IOLog("XonarCS43XXAudioEngine[%p]::free()\n", this);
    
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

IOAudioStream *XonarCS43XXAudioEngine::createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize)
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

void XonarCS43XXAudioEngine::stop(IOService *provider)
{
    IOLog("XonarCS43XXAudioEngine[%p]::stop(%p)\n", this, provider);
    
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

IOReturn XonarCS43XXAudioEngine::performAudioEngineStart()
{
    IOLog("XonarCS43XXAudioEngine[%p]::performAudioEngineStart()\n", this);
    
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

IOReturn XonarCS43XXAudioEngine::performAudioEngineStop()
{
    IOLog("XonarCS43XXAudioEngine[%p]::performAudioEngineStop()\n", this);
    
    // Assuming we don't need interrupts after stopping the audio engine, we can disable them here
    assert(interruptEventSource);
    interruptEventSource->disable();
    
    // Add audio - I/O stop code here
    
    //#error performAudioEngineStop() - driver will not work until audio engine stop code is added
    
    return kIOReturnSuccess;
}

UInt32 XonarCS43XXAudioEngine::getCurrentSampleFrame()
{
    IOLog("XonarCS43XXAudioEngine[%p]::getCurrentSampleFrame()\n", this);
    
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

IOReturn XonarCS43XXAudioEngine::performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate)
{
    IOLog("XonarCS43XXAudioEngine[%p]::peformFormatChange(%p, %p, %p)\n", this, audioStream, newFormat, newSampleRate);
    
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


void XonarCS43XXAudioEngine::interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count)
{
    // Since our interrupt filter always returns false, this function will never be called
    // If the filter returned true, this function would be called on the IOWorkLoop
    return;
}

bool XonarCS43XXAudioEngine::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source)
{
    XonarCS43XXAudioEngine *audioEngine = OSDynamicCast(XonarCS43XXAudioEngine, owner);
    
    // We've cast the audio engine from the owner which we passed in when we created the interrupt
    // event source
    if (audioEngine) {
        // Then, filterInterrupt() is called on the specified audio engine
        audioEngine->filterInterrupt(source->getIntIndex());
    }
    
    return false;
}

void XonarCS43XXAudioEngine::filterInterrupt(int index)
{
    // In the case of our simple device, we only get interrupts when the audio engine loops to the
    // beginning of the buffer.  When that happens, we need to take a timestamp and increment
    // the loop count.  The function takeTimeStamp() does both of those for us.  Additionally,
    // if a different timestamp is to be used (other than the current time), it can be passed
    // in to takeTimeStamp()
    takeTimeStamp();
}
/*
 void XonarCS43XXAudioEngine::update_cs43xx_center_lfe_mix(struct oxygen *chip, bool mixed)
 {
 struct xonar_cs43xx *data = (struct xonar_cs43xx *)chip->model_data;
 
 UInt8 reg;
 
 reg = data->cs4362a_regs[9] & ~CS4362A_ATAPI_MASK;
 if (mixed)
 reg |= CS4362A_ATAPI_B_LR | CS4362A_ATAPI_A_LR;
 else
 reg |= CS4362A_ATAPI_B_R | CS4362A_ATAPI_A_L;
 cs4362a_write_cached(chip, 9, reg);
 }
 
 static const struct snd_kcontrol_new front_panel_switch = {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Front Panel Playback Switch",
 .info = snd_ctl_boolean_mono_info,
 .get = xonar_gpio_bit_switch_get,
 .put = xonar_gpio_bit_switch_put,
 .private_value = GPIO_D1_FRONT_PANEL,
 };
 
 static int rolloff_info(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_info *info)
 {
 static const char *const names[2] = {
 "Fast Roll-off", "Slow Roll-off"
 };
 
 return snd_ctl_enum_info(info, 1, 2, names);
 }
 
 static int rolloff_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_cs43xx *data = chip->model_data;
 
 value->value.enumerated.item[0] =
 (data->cs4398_regs[7] & CS4398_FILT_SEL) != 0;
 return 0;
 }
 
 static int rolloff_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct xonar_cs43xx *data = chip->model_data;
 int changed;
 UInt8 reg;
 
 mutex_lock(&chip->mutex);
 reg = data->cs4398_regs[7];
 if (value->value.enumerated.item[0])
 reg |= CS4398_FILT_SEL;
 else
 reg &= ~CS4398_FILT_SEL;
 changed = reg != data->cs4398_regs[7];
 if (changed) {
 cs4398_write(chip, 7, reg);
 if (reg & CS4398_FILT_SEL)
 reg = data->cs4362a_regs[0x04] | CS4362A_FILT_SEL;
 else
 reg = data->cs4362a_regs[0x04] & ~CS4362A_FILT_SEL;
 cs4362a_write(chip, 0x04, reg);
 }
 mutex_unlock(&chip->mutex);
 return changed;
 }
 
 static const struct snd_kcontrol_new rolloff_control = {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "DAC Filter Playback Enum",
 .info = rolloff_info,
 .get = rolloff_get,
 .put = rolloff_put,
 };
 static int xonar_d1_mixer_init(struct oxygen *chip)
 {
 int err;
 
 err = snd_ctl_add(chip->card, snd_ctl_new1(&front_panel_switch, chip));
 if (err < 0)
 return err;
 err = snd_ctl_add(chip->card, snd_ctl_new1(&rolloff_control, chip));
 if (err < 0)
 return err;
 return 0;
 }
 
 static void dump_cs4362a_registers(struct xonar_cs43xx *data,
 struct snd_info_buffer *buffer)
 {
 unsigned int i;
 
 snd_iprintf(buffer, "\nCS4362A:");
 for (i = 1; i <= 14; ++i)
 snd_iprintf(buffer, " %02x", data->cs4362a_regs[i]);
 snd_iprintf(buffer, "\n");
 }
 
 static void dump_d1_registers(struct oxygen *chip,
 struct snd_info_buffer *buffer)
 {
 struct xonar_cs43xx *data = chip->model_data;
 unsigned int i;
 
 snd_iprintf(buffer, "\nCS4398: 7?");
 for (i = 2; i < 8; ++i)
 snd_iprintf(buffer, " %02x", data->cs4398_regs[i]);
 snd_iprintf(buffer, "\n");
 dump_cs4362a_registers(data, buffer);
 }
 
 
 
 int get_xonar_cs43xx_model(struct oxygen *chip,
 const struct pci_device_id *id)
 {
 switch (id->subdevice) {
 case 0x834f:
 chip->model = model_xonar_d1;
 chip->model.shortname = "Xonar D1";
 break;
 case 0x8275:
 case 0x8327:
 chip->model = model_xonar_d1;
 chip->model.shortname = "Xonar DX";
 chip->model.init = xonar_dx_init;
 break;
 default:
 return -EINVAL;
 }
 return 0;
 }
 
 */
