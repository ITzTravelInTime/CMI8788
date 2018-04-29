#import "ak4396.h"
#import "wm8785.h"
#import <libkern/OSByteOrder.h>
#import <sys/errno.h>
#import <i386/limits.h>
#import </usr/include/libkern/OSAtomic.h>
#import <IOKit/IOLib.h>
#import <IOKit/IOFilterInterruptEventSource.h>
#import "XonarGenericAudioEngine.hpp"

#define INITIAL_SAMPLE_RATE	44100
#define NUM_SAMPLE_FRAMES	16384
#define NUM_CHANNELS		2
#define BIT_DEPTH			16


//#import <architecture/i386/pio.h>

/*
 * C-Media CMI8788 driver for C-Media's reference design and similar models
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
 *  along with this driver; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

/*
 * CMI8788:
 *
 *   SPI 0 -> 1st AK4396 (front)
 *   SPI 1 -> 2nd AK4396 (surround)
 *   SPI 2 -> 3rd AK4396 (center/LFE)
 *   SPI 3 -> WM8785
 *   SPI 4 -> 4th AK4396 (back)
 *
 *   GPIO 0 -> DFS0 of AK5385
 *   GPIO 1 -> DFS1 of AK5385
 *
 * X-Meridian models:
 *   GPIO 4 -> enable extension S/PDIF input
 *   GPIO 6 -> enable on-board S/PDIF input
 *
 * Claro models:
 *   GPIO 6 -> S/PDIF from optical (0) or coaxial (1) input
 *   GPIO 8 -> enable headphone amplifier
 *
 * CM9780:
 *
 *   LINE_OUT -> input of ADC
 *
 *   AUX_IN <- aux
 *   CD_IN  <- CD
 *   MIC_IN <- mic
 *
 *   GPO 0 -> route line-in (0) or AC97 output (1) to ADC input
 */

#define super IOAudioEngine

OSDefineMetaClassAndStructors(XonarGenericAudioEngine, IOAudioEngine)



//static const struct pci_device_id oxygen_ids[] = {
//    /* C-Media's reference design */
//    { OXYGEN_PCI_SUBID(0x10b0, 0x0216), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x10b0, 0x0217), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x10b0, 0x0218), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x10b0, 0x0219), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x13f6, 0x0001), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x13f6, 0x0010), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x13f6, 0x8788), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x147a, 0xa017), .driver_data = MODEL_CMEDIA_REF },
//    { OXYGEN_PCI_SUBID(0x1a58, 0x0910), .driver_data = MODEL_CMEDIA_REF },
//    /* Asus Xonar DG */
//    { OXYGEN_PCI_SUBID(0x1043, 0x8467), .driver_data = MODEL_XONAR_DG },
//    /* Asus Xonar DGX */
//    { OXYGEN_PCI_SUBID(0x1043, 0x8521), .driver_data = MODEL_XONAR_DGX },
//    /* PCI 2.0 HD Audio */
//    { OXYGEN_PCI_SUBID(0x13f6, 0x8782), .driver_data = MODEL_2CH_OUTPUT },
//    /* Kuroutoshikou CMI8787-HG2PCI */
//    { OXYGEN_PCI_SUBID(0x13f6, 0xffff), .driver_data = MODEL_HG2PCI },
//    /* TempoTec HiFier Fantasia */
//    { OXYGEN_PCI_SUBID(0x14c3, 0x1710), .driver_data = MODEL_FANTASIA },
//    /* TempoTec HiFier Serenade */
//    { OXYGEN_PCI_SUBID(0x14c3, 0x1711), .driver_data = MODEL_SERENADE },
//    /* AuzenTech X-Meridian */
//    { OXYGEN_PCI_SUBID(0x415a, 0x5431), .driver_data = MODEL_MERIDIAN },
//    /* AuzenTech X-Meridian 2G */
//    { OXYGEN_PCI_SUBID(0x5431, 0x017a), .driver_data = MODEL_MERIDIAN_2G },
//    /* HT-Omega Claro */
//    { OXYGEN_PCI_SUBID(0x7284, 0x9761), .driver_data = MODEL_CLARO },
//    /* HT-Omega Claro halo */
//    { OXYGEN_PCI_SUBID(0x7284, 0x9781), .driver_data = MODEL_CLARO_HALO },
//    { }
//};





void XonarGenericAudioEngine::wm8785_write(struct oxygen *chip, UInt8 reg, unsigned int value,
                                           XonarAudioEngine *instance)
{
    
    struct generic_data *data = (struct generic_data*) chip->model_data;
    
    instance->oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER |
                     OXYGEN_SPI_DATA_LENGTH_2 |
                     OXYGEN_SPI_CLOCK_160 |
                     (3 << OXYGEN_SPI_CODEC_SHIFT) |
                     OXYGEN_SPI_CEN_LATCH_CLOCK_LO,
                     (reg << 9) | value);
    if (reg < ARRAY_SIZE(data->wm8785_regs))
        data->wm8785_regs[reg] = value;
}


void XonarGenericAudioEngine::ak4396_write(struct oxygen *chip, unsigned int codec,
                                    UInt8 reg, UInt8 value, XonarAudioEngine *instance)
{
    /* maps ALSA channel pair number to SPI output */
    static const UInt8 codec_spi_map[4] = {
        0, 1, 2, 4
    };
    struct generic_data *data = (struct generic_data*) chip->model_data;
    
    instance->oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER |
                     OXYGEN_SPI_DATA_LENGTH_2 |
                     OXYGEN_SPI_CLOCK_160 |
                     (codec_spi_map[codec] << OXYGEN_SPI_CODEC_SHIFT) |
                     OXYGEN_SPI_CEN_LATCH_CLOCK_HI,
                     AK4396_WRITE | (reg << 8) | value);
    data->ak4396_regs[codec][reg] = value;
}


void XonarGenericAudioEngine::ak4396_write_cached(struct oxygen *chip, unsigned int codec,
                                                  UInt8 reg, UInt8 value, XonarAudioEngine *engineInstance)
{
    struct generic_data *data = (struct generic_data*) chip->model_data;
    
    if (value != data->ak4396_regs[codec][reg])
        ak4396_write(chip, codec, reg, value, engineInstance);
}


void XonarGenericAudioEngine::ak4396_registers_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    struct generic_data *data = (struct generic_data*) chip->model_data;
    unsigned int i;
    
    for (i = 0; i < data->dacs; ++i) {
        ak4396_write(chip, i, AK4396_CONTROL_1,
                                     AK4396_DIF_24_MSB | AK4396_RSTN, engineInstance);
        ak4396_write(chip, i, AK4396_CONTROL_2,
                                     data->ak4396_regs[0][AK4396_CONTROL_2],engineInstance);
        ak4396_write(chip, i, AK4396_CONTROL_3,
                                     AK4396_PCM,engineInstance);
        ak4396_write(chip, i, AK4396_LCH_ATT,
                                     chip->dac_volume[i * 2],engineInstance);
        ak4396_write(chip, i, AK4396_RCH_ATT,
                                     chip->dac_volume[i * 2 + 1],engineInstance);
    }
}

void XonarGenericAudioEngine::ak4396_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    struct generic_data *data = (struct generic_data*) chip->model_data;
    
    data->dacs = chip->model.dac_channels_pcm / 2;
    data->ak4396_regs[0][AK4396_CONTROL_2] =
    AK4396_SMUTE | AK4396_DEM_OFF | AK4396_DFS_NORMAL;
    ak4396_registers_init(chip, engineInstance);
    // snd_component_add(chip->card, "AK4396");
}

void XonarGenericAudioEngine::ak5385_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_AK5385_DFS_MASK);
    oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_AK5385_DFS_MASK);
    //  snd_component_add(chip->card, "AK5385");
}

void XonarGenericAudioEngine::wm8785_registers_init(struct oxygen *chip, XonarAudioEngine* engineInstance)
{
    struct generic_data *data = (struct generic_data*) chip->model_data;
    
    wm8785_write(chip, WM8785_R7, 0, engineInstance);
    wm8785_write(chip, WM8785_R0, data->wm8785_regs[0], engineInstance);
    wm8785_write(chip, WM8785_R2, data->wm8785_regs[2], engineInstance);
}

void XonarGenericAudioEngine::wm8785_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    struct generic_data *data = (struct generic_data*) chip->model_data;
    
    data->wm8785_regs[0] =
    WM8785_MCR_SLAVE | WM8785_OSR_SINGLE | WM8785_FORMAT_LJUST;
    data->wm8785_regs[2] = WM8785_HPFR | WM8785_HPFL;
    wm8785_registers_init(chip, engineInstance);
    // snd_component_add(chip->card, "WM8785");
}


void XonarGenericAudioEngine::generic_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    ak4396_init(chip, engineInstance);
    wm8785_init(chip, engineInstance);
}

void XonarGenericAudioEngine::meridian_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL,
                      GPIO_MERIDIAN_DIG_MASK);
    oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                          GPIO_MERIDIAN_DIG_BOARD, GPIO_MERIDIAN_DIG_MASK);
    ak4396_init(chip, engineInstance);
    ak5385_init(chip, engineInstance);
}

void XonarGenericAudioEngine::claro_enable_hp(struct oxygen *chip)
{
    IODelay(300);
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_CLARO_HP);
    oxygen_set_bits16(chip, OXYGEN_GPIO_DATA, GPIO_CLARO_HP);
}

void XonarGenericAudioEngine::claro_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_CLARO_DIG_COAX);
    oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_CLARO_DIG_COAX);
    ak4396_init(chip, engineInstance);
    wm8785_init(chip, engineInstance);
    claro_enable_hp(chip);
}

void XonarGenericAudioEngine::claro_halo_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_CLARO_DIG_COAX);
    oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_CLARO_DIG_COAX);
    ak4396_init(chip, engineInstance);
    ak5385_init(chip, engineInstance);
    claro_enable_hp(chip);
}

void XonarGenericAudioEngine::fantasia_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    ak4396_init(chip, engineInstance);
    //snd_component_add(chip->card, "CS5340");
}

void XonarGenericAudioEngine::stereo_output_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    ak4396_init(chip, engineInstance);
}

void XonarGenericAudioEngine::generic_cleanup(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
}

void XonarGenericAudioEngine::claro_disable_hp(struct oxygen *chip)
{
    oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_CLARO_HP);
}

void XonarGenericAudioEngine::claro_cleanup(struct oxygen *chip)
{
    claro_disable_hp(chip);
}

void XonarGenericAudioEngine::claro_suspend(struct oxygen *chip)
{
    claro_disable_hp(chip);
}

void XonarGenericAudioEngine::generic_resume(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    ak4396_registers_init(chip, engineInstance);
    wm8785_registers_init(chip, engineInstance);
}

void XonarGenericAudioEngine::meridian_resume(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    ak4396_registers_init(chip, engineInstance);
}

void XonarGenericAudioEngine::claro_resume(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    ak4396_registers_init(chip, engineInstance);
    claro_enable_hp(chip);
}

void XonarGenericAudioEngine::stereo_resume(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    ak4396_registers_init(chip, engineInstance);
}

void XonarGenericAudioEngine::set_ak4396_params(struct oxygen *chip,
                                                XonarAudioEngine *engineInstance)
{
    struct generic_data *data = (struct generic_data*) chip->model_data;
    unsigned int i;
    UInt8 value;
    
    value = data->ak4396_regs[0][AK4396_CONTROL_2] & ~AK4396_DFS_MASK;
    if (engineInstance->getSampleRate()->whole <= 54000)
        value |= AK4396_DFS_NORMAL;
    else if (engineInstance->getSampleRate()->whole <= 108000)
        value |= AK4396_DFS_DOUBLE;
    else
        value |= AK4396_DFS_QUAD;
    
    IODelay(1); /* wait for the new MCLK to become stable */
    
    if (value != data->ak4396_regs[0][AK4396_CONTROL_2]) {
        for (i = 0; i < data->dacs; ++i) {
            ak4396_write(chip, i, AK4396_CONTROL_1,
                                         AK4396_DIF_24_MSB, engineInstance);
            ak4396_write(chip, i, AK4396_CONTROL_2, value, engineInstance);
            ak4396_write(chip, i, AK4396_CONTROL_1,
                                         AK4396_DIF_24_MSB | AK4396_RSTN, engineInstance);
        }
    }
}
/*
 void XonarGenericAudioEngine::update_ak4396_volume(struct oxygen *chip)
 {
 struct generic_data *data = (struct generic_data*) chip->model_data;
 unsigned int i;
 
 for (i = 0; i < data->dacs; ++i) {
 ak4396_write_cached(chip, i, AK4396_LCH_ATT,
 chip->dac_volume[i * 2]);
 ak4396_write_cached(chip, i, AK4396_RCH_ATT,
 chip->dac_volume[i * 2 + 1]);
 }
 }
 
 void XonarGenericAudioEngine::update_ak4396_mute(struct oxygen *chip)
 {
 struct generic_data *data = (struct generic_data*) chip->model_data;
 unsigned int i;
 UInt8 value;
 
 value = data->ak4396_regs[0][AK4396_CONTROL_2] & ~AK4396_SMUTE;
 if (chip->dac_mute)
 value |= AK4396_SMUTE;
 for (i = 0; i < data->dacs; ++i)
 ak4396_write_cached(chip, i, AK4396_CONTROL_2, value);
 }
 */
void XonarGenericAudioEngine::set_wm8785_params(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
    struct generic_data *data = (struct generic_data*) chip->model_data;
    unsigned int value;
    
    value = WM8785_MCR_SLAVE | WM8785_FORMAT_LJUST;
    if (engineInstance->getSampleRate()->whole <= 48000)
        value |= WM8785_OSR_SINGLE;
    else if (engineInstance->getSampleRate()->whole <= 96000)
        value |= WM8785_OSR_DOUBLE;
    else
        value |= WM8785_OSR_QUAD;
    if (value != data->wm8785_regs[0]) {
        wm8785_write(chip, WM8785_R7, 0, engineInstance);
        wm8785_write(chip, WM8785_R0, value, engineInstance);
        wm8785_write(chip, WM8785_R2, data->wm8785_regs[2], engineInstance);
    }
}

void XonarGenericAudioEngine::set_ak5385_params(struct oxygen *chip)
{
    unsigned int value;
    
    if (engineInstance->getSampleRate()->whole <= 54000)
        value = GPIO_AK5385_DFS_NORMAL;
    else if (engineInstance->getSampleRate()->whole <= 108000)
        value = GPIO_AK5385_DFS_DOUBLE;
    else
        value = GPIO_AK5385_DFS_QUAD;
    oxygen_write16_masked(chip, OXYGEN_GPIO_DATA,
                          value, GPIO_AK5385_DFS_MASK);
}

bool XonarGenericAudioEngine::init(XonarAudioEngine *audioEngine, struct oxygen *chip, UInt16 model)
{
    //begin APPUL portion of sampleaudioengine::init
    bool result = false;
    
    printf("SamplePCIAudioEngine[%p]::init(%p)\n", this, chip);
    chip->model_data = IOMalloc(chip->model.model_data_size);
    if (!chip) {
        goto Done;
    }
    
    if (!super::init(NULL)) {
        goto Done;
    }
    //end APPUL portion of sampleaudioengine::init
    
    /* assign remaining values for oxygen_generic struct
     * (majority of assignments are in the 'else' of
     * XonarAudioEngine.cpp
     */
    chip->model.shortname = "C-Media CMI8788";
    //chip->model.mixer_init = generic_wm8785_mixer_init,
    chip->model.cleanup = generic_cleanup;
    chip->model.resume = generic_resume;
    /* still not sure about these ALSA dac/adc_{params,volume,mute}
     * fields. i am leaving them commented out as a reference when
     * looking at how to incorporate the equivalent using IOAudiostream
     chip->model.update_dac_volume = update_ak4396_volume;
     chip->model.update_dac_mute = update_ak4396_mute;
     */
    chip->model.set_dac_params = set_ak4396_params;
    chip->model.set_adc_params = set_wm8785_params;
    //chip->model.dump_registers = dump_oxygen_registers;
    //.dac_tlv = ak4396_db_scale,

    //begin generic_init
    ak4396_init(chip,audioEngine);
    wm8785_init(chip,audioEngine);
    //end generic_init
    
    //set registers/engine and finish APPUL sampleaudioengine init
    

    deviceRegisters = (struct xonar_generic*) chip->model_data;
    this->engineInstance = audioEngine;
    result = true;
    
Done:
    
    return result;
}


bool XonarGenericAudioEngine::initHardware(IOService *provider)
{
    bool result = false;
    IOAudioSampleRate initialSampleRate;
    IOAudioStream *audioStream;
    IOWorkLoop *workLoop;
    
    printf("XonarGenericAudioEngine[%p]::initHardware(%p)\n", this, provider);
    
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
                                                                                    XonarGenericAudioEngine::interruptHandler,
                                                                                    XonarGenericAudioEngine::interruptFilter,
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

void XonarGenericAudioEngine::free()
{
    printf("XonarGenericAudioEngine[%p]::free()\n", this);
    
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

IOAudioStream *XonarGenericAudioEngine::createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize)
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

void XonarGenericAudioEngine::stop(IOService *provider)
{
    printf("XonarGenericAudioEngine[%p]::stop(%p)\n", this, provider);
    
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

IOReturn XonarGenericAudioEngine::performAudioEngineStart()
{
    printf("XonarGenericAudioEngine[%p]::performAudioEngineStart()\n", this);
    
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

IOReturn XonarGenericAudioEngine::performAudioEngineStop()
{
    printf("XonarGenericAudioEngine[%p]::performAudioEngineStop()\n", this);
    
    // Assuming we don't need interrupts after stopping the audio engine, we can disable them here
    assert(interruptEventSource);
    interruptEventSource->disable();
    
    // Add audio - I/O stop code here
    
    //#error performAudioEngineStop() - driver will not work until audio engine stop code is added
    
    return kIOReturnSuccess;
}

UInt32 XonarGenericAudioEngine::getCurrentSampleFrame()
{
    printf("XonarGenericAudioEngine[%p]::getCurrentSampleFrame()\n", this);
    
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

IOReturn XonarGenericAudioEngine::performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate)
{
    printf("XonarGenericAudioEngine[%p]::peformFormatChange(%p, %p, %p)\n", this, audioStream, newFormat, newSampleRate);
    
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


void XonarGenericAudioEngine::interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count)
{
    // Since our interrupt filter always returns false, this function will never be called
    // If the filter returned true, this function would be called on the IOWorkLoop
    return;
}

bool XonarGenericAudioEngine::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source)
{
    XonarGenericAudioEngine *audioEngine = OSDynamicCast(XonarGenericAudioEngine, owner);
    
    // We've cast the audio engine from the owner which we passed in when we created the interrupt
    // event source
    if (audioEngine) {
        // Then, filterInterrupt() is called on the specified audio engine
        audioEngine->filterInterrupt(source->getIntIndex());
    }
    
    return false;
}

void XonarGenericAudioEngine::filterInterrupt(int index)
{
    // In the case of our simple device, we only get interrupts when the audio engine loops to the
    // beginning of the buffer.  When that happens, we need to take a timestamp and increment
    // the loop count.  The function takeTimeStamp() does both of those for us.  Additionally,
    // if a different timestamp is to be used (other than the current time), it can be passed
    // in to takeTimeStamp()
    takeTimeStamp();
}



/*
 static void set_no_params(struct oxygen *chip, struct snd_pcm_hw_params *params)
 {
 }
 
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
 struct oxygen *chip = ctl->private_data;
 struct generic_data *data = (struct generic_data*) chip->model_data;
 
 value->value.enumerated.item[0] =
 (data->ak4396_regs[0][AK4396_CONTROL_2] & AK4396_SLOW) != 0;
 return 0;
 }
 
 static int rolloff_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct generic_data *data = chip->model_data;
 unsigned int i;
 int changed;
 UInt8 reg;
 
 mutex_lock(&chip->mutex);
 reg = data->ak4396_regs[0][AK4396_CONTROL_2];
 if (value->value.enumerated.item[0])
 reg |= AK4396_SLOW;
 else
 reg &= ~AK4396_SLOW;
 changed = reg != data->ak4396_regs[0][AK4396_CONTROL_2];
 if (changed) {
 for (i = 0; i < data->dacs; ++i)
 ak4396_write(chip, i, AK4396_CONTROL_2, reg);
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
 
 static int hpf_info(struct snd_kcontrol *ctl, struct snd_ctl_elem_info *info)
 {
 static const char *const names[2] = {
 "None", "High-pass Filter"
 };
 
 return snd_ctl_enum_info(info, 1, 2, names);
 }
 
 static int hpf_get(struct snd_kcontrol *ctl, struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct generic_data *data = chip->model_data;
 
 value->value.enumerated.item[0] =
 (data->wm8785_regs[WM8785_R2] & WM8785_HPFR) != 0;
 return 0;
 }
 
 static int hpf_put(struct snd_kcontrol *ctl, struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 struct generic_data *data = chip->model_data;
 unsigned int reg;
 int changed;
 
 mutex_lock(&chip->mutex);
 reg = data->wm8785_regs[WM8785_R2] & ~(WM8785_HPFR | WM8785_HPFL);
 if (value->value.enumerated.item[0])
 reg |= WM8785_HPFR | WM8785_HPFL;
 changed = reg != data->wm8785_regs[WM8785_R2];
 if (changed)
 wm8785_write(chip, WM8785_R2, reg);
 mutex_unlock(&chip->mutex);
 return changed;
 }
 
 static const struct snd_kcontrol_new hpf_control = {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "ADC Filter Capture Enum",
 .info = hpf_info,
 .get = hpf_get,
 .put = hpf_put,
 };
 
 static int meridian_dig_source_info(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_info *info)
 {
 static const char *const names[2] = { "On-board", "Extension" };
 
 return snd_ctl_enum_info(info, 1, 2, names);
 }
 
 static int claro_dig_source_info(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_info *info)
 {
 static const char *const names[2] = { "Optical", "Coaxial" };
 
 return snd_ctl_enum_info(info, 1, 2, names);
 }
 
 static int meridian_dig_source_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 
 value->value.enumerated.item[0] =
 !!(oxygen_read16(chip, OXYGEN_GPIO_DATA) &
 GPIO_MERIDIAN_DIG_EXT);
 return 0;
 }
 
 static int claro_dig_source_get(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 
 value->value.enumerated.item[0] =
 !!(oxygen_read16(chip, OXYGEN_GPIO_DATA) &
 GPIO_CLARO_DIG_COAX);
 return 0;
 }
 
 static int meridian_dig_source_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 UInt16 old_reg, new_reg;
 int changed;
 
 mutex_lock(&chip->mutex);
 old_reg = oxygen_read16(chip, OXYGEN_GPIO_DATA);
 new_reg = old_reg & ~GPIO_MERIDIAN_DIG_MASK;
 if (value->value.enumerated.item[0] == 0)
 new_reg |= GPIO_MERIDIAN_DIG_BOARD;
 else
 new_reg |= GPIO_MERIDIAN_DIG_EXT;
 changed = new_reg != old_reg;
 if (changed)
 oxygen_write16(chip, OXYGEN_GPIO_DATA, new_reg);
 mutex_unlock(&chip->mutex);
 return changed;
 }
 
 static int claro_dig_source_put(struct snd_kcontrol *ctl,
 struct snd_ctl_elem_value *value)
 {
 struct oxygen *chip = ctl->private_data;
 UInt16 old_reg, new_reg;
 int changed;
 
 mutex_lock(&chip->mutex);
 old_reg = oxygen_read16(chip, OXYGEN_GPIO_DATA);
 new_reg = old_reg & ~GPIO_CLARO_DIG_COAX;
 if (value->value.enumerated.item[0])
 new_reg |= GPIO_CLARO_DIG_COAX;
 changed = new_reg != old_reg;
 if (changed)
 oxygen_write16(chip, OXYGEN_GPIO_DATA, new_reg);
 mutex_unlock(&chip->mutex);
 return changed;
 }
 
 static const struct snd_kcontrol_new meridian_dig_source_control = {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "IEC958 Source Capture Enum",
 .info = meridian_dig_source_info,
 .get = meridian_dig_source_get,
 .put = meridian_dig_source_put,
 };
 
 static const struct snd_kcontrol_new claro_dig_source_control = {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "IEC958 Source Capture Enum",
 .info = claro_dig_source_info,
 .get = claro_dig_source_get,
 .put = claro_dig_source_put,
 };
 
 static int generic_mixer_init(struct oxygen *chip)
 {
 return snd_ctl_add(chip->card, snd_ctl_new1(&rolloff_control, chip));
 }
 
 static int generic_wm8785_mixer_init(struct oxygen *chip)
 {
 int err;
 
 err = generic_mixer_init(chip);
 if (err < 0)
 return err;
 err = snd_ctl_add(chip->card, snd_ctl_new1(&hpf_control, chip));
 if (err < 0)
 return err;
 return 0;
 }
 
 static int meridian_mixer_init(struct oxygen *chip)
 {
 int err;
 
 err = generic_mixer_init(chip);
 if (err < 0)
 return err;
 err = snd_ctl_add(chip->card,
 snd_ctl_new1(&meridian_dig_source_control, chip));
 if (err < 0)
 return err;
 return 0;
 }
 
 static int claro_mixer_init(struct oxygen *chip)
 {
 int err;
 
 err = generic_wm8785_mixer_init(chip);
 if (err < 0)
 return err;
 err = snd_ctl_add(chip->card,
 snd_ctl_new1(&claro_dig_source_control, chip));
 if (err < 0)
 return err;
 return 0;
 }
 
 static int claro_halo_mixer_init(struct oxygen *chip)
 {
 int err;
 
 err = generic_mixer_init(chip);
 if (err < 0)
 return err;
 err = snd_ctl_add(chip->card,
 snd_ctl_new1(&claro_dig_source_control, chip));
 if (err < 0)
 return err;
 return 0;
 }
 
 
 
 static const DECLARE_TLV_DB_LINEAR(ak4396_db_scale, TLV_DB_GAIN_MUTE, 0);
 
 static const struct oxygen_model model_generic = {
 .shortname = "C-Media CMI8788",
 .longname = "C-Media Oxygen HD Audio",
 .chip = "CMI8788",
 .init = generic_init,
 .mixer_init = generic_wm8785_mixer_init,
 .cleanup = generic_cleanup,
 .resume = generic_resume,
 .set_dac_params = set_ak4396_params,
 .set_adc_params = set_wm8785_params,
 .update_dac_volume = update_ak4396_volume,
 .update_dac_mute = update_ak4396_mute,
 .dump_registers = dump_oxygen_registers,
 .dac_tlv = ak4396_db_scale,
 .model_data_size = sizeof(struct generic_data),
 .device_config = PLAYBACK_0_TO_I2S |
 PLAYBACK_1_TO_SPDIF |
 PLAYBACK_2_TO_AC97_1 |
 CAPTURE_0_FROM_I2S_1 |
 CAPTURE_1_FROM_SPDIF |
 CAPTURE_2_FROM_AC97_1 |
 AC97_CD_INPUT,
 .dac_channels_pcm = 8,
 .dac_channels_mixer = 8,
 .dac_volume_min = 0,
 .dac_volume_max = 255,
 .function_flags = OXYGEN_FUNCTION_SPI |
 OXYGEN_FUNCTION_ENABLE_SPI_4_5,
 .dac_mclks = OXYGEN_MCLKS(256, 128, 128),
 .adc_mclks = OXYGEN_MCLKS(256, 256, 128),
 .dac_i2s_format = OXYGEN_I2S_FORMAT_LJUST,
 .adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST,
 };
 
 static int get_oxygen_model(struct oxygen *chip,
 const struct pci_device_id *id)
 {
 static const char *const names[] = {
 [MODEL_MERIDIAN]	= "AuzenTech X-Meridian",
 [MODEL_MERIDIAN_2G]	= "AuzenTech X-Meridian 2G",
 [MODEL_CLARO]		= "HT-Omega Claro",
 [MODEL_CLARO_HALO]	= "HT-Omega Claro halo",
 [MODEL_FANTASIA]	= "TempoTec HiFier Fantasia",
 [MODEL_SERENADE]	= "TempoTec HiFier Serenade",
 [MODEL_HG2PCI]		= "CMI8787-HG2PCI",
 [MODEL_XONAR_DG]        = "Xonar DG",
 [MODEL_XONAR_DGX]       = "Xonar DGX",
 };
 
 chip->model = model_generic;
 switch (id->driver_data) {
 case MODEL_MERIDIAN:
 case MODEL_MERIDIAN_2G:
 chip->model.init = meridian_init;
 chip->model.mixer_init = meridian_mixer_init;
 chip->model.resume = meridian_resume;
 chip->model.set_adc_params = set_ak5385_params;
 chip->model.dump_registers = dump_ak4396_registers;
 chip->model.device_config = PLAYBACK_0_TO_I2S |
 PLAYBACK_1_TO_SPDIF |
 CAPTURE_0_FROM_I2S_2 |
 CAPTURE_1_FROM_SPDIF;
 if (id->driver_data == MODEL_MERIDIAN)
 chip->model.device_config |= AC97_CD_INPUT;
 break;
 case MODEL_CLARO:
 chip->model.init = claro_init;
 chip->model.mixer_init = claro_mixer_init;
 chip->model.cleanup = claro_cleanup;
 chip->model.suspend = claro_suspend;
 chip->model.resume = claro_resume;
 break;
 case MODEL_CLARO_HALO:
 chip->model.init = claro_halo_init;
 chip->model.mixer_init = claro_halo_mixer_init;
 chip->model.cleanup = claro_cleanup;
 chip->model.suspend = claro_suspend;
 chip->model.resume = claro_resume;
 chip->model.set_adc_params = set_ak5385_params;
 chip->model.dump_registers = dump_ak4396_registers;
 chip->model.device_config = PLAYBACK_0_TO_I2S |
 PLAYBACK_1_TO_SPDIF |
 CAPTURE_0_FROM_I2S_2 |
 CAPTURE_1_FROM_SPDIF;
 break;
 case MODEL_FANTASIA:
 case MODEL_SERENADE:
 case MODEL_2CH_OUTPUT:
 case MODEL_HG2PCI:
 chip->model.shortname = "C-Media CMI8787";
 chip->model.chip = "CMI8787";
 if (id->driver_data == MODEL_FANTASIA)
 chip->model.init = fantasia_init;
 else
 chip->model.init = stereo_output_init;
 chip->model.resume = stereo_resume;
 chip->model.mixer_init = generic_mixer_init;
 chip->model.set_adc_params = set_no_params;
 chip->model.dump_registers = dump_ak4396_registers;
 chip->model.device_config = PLAYBACK_0_TO_I2S |
 PLAYBACK_1_TO_SPDIF;
 if (id->driver_data == MODEL_FANTASIA) {
 chip->model.device_config |= CAPTURE_0_FROM_I2S_1;
 chip->model.adc_mclks = OXYGEN_MCLKS(256, 128, 128);
 }
 chip->model.dac_channels_pcm = 2;
 chip->model.dac_channels_mixer = 2;
 break;
 case MODEL_XONAR_DG:
 case MODEL_XONAR_DGX:
 chip->model = model_xonar_dg;
 break;
 }
 if (id->driver_data == MODEL_MERIDIAN ||
 id->driver_data == MODEL_MERIDIAN_2G ||
 id->driver_data == MODEL_CLARO_HALO) {
 chip->model.misc_flags = OXYGEN_MISC_MIDI;
 chip->model.device_config |= MIDI_OUTPUT | MIDI_INPUT;
 }
 if (id->driver_data < ARRAY_SIZE(names) && names[id->driver_data])
 chip->model.shortname = names[id->driver_data];
 return 0;
 }
 
 static int generic_oxygen_probe(struct pci_dev *pci,
 const struct pci_device_id *pci_id)
 {
 static int dev;
 int err;
 
 if (dev >= SNDRV_CARDS)
 return -ENODEV;
 if (!enable[dev]) {
 ++dev;
 return -ENOENT;
 }
 err = oxygen_pci_probe(pci, index[dev], id[dev], THIS_MODULE,
 oxygen_ids, get_oxygen_model);
 if (err >= 0)
 ++dev;
 return err;
 }
 
 static struct pci_driver oxygen_driver = {
 .name = KBUILD_MODNAME,
 .id_table = oxygen_ids,
 .probe = generic_oxygen_probe,
 .remove = oxygen_pci_remove,
 #ifdef CONFIG_PM_SLEEP
 .driver = {
 .pm = &oxygen_pci_pm,
 },
 #endif
 };
 
 module_pci_driver(oxygen_driver);
 */