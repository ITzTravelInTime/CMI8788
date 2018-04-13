/*
 File:SamplePCIAudioEngine.cpp
 
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
#define dev_err(dev, format, args...) do {IOLog("LinuxI2C(dev_err): " format, ##args);} while (0)

#include "SamplePCIAudioEngine.hpp"
#include "ak4396.h"
#include "wm8785.h"
#include "oxygen_regs.h"
#include <libkern/OSByteOrder.h>
#include <sys/errno.h>
#include <i386/limits.h>

#include <IOKit/IOLib.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <architecture/i386/pio.h>

#define INITIAL_SAMPLE_RATE	44100
#define NUM_SAMPLE_FRAMES	16384
#define NUM_CHANNELS		2
#define BIT_DEPTH			16
#define MSEC_PER_SEC        1000L
#define HZ                  1024
#define BUFFER_SIZE			(NUM_SAMPLE_FRAMES * NUM_CHANNELS * BIT_DEPTH / 8)

#define super IOAudioEngine

OSDefineMetaClassAndStructors(SamplePCIAudioEngine, IOAudioEngine)

static inline unsigned long msecs_to_jiffies(const unsigned int m)
{
    return (m + (MSEC_PER_SEC / HZ) - 1) / (MSEC_PER_SEC / HZ);
}

UInt8 oxygen_read8(struct oxygen *chip, unsigned int reg)
{
    return inb(chip->addr + reg);
}
//EXPORT_SYMBOL(oxygen_read8);

UInt16 oxygen_read16(struct oxygen *chip, unsigned int reg)
{
    return inw(chip->addr + reg);
}
//EXPORT_SYMBOL(oxygen_read16);

UInt32 oxygen_read32(struct oxygen *chip, unsigned int reg)
{
    return inl(chip->addr + reg);
}
////EXPORT_SYMBOL(oxygen_read32);

void oxygen_write8(struct oxygen *chip, unsigned int reg, UInt8 value)
{
    outb(value, chip->addr + reg);
    chip->saved_registers._8[reg] = value;
}
//EXPORT_SYMBOL(oxygen_write8);

void oxygen_write16(struct oxygen *chip, unsigned int reg, UInt16 value)
{
    outw(value, chip->addr + reg);
    chip->saved_registers._16[reg / 2] = OSSwapHostToLittleInt16(value);
}
//EXPORT_SYMBOL(oxygen_write16);

void oxygen_write32(struct oxygen *chip, unsigned int reg, UInt32 value)
{
    outl(value, chip->addr + reg);
    chip->saved_registers._32[reg / 4] = OSSwapHostToLittleInt16(value);
}
//EXPORT_SYMBOL(oxygen_write32);

void oxygen_write8_masked(struct oxygen *chip, unsigned int reg,
                          UInt8 value, UInt8 mask)
{
    UInt8 tmp = inb(chip->addr + reg);
    tmp &= ~mask;
    tmp |= value & mask;
    outb(tmp, chip->addr + reg);
    chip->saved_registers._8[reg] = tmp;
}
//EXPORT_SYMBOL(oxygen_write8_masked);

void oxygen_write16_masked(struct oxygen *chip, unsigned int reg,
                           UInt16 value, UInt16 mask)
{
    UInt16 tmp = inw(chip->addr + reg);
    tmp &= ~mask;
    tmp |= value & mask;
    outw(tmp, chip->addr + reg);
    chip->saved_registers._16[reg / 2] = OSSwapHostToLittleInt16(tmp);
}
//EXPORT_SYMBOL(oxygen_write16_masked);

void oxygen_write32_masked(struct oxygen *chip, unsigned int reg,
                           UInt32 value, UInt32 mask)
{
    UInt32 tmp = inl(chip->addr + reg);
    tmp &= ~mask;
    tmp |= value & mask;
    outl(tmp, chip->addr + reg);
    chip->saved_registers._32[reg / 4] = OSSwapHostToLittleInt32(tmp);
}
//EXPORT_SYMBOL(oxygen_write32_masked);

static int oxygen_ac97_wait(struct oxygen *chip, unsigned int mask)
{
    UInt8 status = 0;
    
    /*
     * Reading the status register also clears the bits, so we have to save
     * the read bits in status.
     */
    wait_event_timeout(chip->ac97_waitqueue,
                       ({ status |= oxygen_read8(chip, OXYGEN_AC97_INTERRUPT_STATUS);
        status & mask; }),
                       msecs_to_jiffies(1) + 1);
    /*
     * Check even after a timeout because this function should not require
     * the AC'97 interrupt to be enabled.
     */
    status |= oxygen_read8(chip, OXYGEN_AC97_INTERRUPT_STATUS);
    return status & mask ? 0 : -EIO;
}

/*
 * About 10% of AC'97 register reads or writes fail to complete, but even those
 * where the controller indicates completion aren't guaranteed to have actually
 * happened.
 *
 * It's hard to assign blame to either the controller or the codec because both
 * were made by C-Media ...
 */

void oxygen_write_ac97(struct oxygen *chip, unsigned int codec,
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
    dev_err(chip->card->dev, "AC'97 write timeout\n");
}
//EXPORT_SYMBOL(oxygen_write_ac97);

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
    dev_err(chip->card->dev, "AC'97 read timeout on codec %u\n", codec);
    return 0;
}
//EXPORT_SYMBOL(oxygen_read_ac97);

void oxygen_write_ac97_masked(struct oxygen *chip, unsigned int codec,
                              unsigned int index, UInt16 data, UInt16 mask)
{
    UInt16 value = oxygen_read_ac97(chip, codec, index);
    value &= ~mask;
    value |= data & mask;
    oxygen_write_ac97(chip, codec, index, value);
}
//EXPORT_SYMBOL(oxygen_write_ac97_masked);

static int oxygen_wait_spi(struct oxygen *chip)
{
    unsigned int count;
    
    /*
     * Higher timeout to be sure: 200 us;
     * actual transaction should not need more than 40 us.
     */
    for (count = 50; count > 0; count--) {
        IODelay(4);
        if ((oxygen_read8(chip, OXYGEN_SPI_CONTROL) &
             OXYGEN_SPI_BUSY) == 0)
            return 0;
    }
    dev_err(chip->card->dev, "oxygen: SPI wait timeout\n");
    return -EIO;
}

int oxygen_write_spi(struct oxygen *chip, UInt8 control, unsigned int data)
{
    /*
     * We need to wait AFTER initiating the SPI transaction,
     * otherwise read operations will not work.
     */
    oxygen_write8(chip, OXYGEN_SPI_DATA1, data);
    oxygen_write8(chip, OXYGEN_SPI_DATA2, data >> 8);
    if (control & OXYGEN_SPI_DATA_LENGTH_3)
        oxygen_write8(chip, OXYGEN_SPI_DATA3, data >> 16);
    oxygen_write8(chip, OXYGEN_SPI_CONTROL, control);
    return oxygen_wait_spi(chip);
}
//EXPORT_SYMBOL(oxygen_write_spi);

void oxygen_write_i2c(struct oxygen *chip, UInt8 device, UInt8 map, UInt8 data)
{
    /* should not need more than about 300 us */
    IODelay(1000);
    
    oxygen_write8(chip, OXYGEN_2WIRE_MAP, map);
    oxygen_write8(chip, OXYGEN_2WIRE_DATA, data);
    oxygen_write8(chip, OXYGEN_2WIRE_CONTROL,
                  device | OXYGEN_2WIRE_DIR_WRITE);
}
//EXPORT_SYMBOL(oxygen_write_i2c);

static void _write_uart(struct oxygen *chip, unsigned int port, UInt8 data)
{
    if (oxygen_read8(chip, OXYGEN_MPU401 + 1) & MPU401_TX_FULL)
        IODelay(1e3);
    oxygen_write8(chip, OXYGEN_MPU401 + port, data);
}

void oxygen_reset_uart(struct oxygen *chip)
{
    _write_uart(chip, 1, MPU401_RESET);
    IODelay(1e3); /* wait for ACK */
    _write_uart(chip, 1, MPU401_ENTER_UART);
}
//EXPORT_SYMBOL(oxygen_reset_uart);

void oxygen_write_uart(struct oxygen *chip, UInt8 data)
{
    _write_uart(chip, 0, data);
}
//EXPORT_SYMBOL(oxygen_write_uart);

UInt16 oxygen_read_eeprom(struct oxygen *chip, unsigned int index)
{
    unsigned int timeout;
    
    oxygen_write8(chip, OXYGEN_EEPROM_CONTROL,
                  index | OXYGEN_EEPROM_DIR_READ);
    for (timeout = 0; timeout < 100; ++timeout) {
    //    udelay(1);
        if (!(oxygen_read8(chip, OXYGEN_EEPROM_STATUS)
              & OXYGEN_EEPROM_BUSY))
            break;
    }
    return oxygen_read16(chip, OXYGEN_EEPROM_DATA);
}

void oxygen_write_eeprom(struct oxygen *chip, unsigned int index, UInt16 value)
{
    unsigned int timeout;
    
    oxygen_write16(chip, OXYGEN_EEPROM_DATA, value);
    oxygen_write8(chip, OXYGEN_EEPROM_CONTROL,
                  index | OXYGEN_EEPROM_DIR_WRITE);
    for (timeout = 0; timeout < 10; ++timeout) {
        IODelay(1e3);
        if (!(oxygen_read8(chip, OXYGEN_EEPROM_STATUS)
              & OXYGEN_EEPROM_BUSY))
            return;
    }
    //dev_err(chip->card->dev, "EEPROM write timeout\n");
}


static inline void oxygen_set_bits8(struct oxygen *chip,
                                    unsigned int reg, UInt8 value)
{
    oxygen_write8_masked(chip, reg, value, value);
}

static inline void oxygen_set_bits16(struct oxygen *chip,
                                     unsigned int reg, UInt16 value)
{
    oxygen_write16_masked(chip, reg, value, value);
}

static inline void oxygen_set_bits32(struct oxygen *chip,
                                     unsigned int reg, UInt32 value)
{
    oxygen_write32_masked(chip, reg, value, value);
}

static inline void oxygen_clear_bits8(struct oxygen *chip,
                                      unsigned int reg, UInt8 value)
{
    oxygen_write8_masked(chip, reg, 0, value);
}

static inline void oxygen_clear_bits16(struct oxygen *chip,
                                       unsigned int reg, UInt16 value)
{
    oxygen_write16_masked(chip, reg, 0, value);
}

static inline void oxygen_clear_bits32(struct oxygen *chip,
                                       unsigned int reg, UInt32 value)
{
    oxygen_write32_masked(chip, reg, 0, value);
}

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


static void ak4396_write(struct oxygen *chip, unsigned int codec,
                         UInt8 reg, UInt8 value)
{
    /* maps ALSA channel pair number to SPI output */
    static const UInt8 codec_spi_map[4] = {
        0, 1, 2, 4
    };
    struct generic_data *data = (struct generic_data*)chip->model_data;
    
                      oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER |
                     OXYGEN_SPI_DATA_LENGTH_2 |
                     OXYGEN_SPI_CLOCK_160 |
                     (codec_spi_map[codec] << OXYGEN_SPI_CODEC_SHIFT) |
                     OXYGEN_SPI_CEN_LATCH_CLOCK_HI,
                     AK4396_WRITE | (reg << 8) | value);
    data->ak4396_regs[codec][reg] = value;
}

static void ak4396_write_cached(struct oxygen *chip, unsigned int codec,
                                UInt8 reg, UInt8 value)
{
    struct generic_data *data = (struct generic_data*)chip->model_data;
    
    if (value != data->ak4396_regs[codec][reg])
        ak4396_write(chip, codec, reg, value);
}

static void wm8785_write(struct oxygen *chip, UInt8 reg, unsigned int value)
{
    struct generic_data *data = (struct generic_data*)chip->model_data;
    
    oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER |
                     OXYGEN_SPI_DATA_LENGTH_2 |
                     OXYGEN_SPI_CLOCK_160 |
                     (3 << OXYGEN_SPI_CODEC_SHIFT) |
                     OXYGEN_SPI_CEN_LATCH_CLOCK_LO,
                     (reg << 9) | value);
    if (reg < ARRAY_SIZE(data->wm8785_regs))
        data->wm8785_regs[reg] = value;
}


static void ak4396_registers_init(struct oxygen *chip)
{
    struct generic_data *data = (struct generic_data *)chip->model_data;
    unsigned int i;
    
    for (i = 0; i < data->dacs; ++i) {
        ak4396_write(chip, i, AK4396_CONTROL_1,
                     AK4396_DIF_24_MSB | AK4396_RSTN);
        ak4396_write(chip, i, AK4396_CONTROL_2,
                     data->ak4396_regs[0][AK4396_CONTROL_2]);
        ak4396_write(chip, i, AK4396_CONTROL_3,
                     AK4396_PCM);
        ak4396_write(chip, i, AK4396_LCH_ATT,
                     chip->dac_volume[i * 2]);
        ak4396_write(chip, i, AK4396_RCH_ATT,
                     chip->dac_volume[i * 2 + 1]);
    }
}

static void ak4396_init(struct oxygen *chip)
{
    struct generic_data *data =(struct generic_data*) chip->model_data;
    
    data->dacs = chip->model.dac_channels_pcm / 2;
    data->ak4396_regs[0][AK4396_CONTROL_2] =
    AK4396_SMUTE | AK4396_DEM_OFF | AK4396_DFS_NORMAL;
    ak4396_registers_init(chip);
  //  snd_component_add(chip->card, "AK4396");
}


static void wm8785_registers_init(struct oxygen *chip)
{
    struct generic_data *data = (struct generic_data*)chip->model_data;
    
    wm8785_write(chip, WM8785_R7, 0);
    wm8785_write(chip, WM8785_R0, data->wm8785_regs[0]);
    wm8785_write(chip, WM8785_R2, data->wm8785_regs[2]);
}

static void wm8785_init(struct oxygen *chip)
{
    struct generic_data *data = (struct generic_data*)chip->model_data;
    
    data->wm8785_regs[0] =
    WM8785_MCR_SLAVE | WM8785_OSR_SINGLE | WM8785_FORMAT_LJUST;
    data->wm8785_regs[2] = WM8785_HPFR | WM8785_HPFL;
    wm8785_registers_init(chip);
   // snd_component_add(chip->card, "WM8785");
}


bool SamplePCIAudioEngine::init(struct oxygen *chip)
{
    bool result = false;
    
    IOLog("SamplePCIAudioEngine[%p]::init(%p)\n", this, chip);
    
    if (!chip) {
        goto Done;
    }
    
    if (!super::init(NULL)) {
        goto Done;
    }
    ak4396_init(chip);
    wm8785_init(chip);
    deviceRegisters = (struct generic_data*)chip->model_data;
    
    result = true;
    
Done:
    
    return result;
}

bool SamplePCIAudioEngine::initHardware(IOService *provider)
{
    bool result = false;
    IOAudioSampleRate initialSampleRate;
    IOAudioStream *audioStream;
    IOWorkLoop *workLoop;
    
    IOLog("SamplePCIAudioEngine[%p]::initHardware(%p)\n", this, provider);
    
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
                                                                                    SamplePCIAudioEngine::interruptHandler,
                                                                                    SamplePCIAudioEngine::interruptFilter,
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
    outputBuffer = (SInt16 *)IOMalloc(BUFFER_SIZE);
    if (!outputBuffer) {
        goto Done;
    }
    
    inputBuffer = (SInt16 *)IOMalloc(BUFFER_SIZE);
    if (!inputBuffer) {
        goto Done;
    }
    
    // Create an IOAudioStream for each buffer and add it to this audio engine
    audioStream = createNewAudioStream(kIOAudioStreamDirectionOutput, outputBuffer, BUFFER_SIZE);
    if (!audioStream) {
        goto Done;
    }
    
    addAudioStream(audioStream);
    audioStream->release();
    
    audioStream = createNewAudioStream(kIOAudioStreamDirectionInput, inputBuffer, BUFFER_SIZE);
    if (!audioStream) {
        goto Done;
    }
    
    addAudioStream(audioStream);
    audioStream->release();
    
    result = true;
    
Done:
    
    return result;
}

void SamplePCIAudioEngine::free()
{
    IOLog("SamplePCIAudioEngine[%p]::free()\n", this);
    
    // We need to free our resources when we're going away
    
    if (interruptEventSource) {
        interruptEventSource->release();
        interruptEventSource = NULL;
    }
    
    if (outputBuffer) {
        IOFree(outputBuffer, BUFFER_SIZE);
        outputBuffer = NULL;
    }
    
    if (inputBuffer) {
        IOFree(inputBuffer, BUFFER_SIZE);
        inputBuffer = NULL;
    }
    
    super::free();
}

IOAudioStream *SamplePCIAudioEngine::createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize)
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

void SamplePCIAudioEngine::stop(IOService *provider)
{
    IOLog("SamplePCIAudioEngine[%p]::stop(%p)\n", this, provider);
    
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

IOReturn SamplePCIAudioEngine::performAudioEngineStart()
{
    IOLog("SamplePCIAudioEngine[%p]::performAudioEngineStart()\n", this);
    
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
    
#error performAudioEngineStart() - driver will not work until audio engine start code is added
    
    return kIOReturnSuccess;
}

IOReturn SamplePCIAudioEngine::performAudioEngineStop()
{
    IOLog("SamplePCIAudioEngine[%p]::performAudioEngineStop()\n", this);
    
    // Assuming we don't need interrupts after stopping the audio engine, we can disable them here
    assert(interruptEventSource);
    interruptEventSource->disable();
    
    // Add audio - I/O stop code here
    
#error performAudioEngineStop() - driver will not work until audio engine stop code is added
    
    return kIOReturnSuccess;
}

UInt32 SamplePCIAudioEngine::getCurrentSampleFrame()
{
    IOLog("SamplePCIAudioEngine[%p]::getCurrentSampleFrame()\n", this);
    
    // In order for the erase process to run properly, this function must return the current location of
    // the audio engine - basically a sample counter
    // It doesn't need to be exact, but if it is inexact, it should err towards being before the current location
    // rather than after the current location.  The erase head will erase up to, but not including the sample
    // frame returned by this function.  If it is too large a value, sound data that hasn't been played will be
    // erased.
    
#error getCurrentSampleFrame() - driver will not work until correct sample frame is returned
    
    // Change to return the real value
    return 0;
}

IOReturn SamplePCIAudioEngine::performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate)
{
    IOLog("SamplePCIAudioEngine[%p]::peformFormatChange(%p, %p, %p)\n", this, audioStream, newFormat, newSampleRate);
    
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


void SamplePCIAudioEngine::interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count)
{
    // Since our interrupt filter always returns false, this function will never be called
    // If the filter returned true, this function would be called on the IOWorkLoop
    return;
}

bool SamplePCIAudioEngine::interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source)
{
    SamplePCIAudioEngine *audioEngine = OSDynamicCast(SamplePCIAudioEngine, owner);
    
    // We've cast the audio engine from the owner which we passed in when we created the interrupt
    // event source
    if (audioEngine) {
        // Then, filterInterrupt() is called on the specified audio engine
        audioEngine->filterInterrupt(source->getIntIndex());
    }
    
    return false;
}

void SamplePCIAudioEngine::filterInterrupt(int index)
{
    // In the case of our simple device, we only get interrupts when the audio engine loops to the
    // beginning of the buffer.  When that happens, we need to take a timestamp and increment
    // the loop count.  The function takeTimeStamp() does both of those for us.  Additionally,
    // if a different timestamp is to be used (other than the current time), it can be passed
    // in to takeTimeStamp()
    takeTimeStamp();
}

