/*
 File:SamplePCIAudioDevice.cpp
 
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

//#include <libkern/OSAtomic.h>

#include <IOKit/audio/IOAudioControl.h>
#include <IOKit/audio/IOAudioLevelControl.h>
#include <IOKit/audio/IOAudioToggleControl.h>
#include <IOKit/audio/IOAudioDefines.h>

#include <IOKit/IOLib.h>

#include <IOKit/pci/IOPCIDevice.h>
#include "xonar_hdav.hpp"
#include "CMI8788.hpp"
#include "cm9780.h"
#include "ac97.h"



#define super IOAudioDevice

OSDefineMetaClassAndStructors(PCIAudioDevice, IOAudioDevice)



//static void oxygen_gpio_changed(struct work_struct *work)
//{
//    struct oxygen *chip = container_of(work, struct oxygen, gpio_work);
//    
//    if (chip->model.gpio_changed)
//        chip->model.gpio_changed(chip);
//}


//const struct pci_device_id *
//PCIAudioDevice::oxygen_search_pci_id(struct oxygen *chip, const struct pci_device_id ids[])
//{
//    UInt16 subdevice;
//    
//    /*
//     * Make sure the EEPROM pins are available, i.e., not used for SPI.
//     * (This function is called before we initialize or use SPI.)
//     */
//    oxygen_clear_bits8(chip, OXYGEN_FUNCTION,
//                       OXYGEN_FUNCTION_ENABLE_SPI_4_5);
//    /*
//     * Read the subsystem device ID directly from the EEPROM, because the
//     * chip didn't if the first EEPROM word was overwritten.
//     */
//    subdevice = oxygen_read_eeprom(chip, 2);
//    /* use default ID if EEPROM is missing */
//    if (subdevice == 0xffff && oxygen_read_eeprom(chip, 1) == 0xffff)
//        subdevice = 0x8788;
//    /*
//     * We use only the subsystem device ID for searching because it is
//     * unique even without the subsystem vendor ID, which may have been
//     * overwritten in the EEPROM.
//     */
//    for (; ids->vendor; ++ids)
//        if (ids->subdevice == subdevice &&
//            ids->driver_data != BROKEN_EEPROM_DRIVER_DATA)
//            return ids;
//    return NULL;
//}




void PCIAudioDevice::oxygen_init(struct oxygen *chip)
{
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
}






UInt16 PCIAudioDevice::oxygen_read_eeprom(struct oxygen *chip, unsigned int index)
{
    unsigned int timeout;
    
    oxygen_write8(chip, OXYGEN_EEPROM_CONTROL,
                  index | OXYGEN_EEPROM_DIR_READ);
    for (timeout = 0; timeout < 100; ++timeout) {
        IODelay(1);
        if (!(oxygen_read8(chip, OXYGEN_EEPROM_STATUS)
              & OXYGEN_EEPROM_BUSY))
            break;
    }
    return oxygen_read16(chip, OXYGEN_EEPROM_DATA);
}

void PCIAudioDevice::oxygen_write_eeprom(struct oxygen *chip, unsigned int index, UInt16 value)
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
    dev_err(chip->card->dev, "EEPROM write timeout\n");
}


void PCIAudioDevice::oxygen_restore_eeprom(IOPCIDevice *device, struct oxygen *chip)
                                 // const struct pci_device_id *id)
{
    UInt16 eeprom_id;
    
    eeprom_id = oxygen_read_eeprom(chip, 0);
    if (eeprom_id != OXYGEN_EEPROM_ID &&
        (eeprom_id != 0xffff)) { //|| device-> != 0x8788)) {
        /*
         * This function gets called only when a known card model has
         * been detected, i.e., we know there is a valid subsystem
         * product ID at index 2 in the EEPROM.  Therefore, we have
         * been able to deduce the correct subsystem vendor ID, and
         * this is enough information to restore the original EEPROM
         * contents.
         */
        oxygen_write_eeprom(chip, 1, 0x1043);
        oxygen_write_eeprom(chip, 0, OXYGEN_EEPROM_ID);
        oxygen_set_bits8(chip, OXYGEN_MISC,
                         OXYGEN_MISC_WRITE_PCI_SUBID);
        device->configWrite16(PCI_SUBSYSTEM_VENDOR_ID,
                              0x1043);
        device->configWrite16(PCI_SUBSYSTEM_ID,
                              0x8314);
        oxygen_clear_bits8(chip, OXYGEN_MISC,
                           OXYGEN_MISC_WRITE_PCI_SUBID);
        
        IOLog("PCIAudioDevice[%p]::oxygen_restore_eeprom EEPROM ID restored\n", this);
    }
}


bool PCIAudioDevice::initHardware(IOService *provider)
{
    bool result = false;
    
    IOLog("SamplePCIAudioDevice[%p]::initHardware(%p)\n", this, provider);
    
    if (!super::initHardware(provider)) {
        goto Done;
    }
    
    // Get the PCI device provider
    pciDevice = OSDynamicCast(IOPCIDevice, provider);
    if (!pciDevice) {
        goto Done;
    }
    
    // Config a map for the PCI config base registers
    // We need to keep this map around until we're done accessing the registers
    deviceMap = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
    if (!deviceMap) {
        goto Done;
    }
    
    // Get the virtual address for the registers - mapped in the kernel address space
    deviceRegisters = (struct oxygen *)deviceMap->getVirtualAddress();
    if (!deviceRegisters) {
        goto Done;
    }
    
    // Enable the PCI memory access - the kernel will panic if this isn't done before accessing the
    // mapped registers
    pciDevice->setMemoryEnable(true);
    
    // add the hardware init code here
    
    setDeviceName("ASUS Xonar HDAV1.3 Deluxe");
    setDeviceShortName("CMI8788");
    setManufacturerName("CMedia");
    
    oxygen_restore_eeprom(pciDevice,deviceRegisters);
    //following oxygen_pci_probe...
    /**** MOVED TO AUDIOENGINE (XONAR_HDAV) AS IT FITS BETTER ***
    deviceRegisters->spdif_input_bits_work.init();
    deviceRegisters->gpio_work.init();
    queue_init(&deviceRegisters->ac97_waitqueue);
    deviceRegisters->mutex = OS_SPINLOCK_INIT;
    *******/
    //hardcoding relevant portions from get_xonar_model for HDAV1.3 for the time being.
    //if i can get a single model to work, i'll add others....
    deviceRegisters->model.dac_channels_mixer = 8;
    deviceRegisters->model.dac_mclks = OXYGEN_MCLKS(256, 128, 128);
    deviceRegisters->model.device_config = PLAYBACK_0_TO_I2S |
			 PLAYBACK_1_TO_SPDIF |
			 CAPTURE_0_FROM_I2S_2 |
			 CAPTURE_1_FROM_SPDIF;
    deviceRegisters->model.dac_channels_pcm = 8;
    deviceRegisters->model.dac_channels_mixer = 2;
    deviceRegisters->model.dac_volume_min = 255 - 2*60;
    deviceRegisters->model.dac_volume_max = 255;
    deviceRegisters->model.misc_flags = OXYGEN_MISC_MIDI;
    deviceRegisters->model.function_flags = OXYGEN_FUNCTION_2WIRE;
    deviceRegisters->model.dac_mclks = OXYGEN_MCLKS(512, 128, 128);
    deviceRegisters->model.adc_mclks = OXYGEN_MCLKS(256, 128, 128);
    deviceRegisters->model.dac_i2s_format = OXYGEN_I2S_FORMAT_I2S;
    deviceRegisters->model.adc_i2s_format = OXYGEN_I2S_FORMAT_LJUST;
    
    oxygen_init(deviceRegisters);
    
//#error Put your own hardware initialization code here...and in other routines!!
    
    //At this point, we should be at the chip->model.init() part of the oxygen_pci_probe function.
    if (!createAudioEngine()) {
        goto Done;
    }
    
    result = true;
    
Done:
    
    if (!result) {
        if (deviceMap) {
            deviceMap->release();
            deviceMap = NULL;
        }
    }
    
    return result;
}

void PCIAudioDevice::free()
{
    IOLog("SamplePCIAudioDevice[%p]::free()\n", this);
    
    if (deviceMap) {
        deviceMap->release();
        deviceMap = NULL;
    }
    
    super::free();
}

bool PCIAudioDevice::createAudioEngine()
{
    bool result = false;
    SamplePCIAudioEngine *audioEngine = NULL;
    IOAudioControl *control;
    
    IOLog("SamplePCIAudioDevice[%p]::createAudioEngine()\n", this);
    
    audioEngine = new SamplePCIAudioEngine;
    if (!audioEngine) {
        goto Done;
    }
    
    // Init the new audio engine with the device registers so it can access them if necessary
    // The audio engine subclass could be defined to take any number of parameters for its
    // initialization - use it like a constructor
    if (!audioEngine->init(deviceRegisters)) {
        goto Done;
    }
    
    // Create a left & right output volume control with an int range from 0 to 65535
    // and a db range from -22.5 to 0.0
    // Once each control is added to the audio engine, they should be released
    // so that when the audio engine is done with them, they get freed properly
    control = IOAudioLevelControl::createVolumeControl(65535,	// Initial value
                                                       0,		// min value
                                                       65535,	// max value
                                                       (-22 << 16) + (32768),	// -22.5 in IOFixed (16.16)
                                                       0,		// max 0.0 in IOFixed
                                                       kIOAudioControlChannelIDDefaultLeft,
                                                       kIOAudioControlChannelNameLeft,
                                                       0,		// control ID - driver-defined
                                                       kIOAudioControlUsageOutput);
    if (!control) {
        goto Done;
    }
    
    control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)volumeChangeHandler, this);
    audioEngine->addDefaultAudioControl(control);
    control->release();
    
    control = IOAudioLevelControl::createVolumeControl(65535,	// Initial value
                                                       0,		// min value
                                                       65535,	// max value
                                                       (-22 << 16) + (32768),	// min -22.5 in IOFixed (16.16)
                                                       0,		// max 0.0 in IOFixed
                                                       kIOAudioControlChannelIDDefaultRight,	// Affects right channel
                                                       kIOAudioControlChannelNameRight,
                                                       0,		// control ID - driver-defined
                                                       kIOAudioControlUsageOutput);
    if (!control) {
        goto Done;
    }
    
    control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)volumeChangeHandler, this);
    audioEngine->addDefaultAudioControl(control);
    control->release();
    
    // Create an output mute control
    control = IOAudioToggleControl::createMuteControl(false,	// initial state - unmuted
                                                      kIOAudioControlChannelIDAll,	// Affects all channels
                                                      kIOAudioControlChannelNameAll,
                                                      0,		// control ID - driver-defined
                                                      kIOAudioControlUsageOutput);
    
    if (!control) {
        goto Done;
    }
    
    control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)outputMuteChangeHandler, this);
    audioEngine->addDefaultAudioControl(control);
    control->release();
    
    // Create a left & right input gain control with an int range from 0 to 65535
    // and a db range from 0 to 22.5
    control = IOAudioLevelControl::createVolumeControl(65535,	// Initial value
                                                       0,		// min value
                                                       65535,	// max value
                                                       0,		// min 0.0 in IOFixed
                                                       (22 << 16) + (32768),	// 22.5 in IOFixed (16.16)
                                                       kIOAudioControlChannelIDDefaultLeft,
                                                       kIOAudioControlChannelNameLeft,
                                                       0,		// control ID - driver-defined
                                                       kIOAudioControlUsageInput);
    if (!control) {
        goto Done;
    }
    
    control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)gainChangeHandler, this);
    audioEngine->addDefaultAudioControl(control);
    control->release();
    
    control = IOAudioLevelControl::createVolumeControl(65535,	// Initial value
                                                       0,		// min value
                                                       65535,	// max value
                                                       0,		// min 0.0 in IOFixed
                                                       (22 << 16) + (32768),	// max 22.5 in IOFixed (16.16)
                                                       kIOAudioControlChannelIDDefaultRight,	// Affects right channel
                                                       kIOAudioControlChannelNameRight,
                                                       0,		// control ID - driver-defined
                                                       kIOAudioControlUsageInput);
    if (!control) {
        goto Done;
    }
    
    control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)gainChangeHandler, this);
    audioEngine->addDefaultAudioControl(control);
    control->release();
    
    // Create an input mute control
    control = IOAudioToggleControl::createMuteControl(false,	// initial state - unmuted
                                                      kIOAudioControlChannelIDAll,	// Affects all channels
                                                      kIOAudioControlChannelNameAll,
                                                      0,		// control ID - driver-defined
                                                      kIOAudioControlUsageInput);
    
    if (!control) {
        goto Done;
    }
    
    control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)inputMuteChangeHandler, this);
    audioEngine->addDefaultAudioControl(control);
    control->release();
    
    // Active the audio engine - this will cause the audio engine to have start() and initHardware() called on it
    // After this function returns, that audio engine should be ready to begin vending audio services to the system
    activateAudioEngine(audioEngine);
    // Once the audio engine has been activated, release it so that when the driver gets terminated,
    // it gets freed
    audioEngine->release();
    
    result = true;
    
Done:
    
    if (!result && (audioEngine != NULL)) {
        audioEngine->release();
    }
    
    return result;
}

IOReturn PCIAudioDevice::volumeChangeHandler(IOService *target, IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue)
{
    IOReturn result = kIOReturnBadArgument;
    PCIAudioDevice *audioDevice;
    
    audioDevice = (PCIAudioDevice *)target;
    if (audioDevice) {
        result = audioDevice->volumeChanged(volumeControl, oldValue, newValue);
    }
    
    return result;
}

IOReturn PCIAudioDevice::volumeChanged(IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue)
{
    IOLog("SamplePCIAudioDevice[%p]::volumeChanged(%p, %ld, %ld)\n", this, volumeControl, oldValue, newValue);
    
    if (volumeControl) {
        IOLog("\t-> Channel %ld\n", volumeControl->getChannelID());
    }
    
    // Add hardware volume code change
    
    return kIOReturnSuccess;
}

IOReturn PCIAudioDevice::outputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue)
{
    IOReturn result = kIOReturnBadArgument;
    PCIAudioDevice *audioDevice;
    
    audioDevice = (PCIAudioDevice *)target;
    if (audioDevice) {
        result = audioDevice->outputMuteChanged(muteControl, oldValue, newValue);
    }
    
    return result;
}

IOReturn PCIAudioDevice::outputMuteChanged(IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue)
{
    IOLog("SamplePCIAudioDevice[%p]::outputMuteChanged(%p, %ld, %ld)\n", this, muteControl, oldValue, newValue);
    
    // Add output mute code here
    
    return kIOReturnSuccess;
}

IOReturn PCIAudioDevice::gainChangeHandler(IOService *target, IOAudioControl *gainControl, SInt32 oldValue, SInt32 newValue)
{
    IOReturn result = kIOReturnBadArgument;
    PCIAudioDevice *audioDevice;
    
    audioDevice = (PCIAudioDevice *)target;
    if (audioDevice) {
        result = audioDevice->gainChanged(gainControl, oldValue, newValue);
    }
    
    return result;
}

IOReturn PCIAudioDevice::gainChanged(IOAudioControl *gainControl, SInt32 oldValue, SInt32 newValue)
{
    IOLog("SamplePCIAudioDevice[%p]::gainChanged(%p, %ld, %ld)\n", this, gainControl, oldValue, newValue);
    
    if (gainControl) {
        IOLog("\t-> Channel %ld\n", gainControl->getChannelID());
    }
    
    // Add hardware gain change code here
    
    return kIOReturnSuccess;
}

IOReturn PCIAudioDevice::inputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue)
{
    IOReturn result = kIOReturnBadArgument;
    PCIAudioDevice *audioDevice;
    
    audioDevice = (PCIAudioDevice *)target;
    if (audioDevice) {
        result = audioDevice->inputMuteChanged(muteControl, oldValue, newValue);
    }
    
    return result;
}

IOReturn PCIAudioDevice::inputMuteChanged(IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue)
{
    IOLog("SamplePCIAudioDevice[%p]::inputMuteChanged(%p, %ld, %ld)\n", this, muteControl, oldValue, newValue);
    
    // Add input mute change code here
    
    return kIOReturnSuccess;
}

