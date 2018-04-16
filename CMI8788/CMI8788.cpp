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
#include "CMI8788.hpp"
#include "XonarHDAVAudioEngine.hpp"
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
    XonarAudioEngine *audioEngineInstance = NULL;
    audioEngineInstance = new XonarAudioEngine;
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
    deviceRegisters->model.model_data_size = sizeof(struct xonar_hdav);
    deviceRegisters->mutex = OS_SPINLOCK_INIT;
    pthread_mutex_init(&deviceRegisters->ac97_mutex,NULL);
    pthread_cond_init(&deviceRegisters->ac97_condition,NULL);
    //move oxygen_init to (barely-used) XonarAudioEngine to fill out the latter.
    //oxygen_init(deviceRegisters);
    //Before:AUdioEngine's init didn't do much. now it instantiates everything like oxygen_init.
    //so, by creating the engine, we instantiate the registers as well.
    if (!audioEngineInstance->init(deviceRegisters,0))
        goto Done;
    //#error Put your own hardware initialization code here...and in other routines!!

    
    //At this point, we should be at the chip->model.init() part of the oxygen_pci_probe function.
    if (!createAudioEngine(audioEngineInstance)) {
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

bool PCIAudioDevice::createAudioEngine(XonarAudioEngine *audioEngineInstance)
{
    bool result = false;
    //XonarAudioEngine *AudioEngineInstance = NULL;
    XonarHDAVAudioEngine *audioEngine = NULL;
    IOAudioControl *control;
    
    IOLog("SamplePCIAudioDevice[%p]::createAudioEngine()\n", this);
   // AudioEngineInstance = new XonarAudioEngine;
    audioEngine = new XonarHDAVAudioEngine;
    if (!audioEngine) {
        goto Done;
    }
    
    // Init the new audio engine with the device registers so it can access them if necessary
    // The audio engine subclass could be defined to take any number of parameters for its
    // initialization - use it like a constructor
    
    if (!audioEngine->init(audioEngineInstance,deviceRegisters)) {
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

