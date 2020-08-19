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
#include "PCIAudioDevice.hpp"
#include "XonarHDAVAudioEngine.hpp"
#include "XonarD2XAudioEngine.hpp"
#include "XonarSTAudioEngine.hpp"
#include "XonarGenericAudioEngine.hpp"
#include "XonarCS43XXAudioEngine.hpp"
#include "XonarWM87x6AudioEngine.hpp"

#include "cm9780.h"
#include "ac97.h"


#define super IOAudioDevice

OSDefineMetaClassAndStructors(PCIAudioDevice, IOAudioDevice)


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
    //dev_err(chip->card->dev, "EEPROM write timeout\n");
}


void PCIAudioDevice::oxygen_restore_eeprom(IOPCIDevice *device, struct oxygen *chip)
{
    UInt16 eeprom_id;
    eeprom_id = oxygen_read_eeprom(chip, 0);
    if (eeprom_id != OXYGEN_EEPROM_ID &&
        (eeprom_id != 0xffff || subdev_id != 0x8788)) {
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
        
       kprintf("oxygen_restore_eeprom EEPROM ID restored\n");
    }
}


bool PCIAudioDevice::initHardware(IOService *provider)
{
    char bootArg[16];
    kprintf("XonarAudioDevice::initHardware()\n");
    
    if (PE_parse_boot_argn("-xonaroff", bootArg, sizeof(bootArg)/sizeof(bootArg[0])) || PE_parse_boot_argn("-cmioff", bootArg, sizeof(bootArg)/sizeof(bootArg[0]))){
        kprintf("XonarAudioDevice::initHardware Driver disabled with disable boot arg\n");
        goto Done;
    }
    
    
    
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
    
    /* Get the memory descriptor, which we will attempt to use like getVirtualAddress()
     * cannot use getVirtualAddress like the sample code; assignment and/or access cause panics
     * with memoryDescriptor, assignments do not cause panics but kprintf values will (probably the security
     * issue that phil [@pmj] alluded to) */
    deviceRegisters = (struct oxygen *) IOMalloc(sizeof(struct oxygen)); //deviceMap->getMemoryDescriptor();

    if (!deviceRegisters) {
        goto Done;
    }
    // Enable the PCI memory access - the kernel will panic if this isn't done before accessing the
    // mapped registers
    pciDevice->setMemoryEnable(true);
    pciDevice->setBusMasterEnable(true);
    pciDevice->setIOEnable(true);
    deviceRegisters->addr = deviceMap->getAddress();
    /* many thanks to (github.com/ammulder) whose intel PCI driver code is the reason
     * for the following three lines.
     */

    deviceRegisters->ac97_mutex = IOLockAlloc();
    deviceRegisters->mutex = IOLockAlloc();
    deviceRegisters->reg_lock = IOSimpleLockAlloc();

    vendor_id = pciDevice->extendedConfigRead16(kIOPCIConfigVendorID);
    dev_id = pciDevice->extendedConfigRead16(kIOPCIConfigDeviceID);
    subdev_id = pciDevice->extendedConfigRead16(kIOPCIConfigSubSystemID);
    kprintf("Xonar Vendor ID:0x%04x, Device ID:0x%04x, SubDevice ID:0x%04x, Physical Address:0x%016llx\n",
           vendor_id, dev_id, subdev_id, deviceMap->getAddress());
  
    //     add the hardware init code here

    /*
    if(subdev_id == HDAV_MODEL)
        setDeviceName("ASUS Xonar HDAV1.3 Deluxe");
    else if (subdev_id == ST_MODEL || subdev_id == STX_MODEL || subdev_id == XENSE_MODEL)
        setDeviceName("ASUS Xonar ST(X [II])+Xense models");
    else if (subdev_id == D2_MODEL || subdev_id == D2X_MODEL)
        setDeviceName("ASUS Xonar D2(X) models");
    else if (subdev_id == DS_MODEL || subdev_id == DSX_MODEL || subdev_id == HDAV_SLIM)
        setDeviceName("ASUS Xonar DS(X)+HDAV SLIM models");
    else
        setDeviceName("ASUS Xonar Generic");
    */
    
    
    switch (subdev_id){
        case HDAV_MODEL:
            setDeviceName("ASUS Xonar HDAV1.3 Deluxe");
            break;
        case ST_MODEL:
            setDeviceName("ASUS Xonar ST");
            break;
        case STX_MODEL:
            //TODO: distingush beetween the STX and the STX II
            setDeviceName("ASUS Xonar STX [II]");
            break;
        case XENSE_MODEL:
            setDeviceName("ASUS Xonar Xense");
            break;
        case D2_MODEL:
            setDeviceName("ASUS Xonar D2");
            break;
        case D2X_MODEL:
            setDeviceName("ASUS Xonar D2X");
            break;
        case DX_MODEL:
            setDeviceName("ASUS Xonar D2X");
            break;
        case CS4XX_MODEL:
            setDeviceName("ASUS Xonar CS4XX");
            break;
        case D1_MODEL:
            setDeviceName("ASUS Xonar D1");
            break;
        case DS_MODEL:
            setDeviceName("ASUS Xonar DS");
            break;
        case DSX_MODEL:
            setDeviceName("ASUS Xonar DSX");
            break;
        case HDAV_SLIM:
            setDeviceName("ASUS HDAV");
            break;
        default:
            kprintf("XonarAudioDevice::initHardware unsupported sound card 1\n");
            goto Done;
            break;
    }
    
    setDeviceShortName("CMI8788");
    setManufacturerName("CMedia");
    
    //TODO: check if this line here crashes the driver or not
    setDeviceTransportType(kIOAudioDeviceTransportTypePCI);
    
    oxygen_restore_eeprom(pciDevice,deviceRegisters);

   
    if (!createAudioEngine()) {
        goto Done;
    }
    
    return true;
    
Done:
    if (deviceMap) {
        deviceMap->release();
        deviceMap = NULL;
    }

    return false;
}

void PCIAudioDevice::free()
{
    kprintf("XonarAudioDevice::free()\n");
    
    IOSimpleLockFree(deviceRegisters->reg_lock);
    IOLockFree(deviceRegisters->mutex);
    IOLockFree(deviceRegisters->ac97_mutex);
    if (deviceRegisters) {
        IOFree(deviceRegisters, sizeof(struct oxygen));
        deviceRegisters = NULL;
    }
    if (deviceMap) {
            deviceMap->release();
            deviceMap = NULL;
       }
    super::free();
}

bool PCIAudioDevice::createAudioEngine()
{
    accessibleEngineInstance = new XonarAudioEngine;
    IOAudioControl *control;
    //At this point, we should be at the chip->model.init() part of the oxygen_pci_probe function.
    //chip->model.init() is handled by the init() method of the submodel's class that we wish to instantiate.
    //that is: XonarHDAVAudioEngine::init() contains the code for xonar_hdav_init, etc.
    kprintf("XonarAudioDevice::createAudioEngine()\n");

    bool result = false;
    
    switch (subdev_id){
        case HDAV_MODEL:
            submodelInstance = new XonarHDAVAudioEngine;
            break;
        case ST_MODEL:
        case STX_MODEL:
        case XENSE_MODEL:
            submodelInstance = new XonarSTAudioEngine;
            break;
        case D2_MODEL:
        case D2X_MODEL:
            submodelInstance = new XonarD2XAudioEngine;
            break;
        case DX_MODEL:
        case CS4XX_MODEL:
        case D1_MODEL:
            submodelInstance = new XonarCS43XXAudioEngine;
            break;
        case DS_MODEL:
        case DSX_MODEL:
        case HDAV_SLIM:
            submodelInstance = new XonarWM87x6AudioEngine;
            break;
        default:
            kprintf("XonarAudioDevice::createAudioEngine unsupported sound card 1\n");
            goto Done;
            break;
    }
    
    /*
    if(subdev_id == HDAV_MODEL)
        submodelInstance = new XonarHDAVAudioEngine;
    else if (subdev_id == ST_MODEL || subdev_id == STX_MODEL || subdev_id == XENSE_MODEL)
        submodelInstance = new XonarSTAudioEngine;
    else if (subdev_id == D2_MODEL || subdev_id == D2X_MODEL)
        submodelInstance = new XonarD2XAudioEngine;
    else if (subdev_id == DX_MODEL || subdev_id == CS4XX_MODEL || subdev_id== D1_MODEL)
        submodelInstance = new XonarCS43XXAudioEngine;
    else if (subdev_id == DS_MODEL || subdev_id == DSX_MODEL || subdev_id == HDAV_SLIM)
        submodelInstance = new XonarWM87x6AudioEngine;
    */
    
    if (!submodelInstance)
        goto Done;

    //calling chip->model.init()-equivalent directly below
    // Init the new audio engine with the device registers so it can access them if necessary
    // The audio engine subclass could be defined to take any number of parameters for its
    // initialization - use it like a constructor
    
    switch (subdev_id){
        case HDAV_MODEL:
            if (!( (XonarHDAVAudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters))
                goto Done;
            break;
        case ST_MODEL:
        case STX_MODEL:
        case XENSE_MODEL:
            if (!( (XonarSTAudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters,subdev_id))
                goto Done;
            break;
        case D2_MODEL:
        case D2X_MODEL:
            if (!( (XonarD2XAudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters,subdev_id))
                goto Done;
            break;
        case DX_MODEL:
        case CS4XX_MODEL:
        case D1_MODEL:
            if (!( (XonarCS43XXAudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters,subdev_id))
                goto Done;
            break;
        case DS_MODEL:
        case DSX_MODEL:
        case HDAV_SLIM:
            if (!( (XonarWM87x6AudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters,subdev_id))
                goto Done;
            break;
        default:
            kprintf("XonarAudioDevice::createAudioEngine unsupported sound card 2\n");
            goto Done;
            break;
    }
    
    /*
    if(subdev_id == HDAV_MODEL) {
        if (!( (XonarHDAVAudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters))
            goto Done;
        
    }
    else if (subdev_id == ST_MODEL || subdev_id == STX_MODEL || subdev_id == XENSE_MODEL) {
        if (!( (XonarSTAudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters,subdev_id))
            goto Done;
    }
    else if (subdev_id == D2_MODEL || subdev_id == D2X_MODEL) {
        if (!( (XonarD2XAudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters,subdev_id))
            goto Done;
        
    }
    else if( subdev_id == DX_MODEL || subdev_id == CS4XX_MODEL || subdev_id== D1_MODEL) {
        if (!( (XonarCS43XXAudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters,subdev_id))
            goto Done;
        
    }
    else if (subdev_id == DS_MODEL || subdev_id == DSX_MODEL || subdev_id == HDAV_SLIM) {
        if (!( (XonarWM87x6AudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters,subdev_id))
            goto Done;

    }
     */
    
    /* The remaining portions of oxygen_pci_probe focus on initialising PCM and the mixer.
     * from what i can gather, these portions of the init from the Linux Driver are handled
     * radically differently from OSX, and so this is where OSX-specific/new code will need to
     * handle these differences, where some of the code in oxygen_mixer.c/oxygen_pcm.c may be
     * used for the aforementioned purposes
     */
    
    // Create a left & right output volume control with an int range from 0 to 65535
    // and a db range from -22.5 to 0.0
    // Once each control is added to the audio engine, they should be released
    // so that when the audio engine is done with them, they get freed properly
    kprintf("creating volumecontrol 1\n");
    control = IOAudioLevelControl::createVolumeControl((deviceRegisters->model.dac_volume_max+deviceRegisters->model.dac_volume_min)/2,	// Initial value
                                                       deviceRegisters->model.dac_volume_min,		// min value
                                                       deviceRegisters->model.dac_volume_max,	// max value
                                                       -6000,	// taken from xonar_pcm179x.c:986 DECLARE_TLV_DB_SCALE(pcm1796_db_scale,-6000,50))
                                                       0,		// max 0.0 in IOFixed
                                                       kIOAudioControlChannelIDDefaultLeft,
                                                       kIOAudioControlChannelNameLeft,
                                                       0,		// control ID - driver-defined
                                                       kIOAudioControlUsageOutput);
    if (!control) {
        goto Done;
    }
    
    control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)volumeChangeHandler, this);
    accessibleEngineInstance->addDefaultAudioControl(control);
    control->release();
    
    kprintf("creating volumecontrol2\n");

    control = IOAudioLevelControl::createVolumeControl((deviceRegisters->model.dac_volume_max+deviceRegisters->model.dac_volume_min)/2,	// Initial value
                                                       deviceRegisters->model.dac_volume_min,		// min value
                                                       deviceRegisters->model.dac_volume_max,	// max value
                                                       -6000,	// min -22.5 in IOFixed (16.16)
                                                       0,		// max 0.0 in IOFixed
                                                       kIOAudioControlChannelIDDefaultRight,	// Affects right channel
                                                       kIOAudioControlChannelNameRight,
                                                       0,		// control ID - driver-defined
                                                       kIOAudioControlUsageOutput);
    if (!control) {
        goto Done;
    }
   

    control->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)volumeChangeHandler, this);
    accessibleEngineInstance->addDefaultAudioControl(control);
    control->release();
     kprintf("creating volumecontrol 3\n");
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
    accessibleEngineInstance->addDefaultAudioControl(control);
    control->release();
       kprintf("creating volumecontrol 4\n");
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
    accessibleEngineInstance->addDefaultAudioControl(control);
    control->release();
        kprintf("creating volumecontrol 5\n");
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
    accessibleEngineInstance->addDefaultAudioControl(control);
    control->release();
        kprintf("creating volumecontrol 6\n");
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
    accessibleEngineInstance->addDefaultAudioControl(control);
    control->release();
    
    // Active the audio engine - this will cause the audio engine to have start() and initHardware() called on it
    // After this function returns, that audio engine should be ready to begin vending audio services to the system
    activateAudioEngine(accessibleEngineInstance);
    // Once the audio engine has been activated, release it so that when the driver gets terminated,
    // it gets freed
    submodelInstance->release();
    accessibleEngineInstance->release();
    
    result = true;
    
Done:
    /* release both the submodel and accessible(main) engine instance
     * so that they will automatically free after calling PCIAudioDriver::free()
     */
    if (!result && submodelInstance)
        submodelInstance->release();
    
    if (!result && accessibleEngineInstance)
        accessibleEngineInstance->release();
    
    return result;
}

IOReturn PCIAudioDevice::volumeChangeHandler(IOService *target, IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue)
{
    IOReturn result = kIOReturnBadArgument;
    PCIAudioDevice *audioDevice;
    
    audioDevice = (PCIAudioDevice *)target;
    if (audioDevice) {
        result = audioDevice->volumeChanged(volumeControl, audioDevice->accessibleEngineInstance, oldValue, newValue);
    }
    
    return result;
}

IOReturn PCIAudioDevice::volumeChanged(IOAudioControl *volumeControl, XonarAudioEngine *engine, SInt32 oldValue, SInt32 newValue)
{
    //printf("SamplePCIAudioDevice[%p]::volumeChanged(%p, %ld, %ld)\n", this, volumeControl, oldValue, newValue);
    
    if (volumeControl) {
        //printf("\t-> Channel %ld\n", volumeControl->getChannelID());
    }
    
    
    // Add hardware volume code change
    struct oxygen *chip = engine->chipData;
    unsigned int i;
    int changed;
    
    changed = 0;
    IOLockLock(chip->mutex);
    /* not sure if i keep dac_volume[i] or use "oldValue", but
     * i figure it's a good idea to use the hardware reading
     * at least once for this sort of comparison, as it determines
     * whether the hardware value is updated
     */
    for (i = 0; i < chip->model.dac_channels_mixer; ++i){
        if (newValue != chip->dac_volume[i]) {
            chip->dac_volume[i] = newValue;
            changed = 1;
        }
    }
    if (changed)
        chip->model.update_dac_volume(chip,engine);
    IOLockUnlock(chip->mutex);
    return changed;
    
    
    return kIOReturnSuccess;
}

IOReturn PCIAudioDevice::outputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue)
{
    IOReturn result = kIOReturnBadArgument;
    PCIAudioDevice *audioDevice;
    
    audioDevice = (PCIAudioDevice *)target;
    if (audioDevice) {
        result = audioDevice->outputMuteChanged(muteControl, audioDevice->accessibleEngineInstance, oldValue, newValue);
    }
    
    return result;
}

IOReturn PCIAudioDevice::outputMuteChanged(IOAudioControl *muteControl, XonarAudioEngine *engine, SInt32 oldValue, SInt32 newValue)
{
    //printf("SamplePCIAudioDevice[%p]::outputMuteChanged(%p, %ld, %ld)\n", this, muteControl, oldValue, newValue);
    
    // Add output mute code here
    struct oxygen *chip = engine->chipData;
    int changed;
    
    IOLockLock(chip->mutex);
    changed = (!newValue) != chip->dac_mute;
    if (changed) {
        chip->dac_mute = !newValue;
        chip->model.update_dac_mute(chip, engine);
    }
    IOLockUnlock(chip->mutex);
    return changed;
    
    
    
    
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
    //printf("SamplePCIAudioDevice[%p]::gainChanged(%p, %ld, %ld)\n", this, gainControl, oldValue, newValue);
    
    if (gainControl) {
        //printf("\t-> Channel %ld\n", gainControl->getChannelID());
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
    //printf("SamplePCIAudioDevice[%p]::inputMuteChanged(%p, %ld, %ld)\n", this, muteControl, oldValue, newValue);
    
    // Add input mute change code here
    
    return kIOReturnSuccess;
}

