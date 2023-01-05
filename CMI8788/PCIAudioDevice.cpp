/*
 File:SamplePCIAudioDevice.cpp
 
 Contains:
 
 Version:1.0.0
 
 Copyright:Copyright ) 1997-2000 by APPUL Computer, Inc., All Rights Reserved.
 
 Disclaimer:IMPORTANT:  This APPUL software is supplied to you by APPUL Computer, Inc.
 ("APPUL") in consideration of your agreement to the following terms, and your use,
 installation, modification or redistribution of this APPUL software constitutes acceptance
 of these terms.  If you do not agree with these terms, please do not use, install, modify or
 redistribute this APPUL software.
 
 In consideration of your agreement to abide by the following terms, and subject
 to these terms, APPUL grants you a personal, non-exclusive license, under APPUL's
 copyrights in this original APPUL software (the "APPUL Software"), to use, reproduce,
 modify and redistribute the APPUL Software, with or without modifications, in source and/or
 binary forms; provided that if you redistribute the APPUL Software in its entirety
 and without modifications, you must retain this notice and the following text
 and disclaimers in all such redistributions of the APPUL Software.  Neither the
 name, trademarks, service marks or logos of APPUL Computer, Inc. may be used to
 endorse or promote products derived from the APPUL Software without specific prior
 written permission from APPUL.  Except as expressly stated in this notice, no
 other rights or licenses, express or implied, are granted by APPUL herein,
 including but not limited to any patent rights that may be infringed by your derivative
 works or by other works in which the APPUL Software may be incorporated.
 
 The APPUL Software is provided by APPUL on an "AS IS" basis.  APPUL MAKES NO WARRANTIES,
 EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT,
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE APPUL SOFTWARE
 OR ITS USE AND OPERATION ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL APPUL
 BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE,
 REPRODUCTION, MODIFICATION AND/OR DISTRIBUTION OF THE APPUL SOFTWARE, HOWEVER CAUSED
 AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE), STRICT
 LIABILITY OR OTHERWISE, EVEN IF APPUL HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 */

#include "PCIAudioDevice.hpp"
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/audio/IOAudioDevice.h>
#include <IOKit/audio/IOAudioLevelControl.h>
#include <IOKit/audio/IOAudioToggleControl.h>
#include <IOKit/audio/IOAudioDefines.h>
#include <IOKit/IOBufferMemoryDescriptor.h>

#include "XonarHDAVAudioEngine.hpp"
#include "XonarD2XAudioEngine.hpp"
#include "XonarSTAudioEngine.hpp"
#include "XonarCS43XXAudioEngine.hpp"
#include "XonarWM87x6AudioEngine.hpp"
#include "XonarGenericAudioEngine.hpp"
#include "XonarSPDIFAudioEngine.hpp"
#include "XonarAC97AudioEngine.hpp"
#include "oxygen.h"
#include "ac97.h"
#define DEBUG_CALLS
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
        kprintf("PCIAudioDevice::%-32s EEPROM write timeout\n", __func__);
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
                
                kprintf("PCIAudioDevice::%-32s EEPROM ID restored\n", __func__);
        }
}


bool PCIAudioDevice::initHardware(IOService *provider)
{
        
        bool result = false;
        kprintf("XonarAudioDevice::initHardware()\n");
        
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
        
        deviceMap = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0, kIOInhibitCache);
        if (!deviceMap) {
                goto Done;
        }
        
        /* Get the memory descriptor, which we will attempt to use like getVirtualAddress()
         * cannot use getVirtualAddress like the sample code; assignment and/or access cause panics
         * with memoryDescriptor, assignments do not cause panics but kprintf values will (probably the security
         * issue that phil [@pmj] alluded to) */
        //oxygenBuffer = IOBufferMemoryDescriptor::withOptions(kIODirectionNone, sizeof(struct oxygen));
        //deviceRegisters = (struct oxygen *) oxygenBuffer->getBytesNoCopy(); //deviceMap->getMemoryDescriptor();
        deviceRegisters = (struct oxygen *) IOMalloc(sizeof(struct oxygen)); //deviceMap->getMemoryDescriptor();
        bzero(deviceRegisters, sizeof(struct oxygen));
        bzero(&deviceRegisters->model, sizeof(oxygen_model));
        if (!deviceRegisters) {
                goto Done;
        }
        // Enable the PCI memory access - the kernel will panic if this isn't done before accessing the
        // mapped registers
        pciDevice->setMemoryEnable(true);
        pciDevice->setBusMasterEnable(true);
        pciDevice->setIOEnable(true);
        pciDevice->enablePCIPowerManagement(kPCIPMCPMESupportFromD3Hot);
        pciDevice->enablePCIPowerManagement(kPCIPMCPMESupportFromD0);
        //deviceRegisters->addr = deviceMap->getAddress();
        deviceRegisters->addr = deviceMap;
        deviceRegisters->dev = pciDevice;
        /* many thanks to (github.com/ammulder) whose intel PCI driver code is the reason
         * for the following three lines.
         */
        
        deviceRegisters->ac97_mutex = IOLockAlloc();
        deviceRegisters->mutex = IOLockAlloc();
        deviceRegisters->reg_lock = IOSimpleLockAlloc();
        
        vendor_id = pciDevice->extendedConfigRead16(kIOPCIConfigVendorID);
        dev_id = pciDevice->extendedConfigRead16(kIOPCIConfigDeviceID);
        subdev_id = pciDevice->extendedConfigRead16(kIOPCIConfigSubSystemID);
        kprintf("PCIAudioDevice::%-32s Xonar Vendor ID:0x%04x, Device ID:0x%04x, SubDevice ID:0x%04x, Physical Address:0x%016llx\n",
                __func__, vendor_id, dev_id, subdev_id, deviceMap->getAddress());
        
        //     add the hardware init code here
        const char *modelString;
        switch (subdev_id) {
                case MODEL_HDAV:
                        modelString = "ASUS Xonar HDAV1.3 Deluxe";
                        break;
                case MODEL_ST:
                case MODEL_STX:
                case MODEL_STX2:
                case MODEL_XENSE:
                        modelString = "ASUS Xonar ST(X [II])+Xense models";
                        break;
                case MODEL_CS43XX:
                case MODEL_D1:
                case MODEL_DX:
                        modelString = "Asus Xonar D{1,X}/CS43XX models";
                        break;
                case MODEL_D2:
                case MODEL_D2X:
                        modelString = "ASUS Xonar D2(X) models";
                        break;
                case MODEL_DS:
                case MODEL_DSX:
                case HDAV_SLIM:
                        modelString = "ASUS Xonar DS(X)+HDAV SLIM models";
                        break;
                default:
                        modelString = "Generic CMI8787/8788 Card (Claro, Meridian, Fantasia)";
                        
        }
        setDeviceName(modelString);
        setDeviceModelName(modelString);
        setDeviceTransportType(kIOAudioDeviceTransportTypePCI);
        setDeviceShortName("CMI8787/CMI8788 Card");
        setManufacturerName("CMedia");
        oxygen_restore_eeprom(pciDevice,deviceRegisters);
        
        if (!createAudioEngine()) {
                goto Done;
        }
        
        result = true;
        
Done:
        if (!result) {
                if (deviceMap)
                        OSSafeReleaseNULL(deviceMap);
        }
        
        
        return result;
}


static const UInt32 registers_to_restore[OXYGEN_IO_SIZE / 32] = {
        0xffffffff, 0x00ff077f, 0x00011d08, 0x007f00ff,
        0x00300000, 0x00000fe4, 0x0ff7001f, 0x00000000
};

//APPUL says they'll do everything else if we handle the hardware
//see here: https://developer.apple.com/library/archive/documentation/DeviceDrivers/Conceptual/IOKitFundamentals/Families_Ref/Families_Ref.html#//apple_ref/doc/uid/TP0000021-BABJHDJH

// looks like APPUL is piling up some fun wins against ALSA.
// but since they're fucking cheap and went 'full <you-know-what>',
// they refuse to pay money to give the API its much needed update.
// either way, it looks like we're doing way less compared to ALSA.
IOReturn PCIAudioDevice::performPowerStateChange(IOAudioDevicePowerState oldPowerState,
                                                 IOAudioDevicePowerState newPowerState,
                                                 UInt32 *microsecondsUntilComplete )
{
        if(newPowerState == kIOAudioDeviceSleep) {
                
                //struct snd_card *card = dev_get_drvdata(dev);
                //struct oxygen *chip = card->private_data;
                accessibleEngineInstance->oxygen_trigger();
                unsigned int i, saved_interrupt_mask;
                
                //snd_power_change_state(card, SNDRV_CTL_POWER_D3hot);
                
                //for (i = 0; i < PCM_COUNT; ++i)
                //       snd_pcm_suspend(chip->streams[i]);
                
                if (deviceRegisters->model.suspend)
                        deviceRegisters->model.suspend(deviceRegisters, accessibleEngineInstance);
                
                IOSimpleLockLock(deviceRegisters->reg_lock);
                saved_interrupt_mask = deviceRegisters->interrupt_mask;
                deviceRegisters->interrupt_mask = 0;
                oxygen_write16(deviceRegisters, OXYGEN_DMA_STATUS, 0);
                oxygen_write16(deviceRegisters, OXYGEN_INTERRUPT_MASK, 0);
                IOSimpleLockUnlock(deviceRegisters->reg_lock);
                
                //                        synchronize_irq(chip->irq);
                //                        flush_work(&chip->spdif_input_bits_work);
                //                        flush_work(&chip->gpio_work);
                deviceRegisters->interrupt_mask = saved_interrupt_mask;
                return kIOReturnSuccess;
                
        }
        else if(oldPowerState == kIOAudioDeviceSleep) {
                
                //                        struct snd_card *card = dev_get_drvdata(dev);
                //                        struct oxygen *chip = card->private_data;
                accessibleEngineInstance->oxygen_trigger();
                unsigned int i;
                oxygen_write16(deviceRegisters, OXYGEN_DMA_STATUS, 0);
                oxygen_write16(deviceRegisters, OXYGEN_INTERRUPT_MASK, 0);
                for (i = 0; i < OXYGEN_IO_SIZE; ++i)
                        if (is_bit_set(registers_to_restore, i))
                                oxygen_write8(deviceRegisters, i, deviceRegisters->saved_registers._8[i]);
                if (deviceRegisters->has_ac97_0)
                        accessibleEngineInstance->oxygen_restore_ac97(deviceRegisters, 0);
                if (deviceRegisters->has_ac97_1)
                        accessibleEngineInstance->oxygen_restore_ac97(deviceRegisters, 1);
                
                if (deviceRegisters->model.resume)
                        deviceRegisters->model.resume(deviceRegisters, accessibleEngineInstance);
                
                oxygen_write16(deviceRegisters, OXYGEN_INTERRUPT_MASK, deviceRegisters->interrupt_mask);
                //                        snd_power_change_state(card, SNDRV_CTL_POWER_D0);
                return kIOReturnSuccess;
        }
        else if (oldPowerState == newPowerState)
                return kIOReturnSuccess;
        return kIOReturnError;
        
        //return kIOReturnError;
}

void PCIAudioDevice::free()
{
        kprintf("XonarAudioDevice::free()\n");
        
        if (deviceRegisters) {
            if (deviceRegisters->reg_lock)
                IOSimpleLockFree(deviceRegisters->reg_lock);
            
            if (deviceRegisters->mutex)
                IOLockFree(deviceRegisters->mutex);
            
            if (deviceRegisters->ac97_mutex)
                IOLockFree(deviceRegisters->ac97_mutex);
            
            IOFree(deviceRegisters, sizeof(struct oxygen));
            deviceRegisters = NULL;
        }
    
        if (deviceMap)
                OSSafeReleaseNULL(deviceMap);
    
        super::free();
}

bool PCIAudioDevice::createAudioEngine()
{
#if DEBUG
    kprintf("PCIAudioDevice::%-32s BEGIN\n", __func__);
#endif
    
    bool result = false;
    
    accessibleEngineInstance = new XonarAudioEngine;
    
    //At this point, we should be at the chip->model.init() part of the oxygen_pci_probe function.
    //chip->model.init() is handled by the init() method of the submodel's class that we wish to instantiate.
    //that is: XonarHDAVAudioEngine::init() contains the code for xonar_hdav_init, etc.
    
    if (!accessibleEngineInstance){
        kprintf("PCIAudioDevice::%-32s Audio engine allocation failure\n", __func__);
        goto Done;
    }
    
    if (!accessibleEngineInstance->init(deviceRegisters, subdev_id)){
        kprintf("PCIAudioDevice::%-32s Audio engine initialization failure\n", __func__);
        goto Done;
    }
    
    if (!deviceRegisters->model_data){
        kprintf("PCIAudioDevice::%-32s deviceRegisters->model_data not allocated\n", __func__);
        goto Done;
    }
    
        switch(subdev_id){
                case MODEL_HDAV:
                        submodelInstance = new XonarHDAVAudioEngine;
                        break;
                case MODEL_ST:
                case MODEL_STX:
                case MODEL_STX2:
                case MODEL_XENSE:
                        submodelInstance = new XonarSTAudioEngine;
                        break;
                case MODEL_D2:
                case MODEL_D2X:
                        submodelInstance = new XonarD2XAudioEngine;
                        break;
                case MODEL_DX:
                case MODEL_CS43XX:
                case MODEL_D1:
                        submodelInstance = new XonarCS43XXAudioEngine;
                        break;
                case MODEL_DS:
                case MODEL_DSX:
                case HDAV_SLIM:
                        submodelInstance = new XonarWM87x6AudioEngine;
                        break;
                default:
                        submodelInstance = new XonarGenericAudioEngine;
                        break;
        }
    
        if (!submodelInstance)
                goto Done;
        
        //calling chip->model.init()-equivalent directly below
        // Init the new audio engine with the device registers so it can access them if necessary
        // The audio engine subclass could be defined to take any number of parameters for its
        // initialization - use it like a constructor
        switch (subdev_id) {
                case MODEL_HDAV:
                        if (!( (XonarHDAVAudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters))
                                goto Done;
                        break;
                case MODEL_ST:
                case MODEL_STX:
                case MODEL_STX2:
                case MODEL_XENSE:
                        if (!( (XonarSTAudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters,subdev_id))
                                goto Done;
                        break;
                case MODEL_D2:
                case MODEL_D2X:
                        if (!( (XonarD2XAudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters,subdev_id))
                                goto Done;
                        break;
                case MODEL_DX:
                case MODEL_CS43XX:
                case MODEL_D1:
                        if (!( (XonarCS43XXAudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters,subdev_id))
                                goto Done;
                        break;
                case MODEL_DS:
                case MODEL_DSX:
                case HDAV_SLIM:
                        if (!( (XonarWM87x6AudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters,subdev_id))
                                goto Done;
                        break;
                default:
                        if (!( (XonarGenericAudioEngine*) submodelInstance)->init(accessibleEngineInstance,deviceRegisters,subdev_id))
                                goto Done;
                        break;
                        
        }
        
 
        
        // now we have to create the midi device
        /*
         if (deviceRegisters->model.device_config & (MIDI_OUTPUT | MIDI_INPUT)) {
         //pay some respect to francois ferland's casiousbmididriver
         //(https://github.com/francoisferland/casiousbmididriver)
         CFStringRef boxName = CFSTR("Oxygen MIDI Interface");
         MIDIDeviceCreate(deviceRegisters->Self(), boxName, CFSTR("CMedia"), boxName, &deviceRegisters->midi);
         MIDIEntityRef ent;
         CFStringRef str;
         OSStatus err;
         char portname[64];
         //unsigned int info_flags =  MPU401_INFO_INTEGRATED | MPU401_INFO_IRQ_HOOK;
         sprintf(portname, "MPU401");
         CFStringGetCString(boxName, portname, sizeof(portname), kCFStringEncodingMacRoman);
         str = CFStringCreateWithCString(NULL, portname, 0);
         // make entity for each port, with 1 source, 1 destination
         // (if they have them)
         MIDIDeviceAddEntity(deviceRegisters->midi, str, true, (bool) (deviceRegisters->model.device_config & MIDI_OUTPUT),
         (bool) (deviceRegisters->model.device_config & MIDI_INPUT),
         &ent);
         CFRelease(str);
         __Require_noErr(err = MIDISetupAddDevice(deviceRegisters->midi), Done);
         
         //lets get the source and destination endpoints and set them to the same place
         //boy i hope this is right.
         //logic behind the call below:
         // - we know the address is addr+OXYGEN_MPU401.
         // - we just created the midi device and ent is the associated entity for one input/output port
         kMIDIDriverPropertyUsesSerial; //<-does nothing, but HOW DO WE USE IT?! WHY IS GOOGLE TURNING UP EMPTY?! :((
         if(deviceRegisters->model.device_config & MIDI_OUTPUT) {
         deviceRegisters->midiOut = MIDIEntityGetDestination(MIDIDeviceGetEntity(deviceRegisters->midi,0), 0);
         MIDIEndpointSetRefCons(deviceRegisters->midiOut, NULL, NULL);
         }
         if(deviceRegisters->model.device_config & MIDI_INPUT) {
         deviceRegisters->midiIn = MIDIEntityGetSource(MIDIDeviceGetEntity(deviceRegisters->midi,0), 0);
         MIDIEndpointSetRefCons(deviceRegisters->midiIn, NULL, NULL);
         }
         
         
         //            err = snd_mpu401_uart_new(card, 0, MPU401_HW_CMIPCI,
         //                                      chipData->addr + OXYGEN_MPU401,
         //                                      info_flags, -1, &chip->midi);
         //          if (err < 0)
         //                goto err_card;
         }
         */
        
        
        // Active the audio engine - this will cause the audio engine to have start() and initHardware() called on it
        // After this function returns, that audio engine should be ready to begin vending audio services to the system
    
//        accessibleEngineInstance->spdifEngine->init(accessibleEngineInstance);
//        kprintf("After the init\n");
//        activateAudioEngine(accessibleEngineInstance->spdifEngine);
        //) {
//                kprintf("XonarAudioEngine::%-30s SPDIF engine creation failed.\n", __func__);
//        }
//        else {
//                accessibleEngineInstance->spdifEngine->release();
//                kprintf("XonarAudioEngine::%-30s SPDIF engine successfully created.\n", __func__);
//        }
//        if(!activateAudioEngine(accessibleEngineInstance->ac97Engine)) {
//                kprintf("XonarAudioEngine::%-30s AC97 engine creation failed.\n", __func__);
//        }
//        else{
//                accessibleEngineInstance->ac97Engine->release();
//                kprintf("XonarAudioEngine::%-30s AC97 engine successfully created.\n", __func__);
//        }

        activateAudioEngine(accessibleEngineInstance);
    
        // the best way to integrate the trigger function is to put it **after**
        // the audioEngineStart call--more specifically when we prepare the
        // stream for the respective channel. this way we can set the mask to
        // 'running' and then clear it when we stop it for whatever reason (sleep etc).
        // the engine is not 'running' until it is activated, so we have to do it after
        // this call (or so i think)
        accessibleEngineInstance->oxygen_trigger();
    
        accessibleEngineInstance->release();
        
        //activateAudioEngine(submodelInstance, false);
        
        // Once the audio engine has been activated, release it so that when the driver gets terminated,
        // it gets freed
        submodelInstance->release();
        
        result = true;
        
Done:
        /* release both the submodel and accessible(main) engine instance
         * so that they will automatically free after calling PCIAudioDriver::free()
         */
        if (!result && submodelInstance)
                submodelInstance->release();
        
        if (!result && accessibleEngineInstance)
                accessibleEngineInstance->release();
        
#if DEBUG
        kprintf("PCIAudioDevice::%-32s END\n", __func__);
#endif
        return result;
}


int PCIAudioDevice::oxygen_mixer_init(void)
{
#if DEBUG
        kprintf("PCIAudioDevice::%-32s BEGIN\n", __func__);
#endif
        unsigned int i;
        int err;
        IOAudioPort *masterPort;
        //    err = add_controls(chipData, controls, ARRAY_SIZE(controls));
        
        err = createMasterControl(masterPort);
#if !defined(MULTICH_OUTPUT_ONLY)
        if (err < 0)
                return err;
        if (deviceRegisters->model.device_config & PLAYBACK_1_TO_SPDIF) {
                IOAudioPort *spdifOutput;
                err = createSPDIFoutputControl(spdifOutput);
                if (err < 0)
                        return err;
        }
        if (deviceRegisters->model.device_config & CAPTURE_1_FROM_SPDIF) {
                IOAudioPort *spdifInput;
                err = createSPDIFinputControl(spdifInput);
                if (err < 0)
                        return err;
        }
#endif
        //    for (i = 0; i < ARRAY_SIZE(monitor_controls); ++i) {
        //        if (!(deviceRegisters->model.device_config & monitor_controls[i].pcm_dev))
        //            continue;
        //        err = add_controls(chipData, monitor_controls[i].controls,
        //                   ARRAY_SIZE(monitor_controls[i].controls));
        //        if (err < 0)
        //            return err;
        //    }
        //    if (chipData->has_ac97_0) {
        //        err = add_controls(deviceRegisters, ac97_controls,
        //                   ARRAY_SIZE(ac97_controls));
        //        if (err < 0)
        //            return err;
        //    }
        //    if (chipData->has_ac97_1) {
        //        err = add_controls(deviceRegisters, ac97_fp_controls,
        //                   ARRAY_SIZE(ac97_fp_controls));
        //        if (err < 0)
        //            return err;
        //    }
#if DEBUG
        kprintf("PCIAudioDevice::%-32s END\n", __func__);
#endif
#if defined(MULTICH_OUTPUT_ONLY)
        return 0;
#else
        return deviceRegisters->model.mixer_init ? deviceRegisters->model.mixer_init(deviceRegisters, this, accessibleEngineInstance) : 0;
#endif
}

