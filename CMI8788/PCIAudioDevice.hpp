/*
 File:SamplePCIAudioDevice.h
 
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


#ifndef _SAMPLEPCIAUDIODEVICE_H
#define _SAMPLEPCIAUDIODEVICE_H

#include <IOKit/audio/IOAudioDevice.h>



#define XonarAudioEngine com_CMedia_CMI8788_XonarAudioEngine
#define XonarSPDIFAudioEngine com_CMedia_CMI8788_XonarSPDIFAudioEngine
#define XonarAC97AudioEngine com_CMedia_CMI8788_XonarAC97AudioEngine

class XonarAudioEngine;
class XonarAC97AudioEngine;
class XonarSPDIFAudioEngine;

class IOPCIDevice;
class IOMemoryMap;
class IOAudioEngine;
class IOAudioStream;
class IOAudioToggleControl;
class IOAudioLevelControl;
#define PCIAudioDevice com_CMedia_CMI8788_PCIAudioDevice

class PCIAudioDevice : public IOAudioDevice
{
                OSDeclareDefaultStructors(PCIAudioDevice)
        friend class XonarAudioEngine;      //PCIAudioDevice's top bitch
        friend class XonarSPDIFAudioEngine; //
        friend class XonarAC97AudioEngine;  //PCIAudioDevice's bottom bitch (#AC97LIFE)

        
        XonarAudioEngine            *accessibleEngineInstance;
        IOAudioEngine               *submodelInstance;
        IOPCIDevice					*pciDevice;
        IOMemoryMap					*deviceMap;
        UInt16                      vendor_id;
        UInt16                      dev_id;
        UInt16                      subdev_id;
        //IOBufferMemoryDescriptor    *oxygenBuffer;
        struct oxygen               *deviceRegisters;
        
        virtual bool initHardware(IOService *provider);
        virtual bool createAudioEngine();
        virtual void free();
        
        UInt16 oxygen_read_eeprom(struct oxygen *chip, unsigned int index);
        void oxygen_write_eeprom(struct oxygen *chip, unsigned int index, UInt16 value);
        void oxygen_restore_eeprom(IOPCIDevice *device, struct oxygen *chip);
        
        
        //    int oxygen_pci_probe(struct pci_dev *pci, int index, char *id,
        //                         struct module *owner,
        //                         const struct pci_device_id *ids,
        //                         int (*get_model)(struct oxygen *chip,
        //                                          const struct pci_device_id *id
        //                                          )
        //                         );
        void oxygen_pci_remove(struct pci_dev *pci);
#ifdef CONFIG_PM_SLEEP
        extern const struct dev_pm_ops oxygen_pci_pm;
#endif
        void oxygen_pci_shutdown(struct pci_dev *pci);
        

        int createMasterControl(IOAudioPort *port);
        int createSPDIFinputControl(IOAudioPort *port);
        int createSPDIFoutputControl(IOAudioPort *port);
        int oxygen_mixer_init();
               
        static IOReturn dacVolumeChangeHandler(IOService *target, IOAudioControl *volumeControl, int oldValue, int newValue);
        virtual IOReturn dac_volume_put(IOAudioControl *volumeControl, XonarAudioEngine *engine, int oldValue, int newValue);
        
        static IOReturn dacMuteChangeHandler(IOService *target, IOAudioControl *muteControl, int oldValue, int newValue);
        virtual IOReturn dac_mute_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue);
        
        static IOReturn upmixChangeHandler(IOService *target, IOAudioControl *muteControl, int oldValue, int newValue);
        virtual IOReturn upmix_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue);
        
        static IOReturn spdifOutputDefaultChangeHandler(IOService *target, IOAudioControl *volumeControl, int oldValue, int newValue);
        virtual IOReturn spdif_default_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue);
        
        static IOReturn spdifOutputSwitchChangeHandler(IOService *target, IOAudioControl *volumeControl, int oldValue, int newValue);
        virtual IOReturn spdif_switch_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue);
        virtual IOReturn spdif_switch_get(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue);
        
        static IOReturn spdifOutputPCMChangeHandler(IOService *target, IOAudioControl *volumeControl, int oldValue, int newValue);
        virtual IOReturn spdif_pcm_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue);
        
        static IOReturn spdifInputDefaultChangeHandler(IOService *target, IOAudioControl *volumeControl, int oldValue, int newValue);
        virtual IOReturn spdif_input_default_get(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue);
        
        static IOReturn spdifInputLoopbackSwitchChangeHandler(IOService *target, IOAudioControl *volumeControl, int oldValue, int newValue);
        virtual IOReturn spdif_bit_switch_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue, int private_value);
        
        virtual IOReturn spdif_bit_switch_get(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue, int private_value);
        static IOReturn spdifInputValiditySwitchChangeHandler(IOService *target, IOAudioControl *volumeControl, int oldValue, int newValue);
        
        virtual IOReturn xense_output_switch_put(IOAudioControl *volumeControl, XonarAudioEngine *engine, int oldValue, int newValue);
        virtual IOReturn st_output_switch_put(IOAudioControl *volumeControl, XonarAudioEngine *engine, int oldValue, int newValue);
        virtual IOReturn st_hp_volume_offset_put(IOAudioControl *volumeControl, XonarAudioEngine *engine, int oldValue, int newValue);
        virtual IOReturn gainChanged(IOAudioControl *gainControl, int oldValue, int newValue);
        virtual IOReturn inputMuteChanged(IOAudioControl *muteControl, int oldValue, int newValue);
        virtual IOReturn rolloff_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue);
        virtual IOReturn hp_stereo_volume_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue);
        virtual IOReturn hp_mute_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue);
        virtual IOReturn input_vol_put(IOAudioControl *volumeControl, XonarAudioEngine *engine, const void *oldData, UInt32 oldDataSize,
                                       const void *newData, UInt32 newDataSize, int private_value);
        virtual IOReturn input_sel_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue);
        virtual IOReturn hpf_put(IOAudioControl *muteControl, XonarAudioEngine *engine, int oldValue, int newValue);
            
        virtual IOReturn performPowerStateChange(IOAudioDevicePowerState oldPowerState,
                                                    IOAudioDevicePowerState newPowerState,
                                                 UInt32 *microsecondsUntilComplete );
public:
        
        static IOReturn XenseOutputChangeHandler(IOService *target, IOAudioControl *volumeControl, int oldValue, int newValue);
        static IOReturn STOutputChangeHandler(IOService *target, IOAudioControl *volumeControl, int oldValue, int newValue);
        static IOReturn SThpVolumeOffsetChangeHandler(IOService *target, IOAudioControl *volumeControl, int oldValue, int newValue);
        static IOReturn gainChangeHandler(IOService *target, IOAudioControl *gainControl, int oldValue, int newValue);
        static IOReturn inputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, int oldValue, int newValue);
        static IOReturn DGInputLineVolChangeHandler(IOService *target, IOAudioControl *volumeControl,
                                                const void *oldData, UInt32 oldDataSize, const void *newData, UInt32 newDataSize);
        static IOReturn DGInputFPMicVolChangeHandler(IOService *target, IOAudioControl *volumeControl,
                                                   const void *oldData, UInt32 oldDataSize, const void *newData, UInt32 newDataSize);
        static IOReturn DGInputMICVolChangeHandler(IOService *target, IOAudioControl *volumeControl,
                                                   const void *oldData, UInt32 oldDataSize, const void *newData, UInt32 newDataSize);
        static IOReturn DGInputAuxVolChangeHandler(IOService *target, IOAudioControl *volumeControl,
                                                   const void *oldData, UInt32 oldDataSize, const void *newData, UInt32 newDataSize );
        static IOReturn gpioBitSwitchHandler(IOService *target, IOAudioControl *ToggleControl,
                                      const void *oldData, UInt32 oldDataSize, const void* newData, UInt32 newDataSize);
        static IOReturn RollOffSelectHandler(IOService *target, IOAudioControl *SelectorControl,
                                             const void *oldData, UInt32 oldDataSize, const void* newData, UInt32 newDataSize);

};

#endif /* _SAMPLEPCIAUDIODEVICE_H */
