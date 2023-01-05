/*
 File:XonarD2XAudioEngine.cpp
 
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





#include <IOKit/IOFilterInterruptEventSource.h>
#include "XonarD2XAudioEngine.hpp"

#include "pcm1796.h"
#include "cm9780.h"
#include "cs2000.h"
#include "ac97.h"
#include "oxygen.h"


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





int XonarD2XAudioEngine::xonar_d2_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *engineInstance)
{
        int err;
        
        //   err = snd_ctl_add(chip->card, snd_ctl_new1(&alt_switch, chip));
        if (err < 0)
                return err;
        err = engineInstance->add_pcm1796_controls(chip, dev);
        if (err < 0)
                return err;
        return 0;
}

void XonarD2XAudioEngine::xonar_d2_init(struct oxygen *chip, XonarAudioEngine *engineInstance) {
        
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        data->generic.anti_pop_delay = 300;
        data->generic.output_enable_bit = GPIO_D2_OUTPUT_ENABLE;
        data->dacs = 4;
        
        /* D2(X),ST(X)(II)+XENSE and HDAV call pcm1796x_init. given the difference in their
         data structures, passing the model variable is necessary. however, pcm179x_init
         behaves the same for D2(X)/ST(X)(II)+XENSE since they do not nest the pcm179x struct
         inside their "main" struct like the hdav model, and thus
         it doesn't matter which model value we pass *here* as long as it is *not* hdav */
        
        engineInstance->pcm1796_init(chip);
        
        oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_D2_ALT);
        oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_D2_ALT);
        
        oxygen_ac97_set_bits(chip, 0, CM9780_JACK, CM9780_FMIC2MIC);
        
        engineInstance->xonar_init_cs53x1(chip);
        engineInstance->xonar_enable_output(chip);
        
        //  snd_component_add(chip->card, "PCM1796");
        // snd_component_add(chip->card, "CS5381");
}

void XonarD2XAudioEngine::xonar_d2x_init(struct oxygen *chip, XonarAudioEngine *engineInstance) {
        
        struct xonar_pcm179x *data = (struct xonar_pcm179x *) chip->model_data;
        
        data->generic.ext_power_reg = OXYGEN_GPIO_DATA;
        data->generic.ext_power_int_reg = OXYGEN_GPIO_INTERRUPT_MASK;
        data->generic.ext_power_bit = GPIO_D2X_EXT_POWER;
        oxygen_clear_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_D2X_EXT_POWER);
        engineInstance->xonar_init_ext_power(chip);
        xonar_d2_init(chip, engineInstance);
}


bool XonarD2XAudioEngine::init(XonarAudioEngine *audioEngine, struct oxygen *chip, UInt16 submodel)
{
        bool result = false;
        
        printf("XonarD2XAudioEngine[%p]::init(%p)\n", this, chip);
    
        if (!audioEngine){
            printf("XonarD2XAudioEngine[%p]::init(%p) referenced audio engine is NULL\n", this, chip);
            goto Done;
        }
        
        if (!chip){
            printf("XonarD2XAudioEngine[%p]::init(%p)  referenced chip data is NULL\n", this, chip);
            goto Done;
        }
        
        if (!chip->model_data){
            printf("XonarD2XAudioEngine[%p]::init(%p) referenced chip->mode_data is NULL \n", this, chip);
            goto Done;
        }
    
        data_size = chip->model.model_data_size;
        //chip->model_data = IOMalloc(chip->model.model_data_size);
        deviceRegisters = (struct xonar_pcm179x*) chip->model_data;
        engineInstance = audioEngine;
        
        if (!super::init(NULL)) {
                goto Done;
        }
    
        switch(submodel) {
                case MODEL_D2:
                        xonar_d2_init(chip, audioEngine);
                        break;
                case MODEL_D2X:
                        deviceRegisters->generic.ext_power_reg = OXYGEN_GPIO_DATA;
                        deviceRegisters->generic.ext_power_int_reg = OXYGEN_GPIO_INTERRUPT_MASK;
                        deviceRegisters->generic.ext_power_bit = GPIO_D2X_EXT_POWER;
                        oxygen_clear_bits16(chip, OXYGEN_GPIO_CONTROL, GPIO_D2X_EXT_POWER);
                        audioEngine->xonar_init_ext_power(chip);
                        xonar_d2x_init(chip, audioEngine);
                        break;
                default:
                        goto Done;
                        
        }
        
        audioEngine->setDescription(chip->model.shortname);
        
        result = true;
        
Done:
        
        return result;
}


void XonarD2XAudioEngine::free()
{
        printf("XonarD2XAudioEngine[%p]::free()\n", this);
        
        // We need to free our resources when we're going away
        // the main engine will take care of anything it allocates.
        // the submodel shouldn't free anything it doesn't allocate.
        if(deviceRegisters) {
                //IOFree(deviceRegisters, data_size);
                //IOFree(deviceRegisters->current_rate, sizeof(IOAudioSampleRate));
                //deviceRegisters = NULL;
        }
        /*
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
         */
        super::free();
}

void XonarD2XAudioEngine::stop(IOService *provider)
{
        printf("XonarD2XAudioEngine[%p]::stop(%p)\n", this, provider);
        
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
UInt32 XonarD2XAudioEngine::getCurrentSampleFrame()
{
        kprintf("XonarD2XAudioEngine::getCurrentSampleFrame()\n");
        
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
