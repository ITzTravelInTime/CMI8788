/*
 File:XonarHDAVAudioEngine.cpp
 
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
#include "PCIAudioDevice.hpp"
#include "XonarHDAVAudioEngine.hpp"


#include "oxygen.h"
#include "pcm1796.h"
#include "cm9780.h"
#include "cs2000.h"
#include "ac97.h"

#include <IOKit/audio/IOAudioDefines.h>
#include <IOKit/audio/IOAudioControl.h>
#include <IOKit/audio/IOAudioToggleControl.h>
#include <IOKit/audio/IOAudioPort.h>

#define INITIAL_SAMPLE_RATE	44100
#define NUM_SAMPLE_FRAMES	16384
#define NUM_CHANNELS		2
#define BIT_DEPTH			16


#define super IOAudioEngine

OSDefineMetaClassAndStructors(XonarHDAVAudioEngine, IOAudioEngine)



void XonarHDAVAudioEngine::set_hdav_params(struct oxygen *chip, XonarAudioEngine *engineInstance,
                                           IOAudioStream *currentStream)
{
        struct xonar_hdav *data = (struct xonar_hdav*) chip->model_data;
        
        engineInstance->set_pcm1796_params(chip, engineInstance, currentStream);
        engineInstance->xonar_set_hdmi_params(chip, &data->hdmi, currentStream);
}


void XonarHDAVAudioEngine::xonar_hdav_suspend(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        xonar_hdav_cleanup(chip, engineInstance);
}


void XonarHDAVAudioEngine::xonar_hdav_cleanup(struct oxygen *chip,XonarAudioEngine *engineInstance)
{
        engineInstance->xonar_hdmi_cleanup(chip);
        engineInstance->xonar_disable_output(chip);
        IODelay(2);
}

void XonarHDAVAudioEngine::xonar_hdav_resume(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        struct xonar_hdav *data = (struct xonar_hdav*) chip->model_data;
        
        engineInstance->pcm1796_registers_init(chip);
        engineInstance->xonar_hdmi_resume(chip, &data->hdmi);
        engineInstance->xonar_enable_output(chip);
}
/*
static const struct snd_kcontrol_new hdav_hdmi_control = {
.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
.name = "HDMI Playback Switch",
.info = snd_ctl_boolean_mono_info,
.get = XonarAudioEngine::xonar_gpio_bit_switch_get,
.put = XonarAudioEngine::xonar_gpio_bit_switch_put,
.private_value = GPIO_HDAV_OUTPUT_ENABLE | XONAR_GPIO_BIT_INVERT,
};
*/
int XonarHDAVAudioEngine::xonar_hdav_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *engine)
{
        int err;

        //  err = snd_ctl_add(chip->card, snd_ctl_new1(&hdav_hdmi_control, chip));

        IOAudioPort *HDMIOutputPort;
        IOAudioToggleControl *HDMIswitch;
        HDMIOutputPort = IOAudioPort::withAttributes(kIOAudioPortTypeOutput, "HDMI");
#if DEBUG && (DEBUGLEVEL > 2)
        kprintf("XonarHDAVAudioEngine::%-26s creating HDAV Analog Output Switch Toggle\n", __func__);
#endif
        HDMIswitch = IOAudioToggleControl::create(false,    // initial state - unmuted
                                                  kIOAudioControlChannelNumberInactive,    // Affects all channels
                                                  kIOAudioControlChannelNameAll,
                                                  0,        // control ID - driver-defined
                                                  kIOAudioToggleControlSubTypePhantomPower,
                                                  kIOAudioControlUsageOutput);
        if (!HDMIswitch) {
                goto Done;
        }
        
        HDMIswitch->setValueChangeHandler((IOAudioControl::DataValueChangeHandler) dev->gpioBitSwitchHandler,
                                          dev);
        HDMIswitch->setName("Playback switch");
        HDMIOutputPort->addAudioControl(HDMIswitch);
        dev->attachAudioPort(HDMIOutputPort, dev, NULL);
        HDMIswitch->release();
        HDMIOutputPort->release();
        err = engine->add_pcm1796_controls(chip, dev);
        if (err < 0)
                return err;
        return 0;
Done:
        return -1;
}


bool XonarHDAVAudioEngine::init(XonarAudioEngine *audioEngine, struct oxygen *chip)
{
        
        /* sample driver init code (from SamplePCIAudioEngine.cpp's ::init) */
        bool result = false;
        
        kprintf("XonarHDAVAudioEngine::init()\n");
        
        if (!chip) {
                goto Done;
        }
        
        if (!audioEngine->init(chip, MODEL_HDAV)) {
                goto Done;
        }
        /* end sample driver template */
        data_size = chip->model.model_data_size;
        deviceRegisters = (struct xonar_hdav*) chip->model_data;
        engineInstance = audioEngine;
        
        /* begin ALSA xonar_hdav_init */
        oxygen_write16(chip, OXYGEN_2WIRE_BUS_STATUS,
                       OXYGEN_2WIRE_LENGTH_8 |
                       OXYGEN_2WIRE_INTERRUPT_MASK |
                       OXYGEN_2WIRE_SPEED_STANDARD);
        
        deviceRegisters->pcm179x.generic.anti_pop_delay = 100;
        deviceRegisters->pcm179x.generic.output_enable_bit = GPIO_HDAV_OUTPUT_ENABLE;
        deviceRegisters->pcm179x.generic.ext_power_reg = OXYGEN_GPI_DATA;
        deviceRegisters->pcm179x.generic.ext_power_int_reg = OXYGEN_GPI_INTERRUPT_MASK;
        deviceRegisters->pcm179x.generic.ext_power_bit = GPI_EXT_POWER;
        deviceRegisters->pcm179x.dacs = chip->model.dac_channels_mixer / 2;
        deviceRegisters->pcm179x.h6 = chip->model.dac_channels_mixer > 2;
        chip->model.set_dac_params = set_hdav_params;
        chip->model.set_adc_params = engineInstance->xonar_set_cs53x1_params;
        chip->model.uart_input = audioEngine->xonar_hdmi_uart_input;
        chip->model.resume = xonar_hdav_resume;
        chip->model.cleanup = xonar_hdav_cleanup;
        chip->model.mixer_init = xonar_hdav_mixer_init;
        chip->model.pcm_hardware_filter = engineInstance->xonar_hdmi_pcm_hardware_filter;
        audioEngine->pcm1796_init(chip);
        oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL,
                          GPIO_HDAV_MAGIC | GPIO_INPUT_ROUTE);
        oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA, GPIO_INPUT_ROUTE);
        audioEngine->xonar_init_cs53x1(chip);
        audioEngine->xonar_init_ext_power(chip);
        audioEngine->xonar_hdmi_init(chip, &(deviceRegisters->hdmi));
        audioEngine->xonar_enable_output(chip);
        
        audioEngine->setDescription(chip->model.shortname);
        
        result = true;
        
        goto Done;
        /* last bits of alsa's oxygen_init */
        // snd_component_add(chip->card, "PCM1796");
        // snd_component_add(chip->card, "CS5381");
        /* begin last bits of APPUL's samplepciaudioengine::init */
Done:
        return result;
        
        
        
}

UInt32 XonarHDAVAudioEngine::getCurrentSampleFrame()
{
        kprintf("XonarHDAVAudioEngine::getCurrentSampleFrame()\n");
        
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

void XonarHDAVAudioEngine::free()
{
        kprintf("XonarHDAVAudioEngine::free()\n");

        // We need to free our resources when we're going away
        // the main engine will take care of anything it allocates.
        // the submodel shouldn't free anything it doesn't allocate.

        //if(deviceRegisters) {
               // IOFree(deviceRegisters, data_size);
           //     IOFree(deviceRegisters->pcm179x.current_rate, sizeof(IOAudioSampleRate));
               // deviceRegisters = NULL;
       // }
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
         } */
        super::free();
}

void XonarHDAVAudioEngine::stop(IOService *provider)
{
        kprintf("XonarHDAVAudioEngine::stop()\n");
        
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


