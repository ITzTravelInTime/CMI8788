/*
 File:XonarSTAudioEngine.cpp
 
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
#include "XonarSTAudioEngine.hpp"

#include "oxygen.h"
#include "pcm1796.h"
#include "cm9780.h"
#include "cs2000.h"
#include "ac97.h"

#include <IOKit/audio/IOAudioDefines.h>
#include <IOKit/audio/IOAudioLevelControl.h>
#include <IOKit/audio/IOAudioToggleControl.h>
#include <IOKit/audio/IOAudioPort.h>



#define super IOAudioEngine

OSDefineMetaClassAndStructors(XonarSTAudioEngine, IOAudioEngine)



void XonarSTAudioEngine::xonar_st_init_i2c(struct oxygen *chip)
{
        oxygen_write16(chip, OXYGEN_2WIRE_BUS_STATUS,
                       OXYGEN_2WIRE_LENGTH_8 |
                       OXYGEN_2WIRE_INTERRUPT_MASK |
                       OXYGEN_2WIRE_SPEED_STANDARD);
}


void XonarSTAudioEngine::xonar_st_init_common(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        
        data->generic.output_enable_bit = GPIO_ST_OUTPUT_ENABLE;
        data->dacs = chip->model.dac_channels_mixer / 2;
        data->h6 = chip->model.dac_channels_mixer > 2;
        data->hp_gain_offset = 2*-18;
        
        engineInstance->pcm1796_init(chip);
        
        oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL,
                          GPIO_INPUT_ROUTE | GPIO_ST_HP_REAR |
                          GPIO_ST_MAGIC | GPIO_ST_HP);
        oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA,
                            GPIO_INPUT_ROUTE | GPIO_ST_HP_REAR | GPIO_ST_HP);
        
        
        engineInstance->xonar_enable_output(chip);
        
        //   snd_component_add(chip->card, "PCM1792A");
        //  snd_component_add(chip->card, "CS5381");
}

/*
 static const struct snd_kcontrol_new xense_controls[] = {
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Analog Output",
 .info = st_output_switch_info,
 .get = xense_output_switch_get,
 .put = xense_output_switch_put,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Headphones Impedance Playback Enum",
 .info = st_hp_volume_offset_info,
 .get = st_hp_volume_offset_get,
 .put = st_hp_volume_offset_put,
 },
 };
 */

int XonarSTAudioEngine::xonar_xense_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *engine)
{
        unsigned int i;
        int err;
        /* HAVE TO FIX THIS
         for (i = 0; i < ARRAY_SIZE(xense_controls); ++i) {
         err = snd_ctl_add(chip->card,
         snd_ctl_new1(&xense_controls[i], chip));
         if (err < 0)
         return err;
         }
         */
        IOAudioPort *STOutputPort;
        IOAudioToggleControl *STOutputSwitch;
        STOutputPort = IOAudioPort::withAttributes(kIOAudioPortTypeMixer, "XENSE Output");
        
        kprintf("creating ST Analog Output Switch Toggle\n");
        STOutputSwitch = IOAudioToggleControl::create(false,    // initial state - unmuted
                                                      kIOAudioControlChannelIDAll,    // Affects all channels
                                                      kIOAudioControlChannelNameAll,
                                                      0,        // control ID - driver-defined
                                                      kIOAudioControlUsageOutput);
        if (!STOutputSwitch) {
                goto Done;
        }
        
        STOutputSwitch->setValueChangeHandler((IOAudioControl::IntValueChangeHandler) ((PCIAudioDevice *)engine->audioDevice)->XenseOutputChangeHandler,
                                              engine->audioDevice);
        STOutputSwitch->setName("Analog Out");
        //engine->addDefaultAudioControl(STOutputSwitch);
        STOutputPort->addAudioControl(STOutputSwitch);
        STOutputSwitch->release();
        
        STOutputSwitch = IOAudioToggleControl::create(false,    // initial state - unmuted
                                                      kIOAudioControlChannelIDAll,    // Affects all channels
                                                      kIOAudioControlChannelNameAll,
                                                      0,        // control ID - driver-defined
                                                      kIOAudioOutputPortSubTypeHeadphones,
                                                      kIOAudioControlUsageOutput);
        
        if (!STOutputSwitch) {
                goto Done;
        }
        STOutputSwitch->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)((PCIAudioDevice *)engine->audioDevice)->SThpVolumeOffsetChangeHandler,
                                              engine->audioDevice);
        STOutputSwitch->setName("Headphones Impedance Playback Enum");
        //engine->addDefaultAudioControl(STOutputSwitch);
        STOutputPort->addAudioControl(STOutputSwitch);
        dev->attachAudioPort(STOutputPort, engine, NULL);
        STOutputSwitch->release();
        STOutputPort->release();
        err = engine->add_pcm1796_controls(chip, dev);
        if (err < 0)
                return err;
        return kIOReturnSuccess;
        
Done:
        return kIOReturnError;
}
void XonarSTAudioEngine::xonar_xense_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        
        data->generic.ext_power_reg = OXYGEN_GPI_DATA;
        data->generic.ext_power_int_reg = OXYGEN_GPI_INTERRUPT_MASK;
        data->generic.ext_power_bit = GPI_EXT_POWER;
        engineInstance->xonar_init_ext_power(chip);
        
        data->generic.anti_pop_delay = 100;
        data->has_cs2000 = 1;
        data->cs2000_regs[CS2000_FUN_CFG_1] = CS2000_REF_CLK_DIV_1;
        
        oxygen_write16(chip, OXYGEN_I2S_A_FORMAT,
                       OXYGEN_RATE_48000 |
                       OXYGEN_I2S_FORMAT_I2S |
                       OXYGEN_I2S_MCLK(MCLK_512) |
                       OXYGEN_I2S_BITS_16 |
                       OXYGEN_I2S_MASTER |
                       OXYGEN_I2S_BCLK_64);
        
        xonar_st_init_i2c(chip);
        engineInstance->cs2000_registers_init(chip);
        
        data->generic.output_enable_bit = GPIO_XENSE_OUTPUT_ENABLE;
        data->dacs = 1;
        data->hp_gain_offset = 2*-18;
        
        engineInstance->pcm1796_init(chip);
        
        oxygen_set_bits16(chip, OXYGEN_GPIO_CONTROL,
                          GPIO_INPUT_ROUTE | GPIO_ST_HP_REAR |
                          GPIO_ST_MAGIC | GPIO_XENSE_SPEAKERS);
        oxygen_clear_bits16(chip, OXYGEN_GPIO_DATA,
                            GPIO_INPUT_ROUTE | GPIO_ST_HP_REAR |
                            GPIO_XENSE_SPEAKERS);
        
        engineInstance->xonar_init_cs53x1(chip);
        engineInstance->xonar_enable_output(chip);
        
        //GOTTA MAYBE FIX THIS
        //snd_component_add(chip->card, "PCM1796");
        //snd_component_add(chip->card, "CS5381");
        //snd_component_add(chip->card, "CS2000");
}

/*
 static const struct snd_kcontrol_new st_controls[] = {
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Analog Output",
 .info = st_output_switch_info,
 .get = st_output_switch_get,
 .put = st_output_switch_put,
 },
 {
 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
 .name = "Headphones Impedance Playback Enum",
 .info = st_hp_volume_offset_info,
 .get = st_hp_volume_offset_get,
 .put = st_hp_volume_offset_put,
 },
 };
 
 */
int XonarSTAudioEngine::xonar_st_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *engine)
{
        unsigned int i;
        int err;
        /* HAVE TO FIX THIS
         for (i = 0; i < ARRAY_SIZE(xense_controls); ++i) {
         err = snd_ctl_add(chip->card,
         snd_ctl_new1(&xense_controls[i], chip));
         if (err < 0)
         return err;
         }
         */
        IOAudioPort *STOutputPort;
        IOAudioToggleControl *STOutputSwitch;
        STOutputPort = IOAudioPort::withAttributes(kIOAudioPortTypeMixer, "ST Output");
        
        kprintf("creating ST Analog Output Switch Toggle\n");
        STOutputSwitch = IOAudioToggleControl::create(false,    // initial state - unmuted
                                                      kIOAudioControlChannelIDAll,    // Affects all channels
                                                      kIOAudioControlChannelNameAll,
                                                      0,        // control ID - driver-defined
                                                      kIOAudioControlUsageOutput);
        if (!STOutputSwitch) {
                goto Done;
        }
        
        STOutputSwitch->setValueChangeHandler((IOAudioControl::IntValueChangeHandler) dev->STOutputChangeHandler,
                                              dev);
        STOutputSwitch->setName("Analog Out");
        //engine->addDefaultAudioControl(STOutputSwitch);
        STOutputPort->addAudioControl(STOutputSwitch);
        STOutputSwitch->release();
        
        STOutputSwitch = IOAudioToggleControl::create(false,    // initial state - unmuted
                                                      kIOAudioControlChannelIDAll,    // Affects all channels
                                                      kIOAudioControlChannelNameAll,
                                                      0,        // control ID - driver-defined
                                                      kIOAudioOutputPortSubTypeHeadphones,
                                                      kIOAudioControlUsageOutput);
        
        if (!STOutputSwitch) {
                goto Done;
        }
        STOutputSwitch->setValueChangeHandler((IOAudioControl::IntValueChangeHandler)dev->SThpVolumeOffsetChangeHandler,
                                              dev);
        STOutputSwitch->setName("Headphones Impedance Playback Enum");
        //engine->addDefaultAudioControl(STOutputSwitch);
        STOutputPort->addAudioControl(STOutputSwitch);
        dev->attachAudioPort(STOutputPort, engine, NULL);
        STOutputPort->release();
        STOutputSwitch->release();
        
        err = engine->add_pcm1796_controls(chip, dev);
        if (err < 0)
                return err;
        return 0;
Done:
        return -1;
}




void XonarSTAudioEngine::xonar_st_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        
        data->generic.anti_pop_delay = 100;
        data->h6 = chip->model.dac_channels_mixer > 2;
        data->has_cs2000 = 1;
        data->cs2000_regs[CS2000_FUN_CFG_1] = CS2000_REF_CLK_DIV_1;
        data->broken_i2c = true;
        
        oxygen_write16(chip, OXYGEN_I2S_A_FORMAT,
                       OXYGEN_RATE_48000 |
                       OXYGEN_I2S_FORMAT_I2S |
                       OXYGEN_I2S_MCLK(data->h6 ? MCLK_256 : MCLK_512) |
                       OXYGEN_I2S_BITS_16 |
                       OXYGEN_I2S_MASTER |
                       OXYGEN_I2S_BCLK_64);
        
        xonar_st_init_i2c(chip);
        engineInstance->cs2000_registers_init(chip);
        xonar_st_init_common(chip,engineInstance);
        
        //GOTTA MAYBE FIX THIS
        //snd_component_add(chip->card, "CS2000");
}



void XonarSTAudioEngine::xonar_st_cleanup(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        engineInstance->xonar_disable_output(chip);
}

void XonarSTAudioEngine::xonar_st_suspend(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        xonar_st_cleanup(chip,engineInstance);
}


void XonarSTAudioEngine::xonar_stx_init(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        struct xonar_pcm179x *data = (struct xonar_pcm179x*) chip->model_data;
        
        xonar_st_init_i2c(chip);
        data->generic.anti_pop_delay = 800;
        data->generic.ext_power_reg = OXYGEN_GPI_DATA;
        data->generic.ext_power_int_reg = OXYGEN_GPI_INTERRUPT_MASK;
        data->generic.ext_power_bit = GPI_EXT_POWER;
        engineInstance->xonar_init_ext_power(chip);
        xonar_st_init_common(chip, engineInstance);
}




void XonarSTAudioEngine::xonar_stx_resume(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        engineInstance->pcm1796_registers_init(chip);
        engineInstance->xonar_enable_output(chip);
}

void XonarSTAudioEngine::xonar_st_resume(struct oxygen *chip, XonarAudioEngine *engineInstance)
{
        engineInstance->cs2000_registers_init(chip);
        xonar_stx_resume(chip,engineInstance);
}

void XonarSTAudioEngine::set_st_params(struct oxygen *chip, XonarAudioEngine *instance,
                                       IOAudioStream *currentStream)
{
        instance->update_cs2000_rate(chip, instance->getSampleRate()->whole);
        //original call also sends params struct. need to stay on top of this
        //with the IOAudioStream/Engine classes. will figure that out after
        //the skeleton OOP setup is finished.
        //Linux Call:
        //set_pcm1796_params(chip, params);
        //Mac Call:
        instance->set_pcm1796_params(chip, instance, currentStream);
}





bool XonarSTAudioEngine::init(XonarAudioEngine *audioEngine, struct oxygen *chip, UInt16 model)
{
        /* sample driver init code (from SamplePCIAudioEngine.cpp's ::init) */
        bool result = false;
        
        printf("XonarSTAudioEngine[%p]::init(%p)\n", this, chip);
        
        data_size = chip->model.model_data_size;
        deviceRegisters = (struct xonar_pcm179x*) chip->model_data;
        engineInstance = audioEngine;
        
        if (!chip) {
                goto Done;
        }
        
        if (!audioEngine->init(chip, model)) {
                goto Done;
        }
        
        chip->model.cleanup = xonar_st_cleanup;
        chip->model.suspend = xonar_st_suspend;
        chip->model.resume = xonar_st_resume;
        
        
        switch (model) {
                        
                case MODEL_ST:
                        chip->model.init = xonar_st_init;
                        break;
                case MODEL_STX:
                case MODEL_STX2:
                        chip->model.init = xonar_stx_init;
                        chip->model.set_dac_params = audioEngine->set_pcm1796_params;
                        chip->model.resume = xonar_stx_resume;
                        break;
                case MODEL_XENSE:
                        chip->model.init = xonar_xense_init;
                        chip->model.mixer_init = xonar_xense_mixer_init;
                        break;
                default:
                        kprintf("XonarSTAudioEngine::init() fell into a case without corresponding model!. ID%x\n", model);
                        goto Done;
                        
        }
        
        chip->model.init(chip, engineInstance);
        audioEngine->setDescription(chip->model.shortname);
        
        result = true;
        
Done:
        
        return result;
}


void XonarSTAudioEngine::free()
{
        printf("XonarSTAudioEngine[%p]::free()\n", this);
        
        // We need to free our resources when we're going away
        // the main engine will take care of anything it allocates.
        // the submodel shouldn't free anything it doesn't allocate.
        
        if(deviceRegisters) {
                //IOFree(deviceRegisters->current_rate, sizeof(IOAudioSampleRate));
                //IOFree(deviceRegisters, data_size);
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


void XonarSTAudioEngine::stop(IOService *provider)
{
        printf("XonarSTAudioEngine[%p]::stop(%p)\n", this, provider);
        
        // When our device is being stopped and torn down, we should go ahead and remove
        // the interrupt event source from the IOWorkLoop
        // Additionally, we'll go ahead and release the interrupt event source since it isn't
        // needed any more
//        if (interruptEventSource) {
//                IOWorkLoop *wl;
//
//                wl = getWorkLoop();
//                if (wl) {
//                        wl->removeEventSource(interruptEventSource);
//                }
//
//                interruptEventSource->release();
//                interruptEventSource = NULL;
//        }
        
        // Add code to shut down hardware (beyond what is needed to simply stop the audio engine)
        // There may be nothing needed here
        
        super::stop(provider);
}

UInt32 XonarSTAudioEngine::getCurrentSampleFrame()
{
        kprintf("XonarSTAudioEngine::getCurrentSampleFrame()\n");
        
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
