#ifndef XonarWM87x6AudioEngine_hpp
#define XonarWM87x6AudioEngine_hpp


#include "XonarAudioEngine.hpp"


#define XonarWM87x6AudioEngine com_CMedia_CMI8788_XonarWM87x6AudioEngine

class IOFilterInterruptEventSource;
class IOInterruptEventSource;
class XonarAudioEngine;
class XonarWM87x6AudioEngine : public IOAudioEngine
{
        friend class XonarAudioEngine;
        OSDeclareDefaultStructors(XonarWM87x6AudioEngine)
        struct xonar_wm87x6                  *deviceRegisters;
        int                                     data_size;
        
        XonarAudioEngine                *engineInstance;
        SInt16							*outputBuffer;
        SInt16							*inputBuffer;
        IOFilterInterruptEventSource	*interruptEventSource;
        
        
        
        
public:
        
        virtual bool init(XonarAudioEngine *engine, struct oxygen *regs, UInt16 model);
        virtual void free();
        
        virtual void stop(IOService *provider);
                
        virtual UInt32 getCurrentSampleFrame();
        
        //virtual bool initHardware(IOService *provider);
        //
        //virtual IOAudioStream *createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer,
        //                                                    UInt32 sampleBufferSize);
        //virtual IOReturn performAudioEngineStart();
        //virtual IOReturn performAudioEngineStop();
        //
        //virtual IOReturn performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat,
        //                                             const IOAudioSampleRate *newSampleRate);
        
        //virtual IOReturn clipOutputSamples(const void *mixBuf, void *sampleBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
        //virtual IOReturn convertInputSamples(const void *sampleBuf, void *destBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
        //static void interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count);
        //static bool interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source);
        
        static void wm8776_write_spi(struct oxygen *chip, unsigned int reg, unsigned int value);
        static void wm8776_write_i2c(struct oxygen *chip, unsigned int reg, unsigned int value);
        static void wm8776_write(struct oxygen *chip, unsigned int reg, unsigned int value);
        static void wm8776_write_cached(struct oxygen *chip, unsigned int reg, unsigned int value);
        static void wm8776_registers_init(struct oxygen *chip);
        static void wm8776_init(struct oxygen *chip);
        static void wm8766_write(struct oxygen *chip, unsigned int reg, unsigned int value);
        static void wm8766_write_cached(struct oxygen *chip, unsigned int reg, unsigned int value);
        
        
        static void wm8766_registers_init(struct oxygen *chip);
        static void wm8766_init(struct oxygen *chip);
        
        static void set_wm8776_adc_params(struct oxygen *chip,XonarAudioEngine *audioEngine);
        static void set_wm87x6_dac_params(struct oxygen *chip,XonarAudioEngine *audioEngine,
                                          IOAudioStream *currentStream);
        static void update_wm8776_volume(struct oxygen *chip);
        static void update_wm87x6_volume(struct oxygen *chip);
        static void update_wm8776_mute(struct oxygen *chip);
        static void update_wm87x6_mute(struct oxygen *chip);
        static void xonar_ds_handle_hp_jack(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void update_wm8766_center_lfe_mix(struct oxygen *chip, bool mixed);
        static void xonar_ds_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void xonar_ds_cleanup(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void xonar_ds_suspend(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void xonar_ds_resume(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void xonar_ds_gpio_changed(struct oxygen *chip, XonarAudioEngine *audioEngine);
        
        static void xonar_hdav_slim_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void xonar_hdav_slim_cleanup(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void xonar_hdav_slim_suspend(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void xonar_hdav_slim_resume(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void set_hdav_slim_dac_params(struct oxygen *chip, XonarAudioEngine *audioEngine,
                                             IOAudioStream *currentStream);
        
        static int wm8776_level_control_put(struct snd_kcontrol *ctl, struct snd_ctl_elem_value *value);
        //static int wm8776_bit_switch_get(struct snd_kcontrol *ctl, struct snd_ctl_elem_value *value);
        static int wm8776_bit_switch_put(struct snd_kcontrol *ctl, struct snd_ctl_elem_value *value);
        static int wm8776_field_set(struct snd_kcontrol *ctl, unsigned int value);
        static void wm8776_field_set_from_ctl(struct snd_kcontrol *ctl);
        static int wm8776_field_volume_put(struct snd_kcontrol *ctl,
                                           struct snd_ctl_elem_value *value);
        static int wm8776_field_enum_put(struct snd_kcontrol *ctl,
                                         struct snd_ctl_elem_value *value);
        static int wm8776_input_mux_put(struct snd_kcontrol *ctl,
                                        struct snd_ctl_elem_value *value);
        static int wm8776_hp_vol_put(struct snd_kcontrol *ctl,
                                     struct snd_ctl_elem_value *value);
        static int wm8776_input_vol_put(struct snd_kcontrol *ctl,
                                        struct snd_ctl_elem_value *value);
        static int hpf_put(struct snd_kcontrol *ctl, struct snd_ctl_elem_value *value);
};

#endif /* XonarWM87x6AudioEngine_hpp */
