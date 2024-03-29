#ifndef XonarCS43XXAudioEngine_hpp
#define XonarCS43XXAudioEngine_hpp


#include "XonarAudioEngine.hpp"

#define XonarCS43XXAudioEngine com_CMedia_CMI8788_XonarCS43XXAudioEngine

#ifndef _IOKIT_IOFILTERINTERRUPTEVENTSOURCE_H
class IOFilterInterruptEventSource;
class IOInterruptEventSource;
#endif

class XonarCS43XXAudioEngine : public IOAudioEngine
{
        friend class XonarAudioEngine;
        OSDeclareDefaultStructors(XonarCS43XXAudioEngine)
        struct xonar_cs43xx                   *deviceRegisters;
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
        
        static void xonar_d1_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static int xonar_d1_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *engineInstance);
        
        static void xonar_dx_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
        
        static void xonar_d1_cleanup(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void xonar_d1_suspend(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void xonar_d1_resume(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void xonar_d1_line_mic_ac97_switch(struct oxygen *chip,
                                                  unsigned int reg, unsigned int mute);
        
        //d1
        static void cs4398_write(struct oxygen *chip, UInt8 reg, UInt8 value);
        static void cs4398_write_cached(struct oxygen *chip, UInt8 reg, UInt8 value);
        static void cs4362a_write(struct oxygen *chip, UInt8 reg, UInt8 value);
        static void cs4362a_write_cached(struct oxygen *chip, UInt8 reg, UInt8 value);
        
        
        static void update_cs43xx_volume(struct oxygen *chip);
        static void update_cs43xx_mute(struct oxygen *chip);
        static void set_cs43xx_params(struct oxygen *chip, XonarAudioEngine *engineInstance,
                                      IOAudioStream *currentStream);
        static void cs43xx_registers_init(struct oxygen *chip);
        static void update_cs4362a_volumes(struct oxygen *chip);
        static void update_cs43xx_center_lfe_mix(struct oxygen *chip, bool mixed);
        
        static int rolloff_get(struct snd_kcontrol *ctl, struct snd_ctl_elem_value *value);
        static int rolloff_put(struct snd_kcontrol *ctl, struct snd_ctl_elem_value *value);
        static int rolloff_info(struct snd_kcontrol *ctl, struct snd_ctl_elem_info *info);
};

#endif /* XonarCS43XXAudioEngine_hpp */
