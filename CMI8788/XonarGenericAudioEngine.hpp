#ifndef XonarGenericAudioEngine_hpp
#define XonarGenericAudioEngine_hpp

#include "XonarAudioEngine.hpp"



#define XonarGenericAudioEngine com_CMedia_CMI8788_XonarGenericAudioEngine

class IOFilterInterruptEventSource;
class IOInterruptEventSource;

class XonarGenericAudioEngine : public IOAudioEngine
{
        friend class XonarAudioEngine;
        OSDeclareDefaultStructors(XonarGenericAudioEngine)
        struct xonar_generic                   *deviceRegisters;
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
        
        static void ak4396_write(struct oxygen *chip, unsigned int codec,
                                 UInt8 reg, UInt8 value);
        static void ak4396_write_cached(struct oxygen *chip, unsigned int codec,
                                        UInt8 reg, UInt8 value);
        static void update_ak4396_volume(struct oxygen *chip);
        static void update_ak4396_mute(struct oxygen *chip);
        static void set_ak4396_params(struct oxygen *chip, XonarAudioEngine *engineInstance,
                                      IOAudioStream *currentStream);
        static void ak4396_registers_init(struct oxygen *chip);
        static void ak4396_init(struct oxygen *chip);
        
        static void ak5385_init(struct oxygen *chip);
        static void set_ak5385_params(struct oxygen *chip, XonarAudioEngine *engineInstance);
        
        static void wm8785_write(struct oxygen *chip, UInt8 reg, unsigned int value);
        static void wm8785_registers_init(struct oxygen *chip);
        static void set_wm8785_params(struct oxygen *chip, XonarAudioEngine *engineInstance);
        
        static void wm8785_init(struct oxygen *chip);
        static void generic_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void meridian_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
        
        static void claro_enable_hp(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void claro_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void claro_halo_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void fantasia_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void stereo_output_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void generic_cleanup(struct oxygen *chip, XonarAudioEngine *audioEngineInstance);
        static void claro_disable_hp(struct oxygen *chip);
        static void claro_cleanup(struct oxygen *chip, XonarAudioEngine *audioEngineInstance);
        static void claro_suspend(struct oxygen *chip, XonarAudioEngine *audioEngineInstance);
        static void generic_resume(struct oxygen *chip, XonarAudioEngine *audioEngineInstance);
        static void meridian_resume(struct oxygen *chip, XonarAudioEngine *audioEngineInstance);
        static void claro_resume(struct oxygen *chip, XonarAudioEngine *audioEngineInstance);
        static void stereo_resume(struct oxygen *chip, XonarAudioEngine *audioEngineInstance);
        static int generic_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *audioEngineInstance);
        static int generic_wm8785_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *audioEngineInstance);
        static int meridian_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *audioEngineInstance);
        static int claro_halo_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *audioEngineInstance);
        static int claro_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *audioEngineInstance);
        virtual int output_select_put(struct snd_kcontrol *ctl, struct snd_ctl_elem_value *value);
        

};

#endif /* XonarGenericAudioEngine_hpp */
