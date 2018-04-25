#ifndef XonarWM87x6AudioEngine_hpp
#define XonarWM87x6AudioEngine_hpp

#include <IOKit/audio/IOAudioEngine.h>

#include "PCIAudioDevice.hpp"
#include "XonarAudioEngine.hpp"


#define GPI_EXT_POWER		0x01
#define GPIO_D1_OUTPUT_ENABLE	0x0001
#define GPIO_D1_FRONT_PANEL	0x0002
#define GPIO_D1_MAGIC		0x00c0
#define GPIO_D1_INPUT_ROUTE	0x0100



#define XonarWM87x6AudioEngine com_CMedia_CMI8788_XonarWM87x6AudioEngine

class IOFilterInterruptEventSource;
class IOInterruptEventSource;
class XonarAudioEngine;
class XonarWM87x6AudioEngine : public IOAudioEngine
{
    friend class XonarAudioEngine;
    OSDeclareDefaultStructors(XonarWM87x6AudioEngine)
    struct xonar_wm87x6                  *deviceRegisters;
    //right now i've created 4 since there are 4 I2S input buffers
    // however, i am not sure how to incorporate them yet,
    // as i have to (probably) create an ioaudiostream for each
    // and then add the attributes.
    XonarAudioEngine                *engineInstance;
    IOAudioStream                   *inputs[4];
    SInt16							*outputBuffer;
    SInt16							*inputBuffer;
    IOFilterInterruptEventSource	*interruptEventSource;
    
public:
    
    
    
    
    virtual bool init(XonarAudioEngine *engine, struct oxygen *regs, UInt16 model);
    virtual void free();
    
    virtual bool initHardware(IOService *provider);
    virtual void stop(IOService *provider);
    
    virtual IOAudioStream *createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer,
                                                UInt32 sampleBufferSize);
    virtual IOReturn performAudioEngineStart();
    virtual IOReturn performAudioEngineStop();
    
    virtual UInt32 getCurrentSampleFrame();
    
    virtual IOReturn performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat,
                                         const IOAudioSampleRate *newSampleRate);
    
    //virtual IOReturn clipOutputSamples(const void *mixBuf, void *sampleBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
    //virtual IOReturn convertInputSamples(const void *sampleBuf, void *destBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
    static void interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count);
    static bool interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source);
    virtual void filterInterrupt(int index);
    
    static void wm8776_write_spi(struct oxygen *chip, unsigned int reg, unsigned int value,
                                    XonarAudioEngine *engineInstance);
    static void wm8776_write_i2c(struct oxygen *chip, unsigned int reg, unsigned int value,
                                    XonarAudioEngine *engineInstance);
    static void wm8776_write(struct oxygen *chip, unsigned int reg, unsigned int value,
                                    XonarAudioEngine *engineInstance);
    static void wm8776_write_cached(struct oxygen *chip, unsigned int reg, unsigned int value,
                                    XonarAudioEngine *engineInstance);
    static void wm8776_registers_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void wm8776_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void wm8766_write(struct oxygen *chip, unsigned int reg, unsigned int value,
                                    XonarAudioEngine *engineInstance);
    static void wm8766_write_cached(struct oxygen *chip, unsigned int reg, unsigned int value,
                                    XonarAudioEngine *engineInstance);
    
    
    static void wm8766_registers_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void wm8766_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    
    static void set_wm8776_adc_params(struct oxygen *chip,XonarAudioEngine *audioEngine);
    static void set_wm87x6_dac_params(struct oxygen *chip,XonarAudioEngine *audioEngine);
    static void update_wm8776_volume(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void update_wm87x6_volume(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void update_wm8776_mute(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void update_wm87x6_mute(struct oxygen *chip, XonarAudioEngine *engineInstance);
    
    static void xonar_ds_handle_hp_jack(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void xonar_ds_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void xonar_ds_cleanup(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void xonar_ds_suspend(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void xonar_ds_resume(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void xonar_ds_gpio_changed(struct oxygen *chip, XonarAudioEngine *audioEngine);
    
    static void xonar_hdav_slim_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void xonar_hdav_slim_cleanup(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void xonar_hdav_slim_suspend(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void xonar_hdav_slim_resume(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void set_hdav_slim_dac_params(struct oxygen *chip, XonarAudioEngine *audioEngine);
    

    
};

#endif /* XonarWM87x6AudioEngine_hpp */
