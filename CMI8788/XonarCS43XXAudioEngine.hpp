#ifndef XonarCS43XXAudioEngine_hpp
#define XonarCS43XXAudioEngine_hpp

#import <IOKit/audio/IOAudioEngine.h>

#import "PCIAudioDevice.hpp"
#import "XonarAudioEngine.hpp"


#define GPI_EXT_POWER		0x01
#define GPIO_D1_OUTPUT_ENABLE	0x0001
#define GPIO_D1_FRONT_PANEL	0x0002
#define GPIO_D1_MAGIC		0x00c0
#define GPIO_D1_INPUT_ROUTE	0x0100



#define XonarCS43XXAudioEngine com_CMedia_CMI8788_XonarCS43XXAudioEngine

class IOFilterInterruptEventSource;
class IOInterruptEventSource;
class XonarAudioEngine;
class XonarCS43XXAudioEngine : public IOAudioEngine
{
    friend class XonarAudioEngine;
    OSDeclareDefaultStructors(XonarCS43XXAudioEngine)
    struct xonar_cs43xx                   *deviceRegisters;
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
    
    virtual IOAudioStream *createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize);
    
    virtual IOReturn performAudioEngineStart();
    virtual IOReturn performAudioEngineStop();
    
    virtual UInt32 getCurrentSampleFrame();
    
    virtual IOReturn performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate);
    
    //virtual IOReturn clipOutputSamples(const void *mixBuf, void *sampleBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
    //virtual IOReturn convertInputSamples(const void *sampleBuf, void *destBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
    static void interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count);
    static bool interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source);
    virtual void filterInterrupt(int index);


    static void xonar_d1_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void xonar_dx_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    
    static void xonar_d1_cleanup(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void xonar_d1_suspend(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void xonar_d1_resume(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void xonar_d1_line_mic_ac97_switch(struct oxygen *chip,
                                              unsigned int reg, unsigned int mute);
    
    //d1
    static void cs4398_write(struct oxygen *chip, UInt8 reg, UInt8 value, XonarAudioEngine *engineInstance);
    static void cs4398_write_cached(struct oxygen *chip, UInt8 reg, UInt8 value, XonarAudioEngine *engineInstance);
    static void cs4362a_write(struct oxygen *chip, UInt8 reg, UInt8 value, XonarAudioEngine *engineInstance);
    static void cs4362a_write_cached(struct oxygen *chip, UInt8 reg, UInt8 value, XonarAudioEngine *engineInstance);
    
    
    static void update_cs43xx_volume(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void update_cs43xx_mute(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void set_cs43xx_params(struct oxygen *chip,
                                  XonarAudioEngine *engineInstance);
    static void cs43xx_registers_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void update_cs4362a_volumes(struct oxygen *chip, XonarAudioEngine *engineInstance);
    //static void update_cs43xx_center_lfe_mix(struct oxygen *chip, bool mixed);
    

};

#endif /* XonarCS43XXAudioEngine_hpp */
