#ifndef XonarGenericAudioEngine_hpp
#define XonarGenericAudioEngine_hpp

#include <IOKit/audio/IOAudioEngine.h>

#include "PCIAudioDevice.hpp"
#include "XonarAudioEngine.hpp"


enum {
    MODEL_CMEDIA_REF,
    MODEL_MERIDIAN,
    MODEL_MERIDIAN_2G,
    MODEL_CLARO,
    MODEL_CLARO_HALO,
    MODEL_FANTASIA,
    MODEL_SERENADE,
    MODEL_2CH_OUTPUT,
    MODEL_HG2PCI,
    MODEL_XONAR_DG,
    MODEL_XONAR_DGX,
};

#define GPIO_AK5385_DFS_MASK	0x0003
#define GPIO_AK5385_DFS_NORMAL	0x0000
#define GPIO_AK5385_DFS_DOUBLE	0x0001
#define GPIO_AK5385_DFS_QUAD	0x0002

#define GPIO_MERIDIAN_DIG_MASK	0x0050
#define GPIO_MERIDIAN_DIG_EXT	0x0010
#define GPIO_MERIDIAN_DIG_BOARD	0x0040

#define GPIO_CLARO_DIG_COAX	0x0040
#define GPIO_CLARO_HP		0x0100


#define XonarGenericAudioEngine com_CMedia_CMI8788_XonarGenericAudioEngine

class IOFilterInterruptEventSource;
class IOInterruptEventSource;
class XonarAudioEngine;
class XonarGenericAudioEngine : public IOAudioEngine
{
    friend class XonarAudioEngine;
    OSDeclareDefaultStructors(XonarGenericAudioEngine)
    struct xonar_generic                   *deviceRegisters;
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
    
    static void update_ak4396_volume(struct oxygen *chip);
    static void update_ak4396_mute(struct oxygen *chip);
    
    static void ak4396_write_cached(struct oxygen *chip, unsigned int codec,
                             UInt8 reg, UInt8 value, XonarAudioEngine *engineInstance);
    static void ak4396_registers_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void ak4396_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    
    static void ak5385_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    
    static void wm8785_registers_init(struct oxygen *chip, XonarAudioEngine* engineInstance);
    static void set_wm8785_params(struct oxygen *chip, XonarAudioEngine *engineInstance);

    static void wm8785_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    
    static void generic_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    
    static void meridian_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void claro_enable_hp(struct oxygen *chip);
    static void claro_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void claro_halo_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void fantasia_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void stereo_output_init(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void generic_cleanup(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void claro_disable_hp(struct oxygen *chip);
    static void claro_cleanup(struct oxygen *chip);
    static void claro_suspend(struct oxygen *chip);
    static void generic_resume(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void meridian_resume(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void claro_resume(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void stereo_resume(struct oxygen *chip, XonarAudioEngine *engineInstance);
    static void set_ak4396_params(struct oxygen *chip,
                           XonarAudioEngine *engineInstance);
    void set_ak5385_params(struct oxygen *chip);
    
    
};

#endif /* XonarGenericAudioEngine_hpp */
