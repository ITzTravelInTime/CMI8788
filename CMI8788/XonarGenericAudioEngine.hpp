//
//  XonarGenericAudioEngine.hpp
//  PCIAudioDriver
//
//  Created by Gagan on 2018-04-23.
//  Copyright © 2018 CMedia. All rights reserved.
//

#ifndef XonarGenericAudioEngine_hpp
#define XonarGenericAudioEngine_hpp

#include "PCIAudioDevice.hpp"
#include "oxygen.h"
#include "ak4396.h"
#include "wm8785.h"


class XonarGenericAudioEngine : public IOAudioEngine
{
    friend class XonarD2XAudioEngine;
    OSDeclareDefaultStructors(XonarSTAudioEngine)
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
    
#endif /* XonarGenericAudioEngine_hpp */
