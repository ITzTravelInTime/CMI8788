/*
 File:SamplePCIAudioEngine.h
 
 Contains:
 
 Version:1.0.0
 
 Copyright:Copyright ) 1997-2010 by Apple Computer, Inc., All Rights Reserved.
 
 Disclaimer:IMPORTANT:  This Apple software is supplied to you by Apple Computer, Inc.
 ("Apple") in consideration of your agreement to the following terms, and your use,
 installation, modification or redistribution of this Apple software constitutes acceptance
 of these terms.  If you do not agree with these terms, please do not use, install, modify or
 redistribute this Apple software.
 
 In consideration of your agreement to abide by the following terms, and subject
 to these terms, Apple grants you a personal, non-exclusive license, under Apple's
 copyrights in this original Apple software (the "Apple Software"), to use, reproduce,
 modify and redistribute the Apple Software, with or without modifications, in source and/or
 binary forms; provided that if you redistribute the Apple Software in its entirety
 and without modifications, you must retain this notice and the following text
 and disclaimers in all such redistributions of the Apple Software.  Neither the
 name, trademarks, service marks or logos of Apple Computer, Inc. may be used to
 endorse or promote products derived from the Apple Software without specific prior
 written permission from Apple.  Except as expressly stated in this notice, no
 other rights or licenses, express or implied, are granted by Apple herein,
 including but not limited to any patent rights that may be infringed by your derivative
 works or by other works in which the Apple Software may be incorporated.
 
 The Apple Software is provided by Apple on an "AS IS" basis.  APPLE MAKES NO WARRANTIES,
 EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT,
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE APPLE SOFTWARE
 OR ITS USE AND OPERATION ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL APPLE
 BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE,
 REPRODUCTION, MODIFICATION AND/OR DISTRIBUTION OF THE APPLE SOFTWARE, HOWEVER CAUSED
 AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE), STRICT
 LIABILITY OR OTHERWISE, EVEN IF APPLE HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 */


#ifndef _AC97AUDIOENGINE_H
#define _AC97AUDIOENGINE_H

#include <IOKit/audio/IOAudioStream.h>

#ifndef _IOKIT_IOFILTERINTERRUPTEVENTSOURCE_H
class IOFilterInterruptEventSource;
class IOInterruptEventSource;
#endif

#ifndef _XONARAUDIOENGINE_H
struct IOStream;
#endif

#define XonarAC97AudioEngine com_CMedia_CMI8788_XonarAC97AudioEngine

#define XonarAudioEngine com_CMedia_CMI8788_XonarAudioEngine
class XonarAudioEngine;

class XonarAC97AudioEngine : public IOAudioEngine
{
        OSDeclareDefaultStructors(XonarAC97AudioEngine)
        friend class                    XonarAudioEngine;
        friend class                    PCIAudioDevice;
        IOStream                        *inputStream;
        IOStream                        *outputStream;
        IOFilterInterruptEventSource	*interruptEventSource;
        XonarAudioEngine                *engineInstance;

        int oxygen_ac97_hw_params(IOAudioStream *substream, int formatChange,
                                  const IOAudioStreamFormat *newFormat, const IOAudioStreamFormatExtension *newFormatEx,
                                  const IOAudioSampleRate *newSampleRate);
public:
        
        virtual bool init(XonarAudioEngine *instance);
        virtual void free();
        
        virtual bool initHardware(IOService *provider);
        virtual void stop(IOService *provider);
                
        virtual IOReturn performAudioEngineStart();
        virtual IOReturn performAudioEngineStop();
        
        virtual UInt32 getCurrentSampleFrame();
        
        virtual IOStream *createAudioStream(IOAudioStreamDirection direction, UInt32 sampleBufferSize);

           virtual IOReturn performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate);
        
        virtual IOReturn clipOutputSamples(const void *mixBuf, void *sampleBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
        virtual IOReturn convertInputSamples(const void *sampleBuf, void *destBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
        
//        static void interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count);
//        static bool interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source);
//        virtual void filterInterrupt(int index);
};

#endif /* _SAMPLEPCIAUDIOENGINE_H */
