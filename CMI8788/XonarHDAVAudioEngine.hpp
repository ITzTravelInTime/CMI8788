/*
 File:SamplePCIAudioEngine.h
 
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


#ifndef _XonarHDAVAUDIOENGINE_H
#define _XonarHDAVAUDIOENGINE_H


#include "XonarAudioEngine.hpp"

#define XonarHDAVAudioEngine com_CMedia_CMI8788_XonarHDAVAudioEngine

class IOFilterInterruptEventSource;
class IOInterruptEventSource;

class XonarHDAVAudioEngine : public IOAudioEngine
{
//        friend class XonarAudioEngine;
        OSDeclareDefaultStructors(XonarHDAVAudioEngine)
        
        struct xonar_hdav                   *deviceRegisters;
        int                                 data_size;
       
        SInt16							*outputBuffer;
        SInt16							*inputBuffer;
        
        IOFilterInterruptEventSource	*interruptEventSource;
        XonarAudioEngine                *engineInstance;


public:        
        virtual bool init(XonarAudioEngine *engine, struct oxygen *regs);
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
        
        
        static void xonar_hdav_resume(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void xonar_hdav_cleanup(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void xonar_hdav_suspend(struct oxygen *chip, XonarAudioEngine *engineInstance);
        static void set_hdav_params(struct oxygen *chip, XonarAudioEngine *engineInstance, IOAudioStream *currentStream);
        static int xonar_hdav_mixer_init(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *engineInstance);
        /* HDMI helper functions */
        
};

#endif /* _SAMPLEPCIAUDIOENGINE_H */
