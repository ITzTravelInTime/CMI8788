#include <IOKit/audio/IOAudioBlitterLibDispatch.h>
#include "XonarAudioEngine.hpp"
#include "XonarSPDIFAudioEngine.hpp"
#include "XonarAC97AudioEngine.hpp"
// The function clipOutputSamples() is called to clip and convert samples from the float mix buffer into the actual
// hardware sample buffer.  The samples to be clipped, are guaranteed not to wrap from the end of the buffer to the
// beginning.
// This implementation is very inefficient, but illustrates the clip and conversion process that must take place.
// Each floating-point sample must be clipped to a range of -1.0 to 1.0 and then converted to the hardware buffer
// format

// The parameters are as follows:
//        mixBuf - a pointer to the beginning of the float mix buffer - its size is based on the number of sample frames
//                     times the number of channels for the stream
//        sampleBuf - a pointer to the beginning of the hardware formatted sample buffer - this is the same buffer passed
//                    to the IOAudioStream using setSampleBuffer()
//        firstSampleFrame - this is the index of the first sample frame to perform the clipping and conversion on
//        numSampleFrames - the total number of sample frames to clip and convert
//        streamFormat - the current format of the IOAudioStream this function is operating on
//        audioStream - the audio stream this function is operating on

IOReturn XonarAudioEngine::clipOutputSamples(const void *mixBuf, void *sampleBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream)
{
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s START\n", __func__);
#endif
        UInt32 sampleIndex, maxSampleIndex;
        float *floatMixBuf;
        SInt16 *outputBuf;
        
        // Start by casting the void * mix and sample buffers to the appropriate types - float * for the mix buffer
        // and SInt16 * for the sample buffer (because our sample hardware uses signed 16-bit samples)
        floatMixBuf = (float *)mixBuf;
        outputBuf = (SInt16 *)sampleBuf;
        
        // We calculate the maximum sample index we are going to clip and convert
        // This is an index into the entire sample and mix buffers
        maxSampleIndex = (firstSampleFrame + numSampleFrames) * streamFormat->fNumChannels;
        
        // Loop through the mix/sample buffers one sample at a time and perform the clip and conversion operations
        for (sampleIndex = (firstSampleFrame * streamFormat->fNumChannels); sampleIndex < maxSampleIndex; sampleIndex++) {
                float inSample;
                
                // Fetch the floating point mix sample
                inSample = floatMixBuf[sampleIndex];
                
                // Clip that sample to a range of -1.0 to 1.0
                // A softer clipping operation could be done here
                if (inSample > 1.0) {
                        inSample = 1.0;
                } else if (inSample < -1.0) {
                        inSample = -1.0;
                }
                
                // Scale the -1.0 to 1.0 range to the appropriate scale for signed 16-bit samples and then
                // convert to SInt16 and store in the hardware sample buffer
                if (inSample >= 0) {
                        outputBuf[sampleIndex] = (SInt16) (inSample * 32767.0);
                } else {
                        outputBuf[sampleIndex] = (SInt16) (inSample * 32768.0);
                }
        }
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        return kIOReturnSuccess;
}

// The function convertInputSamples() is responsible for converting from the hardware format
// in the input sample buffer to float samples in the destination buffer and scale the samples
// to a range of -1.0 to 1.0.  This function is guaranteed not to have the samples wrapped
// from the end of the buffer to the beginning.
// This function only needs to be implemented if the device has any input IOAudioStreams

// This implementation is very inefficient, but illustrates the conversion and scaling that needs to take place.

// The parameters are as follows:
//        sampleBuf - a pointer to the beginning of the hardware formatted sample buffer - this is the same buffer passed
//                    to the IOAudioStream using setSampleBuffer()
//        destBuf - a pointer to the float destination buffer - this is the buffer that the CoreAudio.framework uses
//                    its size is numSampleFrames * numChannels * sizeof(float)
//        firstSampleFrame - this is the index of the first sample frame to the input conversion on
//        numSampleFrames - the total number of sample frames to convert and scale
//        streamFormat - the current format of the IOAudioStream this function is operating on
//        audioStream - the audio stream this function is operating on
IOReturn XonarAudioEngine::convertInputSamples(const void *sampleBuf, void *destBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream)
{
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s START\n", __func__);
#endif
        UInt32 numSamplesLeft;
        float *floatDestBuf;
        SInt16 *inputBuf;
        
        // Start by casting the destination buffer to a float *
        floatDestBuf = (float *)destBuf;
        // Determine the starting point for our input conversion
        inputBuf = &(((SInt16 *)sampleBuf)[firstSampleFrame * streamFormat->fNumChannels]);
        
        // Calculate the number of actual samples to convert
        numSamplesLeft = numSampleFrames * streamFormat->fNumChannels;
        
        // Loop through each sample and scale and convert them
        while (numSamplesLeft > 0) {
                SInt16 inputSample;
                
                // Fetch the SInt16 input sample
                inputSample = *inputBuf;
                
                // Scale that sample to a range of -1.0 to 1.0, convert to float and store in the destination buffer
                // at the proper location
                if (inputSample >= 0) {
                        *floatDestBuf = inputSample / 32767.0;
                } else {
                        *floatDestBuf = inputSample / 32768.0;
                }
                
                // Move on to the next sample
                ++inputBuf;
                ++floatDestBuf;
                --numSamplesLeft;
        }
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        return kIOReturnSuccess;
}

IOReturn XonarAC97AudioEngine::clipOutputSamples(const void *mixBuf, void *sampleBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream)
{
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s START\n", __func__);
#endif
        UInt32 sampleIndex, maxSampleIndex;
        float *floatMixBuf;
        SInt16 *outputBuf;
        
        // Start by casting the void * mix and sample buffers to the appropriate types - float * for the mix buffer
        // and SInt16 * for the sample buffer (because our sample hardware uses signed 16-bit samples)
        floatMixBuf = (float *)mixBuf;
        outputBuf = (SInt16 *)sampleBuf;
        
        // We calculate the maximum sample index we are going to clip and convert
        // This is an index into the entire sample and mix buffers
        maxSampleIndex = (firstSampleFrame + numSampleFrames) * streamFormat->fNumChannels;
        
        // Loop through the mix/sample buffers one sample at a time and perform the clip and conversion operations
        for (sampleIndex = (firstSampleFrame * streamFormat->fNumChannels); sampleIndex < maxSampleIndex; sampleIndex++) {
                float inSample;
                
                // Fetch the floating point mix sample
                inSample = floatMixBuf[sampleIndex];
                
                // Clip that sample to a range of -1.0 to 1.0
                // A softer clipping operation could be done here
                if (inSample > 1.0) {
                        inSample = 1.0;
                } else if (inSample < -1.0) {
                        inSample = -1.0;
                }
                
                // Scale the -1.0 to 1.0 range to the appropriate scale for signed 16-bit samples and then
                // convert to SInt16 and store in the hardware sample buffer
                if (inSample >= 0) {
                        outputBuf[sampleIndex] = (SInt16) (inSample * 32767.0);
                } else {
                        outputBuf[sampleIndex] = (SInt16) (inSample * 32768.0);
                }
        }
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        return kIOReturnSuccess;
}

// The function convertInputSamples() is responsible for converting from the hardware format
// in the input sample buffer to float samples in the destination buffer and scale the samples
// to a range of -1.0 to 1.0.  This function is guaranteed not to have the samples wrapped
// from the end of the buffer to the beginning.
// This function only needs to be implemented if the device has any input IOAudioStreams

// This implementation is very inefficient, but illustrates the conversion and scaling that needs to take place.

// The parameters are as follows:
//        sampleBuf - a pointer to the beginning of the hardware formatted sample buffer - this is the same buffer passed
//                    to the IOAudioStream using setSampleBuffer()
//        destBuf - a pointer to the float destination buffer - this is the buffer that the CoreAudio.framework uses
//                    its size is numSampleFrames * numChannels * sizeof(float)
//        firstSampleFrame - this is the index of the first sample frame to the input conversion on
//        numSampleFrames - the total number of sample frames to convert and scale
//        streamFormat - the current format of the IOAudioStream this function is operating on
//        audioStream - the audio stream this function is operating on
IOReturn XonarAC97AudioEngine::convertInputSamples(const void *sampleBuf, void *destBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream)
{
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s START\n", __func__);
#endif
        UInt32 numSamplesLeft;
        float *floatDestBuf;
        SInt16 *inputBuf;
        
        // Start by casting the destination buffer to a float *
        floatDestBuf = (float *)destBuf;
        // Determine the starting point for our input conversion
        inputBuf = &(((SInt16 *)sampleBuf)[firstSampleFrame * streamFormat->fNumChannels]);
        
        // Calculate the number of actual samples to convert
        numSamplesLeft = numSampleFrames * streamFormat->fNumChannels;
        
        // Loop through each sample and scale and convert them
        while (numSamplesLeft > 0) {
                SInt16 inputSample;
                
                // Fetch the SInt16 input sample
                inputSample = *inputBuf;
                
                // Scale that sample to a range of -1.0 to 1.0, convert to float and store in the destination buffer
                // at the proper location
                if (inputSample >= 0) {
                        *floatDestBuf = inputSample / 32767.0;
                } else {
                        *floatDestBuf = inputSample / 32768.0;
                }
                
                // Move on to the next sample
                ++inputBuf;
                ++floatDestBuf;
                --numSamplesLeft;
        }
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        return kIOReturnSuccess;
}

IOReturn XonarSPDIFAudioEngine::clipOutputSamples(const void *mixBuf, void *sampleBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream)
{
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s START\n", __func__);
#endif
        UInt32 sampleIndex, maxSampleIndex;
        float *floatMixBuf;
        SInt16 *outputBuf;
        
        // Start by casting the void * mix and sample buffers to the appropriate types - float * for the mix buffer
        // and SInt16 * for the sample buffer (because our sample hardware uses signed 16-bit samples)
        floatMixBuf = (float *)mixBuf;
        outputBuf = (SInt16 *)sampleBuf;
        
        // We calculate the maximum sample index we are going to clip and convert
        // This is an index into the entire sample and mix buffers
        maxSampleIndex = (firstSampleFrame + numSampleFrames) * streamFormat->fNumChannels;
        
        // Loop through the mix/sample buffers one sample at a time and perform the clip and conversion operations
        for (sampleIndex = (firstSampleFrame * streamFormat->fNumChannels); sampleIndex < maxSampleIndex; sampleIndex++) {
                float inSample;
                
                // Fetch the floating point mix sample
                inSample = floatMixBuf[sampleIndex];
                
                // Clip that sample to a range of -1.0 to 1.0
                // A softer clipping operation could be done here
                if (inSample > 1.0) {
                        inSample = 1.0;
                } else if (inSample < -1.0) {
                        inSample = -1.0;
                }
                
                // Scale the -1.0 to 1.0 range to the appropriate scale for signed 16-bit samples and then
                // convert to SInt16 and store in the hardware sample buffer
                if (inSample >= 0) {
                        outputBuf[sampleIndex] = (SInt16) (inSample * 32767.0);
                } else {
                        outputBuf[sampleIndex] = (SInt16) (inSample * 32768.0);
                }
        }
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        return kIOReturnSuccess;
}

// The function convertInputSamples() is responsible for converting from the hardware format
// in the input sample buffer to float samples in the destination buffer and scale the samples
// to a range of -1.0 to 1.0.  This function is guaranteed not to have the samples wrapped
// from the end of the buffer to the beginning.
// This function only needs to be implemented if the device has any input IOAudioStreams

// This implementation is very inefficient, but illustrates the conversion and scaling that needs to take place.

// The parameters are as follows:
//        sampleBuf - a pointer to the beginning of the hardware formatted sample buffer - this is the same buffer passed
//                    to the IOAudioStream using setSampleBuffer()
//        destBuf - a pointer to the float destination buffer - this is the buffer that the CoreAudio.framework uses
//                    its size is numSampleFrames * numChannels * sizeof(float)
//        firstSampleFrame - this is the index of the first sample frame to the input conversion on
//        numSampleFrames - the total number of sample frames to convert and scale
//        streamFormat - the current format of the IOAudioStream this function is operating on
//        audioStream - the audio stream this function is operating on
IOReturn XonarSPDIFAudioEngine::convertInputSamples(const void *sampleBuf, void *destBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream)
{
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s START\n", __func__);
#endif
        UInt32 numSamplesLeft;
        float *floatDestBuf;
        SInt16 *inputBuf;
        
        // Start by casting the destination buffer to a float *
        floatDestBuf = (float *)destBuf;
        // Determine the starting point for our input conversion
        inputBuf = &(((SInt16 *)sampleBuf)[firstSampleFrame * streamFormat->fNumChannels]);
        
        // Calculate the number of actual samples to convert
        numSamplesLeft = numSampleFrames * streamFormat->fNumChannels;
        
        // Loop through each sample and scale and convert them
        while (numSamplesLeft > 0) {
                SInt16 inputSample;
                
                // Fetch the SInt16 input sample
                inputSample = *inputBuf;
                
                // Scale that sample to a range of -1.0 to 1.0, convert to float and store in the destination buffer
                // at the proper location
                if (inputSample >= 0) {
                        *floatDestBuf = inputSample / 32767.0;
                } else {
                        *floatDestBuf = inputSample / 32768.0;
                }
                
                // Move on to the next sample
                ++inputBuf;
                ++floatDestBuf;
                --numSamplesLeft;
        }
#if DEBUG && (DEBUGLEVEL > 1)
        kprintf("XonarAudioEngine::%-30s END\n", __func__);
#endif
        return kIOReturnSuccess;
}

