CMI8788 ALSA driver port to APPUL Macintosh (MAC) OSX
(Xonar HDAV/D2X/ST etc models)



update(29/12/2022):

1. stream creation is complete and correct.
2. most mixer code is 'there' (save the monitor modes and probably controls for specific submodels, like pcm179x).
3. i have to figure out why the driver will sometimes crash (not panic) when loaded. i suspect it's due to the interrupt handling (or possibly even how the mixer controls interface with the driver), but i'm not sure.
        -when it does load, the system feels surprisingly stable (more so than ever before).  but when you unload it's not clean (it's a PCI kext, what do you expect?)
        - i also have to ensure runAction within the interruptfilter (via IOFilterInterruptEventSource.h) mimics gpio_work/spdif_input_bits_work workqueues (single threaded context w/in filter should be OK [i think siguza told me this])


all code belonged to Clemens Ladisch (clemens@ladisch.de). aka stackoverflwhore "CL"

i'm pretty sure Clemens Ladisch is JUHMAHN for "fucking masher" ;)

http://www.osxbook.com
http://www.newosxbook.com (forum is here: http://newosxbook.com/forum/)

also want to thank:
Siguza on the newosxbook forum for helping me get started, much love to 
@pmj for answering my onslaught of inquiries, my longtime friend
kuljit thiara for allowing me to borrow his macbook pro for one week that allowed me to finish phase one (initialising hardware) of the drier

latest log using fwkpfv (because APPULL likes to fucking ruin things and then not fix them unless the user wants to diminish their intellectual capacity by "upgrading" their operating system to one that is "currently supported" and not solely receiving security updates):
```
u>1407781808 XonarAudioDevice::initHardware()
u>1407781938 PCIAudioDevice::initHardware                     Xonar Vendor ID:0x13f6, Device ID:0x8788, SubDevice ID:0x8314, Physical Address:0x0000000000004000
u>1407782140 PCIAudioDevice::createAudioEngine                BEGIN
u>1407782166 XonarHDAVAudioEngine::init()
u>1407782186 XonarAudioEngine::init
u>1407912857 XonarAudioEngine::initHardware                   BEGIN
u>1407912887 XonarAudioEngine::                               oxygen_pcm_init()
u>1407912922 XonarAudioEngine::createAudioStream              BEGIN
u>1407913828 XonarAudioEngine::                               buffer creation COMPLETE, calling oxygen_open, direction 0, sample buff address: 0x15e60000
u>1407913925 XonarAudioEngine::oxygen_open                    BEGIN
u>1407913946 XonarAudioEngine::                               adding formats+streams for channel 4
u>1407913968 XonarAudioEngine::filterStreams                  BEGIN
u>1407914036 XonarAudioEngine::                               adding format with the following attributes:
u>1407914052 XonarAudioEngine::                               sampleRate->whole: 32000	 sampleRate->fraction: 0
u>1407914069 XonarAudioEngine::                               numChannels:8	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407914085 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407914102 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407914119 XonarAudioEngine::                               
u>1407914263 XonarAudioEngine::                               adding format with the following attributes:
u>1407914277 XonarAudioEngine::                               sampleRate->whole: 44100	 sampleRate->fraction: 0
u>1407914294 XonarAudioEngine::                               numChannels:8	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407914310 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407914326 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407914343 XonarAudioEngine::                               
u>1407914536 XonarAudioEngine::                               adding format with the following attributes:
u>1407914553 XonarAudioEngine::                               sampleRate->whole: 48000	 sampleRate->fraction: 0
u>1407914569 XonarAudioEngine::                               numChannels:8	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407914585 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407914602 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407914618 XonarAudioEngine::                               
u>1407914850 XonarAudioEngine::                               adding format with the following attributes:
u>1407914866 XonarAudioEngine::                               sampleRate->whole: 64000	 sampleRate->fraction: 0
u>1407914883 XonarAudioEngine::                               numChannels:8	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407914900 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407914916 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407914932 XonarAudioEngine::                               
u>1407915196 XonarAudioEngine::                               adding format with the following attributes:
u>1407915212 XonarAudioEngine::                               sampleRate->whole: 88200	 sampleRate->fraction: 0
u>1407915229 XonarAudioEngine::                               numChannels:8	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407915245 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407915261 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407915278 XonarAudioEngine::                               
u>1407915577 XonarAudioEngine::                               adding format with the following attributes:
u>1407915591 XonarAudioEngine::                               sampleRate->whole: 96000	 sampleRate->fraction: 0
u>1407915608 XonarAudioEngine::                               numChannels:8	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407915624 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407915641 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407915658 XonarAudioEngine::                               
u>1407916016 XonarAudioEngine::                               adding format with the following attributes:
u>1407916032 XonarAudioEngine::                               sampleRate->whole: 176400	 sampleRate->fraction: 0
u>1407916049 XonarAudioEngine::                               numChannels:8	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407916065 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407916082 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407916098 XonarAudioEngine::                               
u>1407916510 XonarAudioEngine::                               adding format with the following attributes:
u>1407916525 XonarAudioEngine::                               sampleRate->whole: 192000	 sampleRate->fraction: 0
u>1407916541 XonarAudioEngine::                               numChannels:8	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407916557 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407916574 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407916591 XonarAudioEngine::                               
u>1407917024 XonarAudioEngine::                               adding format with the following attributes:
u>1407917038 XonarAudioEngine::                               sampleRate->whole: 32000	 sampleRate->fraction: 0
u>1407917054 XonarAudioEngine::                               numChannels:6	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407917070 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407917087 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407917103 XonarAudioEngine::                               
u>1407917596 XonarAudioEngine::                               adding format with the following attributes:
u>1407917611 XonarAudioEngine::                               sampleRate->whole: 44100	 sampleRate->fraction: 0
u>1407917627 XonarAudioEngine::                               numChannels:6	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407917643 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407917660 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407917676 XonarAudioEngine::                               
u>1407918217 XonarAudioEngine::                               adding format with the following attributes:
u>1407918234 XonarAudioEngine::                               sampleRate->whole: 48000	 sampleRate->fraction: 0
u>1407918250 XonarAudioEngine::                               numChannels:6	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407918266 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407918283 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407918299 XonarAudioEngine::                               
u>1407918871 XonarAudioEngine::                               adding format with the following attributes:
u>1407918886 XonarAudioEngine::                               sampleRate->whole: 64000	 sampleRate->fraction: 0
u>1407918902 XonarAudioEngine::                               numChannels:6	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407918918 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407918935 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407918951 XonarAudioEngine::                               
u>1407919584 XonarAudioEngine::                               adding format with the following attributes:
u>1407919601 XonarAudioEngine::                               sampleRate->whole: 88200	 sampleRate->fraction: 0
u>1407919617 XonarAudioEngine::                               numChannels:6	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407919633 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407919649 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407919665 XonarAudioEngine::                               
u>1407920313 XonarAudioEngine::                               adding format with the following attributes:
u>1407920330 XonarAudioEngine::                               sampleRate->whole: 96000	 sampleRate->fraction: 0
u>1407920346 XonarAudioEngine::                               numChannels:6	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407920362 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407920378 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407920394 XonarAudioEngine::                               
u>1407921115 XonarAudioEngine::                               adding format with the following attributes:
u>1407921131 XonarAudioEngine::                               sampleRate->whole: 176400	 sampleRate->fraction: 0
u>1407921147 XonarAudioEngine::                               numChannels:6	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407921163 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407921179 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407921195 XonarAudioEngine::                               
u>1407921944 XonarAudioEngine::                               adding format with the following attributes:
u>1407921960 XonarAudioEngine::                               sampleRate->whole: 192000	 sampleRate->fraction: 0
u>1407921976 XonarAudioEngine::                               numChannels:6	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407921992 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407922008 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407922024 XonarAudioEngine::                               
u>1407922841 XonarAudioEngine::                               adding format with the following attributes:
u>1407922858 XonarAudioEngine::                               sampleRate->whole: 32000	 sampleRate->fraction: 0
u>1407922874 XonarAudioEngine::                               numChannels:4	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407922890 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407922906 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407922922 XonarAudioEngine::                               
u>1407923775 XonarAudioEngine::                               adding format with the following attributes:
u>1407923791 XonarAudioEngine::                               sampleRate->whole: 44100	 sampleRate->fraction: 0
u>1407923807 XonarAudioEngine::                               numChannels:4	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407923823 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407923839 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407923855 XonarAudioEngine::                               
u>1407924776 XonarAudioEngine::                               adding format with the following attributes:
u>1407924791 XonarAudioEngine::                               sampleRate->whole: 48000	 sampleRate->fraction: 0
u>1407924807 XonarAudioEngine::                               numChannels:4	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407924823 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407924839 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407924855 XonarAudioEngine::                               
u>1407925817 XonarAudioEngine::                               adding format with the following attributes:
u>1407925833 XonarAudioEngine::                               sampleRate->whole: 64000	 sampleRate->fraction: 0
u>1407925849 XonarAudioEngine::                               numChannels:4	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407925865 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407925881 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407925897 XonarAudioEngine::                               
u>1407926918 XonarAudioEngine::                               adding format with the following attributes:
u>1407926935 XonarAudioEngine::                               sampleRate->whole: 88200	 sampleRate->fraction: 0
u>1407926951 XonarAudioEngine::                               numChannels:4	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407926967 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407926983 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407926999 XonarAudioEngine::                               
u>1407928090 XonarAudioEngine::                               adding format with the following attributes:
u>1407928107 XonarAudioEngine::                               sampleRate->whole: 96000	 sampleRate->fraction: 0
u>1407928123 XonarAudioEngine::                               numChannels:4	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407928138 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407928154 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407928170 XonarAudioEngine::                               
u>1407929362 XonarAudioEngine::                               adding format with the following attributes:
u>1407929378 XonarAudioEngine::                               sampleRate->whole: 176400	 sampleRate->fraction: 0
u>1407929396 XonarAudioEngine::                               numChannels:4	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407929413 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407929428 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407929445 XonarAudioEngine::                               
u>1407930662 XonarAudioEngine::                               adding format with the following attributes:
u>1407930678 XonarAudioEngine::                               sampleRate->whole: 192000	 sampleRate->fraction: 0
u>1407930694 XonarAudioEngine::                               numChannels:4	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407930710 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407930727 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407930743 XonarAudioEngine::                               
u>1407932056 XonarAudioEngine::                               adding format with the following attributes:
u>1407932073 XonarAudioEngine::                               sampleRate->whole: 32000	 sampleRate->fraction: 0
u>1407932089 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407932105 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407932121 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407932137 XonarAudioEngine::                               
u>1407933563 XonarAudioEngine::                               adding format with the following attributes:
u>1407933579 XonarAudioEngine::                               sampleRate->whole: 44100	 sampleRate->fraction: 0
u>1407933595 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407933612 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407933628 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407933644 XonarAudioEngine::                               
u>1407935037 XonarAudioEngine::                               adding format with the following attributes:
u>1407935053 XonarAudioEngine::                               sampleRate->whole: 48000	 sampleRate->fraction: 0
u>1407935069 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407935086 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407935102 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407935117 XonarAudioEngine::                               
u>1407936576 XonarAudioEngine::                               adding format with the following attributes:
u>1407936593 XonarAudioEngine::                               sampleRate->whole: 64000	 sampleRate->fraction: 0
u>1407936609 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407936625 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407936641 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407936657 XonarAudioEngine::                               
u>1407938073 XonarAudioEngine::                               adding format with the following attributes:
u>1407938090 XonarAudioEngine::                               sampleRate->whole: 88200	 sampleRate->fraction: 0
u>1407938106 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407938122 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407938138 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407938154 XonarAudioEngine::                               
u>1407939717 XonarAudioEngine::                               adding format with the following attributes:
u>1407939733 XonarAudioEngine::                               sampleRate->whole: 96000	 sampleRate->fraction: 0
u>1407939750 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407939767 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407939783 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407939798 XonarAudioEngine::                               
u>1407941396 XonarAudioEngine::                               adding format with the following attributes:
u>1407941413 XonarAudioEngine::                               sampleRate->whole: 176400	 sampleRate->fraction: 0
u>1407941429 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407941445 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407941461 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407941477 XonarAudioEngine::                               
u>1407943129 XonarAudioEngine::                               adding format with the following attributes:
u>1407943146 XonarAudioEngine::                               sampleRate->whole: 192000	 sampleRate->fraction: 0
u>1407943163 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407943179 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407943195 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407943210 XonarAudioEngine::                               
u>1407944919 XonarAudioEngine::filterStreams                  END
u>1407944946 XonarAudioEngine::xonar_hdmi_pcm_hardware_filter BEGIN
u>1407944966 XonarAudioEngine::                               hardware filter for PCM_MULTICH, clearing & re-adding streams!
u>1407944982 XonarAudioEngine::                               XonarAudioEngine::filterStreams                  BEGIN
u>1407945562 XonarAudioEngine::                               adding format with the following attributes:
u>1407945578 XonarAudioEngine::                               sampleRate->whole: 44100	 sampleRate->fraction: 0
u>1407945594 XonarAudioEngine::                               numChannels:8	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407945611 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407945627 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407945643 XonarAudioEngine::                               
u>1407945775 XonarAudioEngine::                               adding format with the following attributes:
u>1407945791 XonarAudioEngine::                               sampleRate->whole: 48000	 sampleRate->fraction: 0
u>1407945807 XonarAudioEngine::                               numChannels:8	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407945823 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407945839 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407945855 XonarAudioEngine::                               
u>1407946009 XonarAudioEngine::                               adding format with the following attributes:
u>1407946025 XonarAudioEngine::                               sampleRate->whole: 96000	 sampleRate->fraction: 0
u>1407946040 XonarAudioEngine::                               numChannels:8	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407946056 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407946072 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407946088 XonarAudioEngine::                               
u>1407946286 XonarAudioEngine::                               adding format with the following attributes:
u>1407946302 XonarAudioEngine::                               sampleRate->whole: 192000	 sampleRate->fraction: 0
u>1407946318 XonarAudioEngine::                               numChannels:8	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407946334 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407946350 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407946366 XonarAudioEngine::                               
u>1407946607 XonarAudioEngine::                               adding format with the following attributes:
u>1407946623 XonarAudioEngine::                               sampleRate->whole: 44100	 sampleRate->fraction: 0
u>1407946639 XonarAudioEngine::                               numChannels:6	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407946655 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407946670 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407946686 XonarAudioEngine::                               
u>1407946955 XonarAudioEngine::                               adding format with the following attributes:
u>1407946971 XonarAudioEngine::                               sampleRate->whole: 48000	 sampleRate->fraction: 0
u>1407946987 XonarAudioEngine::                               numChannels:6	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407947003 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407947021 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407947037 XonarAudioEngine::                               
u>1407947338 XonarAudioEngine::                               adding format with the following attributes:
u>1407947354 XonarAudioEngine::                               sampleRate->whole: 96000	 sampleRate->fraction: 0
u>1407947370 XonarAudioEngine::                               numChannels:6	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407947386 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407947402 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407947418 XonarAudioEngine::                               
u>1407947761 XonarAudioEngine::                               adding format with the following attributes:
u>1407947777 XonarAudioEngine::                               sampleRate->whole: 192000	 sampleRate->fraction: 0
u>1407947793 XonarAudioEngine::                               numChannels:6	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407947809 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407947825 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407947841 XonarAudioEngine::                               
u>1407948241 XonarAudioEngine::                               adding format with the following attributes:
u>1407948257 XonarAudioEngine::                               sampleRate->whole: 44100	 sampleRate->fraction: 0
u>1407948273 XonarAudioEngine::                               numChannels:4	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407948289 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407948305 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407948321 XonarAudioEngine::                               
u>1407948751 XonarAudioEngine::                               adding format with the following attributes:
u>1407948767 XonarAudioEngine::                               sampleRate->whole: 48000	 sampleRate->fraction: 0
u>1407948783 XonarAudioEngine::                               numChannels:4	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407948799 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407948815 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407948832 XonarAudioEngine::                               
u>1407949345 XonarAudioEngine::                               adding format with the following attributes:
u>1407949360 XonarAudioEngine::                               sampleRate->whole: 96000	 sampleRate->fraction: 0
u>1407949377 XonarAudioEngine::                               numChannels:4	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407949393 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407949410 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407949426 XonarAudioEngine::                               
u>1407949953 XonarAudioEngine::                               adding format with the following attributes:
u>1407949970 XonarAudioEngine::                               sampleRate->whole: 192000	 sampleRate->fraction: 0
u>1407949985 XonarAudioEngine::                               numChannels:4	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407950001 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407950018 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407950034 XonarAudioEngine::                               
u>1407950622 XonarAudioEngine::                               adding format with the following attributes:
u>1407950638 XonarAudioEngine::                               sampleRate->whole: 44100	 sampleRate->fraction: 0
u>1407950654 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407950671 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407950688 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407950704 XonarAudioEngine::                               
u>1407951320 XonarAudioEngine::                               adding format with the following attributes:
u>1407951336 XonarAudioEngine::                               sampleRate->whole: 48000	 sampleRate->fraction: 0
u>1407951353 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407951369 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407951385 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407951401 XonarAudioEngine::                               
u>1407952079 XonarAudioEngine::                               adding format with the following attributes:
u>1407952095 XonarAudioEngine::                               sampleRate->whole: 96000	 sampleRate->fraction: 0
u>1407952111 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407952128 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407952145 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407952162 XonarAudioEngine::                               
u>1407952963 XonarAudioEngine::                               adding format with the following attributes:
u>1407952979 XonarAudioEngine::                               sampleRate->whole: 192000	 sampleRate->fraction: 0
u>1407952995 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407953011 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407953028 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:4
u>1407953045 XonarAudioEngine::                               
u>1407953886 XonarAudioEngine::filterStreams                  END
u>1407953912 XonarAudioEngine::xonar_hdmi_pcm_hardware_filter END
u>1407953933 XonarAudioEngine::                               formats added without issue.
u>1407953955 XonarAudioEngine::                               calling IOLock and checking spdif bits
u>1407953983 XonarAudioEngine::oxygen_open                    END
u>1407954005 XonarAudioEngine::                               setting sample buffer
u>1407955466 XonarAudioEngine::                               setting sample format
u>1407955767 XonarAudioEngine::createAudioStream              END
u>1407956095 XonarAudioEngine::createAudioStream              BEGIN
u>1407956640 XonarAudioEngine::                               buffer creation COMPLETE, calling oxygen_open, direction 1, sample buff address: 0x15f60000
u>1407956734 XonarAudioEngine::oxygen_open                    BEGIN
u>1407956761 XonarAudioEngine::                               adding formats+streams for channel 1
u>1407956789 XonarAudioEngine::filterStreams                  BEGIN
u>1407956824 XonarAudioEngine::                               adding format with the following attributes:
u>1407956840 XonarAudioEngine::                               sampleRate->whole: 32000	 sampleRate->fraction: 0
u>1407956857 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407956873 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407956889 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:1
u>1407956906 XonarAudioEngine::                               
u>1407957043 XonarAudioEngine::                               adding format with the following attributes:
u>1407957059 XonarAudioEngine::                               sampleRate->whole: 44100	 sampleRate->fraction: 0
u>1407957076 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407957092 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407957109 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:1
u>1407957125 XonarAudioEngine::                               
u>1407957287 XonarAudioEngine::                               adding format with the following attributes:
u>1407957303 XonarAudioEngine::                               sampleRate->whole: 48000	 sampleRate->fraction: 0
u>1407957319 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407957335 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407957352 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:1
u>1407957369 XonarAudioEngine::                               
u>1407957591 XonarAudioEngine::                               adding format with the following attributes:
u>1407957606 XonarAudioEngine::                               sampleRate->whole: 64000	 sampleRate->fraction: 0
u>1407957622 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407957638 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407957655 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:1
u>1407957671 XonarAudioEngine::                               
u>1407957940 XonarAudioEngine::                               adding format with the following attributes:
u>1407957956 XonarAudioEngine::                               sampleRate->whole: 88200	 sampleRate->fraction: 0
u>1407957973 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407957989 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407958006 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:1
u>1407958023 XonarAudioEngine::                               
u>1407958333 XonarAudioEngine::                               adding format with the following attributes:
u>1407958348 XonarAudioEngine::                               sampleRate->whole: 96000	 sampleRate->fraction: 0
u>1407958364 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407958380 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407958397 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:1
u>1407958413 XonarAudioEngine::                               
u>1407958759 XonarAudioEngine::                               adding format with the following attributes:
u>1407958774 XonarAudioEngine::                               sampleRate->whole: 176400	 sampleRate->fraction: 0
u>1407958791 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407958808 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407958824 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:1
u>1407958840 XonarAudioEngine::                               
u>1407959230 XonarAudioEngine::                               adding format with the following attributes:
u>1407959245 XonarAudioEngine::                               sampleRate->whole: 192000	 sampleRate->fraction: 0
u>1407959262 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407959278 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407959294 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:1
u>1407959310 XonarAudioEngine::                               
u>1407959774 XonarAudioEngine::filterStreams                  END
u>1407959800 XonarAudioEngine::xonar_hdmi_pcm_hardware_filter BEGIN
u>1407959821 XonarAudioEngine::xonar_hdmi_pcm_hardware_filter END
u>1407959842 XonarAudioEngine::                               formats added without issue.
u>1407959863 XonarAudioEngine::                               calling IOLock and checking spdif bits
u>1407959887 XonarAudioEngine::oxygen_open                    END
u>1407959907 XonarAudioEngine::                               setting sample buffer
u>1407960114 XonarAudioEngine::                               setting sample format
u>1407960398 XonarAudioEngine::createAudioStream              END
u>1407960539 XonarSPDIFAudioEngine::init                      BEGIN
u>1407960638 XonarSPDIFAudioEngine::init                      END
u>1407960705 XonarSPDIFAudioEngine::initHardware              BEGIN
u>1407960738 XonarSPDIFAudioEngine::createAudioStream         BEGIN
u>1407961096 XonarSPDIFAudioEngine::                          buffer creation COMPLETE, calling oxygen_open, direction 0, sample buff address: 0x15f80000
u>1407961161 XonarAudioEngine::oxygen_open                    BEGIN
u>1407961182 XonarAudioEngine::                               adding formats+streams for channel 3
u>1407961209 XonarAudioEngine::filterStreams                  BEGIN
u>1407961236 XonarAudioEngine::                               adding format with the following attributes:
u>1407961251 XonarAudioEngine::                               sampleRate->whole: 32000	 sampleRate->fraction: 0
u>1407961267 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407961284 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407961301 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:3
u>1407961318 XonarAudioEngine::                               
u>1407961481 XonarAudioEngine::                               adding format with the following attributes:
u>1407961495 XonarAudioEngine::                               sampleRate->whole: 44100	 sampleRate->fraction: 0
u>1407961512 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407961528 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407961545 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:3
u>1407961562 XonarAudioEngine::                               
u>1407961730 XonarAudioEngine::                               adding format with the following attributes:
u>1407961745 XonarAudioEngine::                               sampleRate->whole: 48000	 sampleRate->fraction: 0
u>1407961761 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407961778 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407961795 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:3
u>1407961812 XonarAudioEngine::                               
u>1407962023 XonarAudioEngine::                               adding format with the following attributes:
u>1407962040 XonarAudioEngine::                               sampleRate->whole: 64000	 sampleRate->fraction: 0
u>1407962056 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407962072 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407962089 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:3
u>1407962105 XonarAudioEngine::                               
u>1407962355 XonarAudioEngine::                               adding format with the following attributes:
u>1407962370 XonarAudioEngine::                               sampleRate->whole: 88200	 sampleRate->fraction: 0
u>1407962386 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407962403 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407962419 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:3
u>1407962436 XonarAudioEngine::                               
u>1407962754 XonarAudioEngine::                               adding format with the following attributes:
u>1407962770 XonarAudioEngine::                               sampleRate->whole: 96000	 sampleRate->fraction: 0
u>1407962787 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407962803 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407962819 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:3
u>1407962836 XonarAudioEngine::                               
u>1407963222 XonarAudioEngine::                               adding format with the following attributes:
u>1407963238 XonarAudioEngine::                               sampleRate->whole: 176400	 sampleRate->fraction: 0
u>1407963254 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407963271 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407963287 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:3
u>1407963303 XonarAudioEngine::                               
u>1407963763 XonarAudioEngine::                               adding format with the following attributes:
u>1407963780 XonarAudioEngine::                               sampleRate->whole: 192000	 sampleRate->fraction: 0
u>1407963797 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407963813 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407963829 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:3
u>1407963846 XonarAudioEngine::                               
u>1407964336 XonarAudioEngine::filterStreams                  END
u>1407964363 XonarAudioEngine::xonar_hdmi_pcm_hardware_filter BEGIN
u>1407964384 XonarAudioEngine::xonar_hdmi_pcm_hardware_filter END
u>1407964417 XonarAudioEngine::                               formats added without issue.
u>1407964439 XonarAudioEngine::                               calling IOLock and checking spdif bits
u>1407964460 XonarAudioEngine::oxygen_open                    END
u>1407964480 XonarSPDIFAudioEngine::                          setting sample buffer
u>1407964680 XonarSPDIFAudioEngine::                          setting sample format
u>1407964960 XonarSPDIFAudioEngine::createAudioStream         END
u>1407965086 XonarSPDIFAudioEngine::createAudioStream         BEGIN
u>1407965329 XonarSPDIFAudioEngine::                          buffer creation COMPLETE, calling oxygen_open, direction 1, sample buff address: 0x15fa0000
u>1407965400 XonarAudioEngine::oxygen_open                    BEGIN
u>1407965421 XonarAudioEngine::                               adding formats+streams for channel 2
u>1407965449 XonarAudioEngine::filterStreams                  BEGIN
u>1407965477 XonarAudioEngine::                               adding format with the following attributes:
u>1407965493 XonarAudioEngine::                               sampleRate->whole: 32000	 sampleRate->fraction: 0
u>1407965510 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407965527 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407965543 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:2
u>1407965560 XonarAudioEngine::                               
u>1407965730 XonarAudioEngine::                               adding format with the following attributes:
u>1407965747 XonarAudioEngine::                               sampleRate->whole: 44100	 sampleRate->fraction: 0
u>1407965763 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407965780 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407965797 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:2
u>1407965814 XonarAudioEngine::                               
u>1407966005 XonarAudioEngine::                               adding format with the following attributes:
u>1407966022 XonarAudioEngine::                               sampleRate->whole: 48000	 sampleRate->fraction: 0
u>1407966039 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407966055 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407966072 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:2
u>1407966088 XonarAudioEngine::                               
u>1407966298 XonarAudioEngine::                               adding format with the following attributes:
u>1407966313 XonarAudioEngine::                               sampleRate->whole: 64000	 sampleRate->fraction: 0
u>1407966329 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407966346 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407966362 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:2
u>1407966379 XonarAudioEngine::                               
u>1407966649 XonarAudioEngine::                               adding format with the following attributes:
u>1407966664 XonarAudioEngine::                               sampleRate->whole: 88200	 sampleRate->fraction: 0
u>1407966680 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407966696 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407966713 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:2
u>1407966729 XonarAudioEngine::                               
u>1407967000 XonarAudioEngine::                               adding format with the following attributes:
u>1407967017 XonarAudioEngine::                               sampleRate->whole: 96000	 sampleRate->fraction: 0
u>1407967034 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407967050 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407967066 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:2
u>1407967083 XonarAudioEngine::                               
u>1407967430 XonarAudioEngine::                               adding format with the following attributes:
u>1407967447 XonarAudioEngine::                               sampleRate->whole: 176400	 sampleRate->fraction: 0
u>1407967464 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407967480 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407967497 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:2
u>1407967513 XonarAudioEngine::                               
u>1407967880 XonarAudioEngine::                               adding format with the following attributes:
u>1407967896 XonarAudioEngine::                               sampleRate->whole: 192000	 sampleRate->fraction: 0
u>1407967913 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407967929 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407967945 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:2
u>1407967962 XonarAudioEngine::                               
u>1407968371 XonarAudioEngine::filterStreams                  END
u>1407968398 XonarAudioEngine::filterStreams                  BEGIN
u>1407968551 XonarAudioEngine::                               adding format with the following attributes:
u>1407968566 XonarAudioEngine::                               sampleRate->whole: 44100	 sampleRate->fraction: 0
u>1407968583 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407968600 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407968616 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:2
u>1407968633 XonarAudioEngine::                               
u>1407968753 XonarAudioEngine::                               adding format with the following attributes:
u>1407968770 XonarAudioEngine::                               sampleRate->whole: 48000	 sampleRate->fraction: 0
u>1407968787 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407968803 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407968819 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:2
u>1407968835 XonarAudioEngine::                               
u>1407968997 XonarAudioEngine::                               adding format with the following attributes:
u>1407969013 XonarAudioEngine::                               sampleRate->whole: 88200	 sampleRate->fraction: 0
u>1407969030 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407969046 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407969063 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:2
u>1407969079 XonarAudioEngine::                               
u>1407969279 XonarAudioEngine::                               adding format with the following attributes:
u>1407969295 XonarAudioEngine::                               sampleRate->whole: 96000	 sampleRate->fraction: 0
u>1407969312 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407969329 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407969346 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:2
u>1407969362 XonarAudioEngine::                               
u>1407969592 XonarAudioEngine::                               adding format with the following attributes:
u>1407969607 XonarAudioEngine::                               sampleRate->whole: 176400	 sampleRate->fraction: 0
u>1407969623 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407969639 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407969656 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:2
u>1407969672 XonarAudioEngine::                               
u>1407969963 XonarAudioEngine::                               adding format with the following attributes:
u>1407969978 XonarAudioEngine::                               sampleRate->whole: 192000	 sampleRate->fraction: 0
u>1407969994 XonarAudioEngine::                               numChannels:2	 sampleFormat:lpcm	 NumericRepresentation:sint
u>1407970011 XonarAudioEngine::                               bitDepth:(16, 24)	 bitWidth:(16, 32)	 streamAlignment:LowByte
u>1407970027 XonarAudioEngine::                               streamByteOrder:Li'l PennE, mixable:true, driverTag:2
u>1407970044 XonarAudioEngine::                               
u>1407970381 XonarAudioEngine::filterStreams                  END
u>1407970402 XonarAudioEngine::xonar_hdmi_pcm_hardware_filter BEGIN
u>1407970423 XonarAudioEngine::xonar_hdmi_pcm_hardware_filter END
u>1407970444 XonarAudioEngine::                               formats added without issue.
u>1407970465 XonarAudioEngine::                               calling IOLock and checking spdif bits
u>1407970493 XonarAudioEngine::oxygen_open                    END
u>1407970514 XonarSPDIFAudioEngine::                          setting sample buffer
u>1407970709 XonarSPDIFAudioEngine::                          setting sample format
u>1407970975 XonarSPDIFAudioEngine::createAudioStream         END
u>1407971119 XonarSPDIFAudioEngine::initHardware              END
u>1407971503 XonarAudioEngine::initHardware                   SPDIF engine successfully created.
u>1407971522 XonarAC97AudioEngine::init                       BEGIN
u>1407971574 XonarAC97AudioEngine::initHardware               BEGIN
u>1407971603 XonarAC97AudioEngine::initHardware               END
u>1407971972 XonarAudioEngine::initHardware                   AC97 engine successfully created.
u>1407971991 XonarAudioEngine::                               oxygen_pcm_init completed, initialising mixer
u>1407972009 XonarAudioEngine::                               before SPDIF detect lock
u>1407972029 XonarAudioEngine::                               after SPDIF detect lock
u>1407972045 XonarAudioEngine::                               setting samplerate
u>1407972069 XonarAudioEngine::                               setting number of frames per buffer
u>1407972093 XonarAudioEngine::initHardware                   END
u>1407972180 XonarAudioEngine:oxygen_trigger                 enginestatus is Stopped
u>1407972198 XonarAudioEngine::oxygen_trigger                 adding stream 1 to the pcm_running mask
u>1407972221 XonarAudioEngine::oxygen_trigger                 adding stream 2 to the pcm_running mask
u>1407972242 XonarAudioEngine::oxygen_trigger                 adding stream 3 to the pcm_running mask
u>1407972264 XonarAudioEngine::oxygen_trigger                 adding stream 4 to the pcm_running mask
u>1407972285 PCIAudioDevice::createAudioEngine                END
u>1411739579 XonarAudioEngine::stop                           BEGIN
u>1411739645 XonarAudioEngine::stop                           try to release submodel here..
u>1411739663 XonarHDAVAudioEngine::free()
u>1411739685 XonarAudioEngine::stop                           END
u>1411739706 XonarSPDIFAudioEngine::stop                      BEGIN
u>1411739875 XonarSPDIFAudioEngine::stop                      END
u>1411739914 XonarAC97AudioEngine::stop                       BEGIN
u>1411739953 XonarAC97AudioEngine::stop                       END
u>1411740577 XonarAudioDevice::free()
u>1411741567 XonarAC97AudioEngine::free                       BEGIN
u>1411743377 XonarSPDIFAudioEngine::free                      BEGIN
u>1411743441 XonarSPDIFAudioEngine::free                      END
u>1411744327 XonarAudioEngine::free                          
u>1411744346 XonarAudioEngine::                               freeing chipData->model_data
u>1411744364 XonarAudioEngine::                               releasing interruptEventSource_main
u>1411744383 XonarAudioEngine::                               freeing substream buffers
u>1411744400 XonarAudioEngine::                               freeing output stream
u>1411744430 XonarAudioEngine::                               freeing input stream 1
u>1411744526 XonarAudioEngine::free                           END
```

![Image of driver detection](https://raw.githubusercontent.com/i3roly/CMI8788/master/card_driver_new1.png)
![Image of driver detection](https://raw.githubusercontent.com/i3roly/CMI8788/master/card_driver_new2.png)
