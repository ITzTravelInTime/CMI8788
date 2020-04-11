CMI8788 ALSA driver port to APPUL (Apple) Macintosh (MAC) OSX
(Xonar HDAV/D2X/ST etc models)

still a work in progress (need to finish initialisation, then work on mixer/controls)
* using runAction within interruptfilter via IOFilterInterruptEventSource.h to mimic gpio_work/spdif_input_bits_work workqueues (single threaded context w/in filter should be OK)

update (11 04 2020):
oxygen_{read,write} functions are functional (tested using restore_eeprom function). the remaining work involves:
1. setting up the streams/volume-controls for each submodel's engine
2. it seems IOLockWakeup and IOLockSleepDeadline should be sufficient to replace the waitqueue in linux. i would argue that IOLockWakeup/IOLockSleepDeadline are superior, since multiple threads that satisfy the condition can be woken up (whereas waitqueues can only wake one at a time).

all code belongs to Clemens Ladisch (clemens@ladisch.de).

i'm pretty sure Clemens Ladisch is JUHMAHN for "fucking masher" ;)

http://www.osxbook.com
http://www.newosxbook.com (forum is here: http://newosxbook.com/forum/)

also want to thank Siguza on the newosxbook forum for helping me get started. and much love to @pmj for answering my onslaught of inquiries.

latest log using fwkpfv (because APPULL likes to fucking ruin things and then not fix them unless the user wants to diminish their intellectual capacity by "upgrading" their operating system to one that is "currently supported" and not solely receiving security updates):

u>341736235 XonarAudioDevice::initHardware()
u>341736280 trying to initialise the locks here...
u>341736296 mutex alloc succeeded
u>341736311 SimpleLockAlloc succceeded
u>341738615 oxygen_restore_eeprom EEPROM ID restored
u>341738644 XonarAudioDevice::createAudioEngine()
u>341738660 XonarHDAVAudioEngine[]::init()
u>341738675 XonarAudioEngine::init
u>341738788 just before ac97_* calls
u>341738854 AC'97 write timeout
u>341740174 AC'97 read timeout on codec 0
u>341740255 AC'97 write timeout
u>341740396 AC'97 read timeout on codec 0
u>341740464 AC'97 write timeout
u>341740604 AC'97 read timeout on codec 0
u>341740672 AC'97 write timeout
u>341740738 AC'97 write timeout
u>341740801 AC'97 write timeout
u>341740865 AC'97 write timeout
u>341740929 AC'97 write timeout
u>341741006 AC'97 write timeout
u>341741070 AC'97 write timeout
u>341741134 AC'97 write timeout
u>341741198 AC'97 write timeout
u>341741261 AC'97 write timeout
u>341741324 AC'97 write timeout
u>341741466 AC'97 read timeout on codec 0
u>341741532 AC'97 write timeout
u>341741672 AC'97 read timeout on codec 0
u>341741747 AC'97 write timeout
u>341741887 AC'97 read timeout on codec 0
u>341741950 AC'97 write timeout
u>341741964 chip name is Xonar HDAV1.3+H6
u>341770074 Debugger: Unexpected kernel trap number: 0xe, RIP: 0xffffff7f8a1793a4, CR2: 0x10000
u>341770104 CPU 20 panic trap number 0xe, rip 0xffffff7f8a1793a4
u>341770119 cr0 0x000000008001003b cr2 0x0000000000010000 cr3 0x000000000aabe000 cr4 0x00000000000226e0
u>341770261 Debugger called: <panic>
u>341770278 IOPlatformPanicAction -> AppleAHCIDiskDriver
u>341770295 IOPlatformPanicAction -> AppleAHCIDiskDriver
u>341770311 IOPlatformPanicAction -> AppleAHCIDiskDriver
u>341770327 IOPlatformPanicAction -> AppleAHCIDiskDriver
u>341770342 IOPlatformPanicAction -> AppleSMC
u>341773603 Attempting to commit panic log to NVRAM
u>342152788 ethernet MAC address: 00:00:00:00:00:00
u>342152811 ip address: 0.0.0.0
u>342152831 
u>342152843 Waiting for remote debugger connection.
