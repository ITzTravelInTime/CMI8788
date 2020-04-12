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

u>458411357 XonarAudioDevice::initHardware()<br>
u>458411403 trying to initialise the locks here...<br>
u>458411418 mutex alloc succeeded<br>
u>458411431 SimpleLockAlloc succceeded<br>
u>458413702 oxygen_restore_eeprom EEPROM ID restored<br>
u>458413733 XonarAudioDevice::createAudioEngine()<br>
u>458413749 XonarHDAVAudioEngine[]::init()<br>
u>458413763 XonarAudioEngine::init<br>
u>458413926 AC'97 write timeout<br>
u>458415156 AC'97 read timeout on codec 0<br>
u>458415234 AC'97 write timeout<br>
u>458415369 AC'97 read timeout on codec 0<br>
u>458415438 AC'97 write timeout<br>
u>458415572 AC'97 read timeout on codec 0<br>
u>458415641 AC'97 write timeout<br>
u>458415706 AC'97 write timeout<br>
u>458415770 AC'97 write timeout<br>
u>458415833 AC'97 write timeout<br>
u>458415897 AC'97 write timeout<br>
u>458415960 AC'97 write timeout<br>
u>458416024 AC'97 write timeout<br>
u>458416087 AC'97 write timeout<br>
u>458416151 AC'97 write timeout<br>
u>458416213 AC'97 write timeout<br>
u>458416276 AC'97 write timeout<br>
u>458416410 AC'97 read timeout on codec 0<br>
u>458416476 AC'97 write timeout<br>
u>458416610 AC'97 read timeout on codec 0<br>
u>458416685 AC'97 write timeout<br>
u>458416820 AC'97 read timeout on codec 0<br>
u>458416882 AC'97 write timeout<br>
u>458416899 right before the initialisation of the engine's functions<br>
u>458444781 Debugger: Unexpected kernel trap number: 0xe, RIP: 0xffffff7f925793c6, CR2: 0x10000<br>
u>458444806 CPU 6 panic trap number 0xe, rip 0xffffff7f925793c6<br>
u>458444820 cr0 0x000000008001003b cr2 0x0000000000010000 cr3 0x0000000012ebe000 cr4 0x00000000000226e0<br>
u>458444904 Debugger called: <panic><br>
u>458444921 IOPlatformPanicAction -> AppleAHCIDiskDriver<br>
u>458444938 IOPlatformPanicAction -> AppleAHCIDiskDriver<br>
u>458444953 IOPlatformPanicAction -> AppleAHCIDiskDriver<br>
u>458444969 IOPlatformPanicAction -> AppleAHCIDiskDriver<br>
u>458444984 IOPlatformPanicAction -> AppleSMC<br>
u>458448272 Attempting to commit panic log to NVRAM<br>
u>458840141 ethernet MAC address: 00:00:00:00:00:00<br>
u>458840164 ip address: 0.0.0.0<br>
u>458840184 <br>
u>458840195 Waiting for remote debugger connection.

