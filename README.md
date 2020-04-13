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

u>344271272 XonarAudioDevice::initHardware()<br>
u>344271323 Xonar Vendor ID:0x13f6, Device ID:0x8788, SubDevice ID:0x8314, Physical Address:0x0000000000004000<br>
u>344273579 oxygen_restore_eeprom EEPROM ID restored<br>
u>344273597 XonarAudioDevice::createAudioEngine()<br>
u>344273616 XonarHDAVAudioEngine::init()<br>
u>344273630 XonarAudioEngine::init<br>
u>344273778 AC'97 write timeout<br>
u>344275007 AC'97 read timeout on codec 0<br>
u>344275085 AC'97 write timeout<br>
u>344275219 AC'97 read timeout on codec 0<br>
u>344275287 AC'97 write timeout<br>
u>344275421 AC'97 read timeout on codec 0<br>
u>344275489 AC'97 write timeout<br>
u>344275555 AC'97 write timeout<br>
u>344275619 AC'97 write timeout<br>
u>344275682 AC'97 write timeout<br>
u>344275745 AC'97 write timeout<br>
u>344275809 AC'97 write timeout<br>
u>344275872 AC'97 write timeout<br>
u>344275936 AC'97 write timeout<br>
u>344275997 AC'97 write timeout<br>
u>344276061 AC'97 write timeout<br>
u>344276125 AC'97 write timeout<br>
u>344276258 AC'97 read timeout on codec 0<br>
u>344276325 AC'97 write timeout<br>
u>344276458 AC'97 read timeout on codec 0<br>
u>344276534 AC'97 write timeout<br>
u>344276668 AC'97 read timeout on codec 0<br>
u>344276731 AC'97 write timeout<br>
u>344406290 creating volumecontrol 1<br>
u>344406352 creating volumecontrol2<br>
u>344406381 creating volumecontrol 3<br>
u>344406404 creating volumecontrol 4<br>
u>344406428 creating volumecontrol 5<br>
u>344406452 creating volumecontrol 6<br>
u>344406490 XonarAudioEngine::initHardware()<br>
u>344407820 XonarHDAVAudioEngine::free()<br>
u>347845501 XonarAudioEngine::stop()<br>
u>347845747 XonarAudioEngine::free()<br>
u>347845904 XonarAudioDevice::free()

![Image of driver detection] (card_driver.png)
