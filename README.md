CMI8788 ALSA driver port to APPUL (Apple) Macintosh (MAC) OSX
(Xonar HDAV/D2X/ST etc models)

still a work in progress (need to test gpio/spdif queues* & move alsa mixer/pcm to IOAudioEngine)
* using runAction within interruptfilter via IOFilterInterruptEventSource.h to mimic gpio_work/spdif_input_bits_work workqueues (single threaded context w/in filter should be OK, but i am unable to test as it i think pthread fns are restricted to com.apple.kpi.private)

update (30 04 2018):
oxygen_{read,write} functions are functional (tested using restore_eeprom function). the remaining work involves:
1. setting up the streams/volume-controls for each submodel's engine
2. finding the OSX-equivalent of wait_event_timeout. 
   * it may be the case that i am calling assert_wait_timeout incorrectly.

i will need to put my Xonar HDAV1.3 Deluxe back in my PC soon, as i am going to need it. however, i hope to pick up another card in this family to finish the work. i am hopeful others have used this work as a basis for their own investigations.


all code belongs to Clemens Ladisch (clemens@ladisch.de).

i'm pretty sure Clemens Ladisch is JUHMAHN for "fucking masher" ;)

http://www.osxbook.com
http://www.newosxbook.com (forum is here: http://newosxbook.com/forum/)

also want to thank Siguza on the newosxbook forum for his timely, insightful, helpful, kind, and honest remarks; it is because of him the driver now kernel panics ("successful") instead of failing to link. 

update (23/10/2018 01:52GMT): i've managed to get the driver/kext to load/unload properly, even when a submodel class is chosen dynamically (according to the PCI vendor/submodel IDs!). 

-the driver can only load/unload once. if you try a second time, it will fail to unload and you will have to restart (i am looking at this issue, which is much more mild than having this problem on the first load/unload attempt) 

latest log:

Oct 22, 2018, 7:16:19 PM kernel[0]: SamplePCIAudioDevice[0xffffff81109f7800]::initHardware(0xffffff80f2da2a00) 
Oct 22, 2018, 7:16:19 PM kernel[0]: Xonar Vendor ID:0x13f6, Device ID:0x8788, SubDevice ID:0x8314, Physical Address:16384 
Oct 22, 2018, 7:16:19 PM kernel[0]: CMI8788: EEPROM write timeout 
Oct 22, 2018, 7:16:19 PM kernel[0]: CMI8788: EEPROM write timeout 
Oct 22, 2018, 7:16:19 PM kernel[0]: PCIAudioDevice[0xffffff81109f7800]::oxygen_restore_eeprom EEPROM ID restored 
Oct 22, 2018, 7:16:19 PM kernel[0]: SamplePCIAudioDevice[0xffffff81109f7800]::createAudioEngine() 
Oct 22, 2018, 7:16:19 PM kernel[0]: XonarHDAVAudioEngine[0xffffff80fd6b4200]::init(0xffffff8e209f1000) 
Oct 22, 2018, 7:16:19 PM kernel[0]: XonarAudioEngine[0xffffff8102b4cc00]::init(0xffffff8e209f1000) 
Oct 22, 2018, 7:16:19 PM kernel[0]: Can't unload kext com.CMedia.CMI8788.PCIAudioDriver; classes have instances: 
Oct 22, 2018, 7:16:19 PM kernel[0]: Kext com.CMedia.CMI8788.PCIAudioDriver class com_CMedia_CMI8788_XonarHDAVAudioEngine has 1 instance. 
Oct 22, 2018, 7:16:19 PM kernel[0]: Kext com.CMedia.CMI8788.PCIAudioDriver class com_CMedia_CMI8788_PCIAudioDevice has 1 instance. 
Oct 22, 2018, 7:16:19 PM kernel[0]: Kext com.CMedia.CMI8788.PCIAudioDriver class com_CMedia_CMI8788_XonarAudioEngine has 1 instance.  
Oct 22, 2018, 7:16:19 PM kernel[0]: com_CMedia_CMI8788_PCIAudioDevice: not registry member at registerService() 



