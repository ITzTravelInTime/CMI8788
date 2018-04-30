CMI8788 ALSA driver port to APPUL (Apple) Macintosh (MAC) OSX
(Xonar HDAV/D2X/ST etc models)

still a work in progress (need to test gpio/spdif queues* & move alsa mixer/pcm to IOAudioEngine)
* using runAction within interruptfilter via IOFilterInterruptEventSource.h to mimic gpio_work/spdif_input_bits_work workqueues (single threaded context w/in filter should be OK, but i am unable to test as it i think pthread fns are restricted to com.apple.kpi.private)

update (30 04 2018):
oxygen_{read,write} functions are functional (tested using restore_eeprom function).
the remaining work involves:
    1. setting up the streams/volume-controls for each submodel's engine, 
    2. finding the OSX-equivalent of wait_event_timeout. 
        -it may be the case that i am calling assert_wait_timeout incorrectly.

i will need to put my Xonar HDAV1.3 Deluxe back in my PC soon, as i am going to need it. however, i hope to pick up another card in this family to finish the work. i am hopeful others have used this work as a basis for their own investigations.


all code belongs to Clemens Ladisch (clemens@ladisch.de).

i'm pretty sure Clemens Ladisch is JUHMAHN for "fucking masher" ;)

http://www.osxbook.com
http://www.newosxbook.com (forum is here: http://newosxbook.com/forum/)

also want to thank Siguza on the newosxbook forum for his timely, insightful, helpful, kind, and honest remarks; it is because of him the driver now kernel panics ("successful") instead of failing to link. 