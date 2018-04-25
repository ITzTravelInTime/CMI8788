CMI8788 ALSA driver port to APPUL (Apple) Macintosh (MAC) OSX
(Xonar HDAV/D2X/ST etc models)

still a work in progress (need to test gpio/spdif queues* & move alsa mixer/pcm to IOAudioEngine)
* using runAction within interruptfilter via IOFilterInterruptEventSource.h to mimic gpio_work/spdif_input_bits_work workqueues (single threaded context w/in filter should be OK, but i am unable to test as it i think pthread fns are restricted to com.apple.kpi.private)

all code belongs to Clemens Ladisch (clemens@ladisch.de).

i'm pretty sure Clemens Ladisch is JUHMAHN for "fucking masher" ;)

http://www.osxbook.com
http://www.newosxbook.com (forum is here: http://newosxbook.com/forum/)