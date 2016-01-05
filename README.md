ESP8266 LIFX Application
========================

Interact with LIFX bulbs on the same network as the ESP8266!

Based on the ESP8266\_RTOS\_SDK (included as a git submodule)

Prerequisites
-------------

Must have built the [esp-open-sdk](https://github.com/pfalcon/esp-open-sdk) project. This should have cross-compiled the xtensa-lx106-elf- compiler binaries. They should be added to the system path, but can also be specified manually within the Makefile in the root of this repository.

Don't forget to init and sync the submodule in this repository before
use!

Compiling
---------

Run ```make``` from within the root of this repository.
Do not run ```make``` from within the ```src/``` directory, [it is expected to not work](http://kacangbawang.com/latest-at-firmware-for-esp8266-with-512k-flash/).
This should produce binaries in the /bin directory
"flash.sh" should deploy the binaries to the ESP8266 if
[esptool.py](https://github.com/themadinventor/esptool) has been
installed and is available on the system path.

Note
----

This project *barely* fits on the default ESP8266 512k memory map.
See [this critical blog
post](http://kacangbawang.com/esp8266-512k-flash-compiling-using-rtos-sdk-1-3/)
for details on how to expand the irom0_0_seg
NOTE: you'll need to modify ```flash.sh``` correspondingly!

Hat tipping to [kacang bawang](http://kacangbawang.com/) and
[kolban](http://bbs.espressif.com/viewtopic.php?f=7&t=1339#p4518).
