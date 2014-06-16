Emutex 56-Port GPIO Shield Library
##################################

This is an Arduino library for the Emutex 56-Port GPIO shield, based on
the Maxim Integrated MAX9657 I/O Expander and LED Display Driver.

This shield uses SPI to communicate, and has been tested with the following
Arduino platforms:

* Arduino Uno
* Intel Galileo

Created by Emutex, an embedded software engineering company based in Ireland.
For more information, please visit http://www.emutexlabs.com

To download, click the DOWNLOAD ZIP button, rename the uncompressed folder to
Emutex_MAX9657.  Check that the Emutex_MAX9657 folder contains
Emutex_MAX9657.cpp and Emutex_MAX9657.h

Place the Emutex_MAX9657 library folder in your Arduino libraries folder, and
restart the IDE.

## TODO

* Add support for GPIO input modes
* Add option to allow per-port configuration of input/output/ccled mode
* Consider adding methods to get/set all port values in a single call
* Auto-detect number of MAX9657 devices in daisy-chain.
