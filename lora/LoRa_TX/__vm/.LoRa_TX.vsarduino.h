/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Teensy 3.2 / 3.1, Platform=teensy3, Package=teensy
*/

#define __HARDWARE_MK20dx256__
#define __HARDWARE_MK20DX256__
#define ARDUINO 166
#define ARDUINO_MAIN
#define printf iprintf
#define __TEENSY3__
#define __MK20DX256__
#define TEENSYDUINO 126
#define ARDUINO 166
#define F_CPU 96000000
#define ARDUINO_ARCH_AVR
#define USB_SERIAL
#define LAYOUT_US_ENGLISH
extern "C" void __cxa_pure_virtual() {;}

//
//

#include "C:\Program Files (x86)\Arduino\hardware\teensy\avr\cores\teensy3\arduino.h"
#include <..\LoRa_TX\LoRa_TX.ino>
