# MattairTech Arduino AVR Core

This is a fork of the Arduino AVR core from arduino/Arduino (hardware/arduino/avr/ directory)
on GitHub. This will be used to maintain Arduino support for AVR boards including the
MattairTech MT-DB-U1, MT-DB-U2, MT-DB-U4, and the MT-DB-U6 (see https://www.mattairtech.com/).

This core is intended to be installed using Boards Manager (see below). To update from a
previous version, click on MattairTech AVR Boards in Boards Manager, then click Update.


## What's New

* Initial release of the 1.6.x compatible AVR core.
* Any combination of CDC, HID, or UART can be used (or no combination), by using the Tools->Communication menu.
* Note that switching between CDC and CDC+HID will require re-selecting the COM port.
* More detailed memory usage at end of compilation (see below).
* Merged in upstream updates.


## Summary

Feature                 |	MT-DB-U6				|	MT-DB-U4		|	MT-DB-U2		|	MT-DB-U1
------------------------|-----------------------------------------------|-------------------------------|-------------------------------|------------------------------
Microcontroller		|	AT90USB64/AT90USB128, 8-Bit AVR		|	ATmega32U4, 8-Bit AVR	|	ATmega32U2, 8-Bit AVR	|	AT90USB162, 8-Bit AVR
Clock Speed		|	16 MHz					|	16 MHz			|	16 MHz			|	16 MHz
Flash Memory		|	128 KB (AT90USB128) / 64 KB (AT90USB64)	|	32 KB			|	32 KB			|	16 KB
SRAM			|	8 KB (AT90USB128) / 4 KB (AT90USB64)	|	2.5 KB			|	1 KB			|	512 B
EEPROM			|	4 KB (AT90USB128) / 2 KB (AT90USB64)	|	1 KB			|	1 KB			|	512 B
Digital Pins		|	46*					|	26			|	21			|	21
Analog Input Pins	|	8 (10-bit)				|	11* (10-bit)		|	No analog		|	No analog
PWM Output Pins		|	7*					|	7			|	4			|	4
External Interrupts	|	6* (8 PCINT)*				|	5 (8 PCINT)*		|	8 (13 PCINT)*		|	8 (13 PCINT)*
USB			|	CDC and HID				|	CDC and HID		|	CDC and HID		|	CDC and HID
UART (Serial)		|	1					|	1			|	1			|	1
SPI			|	1					|	1			|	1			|	1
I2C (TWI)		|	1					|	1			|	No I2C			|	No I2C
Operating Voltage	|	5V/3.3V					|	5V/3.3V			|	5V/3.3V			|	5V/3.3V
DC Current per I/O Pin	|	20 mA					|	20 mA			|	20 mA			|	20 mA

* Only INT pins are supported in this core (PCINT pins are not supported).
* MT-DB-U4: 1 additional analog pin is available by disconnecting the LED (solder jumper on rev B and higher boards)
* MT-DB-U6-64/128: 2 additional digital, 2 additional PWM, or 2 additional INT pins available with RTC crystal removed.
  Note however, that the RTC crystal holes are smaller and closer together than the header pin holes.


## Special Notes

* Tools->Communications menu
  Currently, the Tools->Communications menu must be used to select the communications configuration. This configuration
  must match the included libraries. For example, when including the HID and Keyboard libraries, you must select an
  option that includes HID (ie: CDC_HID_UART). This menu is currently needed to select the USB PID that matches the
  USB device configuration (needed for Windows). This may become automatic in a future release.

* Incude platform specific libraries
  You may need to manually include platform specific libraries such as SPI.h, Wire.h, and HID.h.

* EXCEPTION_FOR_57600
  The MattairTech ArduinoCore-avr uses a more accurate baud rate for 57600 than the stock arduino.
  When using the USART to communicate with another Arduino, define EXCEPTION_FOR_57600.

* New interrupt mapping
  The MattairTech ArduinoCore-avr has changed interrupt pin mapping from the previous 1.0.5 release.
  The arduino pin number is now used with attachInterrupt() instead of the interrupt number. See 'Pin Configurations' below.


## Pin Configurations

To determine the Arduino pin number, start at the upper-left corner of the board opposite of the USB connector. This is
pin 0 (most boards have a 0 printed nearby). The numbering increases in a counter-clockwise direction around the board.
Many pins have multiple configurations available. For example, arduino pin 29 (AVR pin D0) on the MT-DB-U6 can be a PWM
output (analogWrite), an external interrupt input, digital I/O, or the SCL pin of I2C.

### MT-DB-U6 (AT90USB64/AT90USB128)

```
================== MattairTech MT-DB-U6 (AT90USB64/AT90USB128) ==========================
INT/Other  PWM  Analog  Digital                      Digital   PWM      INT/Other    Comm
=========================================================================================
                               ---------------------
LED                        0  | E0/L      O O   RST |
                           1  | E1        HWB   E2/B|  37                JUMPER
                           2  | C0              D7  |  36
                           3  | C1   O O        D6  |  35
                           4  | C2   O O        D5  |  34                            XCK
                           5  | C3   O O        D4  |  33
        6 (TC3C)           6  | C4   O O *      D3  |  32                32 (INT3)   TX
        7 (TC3B)           7  | C5  PORT A      D2  |  31                31 (INT2)   RX
        8 (TC3A)           8  | C6              D1  |  30   30 (TC2B)    30 (INT1)   SDA
                           9  | C7              D0  |  29   29 (TC0B)    29 (INT0)   SCL
                              |                     |
JTAG TDI       10 (ADC7)  10  | F7              E3  |  28
JTAG TDO       11 (ADC6)  11  | F6      PWR SW  E7  |  27                27 (INT7)
JTAG TMS       12 (ADC5)  12  | F5        O     B7  |  26   26 (TC1C)
JTAG TCK       13 (ADC4)  13  | F4  - +   O     B6  |  25   25 (TC1B)
               14 (ADC3)  14  | F3  O O         B5  |  24   24 (TC1A)
               15 (ADC2)  15  | F2  PWR IN      B4  |  23   23 (TC2A)
               16 (ADC1)  16  | F1              B3  |  22                            MISO
               17 (ADC0)  17  | F0      * O O   B2  |  21                            MOSI
18 (INT6)                 18  | E6        O O   B1  |  20                            SCLK 
                              | Aref      O O   B0  |  19                            SS
                              | Vbus      ISP   3.3V|
                              | D-     _____    Vcc |
                              | D+    |     |   5V  |
                              | Gnd   | USB |   Gnd |
                               ---------------------

* Pins 38-45 are on the PORT A header. Pins 46 and 47 are the RTC crystal pins E4 and E5
  (in use by the RTC by default). With RTC crystal removed, there are 2 additional digital
  pins (46 and 47), 2 additional PWM pins (TIMER2A on pin 23 and TIMER2B on pin 30), and 2
  additional INT pins (INT4 on pin 46 and INT5 on pin 47). All pins can be used with
  analogRead(). 8 of these pins are actual analog inputs, the rest connect to the internal
  reference (pin 47) or ground.
```

### MT-DB-U4 (ATmega32U4)

```
========================= MattairTech MT-DB-U4 (ATmega32U4) =============================
INT/Other  PWM    Analog   Digital                 Digital  Analog   PWM  INT/other  Comm
=========================================================================================
                                 -------------------
                  0 (ADC11)  0  | B4            RST |
        1 (TC1A)  1 (ADC12)  1  | B5            D7/L| 25  25 (ADC10)  25 (TC4D)  LED
        2 (TC1B)  2 (ADC13)  2  | B6            D6  | 24  24 (ADC9)
        3 (TC3A)             3  | C6            D5  | 23  23 (REF)
        4 (TC4A)             4  | C7            D4  | 22  22 (ADC8)
JUMPER                       5  | E2/B          D3  | 21                   21 (INT3)  TX
                                | Agnd          D2  | 20                   20 (INT2)  RX
                  6 (ADC7)   6  | F7            D1  | 19                   19 (INT1)  SDA
                  7 (ADC6)   7  | F6            D0  | 18        18 (TC0B)  18 (INT0)  SCL
                  8 (ADC5)   8  | F5           xtal1|
                  9 (ADC4)   9  | F4           xtal2|
                  10 (ADC1)  10 | F1            B7  | 17        17 (TC1C)
                  11 (ADC0)  11 | F0            B3  | 16                             MISO
12 (INT6)         12 (TEMP)  12 | E6            B2  | 15                             MOSI
                                | Aref          B1  | 14                             SCLK
                                | Avcc          B0  | 13                             SS
                                | Vbus          3.3V|
                                | D-    _____   Vcc |
                                | D+   |     |  5V  |
                                | Gnd  | USB |  Gnd |
                                 -------------------

* Because of the unusual layout of the ATmega32U4, all pins can be used with analogRead(). 12 of
  these pins are actual analog inputs (1 used by LED), the rest connect to the internal reference,
  internal temperature sensor, or ground.
```

### MT-DB-U1/MT-DB-U2 (AT90USB162/ATmega32U2)

```
=============== MattairTech MT-DB-U1/MT-DB-U2 (AT90USB162/ATmega32U2) ===================
 Comm    Interrupt   PWM   Digital                  Digital  Interrupt    PWM  Comm/other
=========================================================================================
                                 -------------------
SPI SS                       0  | B0            RST |
SPI SCLK                     1  | B1            D7  |  20   20 (INT7)              JUMPER
SPI MOSI                     2  | B2            D6  |  19   19 (INT6)
SPI MISO                     3  | B3            D5  |  18
                             4  | B4            D4  |  17   17 (INT5)
                             5  | B5            D3  |  16   16 (INT3)           USART1 TX
                             6  | B6            D2  |  15   15 (INT2)           USART1 RX
                   7 (TC1C)  7  | B7            D1  |  14   14 (INT1)
         8 (INT4)            8  | C7            D0  |  13   13 (INT0)   13 (TC0B)   LED
                   9 (TC1A)  9  | C6            C2  |  12
                   10 (TC1B) 10 | C5            X1  |
                             11 | C4            X2  |
                                | Vbus          3.3V|
                                | D-    _____   Vcc |
                                | D+   |     |  5V  |
                                | Gnd  | USB |  Gnd |
                                 -------------------
```


### Pin Capabilities

* **Digital: All pins can be used for general purpose I/O** 
* Supports INPUT, OUTPUT, and INPUT_PULLUP.
* Each pin can source or sink a maximum of 20 mA.
* Internal pull-up resistors of 20-50 Kohms (disconnected by default).
* Use the pinMode(), digitalWrite(), and digitalRead() functions.
* **Analog Inputs: 8 pins (MT-DB-U6) or 11 pins (MT-DB-U4) can be configured as ADC analog inputs.**
* These are available using the analogRead() function.
* All pins can be used for GPIO and some pins can be used for other digital functions (ie. pwm or serial).
* Each pin provides 10 bits of resolution (1024 values).
* Each pin measures from ground to 5.0 volts.
* The upper end of the measurement range can be changed using the AREF pin and the analogReference() function.
* **PWM: 7 pins (MT-DB-U6, MT-DB-U4) or 4 pins (MT-DB-U2, MT-DB-U1) can be configured as PWM outputs.**
* Available using the analogWrite() function.
* Each pin provides 8 bits of resolution (256 values) by default.
* **External Interrupts: Up to 8 pins can be configured with external interrupts.**
* 6 pins (MT-DB-U6), 5 pins (MT-DB-U4), or 8 pins (MT-DB-U2, MT-DB-U1).
* Available using the attachInterrupt() function.
* **Serial: 1 pair of pins can be configured for TTL serial I/O.**
* MT-DB-U6: Serial1: pin 31 (RX) and pin 32 (TX).
* MT-DB-U4: Serial1: pin 20 (RX) and pin 21 (TX).
* MT-DB-U2, MT-DB-U1: Serial1: pin 15 (RX) and pin 16 (TX).
* **SPI: 3 or 4 pins can be configured for SPI I/O (SPI).**
* MT-DB-U6: Pin 21 (MOSI), pin 20 (SCK), pin 22 (MISO), and optionally pin 19 (SS, not currently used).
* MT-DB-U4: Pin 15 (MOSI), pin 14 (SCK), pin 16 (MISO), and optionally pin 13 (SS, not currently used).
* MT-DB-U2, MT-DB-U1: Pin 2 (MOSI), pin 1 (SCK), pin 3 (MISO), and optionally pin 0 (SS, not currently used).
* SPI communication using the SPI library.
* **TWI (I2C): 2 pins can be configured for TWI I/O (Wire).**
* MT-DB-U6: Pin 30 (SDA) and pin 29 (SCL).
* MT-DB-U4: Pin 19 (SDA) and pin 18 (SCL).
* MT-DB-U2, MT-DB-U1: TWI not present
* TWI communication using the Wire library.
* **LED: One pin can be configured to light the onboard LED (LED_BUILTIN).**
* Pin 0 (MT-DB-U6), pin 25 (MT-DB-U4), or pin 13 (MT-DB-U2, MT-DB-U1).
* Bring the pin HIGH to turn the LED on.
* **AREF: One pin can be configured as an AREF analog input.**
* The upper end of the analog measurement range can be changed using the analogReference() function.
* **Reset: Bring this line LOW to reset the microcontroller.**


## Using Arduino with MattairTech USB boards

Because of the similarities with the Arduino Leonardo, please read http://arduino.cc/en/Guide/ArduinoLeonardo first.

Within the Arduino IDE Tools menu, select the appropriate MattairTech board, Frequency/Voltage, Processor,
Communications setting, and COM port. There are 2 Frequency/Voltage configurations for each board, 16MHz(5V) and
8MHz(3.3V). You may select 8MHz even if using 5V. When operating at 3.3V, you should select 8MHz. Operating at
16MHz at 3.3V is out of spec, but should work fine at room temperatures. Be sure to select the Communications
setting that matches your sketch (by default, this is CDC_ONLY). This is important.

Note that some example sketches indicate the use of pins using the naming convention D2, D3, etc. These are Arduino
digital pins, not to be confused with port D pins. Most MattairTech USB AVR boards are printed with both port pin
names as well as sequential numbers indicating the Arduino pin number. You may use the 'A' or 'D' prefixes, but they
are simply aliased to the arduino pin number (ie: A2 = D2 = 2).

There are several libraries included with Arduino. Some of these need simple changes to work with MattairTech boards.
Usually, only pin mappings need to be changed.


## Serial Monitor

To print to the Serial Monitor over USB, use 'Serial'. Serial points to SerialUSB (Serial1 is a UART).
Unlike most Arduino boards (ie. Uno), USB AVR based boards do not automatically reset when the serial
monitor is opened. To see what your sketch outputs to the serial monitor from the beginning, the sketch
must wait for the SerialUSB port to open first. Add the following to setup():

```
while (!Serial) ;
```

Remember that if the sketch needs to run without SerialUSB connected, another approach must be used.
You can also reset the board manually with the Reset button if you wish to restart your sketch. However, pressing
the Reset button will reset the AVR chip, which in turn will reset USB communication. This interruption means
that if the serial monitor is open, it will be necessary to close and re-open it to restart communication.


## Updated Tone.cpp

Tone.cpp now supports multiple simultaneous tone generation (one tone per timer).
The MT-DB-U6 currently supports up to 4 simultaneous tones using timers 3, 1, 2,
and 0 if not using the RTC, otherwise, timers 3, 1, and 0 are used for 3 tones.
The MT-DB-U4 currently supports up to 3 simultaneous tones using timers 3, 1, and 0.
A future release may support a fourth tone from timer 4. The MT-DB-U2 and MT-DB-U1
support 2 simultaneous tones using timers 1 and 0. Note that timer 0 has a lower
accuracy for tone generation because it is 8-bit (timers 3 and 1 are 16-bit). Note
also that use of timer 0 temporarily disables the use of delay(), which will return
to normal operation once the tone stops playing. Thus, timer 0 is set with the
lowest priority. For example, if generating DTMF tones on the MT-DB-U4, timers 3
and 1 will be used. However, the MT-DB-U2 and MT-DB-U1 will both use timer 0 for
the second tone. If timer 0 is used, delay() should not be called while timer 0 is
generating a tone. Instead, use _delay_ms(), which is included with avr-libc.

The DTMF_Demo sketch demonstrates usage of Tone.cpp for DTMF generation.


### Detailed Memory Usage Output After Compilation

In this release, two programs are run at the end of compilation to provide more
detailed memory usage. This is enabled only when verbose messages for compilation is
enabled in the IDE Preferences. Just above the normal flash usage message, is the
output from the size utility. Above the size utility output is the output from the
nm utility. The values on the left are in bytes. The letters stand for: T(t)=.text,
D(d)=.data, B(b)=.bss, and everything else (ie: W) resides in flash (in most cases).


## USB Technical Notes

* Note that USB CDC is required for auto-reset into the bootloader to work (otherwise, manually press reset with jumper installed).

ATmegaxxU4: 832 bytes DPRAM, 1 (control, 64 byte max) + 1 (two banks, 256 byte max) + 5 (two banks, 64 byte max) endpoints
AT90USBxxx6/7: 832 bytes DPRAM, 1 (control, 64 byte max) + 1 (two banks, 256 byte max) + 5 (two banks, 64 byte max) endpoints

```
// These are used by the core
#define USB_CONTROL_EP_SIZE		16
#define USB_CONTROL_EP_BANKS		1
#define USB_DEFAULT_EP_SIZE		64
#define USB_DEFAULT_EP_BANKS		2
#define USB_CDC_NOTIFICATION_EP_SIZE	16
#define USB_CDC_NOTIFICATION_EP_BANKS	1
#define USB_CDC_DATA_EP_SIZE		64
#define USB_CDC_DATA_EP_BANKS		2

// These can optionally be used by PluggableUSB libraries
#define USB_HID_EP_SIZE			16 
#define USB_HID_EP_BANKS		1
#define USB_MIDI_EP_SIZE		64
#define USB_MIDI_EP_BANKS		2
#define USB_MSD_EP_SIZE			64
#define USB_MSD_EP_BANKS		2
```

AT90USBxx2: 176 bytes DPRAM, 8 - 64 byte endpoints, 1 (control) + 2 (one bank) + 2 (two banks) endpoints
ATmegaxxU2: 176 bytes DPRAM, 8 - 64 byte endpoints, 1 (control) + 2 (one bank) + 2 (two banks) endpoints

```
// These are used by the core
#define USB_CONTROL_EP_SIZE		16
#define USB_CONTROL_EP_BANKS		1
#define USB_DEFAULT_EP_SIZE		32
#define USB_DEFAULT_EP_BANKS		2
#define USB_CDC_NOTIFICATION_EP_SIZE	16
#define USB_CDC_NOTIFICATION_EP_BANKS	1
#define USB_CDC_DATA_EP_SIZE		32
#define USB_CDC_DATA_EP_BANKS		2

// These can optionally be used by PluggableUSB libraries
#define USB_HID_EP_SIZE			16 
#define USB_HID_EP_BANKS		1
#define USB_MIDI_EP_SIZE		32
#define USB_MIDI_EP_BANKS		2
#define USB_MSD_EP_SIZE			32
#define USB_MSD_EP_BANKS		2
```


## Installation

### Driver Installation

#### Windows

Prior to core version 1.6.9-mt1, sketches compiled with both CDC and HID USB code by default, thus requiring a CDC
driver for the bootloader and a CDC-HID driver for sketches. Now that PluggableUSB is supported, sketches compile
with only CDC code by default. Thus, only one driver is needed. Since HID and MIDI are currently supported (and
MSD potentially in the future), driver installation will be required for each different combination of USB devices.
There are currently four USB composite device combinations that include CDC as well as a CDC only device. Each
supported combination has a unique USB VID:PID pair, and these are listed in the .inf file. Once the first device
is installed (the CDC only device), future installations *might* be automatic, otherwise, you may direct the
installer to the same .inf file. The drivers are signed and support both 32 and 64 bit versions of Windows XP(SP3),
Vista, 7, 8, and 10.

1. If you do not already have the CDC bootloader installed, see below.
2. Download https://www.mattairtech.com/software/MattairTech_CDC_Driver_Signed.zip and unzip into any folder.
3. Plug in the board with the jumper installed. The LED should light.
4. Windows will detect the board. Point the installer to the folder from above to install the bootloader driver.
5. If you don't intend on using Arduino, you can skip the rest of this list. See Using AVRDUDE Standalone below.
6. If you do not already have the test firmware installed, see Using AVRDUDE Standalone below.
7. Press the reset button to run the test firmware (blink sketch).
8. Windows will detect the board. Point the installer to the above folder to install the sketch driver (if needed).
9. Continue with AVR Core Installation below.

#### Linux

0. No driver installation is needed.
1. On some distros, you may need to add your user to the same group as the port (ie: dialout) and/or set udev rules.
2. You MAY have to install and use Arduino as the root user in order to get reliable access to the serial port.
   * This is true even when group permissions are set correctly, and it may fail after previously working.
   * You can also create/modify a udev rule to set permissions on the port so *everyone* can read / write.
3. Continue with AVR Core Installation below.

#### OS X

UNTESTED
0. No driver installation is needed.
1. Plug in the board. You may get a dialog box asking if you wish to open the “Network Preferences”:
   * Click the "Network Preferences..." button, then click "Apply".
   * The board will show up as “Not Configured”, but it will work fine.
2. Continue with AVR Core Installation below.


### AVR Core Installation

* To update from a previous version, click on MattairTech AVR Boards in Boards Manager, then click Update.

1. The MattairTech AVR Core requires Arduino 1.6.7+.
2. In the Arduino IDE 1.6.7+, click File->Preferences.
3. Click the button next to Additional Boards Manager URLs.
4. Add https://www.mattairtech.com/software/arduino/package_MattairTech_index.json.
5. Save preferences, then open the Boards Manager.
6. Install the MattairTech AVR Boards package.
7. Close Boards Manager, then select your board from Tools->Board.
8. Select the Frequency/Voltage with the now visible Tools->Frequency/Voltage menu.
9. Select the processor with the now visible Tools->Processor menu.
9. Select the communications option with the now visible Tools->Communications menu (must match sketch).
10. If you do not already have the bootloader or blink sketch installed, see USB CDC Bootloader below.
11. Plug in the board. The blink sketch should be running.
12. Click Tools->Port and choose the COM port.
13. You can now upload your own sketch.


### Uploading the First Sketch

1. In the Arduino IDE 1.6.7 (or above), open File->Examples->01.Basics->Blink.
2. Change the three instances of '13' to 'LED_BUILTIN'.
3. Be sure the correct options are selected in the Tools menu (see AVR Core Installation above).
4. With the board plugged in, select the correct port from Tools->Port.
5. Click the Upload button. After compiling, the sketch should be transferred to the board.
6. Once the bootloader exits, the blink sketch should be running.


## USB CDC Bootloader (Arduino compatible)

Each board has several bootloaders available. The CDC bootloader can be used with Arduino.
Version 130410 or above is required to support the auto-reset feature. Note that several
boards that were shipped after 130410 but before 130626 still have the old bootloader.

The bootloader enters programming mode only if the jumper is installed, except when using
Arduino auto-reset or when the FLASH is empty. Even with the jumper installed, programming
mode will NOT be entered if the reset was from the watchdog timer, unless the boot key is
enabled and the key matches, as is the case with Arduino auto-reset (the Arduino core
uses a watchdog reset to enter the bootloader).

The default CDC bootloader has the following compile-time options defined:

#define ENABLE_LED_BOOT
#define ENABLE_LED_APPLICATION
#define DISABLE_JTAG_APPLICATION
#define ENABLE_CLKDIV_1_APPLICATION
#define ENABLE_BOOT_KEY
#define ENABLE_RESET_AFTER_PROGRAMMING
#define NO_LOCK_BYTE_WRITE_SUPPORT

An alternate version with the above options undefined is available on the website named
Bootloader_no_options.hex. Use it if the default options interfere with your application.
For example, you may disconnect the LED and use the pin as an analog input.


### Bootloader Firmware Installation Using the Arduino IDE

1. If you do not already have the MattairTech AVR core installed, see AVR Core Installation above.
2. Plug a compatible programmer into a USB port, then connect it to the powered AVR board.
3. Select your programmer from Tools->Programmer.
4. Select your board from Tools->Board.
5. Click Tools->Burn Bootloader. Ignore any messages about not supporting shutdown or reset.
6. Continue with driver installation above.


### Using AVRDUDE Standalone

AVRDUDE can be used standalone. You can use the version included with Arduino
(in arduino-1.6.7/hardware/tools/avr/bin) or download a separate version from
http://download.savannah.gnu.org/releases/avrdude/.

As an example, AVRDUDE will be used to upload the test firmware (blink sketch):

1. Download firmware from https://www.mattairtech.com/software/CDC-bootloader-test-firmware.zip and unzip.
2. If you have not already installed the bootloader driver, see Driver Installation above.
3. Be sure there is a hex file that matches your chip. On the command line (change the hex file to match yours):

```
avrdude -p m32u4 -c avr109 -P usb -U flash:w:"blink.hex"
```
4. On linux, the -P option should be something like /dev/ttyACM0.
5. See http://www.nongnu.org/avrdude/user-manual/avrdude_4.html for details.
6. Press the reset button with the jumper off to load the sketch.
7. When using AVRDUDE standalone, the jumper must be installed before pressing reset to run the bootloader.


## Possible Future Additions

* Features for lower power consumption
* MSC (Mass Storage) USB Device Class
* Host mode CDC
* Better OS X support
* PCINT support


## ChangeLog

* 1.6.9-mt1:
  * See 'What's New' above.

* 1.0.5.1 
  fixes the sketch not running when not connected to a USB host (ie: USB charger).
  Version 1.0.5 fixes several bugs (including BSoD's on Win7-64) and updates the Arduino core files and libraries to 1.0.5.
  Merged in changes to Arduino 1.0.5 core and examples.
  Changed a few //'s to #'s in boards.txt. Fixed blank spaces in board selection list.
  Eliminate descriptor serial numbers.
  Use new PID. Fixed Win7-64 BSoD's and code 10's.
  New inf file to support new PID.
  Initialize USB (HID and CDC) without needing Serial.begin().
  Made USB_WAITFORCONNECT_DISABLED default instead of USB_WAITFORCONNECT_ENABLED.
  Added two nop()'s to USBSerial::readRXEndpoint() so switching USB endpoint banks does not result in returning -1 (empty).
  USBSerial::peek() fixed.
  Wait for USB_DeviceState_Connected state before continuing.
  change keyboardmouse demo to use different pins.

* 1.0.4
  adds HID keyboard and mouse support, adds auto-reset support, updates LUFA to 130303, updates the
  Arduino core files and libraries to 1.0.4, updates the bootloaders, and adds support for the new MT-DB-U6.


## License and credits

This core has been developed by Arduino LLC.
This fork developed by Justin Mattair of MattairTech LLC.

```
Copyright (c) 2015 Arduino LLC.  All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
```

The Bitlash files are Copyright (C) 2008-2012 Bill Roy (bitlash.net)
