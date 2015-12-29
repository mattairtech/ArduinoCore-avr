CDC bootloaders for MattairTech MT-DB-Ux USB boards

Modify MCU and FLASH_SIZE_KB in the makefile to match your board.

CHANGELOG
=====================================
Version 140502:

  Added #define ARDUINO_MODE to AppConfig.h. This eliminates the requirement for the jumper
  to be installed. Arduino users should now always leave the jumper off. If you cannot enter
  the bootloader (ie: sketch compiled with USB_AUTORESET_DISABLED), you can force the
  bootloader by installing the jumper (be sure to re-select the COM port).
  
  Removed #define ENABLE_CLKDIV_1_APPLICATION from AppConfig.h. Now, the bootloader always
  runs at the crystal speed (16MHz). See next entry.
  
  Fixed problem on Linux systems where the LED would sometimes freeze and the USB connection
  would fail. This was due to the bootloader running at 8MHz. Now it always runs at 16MHz.
  Note that when operating at 3.3V, the cpu will be overclocked, but it should work fine.
  
  Fixed problem where AVRDUDE would sometimes freeze at the end of programming. This was due
  to the bootloader exiting before the last ACK was sent to AVRDUDE. This may have affected
  other host software as well.
  
  Increased the time between disconnecting from the USB host and switching to the application.
  
  Updated LUFA library to version 140302.


CDC Bootloader
=====================================
Each board has several bootloaders available. The CDC bootloader can be used with Arduino.
Version 130410 or above is required to support the auto-reset feature. Note that several
boards that were shipped after 130410 but before 130626 still have the old bootloader.
It is strongly recommended to use version 140502 or higher when using with Arduino.

The default CDC bootloader has the following compile-time options defined:

        #define NO_LOCK_BYTE_WRITE_SUPPORT
        #define ENABLE_LED_BOOT
        #define ENABLE_LED_APPLICATION
        #define DISABLE_JTAG_APPLICATION
        #define ENABLE_BOOT_KEY
        #define ENABLE_RESET_AFTER_PROGRAMMING
        #define ARDUINO_MODE

An alternate version with the above options undefined is available on the website named
Bootloader_no_options.hex. Use it if the default options interfere with your application.
For example, you may disconnect the LED and use the pin as an analog input.


License
=====================================
/*
 *           LUFA Library
 *   Copyright (C) Dean Camera, 2014.
 * 
 * dean [at] fourwalledcubicle [dot] com
 *         www.lufa-lib.org
 */

/*
 * Copyright 2014  Dean Camera (dean [at] fourwalledcubicle [dot] com)
 * 
 * Permission to use, copy, modify, distribute, and sell this
 * software and its documentation for any purpose is hereby granted
 * without fee, provided that the above copyright notice appear in
 * all copies and that both that the copyright notice and this
 * permission notice and warranty disclaimer appear in supporting
 * documentation, and that the name of the author not be used in
 * advertising or publicity pertaining to distribution of the
 * software without specific, written prior permission.
 * 
 * The author disclaims all warranties with regard to this
 * software, including all implied warranties of merchantability
 * and fitness.  In no event shall the author be liable for any
 * special, indirect or consequential damages or any damages
 * whatsoever resulting from loss of use, data or profits, whether
 * in an action of contract, negligence or other tortious action,
 * arising out of or in connection with the use or performance of
 * this software.
 */

/*
 * Modified 2 May 2014 by Justin Mattair
 *   for MattairTech MT-DB-Ux boards (www.mattairtech.com)
 */
