/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/


/*
  Modified 28 September 2015 by Justin Mattair
     for MattairTech MT-DB-U6 boards (www.mattairtech.com)
*/

/*
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
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>


// #define USE_RTC

#define NUM_DIGITAL_PINS            48
#define NUM_ANALOG_INPUTS           48

#define LED_BUILTIN	0
#define LED_BUILTIN_RX	0
#define LED_BUILTIN_TX	0

#define TX_RX_LED_INIT          DDRE |= (1<<0)
#define TXLED0                  PORTE |= (1<<0)
#define TXLED1                  PORTE &= ~(1<<0)
#define RXLED0                  PORTE |= (1<<0)
#define RXLED1                  PORTE &= ~(1<<0)

static const uint8_t SDA = 30;
static const uint8_t SCL = 29;

// Map SPI port
static const uint8_t SS   = 19;
static const uint8_t MOSI = 21;
static const uint8_t MISO = 22;
static const uint8_t SCK  = 20;

// Mapping of analog pins as digital I/O
// Note that all pins are supported for digital I/O and analog out
// Note that all pins are supported for analog reading as well,
// however, if no physical ADC input exists on the pin, then either
// a ground measurement is returned, or, in the case of
// pin 47, the internal reference is read.
static const uint8_t A0 = 0;    // GND (shared with LED)
static const uint8_t A1 = 1;    // GND
static const uint8_t A2 = 2;    // GND
static const uint8_t A3 = 3;    // GND
static const uint8_t A4 = 4;    // GND
static const uint8_t A5 = 5;    // GND
static const uint8_t A6 = 6;    // GND
static const uint8_t A7 = 7;    // GND
static const uint8_t A8 = 8;    // GND
static const uint8_t A9 = 9;    // GND
static const uint8_t A10 = 10;  // ADC7
static const uint8_t A11 = 11;  // ADC6
static const uint8_t A12 = 12;  // ADC5
static const uint8_t A13 = 13;  // ADC4
static const uint8_t A14 = 14;  // ADC3
static const uint8_t A15 = 15;  // ADC2
static const uint8_t A16 = 16;  // ADC1
static const uint8_t A17 = 17;  // ADC0
static const uint8_t A18 = 18;  // GND (optional power supply status/control line, pin E6)
static const uint8_t A19 = 19;  // GND
static const uint8_t A20 = 20;  // GND
static const uint8_t A21 = 21;  // GND
static const uint8_t A22 = 22;  // GND
static const uint8_t A23 = 23;  // GND
static const uint8_t A24 = 24;  // GND
static const uint8_t A25 = 25;  // GND
static const uint8_t A26 = 26;  // GND
static const uint8_t A27 = 27;  // GND
static const uint8_t A28 = 28;  // GND
static const uint8_t A29 = 29;  // GND
static const uint8_t A30 = 30;  // GND
static const uint8_t A31 = 31;  // GND
static const uint8_t A32 = 32;  // GND
static const uint8_t A33 = 33;  // GND
static const uint8_t A34 = 34;  // GND
static const uint8_t A35 = 35;  // GND
static const uint8_t A36 = 36;  // GND
static const uint8_t A37 = 37;  // GND (shared with Jumper)
static const uint8_t A38 = 38;  // GND
static const uint8_t A39 = 39;  // GND
static const uint8_t A40 = 40;  // GND
static const uint8_t A41 = 41;  // GND
static const uint8_t A42 = 42;  // GND
static const uint8_t A43 = 43;  // GND
static const uint8_t A44 = 44;  // GND
static const uint8_t A45 = 45;  // GND
static const uint8_t A46 = 46;  // GND (RTC crystal, pin E4)
static const uint8_t A47 = 47;  // internal voltage reference (RTC crystal, pin E5)

// PCINT 0-7 are on pins 19-26
#define digitalPinToPCICR(p)    ( (((p) >= 19) && ((p) <= 26)) ? (&PCICR) : ((uint8_t *)0) )
#define digitalPinToPCICRbit(p) 0
#define digitalPinToPCMSK(p)    ( (((p) >= 19) && ((p) <= 26)) ? (&PCMSK0) : ((uint8_t *)0) )
#define digitalPinToPCMSKbit(p) ( (((p) >= 19) && ((p) <= 26)) ? ((p) - 19) : 0 )

extern const uint8_t PROGMEM analog_pin_to_channel_PGM[];
#define analogPinToChannel(P)  ( pgm_read_byte( analog_pin_to_channel_PGM + (P) ) )

#define digitalPinToInterrupt(p) ((p) == 29 ? INT0 : ((p) == 30 ? INT1 : ((p) == 31 ? INT2 : ((p) == 32 ? INT3 : ((p) == 18 ? INT6 : ((p) == 27 ? INT7 : NOT_AN_INTERRUPT))))))
#define analogInputToDigitalPin(p)  (p)

extern const uint8_t PROGMEM interrupt_num_to_INT_PGM[];
#define intterruptNumTohardwareInt(p)  ( pgm_read_byte( interrupt_num_to_INT_PGM + (p) ) )

// Including Timer 0
#if defined(USE_RTC)
#define digitalPinHasPWM(p)         ((p) == 6 || (p) == 7 || (p) == 8 || (p) == 24 || (p) == 25 || (p) == 26 || (p) == 29)
#else
#define digitalPinHasPWM(p)         ((p) == 6 || (p) == 7 || (p) == 8 || (p) == 23 || (p) == 24 || (p) == 25 || (p) == 26 || (p) == 29 || (p) == 30)
#endif


#ifdef ARDUINO_MAIN

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
        (uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
};

// Interrupt number (ie: as used in attachInterrupt()) to hardware interrupt (ie: INT0) mapping.
// MattairTech boards maps these one to one. If PCINT support is added later, this may become a pin mapping array (like those below).
const uint8_t PROGMEM interrupt_num_to_INT_PGM[] = {
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	INT6,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	INT7,
	NOT_AN_INTERRUPT,
	INT0,
	INT1,
	INT2,
	INT3,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
        PE, /* 0, LED */
        PE,
        PC, /* 2 */
        PC,
        PC,
        PC,
        PC,
        PC,
        PC,
        PC,
        PF, /* 10 */
        PF,
        PF,
        PF,
        PF,
        PF,
        PF,
        PF,
        PE, /* 18 */
        PB, /* 19 */
        PB,
        PB,
        PB,
        PB,
        PB,
        PB,
        PB,
        PE, /* 27 */
        PE,
        PD, /* 29 */
        PD,
        PD,
        PD,
        PD,
        PD,
        PD,
        PD,
        PE, /* 37, Jumper */
        PA, /* 38 */
        PA,
        PA,
        PA,
        PA,
        PA,
        PA,
        PA,
        PE, /* 46, RTC crystal */
        PE, /* 47, RTC crystal */
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
        _BV(0),
        _BV(1),
        _BV(0),
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(4),
        _BV(5),
        _BV(6),
        _BV(7),
        _BV(7),
        _BV(6),
        _BV(5),
        _BV(4),
        _BV(3),
        _BV(2),
        _BV(1),
        _BV(0),
        _BV(6),
        _BV(0),
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(4),
        _BV(5),
        _BV(6),
        _BV(7),
        _BV(7),
        _BV(3),
        _BV(0),
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(4),
        _BV(5),
        _BV(6),
        _BV(7),
        _BV(2),
        _BV(0),
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(4),
        _BV(5),
        _BV(6),
        _BV(7),
        _BV(4),
        _BV(5),
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        TIMER3C,
        TIMER3B,
        TIMER3A,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        TIMER2A,
        TIMER1A,
        TIMER1B,
        TIMER1C,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        TIMER0B,
        TIMER2B,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
};

// These are the actual MUX bits from the datasheet
#define ADC0 0
#define ADC1 1
#define ADC2 2
#define ADC3 3
#define ADC4 4
#define ADC5 5
#define ADC6 6
#define ADC7 7
#define REFERENCE 30
#define GROUND 31

// Returns the MUX bits for the MT-DB-U6 ADC, not the channel
const uint8_t PROGMEM analog_pin_to_channel_PGM[] = {
	GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        ADC7,
        ADC6,
        ADC5,
        ADC4,
        ADC3,
        ADC2,
        ADC1,
        ADC0,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        REFERENCE,
};

#endif /* ARDUINO_MAIN */

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE1       Serial2
#define SERIAL_PORT_HARDWARE_OPEN   Serial1
#define SERIAL_PORT_HARDWARE_OPEN1  Serial2

// Alias SerialUSB to Serial
#define SerialUSB SERIAL_PORT_USBVIRTUAL

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


#endif /* Pins_Arduino_h */
