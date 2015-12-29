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
     for MattairTech MT-DB-U2 boards (www.mattairtech.com)
*/

/*
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
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>


#define NUM_DIGITAL_PINS            21
#define NUM_ANALOG_INPUTS           0

#define LED_BUILTIN	13
#define LED_BUILTIN_RX	13
#define LED_BUILTIN_TX	13

#define TX_RX_LED_INIT		DDRD |= (1<<0)
#define TXLED0			PORTD |= (1<<0)
#define TXLED1			PORTD &= ~(1<<0)
#define RXLED0			PORTD |= (1<<0)
#define RXLED1			PORTD &= ~(1<<0)

// Map SPI port
static const uint8_t SS   = 0;
static const uint8_t MOSI = 2;
static const uint8_t MISO = 3;
static const uint8_t SCK  = 1;

// PCINT Pins: 0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 18
#define digitalPinToPCICR(p)    ( (((p) >= 0) && ((p) <= 7)) || ((p) == 18) || (((p) >= 9) && ((p) <= 12)) ? (&PCICR) : ((uint8_t *)0) )
#define digitalPinToPCICRbit(p) ( (((p) >= 9) && ((p) <= 12)) || ((p) == 18) ? 1 : 0 )
#define digitalPinToPCMSK(p)    ( (((p) >= 0) && ((p) <= 7)) ? (&PCMSK0) : ( (((p) >= 9) && ((p) <= 12)) || ((p) == 18) ? (&PCMSK1) : ((uint8_t *)0) ) )
#define digitalPinToPCMSKbit(p) ( (((p) >= 0) && ((p) <= 7)) ? (p) : ( ((p) == 18) ? 4 : ( (((p) >= 9) && ((p) <= 12)) ? ((p) - 9) : 0 ) ) )

#define digitalPinToInterrupt(p) ((p) == 13 ? INT0 : ((p) == 14 ? INT1 : ((p) == 15 ? INT2 : ((p) == 16 ? INT3 : ((p) == 8 ? INT4 : ((p) == 17 ? INT5 : ((p) == 19 ? INT6 : ((p) == 20 ? INT7 : NOT_AN_INTERRUPT))))))))

extern const uint8_t PROGMEM interrupt_num_to_INT_PGM[];
#define intterruptNumTohardwareInt(p)  ( pgm_read_byte( interrupt_num_to_INT_PGM + (p) ) )

// Including Timer 0
#define digitalPinHasPWM(p)         ((p) == 7 || (p) == 9 || (p) == 10 || (p) == 13)

#ifdef ARDUINO_MAIN

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
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
	INT4,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	INT0,
	INT1,
	INT2,
	INT3,
	INT5,
	NOT_AN_INTERRUPT,
	INT6,
	INT7,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
        PB, /* 0 */
        PB,
        PB,
        PB,
        PB,
        PB,
        PB,
        PB,
        PC, /* 8 */
        PC,
        PC,
        PC,
        PC,
        PD, /* 13 */
        PD,
        PD,
        PD,
        PD,
        PD,
        PD,
        PD,
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
        _BV(0), /* 0, port B */
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(4),
        _BV(5),
        _BV(6),
        _BV(7),
        _BV(7), /* 8, port C */
        _BV(6),
        _BV(5),
        _BV(4),
        _BV(2),
        _BV(0), /* 13, port D */
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(4),
        _BV(5),
        _BV(6),
        _BV(7),
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
        NOT_ON_TIMER, /* 0 - port B */
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        TIMER1C,
        NOT_ON_TIMER, /* 8 - port C */
        TIMER1A,
        TIMER1B,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        TIMER0B, /* 13 - port D */
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
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
#define SERIAL_PORT_MONITOR        Serial
#define SERIAL_PORT_USBVIRTUAL     Serial
#define SERIAL_PORT_HARDWARE       Serial1
#define SERIAL_PORT_HARDWARE_OPEN  Serial1

// Alias SerialUSB to Serial
#define SerialUSB SERIAL_PORT_USBVIRTUAL

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


#endif /* Pins_Arduino_h */
