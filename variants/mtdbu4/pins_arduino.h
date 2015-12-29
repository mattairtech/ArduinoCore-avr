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
     for MattairTech MT-DB-U4 boards (www.mattairtech.com)
*/

/*
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
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>


// Workaround for wrong definitions in "iom32u4.h".
// This should be fixed in the AVR toolchain.
#undef UHCON
#undef UHINT
#undef UHIEN
#undef UHADDR
#undef UHFNUM
#undef UHFNUML
#undef UHFNUMH
#undef UHFLEN
#undef UPINRQX
#undef UPINTX
#undef UPNUM
#undef UPRST
#undef UPCONX
#undef UPCFG0X
#undef UPCFG1X
#undef UPSTAX
#undef UPCFG2X
#undef UPIENX
#undef UPDATX
#undef TCCR2A
#undef WGM20
#undef WGM21
#undef COM2B0
#undef COM2B1
#undef COM2A0
#undef COM2A1
#undef TCCR2B
#undef CS20
#undef CS21
#undef CS22
#undef WGM22
#undef FOC2B
#undef FOC2A
#undef TCNT2
#undef TCNT2_0
#undef TCNT2_1
#undef TCNT2_2
#undef TCNT2_3
#undef TCNT2_4
#undef TCNT2_5
#undef TCNT2_6
#undef TCNT2_7
#undef OCR2A
#undef OCR2_0
#undef OCR2_1
#undef OCR2_2
#undef OCR2_3
#undef OCR2_4
#undef OCR2_5
#undef OCR2_6
#undef OCR2_7
#undef OCR2B
#undef OCR2_0
#undef OCR2_1
#undef OCR2_2
#undef OCR2_3
#undef OCR2_4
#undef OCR2_5
#undef OCR2_6
#undef OCR2_7

#define NUM_DIGITAL_PINS            26
#define NUM_ANALOG_INPUTS           26

#define LED_BUILTIN	25
#define LED_BUILTIN_RX	25
#define LED_BUILTIN_TX	25

#define TX_RX_LED_INIT          DDRD |= (1<<7)
#define TXLED0                  PORTD |= (1<<7)
#define TXLED1                  PORTD &= ~(1<<7)
#define RXLED0                  PORTD |= (1<<7)
#define RXLED1                  PORTD &= ~(1<<7)

static const uint8_t SDA = 19;
static const uint8_t SCL = 18;

// Map SPI port
static const uint8_t SS   = 13;
static const uint8_t MOSI = 15;
static const uint8_t MISO = 16;
static const uint8_t SCK  = 14;

// Mapping of analog pins as digital I/O
// Note that all pins are supported for digital I/O and analog out
// Note that all pins are supported for analog reading as well,
// however, if no physical ADC input exists on the pin, then either
// a ground measurement is returned, or, in the case of pin 12,
// the internal temperature sensor is read, or, in the case of
// pin 23, the internal reference is read.
static const uint8_t A0 = 0;    // ADC11
static const uint8_t A1 = 1;    // ADC12
static const uint8_t A2 = 2;    // ADC13
static const uint8_t A3 = 3;    // GND
static const uint8_t A4 = 4;    // GND
static const uint8_t A5 = 5;    // GND
static const uint8_t A6 = 6;    // ADC7
static const uint8_t A7 = 7;    // ADC6
static const uint8_t A8 = 8;    // ADC5
static const uint8_t A9 = 9;    // ADC4
static const uint8_t A10 = 10;  // ADC1
static const uint8_t A11 = 11;  // ADC0
static const uint8_t A12 = 12;  // internal temperature
static const uint8_t A13 = 13;  // GND
static const uint8_t A14 = 14;  // GND
static const uint8_t A15 = 15;  // GND
static const uint8_t A16 = 16;  // GND
static const uint8_t A17 = 17;  // GND
static const uint8_t A18 = 18;  // GND
static const uint8_t A19 = 19;  // GND
static const uint8_t A20 = 20;  // GND
static const uint8_t A21 = 21;  // GND
static const uint8_t A22 = 22;  // ADC8
static const uint8_t A23 = 23;  // internal voltage reference
static const uint8_t A24 = 24;  // ADC9
static const uint8_t A25 = 25;  // ADC10 (shared with LED)

// PCINT Pins: 0, 1, 2, 13, 14, 15, 16, 17
#define digitalPinToPCICR(p)    ( (((p) >= 0) && ((p) <= 2)) || (((p) >= 13) && ((p) <= 17)) ? (&PCICR) : ((uint8_t *)0) )
#define digitalPinToPCICRbit(p) 0
#define digitalPinToPCMSK(p)    ( (((p) >= 0) && ((p) <= 2)) || (((p) >= 13) && ((p) <= 17)) ? (&PCMSK0) : ((uint8_t *)0) )
#define digitalPinToPCMSKbit(p) ( (((p) >= 0) && ((p) <= 2)) ? ((p) + 4) : ( ((p) == 17) ? 7 : ( (((p) >= 13) && ((p) <= 16)) ? ((p) - 13) : 0 ) ) )

//	__AVR_ATmega32U4__ has an unusual mapping of pins to channels
extern const uint8_t PROGMEM analog_pin_to_channel_PGM[];
#define analogPinToChannel(P)  ( pgm_read_byte( analog_pin_to_channel_PGM + (P) ) )

#define digitalPinToInterrupt(p) ((p) == 18 ? INT0 : ((p) == 19 ? INT1 : ((p) == 20 ? INT2 : ((p) == 21 ? INT3 : ((p) == 12 ? INT6 : NOT_AN_INTERRUPT)))))
#define analogInputToDigitalPin(p)  (p)

extern const uint8_t PROGMEM interrupt_num_to_INT_PGM[];
#define intterruptNumTohardwareInt(p)  ( pgm_read_byte( interrupt_num_to_INT_PGM + (p) ) )

// Including Timer 0 and Timer 4 (10-bit on the ATmega32U4)
#define digitalPinHasPWM(p)         ((p) == 1 || (p) == 2 || (p) == 3 || (p) == 4 || (p) == 17 || (p) == 18 || (p) == 25)

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
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
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
	INT6,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	INT0,
	INT1,
	INT2,
	INT3,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
	NOT_AN_INTERRUPT,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
        PB, /* 0 */
        PB,
        PB,
        PC,
        PC,
        PE,
        PF, /* 6 */
        PF,
        PF,
        PF,
        PF,
        PF,
        PE,
        PB, /* 13 */
        PB,
        PB,
        PB,
        PB,
        PD, /* 18 */
        PD,
        PD,
        PD,
        PD,
        PD,
        PD,
        PD, /* 25, LED */
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
        _BV(4), /* 0, port B */
        _BV(5),
        _BV(6),
        _BV(6), /* 3, port C */
        _BV(7),
        _BV(2), /* 5, port E */
        _BV(7), /* 6, port F */
        _BV(6),
        _BV(5),
        _BV(4),
        _BV(1), /* 10, port F */
        _BV(0),
        _BV(6), /* 12, port E */
        _BV(0), /* 13, port B */
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(7), /* 17, port B */
        _BV(0), /* 18, port D */
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(4),
        _BV(5),
        _BV(6),
        _BV(7), /* 25, port D, LED */
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
        NOT_ON_TIMER,
        TIMER1A,
        TIMER1B,
        TIMER3A,
        TIMER4A,
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
        TIMER1C,
        TIMER0B,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        TIMER4D, /* 25, port D, LED */
};

// These are the actual MUX bits from the datasheet
#define ADC0 0
#define ADC1 1
#define ADC4 4
#define ADC5 5
#define ADC6 6
#define ADC7 7
#define REFERENCE 30
#define GROUND 31
#define ADC8 32
#define ADC9 33
#define ADC10 34
#define ADC11 35
#define ADC12 36
#define ADC13 37
#define TEMPERATURE 39

// Returns the MUX bits for the MT-DB-U4 ADC, not the channel
const uint8_t PROGMEM analog_pin_to_channel_PGM[] = {
	ADC11,
        ADC12,
        ADC13,
        GROUND,
        GROUND,
        GROUND,
        ADC7,
        ADC6,
        ADC5,
        ADC4,
        ADC1,
        ADC0,
        TEMPERATURE,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        GROUND,
        ADC8,
        REFERENCE,
        ADC9,
        ADC10,
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
