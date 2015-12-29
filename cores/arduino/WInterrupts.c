/* -*- mode: jde; c-basic-offset: 2; indent-tabs-mode: nil -*- */

/*
  Part of the Wiring project - http://wiring.uniandes.edu.co

  Copyright (c) 2004-05 Hernando Barragan

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
  
  Modified 24 November 2006 by David A. Mellis
  Modified 1 August 2010 by Mark Sproul
*/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>

#include "wiring_private.h"

static void nothing(void) {
}

static volatile voidFuncPtr intFunc[MAX_HARDWARE_INT_NUM] = {
#if MAX_HARDWARE_INT_NUM > 8
    #warning There are more than 8 external interrupts. Some callbacks may not be initialized.
    nothing,
#endif
#if MAX_HARDWARE_INT_NUM > 7
    nothing,
#endif
#if MAX_HARDWARE_INT_NUM > 6
    nothing,
#endif
#if MAX_HARDWARE_INT_NUM > 5
    nothing,
#endif
#if MAX_HARDWARE_INT_NUM > 4
    nothing,
#endif
#if MAX_HARDWARE_INT_NUM > 3
    nothing,
#endif
#if MAX_HARDWARE_INT_NUM > 2
    nothing,
#endif
#if MAX_HARDWARE_INT_NUM > 1
    nothing,
#endif
#if MAX_HARDWARE_INT_NUM > 0
    nothing,
#endif
};

// volatile static voidFuncPtr twiIntFunc;

void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), int mode) {
  // Remap interruptNum to hardware interrupt
  uint8_t hardwareInt = intterruptNumTohardwareInt(interruptNum);
  
  if(hardwareInt != NOT_AN_INTERRUPT) {
    intFunc[hardwareInt] = userFunc;
    
    // Configure the interrupt mode (trigger on low input, any change, rising
    // edge, or falling edge).  The mode constants were chosen to correspond
    // to the configuration bits in the hardware register, so we simply shift
    // the mode into place.
      
    // Enable the interrupt.
      
    switch (hardwareInt) {
#if defined(EICRA) && defined(EICRB) && defined(EIMSK)
    case INT0:
      EICRA = (EICRA & ~((1 << ISC00) | (1 << ISC01))) | (mode << ISC00);
      EIMSK |= (1 << INT0);
      break;
    case INT1:
      EICRA = (EICRA & ~((1 << ISC10) | (1 << ISC11))) | (mode << ISC10);
      EIMSK |= (1 << INT1);
      break;
    case INT2:
      EICRA = (EICRA & ~((1 << ISC20) | (1 << ISC21))) | (mode << ISC20);
      EIMSK |= (1 << INT2);
      break;
    case INT3:
      EICRA = (EICRA & ~((1 << ISC30) | (1 << ISC31))) | (mode << ISC30);
      EIMSK |= (1 << INT3);
      break;
    case INT4:
      EICRB = (EICRB & ~((1 << ISC40) | (1 << ISC41))) | (mode << ISC40);
      EIMSK |= (1 << INT4);
      break;
    case INT5:
      EICRB = (EICRB & ~((1 << ISC50) | (1 << ISC51))) | (mode << ISC50);
      EIMSK |= (1 << INT5);
      break;
    case INT6:
      EICRB = (EICRB & ~((1 << ISC60) | (1 << ISC61))) | (mode << ISC60);
      EIMSK |= (1 << INT6);
      break;
    case INT7:
      EICRB = (EICRB & ~((1 << ISC70) | (1 << ISC71))) | (mode << ISC70);
      EIMSK |= (1 << INT7);
      break;
#else		
    case INT0:
    #if defined(EICRA) && defined(ISC00) && defined(EIMSK)
      EICRA = (EICRA & ~((1 << ISC00) | (1 << ISC01))) | (mode << ISC00);
      EIMSK |= (1 << INT0);
    #elif defined(MCUCR) && defined(ISC00) && defined(GICR)
      MCUCR = (MCUCR & ~((1 << ISC00) | (1 << ISC01))) | (mode << ISC00);
      GICR |= (1 << INT0);
    #elif defined(MCUCR) && defined(ISC00) && defined(GIMSK)
      MCUCR = (MCUCR & ~((1 << ISC00) | (1 << ISC01))) | (mode << ISC00);
      GIMSK |= (1 << INT0);
    #else
      #error attachInterrupt not finished for this CPU (case 0)
    #endif
      break;

    case INT1:
    #if defined(EICRA) && defined(ISC10) && defined(ISC11) && defined(EIMSK)
      EICRA = (EICRA & ~((1 << ISC10) | (1 << ISC11))) | (mode << ISC10);
      EIMSK |= (1 << INT1);
    #elif defined(MCUCR) && defined(ISC10) && defined(ISC11) && defined(GICR)
      MCUCR = (MCUCR & ~((1 << ISC10) | (1 << ISC11))) | (mode << ISC10);
      GICR |= (1 << INT1);
    #elif defined(MCUCR) && defined(ISC10) && defined(GIMSK) && defined(GIMSK)
      MCUCR = (MCUCR & ~((1 << ISC10) | (1 << ISC11))) | (mode << ISC10);
      GIMSK |= (1 << INT1);
    #else
      #warning attachInterrupt may need some more work for this cpu (case 1)
    #endif
      break;
    
    case INT2:
    #if defined(EICRA) && defined(ISC20) && defined(ISC21) && defined(EIMSK)
      EICRA = (EICRA & ~((1 << ISC20) | (1 << ISC21))) | (mode << ISC20);
      EIMSK |= (1 << INT2);
    #elif defined(MCUCR) && defined(ISC20) && defined(ISC21) && defined(GICR)
      MCUCR = (MCUCR & ~((1 << ISC20) | (1 << ISC21))) | (mode << ISC20);
      GICR |= (1 << INT2);
    #elif defined(MCUCR) && defined(ISC20) && defined(GIMSK) && defined(GIMSK)
      MCUCR = (MCUCR & ~((1 << ISC20) | (1 << ISC21))) | (mode << ISC20);
      GIMSK |= (1 << INT2);
    #endif
      break;
#endif
    }
  }
}

void detachInterrupt(uint8_t interruptNum) {
  // Remap interruptNum to hardware interrupt
  uint8_t hardwareInt = intterruptNumTohardwareInt(interruptNum);
  
  if(hardwareInt != NOT_AN_INTERRUPT) {
    // Disable the interrupt.  (We can't assume that interruptNum is equal
    // to the number of the EIMSK bit to clear, as this isn't true on the 
    // ATmega8.  There, INT0 is 6 and INT1 is 7.)
#if defined(EICRA) && defined(EICRB) && defined(EIMSK)
    EIMSK &= ~(1 << hardwareInt);
#else
    #if defined(EIMSK)
    EIMSK &= ~(1 << hardwareInt);
    #elif defined(GICR)
    GICR &= ~(1 << hardwareInt); // atmega32
    #elif defined(GIMSK)
    GIMSK &= ~(1 << hardwareInt);
    #else
    #error detachInterrupt not finished for this cpu
    #endif
#endif
      
    intFunc[hardwareInt] = nothing;
  }
}

/*
void attachInterruptTwi(void (*userFunc)(void) ) {
  twiIntFunc = userFunc;
}
*/

ISR(INT0_vect) {
    intFunc[INT0]();
}

ISR(INT1_vect) {
    intFunc[INT1]();
}

#if defined(INT2_vect)
ISR(INT2_vect) {
    intFunc[INT2]();
}
#endif

#if defined(INT3_vect)
ISR(INT3_vect) {
    intFunc[INT3]();
}
#endif

#if defined(INT4_vect)
ISR(INT4_vect) {
    intFunc[INT4]();
}
#endif

#if defined(INT5_vect)
ISR(INT5_vect) {
    intFunc[INT5]();
}
#endif

#if defined(INT6_vect)
ISR(INT6_vect) {
    intFunc[INT6]();
}
#endif

#if defined(INT7_vect)
ISR(INT7_vect) {
    intFunc[INT7]();
}
#endif

/*
ISR(TWI_vect) {
  if(twiIntFunc)
    twiIntFunc();
}
*/

