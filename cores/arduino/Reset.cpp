/*
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
*/

#include <Arduino.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include "Reset.h"

#ifdef __cplusplus
extern "C" {
#endif

void Reboot(void)
{
        /* Disconnect from the host - USB interface will be reset later along with the AVR */
	UDCON  |=  (1 << DETACH);
	
	// Disable all interrupts
        cli();
        
        wdt_enable(WDTO_60MS);
        
	*(uint16_t *)MAGIC_KEY_POS = MAGIC_KEY;

        while(1);
}

static int ticks = -1;

void initiateReset(int _ticks) {
	ticks = _ticks;
}

void cancelReset() {
	ticks = -1;
}

void tickReset() {
	if (ticks == -1)
		return;
	ticks--;
	if (ticks == 0)
		Reboot();
}

#ifdef __cplusplus
}
#endif
