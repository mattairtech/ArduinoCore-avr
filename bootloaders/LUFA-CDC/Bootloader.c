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

/** \file
 *
 *  Main source file for the CDC class bootloader. This file contains the complete bootloader logic.
 */

#define  INCLUDE_FROM_BOOTLOADER_C
#include "Bootloader.h"

/** Contains the current baud rate and other settings of the first virtual serial port. This must be retained as some
 *  operating systems will not open the port unless the settings can be set successfully.
 */
static CDC_LineEncoding_t LineEncoding = { .BaudRateBPS = 0,
                                           .CharFormat  = CDC_LINEENCODING_OneStopBit,
                                           .ParityType  = CDC_PARITY_None,
                                           .DataBits    = 8                            };

/** Current address counter. This stores the current address of the FLASH or EEPROM as set by the host,
 *  and is used when reading or writing to the AVRs memory (either FLASH or EEPROM depending on the issued
 *  command.)
 */
static uint32_t CurrAddress;

/** Flag to indicate if the bootloader should be running, or should exit and allow the application code to run
 *  via a watchdog reset. When cleared the bootloader will exit, starting the watchdog and entering an infinite
 *  loop until the AVR restarts and the application runs.
 */
static bool RunBootloader = true;

static AppPtr_t AppStartPtr = (AppPtr_t)0x0000;
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));

bool increasingLevelPWM;
uint8_t compareValue;

/* This timeout is for running the user application automatically after programming.
 * When the timeout period elapses, the bootloader exits and the user application is run (after a watchdog reset).
 * It is different than the Arduino or LUFA versions.
 * The timeout count doesn't start until after a write FLASH block command is completed.
 * The timeout count is reset each time a command is received, thus allowing a FLASH verification.
 * The MT-DB-xx boards do not have a bootloader delay. They have a jumper to select boot mode.
 */
// 488 = 1s
#define TIMEOUT_PERIOD	488
bool TimeoutEnabled = false;
uint16_t Timeout = 0;

uint16_t bootKey = 0x7777;
volatile uint16_t *const bootKeyPtr = (volatile uint16_t *)0x0280;


/** Main program entry point. This routine configures the hardware required by the bootloader, then continuously
 *  runs the bootloader processing routine until instructed to soft-exit, or hard-reset via the watchdog to start
 *  the loaded application code.
 */
int main(void)
{
	/* Save the value of the boot key memory before it is overwritten */
	uint16_t bootKeyPtrVal = *bootKeyPtr;
	*bootKeyPtr = 0;
	
	/* Setup hardware required for the bootloader */
	SetupHardware();
	
	/* programming mode */
#if defined(ENABLE_BOOT_KEY)
	#if defined(ARDUINO_MODE)
	if ((Jumper_GetStatus() && (!((mcusr_mirror & (1 << WDRF)) && (bootKeyPtrVal != bootKey)))) || ((mcusr_mirror & (1 << WDRF)) && (bootKeyPtrVal == bootKey)) || (pgm_read_word(0) == 0xFFFF)) {
	#else
	if ((Jumper_GetStatus() && (!((mcusr_mirror & (1 << WDRF)) && (bootKeyPtrVal != bootKey)))) || (pgm_read_word(0) == 0xFFFF)) {
	#endif
#else
	if ((Jumper_GetStatus() && (!(mcusr_mirror & (1 << WDRF)))) || (pgm_read_word(0) == 0xFFFF)) {
#endif
		/* Relocate the interrupt vector table to the bootloader section */
		MCUCR = (1 << IVCE);
		MCUCR = (1 << IVSEL);
		
		/* Initialize LEDs */
		#if defined(ENABLE_LED_BOOT)
		LED_Init();	// default off
		LED_On();
		#endif
		
		/* Initialize the USB subsystem */
		USB_Init();
		
		/* Enable global interrupts so that the USB stack can function */
		sei();
		
		/* Setup timer 0 for PWM LED and timeouts */
		SetupTimer();
		
		/* Run the USB management task while the bootloader is supposed to be running */
		while (RunBootloader) {
			CDC_Task();
			USB_USBTask();
			
			/* Time out and start the application if one is present */
			if (Timeout > TIMEOUT_PERIOD)
				RunBootloader = false;
			
			LED_Task();
		}
		
		/* Disconnect from the host - USB interface will be reset later along with the AVR */
		USB_Detach();
		
		wdt_enable(WDTO_500MS);
		
		while(1);
	}

#if defined(DISABLE_JTAG_APPLICATION)
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) || defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB647__) || defined(__AVR_AT90USB1287__)
	/* Disable JTAG */
	JTAG_DISABLE();
#endif
#endif

	/* Initialize LEDs */
#if defined(ENABLE_LED_APPLICATION)
	LED_Init();	// default off
	LED_On();
#endif
	
	/* Start the user application */
	AppStartPtr();
}

/** Configures all hardware required for the bootloader. */
void SetupHardware(void)
{
	/* Save mcusr which contains reset source */
	mcusr_mirror = MCUSR;
	
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
	
	clock_prescale_set(clock_div_1);
	
	/* Initialize Jumper */
	Jumper_Init();
	_delay_ms(6);	// allow time for caps to charge
}

/* Setup timer 0 for software PWM LED and timeouts */
static void SetupTimer(void)
{
	compareValue = 0xFF;
	increasingLevelPWM = false;
	TCCR0B = ((0 << CS02) | (1 << CS01) | (1 << CS00));	// prescale 64
}

static void LED_Task(void)
{
	if ((TIFR0 & (1 << OCF0B)) != 0) {
		TIFR0 |= (1 << OCF0B);
		if (increasingLevelPWM) {
			if (compareValue == 0xFF) {
				increasingLevelPWM = false;
				compareValue-=1;
			} else {
				compareValue+=1;
			}
		} else {
			if (compareValue == 0x00) {
				increasingLevelPWM = true;
				compareValue+=1;
			} else {
				compareValue-=1;
			}
		}
		
		if (TimeoutEnabled) {
			Timeout++;
		}
	}
	
#if defined(ENABLE_LED_BOOT)
	if (TCNT0 < compareValue) {
		LED_On();
	} else {
		LED_Off();
	}
#endif
}

/** Event handler for the USB_ConfigurationChanged event. This configures the device's endpoints ready
 *  to relay data to and from the attached USB host.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	/* Setup CDC Notification, Rx and Tx Endpoints */
	Endpoint_ConfigureEndpoint(CDC_NOTIFICATION_EPADDR, EP_TYPE_INTERRUPT,
				   CDC_NOTIFICATION_EPSIZE, 1);

	Endpoint_ConfigureEndpoint(CDC_TX_EPADDR, EP_TYPE_BULK, CDC_TXRX_EPSIZE, 1);

	Endpoint_ConfigureEndpoint(CDC_RX_EPADDR, EP_TYPE_BULK, CDC_TXRX_EPSIZE, 1);
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
	/* Ignore any requests that aren't directed to the CDC interface */
	if ((USB_ControlRequest.bmRequestType & (CONTROL_REQTYPE_TYPE | CONTROL_REQTYPE_RECIPIENT)) !=
	    (REQTYPE_CLASS | REQREC_INTERFACE))
	{
		return;
	}

	/* Process CDC specific control requests */
	switch (USB_ControlRequest.bRequest)
	{
		case CDC_REQ_GetLineEncoding:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Write the line coding data to the control endpoint */
				Endpoint_Write_Control_Stream_LE(&LineEncoding, sizeof(CDC_LineEncoding_t));
				Endpoint_ClearOUT();
			}

			break;
		case CDC_REQ_SetLineEncoding:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Read the line coding data in from the host into the global struct */
				Endpoint_Read_Control_Stream_LE(&LineEncoding, sizeof(CDC_LineEncoding_t));
				Endpoint_ClearIN();
			}

			break;
			
		case CDC_REQ_SetControlLineState:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();
			}
			
			break;
	}
}

#if !defined(NO_BLOCK_SUPPORT)
/** Reads or writes a block of EEPROM or FLASH memory to or from the appropriate CDC data endpoint, depending
 *  on the AVR910 protocol command issued.
 *
 *  \param[in] Command  Single character AVR910 protocol command indicating what memory operation to perform
 */
static void ReadWriteMemoryBlock(const uint8_t Command)
{
	uint16_t BlockSize;
	char     MemoryType;

	uint8_t  HighByte = 0;
	uint8_t  LowByte  = 0;

	BlockSize  = (FetchNextCommandByte() << 8);
	BlockSize |=  FetchNextCommandByte();

	MemoryType =  FetchNextCommandByte();

	if ((MemoryType != MEMORY_TYPE_FLASH) && (MemoryType != MEMORY_TYPE_EEPROM))
	{
		/* Send error byte back to the host */
		WriteNextResponseByte('?');

		return;
	}

	/* Check if command is to read memory */
	if (Command == AVR109_COMMAND_BlockRead)
	{
		/* Re-enable RWW section */
		boot_rww_enable();

		while (BlockSize--)
		{
			if (MemoryType == MEMORY_TYPE_FLASH)
			{
				/* Read the next FLASH byte from the current FLASH page */
				#if (FLASHEND > 0xFFFF)
				WriteNextResponseByte(pgm_read_byte_far(CurrAddress | HighByte));
				#else
				WriteNextResponseByte(pgm_read_byte(CurrAddress | HighByte));
				#endif

				/* If both bytes in current word have been read, increment the address counter */
				if (HighByte)
				  CurrAddress += 2;

				HighByte = !HighByte;
			}
			else
			{
				/* Read the next EEPROM byte into the endpoint */
				WriteNextResponseByte(eeprom_read_byte((uint8_t*)(intptr_t)(CurrAddress >> 1)));

				/* Increment the address counter after use */
				CurrAddress += 2;
			}
		}
	}
	else
	{
		uint32_t PageStartAddress = CurrAddress;

		if (MemoryType == MEMORY_TYPE_FLASH)
		{
			boot_page_erase(PageStartAddress);
			boot_spm_busy_wait();
			
#if defined(ENABLE_RESET_AFTER_PROGRAMMING)
			TimeoutEnabled = true;
#endif
		}

		while (BlockSize--)
		{
			if (MemoryType == MEMORY_TYPE_FLASH)
			{
				/* If both bytes in current word have been written, increment the address counter */
				if (HighByte)
				{
					/* Write the next FLASH word to the current FLASH page */
					boot_page_fill(CurrAddress, ((FetchNextCommandByte() << 8) | LowByte));

					/* Increment the address counter after use */
					CurrAddress += 2;
				}
				else
				{
					LowByte = FetchNextCommandByte();
				}
				
				HighByte = !HighByte;
			}
			else
			{
				/* Write the next EEPROM byte from the endpoint */
				eeprom_write_byte((uint8_t*)((intptr_t)(CurrAddress >> 1)), FetchNextCommandByte());

				/* Increment the address counter after use */
				CurrAddress += 2;
			}
		}

		/* If in FLASH programming mode, commit the page after writing */
		if (MemoryType == MEMORY_TYPE_FLASH)
		{
			/* Commit the flash page to memory */
			boot_page_write(PageStartAddress);

			/* Wait until write operation has completed */
			boot_spm_busy_wait();
		}

		/* Send response byte back to the host */
		WriteNextResponseByte('\r');
	}
}
#endif

/** Retrieves the next byte from the host in the CDC data OUT endpoint, and clears the endpoint bank if needed
 *  to allow reception of the next data packet from the host.
 *
 *  \return Next received byte from the host in the CDC data OUT endpoint
 */
static uint8_t FetchNextCommandByte(void)
{
	/* Select the OUT endpoint so that the next data byte can be read */
	Endpoint_SelectEndpoint(CDC_RX_EPADDR);

	/* If OUT endpoint empty, clear it and wait for the next packet from the host */
	while (!(Endpoint_IsReadWriteAllowed()))
	{
		Endpoint_ClearOUT();

		while (!(Endpoint_IsOUTReceived()))
		{
			if (USB_DeviceState == DEVICE_STATE_Unattached)
			  return 0;
		}
	}

	/* Fetch the next byte from the OUT endpoint */
	return Endpoint_Read_8();
}

/** Writes the next response byte to the CDC data IN endpoint, and sends the endpoint back if needed to free up the
 *  bank when full ready for the next byte in the packet to the host.
 *
 *  \param[in] Response  Next response byte to send to the host
 */
static void WriteNextResponseByte(const uint8_t Response)
{
	/* Select the IN endpoint so that the next data byte can be written */
	Endpoint_SelectEndpoint(CDC_TX_EPADDR);

	/* If IN endpoint full, clear it and wait until ready for the next packet to the host */
	if (!(Endpoint_IsReadWriteAllowed()))
	{
		Endpoint_ClearIN();

		while (!(Endpoint_IsINReady()))
		{
			if (USB_DeviceState == DEVICE_STATE_Unattached)
			  return;
		}
	}

	/* Write the next byte to the IN endpoint */
	Endpoint_Write_8(Response);
}

/** Task to read in AVR910 commands from the CDC data OUT endpoint, process them, perform the required actions
 *  and send the appropriate response back to the host.
 */
static void CDC_Task(void)
{
	/* Select the OUT endpoint */
	Endpoint_SelectEndpoint(CDC_RX_EPADDR);

	/* Check if endpoint has a command in it sent from the host */
	if (!(Endpoint_IsOUTReceived()))
	  return;

	Timeout = 0;
        
	/* Read in the bootloader command (first byte sent from host) */
	uint8_t Command = FetchNextCommandByte();

	if (Command == AVR109_COMMAND_ExitBootloader)
	{
		//RunBootloader = false;

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if ((Command == AVR109_COMMAND_SetLED) || (Command == AVR109_COMMAND_ClearLED) ||
	         (Command == AVR109_COMMAND_SelectDeviceType))
	{
		FetchNextCommandByte();

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if ((Command == AVR109_COMMAND_EnterProgrammingMode) || (Command == AVR109_COMMAND_LeaveProgrammingMode))
	{
		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if (Command == AVR109_COMMAND_ReadPartCode)
	{
		/* Return ATMEGA128 part code - this is only to allow AVRProg to use the bootloader */
		WriteNextResponseByte(DEVICE_CODE);
		WriteNextResponseByte(0x00);
	}
	else if (Command == AVR109_COMMAND_ReadAutoAddressIncrement)
	{
		/* Indicate auto-address increment is supported */
		WriteNextResponseByte('Y');
	}
	else if (Command == AVR109_COMMAND_SetCurrentAddress)
	{
		/* Set the current address to that given by the host (translate 16-bit word address to byte address) */
		CurrAddress   = (FetchNextCommandByte() << 9);
		CurrAddress  |= (FetchNextCommandByte() << 1);

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if (Command == AVR109_COMMAND_ReadBootloaderInterface)
	{
		/* Indicate serial programmer back to the host */
		WriteNextResponseByte('S');
	}
	else if (Command == AVR109_COMMAND_ReadBootloaderIdentifier)
	{
		/* Write the 7-byte software identifier to the endpoint */
		for (uint8_t CurrByte = 0; CurrByte < 7; CurrByte++)
		  WriteNextResponseByte(SOFTWARE_IDENTIFIER[CurrByte]);
	}
	else if (Command == AVR109_COMMAND_ReadBootloaderSWVersion)
	{
		WriteNextResponseByte('0' + BOOTLOADER_VERSION_MAJOR);
		WriteNextResponseByte('0' + BOOTLOADER_VERSION_MINOR);
	}
	else if (Command == AVR109_COMMAND_ReadSignature)
	{
		WriteNextResponseByte(AVR_SIGNATURE_3);
		WriteNextResponseByte(AVR_SIGNATURE_2);
		WriteNextResponseByte(AVR_SIGNATURE_1);
	}
	else if (Command == AVR109_COMMAND_EraseFLASH)
	{
		/* Clear the application section of flash */
		for (uint32_t CurrFlashAddress = 0; CurrFlashAddress < (uint32_t)BOOT_START_ADDR; CurrFlashAddress += SPM_PAGESIZE)
		{
			boot_page_erase(CurrFlashAddress);
			boot_spm_busy_wait();
			boot_page_write(CurrFlashAddress);
			boot_spm_busy_wait();
		}

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	#if !defined(NO_LOCK_BYTE_WRITE_SUPPORT)
	else if (Command == AVR109_COMMAND_WriteLockbits)
	{
		/* Set the lock bits to those given by the host */
		boot_lock_bits_set(FetchNextCommandByte());

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	#endif
	else if (Command == AVR109_COMMAND_ReadLockbits)
	{
		WriteNextResponseByte(boot_lock_fuse_bits_get(GET_LOCK_BITS));
	}
	else if (Command == AVR109_COMMAND_ReadLowFuses)
	{
		WriteNextResponseByte(boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS));
	}
	else if (Command == AVR109_COMMAND_ReadHighFuses)
	{
		WriteNextResponseByte(boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS));
	}
	else if (Command == AVR109_COMMAND_ReadExtendedFuses)
	{
		WriteNextResponseByte(boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS));
	}
	#if !defined(NO_BLOCK_SUPPORT)
	else if (Command == AVR109_COMMAND_GetBlockWriteSupport)
	{
		WriteNextResponseByte('Y');

		/* Send block size to the host */
		WriteNextResponseByte(SPM_PAGESIZE >> 8);
		WriteNextResponseByte(SPM_PAGESIZE & 0xFF);
	}
	else if ((Command == AVR109_COMMAND_BlockWrite) || (Command == AVR109_COMMAND_BlockRead))
	{
		/* Delegate the block write/read to a separate function for clarity */
		ReadWriteMemoryBlock(Command);
	}
	#endif
	#if !defined(NO_FLASH_BYTE_SUPPORT)
	else if (Command == AVR109_COMMAND_FillFlashPageWordHigh)
	{
		/* Write the high byte to the current flash page */
		boot_page_fill(CurrAddress, FetchNextCommandByte());

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if (Command == AVR109_COMMAND_FillFlashPageWordLow)
	{
		/* Write the low byte to the current flash page */
		boot_page_fill(CurrAddress | 0x01, FetchNextCommandByte());

		/* Increment the address */
		CurrAddress += 2;

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if (Command == AVR109_COMMAND_WriteFlashPage)
	{
		/* Commit the flash page to memory */
		boot_page_write(CurrAddress);

		/* Wait until write operation has completed */
		boot_spm_busy_wait();

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if (Command == AVR109_COMMAND_ReadFLASHWord)
	{
		#if (FLASHEND > 0xFFFF)
		uint16_t ProgramWord = pgm_read_word_far(CurrAddress);
		#else
		uint16_t ProgramWord = pgm_read_word(CurrAddress);
		#endif

		WriteNextResponseByte(ProgramWord >> 8);
		WriteNextResponseByte(ProgramWord & 0xFF);
	}
	#endif
	#if !defined(NO_EEPROM_BYTE_SUPPORT)
	else if (Command == AVR109_COMMAND_WriteEEPROM)
	{
		/* Read the byte from the endpoint and write it to the EEPROM */
		eeprom_write_byte((uint8_t*)((intptr_t)(CurrAddress >> 1)), FetchNextCommandByte());

		/* Increment the address after use */
		CurrAddress += 2;

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if (Command == AVR109_COMMAND_ReadEEPROM)
	{
		/* Read the EEPROM byte and write it to the endpoint */
		WriteNextResponseByte(eeprom_read_byte((uint8_t*)((intptr_t)(CurrAddress >> 1))));

		/* Increment the address after use */
		CurrAddress += 2;
	}
	#endif
	else if (Command != AVR109_COMMAND_Sync)
	{
		/* Unknown (non-sync) command, return fail code */
		WriteNextResponseByte('?');
	}

	/* Select the IN endpoint */
	Endpoint_SelectEndpoint(CDC_TX_EPADDR);

	/* Remember if the endpoint is completely full before clearing it */
	bool IsEndpointFull = !(Endpoint_IsReadWriteAllowed());

	/* Send the endpoint data to the host */
	Endpoint_ClearIN();

	/* If a full endpoint's worth of data was sent, we need to send an empty packet afterwards to signal end of transfer */
	if (IsEndpointFull)
	{
		while (!(Endpoint_IsINReady()))
		{
			if (USB_DeviceState == DEVICE_STATE_Unattached)
			  return;
		}

		Endpoint_ClearIN();
	}

	/* Wait until the data has been sent to the host */
	while (!(Endpoint_IsINReady()))
	{
		if (USB_DeviceState == DEVICE_STATE_Unattached)
		  return;
	}

	/* Select the OUT endpoint */
	Endpoint_SelectEndpoint(CDC_RX_EPADDR);

	/* Acknowledge the command from the host */
	Endpoint_ClearOUT();
}

