/*
   Copyright (c) 2011, Peter Barrett
   Copyright (c) 2015, Arduino LLC

   Permission to use, copy, modify, and/or distribute this software for
   any purpose with or without fee is hereby granted, provided that the
   above copyright notice and this permission notice appear in all copies.

   THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
   WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
   BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
   OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
   WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
   ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
   SOFTWARE.
 */

#if !defined(ARDUINO_USB_DISABLED)
#define PLUGGABLE_USB_ENABLED
#endif

#if defined(EPRST6)
#define USB_ENDPOINTS 7 // AtMegaxxU4
#else
#define USB_ENDPOINTS 5 // AtMegaxxU2
#endif
#define USB_ENDPOINTS_MASK 7

// These are now controlled by the boards.txt menu system
// Now that there is Pluggable USB, these will be eliminated
// once an alternate method for USB PID allocation is found.
#if defined(ARDUINO_CDC_MIDI_HID) || defined(ARDUINO_CDC_MSD_HID) || defined(ARDUINO_CDC_ONLY) || defined(ARDUINO_CDC_HID)
#define CDC_ENABLED
#if defined(ARDUINO_CDC_ONLY)
#define CDC_ONLY
#else
#define IAD_PRESENT
#endif
#endif

#if defined(ARDUINO_HID_ONLY) || defined(ARDUINO_CDC_HID) || defined(ARDUINO_CDC_MIDI_HID) || defined(ARDUINO_CDC_MSD_HID)
#define HID_ENABLED
#endif

#if defined(ARDUINO_CDC_MIDI_HID) || defined(ARDUINO_MIDI_ONLY)
#define MIDI_ENABLED
#define IAD_PRESENT
#endif

#if defined(ARDUINO_CDC_MSD_HID) || defined(ARDUINO_MSD_ONLY)
#define MSD_ENABLED
#endif

#define ISERIAL_MAX_LEN     20

#ifdef CDC_ENABLED
#define CDC_INTERFACE_COUNT	2
#define CDC_ENPOINT_COUNT	3
#else
#define CDC_INTERFACE_COUNT	0
#define CDC_ENPOINT_COUNT	0
#endif

#define CDC_ACM_INTERFACE	0	// CDC ACM
#define CDC_DATA_INTERFACE	1	// CDC Data
#define CDC_FIRST_ENDPOINT	1
#define CDC_ENDPOINT_ACM	(CDC_FIRST_ENDPOINT)							// CDC First
#define CDC_ENDPOINT_OUT	(CDC_FIRST_ENDPOINT+1)
#define CDC_ENDPOINT_IN		(CDC_FIRST_ENDPOINT+2)

#define INTERFACE_COUNT		(MSC_INTERFACE + MSC_INTERFACE_COUNT)

#ifdef CDC_ENABLED
#define CDC_RX CDC_ENDPOINT_OUT
#define CDC_TX CDC_ENDPOINT_IN
#endif

#define IMANUFACTURER	1
#define IPRODUCT	2
#define ISERIAL         3
