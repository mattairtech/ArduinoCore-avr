#!/bin/bash

avrdude -p usb1286 -c avrisp2 -P usb -e
avrdude -p usb1286 -c avrisp2 -P usb -U lfuse:w:0x7f:m -U hfuse:w:0x9a:m -U efuse:w:0xfb:m
avrdude -p usb1286 -c avrisp2 -P usb -U flash:w:"Bootloader.hex"