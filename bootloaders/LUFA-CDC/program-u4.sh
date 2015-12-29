#!/bin/bash

avrdude -p m32u4 -c avrisp2 -P usb -e
avrdude -p m32u4 -c avrisp2 -P usb -U lfuse:w:0x7f:m -U hfuse:w:0x98:m -U efuse:w:0xcb:m
avrdude -p m32u4 -c avrisp2 -P usb -U flash:w:"Bootloader.hex"
