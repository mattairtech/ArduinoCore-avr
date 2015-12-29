#!/bin/bash

avrdude -p m32u2 -c avrisp2 -P usb -e
avrdude -p m32u2 -c avrisp2 -P usb -U lfuse:w:0x7f:m -U hfuse:w:0xd8:m -U efuse:w:0xce:m
avrdude -p m32u2 -c avrisp2 -P usb -U flash:w:"Bootloader.hex"
