#
#             LUFA Library
#     Copyright (C) Dean Camera, 2014.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.org
#
# --------------------------------------
#         LUFA Project Makefile.
# --------------------------------------

# Run "make help" for target help.
# Modify MCU and FLASH_SIZE_KB below.

# MCU: at90usb162, atmega32u2, atmega32u4, at90usb646, at90usb1286, at90usb647, at90usb1287
MCU          = atmega32u4
ARCH         = AVR8
BOARD        = NONE
F_CPU        = 16000000
F_USB        = 16000000
OPTIMIZATION = s
TARGET       = Bootloader
SRC          = $(TARGET).c Descriptors.c $(LUFA_SRC_USB)
LUFA_PATH    = ./LUFA
CC_FLAGS     = -DUSE_LUFA_CONFIG_HEADER -IConfig/ -DBOOT_START_ADDR=$(BOOT_START_OFFSET)
LD_FLAGS     = -Wl,--section-start=.text=$(BOOT_START_OFFSET)

# Flash size and bootloader section sizes of the target, in KB. These must
# match the target's total FLASH size and the bootloader size set in the
# device's fuses.
BOOT_SECTION_SIZE_KB  = 4
# FLASH_SIZE_KB: at90usb162=16, atmega32u2=32, atmega32u4=32, at90usb646=64, at90usb1286=128, at90usb647=64, at90usb1287=128
FLASH_SIZE_KB         = 32

# Bootloader address calculation formulas
# Do not modify these macros, but rather modify the dependent values above.
CALC_ADDRESS_IN_HEX   = $(shell printf "0x%X" $$(( $(1) )) )
BOOT_START_OFFSET     = $(call CALC_ADDRESS_IN_HEX, ($(FLASH_SIZE_KB) - $(BOOT_SECTION_SIZE_KB)) * 1024 )
BOOT_SEC_OFFSET       = $(call CALC_ADDRESS_IN_HEX, ($(FLASH_SIZE_KB) * 1024) - $(strip $(1)) )

# Default target
all:

# Include LUFA build script makefiles
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_cppcheck.mk
include $(LUFA_PATH)/Build/lufa_doxygen.mk
include $(LUFA_PATH)/Build/lufa_avrdude.mk
include $(LUFA_PATH)/Build/lufa_atprogram.mk

