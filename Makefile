#  Part of Twister
#
#  Copyright (c) 2009 Simen Svale Skogsrud
#
#  Twister is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  Twister is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with Twister.  If not, see <http://www.gnu.org/licenses/>.


# This is a prototype Makefile. Modify it according to your needs.
# You should at least check the settings for
# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected.
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

DEVICE     = atmega1280 
CLOCK      = 16000000
PROGRAMMER = -c avrisp2 -P /dev/tty.usbmodem621
OBJECTS    = main.o MotionControl.o GCodeParser.o serial_protocol.o \
             libraries/HardwareSerial.o libraries/Print.o libraries/SimplePacket.o \
             libraries/extruder.o Twister.o
FUSES      = -U hfuse:w:0xd9:m -U lfuse:w:0x24:m

# Tune the lines below only if you know what you are doing:

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE) -B 10
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -I. 

# symbolic targets:
all:	twister.hex

.c.o:
	$(COMPILE) -c $< -o $@ 

.cpp.o:
	$(COMPILE) -c $< -o $@ -Ilibraries/

.cc.o:
	$(COMPILE) -c $< -o $@ -Ilibraries/

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:twister.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash fuse

# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID twister.hex

clean:
	rm -f twister.hex main.elf $(OBJECTS)

# file targets:
main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS) -lm

twister.hex: main.elf
	rm -f twister.hex
	avr-objcopy -j .text -j .data -O ihex main.elf twister.hex
	avr-size *.hex *.elf *.o
	avr-size libraries/*.o
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c 
