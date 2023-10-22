# simple AVR Makefile
#
# written by michael cousins (http://github.com/mcous)
# released to the public domain

# Makefile
#
# targets:
#   all:    compiles the source code
#   test:   tests the isp connection to the mcu
#   flash:  writes compiled hex file to the mcu's flash memory
#   fuse:   writes the fuse bytes to the MCU
#   disasm: disassembles the code for debugging
#   clean:  removes all .hex, .elf, and .o files in the source code and library directories

# parameters (change this stuff accordingly)
# project name
PRJ = main
# avr mcu
MCU = attiny13a
# mcu clock frequency
CLK = 1000000
# avr programmer (and port if necessary)
# e.g. PRG = usbtiny -or- PRG = arduino -P /dev/tty.usbmodem411
PRG = stk500v1 -b19200 -v
# fuse values for avr: low, high, and extended
# these values are from an Arduino Uno (ATMega328P)
# see http://www.engbedded.com/fusecalc/ for other MCUs and options
LFU = 0x6A
HFU = 0xFF
EFU = 0x05
# program source files (not including external libraries)
SRC = $(PRJ).cpp
# where to look for external libraries (consisting of .c/.cpp files and .h files)
# e.g. EXT = ../../EyeToSee ../../YouSART
EXT =


#################################################################################################
# \/ stuff nobody needs to worry about until such time that worrying about it is appropriate \/ #
#################################################################################################

# include path
INCLUDE := $(foreach dir, $(EXT), -I$(dir))
# c flags
CFLAGS    = -Wall -O1 -DF_CPU=$(CLK) -mmcu=$(MCU) $(INCLUDE)
# any aditional flags for c++
CPPFLAGS =

# executables
AVRDUDE = avrdude -u -v -v -pt13 -c arduino -P /dev/ttyUSB0 -b19200
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE    = avr-size --format=avr --mcu=$(MCU)
CC      = avr-gcc

# generate list of objects
CFILES    = $(filter %.c, $(SRC))
EXTC     := $(foreach dir, $(EXT), $(wildcard $(dir)/*.c))
CPPFILES  = $(filter %.cpp, $(SRC))
EXTCPP   := $(foreach dir, $(EXT), $(wildcard $(dir)/*.cpp))
OBJ       = $(CFILES:.c=.o) $(EXTC:.c=.o) $(CPPFILES:.cpp=.o) $(EXTCPP:.cpp=.o)

# user targets
# compile all files
all: $(PRJ).hex

# test programmer connectivity
test:
	$(AVRDUDE) -v

erase:
	avrdude -u -v -v -pt13 -c arduino -P /dev/ttyUSB0 -b19200 -e

flashwErase:
	avrdude -u -v -v -V -pt13 -c arduino -P /dev/ttyUSB0 -b19200 -U flash:w:$(PRJ).hex:i

flashread:
	avrdude -u -v -v -pt13 -c arduino -P /dev/ttyUSB0 -b19200 -U flash:r:$(PRJ).hex:i
# flash program to mcu
flash:
	avrdude -u -v -v -D -V -pt13 -c arduino -P /dev/ttyUSB0 -b19200 -U flash:w:$(PRJ).hex:i

# write fuses to mcu
fuse:
	$(AVRDUDE) -U lfuse:w:$(LFU):m -U hfuse:w:$(HFU):m -U lock:w:0xFC:m # -U efuse:w:$(EFU):m 

# generate disassembly files for debugging
disasm: $(PRJ).elf
	$(OBJDUMP) -d $(PRJ).elf

# remove compiled files
clean:
	rm -f *.hex *.elf *.o
	$(foreach dir, $(EXT), rm -f $(dir)/*.o;)

# other targets
# objects from c files
.c.o:
	$(CC) $(CFLAGS) -c $< -o $@

# objects from c++ files
.cpp.o:
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

# elf file
$(PRJ).elf: $(OBJ)
	$(CC) $(CFLAGS) -o $(PRJ).elf $(OBJ)

# hex file
$(PRJ).hex: $(PRJ).elf
	rm -f $(PRJ).hex
	$(OBJCOPY) -j .text -j .data -O ihex $(PRJ).elf $(PRJ).hex
	$(SIZE) $(PRJ).elf
