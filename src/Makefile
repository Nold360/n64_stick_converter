# Makefile for n64_stick_converter
# Available Options:
# make...
#   hex
#   writeflash
#   clean

#####         Target Specific Details          #####
#####     Customize these for your project     #####

# Name of target controller
# (e.g. 'at90s8515', see the available avr-gcc mmcu
# options for possible values)
MCU=attiny24a

# id to use with programmer
# default: PROGRAMMER_MCU=$(MCU)
# In case the programer used, e.g avrdude, doesn't
# accept the same MCU name as avr-gcc (for example
# for ATmega8s, avr-gcc expects 'atmega8' and
# avrdude requires 'm8')
#PROGRAMMER_MCU=m16
#PROGRAMMER_MCU=m8515
PROGRAMMER_MCU=t24

# Name of our project
PROJECTNAME=n64_stick_converter

# Source files
PRJSRC=n64_stick_converter.c

# additional includes (e.g. -I/path/to/mydir)
INC=-I/usr/lib/avr/include/

# libraries to link in (e.g. -lmylib)
LIBS=

# Optimization level,
# use s (size opt), 1, 2, 3 or 0 (off)
OPTLEVEL=s


#####      AVR Dude 'writeflash' options       #####
#####  If you are using the avrdude program
#####  (http://www.bsdhome.com/avrdude/) to write
#####  to the MCU, you can set the following config
#####  options and use 'make writeflash' to program
#####  the device.

# programmer id--check the avrdude for complete list
# of available opts.  These should include stk500,
# avr910, avrisp, bsd, pony and more.  Set this to
# one of the valid "-c PROGRAMMER-ID" values
# described in the avrdude info page.
#
AVRDUDE_PROGRAMMERID=stk500v2

# serial/parallel/usb-port to which your
# hardware programmer is attached
#
AVRDUDE_PORT=/dev/ttyUSB0

# HEXFORMAT -- format for .hex file output
HEXFORMAT=ihex

    #-mtiny-stack                            \
	#-Winline -finline-functions             \
# compiler
CFLAGS= -I. $(INC) -g -mmcu=$(MCU) -O$(OPTLEVEL) \
	-fpack-struct -fshort-enums             \
	-funsigned-bitfields -funsigned-char    \
	-Wall -Wstrict-prototypes               \
	-Wa,-ahldms=$(firstword                  \
	$(filter %.lst, $(<:.c=.lst)))

# linker
LDFLAGS=-Wl,-Map,$(TRG).map -mmcu=$(MCU) -lm $(LIBS)

##### executables ####
CC=avr-gcc
OBJCOPY=avr-objcopy
OBJDUMP=avr-objdump
SIZE=avr-size
AVRDUDE=avrdude
REMOVE=rm -f

##### automatic target names ####
TRG=$(PROJECTNAME).out
DUMPTRG=$(PROJECTNAME).s

HEXROMTRG=$(PROJECTNAME).hex
HEXTRG=$(HEXROMTRG)
GDBINITFILE=gdbinit-$(PROJECTNAME)

# Define all object files.
CFILES=$(filter %.c, $(PRJSRC))


# List all object files we need to create
OBJDEPS=$(CFILES:.c=.o)

# Define all lst files.
LST=$(filter %.lst, $(OBJDEPS:.o=.lst))

.SUFFIXES : .c .cc .cpp .C .o .out .s .S \
	.hex .ee.hex .h .hh .hpp

.PHONY: writeflash clean

# Make targets:
# all, disasm, stats, hex, writeflash/install, clean
default: hex

all: $(TRG)

disasm: $(DUMPTRG) stats

stats: $(TRG)
	$(OBJDUMP) -h $(TRG)
	$(SIZE) $(TRG)

hex: $(HEXTRG)


writeflash: hex
	$(AVRDUDE) -c $(AVRDUDE_PROGRAMMERID)   \
	 -p $(PROGRAMMER_MCU) -P $(AVRDUDE_PORT) -e        \
	 -U flash:w:$(HEXROMTRG)

fuses:
	$(AVRDUDE) -c $(AVRDUDE_PROGRAMMERID) \
	-p $(PROGRAMMER_MCU) -P $(AVRDUDE_PORT) \
	-U hfuse:w:0xDF:m -U lfuse:w:0x62:m

install: writeflash

$(DUMPTRG): $(TRG)
	$(OBJDUMP) -S  $< > $@


$(TRG): $(OBJDEPS)
	$(CC) $(LDFLAGS) -o $(TRG) $(OBJDEPS)


#### Generating object files ####
# object from C
.c.o:
	$(CC) $(CFLAGS) -c $< -o $@


#### Generating hex files ####
# hex files from elf
#####  Generating a gdb initialisation file    #####
.out.hex:
	$(OBJCOPY) -j .text                    \
		-j .data                       \
		-O $(HEXFORMAT) $< $@

.out.ee.hex:
	$(OBJCOPY) -j .eeprom                  \
		--change-section-lma .eeprom=0 \
		-O $(HEXFORMAT) $< $@

#### Cleanup ####
clean:
	$(REMOVE) $(TRG) $(TRG).map $(DUMPTRG)
	$(REMOVE) $(OBJDEPS)
	$(REMOVE) $(LST)
	$(REMOVE) $(HEXTRG)
	
