# make all = Make software and program
# make clean = Clean out built project files.
# make program = Download the hex file to the device, using avrdude.

# Microcontroller Type
MCU = atmega8

# Target file name (without extension).
TARGET = solar_panel

# Programming hardware: type avrdude -c ?
# to get a full listing.
AVRDUDE_PROGRAMMER = linuxgpio

# Output format. (can be srec, ihex, binary)
FORMAT = ihex

# List C source files here. (C dependencies are automatically generated.)
SRC = $(TARGET).c

CFLAGS = -g -Os


# Programming support using avrdude. Settings and variables.


AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex

AVRDUDE_FLAGS = -p $(MCU) -c $(AVRDUDE_PROGRAMMER)

# ---------------------------------------------------------------------------

# Define programs and commands.
SHELL = bash

CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump


# Programming support using avrdude.
AVRDUDE = avrdude

REMOVE = rm -f
COPY = cp


# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after:
MSG_COFF = Converting to AVR COFF:
MSG_EXTENDED_COFF = Converting to AVR Extended COFF:
MSG_FLASH = Creating load file for Flash:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:

# Define all object files.
OBJ = $(SRC:.c=.o)


# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) $(CFLAGS)

# Default target: make program!
all: begin $(TARGET).hex \
	 finished end
	 sudo $(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)

begin:
	@echo
	@echo $(MSG_BEGIN)

finished:
	@echo $(MSG_ERRORS_NONE)

end:
	@echo $(MSG_END)
	@echo


# Program the device.  
program: $(TARGET).hex 
	sudo $(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)


# Create final output files (.hex, .eep) from ELF output file.
%.hex:%.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -j .text -j .data -O $(FORMAT) $< $@


# Link: create ELF output file from object files.
.SECONDARY: $(TARGET).elf
.PRECIOUS: $(OBJ)
%.elf: $(OBJ)
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(ALL_CFLAGS) $(OBJ) -o $(TARGET).elf


# Compile: create object files from C source files.
%.o:%.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(ALL_CFLAGS) $< -o $@


# Target: clean project.
clean: begin clean_list finished end

clean_list:
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(OBJ)

