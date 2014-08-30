#   Makefile - build script
#   Copyright (C) 2014  Nicolas Benes
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.

CC := avr-gcc
OBJDUMP := avr-objdump
OBJCOPY := avr-objcopy
MV := mv
RM := rm
MKDIR := mkdir
DOXYGEN := doxygen
SIZE := avr-size
ECHO := echo
EXPAND := expand -t 2
INDENT := indent

TARGET := rxsms
MCU := atxmega32a4u
# cpu clk frq w/o 'UL'
F_CPU := 32000000

DOCDIR := doc

SRC := $(wildcard *.c)
HDR := $(wildcard *.h)
OBJ := $(patsubst %.c, %.o, $(SRC))

CFLAGS += -mmcu=$(MCU)
CFLAGS += -DF_CPU=$(F_CPU)UL
CFLAGS += -O3
CFLAGS += -fdata-sections
CFLAGS += -ffunction-sections
CFLAGS += -fshort-enums
CFLAGS += -Wall
CFLAGS += -Wextra
#CFLAGS += -Werror-implicit-function-declaration
CFLAGS += -Wmissing-prototypes
CFLAGS += -Wpointer-arith
CFLAGS += -Wstrict-prototypes
CFLAGS += -pedantic
CFLAGS += -mrelax
CFLAGS += -std=gnu99
CFLAGS += -g3
# commented because it's too verbose
#CFLAGS += -save-temps
CFLAGS += -Wswitch-enum

LFLAGS += -mmcu=$(MCU) -O3
LFLAGS += -Wl,-Map,$(TARGET).map
# preserve intermediate files
.SECONDARY:


all: $(TARGET).elf  $(TARGET).hex  $(TARGET).lss $(TARGET).eep
	@$(SIZE) --mcu=$(MCU) --format=avr $<
	@echo DONE

%.hex: %.elf
	$(OBJCOPY) -O ihex -R .eeprom -R .fuse -R .lock $< $@

%.eep: %.elf
	$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
		--change-section-lma .eeprom=0 --no-change-warnings -O ihex $< $@

%.lss: %.elf
	$(MV) $@ $(TARGET).prev.lss || true
	$(MV) $(TARGET)-dense.lss $(TARGET)-dense.prev.lss || true
	$(OBJDUMP) -h -S -z $< | $(EXPAND) > $@
	$(OBJDUMP) -h -d -z $< | $(EXPAND) > $(TARGET)-dense.lss

%.elf: $(OBJ)
	@$(ECHO) Link $@
	@$(CC) $(LFLAGS) -o $@ $^

%.o: %.c $(HDR) Makefile
	@$(ECHO) Compile $<
	@$(CC) $(CFLAGS) -o $@ -c $<

doxy: | $(DOCDIR)
	$(RM) -rf $(wildcard $(DOCDIR)/*)
	$(DOXYGEN) Doxyfile

indent:
	$(INDENT) -kr -psl -nut $(SRC) $(HDR)

clean:
	$(RM) -rf $(DOCDIR) $(TARGET).elf  $(TARGET).hex  \
		$(TARGET).lss $(TARGET).prev.lss \
		$(TARGET)-dense.lss $(TARGET)-dense.prev.lss \
		$(TARGET).eep $(TARGET).map

.PHONY: all clean doxy indent
