ifeq ($(OS),Windows_NT)
	CROSS_COMPILE	?= c:/Espressif/xtensa-lx106-elf/bin/xtensa-lx106-elf-
else
	CROSS_COMPILE	?= ../xtensa-lx106-elf/bin/xtensa-lx106-elf-
endif

PROJNAME		= oc
SDKPATH			= $(CURDIR)/../sdk
FIRMWARE		= firmware/$(PROJNAME).bin
FIRMWARE2		= ../firmware/firmware/firmware.bin
OBJDIR			= build/obj
TARGETAR		= $(OBJDIR)/$(PROJNAME).a
TARGETELF		= $(OBJDIR)/$(PROJNAME).elf
INCLUDEDIRS		= $(addprefix -I,$(CURDIR)/sources $(CURDIR)/../common $(CURDIR)/sources/mcufont $(CURDIR)/sources/boy)
INCLUDES 		= $(wildcard ../common/*.h) $(wildcard sources/*.h) $(wildcard sources/mcufont/*.h) $(wildcard sources/boy/*.h) $(wildcard ../common/diskio/*.h) $(wildcard ../common/fatfs/*.h)
LIBDIR			= $(addprefix -L,$(SDKPATH)/lib)
LIBS			= $(addprefix -l,c gcc)
SOURCES			= $(wildcard sources/*.c) $(wildcard sources/mcufont/*.c) $(wildcard sources/boy/*.c)
OBJECTS			= $(addprefix $(OBJDIR)/, $(SOURCES:%.c=%.o))
#CFLAGS			= -O2 -Wall -mlongcalls -mtext-section-literals -mserialize_volatile -DICACHE_FLASH -nostdlib -std=gnu89
CFLAGS			= -Os -Wall -mlongcalls -mtext-section-literals -DICACHE_FLASH -nostdlib -std=gnu99 -fno-jump-tables -ffreestanding -ffunction-sections
CFLAGS2			= -Os -Wall -mlongcalls -mtext-section-literals -DICACHE_FLASH -nostdlib -std=gnu99 -ffreestanding -ffunction-sections
LDFLAGS			= -u main_start -Wl,-static -Wl,-gc-sections -nostdlib
CC				= $(CROSS_COMPILE)gcc
AR				= $(CROSS_COMPILE)ar
SIZE			= $(CROSS_COMPILE)size


.PHONY: all flash full_flash terminal clean disassemble reboot

all: $(FIRMWARE)

$(OBJDIR)/sources/boy/boy_emu.o $(OBJDIR)/sources/boy/sound.o : CFLAGS = $(CFLAGS2)

$(OBJDIR)/%.o: %.c $(INCLUDES)
	@echo "CC $<"
	@mkdir -p $(dir $@)
	@$(CC) $(INCLUDEDIRS) $(CFLAGS) -c $< -o $@

$(TARGETAR): $(OBJECTS)
	@echo "AR $@"
	@$(AR) cru $@ $(OBJECTS)

$(TARGETELF): $(TARGETAR)
	@echo "LD $@"
	@$(CC) $(LIBDIR) -T$(PROJNAME).ld $(LDFLAGS) -Wl,-Map=$(PROJNAME).map -Wl,--start-group $(LIBS) $(TARGETAR) -Wl,--end-group -o $@
	@$(SIZE) -d $(TARGETELF)

$(FIRMWARE): $(TARGETELF)
	@mkdir -p $(dir $(FIRMWARE))
	@./genbin.sh $(TARGETELF) $(FIRMWARE)

flash: all
	@../esp-utils/build/esp-flasher --developer 0x58000 $(FIRMWARE)
	@cp -f $(FIRMWARE) $(FIRMWARE).prev

full_flash: rebuild
	@rm -f $(FIRMWARE).prev
	@../esp-utils/build/esp-flasher 0x58000 $(FIRMWARE)
	@cp -f $(FIRMWARE) $(FIRMWARE).prev

upload: all
	@../tools/nettool/nettool put 0:/boot/$(PROJNAME).bin $(FIRMWARE)
	@../tools/nettool/nettool reboot 2

upload2: all
	@../tools/nettool/nettool put 0:/boot/$(PROJNAME).bin $(FIRMWARE)
	@../tools/nettool/nettool put 0:/boot/firmware.bin $(FIRMWARE2)
	@../tools/nettool/nettool reboot 3

reboot:
	@../tools/nettool/nettool reboot 2

reboot2:
	@../tools/nettool/nettool reboot 3

terminal:
	@../esp-utils/build/esp-terminal

rebuild: clean all

clean:
	@rm -rf build
	@rm -f $(FIRMWARE)
	@rm -f $(FIRMWARE).prev
	@rm -f $(PROJNAME).map

disassemble:
	@$(CROSS_COMPILE)objdump -d $(TARGETELF) > $(OBJDIR)/disassemble.txt

reset:
	@(cd $(dir $(FIRMWARE)) && ./../esp-utils/build/esp-flasher --reboot)
