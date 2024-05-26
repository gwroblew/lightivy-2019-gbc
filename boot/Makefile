ifeq ($(OS),Windows_NT)
	CROSS_COMPILE	?= c:/Espressif/xtensa-lx106-elf/bin/xtensa-lx106-elf-
else
	CROSS_COMPILE	?= ../xtensa-lx106-elf/bin/xtensa-lx106-elf-
endif

PROJNAME		= boot
SDKPATH			= $(CURDIR)/../sdk
FIRMWARE		= firmware/$(PROJNAME).bin
OBJDIR			= build/obj
TARGETAR		= $(OBJDIR)/$(PROJNAME).a
TARGETELF		= $(OBJDIR)/$(PROJNAME).elf
INCLUDEDIRS		= $(addprefix -I,$(SDKPATH)/include $(CURDIR)/sources $(CURDIR)/../common)
INCLUDES 		= $(wildcard ../common/*.h) $(wildcard sources/*.h) $(wildcard ../common/diskio/*.h) $(wildcard ../common/fatfs/*.h)
LIBDIR			= $(addprefix -L,$(SDKPATH)/lib)
#LIBS			= $(addprefix -l,c gcc phy pp net80211 lwip wpa main json crypto ssl)
LIBS			= $(addprefix -l,c gcc)
SOURCES			= $(wildcard sources/*.c) $(wildcard ../common/diskio/*.c) $(wildcard ../common/fatfs/*.c) $(wildcard ../common/*.c)
OBJECTS			= $(addprefix $(OBJDIR)/, $(SOURCES:%.c=%.o))
#CFLAGS			= -O2 -Wall -mlongcalls -mtext-section-literals -mserialize_volatile -DICACHE_FLASH -nostdlib -std=gnu89
CFLAGS			= -O0 -Wall -mlongcalls -mtext-section-literals -DICACHE_FLASH -DBOOT_IMAGE -nostdlib -std=gnu99 -fno-jump-tables -ffunction-sections
LDFLAGS			= -u main -Wl,-static -Wl,-gc-sections -nostdlib
CC				= $(CROSS_COMPILE)gcc
AR				= $(CROSS_COMPILE)ar
SIZE			= $(CROSS_COMPILE)size


.PHONY: all flash full_flash terminal clean disassemble reboot

all: $(FIRMWARE)

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
	@../esp-utils/build/esp-flasher --developer 0 $(FIRMWARE)
	@cp -f $(FIRMWARE) $(FIRMWARE).prev

full_flash: rebuild
	@rm -f $(FIRMWARE).prev
	@../esp-utils/build/esp-flasher 0 $(FIRMWARE)
	@cp -f $(FIRMWARE) $(FIRMWARE).prev

boot_flash:
	@rm -f $(FIRMWARE).prev
	@../esp-utils/build/esp-flasher 0 $(FIRMWARE)

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

reboot:
	@(cd $(dir $(FIRMWARE)) && ./../esp-utils/build/esp-flasher --reboot)
