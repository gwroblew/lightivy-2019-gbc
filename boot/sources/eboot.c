/* Copyright (c) 2015-2016 Ivan Grokhotkov. All rights reserved.
 * This file is part of eboot bootloader.
 *
 * Redistribution and use is permitted according to the conditions of the
 * 3-clause BSD license to be found in the LICENSE file.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "flash.h"
#include "eboot_command.h"
#include "system.h"
#include "diskio/diskio.h"
#include "printf.h"

#define IRCODE __attribute__((section(".iram2.text")))

#define SWRST do { (*((volatile uint32_t*) 0x60000700)) |= 0x80000000; } while(0);

extern void ets_wdt_enable(void);
extern void ets_wdt_disable(void);

uint32_t IRCODE get_fattime() {
  return ((2019 - 1980) << 25) + (1 << 21) + (1 << 16);
}

void IRCODE dump(uint32_t v) {
    const char* __attribute__ ((aligned (4))) fmtt = "v%08x\n\0\0";
    uint32_t fmt[2];
    fmt[0] = ((uint32_t*) fmtt)[0];
    fmt[1] = ((uint32_t*) fmtt)[1];
    ets_printf((const char*) fmt, v);
}

int IRCODE print_version(const uint32_t flash_addr)
{
    uint32_t ver;
    if (SPIRead(flash_addr + APP_START_OFFSET + sizeof(image_header_t) + sizeof(section_header_t), &ver, sizeof(ver))) {
        return 1;
    }
    dump(ver);
    return 0;
}

#pragma GCC diagnostic ignored "-Wunused-variable"
int IRCODE load_app_from_flash_raw(const uint32_t flash_addr)
{
    image_header_t image_header;
    uint32_t pos = flash_addr + APP_START_OFFSET;

    if (SPIRead(pos, &image_header, sizeof(image_header))) {
        return 1;
    }
    pos += sizeof(image_header);


    for (uint32_t section_index = 0;
        section_index < image_header.num_segments;
        ++section_index)
    {
        section_header_t section_header = {0};
        if (SPIRead(pos, &section_header, sizeof(section_header))) {
            return 2;
        }
        pos += sizeof(section_header);

        const uint32_t address = section_header.address;

        bool load = false;

        if (address < 0x40000000) {
            load = true;
        }

        if (address >= 0x40100000 && address < 0x40108000) {
            load = true;
        }

        if (address >= 0x60000000) {
            load = true;
        }

        if (!load) {
            pos += section_header.size;
            continue;
        }

        if (SPIRead(pos, (void*)address, section_header.size))
            return 3;

        pos += section_header.size;
    }
        ets_putc('#');

    register uint32_t sp asm("a1") = 0x3ffffff0;
    register uint32_t pc asm("a3") = image_header.entry;
    __asm__  __volatile__ ("jx a3");

    return 0;
}
#pragma GCC diagnostic warning "-Wunused-variable"

extern void hal_init();
extern void boot_gpio_init();
extern void set_screen(uint32_t y1, uint32_t y2);

void IRCODE flash_binary(char *fname, uint32_t addr, uint32_t size) {
  int r = file_open(fname, FILE_MODE_READ);
  if (r != RES_OK) {
    lcd_mode();
    printf("%s missing\n", fname + 8);
    return;
  }
  lcd_mode();
  printf("Found %s\n", fname + 8);
  SPIEraseAreaEx(addr, size);
  printf("%s->Flash\n", fname + 8);
  while (1) {
    uint32_t br;
    r = file_read((void *)0x3FFF0000, 32768, &br);
    if (br == 0)
      break;
    br = (br & 0xF000) + ((br & 4095) ? 4096 : 0);
    SPIWrite(addr, (void *)0x3FFF0000, br);
    addr += br;
  }
  sleep_ms(10);
}

const char firmware_bin[] RODATA = "0:/boot/firmware.bin";
const char oc_bin[] RODATA = "0:/boot/oc.bin";

int IRCODE main()
{
    int res = 9;
    struct eboot_command cmd;

    print_version(0);
    boot_gpio_init();
    get_keys();
    sleep_ms(10);
    if ((get_keys() & 8) != 0) {
      hal_init();
      printf("Forced restore...\n");
      int r = disk_init(0);
      if (r == 0) {
        ets_wdt_disable();
        flash_binary((char *)firmware_bin, FLASH_SYS_BASE, FLASH_SYS_SIZE);
        flash_binary((char *)oc_bin, FLASH_OC_BASE, FLASH_OC_SIZE);
        ets_wdt_enable();
      }
    }

    if (eboot_command_read(&cmd) == 0) {
        // valid command was passed via RTC_MEM
        eboot_command_clear();
        ets_putc('@');
    } else {
        // no valid command found
        cmd.action = ACTION_LOAD_APP;
        cmd.args[0] = 0;
        ets_putc('~');
    }

    if (cmd.action == ACTION_COPY_RAW) {
        ets_putc('c'); ets_putc('p'); ets_putc(':');

        hal_init();
        printf("Updating:\n");
        int r = disk_init(0);
        if (r == 0) {
          ets_wdt_disable();
          if (cmd.args[0] == 0) {
            flash_binary((char *)firmware_bin, FLASH_SYS_BASE, FLASH_SYS_SIZE);
          } else if (cmd.args[0] == 1) {
            flash_binary((char *)oc_bin, FLASH_OC_BASE, FLASH_OC_SIZE);
          } else {
            flash_binary((char *)firmware_bin, FLASH_SYS_BASE, FLASH_SYS_SIZE);
            flash_binary((char *)oc_bin, FLASH_OC_BASE, FLASH_OC_SIZE);
          }
          ets_wdt_enable();
        }
        cmd.action = ACTION_LOAD_APP;
        cmd.args[0] = 0;
    }

    if (cmd.action == ACTION_LOAD_APP) {
        ets_putc('l'); ets_putc('d'); ets_putc('\n');
        res = load_app_from_flash_raw(cmd.args[0]);
        //we will get to this only on load fail
        ets_putc('e'); ets_putc(':'); ets_putc('0'+res); ets_putc('\n');
    }

    if (res) {
        SWRST;
    }

    while(true){}
}
