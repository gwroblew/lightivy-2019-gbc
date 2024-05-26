/**
 * MIT License
 *
 * Copyright (c) 2018 Mahyar Koshkouei
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <stdint.h>	/* Required for int types */
#include <time.h>	/* Required for tm struct */

#include <assert.h>

/**
 * Sound support must be provided by an external library. When audio_read() and
 * audio_write() functions are provided, define ENABLE_SOUND before including
 * peanut_gb.h in order for these functions to be used.
 */
//#define ENABLE_SOUND 0

/* Enable LCD drawing. On by default. May be turned off for testing purposes. */
#ifndef ENABLE_LCD
	#define ENABLE_LCD 1
#endif

/* Interrupt masks */
#define VBLANK_INTR	0x01
#define LCDC_INTR	0x02
#define TIMER_INTR	0x04
#define SERIAL_INTR	0x08
#define CONTROL_INTR	0x10
#define ANY_INTR	0x1F

/* Memory section sizes for DMG */
#define WRAM_SIZE	0x2000
#define VRAM_SIZE	0x2000
#define HRAM_SIZE	0x0100
#define OAM_SIZE	0x00A0

/* Memory addresses */
#define ROM_0_ADDR      0x0000
#define ROM_N_ADDR      0x4000
#define VRAM_ADDR       0x8000
#define CART_RAM_ADDR   0xA000
#define WRAM_0_ADDR     0xC000
#define WRAM_1_ADDR     0xD000
#define ECHO_ADDR       0xE000
#define OAM_ADDR        0xFE00
#define UNUSED_ADDR     0xFEA0
#define IO_ADDR         0xFF00
#define HRAM_ADDR       0xFF80
#define INTR_EN_ADDR    0xFFFF

/* Cart section sizes */
#define ROM_BANK_SIZE   0x4000
#define WRAM_BANK_SIZE  0x1000
#define CRAM_BANK_SIZE  0x2000
#define VRAM_BANK_SIZE  0x2000

/* DIV Register is incremented at rate of 16384Hz.
 * 4194304 / 16384 = 256 clock cycles for one increment. */
#define DIV_CYCLES          256

/* Serial clock locked to 8192Hz on DMG.
 * 4194304 / (8192 / 8) = 4096 clock cycles for sending 1 byte. */
#define SERIAL_CYCLES		4096

/* Calculating VSYNC. */
#define DMG_CLOCK_FREQ		4194304.0
#define SCREEN_REFRESH_CYCLES	70224.0
#define VERTICAL_SYNC		(DMG_CLOCK_FREQ/SCREEN_REFRESH_CYCLES)

/* STAT register masks */
#define STAT_LYC_INTR       0x40
#define STAT_MODE_2_INTR    0x20
#define STAT_MODE_1_INTR    0x10
#define STAT_MODE_0_INTR    0x08
#define STAT_LYC_COINC      0x04
#define STAT_MODE           0x03
#define STAT_USER_BITS      0xF8

/* LCDC control masks */
#define LCDC_ENABLE         0x80
#define LCDC_WINDOW_MAP     0x40
#define LCDC_WINDOW_ENABLE  0x20
#define LCDC_TILE_SELECT    0x10
#define LCDC_BG_MAP         0x08
#define LCDC_OBJ_SIZE       0x04
#define LCDC_OBJ_ENABLE     0x02
#define LCDC_BG_ENABLE      0x01

/* LCD characteristics */
#define LCD_LINE_CYCLES     456
#define LCD_MODE_0_CYCLES   0
#define LCD_MODE_2_CYCLES   204
#define LCD_MODE_3_CYCLES   284
#define LCD_VERT_LINES      154
#define LCD_WIDTH           160
#define LCD_HEIGHT          144

/* VRAM Locations */
#define VRAM_TILES_1        (0x8000 - VRAM_ADDR)
#define VRAM_TILES_2        (0x8800 - VRAM_ADDR)
#define VRAM_BMAP_1         (0x9800 - VRAM_ADDR)
#define VRAM_BMAP_2         (0x9C00 - VRAM_ADDR)
#define VRAM_TILES_3        (0x8000 - VRAM_ADDR + VRAM_BANK_SIZE)
#define VRAM_TILES_4        (0x8800 - VRAM_ADDR + VRAM_BANK_SIZE)

/* Interrupt jump addresses */
#define VBLANK_INTR_ADDR    0x0040
#define LCDC_INTR_ADDR      0x0048
#define TIMER_INTR_ADDR     0x0050
#define SERIAL_INTR_ADDR    0x0058
#define CONTROL_INTR_ADDR   0x0060

/* SPRITE controls */
#define NUM_SPRITES         0x28
#define MAX_SPRITES_LINE    0x0A
#define OBJ_PRIORITY        0x80
#define OBJ_FLIP_Y          0x40
#define OBJ_FLIP_X          0x20
#define OBJ_PALETTE         0x10

#define ROM_HEADER_CHECKSUM_LOC	0x014D

#ifndef MIN
	#define MIN(a, b)   ((a) < (b) ? (a) : (b))
#endif

struct cpu_registers_s
{
	/* Combine A and F registers. */
	union
	{
		struct
		{
			/* Define specific bits of Flag register. */
			union
			{
				struct
				{
					uint8_t unused : 4;
					uint8_t c : 1; /* Carry flag. */
					uint8_t h : 1; /* Half carry flag. */
					uint8_t n : 1; /* Add/sub flag. */
					uint8_t z : 1; /* Zero flag. */
				} f_bits;
				uint8_t f;
			};
			uint8_t a;
		};
		uint16_t af;
	};

	union
	{
		struct
		{
			uint8_t c;
			uint8_t b;
		};
		uint16_t bc;
	};

	union
	{
		struct
		{
			uint8_t e;
			uint8_t d;
		};
		uint16_t de;
	};

	union
	{
		struct
		{
			uint8_t l;
			uint8_t h;
		};
		uint16_t hl;
	};

	uint16_t sp; /* Stack pointer */
	uint16_t pc; /* Program counter */
};

struct count_s
{
	uint16_t lcd_count;		/* LCD Timing */
	uint16_t div_count;		/* Divider Register Counter */
	uint16_t tima_count;	/* Timer Counter */
	uint16_t serial_count;
};

struct gb_registers_s
{
	/* TODO: Sort variables in address order. */
	/* Timing */
	uint8_t TIMA, TMA, DIV;
	union
	{
		struct
		{
			uint8_t tac_rate : 2;	/* Input clock select */
			uint8_t tac_enable : 1;	/* Timer enable */
			uint8_t unused : 5;
		};
		uint8_t TAC;
	};

	/* LCD */
	uint8_t LCDC;
	uint8_t STAT;
	uint8_t SCY;
	uint8_t SCX;
	uint8_t LY;
	uint8_t LYC;
	uint8_t DMA;
	uint8_t BGP;
	uint8_t OBP0;
	uint8_t OBP1;
	uint8_t WY;
	uint8_t WX;

	/* Joypad info. */
	uint8_t P1;

	/* Serial data. */
	uint8_t SB;
	uint8_t SC;

	/* Interrupt flag. */
	uint8_t IF;

	/* Interrupt enable. */
	uint8_t IE;
};

#if ENABLE_LCD
	/* Bit mask for the shade of pixel to display */
	#define LCD_COLOUR	0x03
	/**
	* Bit mask for whether a pixel is OBJ0, OBJ1, or BG. Each may have a different
	* palette when playing a DMG game on CGB.
	*/
	#define LCD_PALETTE_OBJ	0x10
	#define LCD_PALETTE_BG	0x20
	/**
	* Bit mask for the two bits listed above.
	* LCD_PALETTE_ALL == 0b00 --> OBJ0
	* LCD_PALETTE_ALL == 0b01 --> OBJ1
	* LCD_PALETTE_ALL == 0b10 --> BG
	* LCD_PALETTE_ALL == 0b11 --> NOT POSSIBLE
	*/
	#define LCD_PALETTE_ALL 0x30
#endif

/**
 * Errors that may occur during emulation.
 */
enum gb_error_e
{
	GB_UNKNOWN_ERROR,
	GB_INVALID_OPCODE,
	GB_INVALID_READ,
	GB_INVALID_WRITE,

	GB_INVALID_MAX
};

/**
 * Errors that may occur during library initialisation.
 */
enum gb_init_error_e
{
	GB_INIT_NO_ERROR,
	GB_INIT_CARTRIDGE_UNSUPPORTED,
	GB_INIT_INVALID_CHECKSUM
};

/**
 * Emulator context.
 *
 * Only values within the `direct` struct may be modified directly by the
 * front-end implementation. Other variables must not be modified.
 */
struct gb_s
{
	/**
	 * Return byte from ROM at given address.
	 *
	 * \param gb_s	emulator context
	 * \param addr	address
	 * \return		byte at address in ROM
	 */
	uint8_t (*gb_rom_read)(struct gb_s*, const uint_fast32_t addr);

	/**
	 * Return byte from cart RAM at given address.
	 *
	 * \param gb_s	emulator context
	 * \param addr	address
	 * \return		byte at address in RAM
	 */
	uint8_t (*gb_cart_ram_read)(struct gb_s*, const uint_fast32_t addr);

	/**
	 * Write byte to cart RAM at given address.
	 *
	 * \param gb_s	emulator context
	 * \param addr	address
	 * \param val	value to write to address in RAM
	 */
	void (*gb_cart_ram_write)(struct gb_s*, const uint_fast32_t addr,
				  const uint8_t val);

	/**
	 * Notify front-end of error.
	 *
	 * \param gb_s			emulator context
	 * \param gb_error_e	error code
	 * \param val			arbitrary value related to error
	 */
	void (*gb_error)(struct gb_s*, const enum gb_error_e, const uint16_t val);

	/* Transmit one byte and return the received byte. */
	uint8_t (*gb_serial_transfer)(struct gb_s*, const uint8_t);

	struct
	{
		unsigned int	gb_halt;
		unsigned int	gb_ime;
		unsigned int	gb_bios_enable;
		unsigned int	gb_frame; /* New frame drawn. */
		enum
		{
			LCD_HBLANK = 0,
			LCD_VBLANK = 1,
			LCD_SEARCH_OAM = 2,
			LCD_TRANSFER = 3
		}				lcd_mode;
	};

	/* Cartridge information:
	 * Memory Bank Controller (MBC) type. */
	uint32_t mbc;
	/* Whether the MBC has internal RAM. */
	uint32_t cart_ram;
	/* Number of ROM banks in cartridge. */
	uint32_t num_rom_banks;
	/* Number of RAM banks in cartridge. */
	uint32_t num_ram_banks;

	uint32_t selected_rom_bank;
  uint32_t rom_bank_addr;

	/* WRAM and VRAM bank selection not available. */
	uint32_t cart_ram_bank;
	uint32_t enable_cart_ram;
	/* Cartridge ROM/RAM mode select. */
	uint32_t cart_mode_select;
	union
	{
		struct
		{
			uint32_t sec;
			uint32_t min;
			uint32_t hour;
			uint32_t yday;
			uint32_t high;
		} rtc_bits;
		uint32_t cart_rtc[5];
	};

	struct cpu_registers_s cpu_reg;
	struct gb_registers_s gb_reg;
	struct count_s counter;

	/* TODO: Allow implementation to allocate WRAM, VRAM and Frame Buffer. */
	uint8_t wram[WRAM_SIZE] __attribute((aligned(4)));
	uint8_t vram[VRAM_SIZE];
	uint8_t hram[HRAM_SIZE];
	uint8_t oam[OAM_SIZE];

	struct
	{
		/**
		 * \param pixels	pixels to draw.
		 * 			Bits 1-0 are the colour to draw.
		 * 			Bits 5-4 are the palette, where:
		 * 				OBJ0 = 0b00,
		 * 				OBJ1 = 0b01,
		 * 				BG = 0b10
		 * 			Other bits are undefined.
		 * 			Bits 5-4 are only required by front-ends
		 * 			which want to use a different colour for
		 * 			different object palettes. This is what
		 * 			the Game Boy Color (CGB) does to DMG
		 * 			games.
		 * \param line		Line to draw pixels on. This is
		 * guaranteed to be between 0-144 inclusive.
		 */

		/* Palettes */
		uint32_t bg_palette[4];
		uint32_t sp_palette[8];

		uint32_t window_clear;
		uint32_t WY; // FIXME: Check requirement

		uint32_t frame_skip_count;
    uint8_t *framebuffer;
	} display;

	/**
	 * Variables that may be modified directly by the front-end.
	 * This method seems to be easier and possibly less overhead than
	 * calling a function to modify these variables each time.
	 *
	 * None of this is thread-safe.
	 */
	struct
	{
		/* Set to enable interlacing. Interlacing will start immediately
		 * (at the next line drawing).
		 */
		unsigned int frame_skip;

		union
		{
			struct
			{
				unsigned int a		: 1;
				unsigned int b		: 1;
				unsigned int select	: 1;
				unsigned int start	: 1;
				unsigned int right	: 1;
				unsigned int left	: 1;
				unsigned int up		: 1;
				unsigned int down	: 1;
			} joypad_bits;
			uint32_t joypad;
		};

		/* Implementation defined data. Set to NULL if not required. */
		void *priv;
	} direct;

  void (*cpu_opcodes[256])(struct gb_s *gb);
  uint32_t cpu_cycles[256];
  uint32_t cpu_sizes[256];

  uint8_t *addr_map[16];
  uint8_t (*addr_read[16])(struct gb_s *gb, const uint_fast16_t addr);
  uint8_t *addr_wmap[16];
  void (*addr_write[16])(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val);

  uint32_t bg_color_map[256];
};

/**
 * Tick the internal RTC by one second.
 */
void gb_tick_rtc(struct gb_s *gb)
{
	/* is timer running? */
	if((gb->cart_rtc[4] & 0x40) == 0)
	{
		if(++gb->rtc_bits.sec == 60)
		{
			gb->rtc_bits.sec = 0;

			if(++gb->rtc_bits.min == 60)
			{
				gb->rtc_bits.min = 0;

				if(++gb->rtc_bits.hour == 24)
				{
					gb->rtc_bits.hour = 0;

					if(++gb->rtc_bits.yday == 0)
					{
						if(gb->rtc_bits.high & 1)  /* Bit 8 of days*/
						{
							gb->rtc_bits.high |= 0x80; /* Overflow bit */
						}

						gb->rtc_bits.high ^= 1;
					}
				}
			}
		}
	}
}

/**
 * Set initial values in RTC.
 * Should be called after gb_init().
 */
void gb_set_rtc(struct gb_s *gb, const struct tm * const time)
{
	gb->cart_rtc[0] = time->tm_sec;
	gb->cart_rtc[1] = time->tm_min;
	gb->cart_rtc[2] = time->tm_hour;
	gb->cart_rtc[3] = time->tm_yday & 0xFF; /* Low 8 bits of day counter. */
	gb->cart_rtc[4] = time->tm_yday >> 8; /* High 1 bit of day counter. */
}

void __update_addr_map(struct gb_s *gb);

uint8_t __cart_ram_read(struct gb_s *gb, const uint_fast16_t addr) {
  if(gb->cart_ram && gb->enable_cart_ram)
  {
    if(gb->mbc == 3 && gb->cart_ram_bank >= 0x08)
      return gb->cart_rtc[gb->cart_ram_bank - 0x08];
    else if((gb->cart_mode_select || gb->mbc != 1) &&
        gb->cart_ram_bank < gb->num_ram_banks)
    {
      return gb->gb_cart_ram_read(gb, addr - CART_RAM_ADDR +
                (gb->cart_ram_bank * CRAM_BANK_SIZE));
    }
    else
      return gb->gb_cart_ram_read(gb, addr - CART_RAM_ADDR);
  }

  return 0;
}

uint8_t __io_read(struct gb_s *gb, const uint_fast16_t addr) {
  if(addr < OAM_ADDR)
    return gb->wram[addr - ECHO_ADDR];

  if(addr < UNUSED_ADDR)
    return gb->oam[addr - OAM_ADDR];

  /* Unusable memory area. Reading from this area returns 0.*/
  if(addr < IO_ADDR)
    return 0xFF;

  /* HRAM */
  if(HRAM_ADDR <= addr && addr < INTR_EN_ADDR)
    return gb->hram[addr - HRAM_ADDR];

  if((addr >= 0xFF06) && (addr <= 0xFF3F))
  {
#ifdef ENABLE_SOUND
    return audio_read(addr);
#else
    return 0;
#endif
  }

  /* IO and Interrupts. */
  switch(addr & 0xFF)
  {
  /* IO Registers */
  case 0x00:
    return 0xC0 | gb->gb_reg.P1;

  case 0x01:
    return gb->gb_reg.SB;

  case 0x02:
    return gb->gb_reg.SC;

  /* Timer Registers */
  case 0x04:
    return gb->gb_reg.DIV;

  case 0x05:
    return gb->gb_reg.TIMA;

  /* Interrupt Flag Register */
  case 0x0F:
    return gb->gb_reg.IF;

  /* LCD Registers */
  case 0x40:
    return gb->gb_reg.LCDC;

  case 0x41:
    return (gb->gb_reg.STAT & STAT_USER_BITS) |
            (gb->gb_reg.LCDC & LCDC_ENABLE ? gb->lcd_mode : LCD_VBLANK);

  case 0x42:
    return gb->gb_reg.SCY;

  case 0x43:
    return gb->gb_reg.SCX;

  case 0x44:
    return gb->gb_reg.LY;

  case 0x45:
    return gb->gb_reg.LYC;

  /* DMA Register */
  case 0x46:
    return gb->gb_reg.DMA;

  /* DMG Palette Registers */
  case 0x47:
    return gb->gb_reg.BGP;

  case 0x48:
    return gb->gb_reg.OBP0;

  case 0x49:
    return gb->gb_reg.OBP1;

  /* Window Position Registers */
  case 0x4A:
    return gb->gb_reg.WY;

  case 0x4B:
    return gb->gb_reg.WX;

  /* Interrupt Enable Register */
  case 0xFF:
    return gb->gb_reg.IE;

  /* Unused registers return 1 */
  default:
    return 0xFF;
  }
}

//uint32_t stats[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//uint32_t statcnt = 0;

/**
 * Internal function used to read bytes.
 */
uint8_t __gb_read(struct gb_s *gb, const uint_fast16_t addr)
{
  uint32_t baddr = addr >> 12;
  uint8_t *paddr = gb->addr_map[baddr];
/*if (addr == gb->cpu_reg.pc)
  stats[baddr]++;
if (++statcnt == 10000000) {
  statcnt = 0;
  for (int i = 0; i < 16; i++)
    printf("%d %d\n", i, stats[i]);
}*/
  if (paddr != NULL)
    return paddr[addr];
  uint8_t (*fn)(struct gb_s *gb, const uint_fast16_t addr) = gb->addr_read[baddr];
  return fn(gb, addr);
}

inline uint32_t __gb_read16(struct gb_s *gb, const uint_fast16_t addr)
{
  uint32_t baddr = addr >> 12;
  uint8_t *paddr = gb->addr_map[baddr];

  if (paddr != NULL)
    switch (addr & 3) {
      case 0:
        return *(uint32_t *)&paddr[addr];
      case 1:
        return (*(uint32_t *)&paddr[addr - 1]) >> 8;
      case 2:
        return ((*(uint32_t *)&paddr[addr - 2]) >> 16);
      case 3:
        return ((*(uint32_t *)&paddr[addr - 3]) >> 24) | ((*(uint32_t *)&paddr[addr + 1]) << 8);
    }

  uint8_t (*fn)(struct gb_s *gb, const uint_fast16_t addr) = gb->addr_read[baddr];
  return fn(gb, addr) | (fn(gb, addr + 1) << 8);
}

void __io_write(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  if(addr < OAM_ADDR)
  {
    gb->wram[addr - ECHO_ADDR] = val;
    return;
  }

  if(addr < UNUSED_ADDR)
  {
    gb->oam[addr - OAM_ADDR] = val;
    return;
  }

  /* Unusable memory area. */
  if(addr < IO_ADDR)
    return;

  if(HRAM_ADDR <= addr && addr < INTR_EN_ADDR)
  {
    gb->hram[addr - HRAM_ADDR] = val;
    return;
  }

  if((addr >= 0xFF06) && (addr <= 0xFF3F))
  {
#ifdef ENABLE_SOUND
    audio_write(addr, val);
#endif
    return;
  }

  /* IO and Interrupts. */
  switch(addr & 0xFF)
  {
  /* Joypad */
  case 0x00:
    /* Only bits 5 and 4 are R/W.
      * The lower bits are overwritten later, and the two most
      * significant bits are unused. */
    gb->gb_reg.P1 = val;

    /* Direction keys selected */
    if((gb->gb_reg.P1 & 0b010000) == 0)
      gb->gb_reg.P1 |= (gb->direct.joypad >> 4);
    /* Button keys selected */
    else
      gb->gb_reg.P1 |= (gb->direct.joypad & 0x0F);

    return;

  /* Serial */
  case 0x01:
    gb->gb_reg.SB = val;
    return;

  case 0x02:
    gb->gb_reg.SC = val;
    return;

  /* Timer Registers */
  case 0x04:
    gb->gb_reg.DIV = 0x00;
    return;

  case 0x05:
    gb->gb_reg.TIMA = val;
    return;

  /* Interrupt Flag Register */
  case 0x0F:
    gb->gb_reg.IF = (val | 0b11100000);
    return;

  /* LCD Registers */
  case 0x40:
    if ((gb->gb_reg.LCDC & LCDC_ENABLE) == 0 && (val & LCDC_ENABLE)) {
      gb->counter.lcd_count = 0;
    }
    gb->gb_reg.LCDC = val;

    /* LY fixed to 0 when LCD turned off. */
    if((gb->gb_reg.LCDC & LCDC_ENABLE) == 0)
    {
      /* Do not turn off LCD outside of VBLANK. This may
        * happen due to poor timing in this emulator. */
      if(gb->lcd_mode != LCD_VBLANK)
      {
        gb->gb_reg.LCDC |= LCDC_ENABLE;
        return;
      }

      gb->gb_reg.STAT = (gb->gb_reg.STAT & ~0x03) | LCD_VBLANK;
      gb->gb_reg.LY = 0;
      gb->counter.lcd_count = 0;
    }
    return;

  case 0x41:
    gb->gb_reg.STAT = (val & 0b01111000);
    return;

  case 0x42:
    gb->gb_reg.SCY = val;
    return;

  case 0x43:
    gb->gb_reg.SCX = val;
    return;

  /* LY (0xFF44) is read only. */
  case 0x45:
    gb->gb_reg.LYC = val;
    return;

  /* DMA Register */
  case 0x46:
    gb->gb_reg.DMA = (val % 0xF1);

    for(uint8_t i = 0; i < OAM_SIZE; i++)
      gb->oam[i] = __gb_read(gb, (gb->gb_reg.DMA << 8) + i);

    return;

  /* DMG Palette Registers */
  case 0x47:
    gb->gb_reg.BGP = val;
    gb->display.bg_palette[0] = (gb->gb_reg.BGP & 0x03);
    gb->display.bg_palette[1] = (gb->gb_reg.BGP >> 2) & 0x03;
    gb->display.bg_palette[2] = (gb->gb_reg.BGP >> 4) & 0x03;
    gb->display.bg_palette[3] = (gb->gb_reg.BGP >> 6) & 0x03;
    return;

  case 0x48:
    gb->gb_reg.OBP0 = val;
    gb->display.sp_palette[0] = (gb->gb_reg.OBP0 & 0x03);
    gb->display.sp_palette[1] = (gb->gb_reg.OBP0 >> 2) & 0x03;
    gb->display.sp_palette[2] = (gb->gb_reg.OBP0 >> 4) & 0x03;
    gb->display.sp_palette[3] = (gb->gb_reg.OBP0 >> 6) & 0x03;
    return;

  case 0x49:
    gb->gb_reg.OBP1 = val;
    gb->display.sp_palette[4] = (gb->gb_reg.OBP1 & 0x03);
    gb->display.sp_palette[5] = (gb->gb_reg.OBP1 >> 2) & 0x03;
    gb->display.sp_palette[6] = (gb->gb_reg.OBP1 >> 4) & 0x03;
    gb->display.sp_palette[7] = (gb->gb_reg.OBP1 >> 6) & 0x03;
    return;

  /* Window Position Registers */
  case 0x4A:
    gb->gb_reg.WY = val;
    return;

  case 0x4B:
    gb->gb_reg.WX = val;
    return;

  /* Turn off boot ROM */
  case 0x50:
    gb->gb_bios_enable = 0;
    return;

  /* Interrupt Enable Register */
  case 0xFF:
    gb->gb_reg.IE = val;
    return;
  }
}

void __mbc_write01(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val)
{
  if(gb->mbc == 2 && addr & 0x10)
    return;
  else if(gb->mbc > 0 && gb->cart_ram)
    gb->enable_cart_ram = ((val & 0x0F) == 0x0A);
}

void __mbc_write3(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val)
{
  if(gb->mbc == 1)
  {
    //selected_rom_bank = val & 0x7;
    gb->selected_rom_bank = (val & 0x1F) | (gb->selected_rom_bank & 0x60);

    if((gb->selected_rom_bank & 0x1F) == 0x00)
      gb->selected_rom_bank++;
  }
  else if(gb->mbc == 2 && addr & 0x10)
  {
    gb->selected_rom_bank = val & 0x0F;

    if(!gb->selected_rom_bank)
      gb->selected_rom_bank++;
  }
  else if(gb->mbc == 3)
  {
    gb->selected_rom_bank = val & 0x7F;

    if(!gb->selected_rom_bank)
      gb->selected_rom_bank++;
  }
  else if(gb->mbc == 5)
    gb->selected_rom_bank = (val & 0x01) << 8 | (gb->selected_rom_bank & 0xFF);

  gb->selected_rom_bank = gb->selected_rom_bank % gb->num_rom_banks;
  __update_addr_map(gb);
}

void __mbc_write2(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val)
{
  if(gb->mbc == 5)
  {
    gb->selected_rom_bank = (gb->selected_rom_bank & 0x100) | val;
    gb->selected_rom_bank =
      gb->selected_rom_bank % gb->num_rom_banks;
    return;
  }
  __mbc_write3(gb, addr, val);
}

void __mbc_write45(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val)
{
  if(gb->mbc == 1)
  {
    gb->cart_ram_bank = (val & 3);
    gb->selected_rom_bank = ((val & 3) << 5) | (gb->selected_rom_bank & 0x1F);
    gb->selected_rom_bank = gb->selected_rom_bank % gb->num_rom_banks;
    __update_addr_map(gb);
  }
  else if(gb->mbc == 3)
    gb->cart_ram_bank = val;
  else if(gb->mbc == 5)
    gb->cart_ram_bank = (val & 0x0F);
}

void __mbc_write67(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val)
{
  gb->cart_mode_select = (val & 1);
}

void __cart_ram_write(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val)
{
  if(gb->cart_ram && gb->enable_cart_ram)
  {
    if(gb->mbc == 3 && gb->cart_ram_bank >= 0x08)
      gb->cart_rtc[gb->cart_ram_bank - 0x08] = val;
    else if(gb->cart_mode_select &&
        gb->cart_ram_bank < gb->num_ram_banks)
    {
      gb->gb_cart_ram_write(gb,
                addr - CART_RAM_ADDR + (gb->cart_ram_bank * CRAM_BANK_SIZE), val);
    }
    else if(gb->num_ram_banks)
      gb->gb_cart_ram_write(gb, addr - CART_RAM_ADDR, val);
  }
}

/**
 * Internal function used to write bytes.
 */
void __gb_write(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val)
{
  uint32_t baddr = addr >> 12;
  uint8_t *paddr = gb->addr_wmap[baddr];

  if (paddr != NULL) {
    paddr[addr] = val;
    return;
  }
  void (*fn)(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) = gb->addr_write[baddr];
  fn(gb, addr, val);
}

void __update_addr_map(struct gb_s *gb) {
  if(gb->mbc == 1 && gb->cart_mode_select)
    gb->rom_bank_addr = ((gb->selected_rom_bank & 0x1F) - 1) * ROM_BANK_SIZE;
  else
    gb->rom_bank_addr = (gb->selected_rom_bank - 1) * ROM_BANK_SIZE;
}

uint8_t gb_rom_bank_read(struct gb_s *gb, const uint_fast32_t addr);

void __init_addr_map(struct gb_s *gb) {
  memset(gb->addr_map, 0, sizeof(gb->addr_map));
  gb->addr_map[0] = gb->addr_map[1] = gb->addr_map[2] = gb->addr_map[3] = rom_buffer;
  gb->addr_map[8] = gb->vram - VRAM_ADDR;
  gb->addr_map[9] = gb->vram - VRAM_ADDR;
  gb->addr_map[12] = gb->wram - WRAM_0_ADDR;
  gb->addr_map[13] = gb->wram - WRAM_1_ADDR + WRAM_BANK_SIZE;
  gb->addr_map[14] = gb->wram - ECHO_ADDR;
  __update_addr_map(gb);

  memset(gb->addr_read, 0, sizeof(gb->addr_read));
  gb->addr_read[4] = gb->addr_read[5] = gb->addr_read[6] = gb->addr_read[7] = gb_rom_bank_read;
  gb->addr_read[10] = gb->addr_read[11] = __cart_ram_read;
  gb->addr_read[15] = __io_read;

  memset(gb->addr_wmap, 0, sizeof(gb->addr_wmap));
  gb->addr_wmap[8] = gb->vram - VRAM_ADDR;
  gb->addr_wmap[9] = gb->vram - VRAM_ADDR;
  gb->addr_wmap[12] = gb->wram - WRAM_0_ADDR;
  gb->addr_wmap[13] = gb->wram - WRAM_1_ADDR + WRAM_BANK_SIZE;
  gb->addr_wmap[14] = gb->wram - ECHO_ADDR;

  memset(gb->addr_write, 0, sizeof(gb->addr_write));
  gb->addr_write[0] = gb->addr_write[1] = __mbc_write01;
  gb->addr_write[2] = __mbc_write2;
  gb->addr_write[3] = __mbc_write3;
  gb->addr_write[4] = gb->addr_write[5] = __mbc_write45;
  gb->addr_write[6] = gb->addr_write[7] = __mbc_write67;
  gb->addr_write[10] = gb->addr_write[11] = __cart_ram_write;
  gb->addr_write[15] = __io_write;
}

/**
 * Resets the context, and initialises startup values.
 */
void gb_reset(struct gb_s *gb)
{
  __init_addr_map(gb);
	gb->gb_halt = 0;
	gb->gb_ime = 1;
	gb->gb_bios_enable = 0;
	gb->lcd_mode = LCD_HBLANK;

	/* Initialise MBC values. */
	gb->selected_rom_bank = 1;
	gb->cart_ram_bank = 0;
	gb->enable_cart_ram = 0;
	gb->cart_mode_select = 0;

	/* Initialise CPU registers as though a DMG. */
	gb->cpu_reg.af = 0x01B0;
	gb->cpu_reg.bc = 0x0013;
	gb->cpu_reg.de = 0x00D8;
	gb->cpu_reg.hl = 0x014D;
	gb->cpu_reg.sp = 0xFFFE;
	/* TODO: Add BIOS support. */
	gb->cpu_reg.pc = 0x0100;

	gb->counter.lcd_count = 0;
	gb->counter.div_count = 0;
	gb->counter.tima_count = 0;
	gb->counter.serial_count = 0;

	gb->gb_reg.TIMA      = 0x00;
	gb->gb_reg.TMA       = 0x00;
	gb->gb_reg.TAC       = 0xF8;
	gb->gb_reg.DIV       = 0xAC;

	gb->gb_reg.IF        = 0xE1;

	gb->gb_reg.LCDC      = 0x91;
	gb->gb_reg.SCY       = 0x00;
	gb->gb_reg.SCX       = 0x00;
	gb->gb_reg.LYC       = 0x00;

	/* Appease valgrind for invalid reads and unconditional jumps. */
	gb->gb_reg.SC = 0x7E;
	gb->gb_reg.STAT = 0;
	gb->gb_reg.LY = 0;

	__gb_write(gb, 0xFF47, 0xFC);    // BGP
	__gb_write(gb, 0xFF48, 0xFF);    // OBJP0
	__gb_write(gb, 0xFF49, 0x0F);    // OBJP1
	gb->gb_reg.WY        = 0x00;
	gb->gb_reg.WX        = 0x00;
	gb->gb_reg.IE        = 0x00;

	gb->direct.joypad = 0xFF;
	gb->gb_reg.P1 = 0xCF;
}

void __gb_execute_cb(struct gb_s *gb, uint32_t opd)
{
	uint8_t cbop = (opd);
	uint8_t r = (cbop & 0x7);
	uint8_t b = (cbop >> 3) & 0x7;
	uint8_t d = (cbop >> 3) & 0x1;
	uint8_t val;
	uint8_t writeback = 1;

	/* Add an additional 8 cycles to these sets of instructions. */
	switch(cbop & 0x0F)
	{
	case 0x06:
	case 0x0E:
		gb->counter.div_count += 8;
	}

	switch(r)
	{
	case 0:
		val = gb->cpu_reg.b;
		break;

	case 1:
		val = gb->cpu_reg.c;
		break;

	case 2:
		val = gb->cpu_reg.d;
		break;

	case 3:
		val = gb->cpu_reg.e;
		break;

	case 4:
		val = gb->cpu_reg.h;
		break;

	case 5:
		val = gb->cpu_reg.l;
		break;

	case 6:
		val = __gb_read(gb, gb->cpu_reg.hl);
		break;

	/* Only values 0-7 are possible here, so we make the final case
	 * default to satisfy -Wmaybe-uninitialized warning. */
	default:
		val = gb->cpu_reg.a;
		break;
	}

	/* TODO: Find out WTF this is doing. */
	switch(cbop >> 6)
	{
	case 0x0:
		cbop = (cbop >> 4) & 0x3;

		switch(cbop)
		{
		case 0x0: /* RdC R */
		case 0x1: /* Rd R */
			if(d) /* RRC R / RR R */
			{
				uint8_t temp = val;
				val = (val >> 1);
				val |= cbop ? (gb->cpu_reg.f_bits.c << 7) : (temp << 7);
				gb->cpu_reg.f_bits.z = (val == 0x00);
				gb->cpu_reg.f_bits.n = 0;
				gb->cpu_reg.f_bits.h = 0;
				gb->cpu_reg.f_bits.c = (temp & 0x01);
			}
			else /* RLC R / RL R */
			{
				uint8_t temp = val;
				val = (val << 1);
				val |= cbop ? gb->cpu_reg.f_bits.c : (temp >> 7);
				gb->cpu_reg.f_bits.z = (val == 0x00);
				gb->cpu_reg.f_bits.n = 0;
				gb->cpu_reg.f_bits.h = 0;
				gb->cpu_reg.f_bits.c = (temp >> 7);
			}

			break;

		case 0x2:
			if(d) /* SRA R */
			{
				gb->cpu_reg.f_bits.c = val & 0x01;
				val = (val >> 1) | (val & 0x80);
				gb->cpu_reg.f_bits.z = (val == 0x00);
				gb->cpu_reg.f_bits.n = 0;
				gb->cpu_reg.f_bits.h = 0;
			}
			else /* SLA R */
			{
				gb->cpu_reg.f_bits.c = (val >> 7);
				val = val << 1;
				gb->cpu_reg.f_bits.z = (val == 0x00);
				gb->cpu_reg.f_bits.n = 0;
				gb->cpu_reg.f_bits.h = 0;
			}

			break;

		case 0x3:
			if(d) /* SRL R */
			{
				gb->cpu_reg.f_bits.c = val & 0x01;
				val = val >> 1;
				gb->cpu_reg.f_bits.z = (val == 0x00);
				gb->cpu_reg.f_bits.n = 0;
				gb->cpu_reg.f_bits.h = 0;
			}
			else /* SWAP R */
			{
				uint8_t temp = (val >> 4) & 0x0F;
				temp |= (val << 4) & 0xF0;
				val = temp;
				gb->cpu_reg.f_bits.z = (val == 0x00);
				gb->cpu_reg.f_bits.n = 0;
				gb->cpu_reg.f_bits.h = 0;
				gb->cpu_reg.f_bits.c = 0;
			}

			break;
		}

		break;

	case 0x1: /* BIT B, R */
		gb->cpu_reg.f_bits.z = !((val >> b) & 0x1);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = 1;
		writeback = 0;
		break;

	case 0x2: /* RES B, R */
		val &= (0xFE << b) | (0xFF >> (8 - b));
		break;

	case 0x3: /* SET B, R */
		val |= (0x1 << b);
		break;
	}

	if(writeback)
	{
		switch(r)
		{
		case 0:
			gb->cpu_reg.b = val;
			break;

		case 1:
			gb->cpu_reg.c = val;
			break;

		case 2:
			gb->cpu_reg.d = val;
			break;

		case 3:
			gb->cpu_reg.e = val;
			break;

		case 4:
			gb->cpu_reg.h = val;
			break;

		case 5:
			gb->cpu_reg.l = val;
			break;

		case 6:
			__gb_write(gb, gb->cpu_reg.hl, val);
			break;

		case 7:
			gb->cpu_reg.a = val;
			break;
		}
	}
}

#if ENABLE_LCD
void __gb_draw_line(struct gb_s *gb)
{
  if (gb->gb_reg.LY < 8 || gb->gb_reg.LY > 127 + 8)
    return;

	if(gb->display.frame_skip_count != 0)
		return;

	uint8_t *pixels = gb->display.framebuffer + 160 * (gb->gb_reg.LY - 8);

	/* If background is enabled, draw it. */
	if(gb->gb_reg.LCDC & LCDC_BG_ENABLE)
	{
		/* Calculate current background line to draw. Constant because
		 * this function draws only this one line each time it is
		 * called. */
		const uint8_t bg_y = gb->gb_reg.LY + gb->gb_reg.SCY;

		/* Get selected background map address for first tile
		 * corresponding to current line.
		 * 0x20 (32) is the width of a background tile, and the bit
		 * shift is to calculate the address. */
		const uint16_t bg_map =
			((gb->gb_reg.LCDC & LCDC_BG_MAP) ?
			 VRAM_BMAP_2 : VRAM_BMAP_1)
			+ (bg_y >> 3) * 0x20;

		/* The displays (what the player sees) X coordinate, drawn right
		 * to left. */
		uint8_t disp_x = LCD_WIDTH - 1;

		/* The X coordinate to begin drawing the background at. */
		uint8_t bg_x = disp_x + gb->gb_reg.SCX;

		/* FIXME: Get tile index for current background tile? */
		uint8_t idx = gb->vram[bg_map + (bg_x >> 3)];
		/* FIXME: Y coordinate of tile pixel to draw? */
		const uint8_t py = (bg_y & 0x07);
		/* FIXME: X coordinate of tile pixel to draw? */
		uint8_t px = 7 - (bg_x & 0x07);

		uint16_t tile;

		/* Select addressing mode. */
		if(gb->gb_reg.LCDC & LCDC_TILE_SELECT)
			tile = VRAM_TILES_1 + idx * 0x10;
		else
			tile = VRAM_TILES_2 + ((idx + 0x80) % 0x100) * 0x10;

		tile += 2 * py;

		/* fetch first tile */
		uint8_t t1 = gb->vram[tile] >> px;
		uint8_t t2 = gb->vram[tile + 1] >> px;

		for(; disp_x != 0xFF; disp_x--)
		{
			if(px == 8)
			{
				/* fetch next tile */
				px = 0;
				bg_x = disp_x + gb->gb_reg.SCX;
				idx = gb->vram[bg_map + (bg_x >> 3)];

				if(gb->gb_reg.LCDC & LCDC_TILE_SELECT)
					tile = VRAM_TILES_1 + idx * 0x10;
				else
					tile = VRAM_TILES_2 + ((idx + 0x80) % 0x100) * 0x10;

				tile += 2 * py;
				t1 = gb->vram[tile];
				t2 = gb->vram[tile + 1];
			}

			/* copy background */
			uint8_t c = (t1 & 0x1) | ((t2 & 0x1) << 1);
			pixels[disp_x] = gb->display.bg_palette[c];
			pixels[disp_x] |= LCD_PALETTE_BG;
			t1 = t1 >> 1;
			t2 = t2 >> 1;
			px++;
		}
		/*for(; disp_x != 0xFF; disp_x--)
		{
      bg_x = disp_x + gb->gb_reg.SCX;
      idx = gb->vram[bg_map + (bg_x >> 3)];

      if(gb->gb_reg.LCDC & LCDC_TILE_SELECT)
        tile = VRAM_TILES_1 + idx * 0x10;
      else
        tile = VRAM_TILES_2 + ((idx + 0x80) & 255) << 4;

      tile += 2 * py;
      t1 = gb->vram[tile];
      t2 = gb->vram[tile + 1];

      uint8_t c1 = (t1 & 0x0F) | (t2 << 4);
      pixels[disp_x] = gb->bg_color_map[c1];
      uint8_t c2 = (t1 >> 4) | (t2 & 0xF0);
      pixels[disp_x - 4] = gb->bg_color_map[c2];
		}*/

	}

	/* draw window */
	if(gb->gb_reg.LCDC & LCDC_WINDOW_ENABLE
			&& gb->gb_reg.LY >= gb->display.WY
			&& gb->gb_reg.WX <= 166)
	{
		/* Calculate Window Map Address. */
		uint16_t win_line = (gb->gb_reg.LCDC & LCDC_WINDOW_MAP) ?
				    VRAM_BMAP_2 : VRAM_BMAP_1;
		win_line += (gb->display.window_clear >> 3) * 0x20;

		uint8_t disp_x = LCD_WIDTH - 1;
		uint8_t win_x = disp_x - gb->gb_reg.WX + 7;

		// look up tile
		uint8_t py = gb->display.window_clear & 0x07;
		uint8_t px = 7 - (win_x & 0x07);
		uint8_t idx = gb->vram[win_line + (win_x >> 3)];

		uint16_t tile;

		if(gb->gb_reg.LCDC & LCDC_TILE_SELECT)
			tile = VRAM_TILES_1 + idx * 0x10;
		else
			tile = VRAM_TILES_2 + ((idx + 0x80) % 0x100) * 0x10;

		tile += 2 * py;

		// fetch first tile
		uint8_t t1 = gb->vram[tile] >> px;
		uint8_t t2 = gb->vram[tile + 1] >> px;

		// loop & copy window
		uint8_t end = (gb->gb_reg.WX < 7 ? 0 : gb->gb_reg.WX - 7) - 1;

		for(; disp_x != end; disp_x--)
		{
			if(px == 8)
			{
				// fetch next tile
				px = 0;
				win_x = disp_x - gb->gb_reg.WX + 7;
				idx = gb->vram[win_line + (win_x >> 3)];

				if(gb->gb_reg.LCDC & LCDC_TILE_SELECT)
					tile = VRAM_TILES_1 + idx * 0x10;
				else
					tile = VRAM_TILES_2 + ((idx + 0x80) % 0x100) * 0x10;

				tile += 2 * py;
				t1 = gb->vram[tile];
				t2 = gb->vram[tile + 1];
			}

			// copy window
			uint8_t c = (t1 & 0x1) | ((t2 & 0x1) << 1);
			pixels[disp_x] = gb->display.bg_palette[c];
			pixels[disp_x] |= LCD_PALETTE_BG;
			t1 = t1 >> 1;
			t2 = t2 >> 1;
			px++;
		}

		gb->display.window_clear++; // advance window line
	}

	// draw sprites
	if(gb->gb_reg.LCDC & LCDC_OBJ_ENABLE)
	{
		for(uint8_t s = NUM_SPRITES - 1;
				s != 0xFF /* && count < MAX_SPRITES_LINE */ ;
				s--)
		{
			/* Sprite Y position. */
			uint8_t OY = gb->oam[4 * s + 0];
			/* If sprite isn't on this line, continue. */
			if(gb->gb_reg.LY +
					(gb->gb_reg.LCDC & LCDC_OBJ_SIZE ?
					 0 : 8) >= OY
					|| gb->gb_reg.LY + 16 < OY)
				continue;

			/* Sprite X position. */
			uint8_t OX = gb->oam[4 * s + 1];
			/* Continue if sprite not visible. */
			if(OX == 0 || OX >= 168)
				continue;

			/* Sprite Tile/Pattern Number. */
			uint8_t OT = gb->oam[4 * s + 2]
				     & (gb->gb_reg.LCDC & LCDC_OBJ_SIZE ? 0xFE : 0xFF);
			/* Additional attributes. */
			uint8_t OF = gb->oam[4 * s + 3];

			// y flip
			uint8_t py = gb->gb_reg.LY - OY + 16;

			if(OF & OBJ_FLIP_Y)
				py = (gb->gb_reg.LCDC & LCDC_OBJ_SIZE ? 15 : 7) - py;

			// fetch the tile
			uint8_t t1 = gb->vram[VRAM_TILES_1 + OT * 0x10 + 2 * py];
			uint8_t t2 = gb->vram[VRAM_TILES_1 + OT * 0x10 + 2 * py + 1];

			// handle x flip
			uint8_t dir, start, end, shift;

			if(OF & OBJ_FLIP_X)
			{
				dir = 1;
				start = (OX < 8 ? 0 : OX - 8);
				end = MIN(OX, LCD_WIDTH);
				shift = 8 - OX + start;
			}
			else
			{
				dir = -1;
				start = MIN(OX, LCD_WIDTH) - 1;
				end = (OX < 8 ? 0 : OX - 8) - 1;
				shift = OX - (start + 1);
			}

			// copy tile
			t1 >>= shift;
			t2 >>= shift;

			for(uint8_t disp_x = start; disp_x != end; disp_x += dir)
			{
				uint8_t c = (t1 & 0x1) | ((t2 & 0x1) << 1);
				// check transparency / sprite overlap / background overlap
#if 0

				if(c
						//	&& OX <= fx[disp_x]
						&& !((OF & OBJ_PRIORITY)
						     && ((pixels[disp_x] & 0x3)
							 && fx[disp_x] == 0xFE)))
#else
				if(c && !(OF & OBJ_PRIORITY
						&& pixels[disp_x] & 0x3))
#endif
				{
					/* Set pixel colour. */
					pixels[disp_x] = (OF & OBJ_PALETTE)
							 ? gb->display.sp_palette[c + 4]
							 : gb->display.sp_palette[c];
					/* Set pixel palette (OBJ0 or OBJ1). */
					pixels[disp_x] |= (OF & OBJ_PALETTE);
					/* Deselect BG palette. */
					pixels[disp_x] &= ~LCD_PALETTE_BG;
				}

				t1 = t1 >> 1;
				t2 = t2 >> 1;
			}
		}
	}
}
#endif

uint32_t RWDATA inst_cycles;

void Op00(struct gb_s *gb, uint32_t opd)  { /* NOP */
}

void Op01(struct gb_s *gb, uint32_t opd)  { /* LD BC, imm */
		gb->cpu_reg.bc = opd;
}

void Op02(struct gb_s *gb, uint32_t opd)  { /* LD (BC), A */
		__gb_write(gb, gb->cpu_reg.bc, gb->cpu_reg.a);
}

void Op03(struct gb_s *gb, uint32_t opd)  { /* INC BC */
		gb->cpu_reg.bc++;
}

void Op04(struct gb_s *gb, uint32_t opd)  { /* INC B */
		gb->cpu_reg.b++;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.b == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.b & 0x0F) == 0x00);
}

void Op05(struct gb_s *gb, uint32_t opd)  { /* DEC B */
		gb->cpu_reg.b--;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.b == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.b & 0x0F) == 0x0F);
}

void Op06(struct gb_s *gb, uint32_t opd)  { /* LD B, imm */
		gb->cpu_reg.b = opd;
}

void Op07(struct gb_s *gb, uint32_t opd)  { /* RLCA */
		gb->cpu_reg.a = (gb->cpu_reg.a << 1) | (gb->cpu_reg.a >> 7);
		gb->cpu_reg.f = (gb->cpu_reg.a & 0x01) << 4;
}

void Op08(struct gb_s *gb, uint32_t opd)  { /* LD (imm), SP */
	{
		uint16_t temp = opd;
		__gb_write(gb, temp++, gb->cpu_reg.sp & 0xFF);
		__gb_write(gb, temp, gb->cpu_reg.sp >> 8);
}
	}

void Op09(struct gb_s *gb, uint32_t opd)  { /* ADD HL, BC */
	{
		uint_fast32_t temp = gb->cpu_reg.hl + gb->cpu_reg.bc;
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(temp ^ gb->cpu_reg.hl ^ gb->cpu_reg.bc) & 0x1000 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFFFF0000) ? 1 : 0;
		gb->cpu_reg.hl = (temp & 0x0000FFFF);
}
	}

void Op0A(struct gb_s *gb, uint32_t opd)  { /* LD A, (BC) */
		gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.bc);
}

void Op0B(struct gb_s *gb, uint32_t opd)  { /* DEC BC */
		gb->cpu_reg.bc--;
}

void Op0C(struct gb_s *gb, uint32_t opd)  { /* INC C */
		gb->cpu_reg.c++;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.c == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.c & 0x0F) == 0x00);
}

void Op0D(struct gb_s *gb, uint32_t opd)  { /* DEC C */
		gb->cpu_reg.c--;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.c == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.c & 0x0F) == 0x0F);
}

void Op0E(struct gb_s *gb, uint32_t opd)  { /* LD C, imm */
		gb->cpu_reg.c = (opd);
}

void Op0F(struct gb_s *gb, uint32_t opd)  { /* RRCA */
		gb->cpu_reg.f = (gb->cpu_reg.a & 0x01) << 4;
		gb->cpu_reg.a = (gb->cpu_reg.a >> 1) | (gb->cpu_reg.a << 7);
}

void Op10(struct gb_s *gb, uint32_t opd)  { /* STOP */
		//gb->gb_halt = 1;
}

void Op11(struct gb_s *gb, uint32_t opd)  { /* LD DE, imm */
		gb->cpu_reg.de = (opd);
}

void Op12(struct gb_s *gb, uint32_t opd)  { /* LD (DE), A */
		__gb_write(gb, gb->cpu_reg.de, gb->cpu_reg.a);
}

void Op13(struct gb_s *gb, uint32_t opd)  { /* INC DE */
		gb->cpu_reg.de++;
}

void Op14(struct gb_s *gb, uint32_t opd)  { /* INC D */
		gb->cpu_reg.d++;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.d == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.d & 0x0F) == 0x00);
}

void Op15(struct gb_s *gb, uint32_t opd)  { /* DEC D */
		gb->cpu_reg.d--;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.d == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.d & 0x0F) == 0x0F);
}

void Op16(struct gb_s *gb, uint32_t opd)  { /* LD D, imm */
		gb->cpu_reg.d = (opd);
}

void Op17(struct gb_s *gb, uint32_t opd)  { /* RLA */
	{
		uint8_t temp = gb->cpu_reg.a;
		gb->cpu_reg.a = (gb->cpu_reg.a << 1) | gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f = ((temp >> 7) & 0x01) << 4;
}
	}

void Op18(struct gb_s *gb, uint32_t opd)  { /* JR imm */
	{
		int8_t temp = (int8_t) (opd);
		gb->cpu_reg.pc += temp;
}
	}

void Op19(struct gb_s *gb, uint32_t opd)  { /* ADD HL, DE */
	{
		uint_fast32_t temp = gb->cpu_reg.hl + gb->cpu_reg.de;
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(temp ^ gb->cpu_reg.hl ^ gb->cpu_reg.de) & 0x1000 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFFFF0000) ? 1 : 0;
		gb->cpu_reg.hl = (temp & 0x0000FFFF);
}
	}

void Op1A(struct gb_s *gb, uint32_t opd)  { /* LD A, (DE) */
		gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.de);
}

void Op1B(struct gb_s *gb, uint32_t opd)  { /* DEC DE */
		gb->cpu_reg.de--;
}

void Op1C(struct gb_s *gb, uint32_t opd)  { /* INC E */
		gb->cpu_reg.e++;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.e == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.e & 0x0F) == 0x00);
}

void Op1D(struct gb_s *gb, uint32_t opd)  { /* DEC E */
		gb->cpu_reg.e--;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.e == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.e & 0x0F) == 0x0F);
}

void Op1E(struct gb_s *gb, uint32_t opd)  { /* LD E, imm */
		gb->cpu_reg.e = (opd);
}

void Op1F(struct gb_s *gb, uint32_t opd)  { /* RRA */
	{
		uint8_t temp = gb->cpu_reg.a;
		gb->cpu_reg.a = gb->cpu_reg.a >> 1 | (gb->cpu_reg.f_bits.c << 7);
		gb->cpu_reg.f = (temp & 0x1) << 4;
}
	}

void Op20(struct gb_s *gb, uint32_t opd)  { /* JP NZ, imm */
		if(!gb->cpu_reg.f_bits.z)
		{
			int8_t temp = (int8_t) (opd);
			gb->cpu_reg.pc += temp;
			inst_cycles += 4;
		}
}

void Op21(struct gb_s *gb, uint32_t opd)  { /* LD HL, imm */
		gb->cpu_reg.hl = (opd);
}

void Op22(struct gb_s *gb, uint32_t opd)  { /* LDI (HL), A */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.a);
		gb->cpu_reg.hl++;
}

void Op23(struct gb_s *gb, uint32_t opd)  { /* INC HL */
		gb->cpu_reg.hl++;
}

void Op24(struct gb_s *gb, uint32_t opd)  { /* INC H */
		gb->cpu_reg.h++;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.h == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.h & 0x0F) == 0x00);
}

void Op25(struct gb_s *gb, uint32_t opd)  { /* DEC H */
		gb->cpu_reg.h--;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.h == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.h & 0x0F) == 0x0F);
}

void Op26(struct gb_s *gb, uint32_t opd)  { /* LD H, imm */
		gb->cpu_reg.h = (opd);
}

void Op27(struct gb_s *gb, uint32_t opd)  { /* DAA */
	{
		uint16_t a = gb->cpu_reg.a;

		if(gb->cpu_reg.f_bits.n)
		{
			if(gb->cpu_reg.f_bits.h)
				a = (a - 0x06) & 0xFF;

			if(gb->cpu_reg.f_bits.c)
				a -= 0x60;
		}
		else
		{
			if(gb->cpu_reg.f_bits.h || (a & 0x0F) > 9)
				a += 0x06;

			if(gb->cpu_reg.f_bits.c || a > 0x9F)
				a += 0x60;
		}

		if((a & 0x100) == 0x100)
			gb->cpu_reg.f_bits.c = 1;

		gb->cpu_reg.a = a;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0);
		gb->cpu_reg.f_bits.h = 0;
}
	}

void Op28(struct gb_s *gb, uint32_t opd)  { /* JP Z, imm */
		if(gb->cpu_reg.f_bits.z)
		{
			int8_t temp = (int8_t) (opd);
			gb->cpu_reg.pc += temp;
			inst_cycles += 4;
		}
}

void Op29(struct gb_s *gb, uint32_t opd)  { /* ADD HL, HL */
	{
		uint_fast32_t temp = gb->cpu_reg.hl + gb->cpu_reg.hl;
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = (temp & 0x1000) ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFFFF0000) ? 1 : 0;
		gb->cpu_reg.hl = (temp & 0x0000FFFF);
}
	}

void Op2A(struct gb_s *gb, uint32_t opd)  { /* LD A, (HL+) */
		gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.hl++);
}

void Op2B(struct gb_s *gb, uint32_t opd)  { /* DEC HL */
		gb->cpu_reg.hl--;
}

void Op2C(struct gb_s *gb, uint32_t opd)  { /* INC L */
		gb->cpu_reg.l++;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.l == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.l & 0x0F) == 0x00);
}

void Op2D(struct gb_s *gb, uint32_t opd)  { /* DEC L */
		gb->cpu_reg.l--;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.l == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.l & 0x0F) == 0x0F);
}

void Op2E(struct gb_s *gb, uint32_t opd)  { /* LD L, imm */
		gb->cpu_reg.l = (opd);
}

void Op2F(struct gb_s *gb, uint32_t opd)  { /* CPL */
		gb->cpu_reg.a = ~gb->cpu_reg.a;
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = 1;
}

void Op30(struct gb_s *gb, uint32_t opd)  { /* JP NC, imm */
		if(!gb->cpu_reg.f_bits.c)
		{
			int8_t temp = (int8_t) (opd);
			gb->cpu_reg.pc += temp;
			inst_cycles += 4;
		}
}

void Op31(struct gb_s *gb, uint32_t opd)  { /* LD SP, imm */
		gb->cpu_reg.sp = (opd);
}

void Op32(struct gb_s *gb, uint32_t opd)  { /* LD (HL), A */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.a);
		gb->cpu_reg.hl--;
}

void Op33(struct gb_s *gb, uint32_t opd)  { /* INC SP */
		gb->cpu_reg.sp++;
}

void Op34(struct gb_s *gb, uint32_t opd)  { /* INC (HL) */
	{
		uint8_t temp = __gb_read(gb, gb->cpu_reg.hl) + 1;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((temp & 0x0F) == 0x00);
		__gb_write(gb, gb->cpu_reg.hl, temp);
}
	}

void Op35(struct gb_s *gb, uint32_t opd)  { /* DEC (HL) */
	{
		uint8_t temp = __gb_read(gb, gb->cpu_reg.hl) - 1;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((temp & 0x0F) == 0x0F);
		__gb_write(gb, gb->cpu_reg.hl, temp);
}
	}

void Op36(struct gb_s *gb, uint32_t opd)  { /* LD (HL), imm */
		__gb_write(gb, gb->cpu_reg.hl, (opd));
}

void Op37(struct gb_s *gb, uint32_t opd)  { /* SCF */
		gb->cpu_reg.f &= 0x90;
		gb->cpu_reg.f_bits.c = 1;
}

void Op38(struct gb_s *gb, uint32_t opd)  { /* JP C, imm */
		if(gb->cpu_reg.f_bits.c)
		{
			int8_t temp = (int8_t) (opd);
			gb->cpu_reg.pc += temp;
			inst_cycles += 4;
		}
}

void Op39(struct gb_s *gb, uint32_t opd)  { /* ADD HL, SP */
	{
		uint_fast32_t temp = gb->cpu_reg.hl + gb->cpu_reg.sp;
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			((gb->cpu_reg.hl & 0xFFF) + (gb->cpu_reg.sp & 0xFFF)) & 0x1000 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp & 0x10000 ? 1 : 0;
		gb->cpu_reg.hl = (uint16_t)temp;
}
	}

void Op3A(struct gb_s *gb, uint32_t opd)  { /* LD A, (HL) */
		gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.hl--);
}

void Op3B(struct gb_s *gb, uint32_t opd)  { /* DEC SP */
		gb->cpu_reg.sp--;
}

void Op3C(struct gb_s *gb, uint32_t opd)  { /* INC A */
		gb->cpu_reg.a++;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.a & 0x0F) == 0x00);
}

void Op3D(struct gb_s *gb, uint32_t opd)  { /* DEC A */
		gb->cpu_reg.a--;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.a & 0x0F) == 0x0F);
}

void Op3E(struct gb_s *gb, uint32_t opd)  { /* LD A, imm */
		gb->cpu_reg.a = (opd);
}

void Op3F(struct gb_s *gb, uint32_t opd)  { /* CCF */
		gb->cpu_reg.f &= 0x90;
		gb->cpu_reg.f_bits.c = ~gb->cpu_reg.f_bits.c;
}

void Op40(struct gb_s *gb, uint32_t opd)  { /* LD B, B */
}

void Op41(struct gb_s *gb, uint32_t opd)  { /* LD B, C */
		gb->cpu_reg.b = gb->cpu_reg.c;
}

void Op42(struct gb_s *gb, uint32_t opd)  { /* LD B, D */
		gb->cpu_reg.b = gb->cpu_reg.d;
}

void Op43(struct gb_s *gb, uint32_t opd)  { /* LD B, E */
		gb->cpu_reg.b = gb->cpu_reg.e;
}

void Op44(struct gb_s *gb, uint32_t opd)  { /* LD B, H */
		gb->cpu_reg.b = gb->cpu_reg.h;
}

void Op45(struct gb_s *gb, uint32_t opd)  { /* LD B, L */
		gb->cpu_reg.b = gb->cpu_reg.l;
}

void Op46(struct gb_s *gb, uint32_t opd)  { /* LD B, (HL) */
		gb->cpu_reg.b = __gb_read(gb, gb->cpu_reg.hl);
}

void Op47(struct gb_s *gb, uint32_t opd)  { /* LD B, A */
		gb->cpu_reg.b = gb->cpu_reg.a;
}

void Op48(struct gb_s *gb, uint32_t opd)  { /* LD C, B */
		gb->cpu_reg.c = gb->cpu_reg.b;
}

void Op49(struct gb_s *gb, uint32_t opd)  { /* LD C, C */
}

void Op4A(struct gb_s *gb, uint32_t opd)  { /* LD C, D */
		gb->cpu_reg.c = gb->cpu_reg.d;
}

void Op4B(struct gb_s *gb, uint32_t opd)  { /* LD C, E */
		gb->cpu_reg.c = gb->cpu_reg.e;
}

void Op4C(struct gb_s *gb, uint32_t opd)  { /* LD C, H */
		gb->cpu_reg.c = gb->cpu_reg.h;
}

void Op4D(struct gb_s *gb, uint32_t opd)  { /* LD C, L */
		gb->cpu_reg.c = gb->cpu_reg.l;
}

void Op4E(struct gb_s *gb, uint32_t opd)  { /* LD C, (HL) */
		gb->cpu_reg.c = __gb_read(gb, gb->cpu_reg.hl);
}

void Op4F(struct gb_s *gb, uint32_t opd)  { /* LD C, A */
		gb->cpu_reg.c = gb->cpu_reg.a;
}

void Op50(struct gb_s *gb, uint32_t opd)  { /* LD D, B */
		gb->cpu_reg.d = gb->cpu_reg.b;
}

void Op51(struct gb_s *gb, uint32_t opd)  { /* LD D, C */
		gb->cpu_reg.d = gb->cpu_reg.c;
}

void Op52(struct gb_s *gb, uint32_t opd)  { /* LD D, D */
}

void Op53(struct gb_s *gb, uint32_t opd)  { /* LD D, E */
		gb->cpu_reg.d = gb->cpu_reg.e;
}

void Op54(struct gb_s *gb, uint32_t opd)  { /* LD D, H */
		gb->cpu_reg.d = gb->cpu_reg.h;
}

void Op55(struct gb_s *gb, uint32_t opd)  { /* LD D, L */
		gb->cpu_reg.d = gb->cpu_reg.l;
}

void Op56(struct gb_s *gb, uint32_t opd)  { /* LD D, (HL) */
		gb->cpu_reg.d = __gb_read(gb, gb->cpu_reg.hl);
}

void Op57(struct gb_s *gb, uint32_t opd)  { /* LD D, A */
		gb->cpu_reg.d = gb->cpu_reg.a;
}

void Op58(struct gb_s *gb, uint32_t opd)  { /* LD E, B */
		gb->cpu_reg.e = gb->cpu_reg.b;
}

void Op59(struct gb_s *gb, uint32_t opd)  { /* LD E, C */
		gb->cpu_reg.e = gb->cpu_reg.c;
}

void Op5A(struct gb_s *gb, uint32_t opd)  { /* LD E, D */
		gb->cpu_reg.e = gb->cpu_reg.d;
}

void Op5B(struct gb_s *gb, uint32_t opd)  { /* LD E, E */
}

void Op5C(struct gb_s *gb, uint32_t opd)  { /* LD E, H */
		gb->cpu_reg.e = gb->cpu_reg.h;
}

void Op5D(struct gb_s *gb, uint32_t opd)  { /* LD E, L */
		gb->cpu_reg.e = gb->cpu_reg.l;
}

void Op5E(struct gb_s *gb, uint32_t opd)  { /* LD E, (HL) */
		gb->cpu_reg.e = __gb_read(gb, gb->cpu_reg.hl);
}

void Op5F(struct gb_s *gb, uint32_t opd)  { /* LD E, A */
		gb->cpu_reg.e = gb->cpu_reg.a;
}

void Op60(struct gb_s *gb, uint32_t opd)  { /* LD H, B */
		gb->cpu_reg.h = gb->cpu_reg.b;
}

void Op61(struct gb_s *gb, uint32_t opd)  { /* LD H, C */
		gb->cpu_reg.h = gb->cpu_reg.c;
}

void Op62(struct gb_s *gb, uint32_t opd)  { /* LD H, D */
		gb->cpu_reg.h = gb->cpu_reg.d;
}

void Op63(struct gb_s *gb, uint32_t opd)  { /* LD H, E */
		gb->cpu_reg.h = gb->cpu_reg.e;
}

void Op64(struct gb_s *gb, uint32_t opd)  { /* LD H, H */
}

void Op65(struct gb_s *gb, uint32_t opd)  { /* LD H, L */
		gb->cpu_reg.h = gb->cpu_reg.l;
}

void Op66(struct gb_s *gb, uint32_t opd)  { /* LD H, (HL) */
		gb->cpu_reg.h = __gb_read(gb, gb->cpu_reg.hl);
}

void Op67(struct gb_s *gb, uint32_t opd)  { /* LD H, A */
		gb->cpu_reg.h = gb->cpu_reg.a;
}

void Op68(struct gb_s *gb, uint32_t opd)  { /* LD L, B */
		gb->cpu_reg.l = gb->cpu_reg.b;
}

void Op69(struct gb_s *gb, uint32_t opd)  { /* LD L, C */
		gb->cpu_reg.l = gb->cpu_reg.c;
}

void Op6A(struct gb_s *gb, uint32_t opd)  { /* LD L, D */
		gb->cpu_reg.l = gb->cpu_reg.d;
}

void Op6B(struct gb_s *gb, uint32_t opd)  { /* LD L, E */
		gb->cpu_reg.l = gb->cpu_reg.e;
}

void Op6C(struct gb_s *gb, uint32_t opd)  { /* LD L, H */
		gb->cpu_reg.l = gb->cpu_reg.h;
}

void Op6D(struct gb_s *gb, uint32_t opd)  { /* LD L, L */
}

void Op6E(struct gb_s *gb, uint32_t opd)  { /* LD L, (HL) */
		gb->cpu_reg.l = __gb_read(gb, gb->cpu_reg.hl);
}

void Op6F(struct gb_s *gb, uint32_t opd)  { /* LD L, A */
		gb->cpu_reg.l = gb->cpu_reg.a;
}

void Op70(struct gb_s *gb, uint32_t opd)  { /* LD (HL), B */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.b);
}

void Op71(struct gb_s *gb, uint32_t opd)  { /* LD (HL), C */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.c);
}

void Op72(struct gb_s *gb, uint32_t opd)  { /* LD (HL), D */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.d);
}

void Op73(struct gb_s *gb, uint32_t opd)  { /* LD (HL), E */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.e);
}

void Op74(struct gb_s *gb, uint32_t opd)  { /* LD (HL), H */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.h);
}

void Op75(struct gb_s *gb, uint32_t opd)  { /* LD (HL), L */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.l);
}

void Op76(struct gb_s *gb, uint32_t opd)  { /* HALT */
		/* TODO: Emulate HALT bug? */
		gb->gb_halt = 1;
}

void Op77(struct gb_s *gb, uint32_t opd)  { /* LD (HL), A */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.a);
}

void Op78(struct gb_s *gb, uint32_t opd)  { /* LD A, B */
		gb->cpu_reg.a = gb->cpu_reg.b;
}

void Op79(struct gb_s *gb, uint32_t opd)  { /* LD A, C */
		gb->cpu_reg.a = gb->cpu_reg.c;
}

void Op7A(struct gb_s *gb, uint32_t opd)  { /* LD A, D */
		gb->cpu_reg.a = gb->cpu_reg.d;
}

void Op7B(struct gb_s *gb, uint32_t opd)  { /* LD A, E */
		gb->cpu_reg.a = gb->cpu_reg.e;
}

void Op7C(struct gb_s *gb, uint32_t opd)  { /* LD A, H */
		gb->cpu_reg.a = gb->cpu_reg.h;
}

void Op7D(struct gb_s *gb, uint32_t opd)  { /* LD A, L */
		gb->cpu_reg.a = gb->cpu_reg.l;
}

void Op7E(struct gb_s *gb, uint32_t opd)  { /* LD A, (HL) */
		gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.hl);
}

void Op7F(struct gb_s *gb, uint32_t opd)  { /* LD A, A */
}

void Op80(struct gb_s *gb, uint32_t opd)  { /* ADD A, B */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.b;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.b ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op81(struct gb_s *gb, uint32_t opd)  { /* ADD A, C */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.c ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op82(struct gb_s *gb, uint32_t opd)  { /* ADD A, D */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.d;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.d ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op83(struct gb_s *gb, uint32_t opd)  { /* ADD A, E */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.e;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.e ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op84(struct gb_s *gb, uint32_t opd)  { /* ADD A, H */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.h;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.h ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op85(struct gb_s *gb, uint32_t opd)  { /* ADD A, L */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.l;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.l ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op86(struct gb_s *gb, uint32_t opd)  { /* ADD A, (HL) */
	{
		uint8_t hl = __gb_read(gb, gb->cpu_reg.hl);
		uint16_t temp = gb->cpu_reg.a + hl;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ hl ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op87(struct gb_s *gb, uint32_t opd)  { /* ADD A, A */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.a;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = temp & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op88(struct gb_s *gb, uint32_t opd)  { /* ADC A, B */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.b + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.b ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op89(struct gb_s *gb, uint32_t opd)  { /* ADC A, C */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.c + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.c ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op8A(struct gb_s *gb, uint32_t opd)  { /* ADC A, D */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.d + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.d ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op8B(struct gb_s *gb, uint32_t opd)  { /* ADC A, E */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.e + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.e ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op8C(struct gb_s *gb, uint32_t opd)  { /* ADC A, H */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.h + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.h ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op8D(struct gb_s *gb, uint32_t opd)  { /* ADC A, L */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.l + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.l ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op8E(struct gb_s *gb, uint32_t opd)  { /* ADC A, (HL) */
	{
		uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
		uint16_t temp = gb->cpu_reg.a + val + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ val ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op8F(struct gb_s *gb, uint32_t opd)  { /* ADC A, A */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.a + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		/* TODO: Optimisation here? */
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.a ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op90(struct gb_s *gb, uint32_t opd)  { /* SUB B */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.b;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.b ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op91(struct gb_s *gb, uint32_t opd)  { /* SUB C */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.c ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op92(struct gb_s *gb, uint32_t opd)  { /* SUB D */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.d;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.d ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op93(struct gb_s *gb, uint32_t opd)  { /* SUB E */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.e;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.e ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op94(struct gb_s *gb, uint32_t opd)  { /* SUB H */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.h;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.h ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op95(struct gb_s *gb, uint32_t opd)  { /* SUB L */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.l;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.l ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op96(struct gb_s *gb, uint32_t opd)  { /* SUB (HL) */
	{
		uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
		uint16_t temp = gb->cpu_reg.a - val;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ val ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op97(struct gb_s *gb, uint32_t opd)  { /* SUB A */
		gb->cpu_reg.a = 0;
		gb->cpu_reg.f = 0xC0;
}

void Op98(struct gb_s *gb, uint32_t opd)  { /* SBC A, B */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.b - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.b ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op99(struct gb_s *gb, uint32_t opd)  { /* SBC A, C */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.c - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.c ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op9A(struct gb_s *gb, uint32_t opd)  { /* SBC A, D */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.d - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.d ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op9B(struct gb_s *gb, uint32_t opd)  { /* SBC A, E */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.e - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.e ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op9C(struct gb_s *gb, uint32_t opd)  { /* SBC A, H */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.h - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.h ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op9D(struct gb_s *gb, uint32_t opd)  { /* SBC A, L */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.l - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.l ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op9E(struct gb_s *gb, uint32_t opd)  { /* SBC A, (HL) */
	{
		uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
		uint16_t temp = gb->cpu_reg.a - val - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ val ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void Op9F(struct gb_s *gb, uint32_t opd)  { /* SBC A, A */
		gb->cpu_reg.a = gb->cpu_reg.f_bits.c ? 0xFF : 0x00;
		gb->cpu_reg.f_bits.z = gb->cpu_reg.f_bits.c ? 0x00 : 0x01;
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = gb->cpu_reg.f_bits.c;
}

void OpA0(struct gb_s *gb, uint32_t opd)  { /* AND B */
		gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.b;
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
}

void OpA1(struct gb_s *gb, uint32_t opd)  { /* AND C */
		gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.c;
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
}

void OpA2(struct gb_s *gb, uint32_t opd)  { /* AND D */
		gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.d;
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
}

void OpA3(struct gb_s *gb, uint32_t opd)  { /* AND E */
		gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.e;
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
}

void OpA4(struct gb_s *gb, uint32_t opd)  { /* AND H */
		gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.h;
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
}

void OpA5(struct gb_s *gb, uint32_t opd)  { /* AND L */
		gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.l;
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
}

void OpA6(struct gb_s *gb, uint32_t opd)  { /* AND B */
		gb->cpu_reg.a = gb->cpu_reg.a & __gb_read(gb, gb->cpu_reg.hl);
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
}

void OpA7(struct gb_s *gb, uint32_t opd)  { /* AND A */
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
}

void OpA8(struct gb_s *gb, uint32_t opd)  { /* XOR B */
		gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.b;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpA9(struct gb_s *gb, uint32_t opd)  { /* XOR C */
		gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.c;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpAA(struct gb_s *gb, uint32_t opd)  { /* XOR D */
		gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.d;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpAB(struct gb_s *gb, uint32_t opd)  { /* XOR E */
		gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.e;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpAC(struct gb_s *gb, uint32_t opd)  { /* XOR H */
		gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.h;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpAD(struct gb_s *gb, uint32_t opd)  { /* XOR L */
		gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.l;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpAE(struct gb_s *gb, uint32_t opd)  { /* XOR (HL) */
		gb->cpu_reg.a = gb->cpu_reg.a ^ __gb_read(gb, gb->cpu_reg.hl);
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpAF(struct gb_s *gb, uint32_t opd)  { /* XOR A */
		gb->cpu_reg.a = 0x00;
		gb->cpu_reg.f = 0x80;
}

void OpB0(struct gb_s *gb, uint32_t opd)  { /* OR B */
		gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.b;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpB1(struct gb_s *gb, uint32_t opd)  { /* OR C */
		gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.c;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpB2(struct gb_s *gb, uint32_t opd)  { /* OR D */
		gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.d;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpB3(struct gb_s *gb, uint32_t opd)  { /* OR E */
		gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.e;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpB4(struct gb_s *gb, uint32_t opd)  { /* OR H */
		gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.h;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpB5(struct gb_s *gb, uint32_t opd)  { /* OR L */
		gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.l;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpB6(struct gb_s *gb, uint32_t opd)  { /* OR (HL) */
		gb->cpu_reg.a = gb->cpu_reg.a | __gb_read(gb, gb->cpu_reg.hl);
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpB7(struct gb_s *gb, uint32_t opd)  { /* OR A */
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpB8(struct gb_s *gb, uint32_t opd)  { /* CP B */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.b;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.b ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
}
	}

void OpB9(struct gb_s *gb, uint32_t opd)  { /* CP C */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.c ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
}
	}

void OpBA(struct gb_s *gb, uint32_t opd)  { /* CP D */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.d;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.d ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
}
	}

void OpBB(struct gb_s *gb, uint32_t opd)  { /* CP E */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.e;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.e ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
}
	}

void OpBC(struct gb_s *gb, uint32_t opd)  { /* CP H */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.h;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.h ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
}
	}

void OpBD(struct gb_s *gb, uint32_t opd)  { /* CP L */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.l;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.l ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
}
	}

	/* TODO: Optimsation by combining similar opcode routines. */
void OpBE(struct gb_s *gb, uint32_t opd)  { /* CP B */
	{
		uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
		uint16_t temp = gb->cpu_reg.a - val;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ val ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
}
	}

void OpBF(struct gb_s *gb, uint32_t opd)  { /* CP A */
		gb->cpu_reg.f = 0xC0;
}

void OpC0(struct gb_s *gb, uint32_t opd)  { /* RET NZ */
		if(!gb->cpu_reg.f_bits.z)
		{
			gb->cpu_reg.pc = __gb_read(gb, gb->cpu_reg.sp++);
			gb->cpu_reg.pc |= __gb_read(gb, gb->cpu_reg.sp++) << 8;
			inst_cycles += 12;
		}
}

void OpC1(struct gb_s *gb, uint32_t opd)  { /* POP BC */
		gb->cpu_reg.c = __gb_read(gb, gb->cpu_reg.sp++);
		gb->cpu_reg.b = __gb_read(gb, gb->cpu_reg.sp++);
}

void OpC2(struct gb_s *gb, uint32_t opd)  { /* JP NZ, imm */
		if(!gb->cpu_reg.f_bits.z)
		{
			uint16_t temp = (opd);
			gb->cpu_reg.pc = temp;
			inst_cycles += 4;
		}
}

void OpC3(struct gb_s *gb, uint32_t opd)  { /* JP imm */
	{
		uint16_t temp = (opd);
		gb->cpu_reg.pc = temp;
}
	}

void OpC4(struct gb_s *gb, uint32_t opd)  { /* CALL NZ imm */
		if(!gb->cpu_reg.f_bits.z)
		{
			uint16_t temp = (opd);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
			gb->cpu_reg.pc = temp;
			inst_cycles += 12;
		}
}

void OpC5(struct gb_s *gb, uint32_t opd)  { /* PUSH BC */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.b);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.c);
}

void OpC6(struct gb_s *gb, uint32_t opd)  { /* ADD A, imm */
	{
		/* Taken from SameBoy, which is released under MIT Licence. */
		uint8_t value = (opd);
		uint16_t calc = gb->cpu_reg.a + value;
		gb->cpu_reg.f_bits.z = ((uint8_t)calc == 0) ? 1 : 0;
		gb->cpu_reg.f_bits.h =
			((gb->cpu_reg.a & 0xF) + (value & 0xF) > 0x0F) ? 1 : 0;
		gb->cpu_reg.f_bits.c = calc > 0xFF ? 1 : 0;
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.a = (uint8_t)calc;
}
	}

void OpC7(struct gb_s *gb, uint32_t opd)  { /* RST 0x0000 */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0000;
}

void OpC8(struct gb_s *gb, uint32_t opd)  { /* RET Z */
		if(gb->cpu_reg.f_bits.z)
		{
			uint16_t temp = __gb_read(gb, gb->cpu_reg.sp++);
			temp |= __gb_read(gb, gb->cpu_reg.sp++) << 8;
			gb->cpu_reg.pc = temp;
			inst_cycles += 12;
		}
}

void OpC9(struct gb_s *gb, uint32_t opd)  { /* RET */
	{
		uint16_t temp = __gb_read(gb, gb->cpu_reg.sp++);
		temp |= __gb_read(gb, gb->cpu_reg.sp++) << 8;
		gb->cpu_reg.pc = temp;
}
	}

void OpCA(struct gb_s *gb, uint32_t opd)  { /* JP Z, imm */
		if(gb->cpu_reg.f_bits.z)
		{
			uint16_t temp = (opd);
			gb->cpu_reg.pc = temp;
			inst_cycles += 4;
		}
}

void OpCB(struct gb_s *gb, uint32_t opd)  { /* CB INST */
		__gb_execute_cb(gb, opd);
}

void OpCC(struct gb_s *gb, uint32_t opd)  { /* CALL Z, imm */
		if(gb->cpu_reg.f_bits.z)
		{
			uint16_t temp = (opd);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
			gb->cpu_reg.pc = temp;
			inst_cycles += 12;
		}
}

void OpCD(struct gb_s *gb, uint32_t opd)  { /* CALL imm */
	{
		uint16_t addr = (opd);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = addr;
	}
}

void OpCE(struct gb_s *gb, uint32_t opd)  { /* ADC A, imm */
	{
		uint8_t value, a, carry;
		value = (opd);
		a = gb->cpu_reg.a;
		carry = gb->cpu_reg.f_bits.c;
		gb->cpu_reg.a = a + value + carry;

		gb->cpu_reg.f_bits.z = gb->cpu_reg.a == 0 ? 1 : 0;
		gb->cpu_reg.f_bits.h =
			((a & 0xF) + (value & 0xF) + carry > 0x0F) ? 1 : 0;
		gb->cpu_reg.f_bits.c =
			(((uint16_t) a) + ((uint16_t) value) + carry > 0xFF) ? 1 : 0;
		gb->cpu_reg.f_bits.n = 0;
}
	}

void OpCF(struct gb_s *gb, uint32_t opd)  { /* RST 0x0008 */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0008;
}

void OpD0(struct gb_s *gb, uint32_t opd)  { /* RET NC */
		if(!gb->cpu_reg.f_bits.c)
		{
			uint16_t temp = __gb_read(gb, gb->cpu_reg.sp++);
			temp |= __gb_read(gb, gb->cpu_reg.sp++) << 8;
			gb->cpu_reg.pc = temp;
			inst_cycles += 12;
		}
}

void OpD1(struct gb_s *gb, uint32_t opd)  { /* POP DE */
		gb->cpu_reg.e = __gb_read(gb, gb->cpu_reg.sp++);
		gb->cpu_reg.d = __gb_read(gb, gb->cpu_reg.sp++);
}

void OpD2(struct gb_s *gb, uint32_t opd)  { /* JP NC, imm */
		if(!gb->cpu_reg.f_bits.c)
		{
			uint16_t temp =  (opd);
			gb->cpu_reg.pc = temp;
			inst_cycles += 4;
		}
}

void OpD4(struct gb_s *gb, uint32_t opd)  { /* CALL NC, imm */
		if(!gb->cpu_reg.f_bits.c)
		{
			uint16_t temp = (opd);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
			gb->cpu_reg.pc = temp;
			inst_cycles += 12;
		}
}

void OpD5(struct gb_s *gb, uint32_t opd)  { /* PUSH DE */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.d);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.e);
}

void OpD6(struct gb_s *gb, uint32_t opd)  { /* SUB imm */
	{
		uint8_t val = (opd);
		uint16_t temp = gb->cpu_reg.a - val;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ val ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp & 0xFF);
}
	}

void OpD7(struct gb_s *gb, uint32_t opd)  { /* RST 0x0010 */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0010;
}

void OpD8(struct gb_s *gb, uint32_t opd)  { /* RET C */
		if(gb->cpu_reg.f_bits.c)
		{
			uint16_t temp = __gb_read(gb, gb->cpu_reg.sp++);
			temp |= __gb_read(gb, gb->cpu_reg.sp++) << 8;
			gb->cpu_reg.pc = temp;
			inst_cycles += 12;
		}
}

void OpD9(struct gb_s *gb, uint32_t opd)  { /* RETI */
	{
		uint16_t temp = __gb_read(gb, gb->cpu_reg.sp++);
		temp |= __gb_read(gb, gb->cpu_reg.sp++) << 8;
		gb->cpu_reg.pc = temp;
		gb->gb_ime = 1;
	}
}

void OpDA(struct gb_s *gb, uint32_t opd)  { /* JP C, imm */
		if(gb->cpu_reg.f_bits.c)
		{
			uint16_t addr = (opd);
			gb->cpu_reg.pc = addr;
			inst_cycles += 4;
		}
}

void OpDC(struct gb_s *gb, uint32_t opd)  { /* CALL C, imm */
		if(gb->cpu_reg.f_bits.c)
		{
			uint16_t temp = (opd);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
			gb->cpu_reg.pc = temp;
			inst_cycles += 12;
		}
}

void OpDE(struct gb_s *gb, uint32_t opd)  { /* SBC A, imm */
	{
		uint8_t temp_8 = (opd);
		uint16_t temp_16 = gb->cpu_reg.a - temp_8 - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp_16 & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ temp_8 ^ temp_16) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp_16 & 0xFF00) ? 1 : 0;
		gb->cpu_reg.a = (temp_16 & 0xFF);
}
	}

void OpDF(struct gb_s *gb, uint32_t opd)  { /* RST 0x0018 */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0018;
}

void OpE0(struct gb_s *gb, uint32_t opd)  { /* LD (0xFF00+imm), A */
		__gb_write(gb, 0xFF00 | ((opd) & 0xFF),
			   gb->cpu_reg.a);
}

void OpE1(struct gb_s *gb, uint32_t opd)  { /* POP HL */
		gb->cpu_reg.l = __gb_read(gb, gb->cpu_reg.sp++);
		gb->cpu_reg.h = __gb_read(gb, gb->cpu_reg.sp++);
}

void OpE2(struct gb_s *gb, uint32_t opd)  { /* LD (C), A */
		__gb_write(gb, 0xFF00 | gb->cpu_reg.c, gb->cpu_reg.a);
}

void OpE5(struct gb_s *gb, uint32_t opd)  { /* PUSH HL */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.h);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.l);
}

void OpE6(struct gb_s *gb, uint32_t opd)  { /* AND imm */
		/* TODO: Optimisation? */
		gb->cpu_reg.a = gb->cpu_reg.a & (opd);
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
}

void OpE7(struct gb_s *gb, uint32_t opd)  { /* RST 0x0020 */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0020;
}

void OpE8(struct gb_s *gb, uint32_t opd)  { /* ADD SP, imm */
	{
		int8_t offset = (int8_t) (opd);
		/* TODO: Move flag assignments for optimisation. */
		gb->cpu_reg.f_bits.z = 0;
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.sp & 0xF) + (offset & 0xF) > 0xF) ? 1 : 0;
		gb->cpu_reg.f_bits.c = ((gb->cpu_reg.sp & 0xFF) + (offset & 0xFF) > 0xFF);
		gb->cpu_reg.sp += offset;
}
	}

void OpE9(struct gb_s *gb, uint32_t opd)  { /* JP (HL) */
		gb->cpu_reg.pc = gb->cpu_reg.hl;
}

void OpEA(struct gb_s *gb, uint32_t opd)  { /* LD (imm), A */
	{
		uint16_t addr = (opd);
		__gb_write(gb, addr, gb->cpu_reg.a);
}
	}

void OpEE(struct gb_s *gb, uint32_t opd)  { /* XOR imm */
		gb->cpu_reg.a = gb->cpu_reg.a ^ (opd);
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpEF(struct gb_s *gb, uint32_t opd)  { /* RST 0x0028 */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0028;
}

void OpF0(struct gb_s *gb, uint32_t opd)  { /* LD A, (0xFF00+imm) */
		gb->cpu_reg.a =
			__gb_read(gb, 0xFF00 | ((opd) & 0xFF));
}

void OpF1(struct gb_s *gb, uint32_t opd)  { /* POP AF */
	{
		uint8_t temp_8 = __gb_read(gb, gb->cpu_reg.sp++);
		gb->cpu_reg.f = temp_8 & 0xF0;
		gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.sp++);
}
	}

void OpF2(struct gb_s *gb, uint32_t opd)  { /* LD A, (C) */
		gb->cpu_reg.a = __gb_read(gb, 0xFF00 | gb->cpu_reg.c);
}

void OpF3(struct gb_s *gb, uint32_t opd)  { /* DI */
		gb->gb_ime = 0;
}

void OpF5(struct gb_s *gb, uint32_t opd)  { /* PUSH AF */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.a);
		__gb_write(gb, --gb->cpu_reg.sp,
			   gb->cpu_reg.f_bits.z << 7 | gb->cpu_reg.f_bits.n << 6 |
			   gb->cpu_reg.f_bits.h << 5 | gb->cpu_reg.f_bits.c << 4);
}

void OpF6(struct gb_s *gb, uint32_t opd)  { /* OR imm */
		gb->cpu_reg.a = gb->cpu_reg.a | (opd);
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
}

void OpF7(struct gb_s *gb, uint32_t opd)  { /* PUSH AF */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0030;
}

void OpF8(struct gb_s *gb, uint32_t opd)  { /* LD HL, SP+/-imm */
	{
		/* Taken from SameBoy, which is released under MIT Licence. */
		int8_t offset = (int8_t) (opd);
		gb->cpu_reg.hl = gb->cpu_reg.sp + offset;
		gb->cpu_reg.f_bits.z = 0;
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.sp & 0xF) + (offset & 0xF) > 0xF) ? 1 : 0;
		gb->cpu_reg.f_bits.c = ((gb->cpu_reg.sp & 0xFF) + (offset & 0xFF) > 0xFF) ? 1 :
				       0;
}
	}

void OpF9(struct gb_s *gb, uint32_t opd)  { /* LD SP, HL */
		gb->cpu_reg.sp = gb->cpu_reg.hl;
}

void OpFA(struct gb_s *gb, uint32_t opd)  { /* LD A, (imm) */
	{
		uint16_t addr = (opd);
		gb->cpu_reg.a = __gb_read(gb, addr);
}
	}

void OpFB(struct gb_s *gb, uint32_t opd)  { /* EI */
		gb->gb_ime = 1;
}

void OpFE(struct gb_s *gb, uint32_t opd)  { /* CP imm */
	{
		uint8_t temp_8 = (opd);
		uint16_t temp_16 = gb->cpu_reg.a - temp_8;
		gb->cpu_reg.f_bits.z = ((temp_16 & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.a ^ temp_8 ^ temp_16) & 0x10) ? 1 : 0;
		gb->cpu_reg.f_bits.c = (temp_16 & 0xFF00) ? 1 : 0;
}
	}

void OpFF(struct gb_s *gb, uint32_t opd)  { /* RST 0x0038 */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0038;
}

#ifdef  OCEMU
static void (*opcodes[])(struct gb_s *gb, uint32_t opd) RODATA = {
#else
static const uint32_t opcodes[] RODATA = {
#endif
  Op00, Op01, Op02, Op03, Op04, Op05, Op06, Op07, Op08, Op09, Op0A, Op0B, Op0C, Op0D, Op0E, Op0F,
  Op10, Op11, Op12, Op13, Op14, Op15, Op16, Op17, Op18, Op19, Op1A, Op1B, Op1C, Op1D, Op1E, Op1F,
  Op20, Op21, Op22, Op23, Op24, Op25, Op26, Op27, Op28, Op29, Op2A, Op2B, Op2C, Op2D, Op2E, Op2F,
  Op30, Op31, Op32, Op33, Op34, Op35, Op36, Op37, Op38, Op39, Op3A, Op3B, Op3C, Op3D, Op3E, Op3F,
  Op40, Op41, Op42, Op43, Op44, Op45, Op46, Op47, Op48, Op49, Op4A, Op4B, Op4C, Op4D, Op4E, Op4F,
  Op50, Op51, Op52, Op53, Op54, Op55, Op56, Op57, Op58, Op59, Op5A, Op5B, Op5C, Op5D, Op5E, Op5F,
  Op60, Op61, Op62, Op63, Op64, Op65, Op66, Op67, Op68, Op69, Op6A, Op6B, Op6C, Op6D, Op6E, Op6F,
  Op70, Op71, Op72, Op73, Op74, Op75, Op76, Op77, Op78, Op79, Op7A, Op7B, Op7C, Op7D, Op7E, Op7F,
  Op80, Op81, Op82, Op83, Op84, Op85, Op86, Op87, Op88, Op89, Op8A, Op8B, Op8C, Op8D, Op8E, Op8F,
  Op90, Op91, Op92, Op93, Op94, Op95, Op96, Op97, Op98, Op99, Op9A, Op9B, Op9C, Op9D, Op9E, Op9F,
  OpA0, OpA1, OpA2, OpA3, OpA4, OpA5, OpA6, OpA7, OpA8, OpA9, OpAA, OpAB, OpAC, OpAD, OpAE, OpAF,
  OpB0, OpB1, OpB2, OpB3, OpB4, OpB5, OpB6, OpB7, OpB8, OpB9, OpBA, OpBB, OpBC, OpBD, OpBE, OpBF,
  OpC0, OpC1, OpC2, OpC3, OpC4, OpC5, OpC6, OpC7, OpC8, OpC9, OpCA, OpCB, OpCC, OpCD, OpCE, OpCF,
  OpD0, OpD1, OpD2, Op00, OpD4, OpD5, OpD6, OpD7, OpD8, OpD9, OpDA, Op00, OpDC, Op00, OpDE, OpDF,
  OpE0, OpE1, OpE2, Op00, Op00, OpE5, OpE6, OpE7, OpE8, OpE9, OpEA, Op00, Op00, Op00, OpEE, OpEF,
  OpF0, OpF1, OpF2, OpF3, Op00, OpF5, OpF6, OpF7, OpF8, OpF9, OpFA, OpFB, Op00, Op00, OpFE, OpFF,
};

static const uint32_t op_cycles[0x100] RODATA =
{
  /* *INDENT-OFF* */
  /*0 1 2  3  4  5  6  7  8  9  A  B  C  D  E  F	*/
  4,12, 8, 8, 4, 4, 8, 4,20, 8, 8, 8, 4, 4, 8, 4,	/* 0x00 */
  4,12, 8, 8, 4, 4, 8, 4,12, 8, 8, 8, 4, 4, 8, 4,	/* 0x10 */
  8,12, 8, 8, 4, 4, 8, 4, 8, 8, 8, 8, 4, 4, 8, 4,	/* 0x20 */
  8,12, 8, 8,12,12,12, 4, 8, 8, 8, 8, 4, 4, 8, 4,	/* 0x30 */
  4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,	/* 0x40 */
  4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,	/* 0x50 */
  4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,	/* 0x60 */
  8, 8, 8, 8, 8, 8, 4, 8, 4, 4, 4, 4, 4, 4, 8, 4,	/* 0x70 */
  4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,	/* 0x80 */
  4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,	/* 0x90 */
  4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,	/* 0xA0 */
  4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,	/* 0xB0 */
  8,12,12,16,12,16, 8,16, 8,16,12, 8,12,24, 8,16,	/* 0xC0 */
  8,12,12, 0,12,16, 8,16, 8,16,12, 0,12, 0, 8,16,	/* 0xD0 */
  12,12,8, 0, 0,16, 8,16,16, 4,16, 0, 0, 0, 8,16,	/* 0xE0 */
  12,12,8, 4, 0,16, 8,16,12, 8,16, 4, 0, 0, 8,16	/* 0xF0 */
  /* *INDENT-ON* */
};

static const uint32_t op_sizes[0x100] RODATA =
{
  1, 3, 1, 1, 1, 1, 2, 1, 3, 1, 1, 1, 1, 1, 2, 1,
  1, 3, 1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1,
  2, 3, 1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1,
  2, 3, 1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 3, 3, 3, 1, 2, 1, 1, 1, 3, 2, 3, 3, 2, 1,
  1, 1, 3, 0, 3, 1, 2, 1, 1, 1, 3, 0, 3, 0, 2, 1,
  2, 1, 1, 0, 0, 1, 2, 1, 2, 1, 3, 0, 0, 0, 2, 1,
  2, 1, 1, 1, 0, 1, 2, 1, 2, 1, 3, 1, 0, 0, 2, 1
};

/**
 * Internal function used to step the CPU.
 */
void __gb_step_cpu(struct gb_s *gb, uint32_t cycles)
{
	uint8_t opcode;
  uint32_t ccount = 0;

  while (ccount < cycles) {
    /* Handle interrupts */
    if((gb->gb_ime || gb->gb_halt) &&
        (gb->gb_reg.IF & gb->gb_reg.IE & ANY_INTR))
    {
      gb->gb_halt = 0;

      if(gb->gb_ime)
      {
        /* Disable interrupts */
        gb->gb_ime = 0;

        /* Push Program Counter */
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);

        /* Call interrupt handler if required. */
        if(gb->gb_reg.IF & gb->gb_reg.IE & VBLANK_INTR)
        {
          gb->cpu_reg.pc = VBLANK_INTR_ADDR;
          gb->gb_reg.IF ^= VBLANK_INTR;
        }
        else if(gb->gb_reg.IF & gb->gb_reg.IE & LCDC_INTR)
        {
          gb->cpu_reg.pc = LCDC_INTR_ADDR;
          gb->gb_reg.IF ^= LCDC_INTR;
        }
        else if(gb->gb_reg.IF & gb->gb_reg.IE & TIMER_INTR)
        {
          gb->cpu_reg.pc = TIMER_INTR_ADDR;
          gb->gb_reg.IF ^= TIMER_INTR;
        }
        else if(gb->gb_reg.IF & gb->gb_reg.IE & SERIAL_INTR)
        {
          gb->cpu_reg.pc = SERIAL_INTR_ADDR;
          gb->gb_reg.IF ^= SERIAL_INTR;
        }
        else if(gb->gb_reg.IF & gb->gb_reg.IE & CONTROL_INTR)
        {
          gb->cpu_reg.pc = CONTROL_INTR_ADDR;
          gb->gb_reg.IF ^= CONTROL_INTR;
        }
      }
    }

    /* Obtain opcode */
    if (gb->gb_halt) {
      inst_cycles = 4;
    } else {
      opcode = __gb_read(gb, gb->cpu_reg.pc);
      inst_cycles = gb->cpu_cycles[opcode];
      uint16_t opc_s = gb->cpu_sizes[opcode];
      uint32_t opd = 0;
      if (opc_s == 2)
        opd = __gb_read(gb, gb->cpu_reg.pc + 1);
      if (opc_s == 3)
        opd = __gb_read16(gb, gb->cpu_reg.pc + 1);
      gb->cpu_reg.pc += opc_s;

      ((void (*)())gb->cpu_opcodes[opcode])(gb, opd);
    }

    /* DIV register timing */
    gb->counter.div_count += inst_cycles;

    if(gb->counter.div_count >= DIV_CYCLES)
    {
      gb->gb_reg.DIV++;
      gb->counter.div_count -= DIV_CYCLES;
    }

    /* TIMA register timing */
    /* TODO: Change tac_enable to struct of TAC timer control bits. */
    if(gb->gb_reg.tac_enable)
    {
      static const uint_fast16_t TAC_CYCLES[4] RODATA = {1024, 16, 64, 256};

      gb->counter.tima_count += inst_cycles;

      if(gb->counter.tima_count >= TAC_CYCLES[gb->gb_reg.tac_rate])
      {
        gb->counter.tima_count -= TAC_CYCLES[gb->gb_reg.tac_rate];

        if(++gb->gb_reg.TIMA == 0)
        {
          gb->gb_reg.IF |= TIMER_INTR;
          /* On overflow, set TMA to TIMA. */
          gb->gb_reg.TIMA = gb->gb_reg.TMA;
        }
      }
    }
    ccount += inst_cycles;

    /* LCD Timing */
    gb->counter.lcd_count += inst_cycles;
  }

  /* Check serial transfer. TODO: Implement outside the main loop. */
    /*if(gb->gb_reg.SC & 0x80)
    {
      gb->counter.serial_count += inst_cycles;

      if(gb->counter.serial_count >= SERIAL_CYCLES)
      {
        if(gb->gb_serial_transfer == NULL)
          gb->gb_reg.SB = 0xFF;
        else
          gb->gb_reg.SB = (gb->gb_serial_transfer)(gb, gb->gb_reg.SB);

        // Inform game of serial TX/RX completion.
        gb->gb_reg.SC &= 0x01;
        gb->gb_reg.IF |= SERIAL_INTR;
        gb->counter.serial_count -= SERIAL_CYCLES;
      }
    }*/
}

void __gb_step_line(struct gb_s *gb) {
	/* TODO Check behaviour of LCD during LCD power off state. */
	/* If LCD is off, don't update LCD state. */
	if((gb->gb_reg.LCDC & LCDC_ENABLE) == 0) {
    __gb_step_cpu(gb, 16);
    return;
  }

	/* OAM access */
	if(gb->lcd_mode == LCD_HBLANK
			&& gb->counter.lcd_count < LCD_MODE_2_CYCLES) {
    __gb_step_cpu(gb, LCD_MODE_2_CYCLES - gb->counter.lcd_count);
		gb->lcd_mode = LCD_SEARCH_OAM;

		if(gb->gb_reg.STAT & STAT_MODE_2_INTR)
			gb->gb_reg.IF |= LCDC_INTR;
  }
	/* Update LCD */
	if (gb->lcd_mode == LCD_SEARCH_OAM
			&& gb->counter.lcd_count < LCD_MODE_3_CYCLES)
	{
    __gb_step_cpu(gb, LCD_MODE_3_CYCLES - gb->counter.lcd_count);
		gb->lcd_mode = LCD_TRANSFER;
#if ENABLE_LCD
		__gb_draw_line(gb);
#endif
	}

	/* New Scanline */
	if(gb->counter.lcd_count < LCD_LINE_CYCLES) {
    __gb_step_cpu(gb, LCD_LINE_CYCLES - gb->counter.lcd_count);

    if(gb->counter.lcd_count < LCD_LINE_CYCLES) {
      __gb_step_cpu(gb, 16);
      return;
    }
  }

  gb->counter.lcd_count -= LCD_LINE_CYCLES;

  /* LYC Update */
  if(gb->gb_reg.LY == gb->gb_reg.LYC)
  {
    gb->gb_reg.STAT |= STAT_LYC_COINC;

    if(gb->gb_reg.STAT & STAT_LYC_INTR)
      gb->gb_reg.IF |= LCDC_INTR;
  }
  else
    gb->gb_reg.STAT &= 0xFB;

  /* Next line */
  gb->gb_reg.LY = gb->gb_reg.LY + 1;
  if (gb->gb_reg.LY == LCD_VERT_LINES)
    gb->gb_reg.LY = 0;

  /* VBLANK Start */
  if(gb->gb_reg.LY == LCD_HEIGHT)
  {
    gb->lcd_mode = LCD_VBLANK;
    gb->gb_frame = 1;
    gb->gb_reg.IF |= VBLANK_INTR;

    if(gb->gb_reg.STAT & STAT_MODE_1_INTR)
      gb->gb_reg.IF |= LCDC_INTR;

#if ENABLE_LCD
    /* If frame skip is activated, check if we need to draw
      * the frame or skip it. */
    gb->display.frame_skip_count++;
    if (gb->display.frame_skip_count > gb->direct.frame_skip)
      gb->display.frame_skip_count = 0;
#endif
  }
  /* Normal Line */
  else if(gb->gb_reg.LY < LCD_HEIGHT)
  {
    if(gb->gb_reg.LY == 0)
    {
      /* Clear Screen */
      // FIXME
      gb->display.WY = gb->gb_reg.WY;
      gb->display.window_clear = 0;
    }

    gb->lcd_mode = LCD_HBLANK;

    if(gb->gb_reg.STAT & STAT_MODE_0_INTR)
      gb->gb_reg.IF |= LCDC_INTR;
  }
}

void gb_run_frame(struct gb_s *gb)
{
	gb->gb_frame = 0;

	while(!gb->gb_frame)
		__gb_step_line(gb);
}

/**
 * Gets the size of the save file required for the ROM.
 */
uint_fast32_t gb_get_save_size(struct gb_s *gb)
{
	const uint_fast16_t ram_size_location = 0x0149;
	const uint_fast32_t ram_sizes[] =
	{
		0x00, 0x800, 0x2000, 0x8000, 0x20000
	};
	uint8_t ram_size = gb->gb_rom_read(gb, ram_size_location);
	return ram_sizes[ram_size];
}

/**
 * Set the function used to handle serial transfer in the front-end. This is
 * optional.
 * gb_serial_transfer takes a byte to transmit and returns the received byte. If
 * no cable is connected to the console, return 0xFF.
 */
void gb_init_serial(struct gb_s *gb,
		    uint8_t (*gb_serial_transfer)(struct gb_s*, const uint8_t))
{
	gb->gb_serial_transfer = gb_serial_transfer;
}

uint8_t gb_colour_hash(struct gb_s *gb)
{
#define ROM_TITLE_START_ADDR	0x0134
#define ROM_TITLE_END_ADDR	0x0143

	uint8_t x = 0;

	for(uint16_t i = ROM_TITLE_START_ADDR; i <= ROM_TITLE_END_ADDR; i++)
		x += gb->gb_rom_read(gb, i);

	return x;
}

/**
 * Initialise the emulator context. gb_reset() is also called to initialise
 * the CPU.
 */
enum gb_init_error_e gb_init(struct gb_s *gb,
			     uint8_t (*gb_rom_read)(struct gb_s*, const uint_fast32_t),
			     uint8_t (*gb_cart_ram_read)(struct gb_s*, const uint_fast32_t),
			     void (*gb_cart_ram_write)(struct gb_s*, const uint_fast32_t, const uint8_t),
			     void (*gb_error)(struct gb_s*, const enum gb_error_e, const uint16_t),
			     void *priv)
{
  memcpy(gb->cpu_opcodes, opcodes, sizeof(opcodes));
  memcpy(gb->cpu_cycles, op_cycles, sizeof(op_cycles));
  memcpy(gb->cpu_sizes, op_sizes, sizeof(op_sizes));

	const uint16_t mbc_location = 0x0147;
	const uint16_t bank_count_location = 0x0148;
	const uint16_t ram_size_location = 0x0149;
	/**
	 * Table for cartridge type (MBC). -1 if invalid.
	 * TODO: MMM01 is untested.
	 * TODO: MBC6 is untested.
	 * TODO: MBC7 is unsupported.
	 * TODO: POCKET CAMERA is unsupported.
	 * TODO: BANDAI TAMA5 is unsupported.
	 * TODO: HuC3 is unsupported.
	 * TODO: HuC1 is unsupported.
	 **/
	const uint8_t cart_mbc[] =
	{
		0, 1, 1, 1, -1, 2, 2, -1, 0, 0, -1, 0, 0, 0, -1, 3,
		3, 3, 3, 3, -1, -1, -1, -1, -1, 5, 5, 5, 5, 5, 5, -1
	};
	const uint8_t cart_ram[] =
	{
		0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
		1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0
	};
	const uint16_t num_rom_banks[] =
	{
		2, 4, 8, 16, 32, 64, 128, 256, 512, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 72, 80, 96, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	};
	const uint8_t num_ram_banks[] = { 0, 1, 1, 4, 16, 8 };

	gb->gb_rom_read = gb_rom_read;
	gb->gb_cart_ram_read = gb_cart_ram_read;
	gb->gb_cart_ram_write = gb_cart_ram_write;
	gb->gb_error = gb_error;
	gb->direct.priv = priv;

	/* Initialise serial transfer function to NULL. If the front-end does
	 * not provide serial support, Peanut-GB will emulate no cable connected
	 * automatically. */
	gb->gb_serial_transfer = NULL;

	/* Check valid ROM using checksum value. */
	{
		uint8_t x = 0;

		for(uint16_t i = 0x0134; i <= 0x014C; i++)
			x = x - gb->gb_rom_read(gb, i) - 1;

		if(x != gb->gb_rom_read(gb, ROM_HEADER_CHECKSUM_LOC))
			return GB_INIT_INVALID_CHECKSUM;
	}

	/* Check if cartridge type is supported, and set MBC type. */
	{
		const uint8_t mbc_value = gb->gb_rom_read(gb, mbc_location);

		if(mbc_value > sizeof(cart_mbc) - 1 ||
				(gb->mbc = cart_mbc[gb->gb_rom_read(gb, mbc_location)]) == 255u)
			return GB_INIT_CARTRIDGE_UNSUPPORTED;
	}

	gb->cart_ram = cart_ram[gb->gb_rom_read(gb, mbc_location)];
	gb->num_rom_banks = num_rom_banks[gb->gb_rom_read(gb, bank_count_location)];
	gb->num_ram_banks = num_ram_banks[gb->gb_rom_read(gb, ram_size_location)];

	gb_reset(gb);

	return GB_INIT_NO_ERROR;
}

/**
 * Returns the title of ROM.
 *
 * \param gb		Initialised context.
 * \param title_str	Allocated string at least 16 characters.
 * \returns		Pointer to start of string, null terminated.
 */
const char* gb_get_rom_name(struct gb_s* gb, char* title_str)
{
	uint_least16_t title_loc = 0x134;
	/* End of title may be 0x13E for newer games. */
	const uint_least16_t title_end = 0x143;
	const char* title_start = title_str;

	for(; title_loc <= title_end; title_loc++)
	{
		const char title_char = gb->gb_rom_read(gb, title_loc);

		if(title_char >= ' ' && title_char <= '_')
		{
			*title_str = title_char;
			title_str++;
		}
		else
			break;
	}

	*title_str = '\0';
	return title_start;
}

#if ENABLE_LCD
void gb_init_lcd(struct gb_s *gb)
{
	gb->direct.frame_skip = 0;
	gb->display.frame_skip_count = 0;

	gb->display.window_clear = 0;
	gb->display.WY = 0;
	return;
}
#endif
