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

#define PROFILE_START

#define ENABLE_GBC    1

#ifdef  ENABLE_GBC
#undef  ROCODE
#define ROCODE
#endif

/**
 * Sound support must be provided by an external library. When audio_read() and
 * audio_write() functions are provided, define ENABLE_SOUND before including
 * peanut_gb.h in order for these functions to be used.
 */
#define ENABLE_SOUND 0

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
#ifdef  ENABLE_GBC
#define WRAM_SIZE	0x8000
#define VRAM_SIZE	0x4000
#else
#define WRAM_SIZE	0x2000
#define VRAM_SIZE	0x2000
#endif
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
	uint32_t lcd_count;		/* LCD Timing */
	uint32_t div_count;		/* Divider Register Counter */
	uint32_t tima_count;	/* Timer Counter */
	uint32_t serial_count;
#ifdef  ENABLE_SOUND
  uint32_t sound_count;
#endif
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

#ifdef  ENABLE_GBC
	/* GBC. */
  /* Speed switch. */
  uint8_t KEY1;
  /* GBC VRAM bank. */
  uint8_t VBK;
  /* GBC VRAM DMA. */
  uint8_t HDMA1;
  uint8_t HDMA2;
  uint8_t HDMA3;
  uint8_t HDMA4;
  uint8_t HDMA5;
  /* GBC palettes. */
  uint8_t BGPI;
  uint8_t BGPD;
  uint8_t OBPI;
  uint8_t OBPD;
  /* Infrared. */
  uint8_t RP;
  /* WRAM BANK */
  uint8_t SVBK;
  /* Undocumented. */
  uint8_t REG6C;
  uint8_t REG72;
  uint8_t REG73;
  uint8_t REG74;
  uint8_t REG75;
  //uint8_t REG76; -- READ ONLY
  //uint8_t REG77; -- READ ONLY
#endif
};

#if ENABLE_LCD
	/* Bit mask for the shade of pixel to display */
	#define LCD_COLOUR	0x03
	/**
	* Bit mask for whether a pixel is OBJ0, OBJ1, or BG. Each may have a different
	* palette when playing a DMG game on CGB.
	*/
	#define LCD_PALETTE_OBJ	0x04
	#define LCD_PALETTE_BG	0x00
	/**
	* Bit mask for the two bits listed above.
	* LCD_PALETTE_ALL == 0b10 --> OBJ0
	* LCD_PALETTE_ALL == 0b11 --> OBJ1
	* LCD_PALETTE_ALL == 0b00 --> BG
	* LCD_PALETTE_ALL == 0b01 --> NOT POSSIBLE
	*/
	#define LCD_PALETTE_ALL 0x08
#ifdef  ENABLE_GBC
  #define LCD_GBC_PAL_BG      0x40
  #define LCD_GBC_PAL_BG_PRI  0x60
  #define LCD_GBC_PAL_OBJ     0x20
#endif
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
#ifdef  ENABLE_GBC
  uint32_t is_gbc;
#endif

	uint32_t selected_rom_bank;
  uint32_t rom_bank_addr;
  uint8_t *rom_bank0;

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
		 * 			Bits 3-2 are the palette, where:
		 * 				OBJ0 = 0b00,
		 * 				OBJ1 = 0b01,
		 * 				BG = 0b10
		 * 			Other bits are undefined.
		 * 			Bits 3-2 are only required by front-ends
		 * 			which want to use a different colour for
		 * 			different object palettes. This is what
		 * 			the Game Boy Color (CGB) does to DMG
		 * 			games.
		 */

		/* Palettes */
    uint16_t gb_palette[3][4];
#ifdef  ENABLE_GBC
    uint8_t bg_palette[4 * 8 * 2];
    uint8_t sp_palette[4 * 8 * 2];
#endif

		uint32_t window_clear;
		uint32_t WY; // FIXME: Check requirement

		uint32_t frame_skip_count;
    uint8_t *framebuffer;
    uint32_t chunk_cnt;
    uint32_t line_cnt;
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
	} direct;

  uint32_t cpu_cycles[256];
  uint32_t cpu_sizes[256];

  uint8_t *addr_map[16];
  uint8_t (*addr_read[16])(struct gb_s *gb, const uint_fast16_t addr);
  uint8_t *addr_wmap[16];
  void (*addr_write[16])(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val);
};

/**
 * Tick the internal RTC by one second.
 */
void ROCODE gb_tick_rtc(struct gb_s *gb)
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
void ROCODE gb_set_rtc(struct gb_s *gb, const struct tm * const time)
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
      return gb_cart_ram_read(gb, addr - CART_RAM_ADDR +
                (gb->cart_ram_bank * CRAM_BANK_SIZE));
    }
    else
      return gb_cart_ram_read(gb, addr - CART_RAM_ADDR);
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

  if((addr >= 0xFF10) && (addr <= 0xFF3F))
  {
#ifdef ENABLE_SOUND
    uint32_t b = audio_read(addr, gb->counter.sound_count);
    gb->counter.sound_count = 0;
    return b;
#else
    return 0xFF;
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
    return (gb->gb_reg.STAT & STAT_USER_BITS) | gb->lcd_mode;

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

#ifdef  ENABLE_GBC
  case 0x4D:
    return gb->gb_reg.KEY1;
  case 0x4F:
    return gb->gb_reg.VBK | 0xFE;
  case 0x51:
    return gb->gb_reg.HDMA1;
  case 0x52:
    return gb->gb_reg.HDMA2;
  case 0x53:
    return gb->gb_reg.HDMA3;
  case 0x54:
    return gb->gb_reg.HDMA4;
  case 0x55:
    return gb->gb_reg.HDMA5;
  case 0x68:
    return gb->gb_reg.BGPI;
  case 0x69:
    return gb->display.bg_palette[gb->gb_reg.BGPI & 0x3F];
  case 0x6A:
    return gb->gb_reg.OBPI;
  case 0x6B:
    return gb->display.sp_palette[gb->gb_reg.OBPI & 0x3F];
  case 0x56:
    return gb->gb_reg.RP;
  case 0x70:
    return gb->gb_reg.SVBK;
  case 0x6C:
    return gb->gb_reg.REG6C;
  case 0x72:
    return gb->gb_reg.REG72;
  case 0x73:
    return gb->gb_reg.REG73;
  case 0x74:
    return gb->gb_reg.REG74;
  case 0x75:
    return gb->gb_reg.REG75;
#endif

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
static inline uint8_t __gb_read(struct gb_s *gb, const uint_fast16_t addr)
{
  uint32_t baddr = addr >> 12;
  uint8_t *paddr = gb->addr_map[baddr];
//if (addr == gb->cpu_reg.pc)
//  stats[baddr]++;
//if (++statcnt == 10000000) {
//  statcnt = 0;
//  for (int i = 0; i < 16; i++)
//    printf("%d %d\n", i, stats[i]);
//}
  if (paddr != NULL) {
//  uint32_t da = addr + paddr;
//  return (*(uint32_t *)(da & 0xFFFFFFFC)) >> ((da & 3) * 8);
/*  register uint32_t da asm("a3") = addr + paddr;
  uint32_t r;
  asm volatile (
      "movi.n     %0, -4      \n"
      "and        %0, a3, %0  \n"
      "l32i.n     %0, %0, 0   \n"
      "ssa8l      a3          \n"
      "srl        %0, %0      \n"
      : "=r"(r) : "r"(da)
  );
  return r;*/
    return paddr[addr];
  }
  uint8_t (*fn)(struct gb_s *gb, const uint_fast16_t addr) = gb->addr_read[baddr];
  return fn(gb, addr);
}

uint32_t __gb_read16(struct gb_s *gb, const uint_fast16_t addr)
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

  if((addr >= 0xFF10) && (addr <= 0xFF3F))
  {
#ifdef ENABLE_SOUND
    audio_write(addr, val, gb->counter.sound_count);
    gb->counter.sound_count = 0;
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
    gb_serial_ready(gb);
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
    gb_set_palette(gb->display.gb_palette[2][(gb->gb_reg.BGP & 0x03)], 0 | LCD_PALETTE_BG);
    gb_set_palette(gb->display.gb_palette[2][((gb->gb_reg.BGP >> 2) & 0x03)], 1 | LCD_PALETTE_BG);
    gb_set_palette(gb->display.gb_palette[2][((gb->gb_reg.BGP >> 4) & 0x03)], 2 | LCD_PALETTE_BG);
    gb_set_palette(gb->display.gb_palette[2][((gb->gb_reg.BGP >> 6) & 0x03)], 3 | LCD_PALETTE_BG);
    return;

  case 0x48:
    gb->gb_reg.OBP0 = val;
    gb_set_palette(gb->display.gb_palette[0][(gb->gb_reg.OBP0 & 0x03)], 0 | LCD_PALETTE_ALL);
    gb_set_palette(gb->display.gb_palette[0][((gb->gb_reg.OBP0 >> 2) & 0x03)], 1 | LCD_PALETTE_ALL);
    gb_set_palette(gb->display.gb_palette[0][((gb->gb_reg.OBP0 >> 4) & 0x03)], 2 | LCD_PALETTE_ALL);
    gb_set_palette(gb->display.gb_palette[0][((gb->gb_reg.OBP0 >> 6) & 0x03)], 3 | LCD_PALETTE_ALL);
    return;

  case 0x49:
    gb->gb_reg.OBP1 = val;
    gb_set_palette(gb->display.gb_palette[1][(gb->gb_reg.OBP1 & 0x03)], 0 | LCD_PALETTE_OBJ | LCD_PALETTE_ALL);
    gb_set_palette(gb->display.gb_palette[1][((gb->gb_reg.OBP1 >> 2) & 0x03)], 1 | LCD_PALETTE_OBJ | LCD_PALETTE_ALL);
    gb_set_palette(gb->display.gb_palette[1][((gb->gb_reg.OBP1 >> 4) & 0x03)], 2 | LCD_PALETTE_OBJ | LCD_PALETTE_ALL);
    gb_set_palette(gb->display.gb_palette[1][((gb->gb_reg.OBP1 >> 6) & 0x03)], 3 | LCD_PALETTE_OBJ | LCD_PALETTE_ALL);
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

#ifdef  ENABLE_GBC
  case 0x4D:
    gb->gb_reg.KEY1 = (gb->gb_reg.KEY1 & 0xFE) | (val & 1);
    break;
  case 0x4F:
    {
    gb->gb_reg.VBK = val & 1;
    uint8_t *vra = gb->vram + gb->gb_reg.VBK * VRAM_BANK_SIZE - VRAM_ADDR;
    gb->addr_map[8] = vra;
    gb->addr_map[9] = vra;
    gb->addr_wmap[8] = vra;
    gb->addr_wmap[9] = vra;
    }
    break;
  case 0x51:
    gb->gb_reg.HDMA1 = val;
    break;
  case 0x52:
    gb->gb_reg.HDMA2 = val & 0xF0;
    break;
  case 0x53:
    gb->gb_reg.HDMA3 = val;
    break;
  case 0x54:
    gb->gb_reg.HDMA4 = val & 0xF0;
    break;
  case 0x55:
    {
    gb->gb_reg.HDMA5 = val;
    if (val & 0x80) {
      gb->gb_reg.HDMA5 = val & 0x7F;
      return;
    }
    uint16_t src = (gb->gb_reg.HDMA1 << 8) | gb->gb_reg.HDMA2;
    uint16_t dst = 0x8000 | ((gb->gb_reg.HDMA3 & 0x1F) << 8) | gb->gb_reg.HDMA4;
    uint32_t cnt = (val + 1) << 4;
    uint8_t *da = gb->addr_wmap[8] + dst;
    while (cnt--)
      *da++ = __gb_read(gb, src++);
    dst += cnt;
    gb->gb_reg.HDMA1 = src >> 8;
    gb->gb_reg.HDMA2 = src & 0xF0;
    gb->gb_reg.HDMA3 = (dst >> 8) & 0x1F;
    gb->gb_reg.HDMA4 = dst & 0xF0;
    gb->gb_reg.HDMA5 = 0xFF;
    }
    break;
  case 0x68:
    gb->gb_reg.BGPI = val;
    break;
  case 0x69:
    {
    gb->gb_reg.BGPD = val;
    gb->display.bg_palette[gb->gb_reg.BGPI & 0x3F] = val;
    uint32_t idx = (gb->gb_reg.BGPI & 0x3F) >> 1;
    uint16_t c = gb->display.bg_palette[idx * 2] | (gb->display.bg_palette[idx * 2 + 1] << 8);
    gb_set_palette(c, idx | LCD_GBC_PAL_BG);
    gb_set_palette(c, idx | LCD_GBC_PAL_BG_PRI);
    if (gb->gb_reg.BGPI & 0x80)
      gb->gb_reg.BGPI = (gb->gb_reg.BGPI & 0x80) | ((gb->gb_reg.BGPI + 1) & 0x3F); 
    }
    break;
  case 0x6A:
    gb->gb_reg.OBPI = val;
    break;
  case 0x6B:
    {
    gb->gb_reg.OBPD = val;
    gb->display.sp_palette[gb->gb_reg.OBPI & 0x3F] = val;
    uint32_t idx = (gb->gb_reg.OBPI & 0x3F) >> 1;
    gb_set_palette(gb->display.sp_palette[idx * 2] | (gb->display.sp_palette[idx * 2 + 1] << 8), idx | LCD_GBC_PAL_OBJ);
    if (gb->gb_reg.OBPI & 0x80)
      gb->gb_reg.OBPI = (gb->gb_reg.OBPI & 0x80) | ((gb->gb_reg.OBPI + 1) & 0x3F); 
    }
    break;
  case 0x56:
    gb->gb_reg.RP = val & 0xC3;
    break;
  case 0x70:
    {
    gb->gb_reg.SVBK = val & 7;
    uint32_t v = gb->gb_reg.SVBK == 0 ? 1 : gb->gb_reg.SVBK;
    gb->addr_map[13] = gb->wram - WRAM_1_ADDR + WRAM_BANK_SIZE * v;
    gb->addr_wmap[13] = gb->wram - WRAM_1_ADDR + WRAM_BANK_SIZE * v;
    }
    break;
  case 0x6C:
    gb->gb_reg.REG6C = (val & 1) | 0xFE;
    break;
  case 0x72:
    gb->gb_reg.REG72 = val;
    break;
  case 0x73:
    gb->gb_reg.REG73 = val;
    break;
  case 0x74:
    gb->gb_reg.REG74 = val;
    break;
  case 0x75:
    gb->gb_reg.REG75 = val & 0x70;
    break;
#endif

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
    __update_addr_map(gb);
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
      gb_cart_ram_write(gb, addr - CART_RAM_ADDR + (gb->cart_ram_bank * CRAM_BANK_SIZE), val);
    }
    else if(gb->num_ram_banks)
      gb_cart_ram_write(gb, addr - CART_RAM_ADDR, val);
  }
}

/**
 * Internal function used to write bytes.
 */
static inline void __gb_write(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val)
{
  uint32_t baddr = addr >> 12;
  uint8_t *paddr = gb->addr_wmap[baddr];
//stats[baddr]++;

  if (paddr != NULL) {
    paddr[addr] = val;
    return;
  }
  void (*fn)(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) = gb->addr_write[baddr];
  fn(gb, addr, val);
}

void __update_addr_map(struct gb_s *gb) {
  if(gb->mbc == 1 && gb->cart_mode_select)
    gb->rom_bank_addr = ((gb->selected_rom_bank & 0x1F) - 1) * ROM_BANK_SIZE + FLASH_ADDR(GAME_ROM_ADDR);
  else
    gb->rom_bank_addr = (gb->selected_rom_bank - 1) * ROM_BANK_SIZE + FLASH_ADDR(GAME_ROM_ADDR);
}

uint8_t gb_rom_bank_read(struct gb_s *gb, const uint_fast32_t addr);

void __init_addr_map(struct gb_s *gb) {
  memset(gb->addr_map, 0, sizeof(gb->addr_map));
  gb->addr_map[0] = gb->addr_map[1] = gb->addr_map[2] = gb->addr_map[3] = gb->rom_bank0;
  gb->addr_map[8] = gb->vram - VRAM_ADDR;
  gb->addr_map[9] = gb->vram - VRAM_ADDR;
  gb->addr_map[12] = gb->wram - WRAM_0_ADDR;
  gb->addr_map[13] = gb->wram - WRAM_1_ADDR + WRAM_BANK_SIZE;
  gb->addr_map[14] = gb->wram - ECHO_ADDR;

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

  __update_addr_map(gb);
}

/**
 * Resets the context, and initialises startup values.
 */
void ROCODE gb_reset(struct gb_s *gb)
{
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
#ifdef  ENABLE_GBC
  if (gb->is_gbc)
    gb->cpu_reg.a = 0x11;
#endif

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

  __init_addr_map(gb);

	__gb_write(gb, 0xFF47, 0xFC);    // BGP
	__gb_write(gb, 0xFF48, 0xFF);    // OBJP0
	__gb_write(gb, 0xFF49, 0x0F);    // OBJP1
	gb->gb_reg.WY        = 0x00;
	gb->gb_reg.WX        = 0x00;
	gb->gb_reg.IE        = 0x00;

	gb->direct.joypad = 0xFF;
	gb->gb_reg.P1 = 0xCF;

#ifdef  ENABLE_GBC
  gb->gb_reg.KEY1 = 0;
  gb->gb_reg.VBK = 0;
  gb->gb_reg.HDMA1 = 0;
  gb->gb_reg.HDMA2 = 0;
  gb->gb_reg.HDMA3 = 0;
  gb->gb_reg.HDMA4 = 0;
  gb->gb_reg.HDMA5 = 0xFF;
  gb->gb_reg.BGPI = 0;
  gb->gb_reg.BGPD = 0;
  gb->gb_reg.OBPI = 0;
  gb->gb_reg.OBPD = 0;
  gb->gb_reg.RP = 0;
  gb->gb_reg.SVBK = 1;
  gb->gb_reg.REG6C = 0xFE;
  gb->gb_reg.REG72 = 0;
  gb->gb_reg.REG73 = 0;
  gb->gb_reg.REG74 = 0;
  gb->gb_reg.REG75 = 0;
#endif

#ifdef  ENABLE_SOUND
  gb->counter.sound_count = 0;
  audio_reset();
#endif
#if ENABLE_LCD
	gb->direct.frame_skip = 0;
	gb->display.frame_skip_count = 0;

	gb->display.window_clear = 0;
	gb->display.WY = 0;
#endif
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

	uint8_t *pixels = gb->display.framebuffer + 512;

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
		uint32_t t1 = gb->vram[tile] >> px;
		uint32_t t2 = (gb->vram[tile + 1] >> px) << 1;

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
				t2 = gb->vram[tile + 1] << 1;
			}

			/* copy background */
			uint32_t c = (t1 & 0x1) | (t2 & 0x2);
			//pixels[disp_x] = c | LCD_PALETTE_BG;
			pixels[disp_x] = c;   // LCD_PALETTE_BG = 0
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
		uint32_t t1 = gb->vram[tile] >> px;
		uint32_t t2 = (gb->vram[tile + 1] >> px) << 1;

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
				t2 = gb->vram[tile + 1] << 1;
			}

			// copy window
			uint8_t c = (t1 & 0x1) | (t2 & 0x2);
			//pixels[disp_x] = c | LCD_PALETTE_BG;
			pixels[disp_x] = c;   // LCD_PALETTE_BG = 0
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
      //t2 <<= 1;

      uint8_t obj = ((OF & OBJ_PALETTE) >> 2) | LCD_PALETTE_ALL;

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
				if(c && !((OF & OBJ_PRIORITY)
						&& (pixels[disp_x] & 0x3)))
#endif
				{
					/* Set pixel colour. */
					pixels[disp_x] = c | obj;
				}
				t1 = t1 >> 1;
				t2 = t2 >> 1;
			}
		}
	}
  gb->display.line_cnt = 160;
  gb_lcd_send_chunk(gb);
}

#ifdef  ENABLE_GBC

void __gb_draw_gbc_line(struct gb_s *gb)
{
  if (gb->gb_reg.LY < 8 || gb->gb_reg.LY > 127 + 8)
    return;

	if(gb->display.frame_skip_count != 0)
		return;

	uint8_t *pixels = gb->display.framebuffer + 512;

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
    uint8_t attr = gb->vram[bg_map + (bg_x >> 3) + VRAM_BANK_SIZE];

		/* FIXME: Y coordinate of tile pixel to draw? */
		const uint8_t py = (bg_y & 0x07);
		/* FIXME: X coordinate of tile pixel to draw? */
		uint8_t px = 7 - (bg_x & 0x07);

		uint16_t tile;

		/* Select addressing mode. */
		if(gb->gb_reg.LCDC & LCDC_TILE_SELECT)
			tile = VRAM_TILES_1 + idx * 0x10 + ((attr & 0x08) << 10);
		else
			tile = VRAM_TILES_2 + ((idx + 0x80) % 0x100) * 0x10 + ((attr & 0x08) << 10);

		tile += 2 * py;

		/* fetch first tile */
		uint32_t t1 = gb->vram[tile] >> px;
		uint32_t t2 = (gb->vram[tile + 1] >> px) << 1;
    uint32_t bgpal = ((attr & 7) << 2) | LCD_GBC_PAL_BG;

		for(; disp_x != 0xFF; disp_x--)
		{
			if(px == 8)
			{
				/* fetch next tile */
				px = 0;
				bg_x = disp_x + gb->gb_reg.SCX;
				idx = gb->vram[bg_map + (bg_x >> 3)];
        attr = gb->vram[bg_map + (bg_x >> 3) + VRAM_BANK_SIZE];
        bgpal = ((attr & 7) << 2) | LCD_GBC_PAL_BG;

				if(gb->gb_reg.LCDC & LCDC_TILE_SELECT)
					tile = VRAM_TILES_1 + idx * 0x10 + ((attr & 0x08) << 10);
				else
					tile = VRAM_TILES_2 + ((idx + 0x80) % 0x100) * 0x10 + ((attr & 0x08) << 10);

				tile += 2 * py;
				t1 = gb->vram[tile];
				t2 = gb->vram[tile + 1] << 1;
			}

			/* copy background */
			uint32_t c = (t1 & 0x1) | (t2 & 0x2);
			pixels[disp_x] = c | bgpal;
			t1 = t1 >> 1;
			t2 = t2 >> 1;
			px++;
		}
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
    uint8_t attr = gb->vram[win_line + (win_x >> 3) + VRAM_BANK_SIZE];

		uint16_t tile;

		if(gb->gb_reg.LCDC & LCDC_TILE_SELECT)
			tile = VRAM_TILES_1 + idx * 0x10 + ((attr & 0x08) << 10);
		else
			tile = VRAM_TILES_2 + ((idx + 0x80) % 0x100) * 0x10 + ((attr & 0x08) << 10);

		tile += 2 * py;

		// fetch first tile
		uint32_t t1 = gb->vram[tile] >> px;
		uint32_t t2 = (gb->vram[tile + 1] >> px) << 1;
    uint32_t bgpal = ((attr & 7) << 2) | LCD_GBC_PAL_BG;

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
        attr = gb->vram[win_line + (win_x >> 3) + VRAM_BANK_SIZE];
        bgpal = ((attr & 7) << 2) | LCD_GBC_PAL_BG;

				if(gb->gb_reg.LCDC & LCDC_TILE_SELECT)
					tile = VRAM_TILES_1 + idx * 0x10 + ((attr & 0x08) << 10);
				else
					tile = VRAM_TILES_2 + ((idx + 0x80) % 0x100) * 0x10 + ((attr & 0x08) << 10);

				tile += 2 * py;
				t1 = gb->vram[tile];
				t2 = gb->vram[tile + 1] << 1;
			}

			// copy window
			uint8_t c = (t1 & 0x1) | (t2 & 0x2);
			pixels[disp_x] = c | bgpal;
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
			uint8_t t1 = gb->vram[VRAM_TILES_1 + OT * 0x10 + 2 * py + ((OF & 0x08) << 10)];
			uint8_t t2 = gb->vram[VRAM_TILES_1 + OT * 0x10 + 2 * py + 1 + ((OF & 0x08) << 10)];

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
      //t2 <<= 1;

      uint8_t obj = ((OF & 7) << 2) | LCD_GBC_PAL_OBJ;

			for(uint8_t disp_x = start; disp_x != end; disp_x += dir)
			{
				uint8_t c = (t1 & 0x1) | ((t2 & 0x1) << 1);
				// check transparency / sprite overlap / background overlap
				if(c && !((OF & OBJ_PRIORITY)
						&& (pixels[disp_x] & 0x3)))
				{
					/* Set pixel colour. */
					pixels[disp_x] = c | obj;
				}
				t1 = t1 >> 1;
				t2 = t2 >> 1;
			}
		}
	}
  gb->display.line_cnt = 160;
  gb_lcd_send_chunk(gb);
}
#endif
#endif

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

const uint_fast16_t TAC_CYCLES[4] IRDATA = {1024, 16, 64, 256};

/**
 * Internal function used to step the CPU.
 */
void __gb_step_cpu(struct gb_s *gb, uint32_t cycles)
{
	uint8_t opcode;
  uint32_t ccount = 0, inst_cycles;

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
    if (gb->gb_halt) {
      inst_cycles = cycles - ccount;
      uint32_t divcc = DIV_CYCLES - gb->counter.div_count;
      if(gb->gb_reg.tac_enable) {
        uint32_t taccc = TAC_CYCLES[gb->gb_reg.tac_rate] - gb->counter.tima_count;
        if (taccc < inst_cycles)
          inst_cycles = taccc;
      }
      if (divcc < inst_cycles)
        inst_cycles = divcc;
      inst_cycles = (inst_cycles & 0xFFFC) + 4;
    } else {
      /* Obtain opcode */
      opcode = __gb_read(gb, gb->cpu_reg.pc);
      inst_cycles = gb->cpu_cycles[opcode];
      uint16_t opc_s = gb->cpu_sizes[opcode];
      uint32_t opd = 0;
      if (opc_s == 2)
        opd = __gb_read(gb, gb->cpu_reg.pc + 1);
      else if (opc_s == 3)
        opd = __gb_read16(gb, gb->cpu_reg.pc + 1);
//printf("%04X %02X %04X %02X\n", gb->cpu_reg.pc, opcode, opd, gb->cpu_reg.a);
      gb->cpu_reg.pc += opc_s;

      switch (opcode) {
  case 0x00:  /* NOP */
    break;

  case 0x01:  /* LD BC, imm */
		gb->cpu_reg.bc = opd;
    break;

  case 0x02:  /* LD (BC), A */
		__gb_write(gb, gb->cpu_reg.bc, gb->cpu_reg.a);
    break;

  case 0x03:  /* INC BC */
		gb->cpu_reg.bc++;
    break;

  case 0x04:  /* INC B */
		gb->cpu_reg.b++;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.b == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.b & 0x0F) == 0x00);
    break;

  case 0x05:  /* DEC B */
		gb->cpu_reg.b--;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.b == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.b & 0x0F) == 0x0F);
    break;

  case 0x06:  /* LD B, imm */
		gb->cpu_reg.b = opd;
    break;

  case 0x07:  /* RLCA */
		gb->cpu_reg.a = (gb->cpu_reg.a << 1) | (gb->cpu_reg.a >> 7);
		gb->cpu_reg.f = (gb->cpu_reg.a & 0x01) << 4;
    break;

  case 0x08:  /* LD (imm), SP */
	{
		uint16_t temp = opd;
		__gb_write(gb, temp++, gb->cpu_reg.sp & 0xFF);
		__gb_write(gb, temp, gb->cpu_reg.sp >> 8);
}
	    break;

  case 0x09:  /* ADD HL, BC */
	{
		uint_fast32_t temp = gb->cpu_reg.hl + gb->cpu_reg.bc;
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(temp ^ gb->cpu_reg.hl ^ gb->cpu_reg.bc) & 0x1000 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 16;
		gb->cpu_reg.hl = temp;
}
	    break;

  case 0x0A:  /* LD A, (BC) */
		gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.bc);
    break;

  case 0x0B:  /* DEC BC */
		gb->cpu_reg.bc--;
    break;

  case 0x0C:  /* INC C */
		gb->cpu_reg.c++;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.c == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.c & 0x0F) == 0x00);
    break;

  case 0x0D:  /* DEC C */
		gb->cpu_reg.c--;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.c == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.c & 0x0F) == 0x0F);
    break;

  case 0x0E:  /* LD C, imm */
		gb->cpu_reg.c = (opd);
    break;

  case 0x0F:  /* RRCA */
		gb->cpu_reg.f = (gb->cpu_reg.a & 0x01) << 4;
		gb->cpu_reg.a = (gb->cpu_reg.a >> 1) | (gb->cpu_reg.a << 7);
    break;

  case 0x10:  /* STOP */
#ifdef  ENABLE_GBC
    if (gb->gb_reg.KEY1 & 1) {
#ifdef  OCEMU
printf("CPU SPEED: %d\n", gb->gb_reg.KEY1);
#endif
      gb->gb_reg.KEY1 ^= 0x80;
      gb->gb_reg.KEY1 &= 0xFE;
    }
#endif
    break;

  case 0x11:  /* LD DE, imm */
		gb->cpu_reg.de = (opd);
    break;

  case 0x12:  /* LD (DE), A */
		__gb_write(gb, gb->cpu_reg.de, gb->cpu_reg.a);
    break;

  case 0x13:  /* INC DE */
		gb->cpu_reg.de++;
    break;

  case 0x14:  /* INC D */
		gb->cpu_reg.d++;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.d == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.d & 0x0F) == 0x00);
    break;

  case 0x15:  /* DEC D */
		gb->cpu_reg.d--;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.d == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.d & 0x0F) == 0x0F);
    break;

  case 0x16:  /* LD D, imm */
		gb->cpu_reg.d = (opd);
    break;

  case 0x17:  /* RLA */
	{
		uint8_t temp = gb->cpu_reg.a;
		gb->cpu_reg.a = (gb->cpu_reg.a << 1) | gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f = ((temp >> 7) & 0x01) << 4;
}
	    break;

  case 0x18:  /* JR imm */
	{
		int8_t temp = (int8_t) (opd);
		gb->cpu_reg.pc += temp;
}
	    break;

  case 0x19:  /* ADD HL, DE */
	{
		uint_fast32_t temp = gb->cpu_reg.hl + gb->cpu_reg.de;
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(temp ^ gb->cpu_reg.hl ^ gb->cpu_reg.de) & 0x1000 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 16;
		gb->cpu_reg.hl = temp;
}
	    break;

  case 0x1A:  /* LD A, (DE) */
		gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.de);
    break;

  case 0x1B:  /* DEC DE */
		gb->cpu_reg.de--;
    break;

  case 0x1C:  /* INC E */
		gb->cpu_reg.e++;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.e == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.e & 0x0F) == 0x00);
    break;

  case 0x1D:  /* DEC E */
		gb->cpu_reg.e--;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.e == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.e & 0x0F) == 0x0F);
    break;

  case 0x1E:  /* LD E, imm */
		gb->cpu_reg.e = (opd);
    break;

  case 0x1F:  /* RRA */
	{
		uint8_t temp = gb->cpu_reg.a;
		gb->cpu_reg.a = gb->cpu_reg.a >> 1 | (gb->cpu_reg.f_bits.c << 7);
		gb->cpu_reg.f = (temp & 0x1) << 4;
}
	    break;

  case 0x20:  /* JP NZ, imm */
		if(!gb->cpu_reg.f_bits.z)
		{
			int8_t temp = (int8_t) (opd);
			gb->cpu_reg.pc += temp;
			inst_cycles += 4;
		}
    break;

  case 0x21:  /* LD HL, imm */
		gb->cpu_reg.hl = (opd);
    break;

  case 0x22:  /* LDI (HL), A */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.a);
		gb->cpu_reg.hl++;
    break;

  case 0x23:  /* INC HL */
		gb->cpu_reg.hl++;
    break;

  case 0x24:  /* INC H */
		gb->cpu_reg.h++;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.h == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.h & 0x0F) == 0x00);
    break;

  case 0x25:  /* DEC H */
		gb->cpu_reg.h--;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.h == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.h & 0x0F) == 0x0F);
    break;

  case 0x26:  /* LD H, imm */
		gb->cpu_reg.h = (opd);
    break;

  case 0x27:  /* DAA */
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
	    break;

  case 0x28:  /* JP Z, imm */
		if(gb->cpu_reg.f_bits.z)
		{
			int8_t temp = (int8_t) (opd);
			gb->cpu_reg.pc += temp;
			inst_cycles += 4;
		}
    break;

  case 0x29:  /* ADD HL, HL */
	{
		uint_fast32_t temp = gb->cpu_reg.hl + gb->cpu_reg.hl;
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = (temp & 0x1000) ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 16;
		gb->cpu_reg.hl = temp;
}
	    break;

  case 0x2A:  /* LD A, (HL+) */
		gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.hl++);
    break;

  case 0x2B:  /* DEC HL */
		gb->cpu_reg.hl--;
    break;

  case 0x2C:  /* INC L */
		gb->cpu_reg.l++;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.l == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.l & 0x0F) == 0x00);
    break;

  case 0x2D:  /* DEC L */
		gb->cpu_reg.l--;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.l == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.l & 0x0F) == 0x0F);
    break;

  case 0x2E:  /* LD L, imm */
		gb->cpu_reg.l = (opd);
    break;

  case 0x2F:  /* CPL */
		gb->cpu_reg.a = ~gb->cpu_reg.a;
		gb->cpu_reg.f  |= 0x60;
    break;

  case 0x30:  /* JP NC, imm */
		if(!gb->cpu_reg.f_bits.c)
		{
			int8_t temp = (int8_t) (opd);
			gb->cpu_reg.pc += temp;
			inst_cycles += 4;
		}
    break;

  case 0x31:  /* LD SP, imm */
		gb->cpu_reg.sp = (opd);
    break;

  case 0x32:  /* LD (HL), A */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.a);
		gb->cpu_reg.hl--;
    break;

  case 0x33:  /* INC SP */
		gb->cpu_reg.sp++;
    break;

  case 0x34:  /* INC (HL) */
	{
		uint8_t temp = __gb_read(gb, gb->cpu_reg.hl) + 1;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((temp & 0x0F) == 0x00);
		__gb_write(gb, gb->cpu_reg.hl, temp);
}
	    break;

  case 0x35:  /* DEC (HL) */
	{
		uint8_t temp = __gb_read(gb, gb->cpu_reg.hl) - 1;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((temp & 0x0F) == 0x0F);
		__gb_write(gb, gb->cpu_reg.hl, temp);
}
	    break;

  case 0x36:  /* LD (HL), imm */
		__gb_write(gb, gb->cpu_reg.hl, (opd));
    break;

  case 0x37:  /* SCF */
		gb->cpu_reg.f &= 0x90;
		gb->cpu_reg.f_bits.c = 1;
    break;

  case 0x38:  /* JP C, imm */
		if(gb->cpu_reg.f_bits.c)
		{
			int8_t temp = (int8_t) (opd);
			gb->cpu_reg.pc += temp;
			inst_cycles += 4;
		}
    break;

  case 0x39:  /* ADD HL, SP */
	{
		uint_fast32_t temp = gb->cpu_reg.hl + gb->cpu_reg.sp;
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			((gb->cpu_reg.hl & 0xFFF) + (gb->cpu_reg.sp & 0xFFF)) & 0x1000 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 16;
		gb->cpu_reg.hl = (uint16_t)temp;
}
	    break;

  case 0x3A:  /* LD A, (HL) */
		gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.hl--);
    break;

  case 0x3B:  /* DEC SP */
		gb->cpu_reg.sp--;
    break;

  case 0x3C:  /* INC A */
		gb->cpu_reg.a++;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.a & 0x0F) == 0x00);
    break;

  case 0x3D:  /* DEC A */
		gb->cpu_reg.a--;
		gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.a & 0x0F) == 0x0F);
    break;

  case 0x3E:  /* LD A, imm */
		gb->cpu_reg.a = (opd);
    break;

  case 0x3F:  /* CCF */
		gb->cpu_reg.f &= 0x90;
		gb->cpu_reg.f_bits.c = ~gb->cpu_reg.f_bits.c;
    break;

  case 0x40:  /* LD B, B */
    break;

  case 0x41:  /* LD B, C */
		gb->cpu_reg.b = gb->cpu_reg.c;
    break;

  case 0x42:  /* LD B, D */
		gb->cpu_reg.b = gb->cpu_reg.d;
    break;

  case 0x43:  /* LD B, E */
		gb->cpu_reg.b = gb->cpu_reg.e;
    break;

  case 0x44:  /* LD B, H */
		gb->cpu_reg.b = gb->cpu_reg.h;
    break;

  case 0x45:  /* LD B, L */
		gb->cpu_reg.b = gb->cpu_reg.l;
    break;

  case 0x46:  /* LD B, (HL) */
		gb->cpu_reg.b = __gb_read(gb, gb->cpu_reg.hl);
    break;

  case 0x47:  /* LD B, A */
		gb->cpu_reg.b = gb->cpu_reg.a;
    break;

  case 0x48:  /* LD C, B */
		gb->cpu_reg.c = gb->cpu_reg.b;
    break;

  case 0x49:  /* LD C, C */
    break;

  case 0x4A:  /* LD C, D */
		gb->cpu_reg.c = gb->cpu_reg.d;
    break;

  case 0x4B:  /* LD C, E */
		gb->cpu_reg.c = gb->cpu_reg.e;
    break;

  case 0x4C:  /* LD C, H */
		gb->cpu_reg.c = gb->cpu_reg.h;
    break;

  case 0x4D:  /* LD C, L */
		gb->cpu_reg.c = gb->cpu_reg.l;
    break;

  case 0x4E:  /* LD C, (HL) */
		gb->cpu_reg.c = __gb_read(gb, gb->cpu_reg.hl);
    break;

  case 0x4F:  /* LD C, A */
		gb->cpu_reg.c = gb->cpu_reg.a;
    break;

  case 0x50:  /* LD D, B */
		gb->cpu_reg.d = gb->cpu_reg.b;
    break;

  case 0x51:  /* LD D, C */
		gb->cpu_reg.d = gb->cpu_reg.c;
    break;

  case 0x52:  /* LD D, D */
    break;

  case 0x53:  /* LD D, E */
		gb->cpu_reg.d = gb->cpu_reg.e;
    break;

  case 0x54:  /* LD D, H */
		gb->cpu_reg.d = gb->cpu_reg.h;
    break;

  case 0x55:  /* LD D, L */
		gb->cpu_reg.d = gb->cpu_reg.l;
    break;

  case 0x56:  /* LD D, (HL) */
		gb->cpu_reg.d = __gb_read(gb, gb->cpu_reg.hl);
    break;

  case 0x57:  /* LD D, A */
		gb->cpu_reg.d = gb->cpu_reg.a;
    break;

  case 0x58:  /* LD E, B */
		gb->cpu_reg.e = gb->cpu_reg.b;
    break;

  case 0x59:  /* LD E, C */
		gb->cpu_reg.e = gb->cpu_reg.c;
    break;

  case 0x5A:  /* LD E, D */
		gb->cpu_reg.e = gb->cpu_reg.d;
    break;

  case 0x5B:  /* LD E, E */
    break;

  case 0x5C:  /* LD E, H */
		gb->cpu_reg.e = gb->cpu_reg.h;
    break;

  case 0x5D:  /* LD E, L */
		gb->cpu_reg.e = gb->cpu_reg.l;
    break;

  case 0x5E:  /* LD E, (HL) */
		gb->cpu_reg.e = __gb_read(gb, gb->cpu_reg.hl);
    break;

  case 0x5F:  /* LD E, A */
		gb->cpu_reg.e = gb->cpu_reg.a;
    break;

  case 0x60:  /* LD H, B */
		gb->cpu_reg.h = gb->cpu_reg.b;
    break;

  case 0x61:  /* LD H, C */
		gb->cpu_reg.h = gb->cpu_reg.c;
    break;

  case 0x62:  /* LD H, D */
		gb->cpu_reg.h = gb->cpu_reg.d;
    break;

  case 0x63:  /* LD H, E */
		gb->cpu_reg.h = gb->cpu_reg.e;
    break;

  case 0x64:  /* LD H, H */
    break;

  case 0x65:  /* LD H, L */
		gb->cpu_reg.h = gb->cpu_reg.l;
    break;

  case 0x66:  /* LD H, (HL) */
		gb->cpu_reg.h = __gb_read(gb, gb->cpu_reg.hl);
    break;

  case 0x67:  /* LD H, A */
		gb->cpu_reg.h = gb->cpu_reg.a;
    break;

  case 0x68:  /* LD L, B */
		gb->cpu_reg.l = gb->cpu_reg.b;
    break;

  case 0x69:  /* LD L, C */
		gb->cpu_reg.l = gb->cpu_reg.c;
    break;

  case 0x6A:  /* LD L, D */
		gb->cpu_reg.l = gb->cpu_reg.d;
    break;

  case 0x6B:  /* LD L, E */
		gb->cpu_reg.l = gb->cpu_reg.e;
    break;

  case 0x6C:  /* LD L, H */
		gb->cpu_reg.l = gb->cpu_reg.h;
    break;

  case 0x6D:  /* LD L, L */
    break;

  case 0x6E:  /* LD L, (HL) */
		gb->cpu_reg.l = __gb_read(gb, gb->cpu_reg.hl);
    break;

  case 0x6F:  /* LD L, A */
		gb->cpu_reg.l = gb->cpu_reg.a;
    break;

  case 0x70:  /* LD (HL), B */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.b);
    break;

  case 0x71:  /* LD (HL), C */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.c);
    break;

  case 0x72:  /* LD (HL), D */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.d);
    break;

  case 0x73:  /* LD (HL), E */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.e);
    break;

  case 0x74:  /* LD (HL), H */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.h);
    break;

  case 0x75:  /* LD (HL), L */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.l);
    break;

  case 0x76:  /* HALT */
		/* TODO: Emulate HALT bug? */
		gb->gb_halt = 1;
    break;

  case 0x77:  /* LD (HL), A */
		__gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.a);
    break;

  case 0x78:  /* LD A, B */
		gb->cpu_reg.a = gb->cpu_reg.b;
    break;

  case 0x79:  /* LD A, C */
		gb->cpu_reg.a = gb->cpu_reg.c;
    break;

  case 0x7A:  /* LD A, D */
		gb->cpu_reg.a = gb->cpu_reg.d;
    break;

  case 0x7B:  /* LD A, E */
		gb->cpu_reg.a = gb->cpu_reg.e;
    break;

  case 0x7C:  /* LD A, H */
		gb->cpu_reg.a = gb->cpu_reg.h;
    break;

  case 0x7D:  /* LD A, L */
		gb->cpu_reg.a = gb->cpu_reg.l;
    break;

  case 0x7E:  /* LD A, (HL) */
		gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.hl);
    break;

  case 0x7F:  /* LD A, A */
    break;

  case 0x80:  /* ADD A, B */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.b;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.b ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x81:  /* ADD A, C */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.c ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x82:  /* ADD A, D */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.d;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.d ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x83:  /* ADD A, E */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.e;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.e ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x84:  /* ADD A, H */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.h;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.h ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x85:  /* ADD A, L */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.l;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.l ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x86:  /* ADD A, (HL) */
	{
		uint8_t hl = __gb_read(gb, gb->cpu_reg.hl);
		uint16_t temp = gb->cpu_reg.a + hl;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ hl ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x87:  /* ADD A, A */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.a;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = temp & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x88:  /* ADC A, B */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.b + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.b ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x89:  /* ADC A, C */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.c + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.c ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x8A:  /* ADC A, D */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.d + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.d ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x8B:  /* ADC A, E */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.e + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.e ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x8C:  /* ADC A, H */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.h + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.h ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x8D:  /* ADC A, L */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.l + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.l ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x8E:  /* ADC A, (HL) */
	{
		uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
		uint16_t temp = gb->cpu_reg.a + val + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ val ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x8F:  /* ADC A, A */
	{
		uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.a + gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = temp >> 4;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x90:  /* SUB B */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.b;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.b ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x91:  /* SUB C */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.c;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.c ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x92:  /* SUB D */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.d;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.d ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x93:  /* SUB E */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.e;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.e ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x94:  /* SUB H */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.h;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.h ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x95:  /* SUB L */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.l;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.l ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x96:  /* SUB (HL) */
	{
		uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
		uint16_t temp = gb->cpu_reg.a - val;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ val ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x97:  /* SUB A */
		gb->cpu_reg.a = 0;
		gb->cpu_reg.f = 0xC0;
    break;

  case 0x98:  /* SBC A, B */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.b - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.b ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x99:  /* SBC A, C */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.c - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.c ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x9A:  /* SBC A, D */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.d - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.d ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x9B:  /* SBC A, E */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.e - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.e ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x9C:  /* SBC A, H */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.h - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.h ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x9D:  /* SBC A, L */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.l - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.l ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x9E:  /* SBC A, (HL) */
	{
		uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
		uint16_t temp = gb->cpu_reg.a - val - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ val ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0x9F:  /* SBC A, A */
		gb->cpu_reg.a = gb->cpu_reg.f_bits.c ? 0xFF : 0x00;
		gb->cpu_reg.f_bits.z = gb->cpu_reg.f_bits.c ? 0x00 : 0x01;
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = gb->cpu_reg.f_bits.c;
    break;

  case 0xA0:  /* AND B */
		gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.b;
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
    break;

  case 0xA1:  /* AND C */
		gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.c;
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
    break;

  case 0xA2:  /* AND D */
		gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.d;
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
    break;

  case 0xA3:  /* AND E */
		gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.e;
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
    break;

  case 0xA4:  /* AND H */
		gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.h;
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
    break;

  case 0xA5:  /* AND L */
		gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.l;
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
    break;

  case 0xA6:  /* AND B */
		gb->cpu_reg.a = gb->cpu_reg.a & __gb_read(gb, gb->cpu_reg.hl);
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
    break;

  case 0xA7:  /* AND A */
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
    break;

  case 0xA8:  /* XOR B */
		gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.b;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xA9:  /* XOR C */
		gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.c;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xAA:  /* XOR D */
		gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.d;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xAB:  /* XOR E */
		gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.e;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xAC:  /* XOR H */
		gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.h;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xAD:  /* XOR L */
		gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.l;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xAE:  /* XOR (HL) */
		gb->cpu_reg.a = gb->cpu_reg.a ^ __gb_read(gb, gb->cpu_reg.hl);
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xAF:  /* XOR A */
		gb->cpu_reg.a = 0x00;
		gb->cpu_reg.f = 0x80;
    break;

  case 0xB0:  /* OR B */
		gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.b;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xB1:  /* OR C */
		gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.c;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xB2:  /* OR D */
		gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.d;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xB3:  /* OR E */
		gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.e;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xB4:  /* OR H */
		gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.h;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xB5:  /* OR L */
		gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.l;
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xB6:  /* OR (HL) */
		gb->cpu_reg.a = gb->cpu_reg.a | __gb_read(gb, gb->cpu_reg.hl);
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xB7:  /* OR A */
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xB8:  /* CP B */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.b;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.b ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
}
	    break;

  case 0xB9:  /* CP C */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.c;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.c ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
}
	    break;

  case 0xBA:  /* CP D */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.d;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.d ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
}
	    break;

  case 0xBB:  /* CP E */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.e;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.e ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
}
	    break;

  case 0xBC:  /* CP H */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.h;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.h ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
}
	    break;

  case 0xBD:  /* CP L */
	{
		uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.l;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ gb->cpu_reg.l ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
}
  break;

  case 0xBE:  /* CP B */
	{
		uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
		uint16_t temp = gb->cpu_reg.a - val;
		gb->cpu_reg.f_bits.z = (temp == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ val ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
}
	    break;

  case 0xBF:  /* CP A */
		gb->cpu_reg.f = 0xC0;
    break;

  case 0xC0:  /* RET NZ */
		if(!gb->cpu_reg.f_bits.z)
		{
			gb->cpu_reg.pc = __gb_read16(gb, gb->cpu_reg.sp);
      gb->cpu_reg.sp += 2;
			inst_cycles += 12;
		}
    break;

  case 0xC1:  /* POP BC */
    gb->cpu_reg.bc = __gb_read16(gb, gb->cpu_reg.sp);
    gb->cpu_reg.sp += 2;
    break;

  case 0xC2:  /* JP NZ, imm */
		if(!gb->cpu_reg.f_bits.z)
		{
			uint16_t temp = (opd);
			gb->cpu_reg.pc = temp;
			inst_cycles += 4;
		}
    break;

  case 0xC3:  /* JP imm */
	{
		uint16_t temp = (opd);
		gb->cpu_reg.pc = temp;
}
	    break;

  case 0xC4:  /* CALL NZ imm */
		if(!gb->cpu_reg.f_bits.z)
		{
			uint16_t temp = (opd);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
			gb->cpu_reg.pc = temp;
			inst_cycles += 12;
		}
    break;

  case 0xC5:  /* PUSH BC */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.b);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.c);
    break;

  case 0xC6:  /* ADD A, imm */
	{
		/* Taken from SameBoy, which is released under MIT Licence. */
		uint8_t value = (opd);
		uint16_t calc = gb->cpu_reg.a + value;
		gb->cpu_reg.f_bits.z = ((uint8_t)calc == 0) ? 1 : 0;
		gb->cpu_reg.f_bits.h =
			((gb->cpu_reg.a & 0xF) + (value & 0xF) > 0x0F) ? 1 : 0;
		gb->cpu_reg.f_bits.c = calc >> 8;
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.a = (uint8_t)calc;
}
	    break;

  case 0xC7:  /* RST 0x0000 */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0000;
    break;

  case 0xC8:  /* RET Z */
		if(gb->cpu_reg.f_bits.z)
		{
			gb->cpu_reg.pc = __gb_read16(gb, gb->cpu_reg.sp);
      gb->cpu_reg.sp += 2;
			inst_cycles += 12;
		}
    break;

  case 0xC9:  /* RET */
	{
			gb->cpu_reg.pc = __gb_read16(gb, gb->cpu_reg.sp);
      gb->cpu_reg.sp += 2;
}
	    break;

  case 0xCA:  /* JP Z, imm */
		if(gb->cpu_reg.f_bits.z)
		{
			uint16_t temp = (opd);
			gb->cpu_reg.pc = temp;
			inst_cycles += 4;
		}
    break;

  case 0xCB:  /* CB INST */
		__gb_execute_cb(gb, opd);
    break;

  case 0xCC:  /* CALL Z, imm */
		if(gb->cpu_reg.f_bits.z)
		{
			uint16_t temp = (opd);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
			gb->cpu_reg.pc = temp;
			inst_cycles += 12;
		}
    break;

  case 0xCD:  /* CALL imm */
	{
		uint16_t addr = (opd);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = addr;
	}
    break;

  case 0xCE:  /* ADC A, imm */
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
	    break;

  case 0xCF:  /* RST 0x0008 */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0008;
    break;

  case 0xD0:  /* RET NC */
		if(!gb->cpu_reg.f_bits.c)
		{
			gb->cpu_reg.pc = __gb_read16(gb, gb->cpu_reg.sp);
      gb->cpu_reg.sp += 2;
			inst_cycles += 12;
		}
    break;

  case 0xD1:  /* POP DE */
    gb->cpu_reg.de = __gb_read16(gb, gb->cpu_reg.sp);
    gb->cpu_reg.sp += 2;
    break;

  case 0xD2:  /* JP NC, imm */
		if(!gb->cpu_reg.f_bits.c)
		{
			uint16_t temp =  (opd);
			gb->cpu_reg.pc = temp;
			inst_cycles += 4;
		}
    break;

  case 0xD4:  /* CALL NC, imm */
		if(!gb->cpu_reg.f_bits.c)
		{
			uint16_t temp = (opd);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
			gb->cpu_reg.pc = temp;
			inst_cycles += 12;
		}
    break;

  case 0xD5:  /* PUSH DE */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.d);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.e);
    break;

  case 0xD6:  /* SUB imm */
	{
		uint8_t val = (opd);
		uint16_t temp = gb->cpu_reg.a - val;
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ val ^ temp) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp >> 8;
		gb->cpu_reg.a = temp;
}
	    break;

  case 0xD7:  /* RST 0x0010 */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0010;
    break;

  case 0xD8:  /* RET C */
		if(gb->cpu_reg.f_bits.c)
		{
			gb->cpu_reg.pc = __gb_read16(gb, gb->cpu_reg.sp);
      gb->cpu_reg.sp += 2;
			inst_cycles += 12;
		}
    break;

  case 0xD9:  /* RETI */
	{
    gb->cpu_reg.pc = __gb_read16(gb, gb->cpu_reg.sp);
    gb->cpu_reg.sp += 2;
		gb->gb_ime = 1;
	}
    break;

  case 0xDA:  /* JP C, imm */
		if(gb->cpu_reg.f_bits.c)
		{
			uint16_t addr = (opd);
			gb->cpu_reg.pc = addr;
			inst_cycles += 4;
		}
    break;

  case 0xDC:  /* CALL C, imm */
		if(gb->cpu_reg.f_bits.c)
		{
			uint16_t temp = (opd);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
			__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
			gb->cpu_reg.pc = temp;
			inst_cycles += 12;
		}
    break;

  case 0xDE:  /* SBC A, imm */
	{
		uint8_t temp_8 = (opd);
		uint16_t temp_16 = gb->cpu_reg.a - temp_8 - gb->cpu_reg.f_bits.c;
		gb->cpu_reg.f_bits.z = (temp_16 == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h =
			(gb->cpu_reg.a ^ temp_8 ^ temp_16) & 0x10 ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp_16 >> 8;
		gb->cpu_reg.a = temp_16;
}
	    break;

  case 0xDF:  /* RST 0x0018 */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0018;
    break;

  case 0xE0:  /* LD (0xFF00+imm), A */
		__gb_write(gb, 0xFF00 | ((opd) & 0xFF),
			   gb->cpu_reg.a);
    break;

  case 0xE1:  /* POP HL */
    gb->cpu_reg.hl = __gb_read16(gb, gb->cpu_reg.sp);
    gb->cpu_reg.sp += 2;
    break;

  case 0xE2:  /* LD (C), A */
		__gb_write(gb, 0xFF00 | gb->cpu_reg.c, gb->cpu_reg.a);
    break;

  case 0xE5:  /* PUSH HL */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.h);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.l);
    break;

  case 0xE6:  /* AND imm */
		/* TODO: Optimisation? */
		gb->cpu_reg.a = gb->cpu_reg.a & (opd);
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
    break;

  case 0xE7:  /* RST 0x0020 */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0020;
    break;

  case 0xE8:  /* ADD SP, imm */
	{
		int8_t offset = (int8_t) (opd);
		/* TODO: Move flag assignments for optimisation. */
		gb->cpu_reg.f_bits.z = 0;
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.sp & 0xF) + (offset & 0xF) > 0xF) ? 1 : 0;
		gb->cpu_reg.f_bits.c = ((gb->cpu_reg.sp & 0xFF) + (offset & 0xFF) > 0xFF);
		gb->cpu_reg.sp += offset;
}
	    break;

  case 0xE9:  /* JP (HL) */
		gb->cpu_reg.pc = gb->cpu_reg.hl;
    break;

  case 0xEA:  /* LD (imm), A */
	{
		uint16_t addr = (opd);
		__gb_write(gb, addr, gb->cpu_reg.a);
}
	    break;

  case 0xEE:  /* XOR imm */
		gb->cpu_reg.a = gb->cpu_reg.a ^ (opd);
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xEF:  /* RST 0x0028 */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0028;
    break;

  case 0xF0:  /* LD A, (0xFF00+imm) */
		gb->cpu_reg.a =
			__gb_read(gb, 0xFF00 | (opd));
    break;

  case 0xF1:  /* POP AF */
	{
    gb->cpu_reg.af = __gb_read16(gb, gb->cpu_reg.sp);
    gb->cpu_reg.f &= 0xF0;
    gb->cpu_reg.sp += 2;
}
	    break;

  case 0xF2:  /* LD A, (C) */
		gb->cpu_reg.a = __gb_read(gb, 0xFF00 | gb->cpu_reg.c);
    break;

  case 0xF3:  /* DI */
		gb->gb_ime = 0;
    break;

  case 0xF5:  /* PUSH AF */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.a);
		__gb_write(gb, --gb->cpu_reg.sp,
			   gb->cpu_reg.f_bits.z << 7 | gb->cpu_reg.f_bits.n << 6 |
			   gb->cpu_reg.f_bits.h << 5 | gb->cpu_reg.f_bits.c << 4);
    break;

  case 0xF6:  /* OR imm */
		gb->cpu_reg.a = gb->cpu_reg.a | (opd);
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xF7:  /* PUSH AF */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0030;
    break;

  case 0xF8:  /* LD HL, SP+/-imm */
	{
		/* Taken from SameBoy, which is released under MIT Licence. */
		int8_t offset = (int8_t) (opd);
		gb->cpu_reg.hl = gb->cpu_reg.sp + offset;
		gb->cpu_reg.f_bits.z = 0;
		gb->cpu_reg.f_bits.n = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.sp & 0xF) + (offset & 0xF) > 0xF) ? 1 : 0;
		gb->cpu_reg.f_bits.c = ((gb->cpu_reg.sp & 0xFF) + (offset & 0xFF) > 0xFF) ? 1 : 0;
  }
	    break;

  case 0xF9:  /* LD SP, HL */
		gb->cpu_reg.sp = gb->cpu_reg.hl;
    break;

  case 0xFA:  /* LD A, (imm) */
	{
		uint16_t addr = (opd);
		gb->cpu_reg.a = __gb_read(gb, addr);
  }
	    break;

  case 0xFB:  /* EI */
		gb->gb_ime = 1;
    break;

  case 0xFE:  /* CP imm */
	{
		uint8_t temp_8 = (opd);
		uint16_t temp_16 = gb->cpu_reg.a - temp_8;
		gb->cpu_reg.f_bits.z = (temp_16 == 0x00);
		gb->cpu_reg.f_bits.n = 1;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.a ^ temp_8 ^ temp_16) & 0x10) ? 1 : 0;
		gb->cpu_reg.f_bits.c = temp_16 >> 8;
}
	    break;

  case 0xFF:  /* RST 0x0038 */
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
		__gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
		gb->cpu_reg.pc = 0x0038;
    break;
      }
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
#ifdef  ENABLE_GBC
    if (gb->gb_reg.KEY1 & 0x80)
      inst_cycles >>= 1;
#endif
    ccount += inst_cycles;

    /* LCD Timing */
    gb->counter.lcd_count += inst_cycles;
#ifdef  ENABLE_SOUND
    gb->counter.sound_count += inst_cycles;
#endif
  }

  /* Check serial transfer. Probably slow enough to handle outside of the main loop. */
  if(gb->gb_reg.SC & 0x80)
  {
    if (gb->gb_reg.SC & 1) {
      // Internal clock.
      gb->counter.serial_count += ccount;
//printf("S# %02X %02X %04X\n", gb->gb_reg.SC, gb->gb_reg.SB, gb->cpu_reg.pc);
      if(gb->counter.serial_count >= SERIAL_CYCLES)
      {
        gb_serial_transfer(gb);

        // Inform game of serial TX/RX completion.
        gb->gb_reg.SC &= 0x01;
        gb->gb_reg.IF |= SERIAL_INTR;
        gb->counter.serial_count -= SERIAL_CYCLES;
      }
    } else {
      // External clock.
      if(gb->counter.serial_count >= SERIAL_CYCLES)
      {
        if (gb_serial_transfer(gb)) {
          gb->gb_reg.SC &= 0x01;
          gb->gb_reg.IF |= SERIAL_INTR;
          gb->counter.serial_count -= SERIAL_CYCLES;
        }
      } else {
        gb->counter.serial_count += ccount;
      }
    }
  }
}

#ifdef  ENABLE_GBC
void __gb_hdma(struct gb_s *gb) {
  uint16_t src = (gb->gb_reg.HDMA1 << 8) | gb->gb_reg.HDMA2;
  uint16_t dst = 0x8000 | ((gb->gb_reg.HDMA3 & 0x1F) << 8) | gb->gb_reg.HDMA4;
  uint32_t cnt = 16;
  uint8_t *da = gb->addr_wmap[8] + dst;
  while (cnt--)
    *da++ = __gb_read(gb, src++);
  dst += cnt;
  gb->gb_reg.HDMA1 = src >> 8;
  gb->gb_reg.HDMA2 = src & 0xF0;
  gb->gb_reg.HDMA3 = (dst >> 8) & 0x1F;
  gb->gb_reg.HDMA4 = dst & 0xF0;
  gb->gb_reg.HDMA5--;
}
#endif

void __gb_step_line(struct gb_s *gb) {
	/* OAM access */
	if(gb->lcd_mode == LCD_HBLANK
			&& gb->counter.lcd_count < 100) {
    __gb_step_cpu(gb, 100 - gb->counter.lcd_count);

  }
#if ENABLE_LCD
  if (gb->display.line_cnt)
    gb_lcd_send_chunk(gb);
#endif
	if(gb->lcd_mode == LCD_HBLANK
			&& gb->counter.lcd_count < LCD_MODE_2_CYCLES) {
    __gb_step_cpu(gb, LCD_MODE_2_CYCLES - gb->counter.lcd_count);
		gb->lcd_mode = LCD_SEARCH_OAM;

		if(gb->gb_reg.STAT & STAT_MODE_2_INTR)
			gb->gb_reg.IF |= LCDC_INTR;
  }
#if ENABLE_LCD
  if (gb->display.line_cnt)
    gb_lcd_send_chunk(gb);
#endif
	/* Update LCD */
	if (gb->lcd_mode == LCD_SEARCH_OAM
			&& gb->counter.lcd_count < LCD_MODE_3_CYCLES)
	{
    __gb_step_cpu(gb, LCD_MODE_3_CYCLES - gb->counter.lcd_count);
		gb->lcd_mode = LCD_TRANSFER;
#if ENABLE_LCD
  	if (gb->gb_reg.LCDC & LCDC_ENABLE) {
#ifdef  ENABLE_GBC
      if (gb->is_gbc)
        __gb_draw_gbc_line(gb);
      else
#endif
     		__gb_draw_line(gb);
    }
#endif
	}
#ifdef  ENABLE_GBC
  if (gb->gb_reg.HDMA5 != 0xFF)
    __gb_hdma(gb);
#endif

	if (gb->counter.lcd_count < 370) {
    __gb_step_cpu(gb, 370 - gb->counter.lcd_count);
  }
#if ENABLE_LCD
  if (gb->display.line_cnt)
    gb_lcd_send_chunk(gb);
#endif
	/* New Scanline */
	if (gb->counter.lcd_count < LCD_LINE_CYCLES) {
    __gb_step_cpu(gb, LCD_LINE_CYCLES - gb->counter.lcd_count);

    if (gb->counter.lcd_count < LCD_LINE_CYCLES) {
      gb->counter.lcd_count = LCD_LINE_CYCLES;
    }
  }
#if ENABLE_LCD
  if (gb->display.line_cnt)
    gb_lcd_send_chunk(gb);
#endif

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
#ifdef  ENABLE_SOUND
  gb->counter.sound_count = 0;
#endif
#if ENABLE_LCD
    /* If frame skip is activated, check if we need to draw
      * the frame or skip it. */
    gb->display.frame_skip_count++;
    if (gb->display.frame_skip_count > gb->direct.frame_skip) {
      gb->display.frame_skip_count = 0;
      gb_lcd_start_frame(gb);
    }
#endif

	while(!gb->gb_frame) {
    for (int i = 0; i < 16 && !gb->gb_frame; i++)
  		__gb_step_line(gb);
#ifdef  ENABLE_SOUND
    gsys->PollAudioBuffer();
#endif
  }
#if ENABLE_LCD
  while (gb->display.line_cnt)
    gb_lcd_send_chunk(gb);
#endif
#ifdef  ENABLE_SOUND
  audio_mix(gb->counter.sound_count);
#endif
}

/**
 * Gets the size of the save file required for the ROM.
 */
uint_fast32_t ROCODE gb_get_save_size(struct gb_s *gb)
{
	const uint_fast16_t ram_size_location = 0x0149;
	const uint_fast32_t ram_sizes[] =
	{
		0x00, 0x800, 0x2000, 0x8000, 0x20000
	};
	uint8_t ram_size = gb->rom_bank0[ram_size_location];
	return ram_sizes[ram_size];
}

uint8_t ROCODE gb_colour_hash(struct gb_s *gb)
{
#define ROM_TITLE_START_ADDR	0x0134
#define ROM_TITLE_END_ADDR	0x0143

	uint8_t x = 0;

	for(uint16_t i = ROM_TITLE_START_ADDR; i <= ROM_TITLE_END_ADDR; i++)
		x += gb->rom_bank0[i];

	return x;
}

/**
 * Initialise the emulator context. gb_reset() is also called to initialise
 * the CPU.
 */
enum gb_init_error_e ROCODE gb_init(struct gb_s *gb, uint8_t *rombuf)
{
  memcpy(gb->cpu_cycles, op_cycles, sizeof(op_cycles));
  memcpy(gb->cpu_sizes, op_sizes, sizeof(op_sizes));

  gb->rom_bank0 = rombuf;

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

	/* Check valid ROM using checksum value. */
	{
		uint8_t x = 0;

		for(uint16_t i = 0x0134; i <= 0x014C; i++)
			x = x - rombuf[i] - 1;

		if(x != rombuf[ROM_HEADER_CHECKSUM_LOC])
			return GB_INIT_INVALID_CHECKSUM;
#ifdef  ENABLE_GBC
    x = rombuf[0x143];
    if (x & 0x80)
      gb->is_gbc = 1;
#endif
	}

	/* Check if cartridge type is supported, and set MBC type. */
	{
		const uint8_t mbc_value = rombuf[mbc_location];

		if(mbc_value > sizeof(cart_mbc) - 1 ||
				(gb->mbc = cart_mbc[rombuf[mbc_location]]) == 255u)
			return GB_INIT_CARTRIDGE_UNSUPPORTED;
	}

	gb->cart_ram = cart_ram[rombuf[mbc_location]];
	gb->num_rom_banks = num_rom_banks[rombuf[bank_count_location]];
	gb->num_ram_banks = num_ram_banks[rombuf[ram_size_location]];

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
const char* ROCODE gb_get_rom_name(struct gb_s* gb, char* title_str)
{
	uint_least16_t title_loc = 0x134;
	/* End of title may be 0x13E for newer games. */
	const uint_least16_t title_end = 0x143;
	const char* title_start = title_str;

	for(; title_loc <= title_end; title_loc++)
	{
		const char title_char = gb->rom_bank0[title_loc];

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
