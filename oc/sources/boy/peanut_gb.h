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
/* Optimization for faster memory access, real size = 0xA0 */
#define OAM_SIZE	0x0100

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
#define LCD_MODE_0_CYCLES   204
#define LCD_MODE_2_CYCLES   80
#define LCD_MODE_3_CYCLES   172
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

#define ADD8(X, Y) \
	{ \
		uint16_t temp = X + Y;  \
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00); \
		gb->cpu_reg.f_bits.n = 0; \
		gb->cpu_reg.f_bits.h = (X ^ Y ^ temp) & 0x10 ? 1 : 0;  \
		gb->cpu_reg.f_bits.c = temp >> 8; \
		X = temp; \
}
#define ADC8(X, Y) \
	{ \
		uint16_t temp = X + Y + gb->cpu_reg.f_bits.c;  \
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00); \
		gb->cpu_reg.f_bits.n = 0; \
		gb->cpu_reg.f_bits.h = (X ^ Y ^ temp) & 0x10 ? 1 : 0;  \
		gb->cpu_reg.f_bits.c = temp >> 8; \
		X = temp; \
}
#define SUB8(X, Y) \
	{ \
		uint16_t temp = X - Y;  \
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00); \
		gb->cpu_reg.f_bits.n = 1; \
		gb->cpu_reg.f_bits.h = (X ^ Y ^ temp) & 0x10 ? 1 : 0;  \
		gb->cpu_reg.f_bits.c = temp >> 8; \
		X = temp; \
}
#define SBC8(X, Y) \
	{ \
		uint16_t temp = X - Y - gb->cpu_reg.f_bits.c;  \
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00); \
		gb->cpu_reg.f_bits.n = 1; \
		gb->cpu_reg.f_bits.h = (X ^ Y ^ temp) & 0x10 ? 1 : 0;  \
		gb->cpu_reg.f_bits.c = temp >> 8; \
		X = temp; \
}
#define CP8(X, Y) \
	{ \
		uint16_t temp = X - Y;  \
		gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00); \
		gb->cpu_reg.f_bits.n = 1; \
		gb->cpu_reg.f_bits.h = (X ^ Y ^ temp) & 0x10 ? 1 : 0;  \
		gb->cpu_reg.f_bits.c = temp >> 8; \
}

#define CB_REG_CASES(r, n) \
case 0x00|(n): RL(r); break; \
case 0x08|(n): RR(r); break; \
case 0x10|(n): RLC(r); break; \
case 0x18|(n): RRC(r); break; \
case 0x20|(n): SLA(r); break; \
case 0x28|(n): SRA(r); break; \
case 0x30|(n): SWAP(r); break; \
case 0x38|(n): SRL(r); break; \
case 0x40|(n): BIT(0, r); break; \
case 0x48|(n): BIT(1, r); break; \
case 0x50|(n): BIT(2, r); break; \
case 0x58|(n): BIT(3, r); break; \
case 0x60|(n): BIT(4, r); break; \
case 0x68|(n): BIT(5, r); break; \
case 0x70|(n): BIT(6, r); break; \
case 0x78|(n): BIT(7, r); break; \
case 0x80|(n): RES(0, r); break; \
case 0x88|(n): RES(1, r); break; \
case 0x90|(n): RES(2, r); break; \
case 0x98|(n): RES(3, r); break; \
case 0xA0|(n): RES(4, r); break; \
case 0xA8|(n): RES(5, r); break; \
case 0xB0|(n): RES(6, r); break; \
case 0xB8|(n): RES(7, r); break; \
case 0xC0|(n): SET(0, r); break; \
case 0xC8|(n): SET(1, r); break; \
case 0xD0|(n): SET(2, r); break; \
case 0xD8|(n): SET(3, r); break; \
case 0xE0|(n): SET(4, r); break; \
case 0xE8|(n): SET(5, r); break; \
case 0xF0|(n): SET(6, r); break; \
case 0xF8|(n): SET(7, r); break;

#define SWAP(r) { \
(r) = (r >> 4) | (r << 4); \
gb->cpu_reg.f = (r == 0) << 7; }

#define BIT(nn,r) { \
		gb->cpu_reg.f = (((r << (7 - nn)) ^ 0x80) & 0x80) | 0x20;   }
#define RES(n,r) { (r) &= ~(1 << (n)); }
#define SET(n,r) { (r) |= (1 << (n)); }

#define RLC(r) { \
				uint8_t temp = r;   \
				r = (r << 1) | gb->cpu_reg.f_bits.c;    \
				gb->cpu_reg.f = ((r == 0x00) << 7) | ((temp & 0x80) >> 3);  }
#define RRC(r) { \
				uint8_t temp = r;   \
				r = (r >> 1) | (gb->cpu_reg.f_bits.c << 7);   \
				gb->cpu_reg.f = ((r == 0x00) << 7) | ((temp & 0x01) << 4);  }

#define RL(r) { \
				uint8_t temp = r;   \
				r = (r << 1) | (r >> 7);   \
				gb->cpu_reg.f = ((r == 0x00) << 7) | ((temp & 0x80) >> 3);  }
#define RR(r) { \
				uint8_t temp = r;   \
				r = (r >> 1) | (r << 7);    \
				gb->cpu_reg.f = ((r == 0x00) << 7) | ((temp & 0x01) << 4);  }

#define SLA(r) { \
				uint32_t cc = (r & 0x80) >> 3;  \
				r = r << 1;   \
				gb->cpu_reg.f = ((r == 0x00) << 7) | cc;  }

#define SRA(r) { \
				uint32_t cc = (r & 0x01) << 4;  \
				r = (r >> 1) | (r & 0x80);  \
				gb->cpu_reg.f = ((r == 0x00) << 7) | cc;  }

#define SRL(r) { \
				uint32_t cc = (r & 0x01) << 4;  \
				r = (r >> 1);  \
				gb->cpu_reg.f = ((r == 0x00) << 7) | cc;  }


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
	int32_t lcd_count;		/* LCD Timing */
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
  #define LCD_GBC_PAL_BG_PRI  0x80
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
    uint32_t  gb_cpu_speed;
		uint32_t	gb_halt;
		uint32_t	gb_ime;
		uint32_t	gb_ints;
    uint32_t  gb_forced_int;
    uint32_t  gb_tac_rate;
    uint32_t  gb_audio_flag;
		uint32_t	gb_bios_enable;
		uint32_t	gb_frame; /* New frame drawn. */
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

	uint8_t wram[WRAM_SIZE] __attribute((aligned(4)));
	uint8_t vram[VRAM_SIZE];
	uint8_t hram[HRAM_SIZE];
	uint8_t oam[OAM_SIZE];

	struct
	{
    uint16_t gb_palette[3][4];
#ifdef  ENABLE_GBC
    uint8_t bg_palette[4 * 8 * 2];
    uint8_t sp_palette[4 * 8 * 2];
    uint8_t flipmap[256];
#endif

		uint32_t window_clear;
		uint32_t WY;

		uint32_t frame_skip_count;
    uint8_t *framebuffer;
    uint32_t chunk_cnt;
    uint32_t line_cnt;
    uint32_t line_num;
	} display;

	struct
	{
		uint32_t frame_skip;

		union
		{
			struct
			{
				uint32_t a		: 1;
				uint32_t b		: 1;
				uint32_t select	: 1;
				uint32_t start	: 1;
				uint32_t right	: 1;
				uint32_t left	: 1;
				uint32_t up		: 1;
				uint32_t down	: 1;
			} joypad_bits;
			uint32_t joypad;
		};
	} direct;

  uint32_t cpu_cycles[256];
  uint8_t inc_flag[256];
  uint8_t dec_flag[256];
  uint32_t tac_cycles[4];
	uint32_t condbits[4];

  uint8_t *addr_map[16];
  uint8_t (*addr_read[16])(struct gb_s *gb, const uint_fast16_t addr);
  uint8_t *addr_wmap[16];
  void (*addr_write[16])(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val);
  void (*write_regs[256])(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val);

  uint32_t pixmap[256];

  uint32_t overlay;
  uint32_t menuidx;
  uint32_t fpsmode;
  uint8_t font[104 * 8];
  uint8_t charmap[20 * 16];
  uint8_t colmap[20 * 16];

  char savefile[160];
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
  /* HRAM */
  if(HRAM_ADDR <= addr && addr < INTR_EN_ADDR)
    return gb->hram[addr - HRAM_ADDR];

  if((addr >= 0xFF10) && (addr <= 0xFF3F))
  {
#ifdef ENABLE_SOUND
    uint32_t b = audio_read(addr, gb->counter.sound_count);
    gb->gb_audio_flag = 1;
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

  case 0x06:
    return gb->gb_reg.TMA;

  case 0x07:
    return gb->gb_reg.TAC;

  /* Interrupt Flag Register */
  case 0x0F:
    return gb->gb_reg.IF;

  /* LCD Registers */
  case 0x40:
    return gb->gb_reg.LCDC;

  case 0x41:
    return gb->gb_reg.STAT;

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

uint8_t __memf_read(struct gb_s *gb, const uint_fast16_t addr) {
  if(addr < OAM_ADDR)
    return gb->wram[addr - ECHO_ADDR];

  if(addr < IO_ADDR)
    return gb->oam[addr - OAM_ADDR];

  return __io_read(gb, addr);
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
  if (baddr < 8) {
    uint32_t da = addr + gb->rom_bank_addr;
    return (*(uint32_t *)(da & 0xFFFFFFFC)) >> ((da & 3) * 8);
  }
  if(addr >= IO_ADDR)
    return __io_read(gb, addr);

  if(addr >= OAM_ADDR)
    return gb->oam[addr - OAM_ADDR];

  if (baddr < 12) {
    return gb_cart_ram_read(gb, addr);
  }
  return gb->wram[addr - ECHO_ADDR];
}

static inline uint32_t __gb_read16(struct gb_s *gb, const uint_fast16_t addr)
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

static inline void __hw_interrupt(struct gb_s *gb, uint8_t i, uint8_t mask) {
  uint8_t oldif = gb->gb_reg.IF;
  i &= 0x1F & mask;
  gb->gb_reg.IF |= i & (gb->gb_ints ^ i);

  if ((gb->gb_reg.IF & (gb->gb_reg.IF ^ oldif) & gb->gb_reg.IE) && gb->gb_ime)
    gb->gb_halt = 0;

  gb->gb_ints &= ~mask;
  gb->gb_ints |= i;
}

static inline void __stat_trigger(struct gb_s *gb) {
	int flag = 0;

	if ((gb->gb_reg.LY < 0x91) && (gb->gb_reg.LY == gb->gb_reg.LYC))
	{
		gb->gb_reg.STAT |= 0x04;
		if (gb->gb_reg.STAT & 0x40) flag = LCDC_INTR;
	}
	else gb->gb_reg.STAT &= ~0x04;

	if (gb->gb_reg.STAT & gb->condbits[gb->gb_reg.STAT&3]) flag = LCDC_INTR;

	if (!(gb->gb_reg.LCDC & 0x80)) flag = 0;
	
	__hw_interrupt(gb, flag, LCDC_INTR);
}

static inline void __stat_change(struct gb_s *gb, int stat) {
	stat &= 3;
	gb->gb_reg.STAT = (gb->gb_reg.STAT & 0x7C) | stat;

	if (stat != 1) __hw_interrupt(gb, 0, VBLANK_INTR);
	__stat_trigger(gb);
}

  /* Joypad */
void __io_reg00(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
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
}

  /* Serial */
void __io_reg01(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.SB = val;
}

void __io_reg02(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.SC = val;
  gb_serial_ready(gb);
}

  /* Timer Registers */
void __io_reg04(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.DIV = 0x00;
}

void __io_reg05(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.TIMA = val;
}

void __io_reg06(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.TMA = val;
}

void __io_reg07(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.TAC = val;
  gb->gb_tac_rate = gb->tac_cycles[val & 3];
}

  /* Interrupt Flag Register */
void __io_reg0F(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.IF = val & 0x1F;
  gb->gb_forced_int = gb->counter.lcd_count;
  gb->counter.lcd_count = 0;
}

  /* LCD Registers */
void __io_reg40(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  uint8_t old = gb->gb_reg.LCDC;
  gb->gb_reg.LCDC = val;
  if ((gb->gb_reg.LCDC ^ old) & LCDC_ENABLE) {
    gb->gb_reg.LY = 0;
    gb->gb_reg.STAT = (gb->gb_reg.STAT & 0xF8) | 2;
    gb->counter.lcd_count += 80;
    __stat_change(gb, 2);
  }
}

void __io_reg41(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.STAT = (gb->gb_reg.STAT & 0x07) | (val & 0x78);
  if (!gb->is_gbc && !(gb->gb_reg.STAT & 2))
    __hw_interrupt(gb, LCDC_INTR, LCDC_INTR);
  __stat_trigger(gb);
}

void __io_reg42(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.SCY = val;
}

void __io_reg43(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.SCX = val;
}

  /* LY (0xFF44) is read only. */
void __io_reg45(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.LYC = val;
}

  /* DMA Register */
void __io_reg46(struct gb_s *gb, const uint_fast16_t ad, const uint8_t val) {
  gb->gb_reg.DMA = val;
  uint16_t addr = val << 8;
  uint8_t *src = gb->addr_map[addr>>12];
  if (src != NULL) {
    uint32_t *sd = (uint32_t *)(src + addr);
    uint32_t *dd = (uint32_t *)gb->oam;
    for(uint32_t i = 0; i < OAM_SIZE >> 2; i++)
      dd[i] = sd[i];
  } else {
    for(uint32_t i = 0; i < OAM_SIZE; i++)
      gb->oam[i] = __gb_read(gb, addr + i);
  }
}

  /* DMG Palette Registers */
void __io_reg47(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.BGP = val;
  gb_set_palette(gb->display.gb_palette[2][(gb->gb_reg.BGP & 0x03)], 0 | LCD_PALETTE_BG);
  gb_set_palette(gb->display.gb_palette[2][((gb->gb_reg.BGP >> 2) & 0x03)], 1 | LCD_PALETTE_BG);
  gb_set_palette(gb->display.gb_palette[2][((gb->gb_reg.BGP >> 4) & 0x03)], 2 | LCD_PALETTE_BG);
  gb_set_palette(gb->display.gb_palette[2][((gb->gb_reg.BGP >> 6) & 0x03)], 3 | LCD_PALETTE_BG);
}

void __io_reg48(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.OBP0 = val;
  gb_set_palette(gb->display.gb_palette[0][(gb->gb_reg.OBP0 & 0x03)], 0 | LCD_PALETTE_ALL);
  gb_set_palette(gb->display.gb_palette[0][((gb->gb_reg.OBP0 >> 2) & 0x03)], 1 | LCD_PALETTE_ALL);
  gb_set_palette(gb->display.gb_palette[0][((gb->gb_reg.OBP0 >> 4) & 0x03)], 2 | LCD_PALETTE_ALL);
  gb_set_palette(gb->display.gb_palette[0][((gb->gb_reg.OBP0 >> 6) & 0x03)], 3 | LCD_PALETTE_ALL);
}

void __io_reg49(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.OBP1 = val;
  gb_set_palette(gb->display.gb_palette[1][(gb->gb_reg.OBP1 & 0x03)], 0 | LCD_PALETTE_OBJ | LCD_PALETTE_ALL);
  gb_set_palette(gb->display.gb_palette[1][((gb->gb_reg.OBP1 >> 2) & 0x03)], 1 | LCD_PALETTE_OBJ | LCD_PALETTE_ALL);
  gb_set_palette(gb->display.gb_palette[1][((gb->gb_reg.OBP1 >> 4) & 0x03)], 2 | LCD_PALETTE_OBJ | LCD_PALETTE_ALL);
  gb_set_palette(gb->display.gb_palette[1][((gb->gb_reg.OBP1 >> 6) & 0x03)], 3 | LCD_PALETTE_OBJ | LCD_PALETTE_ALL);
}

  /* Window Position Registers */
void __io_reg4A(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.WY = val;
}

void __io_reg4B(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.WX = val;
}

  /* Turn off boot ROM */
void __io_reg50(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_bios_enable = 0;
}

#ifdef  ENABLE_GBC
void __io_reg4D(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.KEY1 = (gb->gb_reg.KEY1 & 0xFE) | (val & 1);
}

void __io_reg4F(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.VBK = val & 1;
  uint8_t *vra = gb->vram + gb->gb_reg.VBK * VRAM_BANK_SIZE - VRAM_ADDR;
  gb->addr_map[8] = vra;
  gb->addr_map[9] = vra;
  gb->addr_wmap[8] = vra;
  gb->addr_wmap[9] = vra;
}

void __io_reg51(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.HDMA1 = val;
}

void __io_reg52(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.HDMA2 = val & 0xF0;
}

void __io_reg53(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.HDMA3 = val;
}

void __io_reg54(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.HDMA4 = val & 0xF0;
}

void __io_reg55(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.HDMA5 = val;
  if (val & 0x80) {
    gb->gb_reg.HDMA5 = val & 0x7F;
    return;
  }
  uint16_t src = (gb->gb_reg.HDMA1 << 8) | gb->gb_reg.HDMA2;
  uint16_t dst = 0x8000 | ((gb->gb_reg.HDMA3 & 0x1F) << 8) | gb->gb_reg.HDMA4;
  uint32_t cnt = (val + 1) << 4;
  uint8_t *da = gb->addr_wmap[8] + dst;
  dst += cnt;
  while (cnt--)
    *da++ = __gb_read(gb, src++);
  gb->gb_reg.HDMA1 = src >> 8;
  gb->gb_reg.HDMA2 = src & 0xF0;
  gb->gb_reg.HDMA3 = (dst >> 8) & 0x1F;
  gb->gb_reg.HDMA4 = dst & 0xF0;
  gb->gb_reg.HDMA5 = 0xFF;
}
  
void __io_reg68(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.BGPI = val;
}

void __io_reg69(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.BGPD = val;
  gb->display.bg_palette[gb->gb_reg.BGPI & 0x3F] = val;
  uint32_t idx = (gb->gb_reg.BGPI & 0x3F) >> 1;
  uint16_t c = gb->display.bg_palette[idx * 2] | (gb->display.bg_palette[idx * 2 + 1] << 8);
  gb_set_palette(c, idx | LCD_GBC_PAL_BG);
  gb_set_palette(c, idx | LCD_GBC_PAL_BG | LCD_GBC_PAL_BG_PRI);
  if (gb->gb_reg.BGPI & 0x80)
    gb->gb_reg.BGPI = (gb->gb_reg.BGPI & 0x80) | ((gb->gb_reg.BGPI + 1) & 0x3F); 
}

void __io_reg6A(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.OBPI = val;
}

void __io_reg6B(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.OBPD = val;
  gb->display.sp_palette[gb->gb_reg.OBPI & 0x3F] = val;
  uint32_t idx = (gb->gb_reg.OBPI & 0x3F) >> 1;
  gb_set_palette(gb->display.sp_palette[idx * 2] | (gb->display.sp_palette[idx * 2 + 1] << 8), idx | LCD_GBC_PAL_OBJ);
  if (gb->gb_reg.OBPI & 0x80)
    gb->gb_reg.OBPI = (gb->gb_reg.OBPI & 0x80) | ((gb->gb_reg.OBPI + 1) & 0x3F); 
}

void __io_reg56(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.RP = val & 0xC3;
}

void __io_reg70(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.SVBK = val & 7;
  uint32_t v = gb->gb_reg.SVBK == 0 ? 1 : gb->gb_reg.SVBK;
  gb->addr_map[13] = gb->wram - WRAM_1_ADDR + WRAM_BANK_SIZE * v;
  gb->addr_wmap[13] = gb->wram - WRAM_1_ADDR + WRAM_BANK_SIZE * v;
}

void __io_reg6C(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.REG6C = (val & 1) | 0xFE;
}

void __io_reg72(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.REG72 = val;
}

void __io_reg73(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.REG73 = val;
}

void __io_reg74(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.REG74 = val;
}

void __io_reg75(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.REG75 = val & 0x70;
}
#endif

  /* Interrupt Enable Register */
void __io_regFF(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->gb_reg.IE = val & 0x1F;
}

void __io_regNC(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
}

void __io_regHR(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->hram[addr - HRAM_ADDR] = val;
}

void __io_regAU(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  audio_write(addr, val, gb->counter.sound_count);
  gb->gb_audio_flag = 1;
}

void (* const __write_regs[256])(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) RODATA = {
  __io_reg00, __io_reg01, __io_reg02, __io_regNC, __io_reg04, __io_reg05, __io_reg06, __io_reg07,
  __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_reg0F,
  __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU,
  __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU,
  __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU,
  __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU,
  __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU,
  __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU, __io_regAU,
  __io_reg40, __io_reg41, __io_reg42, __io_reg43, __io_regNC, __io_reg45, __io_reg46, __io_reg47,
  __io_reg48, __io_reg49, __io_reg4A, __io_reg4B, __io_regNC, __io_reg4D, __io_regNC, __io_reg4F,
  __io_reg50, __io_reg51, __io_reg52, __io_reg53, __io_reg54, __io_reg55, __io_reg56, __io_regNC,
  __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_regNC,
  __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_regNC,
  __io_reg68, __io_reg69, __io_reg6A, __io_reg6B, __io_reg6C, __io_regNC, __io_regNC, __io_regNC,
  __io_reg70, __io_regNC, __io_reg72, __io_reg73, __io_reg74, __io_reg75, __io_regNC, __io_regNC,
  __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_regNC, __io_regNC,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR,
  __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regHR, __io_regFF
};

static inline void __io_write(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  gb->write_regs[addr & 0xFF](gb, addr, val);
}

void __memf_write(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) {
  if(addr >= IO_ADDR) {
    __io_write(gb, addr, val);
    return;
  }
  if(addr >= OAM_ADDR) {
    gb->oam[addr - OAM_ADDR] = val;
    return;
  }
  gb->wram[addr - ECHO_ADDR] = val;
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
  if (baddr == 15) {
    if(addr >= IO_ADDR) {
      __io_write(gb, addr, val);
      return;
    }
    if(addr >= OAM_ADDR) {
      gb->oam[addr - OAM_ADDR] = val;
      return;
    }
    gb->wram[addr - ECHO_ADDR] = val;
    return;
  }
  void (*fn)(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) = gb->addr_write[baddr];
  fn(gb, addr, val);
}

static inline void __gb_write16(struct gb_s *gb, const uint_fast16_t addr, const uint16_t val)
{
  uint32_t baddr = addr >> 12;
  uint8_t *paddr = gb->addr_map[baddr];

  if (paddr != NULL)
    switch (addr & 3) {
      case 0:
        *(uint32_t *)&paddr[addr] = (*(uint32_t *)&paddr[addr] & 0xFFFF0000) | val;
        return;
      case 1:
        *(uint32_t *)&paddr[addr - 1] = (*(uint32_t *)&paddr[addr - 1] & 0xFF0000FF) | (val << 8);
        return;
      case 2:
        *(uint32_t *)&paddr[addr - 2] = (*(uint32_t *)&paddr[addr - 2] & 0x0000FFFF) | (val << 16);
        return;
      case 3:
        *(uint32_t *)&paddr[addr - 3] = (*(uint32_t *)&paddr[addr - 3] & 0x00FFFFFF) | (val << 24);
        *(uint32_t *)&paddr[addr + 1] = (*(uint32_t *)&paddr[addr + 1] & 0xFFFFFF00) | (val >> 8);
        return;
    }

  void (*fn)(struct gb_s *gb, const uint_fast16_t addr, const uint8_t val) = gb->addr_write[baddr];
  fn(gb, addr, val);
  fn(gb, addr + 1, val >> 8);
}

void __update_addr_map(struct gb_s *gb) {
  if(gb->mbc == 1 && gb->cart_mode_select)
    gb->rom_bank_addr = ((gb->selected_rom_bank & 0x1F) - 1) * ROM_BANK_SIZE + (uint32_t)FLASH_ADDR(GAME_ROM_ADDR);
  else
    gb->rom_bank_addr = (gb->selected_rom_bank - 1) * ROM_BANK_SIZE + (uint32_t)FLASH_ADDR(GAME_ROM_ADDR);
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

  memset(gb->addr_wmap, 0, sizeof(gb->addr_wmap));
  gb->addr_wmap[8] = gb->vram - VRAM_ADDR;
  gb->addr_wmap[9] = gb->vram - VRAM_ADDR;
  gb->addr_wmap[12] = gb->wram - WRAM_0_ADDR;
  gb->addr_wmap[13] = gb->wram - WRAM_1_ADDR + WRAM_BANK_SIZE;
  gb->addr_wmap[14] = gb->wram - ECHO_ADDR;

  memset(gb->addr_read, 0, sizeof(gb->addr_read));
  gb->addr_read[4] = gb->addr_read[5] = gb->addr_read[6] = gb->addr_read[7] = gb_rom_bank_read;
  gb->addr_read[10] = gb->addr_read[11] = __cart_ram_read;
  gb->addr_read[15] = __memf_read;

  memset(gb->addr_write, 0, sizeof(gb->addr_write));
  gb->addr_write[0] = gb->addr_write[1] = __mbc_write01;
  gb->addr_write[2] = __mbc_write2;
  gb->addr_write[3] = __mbc_write3;
  gb->addr_write[4] = gb->addr_write[5] = __mbc_write45;
  gb->addr_write[6] = gb->addr_write[7] = __mbc_write67;
  gb->addr_write[10] = gb->addr_write[11] = __cart_ram_write;
  gb->addr_write[15] = __memf_write;

  __update_addr_map(gb);
}

void __restore_addr_map(struct gb_s *gb) {
  memcpy(gb->write_regs, __write_regs, sizeof(gb->write_regs));

  __init_addr_map(gb);

  uint8_t *vra = gb->vram + gb->gb_reg.VBK * VRAM_BANK_SIZE - VRAM_ADDR;
  gb->addr_map[8] = vra;
  gb->addr_map[9] = vra;
  gb->addr_wmap[8] = vra;
  gb->addr_wmap[9] = vra;

  uint32_t v = gb->gb_reg.SVBK == 0 ? 1 : gb->gb_reg.SVBK;
  gb->addr_map[13] = gb->wram - WRAM_1_ADDR + WRAM_BANK_SIZE * v;
  gb->addr_wmap[13] = gb->wram - WRAM_1_ADDR + WRAM_BANK_SIZE * v;
}


/**
 * Resets the context, and initialises startup values.
 */
void ROCODE gb_reset(struct gb_s *gb)
{
	gb->gb_halt = 0;
	gb->gb_ime = 1;
  gb->gb_ints = 0;
  gb->gb_forced_int = 0;
	gb->gb_bios_enable = 0;

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
  gb->gb_tac_rate = gb->tac_cycles[0];
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

  gb->gb_cpu_speed = 0;
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
  audio_reset(1);
#endif
#if ENABLE_LCD
	gb->direct.frame_skip = 0;
	gb->display.frame_skip_count = 0;

	gb->display.window_clear = 0;
	gb->display.WY = 0;
#endif
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
  //if (gb->gb_reg.LY < 8 || gb->gb_reg.LY > 127 + 8)
  //  return;
  /*uint32_t ly = gb->gb_reg.LY;
  uint32_t mod9 = (ly >> 6) + (ly & 7) - ((ly >> 3) & 7);
  if (mod9 == 0)
    return;*/

	if(gb->display.frame_skip_count != 0)
		return;

  uint32_t by = gb->gb_reg.LY & 7;
  if (!by && gb->gb_reg.LY > 6 && gb->gb_reg.LY < 132)
    return;

	uint8_t *pixels = gb->display.framebuffer + 512;

	/* If background is enabled, draw it. */
	if(gb->gb_reg.LCDC & LCDC_BG_ENABLE)
	{
		const uint8_t bg_y = gb->gb_reg.LY + gb->gb_reg.SCY;

		const uint16_t bg_map =
			((gb->gb_reg.LCDC & LCDC_BG_MAP) ?
			 VRAM_BMAP_2 : VRAM_BMAP_1)
			+ (bg_y >> 3) * 0x20;

		uint32_t bg_x = LCD_WIDTH - 1 + gb->gb_reg.SCX;
    uint32_t mapidx = (bg_x >> 3) & 0x1F;
		uint8_t idx = gb->vram[bg_map + mapidx];
    uint8_t attr = gb->vram[bg_map + mapidx + VRAM_BANK_SIZE];

		const uint8_t pyc = (bg_y & 0x07);
		uint8_t px = 7 - (bg_x & 0x07);

		uint32_t tile;

		/* Select addressing mode. */
		if(gb->gb_reg.LCDC & LCDC_TILE_SELECT)
			tile = VRAM_TILES_1 + idx * 0x10 + ((attr & 0x08) << 10);
		else
			tile = VRAM_TILES_2 + ((idx + 0x80) % 0x100) * 0x10 + ((attr & 0x08) << 10);

    uint8_t py = pyc;
    if (attr & 0x40)
      py = 7 - py;
		tile += 2 * py;

		/* fetch first tile */
		uint32_t t1 = gb->vram[tile];
		uint32_t t2 = gb->vram[tile + 1];
    if (attr & 0x20) {
      t1 = gb->display.flipmap[t1];
      t2 = gb->display.flipmap[t2];
    }
		t1 = t1 >> px;
		t2 = (t2 >> px);
    uint32_t bgpal = ((attr & 7) << 2) | LCD_GBC_PAL_BG | (attr & 0x80);
    bgpal |= bgpal << 8;
    bgpal |= bgpal << 16;
    px = 8 - px;
    uint8_t *ptr = pixels + LCD_WIDTH - 1;
    uint32_t dx = 0;
    if (px != 8) {
      t2 <<= 1;
      for (;px != 0; px--) {
        uint32_t c = (t1 & 0x1) | (t2 & 0x2);
        *ptr-- = c | bgpal;
        t1 = t1 >> 1;
        t2 = t2 >> 1;
      }
      mapidx = (mapidx - 1) & 0x1F;
      idx = gb->vram[bg_map + mapidx];
      attr = gb->vram[bg_map + mapidx + VRAM_BANK_SIZE];
      bgpal = ((attr & 7) << 2) | LCD_GBC_PAL_BG | (attr & 0x80);
      bgpal |= bgpal << 8;
      bgpal |= bgpal << 16;

      if(gb->gb_reg.LCDC & LCDC_TILE_SELECT)
        tile = VRAM_TILES_1 + idx * 0x10 + ((attr & 0x08) << 10);
      else
        tile = VRAM_TILES_2 + ((idx + 0x80) % 0x100) * 0x10 + ((attr & 0x08) << 10);

      py = pyc;
      if (attr & 0x40)
        py = 7 - py;
      tile += 2 * py;
      t1 = gb->vram[tile];
      t2 = gb->vram[tile + 1];
      if (attr & 0x20) {
        t1 = gb->display.flipmap[t1];
        t2 = gb->display.flipmap[t2];
      }
      dx++;
    }
		for(; dx < 20; dx++)	{
      uint32_t c1 = gb->pixmap[((t1 & 0x0F) | (t2 << 4)) & 0xFF] | bgpal;
      *ptr-- = c1;
      *ptr-- = c1 >> 8;
      *ptr-- = c1 >> 16;
      *ptr-- = c1 >> 24;
      uint32_t c2 = gb->pixmap[(t1 >> 4) | (t2 & 0xF0)] | bgpal;
      *ptr-- = c2;
      *ptr-- = c2 >> 8;
      *ptr-- = c2 >> 16;
      *ptr-- = c2 >> 24;
      mapidx = (mapidx - 1) & 0x1F;
      idx = gb->vram[bg_map + mapidx];
      attr = gb->vram[bg_map + mapidx + VRAM_BANK_SIZE];
      bgpal = ((attr & 7) << 2) | LCD_GBC_PAL_BG | (attr & 0x80);
      bgpal |= bgpal << 8;
      bgpal |= bgpal << 16;

      if(gb->gb_reg.LCDC & LCDC_TILE_SELECT)
        tile = VRAM_TILES_1 + idx * 0x10 + ((attr & 0x08) << 10);
      else
        tile = VRAM_TILES_2 + ((idx + 0x80) % 0x100) * 0x10 + ((attr & 0x08) << 10);

      py = pyc;
      if (attr & 0x40)
        py = 7 - py;
      tile += 2 * py;
      t1 = gb->vram[tile];
      t2 = gb->vram[tile + 1];
      if (attr & 0x20) {
        t1 = gb->display.flipmap[t1];
        t2 = gb->display.flipmap[t2];
      }
		}
    px = ptr - pixels + 1;
    for (t2 <<= 1;px != 0; px--) {
      uint32_t c = (t1 & 0x1) | (t2 & 0x2);
      *ptr-- = c | bgpal;
      t1 = t1 >> 1;
      t2 = t2 >> 1;
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
		const uint8_t pyc = gb->display.window_clear & 0x07;
		uint8_t px = 7 - (win_x & 0x07);
		uint8_t idx = gb->vram[win_line + (win_x >> 3)];
    uint8_t attr = gb->vram[win_line + (win_x >> 3) + VRAM_BANK_SIZE];

		uint16_t tile;

		if(gb->gb_reg.LCDC & LCDC_TILE_SELECT)
			tile = VRAM_TILES_1 + idx * 0x10 + ((attr & 0x08) << 10);
		else
			tile = VRAM_TILES_2 + ((idx + 0x80) % 0x100) * 0x10 + ((attr & 0x08) << 10);

    uint8_t py = pyc;
    if (attr & 0x40)
      py = 7 - py;
		tile += 2 * py;

		// fetch first tile
		uint32_t t1 = gb->vram[tile];
		uint32_t t2 = gb->vram[tile + 1];
    if (attr & 0x20) {
      t1 = gb->display.flipmap[t1];
      t2 = gb->display.flipmap[t2];
    }
		t1 = t1 >> px;
		t2 = (t2 >> px) << 1;
    uint32_t bgpal = ((attr & 7) << 2) | LCD_GBC_PAL_BG | (attr & 0x80);

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
        bgpal = ((attr & 7) << 2) | LCD_GBC_PAL_BG | (attr & 0x80);

				if(gb->gb_reg.LCDC & LCDC_TILE_SELECT)
					tile = VRAM_TILES_1 + idx * 0x10 + ((attr & 0x08) << 10);
				else
					tile = VRAM_TILES_2 + ((idx + 0x80) % 0x100) * 0x10 + ((attr & 0x08) << 10);

        py = pyc;
        if (attr & 0x40)
          py = 7 - py;
				tile += 2 * py;
				t1 = gb->vram[tile];
				t2 = gb->vram[tile + 1];
        if (attr & 0x20) {
          t1 = gb->display.flipmap[t1];
          t2 = gb->display.flipmap[t2];
        }
        t2 <<= 1;
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
  //int count = 0;
	if(gb->gb_reg.LCDC & LCDC_OBJ_ENABLE)
	{
		for(uint8_t s = NUM_SPRITES - 1;
				s != 0xFF ;//&& count < MAX_SPRITES_LINE;
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
      //count++;

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
			uint32_t t1 = gb->vram[VRAM_TILES_1 + OT * 0x10 + 2 * py + ((OF & 0x08) << 10)];
			uint32_t t2 = gb->vram[VRAM_TILES_1 + OT * 0x10 + 2 * py + 1 + ((OF & 0x08) << 10)];

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
      t2 <<= 1;

      uint8_t obj = ((OF & 7) << 2) | LCD_GBC_PAL_OBJ;

      if (OF & OBJ_PRIORITY) {
        for(uint8_t disp_x = start; disp_x != end; disp_x += dir)
        {
          uint8_t c = (t1 & 0x1) | (t2 & 0x2);
          // check transparency / sprite overlap / background overlap
          uint8_t bc = pixels[disp_x];
          if(c && !(bc & 0x3)) {
            /* Set pixel colour. */
            pixels[disp_x] = c | obj;
          }
          t1 = t1 >> 1;
          t2 = t2 >> 1;
        }
      } else {
        for(uint8_t disp_x = start; disp_x != end; disp_x += dir)
        {
          uint8_t c = (t1 & 0x1) | (t2 & 0x2);
          // check transparency / sprite overlap / background overlap
          uint8_t bc = pixels[disp_x];
          if(c && (!(bc & 0x3) || !(bc & 0x80))) {
            /* Set pixel colour. */
            pixels[disp_x] = c | obj;
          }
          t1 = t1 >> 1;
          t2 = t2 >> 1;
        }
      }
		}
	}
  if (gb->overlay && gb->display.line_num < 128) {
    uint32_t ty = gb->display.line_num >> 3;
    uint32_t cy = gb->display.line_num & 7;

    for (int i = 0; i < 20; i++) {
      uint32_t cc = gb->colmap[ty * 20 + i];
      if (cc == 0)
        continue;
      uint32_t cd = gb->font[(gb->charmap[ty * 20 + i] - 24) * 8 + cy];
      uint8_t *pp = &pixels[i * 8];

      for (int j = 0; j < 8; j++, cd >>= 1) {
        if (cd & 1)
          *pp++ = cc;
        else
          *pp++ = 0xF0;
      }
    }
  }
  gb->display.line_num++;
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

/**
 * Internal function used to step the CPU.
 */
void __gb_step_cpu(struct gb_s *gb, int32_t cycles)
{
	uint8_t opcode;
  uint32_t inst_cycles;

#ifdef  PROFILE
  PROFILE_BEGIN
#endif
  gb->counter.lcd_count += cycles;
#ifdef  ENABLE_SOUND
  uint32_t old_lcd_count = gb->counter.lcd_count;
  gb->gb_audio_flag = 0;
#endif

  while (gb->counter.lcd_count > 0 && gb->gb_halt) {
    /* Handle interrupts */
    if (gb->gb_reg.IF & gb->gb_reg.IE & ANY_INTR)
    {
      gb->gb_halt = 0;
      /* Disable interrupts */
      gb->gb_ime = 0;

      /* Push Program Counter */
      gb->cpu_reg.sp -= 2;
      __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc);
      switch (gb->gb_reg.IE & gb->gb_reg.IF & ANY_INTR) {
        case 0x01: case 0x03: case 0x05: case 0x07:
        case 0x09: case 0x0B: case 0x0D: case 0x0F:
        case 0x11: case 0x13: case 0x15: case 0x17:
        case 0x19: case 0x1B: case 0x1D: case 0x1F:
          gb->cpu_reg.pc = VBLANK_INTR_ADDR;
          gb->gb_reg.IF ^= VBLANK_INTR;
          break;
        case 0x02: case 0x06: case 0x0A: case 0x0E:
        case 0x12: case 0x16: case 0x1A: case 0x1E:
          gb->cpu_reg.pc = LCDC_INTR_ADDR;
          gb->gb_reg.IF ^= LCDC_INTR;
          break;
        case 0x04: case 0x0C: case 0x14: case 0x1C:
          gb->cpu_reg.pc = TIMER_INTR_ADDR;
          gb->gb_reg.IF ^= TIMER_INTR;
          break;
        case 0x08: case 0x18:
          gb->cpu_reg.pc = SERIAL_INTR_ADDR;
          gb->gb_reg.IF ^= SERIAL_INTR;
          break;
        case 0x10:
          gb->cpu_reg.pc = CONTROL_INTR_ADDR;
          gb->gb_reg.IF ^= CONTROL_INTR;
          break;
      }
      break;
    }
    inst_cycles = gb->counter.lcd_count << gb->gb_cpu_speed;
    uint32_t divcc = DIV_CYCLES - gb->counter.div_count;
    if(gb->gb_reg.tac_enable) {
      uint32_t taccc = gb->gb_tac_rate - gb->counter.tima_count;
      if (taccc < inst_cycles)
        inst_cycles = taccc;
    }
    if (divcc < inst_cycles)
      inst_cycles = divcc;
    if (inst_cycles == 0)
      inst_cycles = 4;

__halt_entry:
    /* DIV register timing */
    gb->counter.div_count += inst_cycles;
    gb->gb_reg.DIV = gb->counter.div_count >> 8;

    /* TIMA register timing */
    if(gb->gb_reg.tac_enable)
    {
      gb->counter.tima_count += inst_cycles;

      if(gb->counter.tima_count >= gb->gb_tac_rate)
      {
        gb->counter.tima_count -= gb->gb_tac_rate;

        if(++gb->gb_reg.TIMA == 0)
        {
          __hw_interrupt(gb, TIMER_INTR, TIMER_INTR);
          __hw_interrupt(gb, 0, TIMER_INTR);
          /* On overflow, set TMA to TIMA. */
          gb->gb_reg.TIMA = gb->gb_reg.TMA;
        }
      }
    }
#ifdef  ENABLE_GBC
    inst_cycles >>= gb->gb_cpu_speed;
#endif

    /* LCD Timing */
    gb->counter.lcd_count -= inst_cycles;
  }

__int_entry:
  /* Handle interrupts */
  if (gb->gb_ime && (gb->gb_reg.IF & gb->gb_reg.IE & ANY_INTR))
  {
    /* Disable interrupts */
    gb->gb_ime = 0;

    /* Push Program Counter */
      gb->cpu_reg.sp -= 2;
      __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc);
    switch (gb->gb_reg.IE & gb->gb_reg.IF & ANY_INTR) {
      case 0x01: case 0x03: case 0x05: case 0x07:
      case 0x09: case 0x0B: case 0x0D: case 0x0F:
      case 0x11: case 0x13: case 0x15: case 0x17:
      case 0x19: case 0x1B: case 0x1D: case 0x1F:
        gb->cpu_reg.pc = VBLANK_INTR_ADDR;
        gb->gb_reg.IF ^= VBLANK_INTR;
        break;
      case 0x02: case 0x06: case 0x0A: case 0x0E:
      case 0x12: case 0x16: case 0x1A: case 0x1E:
        gb->cpu_reg.pc = LCDC_INTR_ADDR;
        gb->gb_reg.IF ^= LCDC_INTR;
        break;
      case 0x04: case 0x0C: case 0x14: case 0x1C:
        gb->cpu_reg.pc = TIMER_INTR_ADDR;
        gb->gb_reg.IF ^= TIMER_INTR;
        break;
      case 0x08: case 0x18:
        gb->cpu_reg.pc = SERIAL_INTR_ADDR;
        gb->gb_reg.IF ^= SERIAL_INTR;
        break;
      case 0x10:
        gb->cpu_reg.pc = CONTROL_INTR_ADDR;
        gb->gb_reg.IF ^= CONTROL_INTR;
        break;
    }
  }
  while (gb->counter.lcd_count > 0) {
    /* Obtain opcode */
    opcode = __gb_read(gb, gb->cpu_reg.pc++);
    inst_cycles = gb->cpu_cycles[opcode];
//printf("%04X %02X %02X\n", gb->cpu_reg.pc, opcode, __gb_read(gb, gb->cpu_reg.pc));

    switch (opcode) {
  case 0x00:  /* NOP */
    break;

  case 0x01:  /* LD BC, imm */
		gb->cpu_reg.bc = __gb_read16(gb, gb->cpu_reg.pc);
    gb->cpu_reg.pc += 2;
    break;

  case 0x02:  /* LD (BC), A */
		__gb_write(gb, gb->cpu_reg.bc, gb->cpu_reg.a);
    break;

  case 0x03:  /* INC BC */
		gb->cpu_reg.bc++;
    break;

  case 0x04:  /* INC B */
		gb->cpu_reg.b++;
    gb->cpu_reg.f = gb->inc_flag[gb->cpu_reg.b] | (gb->cpu_reg.f & 0x10);
    break;

  case 0x05:  /* DEC B */
		gb->cpu_reg.b--;
    gb->cpu_reg.f = gb->dec_flag[gb->cpu_reg.b] | (gb->cpu_reg.f & 0x10);
    break;

  case 0x06:  /* LD B, imm */
		gb->cpu_reg.b = __gb_read(gb, gb->cpu_reg.pc++);
    break;

  case 0x07:  /* RLCA */
		gb->cpu_reg.a = (gb->cpu_reg.a << 1) | (gb->cpu_reg.a >> 7);
		gb->cpu_reg.f = (gb->cpu_reg.a & 0x01) << 4;
    break;

  case 0x08:  /* LD (imm), SP */
	{
		uint16_t temp = __gb_read16(gb, gb->cpu_reg.pc);
    gb->cpu_reg.pc += 2;
		__gb_write16(gb, temp, gb->cpu_reg.sp);
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
    gb->cpu_reg.f = gb->inc_flag[gb->cpu_reg.c] | (gb->cpu_reg.f & 0x10);
    break;

  case 0x0D:  /* DEC C */
		gb->cpu_reg.c--;
    gb->cpu_reg.f = gb->dec_flag[gb->cpu_reg.c] | (gb->cpu_reg.f & 0x10);
    break;

  case 0x0E:  /* LD C, imm */
		gb->cpu_reg.c = __gb_read(gb, gb->cpu_reg.pc++);
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
      gb->gb_cpu_speed = (gb->gb_reg.KEY1 & 0x80) != 0;
    }
#endif
    break;

  case 0x11:  /* LD DE, imm */
		gb->cpu_reg.de = __gb_read16(gb, gb->cpu_reg.pc);
    gb->cpu_reg.pc += 2;
    break;

  case 0x12:  /* LD (DE), A */
		__gb_write(gb, gb->cpu_reg.de, gb->cpu_reg.a);
    break;

  case 0x13:  /* INC DE */
		gb->cpu_reg.de++;
    break;

  case 0x14:  /* INC D */
		gb->cpu_reg.d++;
    gb->cpu_reg.f = gb->inc_flag[gb->cpu_reg.d] | (gb->cpu_reg.f & 0x10);
    break;

  case 0x15:  /* DEC D */
		gb->cpu_reg.d--;
    gb->cpu_reg.f = gb->dec_flag[gb->cpu_reg.d] | (gb->cpu_reg.f & 0x10);
    break;

  case 0x16:  /* LD D, imm */
		gb->cpu_reg.d = __gb_read(gb, gb->cpu_reg.pc++);
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
		int8_t temp = (int8_t) __gb_read(gb, gb->cpu_reg.pc++);
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
    gb->cpu_reg.f = gb->inc_flag[gb->cpu_reg.e] | (gb->cpu_reg.f & 0x10);
    break;

  case 0x1D:  /* DEC E */
		gb->cpu_reg.e--;
    gb->cpu_reg.f = gb->dec_flag[gb->cpu_reg.e] | (gb->cpu_reg.f & 0x10);
    break;

  case 0x1E:  /* LD E, imm */
		gb->cpu_reg.e = __gb_read(gb, gb->cpu_reg.pc++);
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
			int8_t temp = (int8_t) __gb_read(gb, gb->cpu_reg.pc++);
			gb->cpu_reg.pc += temp;
			inst_cycles += 4;
		}
		else
			gb->cpu_reg.pc++;
    break;

  case 0x21:  /* LD HL, imm */
		gb->cpu_reg.hl = __gb_read16(gb, gb->cpu_reg.pc);
    gb->cpu_reg.pc += 2;
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
    gb->cpu_reg.f = gb->inc_flag[gb->cpu_reg.h] | (gb->cpu_reg.f & 0x10);
    break;

  case 0x25:  /* DEC H */
		gb->cpu_reg.h--;
    gb->cpu_reg.f = gb->dec_flag[gb->cpu_reg.h] | (gb->cpu_reg.f & 0x10);
    break;

  case 0x26:  /* LD H, imm */
		gb->cpu_reg.h = __gb_read(gb, gb->cpu_reg.pc++);
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
			int8_t temp = (int8_t) __gb_read(gb, gb->cpu_reg.pc++);
			gb->cpu_reg.pc += temp;
			inst_cycles += 4;
		}
		else
			gb->cpu_reg.pc++;
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
    gb->cpu_reg.f = gb->inc_flag[gb->cpu_reg.l] | (gb->cpu_reg.f & 0x10);
    break;

  case 0x2D:  /* DEC L */
		gb->cpu_reg.l--;
    gb->cpu_reg.f = gb->dec_flag[gb->cpu_reg.l] | (gb->cpu_reg.f & 0x10);
    break;

  case 0x2E:  /* LD L, imm */
		gb->cpu_reg.l = __gb_read(gb, gb->cpu_reg.pc++);
    break;

  case 0x2F:  /* CPL */
		gb->cpu_reg.a = ~gb->cpu_reg.a;
		gb->cpu_reg.f  |= 0x60;
    break;

  case 0x30:  /* JP NC, imm */
		if(!gb->cpu_reg.f_bits.c)
		{
			int8_t temp = (int8_t) __gb_read(gb, gb->cpu_reg.pc++);
			gb->cpu_reg.pc += temp;
			inst_cycles += 4;
		}
		else
			gb->cpu_reg.pc++;
    break;

  case 0x31:  /* LD SP, imm */
		gb->cpu_reg.sp = __gb_read16(gb, gb->cpu_reg.pc);
    gb->cpu_reg.pc += 2;
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
    gb->cpu_reg.f = gb->inc_flag[temp] | (gb->cpu_reg.f & 0x10);
		__gb_write(gb, gb->cpu_reg.hl, temp);
}
	    break;

  case 0x35:  /* DEC (HL) */
	{
		uint8_t temp = __gb_read(gb, gb->cpu_reg.hl) - 1;
    gb->cpu_reg.f = gb->dec_flag[temp] | (gb->cpu_reg.f & 0x10);
		__gb_write(gb, gb->cpu_reg.hl, temp);
}
	    break;

  case 0x36:  /* LD (HL), imm */
		__gb_write(gb, gb->cpu_reg.hl, __gb_read(gb, gb->cpu_reg.pc++));
    break;

  case 0x37:  /* SCF */
		gb->cpu_reg.f &= 0x90;
		gb->cpu_reg.f_bits.c = 1;
    break;

  case 0x38:  /* JP C, imm */
		if(gb->cpu_reg.f_bits.c)
		{
			int8_t temp = (int8_t) __gb_read(gb, gb->cpu_reg.pc++);
			gb->cpu_reg.pc += temp;
			inst_cycles += 4;
		}
		else
			gb->cpu_reg.pc++;
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
    gb->cpu_reg.f = gb->inc_flag[gb->cpu_reg.a] | (gb->cpu_reg.f & 0x10);
    break;

  case 0x3D:  /* DEC A */
		gb->cpu_reg.a--;
    gb->cpu_reg.f = gb->dec_flag[gb->cpu_reg.a] | (gb->cpu_reg.f & 0x10);
    break;

  case 0x3E:  /* LD A, imm */
		gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.pc++);
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
    goto __halt_entry;

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
    ADD8(gb->cpu_reg.a, gb->cpu_reg.b)
    break;

  case 0x81:  /* ADD A, C */
    ADD8(gb->cpu_reg.a, gb->cpu_reg.c)
    break;

  case 0x82:  /* ADD A, D */
    ADD8(gb->cpu_reg.a, gb->cpu_reg.d)
    break;

  case 0x83:  /* ADD A, E */
    ADD8(gb->cpu_reg.a, gb->cpu_reg.e)
    break;

  case 0x84:  /* ADD A, H */
    ADD8(gb->cpu_reg.a, gb->cpu_reg.h)
    break;

  case 0x85:  /* ADD A, L */
    ADD8(gb->cpu_reg.a, gb->cpu_reg.l)
    break;

  case 0x86:  /* ADD A, (HL) */
    {
		uint8_t hl = __gb_read(gb, gb->cpu_reg.hl);
    ADD8(gb->cpu_reg.a, hl)
    }
    break;

  case 0x87:  /* ADD A, A */
    ADD8(gb->cpu_reg.a, gb->cpu_reg.a)
    break;

  case 0x88:  /* ADC A, B */
    ADC8(gb->cpu_reg.a, gb->cpu_reg.b)
    break;

  case 0x89:  /* ADC A, C */
    ADC8(gb->cpu_reg.a, gb->cpu_reg.c)
    break;

  case 0x8A:  /* ADC A, D */
    ADC8(gb->cpu_reg.a, gb->cpu_reg.d)
    break;

  case 0x8B:  /* ADC A, E */
    ADC8(gb->cpu_reg.a, gb->cpu_reg.e)
    break;

  case 0x8C:  /* ADC A, H */
    ADC8(gb->cpu_reg.a, gb->cpu_reg.h)
    break;

  case 0x8D:  /* ADC A, L */
    ADC8(gb->cpu_reg.a, gb->cpu_reg.l)
    break;

  case 0x8E:  /* ADC A, (HL) */
    {
		uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
    ADC8(gb->cpu_reg.a, val)
    }
    break;

  case 0x8F:  /* ADC A, A */
    ADC8(gb->cpu_reg.a, gb->cpu_reg.a)
    break;

  case 0x90:  /* SUB B */
    SUB8(gb->cpu_reg.a, gb->cpu_reg.b)
    break;

  case 0x91:  /* SUB C */
    SUB8(gb->cpu_reg.a, gb->cpu_reg.c)
    break;

  case 0x92:  /* SUB D */
    SUB8(gb->cpu_reg.a, gb->cpu_reg.d)
    break;

  case 0x93:  /* SUB E */
    SUB8(gb->cpu_reg.a, gb->cpu_reg.e)
    break;

  case 0x94:  /* SUB H */
    SUB8(gb->cpu_reg.a, gb->cpu_reg.h)
    break;

  case 0x95:  /* SUB L */
    SUB8(gb->cpu_reg.a, gb->cpu_reg.l)
    break;

  case 0x96:  /* SUB (HL) */
    {
		uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
    SUB8(gb->cpu_reg.a, val)
    }
    break;

  case 0x97:  /* SUB A */
		gb->cpu_reg.a = 0;
		gb->cpu_reg.f = 0xC0;
    break;

  case 0x98:  /* SBC A, B */
    SBC8(gb->cpu_reg.a, gb->cpu_reg.b)
    break;

  case 0x99:  /* SBC A, C */
    SBC8(gb->cpu_reg.a, gb->cpu_reg.c)
    break;

  case 0x9A:  /* SBC A, D */
    SBC8(gb->cpu_reg.a, gb->cpu_reg.d)
    break;

  case 0x9B:  /* SBC A, E */
    SBC8(gb->cpu_reg.a, gb->cpu_reg.e)
    break;

  case 0x9C:  /* SBC A, H */
    SBC8(gb->cpu_reg.a, gb->cpu_reg.h)
    break;

  case 0x9D:  /* SBC A, L */
    SBC8(gb->cpu_reg.a, gb->cpu_reg.l)
    break;

  case 0x9E:  /* SBC A, (HL) */
    {
		uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
    SBC8(gb->cpu_reg.a, val)
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
		CP8(gb->cpu_reg.a, gb->cpu_reg.b)
    break;

  case 0xB9:  /* CP C */
		CP8(gb->cpu_reg.a, gb->cpu_reg.c)
    break;

  case 0xBA:  /* CP D */
		CP8(gb->cpu_reg.a, gb->cpu_reg.d)
    break;

  case 0xBB:  /* CP E */
		CP8(gb->cpu_reg.a, gb->cpu_reg.e)
    break;

  case 0xBC:  /* CP H */
		CP8(gb->cpu_reg.a, gb->cpu_reg.h)
    break;

  case 0xBD:  /* CP L */
		CP8(gb->cpu_reg.a, gb->cpu_reg.l)
    break;

  case 0xBE:  /* CP (HL) */
    {
		uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
		CP8(gb->cpu_reg.a, val)
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
			uint16_t temp = __gb_read16(gb, gb->cpu_reg.pc);
			gb->cpu_reg.pc = temp;
			inst_cycles += 4;
		}
		else
			gb->cpu_reg.pc += 2;
    break;

  case 0xC3:  /* JP imm */
	{
		uint16_t temp = __gb_read16(gb, gb->cpu_reg.pc);
		gb->cpu_reg.pc = temp;
}
	    break;

  case 0xC4:  /* CALL NZ imm */
		if(!gb->cpu_reg.f_bits.z)
		{
			uint16_t temp = __gb_read16(gb, gb->cpu_reg.pc);
      gb->cpu_reg.sp -= 2;
      __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc + 2);
			gb->cpu_reg.pc = temp;
			inst_cycles += 12;
		}
		else
			gb->cpu_reg.pc += 2;
    break;

  case 0xC5:  /* PUSH BC */
    gb->cpu_reg.sp -= 2;
    __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.bc);
    break;

  case 0xC6:  /* ADD A, imm */
    {
		uint8_t value = __gb_read(gb, gb->cpu_reg.pc++);
    ADD8(gb->cpu_reg.a, value)
    }
    break;

  case 0xC7:  /* RST 0x0000 */
    gb->cpu_reg.sp -= 2;
    __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc);
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
			uint16_t temp = __gb_read16(gb, gb->cpu_reg.pc);
			gb->cpu_reg.pc = temp;
			inst_cycles += 4;
		}
		else
			gb->cpu_reg.pc += 2;
    break;

  case 0xCB:  /* CB INST */
    {
    uint8_t val, cbop = __gb_read(gb, gb->cpu_reg.pc++);
    /* Add an additional 8 cycles to these sets of instructions. */
    if ((cbop & 0x07) == 6) {
      inst_cycles += 4;
			val = __gb_read(gb, gb->cpu_reg.hl);
    }
		switch (cbop)
		{
			CB_REG_CASES(gb->cpu_reg.b, 0);
			CB_REG_CASES(gb->cpu_reg.c, 1);
			CB_REG_CASES(gb->cpu_reg.d, 2);
			CB_REG_CASES(gb->cpu_reg.e, 3);
			CB_REG_CASES(gb->cpu_reg.h, 4);
			CB_REG_CASES(gb->cpu_reg.l, 5);
      CB_REG_CASES(val, 6);
			CB_REG_CASES(gb->cpu_reg.a, 7);
		}
    if ((cbop & 0x07) == 6 && (cbop & 0xC0) != 0x40)
      __gb_write(gb, gb->cpu_reg.hl, val);
      inst_cycles += 4;
    }
    break;

  case 0xCC:  /* CALL Z, imm */
		if(gb->cpu_reg.f_bits.z)
		{
			uint16_t temp = __gb_read16(gb, gb->cpu_reg.pc);
      gb->cpu_reg.sp -= 2;
      __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc + 2);
			gb->cpu_reg.pc = temp;
			inst_cycles += 12;
		}
		else
			gb->cpu_reg.pc += 2;
    break;

  case 0xCD:  /* CALL imm */
	{
		uint16_t addr = __gb_read16(gb, gb->cpu_reg.pc);
    gb->cpu_reg.sp -= 2;
    __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc + 2);
		gb->cpu_reg.pc = addr;
	}
    break;

  case 0xCE:  /* ADC A, imm */
    {
		uint8_t value = __gb_read(gb, gb->cpu_reg.pc++);
    ADC8(gb->cpu_reg.a, value)
    }
    break;

  case 0xCF:  /* RST 0x0008 */
    gb->cpu_reg.sp -= 2;
    __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc);
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
			uint16_t temp =  __gb_read16(gb, gb->cpu_reg.pc);
			gb->cpu_reg.pc = temp;
			inst_cycles += 4;
		}
		else
			gb->cpu_reg.pc += 2;
    break;

  case 0xD4:  /* CALL NC, imm */
		if(!gb->cpu_reg.f_bits.c)
		{
			uint16_t temp = __gb_read16(gb, gb->cpu_reg.pc);
      gb->cpu_reg.sp -= 2;
      __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc + 2);
			gb->cpu_reg.pc = temp;
			inst_cycles += 12;
		}
		else
			gb->cpu_reg.pc += 2;
    break;

  case 0xD5:  /* PUSH DE */
    gb->cpu_reg.sp -= 2;
    __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.de);
    break;

  case 0xD6:  /* SUB imm */
    {
		uint8_t value = __gb_read(gb, gb->cpu_reg.pc++);
    SUB8(gb->cpu_reg.a, value)
    }
    break;

  case 0xD7:  /* RST 0x0010 */
    gb->cpu_reg.sp -= 2;
    __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc);
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
    goto __halt_entry;

  case 0xDA:  /* JP C, imm */
		if(gb->cpu_reg.f_bits.c)
		{
			uint16_t addr = __gb_read16(gb, gb->cpu_reg.pc);
			gb->cpu_reg.pc = addr;
			inst_cycles += 4;
		}
		else
			gb->cpu_reg.pc += 2;
    break;

  case 0xDC:  /* CALL C, imm */
		if(gb->cpu_reg.f_bits.c)
		{
			uint16_t temp = __gb_read16(gb, gb->cpu_reg.pc);
      gb->cpu_reg.sp -= 2;
      __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc + 2);
			gb->cpu_reg.pc = temp;
			inst_cycles += 12;
		}
		else
			gb->cpu_reg.pc += 2;
    break;

  case 0xDE:  /* SBC A, imm */
    {
		uint8_t value = __gb_read(gb, gb->cpu_reg.pc++);
    SBC8(gb->cpu_reg.a, value)
    }
    break;

  case 0xDF:  /* RST 0x0018 */
    gb->cpu_reg.sp -= 2;
    __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc);
		gb->cpu_reg.pc = 0x0018;
    break;

  case 0xE0:  /* LD (0xFF00+imm), A */
		__io_write(gb, 0xFF00 | __gb_read(gb, gb->cpu_reg.pc++), gb->cpu_reg.a);
    break;

  case 0xE1:  /* POP HL */
    gb->cpu_reg.hl = __gb_read16(gb, gb->cpu_reg.sp);
    gb->cpu_reg.sp += 2;
    break;

  case 0xE2:  /* LD (C), A */
		__io_write(gb, 0xFF00 | gb->cpu_reg.c, gb->cpu_reg.a);
    break;

  case 0xE5:  /* PUSH HL */
    gb->cpu_reg.sp -= 2;
    __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.hl);
    break;

  case 0xE6:  /* AND imm */
		/* TODO: Optimisation? */
		gb->cpu_reg.a = gb->cpu_reg.a & __gb_read(gb, gb->cpu_reg.pc++);
		gb->cpu_reg.f = ((gb->cpu_reg.a == 0x00) << 7) | 0x20;
    break;

  case 0xE7:  /* RST 0x0020 */
    gb->cpu_reg.sp -= 2;
    __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc);
		gb->cpu_reg.pc = 0x0020;
    break;

  case 0xE8:  /* ADD SP, imm */
	{
		int8_t offset = (int8_t) __gb_read(gb, gb->cpu_reg.pc++);
		gb->cpu_reg.f = 0;
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
		uint16_t addr = __gb_read16(gb, gb->cpu_reg.pc);
    gb->cpu_reg.pc += 2;
		__gb_write(gb, addr, gb->cpu_reg.a);
}
	    break;

  case 0xEE:  /* XOR imm */
		gb->cpu_reg.a = gb->cpu_reg.a ^ __gb_read(gb, gb->cpu_reg.pc++);
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xEF:  /* RST 0x0028 */
    gb->cpu_reg.sp -= 2;
    __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc);
		gb->cpu_reg.pc = 0x0028;
    break;

  case 0xF0:  /* LD A, (0xFF00+imm) */
		gb->cpu_reg.a =	__io_read(gb, 0xFF00 | __gb_read(gb, gb->cpu_reg.pc++));
    break;

  case 0xF1:  /* POP AF */
	{
    gb->cpu_reg.af = __gb_read16(gb, gb->cpu_reg.sp);
    gb->cpu_reg.f &= 0xF0;
    gb->cpu_reg.sp += 2;
}
	    break;

  case 0xF2:  /* LD A, (C) */
		gb->cpu_reg.a = __io_read(gb, 0xFF00 | gb->cpu_reg.c);
    break;

  case 0xF3:  /* DI */
		gb->gb_ime = 0;
    break;

  case 0xF5:  /* PUSH AF */
    gb->cpu_reg.sp -= 2;
    __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.af & 0xFFF0);
    break;

  case 0xF6:  /* OR imm */
		gb->cpu_reg.a = gb->cpu_reg.a | __gb_read(gb, gb->cpu_reg.pc++);
		gb->cpu_reg.f = (gb->cpu_reg.a == 0x00) << 7;
    break;

  case 0xF7:  /* PUSH AF */
    gb->cpu_reg.sp -= 2;
    __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc);
		gb->cpu_reg.pc = 0x0030;
    break;

  case 0xF8:  /* LD HL, SP+/-imm */
	{
		int8_t offset = (int8_t) __gb_read(gb, gb->cpu_reg.pc++);
		gb->cpu_reg.hl = gb->cpu_reg.sp + offset;
		gb->cpu_reg.f = 0;
		gb->cpu_reg.f_bits.h = ((gb->cpu_reg.sp & 0xF) + (offset & 0xF) > 0xF) ? 1 : 0;
		gb->cpu_reg.f_bits.c = ((gb->cpu_reg.sp & 0xFF) + (offset & 0xFF) > 0xFF) ? 1 : 0;
  }
	    break;

  case 0xF9:  /* LD SP, HL */
		gb->cpu_reg.sp = gb->cpu_reg.hl;
    break;

  case 0xFA:  /* LD A, (imm) */
	{
		uint16_t addr = __gb_read16(gb, gb->cpu_reg.pc);
    gb->cpu_reg.pc += 2;
		gb->cpu_reg.a = __gb_read(gb, addr);
  }
	    break;

  case 0xFB:  /* EI */
		gb->gb_ime = 1;
    goto __halt_entry;

  case 0xFE:  /* CP imm */
    {
		uint8_t value = __gb_read(gb, gb->cpu_reg.pc++);
    CP8(gb->cpu_reg.a, value)
    }
    break;

  case 0xFF:  /* RST 0x0038 */
    gb->cpu_reg.sp -= 2;
    __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc);
		gb->cpu_reg.pc = 0x0038;
    break;
    }

    /* DIV register timing */
    gb->counter.div_count += inst_cycles;
    gb->gb_reg.DIV = gb->counter.div_count >> 8;

    /* TIMA register timing */
    if(gb->gb_reg.tac_enable)
    {
      gb->counter.tima_count += inst_cycles;

      if(gb->counter.tima_count >= gb->gb_tac_rate)
      {
        gb->counter.tima_count -= gb->gb_tac_rate;

        if(++gb->gb_reg.TIMA == 0)
        {
          __hw_interrupt(gb, TIMER_INTR, TIMER_INTR);
          __hw_interrupt(gb, 0, TIMER_INTR);
          /* On overflow, set TMA to TIMA. */
          gb->gb_reg.TIMA = gb->gb_reg.TMA;
          /* Handle interrupts */
          if (gb->gb_ime && (gb->gb_reg.IF & gb->gb_reg.IE & TIMER_INTR))
          {
            /* Disable interrupts */
            gb->gb_ime = 0;

            /* Push Program Counter */
            gb->cpu_reg.sp -= 2;
            __gb_write16(gb, gb->cpu_reg.sp, gb->cpu_reg.pc);
            gb->cpu_reg.pc = TIMER_INTR_ADDR;
            gb->gb_reg.IF ^= TIMER_INTR;
          }
        }
      }
    }
#ifdef  ENABLE_GBC
    inst_cycles >>= gb->gb_cpu_speed;
#endif

    /* LCD Timing */
    gb->counter.lcd_count -= inst_cycles;
  }
#ifdef  PROFILE
  PROFILE_END
#endif

  /* Check serial transfer. Probably slow enough to handle outside of the main loop. */
  /*if(gb->gb_reg.SC & 0x80)
  {
    if (gb->gb_reg.SC & 1) {
      // Internal clock.
      gb->counter.serial_count += 0;
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
        gb->counter.serial_count += 0;
      }
    }
  }*/

  if (gb->gb_forced_int) {
    gb->counter.lcd_count += gb->gb_forced_int;
    gb->gb_forced_int = 0;
    goto __int_entry;
  }
#ifdef  ENABLE_SOUND
  old_lcd_count -= gb->counter.lcd_count;
  gb->counter.sound_count += old_lcd_count;
  if (gb->gb_audio_flag) {
    gb->gb_audio_flag = 0;
    audio_mix(gb->counter.sound_count);
    gb->counter.sound_count = 0;
  }
#endif
}

#ifdef  ENABLE_GBC
void __gb_hdma(struct gb_s *gb) {
  uint16_t src = (gb->gb_reg.HDMA1 << 8) | gb->gb_reg.HDMA2;
  uint16_t dst = 0x8000 | ((gb->gb_reg.HDMA3 & 0x1F) << 8) | gb->gb_reg.HDMA4;
  uint32_t cnt = 16;
  uint8_t *da = gb->addr_wmap[8] + dst;
  dst += cnt;
  while (cnt--)
    *da++ = __gb_read(gb, src++);
  gb->gb_reg.HDMA1 = src >> 8;
  gb->gb_reg.HDMA2 = src & 0xF0;
  gb->gb_reg.HDMA3 = (dst >> 8) & 0x1F;
  gb->gb_reg.HDMA4 = dst & 0xF0;
  gb->gb_reg.HDMA5--;
}
#endif

void __gb_step_line(struct gb_s *gb) {
	while (!(gb->gb_reg.LCDC & 0x80)) {
    switch (gb->gb_reg.STAT & 3) {
      case 0:
      case 1:
        __stat_change(gb, 2);
        __gb_step_cpu(gb, 80);
        break;
      case 2:
        __stat_change(gb, 3);
        __gb_step_cpu(gb, 172);
        break;
      case 3:
        __stat_change(gb, 0);
#ifdef  ENABLE_GBC
        if (gb->gb_reg.HDMA5 != 0xFF)
          __gb_hdma(gb);
#endif
        __gb_step_cpu(gb, 204);
        break;
    }
	}
  do {
    switch (gb->gb_reg.STAT & 3) {
      case 1:
        if (!(gb->gb_ints & VBLANK_INTR)) {
          __hw_interrupt(gb, VBLANK_INTR, VBLANK_INTR);
          gb->counter.lcd_count += 436;
          break;
        }
        if (gb->gb_reg.LY == 0) {
          gb->display.window_clear = 0;
          gb->display.WY = gb->gb_reg.WY;
          __stat_change(gb, 2);
          gb->counter.lcd_count += 80;
          break;
        }
        else if (gb->gb_reg.LY < 152)
          gb->counter.lcd_count += 456;
        else if (gb->gb_reg.LY == 152)
          gb->counter.lcd_count += 56;
        else
        {
          gb->gb_reg.LY = -1;
          gb->counter.lcd_count += 400;
          gb->gb_frame = 1;
        }
        gb->gb_reg.LY++;
        __stat_trigger(gb);
        break;
      case 2:
#if ENABLE_LCD
#ifdef  ENABLE_GBC
        if (gb->is_gbc)
          __gb_draw_gbc_line(gb);
        else
#endif
          __gb_draw_line(gb);
#endif
        __stat_change(gb, 3);
        __gb_step_cpu(gb, 92);
#if ENABLE_LCD
        if (gb->display.line_cnt)
          gb_lcd_send_chunk(gb);
#endif
        gb->counter.lcd_count += 172 - 92;
        break;
      case 3:
        __stat_change(gb, 0);
#ifdef  ENABLE_GBC
        if (gb->gb_reg.HDMA5 != 0xFF)
          __gb_hdma(gb);
#endif
#if ENABLE_LCD
        if (gb->display.line_cnt)
          gb_lcd_send_chunk(gb);
#endif
        __gb_step_cpu(gb, 100);
#if ENABLE_LCD
        if (gb->display.line_cnt)
          gb_lcd_send_chunk(gb);
#endif
        gb->counter.lcd_count += 204 - 100;
        break;
      case 0:
#if ENABLE_LCD
        if (gb->display.line_cnt)
          gb_lcd_send_chunk(gb);
#endif
        if (++gb->gb_reg.LY >= 144)
        {
          if (gb->gb_halt)
          {
            __hw_interrupt(gb, VBLANK_INTR, VBLANK_INTR);
            gb->counter.lcd_count += 456;
          }
          else gb->counter.lcd_count += 20;
          __stat_change(gb, 1);
          break;
        }
        __stat_change(gb, 2);
        gb->counter.lcd_count += 80;
        break;
    }
    __gb_step_cpu(gb, 0);
  } while ((gb->gb_reg.STAT & 3) != 2 && (gb->gb_reg.LCDC & LCDC_ENABLE));
}

void gb_run_frame(struct gb_s *gb)
{
	gb->gb_frame = 0;
#ifdef  ENABLE_SOUND
  gb->counter.sound_count = 0;
#endif
#if ENABLE_LCD
  gb->display.line_num = 0;
  /* If frame skip is activated, check if we need to draw
    * the frame or skip it. */
  gb->display.frame_skip_count++;
  if (gb->display.frame_skip_count > gb->direct.frame_skip) {
    gb->display.frame_skip_count = 0;
    gb_lcd_start_frame(gb);
  }
#endif

	while(!gb->gb_frame) {
    for (int i = 0; i < 32 && !gb->gb_frame; i++)
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
  memcpy(gb->write_regs, __write_regs, sizeof(gb->write_regs));
  gb->tac_cycles[0] = 1024;
  gb->tac_cycles[1] = 16;
  gb->tac_cycles[2] = 64;
  gb->tac_cycles[3] = 256;
  gb->condbits[0] = 0x08;
  gb->condbits[1] = 0x30;
  gb->condbits[2] = 0x20;
  gb->condbits[3] = 0x00;

  for (int i = 0; i < 256; i++) {
		gb->inc_flag[i] = ((i == 0x00) << 7) | (((i & 0x0F) == 0x00) << 5);
  }
  for (int i = 0; i < 256; i++) {
		gb->dec_flag[i] = ((i == 0x00) << 7) | (((i & 0x0F) == 0x0F) << 5) | 0x40;
  }

  for (int i = 0; i < 256; i++) {
    gb->pixmap[i] = ((((i >> 0) & 1) | ((i >> 3) & 2)) << 0) | ((((i >> 1) & 1) | ((i >> 4) & 2)) << 8)
      | ((((i >> 2) & 1) | ((i >> 5) & 2)) << 16) | ((((i >> 3) & 1) | ((i >> 6) & 2)) << 24);
  }

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

#ifdef  ENABLE_GBC
  for (int i = 0; i < 256; i++) {
    uint32_t v = (i >> 4) | (i << 4);
    v = ((v & 0x33) << 2) | ((v & 0xCC) >> 2);
    gb->display.flipmap[i] = ((v & 0x55) << 1) | ((v & 0xAA) >> 1);
  }
#endif
  gb->overlay = 0;
  gb->fpsmode = 2;

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
