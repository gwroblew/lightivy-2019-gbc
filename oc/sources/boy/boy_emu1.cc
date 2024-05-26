
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "oc.h"
#include "boy_emu.h"

#define PROFILE     (1)

#define PROFILE_START   { __prof_result = 0; }
#define PROFILE_BEGIN   { __prof_ccount = asm_ccount(); }
#define PROFILE_END     { __prof_result += asm_ccount() - __prof_ccount; }

#ifdef  PROFILE
uint32_t RWDATA __prof_ccount;
uint32_t RWDATA __prof_result;

static inline uint32_t asm_ccount(void) {
#ifndef OCEMU
  uint32_t r;
  asm volatile ("rsr %0, ccount" : "=r"(r));
  return r;
#else
  return 0;
#endif
}
#endif

extern SYSCALLS * RWDATA gsys;
extern uint16_t * RWDATA framebuffer;

uint8_t * RWDATA rom_buffer;
uint8_t * RWDATA cart_ram_buffer;

struct gb_s;
enum gb_error_e;

#ifdef  OCEMU
extern uint16_t *framebuf;
uint16_t *frameptr;
extern void update_frame();
#endif

void gb_lcd_start_frame(struct gb_s *gb);

void gb_lcd_send_chunk(struct gb_s *gb);

/**
 * Return byte from ROM at given address.
 *
 * \param gb_s	emulator context
 * \param addr	address
 * \return		byte at address in ROM
 */
uint8_t gb_rom_read(struct gb_s*, const uint_fast32_t addr);

/**
 * Return byte from cart RAM at given address.
 *
 * \param gb_s	emulator context
 * \param addr	address
 * \return		byte at address in RAM
 */
uint8_t gb_cart_ram_read(struct gb_s*, const uint_fast32_t addr);

/**
 * Write byte to cart RAM at given address.
 *
 * \param gb_s	emulator context
 * \param addr	address
 * \param val	value to write to address in RAM
 */
void gb_cart_ram_write(struct gb_s*, const uint_fast32_t addr, const uint8_t val);

/**
 * Notify front-end of error.
 *
 * \param gb_s			emulator context
 * \param gb_error_e	error code
 * \param val			arbitrary value related to error
 */
void gb_error(struct gb_s*, const enum gb_error_e, const uint16_t val);

/* Transmit one byte and return 1 if success. */
uint32_t gb_serial_transfer(struct gb_s *gb);
void gb_serial_ready(struct gb_s *gb);

static inline void gb_set_palette(uint32_t color, uint32_t idx) {
  uint16_t *pal = framebuffer;
  //uint16_t c = ((r << 8) & 0xF800) | ((g << 3) & 0x7E0) | (b >> 3);
  //pal[idx] = (c << 8) | (c >> 8);
  //uint16_t c = ((rgb & 0x7FE0) << 1) | (rgb & 0x1F);
  uint16_t c = ((color & 0x7C00) >> 10) | ((color & 0x3E0) << 1) | ((color & 0x1F) << 11);
  pal[idx] = (c << 8) | (c >> 8);
}

#include "sound.h"
#include "peanut_gb.h"

void gb_lcd_start_frame(struct gb_s *gb) {
#ifdef  OCEMU
  frameptr = framebuf;
#else
#endif
  gb->display.chunk_cnt = 5 * 128;
  gb->display.line_cnt = 0;
}

void gb_lcd_send_chunk(struct gb_s *gb) {
  if (gb->display.chunk_cnt == 0) {
    gb->display.line_cnt = 0;
    return;
  }
  uint16_t *pal = (uint16_t *)gb->display.framebuffer;
  uint16_t *dst = (uint16_t *)(pal + 256 + 80) + 160 - gb->display.line_cnt;
  if (gb->display.line_cnt == 160) {
    uint8_t *src = (uint8_t *)(pal + 256) + (160 - gb->display.line_cnt);
    for (int i = 0; i < 160; i++) {
      *dst++ = pal[*src++];
    }
    dst -= 160;
  }
  gb->display.line_cnt -= 32;
  gb->display.chunk_cnt--;
#ifdef  OCEMU
  for (int i = 0; i < 32; i++) {
    uint16_t c = *dst++;
    *frameptr++ = (c << 8) | (c >> 8);
  }
  if (gb->display.chunk_cnt == 0)
    update_frame();
#else
  gsys->SendScreenData((uint32_t *)dst, 16);
#endif
}

struct gb_s * RWDATA gb;

/*#define ROM_CACHE_SIZE    (32)
#define ROM_CACHE_MASK    (ROM_CACHE_SIZE - 1)
#define ROM_ADDR_MASK     (0xFFFFFFFF ^ ROM_CACHE_MASK)

uint8_t RWDATA rom_cache[ROM_CACHE_SIZE];
uint32_t RWDATA rom_addr;*/

/**
 * Returns a byte from the ROM file at the given address.
 */
uint8_t gb_rom_read(struct gb_s *gb, const uint_fast32_t addr)
{
#ifndef OCEMU
  register uint32_t da asm("a3") = addr + (uint32_t)gb->rom_bank0;
  uint32_t r;
  asm volatile (
      "movi.n     %0, -4      \n"
      "and        %0, a3, %0  \n"
      "l32i.n     %0, %0, 0   \n"
      "ssa8l      a3          \n"
      "srl        %0, %0      \n"
      : "=r"(r) : "r"(da)
  );
  return r;
#else
  uint8_t *ptr = addr + FLASH_ADDR(GAME_ROM_ADDR);//gb->rom_bank0;
  return *ptr;
#endif
//  uint32_t da = addr + rom_buffer;
//  return (*(uint32_t *)(da & 0xFFFFFFFC)) >> ((da & 3) * 8);
/*  uint32_t caddr = addr & ROM_ADDR_MASK;
  if (rom_addr == caddr) {
    return rom_cache[addr & ROM_CACHE_MASK];
  }
  rom_addr = caddr;
  caddr >>= 2;
  uint32_t *dst = (uint32_t *)rom_cache;
  uint32_t *src = (uint32_t *)GB_ROM;
  for (int i = 0; i < ROM_CACHE_SIZE / 4; i++)
    dst[i] = src[caddr + i];
  return rom_cache[addr & ROM_CACHE_MASK];*/
}

uint8_t gb_rom_bank_read(struct gb_s *gb, const uint_fast32_t add)
{
#ifndef OCEMU
  register uint32_t da asm("a3") = add + gb->rom_bank_addr;
  uint32_t r;
  asm volatile (
      "movi.n     %0, -4      \n"
      "and        %0, a3, %0  \n"
      "l32i.n     %0, %0, 0   \n"
      "ssa8l      a3          \n"
      "srl        %0, %0      \n"
      : "=r"(r) : "r"(da)
  );
  return r;
#else
  uint32_t da = add + gb->rom_bank_addr;
  return (*(uint32_t *)(da & 0xFFFFFFFC)) >> ((da & 3) * 8);
#endif
}

/**
 * Returns a byte from the cartridge RAM at the given address.
 */
uint8_t gb_cart_ram_read(struct gb_s *gb, const uint_fast32_t addr)
{
//  printf("R: %04X\n", addr);
#ifdef  OCEMU
  return cart_ram_buffer[addr & 0x1FFF];
#else
  register uint32_t da asm("a3") = (addr & 0x1FFF) + (uint32_t)cart_ram_buffer;
  uint32_t r;
  asm volatile (
      "movi.n     %0, -4      \n"
      "and        %0, a3, %0  \n"
      "l32i.n     %0, %0, 0   \n"
      "ssa8l      a3          \n"
      "srl        %0, %0      \n"
      : "=r"(r) : "r"(da)
  );
  return r;
#endif
}

/**
 * Writes a given byte to the cartridge RAM at the given address.
 */
void gb_cart_ram_write(struct gb_s *gb, const uint_fast32_t addr,
		       const uint8_t val)
{
//  printf("W: %04X\n", addr);
  cart_ram_buffer[addr & 0x1FFF] = val;
}

/**
 * Automatically assigns a colour palette to the game using a given game
 * checksum.
 * TODO: Not all checksums are programmed in yet because I'm lazy.
 */
void auto_assign_palette(uint8_t game_checksum)
{
	size_t palette_bytes = 3 * 4 * sizeof(uint16_t);

	switch(game_checksum)
	{
	/* Balloon Kid and Tetris Blast */
	case 0x71:
	case 0xFF:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x7E60, 0x7C00, 0x0000 }, /* OBJ0 */
			{ 0x7FFF, 0x7E60, 0x7C00, 0x0000 }, /* OBJ1 */
			{ 0x7FFF, 0x7E60, 0x7C00, 0x0000 }  /* BG */
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* Pokemon Yellow and Tetris */
	case 0x15:
	case 0xDB:
	case 0x95: /* Not officially */
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x7FE0, 0x7C00, 0x0000 }, /* OBJ0 */
			{ 0x7FFF, 0x7FE0, 0x7C00, 0x0000 }, /* OBJ1 */
			{ 0x7FFF, 0x7FE0, 0x7C00, 0x0000 }  /* BG */
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* Donkey Kong */
	case 0x19:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x7E10, 0x48E7, 0x0000 }, /* OBJ0 */
			{ 0x7FFF, 0x7E10, 0x48E7, 0x0000 }, /* OBJ1 */
			{ 0x7FFF, 0x7E60, 0x7C00, 0x0000 }  /* BG */
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* Pokemon Blue */
	case 0x61:
	case 0x45:

	/* Pokemon Blue Star */
	case 0xD8:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x7E10, 0x48E7, 0x0000 }, /* OBJ0 */
			{ 0x7FFF, 0x329F, 0x001F, 0x0000 }, /* OBJ1 */
			{ 0x7FFF, 0x329F, 0x001F, 0x0000 }  /* BG */
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* Pokemon Red */
	case 0x14:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x3FE6, 0x0200, 0x0000 }, /* OBJ0 */
			{ 0x7FFF, 0x7E10, 0x48E7, 0x0000 }, /* OBJ1 */
			{ 0x7FFF, 0x7E10, 0x48E7, 0x0000 }  /* BG */
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* Pokemon Red Star */
	case 0x8B:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x7E10, 0x48E7, 0x0000 }, /* OBJ0 */
			{ 0x7FFF, 0x329F, 0x001F, 0x0000 }, /* OBJ1 */
			{ 0x7FFF, 0x3FE6, 0x0200, 0x0000 }  /* BG */
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* Kirby */
	case 0x27:
	case 0x49:
	case 0x5C:
	case 0xB3:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7D8A, 0x6800, 0x3000, 0x0000 }, /* OBJ0 */
			{ 0x001F, 0x7FFF, 0x7FEF, 0x021F }, /* OBJ1 */
			{ 0x527F, 0x7FE0, 0x0180, 0x0000 }  /* BG */
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* Donkey Kong Land [1/2/III] */
	case 0x18:
	case 0x6A:
	case 0x4B:
	case 0x6B:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7F08, 0x7F40, 0x48E0, 0x2400 }, /* OBJ0 */
			{ 0x7FFF, 0x2EFF, 0x7C00, 0x001F }, /* OBJ1 */
			{ 0x7FFF, 0x463B, 0x2951, 0x0000 }  /* BG */
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* Link's Awakening */
	case 0x70:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x03E0, 0x1A00, 0x0120 }, /* OBJ0 */
			{ 0x7FFF, 0x329F, 0x001F, 0x001F }, /* OBJ1 */
			{ 0x7FFF, 0x7E10, 0x48E7, 0x0000 }  /* BG */
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* Mega Man [1/2/3] & others I don't care about. */
	case 0x01:
	case 0x10:
	case 0x29:
	case 0x52:
	case 0x5D:
	case 0x68:
	case 0x6D:
	case 0xF6:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x329F, 0x001F, 0x0000 }, /* OBJ0 */
			{ 0x7FFF, 0x3FE6, 0x0200, 0x0000 }, /* OBJ1 */
			{ 0x7FFF, 0x7EAC, 0x40C0, 0x0000 }  /* BG */
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	default:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x5294, 0x294A, 0x0000 },
			{ 0x7FFF, 0x5294, 0x294A, 0x0000 },
			{ 0x7FFF, 0x5294, 0x294A, 0x0000 }
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
	}
	}

  for (int i = 0; i < 3; i++) {
    for(int j = 0; j < 4; j++) {
      uint32_t c = gb->display.gb_palette[i][j];
      gb->display.gb_palette[i][j] = ((c & 0x7C00) >> 10) | (c & 0x3E0) | ((c & 0x1F) << 10);
    }
  }
}

#define NUMBER_OF_PALETTES 12

/**
 * Assigns a palette. This is used to allow the user to manually select a
 * different colour palette if one was not found automatically, or if the user
 * prefers a different colour palette.
 * selection is the requestion colour palette. This should be a maximum of
 * NUMBER_OF_PALETTES - 1. The default greyscale palette is selected otherwise.
 */
void manual_assign_palette(uint8_t selection)
{
	size_t palette_bytes = 3 * 4 * sizeof(uint16_t);

	switch(selection)
	{
	/* 0x05 (Right) */
	case 0:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x2BE0, 0x7D00, 0x0000 },
			{ 0x7FFF, 0x2BE0, 0x7D00, 0x0000 },
			{ 0x7FFF, 0x2BE0, 0x7D00, 0x0000 }
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* 0x07 (A + Down) */
	case 1:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x7FE0, 0x7C00, 0x0000 },
			{ 0x7FFF, 0x7FE0, 0x7C00, 0x0000 },
			{ 0x7FFF, 0x7FE0, 0x7C00, 0x0000 }
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* 0x12 (Up) */
	case 2:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x7EAC, 0x40C0, 0x0000 },
			{ 0x7FFF, 0x7EAC, 0x40C0, 0x0000 },
			{ 0x7FFF, 0x7EAC, 0x40C0, 0x0000 }
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* 0x13 (B + Right) */
	case 3:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x0000, 0x0210, 0x7F60, 0x7FFF },
			{ 0x0000, 0x0210, 0x7F60, 0x7FFF },
			{ 0x0000, 0x0210, 0x7F60, 0x7FFF }
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* 0x16 (B + Left, DMG Palette) */
	default:
	case 4:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x5294, 0x294A, 0x0000 },
			{ 0x7FFF, 0x5294, 0x294A, 0x0000 },
			{ 0x7FFF, 0x5294, 0x294A, 0x0000 }
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* 0x17 (Down) */
	case 5:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FF4, 0x7E52, 0x4A5F, 0x0000 },
			{ 0x7FF4, 0x7E52, 0x4A5F, 0x0000 },
			{ 0x7FF4, 0x7E52, 0x4A5F, 0x0000 }
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* 0x19 (B + Up) */
	case 6:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x7EAC, 0x40C0, 0x0000 },
			{ 0x7FFF, 0x7EAC, 0x40C0, 0x0000 },
			{ 0x7F98, 0x6670, 0x41A5, 0x2CC1 }
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* 0x1C (A + Right) */
	case 7:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x7E10, 0x48E7, 0x0000 },
			{ 0x7FFF, 0x7E10, 0x48E7, 0x0000 },
			{ 0x7FFF, 0x3FE6, 0x0198, 0x0000 }
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* 0x0D (A + Left) */
	case 8:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x7E10, 0x48E7, 0x0000 },
			{ 0x7FFF, 0x7EAC, 0x40C0, 0x0000 },
			{ 0x7FFF, 0x463B, 0x2951, 0x0000 }
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* 0x10 (A + Up) */
	case 9:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x3FE6, 0x0200, 0x0000 },
			{ 0x7FFF, 0x329F, 0x001F, 0x0000 },
			{ 0x7FFF, 0x7E10, 0x48E7, 0x0000 }
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* 0x18 (Left) */
	case 10:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x7E10, 0x48E7, 0x0000 },
			{ 0x7FFF, 0x3FE6, 0x0200, 0x0000 },
			{ 0x7FFF, 0x329F, 0x001F, 0x0000 }
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}

	/* 0x1A (B + Down) */
	case 11:
	{
		const uint16_t palette[3][4] =
		{
			{ 0x7FFF, 0x329F, 0x001F, 0x0000 },
			{ 0x7FFF, 0x3FE6, 0x0200, 0x0000 },
			{ 0x7FFF, 0x7FE0, 0x3D20, 0x0000 }
		};
		memcpy(gb->display.gb_palette, palette, palette_bytes);
		break;
	}
	}

  for (int i = 0; i < 3; i++) {
    for(int j = 0; j < 4; j++) {
      uint32_t c = gb->display.gb_palette[i][j];
      gb->display.gb_palette[i][j] = ((c & 0x7C00) >> 10) | (c & 0x3E0) | ((c & 0x1F) << 10);
    }
  }
}

uint32_t RWDATA last_ticks;
uint32_t RWDATA rtc_timer;
uint32_t RWDATA palidx;
uint32_t RWDATA lastkeys;

void ROCODE gb_error_callback(struct gb_s *gb, const enum gb_error_e err, const uint16_t val) {
  gsys->Printf("Error: %d %04X\n", err, val);
  gsys->SleepMs(3000);
}

uint16_t RWDATA serial_byte_in;
uint16_t RWDATA serial_byte_flag;

uint32_t ROCODE gb_serial_transfer(struct gb_s *gb) {
  if (gb->gb_reg.SC & 1) {
    // Internal clock.
    gb->gb_reg.SB = serial_byte_in;
    serial_byte_in = 0xFF;
    return 1;
  }
  // External clock.
  if (serial_byte_flag == 0)
    return 0;
  gb->gb_reg.SB = serial_byte_in;
  serial_byte_in = 0xFF;
  serial_byte_flag = 0;
  return 1;
}

void ROCODE gb_serial_ready(struct gb_s *gb) {
}

void ROCODE gb_serial_recieve(uint8_t b) {
  serial_byte_in = b;
  serial_byte_flag = 1;
}

#include "font8x8_basic.h"

/*const uint8_t RODATA font8x8_borders[64] = {
  0xFF, 0xFF, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
  0xFF, 0xFF, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
  0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0xFF, 0xFF,
  0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xFF, 0xFF,
  0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
  0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
  0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
};*/

const uint8_t RODATA font8x8_borders[64] = {
  0x00, 0xFE, 0xFE, 0x06, 0x06, 0x06, 0x06, 0x06,
  0x00, 0x7F, 0x7F, 0x60, 0x60, 0x60, 0x60, 0x60,
  0x06, 0x06, 0x06, 0x06, 0x06, 0xFE, 0xFE, 0x00,
  0x60, 0x60, 0x60, 0x60, 0x60, 0x7F, 0x7F, 0x00,
  0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00,
  0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
  0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60,
};

// 01234567890123456789
//   ----------------
//   |  Volume: X   |
//   |  Load Game   |
//   |  Save Game   |
//   |    Reset     |
//   |  Mode: 20FPS |
//   |    Exit      |
//   ----------------

extern void rom_i2c_writeReg_Mask(uint32_t block, uint32_t host_id,
                                  uint32_t reg_add, uint32_t Msb, uint32_t Lsb, uint32_t indata);

#define SAR_BASE ((volatile uint32_t *)0x60000D00)
#define SAR_CFG (SAR_BASE + 20)
#define SAR_TIM1 (SAR_BASE + 21)
#define SAR_TIM2 (SAR_BASE + 22)
#define SAR_CFG1 (SAR_BASE + 23)
#define SAR_CFG2 (SAR_BASE + 24)
#define SAR_DATA (SAR_BASE + 32)
//static volatile uint8_t *const tout_dis_txpwr_track = (void *)0x3ffe9674;

void ROCODE adc_enable() {
  gsys->GetBatteryLevel();
#ifndef OCEMU
  // Inform the vendor libs that ADC is in use
  //*tout_dis_txpwr_track = 1;

  // ADC clock speed is 3 MHz, /16/8 means one set of 8 SAR samples takes about 42 us.
  uint32_t clk_div = 16;
  uint32_t win_cnt = 8;
  *SAR_CFG = (*SAR_CFG & 0xFFFF00E3) | ((win_cnt - 1) << 2) | (clk_div << 8);
  *SAR_TIM1 = (*SAR_TIM1 & 0xFF000000) | (clk_div * 5 + ((clk_div - 1) << 16) + ((clk_div - 1) << 8) - 1);
  *SAR_TIM2 = (*SAR_TIM2 & 0xFF000000) | (clk_div * 11 + ((clk_div * 3 - 1) << 8) + ((clk_div * 11 - 1) << 16) - 1);
  rom_i2c_writeReg_Mask(108, 2, 0, 5, 5, 1);
  *SAR_CFG1 |= 1 << 21;
#endif
}

void ROCODE adc_sample() {
  // Start reading next sample.
  uint32_t v = *SAR_CFG;
  *SAR_CFG = v & ~2;
  *SAR_CFG = v | 2;
}

uint32_t ROCODE adc_read_result() {
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) {
    uint32_t x = ~SAR_DATA[i] & 0x7FF;
    x += (x & 0xFF) * 23 / 256;  // Improves linearity a bit
    sum += x;
  }
  return sum >> 4;
}

uint32_t ROCODE getadc() {
#ifdef OCEMU
  return 955;
#else
  adc_sample();
  gsys->SleepUs(100);
  return adc_read_result();
#endif
}

void ROCODE boy_Menu() {
  strcpy(&gb->charmap[4 * 20 + 2], "\x18\x1C\x1C\x1C\x1C\x1C\x1C\x1C\x1C\x1C\x1C\x1C\x1C\x1C\x1C\x19");
  strcpy(&gb->charmap[5 * 20 + 2], "\x1E  Volume: 0   \x1F");
  strcpy(&gb->charmap[6 * 20 + 2], "\x1E  Load Game   \x1F");
  strcpy(&gb->charmap[7 * 20 + 2], "\x1E  Save Game   \x1F");
  strcpy(&gb->charmap[8 * 20 + 2], "\x1E    Reset     \x1F");
  strcpy(&gb->charmap[9 * 20 + 2], "\x1E Mode: 20 FPS \x1F");
  strcpy(&gb->charmap[10 * 20 + 2], "\x1E Battery: 000 \x1F");
  strcpy(&gb->charmap[11 * 20 + 2], "\x1A\x1D\x1D\x1D\x1D\x1D\x1D\x1D\x1D\x1D\x1D\x1D\x1D\x1D\x1D\x1B");
  uint32_t bl = getadc();
  bl -= 840;
  if (bl > 1000)
    bl = 0;
  bl = bl * 100 / (1024 - 840);
  if (bl > 100)
    bl = 100;
  uint32_t bl1 = bl % 10;
  bl /= 10;
  uint32_t bl2 = bl % 10;
  bl /= 10;
  gb->charmap[10 * 20 + 15] = bl1 + '0';
  gb->charmap[10 * 20 + 14] = 32;
  gb->charmap[10 * 20 + 13] = 32;
  if (bl != 0) {
    gb->charmap[10 * 20 + 14] = bl2 + '0';
    gb->charmap[10 * 20 + 13] = bl + '0';
  } else if (bl2 != 0)
    gb->charmap[10 * 20 + 14] = bl2 + '0';
}

void ROCODE boy_MenuSel() {
  for (int i = 5; i < 11; i++) {
    gb->colmap[i*20+2] = 0xF3;
    memset(&gb->colmap[i*20+3], 0xF2, 14);
    gb->colmap[i * 20 + 17] = 0xF3;
  }
  memset(&gb->colmap[10 * 20 + 3], 0xF4, 14);
  memset(&gb->colmap[4 * 20 + 2], 0xF3, 16);
  memset(&gb->colmap[11 * 20 + 2], 0xF3, 16);
  memset(&gb->colmap[(gb->menuidx + 5) * 20 + 3], 0xF1, 14);
  uint32_t v = audio_get_volume();
  gb->charmap[5 * 20 + 13] = 32;
  gb->charmap[5 * 20 + 14] = 32;
  gb->charmap[5 * 20 + 14] = (v % 10) + '0';
  v /= 10;
  if (v != 0)
    gb->charmap[5 * 20 + 13] = v + '0';
  switch (gb->fpsmode) {
    case 0:
      gb->charmap[9 * 20 + 10] = '6';
      break;
    case 1:
      gb->charmap[9 * 20 + 10] = '3';
      break;
    case 2:
      gb->charmap[9 * 20 + 10] = '2';
      break;
  }
}

void ROCODE boy_Init(char *savefile) {
  adc_enable();
  uint8_t *audiobuf;
  rom_buffer = ((uint8_t *)framebuffer) + 1 * 1024;
#ifdef  ENABLE_GBC
  audiobuf = rom_buffer + 16 * 1024;
  audio_init(rom_buffer + 20 * 1024 + 256);
  gb = (struct gb_s *)(rom_buffer + 21 * 1024);
  //printf("GB: %d\n", sizeof(struct gb_s));
  cart_ram_buffer = (uint8_t *)gsys->GetMemBlock(MEM_TYPE_IRAM, 16384) + 8192;
#else
  audiobuf = rom_buffer + 16 * 1024;
  audio_init(rom_buffer + 20 * 1024 + 256);
  gb = (struct gb_s *)(rom_buffer + 21 * 1024);
  cart_ram_buffer = (uint8_t *)gsys->GetMemBlock(MEM_TYPE_IRAM, 16384) + 8192;
#endif
  memcpy(rom_buffer, FLASH_ADDR(0x80000), 16384);
  palidx = 0;
  lastkeys = 0;

  //rom_addr = 0xFFFFFFFF;
  serial_byte_in = 0xFF;
  serial_byte_flag = 0;

  memset(gb, 0, sizeof(struct gb_s));
  strcpy(gb->savefile, savefile);
  int l = strlen(savefile);
  strcpy(&gb->savefile[l - 3], "bin");

  rtc_timer = 0;
  last_ticks = gsys->GetMsTicks();

	enum gb_init_error_e gb_ret = gb_init(gb, rom_buffer);

	switch(gb_ret)
	{
	case GB_INIT_NO_ERROR:
		break;
	case GB_INIT_CARTRIDGE_UNSUPPORTED:
		gsys->Printf("Unsupported cartridge.");
		break;
	case GB_INIT_INVALID_CHECKSUM:
		gsys->Printf("Invalid ROM: Checksum failure.");
		break;
	default:
		gsys->Printf("Unknown error: %d\n", gb_ret);
		break;
	}
  if (gb_ret != GB_INIT_NO_ERROR)
    return;

  if (gb_get_save_size(gb) > 0) {
    gsys->Printf("RAM: %d\n", gb_get_save_size(gb));
  }

	/* Set the RTC of the game cartridge. Only used by games that support it. */
	{
		struct tm timeinfo;
    memset(&timeinfo, 0, sizeof(timeinfo));
    timeinfo.tm_year = 2019;
    timeinfo.tm_mon = 6;
    timeinfo.tm_mday = 15;
    timeinfo.tm_yday = 150;
    timeinfo.tm_wday = 4;
    timeinfo.tm_hour = 11;
    timeinfo.tm_min = 21;

		/* You could potentially force the game to allow the player to
		 * reset the time by setting the RTC to invalid values.
		 *
		 * Using memset(gb->cart_rtc, 0xFF, sizeof(gb->cart_rtc)) for
		 * example causes Pokemon Gold/Silver to say "TIME NOT SET",
		 * allowing the player to set the time without having some dumb
		 * password.
		 *
		 * The memset has to be done directly to gb->cart_rtc because
		 * gb_set_rtc() processes the input values, which may cause
		 * games to not detect invalid values.
		 */

		/* Set RTC. Only games that specify support for RTC will use
		 * these values. */
		gb_set_rtc(gb, &timeinfo);
	}

#if ENABLE_LCD
  gb->direct.frame_skip = gb->fpsmode;
  gb->display.framebuffer = (uint8_t *)framebuffer;
#endif

	{
		char title_str[16] = "";
		gsys->Printf("ROM: %s\n", gb_get_rom_name(gb, title_str));
		gsys->Printf("MBC: %d  ROM: %d kB  RAM: %d kB\n", gb->mbc, gb->num_rom_banks * 16, gb->num_ram_banks * 8);
	}

	auto_assign_palette(gb_colour_hash(gb));

  memcpy(gb->font + 64, font8x8_basic, 96 * 8);
  memcpy(gb->font, font8x8_borders, 64);
  gb_set_palette(0x0000, 0xF0);
  gb_set_palette(0x7FFF, 0xF1);
  gb_set_palette(0x3DEF, 0xF2);
  gb_set_palette(0x6543, 0xF3);
  gb_set_palette(0x0FE7, 0xF4);
  boy_Menu();

#ifdef  ENABLE_SOUND
  gsys->PlayAudio(22050, (SOUNDSYS *)audiobuf);
#endif
}

//extern uint32_t RWDATA __prof_result;

void ROCODE load_game() {
  if (gsys->FileOpen(gb->savefile, FILE_MODE_READ) == 0) {
    uint32_t br;
    gsys->FileRead(cart_ram_buffer, 8192, &br);
    gsys->FileRead(gb, sizeof(struct gb_s), &br);
    gsys->FileClose();
    gb->rom_bank0 = rom_buffer;
    gb->display.framebuffer = (uint8_t *)framebuffer;
    __restore_addr_map(gb);
    for (int idx = 0; idx < 32; idx++) {
      uint16_t c = gb->display.bg_palette[idx * 2] | (gb->display.bg_palette[idx * 2 + 1] << 8);
      gb_set_palette(c, idx | LCD_GBC_PAL_BG);
      gb_set_palette(c, idx | LCD_GBC_PAL_BG | LCD_GBC_PAL_BG_PRI);
      gb_set_palette(gb->display.sp_palette[idx * 2] | (gb->display.sp_palette[idx * 2 + 1] << 8), idx | LCD_GBC_PAL_OBJ);
    }
  }
}

void ROCODE save_game() {
  gsys->FileOpen(gb->savefile, FILE_MODE_CREATE_ALWAYS | FILE_MODE_WRITE);
  gsys->FileWrite(cart_ram_buffer, 8192);
  gsys->FileWrite(gb, sizeof(struct gb_s));
  gsys->FileClose();
}

#ifdef OCEMU
void _ResetVector() {
  gsys->Reset(0);
}
#else
extern void _ResetVector();
#endif

uint32_t boy_Step() {
  uint32_t k = gsys->GetKeys();

  if ((k & KEY_START) && k != lastkeys) {
    gb->overlay = !gb->overlay;
    if (gb->overlay) {
      boy_Menu();
      boy_MenuSel();
    }
  }

  gb->direct.joypad = 255;
  if (gb->overlay == 0) {
    if (k & KEY_A)
      gb->direct.joypad_bits.a = 0;
    if (k & KEY_RIGHT)
      gb->direct.joypad_bits.right = 0;
    if (k & KEY_LEFT)
      gb->direct.joypad_bits.left = 0;
    if (k & KEY_UP)
      gb->direct.joypad_bits.up = 0;
    if (k & KEY_DOWN)
      gb->direct.joypad_bits.down = 0;
    if (k & KEY_B)
      gb->direct.joypad_bits.b = 0;
    if (k & KEY_X)
      gb->direct.joypad_bits.start = 0;
    if (k & KEY_Y)
      //printf("%04X %02X %02X\n", gb->cpu_reg.pc, gb_rom_read(gb, gb->cpu_reg.pc), gb_rom_read(gb, gb->cpu_reg.pc + 1));
      gb->direct.joypad_bits.select = 0;
  } else {
    if ((k & KEY_UP) && k != lastkeys) {
      gb->menuidx--;
      if (gb->menuidx > 10)
        gb->menuidx = 4;
      boy_MenuSel();
    }
    if ((k & KEY_DOWN) && k != lastkeys) {
      gb->menuidx++;
      if (gb->menuidx > 4)
        gb->menuidx = 0;
      boy_MenuSel();
    }
    if ((k & KEY_A) && k != lastkeys) {
      /*palidx++;
    if (palidx == NUMBER_OF_PALETTES)
      palidx = 0;
    manual_assign_palette(palidx);*/
      switch (gb->menuidx) {
        case 0: {
            uint32_t v = audio_get_volume() + 1;
            if (v > 16)
              v = 16;
            audio_set_volume(v);
            boy_MenuSel();
          } break;
        case 1:
          load_game();
          gb->overlay = 0;
          break;
        case 2:
          gsys->StopAudio();
          save_game();
          gb->overlay = 0;
#ifdef ENABLE_SOUND
          gsys->PlayAudio(22050, NULL);
#endif
          break;
        case 3:
          gb_reset(gb);
          gb->overlay = 0;
          break;
        case 4:
          gb->fpsmode++;
          if (gb->fpsmode > 2)
            gb->fpsmode = 0;
          boy_MenuSel();
          break;
        case 5:
          gsys->StopAudio();
          memset(0x3FFE8000, 0, 0x18000);
          _ResetVector();
          break;
      }
    }
    if ((lastkeys & KEY_A) && k != lastkeys && gb->menuidx == 5)
      return 0;
    if ((k & KEY_B) && k != lastkeys) {
      switch (gb->menuidx) {
        case 0: {
          uint32_t v = audio_get_volume() - 1;
          if (v > 256)
            v = 0;
          audio_set_volume(v);
          boy_MenuSel();
        } break;
        case 4:
          gb->fpsmode--;
          if (gb->fpsmode > 2)
            gb->fpsmode = 2;
          boy_MenuSel();
          break;
      }
    }
    if ((k & KEY_RIGHT) && k != lastkeys) {
      uint32_t v = audio_get_volume() + 1;
      if (v > 16)
        v = 16;
      audio_set_volume(v);
      boy_MenuSel();
    }
    if ((k & KEY_LEFT) && k != lastkeys) {
      uint32_t v = audio_get_volume() - 1;
      if (v > 256)
        v = 0;
      audio_set_volume(v);
      boy_MenuSel();
    }
  }
  lastkeys = k;

  gsys->SetScreen(0, 127);
  gsys->FlashSwitch(1, 0);
  gb_run_frame(gb);
  rtc_timer += 17;
  if (gb->fpsmode != 0) {
    while (gb->display.frame_skip_count != 1) {
      gb_run_frame(gb);
      rtc_timer += 17;
    }
  }
  gsys->FlashSwitch(0, 0);

  uint32_t new_ticks = gsys->GetMsTicks();
  uint32_t ticks = new_ticks - last_ticks;
  if (new_ticks < last_ticks)
    ticks += 26844;
  uint32_t comp = ticks - 51;
#ifndef OCEMU
//  gsys->Printf("%d %d\n", new_ticks - last_ticks, __prof_result);
#endif
#ifdef  PROFILE
  PROFILE_START
#endif

  if (comp > 0 && comp < 10000) {
    rtc_timer += comp;
  }
  last_ticks = new_ticks;

  //if (ticks < 51)
  //  gsys->SleepMs(51 - ticks);
  while (rtc_timer >= 1000)
  {
    rtc_timer -= 1000;
    gb_tick_rtc(gb);
  }
  return 1;
}
