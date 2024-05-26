#include <string.h>

#include "system.h"
#include "display.h"

#include "mcufont.h"

uint16_t * RWDATA framebuffer;

extern SYSCALLS *gsys;

static const struct mf_font_s * RWDATA font;

void display_clear() {
  memset(framebuffer, 0, FRAMEBUFFER_SIZE);
}

void display_show_frame() {
  gsys->UpdateScreen16(framebuffer, 0, 127, 0);
}

void display_init(uint16_t *fb) {
  //framebuffer = fb;
  font = NULL;
  display_clear();
  display_show_frame();
}

static void inline display_set_pixel(uint32_t x, uint32_t y, uint16_t c) {
  framebuffer[y * SCREEN_WIDTH + x] = c;
}

void display_draw_line(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint16_t color) {
	uint32_t i;
	uint32_t x;
	uint32_t y;
	int32_t  xinc;
	int32_t  yinc;
	int32_t  dx;
	int32_t  dy;
	int32_t  e;

	/*if (x1 > x2) {
		dx = x1;
		x1 = x2;
		x2 = dx;
		dy = y1;
		y1 = y2;
		y2 = dy;
	}*/

	dx = x2 - x1;
	dy = y2 - y1;

	x = x1;
	y = y1;

	if (dx < 0) {
		xinc = -1;
		dx   = -dx;
	} else {
		xinc = 1;
	}

	if (dy < 0) {
		yinc = -1;
		dy   = -dy;
	} else {
		yinc = 1;
	}

	if (dx > dy) {
		e = dy - dx;
    uint32_t idx = y * SCREEN_WIDTH + x;
    int32_t iyinc = yinc * SCREEN_WIDTH;
		for (i = 0; i <= dx; i++) {
			framebuffer[idx] = color;
			if (e >= 0) {
				e -= dx;
        //y += yinc;
        idx += iyinc;
			}

			e += dy;
      //x += xinc;
      idx += xinc;
		}
	} else {
		e = dx - dy;
    uint32_t idx = y * SCREEN_WIDTH + x;
    int32_t iyinc = yinc * SCREEN_WIDTH;
		for (i = 0; i <= dy; i++) {
			framebuffer[idx] = color;
			if (e >= 0) {
				e -= dy;
				//x += xinc;
        idx += xinc;
			}

			e += dx;
			//y += yinc;
      idx += iyinc;
		}
	}
}

void display_draw_hline(uint32_t x1, uint32_t y1, uint32_t x2, uint16_t color) {
  uint32_t idx = x1 + y1 * SCREEN_WIDTH;
  for (int i = x1; i <= x2; i++, idx++)
    framebuffer[idx] = color;
}

void display_draw_vline(uint32_t x1, uint32_t y1, uint32_t y2, uint16_t color) {
  uint32_t idx = x1 + y1 * SCREEN_WIDTH;
  for (int i = y1; i <= y2; i++, idx += SCREEN_WIDTH)
    framebuffer[idx] = color;
}

void display_draw_rectangle(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint16_t color) {
  display_draw_hline(x1, y1, x2, color);
  display_draw_hline(x1, y2, x2, color);
  display_draw_vline(x1, y1, y2, color);
  display_draw_vline(x2, y1, y2, color);
}

void display_draw_box(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint16_t color) {
  for (int i = y1; i <= y2; i++)
    display_draw_hline(x1, i, x2, color);
}

void display_draw_circle(uint32_t x, uint32_t y, uint32_t radius, uint16_t color, uint8_t octant_mask) {
	uint32_t offset_x;
	uint32_t offset_y;
	int32_t     error;

	if (radius == 0) {
		display_set_pixel(x, y, color);
		return;
	}

	offset_x = 0;
	offset_y = radius;
	error    = 3 - 2 * radius;

	while (offset_x <= offset_y) {
		if (octant_mask & (1 << 0)) {
			display_set_pixel(x + offset_y, y - offset_x, color);
		}

		if (octant_mask & (1 << 1)) {
			display_set_pixel(x + offset_x, y - offset_y, color);
		}

		if (octant_mask & (1 << 2)) {
			display_set_pixel(x - offset_x, y - offset_y, color);
		}

		if (octant_mask & (1 << 3)) {
			display_set_pixel(x - offset_y, y - offset_x, color);
		}

		if (octant_mask & (1 << 4)) {
			display_set_pixel(x - offset_y, y + offset_x, color);
		}

		if (octant_mask & (1 << 5)) {
			display_set_pixel(x - offset_x, y + offset_y, color);
		}

		if (octant_mask & (1 << 6)) {
			display_set_pixel(x + offset_x, y + offset_y, color);
		}

		if (octant_mask & (1 << 7)) {
			display_set_pixel(x + offset_y, y + offset_x, color);
		}

		if (error < 0) {
			error += ((offset_x << 2) + 6);
		} else {
			error += (((offset_x - offset_y) << 2) + 10);
			--offset_y;
		}

		++offset_x;
	}
}

void display_draw_filled_circle(uint32_t x, uint32_t y, uint32_t radius, uint16_t color, uint8_t quadrant_mask)
{
	uint32_t offset_x;
	uint32_t offset_y;
	int32_t     error;

	if (radius == 0) {
		display_set_pixel(x, y, color);
		return;
	}

	offset_x = 0;
	offset_y = radius;
	error    = 3 - 2 * radius;

	while (offset_x <= offset_y) {
		if (quadrant_mask & (1 << 0)) {
			display_draw_vline(x + offset_y, y - offset_x, y, color);
			display_draw_vline(x + offset_x, y - offset_y, y, color);
		}

		if (quadrant_mask & (1 << 1)) {
			display_draw_vline(x - offset_y, y - offset_x, y, color);
			display_draw_vline(x - offset_x, y - offset_y, y, color);
		}

		if (quadrant_mask & (1 << 2)) {
			display_draw_vline(x - offset_y, y, y + offset_x, color);
			display_draw_vline(x - offset_x, y, y + offset_y, color);
		}

		if (quadrant_mask & (1 << 3)) {
			display_draw_vline(x + offset_y, y, y + offset_x, color);
			display_draw_vline(x + offset_x, y, y + offset_y, color);
		}

		if (error < 0) {
			error += ((offset_x << 2) + 6);
		} else {
			error += (((offset_x - offset_y) << 2) + 10);
			--offset_y;
		}

		++offset_x;
	}
}

/*void display_put_bitmap(uint32_t x, uint32_t y, struct Bitmap *bitmap) {
  uint16_t *dst = framebuffer + y * SCREEN_WIDTH + x;
  uint16_t *src = bitmap->data;
  for (int i = 0; i < bitmap->height; i++) {
    memcpy(dst, src, bitmap->width * sizeof(uint16_t));
    src += bitmap->width;
    dst += SCREEN_WIDTH - bitmap->width;
  }
}

void display_put_bitmap_mask(uint32_t x, uint32_t y, struct Bitmap *bitmap, uint16_t mask) {
  uint16_t *dst = framebuffer + y * SCREEN_WIDTH + x;
  uint16_t *src = bitmap->data;
  for (int i = 0; i < bitmap->height; i++) {
    for (int j = 0; j < bitmap->width; j++) {
      uint16_t p = *src++;
      if (p == mask) {
        dst++;
        continue;
      }
      *dst++ = p;
    }
    src += bitmap->width;
    dst += SCREEN_WIDTH - bitmap->width;
  }
}*/

static void pixel_callback(int16_t x, int16_t y, uint8_t count, uint8_t alpha, void *state)
{
  uint32_t pos;
  //uint16_t value;
  
  if (y < 0 || y >= SCREEN_HEIGHT) return;
  if (x < 0 || x + count >= SCREEN_WIDTH) return;
  
  while (count--)
  {
    pos = (uint32_t)SCREEN_WIDTH * y + x;
    //value = framebuffer[pos];
    //value -= alpha;
    //if (value < 0) value = 0;
    uint16_t a = alpha;
    uint16_t c = ((a << 8) & 0xF800) | ((a << 3) & 0x07E0) | (a >> 3);
    c = (c >> 8) | (c << 8);
    framebuffer[pos] = alpha != 0 ? c : 0;
    x++;
  }
}

/* Callback to render characters. */
static uint8_t character_callback(int16_t x, int16_t y, mf_char character, void *state)
{
  return mf_render_character(font, x, y, character, pixel_callback, state);
}

const char default_font[] RODATA = "DejaVuSans12";

void display_draw_text(uint32_t x, uint32_t y, uint8_t *str) {
  if (font == NULL) {
    font = mf_find_font(default_font);
  }
  mf_render_aligned(font, x, y,
                    MF_ALIGN_LEFT, (char *)str, strlen((char *)str),
                    character_callback, NULL);
}

void display_draw_aligned(uint32_t y, uint8_t *str) {
  if (font == NULL) {
    font = mf_find_font(default_font);
  }
  mf_render_aligned(font, SCREEN_WIDTH / 2, y,
                    MF_ALIGN_CENTER, (char *)str, strlen((char *)str),
                    character_callback, NULL);
}
