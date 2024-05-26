#ifndef DISPLAY_DISPLAY_H
#define DISPLAY_DISPLAY_H

#define SCREEN_WIDTH    (160)
#define SCREEN_HEIGHT   (128)

#define SCREEN_PIXELS (SCREEN_WIDTH * SCREEN_HEIGHT)
#define FRAMEBUFFER_SIZE (SCREEN_PIXELS * 2)

struct XY {
  uint8_t x;
  uint8_t y;
};

struct TileFrame {
  int8_t dx;
  int8_t dy;
  uint16_t delay;
};

enum AnimationType {
  AT_Loop = 0,
  AT_ForthBack = 1
};

struct Animation {
  struct XY start_tile;
  uint16_t frame_cnt;
  enum AnimationType type;
  const struct TileFrame * const frames;
};

struct Sprite {
  const char *name;
  struct XY base_tile;
  const struct Animation * const animations;
};

#define ANIMATION_SLOT_MOVE_UP      (0)
#define ANIMATION_SLOT_MOVE_DOWN    (1)
#define ANIMATION_SLOT_MOVE_RIGHT   (2)
#define ANIMATION_SLOT_MOVE_LEFT    (3)

struct Bitmap {
  uint16_t width;
  uint16_t height;
  const uint8_t * const bytes;
  uint16_t trans_color;
  uint16_t tile_width;
  uint16_t tile_height;
  const struct Sprite * const sprites;
};

struct SpriteRuntime {
  uint8_t  bitmap;
  uint8_t  sprite;
  uint16_t x;
  uint16_t y;
  uint16_t last_anim;
  uint16_t last_frame;
  uint16_t last_dir;
};

void display_clear();
void display_show_frame();
void display_init();

void display_draw_line(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint16_t color);
void display_draw_hline(uint32_t x1, uint32_t y1, uint32_t x2, uint16_t color);
void display_draw_vline(uint32_t x1, uint32_t y1, uint32_t y2, uint16_t color);
void display_draw_rectangle(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint16_t color);
void display_draw_box(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint16_t color);
void display_draw_circle(uint32_t x, uint32_t y, uint32_t radius, uint16_t color, uint8_t octant_mask);
void display_draw_filled_circle(uint32_t x, uint32_t y, uint32_t radius, uint16_t color, uint8_t quadrant_mask);
void display_draw_text(uint32_t x, uint32_t y, unsigned char *str);
void display_draw_aligned(uint32_t y, uint8_t *str);

#endif
