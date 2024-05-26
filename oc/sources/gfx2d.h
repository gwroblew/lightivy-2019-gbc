#ifndef __GFX_H
#define __GFX_H

#include <stdint.h>

typedef struct {
  uint16_t width;
  uint16_t height;
  uint16_t *data;
} BITMAP16;

typedef struct {
  uint16_t width;
  uint16_t height;
  uint8_t tile_width;
  uint8_t tile_height;
  uint8_t *data;

} TILEMAP;

typedef struct {
  BITMAP16 *background;
  TILEMAP *tiles;
  SPRITE *sprites;
} TILEDSCENE16;


#endif
