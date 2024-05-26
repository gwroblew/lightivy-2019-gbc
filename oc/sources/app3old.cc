#include <stdlib.h>
#include <string.h>
#include "oc.h"
#include "display.h"
#include "control.h"
#include "config.h"

#include "gfx3d.h"

extern SYSCALLS * RWDATA gsys;
extern uint16_t * RWDATA framebuffer;

uint32_t RWDATA rx;
uint32_t RWDATA ry;
uint32_t RWDATA rz;
uint32_t RWDATA cx;
uint32_t RWDATA cy;
uint32_t RWDATA cz;
uint32_t RWDATA yshow;

extern int32_t * RWDATA divmap;

extern void scan_draw_init();
extern void buffer_clear(uint16_t *zbuf);

extern void scan_triangle3(int32_t *p0, int32_t *p1, int32_t *p2, uint32_t color);
extern void scan_triangle4(int32_t *p0, int32_t *p1, int32_t *p2, uint32_t color, uint32_t uv1, uint32_t uv2, uint32_t uv3);

#define RAND    (seed = (((seed ^ 0xBACADA55) + 0x9123) >> 11) ^ (seed + 0x7432))

extern uint32_t RWDATA audio_ptr;
extern uint32_t RWDATA audio_size;

extern void fill_audio();

static unsigned isqrt(unsigned long val) {
    unsigned long temp, g=0, b = 0x80, bshft = 7;
    do {
        if (val >= (temp = (((g << 1) + b)<<bshft--))) {
           g += b;
           val -= temp;
        }
    } while (b >>= 1);
    return g;
}

#define ABS(x)  (((x) < 0)?-(x):(x))

void shader(uint32_t rx, uint32_t ry, uint32_t rz, uint16_t *framebuffer) {
  uint16_t *fb = framebuffer;

  int ox = 0;
  int oy = 0;
  int oz = 150;
  rotate_init(rx, ry, rz);
  ROTATE_POINT(ox, oy, oz);
  ox += cx;
  oy += cy;
  oz += cz;
  for (int y = 0; y < 128; y+=2) {
    for (int x = 0; x < 160; x+=2) {
      int dy = y - 64;
      int dx = x - 80;
      int dz = -100;
      int d = 100, t = 16, cnt = 0;
      int dl = isqrt(dx * dx + dy * dy + dz * dz);
      int px, py, pz;
      //dx = (dx << 8) / dl;
      //dy = (dy << 8) / dl;
      //dz = (dz << 8) / dl;
      int32_t dm = divmap[dl];
      dx = (dx * dm) >> 8;
      dy = (dy * dm) >> 8;
      dz = (dz * dm) >> 8;

      ROTATE_POINT(dx, dy, dz);

      while (t < 512 && cnt < 15) {
        px = ox + ((t * dx) >> 8);
        py = oy + ((t * dy) >> 8);
        pz = oz + ((t * dz) >> 8);
        px = ((px + 64) & 127) - 64;
        py = ((py + 64) & 127) - 64;
        pz = ((pz + 64) & 127) - 64;

        // CUBE
        int bx = ABS(px) - 1;
        int by = ABS(py) - 20;
        int bz = ABS(pz) - 30;
        bx = bx < 0 ? 0 : bx;
        by = by < 0 ? 0 : by;
        bz = bz < 0 ? 0 : bz;
        d = isqrt(bx * bx + by * by + bz * bz) - 15;
        //int d2 = isqrt(px * px + py * py + pz * pz) - 35;
        //if (d2 < d)
        //  d = d2;

        // DONUT
        //int bx = isqrt(px * px + pz * pz) - 40;
        //int by = py;
        //d = isqrt(bx * bx + by * by) - 15;

        // CONE
        //int bx = isqrt(px * px + py * py);
        //d = (200 * bx + 150 * pz) >> 8;

        // OCTAHEDRON
        //d = (((ABS(px) + ABS(py) + ABS(pz) - 30) * 147) >> 8) - 10;

        //if (by > bx)
        //  bx = by;
        //if (bz > bx)
        //  bx = bz;
        //d = bx - 7;
        if (d < 3)
          break;

        t += d;
        cnt++;
      }
      /*if (t > 1024) {
        *fb++ = 0xE007;
        continue;
      }
      if (d > 2) {
        *fb++ = 0xE003;
        continue;
      }
      int cx = px * dx;
      int cy = py * dy;
      int cz = pz * dz;
      int c = cx + cy + cz;
      if (c < 0)
        c = -c;*/
      //cnt = c / (5800 / 32);
      //if (cnt > 31)
      //  cnt = 31;
      cnt = 31 - (cnt << 1);
      uint16_t pc = (cnt << 11) | (cnt << 6) | cnt;
      if (d > 2)
        pc &= 0x7E0;
      pc = (pc >> 8) | (pc << 8);
      fb[160] = pc;
      fb[161] = pc;
      *fb++ = pc;
      *fb++ = pc;
    }
    fb += 160;
    if (y & 32)
      fill_audio();
  }
}

void main_loop3() {
  fill_audio();
  volatile uint32_t ticks = gsys->GetMsTicks();

  uint32_t seed = 0x12345678;

  rx += 2; if (rx >= 720) rx -= 720;
  ry += 5; if (ry >= 720) ry -= 720;
  rz += 3; if (rz >= 720) rz -= 720;
  cx += 2;
  cy += 3;
  cz += 5;

  uint32_t k = gsys->GetKeys();

  fill_audio();

  shader(rx, ry, rz, framebuffer);

  gsys->UpdateScreen16(framebuffer, 63 - yshow, 64 + yshow, 0);
  if (yshow < 63) {
    yshow += 4;
    if (yshow > 63)
      yshow = 63;
  }
#ifndef OCEMU
  //gsys->Printf("%d %d %d\n", ticks, ticks2, ticks3);
#endif

#ifdef OCEMU
  gsys->SleepMs(20);
#else
  gsys->SleepMs(1);
#endif
}

void main_start3() {
  rx = 0;
  ry = 0;
  rz = 0;
  cx = 0;
  cy = 0;
  cz = 0;
  yshow = 0;

  scan_draw_init();

  gsys->Printf("Loading music...\n");
  audio_size = load_file("0:/ba1.raw", 0, 0x800000, gsys->GetMemBlock(0), 32768);

  audio_ptr = 0;
  gsys->PlayAudio(22050);
}
