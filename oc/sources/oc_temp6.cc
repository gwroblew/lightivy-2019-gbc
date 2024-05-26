#include <stdlib.h>
#include <string.h>
#include "oc.h"
#include "display.h"
#include "control.h"
#include "config.h"

#include "gfx3d.h"
//#include "test.h"
//#include "wolf.h"
#include "test2.h"

#define TC(x, y)  ((x << 24) | (y << 8))

const OBJECT3D RODATA testobject = {
  .point_cnt = 8,
  .face_cnt = 8
};
const POINT3D testpoints[] RODATA = {
  { -1024, -1024, -1024 },
  { -1024, -1024, 1024 },
  { -1024, 1024, 1024 },
  { -1024, 1024, -1024 },
  { 1024, -1024, -1024 },
  { 1024, -1024, 1024 },
  { 1024, 1024, 1024 },
  { 1024, 1024, -1024 }
};
const FACE3DT testfaces[] RODATA = {
  { 0, 1, 2, 3, TC(0, 0), TC(0, 63), TC(63, 63) },
  { 2, 3, 0, 3, TC(63, 63), TC(63, 0), TC(0, 0) },
  { 6, 5, 4, 4, TC(0, 0), TC(0, 63), TC(63, 63) },
  { 4, 7, 6, 4, TC(63, 63), TC(63, 0), TC(0, 0) },
  { 0, 5, 1, 5, TC(0, 0), TC(0, 63), TC(63, 63) },
  { 4, 5, 0, 5, TC(63, 63), TC(0, 63), TC(0, 0) },
  { 2, 7, 3, 6, TC(0, 0), TC(0, 63), TC(63, 63) },
  { 6, 7, 2, 6, TC(63, 63), TC(0, 63), TC(0, 0) }
};

SYSCALLS * RWDATA gsys;
extern uint16_t * RWDATA framebuffer;
uint16_t RWDATA keyCounter[8];
uint32_t RWDATA lastKeyTick;
uint16_t RWDATA keyDelay;
uint16_t RWDATA keyRepeat;
OCDEFER RWDATA deffuncs[MAX_DEFERRED_FUNCS];

uint32_t RWDATA rx;
uint32_t RWDATA ry;
uint32_t RWDATA rz;

void init_defer_funcs() {
  for (int i = 0; i < MAX_DEFERRED_FUNCS; i++)
    deffuncs[i].func = NULL;
}

void add_defer_func(uint32_t start_tick, uint32_t interval, uint32_t (*func)(OCDEFER *def), void *arg) {
  int i;
  for (i = 0; i < MAX_DEFERRED_FUNCS; i++)
    if (deffuncs[i].func == NULL)
      break;
  if (i == MAX_DEFERRED_FUNCS) {
    DEBUG("Too many deferred functions! New: 0x%08X\n", func);
    return;
  }
  deffuncs[i].start_tick = start_tick + gsys->GetMsTicks();
  deffuncs[i].step_ticks = interval;
  deffuncs[i].func = func;
  deffuncs[i].arg = arg;
}

void remove_defer_func(uint32_t (*func)(OCDEFER *def), void *arg) {
  int i;
  for (i = 0; i < MAX_DEFERRED_FUNCS; i++)
    if (deffuncs[i].func == func && deffuncs[i].arg == arg)
      break;
  if (i == MAX_DEFERRED_FUNCS)
    return;
  deffuncs[i].func = NULL;
}

void call_defer_funcs(uint32_t current_tick) {
  for (int i = 0; i < MAX_DEFERRED_FUNCS; i++) {
    if (deffuncs[i].func == NULL)
      continue;
    if (current_tick < deffuncs[i].start_tick)
      continue;
    uint32_t r = deffuncs[i].func(&deffuncs[i]);
    if (deffuncs[i].step_ticks == 0 || r == DEFER_RESULT_DONE) {
      deffuncs[i].func = NULL;
      continue;
    }
    deffuncs[i].start_tick += deffuncs[i].step_ticks;
  }
}

uint32_t timer_1s(OCDEFER *def) {

  if (gsys->WifiStatus() == WIFI_STATUS_GOT_IP) {
    gsys->WifiSendUDP("test2\n", 6);
    //TIME dt;
    //gsys->GetTime(&dt);
    //gsys->Printf("%d %d %d %d %d %d\n", dt.tm_year, dt.tm_mon+1, dt.tm_mday, dt.tm_hour, dt.tm_min, dt.tm_sec);
  }
  return DEFER_RESULT_OK;
}

/*
insert
draw with z-buffer @ screen update

find idx for z > zprev && z < znext
insert
draw back to front

find idx for x0 > x0prev && x0 < x0next
insert
draw while tracking x and z on stack @ screen update

next:
if x0pop < x0next
  pop x0 x1
else
  x0 = x0next
  x1 = x1next
  z = znext
loop:
if x1 > x0next
  if z > znext
    draw x0 x0next-1
    if x1 > x1next
      push x1next+1 x1
    goto next
  if x1 < x1next
    push x1+1 x1next
  skip next
  goto loop
draw x0 x1
goto next
*/

int32_t * RWDATA divmap;

void scan_draw_init() {
  int32_t *dst = (int32_t *)gsys->GetMemBlock(1) + (10 * 1024 + 512) / 4;
  divmap = dst;

  *dst++ = 65536;
  for (int i = 1; i < 384; i++) {
    *dst++ = 65536 / i;
  }
}

void buffer_clear(uint16_t *zbuf) {
  uint32_t *dst = (uint32_t *)zbuf;
  for (int i = 0; i < 5120; i++)
    *dst++ = 0x7FFF7FFF;
}

inline void scan_xLine3(int x0, int x1, int z1, int z2, uint32_t color, uint8_t *buf, uint16_t *zbuf) {
  if (x0 > SCREEN_WIDTH - 1)
    return;
  if (x1 < 0)
    return;
  if(x0 < 0) {
    int dx = x1 - x0;
    if (dx > 1)
      z1 += (z1 - z2) * x0 / dx;
    x0 = 0;
  }
  if(x1 > SCREEN_WIDTH - 1) {
    int dx = x1 - x0;
    if (dx > 1)
      z2 -= (z1 - z2) * (SCREEN_WIDTH - x1) / dx;
    x1 = SCREEN_WIDTH - 1;
  }

  int dx = x1 - x0 + 1;
  int dz = ((z2 - z1) * divmap[dx]) >> 7;
  z1 <<= 8;
  uint8_t *dst = buf + x0;
  uint16_t *zb = zbuf + (x0 >> 1);

  if (x0 & 1) {
    if (*zb > (z1 >> 8)) {
      *dst = color;
    }
    x0++;
    dst++;
    zb++;
    z1 += (dz >> 1);
  }
  while (x0 <= x1) {
    if (*zb > (z1 >> 8)) {
      *dst = color;
      if (x0 == x1)
        break;
      dst[1] = color;
      *zb = z1 >> 8;
    }
    x0 += 2;
    dst += 2;
    zb++;
    z1 += dz;
  }
}

#define STA   (8)
#define SWAP(p1, p2)  { int tx = p1##x, ty = p1##y, tz = p1##z; p1##x = p2##x; p1##y = p2##y; p1##z = p2##z; p2##x = tx; p2##y = ty; p2##z = tz; }

void scan_triangle3(int32_t *p0, int32_t *p1, int32_t *p2, uint32_t color) {
  int p0x = p0[0];
  int p0y = p0x >> 16;
  int p0z = p0[1];
  int p1x = p1[0];
  int p1y = p1x >> 16;
  int p1z = p1[1];
  int p2x = p2[0];
  int p2y = p2x >> 16;
  int p2z = p2[1];

  if (p1y < p0y)
    SWAP(p0, p1)
  if (p2y < p1y)
    SWAP(p1, p2)
  if (p1y < p0y)
    SWAP(p0, p1)
  if (p0y > SCREEN_HEIGHT - 1)
    return;
  if (p2y < 0)
    return;

  p0x = (int16_t)p0x;
  p1x = (int16_t)p1x;
  p2x = (int16_t)p2x;

  int y = p0y;
  int xac = p0x << STA;
  int xab = p0x << STA;
  int xbc = p1x << STA;
  int xaci = 0;
  int xabi = 0;
  int xbci = 0;
  int zac = p0z << STA;
  int zab = p0z << STA;
  int zbc = p1z << STA;
  int zaci = 0;
  int zabi = 0;
  int zbci = 0;
  if(p1y != p0y) {
    int dy = p1y - p0y;
    xabi = ((p1x - p0x) * divmap[dy]) >> (16 - STA);
    zabi = ((p1z - p0z) * divmap[dy]) >> (16 - STA);
  }
  if(p2y != p0y) {
    int dy = p2y - p0y;
    xaci = ((p2x - p0x) * divmap[dy]) >> (16 - STA);
    zaci = ((p2z - p0z) * divmap[dy]) >> (16 - STA);
  }
  if(p2y != p1y) {
    int dy = p2y - p1y;
    xbci = ((p2x - p1x) * divmap[dy]) >> (16 - STA);
    zbci = ((p2z - p1z) * divmap[dy]) >> (16 - STA);
  }

  uint8_t *buf = (uint8_t *)framebuffer;
  uint8_t *dst = buf + y * SCREEN_WIDTH;
  uint16_t *zbuf = (uint16_t *)(framebuffer + 10240);
  uint16_t *zdst = zbuf + y * (SCREEN_WIDTH / 2);

  int y1 = p1y;
  int y2 = p2y;
  int ye = y1;
  if (ye >= SCREEN_HEIGHT)
    ye = SCREEN_HEIGHT - 1;
  if (ye < 0) {
    int yc = ye - y;
    xac += xaci * yc;
    zac += zaci * yc;
    dst = buf;
    zdst = zbuf;
    y = ye;
  } else {
    if (y < 0) {
      xab -= xabi * y;
      xac -= xaci * y;
      zab -= zabi * y;
      zac -= zaci * y;
      dst = buf;
      zdst = zbuf;
      y = 0;
    }

    if (xabi < xaci) {
      for(; y < ye; y++, dst += SCREEN_WIDTH, zdst += SCREEN_WIDTH / 2)
      {
        scan_xLine3(xab >> STA, xac >> STA, zab >> STA, zac >> STA, color, dst, zdst);
        xab += xabi;
        xac += xaci;
        zab += zabi;
        zac += zaci;
      }
    } else {
      for(; y < ye; y++, dst += SCREEN_WIDTH, zdst += SCREEN_WIDTH / 2)
      {
        scan_xLine3(xac >> STA, xab >> STA, zac >> STA, zab >> STA, color, dst, zdst);
        xab += xabi;
        xac += xaci;
        zab += zabi;
        zac += zaci;
      }
    }
  }
  ye = y2;
  if (ye >= SCREEN_HEIGHT)
    ye = SCREEN_HEIGHT - 1;
  if (y < 0) {
    xbc -= xbci * y;
    xac -= xaci * y;
    zbc -= zbci * y;
    zac -= zaci * y;
    dst = buf;
    zdst = zbuf;
    y = 0;
  }

  if (xbc < xac) {
    for(; y < ye; y++, dst += SCREEN_WIDTH, zdst += SCREEN_WIDTH / 2)
    {
      scan_xLine3(xbc >> STA, xac >> STA, zbc >> STA, zac >> STA, color, dst, zdst);
      xbc += xbci;
      xac += xaci;
      zbc += zbci;
      zac += zaci;
    }
  } else {
    for(; y < ye; y++, dst += SCREEN_WIDTH, zdst += SCREEN_WIDTH / 2)
    {
      scan_xLine3(xac >> STA, xbc >> STA, zac >> STA, zbc >> STA, color, dst, zdst);
      xbc += xbci;
      xac += xaci;
      zbc += zbci;
      zac += zaci;
    }
  }
}

void scan_triangle4(int32_t *p0, int32_t *p1, int32_t *p2, uint32_t color, uint32_t uv1, uint32_t uv2, uint32_t uv3) {
  int p0x = p0[0];
  int p0y = p0x >> 16;
  int p0z = p0[1];
  int p1x = p1[0];
  int p1y = p1x >> 16;
  int p1z = p1[1];
  int p2x = p2[0];
  int p2y = p2x >> 16;
  int p2z = p2[1];

  if (p1y < p0y)
    SWAP(p0, p1)
  if (p2y < p1y)
    SWAP(p1, p2)
  if (p1y < p0y)
    SWAP(p0, p1)
  if (p0y > SCREEN_HEIGHT - 1)
    return;
  if (p2y < 0)
    return;

  p0x = (int16_t)p0x;
  p1x = (int16_t)p1x;
  p2x = (int16_t)p2x;

  int y = p0y;
  int xac = p0x << STA;
  int xab = p0x << STA;
  int xbc = p1x << STA;
  int xaci = 0;
  int xabi = 0;
  int xbci = 0;
  int zac = p0z << STA;
  int zab = p0z << STA;
  int zbc = p1z << STA;
  int zaci = 0;
  int zabi = 0;
  int zbci = 0;
  if(p1y != p0y) {
    int dy = p1y - p0y;
    xabi = ((p1x - p0x) * divmap[dy]) >> (16 - STA);
    zabi = ((p1z - p0z) * divmap[dy]) >> (16 - STA);
  }
  if(p2y != p0y) {
    int dy = p2y - p0y;
    xaci = ((p2x - p0x) * divmap[dy]) >> (16 - STA);
    zaci = ((p2z - p0z) * divmap[dy]) >> (16 - STA);
  }
  if(p2y != p1y) {
    int dy = p2y - p1y;
    xbci = ((p2x - p1x) * divmap[dy]) >> (16 - STA);
    zbci = ((p2z - p1z) * divmap[dy]) >> (16 - STA);
  }

  uint8_t *buf = (uint8_t *)framebuffer;
  uint8_t *dst = buf + y * SCREEN_WIDTH;
  uint16_t *zbuf = (uint16_t *)(framebuffer + 10240);
  uint16_t *zdst = zbuf + y * (SCREEN_WIDTH / 2);

  int y1 = p1y;
  int y2 = p2y;
  int ye = y1;
  if (ye >= SCREEN_HEIGHT)
    ye = SCREEN_HEIGHT - 1;
  if (ye < 0) {
    int yc = ye - y;
    xac += xaci * yc;
    zac += zaci * yc;
    dst = buf;
    zdst = zbuf;
    y = ye;
  } else {
    if (y < 0) {
      xab -= xabi * y;
      xac -= xaci * y;
      zab -= zabi * y;
      zac -= zaci * y;
      dst = buf;
      zdst = zbuf;
      y = 0;
    }

    if (xabi < xaci) {
      for(; y < ye; y++, dst += SCREEN_WIDTH, zdst += SCREEN_WIDTH / 2)
      {
        scan_xLine3(xab >> STA, xac >> STA, zab >> STA, zac >> STA, color, dst, zdst);
        xab += xabi;
        xac += xaci;
        zab += zabi;
        zac += zaci;
      }
    } else {
      for(; y < ye; y++, dst += SCREEN_WIDTH, zdst += SCREEN_WIDTH / 2)
      {
        scan_xLine3(xac >> STA, xab >> STA, zac >> STA, zab >> STA, color, dst, zdst);
        xab += xabi;
        xac += xaci;
        zab += zabi;
        zac += zaci;
      }
    }
  }
  ye = y2;
  if (ye >= SCREEN_HEIGHT)
    ye = SCREEN_HEIGHT - 1;
  if (y < 0) {
    xbc -= xbci * y;
    xac -= xaci * y;
    zbc -= zbci * y;
    zac -= zaci * y;
    dst = buf;
    zdst = zbuf;
    y = 0;
  }

  if (xbc < xac) {
    for(; y < ye; y++, dst += SCREEN_WIDTH, zdst += SCREEN_WIDTH / 2)
    {
      scan_xLine3(xbc >> STA, xac >> STA, zbc >> STA, zac >> STA, color, dst, zdst);
      xbc += xbci;
      xac += xaci;
      zbc += zbci;
      zac += zaci;
    }
  } else {
    for(; y < ye; y++, dst += SCREEN_WIDTH, zdst += SCREEN_WIDTH / 2)
    {
      scan_xLine3(xac >> STA, xbc >> STA, zac >> STA, zbc >> STA, color, dst, zdst);
      xbc += xbci;
      xac += xaci;
      zbc += zbci;
      zac += zaci;
    }
  }
}

#define USE_FLASH   1

#define RAND    (seed = (((seed ^ 0xBACADA55) + 0x9123) >> 11) ^ (seed + 0x7432))

uint32_t RWDATA mx;
uint32_t RWDATA my;
int32_t RWDATA height;
int32_t RWDATA horizon;
int32_t RWDATA angle;
int32_t RWDATA roll;

int32_t RWDATA testh;

typedef struct {
  uint16_t start;
  uint16_t cstart;
  uint16_t hend;
} VOXELRAY;

typedef struct {
  int8_t x;
  int8_t y;
  uint16_t h;
} VOXELCOL;

typedef struct {
  int16_t off;
  uint16_t h;
} VOXELCOL2;

uint32_t gen_ray(VOXELRAY *ray, int32_t sx, int32_t sy, VOXELCOL *cols, uint32_t cc) {
  int i;
  uint8_t *fb = (uint8_t *)framebuffer;
  for (i = 0; i < cc; i++) {
    if (sx >= 0 && sx <= 159 && sy >= 0 && sy <= 127) {
      ray->hend = cols[i - 1].h;
      break;
    }
    sx += cols[i].x;
    sy += cols[i].y;
  }
  if (i == cc)
    return 0;
  for (;i < cc; i++) {
    if (!(sx >= 0 && sx <= 159 && sy >= 0 && sy <= 127))
      break;
    //if (i < cc / 2)
    //  *(fb + sy * 160 + sx) = 0xFF;
    sx += cols[i].x;
    sy += cols[i].y;
  }
  i--;
  sx -= cols[i].x;
  sy -= cols[i].y;
  ray->start = sy * 160 + sx;
  ray->cstart = i - 1;

  return 1;
}

typedef struct {
  uint16_t cc;
  uint16_t rc;
} VOXELRAYS;

void precalc_voxel(uint32_t angle, uint8_t *buf, VOXELRAYS *vr) {
  VOXELCOL *cols = buf;
  VOXELCOL2 cols2[300];
  VOXELRAY rays[200];
  int16_t ha[200];
  int16_t hi[300];
  int32_t a = angle - 80;
  if (a < 0)
    a += 720;

  int16_t sinr = sintable[roll];
  int16_t cosr = sintable[roll + 180];
  int32_t botx = (208 * sinr) >> 16;
  int32_t boty = (208 * cosr) >> 16;
  int32_t topx = (-208 * sinr) >> 16;
  int32_t topy = (-208 * cosr) >> 16;

  int32_t vdx = botx - topx;
  int32_t vdy = boty - topy;
  int32_t xinc = 1;
  int32_t yinc = 1;
  int32_t e;
  int32_t vx = botx, vy = boty;
  uint32_t cc = 0;

	if (vdx < 0) {
		xinc = -1;
		vdx   = -vdx;
	}
	if (vdy < 0) {
		yinc = -1;
		vdy   = -vdy;
	}

	if (vdx > vdy) {
		e = vdy - vdx;
		for (int i = 0; i <= vdx; i++) {
      cols[cc].y = 0;
			if (e >= 0) {
				e -= vdx;
        vy += yinc;
        cols[cc].x = 0;
        cols[cc].h = (i * 209) / vdx;
        cols[cc++].y = yinc;
        cols[cc].y = 0;
			}
			e += vdy;
      vx += xinc;
      cols[cc].h = (i * 209) / vdx;
      cols[cc++].x = xinc;
		}
	} else {
		e = vdx - vdy;
		for (int i = 0; i <= vdy; i++) {
      cols[cc].x = 0;
			if (e >= 0) {
				e -= vdy;
				vx += xinc;
        cols[cc].x = xinc;
        cols[cc].h = (i * 209) / vdy;
        cols[cc++].y = 0;
        cols[cc].x = 0;
			}
			e += vdx;
			vy += yinc;
      cols[cc].h = (i * 209) / vdy;
      cols[cc++].y = yinc;
		}
	}

  int32_t leftx = (-120 * cosr - 104 * sinr) >> 15;
  int32_t lefty = (-104 * cosr + 120 * sinr) >> 15;
  int32_t rightx = (120 * cosr - 104 * sinr) >> 15;
  int32_t righty = (-104 * cosr - 120 * sinr) >> 15;

  int32_t dx = rightx - leftx;
  int32_t dy = righty - lefty;
  int32_t hx = leftx, hy = lefty;
  int32_t rc = 0;

  xinc = yinc = 1;

	if (dx < 0) {
		xinc = -1;
		dx   = -dx;
	}
	if (dy < 0) {
		yinc = -1;
		dy   = -dy;
	}

  int sk = 0;
  int32_t zd = dx;
  int32_t rx0 = 0, rx1 = 0;
	if (dx > dy) {
		e = dy - dx;
		for (int i = 0; i <= dx; i++) {
      uint32_t dc = gen_ray(&rays[rc], hx + 80, hy + 64, cols, cc);
      if (rx0 == 0 && dc != 0)
        rx0 = (((i - dx / 2) * 241) << 6) / dx;
      if (dc != 0)
        rx1 = (((i - dx / 2) * 241) << 6) / dx;
      rc += dc;
			if (e >= 0) {
				e -= dx;
        hy += yinc;
			}
			e += dy;
      hx += xinc;
		}
	} else {
    zd = dy;
		e = dx - dy;
		for (int i = 0; i <= dy; i++) {
      uint32_t dc = gen_ray(&rays[rc], hx + 80, hy + 64, cols, cc);
      if (rx0 == 0 && dc != 0)
        rx0 = (((i - dy / 2) * 241) << 6) / dy;
      if (dc != 0)
        rx1 = (((i - dy / 2) * 241) << 6) / dy;
      rc += dc;
			if (e >= 0) {
				e -= dy;
				hx += xinc;
			}
			e += dx;
			hy += yinc;
		}
	}

  for (int i = 0; i < rc; i++) {
    ha[i] = cols[rays[i].cstart].h;
  }

  for (int i = 0, j = 0; i < cc; i++) {
    cols2[i].off = cols[i].y * 160 + cols[i].x;
    cols2[i].h = cols[i].h;
    while (j < cols[i].h)
      hi[j++] = i;
  }

}

void render_voxel() {
  //VOXELRAY *rays = gsys->GetMemBlock(1);
  VOXELCOL cols[320];
  VOXELCOL2 cols2[320];
  VOXELRAY rays[200];
  int16_t ha[200];
  int16_t hi[320];
  int32_t a = angle - 80;
  if (a < 0)
    a += 720;

  int16_t sinr = sintable[roll];
  int16_t cosr = sintable[roll + 180];
  int32_t botx = (208 * sinr) >> 16;
  int32_t boty = (208 * cosr) >> 16;
  int32_t topx = (-208 * sinr) >> 16;
  int32_t topy = (-208 * cosr) >> 16;

  int32_t vdx = botx - topx;
  int32_t vdy = boty - topy;
  int32_t xinc = 1;
  int32_t yinc = 1;
  int32_t e;
  int32_t vx = botx, vy = boty;
  uint32_t cc = 0;

	if (vdx < 0) {
		xinc = -1;
		vdx   = -vdx;
	}
	if (vdy < 0) {
		yinc = -1;
		vdy   = -vdy;
	}

	if (vdx > vdy) {
		e = vdy - vdx;
		for (int i = 0; i <= vdx; i++) {
      cols[cc].y = 0;
			if (e >= 0) {
				e -= vdx;
        vy += yinc;
        cols[cc].x = 0;
        cols[cc].h = (i * 209) / vdx;
        cols[cc++].y = yinc;
        cols[cc].y = 0;
			}
			e += vdy;
      vx += xinc;
      cols[cc].h = (i * 209) / vdx;
      cols[cc++].x = xinc;
		}
	} else {
		e = vdx - vdy;
		for (int i = 0; i <= vdy; i++) {
      cols[cc].x = 0;
			if (e >= 0) {
				e -= vdy;
				vx += xinc;
        cols[cc].x = xinc;
        cols[cc].h = (i * 209) / vdy;
        cols[cc++].y = 0;
        cols[cc].x = 0;
			}
			e += vdx;
			vy += yinc;
      cols[cc].h = (i * 209) / vdy;
      cols[cc++].y = yinc;
		}
	}

  int32_t leftx = (-120 * cosr - 104 * sinr) >> 15;
  int32_t lefty = (-104 * cosr + 120 * sinr) >> 15;
  int32_t rightx = (120 * cosr - 104 * sinr) >> 15;
  int32_t righty = (-104 * cosr - 120 * sinr) >> 15;

  int32_t dx = rightx - leftx;
  int32_t dy = righty - lefty;
  int32_t hx = leftx, hy = lefty;
  int32_t rc = 0;

  xinc = yinc = 1;

	if (dx < 0) {
		xinc = -1;
		dx   = -dx;
	}
	if (dy < 0) {
		yinc = -1;
		dy   = -dy;
	}

  int sk = 0;
  int32_t zd = dx;
  int32_t rx0 = 0, rx1 = 0;
	if (dx > dy) {
		e = dy - dx;
		for (int i = 0; i <= dx; i++) {
      uint32_t dc = gen_ray(&rays[rc], hx + 80, hy + 64, cols, cc);
      if (rx0 == 0 && dc != 0)
        rx0 = (((i - dx / 2) * 241) << 6) / dx;
      if (dc != 0)
        rx1 = (((i - dx / 2) * 241) << 6) / dx;
      rc += dc;
			if (e >= 0) {
				e -= dx;
        hy += yinc;
			}
			e += dy;
      hx += xinc;
		}
	} else {
    zd = dy;
		e = dx - dy;
		for (int i = 0; i <= dy; i++) {
      uint32_t dc = gen_ray(&rays[rc], hx + 80, hy + 64, cols, cc);
      if (rx0 == 0 && dc != 0)
        rx0 = (((i - dy / 2) * 241) << 6) / dy;
      if (dc != 0)
        rx1 = (((i - dy / 2) * 241) << 6) / dy;
      rc += dc;
			if (e >= 0) {
				e -= dy;
				hx += xinc;
			}
			e += dx;
			hy += yinc;
		}
	}

  for (int i = 0; i < rc; i++) {
    ha[i] = cols[rays[i].cstart].h;
  }

  for (int i = 0, j = 0; i < cc; i++) {
    cols2[i].off = cols[i].y * 160 + cols[i].x;
    cols2[i].h = cols[i].h;
    while (j <= cols[i].h)
      hi[j++] = i;
  }
  //printf("%d %d\n", cc, rc);

  int16_t sina = sintable[a];
  int16_t cosa = sintable[a + 180];
  uint8_t *fb = (uint8_t *)framebuffer;
  uint16_t *zb = framebuffer + 10 * 1024;

  int32_t zz = 240 << 6;
  int32_t zz1 = -(rx0 * 240) / 80;
  int32_t zz2 = (rx1 * 240) / 80;
  int32_t lx = (-zz1 * cosa - zz * sina) >> 6;
  int32_t ly = (-zz * cosa + zz1 * sina) >> 6;
  int32_t rx = (zz2 * cosa - zz * sina) >> 6;
  int32_t ry = (-zz * cosa - zz2 * sina) >> 6;
  dx = (rx - lx) / rc;
  dy = (ry - ly) / rc;
  int32_t cx = (mx << 15) + 0x7FF8000;
  int32_t cy = (my << 15) + 0x7FF8000;

  lx += cx;
  ly += cy;
  for (int x = 0; x < rc; x++, lx += dx, ly += dy) {
    int32_t z = 0, dz;
    int32_t rdx = lx - cx;
    int32_t rdy = ly - cy;
    int32_t rx = cx;
    int32_t ry = cy;
    xinc = yinc = 1 << 15;

    if (rdx < 0) {
      xinc = -1 << 15;
      rdx   = -rdx;
    }
    if (rdy < 0) {
      yinc = -1 << 15;
      rdy   = -rdy;
    }

    if (rdx > rdy) {
      e = rdy - rdx;
      dz = (240 << 16) / (rdx >> 15);
      uint8_t *dst = fb + rays[x].start;
      uint32_t ci = rays[x].cstart;
      for (int i = 0; i <= (rdx >> 15); i++, z += dz) {

        if (e >= 0) {
          e -= rdx;
          ry += yinc;
        }
        e += rdy;
        rx += xinc;

        if (i < 5)
          continue;

        uint32_t j0 = ((ry >> 15) & 3);
        uint32_t k0 = ((rx >> 15) & 3);
        uint32_t j1 = (ry >> 17) & 0x3F;
        uint32_t k1 = (rx >> 17) & 0x3F;
        uint32_t v;
  #ifdef  USE_FLASH
        uint32_t addr = 0x80000 + j1 * 512 * 4 + k1 * 32 + j0 * 8 + k0 * 2;
  #ifdef  OCEMU
        gsys->FlashRead(addr & 0xFFFFFFFC, &v, 4);
  #else
        v = *(uint32_t *)((addr & 0xFFFFFFFC) + 0x40200000);
  #endif
        if (addr & 2)
          v >>= 16;
        else
          v &= 0xFFFF;
  #else
        uint32_t addr = 0x0000 + j1 * 512 * 4 + k1 * 32 + j0 * 8 + k0 * 2;
        uint32_t addrc = addr & 0xFFFFFFC0;
        if (addrc != cacheaddr) {
          cacheaddr = addrc;
          gsys->RamRead(addrc, cachedata, 64);
        }
        v = cachedata[(addr & 63) >> 1];
  #endif

        int h = (v >> 8);
        h = ((((height - h) * testh) * divmap[z >> 16]) >> 16) + horizon;
        uint8_t c = v;
        if (h > ha[x])
          continue;
        if (h < rays[x].hend)
          h = rays[x].hend;
        uint32_t he = hi[h];
        /*if (x & 1) {
          uint16_t *zdst = zb + h * 80 + x / 2;
          for (int y = h; y < rays[x].h; y++, dst += 160, zdst += 80) {
            *dst = c;
            *zdst = z;
          }
        } else {*/
          //for (int y = rays[x].h; y > h; y--, dst += cols[h].y * 160 + cols[h].x) {
          //for (int y = h; y < cols[ci].h; dst -= cols[ci].y * 160 + cols[ci].x, ci--) {
          for (; ci >= he; dst -= cols2[ci].off, ci--) {
            *dst = c;
            if (dst < framebuffer)
              printf("#");
          }
        //}
        ha[x] = h;
      }
    } else {
      e = rdx - rdy;
      dz = (240 << 16) / (rdy >> 15);
      uint8_t *dst = fb + rays[x].start;
      uint32_t ci = rays[x].cstart;
      for (int i = 0; i <= (rdy >> 15); i++, z += dz) {
        if (e >= 0) {
          e -= rdy;
          rx += xinc;
        }
        e += rdx;
        ry += yinc;

        if (i < 5)
          continue;

        uint32_t j0 = ((ry >> 15) & 3);
        uint32_t k0 = ((rx >> 15) & 3);
        uint32_t j1 = (ry >> 17) & 0x3F;
        uint32_t k1 = (rx >> 17) & 0x3F;
        uint32_t v;
  #ifdef  USE_FLASH
        uint32_t addr = 0x80000 + j1 * 512 * 4 + k1 * 32 + j0 * 8 + k0 * 2;
  #ifdef  OCEMU
        gsys->FlashRead(addr & 0xFFFFFFFC, &v, 4);
  #else
        v = *(uint32_t *)((addr & 0xFFFFFFFC) + 0x40200000);
  #endif
        if (addr & 2)
          v >>= 16;
        else
          v &= 0xFFFF;
  #else
        uint32_t addr = 0x0000 + j1 * 512 * 4 + k1 * 32 + j0 * 8 + k0 * 2;
        uint32_t addrc = addr & 0xFFFFFFC0;
        if (addrc != cacheaddr) {
          cacheaddr = addrc;
          gsys->RamRead(addrc, cachedata, 64);
        }
        v = cachedata[(addr & 63) >> 1];
  #endif

        int h = (v >> 8);
        h = ((((height - h) * testh) * divmap[z >> 16]) >> 16) + horizon;
        uint8_t c = v;
        if (h > ha[x])
          continue;
        if (h < rays[x].hend)
          h = rays[x].hend;
        uint32_t he = hi[h];
        /*if (x & 1) {
          uint16_t *zdst = zb + h * 80 + x / 2;
          for (int y = h; y < rays[x].h; y++, dst += 160, zdst += 80) {
            *dst = c;
            *zdst = z;
          }
        } else {*/
          //for (int y = rays[x].h; y > h; y--, dst += cols[h].y * 160 + cols[h].x) {
          //for (int y = h; y < cols[ci].h; dst -= cols[ci].y * 160 + cols[ci].x, ci--) {
          for (; ci >= he; dst -= cols2[ci].off, ci--) {
            *dst = c;
            if (dst < framebuffer)
              printf("#");
          }
        //}
        ha[x] = h;
      }
    }
  }
}

// MOVE divmap to stack

#define ONKEY(kc, v)  if ((k & (kc)) == (kc)) { v; }

void main_loop() {
  volatile uint32_t ticks = gsys->GetMsTicks();
  call_defer_funcs(ticks);

  uint32_t seed = 0x12345678;

  rx += 2; if (rx >= 720) rx -= 720;
  ry += 5; if (ry >= 720) ry -= 720;
  rz += 3; if (rz >= 720) rz -= 720;

  /*ticks = gsys->GetMsTicks();
  uint32_t x = 1, rand = 0;
  for (int i = 0; i < 10000; i++) {
    uint32_t *addr = (rand & 0x0003FFFC) + 0x40280000;
    x += *addr;
    rand += 32;
  }
  ticks = gsys->GetMsTicks() - ticks;
  gsys->Printf("%d %d\n", ticks, x);
  gsys->SleepMs(20);
  return;*/
  uint32_t k = gsys->GetKeys();

  ONKEY(KEY_UP, roll++);
  ONKEY(KEY_DOWN, roll--);
  ONKEY(KEY_LEFT, mx -= 1; my -= 1);
  ONKEY(KEY_RIGHT, mx += 1; my -= 1);
  ONKEY(KEY_A, horizon++);
  ONKEY(KEY_B, horizon--);
  ONKEY(KEY_X, angle+=2);
  ONKEY(KEY_Y, angle-=2);
//  ONKEY(KEY_X | KEY_UP, testh++; printf("%d\n", testh));
//  ONKEY(KEY_X | KEY_DOWN, testh--; printf("%d\n", testh));
  //ONKEY(KEY_Y | KEY_UP, height++; printf("%d\n", height));
  //ONKEY(KEY_Y | KEY_DOWN, height--; printf("%d\n", height));
  if (angle >= 720)
    angle -= 720;
  if (angle < 0)
    angle += 720;
  if (roll >= 720)
    roll -= 720;
  if (roll < 0)
    roll += 720;

  memset(framebuffer, 0, 20480);
  uint16_t *pal = (uint16_t *)gsys->GetMemBlock(1) + 5 * 1024;
  render_voxel();
/*  uint8_t *fb = (uint8_t *)framebuffer;
  int16_t ha[160];
  uint16_t lb[256];
  for (int i = 0; i < 160; i++)
    ha[i] = 128;
  uint32_t dz = 1 << 8;
#ifndef USE_FLASH
    gsys->RamRead(0 + 512 * ((my + 5) & 0xFF), lb, 512);
#endif
  for (int32_t zz = 5 << 8; zz < 160 << 8; dz += 0) {
    zz += dz;
    int32_t z = zz >> 8;
    uint32_t yy = (my + z) & 0xFF;
#ifdef  USE_FLASH
    gsys->FlashRead(0x80000 + 512 * yy, lb, 512);
#else
    uint32_t rcnt = 0, rp = 0;
#endif
    uint32_t x0 = ((800 + mx - z) & 0xFF) << 8;
    uint32_t dx = ((2 * z) << 8) / 160;
    for (uint32_t x = 0; x < 160; x++) {
#ifndef USE_FLASH
      if (rcnt == 20) {
        if (x > 20) {
          gsys->RamReadEnd(lb + rp * 32, 64);
          rp++;
        }
        gsys->RamReadStart(0 + 512 * yy + rp * 64, 64);
        rcnt = 0;
      }
      rcnt++;
#endif
      int h = (lb[x0 >> 8] >> 8);
      h = ((((height - h) * 12) * divmap[z]) >> 16) + horizon;
      uint8_t c = lb[x0 >> 8];
      x0 = (x0 + dx) & 0xFFFF;
      if (h > 127)
        continue;
      if (h < 0)
        h = 0;
      if (h > ha[x])
        continue;
      uint8_t *dst = fb + h * 160 + x;
      for (int y = h; y < ha[x]; y++, dst += 160)
        *dst = c;
      ha[x] = h;
    }
#ifndef USE_FLASH
    gsys->RamReadEnd(lb + rp * 32, 64);
    rp++;
    gsys->RamRead(0 + 512 * yy + rp * 64, lb + rp * 32, 64);
#endif
  }*/
  memcpy(framebuffer + 10240, pal, 512);
  ticks = gsys->GetMsTicks() - ticks;
  gsys->UpdateScreen8(framebuffer, framebuffer + 10240, 0, 127, 1);
#ifndef OCEMU
  gsys->Printf("%d\n", ticks);
#else
  gsys->SleepMs(20);
#endif
  if (k != KEY_X) {
    //mx += 1;
    //my += 1;
  }
  return;

  POINT2DZ *pp = gsys->GetMemBlock(1);

  uint32_t ticks2 = gsys->GetMsTicks();
  transform_meshz(testpoints, pp, testobject.point_cnt, rx, ry, rz, 80, 64, 3060, 100);

  for (int i = 0; i < 64; i++) {
    uint16_t c = (i << 5);
    pal[i + 64] = (c << 8) | (c >> 8);
  }

  uint16_t *zbuf = (uint16_t *)(framebuffer + 10240);

  buffer_clear(zbuf);

  for (int i = 0; i < testobject.face_cnt; i++) {
    uint32_t c = RAND & 255;
    c = testfaces[i].color + 10;
    c = c | (c << 8) | (c << 16) | (c << 24);
    if (testfaces[i].p1 == 0 && testfaces[i].p2 == 0)
      break;
    POINT2DZ *p1 = &pp[testfaces[i].p1];
    POINT2DZ *p2 = &pp[testfaces[i].p2];
    POINT2DZ *p3 = &pp[testfaces[i].p3];
    int32_t dx1 = p2->x - p1->x;
    int32_t dy1 = p2->y - p1->y;
    int32_t dx2 = p3->x - p1->x;
    int32_t dy2 = p3->y - p1->y;
    int32_t n = dx2 * dy1 - dx1 * dy2;
    //c = 64 + n / 2;
    //if (c > 127)
    //  c = 127;
    //c = (testpoints[testfaces[i].p1].y + 2048) >> 8;
    if (n > 0)
      scan_triangle3(p1, p2, p3, c);
  }
  ticks2 = gsys->GetMsTicks() - ticks2;

  uint32_t ticks3 = gsys->GetMsTicks();
  memcpy(framebuffer + 10240, pal, 512);
  gsys->UpdateScreen8(framebuffer, framebuffer + 10240, 0, 127, 1);
  ticks3 = gsys->GetMsTicks() - ticks3;
#ifndef OCEMU
  gsys->Printf("%d %d\n", ticks2, ticks3);
#endif

#ifdef OCEMU
  gsys->SleepMs(20);
#else
  gsys->SleepMs(1);
#endif
}

void main_init(SYSCALLS *sys) {
  gsys = sys;
  sys->DiskInit();
  config_init(sys->GetMemBlock(1));
  init_defer_funcs();
  framebuffer = sys->GetMemBlock(0);
  memset(keyCounter, 0, sizeof(keyCounter));
  lastKeyTick = 0;
  keyDelay = KEY_DELAY;
  keyRepeat = KEY_REPEAT;

  rx = 0;
  ry = 0;
  rz = 0;
}

uint32_t load_file(const char *fname, uint32_t addr, int32_t size, uint32_t *buf, uint32_t bufsize) {
  uint32_t br = 0;
  gsys->FileOpen(fname, FILE_MODE_READ);

  if (addr > 8 * 1024 * 1024 && addr < 0x40200000) {
    gsys->FileRead(addr, size, &br);
    gsys->FileClose();
    return br;
  }

  while (size > 0) {
    uint32_t tr;
    uint32_t chunk = size;
    if (chunk > bufsize)
      chunk = bufsize;
    memset(buf, 0xFF, bufsize);
    gsys->FileRead(buf, chunk, &tr);
    if (tr == 0)
      break;
    tr += (4 - (tr & 3)) & 3;
    br += tr;
    if (addr < 8 * 1024 * 1024)
      gsys->RamWrite(addr, buf, tr);
    else {
      uint8_t cmpbuf[512];
      uint32_t s = tr;
      uint32_t taddr = addr - 0x40200000;
      uint8_t *tnew = (uint8_t *)buf;
      while (s > 0) {
        uint32_t part = s;
        if (part > 512)
          part = 512;
        gsys->FlashRead(taddr, cmpbuf, part);
        if (memcmp(cmpbuf, tnew, part) != 0)
          break;
        s -= part;
        taddr += part;
        tnew += part;
      }
      if (s > 0) {
        gsys->FlashErase(addr - 0x40200000, tr < 4096 ? 4096 : tr);
        gsys->FlashWrite(addr - 0x40200000, buf, tr);
      }
    }
    size -= tr;
    addr += tr;
  }
  gsys->FileClose();
  return br;
}

void main_start() {
  add_defer_func(0, 1000, timer_1s, 0);

  gsys->Printf("Bat: %d\n", gsys->GetBatteryLevel());
  gsys->SleepMs(500);

  DEBUG("init\n");
  display_init(framebuffer);
  DEBUG("text\n");
  display_draw_text(20, 50, "Test Text 123");
  DEBUG("update\n");
  gsys->UpdateScreen16(framebuffer, 0, 127, 0);
  gsys->Printf("bubugugu\n");

#ifdef  USE_FLASH
  load_file("0:/map1s.bin", 0x40280000, 2 * 256 * 256 + 512, gsys->GetMemBlock(0), 32768);
#else
  load_file("0:/map1s.bin", 0, 2 * 256 * 256 + 512, gsys->GetMemBlock(0), 32768);
#endif

  static const char RODATA ssid[32] = "2WIRE774";
  static const char RODATA pwd[64] = "deadbeef73";
  gsys->WifiConnect((char *)ssid, (char *)pwd);
    gsys->Printf("%d %d\n", gsys->GetFreeMem(), gsys->GetDiskSpeed());

  uint16_t *pal = (uint16_t *)gsys->GetMemBlock(1) + 5 * 1024;
#ifdef  USE_FLASH
  gsys->FlashRead(0x80000 + 2 * 256 * 256, pal, 512);
#else
  gsys->RamRead(0 + 2 * 256 * 256, pal, 512);
#endif

  scan_draw_init();

  mx = 0;
  my = 0;
  height = 160;
  horizon = 32;
  angle = 0;
  roll = 0;
  testh = 16;
}

void wifi_status_callback(uint32_t event, WIFIEVENT *data) {
    gsys->Printf("%d %d %X\n", event, data->reason, data->ip);
  if (event == WIFI_EVENT_GOT_IP) {
    gsys->WifiEnableMDNS(NULL);
    gsys->WifiResolveName("www.google.com");
    gsys->Printf("%d\n", gsys->GetFreeMem());
    gsys->WifiEnableUDP(0xFF01A8C0, 5555);
    control_init();
  }
}

void wifi_scan_callback(WIFINET *netbuf, uint32_t count) {

}

void tcp_connect_callback(TCPCONN *conn) {
  control_connect_callback(conn);
}

void tcp_reconnect_callback(TCPCONN *conn, int8_t error) {

}

void tcp_disconnect_callback(TCPCONN *conn) {
  control_disconnect_callback(conn);
  gsys->Printf("Done %X\n", gsys->WifiGetIP());
}

void tcp_sent_callback(TCPCONN *conn) {
  control_sent_callback(conn);
}

void tcp_recv_callback(TCPCONN *conn, void *data, uint32_t size) {
  control_recv_callback(conn, data, size);
}

void udp_recv_callback(UDPCONN *conn, void *data, uint32_t size) {
  gsys->Printf("UDP: %s\n", data);
}

void dns_result_callback(const char *name, uint32_t ip) {
  gsys->Printf("IP: %X %d\n", ip, gsys->GetFreeMem());
}

#ifndef OCEMU
#define OCDATA __attribute__((section(".irom2.text"))) STORE_ATTR
#else
#define OCDATA
#endif

const OCCALLS OCDATA __calltable = {
  .magic = FLASH_OC_MAGIC,
  .version = 1,
  .MainInit = main_init,
  .MainStart = main_start,
  .MainLoop = main_loop,
  .WifiStatusCallback = wifi_status_callback,
  .WifiScanCallback = wifi_scan_callback,
  .TcpConnectCallback = tcp_connect_callback,
  .TcpReconnectCallback = tcp_reconnect_callback,
  .TcpDisconnectCallback = tcp_disconnect_callback,
  .TcpSentCallback = tcp_sent_callback,
  .TcpRecvCallback = tcp_recv_callback,
  .UdpRecvCallback = udp_recv_callback,
  .DnsResultCallback = dns_result_callback,
  .ConfigGetString = config_get_string,
  .ConfigGetNumber = config_get_number
};
