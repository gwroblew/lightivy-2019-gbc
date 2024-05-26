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
  for (int i = 1; i < 256; i++) {
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

#define RAND    (seed = (((seed ^ 0xBACADA55) + 0x9123) >> 11) ^ (seed + 0x7432))

// MOVE divmap to stack

uint32_t __attribute__((optimize("O0"))) foo(uint32_t c) {
  for (int j = 0; j < 5; j++)
    c++;
  return c;
}

void main_loop() {
  volatile uint32_t ticks = gsys->GetMsTicks();
  call_defer_funcs(ticks);

  uint32_t seed = 0x12345678;

  rx += 2; if (rx >= 720) rx -= 720;
  ry += 5; if (ry >= 720) ry -= 720;
  rz += 3; if (rz >= 720) rz -= 720;

  /*ticks = gsys->GetMsTicks();
  uint32_t x = 1, rand = 0;
  for (int i = 0; i < 100000; i++) {
    uint16_t *addr = (rand & 0x0003FFFC) + 0x40280000;
    x += *addr;
    rand += 501;
  }
  ticks = gsys->GetMsTicks() - ticks;
  gsys->Printf("%d %d\n", ticks, x);
  gsys->SleepMs(20);
  return;*/
  ticks = gsys->GetMsTicks();
  uint32_t x = 1, rand = 0, c = 0;
  gsys->RamMode();
    //WRITE_PERI_REG(SPI_USER(1), SPI_USR_COMMAND | SPI_USR_ADDR | SPI_USR_DUMMY | SPI_USR_MISO | SPI_CK_I_EDGE);
    (*((volatile uint32_t *)(((0x60000200-1*0x100) + 0x1C)))) = (uint32_t)(((1UL << (31))) | ((1UL << (30))) | ((1UL << (29))) | ((1UL << (28))) | ((1UL << (6))));
    while ((*((volatile uint32_t *)(((0x60000200-1*0x100) + 0x0)))) & (1UL << 18)) c++;
  for (int i = 0; i < 10000; i++) {
    uint32_t addr = (rand & 0x0003FFFF);
    //gsys->RamRead(addr, &x, 1);
    //WRITE_PERI_REG(SPI_USER1(1), (23<<SPI_USR_ADDR_BITLEN_S) | (7<<SPI_USR_DUMMY_CYCLELEN_S) | ((size * 8 - 1)<<SPI_USR_MISO_BITLEN_S));
    (*((volatile uint32_t *)(((0x60000200-1*0x100) + 0x20)))) = (uint32_t)((23<<26) | (7<<0) | ((2 * 8 - 1)<<8));
    //WRITE_PERI_REG(SPI_USER2(1), (7<<SPI_USR_COMMAND_BITLEN_S) | 0x0B);
    (*((volatile uint32_t *)(((0x60000200-1*0x100) + 0x24)))) = (uint32_t)((7<<28) | 0x0B);
    //GPIO_OUTPUT_SET(PIN_RAM_CS, 0);
    (*((volatile uint32_t *)(0x60000200+0x108))) = 1UL << 4;
    //WRITE_PERI_REG(SPI_ADDR(1), addr<<8);
    (*((volatile uint32_t *)(((0x60000200-1*0x100) + 0x4)))) = (uint32_t)(addr<<8);
    //SET_PERI_REG_MASK(SPI_CMD(1), SPI_USR);
    //WRITE_PERI_REG((((0x60000200-1*0x100) + 0x0)), (READ_PERI_REG(((0x60000200-1*0x100) + 0x0))|(((1UL << (18))))));
    (*((volatile uint32_t *)(((0x60000200-1*0x100) + 0x0)))) = (*((volatile uint32_t *)(((0x60000200-1*0x100) + 0x0)))) | (1UL << 18);
    //while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
    c = foo(c);
    while ((*((volatile uint32_t *)(((0x60000200-1*0x100) + 0x0)))) & (1UL << 18)) c |= 131072;
    //GPIO_OUTPUT_SET(PIN_RAM_CS, 1);
    (*((volatile uint32_t *)(0x60000200+0x104))) = 1UL << 4;
    //x += READ_PERI_REG((SPI_W0(1)));
    x += (*((volatile uint32_t *)((((0x60000200-1*0x100) +0x40) + 0))));
    rand += 501;
  }
  ticks = gsys->GetMsTicks() - ticks;
  gsys->LcdMode();
  gsys->Printf("%d %d %d\n", ticks, c, x&15);
  gsys->SleepMs(20);
  return;

  POINT2DZ *pp = gsys->GetMemBlock(1);

  uint32_t ticks2 = gsys->GetMsTicks();
  transform_meshz(testpoints, pp, testobject.point_cnt, rx, ry, rz, 80, 64, 3060, 100);

  uint16_t *pal = (uint16_t *)gsys->GetMemBlock(1) + 5 * 1024;
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

  static const char RODATA ssid[32] = "2WIRE774";
  static const char RODATA pwd[64] = "deadbeef73";
  gsys->WifiConnect((char *)ssid, (char *)pwd);
    gsys->Printf("%d %d\n", gsys->GetFreeMem(), gsys->GetDiskSpeed());

  uint16_t *pal = (uint16_t *)gsys->GetMemBlock(1) + 5 * 1024;
  uint32_t seed = 0x12345678;

  uint16_t *pd = pal;
  for (int i = 0; i < 256; i++) {
    *pd++ = seed;
    seed = (((seed ^ 0xBACADA55) + 0x9123) << 11) ^ (seed + 0x7432);
  }
  scan_draw_init();
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
