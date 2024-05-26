#include <stdlib.h>
#include <string.h>
#include "oc.h"
#include "display.h"
#include "control.h"
#include "config.h"

#include "gfx3d.h"
//#include "test.h"
#include "wolf.h"
#include "test2.h"

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

uint32_t RWDATA rubcnt;

#define RAND    (seed = (((seed ^ 0xBACADA55) + 0x9123) >> 11) ^ (seed + 0x7432))

#define FASTZB   1

// MOVE divmap to stack

void main_loop() {
  volatile uint32_t ticks = gsys->GetMsTicks();
  call_defer_funcs(ticks);

  uint32_t seed = 0x12345678;

  //gsys->SleepMs(20);
  //gsys->UpdateScreen16(framebuffer, 0, 127, 0);
  //gsys->Printf("%d\n", gsys->GetFreeMem());
  //gsys->Printf("%d %X\n", gsys->GetFreeMem(), gsys->WifiGetIP());
  rx += 5; if (rx >= 720) rx -= 720;
  ry += 2; if (ry >= 720) ry -= 720;
  rz += 3; if (rz >= 720) rz -= 720;

  /*rotate_init(rx, ry, rz);
  ticks = gsys->GetMsTicks();
  int x = 123456, y = 123456, z = 123456;
  for (int i = 0; i < 100000; i++)
    ROTATE_POINT(x, y, z);
  ticks = gsys->GetMsTicks() - ticks;*/
  /*ticks = gsys->GetMsTicks();
  int x = 123456, y = 123456, z = 123;
  for (int i = 0; i < 50000; i++)
    //x += (x + y) / z;
    x += (x * divmap[(z + x)&255]) >> 7;
  ticks = gsys->GetMsTicks() - ticks;
  gsys->Printf("%d %d\n", ticks, x);
  gsys->SleepMs(20);
  return;*/

  /*shader();
  gsys->UpdateScreen16(framebuffer, 0, 127, 0);
  //gsys->Printf("%d %d\n", ticks, x + y + z);
  gsys->SleepMs(1);
  return;*/

  POINT2DZ *pp = gsys->GetMemBlock(1);

  uint32_t ticks2 = gsys->GetMsTicks();
  transform_meshz(testpoints, pp, testobject.point_cnt, rx, ry, rz, 80, 64, 2760, 100);

  uint16_t *pal = (uint16_t *)gsys->GetMemBlock(1) + 5 * 1024;
  pal[255] = 0xFFFF;

#ifdef FASTZB
  SCANLINE *sb = (SCANLINE *)(framebuffer + 256);
  scan_clear(sb);
#else
  SCANLINE2 *sb2 = (SCANLINE2 *)(framebuffer + 256);
  scan_clear2(sb2);
#endif

  for (int i = 0; i < testobject.face_cnt; i++) {
    uint32_t c = RAND & 255;
    //c = testfaces[i].color + 10;
    c = c | (c << 8) | (c << 16) | (c << 24);
    POINT2DZ *p1 = &pp[testfaces[i].p1];
    POINT2DZ *p2 = &pp[testfaces[i].p2];
    POINT2DZ *p3 = &pp[testfaces[i].p3];
    int32_t dx1 = p2->x - p1->x;
    int32_t dy1 = p2->y - p1->y;
    int32_t dx2 = p3->x - p1->x;
    int32_t dy2 = p3->y - p1->y;
    if (dx1 * dy2 - dx2 * dy1 < 0)
#ifdef FASTZB
      scan_triangle1(p1, p2, p3, c);
#else
      scan_triangle(p1, p2, p3, c);
#endif
  }
  transform_meshz(testpoints2, pp, testobject2.point_cnt, rx, rz, ry, 80, 64, 4260, 100);
  for (int i = 0; i < testobject2.face_cnt; i++) {
    uint32_t c = RAND & 255;
    c = testfaces2[i].color + 20;
    c = c | (c << 8) | (c << 16) | (c << 24);
    POINT2DZ *p1 = &pp[testfaces2[i].p1];
    POINT2DZ *p2 = &pp[testfaces2[i].p2];
    POINT2DZ *p3 = &pp[testfaces2[i].p3];
    int32_t dx1 = p2->x - p1->x;
    int32_t dy1 = p2->y - p1->y;
    int32_t dx2 = p3->x - p1->x;
    int32_t dy2 = p3->y - p1->y;
//    if (dx1 * dy2 - dx2 * dy1 < 0)
#ifdef FASTZB
//      scan_triangle1(p1, p2, p3, c);
#else
      scan_triangle(p1, p2, p3, c);
#endif
  }
  ticks2 = gsys->GetMsTicks() - ticks2;

  uint32_t fill = 0;
#ifdef FASTZB
  for (int i = 0; i < SCREEN_HEIGHT; i++)
    fill += sb[i].cnt;
#else
  for (int i = 0; i < SCREEN_HEIGHT; i++)
    fill += sb2[i].cnt;
#endif
  ticks = gsys->GetMsTicks();
#ifdef FASTZB
  scan_draw1(sb, (uint8_t *)framebuffer);
#else
  scan_draw(sb2, (uint8_t *)framebuffer);
#endif
  ticks = gsys->GetMsTicks() - ticks;
  /*gsys->RamWrite(rubcnt*20*1024, framebuffer, 20*1024);

  for (int i = 0; i < 128; i++)
    gsys->RamRead(((i + rubcnt) & 127)*20*1024+i*160, framebuffer+i*80, 160);

  rubcnt++;
  if (rubcnt == 128)
    rubcnt = 0;*/

  uint32_t ticks3 = gsys->GetMsTicks();
  memcpy(framebuffer + 10240, pal, 512);
  gsys->UpdateScreen8(framebuffer, framebuffer + 10240, 0, 127, 1);
  ticks3 = gsys->GetMsTicks() - ticks3;
#ifndef OCEMU
  gsys->Printf("%d %d %d %d\n", ticks2, ticks, ticks3, (fill * 100) / (SCREEN_HEIGHT * MAX_SCAN_FRAGS));
#endif

  /*for (int i = 0; i < testobject.point_cnt; i++) {
    if (pp[i].x < 0 || pp[i].x > 159 || pp[i].y < 0 || pp[i].y > 127)
      continue;
    framebuffer[pp[i].y * 160 + pp[i].x] = 0xFFFF;
  }
  gsys->UpdateScreen16(framebuffer, 0, 127, 1);*/
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
  rubcnt = 0;
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
