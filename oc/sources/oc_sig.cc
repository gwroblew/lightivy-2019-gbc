#include <stdlib.h>
#include <string.h>
#include "oc.h"
#include "display.h"
#include "control.h"
#include "config.h"

#include "gfx3d.h"

#define TC(x, y)  ((x << 24) | (y << 8))

/*const OBJECT3D RODATA testobject = {
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
};*/

SYSCALLS * RWDATA gsys;
extern uint16_t * RWDATA framebuffer;
uint16_t RWDATA keyCounter[8];
uint32_t RWDATA lastKeyTick;
uint16_t RWDATA keyDelay;
uint16_t RWDATA keyRepeat;
OCDEFER RWDATA deffuncs[MAX_DEFERRED_FUNCS];

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

  //if (gsys->WifiStatus() == WIFI_STATUS_GOT_IP) {
    //gsys->WifiSendUDP("test2\n", 6);
    //TIME dt;
    //gsys->GetTime(&dt);
    //gsys->Printf("%d %d %d %d %d %d\n", dt.tm_year, dt.tm_mon+1, dt.tm_mday, dt.tm_hour, dt.tm_min, dt.tm_sec);
  //}
  return DEFER_RESULT_OK;
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
}

#ifdef  OCEMU
uint32_t load_file_emu(const char *fname, void *addr, int32_t size, uint32_t *buf, uint32_t bufsize) {
  if ((uint64_t)addr < 0x40200000) {
    return load_file(fname, (uint32_t)addr, size, buf, bufsize);
  }
  uint32_t br = 0;
  gsys->FileOpen(fname, FILE_MODE_READ);
  gsys->FileRead(addr, size, &br);
  gsys->FileClose();
  return br;
}
#endif

uint32_t load_file(const char *fname, uint32_t addr, int32_t size, uint32_t *buf, uint32_t bufsize) {
  uint32_t br = 0;
  gsys->FileOpen(fname, FILE_MODE_READ);
  //printf("%s %x %x\n", fname, addr, size);

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
        uint32_t te = (tr & 0xFFFF000) + ((tr & 4095) ? 4096 : 0);
        gsys->FlashErase(addr - 0x40200000, te);
        gsys->FlashWrite(addr - 0x40200000, buf, tr);
      }
    }
    size -= tr;
    addr += tr;
    gsys->SleepMs(1);
  }
  gsys->FileClose();
  return br;
}

uint16_t RWDATA appid;
uint16_t RWDATA appsel;

void mark_appsel() {
  for (int i = 0; i < 11 * 160; i++)
    framebuffer[(22 + appsel * 12) * 160 + i] ^= 0x1F00;
}

void app_start() {
  appid = appsel + 1;
  memset(framebuffer, 0, 160 * 128 * 2);
  gsys->UpdateScreen16(framebuffer, 0, 127, 0);

  if (appid == 1) {
    main_start1();
    return;
  }
  if (appid == 2) {
    main_start2();
    return;
  }
  if (appid == 3) {
    main_start3();
    return;
  }
}

extern uint8_t RWDATA state;

#include "siglogo.h"

void main_menu() {
  memset(framebuffer, 0, 128 * 160 * 2);
  memcpy(framebuffer + 128 * 160 - SIGLOGO_LEN / 2, siglogo, SIGLOGO_LEN);
  appid = 0;
  appsel = 0;
  display_draw_aligned(2, "=== Menu ===");
  display_draw_aligned(20, "Flappy Birds 3D");
  display_draw_aligned(32, "3D Demo");
  display_draw_aligned(44, "Distance Fields");
  mark_appsel();
  gsys->UpdateScreen16(framebuffer, 0, 127, 0);
  //gsys->Printf("bubugugu\n");
  lastKeyTick = gsys->GetKeys();
}

extern uint32_t RWDATA objcnt;
extern uint32_t RWDATA fadecnt;
extern uint32_t RWDATA screen_ymax;

void main_loop() {
  volatile uint32_t ticks = gsys->GetMsTicks();
  call_defer_funcs(ticks);
  if (state != 0) {
    gsys->StopAudio();
    return;
  }
  uint32_t k = gsys->GetKeys();
  if (k & KEY_Y) {
    gsys->StopAudio();
    gsys->Reset(0);
    /*while (gsys->GetKeys() & KEY_Y)
      gsys->SleepMs(1);
    objcnt = 0;
    fadecnt = 0;
    screen_ymax = 127;
    main_menu();
    return;*/
  }
  if (appid == 0) {
    if (k != lastKeyTick) {
      lastKeyTick = k;
      mark_appsel();
      ONKEY(KEY_UP, appsel = appsel > 0 ? (appsel - 1) : appsel);
      ONKEY(KEY_DOWN, appsel = appsel < 2 ? (appsel + 1) : appsel);
      ONKEY(KEY_A, app_start(); return);
      mark_appsel();
      gsys->UpdateScreen16(framebuffer, 0, 127, 0);
    }
    return;
  }
  if (appid == 1) {
    main_loop1();
    return;
  }
  if (appid == 2) {
    main_loop2();
    return;
  }
  if (appid == 3) {
    main_loop3();
    return;
  }
}

void main_start() {
  objcnt = 0;
  fadecnt = 0;
  screen_ymax = 127;
  //add_defer_func(0, 1000, timer_1s, 0);

  //gsys->Printf("Bat: %d\n", gsys->GetBatteryLevel());
  //gsys->SleepMs(500);

  //DEBUG("init\n");
  display_init(framebuffer);
  //DEBUG("text\n");
  //display_draw_text(20, 50, "Test Text 123");
  //DEBUG("update\n");

  //static const char RODATA ssid[32] = "2WIRE774";
  //static const char RODATA pwd[64] = "deadbeef73";
  //gsys->WifiConnect((char *)ssid, (char *)pwd);
  //gsys->Printf("%d %d\n", gsys->GetFreeMem(), gsys->GetDiskSpeed());

  main_menu();
}

void wifi_status_callback(uint32_t event, WIFIEVENT *data) {
    gsys->Printf("%d %d %X\n", event, data->reason, data->ip);
  if (event == WIFI_EVENT_GOT_IP) {
    //gsys->WifiEnableMDNS(NULL);
    //gsys->WifiResolveName("www.google.com");
    //gsys->Printf("%d\n", gsys->GetFreeMem());
    //gsys->WifiEnableUDP(0xFF01A8C0, 5555);
//    control_init();
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
  //gsys->Printf("Done %X\n", gsys->WifiGetIP());
}

void tcp_sent_callback(TCPCONN *conn) {
  control_sent_callback(conn);
}

void tcp_recv_callback(TCPCONN *conn, void *data, uint32_t size) {
  control_recv_callback(conn, data, size);
}

void udp_recv_callback(UDPCONN *conn, void *data, uint32_t size) {
  //gsys->Printf("UDP: %s\n", data);
}

void dns_result_callback(const char *name, uint32_t ip) {
  //gsys->Printf("IP: %X %d\n", ip, gsys->GetFreeMem());
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
