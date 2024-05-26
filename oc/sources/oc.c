#include <stdlib.h>
#include <string.h>
#include "oc.h"
#include "display.h"
#include "control.h"
#include "config.h"

//#include "spec.h"
//#include "nes_emu.h"
#include "boy_emu.h"

SYSCALLS * RWDATA gsys;
uint32_t RWDATA ctrlmode;
extern uint16_t * RWDATA framebuffer;
uint16_t RWDATA keyCounter[10];
uint32_t RWDATA lastKeyTick;
uint16_t RWDATA keyDelay;
uint16_t RWDATA keyRepeat;
OCDEFER RWDATA deffuncs[MAX_DEFERRED_FUNCS];

void ROCODE init_defer_funcs() {
  for (int i = 0; i < MAX_DEFERRED_FUNCS; i++)
    deffuncs[i].func = NULL;
}

void ROCODE add_defer_func(uint32_t start_tick, uint32_t interval, uint32_t (*func)(OCDEFER *def), void *arg) {
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

void ROCODE remove_defer_func(uint32_t (*func)(OCDEFER *def), void *arg) {
  int i;
  for (i = 0; i < MAX_DEFERRED_FUNCS; i++)
    if (deffuncs[i].func == func && deffuncs[i].arg == arg)
      break;
  if (i == MAX_DEFERRED_FUNCS)
    return;
  deffuncs[i].func = NULL;
}

void ROCODE call_defer_funcs(uint32_t current_tick) {
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

uint32_t ROCODE timer_1s(OCDEFER *def) {

  //if (gsys->WifiStatus() == WIFI_STATUS_GOT_IP) {
    //gsys->WifiSendUDP("test2\n", 6);
    //TIME dt;
    //gsys->GetTime(&dt);
    //gsys->Printf("%d %d %d %d %d %d\n", dt.tm_year, dt.tm_mon+1, dt.tm_mday, dt.tm_hour, dt.tm_min, dt.tm_sec);
  //}
  return DEFER_RESULT_OK;
}

extern int RODATA _text_start, _text_end, _iram1_text_start, _iram1_text_end;

void ROCODE main_init(SYSCALLS *sys) {
  gsys = sys;
  sys->DiskInit();
  ctrlmode = 0;
  uint32_t k = sys->GetKeys();
  if (k & KEY_RIGHT) {
    ctrlmode = 1;
    init_defer_funcs();
    control_set_workbuf(gsys->GetMemBlock(MEM_TYPE_RAM, WORKBUF_SIZE));
  }
  return;

  /*config_init(sys->GetMemBlock(1));
  framebuffer = sys->GetMemBlock(0);*/
}

#ifndef  OCEMU
static inline uint32_t asm_ccount(void) {
    uint32_t r;
    asm volatile ("rsr %0, ccount" : "=r"(r));
    return r;
}

uint32_t get_raw_ticks() {
  return asm_ccount() / 160000;
}

void raw_sleep_ms(uint32_t ticks) {
  uint32_t cnt = asm_ccount();
  uint32_t end = cnt + 160000 * ticks;
  if (end > cnt) {
    while (asm_ccount() < end);
  } else {
    while (asm_ccount() > end);
    while (asm_ccount() < end);
  }
}

void raw_sleep_us(uint32_t ticks) {
  uint32_t cnt = asm_ccount();
  uint32_t end = cnt + 160 * ticks;
  if (end > cnt) {
    while (asm_ccount() < end);
  } else {
    while (asm_ccount() > end);
    while (asm_ccount() < end);
  }
}

void poll_audio() {
  SOUNDSYS *ss = (SOUNDSYS *)SOUNDSYSPTR;

  if (ss == NULL)
    return;

  uint32_t slc_intr_status = *(volatile uint32_t *)0x60000B08;
	if (slc_intr_status & (1 << 17)) {
    *(volatile uint32_t *)0x60000B10 = 0xffffffff;
    uint32_t lastdma = ss->dmaidx;

    ss->dmaidx = (ss->dmaidx + 1) % I2SDMABUFCNT;
		if (ss->dmaidx == ss->lastidx) {
      memcpy(ss->i2sBuf[ss->dmaidx], ss->i2sBuf[lastdma], 1024);
      ss->lastidx = (ss->lastidx + 1) % I2SDMABUFCNT;
			ss->underrunCnt++;
		}
	}
}

uint32_t *get_audio_buf() {
  poll_audio();
  SOUNDSYS *ss = (SOUNDSYS *)SOUNDSYSPTR;
  int next = (ss->lastidx + 1) % I2SDMABUFCNT;
  if (next == ss->dmaidx) {
    return NULL;
  }
  ss->lastidx = next;
  return (uint32_t *)ss->i2sBuf[next];
}

#endif

uint32_t ROCODE load_file2(const char *fname, uint32_t ramflag, uint32_t addr, int32_t size, uint32_t *buf, uint32_t bufsize) {
  uint32_t br = 0;
  gsys->FileOpen((char *)fname, FILE_MODE_READ);

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
    if (ramflag)
      gsys->RamWrite(addr, buf, tr);
    else {
      uint8_t cmpbuf[512];
      uint32_t s = tr;
      uint32_t taddr = addr;
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
        gsys->FlashErase(addr, te);
        gsys->FlashWrite(addr, buf, tr);
      }
    }
    size -= tr;
    addr += tr;
    gsys->SleepMs(1);
  }
  gsys->FileClose();
  return br;
}

void ROCODE load_image(FILEINFO *fi, uint16_t *fb) {
  char fn[160];
  strcpy(fn, "0:/gbc/images/");
  strcpy(&fn[14], fi->filename);
  gsys->FileOpen(fn, FILE_MODE_READ);
  uint32_t br;
  for (int i = 0; i < 128; i++) {
    uint16_t *fl = fb + i * 160 + 16;
    gsys->FileRead(fl, 256, &br);
  }
  gsys->FileClose();
  gsys->LcdMode();
  gsys->UpdateScreen16(fb, 0, 127, 0);
}

uint32_t ROCODE load_val(char *fn) {
  uint32_t gi = 0, br;
  gsys->FileOpen(fn, FILE_MODE_READ);
  gsys->FileRead(&gi, 4, &br);
  gsys->FileClose();
  return gi;
}

void ROCODE save_val(char *fn, uint32_t gi) {
  gsys->FileOpen(fn, FILE_MODE_CREATE_ALWAYS | FILE_MODE_WRITE);
  gsys->FileWrite(&gi, 4);
  gsys->FileClose();
}

#define UIMODE_SELECT   0
#define UIMODE_GAME     1
#define UIMODE_MENU     2

uint32_t RWDATA uimode;
uint32_t RWDATA gamecnt;
uint16_t RWDATA gameidx;
uint16_t RWDATA gameodx;

const char RODATA savegameidx[] = "0:/gbc/gameidx.bin";

void ROCODE main_loop() {
  //volatile uint32_t ticks = gsys->GetMsTicks();
  //call_defer_funcs(ticks);
  if (ctrlmode) {
    uint32_t t = gsys->GetMsTicks();
    call_defer_funcs(t);
    gsys->SleepMs(1);
    return;
  }
  if (uimode == UIMODE_GAME)
    goto __loop;

  char gn[160];
  uint32_t new_ticks = gsys->GetMsTicks();
  uint32_t ticks = new_ticks - lastKeyTick;
  if (new_ticks < lastKeyTick)
    ticks += 26844;
  if (new_ticks == lastKeyTick)
    return;
  lastKeyTick = new_ticks;
  uint32_t k = 0, kd = gsys->GetKeys();

  for (int i = 0; i < 9; i++, kd >>= 1) {
    k >>= 1;
    if (kd & 1) {
      if (keyCounter[i] == 0)
        k |= 256;
      keyCounter[i] += ticks;
      if (keyCounter[i] > keyDelay) {
        k |= 256;
        keyCounter[i] -= keyRepeat;
      }
      continue;
    }
    keyCounter[i] = 0;
  }
  if (uimode == UIMODE_SELECT) {
    FILEINFO **fp = (FILEINFO **)(framebuffer + 20480);
    if (gameodx != gameidx) {
      load_image(fp[gameidx], framebuffer);
      gameodx = gameidx;
    }
    if (k & KEY_A) {
      strcpy(gn, "0:/gbc/");
      strcpy(&gn[7], fp[gameidx]->filename);
      int l = strlen(gn);
      strcpy(&gn[l - 3], "gbc");
      uimode = UIMODE_GAME;
      save_val(savegameidx, gameidx);

      gsys->Printf("Loading ROM:\n%s\n", gn+7);
      load_file2(gn, 0, 0x80000, 16384, (uint32_t *)framebuffer, 0x8000);
      load_file2(gn, 0, GAME_ROM_ADDR, GAME_ROM_SIZE, (uint32_t *)framebuffer, 0x8000);
#ifndef OCEMU
      gsys->FlashSwitch(0, 0);
      gsys->NoSDK();
      uint32_t text_start = (uint32_t)&_text_start;
      uint32_t text_end = (uint32_t)&_text_end;
      uint32_t iram_start = (uint32_t)&_iram1_text_start;
      uint32_t iram_end = (uint32_t)&_iram1_text_end;
      uint32_t src = (text_end - text_start) + 0x40258000;
      uint32_t len = iram_end - iram_start;
      gsys->Printf("Copy: %X %X\n", src, len);
      if (len > 0x8000) {
        gsys->Printf("Code too long!\n");
        while(1)
          gsys->SleepMs(100);
      }
      memcpy((void *)0x40102000, (void *)src, len);
      gsys->SleepMs = raw_sleep_ms;
      gsys->SleepUs = raw_sleep_us;
      gsys->GetMsTicks = get_raw_ticks;
      gsys->GetAudioBuffer = get_audio_buf;
      gsys->PollAudioBuffer = poll_audio;

      framebuffer = gsys->GetMemBlock(MEM_TYPE_RAM, 80 * 1024);
      gsys->Printf("Init.\n");
#endif
      boy_Init(gn);
      goto __loop;
    }
    if ((k & KEY_UP) && gameidx < gamecnt - 1)
      gameidx++;
    if ((k & KEY_DOWN) && gameidx > 0)
      gameidx--;
    if ((k & KEY_RIGHT) && gameidx < gamecnt - 1) {
      char c = fp[gameidx]->filename[0];
      while (gameidx < gamecnt - 1 && c == fp[gameidx]->filename[0])
        gameidx++;
    }
    if ((k & KEY_LEFT) && gameidx > 0) {
      char c = fp[gameidx]->filename[0];
      while (gameidx > 0 && c == fp[gameidx]->filename[0])
        gameidx--;
      c = fp[gameidx]->filename[0];
      while (gameidx > 0 && c == fp[gameidx]->filename[0])
        gameidx--;
      gameidx++;
    }
    return;
  }
__loop:
#ifndef OCEMU
  while(1) {
#endif
    boy_Step();
#ifndef OCEMU
    HARD_WDT_FEED
  }
#endif
}

void ROCODE sort_games(uint32_t cnt, FILEINFO **fp) {
  FILEINFO *fi = (FILEINFO *)(FLASH_ADDR(0xE0000));
  for (int i = 0; i < cnt; i++)
    fp[i] = &fi[i];

  for (int i = 0; i < cnt - 1; i++) {
    int min = i;
    for (int j = i + 1; j < cnt; j++)
      if (strcmp(fp[min]->filename, fp[j]->filename) > 0)
        min = j;
    FILEINFO *ft = fp[i];
    fp[i] = fp[min];
    fp[min] = ft;
  }
}

void ROCODE main_start() {
  gsys->GetBatteryLevel();
  if (ctrlmode) {
    //add_defer_func(0, 1000, timer_1s, 0);

    gsys->Printf("Bat: %d\n", gsys->GetBatteryLevel());
    gsys->SleepMs(500);

    static const char RODATA ssid[32] = "2WIRE774";
    static const char RODATA pwd[64] = "deadbeef73";
    gsys->WifiConnect((char *)ssid, (char *)pwd);
    gsys->Printf("%d %d\n", gsys->GetFreeMem(), gsys->GetDiskSpeed());
    return;
  }

  memset(keyCounter, 0, sizeof(keyCounter));
  lastKeyTick = 0;
  keyDelay = KEY_DELAY;
  keyRepeat = KEY_REPEAT;
  uimode = UIMODE_SELECT;
#ifdef OCEMU
  framebuffer = gsys->GetMemBlock(MEM_TYPE_RAM, 80 * 1024);
#else
  framebuffer = gsys->GetMemBlock(MEM_TYPE_RAM, 42 * 1024);
#endif
  memset(framebuffer, 0, 40960);
  FILEINFO *ff = (FILEINFO *)FLASH_ADDR(0xE0000);
  FILEINFO **fp = (FILEINFO **)(framebuffer + 20480);
  if (gsys->FileOpen("0:/boot/splash.raw", FILE_MODE_READ) == 0) {
    uint32_t br;
    gsys->FileRead(framebuffer, 40960, &br);
    gsys->FileClose();
    gsys->LcdMode();
    gsys->UpdateScreen16(framebuffer, 0, 127, 0);
#ifndef OCEMU
    uint32_t nk, k = gsys->GetKeys();
    while ((nk = gsys->GetKeys()) == k)
      gsys->SleepMs(10);
    while (nk == gsys->GetKeys())
      gsys->SleepMs(10);
#endif
    gsys->Printf("Loading...\n");
  }
  uint32_t bufsize = 512;
  int res = gsys->ReadDir("0:/gbc/images", ff, &bufsize);
  gsys->Printf("Games: %d\n", bufsize);
  sort_games(bufsize, fp);
  gamecnt = bufsize;
  gameidx = load_val(savegameidx);
  gameodx = 1;
}

void ROCODE wifi_status_callback(uint32_t event, WIFIEVENT *data) {
  gsys->Printf("%d %d %X\n", event, data->reason, data->ip);
  if (event == WIFI_EVENT_GOT_IP) {
    //gsys->WifiEnableMDNS(NULL);
    //gsys->WifiResolveName("www.google.com");
    //gsys->Printf("%d\n", gsys->GetFreeMem());
    //gsys->WifiEnableUDP(0xFF01A8C0, 5555);
    control_init();
  }
}

void ROCODE wifi_scan_callback(WIFINET *netbuf, uint32_t count) {

}

void ROCODE tcp_connect_callback(TCPCONN *conn) {
  control_connect_callback(conn);
}

void ROCODE tcp_reconnect_callback(TCPCONN *conn, int8_t error) {

}

void ROCODE tcp_disconnect_callback(TCPCONN *conn) {
  control_disconnect_callback(conn);
  //gsys->Printf("Done %X\n", gsys->WifiGetIP());
}

void ROCODE tcp_sent_callback(TCPCONN *conn) {
  control_sent_callback(conn);
}

void ROCODE tcp_recv_callback(TCPCONN *conn, void *data, uint32_t size) {
  control_recv_callback(conn, data, size);
}

extern void ROCODE gb_serial_recieve(uint8_t b);

void ROCODE udp_recv_callback(UDPCONN *conn, void *data, uint32_t size) {
  //gsys->Printf("UDP: %02X\n", *(uint8_t *)data);
}

void ROCODE dns_result_callback(const char *name, uint32_t ip) {
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
