#include <stdlib.h>
#include <string.h>
#include "oc.h"
#include "display.h"

SYSCALLS * RWDATA gsys;
extern uint16_t * RWDATA framebuffer;
uint16_t RWDATA keyCounter[8];
uint32_t RWDATA lastKeyTick;
uint16_t RWDATA keyDelay;
uint16_t RWDATA keyRepeat;

inline void xLine(int x0, int x1, int y, uint32_t color)
{
  if(x0 > x1)
  {
    int xb = x0;
    x0 = x1;
    x1 = xb;
  }
  if (x0 > SCREEN_WIDTH - 1)
    return;
  if (x1 < 0)
    return;
  if(x0 < 0) x0 = 0;
  if(x1 > SCREEN_WIDTH - 1) x1 = SCREEN_WIDTH - 1;

  uint8_t *dst = framebuffer + y * 160 + x0;

  if (x0 & 3) {
    int xd = 4 - (x0 & 3);
    for(int x = 0; x < xd; x++)
      *dst++ = color;
    x0 += xd;
  }
  uint32_t *d4 = (uint32_t *)dst;
  for(int x = x0; x < (x1 & 0xfffc); x += 4)
    *d4++ = color;
  dst = (uint8_t *)d4;

  for(int x = 0; x < (x1 & 3); x++)
    *dst++ = color;
}

void triangle(int16_t *p0, int16_t *p1, int16_t *p2, uint32_t color)
{
  if (p1[1] < p0[1]) {
    int16_t *vb = p0; p0 = p1; p1 = vb;
  }
  if (p2[1] < p1[1]) {
    int16_t *vb = p1; p1 = p2; p2 = vb;
  }
  if (p1[1] < p0[1]) {
    int16_t *vb = p0; p0 = p1; p1 = vb;
  }

  if (p0[1] > SCREEN_HEIGHT - 1)
    return;
  if (p2[1] < 0)
    return;

  int y = p0[1];
  int xac = p0[0] << 16;
  int xab = p0[0] << 16;
  int xbc = p1[0] << 16;
  int xaci = 0;
  int xabi = 0;
  int xbci = 0;
  if(p1[1] != p0[1])
    xabi = ((p1[0] - p0[0]) << 16) / (p1[1] - p0[1]);
  if(p2[1] != p0[1])
    xaci = ((p2[0] - p0[0]) << 16) / (p2[1] - p0[1]);
  if(p2[1] != p1[1])
    xbci = ((p2[0] - p1[0]) << 16) / (p2[1] - p1[1]);

  for(; y < p1[1] && y < SCREEN_HEIGHT; y++)
  {
    if(y >= 0)
      xLine(xab >> 16, xac >> 16, y, color);
    xab += xabi;
    xac += xaci;
  }
  for(; y < p2[1] && y < SCREEN_HEIGHT; y++)
  {
    if(y >= 0)
      xLine(xbc >> 16, xac >> 16, y, color);
    xbc += xbci;
    xac += xaci;
  }
}

void main_start2(SYSCALLS *sys) {
  gsys = sys;

  uint32_t t = sys->GetMsTicks();

  for (int j = 1; j < 1001; j++) {
    uint32_t *src = sys->GetMemBlock(0);
    uint32_t *dst = src;
    uint32_t s = *src++ + j;
    uint32_t c = *src++ + j;
    for (int i = 0; i < 1000; i++) {
      uint32_t x = *src++;
      uint32_t y = *src++;
      *dst++ = ((x * c + y * s) >> 8) / s;
      *dst++ = ((y * c - x * s) >> 8) / s;
    }
  }

  sys->Debug("bubu %d\n", 1000000000 / (sys->GetMsTicks() - t));
  while(1)
    sys->SleepMs(10);
}

#define RAND    (seed = (((seed ^ 0xBACADA55) + 0x9123) >> 11) ^ (seed + 0x7432))

void main_start3(SYSCALLS *sys) {
  gsys = sys;

  framebuffer = sys->GetMemBlock(0);
  uint16_t *pal = (uint16_t *)(framebuffer + 20 * 1024);
  uint32_t seed = 0x12345678;

  uint16_t *pd = pal;
  for (int i = 0; i < 256; i++) {
    *pd++ = seed;
    seed = (((seed ^ 0xBACADA55) + 0x9123) << 11) ^ (seed + 0x7432);
  }

  uint32_t t = sys->GetMsTicks();

  for (int i = 0; i < 10000; i++) {
    pd[0] = (RAND & 127) - 16;
    pd[1] = (RAND & 127) - 16;
    pd[2] = pd[0] + (RAND & 15);
    pd[3] = pd[1] + (RAND & 15);
    pd[4] = pd[0] + (RAND & 15);
    pd[5] = pd[1] + (RAND & 15);
    uint8_t c = RAND;
    triangle(&pd[0], &pd[2], &pd[4], c | (c << 8) | (c << 16) | (c << 24));
  }
  t = sys->GetMsTicks() - t;

  sys->UpdateScreen8(framebuffer, pal, 0, 127, 0);
  sys->Debug("bubu %d\n", 10000000 / t);
  while(1)
    sys->SleepMs(10);
}

void main_start4(SYSCALLS *sys) {
  gsys = sys;
  uint32_t *flag = sys->GetMemBlock(1);
  uint32_t br;

  if (*flag == 0) {
    framebuffer = sys->GetMemBlock(0);
    sys->DiskInit(0);
    sys->Debug("%d\n", sys->FileOpen("0:/kungfuhs.rgb", FILE_MODE_READ));
    sys->SleepMs(1000);

    *flag = 1;
  }

  for (int i = 0; i < 1000; i++) {
    uint32_t t = sys->GetMsTicks();
    sys->FileRead(framebuffer, 40*1024, &br);
    sys->UpdateScreen16(framebuffer, 0, 127, 0);
    uint32_t st = 100 - (sys->GetMsTicks() - t);
    if (st > 100)
      st = 1;
    sys->SleepMs(st);
    sys->Debug("%d\n", i);
  }
  while(1)
    sys->SleepMs(10);
}

void main_start5(SYSCALLS *sys) {
  gsys = sys;
  uint32_t *flag = sys->GetMemBlock(1);
  uint32_t *ptr = flag + 1;
  uint32_t *size = flag + 2;
  uint32_t br;
  uint32_t *sndbuf;

  if (*flag == 0) {
    framebuffer = sys->GetMemBlock(0);
    sys->DiskInit(0);
    sys->Printf("Bat: %d\n", sys->GetBatteryLevel());
    sys->Debug("%d\n", sys->FileOpen("0:/song2.raw", FILE_MODE_READ));

    int i = 0;
    *size = 0;
    while(sys->FileRead(framebuffer, 32768, &br) == 0) {
      if (br == 0)
        break;
      sys->RamMode();
      sys->RamWrite(i * 32768, framebuffer, 32768);
      sys->SleepMs(1);
      i++;
      *size = *size + br;
    }
    sys->FileClose();
    sys->Printf("Done.\n");

    //sys->SetCpuSpeed(80);
    sys->PlayAudio(22050);
    //sys->SetCpuSpeed(160);

    static const char RODATA ssid[32] = "2WIRE774";
    static const char RODATA pwd[64] = "deadbeef73";
    sys->WifiConnect((char *)ssid, (char *)pwd);

    *flag = 1;
    *ptr = 0;
  }

  while ((sndbuf = gsys->GetAudioBuffer()) != NULL) {
    //sys->FileRead(framebuffer, 256, &br);
    sys->RamMode();
    sys->RamRead(*ptr, framebuffer, 256);
    *ptr = *ptr + 256;
    if (*ptr > *size)
      *ptr = 0;

    int8_t *tmpptr = (int8_t *)framebuffer;
    for (int i = 0; i < 256; i++) {
      int16_t t0 = *tmpptr++ << 7;
      *sndbuf++ = t0; //(t0 & 65535) | (t0 << 16);
    }
  }
}

void makevolmap(uint32_t *volmap, uint32_t vol) {
  for (int i = 0; i < 256; i++) {
    int8_t s = i;
    int vs = s * vol;
    if (vs > 32767)
      vs = 32767;
    if (vs < -32767)
      vs = -32767;
    *volmap++ = (vs & 65535) | (vs << 16);
  }
}

void main_start6(SYSCALLS *sys) {
  gsys = sys;
  uint32_t *flag = sys->GetMemBlock(1);
  uint32_t *asize = flag + 1;
  uint32_t *vol = flag + 2;
  uint32_t *volmap = flag + 4;
  uint8_t *abuf = (uint8_t *)(flag + 4 + 256);
  uint32_t br;
  uint32_t *sndbuf;

  if (*flag == 0) {
    framebuffer = sys->GetMemBlock(0);
    memset(keyCounter, 0, sizeof(keyCounter));
    lastKeyTick = 0;
    keyDelay = KEY_DELAY;
    keyRepeat = KEY_REPEAT;

    sys->DiskInit(0);
    sys->Printf("Bat: %d\n", sys->GetBatteryLevel());
    sys->Printf("%d\n", sys->FileOpen("0:/kungfuhs3.ltv", FILE_MODE_READ));

    //sys->SetCpuSpeed(80);
    sys->PlayAudio(22050);
    //sys->SetCpuSpeed(160);

    static const char RODATA ssid[32] = "2WIRE774";
    static const char RODATA pwd[64] = "deadbeef73";
    sys->WifiConnect((char *)ssid, (char *)pwd);

    *flag = 1;
    *asize = 0;
    *vol = 1;
    makevolmap(volmap, *vol);
  }

  while(1){
    uint32_t as = 0;
    uint32_t fr = 0;
    uint8_t *ab = (uint8_t *)abuf;

    if (*asize < 2048) {
//    if (*asize < 1024) {
      sys->FileRead(&as, 4, &br);
      if (br == 0 || as == 0)
        return;
      sys->FileRead(abuf + *asize, as, &br);
      *asize = *asize + br;
      fr = 1;
    }
    while ((sndbuf = gsys->GetAudioBuffer()) != NULL && *asize >= 512) {
      int16_t *tmpptr = (int16_t *)ab;
//    while ((sndbuf = gsys->GetAudioBuffer()) != NULL && *asize >= 256) {
//      uint8_t *tmpptr = (uint8_t *)ab;
      for (int i = 0; i < 256; i++) {
        int32_t t0 = *tmpptr++;
        t0 *= *vol;
        t0 >>= 2;
        if (t0 < -32767)
          t0 = -32767;
        if (t0 > 32767)
          t0 = 32767;
        *sndbuf++ = t0 & 65535;//) | (t0 << 16);
        //*sndbuf++ = volmap[*tmpptr++];
      }
      ab += 512;
      *asize = *asize - 512;
//      ab += 256;
//      *asize = *asize - 256;
    }
    if (*asize > 0 && abuf != ab) {
      memcpy(abuf, ab, *asize);
    }
    uint32_t ticks = sys->GetMsTicks();
    if (ticks != lastKeyTick) {
      ticks -= lastKeyTick;
      lastKeyTick += ticks;
      uint32_t k = 0, kd = sys->GetKeys();

      for (int i = 0; i < 8; i++, kd >>= 1) {
        k >>= 1;
        if (kd & 1) {
          if (keyCounter[i] == 0)
            k |= 128;
          keyCounter[i] += ticks;
          if (keyCounter[i] > keyDelay) {
            k |= 128;
            keyCounter[i] -= keyRepeat;
          }
          continue;
        }
        keyCounter[i] = 0;
      }

      if (k == KEY_UP && *vol < 1500) {
        *vol += 1;
        sys->Debug("V: %d\n", *vol);
        makevolmap(volmap, *vol);
      }
      if (k == KEY_DOWN && *vol > 1) {
        *vol -= 1;
        sys->Debug("V: %d\n", *vol);
        makevolmap(volmap, *vol);
      }
    }
    if (fr != 0) {
      sys->FileRead(framebuffer, 40960, &br);
      sys->UpdateScreen16(framebuffer, 0, 127, 0);
      break;
    }
  }
}

uint32_t RWDATA lastTick;
uint32_t RWDATA flag;
uint32_t RWDATA wifistat;

void main_start(SYSCALLS *sys) {
  gsys = sys;
  uint32_t br;

  framebuffer = sys->GetMemBlock(0);
  memset(keyCounter, 0, sizeof(keyCounter));
  lastKeyTick = 0;
  keyDelay = KEY_DELAY;
  keyRepeat = KEY_REPEAT;
  flag = 0;
  lastTick = sys->GetMsTicks();
  wifistat = 0;

  sys->DiskInit(0);
  sys->Printf("Bat: %d\n", sys->GetBatteryLevel());
  sys->SleepMs(500);

  DEBUG("init\n");
  display_init(framebuffer);
  DEBUG("text\n");
  display_draw_text(20, 50, "Test Text 123");
  DEBUG("update\n");
  gsys->UpdateScreen16(framebuffer, 0, 127, 0);

  static const char RODATA ssid[32] = "2WIRE774";
  static const char RODATA pwd[64] = "deadbeef73";
  gsys->WifiConnect((char *)ssid, (char *)pwd);
}

void main_loop() {
  uint32_t ticks = gsys->GetMsTicks();
  //flag = 1;

  /*if ((ticks - lastTick) > 10000) {

    lastTick = ticks;
    gsys->Printf("%d %X\n", gsys->GetFreeMem(), gsys->WifiGetIP());
  }*/
  //gsys->SleepMs(20);
  gsys->UpdateScreen16(framebuffer, 0, 127, 0);
  gsys->Printf("%d %d\n", gsys->GetFreeMem(), flag);
  //gsys->Printf("%d %X\n", gsys->GetFreeMem(), gsys->WifiGetIP());
  //flag = 0;
}

void wifi_status_callback(uint32_t event, WIFIEVENT *data) {
    gsys->Printf("%d %d %d %X\n", event, data->reason, flag, data->ip);
  if (event == WIFI_EVENT_GOT_IP) {
    gsys->WifiListenTCP(8080);
  }
}

void wifi_scan_callback(WIFINET *netbuf, uint32_t count) {

}

void tcp_connect_callback(TCPCONN *conn) {

}

void tcp_reconnect_callback(TCPCONN *conn, int8_t error) {

}

void tcp_disconnect_callback(TCPCONN *conn) {
  gsys->FileClose();
  gsys->Printf("%d %X\n", wifistat, gsys->WifiGetIP());
  flag = wifistat;
  wifistat = 0;
}

void tcp_sent_callback(TCPCONN *conn) {

}

void tcp_recv_callback(TCPCONN *conn, void *data, uint32_t size) {
  if (wifistat == 0) {
    gsys->FileOpen("0:/boot/test.mp3", FILE_MODE_CREATE_ALWAYS | FILE_MODE_WRITE);
    gsys->Printf("%d %X\n", wifistat, gsys->WifiGetIP());
  }
  gsys->FileWrite(data, size);
  wifistat += size;
}

void udp_recv_callback(UDPCONN *conn, void *data, uint32_t size) {

}

#define OCDATA __attribute__((section(".irom2.text"))) STORE_ATTR

const OCCALLS OCDATA __calltable = {
  .MainStart = main_start,
  .MainLoop = main_loop,
  .WifiStatusCallback = wifi_status_callback,
  .WifiScanCallback = wifi_scan_callback,
  .TcpConnectCallback = tcp_connect_callback,
  .TcpReconnectCallback = tcp_reconnect_callback,
  .TcpDisconnectCallback = tcp_disconnect_callback,
  .TcpSentCallback = tcp_sent_callback,
  .TcpRecvCallback = tcp_recv_callback,
  .UdpRecvCallback = udp_recv_callback
};
