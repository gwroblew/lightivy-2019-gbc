#define __USE_LARGEFILE64     1
#define __USE_FILE_OFFSET64   1

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <pthread.h>

#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <netinet/in.h>
#include <dirent.h>
#include <error.h>
#include "SDL2/SDL.h"
#include "SDL2/SDL_ttf.h"
#include "SDL2/SDL_hints.h"
#include <SDL2/SDL_scancode.h>
#include <SDL2/SDL_net.h>

#include "system.h"

#define SCREEN_WIDTH  (160)
#define SCREEN_HEIGHT (128)
#define SCREEN_SCALE  (4)

#define WINDOW_PANEL_HEIGHT   (64)
#define WINDOW_WIDTH  (SCREEN_WIDTH * SCREEN_SCALE)
#define WINDOW_HEIGHT (SCREEN_HEIGHT * SCREEN_SCALE + WINDOW_PANEL_HEIGHT)

int quit = 0;
int frame_count = 0;
float frame_rate = 0.0;
uint32_t ticks_ms;
uint32_t netid;

SDL_Window *window = NULL;
SDL_Renderer *renderer = NULL;
uint8_t *pixels;
SDL_Texture *texture = NULL;
SDL_Surface *surface = NULL;
uint16_t *framebuf;
SDL_Texture *ftexture = NULL;
SDL_Surface *fsurface = NULL;
TTF_Font* fontsans14;
TTF_Font* fontmono14;
SDL_Color whiteColor = { 255, 255, 255 };
SDL_Color blackColor = { 0, 0, 0 };
const char *keymap[1024];

uint32_t gettick()
{
  struct timeval tv;

  gettimeofday(&tv, NULL);

  return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

///////////////////////////////////////////////////////////////////////////////////////////

uint32_t keyState = 0;
uint32_t keyPressed = 0;
uint32_t keyReleased = 0;

static inline void handleKey(uint32_t type, uint32_t bit) {
  if (type == SDL_KEYDOWN) {
    keyState |= bit;
    keyPressed |= bit;
  } else {
    keyState &= ~(bit);
    keyReleased |= bit;
  }
}

uint32_t get_keys() {
  return keyState;
}

uint32_t keysGetPressChange() {
  uint32_t rv = keyPressed;
  keyPressed = 0;
  return rv;
}

uint32_t keysGetReleaseChange() {
  uint32_t rv = keyReleased;
  keyReleased = 0;
  return rv;
}

///////////////////////////////////////////////////////////////////////////////////////////

SDL_AudioSpec audioSpec;
SDL_AudioDeviceID audioDev;
uint32_t audiobuf[4][AUDIO_BUFFER_SIZE];
uint32_t readbuf = 0, writebuf = 0;

void audioCallback(void *userdata, uint8_t *stream, int len) {
  if (len > 1024)
    len = 1024;
  memcpy(stream, audiobuf[readbuf], len);
  readbuf = (readbuf + 1) & 3;
}

void audioInitialize() {
  audioSpec.callback = audioCallback;
  //audioSpec.callback = NULL;
  audioSpec.channels = 2;
  audioSpec.format = AUDIO_S16;
  audioSpec.freq = 22050;
  audioSpec.padding = 0;
  audioSpec.samples = AUDIO_BUFFER_SIZE;
  audioSpec.silence = 0;
  audioSpec.size = AUDIO_BUFFER_SIZE * 4;
  audioSpec.userdata = 0;
}

uint32_t play_audio(uint32_t rate, SOUNDSYS *ss) {
  if (rate < 11800 || rate > 48000)
    return 0;

  audioSpec.freq = rate;
  audioDev = SDL_OpenAudioDevice(NULL, 0, &audioSpec, NULL, 0);
  SDL_PauseAudioDevice(audioDev, 0);
  return AUDIO_BUFFER_SIZE;
}

void stop_audio() {
  SDL_PauseAudioDevice(audioDev, 1);
}

uint32_t *get_audio_buffer() {
  if (readbuf == writebuf)
    return NULL;

  uint32_t b = writebuf;
  writebuf = (writebuf + 1) & 3;
  return audiobuf[b];
}

///////////////////////////////////////////////////////////////////////////////////////////

uint32_t get_ms_ticks() {
  return ticks_ms;
}

void sleep_ms(uint32_t ms) {
  SDL_Delay(ms);
}

void sleep_us(uint32_t us) {
  sleep_ms(us / 1000);
}

void reset(uint32_t mode) {
  printf("########################## RESET #########################\n");
  printf("######################### mode: %d #########################\n", mode);
}

#define ALIGN_CHECK(addr)     \
  { uint64_t a = (uint64_t)addr; if (a & 3) printf("!!! Memory alignment error: 0x%lX, at %s !!!\n", a, __func__); }

///////////////////////////////////////////////////////////////////////////////////////////

static uint32_t fpscnt = 0;
static uint32_t lasttick;
static float lastfps;

void set_screen(uint32_t y1, uint32_t y2) {
}

void send_screen_data(uint32_t *buf, uint32_t size) {
}

void update_frame() {
  SDL_Rect srcrect = { 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT };
  SDL_Rect dstrect = { 0, 0, SCREEN_WIDTH * SCREEN_SCALE, SCREEN_HEIGHT * SCREEN_SCALE };
  SDL_BlitScaled(fsurface, &srcrect, surface, &dstrect);
  SDL_UpdateTexture(texture, NULL, pixels, WINDOW_WIDTH * 4);
  SDL_RenderClear(renderer);
  SDL_RenderCopy(renderer, texture, NULL, NULL);
  SDL_RenderPresent(renderer);
}

void clear_screen(uint16_t c) {
  uint16_t *fb = framebuf;

  for (int i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; i++)
    *fb++ = c;
  fpscnt++;
  update_frame();
}

void update_screen16(uint16_t *fb, uint32_t y1, uint32_t y2, int clear) {
  ALIGN_CHECK(fb)
  SDL_Rect srcrect = { 0, y1, SCREEN_WIDTH, y2 + 1 };
  SDL_Rect dstrect = { 0, y1, SCREEN_WIDTH * SCREEN_SCALE, (y2 + 1) * SCREEN_SCALE - 1 };

  //SDL_UpdateTexture(ftexture, NULL, fb, SCREEN_WIDTH * 2);
  //memcpy(framebuf + y1 * SCREEN_WIDTH, fb + y1 * SCREEN_WIDTH, SCREEN_WIDTH * (y2 - y1) * 2);
  uint16_t *src = fb + y1 * SCREEN_WIDTH;
  uint16_t *dst = framebuf + y1 * SCREEN_WIDTH;
  for (int i = 0; i < (y2 - y1 + 1) * SCREEN_WIDTH; i++) {
    uint16_t d = *src++;
    d = (d << 8) | (d >> 8);
    *dst++ = d;
  }
  SDL_BlitScaled(fsurface, &srcrect, surface, &dstrect);
  SDL_UpdateTexture(texture, NULL, pixels, WINDOW_WIDTH * 4);
  SDL_RenderClear(renderer);
  SDL_RenderCopy(renderer, texture, NULL, NULL);
  SDL_RenderPresent(renderer);
  fpscnt++;
  if (clear)
    memset(fb, 0, (y2 - y1 + 1) * SCREEN_WIDTH * 2);
}

void update_screen8(uint8_t *fb, uint16_t *pb, uint32_t y1, uint32_t y2, int clear) {
  ALIGN_CHECK(fb)
  ALIGN_CHECK(pb)
  SDL_Rect srcrect = { 0, y1, SCREEN_WIDTH, y2 + 1 };
  SDL_Rect dstrect = { 0, y1, SCREEN_WIDTH * SCREEN_SCALE, (y2 + 1) * SCREEN_SCALE - 1 };
  uint16_t pal[256];

  uint16_t *sr = pb;
  uint16_t *ds = pal;
  for (int i = 0; i < 256; i++) {
    uint16_t d = *sr++;
    d = (d << 8) | (d >> 8);
    *ds++ = d;
  }

  //SDL_UpdateTexture(ftexture, NULL, fb, SCREEN_WIDTH * 2);
  //memcpy(framebuf + y1 * SCREEN_WIDTH, fb + y1 * SCREEN_WIDTH, SCREEN_WIDTH * (y2 - y1) * 2);
  uint8_t *src = fb + y1 * SCREEN_WIDTH;
  uint16_t *dst = framebuf + y1 * SCREEN_WIDTH;
  uint32_t p1;
  uint32_t pl[160];
  for (int i = 0, j = 0; i < (y2 - y1 + 1) * SCREEN_WIDTH; i++) {
    *dst++ = pal[*src++];
    /*uint32_t p2 = p1;
    p1 = pal[*src++];
    p1 = ((p1 & 0xF800) << 6) | ((p1 & 0x7E0) << 3) | ((p1 & 0x1F) << 1);
    p1 = ((p1 * 2 + p2 + pl[j]) >> 2) & 0x3F3F3F;
    //p1 = ((p1 & 0xF800) << 8) | ((p1 & 0x7E0) << 4) | ((p1 & 0x1F) << 1);
    //p1 = ((p1 * 3 + p2 * 3 + pl[j] * 2) >> 3) & 0xFC7E3F;
    pl[j] = p1;
    *dst++ = ((p1 & 0x3E0000) >> 6) | ((p1 & 0x3F00) >> 3) | ((p1 & 0x3E) >> 1);
    //*dst++ = ((p1 & 0xF80000) >> 8) | ((p1 & 0x7E00) >> 4) | ((p1 & 0x3E) >> 1);
    j++;
    if (j == 160)
      j = 0;*/
  }
  SDL_BlitScaled(fsurface, &srcrect, surface, &dstrect);
  SDL_UpdateTexture(texture, NULL, pixels, WINDOW_WIDTH * 4);
  SDL_RenderClear(renderer);
  SDL_RenderCopy(renderer, texture, NULL, NULL);
  SDL_RenderPresent(renderer);
  fpscnt++;
  if (clear)
    memset(fb, 0, (y2 - y1 + 1) * SCREEN_WIDTH);
}

void update_screen4(uint8_t *fb, uint16_t *pb, uint32_t y1, uint32_t y2, int clear) {
  ALIGN_CHECK(fb)
  ALIGN_CHECK(pb)
  SDL_Rect srcrect = { 0, y1, SCREEN_WIDTH, y2 + 1 };
  SDL_Rect dstrect = { 0, y1, SCREEN_WIDTH * SCREEN_SCALE, (y2 + 1) * SCREEN_SCALE - 1 };
  uint16_t pal[256];

  uint16_t *sr = pb;
  uint16_t *ds = pal;
  for (int i = 0; i < 256; i++) {
    uint16_t d = *sr++;
    d = (d << 8) | (d >> 8);
    *ds++ = d;
  }

  uint8_t *src = fb + y1 * SCREEN_WIDTH / 2;
  uint16_t *dst = framebuf + y1 * SCREEN_WIDTH;
  for (int i = 0; i < (y2 - y1 + 1) * SCREEN_WIDTH; i++) {
    uint32_t p1 = *src++;
    uint32_t p2 = p1 & 15;
    *dst++ = pal[p1 >> 4];
    *dst++ = pal[p2];
  }
  SDL_BlitScaled(fsurface, &srcrect, surface, &dstrect);
  SDL_UpdateTexture(texture, NULL, pixels, WINDOW_WIDTH * 4);
  SDL_RenderClear(renderer);
  SDL_RenderCopy(renderer, texture, NULL, NULL);
  SDL_RenderPresent(renderer);
  fpscnt++;
  if (clear)
    memset(fb, 0, (y2 - y1 + 1) * SCREEN_WIDTH / 2);
}

float displayGetFPS() {
  uint32_t diff = ticks_ms - lasttick;
  if (diff == 0)
    return 0.0f;
  if (diff < 1000)
    return lastfps;
  lasttick = ticks_ms;
  lastfps = (float)fpscnt * 1000 / diff;
  fpscnt = 0;
  return lastfps;
}

void lcd_mode(){
}
void ram_mode(){
}
void sd_mode(){
}
void set_cpu_speed(uint32_t mhz){
}

uint32_t dramsize = 0;
uint32_t iramsize = 0;
uint8_t flashmem[4*1024*1024];
uint8_t spiram[8*1024*1024];

void *get_mem_block(uint32_t type, uint32_t size){
  if (type == MEM_TYPE_RAM) {
    dramsize += size;
    if (dramsize > 52 * 1024)
      printf("Out of DRAM: %d %d\n", dramsize, size);
  } else {
    iramsize += size;
    if (iramsize > 16 * 1024)
      printf("Out of IRAM: %d %d\n", dramsize, size);
  }
  return malloc(size);
}

uint32_t get_free_mem() {
  return 12345;
}

// More aggresive:  || (addr & 4095) != 0 || (size & 4095) != 0
#define CHECK_FLASH_BLOCK(addr, size)   \
  if ((addr + size) > 4*1024*1024 || (addr & 3) != 0 || (size & 3) != 0) {  \
    printf("Illegal flash block read: %0x %0x\n", addr, size);  \
    return 1;   \
  }

int flash_read_mem(uint32_t addr, void *data, uint32_t size) {
  ALIGN_CHECK(addr)
  ALIGN_CHECK(data)
  CHECK_FLASH_BLOCK(addr, size)
  memcpy(data, flashmem + addr, size);
  return 0;
}

int flash_write_mem(uint32_t addr, void *data, uint32_t size) {
  ALIGN_CHECK(addr)
  ALIGN_CHECK(data)
  CHECK_FLASH_BLOCK(addr, size)
  memcpy(flashmem + addr, data, size);
  return 0;
}

int flash_erase(uint32_t addr, uint32_t size) {
  ALIGN_CHECK(addr)
  CHECK_FLASH_BLOCK(addr, size)
  /*if ((addr & 65535) != 0 || (size & 65535) != 0) {
    printf("Illegal flash block erase: %0x %0x\n", addr, size);
    return 1;
  }*/
  memset(flashmem + addr, 0, size);
  return 0;
}

uint32_t get_battery_level(){
  return 1021;
}

int disk_init(){
  return 0;
}

uint32_t get_disk_speed() {
  return 20;
}

FILE *fh = NULL;

int file_open(char *fname, uint32_t mode) {
  ALIGN_CHECK(fname)
  char fn[1024];
  strcpy(fn, "sdcard");
  strcat(fn, fname + 2);
  if (mode & 1)
    fh = fopen(fn, "rb");
  else
    fh = fopen(fn, "wb");
  if (fh != NULL)
    return 0;
  return 1;
}

int file_read(void *buffer, uint32_t size, uint32_t *read) {
  ALIGN_CHECK(buffer)
  ALIGN_CHECK(size)
  if (fh == NULL)
    return 1;
  uint32_t br = fread(buffer, 1, size, fh);
  *read = br;
  if (br != 0)
    return 0;
  return 1;
}

int file_write(void *buffer, uint32_t size) {
  ALIGN_CHECK(buffer)
  ALIGN_CHECK(size)
  if (fh == NULL)
    return 1;
  uint32_t br = fwrite(buffer, 1, size, fh);
  if (br != 0)
    return 0;
  return 1;
}

int file_close() {
  if (fh != NULL) {
    fclose(fh);
    fh = NULL;
  }
}

uint64_t file_size() {
  if (fh == NULL)
    return 0;
  uint64_t pos = ftell(fh);
  fseek(fh, 0, SEEK_END);
  uint64_t size = ftell(fh);
  fseek(fh, pos, SEEK_SET);
  return size;
}

int file_seek(uint64_t offset) {
  if (fh == NULL)
    return 1;
  fseek(fh, offset, SEEK_SET);
  return 0;
}

int delete_file(char *fname) {
  ALIGN_CHECK(fname)
  return 0;
}

int make_dir(char *fname) {
  ALIGN_CHECK(fname)
  return 0;
}

int read_dir(char *apath, FILEINFO *filebuf, uint32_t *bufsize) {
  ALIGN_CHECK(apath)
  ALIGN_CHECK(filebuf)
  char path[1000];
  strcpy(path, "sdcard");
  strcat(path, apath + 2);
  DIR *dp;
  struct dirent *files;
  *bufsize = 0;
  int i = 0;
  if((dp=opendir(path))==NULL)
    return 1;
  char newp[1000];
  struct stat buf;
  memset(filebuf, 255, *bufsize * sizeof(FILEINFO));
  while((files=readdir(dp))!=NULL)
  {
    if(!strcmp(files->d_name,".") || !strcmp(files->d_name,".."))
      continue;

    strcpy(newp,path);
    strcat(newp,"/");
    strcat(newp,files->d_name); 

    if(stat(newp,&buf)==-1)
      continue;
    if(S_ISDIR(buf.st_mode))
    {
      filebuf[i].flags = FILE_FLAG_DIR;
    }
    strncpy(filebuf[i].filename, files->d_name, FILEINFO_FILENAME_SIZE);
    filebuf[i].filename[FILEINFO_FILENAME_SIZE - 1] = 0;
    i++;
  }
  *bufsize = i;
  return 0;
}

const char *get_disk_error(int error) {
  if (error == 0)
    return "OK";
  return "Disk Error";
}

#define CHECK_RAM_BLOCK(addr, size)   \
  if ((addr + size) > 8*1024*1024) {  \
    printf("Illegal SPI RAM block read: %0x %0x\n", addr, size);  \
    return;   \
  }

uint32_t ramstart, ramsize;
void *ramdata;

void ram_read_start(uint32_t addr, uint32_t size){
  ALIGN_CHECK(addr)
  CHECK_RAM_BLOCK(addr, size)
  ramstart = addr;
}

void ram_read_end(void *data, uint32_t size){
  ALIGN_CHECK(data)
  memcpy(data, spiram + ramstart, size);
}

void ram_read(uint32_t addr, void *data, uint32_t size) {
  ALIGN_CHECK(addr)
  ALIGN_CHECK(data)
  CHECK_RAM_BLOCK(addr, size)
  memcpy(data, spiram + addr, size);
}

void ram_write_start(uint32_t addr, void *data, uint32_t size) {
  ALIGN_CHECK(addr)
  ALIGN_CHECK(data)
  CHECK_RAM_BLOCK(addr, size)
  ramstart = addr;
  ramdata = data;
  ramsize = size;
}

void ram_write_end() {
  memcpy(spiram + ramstart, ramdata, ramsize);
}

void ram_write(uint32_t addr, void *data, uint32_t size) {
  ALIGN_CHECK(addr)
  ALIGN_CHECK(data)
  memcpy(spiram + addr, data, size);
}

uint32_t GetPrimaryIp() {
    //assert(buflen >= 16);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    //assert(sock != -1);

    uint16_t kDnsPort = 53;
    struct sockaddr_in serv;
    memset(&serv, 0, sizeof(serv));
    serv.sin_family = AF_INET;
    serv.sin_addr.s_addr = 0x08080808;
    serv.sin_port = htons(kDnsPort);

    int err = connect(sock, (const struct sockaddr*) &serv, sizeof(serv));
    //assert(err != -1);

    struct sockaddr_in name;
    socklen_t namelen = sizeof(name);
    err = getsockname(sock, (struct sockaddr*) &name, &namelen);
    //assert(err != -1);

    close(sock);
    return name.sin_addr.s_addr;
}

extern const OCCALLS __calltable;

#define MAX_SOCKETS     (16)

uint32_t wifi_current_status = WIFI_STATUS_IDLE;
uint32_t wifi_scan_tick = 0;
uint32_t wifi_connect_tick = 0;
uint32_t wifi_disconnect_tick = 0;
uint32_t wifi_client_tick = 0;
uint16_t fake_local_port = 100;
SDLNet_SocketSet socket_set;
TCPsocket client_socket = NULL;
TCPCONN tcpclient;
uint16_t client_local_port;
UDPsocket udpsock = NULL;
UDPpacket *udppack = NULL;
uint32_t udp_ip = 0;
uint32_t udp_port = 0;

void BuildTcpClientConn(TCPsocket socket, uint16_t port) {
  IPaddress *ip = SDLNet_TCP_GetPeerAddress(socket);  
  tcpclient.remote_ip = ip->host;
  tcpclient.remote_port = ip->port;
  tcpclient.local_port = port;
}

int wifi_scan(WIFINET *netbuf, uint32_t bufsize) {
  ALIGN_CHECK(netbuf)
  wifi_scan_tick = gettick();
  return 0;
}

int wifi_connect(char *ssid, char *pwd){
  ALIGN_CHECK(ssid)
  wifi_connect_tick = gettick();
  printf("Connecting to WiFi: %s\n", ssid);
  return 0;
}

int wifi_disconnect() {
  wifi_disconnect_tick = gettick();
  return 0;
}

uint32_t wifi_get_ip() {
  return GetPrimaryIp();
}

uint32_t wifi_get_signal() {
  return 15;
}

UDPsocket udpclsock;

int wifi_enable_udp(uint32_t ip, uint32_t port) {

  if (netid == 1)
    port = 5552;
  else
    port = 5551;


  udpsock = SDLNet_UDP_Open(port);
  if (udpsock == NULL)
    return 1;
  udp_ip = ip;
  udp_port = port;
  udppack = SDLNet_AllocPacket(256);
  udppack->address.host = ip;
  udppack->address.port = port;

  udpclsock = SDLNet_UDP_Open(0);
  if (udpclsock == NULL)
    printf("BUBUGUGU\n");
  if (netid == 1) {
    udppack->address.host = 0x0100007F;
    udppack->address.port = 5551;
  } else {
    udppack->address.host = 0x0100007F;
    udppack->address.port = 5552;
  }

  udppack->channel = -1;
  IPaddress addr;
  addr.host = ip;
  addr.port = port;

  if (netid == 1) {
    addr.host = 0x0100007F;
    addr.port = 5551;
  } else {
    addr.host = 0x0100007F;
    addr.port = 5552;
  }

  //SDLNet_UDP_Bind(udpsock, 0, &addr);
  return 0;
}

int wifi_disable_udp() {
  if (udpsock == NULL)
    return 0;
  SDLNet_UDP_Unbind(udpsock, 0);
  SDLNet_FreePacket(udppack);
  SDLNet_UDP_Close(udpsock);
  udpsock = NULL;
  return 0;
}

int wifi_send_udp(void *data, uint32_t size) {
  //ALIGN_CHECK(data)
  if (udpsock == NULL)
    return 1;
  if (size > 220) {
    printf("UDP packet size TOO BIG: %d\n", size);
    size = 220;
  }
  memcpy(udppack->data, data, size);
  udppack->len = size;
  udppack->status = 0;

  if (netid == 1) {
    udppack->address.host = 0x0100007F;
    udppack->address.port = (5551 >> 8) | (5551 << 8);
  } else {
    udppack->address.host = 0x0100007F;
    udppack->address.port = (5552 >> 8) | (5552 << 8);
  }

  return !SDLNet_UDP_Send(udpclsock, -1, udppack);
}

int wifi_connect_tcp(uint32_t ip, uint32_t port) {
  IPaddress ipa = { .host = ip, .port = htons(port) };
  client_socket = SDLNet_TCP_Open(&ipa);
  if (client_socket == NULL)
    return 1;
  client_local_port = fake_local_port++;
  SDLNet_TCP_AddSocket(socket_set, client_socket);
  BuildTcpClientConn(client_socket, client_local_port);
  __calltable.TcpConnectCallback(&tcpclient);
  return 0;
}

void close_client_socket() {
  if (client_socket != NULL) {
    SDLNet_TCP_DelSocket(socket_set, client_socket);
    SDLNet_TCP_Close(client_socket);
    BuildTcpClientConn(client_socket, client_local_port);
    __calltable.TcpDisconnectCallback(&tcpclient);
  }
  client_socket = NULL;
}

int wifi_disconnect_tcp() {
  if (client_socket != NULL)
    close_client_socket();
  return 0;
}

int wifi_send_tcp(void *data, uint32_t size) {
  ALIGN_CHECK(data)
  if (client_socket != NULL) {
    uint32_t sent = SDLNet_TCP_Send(client_socket, data, size);
    if (sent < size) {
      close_client_socket();
    } else {
      wifi_client_tick = gettick();
    }
    return 0;
  }
  return 1;
}

uint32_t wifi_status() {
  return wifi_current_status;
}

int wifi_resolve_name(char *domain) {
  IPaddress ip;
  if (SDLNet_ResolveHost(&ip, domain, 80) == 0) {
    __calltable.DnsResultCallback(domain, ip.host);
  }
  return 0;
}

int wifi_enable_mdns(char *name) {
}

int wifi_disable_mdns(char *name) {
}

int wifi_set_hostname(char *name) {
}

const char *wifi_get_error(int error) {
  if (error == 0)
    return "OK";
  return "Network error";
}

uint32_t time_offset = 0;

uint32_t get_time(TIME *datetime) {
  time_t t = time(NULL) + time_offset;

  if (datetime != NULL) {
    struct tm *dt = gmtime(&t);
    datetime->tm_year = dt->tm_year;
    datetime->tm_mon = dt->tm_mon;
    datetime->tm_mday = dt->tm_mday;
    datetime->tm_hour = dt->tm_hour;
    datetime->tm_min = dt->tm_min;
    datetime->tm_sec = dt->tm_sec;
    datetime->tm_yday = dt->tm_yday;
    datetime->tm_wday = dt->tm_wday;
  }

  return t;
}

void set_time(uint32_t time_s) {
  time_offset = time_s - time(NULL);
}

void flash_switch(uint32_t n, uint32_t c) {
}

SYSCALLS syscalls = {
  .Debug = printf,
  .Reset = reset,
  .FlashSwitch = flash_switch,
  .SleepMs = sleep_ms,
  .SleepUs = sleep_us,
  .GetMsTicks = get_ms_ticks,
  .GetMemBlock = get_mem_block,
  .GetFreeMem = get_free_mem,
  .SetCpuSpeed = set_cpu_speed,
  .LcdMode = lcd_mode,
  .SdMode = sd_mode,
  .RamMode = ram_mode,
  .RamRead = ram_read,
  .RamWrite = ram_write,
  .RamReadStart = ram_read_start,
  .RamReadEnd = ram_read_end,
  .RamWriteStart = ram_write_start,
  .RamWriteEnd = ram_write_end,
  .FlashRead = flash_read_mem,
  .FlashWrite = flash_write_mem,
  .FlashErase = flash_erase,
  .GetKeys = get_keys,
  .ClearScreen = clear_screen,
  .UpdateScreen16 = update_screen16,
  .UpdateScreen8 = update_screen8,
  .UpdateScreen4 = update_screen4,
  .SetScreen = set_screen,
  .SendScreenData = send_screen_data,
  .Printf = printf,
  .Snprintf = snprintf,
  .Vsnprintf = vsnprintf,
  .PlayAudio = play_audio,
  .StopAudio = stop_audio,
  .GetAudioBuffer = get_audio_buffer,
  .PollAudioBuffer = lcd_mode,
  .GetBatteryLevel = get_battery_level,
  .DiskInit = disk_init,
  .GetDiskSpeed = get_disk_speed,
  .FileOpen = file_open,
  .FileRead = file_read,
  .FileWrite = file_write,
  .FileSeek = file_seek,
  .FileSize = file_size,
  .FileClose = file_close,
  .ReadDir = read_dir,
  .DeleteFile = delete_file,
  .DeleteDir = delete_file,
  .MakeDir = make_dir,
  .GetDiskError = get_disk_error,
  .WifiScan = wifi_scan,
  .WifiConnect = wifi_connect,
  .WifiDisconnect = wifi_disconnect,
  .WifiGetIP = wifi_get_ip,
  .WifiGetSignal = wifi_get_signal,
  .WifiEnableUDP = wifi_enable_udp,
  .WifiDisableUDP = wifi_disable_udp,
  .WifiSendUDP = wifi_send_udp,
  .WifiConnectTCP = wifi_connect_tcp,
  .WifiDisconnectTCP = wifi_disconnect_tcp,
  .WifiSendTCP = wifi_send_tcp,
  .WifiStatus = wifi_status,
  .WifiResolveName = wifi_resolve_name,
  .WifiEnableMDNS = wifi_enable_mdns,
  .WifiDisableMDNS = wifi_disable_mdns,
  .WifiSetHostname = wifi_set_hostname,
  .GetWifiError = wifi_get_error,
  .GetTime = get_time,
  .SetTime = set_time
};

///////////////////////////////////////////////////////////////////////////////////////////

void drawtext(int x, int y, SDL_Color fc, SDL_Color bc, const char *str, ...) {
  char temp[1024];
  va_list args;

  va_start(args, str);
  vsprintf(temp, str, args);

  SDL_Surface* textSurface = TTF_RenderText_Shaded(fontsans14, temp, fc, bc);
  // Pass zero for width and height to draw the whole surface 
  SDL_Rect textLocation = { (short)x, (short)y, 0, 0 };
  SDL_BlitSurface(textSurface, NULL, surface, &textLocation);
  SDL_FreeSurface(textSurface);
  va_end(args);
}

uint8_t *so(int x, int y) {
  return pixels + (WINDOW_WIDTH * y + x) * 4;
}

void drawbox(int x, int y, int w, int h, uint32_t c) {
  SDL_Rect rect = { (short)x, (short)y, (uint16_t)w, (uint16_t)h };
  SDL_FillRect(surface, &rect, c);
}

char relpath[1024];
char tmppath[1024];

char *getpath(const char *filename) {
  strcpy(tmppath, relpath);
  strcat(tmppath, filename);
  return tmppath;
}

int mainPlay(int argc, char **argv){
  SDL_Event event;
  int64_t lasttimestamp = 0;
  SDL_Rect srcrect = { 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT };
  SDL_Rect dstrect = { 0, 0, SCREEN_WIDTH * SCREEN_SCALE, SCREEN_HEIGHT * SCREEN_SCALE };

  socket_set = SDLNet_AllocSocketSet(32);

  lasttick = ticks_ms = gettick();
  audioInitialize();

  __calltable.MainInit(&syscalls);
  __calltable.MainStart();

  while (!quit) {
    uint32_t tick = gettick();
    ticks_ms = tick;
    __calltable.MainLoop();

    if (wifi_connect_tick != 0 && (tick - wifi_connect_tick) > 1000) {
      WIFIEVENT we = {
        .ssid = "WifiNet",
        .ssidlen = 7,
        .channel = 11
      };
      wifi_current_status = WIFI_STATUS_CONNECTING;
      __calltable.WifiStatusCallback(WIFI_EVENT_CONNECTED, &we);
      we.ip = GetPrimaryIp();
      we.mask = 0x00FFFFFF;
      we.gateway = 0x0101A8C0;
      wifi_current_status = WIFI_STATUS_GOT_IP;
      __calltable.WifiStatusCallback(WIFI_EVENT_GOT_IP, &we);
      wifi_connect_tick = 0;
    }

    if (SDLNet_CheckSockets(socket_set, 0) != 0) {
      if (client_socket != NULL && SDLNet_SocketReady(client_socket)) {
        uint8_t data[536 * 4];
        uint32_t size = SDLNet_TCP_Recv(client_socket, data, 536 * 4);

        if (size != 0) {
          BuildTcpClientConn(client_socket, client_local_port);
          uint8_t *ptr = data;
          while (size > 0) {
            uint32_t chunk = size > 536 ? 536 : size;
            __calltable.TcpRecvCallback(&tcpclient, ptr, chunk);
            ptr += chunk;
            size -= chunk;
          }
        } else {
          close_client_socket();
        }
      }
    }
    if (wifi_client_tick != 0 && (tick - wifi_client_tick) > 2) {
      if (client_socket != NULL) {
        BuildTcpClientConn(client_socket, client_local_port);
        __calltable.TcpSentCallback(&tcpclient);
      }
      wifi_client_tick = 0;
    }
    if (udpsock != NULL && SDLNet_UDP_Recv(udpsock, udppack) != 0) {
      UDPCONN udpconn;
      udpconn.local_port = udp_port;
      udpconn.remote_ip = udppack->address.host;
      udpconn.remote_port = udppack->address.port;
      __calltable.UdpRecvCallback(&udpconn, udppack->data, udppack->len);
    }

    while(SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT) {
        quit = 1;
        break;
      }
      if ((event.type == SDL_KEYDOWN || event.type == SDL_KEYUP) && !event.key.repeat) {
        if (event.key.keysym.sym == SDLK_ESCAPE) {
          //SDL_SetWindowGrab(window, SDL_FALSE);
          quit = 1;
          break;
        }
        switch (event.key.keysym.sym) {
          case SDLK_DOWN:
            handleKey(event.type, KEY_B);
            break;
          case SDLK_UP:
            handleKey(event.type, KEY_X);
            break;
          case SDLK_RIGHT:
            handleKey(event.type, KEY_A);
            break;
          case SDLK_LEFT:
            handleKey(event.type, KEY_Y);
            break;
          case SDLK_w:
            handleKey(event.type, KEY_UP);
            break;
          case SDLK_s:
            handleKey(event.type, KEY_DOWN);
            break;
          case SDLK_a:
            handleKey(event.type, KEY_LEFT);
            break;
          case SDLK_d:
            handleKey(event.type, KEY_RIGHT);
            break;
          case SDLK_m:
            handleKey(event.type, KEY_START);
            break;
        }
        continue;
      }
      if (event.type == SDL_MOUSEBUTTONDOWN) {
        switch (event.button.button) {
          case SDL_BUTTON_LEFT:
            break;
          case SDL_BUTTON_RIGHT:
            break;
        }
        continue;
      }
    }
    if (quit)
      break;

    //drawtext(20, 176*4 + 22, whiteColor, blackColor, "RAM: %d  QSPI: %d  Volume: %d       ", ram_size, qspi_size, volume);

    //SDL_UpdateTexture(ftexture, NULL, framebuf, SCREEN_WIDTH * 2);
    //SDL_BlitScaled(fsurface, &srcrect, surface, &dstrect);
    //SDL_UpdateTexture(texture, NULL, pixels, WINDOW_WIDTH * 4);
    //SDL_RenderClear(renderer);
    //SDL_RenderCopy(renderer, texture, NULL, NULL);
    //SDL_RenderPresent(renderer);
  }

  TTF_CloseFont(fontsans14); 
  TTF_CloseFont(fontmono14); 
  TTF_Quit();   
  SDL_Quit();
  return 0;
}

int main(int argc, char **argv)
{
  strcpy(relpath, argv[0]);
  *strstr(relpath, "emuivy") = 0;

  netid = 1;
  if (argc > 1)
    netid = 2;

  pixels = (uint8_t *)malloc(WINDOW_WIDTH * WINDOW_HEIGHT * 4);
  framebuf = (uint16_t *)malloc(SCREEN_WIDTH * SCREEN_HEIGHT * 2);

  if( SDL_Init( SDL_INIT_EVERYTHING ) == -1 ) {
    return 1;
  }
  TTF_Init();
  SDLNet_Init();

  window = SDL_CreateWindow("LightIvy Emulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);
  renderer = SDL_CreateRenderer(window, -1, 0);
  texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, WINDOW_WIDTH, WINDOW_HEIGHT);
  surface = SDL_CreateRGBSurfaceFrom(pixels, WINDOW_WIDTH, WINDOW_HEIGHT, 32, WINDOW_WIDTH * 4, 0x00FF0000, 0x0000FF00, 0x000000FF, 0xFF000000);
  ftexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB565, SDL_TEXTUREACCESS_STREAMING, SCREEN_WIDTH, SCREEN_HEIGHT);
  fsurface = SDL_CreateRGBSurfaceFrom(framebuf, SCREEN_WIDTH, SCREEN_HEIGHT, 16, SCREEN_WIDTH * 2, 0x00F800, 0x000007E0, 0x0000001F, 0);

  //SDL_SetRelativeMouseMode(SDL_TRUE);
  //SDL_SetHint(SDL_HINT_GRAB_KEYBOARD, "1");
  //SDL_SetWindowGrab(window, SDL_TRUE);

  fontsans14 = TTF_OpenFont(getpath("OpenSans-Regular.ttf"), 16);
  fontmono14 = TTF_OpenFont(getpath("SourceCodePro-Regular.otf"), 14);

  return mainPlay(argc, argv);
}
