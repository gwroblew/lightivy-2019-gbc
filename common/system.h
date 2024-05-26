#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include <stdarg.h>
#include <stdint.h>

#define __inline__ inline

#define HAS_START_BUTTON    1
//#define LIGHTIVY_TH         1

//#define BITBANG_SPI       1

#define PIN_LCD_CLK       (14)
#define PIN_LCD_DATA      (13)
#define PIN_LCD_CS        (16)
#define PIN_LCD_RS        (0)
#define PIN_RAM_CS        (4)
#define PIN_SD_CS         (1)
#define PIN_BUTTONS_DATA  (5)

#ifndef STORE_ATTR
#define STORE_ATTR __attribute__((aligned(4)))
#endif

#ifdef  OCEMU
#define ROCODE
#define RODATA
#define IRDATA
#define RWDATA
#else
#ifndef BOOT_IMAGE
#define ROCODE __attribute__((section(".irom0.text")))
#define RODATA __attribute__((section(".irom.text"))) STORE_ATTR
#else
#define ROCODE
#define RODATA
#endif
#define IRDATA __attribute__((section(".iram.text"))) STORE_ATTR
#define RWDATA __attribute__((section(".dram.text"))) STORE_ATTR
#endif

#define SD_CS   PIN_SD_CS
#define SD_MOSI PIN_LCD_DATA
#define SD_MISO (12)
#define SD_SCK  PIN_LCD_CLK

#ifndef WRITE_PERI_REG
#define WRITE_PERI_REG(addr, val) (*((volatile uint32_t *)addr)) = (uint32_t)(val)
#endif
#define HARD_WDT_FEED   { WRITE_PERI_REG(0x60000914, 0x73); }   // watchdog

#define NOSDK   (*(volatile uint32_t *)0x3FFFFFF8)

#ifdef  LIGHTIVY_FIRMWARE
#define DEBUG(fmt, ...) do {	\
	static const char flash_str[] RODATA = fmt;	\
	debug(flash_str, ##__VA_ARGS__);	\
	} while(0)
#else
#define DEBUG(fmt, ...) do {	\
	static const char flash_str[] RODATA = fmt;	\
	gsys->Debug(flash_str, ##__VA_ARGS__);	\
	} while(0)
#endif

#define AUDIO_BUFFER_SIZE       (256)

struct dma_desc
{
	uint32_t	blocksize:12;
	uint32_t	datalen:12;
	uint32_t	unused:5;
	uint32_t	sub_sof:1;
	uint32_t 	eof:1;
	uint32_t	owner:1;

	uint32_t	buf_ptr;
	uint32_t	next_link_ptr;
};

//Parameters for the I2S DMA behaviour
#define I2SDMABUFCNT (4)			//Number of buffers in the I2S circular buffer
#define I2SDMABUFLEN (AUDIO_BUFFER_SIZE)	//Length of one buffer, in 32-bit words.

typedef struct {
  unsigned int buffer[I2SDMABUFCNT * I2SDMABUFLEN];
  //Pointer to the I2S DMA buffer data
  unsigned int *i2sBuf[I2SDMABUFCNT];
  //I2S DMA buffer descriptors
  struct dma_desc i2sBufDesc[I2SDMABUFCNT];
  //DMA underrun counter
  long underrunCnt;

  int dmaidx;
  int lastidx;
} SOUNDSYS;

#define SOUNDSYSPTR   (*(volatile SOUNDSYS **)0x3FFFFFFC)

void i2sPollSoundStatus();

#define FILEINFO_FILENAME_SIZE    (124)

typedef struct {
  char filename[FILEINFO_FILENAME_SIZE];
  uint32_t flags;
} FILEINFO;

extern void gpio_set_pin_level(int pin, int level);
extern int gpio_get_pin_level(int pin);
extern void delay_ms(unsigned int millis);

extern unsigned char oc_pgm_read_byte(const unsigned char *ptr);
extern void oc_spi_send_byte(unsigned char d);
extern void oc_spi_send_bytes64(uint32_t *src);
extern unsigned char oc_spi_read_byte();
extern void oc_spi_read_bytes64(uint32_t *dst);
extern void oc_set_sd_speed(uint32_t mhz);

extern uint32_t flash_file(char *fname, uint32_t offset, uint32_t size, uint32_t addr);
extern uint32_t get_flash_checksum(uint32_t addr, uint32_t size);
extern uint32_t read_checksum(char *fname);
extern uint32_t write_checksum(char *fname, uint32_t data);

extern void debug(const char *str, ...);
extern void reset();
extern void nosdk();
extern void flash_switch(uint32_t n, uint32_t c);
extern void sleep_ms(unsigned int millis);
extern void sleep_us(unsigned int millis);
extern void lcd_mode();
extern void sd_mode();
extern void ram_mode();
extern void *get_mem_block(uint32_t type, uint32_t size);
extern uint32_t get_free_mem();
extern uint32_t get_ms_ticks();
extern void set_cpu_speed(uint32_t mhz);
extern uint32_t get_keys();
extern uint32_t play_audio(uint32_t rate, SOUNDSYS *ss);
extern void stop_audio();
extern uint32_t * get_audio_buffer();
extern int flash_read_mem(uint32_t addr, void *data, uint32_t size);
extern int flash_write_mem(uint32_t addr, void *data, uint32_t size);
extern int flash_erase(uint32_t addr, uint32_t size);
extern void clear_screen(uint16_t c);
extern void update_screen16(uint16_t *fb, uint32_t y1, uint32_t y2, int clear);
extern void update_screen8(uint8_t *fb, uint16_t *pb, uint32_t y1, uint32_t y2, int clear);
extern void update_screen4(uint8_t *fb, uint16_t *pb, uint32_t y1, uint32_t y2, int clear);
extern void set_screen(uint32_t y1, uint32_t y2);
extern void sendwords(uint32_t *data, uint32_t size);
extern uint32_t get_battery_level();
extern int disk_init();
extern uint32_t get_disk_speed();
extern int file_open(char *fname, uint32_t mode);
extern int file_read(void *buffer, uint32_t size, uint32_t *read);
extern int file_write(void *buffer, uint32_t size);
extern int file_seek(uint64_t offset);
extern uint64_t file_size();
extern int file_close();
extern int delete_file(char *fname);
extern int make_dir(char *fname);
extern int read_dir(char *path, FILEINFO *filebuf, uint32_t *bufsize);
extern const char *get_disk_error(int error);
extern void ram_read_start(uint32_t addr, uint32_t size);
extern void ram_read_end(void *data, uint32_t size);
extern void ram_read(uint32_t addr, void *data, uint32_t size);
extern void ram_write_start(uint32_t addr, void *data, uint32_t size);
extern void ram_write_end();
extern void ram_write(uint32_t addr, void *data, uint32_t size);

#ifdef  HAS_START_BUTTON
#define KEY_UP                  (32)
#define KEY_DOWN                (256)
#define KEY_RIGHT               (64)
#define KEY_LEFT                (128)
#define KEY_A                   (4)
#define KEY_B                   (2)
#define KEY_X                   (8)
#define KEY_Y                   (16)
#define KEY_START               (1)
#else
#define KEY_UP                  (16)
#define KEY_DOWN                (128)
#define KEY_RIGHT               (32)
#define KEY_LEFT                (64)
#define KEY_A                   (2)
#define KEY_B                   (1)
#define KEY_X                   (4)
#define KEY_Y                   (8)
#define KEY_START               (KEY_UP | KEY_DOWN)
#endif

#define KEY_DELAY               (500)
#define KEY_REPEAT              (125)

#define MEM_TYPE_RAM            (0)
#define MEM_TYPE_IRAM           (1)

#define MEM_DRAM_BASE_NOSDK     (0x3FFE8000)
#define MEM_IRAM_BASE           (0x40108000)
#define MEM_IRAM_CODE_NOSDK     (0x40102000)

#define RESET_MODE_CLEAN            (0)
#define RESET_MODE_FLASH_FIRMWARE   (1)
#define RESET_MODE_FLASH_OC         (2)
#define RESET_MODE_FLASH_ALL        (3)

#define FILE_MODE_READ          (1)
#define FILE_MODE_WRITE         (2)
#define	FILE_MODE_CREATE_NEW		(0x04)
#define	FILE_MODE_CREATE_ALWAYS	(0x08)
#define	FILE_MODE_OPEN_ALWAYS		(0x10)
#define	FILE_MODE_OPEN_APPEND		(0x30)

#define FILE_FLAG_DIR           (1)

typedef struct {
  char ssid[32];
} WIFINET;

typedef struct {
  uint16_t local_port;
  uint16_t remote_port;
  uint32_t remote_ip;
} TCPCONN;

typedef struct {
  uint16_t local_port;
  uint16_t remote_port;
  uint32_t remote_ip;
} UDPCONN;

#define WIFI_STATUS_IDLE            (0)
#define WIFI_STATUS_CONNECTING      (1)
#define WIFI_STATUS_WRONG_PASSWORD  (2)
#define WIFI_STATUS_NO_AP_FOUND     (3)
#define WIFI_STATUS_CONNECT_FAIL    (4)
#define WIFI_STATUS_GOT_IP          (5)

#define WIFI_EVENT_CONNECTED    (1)     // ssid, channel
#define WIFI_EVENT_DISCONNECTED (2)     // ssid, reason
#define WIFI_EVENT_GOT_IP       (3)     // ip, mask, gateway

typedef struct {
  uint32_t tm_year;
  uint32_t tm_mon;
  uint32_t tm_mday;
  uint32_t tm_hour;
  uint32_t tm_min;
  uint32_t tm_sec;
  uint32_t tm_wday;
  uint32_t tm_yday;
} TIME;

typedef struct {
  char ssid[33];
  uint8_t ssidlen;
  uint8_t channel;
  uint8_t reason;
  uint32_t ip;
  uint32_t mask;
  uint32_t gateway;
} WIFIEVENT;

typedef struct {
  void (*Debug)(const char *str, ...);
  void (*Reset)(uint32_t mode);
  void (*NoSDK)();
  void (*FlashSwitch)(uint32_t n, uint32_t c);
  void (*SleepMs)(uint32_t ms);
  void (*SleepUs)(uint32_t us);
  uint32_t (*GetMsTicks)();
  void *(*GetMemBlock)(uint32_t type, uint32_t size);
  uint32_t (*GetFreeMem)();
  void (*SetCpuSpeed)(uint32_t mhz);
  void (*LcdMode)();
  void (*SdMode)();
  void (*RamMode)();
  void (*RamRead)(uint32_t addr, void *data, uint32_t size);
  void (*RamWrite)(uint32_t addr, void *data, uint32_t size);
  void (*RamReadStart)(uint32_t addr, uint32_t size);
  void (*RamReadEnd)(void *data, uint32_t size);
  void (*RamWriteStart)(uint32_t addr, void *data, uint32_t size);
  void (*RamWriteEnd)();
  int (*FlashRead)(uint32_t addr, void *data, uint32_t size);
  int (*FlashWrite)(uint32_t addr, void *data, uint32_t size);
  int (*FlashErase)(uint32_t addr, uint32_t size);
  uint32_t (*GetKeys)();
  void (*ClearScreen)(uint16_t c);
  void (*UpdateScreen16)(uint16_t *fb, uint32_t y1, uint32_t y2, int clear);
  void (*UpdateScreen8)(uint8_t *fb, uint16_t *pb, uint32_t y1, uint32_t y2, int clear);
  void (*UpdateScreen4)(uint8_t *fb, uint16_t *pb, uint32_t y1, uint32_t y2, int clear);
  void (*SetScreen)(uint32_t y1, uint32_t y2);
  void (*SendScreenData)(uint32_t *data, uint32_t size);
  int (*Printf)(const char *str, ...);
  int (*Snprintf)(char* buffer, uint32_t count, const char* format, ...);
  int (*Vsnprintf)(char* buffer, uint32_t count, const char* format, va_list va);
  uint32_t (*PlayAudio)(uint32_t rate, SOUNDSYS *ss);
  void (*StopAudio)();
  uint32_t *(*GetAudioBuffer)();
  void (*PollAudioBuffer)();
  uint32_t (*GetBatteryLevel)();
  int (*DiskInit)();
  uint32_t (*GetDiskSpeed)();
  int (*FileOpen)(char *fname, uint32_t mode);
  int (*FileRead)(void *buffer, uint32_t size, uint32_t *read);
  int (*FileWrite)(void *buffer, uint32_t size);
  int (*FileSeek)(uint64_t offset);
  uint64_t (*FileSize)();
  int (*FileClose)();
  int (*ReadDir)(char *path, FILEINFO *filebuf, uint32_t *bufsize);
  int (*DeleteFile)(char *fname);
  int (*DeleteDir)(char *dirname);
  int (*MakeDir)(char *dirname);
  const char *(*GetDiskError)(int error);
  int (*WifiScan)(WIFINET *netbuf, uint32_t bufsize);
  int (*WifiConnect)(char *ssid, char *pwd);
  int (*WifiDisconnect)();
  uint32_t (*WifiGetIP)();
  uint32_t (*WifiGetSignal)();
  int (*WifiEnableUDP)(uint32_t ip, uint32_t port);
  int (*WifiDisableUDP)();
  int (*WifiSendUDP)(void *data, uint32_t size);
  int (*WifiConnectTCP)(uint32_t ip, uint32_t port);
  int (*WifiDisconnectTCP)();
  int (*WifiSendTCP)(void *data, uint32_t size);
  uint32_t (*WifiStatus)();
  int (*WifiResolveName)(char *domain);
  int (*WifiEnableMDNS)(char *name);
  int (*WifiDisableMDNS)();
  int (*WifiSetHostname)(char *name);
  const char *(*GetWifiError)(int error);
  uint32_t (*GetTime)(TIME *datetime);
  void (*SetTime)(uint32_t time_s);
} SYSCALLS;

typedef struct {
  uint32_t magic;
  uint32_t version;
  void (*MainInit)(SYSCALLS *sys);
  void (*MainStart)();
  void (*MainLoop)();
  void (*WifiStatusCallback)(uint32_t event, WIFIEVENT *data);
  void (*WifiScanCallback)(WIFINET *netbuf, uint32_t count);
  void (*TcpConnectCallback)(TCPCONN *conn);
  void (*TcpReconnectCallback)(TCPCONN *conn, int8_t error);
  void (*TcpDisconnectCallback)(TCPCONN *conn);
  void (*TcpSentCallback)(TCPCONN *conn);
  void (*TcpRecvCallback)(TCPCONN *conn, void *data, uint32_t size);
  void (*UdpRecvCallback)(UDPCONN *conn, void *data, uint32_t size);
  void (*DnsResultCallback)(const char *name, uint32_t ip);
  const char *(* ConfigGetString)(const char *name, const char *def);
  int (*ConfigGetNumber)(const char *name, int def);
} OCCALLS;

#define FLASH_OC_MAGIC    (0x1B3E6F2A)

#define FLASH_OC_BASE     (0x58000)
#define FLASH_OC_SIZE     (0x28000)

#define FLASH_SYS_BASE    (0xC000)
#define FLASH_SYS_SIZE    (0x4C000)

#define OCCALL   ((OCCALLS *)(0x40200000 + FLASH_OC_BASE))
#ifdef OCEMU
extern uint8_t flashmem[];
#define FLASH_ADDR(x)   ((void *)(flashmem + (x & 0x3FFFFF)))
#else
#define FLASH_ADDR(x)   ((void *)(0x40200000 + (x & 0xFFFFF)))
#endif

#endif /* _SYSTEM_H_ */
