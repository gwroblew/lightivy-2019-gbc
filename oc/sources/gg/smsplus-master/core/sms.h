#ifndef _SMS_H_
#define _SMS_H_

#include <stdint.h>

enum {
    SLOT_BIOS   = 0,
    SLOT_CARD   = 1,
    SLOT_CART   = 2,
    SLOT_EXP    = 3
};

enum {
    MAPPER_NONE         = 0,
    MAPPER_SEGA         = 1,
    MAPPER_CODIES       = 2,
    MAPPER_KOREA        = 3,
    MAPPER_KOREA2       = 4
};

enum {
    DISPLAY_NTSC        = 0,
    DISPLAY_PAL         = 1
};

enum {
    FPS_NTSC        = 60,
    FPS_PAL         = 50
};

enum {
    CLOCK_NTSC        = 3579545,
    CLOCK_PAL         = 3579545
};

enum {
    CONSOLE_SMS         = 0x20,
    CONSOLE_SMSJ        = 0x21,
    CONSOLE_SMS2        = 0x22,

    CONSOLE_GG          = 0x40,
    CONSOLE_GGMS        = 0x41,

    CONSOLE_MD          = 0x80,
    CONSOLE_MDPBC       = 0x81,
    CONSOLE_GEN         = 0x82,
    CONSOLE_GENPBC      = 0x83
};

#define HWTYPE_SMS  CONSOLE_SMS
#define HWTYPE_GG   CONSOLE_GG
#define HWTYPE_MD   CONSOLE_MD

#define IS_SMS      (sms.console & HWTYPE_SMS)
#define IS_GG       (sms.console & HWTYPE_GG)
#define IS_MD       (sms.console & HWTYPE_MD)

enum {
    TERRITORY_DOMESTIC  = 0,
    TERRITORY_EXPORT    = 1
};

/* SMS context */
typedef struct
{
    uint8_t wram[0x2000];
    uint8_t paused;
    uint8_t save;
    uint8_t territory;
    uint8_t console;
    uint8_t display;
    uint8_t fm_detect;
    uint8_t use_fm;
    uint8_t memctrl;
    uint8_t ioctrl;
    struct {
        uint8_t pdr;      /* Parallel data register */
        uint8_t ddr;      /* Data direction register */
        uint8_t txdata;   /* Transmit data buffer */
        uint8_t rxdata;   /* Receive data buffer */
        uint8_t sctrl;    /* Serial mode control and status */
    } sio;
    struct {
        int type;
    } device[2];
} sms_t;

/* Global data */
extern sms_t sms;

extern uint8_t dummy_write[0x400];
extern uint8_t dummy_read[0x400];

/* Function prototypes */
void sms_init(void);
void sms_reset(void);
void sms_shutdown(void);
void sms_mapper_w(int address, int data);
int sms_irq_callback(int param);

#endif /* _SMS_H_ */
