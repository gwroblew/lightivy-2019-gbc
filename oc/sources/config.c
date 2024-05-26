#include <stddef.h>
#include <string.h>
#include "oc.h"
#include "config.h"

static const char config_filename[] RODATA = "0:/boot/config.txt";

#define MAGIC_HEADER          0x5B4AD496
#define CONFIG_FLASH_ADDR     0x57000

#define MAX_CONFIG_SIZE     3072
#define MAX_CONFIG_KEYS     254

typedef struct {
  uint16_t key;
  uint16_t value;
} CONFIGKEY;

// Size must be 4096
typedef struct {
  char text[MAX_CONFIG_SIZE];
  CONFIGKEY keys[MAX_CONFIG_KEYS];
  uint32_t count;
  uint32_t header;
} CONFIG;


#define CHECK_END   if (src >= end) break;
#define SKIP_SPACE  { while (src < end && *src < 33) src++; CHECK_END }
#define SKIP_LINE   { while (src < end && !(*src == 10 || *src == 13 || *src == 0)) src++; while (src < end && *src < 32) src++; continue; }
#define SKIP_ID     { while (src < end && ((*src >= 'a' && *src <= 'z') || (*src >= 'A' && *src <= 'Z'))) src++; CHECK_END }
#define SKIP_VALUE  { while (src < end && *src > 32 && *src <= 'z') src++; }

void config_parse(CONFIG *conf, uint32_t size) {
  char *src = conf->text;
  char *start = src;
  char *end = src + size;
  int i = 0;

  while (src < end) {
    SKIP_SPACE
    if (*src == '#')
      SKIP_LINE
    conf->keys[i].key = src - start;
    SKIP_ID
    if (*src != ':') {
      *src++ = 0;
      SKIP_SPACE
    }
    if (*src != ':')
      SKIP_LINE
    *src++ = 0;
    SKIP_SPACE
    conf->keys[i].value = src - start;
    SKIP_VALUE
    *src++ = 0;
    i++;
  }
  conf->count = i;
  conf->header = MAGIC_HEADER;
}

void config_init(char *buf) {
  int r = gsys->FileOpen((char *)config_filename, FILE_MODE_READ);
  if (r != 0)
    return;
  uint32_t br = 0;
  CONFIG *conf = (CONFIG *)buf;
  memset(buf, 255, sizeof(CONFIG));
  gsys->FileRead(buf, MAX_CONFIG_SIZE - 4, &br);
  gsys->FileClose();
  if (br == 0)
    return;
  config_parse(conf, br);
  uint32_t *p1 = (uint32_t *)conf;
  uint32_t *p2 = (uint32_t *)FLASH_ADDR(CONFIG_FLASH_ADDR);
  int i;
  for (i = 0; i < 1024; i++)
    if (*p1++ != *p2++)
      break;
  if (i == 1024)
    return;
  gsys->FlashErase(CONFIG_FLASH_ADDR, 4096);
  gsys->FlashWrite(CONFIG_FLASH_ADDR, conf, 4096);
}

const char * config_get_string(const char *name, const char *def) {
  CONFIG *conf = (CONFIG *)FLASH_ADDR(CONFIG_FLASH_ADDR);
  if (conf->header != MAGIC_HEADER)
    return def;
  for (int i = 0; i < conf->count; i++)
    if (strcmp(name, conf->text + conf->keys[i].key) == 0)
      return (const char *)(conf->text + conf->keys[i].value);
  return def;
}

extern int control_atoi(char *str);

int config_get_number(const char *name, int def) {
  const char *val = config_get_string(name, NULL);
  if (val == NULL)
    return def;
  return control_atoi((char *)val);
}

int config_set(char *buf, char *name, char *value) {
  CONFIG *conf = (CONFIG *)FLASH_ADDR(CONFIG_FLASH_ADDR);
  if (conf->header != MAGIC_HEADER)
    return 100;
  char *dst = buf;
  *dst = 0;
  for (int i = 0; i < conf->count; i++) {
    if (strcmp(name, conf->text + conf->keys[i].key) == 0) {
      strcat(dst, name);
      strcat(dst, ": ");
      strcat(dst, value);
      strcat(dst, "\n");
      continue;
    }
    strcat(dst, conf->text + conf->keys[i].key);
    strcat(dst, ": ");
    strcat(dst, conf->text + conf->keys[i].value);
    strcat(dst, "\n");
  }
  uint32_t size = strlen(dst);
  int r = gsys->FileOpen((char *)config_filename, FILE_MODE_CREATE_ALWAYS | FILE_MODE_WRITE);
  if (r != 0)
    return r;
  gsys->FileWrite(buf, size);
  gsys->FileClose();
  config_init(buf);
  return 0;
}
