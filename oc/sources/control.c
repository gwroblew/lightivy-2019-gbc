#include <stdlib.h>
#include <string.h>
#include "control.h"
#include "oc.h"

#define NETBUF_SIZE     (252)

#define CONTROL_STATE_IDLE      (0)
#define CONTROL_STATE_INIT      (1)
#define CONTROL_STATE_FIELD     (2)
#define CONTROL_STATE_VALUE     (3)
#define CONTROL_STATE_COMMAND   (4)

#define CONTROL_FIELD_PASSWORD  (1)
#define CONTROL_FIELD_FILENAME  (2)
#define CONTROL_FIELD_FILESIZE  (3)
#define CONTROL_FIELD_MODE      (4)

#define CONTROL_COMMAND_GET     (1)
#define CONTROL_COMMAND_PUT     (2)
#define CONTROL_COMMAND_LIST    (3)
#define CONTROL_COMMAND_VERIFY  (4)
#define CONTROL_COMMAND_REBOOT  (5)

static const char field_password[] RODATA = "password";
static const char field_filename[] RODATA = "filename";
static const char field_filesize[] RODATA = "filesize";
static const char field_mode[] RODATA = "mode";

static const char command_get[] RODATA = "GET";
static const char command_put[] RODATA = "PUT";
static const char command_list[] RODATA = "LIST";
static const char command_verify[] RODATA = "VERIFY";
static const char command_reboot[] RODATA = "REBOOT";

static const char response_ok[] RODATA = "OK";

uint8_t RWDATA state;
static uint8_t RWDATA field;
static uint8_t RWDATA command;
static uint8_t RWDATA insize;
static uint32_t RWDATA filesize;
static uint32_t RWDATA bytesin;
static char RWDATA filename[NETBUF_SIZE];
static uint8_t RWDATA netbuf[NETBUF_SIZE];
static uint8_t * RWDATA workbuf;

uint32_t ROCODE get_file_checksum(char *fname) {
  if (gsys->FileOpen(fname, FILE_MODE_READ) != 0)
    return 0;
  uint32_t *buf = (uint32_t *)workbuf;
  uint32_t bufsize = WORKBUF_SIZE;
  uint32_t br, chk = 0;

  while (1) {
    br = 0;
    gsys->FileRead(buf, bufsize, &br);
    if (br == 0)
      break;
    uint8_t *bb = (uint8_t *)buf;
    while ((br & 3) != 0) {
      bb[br++] = 0;
    }
    uint32_t *ptr = buf;
    for (uint32_t i = 0; i < br; i += 4)
      chk ^= *ptr++;
  }
  gsys->FileClose();
  return chk;
}

int ROCODE control_atoi(char *str)
{
  if (*str == '\0')
    return 0;
  int res = 0;
  int sign = 1;
  int i = 0;

  if (str[0] == '-') {
    sign = -1;
    i++;
  }

  for (; str[i] != '\0'; ++i) {
    if ( str[i] < '0' || str[i] > '9')
      return sign*res;
    res = res*10 + str[i] - '0';
  }
  return sign*res;
}

uint32_t ROCODE control_deferred_connect(OCDEFER *def) {
  if (state == CONTROL_STATE_IDLE) {
    gsys->WifiConnectTCP(0x4601A8C0, 7070);
  }
  return DEFER_RESULT_OK;
}

uint32_t ROCODE control_deferred_close(OCDEFER *def) {
  gsys->WifiDisconnectTCP();
  state = CONTROL_STATE_IDLE;
  return DEFER_RESULT_DONE;
}

void ROCODE control_set_workbuf(uint8_t *wb) {
  workbuf = wb;
}

void ROCODE control_init() {
  state = CONTROL_STATE_IDLE;
  add_defer_func(0, 1000, control_deferred_connect, 0);
}

void ROCODE control_connect_callback(TCPCONN *conn) {
  state = CONTROL_STATE_INIT;
  command = 0;
  insize = 0;
  bytesin = 0;
  filesize = 0;
}

void ROCODE control_disconnect_callback(TCPCONN *conn) {
  switch (command) {
    case CONTROL_COMMAND_PUT:
    case CONTROL_COMMAND_GET:
      gsys->FileClose();
      break;
  }
  state = CONTROL_STATE_IDLE;
}

static void ROCODE send_file_chunk() {
  uint32_t br = 0;
  void *data = workbuf;
  uint32_t size = WORKBUF_SIZE;

  gsys->FileRead(data, size, &br);
  if (br == 0) {
    gsys->FileClose();
    add_defer_func(0, 0, control_deferred_close, 0);
    return;
  }
  if (gsys->WifiSendTCP(data, br) != 0) {
    add_defer_func(0, 0, control_deferred_close, 0);
  }
}

void ROCODE control_sent_callback(TCPCONN *conn) {
  if (command == CONTROL_COMMAND_GET)
    send_file_chunk();
  if (command == CONTROL_COMMAND_LIST || command == CONTROL_COMMAND_VERIFY)
    add_defer_func(0, 0, control_deferred_close, 0);
}

uint32_t ROCODE control_read_string(uint8_t **pptr, uint32_t *psize) {
  uint8_t *ptr = *pptr;
  uint32_t size = *psize;
  uint32_t done = 0;

  while (size != 0) {
    if (*ptr < 32) {
      if (insize == 0) {
        while (size != 0 && *ptr < 32) {
          ptr++;
          size--;
        }
        continue;
      }
      netbuf[insize] = 0;
      done = 1;
      while (size != 0 && *ptr < 32) {
        ptr++;
        size--;
      }
      break;
    }
    netbuf[insize] = *ptr++;
    insize++;
    size--;
    if (insize == NETBUF_SIZE - 1) {
      netbuf[insize] = 0;
      done = 1;
      break;
    }
  }

  *pptr = ptr;
  *psize = size;
  return done;
}

static void ROCODE read_sd_dir(char *path) {
  FILEINFO *fi = (FILEINFO *)workbuf;
  uint32_t bufsize = 128;
  gsys->ReadDir(path, fi, &bufsize);
  char *dst = (char *)(fi + 128);
  char *ptr = dst;
  for (int i = 0; i < bufsize; i++) {
    if (fi[i].flags & FILE_FLAG_DIR) {
      memcpy(ptr, "dir,", 4);
      ptr += 4;
    } else {
      memcpy(ptr, "file,", 5);
      ptr += 5;
    }
    uint32_t l = strlen(fi[i].filename);
    memcpy(ptr, fi[i].filename, l);
    ptr += l;
    *ptr++ = '\n';
  }
  if (gsys->WifiSendTCP(dst, ptr - dst) != 0) {
    add_defer_func(0, 0, control_deferred_close, 0);
  }
}

void ROCODE control_recv_callback(TCPCONN *conn, void *data, uint32_t size) {
  uint8_t *ptr = data;
  int cnt;
  uint32_t chk;

  while (size != 0) {
    if (state == CONTROL_STATE_INIT) {
      if (control_read_string(&ptr, &size)) {
        if (strcmp((char *)netbuf, command_get) == 0) {
          command = CONTROL_COMMAND_GET;
        } else if (strcmp((char *)netbuf, command_put) == 0) {
          command = CONTROL_COMMAND_PUT;
        } else if (strcmp((char *)netbuf, command_list) == 0) {
          command = CONTROL_COMMAND_LIST;
        } else if (strcmp((char *)netbuf, command_verify) == 0) {
          command = CONTROL_COMMAND_VERIFY;
        } else if (strcmp((char *)netbuf, command_reboot) == 0) {
          command = CONTROL_COMMAND_REBOOT;
        } else {
          add_defer_func(0, 0, control_deferred_close, 0);
          size = 0;
          break;
        }
        state = CONTROL_STATE_FIELD;
        insize = 0;
      }
      continue;
    } else if (state == CONTROL_STATE_FIELD) {
      if (control_read_string(&ptr, &size)) {
        if (strcmp((char *)netbuf, field_password) == 0) {
          field = CONTROL_FIELD_PASSWORD;
        } else if (strcmp((char *)netbuf, field_filename) == 0) {
          field = CONTROL_FIELD_FILENAME;
        } else if (strcmp((char *)netbuf, field_filesize) == 0) {
          field = CONTROL_FIELD_FILESIZE;
        } else if (strcmp((char *)netbuf, field_mode) == 0) {
          field = CONTROL_FIELD_MODE;
        } else if (netbuf[0] == '#' && netbuf[1] == 0) {
          state = CONTROL_STATE_COMMAND;

          switch (command) {
            case CONTROL_COMMAND_PUT:
              gsys->FileOpen(filename, FILE_MODE_CREATE_ALWAYS | FILE_MODE_WRITE);
              break;
            case CONTROL_COMMAND_GET:
              gsys->FileOpen(filename, FILE_MODE_READ);
              send_file_chunk();
              size = 0;
              break;
            case CONTROL_COMMAND_LIST:
              read_sd_dir(filename);
              size = 0;
              break;
            case CONTROL_COMMAND_VERIFY:
              chk = get_file_checksum(filename);
              cnt = gsys->Snprintf((char *)netbuf, 250, "%u\n", chk);
              gsys->WifiSendTCP(netbuf, cnt);
              size = 0;
              break;
            case CONTROL_COMMAND_REBOOT:
              gsys->SleepMs(1000);
              gsys->Reset(filesize);
              break;
            default:
              add_defer_func(0, 0, control_deferred_close, 0);
              size = 0;
              break;
          }
          continue;
        } else {
          add_defer_func(0, 0, control_deferred_close, 0);
          break;
        }
        state = CONTROL_STATE_VALUE;
        insize = 0;
      }
      continue;
    } else if (state == CONTROL_STATE_VALUE) {
      if (control_read_string(&ptr, &size)) {
        switch(field) {
          case CONTROL_FIELD_FILENAME:
            memcpy(filename, netbuf, NETBUF_SIZE);
            break;
          case CONTROL_FIELD_FILESIZE:
          case CONTROL_FIELD_MODE:
            filesize = control_atoi((char *)netbuf);
            bytesin = 0;
            break;
          case CONTROL_FIELD_PASSWORD:
            break;
        }
        state = CONTROL_STATE_FIELD;
        insize = 0;
      }
      continue;
    } else if (state == CONTROL_STATE_COMMAND) {
      switch (command) {
        case CONTROL_COMMAND_PUT:
          if (((uint32_t)ptr) & 3) {
            size = 0;
            add_defer_func(0, 0, control_deferred_close, 0);
            break;
          }
          if ((bytesin + size) > filesize) {
            size = filesize - bytesin;
          }
          if (size > 0)
            gsys->FileWrite(ptr, size);
          bytesin += size;
          size = 0;
          if (bytesin >= filesize) {
            gsys->WifiSendTCP((void *)response_ok, sizeof(response_ok));
            add_defer_func(500, 0, control_deferred_close, 0);
          }
          break;
        default:
          size = 0;
          break;
      }
      continue;
    }
    add_defer_func(0, 0, control_deferred_close, 0);
    break;
  }
}
