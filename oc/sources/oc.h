#ifndef __OC_H
#define __OC_H

#include "system.h"

extern SYSCALLS *gsys;

#define MAX_DEFERRED_FUNCS    (8)

#define DEFER_RESULT_OK       (0)
#define DEFER_RESULT_DONE     (1)

typedef struct __OCDEFER {
  uint32_t start_tick;
  uint32_t step_ticks;
  uint32_t (*func)(struct __OCDEFER *def);
  void *arg;
} OCDEFER;

void add_defer_func(uint32_t start_tick, uint32_t interval, uint32_t (*func)(OCDEFER *def), void *arg);
void remove_defer_func(uint32_t (*func)(OCDEFER *def), void *arg);

uint32_t load_file(const char *fname, uint32_t addr, int32_t size, uint32_t *buf, uint32_t bufsize);

#ifndef OCEMU
#define LOAD_FILE   load_file
#else
#define LOAD_FILE   load_file_emu
uint32_t load_file_emu(const char *fname, void *addr, int32_t size, uint32_t *buf, uint32_t bufsize);
#endif

#define ONKEY(kc, v)  if ((k & (kc)) == (kc)) { v; }

void main_start1();
void main_loop1();
void main_start2();
void main_loop2();
void main_start3();
void main_loop3();
void fill_audio();

#endif
