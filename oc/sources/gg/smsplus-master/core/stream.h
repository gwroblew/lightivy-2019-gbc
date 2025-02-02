#ifndef _STREAM_H_
#define _STREAM_H_

#include <stdint.h>

enum {
    SND_PSG_DATA,
    SND_PSG_STEREO,
    SND_FM_REG,
    SND_FM_DATA,
};

typedef struct x {
    uint32_t sample_count;
    struct x *next;
    uint8_t type;
    uint8_t data;
    uint8_t filler[6];
} sndstk_t;

void sndstk_copy(sndstk_t *src, sndstk_t *dst);
void sndstk_erase(sndstk_t *p);
sndstk_t *sndstk_new(void);
void sndstk_delete(sndstk_t *p);
void sndstk_insert(sndstk_t *p);
sndstk_t *sndstk_peek(void);
sndstk_t *sndstk_remove(void);
void stream_process(void);

#endif /* _STREAM_H_ */
