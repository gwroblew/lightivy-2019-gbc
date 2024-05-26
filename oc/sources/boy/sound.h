#ifndef __SOUND_H__
#define __SOUND_H__

#define SAMPLE_RATE   (22050)

struct sndchan
{
	int on;
	unsigned pos;
	int cnt, encnt, swcnt;
	int len, enlen, swlen;
	int swfreq;
	int freq;
	int envol, endir;
};

struct snd
{
	int rate;
	struct sndchan ch[4];
  uint32_t *outbuf;
  uint32_t outcnt;
  uint32_t sndclk;
  uint8_t dmgwave[16];
  uint8_t cgbwave[16];
  uint8_t sqwave[4][8];
  int freqtab[8];
  uint8_t noise7[16];
  uint8_t noise15[256];
};

extern struct snd *snd;

void audio_init(uint8_t *buf);
void audio_write(uint32_t addr, uint8_t b, uint32_t clk);
uint8_t audio_read(uint32_t addr, uint32_t clk);
void audio_dirty();
void audio_reset(uint32_t cgbmode);
void audio_mix(uint32_t clk);
void audio_set_volume(uint32_t v);
uint32_t audio_get_volume();

#endif
