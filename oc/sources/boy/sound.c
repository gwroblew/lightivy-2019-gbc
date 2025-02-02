#include <stdint.h>
#include <string.h>

#include "oc.h"
#include "sound.h"

#define RI_NR10 0x10
#define RI_NR11 0x11
#define RI_NR12 0x12
#define RI_NR13 0x13
#define RI_NR14 0x14
#define RI_NR21 0x16
#define RI_NR22 0x17
#define RI_NR23 0x18
#define RI_NR24 0x19
#define RI_NR30 0x1A
#define RI_NR31 0x1B
#define RI_NR32 0x1C
#define RI_NR33 0x1D
#define RI_NR34 0x1E
#define RI_NR41 0x20
#define RI_NR42 0x21
#define RI_NR43 0x22
#define RI_NR44 0x23
#define RI_NR50 0x24
#define RI_NR51 0x25
#define RI_NR52 0x26

#define REG(x)    regs[x]

#define R_NR10 REG(RI_NR10)
#define R_NR11 REG(RI_NR11)
#define R_NR12 REG(RI_NR12)
#define R_NR13 REG(RI_NR13)
#define R_NR14 REG(RI_NR14)
#define R_NR21 REG(RI_NR21)
#define R_NR22 REG(RI_NR22)
#define R_NR23 REG(RI_NR23)
#define R_NR24 REG(RI_NR24)
#define R_NR30 REG(RI_NR30)
#define R_NR31 REG(RI_NR31)
#define R_NR32 REG(RI_NR32)
#define R_NR33 REG(RI_NR33)
#define R_NR34 REG(RI_NR34)
#define R_NR41 REG(RI_NR41)
#define R_NR42 REG(RI_NR42)
#define R_NR43 REG(RI_NR43)
#define R_NR44 REG(RI_NR44)
#define R_NR50 REG(RI_NR50)
#define R_NR51 REG(RI_NR51)
#define R_NR52 REG(RI_NR52)

#define VOL_R   (*(uint32_t *)regs)
#define VOL_L   (*(uint32_t *)&regs[4])
#define VOLUME  (*(uint32_t *)&regs[8])

static const uint8_t noise7_ro[] RODATA =
{
    0xfb,0xe7,0xae,0x1b,0xa6,0x2b,0x05,0xe3,
    0xb6,0x4a,0x42,0x72,0xd1,0x19,0xaa,0x03
};

static const uint8_t noise15_ro[] RODATA =
{
0xff,0xfb,0xff,0xe7,0xff,0xaf,0xfe,0x1f,
0xfb,0xbf,0xe6,0x7f,0xaa,0xfe,0x01,0xfb,
0xfb,0xe7,0xe7,0xaf,0xae,0x1e,0x1b,0xbb,
0xa6,0x66,0x2a,0xab,0x00,0x05,0xff,0xe3,
0xff,0xb7,0xfe,0x4f,0xfa,0x5f,0xe2,0x3f,
0xb3,0x7e,0x54,0xfa,0x05,0xe3,0xe3,0xb7,
0xb6,0x4e,0x4a,0x5a,0x42,0x22,0x73,0x32,
0xd5,0x51,0x00,0x19,0xff,0xab,0xfe,0x07,
0xfb,0xef,0xe7,0x9f,0xae,0xbe,0x18,0x7b,
0xae,0xe6,0x19,0xab,0xaa,0x06,0x03,0xeb,
0xf7,0x87,0xce,0xef,0x59,0x9c,0x2a,0xb7,
0x00,0x4d,0xfe,0x53,0xfa,0x17,0xe3,0x8f,
0xb6,0xde,0x49,0x3a,0x49,0x62,0x48,0xb2,
0x4c,0x52,0x56,0x12,0x0b,0x93,0xc6,0x97,
0x68,0x8c,0x8c,0xd4,0xd5,0x05,0x01,0xe1,
0xfb,0xbb,0xe6,0x67,0xaa,0xae,0x00,0x1b,
0xff,0xa7,0xfe,0x2f,0xfb,0x1f,0xe5,0xbf,
0xa2,0x7e,0x32,0xfb,0x51,0xe4,0x1b,0xa7,
0xa6,0x2e,0x2b,0x1b,0x05,0xa5,0xe2,0x23,
0xb3,0x36,0x55,0x4a,0x00,0x43,0xfe,0x77,
0xfa,0xcf,0xe1,0x5f,0xb8,0x3e,0x6f,0x7a,
0x9c,0xe0,0xb5,0xbc,0x42,0x76,0x72,0xca,
0xd1,0x41,0x18,0x79,0xae,0xea,0x19,0x83,
0xaa,0xf6,0x01,0xcb,0xfb,0x47,0xe4,0x6f,
0xa6,0x9e,0x28,0xbb,0x0c,0x65,0xd6,0xa3,
0x08,0x35,0xcf,0x43,0x5c,0x74,0x36,0xc7,
0x49,0x6c,0x48,0x96,0x4c,0x8a,0x54,0xc2,
0x05,0x73,0xe0,0xd7,0xbd,0x0e,0x71,0xda,
0xdb,0x21,0x25,0x39,0x21,0x69,0x38,0x89,
0x6c,0xc8,0x95,0x4c,0x80,0x54,0xfe,0x05,
0xfb,0xe3,0xe7,0xb7,0xae,0x4e,0x1a,0x5b,
0xa2,0x26,0x33,0x2b,0x55,0x04,0x01,0xe7
};

static const uint8_t dmgwave_ro[16] RODATA =
{
	0xac, 0xdd, 0xda, 0x48,
	0x36, 0x02, 0xcf, 0x16,
	0x2c, 0x04, 0xe5, 0x2c,
	0xac, 0xdd, 0xda, 0x48
};

static const uint8_t cgbwave_ro[16] RODATA =
{
	0x00, 0xff, 0x00, 0xff,
	0x00, 0xff, 0x00, 0xff,
	0x00, 0xff, 0x00, 0xff,
	0x00, 0xff, 0x00, 0xff,
};

static const uint8_t sqwave_ro[4][8] RODATA =
{
	{  0, 0,-1, 0, 0, 0, 0, 0 },
	{  0,-1,-1, 0, 0, 0, 0, 0 },
	{ -1,-1,-1,-1, 0, 0, 0, 0 },
	{ -1, 0, 0,-1,-1,-1,-1,-1 }
};

static const int freqtab_ro[8] RODATA =
{
	(1<<14)*2,
	(1<<14),
	(1<<14)/2,
	(1<<14)/3,
	(1<<14)/4,
	(1<<14)/5,
	(1<<14)/6,
	(1<<14)/7
};

static uint8_t RWDATA regs[64];

struct snd RWDATA *snd;

#define RATE (snd->rate)
#define S1 (snd->ch[0])
#define S2 (snd->ch[1])
#define S3 (snd->ch[2])
#define S4 (snd->ch[3])

static void update_vol() {
  VOL_L = ((R_NR50 & 0x07) * VOLUME) >> 4;
  VOL_R = (((R_NR50 & 0x70) >> 4) * VOLUME) >> 4;
}

void audio_set_volume(uint32_t v) {
#ifdef LIGHTIVY_TH
  VOLUME = v;
#else
  VOLUME = v << 4;
#endif
  update_vol();
}

uint32_t audio_get_volume() {
#ifdef LIGHTIVY_TH
  return VOLUME;
#else
  return VOLUME >> 4;
#endif
}

void ROCODE audio_init(uint8_t *buf)
{
  snd = (struct snd *)buf;
	memset(snd, 0, sizeof(struct snd));
  memcpy(snd->dmgwave, dmgwave_ro, sizeof(dmgwave_ro));
  memcpy(snd->cgbwave, cgbwave_ro, sizeof(cgbwave_ro));
  memcpy(snd->sqwave, sqwave_ro, sizeof(sqwave_ro));
  memcpy(snd->freqtab, freqtab_ro, sizeof(freqtab_ro));
  memcpy(snd->noise7, noise7_ro, 16);
  memcpy(snd->noise15, noise15_ro, 256);
  audio_set_volume(3);
}

inline static void s1_freq_d(int d)
{
	if (RATE > (d<<4)) S1.freq = 0;
	else S1.freq = (RATE << 17)/d;
}

inline static void s1_freq()
{
	s1_freq_d(2048 - (((R_NR14&7)<<8) + R_NR13));
}

inline static void s2_freq()
{
	int d = 2048 - (((R_NR24&7)<<8) + R_NR23);
	if (RATE > (d<<4)) S2.freq = 0;
	else S2.freq = (RATE << 17)/d;
}

inline static void s3_freq()
{
	int d = 2048 - (((R_NR34&7)<<8) + R_NR33);
	if (RATE > (d<<3)) S3.freq = 0;
	else S3.freq = (RATE << 21)/d;
}

inline static void s4_freq()
{
	S4.freq = (snd->freqtab[R_NR43&7] >> (R_NR43 >> 4)) * RATE;
	if (S4.freq >> 18) S4.freq = 1<<18;
}

void audio_dirty()
{
	S1.swlen = ((R_NR10>>4) & 7) << 14;
	S1.len = (64-(R_NR11&63)) << 13;
	S1.envol = R_NR12 >> 4;
	S1.endir = (R_NR12>>3) & 1;
	S1.endir |= S1.endir - 1;
	S1.enlen = (R_NR12 & 7) << 15;
	s1_freq();
	S2.len = (64-(R_NR21&63)) << 13;
	S2.envol = R_NR22 >> 4;
	S2.endir = (R_NR22>>3) & 1;
	S2.endir |= S2.endir - 1;
	S2.enlen = (R_NR22 & 7) << 15;
	s2_freq();
	S3.len = (256-R_NR31) << 20;
	s3_freq();
	S4.len = (64-(R_NR41&63)) << 13;
	S4.envol = R_NR42 >> 4;
	S4.endir = (R_NR42>>3) & 1;
	S4.endir |= S4.endir - 1;
	S4.enlen = (R_NR42 & 7) << 15;
	s4_freq();
  update_vol();
}

void audio_off()
{
	memset(&S1, 0, sizeof S1);
	memset(&S2, 0, sizeof S2);
	memset(&S3, 0, sizeof S3);
	memset(&S4, 0, sizeof S4);
	R_NR10 = 0x80;
	R_NR11 = 0xBF;
	R_NR12 = 0xF3;
	R_NR14 = 0xBF;
	R_NR21 = 0x3F;
	R_NR22 = 0x00;
	R_NR24 = 0xBF;
	R_NR30 = 0x7F;
	R_NR31 = 0xFF;
	R_NR32 = 0x9F;
	R_NR34 = 0xBF;
	R_NR41 = 0xFF;
	R_NR42 = 0x00;
	R_NR43 = 0x00;
	R_NR44 = 0xBF;
	R_NR50 = 0x77;
	R_NR51 = 0xF3;
	R_NR52 = 0x70;
	audio_dirty();
}

void ROCODE audio_reset(uint32_t cgbmode)
{
	snd->rate = (1<<21) / SAMPLE_RATE;
	memcpy(regs + 0x30, cgbmode ? snd->cgbwave : snd->dmgwave, 16);
	audio_off();
	R_NR52 = 0xF1;
}

void audio_mix(uint32_t clk)
{
	int s, l, r, f, n;

  snd->sndclk += clk >> 1;
  if (snd->outbuf == NULL) {
    while ((snd->outbuf = gsys->GetAudioBuffer()) == NULL) {
      gsys->SleepMs(1);
    }
    snd->outcnt = 0;
  }

	for (;snd->sndclk >= RATE; snd->sndclk -= RATE)
	{
		l = r = 0;

		if (S1.on)
		{
			s = snd->sqwave[R_NR11>>6][(S1.pos>>18)&7] & S1.envol;
			S1.pos += S1.freq;
			if ((R_NR14 & 64) && ((S1.cnt += RATE) >= S1.len))
				S1.on = 0;
			if (S1.enlen && (S1.encnt += RATE) >= S1.enlen)
			{
				S1.encnt -= S1.enlen;
				S1.envol += S1.endir;
				if (S1.envol < 0) S1.envol = 0;
				if (S1.envol > 15) S1.envol = 15;
			}
			if (S1.swlen && (S1.swcnt += RATE) >= S1.swlen)
			{
				S1.swcnt -= S1.swlen;
				f = S1.swfreq;
				n = (R_NR10 & 7);
				if (R_NR10 & 8) f -= (f >> n);
				else f += (f >> n);
				if (f > 2047)
					S1.on = 0;
				else
				{
					S1.swfreq = f;
					R_NR13 = f;
					R_NR14 = (R_NR14 & 0xF8) | (f>>8);
//					s1_freq_d(2048 - f);
  int d = 2048 - f;
	if (RATE > (d<<4)) S1.freq = 0;
	else S1.freq = (RATE << 17)/d;
// --- inline
				}
			}
			s <<= 2;
			if (R_NR51 & 1) r += s;
			if (R_NR51 & 16) l += s;
		}

		if (S2.on)
		{
			s = snd->sqwave[R_NR21>>6][(S2.pos>>18)&7] & S2.envol;
			S2.pos += S2.freq;
			if ((R_NR24 & 64) && ((S2.cnt += RATE) >= S2.len))
				S2.on = 0;
			if (S2.enlen && (S2.encnt += RATE) >= S2.enlen)
			{
				S2.encnt -= S2.enlen;
				S2.envol += S2.endir;
				if (S2.envol < 0) S2.envol = 0;
				if (S2.envol > 15) S2.envol = 15;
			}
			s <<= 2;
			if (R_NR51 & 2) r += s;
			if (R_NR51 & 32) l += s;
		}

		if (S3.on)
		{
			s = regs[((S3.pos>>22) & 15) + 0x30];
			if (S3.pos & (1<<21)) s &= 15;
			else s >>= 4;
			s -= 8;
			S3.pos += S3.freq;
			if ((R_NR34 & 64) && ((S3.cnt += RATE) >= S3.len))
				S3.on = 0;
			if (R_NR32 & 96) s <<= (3 - ((R_NR32>>5)&3));
			else s = 0;
			if (R_NR51 & 4) r += s;
			if (R_NR51 & 64) l += s;
		}

		if (S4.on)
		{
			if (R_NR43 & 8) s = 1 & (snd->noise7[
				(S4.pos>>20)&15] >> (7-((S4.pos>>17)&7)));
			else s = 1 & ((snd->noise15[
				(S4.pos>>20)&255] ^ snd->noise7[S4.pos>>28]) >> (7-((S4.pos>>17)&7)));
			s = (-s) & S4.envol;
			S4.pos += S4.freq;
			if ((R_NR44 & 64) && ((S4.cnt += RATE) >= S4.len))
				S4.on = 0;
			if (S4.enlen && (S4.encnt += RATE) >= S4.enlen)
			{
				S4.encnt -= S4.enlen;
				S4.envol += S4.endir;
				if (S4.envol < 0) S4.envol = 0;
				if (S4.envol > 15) S4.envol = 15;
			}
			s += s << 1;
			if (R_NR51 & 8) r += s;
			if (R_NR51 & 128) l += s;
		}

		l *= VOL_L;
		r *= VOL_R;

		snd->outbuf[snd->outcnt++] = l + r;

    if (snd->outcnt > 255) {
//#ifdef  OCEMU
      while ((snd->outbuf = gsys->GetAudioBuffer()) == NULL) {
        gsys->SleepMs(1);
      }
//#else
//      uint32_t *ob = gsys->GetAudioBuffer();
//      if (ob != NULL)
//        snd->outbuf = ob;
//#endif
      snd->outcnt = 0;
    }
	}
	R_NR52 = (R_NR52&0xf0) | S1.on | (S2.on<<1) | (S3.on<<2) | (S4.on<<3);
}

uint8_t audio_read(uint32_t r, uint32_t clk)
{
  //audio_mix(clk);
	return REG(r & 0xFF);
}

void s1_init()
{
	S1.swcnt = 0;
	S1.swfreq = ((R_NR14&7)<<8) + R_NR13;
	S1.envol = R_NR12 >> 4;
	S1.endir = (R_NR12>>3) & 1;
	S1.endir |= S1.endir - 1;
	S1.enlen = (R_NR12 & 7) << 15;
	if (!S1.on) S1.pos = 0;
	S1.on = 1;
	S1.cnt = 0;
	S1.encnt = 0;
}

void s2_init()
{
	S2.envol = R_NR22 >> 4;
	S2.endir = (R_NR22>>3) & 1;
	S2.endir |= S2.endir - 1;
	S2.enlen = (R_NR22 & 7) << 15;
	if (!S2.on) S2.pos = 0;
	S2.on = 1;
	S2.cnt = 0;
	S2.encnt = 0;
}

void s3_init()
{
	int i;
	if (!S3.on) S3.pos = 0;
	S3.cnt = 0;
	S3.on = R_NR30 >> 7;
	if (S3.on) for (i = 0; i < 16; i++)
		regs[i + 0x30] ^= 0x13;
}

void s4_init()
{
	S4.envol = R_NR42 >> 4;
	S4.endir = (R_NR42>>3) & 1;
	S4.endir |= S4.endir - 1;
	S4.enlen = (R_NR42 & 7) << 15;
	S4.on = 1;
	S4.pos = 0;
	S4.cnt = 0;
	S4.encnt = 0;
}

void audio_write(uint32_t r, uint8_t b, uint32_t clk)
{
  r &= 0xFF;
	if (!(R_NR52 & 128) && r != RI_NR52) return;
	if ((r & 0xF0) == 0x30)
	{
    //if (S3.on)
    //  audio_mix(clk);
		if (!S3.on)
			regs[r] = b;
		return;
	}
  //audio_mix(clk);
	switch (r)
	{
	case RI_NR10:
		R_NR10 = b;
		S1.swlen = ((R_NR10>>4) & 7) << 14;
		S1.swfreq = ((R_NR14&7)<<8) + R_NR13;
		break;
	case RI_NR11:
		R_NR11 = b;
		S1.len = (64-(R_NR11&63)) << 13;
		break;
	case RI_NR12:
		R_NR12 = b;
		S1.envol = R_NR12 >> 4;
		S1.endir = (R_NR12>>3) & 1;
		S1.endir |= S1.endir - 1;
		S1.enlen = (R_NR12 & 7) << 15;
		break;
	case RI_NR13:
		R_NR13 = b;
		s1_freq();
		break;
	case RI_NR14:
		R_NR14 = b;
		s1_freq();
		if (b & 128) s1_init();
		break;
	case RI_NR21:
		R_NR21 = b;
		S2.len = (64-(R_NR21&63)) << 13;
		break;
	case RI_NR22:
		R_NR22 = b;
		S2.envol = R_NR22 >> 4;
		S2.endir = (R_NR22>>3) & 1;
		S2.endir |= S2.endir - 1;
		S2.enlen = (R_NR22 & 7) << 15;
		break;
	case RI_NR23:
		R_NR23 = b;
		s2_freq();
		break;
	case RI_NR24:
		R_NR24 = b;
		s2_freq();
		if (b & 128) s2_init();
		break;
	case RI_NR30:
		R_NR30 = b;
		if (!(b & 128)) S3.on = 0;
		break;
	case RI_NR31:
		R_NR31 = b;
		S3.len = (256-R_NR31) << 13;
		break;
	case RI_NR32:
		R_NR32 = b;
		break;
	case RI_NR33:
		R_NR33 = b;
		s3_freq();
		break;
	case RI_NR34:
		R_NR34 = b;
		s3_freq();
		if (b & 128) s3_init();
		break;
	case RI_NR41:
		R_NR41 = b;
		S4.len = (64-(R_NR41&63)) << 13;
		break;
	case RI_NR42:
		R_NR42 = b;
		S4.envol = R_NR42 >> 4;
		S4.endir = (R_NR42>>3) & 1;
		S4.endir |= S4.endir - 1;
		S4.enlen = (R_NR42 & 7) << 15;
		break;
	case RI_NR43:
		R_NR43 = b;
		s4_freq();
		break;
	case RI_NR44:
		R_NR44 = b;
		if (b & 128) s4_init();
		break;
	case RI_NR50:
		R_NR50 = b;
    update_vol();
		break;
	case RI_NR51:
		R_NR51 = b;
		break;
	case RI_NR52:
		R_NR52 = b;
		if (!(R_NR52 & 128))
			audio_off();
		break;
	default:
		return;
	}
}
