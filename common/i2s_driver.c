
#include <string.h>
#include "ets_sys.h"
#include "gpio.h"

#include "system.h"

#include "slc_register.h"


#define DR_REG_I2S_BASE (0x60000e00)

#define I2STXFIFO  (DR_REG_I2S_BASE + 0x0000)
#define I2SRXFIFO  (DR_REG_I2S_BASE + 0x0004)
#define I2SCONF  (DR_REG_I2S_BASE + 0x0008)
#define I2S_BCK_DIV_NUM 0x0000003F
#define I2S_BCK_DIV_NUM_S 22
#define I2S_CLKM_DIV_NUM 0x0000003F
#define I2S_CLKM_DIV_NUM_S 16
#define I2S_BITS_MOD 0x0000000F
#define I2S_BITS_MOD_S 12
#define I2S_RECE_MSB_SHIFT (BIT(11))
#define I2S_TRANS_MSB_SHIFT (BIT(10))
#define I2S_I2S_RX_START (BIT(9))
#define I2S_I2S_TX_START (BIT(8))
#define I2S_MSB_RIGHT (BIT(7))
#define I2S_RIGHT_FIRST (BIT(6))
#define I2S_RECE_SLAVE_MOD (BIT(5))
#define I2S_TRANS_SLAVE_MOD (BIT(4))
#define I2S_I2S_RX_FIFO_RESET (BIT(3))
#define I2S_I2S_TX_FIFO_RESET (BIT(2))
#define I2S_I2S_RX_RESET (BIT(1))
#define I2S_I2S_TX_RESET (BIT(0))
#define I2S_I2S_RESET_MASK 0xf

 #define I2SINT_RAW (DR_REG_I2S_BASE + 0x000c)
#define I2S_I2S_TX_REMPTY_INT_RAW (BIT(5))
#define I2S_I2S_TX_WFULL_INT_RAW (BIT(4))
#define I2S_I2S_RX_REMPTY_INT_RAW (BIT(3))
#define I2S_I2S_RX_WFULL_INT_RAW (BIT(2))
#define I2S_I2S_TX_PUT_DATA_INT_RAW (BIT(1))
#define I2S_I2S_RX_TAKE_DATA_INT_RAW (BIT(0))


#define I2SINT_ST (DR_REG_I2S_BASE + 0x0010)
#define I2S_I2S_TX_REMPTY_INT_ST (BIT(5))
#define I2S_I2S_TX_WFULL_INT_ST (BIT(4))
#define I2S_I2S_RX_REMPTY_INT_ST (BIT(3))
#define I2S_I2S_RX_WFULL_INT_ST (BIT(2))
#define I2S_I2S_TX_PUT_DATA_INT_ST (BIT(1))
#define I2S_I2S_RX_TAKE_DATA_INT_ST (BIT(0))

 #define I2SINT_ENA (DR_REG_I2S_BASE + 0x0014)
#define I2S_I2S_TX_REMPTY_INT_ENA (BIT(5))
#define I2S_I2S_TX_WFULL_INT_ENA (BIT(4))
#define I2S_I2S_RX_REMPTY_INT_ENA (BIT(3))
#define I2S_I2S_RX_WFULL_INT_ENA (BIT(2))
#define I2S_I2S_TX_PUT_DATA_INT_ENA (BIT(1))
#define I2S_I2S_RX_TAKE_DATA_INT_ENA (BIT(0))

 #define I2SINT_CLR (DR_REG_I2S_BASE + 0x0018)
#define I2S_I2S_TX_REMPTY_INT_CLR (BIT(5))
#define I2S_I2S_TX_WFULL_INT_CLR (BIT(4))
#define I2S_I2S_RX_REMPTY_INT_CLR (BIT(3))
#define I2S_I2S_RX_WFULL_INT_CLR (BIT(2))
#define I2S_I2S_PUT_DATA_INT_CLR (BIT(1))
#define I2S_I2S_TAKE_DATA_INT_CLR (BIT(0))

#define I2STIMING (DR_REG_I2S_BASE + 0x001c)
#define I2S_TRANS_BCK_IN_INV (BIT(22))
#define I2S_RECE_DSYNC_SW (BIT(21))
#define I2S_TRANS_DSYNC_SW (BIT(20))
#define I2S_RECE_BCK_OUT_DELAY 0x00000003
#define I2S_RECE_BCK_OUT_DELAY_S 18
#define I2S_RECE_WS_OUT_DELAY 0x00000003
#define I2S_RECE_WS_OUT_DELAY_S 16
#define I2S_TRANS_SD_OUT_DELAY 0x00000003
#define I2S_TRANS_SD_OUT_DELAY_S 14
#define I2S_TRANS_WS_OUT_DELAY 0x00000003
#define I2S_TRANS_WS_OUT_DELAY_S 12
#define I2S_TRANS_BCK_OUT_DELAY 0x00000003
#define I2S_TRANS_BCK_OUT_DELAY_S 10
#define I2S_RECE_SD_IN_DELAY 0x00000003
#define I2S_RECE_SD_IN_DELAY_S 8
#define I2S_RECE_WS_IN_DELAY 0x00000003
#define I2S_RECE_WS_IN_DELAY_S 6
#define I2S_RECE_BCK_IN_DELAY 0x00000003
#define I2S_RECE_BCK_IN_DELAY_S 4
#define I2S_TRANS_WS_IN_DELAY 0x00000003
#define I2S_TRANS_WS_IN_DELAY_S 2
#define I2S_TRANS_BCK_IN_DELAY 0x00000003
#define I2S_TRANS_BCK_IN_DELAY_S 0

#define I2S_FIFO_CONF (DR_REG_I2S_BASE + 0x0020)
#define I2S_I2S_RX_FIFO_MOD 0x00000007
#define I2S_I2S_RX_FIFO_MOD_S 16
#define I2S_I2S_TX_FIFO_MOD 0x00000007
#define I2S_I2S_TX_FIFO_MOD_S 13
#define I2S_I2S_DSCR_EN (BIT(12))
#define I2S_I2S_TX_DATA_NUM 0x0000003F
#define I2S_I2S_TX_DATA_NUM_S 6
#define I2S_I2S_RX_DATA_NUM 0x0000003F
#define I2S_I2S_RX_DATA_NUM_S 0


#define I2SRXEOF_NUM (DR_REG_I2S_BASE + 0x0024)
#define I2S_I2S_RX_EOF_NUM 0xFFFFFFFF
#define I2S_I2S_RX_EOF_NUM_S 0

#define I2SCONF_SIGLE_DATA (DR_REG_I2S_BASE + 0x0028)
#define I2S_I2S_SIGLE_DATA 0xFFFFFFFF
#define I2S_I2S_SIGLE_DATA_S 0

#define I2SCONF_CHAN (DR_REG_I2S_BASE + 0x002c)
#define I2S_RX_CHAN_MOD 0x00000003
#define I2S_RX_CHAN_MOD_S 3
#define I2S_TX_CHAN_MOD 0x00000007
#define I2S_TX_CHAN_MOD_S 0

//We need some defines that aren't in some RTOS SDK versions. Define them here if we can't find them.
#ifndef i2c_bbpll
#define i2c_bbpll                                 0x67
#define i2c_bbpll_en_audio_clock_out            4
#define i2c_bbpll_en_audio_clock_out_msb        7
#define i2c_bbpll_en_audio_clock_out_lsb        7
#define i2c_bbpll_hostid                           4

#define i2c_writeReg_Mask(block, host_id, reg_add, Msb, Lsb, indata)  rom_i2c_writeReg_Mask(block, host_id, reg_add, Msb, Lsb, indata)
#define i2c_readReg_Mask(block, host_id, reg_add, Msb, Lsb)  rom_i2c_readReg_Mask(block, host_id, reg_add, Msb, Lsb)
#define i2c_writeReg_Mask_def(block, reg_add, indata) \
      i2c_writeReg_Mask(block, block##_hostid,  reg_add,  reg_add##_msb,  reg_add##_lsb,  indata)
#define i2c_readReg_Mask_def(block, reg_add) \
      i2c_readReg_Mask(block, block##_hostid,  reg_add,  reg_add##_msb,  reg_add##_lsb)
#endif
#ifndef ETS_SLC_INUM
#define ETS_SLC_INUM       1
#endif

unsigned int * ROCODE i2sGetNextBuffer2() {
  SOUNDSYS *ss = (SOUNDSYS *)SOUNDSYSPTR;
  int next = (ss->lastidx + 1) % I2SDMABUFCNT;
  if (next == ss->dmaidx) {
    return NULL;
  }
  ss->lastidx = next;
  return ss->i2sBuf[next];
}

void ROCODE i2sPollSoundStatus() {
  SOUNDSYS *ss = (SOUNDSYS *)SOUNDSYSPTR;

  if (ss == NULL)
    return;

  uint32 slc_intr_status = READ_PERI_REG(SLC_INT_STATUS);
	if (slc_intr_status & SLC_RX_EOF_INT_ST) {
    WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);//slc_intr_status);

    ss->dmaidx = (ss->dmaidx + 1) % I2SDMABUFCNT;
		if (ss->dmaidx == ss->lastidx) {
			ss->underrunCnt++;
		}
	}
}

unsigned int * ROCODE i2sGetNextBuffer() {
  unsigned int *buf = i2sGetNextBuffer2();

  if (!NOSDK)
    return buf;

  i2sPollSoundStatus();
  if (buf != NULL)
    return buf;
  return i2sGetNextBuffer2();
}

//This routine is called as soon as the DMA routine has something to tell us. All we
//handle here is the RX_EOF_INT status, which indicate the DMA has sent a buffer whose
//descriptor has the 'EOF' field set to 1.
LOCAL void slc_isr(void *arg) {
	//struct dma_desc *finishedDesc;
	uint32 slc_intr_status;

	//Grab int status
	slc_intr_status = READ_PERI_REG(SLC_INT_STATUS);
	//clear all intr flags
	WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);//slc_intr_status);
	if (slc_intr_status & SLC_RX_EOF_INT_ST) {
    SOUNDSYS *ss = (SOUNDSYS *)SOUNDSYSPTR;
		//The DMA subsystem is done with this block: Push it on the queue so it can be re-used.
		//finishedDesc=(struct dma_desc*)READ_PERI_REG(SLC_RX_EOF_DES_ADDR);
		//Dump the buffer on the queue so the rest of the software can fill it.
		//unsigned int *ptr = (unsigned int *)(&finishedDesc->buf_ptr);
    //dmaidx = (ptr - buffer) / I2SDMABUFLEN;
    ss->dmaidx = (ss->dmaidx + 1) % I2SDMABUFCNT;
		if (ss->dmaidx == ss->lastidx) {
			//All buffers are empty. This means we have an underflow on our hands.
			ss->underrunCnt++;
		}
	}
}

#define BASEFREQ (160000000L)
#define ABS(x) (((x)>0)?(x):(-(x)))

//Set the I2S sample rate, in HZ
void ROCODE i2sSetRate(int rate, int lockBitcount) {
	//Find closest divider 
	int bestclkmdiv = 0, bestbckdiv = 0, bestfreq=0;
	int tstfreq;
	int bckdiv, clkmdiv, bits;
	/*
		CLK_I2S = 160MHz / I2S_CLKM_DIV_NUM
		BCLK = CLK_I2S / I2S_BCK_DIV_NUM
		WS = BCLK/ 2 / (16 + I2S_BITS_MOD)
		Note that I2S_CLKM_DIV_NUM must be >5 for I2S data
		I2S_CLKM_DIV_NUM - 5-63
		I2S_BCK_DIV_NUM - 2-63
		
		We also have the option to send out more than 2x16 bit per sample. Most I2S codecs will
		ignore the extra bits and in the case of the 'fake' PWM/delta-sigma outputs, they will just lower the output
		voltage a bit, so we add them when it makes sense. Some of them, however, won't accept it, that's
		why we have the option not to do this.
	*/
	for (bckdiv=2; bckdiv<64; bckdiv++) {
		for (clkmdiv=5; clkmdiv<64; clkmdiv++) {
			for (bits=17; bits<(lockBitcount?18:18); bits++) {
				tstfreq=BASEFREQ/(bckdiv*clkmdiv*bits*2);
				if (ABS(rate-tstfreq)<ABS(rate-bestfreq)) {
					bestfreq=tstfreq;
					bestclkmdiv=clkmdiv;
					bestbckdiv=bckdiv;
				}
			}
		}
	}

	CLEAR_PERI_REG_MASK(I2SCONF, I2S_TRANS_SLAVE_MOD|
						(I2S_BITS_MOD<<I2S_BITS_MOD_S)|
						(I2S_BCK_DIV_NUM <<I2S_BCK_DIV_NUM_S)|
						(I2S_CLKM_DIV_NUM<<I2S_CLKM_DIV_NUM_S));
	/*SET_PERI_REG_MASK(I2SCONF, I2S_RIGHT_FIRST|I2S_MSB_RIGHT|I2S_RECE_SLAVE_MOD|
						I2S_RECE_MSB_SHIFT|I2S_TRANS_MSB_SHIFT|
						((bestbits-16)<<I2S_BITS_MOD_S)|
						(((bestbckdiv)&I2S_BCK_DIV_NUM )<<I2S_BCK_DIV_NUM_S)|
						(((bestclkmdiv)&I2S_CLKM_DIV_NUM)<<I2S_CLKM_DIV_NUM_S));*/
	SET_PERI_REG_MASK(I2SCONF, I2S_RIGHT_FIRST|I2S_MSB_RIGHT|
						I2S_RECE_MSB_SHIFT|I2S_TRANS_MSB_SHIFT|
						(1<<I2S_BITS_MOD_S)|
						(((bestbckdiv)&I2S_BCK_DIV_NUM )<<I2S_BCK_DIV_NUM_S)|
						(((bestclkmdiv)&I2S_CLKM_DIV_NUM)<<I2S_CLKM_DIV_NUM_S));
  WRITE_PERI_REG(I2STIMING, (3 << I2S_TRANS_BCK_OUT_DELAY_S) | (3 << I2S_TRANS_BCK_IN_DELAY_S));
	//SET_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN|(0<<I2S_I2S_RX_FIFO_MOD_S)|(0<<I2S_I2S_TX_FIFO_MOD_S));

	//tx/rx binaureal
	SET_PERI_REG_MASK(I2SCONF_CHAN, (2<<I2S_TX_CHAN_MOD_S)|(2<<I2S_RX_CHAN_MOD_S));
}

// Initialize I2S subsystem for DMA circular buffer use
//
void ROCODE i2sInit(int rate, int lockBits, SOUNDSYS *ss) {
	int x, y;

  SOUNDSYSPTR = ss;

  memset(ss, 0, sizeof(SOUNDSYS));

  unsigned int *ptr = ss->buffer;
	//First, take care of the DMA buffers.
	for (y=0; y<I2SDMABUFCNT; y++, ptr += I2SDMABUFLEN) {
		//Allocate memory for this DMA sample buffer.
		ss->i2sBuf[y]=ptr;
//for (int i = 0; i < 256; i++)
//  ptr[i] = i << 6;
	}

	//Reset DMA
	SET_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST|SLC_TXLINK_RST);
	CLEAR_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST|SLC_TXLINK_RST);

	//Clear DMA int flags
	SET_PERI_REG_MASK(SLC_INT_CLR,  0xffffffff);
	CLEAR_PERI_REG_MASK(SLC_INT_CLR,  0xffffffff);

	//Enable and configure DMA
	CLEAR_PERI_REG_MASK(SLC_CONF0, (SLC_MODE<<SLC_MODE_S));
	SET_PERI_REG_MASK(SLC_CONF0,(1<<SLC_MODE_S));
	SET_PERI_REG_MASK(SLC_RX_DSCR_CONF,SLC_INFOR_NO_REPLACE|SLC_TOKEN_NO_REPLACE);
	CLEAR_PERI_REG_MASK(SLC_RX_DSCR_CONF, SLC_RX_FILL_EN|SLC_RX_EOF_MODE | SLC_RX_FILL_MODE);

	//Initialize DMA buffer descriptors in such a way that they will form a circular
	//buffer.
	for (x=0; x<I2SDMABUFCNT; x++) {
		ss->i2sBufDesc[x].owner=1;
		ss->i2sBufDesc[x].eof=1;
		ss->i2sBufDesc[x].sub_sof=0;
		ss->i2sBufDesc[x].datalen=I2SDMABUFLEN*4;
		ss->i2sBufDesc[x].blocksize=I2SDMABUFLEN*4;
		ss->i2sBufDesc[x].buf_ptr=(uint32_t)&ss->i2sBuf[x][0];
		ss->i2sBufDesc[x].unused=0;
		ss->i2sBufDesc[x].next_link_ptr=(int)((x<(I2SDMABUFCNT-1))?(&ss->i2sBufDesc[x+1]):(&ss->i2sBufDesc[0]));
	}

	//Feed dma the 1st buffer desc addr
	//To send data to the I2S subsystem, counter-intuitively we use the RXLINK part, not the TXLINK as you might
	//expect. The TXLINK part still needs a valid DMA descriptor, even if it's unused: the DMA engine will throw
	//an error at us otherwise. Just feed it any random descriptor.
	CLEAR_PERI_REG_MASK(SLC_TX_LINK,SLC_TXLINK_DESCADDR_MASK);
	SET_PERI_REG_MASK(SLC_TX_LINK, ((uint32)&ss->i2sBufDesc[1]) & SLC_TXLINK_DESCADDR_MASK); //any random desc is OK, we don't use TX but it needs something valid
	CLEAR_PERI_REG_MASK(SLC_RX_LINK,SLC_RXLINK_DESCADDR_MASK);
	SET_PERI_REG_MASK(SLC_RX_LINK, ((uint32)&ss->i2sBufDesc[0]) & SLC_RXLINK_DESCADDR_MASK);

	//Attach the DMA interrupt
  if (!NOSDK)
    ets_isr_attach(ETS_SLC_INUM, slc_isr, 0);
	//Enable DMA operation intr
	WRITE_PERI_REG(SLC_INT_ENA,  SLC_RX_EOF_INT_ENA);
	//clear any interrupt flags that are set
	WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);
	///enable DMA intr in cpu
  if (!NOSDK)
    ETS_INTR_ENABLE(ETS_SLC_INUM);
  /*__asm__ __volatile__(
      "movi       a2, 2\n"
      "wsr        a2, intenable\n"
      "rsync          \n"
      : : :"a2", "memory");*/

	//Start transmission
	SET_PERI_REG_MASK(SLC_TX_LINK, SLC_TXLINK_START);
	SET_PERI_REG_MASK(SLC_RX_LINK, SLC_RXLINK_START);

//----

	//Init pins to i2s functions
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, 1); //FUNC_I2SO_DATA);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, 1); //FUNC_I2SO_WS);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, 1); //FUNC_I2SO_BCK);

	//Enable clock to i2s subsystem
	//i2c_writeReg_Mask_def(i2c_bbpll, i2c_bbpll_en_audio_clock_out, 1);

	//Reset I2S subsystem
	CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);
	SET_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);
	CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);

	//Select 16bits per channel (FIFO_MOD=0), no DMA access (FIFO only)
	CLEAR_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN|(I2S_I2S_RX_FIFO_MOD<<I2S_I2S_RX_FIFO_MOD_S)|(I2S_I2S_TX_FIFO_MOD<<I2S_I2S_TX_FIFO_MOD_S));
	//Enable DMA in i2s subsystem
	SET_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN);

	//Clear int
	SET_PERI_REG_MASK(I2SINT_CLR,   I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
			I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);
	CLEAR_PERI_REG_MASK(I2SINT_CLR, I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
			I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);

	//tx/rx binaureal
	CLEAR_PERI_REG_MASK(I2SCONF_CHAN, (I2S_TX_CHAN_MOD<<I2S_TX_CHAN_MOD_S)|(I2S_RX_CHAN_MOD<<I2S_RX_CHAN_MOD_S));

	//trans master&rece slave,MSB shift,right_first,msb right
	/*CLEAR_PERI_REG_MASK(I2SCONF, I2S_TRANS_SLAVE_MOD|
						(I2S_BITS_MOD<<I2S_BITS_MOD_S)|
						(I2S_BCK_DIV_NUM <<I2S_BCK_DIV_NUM_S)|
						(I2S_CLKM_DIV_NUM<<I2S_CLKM_DIV_NUM_S));
	SET_PERI_REG_MASK(I2SCONF, I2S_RIGHT_FIRST|I2S_MSB_RIGHT|I2S_RECE_SLAVE_MOD|
						I2S_RECE_MSB_SHIFT|I2S_TRANS_MSB_SHIFT|
						((16&I2S_BCK_DIV_NUM )<<I2S_BCK_DIV_NUM_S)|
						((7&I2S_CLKM_DIV_NUM)<<I2S_CLKM_DIV_NUM_S));*/
  i2sSetRate(rate, lockBits);

	//No idea if ints are needed...
	//clear int
	SET_PERI_REG_MASK(I2SINT_CLR,   I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
			I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);
	CLEAR_PERI_REG_MASK(I2SINT_CLR,   I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
			I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);
	//enable int
	//SET_PERI_REG_MASK(I2SINT_ENA,   I2S_I2S_TX_REMPTY_INT_ENA|I2S_I2S_TX_WFULL_INT_ENA|
	//    I2S_I2S_RX_REMPTY_INT_ENA|I2S_I2S_TX_PUT_DATA_INT_ENA|I2S_I2S_RX_TAKE_DATA_INT_ENA);

	//Start transmission
	SET_PERI_REG_MASK(I2SCONF,I2S_I2S_TX_START);
}

void ROCODE i2sStop() {
	CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_TX_START);
	SET_PERI_REG_MASK(I2SINT_CLR,   I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
			I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);
	CLEAR_PERI_REG_MASK(I2SINT_CLR,   I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
			I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);
}
