
#ifndef BOOT_IMAGE
#define USE_OPTIMIZE_PRINTF   (1)
#endif
#define LIGHTIVY_FIRMWARE     (1)

#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_interface.h"
#include "mem.h"

#include "spi_register.h"
#include "spi_flash.h"

#include "system.h"
#include "font8x8_basic.h"

#include "diskio/sd_mmc.h"
#include "diskio/diskio.h"
#include "fatfs/ff.h"
#include "eboot_command.h"

#include <stdarg.h>
#include "printf.h"

#ifndef BOOT_IMAGE

#define SPI_FLASH_SIZE_MAP    4
#define SYSTEM_PARTITION_RF_CAL_ADDR						0x3fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR						0x3fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR				0x3fd000

static const partition_item_t at_partition_table[] = {
    { SYSTEM_PARTITION_RF_CAL,  						SYSTEM_PARTITION_RF_CAL_ADDR, 						0x1000},
    { SYSTEM_PARTITION_PHY_DATA, 						SYSTEM_PARTITION_PHY_DATA_ADDR, 					0x1000},
    { SYSTEM_PARTITION_SYSTEM_PARAMETER, 				SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR, 			0x3000},
};

void ICACHE_FLASH_ATTR user_pre_init(void)
{
  if(!system_partition_table_regist(at_partition_table, sizeof(at_partition_table)/sizeof(at_partition_table[0]),SPI_FLASH_SIZE_MAP)) {
    // TODO: Consider reflashing after N reboots.
    os_printf("System partition init failure!\n");
    while(1);
  }
}

/*uint32 user_iram_memory_is_enabled(void)
{
    return 1;
}*/
#endif

void ROCODE gpio16_output_conf(void)
{
    WRITE_PERI_REG(PAD_XPD_DCDC_CONF,
                   (READ_PERI_REG(PAD_XPD_DCDC_CONF) & 0xffffffbc) | (uint32)0x1); 	// mux configuration for XPD_DCDC to output rtc_gpio0

    WRITE_PERI_REG(RTC_GPIO_CONF,
                   (READ_PERI_REG(RTC_GPIO_CONF) & (uint32)0xfffffffe) | (uint32)0x0);	//mux configuration for out enable

    WRITE_PERI_REG(RTC_GPIO_ENABLE,
                   (READ_PERI_REG(RTC_GPIO_ENABLE) & (uint32)0xfffffffe) | (uint32)0x1);	//out enable
}

void ROCODE gpio16_output_set(uint8 value)
{
    WRITE_PERI_REG(RTC_GPIO_OUT,
                   (READ_PERI_REG(RTC_GPIO_OUT) & (uint32)0xfffffffe) | (uint32)(value & 1));
}

void ROCODE gpio16_input_conf(void)
{
    WRITE_PERI_REG(PAD_XPD_DCDC_CONF,
                   (READ_PERI_REG(PAD_XPD_DCDC_CONF) & 0xffffffbc) | (uint32)0x1); 	// mux configuration for XPD_DCDC and rtc_gpio0 connection

    WRITE_PERI_REG(RTC_GPIO_CONF,
                   (READ_PERI_REG(RTC_GPIO_CONF) & (uint32)0xfffffffe) | (uint32)0x0);	//mux configuration for out enable

    WRITE_PERI_REG(RTC_GPIO_ENABLE,
                   READ_PERI_REG(RTC_GPIO_ENABLE) & (uint32)0xfffffffe);	//out disable
}

uint8 ROCODE gpio16_input_get(void)
{
    return (uint8)(READ_PERI_REG(RTC_GPIO_IN_DATA) & 1);
}

void ROCODE LCD_CS_SET() {
  gpio16_output_set(1);
}

void ROCODE LCD_CS_CLEAR() {
  gpio16_output_set(0);
}

void ROCODE delay_us(uint16_t d) {
  os_delay_us(d);
}

void ROCODE oc_spi_send_byte(uint8_t d) {
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
	WRITE_PERI_REG(SPI_USER1(1), ((7&SPI_USR_MOSI_BITLEN)<<SPI_USR_MOSI_BITLEN_S));
  WRITE_PERI_REG(SPI_W0(1), d);
  SET_PERI_REG_MASK(SPI_CMD(1), SPI_USR);
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
}

void ROCODE oc_spi_send_bytes64(uint32_t *src) {
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
	WRITE_PERI_REG(SPI_USER1(1), ((511&SPI_USR_MOSI_BITLEN)<<SPI_USR_MOSI_BITLEN_S));
  for (int i = 0; i < 64; i += 4)
    WRITE_PERI_REG(SPI_W0(1) + i, *src++);
  SET_PERI_REG_MASK(SPI_CMD(1), SPI_USR);
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
}

uint8_t ROCODE oc_spi_read_byte() {
	//while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
	WRITE_PERI_REG(SPI_USER1(1), (7<<SPI_USR_MISO_BITLEN_S) | (7<<SPI_USR_MOSI_BITLEN_S));
  WRITE_PERI_REG(SPI_W0(1), 0xff);
  SET_PERI_REG_MASK(SPI_CMD(1), SPI_USR);
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  return READ_PERI_REG(SPI_W0(1));
}

void ROCODE oc_spi_read_bytes64(uint32_t *dst) {
	//while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
	WRITE_PERI_REG(SPI_USER1(1), (511<<SPI_USR_MISO_BITLEN_S) | (511<<SPI_USR_MOSI_BITLEN_S));
  for (int i = 0; i < 64; i += 4)
    WRITE_PERI_REG(SPI_W0(1) + i, 0xffffffff);
  SET_PERI_REG_MASK(SPI_CMD(1), SPI_USR);
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  for (int i = 0; i < 64; i += 4)
    *dst++ = READ_PERI_REG(SPI_W0(1) + i);
}

void ROCODE sendword(uint16_t d) {
	//while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
	WRITE_PERI_REG(SPI_USER1(1), ((15&SPI_USR_MOSI_BITLEN)<<SPI_USR_MOSI_BITLEN_S));
  WRITE_PERI_REG(SPI_W0(1), d);
  SET_PERI_REG_MASK(SPI_CMD(1), SPI_USR);
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
}

void sendwords(uint32_t *data, uint32_t size) {
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
	WRITE_PERI_REG(SPI_USER1(1), (((size * 32 - 1)&SPI_USR_MOSI_BITLEN)<<SPI_USR_MOSI_BITLEN_S));
  int i;
  for (i = 0; i < size; i++)
    WRITE_PERI_REG((SPI_W0(1) + i * 4), data[i]);
  SET_PERI_REG_MASK(SPI_CMD(1), SPI_USR);
	//while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
}

void ROCODE LCD_DATA_WR(uint8_t data) {
  oc_spi_send_byte(data);
}

void ROCODE sleep_ms(uint32_t ms) {
  while(ms--) {
    os_delay_us(1000);
#ifndef BOOT_IMAGE
    system_soft_wdt_feed();
#else
    HARD_WDT_FEED
#endif
  }
}

void ROCODE sleep_us(uint32_t us) {
  while(us >= 1000) {
    os_delay_us(1000);
#ifndef BOOT_IMAGE
    system_soft_wdt_feed();
#else
    HARD_WDT_FEED
#endif
    us -= 1000;
  }
  os_delay_us(us);
}

// ST7735 specific commands used in init
#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B // PASET
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

void ROCODE lcdWriteCommand(uint8_t uiCommand )
{
    GPIO_OUTPUT_SET(PIN_LCD_RS, 0);
    LCD_CS_CLEAR();
    LCD_DATA_WR( uiCommand );
    LCD_CS_SET();
}

void ROCODE lcdWriteData(uint8_t uiData )
{
    GPIO_OUTPUT_SET(PIN_LCD_RS, 1);
    LCD_CS_CLEAR();
    LCD_DATA_WR( uiData );
    LCD_CS_SET();
}

#define lcdWriteData16(uiData)  sendword(uiData);

unsigned char ROCODE oc_pgm_read_byte(const unsigned char *ptr)
{
	union {
		uint32_t u;   // unsigned integer
		uint8_t b[4]; // bytes
	} tmp;

	// access to ROM memory should be 4-bytes aligned!
	// so read the whole 4-bytes at aligned address
	tmp.u = *(const uint32_t*)((uint32_t)ptr & ~(4-1));

	// and then, get the requested byte
	return tmp.b[(uint32_t)ptr & (4-1)];
}

uint8_t ROCODE pgm_read_byte(const void *ptr)
{
	union {
		uint32_t u;   // unsigned integer
		uint8_t b[4]; // bytes
	} tmp;

	// access to ROM memory should be 4-bytes aligned!
	// so read the whole 4-bytes at aligned address
	tmp.u = *(const uint32_t*)((uint32_t)ptr & ~(4-1));

	// and then, get the requested byte
	return tmp.b[(uint32_t)ptr & (4-1)];
}

void ROCODE lcd_init(void)
{
  sleep_ms(200);

    lcdWriteCommand(ST7735_SWRESET);
    sleep_ms(150);
    lcdWriteCommand(ST7735_SLPOUT);
    sleep_ms(150);
    lcdWriteCommand(ST7735_FRMCTR1);
    lcdWriteData(0x07);
    lcdWriteData(0x2C);
    lcdWriteData(0x2D);
    lcdWriteCommand(ST7735_FRMCTR2);
    lcdWriteData(0x07);
    lcdWriteData(0x2C);
    lcdWriteData(0x2D);
    lcdWriteCommand(ST7735_FRMCTR3);
    lcdWriteData(0x07);
    lcdWriteData(0x2C);
    lcdWriteData(0x2D);
    lcdWriteData(0x07);
    lcdWriteData(0x2C);
    lcdWriteData(0x2D);
    lcdWriteCommand(ST7735_INVCTR);
    lcdWriteData(0x07);
    lcdWriteCommand(ST7735_PWCTR1);
    lcdWriteData(0xA2);
    lcdWriteData(0x02);
    lcdWriteData(0x84);
    lcdWriteCommand(ST7735_PWCTR2);
    lcdWriteData(0xC5);
    lcdWriteCommand(ST7735_PWCTR3);
    lcdWriteData(0x0A);
    lcdWriteData(0x00);
    lcdWriteCommand(ST7735_PWCTR4);
    lcdWriteData(0x8A);
    lcdWriteData(0x2A);
    lcdWriteCommand(ST7735_PWCTR5);
    lcdWriteData(0x8A);
    lcdWriteData(0xEE);
    lcdWriteCommand(ST7735_VMCTR1);
    lcdWriteData(0x0E);
    lcdWriteCommand(ST7735_INVOFF);
    lcdWriteCommand(ST7735_MADCTL);
    lcdWriteData(0x60);
    lcdWriteCommand(ST7735_COLMOD);
    lcdWriteData(0x05);
    lcdWriteCommand(ST7735_CASET);
    lcdWriteData(0x00);
    lcdWriteData(0x00);
    lcdWriteData(0x00);
    lcdWriteData(0x9F);
    lcdWriteCommand(ST7735_RASET);
    lcdWriteData(0x00);
    lcdWriteData(0x00);
    lcdWriteData(0x00);
    lcdWriteData(0x7F);
    lcdWriteCommand(ST7735_GMCTRP1);
    lcdWriteData(0x02);
    lcdWriteData(0x1c);
    lcdWriteData(0x07);
    lcdWriteData(0x12);
    lcdWriteData(0x37);
    lcdWriteData(0x32);
    lcdWriteData(0x29);
    lcdWriteData(0x2d);
    lcdWriteData(0x29);
    lcdWriteData(0x25);
    lcdWriteData(0x2B);
    lcdWriteData(0x39);
    lcdWriteData(0x00);
    lcdWriteData(0x01);
    lcdWriteData(0x03);
    lcdWriteData(0x10);
    lcdWriteCommand(ST7735_GMCTRN1);
    lcdWriteData(0x03);
    lcdWriteData(0x1d);
    lcdWriteData(0x07);
    lcdWriteData(0x06);
    lcdWriteData(0x2E);
    lcdWriteData(0x2C);
    lcdWriteData(0x29);
    lcdWriteData(0x2D);
    lcdWriteData(0x2E);
    lcdWriteData(0x2E);
    lcdWriteData(0x37);
    lcdWriteData(0x3F);
    lcdWriteData(0x00);
    lcdWriteData(0x00);
    lcdWriteData(0x02);
    lcdWriteData(0x10);
    lcdWriteCommand(ST7735_NORON);
    sleep_ms(10);
    lcdWriteCommand(ST7735_DISPON);
    sleep_ms(100);
}

void ROCODE gpio_set_pin_level(int pin, int level) {
  if (pin == PIN_LCD_DATA) {
    oc_spi_send_byte(0xff);
    return;
  }

  if (level)
    gpio_output_set(1 << pin, 0, 0, 0);
  else
    gpio_output_set(0, 1 << pin, 0, 0);
}

int ROCODE gpio_get_pin_level(int pin) {
  return (gpio_input_get() & (1 << pin)) ? 1 : 0;
}

#define REG_UART_BASE(i)                (0x60000000 + (i)*0xf00)
#define UART_STATUS(i)                  (REG_UART_BASE(i) + 0x1C)
#define UART_TXFIFO_CNT                 0x000000FF
#define UART_TXFIFO_CNT_S               16
#define UART_FIFO(i)                    REG_UART_BASE(i)

void ROCODE debug_flush(uint32_t cnt) {
  while (true){
    uint32_t fifo_cnt = READ_PERI_REG(UART_STATUS(0)) & (UART_TXFIFO_CNT<<UART_TXFIFO_CNT_S);
    if (((fifo_cnt >> UART_TXFIFO_CNT_S) & UART_TXFIFO_CNT) < cnt) {
      break;
    }
    HARD_WDT_FEED
  }
}

void ROCODE debug_out(char c, void *dummy) {
  debug_flush(126);
  WRITE_PERI_REG(UART_FIFO(0), c);
}

void ROCODE debug(const char *str, ...) {
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
  va_list va;
  va_start(va, str);
  __vfctprintf(debug_out, NULL, str, va);
  va_end(va);
  debug_flush(1);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_GPIO1);
}

#ifndef BOOT_IMAGE
void ROCODE reset(uint32_t mode) {
  if (mode > 0) {
    struct eboot_command cmd;
    memset(&cmd, 0, sizeof(struct eboot_command));
    cmd.action = ACTION_COPY_RAW;
    cmd.args[0] = mode - 1;
    eboot_command_write(&cmd);
  }
  system_restart();
}
#endif

void ROCODE lcd_mode() {
#ifdef  BITBANG_SPI
  gpio_output_set(0, 0, 0, 1 << PIN_LCD_CLK);
  gpio_output_set(0, 0, 0, 1 << PIN_LCD_DATA);
  gpio_output_set(0, 0, 0, 1 << 12);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, 2);//configure io to spi mode
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 2);//configure io to spi mode	
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, 2);//configure io to spi mode	
#endif

	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  WRITE_PERI_REG(SPI_USER(1), SPI_USR_MOSI | SPI_CK_I_EDGE);
  CLEAR_PERI_REG_MASK(PERIPHS_IO_MUX, SPI1_CLK_EQU_SYS_CLK);
  //WRITE_PERI_REG(SPI_CLOCK(1), (7 << SPI_CLKCNT_N_S) | (7 << SPI_CLKCNT_H_S) | (3 << SPI_CLKCNT_L_S));  // 10MHz
  //WRITE_PERI_REG(SPI_CLOCK(1), (3 << SPI_CLKCNT_N_S) | (3 << SPI_CLKCNT_H_S) | (1 << SPI_CLKCNT_L_S));  // 20MHz
  WRITE_PERI_REG(SPI_CLOCK(1), (2 << SPI_CLKCNT_N_S) | (2 << SPI_CLKCNT_H_S) | (1 << SPI_CLKCNT_L_S));  // 27MHz
  //WRITE_PERI_REG(SPI_CLOCK(1), (1 << SPI_CLKCNT_N_S) | (1 << SPI_CLKCNT_H_S) | (0 << SPI_CLKCNT_L_S));  // 40MHz - works, but with noise on screen
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
}

void ROCODE sd_mode() {
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  WRITE_PERI_REG(SPI_USER(1), SPI_USR_MOSI | SPI_CK_I_EDGE | 1); // SPI_DOUTDIN
  CLEAR_PERI_REG_MASK(PERIPHS_IO_MUX, SPI1_CLK_EQU_SYS_CLK);
  WRITE_PERI_REG(SPI_CLOCK(1), (3 << SPI_CLKCNT_N_S) | (3 << SPI_CLKCNT_H_S) | (1 << SPI_CLKCNT_L_S));  // 20MHz
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available

#ifdef  BITBANG_SPI
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14);
  gpio_output_set(0, 1 << PIN_LCD_CLK, 1 << PIN_LCD_CLK, 0);
  gpio_output_set(1 << PIN_LCD_DATA, 0, 1 << PIN_LCD_DATA, 0);
  gpio_output_set(0, 0, 0, 1 << 12);
#endif
}

uint16_t IRDATA max_sd_speed;
uint16_t IRDATA cur_sd_speed;

void ROCODE init_sd_speed() {
  max_sd_speed = 20;
}

uint32_t ROCODE lower_sd_speed() {
  if (max_sd_speed > 10) {
    max_sd_speed = 10;
    return 10;
  }
  if (max_sd_speed > 4) {
    max_sd_speed = 4;
    return 4;
  }
  if (max_sd_speed > 2) {
    max_sd_speed = 2;
    return 2;
  }
  if (max_sd_speed > 1) {
    max_sd_speed = 1;
    return 1;
  }
  return 0;
}

void ROCODE oc_set_sd_speed(uint32_t mhz) {
  if (mhz > max_sd_speed)
    mhz = max_sd_speed;
  if (mhz < 1) {
    WRITE_PERI_REG(SPI_CLOCK(1), (19 << SPI_CLKDIV_PRE_S) | (7 << SPI_CLKCNT_N_S) | (7 << SPI_CLKCNT_H_S) | (3 << SPI_CLKCNT_L_S));  // 500kHz
    cur_sd_speed = 1;
    return;
  }
  if (mhz < 3) {
    WRITE_PERI_REG(SPI_CLOCK(1), (3 << SPI_CLKDIV_PRE_S) | (7 << SPI_CLKCNT_N_S) | (7 << SPI_CLKCNT_H_S) | (3 << SPI_CLKCNT_L_S));  // 2.5MHz
    cur_sd_speed = 2;
    return;
  }
  if (mhz < 5) {
    WRITE_PERI_REG(SPI_CLOCK(1), (2 << SPI_CLKDIV_PRE_S) | (7 << SPI_CLKCNT_N_S) | (7 << SPI_CLKCNT_H_S) | (3 << SPI_CLKCNT_L_S));  // 3.33MHz
    cur_sd_speed = 3;
    return;
  }
  // Most cards I tested report 6 MHz on SPI interface, but they work on 20 MHz.
  if (mhz < 15 && max_sd_speed < 20) {
    WRITE_PERI_REG(SPI_CLOCK(1), (7 << SPI_CLKCNT_N_S) | (7 << SPI_CLKCNT_H_S) | (3 << SPI_CLKCNT_L_S));  // 10MHz
    cur_sd_speed = 10;
    return;
  }
  WRITE_PERI_REG(SPI_CLOCK(1), (3 << SPI_CLKCNT_N_S) | (3 << SPI_CLKCNT_H_S) | (1 << SPI_CLKCNT_L_S));  // 20MHz
  cur_sd_speed = 20;
}

void ROCODE ram_mode() {
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  SET_PERI_REG_MASK(PERIPHS_IO_MUX, SPI1_CLK_EQU_SYS_CLK);
  WRITE_PERI_REG(SPI_CLOCK(1), SPI_CLK_EQU_SYSCLK);
  //WRITE_PERI_REG(SPI_CLOCK(1), (3 << SPI_CLKCNT_N_S) | (3 << SPI_CLKCNT_H_S) | (1 << SPI_CLKCNT_L_S));  // 20MHz
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
}

void ROCODE ram_write_word(uint32_t addr, uint32_t data) {
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  WRITE_PERI_REG(SPI_USER(1), SPI_USR_COMMAND | SPI_USR_ADDR | SPI_USR_MOSI | SPI_CK_I_EDGE); // SPI_DOUTDIN

	WRITE_PERI_REG(SPI_USER1(1), (23<<SPI_USR_ADDR_BITLEN_S) | (31<<SPI_USR_MOSI_BITLEN_S));
  WRITE_PERI_REG(SPI_USER2(1), (7<<SPI_USR_COMMAND_BITLEN_S) | 0x02);
  WRITE_PERI_REG(SPI_ADDR(1), addr<<8);
  WRITE_PERI_REG(SPI_W0(1), data);
  GPIO_OUTPUT_SET(PIN_RAM_CS, 0);
  SET_PERI_REG_MASK(SPI_CMD(1), SPI_USR);
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  GPIO_OUTPUT_SET(PIN_RAM_CS, 1);
}

uint32_t ROCODE ram_read_word(uint32_t addr) {
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  WRITE_PERI_REG(SPI_USER(1), SPI_USR_COMMAND | SPI_USR_ADDR | SPI_USR_DUMMY | SPI_USR_MISO | SPI_CK_I_EDGE);

	WRITE_PERI_REG(SPI_USER1(1), (23<<SPI_USR_ADDR_BITLEN_S) | (7<<SPI_USR_DUMMY_CYCLELEN_S) | (31<<SPI_USR_MISO_BITLEN_S));
  WRITE_PERI_REG(SPI_USER2(1), (7<<SPI_USR_COMMAND_BITLEN_S) | 0x0B);
  WRITE_PERI_REG(SPI_ADDR(1), addr<<8);
  GPIO_OUTPUT_SET(PIN_RAM_CS, 0);
  SET_PERI_REG_MASK(SPI_CMD(1), SPI_USR);
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  GPIO_OUTPUT_SET(PIN_RAM_CS, 1);
  return READ_PERI_REG(SPI_W0(1));
}

void ROCODE ram_read_start(uint32_t addr, uint32_t size) {
  WRITE_PERI_REG(SPI_USER(1), SPI_USR_COMMAND | SPI_USR_ADDR | SPI_USR_DUMMY | SPI_USR_MISO | SPI_CK_I_EDGE);
	WRITE_PERI_REG(SPI_USER1(1), (23<<SPI_USR_ADDR_BITLEN_S) | (7<<SPI_USR_DUMMY_CYCLELEN_S) | ((size * 8 - 1)<<SPI_USR_MISO_BITLEN_S));
  WRITE_PERI_REG(SPI_USER2(1), (7<<SPI_USR_COMMAND_BITLEN_S) | 0x0B);
  GPIO_OUTPUT_SET(PIN_RAM_CS, 0);
  WRITE_PERI_REG(SPI_ADDR(1), addr<<8);
  SET_PERI_REG_MASK(SPI_CMD(1), SPI_USR);
}

void ROCODE ram_read_end(void *data, uint32_t size) {
  uint32_t *dest = data;
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  GPIO_OUTPUT_SET(PIN_RAM_CS, 1);
  for (int i = 0; i < size; i += 4)
    *dest++ = READ_PERI_REG((SPI_W0(1) + i));
}

void ROCODE ram_read_block(uint32_t addr, void *data, uint32_t size) {
  uint32_t *dest = data;
  WRITE_PERI_REG(SPI_USER(1), SPI_USR_COMMAND | SPI_USR_ADDR | SPI_USR_DUMMY | SPI_USR_MISO | SPI_CK_I_EDGE);
	WRITE_PERI_REG(SPI_USER1(1), (23<<SPI_USR_ADDR_BITLEN_S) | (7<<SPI_USR_DUMMY_CYCLELEN_S) | ((size * 8 - 1)<<SPI_USR_MISO_BITLEN_S));
  WRITE_PERI_REG(SPI_USER2(1), (7<<SPI_USR_COMMAND_BITLEN_S) | 0x0B);
  GPIO_OUTPUT_SET(PIN_RAM_CS, 0);
  WRITE_PERI_REG(SPI_ADDR(1), addr<<8);
  SET_PERI_REG_MASK(SPI_CMD(1), SPI_USR);
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  GPIO_OUTPUT_SET(PIN_RAM_CS, 1);
  for (int i = 0; i < size; i += 4)
    *dest++ = READ_PERI_REG((SPI_W0(1) + i));
}

void ROCODE ram_read(uint32_t addr, void *data, uint32_t size) {
  uint8_t *dest = data;
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  while(size >= 64) {
    ram_read_block(addr, dest, 64);
    size -= 64;
    addr += 64;
    dest += 64;
  }
  if (size > 0)
    ram_read_block(addr, dest, size);
}

void ROCODE ram_write_start(uint32_t addr, void *data, uint32_t size) {
  uint32_t *dest = data;
  WRITE_PERI_REG(SPI_USER(1), SPI_USR_COMMAND | SPI_USR_ADDR | SPI_USR_MOSI | SPI_CK_I_EDGE); // SPI_DOUTDIN
	WRITE_PERI_REG(SPI_USER1(1), (23<<SPI_USR_ADDR_BITLEN_S) | ((size * 8 - 1)<<SPI_USR_MOSI_BITLEN_S));
  WRITE_PERI_REG(SPI_USER2(1), (7<<SPI_USR_COMMAND_BITLEN_S) | 0x02);
  GPIO_OUTPUT_SET(PIN_RAM_CS, 0);
  WRITE_PERI_REG(SPI_ADDR(1), addr<<8);
  for (int i = 0; i < size; i += 4)
    WRITE_PERI_REG((SPI_W0(1) + i), *dest++);
  SET_PERI_REG_MASK(SPI_CMD(1), SPI_USR);
}

void ROCODE ram_write_end() {
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  GPIO_OUTPUT_SET(PIN_RAM_CS, 1);
}

void ROCODE ram_write_block(uint32_t addr, void *data, uint32_t size) {
  uint32_t *dest = data;
  WRITE_PERI_REG(SPI_USER(1), SPI_USR_COMMAND | SPI_USR_ADDR | SPI_USR_MOSI | SPI_CK_I_EDGE); // SPI_DOUTDIN
	WRITE_PERI_REG(SPI_USER1(1), (23<<SPI_USR_ADDR_BITLEN_S) | ((size * 8 - 1)<<SPI_USR_MOSI_BITLEN_S));
  WRITE_PERI_REG(SPI_USER2(1), (7<<SPI_USR_COMMAND_BITLEN_S) | 0x02);
  GPIO_OUTPUT_SET(PIN_RAM_CS, 0);
  WRITE_PERI_REG(SPI_ADDR(1), addr<<8);
  for (int i = 0; i < size; i += 4)
    WRITE_PERI_REG((SPI_W0(1) + i), *dest++);
  SET_PERI_REG_MASK(SPI_CMD(1), SPI_USR);
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  GPIO_OUTPUT_SET(PIN_RAM_CS, 1);
}

void ROCODE ram_write(uint32_t addr, void *data, uint32_t size) {
  uint8_t *dest = data;
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  while(size >= 64) {
    ram_write_block(addr, dest, 64);
    size -= 64;
    addr += 64;
    dest += 64;
  }
  if (size > 0)
    ram_write_block(addr, dest, size);
}

uint32_t ROCODE xram_read_word(uint32_t addr) {
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  WRITE_PERI_REG(SPI_USER(1), SPI_USR_COMMAND | SPI_USR_ADDR | SPI_USR_MISO | SPI_CK_I_EDGE);

	WRITE_PERI_REG(SPI_USER1(1), (23<<SPI_USR_ADDR_BITLEN_S) | (31<<SPI_USR_MISO_BITLEN_S));
  WRITE_PERI_REG(SPI_USER2(1), (7<<SPI_USR_COMMAND_BITLEN_S) | 0x03);
  WRITE_PERI_REG(SPI_ADDR(1), addr<<8);
  GPIO_OUTPUT_SET(PIN_RAM_CS, 0);
  SET_PERI_REG_MASK(SPI_CMD(1), SPI_USR);
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  GPIO_OUTPUT_SET(PIN_RAM_CS, 1);
  return READ_PERI_REG(SPI_W0(1));
}

int ROCODE ram_read_id() {
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  WRITE_PERI_REG(SPI_USER(1), SPI_USR_COMMAND | SPI_USR_ADDR | SPI_USR_MISO | SPI_CK_I_EDGE); // SPI_DOUTDIN

	WRITE_PERI_REG(SPI_USER1(1), (23<<SPI_USR_ADDR_BITLEN_S) | (15<<SPI_USR_MISO_BITLEN_S));
  WRITE_PERI_REG(SPI_USER2(1), (7<<SPI_USR_COMMAND_BITLEN_S) | 0x9F);
  WRITE_PERI_REG(SPI_ADDR(1), 0);
  GPIO_OUTPUT_SET(PIN_RAM_CS, 0);
  SET_PERI_REG_MASK(SPI_CMD(1), SPI_USR);
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  GPIO_OUTPUT_SET(PIN_RAM_CS, 1);
  return (int)READ_PERI_REG(SPI_W0(1));
}

extern void i2sStop();
extern void i2sInit(int rate, int lockBitcount, SOUNDSYS *ss);
extern unsigned int *i2sGetNextBuffer();

#ifndef BOOT_IMAGE
uint32_t IRDATA dramsize;
uint32_t IRDATA iramsize;

void * ROCODE get_mem_block(uint32_t type, uint32_t size) {

  if (type == MEM_TYPE_RAM) {
    if (!NOSDK)
      return (void *)os_malloc_dram(size);

    if (dramsize + size > 80 * 1024)
      return NULL;
    uint8_t * ptr = (uint8_t *)MEM_DRAM_BASE_NOSDK + dramsize;
    dramsize += size;
    return ptr;
  }

  if (!NOSDK)
    return (void *)os_malloc_iram(size);
  if (iramsize + size > 16384)
    return NULL;
  uint8_t * ptr = (uint8_t *)MEM_IRAM_BASE + iramsize;
  iramsize += size;
  return ptr;
}

uint32_t ROCODE get_free_mem() {
  return system_get_free_heap_size();
}
#endif

uint32_t ROCODE get_ms_ticks() {
#ifndef BOOT_IMAGE
  return system_get_time() / 1000;
#else
  return 0;
#endif
}

void ROCODE set_cpu_speed(uint32_t mhz) {
#ifndef BOOT_IMAGE
  system_update_cpu_freq(mhz);
#endif
}

uint32_t ROCODE get_keys() {
#ifdef  BOOT_IMAGE
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_GPIO1);
#endif
  uint32_t i, b = 0;
  LCD_CS_CLEAR();
  delay_us(1);
  LCD_CS_SET();
  delay_us(1);
#ifdef  HAS_START_BUTTON
  for (i = 0; i < 9; i++) {
#else
  for (i = 0; i < 8; i++) {
#endif
    b <<= 1;
    b += (gpio_input_get() >> 5) & 1;
    GPIO_OUTPUT_SET(PIN_SD_CS, 0);
    delay_us(1);
    GPIO_OUTPUT_SET(PIN_SD_CS, 1);
    delay_us(1);
  }
#ifdef  BOOT_IMAGE
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
#endif
#ifdef  HAS_START_BUTTON
  return b^0x1FF;
#else
  return b^0xFF;
#endif
}

uint32_t ROCODE play_audio(uint32_t rate, SOUNDSYS *ss) {
  i2sInit(rate, 0, ss);
  return 256;
}

void ROCODE stop_audio() {
  i2sStop();
}

uint32_t * ROCODE get_audio_buffer() {
  return i2sGetNextBuffer();
}

#ifndef  BOOT_IMAGE
int ROCODE flash_read_mem(uint32_t addr, void *data, uint32_t size) {
  return spi_flash_read(addr, data, size);
}

int ROCODE flash_write_mem(uint32_t addr, void *data, uint32_t size) {
  return spi_flash_write(addr, data, size);
}

int ROCODE flash_erase(uint32_t addr, uint32_t size) {
  uint16_t sec = addr / 4096;
  for (int i = 0; i < size / 4096; i++, sec++) {
    int r = spi_flash_erase_sector(sec);
    system_soft_wdt_feed();
    if (r != 0)
      return r;
  }
  return 0;
}
#endif

void ROCODE clear_screen(uint16_t c) {
  lcd_mode();
  lcdWriteCommand(ST7735_RAMWR);
  int i;
  uint32_t pix[16];
  GPIO_OUTPUT_SET(PIN_LCD_RS, 1);
  LCD_CS_CLEAR();
  uint32_t cc = (c << 16) | c;
  for (i = 0; i < 16; i++)
    pix[i] = cc;
  for (i = 0; i < 160 * 128 / 32; i++) {
    sendwords(pix, 16);
  }
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  LCD_CS_SET();
}

void ROCODE set_screen(uint32_t y1, uint32_t y2) {
  lcd_mode();
  lcdWriteCommand(ST7735_RASET);
  lcdWriteData(0x00);
  lcdWriteData(y1);
  lcdWriteData(0x00);
  lcdWriteData(y2);
  lcdWriteCommand(ST7735_RAMWR);
  GPIO_OUTPUT_SET(PIN_LCD_RS, 1);
  LCD_CS_CLEAR();
}

void ROCODE update_screen16(uint16_t *fb, uint32_t y1, uint32_t y2, int clear) {
  set_screen(y1, y2);
  fb += y1 * 160;
  uint32_t *dst = (uint32_t *)fb;
  for (int i = 0; i < (y2 - y1 + 1) * 160 / 32; i++) {
    sendwords(dst, 16);

    if (NOSDK && (i & 63) == 0)
      i2sPollSoundStatus();

    if (clear) {
      for (int j = 0; j < 16; j++)
        dst[j] = 0;
    }
    dst += 16;
  }
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  LCD_CS_SET();
}

void ROCODE update_screen8(uint8_t *fb, uint16_t *pb, uint32_t y1, uint32_t y2, int clear) {
  set_screen(y1, y2);
  fb += y1 * 160;
  uint32_t pix[16];
  for (int i = 0; i < (y2 - y1 + 1) * 160 / 32; i++) {
    uint32_t *dst = (uint32_t *)fb;
    uint16_t *pc = (uint16_t *)pix;
    for (int j = 0; j < 32; j++) {
      *pc++ = pb[*fb++];
    }
    sendwords(pix, 16);

    if (NOSDK && (i & 63) == 0)
      i2sPollSoundStatus();

    if (clear) {
      for (int j = 0; j < 8; j++)
        dst[j] = 0;
    }
  }
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  LCD_CS_SET();
}

void ROCODE update_screen4(uint8_t *fb, uint16_t *pb, uint32_t y1, uint32_t y2, int clear) {
  set_screen(y1, y2);
  fb += y1 * 80;
  uint32_t pix[16];
  for (int i = 0; i < (y2 - y1 + 1) * 160 / 32; i++) {
    uint32_t *dst = (uint32_t *)fb;
    uint16_t *pc = (uint16_t *)pix;
    for (int j = 0; j < 16; j++) {
      uint32_t p1 = *fb++;
      uint32_t p2 = p1 & 15;
      *pc++ = pb[p1 >> 4];
      *pc++ = pb[p2];
    }
    sendwords(pix, 16);

    if (NOSDK && (i & 63) == 0)
      i2sPollSoundStatus();

    if (clear) {
        dst[0] = dst[1] = dst[2] = dst[3] = 0;
    }
  }
	while(READ_PERI_REG(SPI_CMD(1))&SPI_USR);		//waiting for spi module available
  LCD_CS_SET();
}

uint32_t ROCODE get_battery_level() {
#ifndef BOOT_IMAGE
  return system_adc_read();
#else
  return 0;
#endif
}

static char IRDATA logbuf[20];
static uint32_t IRDATA logcol;

void ROCODE log_line() {
  int j, k, l;

  GPIO_OUTPUT_SET(PIN_LCD_RS, 1);
  LCD_CS_CLEAR();
  for (j = 0; j < 8; j++) {
    for (k = 0; k < 20; k++) {
      uint16_t c = 0xffff;
      int d = pgm_read_byte(&font8x8_basic[(unsigned char)logbuf[k] - 32][j]);

      for (l = 0; l < 8; l++) {
        if (d & 1) {
          lcdWriteData16(c);
        } else {
          lcdWriteData16(0);
        }
        d >>= 1;
      }
    }
  }
  LCD_CS_SET();
  logcol = 0;
  memset(logbuf, 32, 20);
}

void ROCODE _putchar(char c) {
  if (c == 10) {
    log_line();
    return;
  }
  if (c < 32)
    c = 32;
  logbuf[logcol++] = c & 0x7f;
  if (logcol == 20)
    log_line();
}

static FATFS IRDATA fs;
static FIL IRDATA file_object;

int ROCODE disk_init() {
  TCHAR root_directory[4];
  dstatus_t r;
  FRESULT fr;

  sd_mode();
  init_sd_speed();
  do {
    r = disk_initialize(0);
    if (r == RES_OK)
      break;
  } while (lower_sd_speed() != 0);
  if (r != RES_OK)
    return r;

  root_directory[0] = '0';
  root_directory[1] = ':';
  root_directory[2] = 0;

  file_object.obj.fs = NULL;

  do {
    memset(&fs, 0, sizeof(FATFS));
    fr = f_mount(&fs, root_directory, 1);
    if (fr == FR_OK)
      break;
  } while (lower_sd_speed() != 0);
  return fr;
}

uint32_t ROCODE get_disk_speed() {
  return cur_sd_speed;
}

int ROCODE file_open(char *fname, uint32_t mode) {
  sd_mode();
  int r = f_open(&file_object, fname, mode);
  if (r != FR_OK)
    file_object.obj.fs = NULL;
  return r;
}

int ROCODE file_read(void *buffer, uint32_t size, uint32_t *read) {
  sd_mode();
  return f_read(&file_object, buffer, size, read);
}

int ROCODE file_write(void *buffer, uint32_t size) {
  uint32_t wr;
  sd_mode();
  return f_write(&file_object, buffer, size, &wr);
}

int ROCODE file_seek(uint64_t offset) {
  sd_mode();
  return f_lseek(&file_object, offset);
}

int ROCODE file_close() {
  sd_mode();
  return f_close(&file_object);
}

uint64_t ROCODE file_size() {
  if (file_object.obj.sclust == 0)
    return 0;
  return file_object.obj.objsize;
}

int ROCODE delete_file(char *fname) {
  sd_mode();
  return f_unlink(fname);
}

int ROCODE make_dir(char *fname) {
  sd_mode();
  return f_mkdir(fname);
}

int ROCODE read_dir(char *path, FILEINFO *filebuf, uint32_t *bufsize) {
	FRESULT res;
	FILINFO fno;
	DIR dir;
  uint32_t i = 0;
  FILEINFO fi;

  if ((uint32_t)filebuf > (uint32_t)FLASH_ADDR(0))
    flash_erase((uint32_t)filebuf - (uint32_t)FLASH_ADDR(0), ((*bufsize * sizeof(FILEINFO)) + 4095) & 0xFF000);

  sd_mode();
	res = f_opendir(&dir, path);
	if (res == FR_OK) {
		for (; i < *bufsize;) {
			res = f_readdir(&dir, &fno);
			if (res != FR_OK || fno.fname[0] == 0) {
				break;
			}
			if (fno.fname[0] == '.') {
				continue;
			}
      fi.flags = 0;
			if (fno.fattrib & AM_DIR) {
        fi.flags = FILE_FLAG_DIR;
			}
      os_strncpy(fi.filename, fno.fname, FILEINFO_FILENAME_SIZE);
      fi.filename[FILEINFO_FILENAME_SIZE - 1] = 0;
      if ((uint32_t)filebuf > (uint32_t)FLASH_ADDR(0)) {
        flash_write_mem((uint32_t)&filebuf[i] - (uint32_t)FLASH_ADDR(0), &fi, sizeof(FILEINFO));
      } else {
        filebuf[i] = fi;
      }
      i++;
		}
    f_closedir(&dir);
	}
  *bufsize = i;
  return res;
}

static const char disk_error_unknown[] RODATA = "Unknown Error";
#define DISK_ERROR(e, m)  { static const char disk_error_##e[] RODATA = m; if (error == e) return disk_error_##e; }

const char * ROCODE get_disk_error(int error) {
  DISK_ERROR(0, "OK")
  DISK_ERROR(1, "Disk I/O Error")
  DISK_ERROR(2, "Internal Error")
  DISK_ERROR(3, "Disk Not Ready")
  DISK_ERROR(4, "File Not Found")
  DISK_ERROR(5, "Path Not Found")
  DISK_ERROR(6, "Invalid Path Name")
  DISK_ERROR(7, "Access Denied / Directory Full")
  DISK_ERROR(8, "Access Denied")
  DISK_ERROR(9, "Invalid Object")
  DISK_ERROR(10, "Write Protected")
  DISK_ERROR(11, "Invalid Drive")
  DISK_ERROR(12, "Volume Not Enabled")
  DISK_ERROR(13, "No Filesystem")
  DISK_ERROR(14, "MKFS Aborted")
  DISK_ERROR(15, "Disk Timeout")
  DISK_ERROR(16, "File Locked")
  DISK_ERROR(17, "LFN Out of Memory")
  DISK_ERROR(18, "Too Many Open Files")
  DISK_ERROR(19, "Invalid Parameter")
  return disk_error_unknown;
}

#ifndef BOOT_IMAGE
extern void ROCODE system_init();
#else

void ROCODE boot_gpio_init() {
  gpio_init();
  gpio16_output_conf();
  PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);
  PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO4_U);
  PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO5_U);
  gpio_output_set(1 << PIN_SD_CS, 0, 1 << PIN_SD_CS, 0);
  gpio_output_set(0, 0, 0, 1 << PIN_BUTTONS_DATA);
  GPIO_OUTPUT_SET(PIN_RAM_CS, 1);
  GPIO_OUTPUT_SET(PIN_SD_CS, 1);
  LCD_CS_SET();
  os_delay_us(1000);
}

#endif

void ROCODE hal_init()
{
  // init gpio subsystem
  gpio_init();
#ifndef BOOT_IMAGE
  uart_div_modify(0, UART_CLK_FREQ / 921600);
#endif

  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);
  PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
  gpio_output_set(1, 0, 1, 0);
  gpio16_output_set(1);
  gpio16_output_conf();
#ifndef BOOT_IMAGE
  os_printf("\n\nLightIvy Debug Output\n");
#endif
  logcol = 0;
  memset(logbuf, 32, 20);

  GPIO_REG_WRITE(GPIO_PIN0_ADDRESS + 4 * PIN_LCD_CLK, GPIO_REG_READ(GPIO_PIN0_ADDRESS + 4 * PIN_LCD_CLK) & (15 << 7));
  GPIO_REG_WRITE(GPIO_PIN0_ADDRESS + 4 * PIN_LCD_DATA, GPIO_REG_READ(GPIO_PIN0_ADDRESS + 4 * PIN_LCD_DATA) & (15 << 7));
  gpio_output_set(0, 0, 0, 1 << PIN_LCD_CLK);
  gpio_output_set(0, 0, 0, 1 << PIN_LCD_DATA);
  gpio_output_set(0, 0, 0, 1 << 12);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, 2);//configure io to spi mode
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 2);//configure io to spi mode
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, 2);//configure io to spi mode
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);
  PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO4_U);
  PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO5_U);
  PIN_PULLUP_DIS(PERIPHS_IO_MUX_MTDI_U);
  gpio_output_set(1 << PIN_RAM_CS, 0, 1 << PIN_RAM_CS, 0);
  gpio_output_set(1 << PIN_SD_CS, 0, 1 << PIN_SD_CS, 0);
  gpio_output_set(0, 0, 0, 1 << PIN_BUTTONS_DATA);
  GPIO_OUTPUT_SET(PIN_RAM_CS, 1);
  GPIO_OUTPUT_SET(PIN_SD_CS, 1);
  LCD_CS_SET();
  os_delay_us(1000);

  CLEAR_PERI_REG_MASK(SPI_PIN(1), SPI_IDLE_EDGE);

  CLEAR_PERI_REG_MASK(SPI_CTRL(1), SPI_WR_BIT_ORDER);
  CLEAR_PERI_REG_MASK(SPI_CTRL(1), SPI_RD_BIT_ORDER);

  WRITE_PERI_REG(SPI_USER(1), SPI_USR_MOSI | SPI_CK_I_EDGE);
  WRITE_PERI_REG(SPI_CTRL(1), 0);
  WRITE_PERI_REG(SPI_CTRL1(1), 0);

  // SPI mode type
  CLEAR_PERI_REG_MASK(SPI_SLAVE(1), SPI_SLAVE_MODE);
  // SPI Speed
  CLEAR_PERI_REG_MASK(PERIPHS_IO_MUX, SPI1_CLK_EQU_SYS_CLK);
  WRITE_PERI_REG(SPI_CLOCK(1), (7 << SPI_CLKCNT_N_S) | (7 << SPI_CLKCNT_H_S) | (3 << SPI_CLKCNT_L_S));  // 10MHz

	WRITE_PERI_REG(SPI_USER1(1), (7<<SPI_USR_MOSI_BITLEN_S)|(7<<SPI_USR_MISO_BITLEN_S));
  lcd_mode();
  sleep_ms(1);

  lcd_init();
  clear_screen(0xabcd);
  sd_mode();
  sleep_ms(1);
  sd_mmc_init();

  system_set_os_print(0);

  ram_mode();
  int id = ram_read_id();
  ram_write_word(1120, 98765);
  ram_write_word(124, 123);
  uint32_t data = ram_read_word(1120);
  lcd_mode();
  if (data != 98765) {
    DEBUG("RAM failure: %d\n", data);
  }
  DEBUG("RAM ID: %x\n", id);
}

#ifndef BOOT_IMAGE

#define __STRINGIFY(a) #a
#define xt_rsil(level) (__extension__({uint32_t state; __asm__ __volatile__("rsil %0," __STRINGIFY(level) : "=a" (state)); state;}))
#define xt_wsr_ps(state)  __asm__ __volatile__("wsr %0,ps; isync" :: "a" (state) : "memory")

#define interrupts() xt_rsil(0)
#define noInterrupts() xt_rsil(15)

void dummy() {
}

extern void ets_wdt_disable();
extern void Cache_Read_Disable();
extern void Cache_Read_Enable(uint32_t a, uint32_t b, uint32_t c);

void ROCODE nosdk() {
  system_soft_wdt_stop();
  ets_wdt_disable();
  noInterrupts();
  ETS_FRC1_INTR_DISABLE();
  ETS_FRC_TIMER1_NMI_INTR_ATTACH(dummy);
  *(volatile uint8_t *)(0x40100020) = 0x10;
  *(volatile uint8_t *)(0x40100021) = 0x33;
  *(volatile uint8_t *)(0x40100022) = 0x00;

  __asm__ __volatile__(
      "movi       a2, 0\n"
      "wsr        a2, intenable\n"
      "rsync          \n"
      : : :"a2", "memory");

  dramsize = 0;
  iramsize = 0;
  SOUNDSYSPTR = NULL;
  NOSDK = 1;
}

void flash_switch(uint32_t n, uint32_t c) {
  //Cache_Read_Disable();
  Cache_Read_Enable(n & 1, n >> 1, c);
  if (c != 0)
    *(volatile uint32_t *)0x3FF00024 = ((*(volatile uint32_t *)0x3FF00024) & 0xFFFFFFE7) | 0x18;
  else
    *(volatile uint32_t *)0x3FF00024 = ((*(volatile uint32_t *)0x3FF00024) & 0xFFFFFFE7) | 0x08;
}

//void ROCODE user_init() {
void user_init() {
  hal_init();

  system_update_cpu_freq(SYS_CPU_160MHZ);

  SOUNDSYSPTR = NULL;
  NOSDK = 0;
  dramsize = 0;
  iramsize = 0;

  DEBUG("Heap: %d\n", system_get_free_heap_size());
  printf("Heap: %d\n", system_get_free_heap_size());
  struct rst_info *reset_info = system_get_rst_info();

	if (reset_info->reason == REASON_WDT_RST ||
		reset_info->reason == REASON_EXCEPTION_RST ||
		reset_info->reason == REASON_SOFT_WDT_RST) {
		if (reset_info->reason == REASON_EXCEPTION_RST) {
			printf("Fatal exception (%d):\n\r", reset_info->exccause);
		}
		printf("epc1=0x%08x, epc2=0x%08x, epc3=0x%08x, excvaddr=0x%08x, depc=0x%08x\n\r",
				reset_info->epc1, reset_info->epc2, reset_info->epc3, reset_info->excvaddr, reset_info->depc);
    while(1)
      sleep_ms(10);
	}

  system_init_done_cb(system_init);
}
#endif
