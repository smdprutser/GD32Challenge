

// simple SPI example used by the comparison between STM32F103 and GD32F103
// more info at https://SMDprutser.nl
//
// delay is made by wasting CPU cycles, this will benefit the GD32 part as the clock speed is higher
//
// Written by (C)hris van Dongen 20180425
//
// TODO add a license (TBD)


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/spi.h>
#include "logo.h"
#include "font.h"
#include "sine.h"

// provides the clock setup at 108Mhz for GD32
#include "../gd32.c"

void ST7735_writedat(uint8_t d);
void ST7735_writecmd(uint8_t c);
void ST7735_sendinitseq(const uint8_t *addr );
void delayms(int i);
void putc(int x, int y, char c);
void putpixel(int x, int y, uint16_t color);
void clearline(int x);


// got most from adafruit 
// please support them

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
#define ST7735_RASET   0x2B
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


#define DELAY 0x80
static const uint8_t 
  Bcmd[] = {                  // Initialization commands for 7735B screens
    18,                       // 18 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
      50,                     //     50 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
      255,                    //     255 = 500 ms delay
    ST7735_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
      0x05,                   //     16-bit color
      10,                     //     10 ms delay
    ST7735_FRMCTR1, 3+DELAY,  //  4: Frame rate control, 3 args + delay:
      0x00,                   //     fastest refresh
      0x06,                   //     6 lines front porch
      0x03,                   //     3 lines back porch
      10,                     //     10 ms delay
    ST7735_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
      0x08,                   //     Row addr/col addr, bottom to top refresh
    ST7735_DISSET5, 2      ,  //  6: Display settings #5, 2 args, no delay:
      0x15,                   //     1 clk cycle nonoverlap, 2 cycle gate
                              //     rise, 3 cycle osc equalize
      0x02,                   //     Fix on VTL
    ST7735_INVCTR , 1      ,  //  7: Display inversion control, 1 arg:
      0x0,                    //     Line inversion
    ST7735_PWCTR1 , 2+DELAY,  //  8: Power control, 2 args + delay:
      0x02,                   //     GVDD = 4.7V
      0x70,                   //     1.0uA
      10,                     //     10 ms delay
    ST7735_PWCTR2 , 1      ,  //  9: Power control, 1 arg, no delay:
      0x05,                   //     VGH = 14.7V, VGL = -7.35V
    ST7735_PWCTR3 , 2      ,  // 10: Power control, 2 args, no delay:
      0x01,                   //     Opamp current small
      0x02,                   //     Boost frequency
    ST7735_VMCTR1 , 2+DELAY,  // 11: Power control, 2 args + delay:
      0x3C,                   //     VCOMH = 4V
      0x38,                   //     VCOML = -1.1V
      10,                     //     10 ms delay
    ST7735_PWCTR6 , 2      ,  // 12: Power control, 2 args, no delay:
      0x11, 0x15,
    ST7735_GMCTRP1,16      ,  // 13: Magical unicorn dust, 16 args, no delay:
      0x09, 0x16, 0x09, 0x20, //     (seriously though, not sure what
      0x21, 0x1B, 0x13, 0x19, //      these config values represent)
      0x17, 0x15, 0x1E, 0x2B,
      0x04, 0x05, 0x02, 0x0E,
    ST7735_GMCTRN1,16+DELAY,  // 14: Sparkles and rainbows, 16 args + delay:
      0x0B, 0x14, 0x08, 0x1E, //     (ditto)
      0x22, 0x1D, 0x18, 0x1E,
      0x1B, 0x1A, 0x24, 0x2B,
      0x06, 0x06, 0x02, 0x0F,
      10,                     //     10 ms delay
    ST7735_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 2
      0x00, 0x81,             //     XEND = 129
    ST7735_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 1
      0x00, 0x81,             //     XEND = 160
    ST7735_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,   DELAY,  // 18: Main screen turn on, no args, w/delay
      255 },                  //     255 = 500 ms delay

  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,  
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0x48,                   //     row addr/col addr, bottom to top refresh // was c8
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

  Rcmd2green[] = {            // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,        //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,             //     XSTART = 0
      0x00, 0x9F+0x01 },      //     XEND = 159

  Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F },           //     XEND = 159

  Rcmd2green144[] = {              // Init for 7735R, part 2 (green 1.44 tab)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F },           //     XEND = 127

  Rcmd2green160x80[] = {              // Init for 7735R, part 2 (mini 160x80)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 79
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F },           //     XEND = 159


  Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay




const char scrolltext[]="                          This is part 4 in our series GD32 vs STM32..... This time we address the SPI peripheral.....                           ";

int main(void)
{
	int i;
	int pos;
	uint8_t sinepos;

#ifdef USE_STM
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
#elif USE_GD
	rcc_clock_setup_in_hse_8mhz_out_108mhz();
#else 
	#error "Chip is not defined!!"
#endif

	// enable GPIO
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_AFIO);

	// setup the pins
	// UART1 GPA9-TX GPA10-RX
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO10);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);

	// setup peripheral to 115200,n,8,1 no flowcontrol
	rcc_periph_clock_enable(RCC_USART1);
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_enable(USART1);

	//setup SPI pins
	// A7 MOSI, A6 MISO, A5 CLK, A4 /CS, A3 D/!C
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);		// reset
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO3);		// D/!C
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);		// CS
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5);	// CLK
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO6);			// MISO
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7);	// MOSI

	// setup for mode 0 SPI 36MHz/54Mhz (GPIO can do max 50MHz)
	rcc_periph_clock_enable(RCC_SPI1);
	spi_reset(SPI1);
//	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_2, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);	// mode 0
//	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_2, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);	// mode 1
//	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_2, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);	// mode 2
	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_2, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);	// mode 3
	spi_set_full_duplex_mode(SPI1);
	gpio_set(GPIOA, GPIO3);	// cs=1
	spi_enable(SPI1);

	gpio_clear(GPIOA, GPIO2);
	delayms(100);
	gpio_set(GPIOA, GPIO2);

	// init display
	ST7735_sendinitseq(Rcmd1);
	ST7735_sendinitseq(Rcmd2red);
	ST7735_sendinitseq(Rcmd3);

	//setup window and enable framebuffer
	ST7735_writecmd(ST7735_CASET);
	ST7735_writedat(0x00);
	ST7735_writedat(0x00);
	ST7735_writedat(0x00);
	ST7735_writedat(0x7f-45);
	ST7735_writecmd(ST7735_RASET);
	ST7735_writedat(0x00);
	ST7735_writedat(0x00);
	ST7735_writedat(0x00);
	ST7735_writedat(0x9F);
	ST7735_writecmd(ST7735_RAMWR);

	gpio_set(GPIOA, GPIO3);		// set D/!C=1
	gpio_clear(GPIOA, GPIO4);	// set CS=0

	// clear bottom screen
	for(i=0; i<84*160; i++)
	{
		spi_xfer(SPI1, 0x00);
		spi_xfer(SPI1, 0x00);
	}

	//setup window and enable framebuffer
	ST7735_writecmd(ST7735_CASET);
	ST7735_writedat(0x00);
	ST7735_writedat(0x7f-44);
	ST7735_writedat(0x00);
	ST7735_writedat(0x7f);
	ST7735_writecmd(ST7735_RASET);
	ST7735_writedat(0x00);
	ST7735_writedat(0x00);
	ST7735_writedat(0x00);
	ST7735_writedat(0x9F);
	ST7735_writecmd(ST7735_RAMWR);

	gpio_set(GPIOA, GPIO3);		// set D/!C=1
	gpio_clear(GPIOA, GPIO4);	// set CS=0

	//display logo
	for(i=0; i<160*45*2; i++)
	{
		spi_xfer(SPI1, image_data_Logo_prutser160x45_bw[i]);
	}
	

	pos=0;
	sinepos=0;
	while(1)
	{
		pos++;
		sinepos+=12;

		for(i=0; i<26; i++)
		{
			putc(i*6, (sine[(sinepos+5*i)&0xFF]/4)+(sine[((sinepos)+3*i)&0xFF]/4), scrolltext[pos+i]);

			if(pos>120) pos=0;
		}

		delayms(1500);
	}

	return 0;
}

void ST7735_writedat(uint8_t d)
{
	gpio_set(GPIOA, GPIO3);		// set D/!C=1
	gpio_clear(GPIOA, GPIO4);	// set CS=0
	spi_xfer(SPI1, d);		// send byte
	gpio_set(GPIOA, GPIO3);		// set C1=1
}
void ST7735_writecmd(uint8_t c)
{
	gpio_clear(GPIOA, GPIO3);	// set D/!C=1
	gpio_clear(GPIOA, GPIO4);	// set CS=0
	spi_xfer(SPI1, c);		// send byte
	gpio_set(GPIOA, GPIO3);		// set C1=1
}

void delayms(int i)
{
	int j;

	while(i--)
	{
		for (j = 0; j < 800; j++) __asm__("nop");
	}
}


void ST7735_sendinitseq(const uint8_t *addr )
{
	uint8_t  numCommands, numArgs;
	uint16_t ms;

	numCommands = *(addr++);			// Number of commands to follow
	while(numCommands--) 
	{						// For each command...
		ST7735_writecmd(*(addr++));		//   Read, issue command
		numArgs=*(addr++);			//   Number of args to follow
		ms=numArgs&DELAY;			//   If hibit set, delay follows args
		numArgs&=~DELAY;			//   Mask out delay bit
		while(numArgs--) 
		{					//   For each argument...
			ST7735_writedat(*(addr++));	//     Read, issue argument
		}		

		if(ms)
		{
			ms=*(addr++);			// Read post-command delay time (ms)
			if(ms == 255) ms = 500;		// If 255, delay for 500 ms
			delayms(ms);
		}
	}
}

void putc(int x, int y, char c)
{
	int i, j, offset;
	uint8_t mask, temp;

	offset=c*5;

	// display char
	for(i=0; i<5; i++)
	{
		mask=0x01;
		clearline(x+i);
		for(j=0; j<8; j++)
		{
			if(font[offset+i]&mask) putpixel(x+i, y+j, 0xFFFF);
//				else putpixel(x+i, y+j, 0x0000);
			mask<<=1;
		}
	}

	clearline(x+5);
}

void putpixel(int x, int y, uint16_t color)
{
	//setup window and enable framebuffer
	ST7735_writecmd(ST7735_CASET);
	ST7735_writedat(0x00);
	ST7735_writedat(y);
	ST7735_writedat(0x00);
	ST7735_writedat(y);
	ST7735_writecmd(ST7735_RASET);
	ST7735_writedat(0x00);
	ST7735_writedat(x);
	ST7735_writedat(0x00);
	ST7735_writedat(x);
	ST7735_writecmd(ST7735_RAMWR);

	gpio_set(GPIOA, GPIO3);		// set D/!C=1
	gpio_clear(GPIOA, GPIO4);	// set CS=0

	spi_xfer(SPI1, color>>8);
	spi_xfer(SPI1, color&0x0FF);
}

void clearline(int x)
{
	int i;

	//setup window and enable framebuffer
	ST7735_writecmd(ST7735_CASET);
	ST7735_writedat(0x00);
	ST7735_writedat(0x00);
	ST7735_writedat(0x00);
	ST7735_writedat(0x7f-45);
	ST7735_writecmd(ST7735_RASET);
	ST7735_writedat(0x00);
	ST7735_writedat(x);
	ST7735_writedat(0x00);
	ST7735_writedat(x);
	ST7735_writecmd(ST7735_RAMWR);

	gpio_set(GPIOA, GPIO3);		// set D/!C=1
	gpio_clear(GPIOA, GPIO4);	// set CS=0

	for(i=0; i<(0x7f-45)*2; i++)
		spi_xfer(SPI1, 0x00);
}




