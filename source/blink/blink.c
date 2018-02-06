

// simple blink a led example used by the comparison between STM32F103 and GD32F103
// more info at https://SMDprutser.nl
//
// delay is made by wasting CPU cycles, this will benefit the GD32 part as the clock speed is higher
//
// Written by (C)hris van Dongen 20180206
//
// TODO add a license (TBD)


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>

// provides the clock setup at 108Mhz for GD32
#include "../gd32.c"

int main(void)
{
	int i;

#ifdef USE_STM
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
#elif USE_GD
	rcc_clock_setup_in_hse_8mhz_out_108mhz();
#else 
	#error "Chip is not defined!!"
#endif

	// enable GPIO
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);

	// setup the pins
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO8);

	// enable PLL/2 on PA8 (may not exceed 50MHz)
	// seems capable of outputtIing 52ish MHz on a GD32
	rcc_set_mco(RCC_CFGR_MCO_PLL_DIV2);		// PLL/2
//	rcc_set_mco(RCC_CFGR_MCO_HSI);			// internal oscillator
//	rcc_set_mco(RCC_CFGR_MCO_HSE);			// external oscillator

	while (1)
	{
		gpio_toggle(GPIOA, GPIO0);
		for (i = 0; i < 800000; i++)
			__asm__("nop");
	}

	return 0;
}
