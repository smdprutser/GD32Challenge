

// simple blink a led example used by the comparison between STM32F103 and GD32F103
// more info at https://SMDprutser.nl
//
// delay is made by wasting CPU cycles, this will benefit the GD32 part as the clock speed is higher
//
// Written by (C)hris van Dongen 20180315
//
// TODO add a license (TBD)


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>

// provides the clock setup at 108Mhz for GD32
#include "../gd32.c"

int main(void)
{
	uint8_t d;

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

	// UART2 GPA2-TX GPA3-RX
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO3);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO2);

	// setup peripheral to 115200,n,8,1 no flowcontrol
	rcc_periph_clock_enable(RCC_USART2);
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	usart_enable(USART2);

	// UART3 GPB10-TX GPB11-RX
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO11);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO10);

	// setup peripheral to 115200,n,8,1 no flowcontrol
	rcc_periph_clock_enable(RCC_USART3);
	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
	usart_enable(USART3);

	while (1)
	{
		if((USART_SR(USART1) & USART_SR_RXNE))
		{
			d=usart_recv(USART1);
			usart_send_blocking(USART1, d);
		}

		if((USART_SR(USART2) & USART_SR_RXNE))
		{
			d=usart_recv(USART2);
			usart_send_blocking(USART2, d);
		}

		if((USART_SR(USART3) & USART_SR_RXNE))
		{
			d=usart_recv(USART3);
			usart_send_blocking(USART3, d);
		}
	}

	return 0;
}
