/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------


#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "global_configs.h"
#include "Timer.h"
#include "BlinkLed.h"

#include "dv_led.h"
#include "dv_stm32f429_lcd.h"
#include "dv_stm32f429_i2c.h"
#include "dv_stm32f429_uart.h"
#include "dv_stm32f429_spi.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F4 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 3 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

void main_setup(void)
{
  // Send a greeting to the trace device (skipped on Release).
  //trace_puts("Hello ARM World!");

  // At this stage the system clock should have already been configured
  // at high speed.
  //trace_printf("System clock: %u Hz\n", SystemCoreClock);

  timer_start();
  dv_led_init();

#if USE_BLINKLED_PROCESS
	blink_led_init();
#endif

#if USE_LCD_PROCESS
	dv_stm32f429_lcd_setup();
#endif

#if USE_I2C_PROCESS
	dv_stm32f429_i2c_setup();
#endif

#if USE_UART_PROCESS
    dv_stm32f429_uart_setup();
#endif

#if USE_SPI_PROCESS
    dv_stm32f429_SPI_setup();
#endif
}

void main_loop(void)
{
#if USE_BLINKLED_PROCESS
	uint32_t seconds = 0;
	while (1)
	{
		blink_led_on();
		timer_sleep(seconds == 0 ? TIMER_FREQUENCY_HZ : BLINK_ON_TICKS);

		blink_led_off();
		timer_sleep(BLINK_OFF_TICKS);

		++seconds;
		// Count seconds on the trace device.
		//trace_printf("Second %u\n", seconds);
	}
#endif

#if USE_LCD_PROCESS
	dv_stm32f429_lcd_process();
#endif

#if USE_I2C_PROCESS
	dv_stm32f429_i2c_process();
#endif

#if USE_UART_PROCESS
    dv_stm32f429_uart_process();
#endif

#if USE_SPI_PROCESS
    dv_stm32f429_spi_process();
#endif
}

int
main(int argc, char* argv[])
{
	main_setup();

	while (1)
	{
		main_loop();
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
