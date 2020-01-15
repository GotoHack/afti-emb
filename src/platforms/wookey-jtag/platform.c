/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the STM32
 * implementation.
 */

#include "general.h"
#include "cdcacm.h"
#include "usbuart.h"
#include "morse.h"

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>

//#include <libopencm3/stm32/f1/adc.h>
#if 0
static void adc_init(void);
static void setup_vbus_irq(void);
#endif
/* Pins PB[7:5] are used to detect hardware revision.
 * 000 - Original production build.
 * 001 - Mini production build.
 * 010 - Mini V2.0e and later.
 */
int platform_hwversion(void)
{
#if 0
	static int hwversion = -1;
	uint16_t hwversion_pins = GPIO7 | GPIO6 | GPIO5;
	uint16_t unused_pins = hwversion_pins ^ 0xFFFF;

	/* Only check for version if this is the first time. */
	if (hwversion == -1) {
		/* Configure the hardware version pins as input pull-up/down */
		gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
				GPIO_CNF_INPUT_PULL_UPDOWN,
				hwversion_pins);

		/* Enable the weak pull up. */
		gpio_set(GPIOB, hwversion_pins);

		/* Wait a little to make sure the pull up is in effect... */
		for(int i = 0; i < 100; i++) asm("nop");

		/* Get all pins that are pulled low in hardware.
		 * This also sets all the "unused" pins to 1.
		 */
		uint16_t pins_negative = gpio_get(GPIOB, hwversion_pins) | unused_pins;

		/* Enable the weak pull down. */
		gpio_clear(GPIOC, hwversion_pins);

		/* Wait a little to make sure the pull down is in effect... */
		for(int i = 0; i < 100; i++) asm("nop");

		/* Get all the pins that are pulled high in hardware. */
		uint16_t pins_positive = gpio_get(GPIOB, hwversion_pins);

		/* Hardware version is the id defined by the pins that are
		 * asserted low or high by the hardware. This means that pins
		 * that are left floating are 0 and those that are either
		 * pulled high or low are 1.
		 */
		hwversion = (((pins_positive ^ pins_negative) ^ 0xFFFF) & hwversion_pins) >> 5;
	}
#else
    static int hwversion = 0;
#endif
	return hwversion;
}

void platform_init(void)
{
	SCS_DEMCR |= SCS_DEMCR_VC_MON_EN;
#ifdef ENABLE_DEBUG
	void initialise_monitor_handles(void);
	initialise_monitor_handles();
#endif

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_CRC);

#if 0
	/* Setup GPIO ports */
	gpio_clear(USB_PU_PORT, USB_PU_PIN);
	gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, USB_PU_PIN);
#endif

	gpio_set_mode(JTAG_PORT, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL,
			TMS_DIR_PIN | TMS_PIN | TCK_PIN | TDI_PIN);
	gpio_set_mode(JTAG_PORT, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL,
			TMS_DIR_PIN | TCK_PIN | TDI_PIN);
	gpio_set_mode(JTAG_PORT, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_INPUT_FLOAT, TMS_PIN);

#if 0
    /* This needs some fixing... */
	/* Toggle required to sort out line drivers... */
	gpio_port_write(GPIOA, 0x8102);
	gpio_port_write(GPIOB, 0x2000);

	gpio_port_write(GPIOA, 0x8182);
	gpio_port_write(GPIOB, 0x2002);
#endif

	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL,
			LED_UART | LED_IDLE_RUN | LED_ERROR);

#if 0
	/* FIXME: This pin in intended to be input, but the TXS0108 fails
	 * to release the device from reset if this floats. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);
	/* Enable SRST output. Original uses a NPN to pull down, so setting the
	 * output HIGH asserts. Mini is directly connected so use open drain output
	 * and set LOW to assert.
	 */
#endif
	platform_srst_set_val(false);
	gpio_set_mode(SRST_PORT, GPIO_MODE_OUTPUT_50_MHZ,
			(((platform_hwversion() == 0) ||
			  (platform_hwversion() >= 3))
			 ? GPIO_CNF_OUTPUT_PUSHPULL
			 : GPIO_CNF_OUTPUT_OPENDRAIN),
			SRST_PIN);

#if 0
	/* Enable internal pull-up on PWR_BR so that we don't drive
	   TPWR locally or inadvertently supply power to the target. */
	if (platform_hwversion () == 1) {
		gpio_set(PWR_BR_PORT, PWR_BR_PIN);
		gpio_set_mode(PWR_BR_PORT, GPIO_MODE_INPUT,
		              GPIO_CNF_INPUT_PULL_UPDOWN, PWR_BR_PIN);
	} else if (platform_hwversion() > 1) {
		gpio_set(PWR_BR_PORT, PWR_BR_PIN);
		gpio_set_mode(PWR_BR_PORT, GPIO_MODE_OUTPUT_50_MHZ,
		              GPIO_CNF_OUTPUT_OPENDRAIN, PWR_BR_PIN);
	}

	if (platform_hwversion() > 0) {
		adc_init();
	} else {
		gpio_clear(GPIOB, GPIO0);
		gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
				GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
	}
#endif

	/* Relocate interrupt vector table here */
	extern int vector_table;
	SCB_VTOR = (uint32_t)&vector_table;

	platform_timing_init();
	//usbuart_init();
	cdcacm_init();


#if 0
	setup_vbus_irq();
#endif
}

void platform_srst_set_val(bool assert)
{
	gpio_set_val(TMS_PORT, TMS_PIN, 1);
	if ((platform_hwversion() == 0) ||
	    (platform_hwversion() >= 3)) {
		gpio_set_val(SRST_PORT, SRST_PIN, assert);
	} else {
		gpio_set_val(SRST_PORT, SRST_PIN, !assert);
	}
	if (assert) {
		for(int i = 0; i < 10000; i++) asm("nop");
	}
}


bool platform_srst_get_val(void)
{
	if (platform_hwversion() == 0) {
		return gpio_get(SRST_SENSE_PORT, SRST_SENSE_PIN) == 0;
	} else if (platform_hwversion() >= 3) {
		return gpio_get(SRST_SENSE_PORT, SRST_SENSE_PIN) != 0;
	} else {
		return gpio_get(SRST_PORT, SRST_PIN) == 0;
	}
}


bool platform_target_get_power(void)
{
	return 0;
}


void platform_target_set_power(bool power)
{
    if (platform_hwversion() > 0) {
		gpio_set_val(PWR_BR_PORT, PWR_BR_PIN, !power);
	}
}

const char *platform_target_voltage(void)
{
    return "ABSENT!";
}

void platform_request_boot(void)
{
    ;
}

#if 0

void exti15_10_isr(void)
{
	if (gpio_get(USB_VBUS_PORT, USB_VBUS_PIN)) {
		/* Drive pull-up high if VBUS connected */
		gpio_set_mode(USB_PU_PORT, GPIO_MODE_OUTPUT_10_MHZ,
				GPIO_CNF_OUTPUT_PUSHPULL, USB_PU_PIN);
	} else {
		/* Allow pull-up to float if VBUS disconnected */
		gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT,
				GPIO_CNF_INPUT_FLOAT, USB_PU_PIN);
	}

	exti_reset_request(USB_VBUS_PIN);
}


static void setup_vbus_irq(void)
{
	nvic_set_priority(USB_VBUS_IRQ, IRQ_PRI_USB_VBUS);
	nvic_enable_irq(USB_VBUS_IRQ);

	gpio_set(USB_VBUS_PORT, USB_VBUS_PIN);
	gpio_set(USB_PU_PORT, USB_PU_PIN);

	gpio_set_mode(USB_VBUS_PORT, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_PULL_UPDOWN, USB_VBUS_PIN);

	/* Configure EXTI for USB VBUS monitor */
	exti_select_source(USB_VBUS_PIN, USB_VBUS_PORT);
	exti_set_trigger(USB_VBUS_PIN, EXTI_TRIGGER_BOTH);
	exti_enable_request(USB_VBUS_PIN);

	exti15_10_isr();
}
#endif
