/*
 * gpio.c
 *
 *  Created on: 2017��2��14��
 *      Author: ldc
 */

#include "platform-samr21.h"

void setPinFunc(uint32_t portpin, enum FUNCS func)
{
	uint8_t port = GPIOPORT(portpin);
	uint8_t pin = GPIOPIN(portpin);
    PORT->Group[port].PINCFG[pin].bit.PMUXEN = PORT_PINCFG_PMUXEN;
    if ((pin & 0x1) == 0x1)
    {
    	PORT->Group[port].PMUX[pin >> 1].bit.PMUXO = func;
    } else {
        PORT->Group[port].PMUX[pin >> 1].bit.PMUXE = func;
    }
}

int8_t IrqPinMap[2][32] =
{
           /* 0 */ /* 1 */ /* 2 */ /* 3 */ /* 4 */ /* 5 */ /* 6 */ /* 7 */ /* 8 */ /* 9 */ /* 10 *//* 11 *//* 12 *//* 13 *//* 14 *//* 15 */
    {/* A */ -1,      1,     -1,     -1,      4,      5,      6,      7,     -1,      9,      10,     11,     12,     13,     14,     15,
           /* 16 *//* 17 *//* 18 *//* 19 *//* 20 *//* 21 *//* 22 *//* 23 *//* 24 *//* 25 *//* 26 *//* 27 *//* 28 *//* 29 *//* 30 *//* 31 */
     /* A */ -1,      1,      2,      3,      -1,     -1,     6,      7,      12,     13,     -1,     15,     8,      -1,     10,     11,
    },
           /* 0 */ /* 1 */ /* 2 */ /* 3 */ /* 4 */ /* 5 */ /* 6 */ /* 7 */ /* 8 */ /* 9 */ /* 10 *//* 11 *//* 12 *//* 13 *//* 14 *//* 15 */
    {/* B */  0,     -1,      2,      3,     -1,     -1,     -1,     -1,     -1,   	 -1,      -1,     -1,     -1,     -1,     -1,     -1,
           /* 16 *//* 17 *//* 18 *//* 19 *//* 20 *//* 21 *//* 22 *//* 23 *//* 24 *//* 25 *//* 26 *//* 27 *//* 28 *//* 29 *//* 30 *//* 31 */
     /* B */  0,      1,      -1,     -1,     -1,     -1,     6,      7,     -1,     -1,      -1,     -1,     -1,     -1,     -1,     -1,
    },
};

int8_t findExternalInterruptMap(uint32_t portpin)
{
	int8_t port = GPIOPORT(portpin);
	int8_t pin = GPIOPIN(portpin);

	if (port >= GPIO_PORTC)
	{
		return -1;
	}

	return IrqPinMap[port][pin];
}

ThreadError externIrqSetup(uint32_t portpin, int flags)
{
	enum { NONE = 0, RISE, FALL, BOTH, HIGH, LOW };
	uint32_t old_ext_irq = 0, old_wakeup = 0, old_config = 0;
	int8_t extint = -1, index = 0, offset = 0;
	uint8_t int_mode = 0;

	extint = findExternalInterruptMap(portpin);
	if (extint == -1)
	{
		return kThreadError_Failed;
	}

	index = extint >> 0x3;
	offset = (extint & 0x7) << 2;

    if (flags & GPIO_INT_DEBOUNCE)
    {
    	int_mode = 1U << 3;
    }
	if (flags & GPIO_INT_DOUBLE_EDGE) {
		int_mode |= BOTH;
	} else {
		if (flags & GPIO_INT_EDGE) {
			if (flags & GPIO_INT_ACTIVE_HIGH) {
				int_mode |= RISE;
			} else {
				int_mode |= FALL;
			}
		} else {
			if (flags & GPIO_INT_ACTIVE_HIGH) {
				int_mode |= HIGH;
			} else {
				int_mode |= LOW;
			}
		}
	}

    while (EIC->STATUS.bit.SYNCBUSY) {}
    if (EIC->CTRL.bit.ENABLE == 1)
    {
    	old_ext_irq = EIC->INTENSET.vec.EXTINT;
    	old_wakeup = EIC->WAKEUP.vec.WAKEUPEN;
    	old_config = EIC->CONFIG[index].reg;
    }
    EIC->CTRL.bit.SWRST = 1;
    while (EIC->STATUS.bit.SYNCBUSY) {}

    setPinFunc(portpin, FUNCA);

    EIC->WAKEUP.reg = old_wakeup | (1U << extint);
	EIC->CONFIG[index].reg = old_config | (int_mode << offset);

	EIC->INTENSET.reg = old_ext_irq | (1U << extint);

	while (EIC->STATUS.bit.SYNCBUSY) {}
	EIC->CTRL.bit.ENABLE = 1;

	return kThreadError_None;
}

ThreadError samr21GpioConfig(uint32_t portpin, int flags)
{
	uint8_t port = GPIOPORT(portpin);
	uint8_t pin = GPIOPIN(portpin);

	uint32_t mask = 1U << pin;

	/* Disable the pin and return as setup is meaningless now */
	if (flags & GPIO_PIN_DISABLE) {
		PORT->Group[port].DIRCLR.reg = mask;
		PORT->Group[port].WRCONFIG.reg = PORT_WRCONFIG_WRPINCFG | (mask & 0x0000ffff);
		PORT->Group[port].WRCONFIG.reg = PORT_WRCONFIG_HWSEL | PORT_WRCONFIG_WRPINCFG | ((mask & 0xffff0000) >> 16);

		return kThreadError_None;
	}

	/* Setup the pin direction */
	if ((flags & GPIO_DIR_MASK) == GPIO_DIR_OUT) {
		PORT->Group[port].DIRSET.reg = mask;
		PORT->Group[port].WRCONFIG.reg = PORT_WRCONFIG_WRPINCFG | (mask & 0x0000ffff);
		PORT->Group[port].WRCONFIG.reg = PORT_WRCONFIG_HWSEL | PORT_WRCONFIG_WRPINCFG | ((mask & 0xffff0000) >> 16);
	} else {
		PORT->Group[port].DIRCLR.reg = mask;
		PORT->Group[port].WRCONFIG.reg = PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_INEN | (mask & 0x0000ffff);
		PORT->Group[port].WRCONFIG.reg = PORT_WRCONFIG_HWSEL | PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_INEN | ((mask & 0xffff0000) >> 16);
	}

	/* Pull-up? */
	if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_UP)
	{
		/* Enable pull-up */
		PORT->Group[port].DIRCLR.reg = mask;
		PORT->Group[port].PINCFG[pin].bit.PULLEN = 1;
		PORT->Group[port].OUTSET.reg = mask;
	}
	else if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_DOWN)
	{
		/* Disable pull-up */
		PORT->Group[port].DIRCLR.reg = mask;
		PORT->Group[port].PINCFG[pin].bit.PULLEN = 1;
		PORT->Group[port].OUTCLR.reg = mask;
	}
	else if ((flags & GPIO_PUD_MASK) == GPIO_PUD_NORMAL)
	{
		PORT->Group[port].PINCFG[pin].bit.PULLEN = 0;
	}

	/* Setup interrupt config */
	if (flags & GPIO_INT)
	{
		if (externIrqSetup(pin, flags) != kThreadError_None)
		{
			return kThreadError_Failed;
		}
	}

	return kThreadError_None;
}

void samr21GpioWrite(uint32_t portpin, uint32_t value)
{
	uint8_t port = GPIOPORT(portpin);
	uint8_t pin = GPIOPIN(portpin);
	uint32_t mask = 1U << pin;

	if (value)
	{
		PORT->Group[port].OUTSET.reg = mask;
	}
	else
	{
		PORT->Group[port].OUTCLR.reg = mask;
	}
}

uint32_t samr21GpioRead(uint32_t portpin, uint32_t *value)
{
	uint8_t port = GPIOPORT(portpin);
	uint8_t pin = GPIOPIN(portpin);

	*value = (PORT->Group[port].IN.reg >> pin) & 0x01;

	return *value;
}

void samr21GpioIsr(void *arg)
{

	uint32_t int_stat;

	int_stat = EIC->INTFLAG.reg;
	EIC->INTFLAG.reg = int_stat;

}

void samr21GpioEnableCallback(uint32_t portpin)
{
	int8_t extint;

	extint = findExternalInterruptMap(portpin);
	EIC->INTENSET.reg = 1U << extint;

}

void samr21GpioDisableCallback(uint32_t portpin)
{
	int8_t extint;

	extint = findExternalInterruptMap(portpin);
	EIC->INTENCLR.reg = 1U << extint;

}

void samr21GpioInit(void)
{
    /* enable sync and async clocks */
    PM->APBBMASK.reg |= PM_APBBMASK_PORT;
    PM->APBAMASK.reg |= PM_APBAMASK_EIC;
    GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_EIC);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

    NVIC_EnableIRQ(EIC_IRQn);
}
