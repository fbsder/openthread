/*
 * gpio.c
 *
 *  Created on: 2017Äê2ÔÂ14ÈÕ
 *      Author: ldc
 */

#include "platform-samr21.h"

void set_pin_func(uint32_t portpin, enum FUNCS func)
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

int8_t irq_pin_map[2][32] =
{
		/* 0 */ /* 1 */ /* 2 */ /* 3 */ /* 4 */ /* 5 */ /* 6 */ /* 7 */ /* 8 */ /* 9 */ /* 10 *//* 11 *//* 12 *//* 13 *//* 14 *//* 15 */
		{/* A */ -1,		 1,	    -1,	    -1,	     4,	     5,	     6,	     7,	    -1,	 	 9,	     10,	 11,	 12,	 13,	 14,	 15,
				/* 16 *//* 17 *//* 18 *//* 19 *//* 20 *//* 21 *//* 22 *//* 23 *//* 24 *//* 25 *//* 26 *//* 27 *//* 28 *//* 29 *//* 30 *//* 31 */
				/* A */ -1,		 1,	     2,	     3,	     -1,     -1,     6,	     7,	     12,     13,     -1,     15,	 8,      -1,	 10,     11,
		},
		/* 0 */ /* 1 */ /* 2 */ /* 3 */ /* 4 */ /* 5 */ /* 6 */ /* 7 */ /* 8 */ /* 9 */ /* 10 *//* 11 *//* 12 *//* 13 *//* 14 *//* 15 */
		{/* B */  0,		-1,	     2,	     3,	    -1,	    -1,	    -1,	    -1,	    -1,	 	-1,	     -1,	 -1,	 -1,	 -1,	 -1,	 -1,
				/* 16 *//* 17 *//* 18 *//* 19 *//* 20 *//* 21 *//* 22 *//* 23 *//* 24 *//* 25 *//* 26 *//* 27 *//* 28 *//* 29 *//* 30 *//* 31 */
				/* B */  0,		 1,	     -1,     -1,     -1,     -1,     6,	     7,	     -1,     -1,     -1,     -1,	 -1,     -1,	 -1,     -1,
		},
};

int8_t find_ext_int(struct device *dev, uint32_t portpin)
{
	(void)dev;
	int8_t port = GPIOPORT(portpin);
	int8_t pin = GPIOPIN(portpin);

	if (port >= GPIO_PORTC)
	{
		return -1;
	}

	return irq_pin_map[port][pin];
}

