/**
 * @file
 *   This file implements the OpenThread platform abstraction for radio communication.
 *
 */
#ifndef RADIO_RF233_H_
#define RADIO_RF233_H_

#include "platform-samr21.h"

#define RF233IRQPIN     GPIOGROUP(GPIO_PORTB, 0)
#define RF233RSTPIN     GPIOGROUP(GPIO_PORTB, 15)
#define RF233SLPPIN     GPIOGROUP(GPIO_PORTA, 20)

#define AT86RF233_PARTNUM (0xB)


/* AUTOACK should be enabled by default, disable it only for testing */
#define AT86RF233_AUTOACK_ENABLED       (true)

#define AT86RF233_FCS_LENGTH        (2)
#define AT86RF233_PSDU_LENGTH       (125)
#define AT86RF233_WAKEUP_DELAY      (1000)

#endif
