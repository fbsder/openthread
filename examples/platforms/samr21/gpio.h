
#ifndef SAMR21_GPIO_H_
#define SAMR21_GPIO_H_

#include <stdint.h>
#include <openthread-types.h>

ThreadError samr21GpioConfig(uint32_t portpin, int flags);
void samr21GpioWrite(uint32_t portpin, uint32_t value);
uint32_t samr21GpioRead(uint32_t portpin, uint32_t *value);
void samr21GpioInit(void);

#endif
