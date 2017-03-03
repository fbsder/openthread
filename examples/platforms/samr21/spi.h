#ifndef SAMR21_SPI_H_
#define SAMR21_SPI_H_

#include <stdint.h>
#include <openthread/types.h>

int samr21SpiInit(void);
int samr21SpiTransceive(const void *tx_buf, uint32_t tx_buf_len, void *rx_buf, uint32_t rx_buf_len);
int samr21SpiConfigure(uint32_t flags);

#endif
