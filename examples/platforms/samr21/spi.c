/*
 * spi.c
 *
 *  Created on: 2017骞�2鏈�14鏃�
 *      Author: ldc
 */

#define SPIACTIVE   0
#define SPIDEACTIVE 1

enum
{
    kPlatformClock = 48000000,
    kBaudRate = 5000000,
};

struct spi_data
{
    const uint8_t *tx_buf;
    uint32_t tx_cnt;
    uint32_t tx_buf_len;
    uint8_t *rx_buf;
    uint32_t rx_cnt;
    uint32_t rx_buf_len;
    uint32_t xfer_len;
};

#define MUXGROUP    1
#define MUXDIPIN    0
uint16_t mux_group = MUXGROUP; /* see DS: SPI CTRLA DOPO*/
uint16_t mux_pad0 = MUXDIPIN; /* see DS: SPI CTRLA DIPO */
uint16_t pad0_pin = 0x553;
uint16_t pad1_pin = 0x53f;
uint16_t pad2_pin = 0x53e;
uint16_t pad3_pin = 0x552;

#define CS_PORT     1
#define CS_PIN      31
uint16_t cs_portpin = GPIOGROUP(CS_PORT, CS_PIN);

struct spi_data local_data;

static void pinmuxSetDefault(void)
{
    SercomSpi *spi = (SercomSpi *) SERCOM4;

    setPinFunc(GPIOPORTPIN(pad0_pin), GPIOFUNC(pad0_pin));
    setPinFunc(GPIOPORTPIN(pad1_pin), GPIOFUNC(pad1_pin));
    setPinFunc(GPIOPORTPIN(pad2_pin), GPIOFUNC(pad2_pin));
    setPinFunc(GPIOPORTPIN(pad3_pin), GPIOFUNC(pad3_pin));

    spi->CTRLB.reg |= SERCOM_SPI_CTRLB_MSSEN;
    while (spi->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_CTRLB)
    {
    }
}

static void pinmuxSetPort(void)
{
    setPinFunc(GPIOPORTPIN(pad0_pin), GPIOFUNC(pad0_pin));

    if ((mux_group == 0) || (mux_group == 2))
    {
        setPinFunc(GPIOPORTPIN(pad1_pin), GPIOFUNC(pad1_pin));
    }
    else if ((mux_group == 1) || (mux_group == 3))
    {
        setPinFunc(GPIOPORTPIN(pad2_pin), GPIOFUNC(pad2_pin));
    }

    // set pin output
    samr21GpioConfig(cs_portpin, GPIO_DIR_OUT);

    setPinFunc(GPIOPORTPIN(pad3_pin), GPIOFUNC(pad3_pin));
}

static void selectCSPort(uint8_t val)
{
    samr21GpioWrite(cs_portpin, val);
}

int samr21SpiConfigure(uint32_t flags)
{
    uint32_t cfg;
    SercomSpi *spi = (SercomSpi *) SERCOM4;

    spi->CTRLA.reg &= (~SERCOM_SPI_CTRLA_ENABLE);
    while (spi->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_ENABLE)
    {
    }

    spi->BAUD.reg = ((float) kPlatformClock / (float) (2 * kBaudRate)) - 1;
    cfg = spi->CTRLA.reg;

    /*
     * DS p487
     *  mode          CPOL          CPHA          Leading Edge          Trailing Edge
     *  0x0           0             0            Rising,sample         Failing,change
     *  0x1           0             1            Rising,change         Failing,sample
     *  0x2           1             0            Failing,sample        Rising,change
     *  0x3           1             1            Failing,change        Rising,sample
     */
    if ((flags & SPI_MODE_CPOL) == SPI_MODE_CPOL)
    {
        cfg &= ~SERCOM_SPI_CTRLA_CPOL; /* low when idle */
    }
    else
    {
        cfg |= SERCOM_SPI_CTRLA_CPOL; /* high when idle */
    }
    if ((flags & SPI_MODE_CPHA) == SPI_MODE_CPHA)
    {
        cfg &= ~SERCOM_SPI_CTRLA_CPHA; /* sample on leading edge */
    }
    else
    {
        cfg |= SERCOM_SPI_CTRLA_CPHA; /* sample on trailing edge */
    }
    if ((flags & SPI_TRANSFER_MSB) == SPI_TRANSFER_MSB)
    {
        cfg &= ~SERCOM_SPI_CTRLA_DORD;
    }
    else
    {
        cfg |= SERCOM_SPI_CTRLA_DORD;
    }

    cfg |= SERCOM_SPI_CTRLA_ENABLE;

    spi->CTRLA.reg = cfg;
    while (spi->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_ENABLE)
    {
    }

    return 0;
}

/** Check interrupt flag of RXC and update transaction runtime information. */
static inline void spiRXCheckAndReceive(struct spi_data *spi_data)
{
    uint32_t data;

    while (!(spi->INTFLAG.reg & SERCOM_SPI_INTFLAG_RXC))
    {
    }

    data = spi->DATA.reg;

    if (spi_data->rx_cnt < spi_data->rx_buf_len)
    {
        *spi_data->rx_buf++ = (uint8_t) data;
    }

    spi_data->rx_cnt++;
}

/** Check interrupt flag of DRE and update transaction runtime information. */
static inline void spiTXCheckAndSend(struct spi_data *spi_data)
{
    uint32_t data;
    SercomSpi *spi = (SercomSpi *) SERCOM4;

    while (!(spi->INTFLAG.reg & SERCOM_SPI_INTFLAG_DRE))
    {
    }

    if (spi_data->tx_cnt < spi_data->tx_buf_len)
    {
        data = *spi_data->tx_buf++;
    }
    else
    {
        data = 0x0;
    }

    spi_data->tx_cnt++;
    spi->DATA.reg = data;
}

static inline int32_t spiErrorCheck(void)
{
    SercomSpi *spi = (SercomSpi *) SERCOM4;

    if (spi->INTFLAG.reg & SERCOM_SPI_INTFLAG_ERROR)
    {
        spi->STATUS.reg = ~0;
        spi->INTFLAG.reg = SERCOM_SPI_INTFLAG_ERROR;
        return -1;
    }

    return 0;
}

int samr21SpiTransceive(const void *tx_buf, uint32_t tx_buf_len, void *rx_buf, uint32_t rx_buf_len)
{
    int rc;
    SercomSpi *spi = (SercomSpi *) SERCOM4;

    local_data->tx_cnt = local_data->rx_cnt = 0;
    local_data->rx_buf = rx_buf;
    local_data->tx_buf = tx_buf;
    local_data->tx_buf_len = tx_buf_len;
    local_data->rx_buf_len = rx_buf_len;
    if (tx_buf_len > rx_buf_len)
    {
        local_data->xfer_len = tx_buf_len;
    }
    else
    {
        local_data->xfer_len = rx_buf_len;
    }

    selectCSPort(SPIACTIVE);

    /* If settings are not applied (pending), we can not go on */
    if (spi->SYNCBUSY.reg & (SERCOM_SPI_SYNCBUSY_SWRST | SERCOM_SPI_SYNCBUSY_ENABLE | SERCOM_SPI_SYNCBUSY_CTRLB))
    {
        return -1;
    }

    if (!spi->CTRLA.bit.ENABLE)
    {
        return -1;
    }

    for (int i = 0; i < samr21_data->xfer_len; i++)
    {
        spiTXCheckAndSend(local_data);

        spiRXCheckAndReceive(local_data);

        rc = spiErrorCheck();
        if (rc < 0)
        {
            break;
        }
    }

    /* Wait until SPI bus idle */
    while (!(spi->INTFLAG.reg & (SERCOM_SPI_INTFLAG_TXC | SERCOM_SPI_INTFLAG_DRE)))
    {
    }
    spi->INTFLAG.reg |= SERCOM_SPI_INTFLAG_TXC | SERCOM_SPI_INTFLAG_DRE;

    selectCSPort(SPIDEACTIVE);

    return rc != 0 ? rc : 0;
}

int samr21SpiInit(void)
{
    SercomSpi *spi = (SercomSpi *) SERCOM4;

    uint8_t id = 0x4;

    /* enable sync and async clocks */
    PM->APBCMASK.reg |= 1U << (PM_APBCMASK_SERCOM0_Pos + id);
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | ((GCLK_CLKCTRL_ID_SERCOM0_CORE_Val + id) << GCLK_CLKCTRL_ID_Pos);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    {
    }

    /* reset the Sercom device */
    spi->CTRLA.reg = SERCOM_SPI_CTRLA_SWRST;
    while (spi->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_SWRST)
    {
    }

    /* set master mode */
    spi->CTRLA.reg = SERCOM_SPI_CTRLA_DIPO(mux_pad0) | SERCOM_SPI_CTRLA_DOPO(mux_group) | SERCOM_SPI_CTRLA_MODE_SPI_MASTER;
    spi->CTRLB.reg &= (~(SERCOM_SPI_CTRLB_MSSEN | SERCOM_SPI_CTRLB_AMODE_Msk | SERCOM_SPI_CTRLB_SSDE | SERCOM_SPI_CTRLB_PLOADEN)) | SERCOM_SPI_CTRLB_RXEN;
    while (spi->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_CTRLB)
    {
    }

    spi->BAUD.reg = ((float) kPlatformClock / (float) (2 * kBaudRate)) - 1;
    spi->DBGCTRL.reg = 0;

    /* configure pins */
    pinmuxSetDefault();

    pinmuxSetPort();

    selectCSPort(SPIDEACTIVE);

    /* finally, enable the device */
    spi->CTRLB.reg |= SERCOM_SPI_CTRLB_RXEN;
    while (spi->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_CTRLB)
    {
    }
    spi->CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;
    while (spi->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_ENABLE)
    {
    }

    return 0;
}

