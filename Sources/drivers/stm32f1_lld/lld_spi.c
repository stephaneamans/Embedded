/*
 * lld_spi.c
 *
 * Created on: Jun 25, 2021
 * Author: Stephane Amans
 *
 */

/* Include files        */
#include <stdio.h>
#include <string.h>

#include "bsp.h"
#include "fault.h"
#include "spi.h"

#include "regbase_spi.h"

#include "configuration_soc.h"
#if defined(SPI_1)

/* Defines */
#define SPI_CR1_CPHA_BIT_MASK             0x1
#define SPI_CR1_CPOL_BIT_MASK             0x2
#define SPI_CR1_MSTR_BIT_MASK             0x4
#define SPI_CR1_BR_BIT_MASK              0x38
#define SPI_CR1_SPE_BIT_MASK             0x40
#define SPI_CR1_LSBFIRST_BIT_MASK        0x80
#define SPI_CR1_DFF_BIT_MASK            0x800

#define SPI_CR2_SSOE_BIT_MASK             0x4

#define SPI_SR_TXE_BIT_MASK               0x2
#define SPI_SR_RXNE_BIT_MASK              0x1

#define RX_BUFFER_LENGTH    256
#define TX_BUFFER_LENGTH    256

/* SPI private structure definition :        */
struct t_spi_private
{
	struct t_spi_regs *reg;
	struct t_spi_slave *spi_slaves;
	uint32_t clock_frequency;
	enum t_frame_length frame_length;
	enum t_frame_direction frame_direction;
	enum t_clock_phase clock_phase;
	enum t_clock_polarity clock_polarity;
	enum t_cs_active cs_active;

	bool write_end;
	bool read_end;
	uint8_t *data_buffer_tx;
	uint32_t length_tx;
	uint8_t *data_buffer_rx;
	uint32_t length_rx;
	t_error_handling error;
	struct t_dma_driver *tx_dma;
    struct t_dma_driver *rx_dma;
};

//    struct t_gpio_driver *cs;
//    uint16_t freq_khz;
//    enum t_frame_length frame_length;
//    enum t_frame_direction frame_direction;
//    enum t_clock_phase clk_phase;
//    enum t_clock_polarity clk_polarity;


/* Static driver structure. */
struct t_spi_driver spi_driver[SPI_IP_NUMBER];
static struct t_spi_private priv[SPI_IP_NUMBER];


/** Get transmission complete flag in interruption and DMA modes.
 *
  * \param usart: Pointer to the USART driver.
 *
 *
 * \return: bool .
 *
 */
/*static bool tx_complete_irq(struct t_usart_driver *driver)
{
    return driver->priv->write_end;
}

static bool tx_complete_dma(struct t_usart_driver *driver)
{
    bool status = true;
    if(driver->priv->write_end == false)
    {
        status = false;
        if(driver->priv->tx_dma->reg->CNDTR == 0)
        {
            status = true;
            driver->priv->write_end = true;
        }
    }
    return status;
}*/


/** Get reception complete flag in interruption and DMA modes.
 *
  * \param usart: Pointer to the USART driver.
 *
 *
 * \return: bool.
 *
 */
/*static bool rx_complete_irq(struct t_usart_driver *driver)
{
    return driver->priv->read_end;
}

static bool rx_complete_dma(struct t_usart_driver *driver)
{
  bool status = true;
  if(driver->priv->read_end == false)
  {
        status = false;
        if(driver->priv->rx_dma->reg->CNDTR == 0)
        {
            status = true;
            driver->priv->read_end = true;
        }
    }
    return status;
}*/


/** Function to receive bytes data with DMA.
 *
 * \param driver: Pointer to the USART driver.
 * \param data_buffer: Pointer to the data to send.
 * \param data_length: Length of the datas to send.
 *
 * \return: t_error_handling code or ERROR_OK.
 *
 */
/*static t_error_handling receive_dma(struct t_usart_driver *driver, uint8_t *data_buffer, uint32_t data_length)
{
    t_error_handling error = ERROR_OK;
    if(rx_complete_dma(driver))
    {
        error = dma_transfer(driver->priv->rx_dma, data_buffer, &driver->reg->DR, data_length);
        driver->priv->length_rx = data_length;
        driver->priv->read_end = false;
        driver->reg->CR1 |= RX_ENABLE;
        error = dma_start_transfer(driver->priv->rx_dma);
    }
    else
    {
        error = ERROR_USART_NOT_READY_TO_SEND;
        driver->priv->error = ERROR_USART_NOT_READY_TO_SEND;
    }
    return error;
}*/


/** Function to receive bytes data with interrupts.
 *
 * \param driver: Pointer to the USART driver.
 * \param data_buffer: Pointer to the data to send.
 * \param data_length: Length of the datas to send.
 *
 * \return: t_error_handling code or ERROR_OK.
 *
 */
/*static t_error_handling receive_irq(struct t_usart_driver *driver, uint8_t *data_buffer, uint32_t data_length)
{
    t_error_handling error = ERROR_OK;
    if((driver->reg->CR1 & RX_ENABLE) != RX_ENABLE)
    {
        driver->priv->read_end = false;
        driver->priv->data_buffer_rx = data_buffer;
        driver->priv->length_rx = data_length;
        driver->reg->SR &= ~RX_NOT_EMPTY_FLAG;
        driver->reg->CR1 |= (ENABLE_IDLE_IRQ | ENABLE_RXNE_IRQ | ENABLE_PE_IRQ | RX_ENABLE);
    }
    else
    {
        error = ERROR_USART_NOT_READY_TO_SEND;
        driver->priv->error = ERROR_USART_NOT_READY_TO_SEND;
    }
    return error;
}*/


/** Simple polling function to read bytes data by polling.
 *
 * \param driver: Pointer to the USART driver.
 * \param data_buffer: Pointer to the data to send.
 * \param data_length: Length of the datas to send.
 *
 * \return : Error code or ERROR_OK.
 *
 */
/*static t_error_handling receive_poll(struct t_usart_driver *driver, uint8_t *data_buffer, uint32_t data_length)
{
    t_error_handling error = ERROR_OK;
    driver->priv->error = ERROR_OK;
    driver->priv->read_end = false;
    driver->reg->CR1 |= RX_ENABLE;
    for(uint32_t index = 0; index < data_length; index++)
    {
        while((driver->reg->SR & RX_NOT_EMPTY_FLAG) == 0){}
        *data_buffer = driver->reg->DR;
        if((driver->reg->SR & PARITY_ERROR_FLAG) == PARITY_ERROR_FLAG)
        {
            error = ERROR_USART_PARITY;
            break;
        }
        else if((driver->reg->SR & FRAMING_ERROR_FLAG) == FRAMING_ERROR_FLAG)
        {
            error = ERROR_USART_FRAMING;
            break;
        }
        else if((driver->reg->SR & NOISE_ERROR_FLAG) == NOISE_ERROR_FLAG)
        {
            error = ERROR_USART_NOISE;
            break;
        }
        else if((driver->reg->SR & OVERRUN_ERROR_FLAG) == OVERRUN_ERROR_FLAG)
        {
            error = ERROR_USART_OVERRUN;
            break;
        }
        else
        {
        }
        data_buffer++;
    }
    driver->reg->CR1 &= ~RX_ENABLE;
    driver->priv->read_end = true;
    driver->priv->error = error;
    return error;
}*/


/** Function to transmit a bytes data with DMA.
 *
 * \param usart: Pointer to the USART driver.
 * \param data_buffer: Pointer to the data location.
 * \param data_length: Length of the datas to send.
 *
* \return: t_error_handling code or ERROR_OK.
 *
 */
/*static t_error_handling transmit_dma(struct t_usart_driver *driver, uint8_t *data_buffer, uint32_t data_length)
{
    t_error_handling error = ERROR_OK;
    if(tx_complete_dma(driver))
    {
        error = dma_transfer(driver->priv->tx_dma, data_buffer, &driver->reg->DR, data_length);
        driver->priv->length_tx = data_length;
        driver->priv->write_end = false;
        driver->reg->CR1 |= TX_ENABLE;
        error = dma_start_transfer(driver->priv->tx_dma);
    }
    else
    {
        error = ERROR_USART_NOT_READY_TO_SEND;
        driver->priv->error = ERROR_USART_NOT_READY_TO_SEND;
    }
return error;
}*/


/** Function to transmit bytes data with interrupts.
 *
 * \param usart: Pointer to the USART driver.
 * \param data_buffer: Pointer to the data location.
 * \param data_length: Length of the datas to send.
 *
 * \return: t_error_handling code or ERROR_OK.
 *
 */
/*static t_error_handling transmit_irq(struct t_usart_driver *driver, uint8_t *data_buffer, uint32_t data_length)
{
    t_error_handling error = ERROR_OK;
    if((driver->reg->CR1 & TX_ENABLE) != TX_ENABLE)
    {
        driver->priv->write_end = false;
        driver->priv->data_buffer_tx = data_buffer;
        driver->priv->length_tx = data_length;
        driver->reg->CR1 |= TX_ENABLE;
        driver->reg->SR = 0x00;
        driver->reg->DR = *data_buffer;
        priv->data_buffer_tx++;
        priv->length_tx--;
        driver->reg->CR1 |= (ENABLE_IDLE_IRQ | ENABLE_TC_IRQ | ENABLE_TXE_IRQ | ENABLE_PE_IRQ);
    }
    else
    {
        error = ERROR_USART_NOT_READY_TO_SEND;
        driver->priv->error = ERROR_USART_NOT_READY_TO_SEND;
    }
return error;
}*/

/** Simple blocking function to transmit bytes data.
 *
 * \param usart: Pointer to the USART driver.
 * \param data_buffer: Pointer to the data location.
 * \param data_length: Length of the datas to send.
 *
 * \return: t_error_handling code or ERROR_OK.
 *
 */
/*static t_error_handling transmit_poll(struct t_usart_driver *driver, uint8_t *data_buffer, uint32_t data_length)
{
    t_error_handling error = ERROR_OK;
    driver->priv->write_end = false;
    driver->reg->CR1 |= TX_ENABLE;
    for(uint32_t index = 0; index < data_length; index++)
    {
        driver->reg->DR = *data_buffer;
        while((driver->reg->SR & TXE_FLAG) != TXE_FLAG){}
        data_buffer++;
    }
    while((driver->reg->SR & TC_FLAG) != TC_FLAG){}
    driver->reg->CR1 &= ~TX_ENABLE;
    driver->priv->error = error;
    return error;
}*/

/** Compute divider from the bus frequency.
 *
 * \param bus_frequency: USART bus frequency value.
 *
 *
 * \return: t_error_handling code or ERROR_OK.
 *
 */
/*static uint16_t compute_baudrate_divider(uint32_t bus_frequency, uint32_t baudrate)
{
    uint32_t local_divider = baudrate * 16;
    uint16_t mantissa = bus_frequency / local_divider;
    uint8_t fraction = (uint8_t)(((bus_frequency % local_divider) * 16) / local_divider);

    if(fraction > 16)
    {
        mantissa++;
        fraction -= 16;
    }
    return ((mantissa << 4) | (fraction & 0x0F));
}*/


static t_error_handling compute_frequency_divider(uint32_t bus_frequency, uint16_t spi_frequency, uint8_t *divider_code)
{
    t_error_handling error = ERROR_OK;
    uint32_t local_spi_frequency = (uint32_t)spi_frequency * 1000;
    uint16_t divider = (uint16_t)(bus_frequency/local_spi_frequency);
    uint32_t modulo = bus_frequency%local_spi_frequency;

    if(divider < 3)
    {
        *divider_code = 0;
    }
    else if((divider >= 3) && (divider < 6))
    {
        *divider_code = (1 << 3) & SPI_CR1_BR_BIT_MASK;
    }
    else if((divider >= 6) && (divider < 12))
    {
        *divider_code = (2 << 3) & SPI_CR1_BR_BIT_MASK;
    }
    else if((divider >= 12) && (divider < 20))
    {
        *divider_code = (3 << 3) & SPI_CR1_BR_BIT_MASK;
    }
    else if((divider >= 20) && (divider < 48))
    {
        *divider_code = (4 << 3) & SPI_CR1_BR_BIT_MASK;
    }
    else if((divider >= 48) && (divider < 96))
    {
        *divider_code = (5 << 3) & SPI_CR1_BR_BIT_MASK;
    }
    else if((divider >= 96) && (divider < 192))
    {
        *divider_code = (6 << 3) & SPI_CR1_BR_BIT_MASK;
    }
    else
    {
    	*divider_code = (7 << 3) & SPI_CR1_BR_BIT_MASK;
    }
    if(modulo != 0)
    {
        error = ERROR_WRONG_CLOCK_SET;
    }
    return error;
};

static t_error_handling set_chip_select(struct t_spi_driver *driver, enum t_cs_active cs_active)
{
    t_error_handling error = ERROR_OK;
    driver->priv->reg->CR1 &= ~SPI_CR1_SPE_BIT_MASK;
    if(cs_active == spi_cs_inactive)
    {
        driver->priv->reg->CR2 &= ~SPI_CR2_SSOE_BIT_MASK;
    }
    else if(cs_active == spi_cs_active)
    {
        driver->priv->reg->CR2 |= SPI_CR2_SSOE_BIT_MASK;
    }
    else
    {
        error = ERROR_WRONG_VALUE;
    }
    driver->priv->reg->CR1 |= SPI_CR1_SPE_BIT_MASK;
    return error;
}

static t_error_handling set_clock_phase(struct t_spi_driver *driver, enum t_clock_phase clock_phase)
{
    t_error_handling error = ERROR_OK;
    driver->priv->reg->CR1 &= ~SPI_CR1_SPE_BIT_MASK;
	if(clock_phase == spi_clk_first)
    {
        driver->priv->reg->CR1 &= ~SPI_CR1_CPHA_BIT_MASK;
    }
    else if(clock_phase == spi_clk_second)
    {
    	driver->priv->reg->CR1 |= SPI_CR1_CPHA_BIT_MASK;
    }
    else
    {
        error = ERROR_WRONG_VALUE;
    }
	driver->priv->reg->CR1 |= SPI_CR1_SPE_BIT_MASK;
    driver->priv->clock_phase = clock_phase;
    return error;
}

static t_error_handling set_clock_polarity(struct t_spi_driver *driver, enum t_clock_polarity clock_polarity)
{
    t_error_handling error = ERROR_OK;
    driver->priv->reg->CR1 &= ~SPI_CR1_SPE_BIT_MASK;
    if(clock_polarity == spi_clk_rising)
    {
        driver->priv->reg->CR1 &= ~SPI_CR1_CPOL_BIT_MASK;
    }
    else if(clock_polarity == spi_clk_falling)
    {
        driver->priv->reg->CR1 |= SPI_CR1_CPOL_BIT_MASK;
    }
    else
    {
        error = ERROR_WRONG_VALUE;
    }
    driver->priv->reg->CR1 |= SPI_CR1_SPE_BIT_MASK;
    driver->priv->clock_polarity = clock_polarity;
    return error;
}

static t_error_handling set_frame_direction(struct t_spi_driver *driver, enum t_frame_direction frame_direction)
{
    t_error_handling error = ERROR_OK;
    driver->priv->reg->CR1 &= ~SPI_CR1_SPE_BIT_MASK;
    if(frame_direction == spi_frame_msb_first)
    {
        driver->priv->reg->CR1 &= ~SPI_CR1_LSBFIRST_BIT_MASK;
    }
    else if(frame_direction == spi_frame_lsb_first)
    {
        driver->priv->reg->CR1 |= SPI_CR1_LSBFIRST_BIT_MASK;
    }
    else
    {
        error = ERROR_WRONG_VALUE;
    }
    driver->priv->reg->CR1 |= SPI_CR1_SPE_BIT_MASK;
    driver->priv->frame_direction = frame_direction;
    return error;
};

static t_error_handling set_frame_length(struct t_spi_driver *driver, enum t_frame_length frame_length)
{
	t_error_handling error = ERROR_OK;
	driver->priv->reg->CR1 &= ~SPI_CR1_SPE_BIT_MASK;
	if(frame_length == spi_frame_8_bits)
    {
        driver->priv->reg->CR1 &= ~SPI_CR1_DFF_BIT_MASK;
    }
    else if(frame_length == spi_frame_16_bits)
    {
    	driver->priv->reg->CR1 |= SPI_CR1_DFF_BIT_MASK;
    }
    else
    {
        error = ERROR_WRONG_VALUE;
    }
    driver->priv->reg->CR1 |= SPI_CR1_SPE_BIT_MASK;
    driver->priv->frame_length = frame_length;
    return error;
}

static t_error_handling set_frequency(struct t_spi_driver *driver, uint16_t freq_khz)
{
    t_error_handling error = ERROR_OK;
    driver->priv->reg->CR1 &= ~SPI_CR1_SPE_BIT_MASK;
    uint16_t local_mask = 0;
    error = compute_frequency_divider(driver->priv->clock_frequency,
                                      freq_khz,
                                      (uint8_t*)&local_mask);
    driver->priv->reg->CR1 &= ~SPI_CR1_BR_BIT_MASK;
    driver->priv->reg->CR1 |= local_mask;
    driver->priv->reg->CR1 |= SPI_CR1_SPE_BIT_MASK;
    driver->freq_khz = freq_khz;
    return error;
}

t_error_handling spi_transfer(struct t_spi_driver *driver, struct t_spi_slave *slave, struct t_spi_data *data)
{
	t_error_handling error = ERROR_OK;

    struct t_gpio_driver *cs;
    if(slave->freq_khz != driver->freq_khz)
    {
        error = set_frequency(driver, slave->freq_khz);
        if(error != ERROR_OK)
        {
            goto end;
        };
    };

    if(slave->frame_length != driver->priv->frame_length)
    {
        error = set_frame_length(driver, slave->frame_length);
        if(error != ERROR_OK)
        {
            goto end;
        };
    };

    if(slave->frame_direction != driver->priv->frame_direction)
    {
        error = set_frame_direction(driver, slave->frame_direction);
    	if(error != ERROR_OK)
        {
            goto end;
        };
    };

    if(slave->clock_phase != driver->priv->clock_phase)
    {
        error = set_clock_phase(driver, slave->clock_phase);
        if(error != ERROR_OK)
        {
            goto end;
        };
    };

    if(slave->clock_polarity != driver->priv->clock_polarity)
    {
        error = set_clock_polarity(driver, slave->clock_polarity);
        if(error != ERROR_OK)
        {
            goto end;
        };
    };

    if(slave->cs_active != driver->priv->cs_active)
    {
        error = set_chip_select(driver, slave->cs_active);
        if(error != ERROR_OK)
        {
           goto end;
        };
    };

    slave->cs->methods->gpio_clear(slave->cs, 0);

	while(data->data_length > 0)
    {
        driver->priv->reg->DR = *data->write_buffer;
        while((driver->priv->reg->SR & SPI_SR_TXE_BIT_MASK) != SPI_SR_TXE_BIT_MASK){}
        while((driver->priv->reg->SR & SPI_SR_RXNE_BIT_MASK) == SPI_SR_RXNE_BIT_MASK){}
        *data->read_buffer = driver->priv->reg->DR;
        data->write_buffer++;
        data->read_buffer++;
        data->data_length--;
    };
end:
    return error;
}

void spi_init(struct t_spi_driver *driver, const struct t_spi_config *config)
{
    t_error_handling error  = ERROR_OK;

    /* Clear the driver instance */
    memset(driver, 0, sizeof(struct t_spi_driver));

    /* Get the SoC frequency parameters */
    struct t_clock_driver *clock_driver = get_clock_driver();

    /* Clear the private structure instance */
    memset(driver->priv, 0, sizeof(struct t_spi_private));

    /* Associate private instance to the driver */
    driver->priv = &priv[config->instance_number-1];

    /* For any SPI instance */
    if(config->instance_number == 1)
    {
   	    enable_clock(SPI1);
   	    driver->priv->clock_frequency = clock_driver->APB2_clk_freq;
    }
    else if(config->instance_number == 2)
    {
    	enable_clock(SPI2);
    	driver->priv->clock_frequency = clock_driver->APB1_clk_freq;
    }

    driver->priv->reg = (struct t_spi_regs*)config->base_address;
    struct t_spi_regs *reg = driver->priv->reg;
    driver->priv->frame_length = config->frame_length;

    /* Enable SPI in master mode */
    reg->CR1 |= (SPI_CR1_SPE_BIT_MASK | SPI_CR1_MSTR_BIT_MASK);
}

struct t_spi_driver *spi_get_driver(uint8_t spi_number)
{
    struct t_spi_driver *driver = 0;
    if((spi_number > 0)  && (spi_number <= SPI_IP_NUMBER))
    {
        driver = &spi_driver[spi_number - 1];
    }
    else
    {
        driver = NULL;
    }
    return driver;
}

//t_error_handling usart_get_error_status(struct t_usart_driver *driver)/
//{
//    return driver->priv->error;
//}


void SPI0_IRQHandler(void)        			/* USART1 global interrupt                          */
{
    /**	SPI1 IRQ handler.
    *
    * \param void : No parameter.
    *
    * \return : No return value.
    */
/*    uint8_t sr_register = usart_driver[0].reg->SR;
    if((sr_register & PARITY_ERROR_FLAG) == PARITY_ERROR_FLAG)
    {
        usart_driver[0].priv->error = ERROR_USART_PARITY;
    }
    else if((sr_register & FRAMING_ERROR_FLAG) == FRAMING_ERROR_FLAG)
    {
        usart_driver[0].priv->error  = ERROR_USART_FRAMING;
    }
    else if((sr_register & NOISE_ERROR_FLAG) == NOISE_ERROR_FLAG)
    {
        usart_driver[0].priv->error  = ERROR_USART_NOISE;
    }
    else if((sr_register & OVERRUN_ERROR_FLAG) == OVERRUN_ERROR_FLAG)
    {
        usart_driver[0].priv->error  = ERROR_USART_OVERRUN;
    }
    else if((sr_register & RX_NOT_EMPTY_FLAG) == RX_NOT_EMPTY_FLAG)
    {
        *usart_driver[0].priv->data_buffer_rx = usart_driver[0].reg->DR;
        priv->length_rx--;
        if(priv->length_rx == 0)
        {
            usart_driver[0].reg->CR1 &= ~(ENABLE_IDLE_IRQ | RX_NOT_EMPTY_FLAG | ENABLE_PE_IRQ | RX_ENABLE);
            usart_driver[0].priv->read_end = true;
        }
        else
        {
            usart_driver[0].priv->data_buffer_rx++;
        }
    }
    else if(((sr_register & TXE_FLAG) == TXE_FLAG) &&
             (usart_driver[0].priv->length_tx > 0))
    {
        usart_driver[0].reg->DR = *usart_driver[0].priv->data_buffer_tx;
        usart_driver[0].priv->data_buffer_tx++;
        usart_driver[0].priv->length_tx--;
    }
    else if((sr_register & TC_FLAG) == TC_FLAG)
    {
        usart_driver[0].reg->CR1 &= ~TX_ENABLE;
        usart_driver[0].reg->CR1 &= ~(ENABLE_IDLE_IRQ | ENABLE_TC_IRQ | ENABLE_TXE_IRQ | ENABLE_PE_IRQ);
        usart_driver[0].priv->write_end = true;
    }
    else
    {
    }*/
//    clear_pending_nvic_irq(IRQ_USART_1); /* Clear any USART 1 NVIC pending interrupt.   */
}

void SPI1_IRQHandler(void)
{
    /**	USART2 IRQ handler.
    *
    * \param void : No parameter.
    *
    * \return : No return value.
    */

}

#endif /* SPI */
