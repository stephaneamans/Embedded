/*
 * spi.h
 *
 * Created on: Jun 25, 2021
 * Author: Stephane Amans
 */

#ifndef SPI_H_
#define SPI_H_

/* Include files:        */
#include <stdint.h>
#include <stdbool.h>

#include "configuration_module_activation.h"
#include "configuration_spi.h"

#include "gpio.h"

#include "lld_clock.h"
#include "lld_dma.h"
#include "lld_nvic.h"

#define USART_BUFFER_TX_LENGTH 256
#define USART_BUFFER_RX_LENGTH 256

#define USART_BUFFER_LENGTH 256

enum t_frame_length
{
    spi_frame_8_bits,
	spi_frame_16_bits
};

enum t_frame_direction
{
    spi_frame_msb_first,
	spi_frame_lsb_first
};

enum t_clock_phase
{
    spi_clk_first,
	spi_clk_second
};

enum t_clock_polarity
{
    spi_clk_rising,
	spi_clk_falling
};

enum t_cs_active
{
    spi_cs_inactive,
	spi_cs_active
};

/*


Configuring the SPI for half-duplex communication
The SPI is capable of operating in half-duplex mode in 2 configurations.
• 1 clock and 1 bidirectional data wire
• 1 clock and 1 data wire (receive-only or transmit-only)
1 clock and 1 bidirectional data wire (BIDIMODE = 1)
This mode is enabled by setting the BIDIMODE bit in the SPI_CR1 register. In this mode
SCK is used for the clock and MOSI in master or MISO in slave mode is used for data
communication. The transfer direction (Input/Output) is selected by the BIDIOE bit in the
SPI_CR1 register. When this bit is 1, the data line is output otherwise it is input.
1 clock and 1 unidirectional data wire (BIDIMODE = 0)
In this mode, the application can use the SPI either in transmit-only mode or in receive-only
mode.
• Transmit-only mode is similar to full-duplex mode (BIDIMODE=0, RXONLY=0): the
data are transmitted on the transmit pin (MOSI in master mode or MISO in slave mode)
and the receive pin (MISO in master mode or MOSI in slave mode) can be used as a
general-purpose IO. In this case, the application just needs to ignore the Rx buffer (if
the data register is read, it does not contain the received value).
• In receive-only mode, the application can disable the SPI output function by setting the
RXONLY bit in the SPI_CR1 register. In this case, it frees the transmit IO pin (MOSI in
master mode or MISO in slave mode), so it can be used for other purposes.
To start the communication in receive-only mode, configure and enable the SPI:
25.3.5
• In master mode, the communication starts immediately and stops when the SPE bit is
cleared and the current reception stops. There is no need to read the BSY flag in this
mode. It is always set when an SPI communication is ongoing.
• In slave mode, the SPI continues to receive as long as the NSS is pulled down (or the
SSI bit is cleared in NSS software mode) and the SCK is running.





Data transmission and reception procedures
Rx and Tx buffers
In reception, data are received and then stored into an internal Rx buffer while In
transmission, data are first stored into an internal Tx buffer before being transmitted.
A read access of the SPI_DR register returns the Rx buffered value whereas a write access
to the SPI_DR stores the written data into the Tx buffer.



Start sequence in master mode
•
•
•
•
In full-duplex (BIDIMODE=0 and RXONLY=0)
– The sequence begins when data are written into the SPI_DR register (Tx buffer).
– The data are then parallel loaded from the Tx buffer into the 8-bit shift register
during the first bit transmission and then shifted out serially to the MOSI pin.
– At the same time, the received data on the MISO pin is shifted in serially to the 8-
bit shift register and then parallel loaded into the SPI_DR register (Rx buffer).
In unidirectional receive-only mode (BIDIMODE=0 and RXONLY=1)
– The sequence begins as soon as SPE=1
– Only the receiver is activated and the received data on the MISO pin are shifted in
serially to the 8-bit shift register and then parallel loaded into the SPI_DR register
(Rx buffer).
In bidirectional mode, when transmitting (BIDIMODE=1 and BIDIOE=1)
– The sequence begins when data are written into the SPI_DR register (Tx buffer).
– The data are then parallel loaded from the Tx buffer into the 8-bit shift register
during the first bit transmission and then shifted out serially to the MOSI pin.
– No data are received.
In bidirectional mode, when receiving (BIDIMODE=1 and BIDIOE=0)
– The sequence begins as soon as SPE=1 and BIDIOE=0.
– The received data on the MOSI pin are shifted in serially to the 8-bit shift register
and then parallel loaded into the SPI_DR register (Rx buffer).
– The transmitter is not activated and no data are shifted out serially to the MOSI
pin.



Handling data transmission and reception
The TXE flag (Tx buffer empty) is set when the data are transferred from the Tx buffer to the
shift register. It indicates that the internal Tx buffer is ready to be loaded with the next data.
An interrupt can be generated if the TXEIE bit in the SPI_CR2 register is set. Clearing the
TXE bit is performed by writing to the SPI_DR register.
Note:
The software must ensure that the TXE flag is set to 1 before attempting to write to the Tx
buffer. Otherwise, it overwrites the data previously written to the Tx buffer.
The RXNE flag (Rx buffer not empty) is set on the last sampling clock edge, when the data
are transferred from the shift register to the Rx buffer. It indicates that data are ready to be
read from the SPI_DR register. An interrupt can be generated if the RXNEIE bit in the
SPI_CR2 register is set. Clearing the RXNE bit is performed by reading the SPI_DR
register.
For some configurations, the BSY flag can be used during the last data transfer to wait until
the completion of the transfer.
Full-duplex transmit and receive procedure in master or slave mode (BIDIMODE=0 and
RXONLY=0)
The software has to follow this procedure to transmit and receive data (see Figure 241 and
Figure 242):
1. Enable the SPI by setting the SPE bit to 1.
2. Write the first data item to be transmitted into the SPI_DR register (this clears the TXE
flag).
3. Wait until TXE=1 and write the second data item to be transmitted. Then wait until
RXNE=1 and read the SPI_DR to get the first received data item (this clears the RXNE
bit). Repeat this operation for each data item to be transmitted/received until the n–1
received data.
4. Wait until RXNE=1 and read the last received data.
5. Wait until TXE=1 and then wait until BSY=0 before disabling the SPI.
This procedure can also be implemented using dedicated interrupt subroutines launched at
each rising edges of the RXNE or TXE flag.
*/






struct t_spi_data
{
    uint16_t *write_buffer;
    uint16_t *read_buffer;
    uint32_t data_length;
};

struct t_spi_slave
{
    struct t_gpio_driver *cs;
    uint16_t freq_khz;
    enum t_frame_length frame_length;
    enum t_frame_direction frame_direction;
    enum t_clock_phase clock_phase;
    enum t_clock_polarity clock_polarity;
    enum t_cs_active cs_active;
};





/* SPI configuration structure definition :        */
struct t_spi_config
{
	uintptr_t base_address;
	uint8_t instance_number;
	uint16_t freq_khz;
	enum t_frame_length frame_length;
	enum t_frame_direction frame_direction;
	enum t_clock_phase clk_phase;
	enum t_clock_polarity clk_polarity;
	enum t_cs_active cs_active;
    struct
    {
    	enum irq_priority priority;
    	struct t_dma_driver *tx_dma_channel;
    	struct t_dma_driver *rx_dma_channel;
    }irq_dma;
};

/* SPI driver structure definition :        */
struct t_spi_driver
{
	uint16_t freq_khz;
	struct t_spi_private *priv;
};

//struct t_dma_methods
//{
/** Pointer to send data via USART.
 * This will call static functions send_usart_poll, send_usart_irq
 * or send_usart_dma function of the configuration. This pointer is affected
 * at the init of the driver.
 *
 * \param usart: address of the USART driver.
 * \param data_buffer: Pointer to the data buffer to send.
 * \param data_length: Data length.
 *
* \return: t_error_handling code or ERROR_OK.
 *
 */
//    t_error_handling (*transmit)(struct t_usart_driver *driver, uint8_t *data_buffer, uint32_t data_length);

/** Pointer to receive data via USART.
 * This will call static functions receive_usart_poll, receive_usart_irq
 * or receive_usart_dma function of the configuration. This pointer is affected
 * at the init of the driver.
 *
 * \param usart: address of the USART driver.
 * \param data_buffer: Pointer to the data receive buffer.
 *
 * \return: t_error_handling code or ERROR_OK.
 *
 */
//    t_error_handling (*receive)(struct t_usart_driver *driver, uint8_t *data_buffer, uint32_t data_length);

/** Get transmission complete flag in DMA mode.
*
* \param usart: Pointer to the USART driver.
*
*
* \return: bool .
*
*/
//    bool (*tx_complete)(struct t_usart_driver *driver);

/** Get reception complete flag in DMA mode.
*
* \param usart: Pointer to the USART driver.
*
*
* \return: bool.
*
*/
//    bool (*rx_complete)(struct t_usart_driver *driver);

//}methods;
//};

/* USARTx_CRx, Peripheral configuration register group definition:        */

/* DMA bits configuration :             */
#define DMA_RX_ENABLE           0x01
#define DMA_TX_ENABLE           0x02
#define DMA_RX_TX_ENABLE        0x03

extern struct t_spi_driver spi_driver[SPI_IP_NUMBER];
/* Functions prototypes:                */

/** Configure USART (baudrate, interruptions, mode, ...):
 *
 * \param driver: Driver structure
 * \param config: Configuration structure.
 *
 * \return: t_error_handling code or ERROR_OK.
 *
 */
void spi_init(struct t_spi_driver *driver, const struct t_spi_config *config);
t_error_handling spi_transfer(struct t_spi_driver *driver, struct t_spi_slave *slave, struct t_spi_data *data);

/** Get USART RX transfer status:
 *
 * \param driver: Driver structure.
 *
 * \return: bool : RX transfer status.
 *
 */
//bool usart_get_rx_transfer_status(struct t_usart_driver *driver);

/** Get USART TX transfer status:
 *
 * \param driver: Driver structure.
 *
  * \return: bool : TX transfer status.
 *
 */
//bool usart_get_tx_transfer_status(struct t_usart_driver *driver);

/** Get USART error status:
 *
 * \param driver: Driver structure.
 *
 * \return: t_error_handling code or ERROR_OK.
 *
 */
//t_error_handling usart_get_error_status(struct t_usart_driver *driver);


/** Return SPI driver:
 *
 * \param spi_number: number of the spi driver, must be between 1 and the maximum number of spi IPs available in the IC.
 *
 * \return: Pointer to the driver or 0 if error.
 *
 */
struct t_spi_driver *spi_get_driver(uint8_t spi_number);


#endif /* SPI_H_ */
