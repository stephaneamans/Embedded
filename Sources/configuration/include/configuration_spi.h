#ifndef CONFIGURATION_SPI_H
#define CONFIGURATION_SPI_H

/*********************************
 * SPI definitions
 ********************************/

/* SPI1 configuration                     */
#define SPI1_BASE                  (APB2PERIPH_BASE + 0x00003000U)
#define SPI1_FREQUENCY_KHZ         4500
#define SPI1_FRAME_LENGTH          spi_frame_8_bits
#define SPI1_FRAME_DIRECTION       spi_frame_msb_first
#define SPI1_CLOCK_PHASE           spi_clk_first
#define SPI1_CLOCK_POLARITY        spi_clk_rising
#define SPI1_CS_ACTIVE             spi_cs_active
#define SPI1_DMA_DRIVER_TX         &dma_driver[3]
#define SPI1_DMA_DRIVER_RX         &dma_driver[4]

/* SPI2 configuration                     */
#define SPI2_BASE                  (APB1PERIPH_BASE + 0x00003800U)
#define SPI2_FREQUENCY_KHZ         8000
#define SPI2_FRAME_LENGTH          spi_frame_8_bits
#define SPI2_FRAME_DIRECTION       spi_frame_msb_first
#define SPI2_CLOCK_PHASE           spi_clk_first
#define SPI2_CLOCK_POLARITY        spi_clk_rising
#define SPI2_CS_ACTIVE             spi_cs_active
#define SPI2_DMA_DRIVER_TX         0
#define SPI2_DMA_DRIVER_RX         0

#endif /* CONFIGURATION_SPI_H_ */
