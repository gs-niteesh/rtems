/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief SPI support API.
 *
 * Based on bsps/m68k/gen68360/spi/m360_spi.h
 */

/*
 * Copyright (c) 2018 Pierre-Louis Garnier <garnie_a@epita.fr>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_BEAGLE_SPI_H
#define LIBBSP_ARM_BEAGLE_SPI_H

#include <bsp.h>
#include <rtems/libi2c.h>
#include <rtems/irq.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define BEAGLE_SPI_TIMEOUT 1000

#define BEAGLE_SPI_0_BUS_PATH "/dev/spi-0"

#define AM335X_SPI_MAX_SPEED 48000000

#define MCSPI_EXTCLK_MSK          0xfff
#define MCSPI_CONF_CLK_SHIFT      2
#define MCSPI_CONF_CLK_MSK        0xf
#define MCSPI_CONF_CLKG           (1 << 29)
#define MCSPI_CTRL_EXTCLK_MSK     0xff
#define MCSPI_CTRL_EXTCLK_SHIFT   8

typedef struct {
  uint32_t MCSPI_REVISION;
  uint32_t dummy1[68];
  uint32_t MCSPI_SYSCONFIG;
  uint32_t MCSPI_SYSSTATUS;
  uint32_t MCSPI_IRQSTATUS;
  uint32_t MCSPI_IRQENABLE;
  uint32_t dummy2[2];
  uint32_t MCSPI_SYST;
  uint32_t MCSPI_MODULCTRL;
  uint32_t MCSPI_CH0CONF;
  uint32_t MCSPI_CH0STAT;
  uint32_t MCSPI_CH0CTRL;
  uint32_t MCSPI_TX0;
  uint32_t MCSPI_RX0;
  uint32_t MCSPI_CH1CONF;
  uint32_t MCSPI_CH1STAT;
  uint32_t MCSPI_CH1CTRL;
  uint32_t MCSPI_TX1;
  uint32_t MCSPI_RX1;
  uint32_t MCSPI_CH2CONF;
  uint32_t MCSPI_CH2STAT;
  uint32_t MCSPI_CH2CTRL;
  uint32_t MCSPI_TX2;
  uint32_t MCSPI_RX2;
  uint32_t MCSPI_CH3CONF;
  uint32_t MCSPI_CH3CTRL;
  uint32_t MCSPI_TX3;
  uint32_t MCSPI_RX3;
  uint32_t MCSPI_XFERLEVEL;
  uint32_t dummy3[8];
  uint32_t MCSPI_DAFTX;
} am335x_spi_regs;

#define BBB_SPI_TIMEOUT 1000

#define BBB_SPI_0_BUS_PATH "/dev/spi-0"

#define BBB_SPI_0_IRQ AM335X_INT_SPI0INT

typedef enum {
  SPI0,
  SPI1,
  SPI_COUNT
} bbb_spi_id_t;



typedef struct BEAGLE_SPI_BufferDescriptor_ {
    unsigned short      status;
    unsigned short      length;
    volatile void       *buffer;
} BEAGLE_SPI_BufferDescriptor_t;

typedef struct beagle_spi_softc {
  int                     initialized;
  rtems_id                task_id;
  uintptr_t               regs_base;
  rtems_vector_number     irq;
} beagle_spi_softc_t;

typedef struct {
  rtems_libi2c_bus_t  bus_desc;
  beagle_spi_softc_t softc;
} beagle_spi_desc_t;

/*
 * Initialize the driver
 *
 * Returns: o = ok or error code
 */
rtems_status_code beagle_spi_init
(
 rtems_libi2c_bus_t *bh                  /* bus specifier structure        */
);

/*
 * Receive some bytes from SPI device
 *
 * Returns: number of bytes received or (negative) error code
 */
int beagle_spi_read_bytes
(
 rtems_libi2c_bus_t *bh,                 /* bus specifier structure        */
 unsigned char *buf,                     /* buffer to store bytes          */
 int len                                 /* number of bytes to receive     */
);

/*
 * Send some bytes to SPI device
 *
 * Returns: number of bytes sent or (negative) error code
 */
int beagle_spi_write_bytes
(
 rtems_libi2c_bus_t *bh,                 /* bus specifier structure        */
 unsigned char *buf,                     /* buffer to send                 */
 int len                                 /* number of bytes to send        */
);

/*
 * Set SPI to desired baudrate/clock mode/character mode
 *
 * Returns: rtems_status_code
 */
rtems_status_code beagle_spi_set_tfr_mode
(
 rtems_libi2c_bus_t *bh,                 /* bus specifier structure        */
 const rtems_libi2c_tfr_mode_t *tfr_mode /* transfer mode info             */
);

/*
 * Perform selected ioctl function for SPI
 *
 * Returns: rtems_status_code
 */
int beagle_spi_ioctl
(
 rtems_libi2c_bus_t *bh,                 /* bus specifier structure        */
 int                 cmd,                /* ioctl command code             */
 void               *arg                 /* additional argument array      */
);

/*
 * Register SPI bus and devices
 *
 * Returns: Bus number or error code
 */
rtems_status_code bsp_register_spi
(
  const char         *bus_path,
  uintptr_t           register_base,
  rtems_vector_number irq
);

static inline rtems_status_code bbb_register_spi_0(void)
{
  return bsp_register_spi(
    BBB_SPI_0_BUS_PATH,
    AM335X_SPI0_BASE,
    BBB_SPI_0_IRQ
  );
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_BEAGLE_SPI_H */
