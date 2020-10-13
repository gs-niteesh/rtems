/*-
 * Copyright (c) 2016 Rubicon Communications, LLC (Netgate)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/intr.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/spibus/spi.h>
#include <dev/spibus/spibusvar.h>

#include <arm/ti/ti_sysc.h>
#include <arm/ti/ti_spireg.h>
#include <arm/ti/ti_spivar.h>

#include "spibus_if.h"

static void ti_spi_intr(void *);
static int ti_spi_detach(device_t);

static void
ti_spi_set_clock(struct ti_spi_softc *sc, int ch, int freq)
{
	uint32_t clkdiv, conf, div, extclk, reg;

	clkdiv = TI_SPI_GCLK / freq;
	if (clkdiv > MCSPI_EXTCLK_MSK) {
		extclk = 0;
		clkdiv = 0;
		div = 1;
		while (TI_SPI_GCLK / div > freq && clkdiv <= 0xf) {
			clkdiv++;
			div <<= 1;
		}
		conf = clkdiv << MCSPI_CONF_CLK_SHIFT;
	} else {
		extclk = clkdiv >> 4;
		clkdiv &= MCSPI_CONF_CLK_MSK;
		conf = MCSPI_CONF_CLKG | clkdiv << MCSPI_CONF_CLK_SHIFT;
	}

	reg = TI_SPI_READ(sc, MCSPI_CTRL_CH(ch));
	reg &= ~(MCSPI_CTRL_EXTCLK_MSK << MCSPI_CTRL_EXTCLK_SHIFT);
	reg |= extclk << MCSPI_CTRL_EXTCLK_SHIFT;
	TI_SPI_WRITE(sc, MCSPI_CTRL_CH(ch), reg);

	reg = TI_SPI_READ(sc, MCSPI_CONF_CH(ch));
	reg &= ~(MCSPI_CONF_CLKG | MCSPI_CONF_CLK_MSK << MCSPI_CONF_CLK_SHIFT);
	TI_SPI_WRITE(sc, MCSPI_CONF_CH(ch), reg | conf);
}

static int
ti_spi_attach(device_t dev)
{
	int err, i, rid, timeout;
	struct ti_spi_softc *sc;
	uint32_t rev;

	/* Activate the McSPI module. */
	err = ti_sysc_clock_enable(device_get_parent(dev));
	if (err) {
		device_printf(dev, "Error: failed to activate source clock\n");
		return (err);
	}

	/* Get the number of available channels. */
	if ((OF_getencprop(ofw_bus_get_node(dev), "ti,spi-num-cs",
	    &sc->sc_numcs, sizeof(sc->sc_numcs))) <= 0) {
		sc->sc_numcs = 2;
	}

	/* Setup memory and interrupt resources */

	/* Issue a softreset to the controller */
	TI_SPI_WRITE(sc, MCSPI_SYSCONFIG, MCSPI_SYSCONFIG_SOFTRESET);
	timeout = 1000;
	while (!(TI_SPI_READ(sc, MCSPI_SYSSTATUS) &
	    MCSPI_SYSSTATUS_RESETDONE)) {
		if (--timeout == 0) {
			device_printf(dev,
			    "Error: Controller reset operation timed out\n");
			ti_spi_detach(dev);
			return (ENXIO);
		}
		DELAY(100);
	}

	/* Set Master mode, single channel. */
	TI_SPI_WRITE(sc, MCSPI_MODULCTRL, MCSPI_MODULCTRL_SINGLE);

	/* Clear pending interrupts and disable interrupts. */
	TI_SPI_WRITE(sc, MCSPI_IRQENABLE, 0x0);
	TI_SPI_WRITE(sc, MCSPI_IRQSTATUS, 0xffff);

	for (i = 0; i < sc->sc_numcs; i++) {
		/*
		 * Default to SPI mode 0, CS active low, 8 bits word length and
		 * 500kHz clock.
		 */
		TI_SPI_WRITE(sc, MCSPI_CONF_CH(i),
		    MCSPI_CONF_DPE0 | MCSPI_CONF_EPOL |
		    (8 - 1) << MCSPI_CONF_WL_SHIFT);
		/* Set initial clock - 500kHz. */
		ti_spi_set_clock(sc, i, 500000);
	}
}

static int
ti_spi_fill_fifo(struct ti_spi_softc *sc)
{
	int bytes, timeout;
	struct spi_command *cmd;
	uint32_t written;
	uint8_t *data;

	cmd = sc->sc_cmd;
	bytes = min(sc->sc_len - sc->sc_written, sc->sc_fifolvl);
	while (bytes-- > 0) {
		data = (uint8_t *)cmd->tx_cmd;
		written = sc->sc_written++;
		if (written >= cmd->tx_cmd_sz) {
			data = (uint8_t *)cmd->tx_data;
			written -= cmd->tx_cmd_sz;
		}
		if (sc->sc_fifolvl == 1) {
			/* FIFO disabled. */
			timeout = 1000;
			while (--timeout > 0 && (TI_SPI_READ(sc,
			    MCSPI_STAT_CH(sc->sc_cs)) & MCSPI_STAT_TXS) == 0) {
				DELAY(100);
			}
			if (timeout == 0)
				return (-1);
		}
		TI_SPI_WRITE(sc, MCSPI_TX_CH(sc->sc_cs), data[written]);
	}

	return (0);
}

static int
ti_spi_drain_fifo(struct ti_spi_softc *sc)
{
	int bytes, timeout;
	struct spi_command *cmd;
	uint32_t read;
	uint8_t *data;

	cmd = sc->sc_cmd;
	bytes = min(sc->sc_len - sc->sc_read, sc->sc_fifolvl);
	while (bytes-- > 0) {
		data = (uint8_t *)cmd->rx_cmd;
		read = sc->sc_read++;
		if (read >= cmd->rx_cmd_sz) {
			data = (uint8_t *)cmd->rx_data;
			read -= cmd->rx_cmd_sz;
		}
		if (sc->sc_fifolvl == 1) {
			/* FIFO disabled. */
			timeout = 1000;
			while (--timeout > 0 && (TI_SPI_READ(sc,
			    MCSPI_STAT_CH(sc->sc_cs)) & MCSPI_STAT_RXS) == 0) {
				DELAY(100);
			}
			if (timeout == 0)
				return (-1);
		}
		data[read] = TI_SPI_READ(sc, MCSPI_RX_CH(sc->sc_cs));
	}

	return (0);
}

static void
ti_spi_intr(void *arg)
{
	int eow;
	struct ti_spi_softc *sc;
	uint32_t status;

	eow = 0;
	sc = (struct ti_spi_softc *)arg;
	TI_SPI_LOCK(sc);
	status = TI_SPI_READ(sc, MCSPI_IRQSTATUS);

	/*
	 * No new TX_empty or RX_full event will be asserted while the CPU has
	 * not performed the number of writes or reads defined by
	 * MCSPI_XFERLEVEL[AEL] and MCSPI_XFERLEVEL[AFL].  It is responsibility
	 * of CPU perform the right number of writes and reads.
	 */
	if (status & MCSPI_IRQ_TX0_EMPTY)
		ti_spi_fill_fifo(sc);
	if (status & MCSPI_IRQ_RX0_FULL)
		ti_spi_drain_fifo(sc);

	if (status & MCSPI_IRQ_EOW)
		eow = 1;
		
	/* Clear interrupt status. */
	TI_SPI_WRITE(sc, MCSPI_IRQSTATUS, status);

	/* Check for end of transfer. */
	if (sc->sc_written == sc->sc_len && sc->sc_read == sc->sc_len) {
		sc->sc_flags |= TI_SPI_DONE;
		wakeup(sc->sc_dev);
	}

	TI_SPI_UNLOCK(sc);
}

static int
ti_spi_pio_transfer(struct ti_spi_softc *sc)
{

	while (sc->sc_len - sc->sc_written > 0) {
		if (ti_spi_fill_fifo(sc) == -1)
			return (EIO);
		if (ti_spi_drain_fifo(sc) == -1)
			return (EIO);
	}

	return (0);
}

static int
ti_spi_gcd(int a, int b)
{
	int m;

	while ((m = a % b) != 0) {
		a = b;
		b = m;
	}

	return (b);
}

static int
ti_spi_transfer(device_t dev, device_t child, struct spi_command *cmd)
{
	int err;
	struct ti_spi_softc *sc;
	uint32_t clockhz, cs, mode, reg;

	sc = device_get_softc(dev);

	KASSERT(cmd->tx_cmd_sz == cmd->rx_cmd_sz, 
	    ("TX/RX command sizes should be equal"));
	KASSERT(cmd->tx_data_sz == cmd->rx_data_sz, 
	    ("TX/RX data sizes should be equal"));

	/* Get the proper chip select for this child. */
	spibus_get_cs(child, &cs);
	spibus_get_clock(child, &clockhz);
	spibus_get_mode(child, &mode);

	cs &= ~SPIBUS_CS_HIGH;

	if (cs > sc->sc_numcs) {
		device_printf(dev, "Invalid chip select %d requested by %s\n",
		    cs, device_get_nameunit(child));
		return (EINVAL);
	}

	if (mode > 3)
	{
	    device_printf(dev, "Invalid mode %d requested by %s\n", mode,
		    device_get_nameunit(child));
	    return (EINVAL);
	}

	TI_SPI_LOCK(sc);

	/* If the controller is in use wait until it is available. */
	while (sc->sc_flags & TI_SPI_BUSY)
		mtx_sleep(dev, &sc->sc_mtx, 0, "ti_spi", 0);

	/* Now we have control over SPI controller. */
	sc->sc_flags = TI_SPI_BUSY;

	/* Save the SPI command data. */
	sc->sc_cs = cs;
	sc->sc_cmd = cmd;
	sc->sc_read = 0;
	sc->sc_written = 0;
	sc->sc_len = cmd->tx_cmd_sz + cmd->tx_data_sz;
	sc->sc_fifolvl = ti_spi_gcd(sc->sc_len, TI_SPI_FIFOSZ);
	if (sc->sc_fifolvl < 2 || sc->sc_len > 0xffff)
		sc->sc_fifolvl = 1;	/* FIFO disabled. */
	/* Disable FIFO for now. */
	sc->sc_fifolvl = 1;

	/* Set the bus frequency. */
	ti_spi_set_clock(sc, sc->sc_cs, clockhz);

	/* Disable the FIFO. */
	TI_SPI_WRITE(sc, MCSPI_XFERLEVEL, 0);

	/* 8 bits word, d0 miso, d1 mosi, mode 0 and CS active low. */
	reg = TI_SPI_READ(sc, MCSPI_CONF_CH(sc->sc_cs));
	reg &= ~(MCSPI_CONF_FFER | MCSPI_CONF_FFEW | MCSPI_CONF_SBPOL |
	    MCSPI_CONF_SBE | MCSPI_CONF_TURBO | MCSPI_CONF_IS |
	    MCSPI_CONF_DPE1 | MCSPI_CONF_DPE0 | MCSPI_CONF_DMAR |
	    MCSPI_CONF_DMAW | MCSPI_CONF_EPOL);
	reg |= MCSPI_CONF_DPE0 | MCSPI_CONF_EPOL | MCSPI_CONF_WL8BITS;
	reg |= mode; /* POL and PHA are the low bits, we can just OR-in mode */
	TI_SPI_WRITE(sc, MCSPI_CONF_CH(sc->sc_cs), reg);

#if 0
	/* Enable channel interrupts. */
	reg = TI_SPI_READ(sc, MCSPI_IRQENABLE);
	reg |= 0xf;
	TI_SPI_WRITE(sc, MCSPI_IRQENABLE, reg);
#endif

	/* Start the transfer. */
	reg = TI_SPI_READ(sc, MCSPI_CTRL_CH(sc->sc_cs));
	TI_SPI_WRITE(sc, MCSPI_CTRL_CH(sc->sc_cs), reg | MCSPI_CTRL_ENABLE);

	/* Force CS on. */
	reg = TI_SPI_READ(sc, MCSPI_CONF_CH(sc->sc_cs));
	TI_SPI_WRITE(sc, MCSPI_CONF_CH(sc->sc_cs), reg |= MCSPI_CONF_FORCE);

	err = 0;
	if (sc->sc_fifolvl == 1)
		err = ti_spi_pio_transfer(sc);

	/* Force CS off. */
	reg = TI_SPI_READ(sc, MCSPI_CONF_CH(sc->sc_cs));
	reg &= ~MCSPI_CONF_FORCE;
	TI_SPI_WRITE(sc, MCSPI_CONF_CH(sc->sc_cs), reg);

	/* Disable IRQs. */
	reg = TI_SPI_READ(sc, MCSPI_IRQENABLE);
	reg &= ~0xf;
	TI_SPI_WRITE(sc, MCSPI_IRQENABLE, reg);
	TI_SPI_WRITE(sc, MCSPI_IRQSTATUS, 0xf);

	/* Disable the SPI channel. */
	reg = TI_SPI_READ(sc, MCSPI_CTRL_CH(sc->sc_cs));
	reg &= ~MCSPI_CTRL_ENABLE;
	TI_SPI_WRITE(sc, MCSPI_CTRL_CH(sc->sc_cs), reg);

	/* Disable FIFO. */
	reg = TI_SPI_READ(sc, MCSPI_CONF_CH(sc->sc_cs));
	reg &= ~(MCSPI_CONF_FFER | MCSPI_CONF_FFEW);
	TI_SPI_WRITE(sc, MCSPI_CONF_CH(sc->sc_cs), reg);

	/* Release the controller and wakeup the next thread waiting for it. */
	sc->sc_flags = 0;
	wakeup_one(dev);
	TI_SPI_UNLOCK(sc);

	return (err);
}
