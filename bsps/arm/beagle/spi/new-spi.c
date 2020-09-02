#include <rtems/bspIo.h>
#include <rtems/irq-extension.h>
#include <rtems/score/assert.h>
#include <dev/spi/spi.h>
#include <libcpu/am335x.h>
#include <bsp.h>
#include <bsp/spi.h>
#include <ofw/ofw.h>
#include <arm/ti/ti_hwmods.h>
#include <errno.h>

#define EVENT_TXEMPTY     RTEMS_EVENT_0
#define EVENT_RXFULL      RTEMS_EVENT_1

typedef struct {
  spi_bus base;
  volatile am335x_spi_regs *regs;
  rtems_vector_number irq;
  rtems_id taskid;
  uint8_t cs;
  uint8_t channel;
  uint8_t bits_per_word;
  uint32_t speed_hz;
  uint32_t mode;

  const spi_ioc_transfer *msg;
  uint32_t todo;
  uint32_t in_tranfer;
  const uint8_t *tx_buf;
  uint8_t *rx_buf;
} am335x_spi_bus;

static void am335x_spi_clk_enable(phandle_t node)
{
  clk_ident_t spi_clk;

  spi_clk = ti_hwmods_get_clock(node);
  ti_prcm_clk_enable(spi_clk);
}

static inline void am335x_spi_clear_irqstatus(
  volatile am335x_spi_regs *reg,
  uint32_t irqs
)
{
  reg->SYST &= ~AM335X_SPI_SYST_SSB;
  reg->IRQSTATUS = irqs;
}

static void am335x_spi_reset(am335x_spi_bus *bus)
{
  am335x_spi_regs *regs;
  int timeout = BBB_SPI_TIMEOUT;

  regs = (am335x_spi_regs *)bus->regs;

  regs->SYSCONFIG |= AM335X_SPI_SYSCONFIG_SOFTRESET;

  while (
    (regs->SYSSTATUS & AM335X_SPI_SYSSTATUS_RESETDONE) == 0
    && timeout--
  ) {
    if (timeout <= 0) {
      printk("ERROR: Timeout in soft-reset\n");
      return;
    }

    udelay(1000);
  }
}

static void
am335x_spi_set_clock(am335x_spi_bus *bus, int freq)
{
  uint32_t clkdiv;
  uint32_t conf;
  uint32_t div;
  uint32_t extclk;
  uint32_t reg;

  clkdiv = AM335X_SPI_MAX_SPEED / freq;
  if (clkdiv > MCSPI_EXTCLK_MSK) {
    extclk = 0;
    clkdiv = 0;
    div = 1;
    while (AM335X_SPI_MAX_SPEED / div > freq && clkdiv <= 0xf) {
      clkdiv++;
      div <<= 1;
    }
    conf = clkdiv << MCSPI_CONF_CLK_SHIFT;
  } else {
    extclk = clkdiv >> 4;
    clkdiv &= MCSPI_CONF_CLK_MSK;
    conf = MCSPI_CONF_CLKG | clkdiv << MCSPI_CONF_CLK_SHIFT;
  }

  reg = bus->ctrl_reg;
  reg &= ~(MCSPI_CTRL_EXTCLK_MSK << MCSPI_CTRL_EXTCLK_SHIFT);
  reg |= extclk << MCSPI_CTRL_EXTCLK_SHIFT;
  bus->ctrl_reg = reg;

  reg = bus->conf_reg;
  reg &= ~(MCSPI_CONF_CLKG | MCSPI_CONF_CLK_MSK << MCSPI_CONF_CLK_SHIFT);
  bus->conf_reg = reg | conf;
}

static void am335x_spi_to_buffer(am335x_spi_bus *bus)
{
  volatile am335x_spi_regs *regs;
  const spi_ioc_transfer *msg;

  regs = bus->regs;
  msg = bus->msg;

  if (bus->msg_todo > 0) {

    if (
      msg->speed_hz != bus->speed_hz
        || msg->bits_per_word != bus->bits_per_word
        || msg->mode != bus->mode
        || msg->cs != bus->cs
    ) {
      am335x_spi_configure(
        bus,
        msg->speed_hz,
        msg->bits_per_word,
        msg->mode,
        msg->cs
      );
    }

    bus->todo = msg->len;
    bus->rx_buf = msg->rx_buf;
    bus->tx_buf = msg->tx_buf;


  } else {
    /* clear interrupts */
    /* send event */
  }
}

static int am335x_spi_transfer(
  spi_bus *base,
  const spi_ioc_transfer *msgs,
  uint32_t msg_count
)
{
  am335x_spi_bus *bus;
  int rv = 0;

  bus = (am335x_spi_bus *) base;

  bus->todo = msg_count;
  bus->msg = &msgs[0];
  bus->taskid = rtems_task_self();

  am335x_spi_to_buffer(bus);
  rtems_event_transient_receive(RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  am335x_spi_disable_channel(bus);

  return rv;
}

static void am335x_spi_interrupt( void *arg )
{
  am335x_spi_bus *bus = arg;
  volatile am335x_spi_regs *regs = bus->regs;
  uint32_t irq = 0;
  uint32_t events = 0;
  uint32_t tmp;
  rtems_status_code sc;
  const uint32_t handled_irqs = AM335X_SPI_IRQSTATUS_TX0_EMPTY | AM335X_SPI_IRQSTATUS_RX0_FULL;

  while(((tmp = regs->IRQSTATUS) & handled_irqs) != 0) {
    irq |= tmp;
    am335x_spi_clear_irqstatus(regs, tmp);
  }

  if (irq & AM335X_SPI_IRQSTATUS_TX0_EMPTY) {
    events |= EVENT_TXEMPTY;
  }

  if (irq & AM335X_SPI_IRQSTATUS_RX0_FULL) {
    events |= EVENT_RXFULL;
  }

  sc = rtems_event_send(bus->taskid, events);
  _Assert( sc == RTEMS_SUCCESSFUL );
  (void) sc; /* suppress warning in case of no assert */
}


static int am335x_spi_configure(
  am335x_spi_bus *bus,
  uint32_t speed_hz,
  uint8_t bits_per_word,
  uint32_t mode,
  uint8_t cs
)
{
  volatile am335x_spi_regs *regs = bus->regs;
  volatile uint32_t *ch_conf;

  if ( speed_hz > AM335X_SPI_MAX_SPEED || bits_per_word > 32 || cs > 1 ) {
    return -EINVAL;
  }

  /*
   * Enable transmit/receive mode.
   * Set SPIDAT[0] as MISO and SPIDAT[1] as MOSI.
   */
  ch_conf = (volatile uint32_t *)AM335X_SPI_CH_REG(regs, cs, AM335X_SPI_CH_CONF);
  ch_conf &= ~AM335X_SPI_CH0CONF_TRM_MASK;
  ch_conf |= AM335X_SPI_CH0CONF_DPE0;
  ch_conf &= ~AM335X_SPI_CH0CONF_DPE1;
  ch_conf &= ~AM335X_SPI_CH0CONF_IS;

  /* Set no of bits per data frame */
  ch_conf |= AM335X_SPI_CH0CONF_WL(bits_per_word - 1);

  /* Set SPI clock speed */
  am335x_spi_set_clock(bus, speed_hz);

  /* Configure SPI mode */
  if ((mode & SPI_CPOL) != 0) {
    ch_conf |= AM335X_SPI_CH0CONF_POL;
  } else {
    ch_conf &= ~AM335X_SPI_CH0CONF_POL;
  }

  if ((mode & SPI_CPHA) != 0) {
    ch_conf |= AM335X_SPI_CH0CONF_PHA;
  } else {
    ch_conf &= ~AM335X_SPI_CH0CONF_PHA;
  }

  /* Configure CS as active HIGH/LOW */
  if ((mode & SPI_CS_HIGH) != 0) {
    ch_conf &= ~AM335X_SPI_CH0CONF_EPOL;
  } else {
    ch_conf |= AM335X_SPI_CH0CONF_EPOL;
  }

  bus->speed_hz = speed_hz;
  bus->mode = mode;
  bus->bits_per_word = bits_per_word;
  bus->cs = cs;

  return 0;
}

static int am335x_spi_setup(spi_bus *base)
{
  am335x_spi_bus *bus;

  bus = (am335x_spi_bus *)base;

  am335x_spi_configure(
    bus,
    bus->speed_hz,
    bus->bits_per_word,
    bus->mode,
    bus->cs
  );

  return 0;
}

static int am335x_spi_init(
  am335x_spi_bus *bus,
  phandle_t node
)
{
  rtems_status_code sc;
  volatile am335x_spi_regs *regs;

  regs = bus->regs;

  am335x_spi_clk_enable(node);
  am335x_spi_reset(bus);

  /*
   * Enable master mode, slave select, single channel
   */
  regs->MODULCTRL &= ~AM335X_SPI_MODULCTRL_MS;
  regs->MODULCTRL &= ~AM335X_SPI_MODULCTRL_PIN34;
  regs->MODULCTRL |= AM335X_SPI_MODULCTRL_SINGLE;

  /* Clear and disable interrupts */
  regs->IRQENABLE = 0x0;
  regs->IRQSTATUS = ~0x0;

  am335x_spi_configure(
    bus,
    bus->base.max_speed_hz,
    bus->base.bits_per_word,
    0,
    0
  );

  sc = rtems_interrupt_handler_install(
    bus->irq,
    "SPI",
    RTEMS_INTERRUPT_UNIQUE,
    am335x_spi_interrupt,
    bus
  );
  if (sc != RTEMS_SUCCESSFUL) {
    return -EAGAIN;
  }

  return 0;
}

static void am335x_spi_destroy( spi_bus *base )
{
  am335x_spi_bus *bus;

  bus = (am335x_spi_bus *) base;
  rtems_interrupt_handler_remove(bus->irq, am335x_spi_interrupt, bus);
  spi_bus_destroy_and_free(&bus->base);
}

static int am335x_spi_bus_register(
  phandle_t node
)
{
  am335x_spi_bus *bus;
  rtems_ofw_memory_area reg;
  rtems_vector_number irq;
  const char *bus_path = "/dev/spi-0"; // Change
  int rv;

  bus = (am335x_spi_bus *)spi_bus_alloc_and_init(sizeof(*bus));
  if (bus == NULL) {
    return -1;
  }

  bus->taskid = rtems_task_self();

  rv = rtems_ofw_get_reg(node, &reg, sizeof(reg));
  if (rv <= 0) {
    return EINVAL;
  }

  rv = rtems_ofw_get_interrupts(node, &irq, sizeof(irq));
  if (rv <= 0) {
    return EINVAL;
  }

  bus->regs = (volatile am335x_spi_regs *)reg.start;
  bus->base.max_speed_hz = AM335X_SPI_MAX_SPEED;
  bus->base.bits_per_word = 8;
  bus->irq = irq;

  rv = am335x_spi_init(bus, node);
  if (rv != 0) {
    (*bus->base.destroy)(&bus->base);
    rtems_set_errno_and_return_minus_one(rv);
  }

  bus->base.setup = am335x_spi_setup;
  bus->base.transfer = am335x_spi_transfer;
  bus->base.destroy = am335x_spi_destroy;

  return spi_bus_register( &bus->base, bus_path );
}