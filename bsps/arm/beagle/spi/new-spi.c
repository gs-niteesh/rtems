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
  uint32_t speed_hz;
  uint8_t bits_per_word;
  uint32_t mode;
} am335x_spi_bus;

static void am335x_spi_clk_config(phandle_t node)
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
  reg->MCSPI_SYST &= ~AM335X_SPI_SYST_SSB;
  reg->MCSPI_IRQSTATUS = irqs;
}

static void am335x_spi_reset(am335x_spi_bus *bus)
{
  am335x_spi_regs *regs;
  int timeout = BBB_SPI_TIMEOUT;

  regs = (am335x_spi_regs *)bus->regs;

  regs->MCSPI_SYSCONFIG |= AM335X_SPI_SYSCONFIG_SOFTRESET;

  while (
    (regs->MCSPI_SYSSTATUS & AM335X_SPI_SYSSTATUS_RESETDONE) == 0
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

  reg = bus->regs->MCSPI_CH0CTRL;
  reg &= ~(MCSPI_CTRL_EXTCLK_MSK << MCSPI_CTRL_EXTCLK_SHIFT);
  reg |= extclk << MCSPI_CTRL_EXTCLK_SHIFT;
  bus->regs->MCSPI_CH0CTRL = reg;

  reg = bus->regs->MCSPI_CH0CONF;
  reg &= ~(MCSPI_CONF_CLKG | MCSPI_CONF_CLK_MSK << MCSPI_CONF_CLK_SHIFT);
  bus->regs->MCSPI_CH0CONF = reg | conf;
}

static int am335x_spi_transfer(
  spi_bus *base,
  const spi_ioc_transfer *msgs,
  uint32_t msg_count
)
{
  am335x_spi_bus *bus;
  int rv;

  bus = (am335x_spi_bus *) base;
  (void)bus;
  (void)rv;

  return -1;
}

static int am335x_spi_configure(
  am335x_spi_bus *bus,
  uint32_t speed_hz,
  uint32_t mode,
  uint8_t bits_per_word,
  uint8_t cs
)
{
  volatile am335x_spi_regs *regs = bus->regs;

  if ( speed_hz > AM335X_SPI_MAX_SPEED || bits_per_word > 32 ) {
    return -EINVAL;
  }

  (void) regs;
  (void) cs;
  (void) bits_per_word;
  (void) mode;
  (void) speed_hz;




  return -1;
}

static int am335x_spi_setup(spi_bus *base)
{
  am335x_spi_bus *bus;

  bus = (am335x_spi_bus *)base;

  am335x_spi_configure(
    bus,
    bus->speed_hz,
    bus->mode,
    bus->bits_per_word,
    bus->cs
  );

  return 0;
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

  while(((tmp = regs->MCSPI_IRQSTATUS) & handled_irqs) != 0) {
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

static int am335x_spi_init(
  am335x_spi_bus *bus,
  phandle_t node
)
{
  rtems_status_code sc;
  volatile am335x_spi_regs *regs;

  regs = bus->regs;

  am335x_spi_clk_config(node);
  am335x_spi_reset(bus);

  /*
   * Reference: AM335x TRM, beagle old SPI driver
   *
   * TODO: Allow to configure other channels
   */
  regs->MCSPI_MODULCTRL &= ~AM335X_SPI_MODULCTRL_PIN34;
  regs->MCSPI_MODULCTRL &= ~AM335X_SPI_MODULCTRL_MS;
  regs->MCSPI_MODULCTRL |= AM335X_SPI_MODULCTRL_SINGLE;

  regs->MCSPI_CH0CONF &= ~AM335X_SPI_CH0CONF_TRM_MASK;
  regs->MCSPI_CH0CONF |= AM335X_SPI_CH0CONF_DPE0;
  regs->MCSPI_CH0CONF &= ~AM335X_SPI_CH0CONF_DPE1;
  regs->MCSPI_CH0CONF &= ~AM335X_SPI_CH0CONF_IS;

  regs->MCSPI_CH0CONF |= AM335X_SPI_CH0CONF_WL(8 - 1);

  regs->MCSPI_CH0CONF &= ~AM335X_SPI_CH0CONF_PHA;
  regs->MCSPI_CH0CONF &= ~AM335X_SPI_CH0CONF_POL;

  regs->MCSPI_CH0CONF |= AM335X_SPI_CH0CONF_EPOL;
  regs->MCSPI_CH0CONF |= AM335X_SPI_CH0CONF_CLKD(0x1);

  sc = rtems_interrupt_handler_install(
    bus->irq,
    "SPI",
    RTEMS_INTERRUPT_UNIQUE,
    am335x_spi_interrupt,
    bus
  );
  if (sc != RTEMS_SUCCESSFUL) {
    return EAGAIN;
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