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

#define AM335X_SPI_FIFO_SIZE       1
#define AM335X_SPI_MAX_CHIPSELECTS 2

typedef struct am335x_spi_bus am335x_spi_bus;

struct am335x_spi_bus {
  spi_bus base;
  volatile am335x_spi_regs *regs;
  uint32_t speed_hz;
  uint32_t mode;
  uint8_t bits_per_word;
  uint8_t cs;
  uint32_t msg_todo;
  uint32_t todo;
  uint32_t in_transfer;
  const uint8_t *tx_buf;
  uint8_t *rx_buf;
  const spi_ioc_transfer *msg;
  void (*push)(am335x_spi_bus*, volatile am335x_spi_regs *);
  void (*pop)(am335x_spi_bus*, volatile am335x_spi_regs *);
  rtems_vector_number irq;
  rtems_id taskid;
};

static bool am335x_spi_is_rx_fifo_not_empty(volatile am335x_spi_regs *regs)
{
  return (regs->IRQSTATUS & AM335X_SPI_IRQSTATUS_RX0_FULL);
}

static void am335x_spi_clk_enable(phandle_t node)
{
  clk_ident_t spi_clk;

  spi_clk = ti_hwmods_get_clock(node);
  ti_prcm_clk_enable(spi_clk);
}

static void am335x_spi_done(am335x_spi_bus *bus)
{
  rtems_event_transient_send(bus->taskid);
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

#define AM335X_SPI_PUSH(type) \
static void am335x_spi_push_##type(am335x_spi_bus *bus, volatile am335x_spi_regs *regs) \
{ \
  type val = 0; \
  if (bus->tx_buf != NULL) { \
    val = *(type *)bus->tx_buf; \
    bus->tx_buf += sizeof(type); \
  } \
  bus->todo -= sizeof(type); \
  regs->TX0 = val; \
}

#define AM335X_SPI_POP(type) \
static void am335x_spi_pop_##type(am335x_spi_bus *bus, volatile am335x_spi_regs *regs) \
{ \
  uint32_t val = regs->RX0; \
  if (bus->rx_buf != NULL) { \
    *(type *)bus->rx_buf = val; \
    bus->rx_buf += sizeof(type); \
  } \
}

AM335X_SPI_PUSH(uint8_t)
AM335X_SPI_POP(uint8_t)
AM335X_SPI_PUSH(uint16_t)
AM335X_SPI_POP(uint16_t)
AM335X_SPI_PUSH(uint32_t)
AM335X_SPI_POP(uint32_t)

static void am335x_spi_set_push_pop(
  am335x_spi_bus *bus,
  uint32_t len,
  uint8_t bits_per_word
)
{
  if (bits_per_word <= 8) {
    bus->push = am335x_spi_push_uint8_t;
    bus->pop = am335x_spi_pop_uint8_t;
  } else if (bits_per_word <= 16) {
    bus->push = am335x_spi_push_uint16_t;
    bus->pop = am335x_spi_pop_uint16_t;
  } else {
    bus->push = am335x_spi_push_uint32_t;
    bus->pop = am335x_spi_pop_uint32_t;
  }
}

static void am335x_spi_push(
  am335x_spi_bus *bus,
  volatile am335x_spi_regs *regs
)
{
  while (bus->todo > 0 && bus->in_transfer < AM335X_SPI_FIFO_SIZE) {
    (*bus->push)(bus, regs);
    ++bus->in_transfer;
  }
}

static void
am335x_spi_set_clock(am335x_spi_bus *bus, int freq)
{
  uint32_t clkdiv;
  uint32_t conf;
  uint32_t div;
  uint32_t extclk;
  volatile uint32_t *reg;

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

  reg = AM335X_SPI_CH_REG(bus->regs, bus->cs, AM335X_SPI_CH_CTRL);
  *reg &= ~(MCSPI_CTRL_EXTCLK_MSK << MCSPI_CTRL_EXTCLK_SHIFT);
  *reg |= extclk << MCSPI_CTRL_EXTCLK_SHIFT;

  reg = AM335X_SPI_CH_REG(bus->regs, bus->cs, AM335X_SPI_CH_CONF);
  *reg &= ~(MCSPI_CONF_CLKG | MCSPI_CONF_CLK_MSK << MCSPI_CONF_CLK_SHIFT);
  *reg |= conf;
}

static void am335x_spi_enable_channel(
  am335x_spi_bus *bus,
  uint8_t cs
)
{
  volatile uint32_t *ch_conf;
  volatile uint32_t *ch_ctrl;

  // ch_ctrl = AM335X_SPI_CH_REG(bus->regs, cs, AM335X_SPI_CH_CTRL);
  ch_conf = AM335X_SPI_CH_REG(bus->regs, cs, AM335X_SPI_CH_CONF);

  /* TODO: Fix assignement prob */
  ch_ctrl = (volatile uint32_t *)(0x48030000 + 0x134);
  *ch_ctrl |= AM335X_SPI_CH0CTRL_EN;
  *ch_conf |= AM335X_SPI_CH0CONF_FORCE;
}

static void am335x_spi_disable_channel(
  am335x_spi_bus *bus,
  uint8_t cs
)
{
  volatile uint32_t *ch_conf;
  volatile uint32_t *ch_ctrl;

  ch_conf = AM335X_SPI_CH_REG(bus->regs, cs, AM335X_SPI_CH_CONF);
  /* TODO: Fix assignement prob */
  ch_ctrl = (volatile uint32_t *)(0x48030000 + 0x134);

  *ch_conf &= ~AM335X_SPI_CH0CONF_FORCE;
  *ch_ctrl &= ~AM335X_SPI_CH0CTRL_EN;
}

static int am335x_spi_configure(
  am335x_spi_bus *bus,
  uint32_t speed_hz,
  uint8_t bits_per_word,
  uint32_t mode,
  uint8_t cs
);

static void am335x_spi_next_msg(
  am335x_spi_bus *bus,
  volatile am335x_spi_regs *regs
)
{
  if (bus->msg_todo > 0) {
    const spi_ioc_transfer *msg;

    msg = bus->msg;

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
    if ((msg->mode & SPI_NO_CS) != 0) {
      /* TODO */
      printk("unimplemented: msg->mode & SPI_NO_CS\n");
    } else {
      am335x_spi_enable_channel(bus, msg->cs);
    }

    bus->todo = msg->len;
    bus->rx_buf = msg->rx_buf;
    bus->tx_buf = msg->tx_buf;
    am335x_spi_set_push_pop(bus, msg->len, msg->bits_per_word);
    am335x_spi_push(bus, regs);
    /* TODO: Use the correct channel intr values */
    bus->regs->IRQENABLE = AM335X_SPI_IRQENABLE_RX0_FULL | AM335X_SPI_IRQENABLE_TX0_EMPTY;
  } else {
    /* Clear the interrupts */
    bus->regs->SYST &= ~AM335X_SPI_SYST_SSB;
    bus->regs->IRQSTATUS = AM335X_SPI_IRQENABLE_RX0_FULL | AM335X_SPI_IRQENABLE_TX0_EMPTY;
    am335x_spi_done(bus);
  }
}

static void am335x_spi_interrupt(void *arg)
{
  am335x_spi_bus *bus;
  volatile am335x_spi_regs *regs;

  bus = arg;
  regs = bus->regs;

  while (am335x_spi_is_rx_fifo_not_empty(regs) && bus->in_transfer > 0) {
    (*bus->pop)(bus, regs);
    --bus->in_transfer;
  }

  if (bus->todo > 0) {
    am335x_spi_push(bus, regs);
  } else if (bus->in_transfer > 0) {
    regs->IRQENABLE = AM335X_SPI_IRQENABLE_RX0_FULL;
  } else {
    --bus->msg_todo;
    ++bus->msg;
    am335x_spi_next_msg(bus, regs);
  }
}

static int am335x_spi_check_messages(
  am335x_spi_bus *bus,
  const spi_ioc_transfer *msg,
  uint32_t count
)
{
  while (count > 0) {
    if (msg->delay_usecs != 0) {
      return -EINVAL;
    }
    if (msg->bits_per_word > 32) {
      return -EINVAL;
    }
    if ((msg->mode &
        ~(SPI_CPHA | SPI_CPOL | SPI_LOOP | SPI_NO_CS)) != 0) {
      return -EINVAL;
    }
    if ((msg->mode & SPI_NO_CS) == 0 &&
        (msg->cs > AM335X_SPI_MAX_CHIPSELECTS)) {
      return -EINVAL;
    }
    if (msg->cs_change != 0) {
      return -EINVAL;
    }

    ++msg;
    --count;
  }

  return 0;
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

  rv = am335x_spi_check_messages(bus, msgs, msg_count);

  if (rv != 0) {
    printk("Error checking message\n");
    return rv;
  }

  bus->msg_todo = msg_count;
  bus->msg = &msgs[0];
  bus->taskid = rtems_task_self();

  am335x_spi_next_msg(bus, bus->regs);
  rtems_event_transient_receive(RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  am335x_spi_disable_channel(bus, bus->cs);

  return rv;
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
    printk("spi_configure: invalid msg\n");
    return -EINVAL;
  }

  /*
   * Enable transmit/receive mode.
   * Set SPIDAT[0] as MISO and SPIDAT[1] as MOSI.
   */
  ch_conf = AM335X_SPI_CH_REG(regs, cs, AM335X_SPI_CH_CONF);
  *ch_conf &= ~AM335X_SPI_CH0CONF_TRM_MASK;
  *ch_conf |= AM335X_SPI_CH0CONF_DPE0;
  *ch_conf &= ~AM335X_SPI_CH0CONF_DPE1;
  *ch_conf &= ~AM335X_SPI_CH0CONF_IS;

  /* Set no of bits per data frame */
  *ch_conf |= AM335X_SPI_CH0CONF_WL(bits_per_word - 1);

  /* Set SPI clock speed */
  am335x_spi_set_clock(bus, speed_hz);

  /* Configure SPI mode */
  if ((mode & SPI_CPOL)) {
    *ch_conf |= AM335X_SPI_CH0CONF_POL;
  } else {
    *ch_conf &= ~AM335X_SPI_CH0CONF_POL;
  }

  if ((mode & SPI_CPHA)) {
    *ch_conf |= AM335X_SPI_CH0CONF_PHA;
  } else {
    *ch_conf &= ~AM335X_SPI_CH0CONF_PHA;
  }

  /* Configure CS as active HIGH/LOW */
  if ((mode & SPI_CS_HIGH)) {
    *ch_conf &= ~AM335X_SPI_CH0CONF_EPOL;
  } else {
    *ch_conf |= AM335X_SPI_CH0CONF_EPOL;
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
    printk("error installing handler\n");
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
    printk("error allocating bus\n");
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


int spi_register_0() {
  phandle_t node;
  
  node = rtems_ofw_find_device("/ocp/spi@48030000");
  return am335x_spi_bus_register(node);
}