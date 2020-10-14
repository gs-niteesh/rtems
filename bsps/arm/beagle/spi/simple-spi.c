#include <rtems/bspIo.h>
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
#define CLEAR(reg, bit) ((reg) &= ~(bit))
#define SET(reg, bit) ((reg) |= (bit))

typedef struct am335x_spi_bus am335x_spi_bus;

struct am335x_spi_bus {
  spi_bus base;
  volatile am335x_spi_regs *regs;
  uint32_t speed_hz; /* Unused */
  uint32_t mode; /* Unused */
  uint8_t bits_per_word; /* Unused */
  uint8_t cs; /* Unused */
  const spi_ioc_transfer *msg;
  rtems_id taskid;
};

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
      printk("SPI Error: Timeout in soft-reset\n");
      return;
    }
    udelay(1000);
  }
  printk("reset finished\n");
}

static int am335x_spi_transfer(
  spi_bus *base,
  const spi_ioc_transfer *msgs,
  uint32_t msg_count
)
{
  am335x_spi_bus *bus;
  volatile am335x_spi_regs *regs;

  bus = (am335x_spi_bus *) base;
  regs = bus->regs;

  SET(regs->CH0CTRL, 1 << 0); /* Enable the channel */
  SET(regs->CH0CONF, AM335X_SPI_CH0CONF_FORCE); /* Enable SPI Line */

  for (int i = 0; i < msgs->len; i++) {
    /*
     * Should I wait for the tranmission to get complete?
     */
    int timeout = 1000;
    while (--timeout > 0 && ((regs->CH0STAT & AM335X_SPI_CH0STAT_TXS) == 0)) {
        udelay(10);
    }
    if (timeout == 0)
      printk("push timeout\n");

    /*
     * NOTE: The first byte is pushed properly but it timeouts on second bit
     */
    printk("pushing: %x\n", ((uint8_t *)msgs->tx_buf)[i]);
    regs->TX0 = ((uint8_t *)msgs->tx_buf)[i];

    timeout = 1000;
    while (--timeout > 0 && ((regs->CH0STAT & AM335X_SPI_CH0STAT_RXS) == 0) {
      udelay(10);
    }
    if (timeout == 0)
      printk("pull timeout\n");
    ((uint8_t *)msgs->rx_buf)[i] = regs->RX0;
  }
  CLEAR(regs->CH0CONF, AM335X_SPI_CH0CONF_FORCE); /* Disable SPI Line*/
  CLEAR(regs->CH0CTRL, 1 << 0); /* Disable the channel */

  return 0;
}

static int am335x_spi_init(am335x_spi_bus *bus);

static int am335x_spi_setup(spi_bus *base)
{
  /* Single channel master mode 4pin SPI */
  /* 2Mhz clock */
  return am335x_spi_init((am335x_spi_bus *) base);
}

static void am335x_spi_destroy( spi_bus *base )
{
  am335x_spi_bus *bus;

  bus = (am335x_spi_bus *) base;
  spi_bus_destroy_and_free(&bus->base);
}

static void am335x_spi_clock_enable(void)
{
  /* Writing to MODULEMODE field of AM335X_CM_PER_SPI0_CLKCTRL register. */
  REG( AM335X_CM_PER_ADDR + AM335X_CM_PER_SPI0_CLKCTRL ) |=
    AM335X_CM_PER_SPI0_CLKCTRL_MODULEMODE_ENABLE;

  /* Waiting for MODULEMODE field to reflect the written value. */
  while ( AM335X_CM_PER_SPI0_CLKCTRL_MODULEMODE_ENABLE !=
          ( REG( AM335X_CM_PER_ADDR + AM335X_CM_PER_SPI0_CLKCTRL ) &
            AM335X_CM_PER_SPI0_CLKCTRL_MODULEMODE ) )
    continue;

  /*
   * Waiting for IDLEST field in AM335X_CM_PER_SPI0_CLKCTRL
   * register to attain desired value.
   */
  while ( ( AM335X_CM_PER_CONTROL_CLKCTRL_IDLEST_FUNC <<
            AM335X_CM_PER_CONTROL_CLKCTRL_IDLEST_SHIFT ) !=
          ( REG( AM335X_CM_PER_ADDR + AM335X_CM_PER_SPI0_CLKCTRL ) &
            AM335X_CM_PER_CONTROL_CLKCTRL_IDLEST ) )
    continue;
  printk("SPI clock enable finish\n");
}

/*
 * Reference ti mcspi driver
 * freq in Mhz
 */
static void am335x_spi_clock_setup(am335x_spi_bus *bus, uint32_t freq)
{
  unsigned int fRatio = 0;
  unsigned int extClk = 0;
  unsigned int clkD = 0;
  volatile am335x_spi_regs *regs = bus->regs;

  /* Calculate the value of fRatio. */
  fRatio = (48000000 / freq);

  /* checking for power of 2 */
  if(0 != (fRatio & (fRatio - 1))) 
    {
        /* Set the clock granularity to 1 clock cycle.*/
        regs->CH0CONF |= 1 << 29;

        /* Calculate the ratios clkD and extClk based on fClk */
        extClk = (fRatio - 1) >> 4;
        clkD = (fRatio - 1) & 0xf;
        /*Clear the extClk field of MCSPI_CHCTRL register.*/
        regs->CH0CTRL &= ~0x0000FF00u;
        /* Set the extClk field of MCSPI_CHCTRL register.*/
        regs->CH0CTRL |= (extClk << 0x00000008u);
    }
    else
    {
        /* Clock granularity of power of 2.*/
        regs->CH0CONF &= ~0x20000000u;

        while(1 != fRatio)
        {
            fRatio /= 2;
            clkD++;
        }
    }
    /*Clearing the clkD field of MCSPI_CHCONF register.*/
    regs->CH0CONF &= ~0x0000003Cu;
    /* Configure the clkD field of MCSPI_CHCONF register.*/
    regs->CH0CONF |= (clkD << AM335X_SPI_CH0CONF_CLKD_SHIFT);
    printk("CLOCK CH0CONF %x\n", regs->CH0CONF);
}

static int am335x_spi_init(am335x_spi_bus *bus)
{
  volatile am335x_spi_regs *regs = bus->regs;

  /* Enable the clocks */
  am335x_spi_clock_enable();
  /* reset the controller */
  am335x_spi_reset(bus);
  /* setup the freq */
  am335x_spi_clock_setup(bus, 2000000);

  /*
   * Enable master mode, slave select, single channel
   */
  uint32_t mctrl = regs->MODULCTRL;
  CLEAR(mctrl, AM335X_SPI_MODULCTRL_MS); /* Set master mode */
  CLEAR(mctrl, AM335X_SPI_MODULCTRL_PIN34); /* SPIEN is used */
  SET(mctrl, AM335X_SPI_MODULCTRL_SINGLE); /* Single channel mode */
  regs->MODULCTRL = mctrl;
  printk("MODULE CTRL: %x\n", regs->MODULCTRL);

  /* Clear and disable interrupts */
  regs->IRQENABLE = 0x0;
  regs->IRQSTATUS = ~0x0;

  /*
   * Enable transmit/receive mode.
   * Set SPIDAT[0] as MISO and SPIDAT[1] as MOSI.
   */
  uint32_t ch0conf = regs->CH0CONF;
  ch0conf &= ~((1 << 12) | (1 << 13)); /* Clear T/R Mode bits */
  SET(ch0conf, AM335X_SPI_CH0CONF_DPE0); /* Tranmission disable d0 */
  CLEAR(ch0conf, AM335X_SPI_CH0CONF_DPE1); /* Tranmission enable d1 */
  CLEAR(ch0conf, AM335X_SPI_CH0CONF_IS); /* reception enable d0 */
  SET(ch0conf, AM335X_SPI_CH0CONF_EPOL); /* CS Active high */

  /* Set no of bits per data frame */
  ch0conf |= AM335X_SPI_CH0CONF_WL(7); /* 8 bits per data frame */

  /* SPI MODE 0 */
  CLEAR(ch0conf, 1 << 1); /* CPOL = 0 */
  CLEAR(ch0conf, 1 << 0); /* CPHA = 0 */

  regs->CH0CONF = ch0conf;
  printk("CH0 CONF: %x\n", regs->CH0CONF);
  printk("spi init finish\n");
  return 0;
}

static int am335x_spi_bus_register(
  phandle_t node
)
{
  am335x_spi_bus *bus;
  rtems_ofw_memory_area reg;
  const char *bus_path = "/dev/spi-0";
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

  bus->regs = (volatile am335x_spi_regs *)reg.start;
  bus->base.max_speed_hz = AM335X_SPI_MAX_SPEED;
  bus->base.bits_per_word = 8;

  rv = am335x_spi_init(bus);
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