/*
 *  COPYRIGHT (c) 1989-2012.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems.h>
#include <dirent.h>
#include <sys/types.h>
#include <tmacros.h>
#include <bsp/spi.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <dev/spi/spi.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <rtems/bspIo.h>

const char rtems_test_name[] = "HELLO WORLD";
const char *bus = "/dev/spi-0";

static int register_spi(void) {
  return spi_register_0();
}

static rtems_task Init(
  rtems_task_argument ignored
)
{

  uint8_t buffer[1024] = {
    0xcb,
    0xb4,
    0xbd,
    0x4e,
    0x8a, 0x36, 0xc4, 0xcc,
    0xd4, 0x69, 0xba, 0x1e,
    0x9a, 0x91, 0xd9, 0xd4,
    0xea, 0xd4, 0x9e,
    0xbf
  };

  spi_ioc_transfer msg = {
    .len = 20,
    .rx_buf = buffer,
    .tx_buf = buffer,
    .speed_hz = 100000,
    .bits_per_word = 8,
    .mode = SPI_MODE_1,
    .cs = (uint8_t)0
  };

  printf( "TESTING SPI\n" );
  // printf("register_spi: %d\n", bbb_register_spi_0());
  printf("register_spi: %d\n", register_spi());

  int fd = open(BBB_SPI_0_BUS_PATH, O_RDWR);
  if (fd < 0) {
    perror("opening bus\n");
  }

  int rv = ioctl(fd, SPI_IOC_MESSAGE(1), &msg);
  if (rv == -1) {
    printk("Couldn't send the msg\n");
  } else {
    printf("recevied: ");
    for (int j = 0; j < msg.len; j++) {
      printk(" %02x", ((uint8_t *)msg.rx_buf)[j]);
    }
  }
  close(fd);
  while(1);

}


/* NOTICE: the clock driver is explicitly disabled */
// #define CONFIGURE_APPLICATION_NEED_TIMER_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_SIMPLE_CONSOLE_DRIVER

#define CONFIGURE_MAXIMUM_FILE_DESCRIPTORS 32
#define CONFIGURE_UNLIMITED_OBJECTS
#define CONFIGURE_UNIFIED_WORK_AREAS
#define CONFIGURE_MAXIMUM_DRIVERS 20

#define CONFIGURE_SHELL_COMMANDS_INIT
#define CONFIGURE_SHELL_COMMANDS_ALL
#include <rtems/shellconfig.h>

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INIT_TASK_ATTRIBUTES RTEMS_FLOATING_POINT

#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
