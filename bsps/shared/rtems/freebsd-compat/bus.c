/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup LIBFREEBSD
 *
 * @brief
 */

/*
 * Copyright (C) <2020> Niteesh Babu <niteesh.gs@gmail.com>
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rtems/bspIo.h>
#include <libfdt.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <ofw/ofw.h>
#include <rtems/freebsd-compat/bus.h>
#include <rtems/freebsd-compat/resource.h>

typedef unsigned int u_int;

/**
 * @brief Return the device's softc field
 *
 * The softc is allocated and zeroed when a driver is attached, based
 * on the size field of the driver.
 */
void *
device_get_softc(device_t dev)
{

  return (dev->softc);
}

bus_space_handle_t
rman_get_bushandle(struct resource *r)
{

  return (r->r_bushandle);
}

bus_space_tag_t
rman_get_bustag(struct resource *r)
{

  return (r->r_bustag);
}

int
bus_alloc_resources(device_t dev, struct resource_spec *rs,
    struct resource **res)
{
  int i;

  for (i = 0; rs[i].type != -1; i++) {
    res[i] = bus_alloc_resource_any(dev,
    rs[i].type, &rs[i].rid, rs[i].flags);

    if (res[i] == NULL) {
      return (-1);
    }
  }
  return (0);
}

struct resource *
bus_alloc_resource(device_t dev, int type, int *rid, rman_res_t start,
    rman_res_t end, rman_res_t count, u_int flags)
{
  int node;
  int rv;
  rtems_ofw_memory_area reg[*rid + 1];
  struct resource *res;

  node = dev->node;

  /*
   * We only support querying register values from FDT.
   */
  assert(type == SYS_RES_MEMORY);

  res = (struct resource *)malloc(sizeof(struct resource));

  if (res != NULL) {
    rv = rtems_ofw_get_reg(node, &reg[0], sizeof(reg));
    if (rv == -1) {
      return NULL;
    }
    res->r_bushandle = reg[*rid].start;
  }

  return (res);
}
