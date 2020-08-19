/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup FreeBSDCompat
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

#ifndef _FREEBSD_COMPAT_BUS_H
#define _FREEBSD_COMPAT_BUS_H

#include <sys/types.h>
#include "device.h"
#include "rman.h"

struct resource_spec {
  int type;
  int rid;
  int flags;
};
#define RESOURCE_SPEC_END {-1, 0, 0}

/*
 * Write a 1, 2, 4, or 8 byte quantity to bus space
 * described by tag/handle/offset.
 */
static inline void
bus_space_write_1(bus_space_tag_t bst, bus_space_handle_t bsh, bus_size_t ofs,
    uint8_t val)
{
  uint8_t volatile *bsp = (uint8_t volatile *)(bsh + ofs);
  *bsp = val;
}

static inline void
bus_space_write_2(bus_space_tag_t bst, bus_space_handle_t bsh, bus_size_t ofs,
    uint16_t val)
{
  uint16_t volatile *bsp = (uint16_t volatile *)(bsh + ofs);
  *bsp = val;
}

static inline void
bus_space_write_4(bus_space_tag_t bst, bus_space_handle_t bsh, bus_size_t ofs,
    uint32_t val)
{
  uint32_t volatile *bsp = (uint32_t volatile *)(bsh + ofs);
  *bsp = val;
}

/*
 * Read a 1, 2, 4, or 8 byte quantity from bus space
 * described by tag/handle/offset.
 */
static inline uint8_t
bus_space_read_1(bus_space_tag_t tag, bus_space_handle_t handle,
     bus_size_t offset)
{

  return (*(volatile uint8_t *)(handle + offset));
}

static inline uint16_t
bus_space_read_2(bus_space_tag_t tag, bus_space_handle_t handle,
     bus_size_t offset)
{
  return (*(volatile uint16_t *)(handle + offset));
}

static inline uint32_t
bus_space_read_4(bus_space_tag_t tag, bus_space_handle_t handle,
        bus_size_t offset)
{
  return (*(volatile uint32_t *)(handle + offset));
}

bus_space_handle_t rman_get_bushandle(struct resource *);
bus_space_tag_t rman_get_bustag(struct resource *);

void	*device_get_softc(device_t dev);

int	bus_alloc_resources(device_t dev, struct resource_spec *rs,
          struct resource **res);

struct	resource *bus_alloc_resource(device_t dev, int type, int *rid,
             rman_res_t start, rman_res_t end,
             rman_res_t count, u_int flags);

static __inline struct resource *
bus_alloc_resource_any(device_t dev, int type, int *rid, uint32_t flags)
{
  return (bus_alloc_resource(dev, type, rid, 0, ~0, 1, flags));
}

#endif /* _FREEBSD_COMPAT_BUS_H_ */
