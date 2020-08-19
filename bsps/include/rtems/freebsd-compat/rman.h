
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

#ifndef _FREEBSD_COMPAT_RMAN_H
#define _FREEBSD_COMPAT_RMAN_H

#include <stdint.h>

typedef uint32_t bus_size_t;

/*
 * Access methods for bus resources and address space.
 */
typedef int       bus_space_tag_t;
typedef uintptr_t bus_space_handle_t;

#define	RF_ALLOCATED    0x0001	/* resource has been reserved */
#define	RF_ACTIVE       0x0002	/* resource allocation has been activated */
#define	RF_SHAREABLE    0x0004	/* resource permits contemporaneous sharing */
#define	RF_SPARE1       0x0008
#define	RF_SPARE2       0x0010
#define	RF_FIRSTSHARE   0x0020	/* first in sharing list */
#define	RF_PREFETCHABLE 0x0040	/* resource is prefetchable */
#define	RF_OPTIONAL     0x0080	/* for bus_alloc_resources() */
#define	RF_UNMAPPED     0x0100	/* don't map resource when activating */

struct resource {
  bus_space_tag_t     r_bustag;
  bus_space_handle_t  r_bushandle;/* bus_space handle */
};

#endif /* _FREEBSD_COMPAT_RMAN_H */
