/*
 * Asynchronous Microarray MAFP8800 transport for libfprint
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#pragma once

#include "fpi-device.h"
#include "fpi-ssm.h"

G_BEGIN_DECLS

FpiSsm *mafp8800_fp36_capture_new (FpDevice      *device,
                                   int            spi_fd,
                                   GCancellable  *cancellable,
                                   guint8         gain,
                                   guint8         integration,
                                   guint8         dac,
                                   guint8        *frame,
                                   gsize          frame_size);

G_END_DECLS
