/*
 * Asynchronous Microarray MAFP8800 press acquisition for libfprint
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

#define MAFP8800_FP36_STABILITY_ATTEMPTS 20

FpiSsm *mafp8800_fp36_acquire_press_new (FpDevice     *device,
                                         int           spi_fd,
                                         GCancellable *cancellable,
                                         guint8        gain,
                                         const guint8 *background,
                                         gsize         background_size,
                                         guint8       *frame,
                                         gsize         frame_size,
                                         gboolean     *stable,
                                         guint        *changed_pixels,
                                         guint64      *stability_sad);

FpiSsm *mafp8800_fp36_wait_removal_new (FpDevice     *device,
                                        int           spi_fd,
                                        GCancellable *cancellable,
                                        guint8        gain,
                                        const guint8 *background,
                                        gsize         background_size,
                                        guint8       *scratch_frame,
                                        gsize         scratch_frame_size);

G_END_DECLS
