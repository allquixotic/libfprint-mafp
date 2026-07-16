/*
 * Microarray MAFP8800 press-image quality helpers for libfprint
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#pragma once

#include <gio/gio.h>

G_BEGIN_DECLS

#define MAFP8800_FP36_DETECT_PIXEL_DELTA 320
#define MAFP8800_FP36_DETECT_MIN_CHANGED_PIXELS 1786
#define MAFP8800_FP36_STABLE_SAD_LIMIT 114687

gboolean mafp8800_fp36_measure_finger (const guint8 *background,
                                       gsize         background_size,
                                       const guint8 *frame,
                                       gsize         frame_size,
                                       gboolean     *finger_present,
                                       guint        *changed_pixels,
                                       GError      **error);

gboolean mafp8800_fp36_measure_stability (const guint8 *previous,
                                          gsize         previous_size,
                                          const guint8 *current,
                                          gsize         current_size,
                                          gboolean     *stable,
                                          guint64      *sum_absolute_difference,
                                          GError      **error);

G_END_DECLS
