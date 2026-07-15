/*
 * Microarray MAFP8800 protocol helpers for libfprint
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#pragma once

#include <gio/gio.h>

G_BEGIN_DECLS

#define MAFP8800_FP36_ROWS 160
#define MAFP8800_FP36_COLUMNS 37
#define MAFP8800_FP36_ROW_SIZE (MAFP8800_FP36_COLUMNS * sizeof (guint16))
#define MAFP8800_FP36_FRAME_SIZE (MAFP8800_FP36_ROWS * MAFP8800_FP36_ROW_SIZE)
#define MAFP8800_FP36_RAW_SIZE 20480
#define MAFP8800_FP36_ENHANCED_COLUMNS (MAFP8800_FP36_COLUMNS - 1)
#define MAFP8800_FP36_ENHANCED_PIXELS \
  (MAFP8800_FP36_ROWS * MAFP8800_FP36_ENHANCED_COLUMNS)

gboolean mafp8800_parse_fp36_rows (const guint8 *raw,
                                   gsize         raw_size,
                                   guint8       *frame,
                                   gsize         frame_size,
                                   guint         expected_rows,
                                   guint        *parsed_rows,
                                   GError      **error);

gboolean mafp8800_enhance_fp36_frame (const guint8 *background,
                                      gsize         background_size,
                                      const guint8 *finger,
                                      gsize         finger_size,
                                      guint16      *enhanced,
                                      gsize         enhanced_pixels,
                                      GError      **error);

G_END_DECLS
