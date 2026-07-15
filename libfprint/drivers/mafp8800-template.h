/*
 * Versioned MAFP8800 host-template format
 *
 * All multi-byte integers are little-endian.  Unused bytes are zero so that
 * every logical template has one canonical serialization.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#pragma once

#include <gio/gio.h>

G_BEGIN_DECLS

#define MAFP8800_TEMPLATE_MAGIC 0x4D
#define MAFP8800_TEMPLATE_VERSION 1
#define MAFP8800_TEMPLATE_HEADER_SIZE 4
#define MAFP8800_TEMPLATE_MAX_SAMPLES 8
#define MAFP8800_TEMPLATE_SAMPLE_MAGIC 0xEF
#define MAFP8800_TEMPLATE_MAX_KEYPOINTS 50
#define MAFP8800_TEMPLATE_BANKS 2
#define MAFP8800_TEMPLATE_DESCRIPTOR_SIZE 16
#define MAFP8800_TEMPLATE_KEYPOINT_META_SIZE 4
#define MAFP8800_TEMPLATE_KEYPOINT_SIZE \
  (MAFP8800_TEMPLATE_DESCRIPTOR_SIZE + MAFP8800_TEMPLATE_KEYPOINT_META_SIZE)
#define MAFP8800_TEMPLATE_BANK_DATA_SIZE \
  (MAFP8800_TEMPLATE_MAX_KEYPOINTS * MAFP8800_TEMPLATE_KEYPOINT_SIZE)
#define MAFP8800_TEMPLATE_BANK_SIZE \
  (4 + MAFP8800_TEMPLATE_BANK_DATA_SIZE)
#define MAFP8800_TEMPLATE_SAMPLE_SIZE \
  (4 + MAFP8800_TEMPLATE_BANKS * MAFP8800_TEMPLATE_BANK_SIZE)
#define MAFP8800_TEMPLATE_SIZE \
  (MAFP8800_TEMPLATE_HEADER_SIZE + \
   MAFP8800_TEMPLATE_MAX_SAMPLES * MAFP8800_TEMPLATE_SAMPLE_SIZE)

void mafp8800_template_initialize (guint8 *data,
                                   gsize   size);
void mafp8800_template_set_sample_count (guint8 *data,
                                         gsize   size,
                                         guint   sample_count);
gboolean mafp8800_template_validate (const guint8 *data,
                                     gsize         size,
                                     guint        *sample_count,
                                     GError      **error);

G_END_DECLS
