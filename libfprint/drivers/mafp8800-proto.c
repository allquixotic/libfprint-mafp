/*
 * Microarray MAFP8800 protocol helpers for libfprint
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include "mafp8800-proto.h"

#define FP36_ROW_MARKER_SIZE 4

static gboolean
is_fp36_row_marker (const guint8 *data)
{
  return data[0] == 0x00 && data[1] == 0x00 &&
         data[2] == 0x0A && (data[3] & 0xF0) == 0x50;
}

/**
 * mafp8800_parse_fp36_rows:
 * @raw: raw bytes returned by an FP36 image-read transaction
 * @raw_size: size of @raw
 * @frame: destination for decoded little-endian rows
 * @frame_size: size of @frame
 * @expected_rows: exact number of rows required
 * @parsed_rows: (out) (optional): rows found before success or failure
 * @error: return location for an error
 *
 * Extracts rows prefixed by the four-byte FP36 marker and converts each
 * 16-bit sample from wire big-endian to little-endian. The operation is
 * transactional from the caller's perspective: @frame is cleared on every
 * failure, so an incomplete capture can never be mistaken for a complete or
 * previously captured frame.
 *
 * Returns: %TRUE only if exactly @expected_rows rows were decoded.
 */
gboolean
mafp8800_parse_fp36_rows (const guint8 *raw,
                          gsize         raw_size,
                          guint8       *frame,
                          gsize         frame_size,
                          guint         expected_rows,
                          guint        *parsed_rows,
                          GError      **error)
{
  gsize required_size;
  gsize offset = 0;
  guint rows = 0;

  g_return_val_if_fail (raw != NULL, FALSE);
  g_return_val_if_fail (frame != NULL, FALSE);
  g_return_val_if_fail (error == NULL || *error == NULL, FALSE);

  if (parsed_rows)
    *parsed_rows = 0;

  if (expected_rows == 0 || expected_rows > MAFP8800_FP36_ROWS)
    {
      g_set_error (error,
                   G_IO_ERROR,
                   G_IO_ERROR_INVALID_ARGUMENT,
                   "Invalid FP36 row count %u (expected 1..%u)",
                   expected_rows,
                   MAFP8800_FP36_ROWS);
      return FALSE;
    }

  required_size = (gsize) expected_rows * MAFP8800_FP36_ROW_SIZE;
  if (frame_size < required_size)
    {
      g_set_error (error,
                   G_IO_ERROR,
                   G_IO_ERROR_NO_SPACE,
                   "FP36 frame buffer has size %" G_GSIZE_FORMAT
                   ", requires %" G_GSIZE_FORMAT,
                   frame_size,
                   required_size);
      return FALSE;
    }

  memset (frame, 0, required_size);

  while (rows < expected_rows &&
         offset <= raw_size &&
         raw_size - offset >= FP36_ROW_MARKER_SIZE)
    {
      gsize source;
      guint8 *destination;

      if (!is_fp36_row_marker (raw + offset))
        {
          offset++;
          continue;
        }

      source = offset + FP36_ROW_MARKER_SIZE;
      if (source > raw_size ||
          raw_size - source < MAFP8800_FP36_ROW_SIZE)
        break;

      destination = frame + (gsize) rows * MAFP8800_FP36_ROW_SIZE;
      for (gsize byte = 0; byte < MAFP8800_FP36_ROW_SIZE; byte += 2)
        {
          destination[byte] = raw[source + byte + 1];
          destination[byte + 1] = raw[source + byte];
        }

      rows++;
      offset = source + MAFP8800_FP36_ROW_SIZE;
    }

  if (parsed_rows)
    *parsed_rows = rows;

  if (rows != expected_rows)
    {
      memset (frame, 0, required_size);
      g_set_error (error,
                   G_IO_ERROR,
                   G_IO_ERROR_INVALID_DATA,
                   "Incomplete FP36 frame: decoded %u of %u rows from %"
                   G_GSIZE_FORMAT " bytes",
                   rows,
                   expected_rows,
                   raw_size);
      return FALSE;
    }

  return TRUE;
}
