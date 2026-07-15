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

static guint16
fp36_frame_pixel (const guint8 *frame, guint row, guint column)
{
  gsize offset = (gsize) row * MAFP8800_FP36_ROW_SIZE + column * 2;

  return (guint16) frame[offset] | ((guint16) frame[offset + 1] << 8);
}

/**
 * mafp8800_enhance_fp36_frame:
 * @background: decoded no-finger FP36 frame
 * @background_size: size of @background
 * @finger: decoded finger FP36 frame
 * @finger_size: size of @finger
 * @enhanced: destination for normalized 16-bit pixels
 * @enhanced_pixels: number of elements available in @enhanced
 * @error: return location for an error
 *
 * Subtracts a finger frame from its no-finger background, discards the
 * sensor's non-image column zero, and linearly normalizes the signed result.
 * Signed arithmetic deliberately avoids the imported implementation's
 * modulo-65536 underflow behavior.
 *
 * Returns: %TRUE on success. A low-contrast image is valid and produces an
 * all-zero output.
 */
gboolean
mafp8800_enhance_fp36_frame (const guint8 *background,
                             gsize         background_size,
                             const guint8 *finger,
                             gsize         finger_size,
                             guint16      *enhanced,
                             gsize         enhanced_pixels,
                             GError      **error)
{
  gint32 minimum = G_MAXINT32;
  gint32 maximum = G_MININT32;

  g_return_val_if_fail (background != NULL, FALSE);
  g_return_val_if_fail (finger != NULL, FALSE);
  g_return_val_if_fail (enhanced != NULL, FALSE);
  g_return_val_if_fail (error == NULL || *error == NULL, FALSE);

  if (background_size < MAFP8800_FP36_FRAME_SIZE ||
      finger_size < MAFP8800_FP36_FRAME_SIZE)
    {
      g_set_error_literal (error,
                           G_IO_ERROR,
                           G_IO_ERROR_INVALID_ARGUMENT,
                           "FP36 enhancement requires two complete frames");
      return FALSE;
    }

  if (enhanced_pixels < MAFP8800_FP36_ENHANCED_PIXELS)
    {
      g_set_error (error,
                   G_IO_ERROR,
                   G_IO_ERROR_NO_SPACE,
                   "FP36 enhanced buffer has %" G_GSIZE_FORMAT
                   " pixels, requires %u",
                   enhanced_pixels,
                   MAFP8800_FP36_ENHANCED_PIXELS);
      return FALSE;
    }

  memset (enhanced,
          0,
          MAFP8800_FP36_ENHANCED_PIXELS * sizeof (guint16));

  for (guint row = 0; row < MAFP8800_FP36_ROWS; row++)
    for (guint column = 1; column < MAFP8800_FP36_COLUMNS; column++)
      {
        gint32 delta =
          (gint32) fp36_frame_pixel (background, row, column) -
          (gint32) fp36_frame_pixel (finger, row, column);

        minimum = MIN (minimum, delta);
        maximum = MAX (maximum, delta);
      }

  if (maximum - minimum <= 50)
    return TRUE;

  for (guint row = 0; row < MAFP8800_FP36_ROWS; row++)
    for (guint column = 1; column < MAFP8800_FP36_COLUMNS; column++)
      {
        gsize index =
          (gsize) row * MAFP8800_FP36_ENHANCED_COLUMNS + column - 1;
        gint32 delta =
          (gint32) fp36_frame_pixel (background, row, column) -
          (gint32) fp36_frame_pixel (finger, row, column);
        guint64 normalized =
          (guint64) (delta - minimum) * G_MAXUINT16 /
          (guint32) (maximum - minimum);

        enhanced[index] = (guint16) normalized;
      }

  return TRUE;
}
