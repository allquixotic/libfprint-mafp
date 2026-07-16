/*
 * Microarray MAFP8800 press-image quality helpers for libfprint
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include "mafp8800-press.h"
#include "mafp8800-proto.h"

static gboolean
mafp_validate_frame_pair (const guint8 *first,
                          gsize         first_size,
                          const guint8 *second,
                          gsize         second_size,
                          GError      **error)
{
  if (!first || !second)
    {
      g_set_error_literal (error,
                           G_IO_ERROR,
                           G_IO_ERROR_INVALID_ARGUMENT,
                           "FP36 frame pointer is NULL");
      return FALSE;
    }

  if (first_size < MAFP8800_FP36_FRAME_SIZE ||
      second_size < MAFP8800_FP36_FRAME_SIZE)
    {
      g_set_error (error,
                   G_IO_ERROR,
                   G_IO_ERROR_INVALID_ARGUMENT,
                   "FP36 frame is too small (%" G_GSIZE_FORMAT
                   "/%" G_GSIZE_FORMAT "; need %u bytes each)",
                   first_size,
                   second_size,
                   (guint) MAFP8800_FP36_FRAME_SIZE);
      return FALSE;
    }

  return TRUE;
}

static guint16
mafp_frame_pixel (const guint8 *frame, guint row, guint column)
{
  const gsize offset =
    (gsize) row * MAFP8800_FP36_ROW_SIZE + column * sizeof (guint16);

  return (guint16) frame[offset] | ((guint16) frame[offset + 1] << 8);
}

/**
 * mafp8800_fp36_measure_finger:
 * @background: complete uncovered FP36 frame
 * @background_size: size of @background
 * @frame: complete frame to classify
 * @frame_size: size of @frame
 * @finger_present: (out): whether the frame crosses the presence threshold
 * @changed_pixels: (out) (optional): number of sufficiently darkened pixels
 * @error: return location for an input error
 *
 * Classifies one decoded frame without allocating memory. Column zero is a
 * sensor metadata/dummy column and is intentionally excluded. Outputs are
 * reset before validation so callers cannot accidentally reuse stale results.
 *
 * Returns: %TRUE when the inputs were valid
 */
gboolean
mafp8800_fp36_measure_finger (const guint8 *background,
                              gsize         background_size,
                              const guint8 *frame,
                              gsize         frame_size,
                              gboolean     *finger_present,
                              guint        *changed_pixels,
                              GError      **error)
{
  guint changed = 0;

  if (finger_present)
    *finger_present = FALSE;
  if (changed_pixels)
    *changed_pixels = 0;

  if (!finger_present)
    {
      g_set_error_literal (error,
                           G_IO_ERROR,
                           G_IO_ERROR_INVALID_ARGUMENT,
                           "FP36 finger-present output is NULL");
      return FALSE;
    }

  if (!mafp_validate_frame_pair (background,
                                 background_size,
                                 frame,
                                 frame_size,
                                 error))
    return FALSE;

  for (guint row = 0; row < MAFP8800_FP36_ROWS; row++)
    for (guint column = 1; column < MAFP8800_FP36_COLUMNS; column++)
      {
        const gint32 background_value = mafp_frame_pixel (background,
                                                          row,
                                                          column);
        const gint32 frame_value = mafp_frame_pixel (frame, row, column);

        if (background_value - frame_value >
            MAFP8800_FP36_DETECT_PIXEL_DELTA)
          changed++;
      }

  *finger_present =
    changed >= MAFP8800_FP36_DETECT_MIN_CHANGED_PIXELS;
  if (changed_pixels)
    *changed_pixels = changed;

  return TRUE;
}

/**
 * mafp8800_fp36_measure_stability:
 * @previous: first complete finger frame
 * @previous_size: size of @previous
 * @current: second complete finger frame
 * @current_size: size of @current
 * @stable: (out): whether the frames cross the stability threshold
 * @sum_absolute_difference: (out) (optional): measured frame difference
 * @error: return location for an input error
 *
 * Computes a bounded 64-bit sum of absolute pixel differences without
 * allocating memory. Column zero is intentionally excluded.
 *
 * Returns: %TRUE when the inputs were valid
 */
gboolean
mafp8800_fp36_measure_stability (const guint8 *previous,
                                 gsize         previous_size,
                                 const guint8 *current,
                                 gsize         current_size,
                                 gboolean     *stable,
                                 guint64      *sum_absolute_difference,
                                 GError      **error)
{
  guint64 difference_sum = 0;

  if (stable)
    *stable = FALSE;
  if (sum_absolute_difference)
    *sum_absolute_difference = 0;

  if (!stable)
    {
      g_set_error_literal (error,
                           G_IO_ERROR,
                           G_IO_ERROR_INVALID_ARGUMENT,
                           "FP36 stable output is NULL");
      return FALSE;
    }

  if (!mafp_validate_frame_pair (previous,
                                 previous_size,
                                 current,
                                 current_size,
                                 error))
    return FALSE;

  for (guint row = 0; row < MAFP8800_FP36_ROWS; row++)
    for (guint column = 1; column < MAFP8800_FP36_COLUMNS; column++)
      {
        const gint32 previous_value = mafp_frame_pixel (previous, row, column);
        const gint32 current_value = mafp_frame_pixel (current, row, column);
        const gint32 difference = previous_value - current_value;

        difference_sum += difference < 0 ? -(gint64) difference : difference;
      }

  *stable = difference_sum <= MAFP8800_FP36_STABLE_SAD_LIMIT;
  if (sum_absolute_difference)
    *sum_absolute_difference = difference_sum;

  return TRUE;
}
