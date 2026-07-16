/*
 * Unit tests for MAFP8800 press-image quality helpers
 * Copyright (C) 2026
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include <glib.h>

#include "drivers/mafp8800-press.h"
#include "drivers/mafp8800-proto.h"

static void
set_pixel (guint8 *frame, guint row, guint column, guint16 value)
{
  const gsize offset =
    (gsize) row * MAFP8800_FP36_ROW_SIZE + column * sizeof (guint16);

  frame[offset] = value & 0xff;
  frame[offset + 1] = value >> 8;
}

static void
set_changed_pixels (guint8 *background,
                    guint8 *frame,
                    guint   count,
                    guint16 difference)
{
  guint changed = 0;

  memset (background, 0, MAFP8800_FP36_FRAME_SIZE);
  memset (frame, 0, MAFP8800_FP36_FRAME_SIZE);

  for (guint row = 0; row < MAFP8800_FP36_ROWS; row++)
    for (guint column = 1; column < MAFP8800_FP36_COLUMNS; column++)
      {
        set_pixel (background, row, column, 1000);
        set_pixel (frame,
                   row,
                   column,
                   changed++ < count ? 1000 - difference : 1000);
      }
}

static void
test_presence_thresholds (void)
{
  g_autofree guint8 *background = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  g_autofree guint8 *frame = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  gboolean present = TRUE;
  guint changed = G_MAXUINT;

  set_changed_pixels (background,
                      frame,
                      MAFP8800_FP36_DETECT_MIN_CHANGED_PIXELS,
                      MAFP8800_FP36_DETECT_PIXEL_DELTA);
  g_assert_true (mafp8800_fp36_measure_finger (
                   background,
                   MAFP8800_FP36_FRAME_SIZE,
                   frame,
                   MAFP8800_FP36_FRAME_SIZE,
                   &present,
                   &changed,
                   NULL));
  g_assert_false (present);
  g_assert_cmpuint (changed, ==, 0);

  set_changed_pixels (background,
                      frame,
                      MAFP8800_FP36_DETECT_MIN_CHANGED_PIXELS - 1,
                      MAFP8800_FP36_DETECT_PIXEL_DELTA + 1);
  g_assert_true (mafp8800_fp36_measure_finger (
                   background,
                   MAFP8800_FP36_FRAME_SIZE,
                   frame,
                   MAFP8800_FP36_FRAME_SIZE,
                   &present,
                   &changed,
                   NULL));
  g_assert_false (present);
  g_assert_cmpuint (changed,
                    ==,
                    MAFP8800_FP36_DETECT_MIN_CHANGED_PIXELS - 1);

  set_changed_pixels (background,
                      frame,
                      MAFP8800_FP36_DETECT_MIN_CHANGED_PIXELS,
                      MAFP8800_FP36_DETECT_PIXEL_DELTA + 1);
  g_assert_true (mafp8800_fp36_measure_finger (
                   background,
                   MAFP8800_FP36_FRAME_SIZE,
                   frame,
                   MAFP8800_FP36_FRAME_SIZE,
                   &present,
                   &changed,
                   NULL));
  g_assert_true (present);
  g_assert_cmpuint (changed,
                    ==,
                    MAFP8800_FP36_DETECT_MIN_CHANGED_PIXELS);
}

static void
test_dummy_column_ignored (void)
{
  g_autofree guint8 *background = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  g_autofree guint8 *frame = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  gboolean present = TRUE;
  gboolean stable = FALSE;
  guint changed = G_MAXUINT;
  guint64 sad = G_MAXUINT64;

  for (guint row = 0; row < MAFP8800_FP36_ROWS; row++)
    set_pixel (background, row, 0, G_MAXUINT16);

  g_assert_true (mafp8800_fp36_measure_finger (
                   background,
                   MAFP8800_FP36_FRAME_SIZE,
                   frame,
                   MAFP8800_FP36_FRAME_SIZE,
                   &present,
                   &changed,
                   NULL));
  g_assert_false (present);
  g_assert_cmpuint (changed, ==, 0);

  g_assert_true (mafp8800_fp36_measure_stability (
                   background,
                   MAFP8800_FP36_FRAME_SIZE,
                   frame,
                   MAFP8800_FP36_FRAME_SIZE,
                   &stable,
                   &sad,
                   NULL));
  g_assert_true (stable);
  g_assert_cmpuint (sad, ==, 0);
}

static void
test_stability_thresholds (void)
{
  g_autofree guint8 *previous = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  g_autofree guint8 *current = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  gboolean stable = FALSE;
  guint64 sad = 0;

  set_pixel (previous, 0, 1, 57343);
  set_pixel (previous, 0, 2, 57344);
  g_assert_true (mafp8800_fp36_measure_stability (
                   previous,
                   MAFP8800_FP36_FRAME_SIZE,
                   current,
                   MAFP8800_FP36_FRAME_SIZE,
                   &stable,
                   &sad,
                   NULL));
  g_assert_true (stable);
  g_assert_cmpuint (sad, ==, MAFP8800_FP36_STABLE_SAD_LIMIT);

  set_pixel (previous, 0, 2, 57345);
  g_assert_true (mafp8800_fp36_measure_stability (
                   previous,
                   MAFP8800_FP36_FRAME_SIZE,
                   current,
                   MAFP8800_FP36_FRAME_SIZE,
                   &stable,
                   &sad,
                   NULL));
  g_assert_false (stable);
  g_assert_cmpuint (sad, ==, MAFP8800_FP36_STABLE_SAD_LIMIT + 1);
}

static void
test_invalid_inputs_clear_outputs (void)
{
  g_autofree guint8 *frame = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);

  g_autoptr(GError) error = NULL;
  gboolean result = TRUE;
  guint changed = G_MAXUINT;
  guint64 sad = G_MAXUINT64;

  g_assert_false (mafp8800_fp36_measure_finger (
                    NULL,
                    MAFP8800_FP36_FRAME_SIZE,
                    frame,
                    MAFP8800_FP36_FRAME_SIZE,
                    &result,
                    &changed,
                    &error));
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_ARGUMENT);
  g_assert_false (result);
  g_assert_cmpuint (changed, ==, 0);

  g_clear_error (&error);
  result = TRUE;
  sad = G_MAXUINT64;
  g_assert_false (mafp8800_fp36_measure_stability (
                    frame,
                    MAFP8800_FP36_FRAME_SIZE - 1,
                    frame,
                    MAFP8800_FP36_FRAME_SIZE,
                    &result,
                    &sad,
                    &error));
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_ARGUMENT);
  g_assert_false (result);
  g_assert_cmpuint (sad, ==, 0);

  g_clear_error (&error);
  g_assert_false (mafp8800_fp36_measure_finger (
                    frame,
                    MAFP8800_FP36_FRAME_SIZE,
                    frame,
                    MAFP8800_FP36_FRAME_SIZE,
                    NULL,
                    NULL,
                    &error));
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_ARGUMENT);
}

static void
test_randomized_reference (void)
{
  g_autofree guint8 *first = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  g_autofree guint8 *second = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  GRand *random = g_rand_new_with_seed (0x880036);

  for (guint iteration = 0; iteration < 1000; iteration++)
    {
      guint expected_changed = 0;
      guint64 expected_sad = 0;
      gboolean present = FALSE;
      gboolean stable = FALSE;
      guint changed = 0;
      guint64 sad = 0;

      for (guint row = 0; row < MAFP8800_FP36_ROWS; row++)
        for (guint column = 0; column < MAFP8800_FP36_COLUMNS; column++)
          {
            const guint16 a = (guint16) g_rand_int_range (random, 0, 65536);
            const guint16 b = (guint16) g_rand_int_range (random, 0, 65536);
            const gint32 difference = (gint32) a - b;

            set_pixel (first, row, column, a);
            set_pixel (second, row, column, b);
            if (column == 0)
              continue;
            if (difference > MAFP8800_FP36_DETECT_PIXEL_DELTA)
              expected_changed++;
            expected_sad += difference < 0 ? -(gint64) difference : difference;
          }

      g_assert_true (mafp8800_fp36_measure_finger (
                       first,
                       MAFP8800_FP36_FRAME_SIZE,
                       second,
                       MAFP8800_FP36_FRAME_SIZE,
                       &present,
                       &changed,
                       NULL));
      g_assert_cmpuint (changed, ==, expected_changed);
      g_assert_cmpint (present,
                       ==,
                       expected_changed >=
                       MAFP8800_FP36_DETECT_MIN_CHANGED_PIXELS);

      g_assert_true (mafp8800_fp36_measure_stability (
                       first,
                       MAFP8800_FP36_FRAME_SIZE,
                       second,
                       MAFP8800_FP36_FRAME_SIZE,
                       &stable,
                       &sad,
                       NULL));
      g_assert_cmpuint (sad, ==, expected_sad);
      g_assert_cmpint (stable,
                       ==,
                       expected_sad <= MAFP8800_FP36_STABLE_SAD_LIMIT);
    }

  g_rand_free (random);
}

int
main (int argc, char **argv)
{
  g_test_init (&argc, &argv, NULL);

  g_test_add_func ("/mafp8800/press/presence-thresholds",
                   test_presence_thresholds);
  g_test_add_func ("/mafp8800/press/dummy-column-ignored",
                   test_dummy_column_ignored);
  g_test_add_func ("/mafp8800/press/stability-thresholds",
                   test_stability_thresholds);
  g_test_add_func ("/mafp8800/press/invalid-inputs-clear-outputs",
                   test_invalid_inputs_clear_outputs);
  g_test_add_func ("/mafp8800/press/randomized-reference",
                   test_randomized_reference);

  return g_test_run ();
}
