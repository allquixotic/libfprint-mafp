/*
 * MAFP8800 protocol parser unit tests
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include <glib.h>

#include "drivers/mafp8800-proto.h"

#define GUARD_SIZE 32
#define TEST_ROWS 4
#define TEST_RAW_SIZE 1024
#define TEST_MARKER_SIZE 4

static gsize
append_row (guint8 *raw, gsize offset, guint row)
{
  raw[offset++] = 0x00;
  raw[offset++] = 0x00;
  raw[offset++] = 0x0A;
  raw[offset++] = 0x50 | (row & 0x0F);

  for (guint column = 0; column < MAFP8800_FP36_COLUMNS; column++)
    {
      guint16 value = (guint16) (row * 0x100 + column);

      raw[offset++] = value >> 8;
      raw[offset++] = value & 0xFF;
    }

  return offset;
}

static void
assert_guards (const guint8 *allocation, gsize frame_size)
{
  for (gsize i = 0; i < GUARD_SIZE; i++)
    {
      g_assert_cmphex (allocation[i], ==, 0xA5);
      g_assert_cmphex (allocation[GUARD_SIZE + frame_size + i], ==, 0xA5);
    }
}

static void
test_parse_complete (void)
{
  const gsize frame_size = TEST_ROWS * MAFP8800_FP36_ROW_SIZE;
  g_autofree guint8 *raw = g_malloc0 (TEST_RAW_SIZE);
  g_autofree guint8 *allocation = g_malloc0 (frame_size + 2 * GUARD_SIZE);
  guint8 *frame = allocation + GUARD_SIZE;
  g_autoptr(GError) error = NULL;
  guint parsed_rows = 0;
  gsize offset = 7;

  memset (allocation, 0xA5, frame_size + 2 * GUARD_SIZE);
  memset (raw, 0xCC, TEST_RAW_SIZE);
  for (guint row = 0; row < TEST_ROWS; row++)
    {
      offset = append_row (raw, offset, row);
      offset += row + 1;
    }

  g_assert_true (mafp8800_parse_fp36_rows (raw,
                                           TEST_RAW_SIZE,
                                           frame,
                                           frame_size,
                                           TEST_ROWS,
                                           &parsed_rows,
                                           &error));
  g_assert_no_error (error);
  g_assert_cmpuint (parsed_rows, ==, TEST_ROWS);
  assert_guards (allocation, frame_size);

  for (guint row = 0; row < TEST_ROWS; row++)
    for (guint column = 0; column < MAFP8800_FP36_COLUMNS; column++)
      {
        gsize index = ((gsize) row * MAFP8800_FP36_COLUMNS + column) * 2;
        guint16 expected = (guint16) (row * 0x100 + column);

        g_assert_cmphex (frame[index], ==, expected & 0xFF);
        g_assert_cmphex (frame[index + 1], ==, expected >> 8);
      }
}

static void
test_parse_incomplete_clears_output (void)
{
  const gsize frame_size = TEST_ROWS * MAFP8800_FP36_ROW_SIZE;
  g_autofree guint8 *raw = g_malloc0 (TEST_RAW_SIZE);
  g_autofree guint8 *allocation = g_malloc0 (frame_size + 2 * GUARD_SIZE);
  guint8 *frame = allocation + GUARD_SIZE;
  g_autoptr(GError) error = NULL;
  guint parsed_rows = 0;
  gsize offset = 0;

  memset (allocation, 0xA5, frame_size + 2 * GUARD_SIZE);
  for (guint row = 0; row < TEST_ROWS - 1; row++)
    offset = append_row (raw, offset, row);

  g_assert_false (mafp8800_parse_fp36_rows (raw,
                                            offset,
                                            frame,
                                            frame_size,
                                            TEST_ROWS,
                                            &parsed_rows,
                                            &error));
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA);
  g_assert_cmpuint (parsed_rows, ==, TEST_ROWS - 1);
  assert_guards (allocation, frame_size);
  for (gsize i = 0; i < frame_size; i++)
    g_assert_cmphex (frame[i], ==, 0x00);
}

static void
test_parse_truncated_row (void)
{
  const gsize frame_size = MAFP8800_FP36_ROW_SIZE;
  g_autofree guint8 *raw = g_malloc0 (TEST_MARKER_SIZE + frame_size);
  g_autofree guint8 *frame = g_malloc0 (frame_size);
  g_autoptr(GError) error = NULL;
  guint parsed_rows = G_MAXUINT;
  gsize full_size;

  full_size = append_row (raw, 0, 0);
  g_assert_false (mafp8800_parse_fp36_rows (raw,
                                            full_size - 1,
                                            frame,
                                            frame_size,
                                            1,
                                            &parsed_rows,
                                            &error));
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA);
  g_assert_cmpuint (parsed_rows, ==, 0);
}

static void
test_parse_small_destination (void)
{
  const gsize frame_size = MAFP8800_FP36_ROW_SIZE;
  g_autofree guint8 *raw = g_malloc0 (TEST_MARKER_SIZE + frame_size);
  g_autofree guint8 *allocation = g_malloc0 (frame_size + 2 * GUARD_SIZE);
  guint8 *frame = allocation + GUARD_SIZE;
  g_autoptr(GError) error = NULL;

  append_row (raw, 0, 0);
  memset (allocation, 0xA5, frame_size + 2 * GUARD_SIZE);

  g_assert_false (mafp8800_parse_fp36_rows (raw,
                                            TEST_MARKER_SIZE + frame_size,
                                            frame,
                                            frame_size - 1,
                                            1,
                                            NULL,
                                            &error));
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_NO_SPACE);
  assert_guards (allocation, frame_size);
  for (gsize i = 0; i < frame_size; i++)
    g_assert_cmphex (frame[i], ==, 0xA5);
}

static void
test_parse_all_truncation_boundaries (void)
{
  const guint rows = 8;
  const gsize frame_size = rows * MAFP8800_FP36_ROW_SIZE;
  const gsize raw_size = rows * (4 + MAFP8800_FP36_ROW_SIZE);
  g_autofree guint8 *raw = g_malloc0 (raw_size);
  g_autofree guint8 *frame = g_malloc0 (frame_size);
  gsize offset = 0;

  for (guint row = 0; row < rows; row++)
    offset = append_row (raw, offset, row);
  g_assert_cmpuint (offset, ==, raw_size);

  for (gsize cut = 0; cut < raw_size; cut++)
    {
      g_autoptr(GError) error = NULL;

      memset (frame, 0x5A, frame_size);
      g_assert_false (mafp8800_parse_fp36_rows (raw,
                                                cut,
                                                frame,
                                                frame_size,
                                                rows,
                                                NULL,
                                                &error));
      g_assert_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA);
      for (gsize i = 0; i < frame_size; i++)
        g_assert_cmphex (frame[i], ==, 0x00);
    }
}

static void
test_parse_randomized_layouts (void)
{
  const guint rows = 16;
  const gsize frame_size = rows * MAFP8800_FP36_ROW_SIZE;
  g_autoptr(GRand) random = g_rand_new_with_seed (0x4D414650);
  g_autofree guint8 *raw = g_malloc0 (MAFP8800_FP36_RAW_SIZE);
  g_autofree guint8 *frame = g_malloc0 (frame_size);

  for (guint iteration = 0; iteration < 1000; iteration++)
    {
      g_autoptr(GError) error = NULL;
      gsize offset = g_rand_int_range (random, 0, 32);

      memset (raw, 0xE7, MAFP8800_FP36_RAW_SIZE);
      for (guint row = 0; row < rows; row++)
        {
          offset = append_row (raw, offset, row);
          offset += g_rand_int_range (random, 0, 17);
        }

      g_assert_true (mafp8800_parse_fp36_rows (raw,
                                               offset,
                                               frame,
                                               frame_size,
                                               rows,
                                               NULL,
                                               &error));
      g_assert_no_error (error);
    }
}

static void
set_frame_pixel (guint8 *frame, guint row, guint column, guint16 value)
{
  gsize offset = (gsize) row * MAFP8800_FP36_ROW_SIZE + column * 2;

  frame[offset] = value & 0xFF;
  frame[offset + 1] = value >> 8;
}

static void
test_enhance_normalizes_signed_delta (void)
{
  g_autofree guint8 *background = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  g_autofree guint8 *finger = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  g_autofree guint16 *enhanced =
    g_new0 (guint16, MAFP8800_FP36_ENHANCED_PIXELS);
  g_autoptr(GError) error = NULL;

  for (guint row = 0; row < MAFP8800_FP36_ROWS; row++)
    for (guint column = 0; column < MAFP8800_FP36_COLUMNS; column++)
      {
        guint16 delta = (guint16) (row * MAFP8800_FP36_COLUMNS + column);

        set_frame_pixel (background, row, column, 10000);
        set_frame_pixel (finger, row, column, 10000 - delta);
      }

  g_assert_true (mafp8800_enhance_fp36_frame (
                   background,
                   MAFP8800_FP36_FRAME_SIZE,
                   finger,
                   MAFP8800_FP36_FRAME_SIZE,
                   enhanced,
                   MAFP8800_FP36_ENHANCED_PIXELS,
                   &error));
  g_assert_no_error (error);
  g_assert_cmpuint (enhanced[0], ==, 0);
  g_assert_cmpuint (enhanced[MAFP8800_FP36_ENHANCED_PIXELS - 1], ==,
                    G_MAXUINT16);
  for (gsize i = 1; i < MAFP8800_FP36_ENHANCED_PIXELS; i++)
    g_assert_cmpuint (enhanced[i], >=, enhanced[i - 1]);
}

static void
test_enhance_low_contrast_is_zero (void)
{
  g_autofree guint8 *background = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  g_autofree guint8 *finger = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  g_autofree guint16 *enhanced =
    g_new (guint16, MAFP8800_FP36_ENHANCED_PIXELS);
  g_autoptr(GError) error = NULL;

  memset (enhanced,
          0xA5,
          MAFP8800_FP36_ENHANCED_PIXELS * sizeof (guint16));
  g_assert_true (mafp8800_enhance_fp36_frame (
                   background,
                   MAFP8800_FP36_FRAME_SIZE,
                   finger,
                   MAFP8800_FP36_FRAME_SIZE,
                   enhanced,
                   MAFP8800_FP36_ENHANCED_PIXELS,
                   &error));
  g_assert_no_error (error);
  for (gsize i = 0; i < MAFP8800_FP36_ENHANCED_PIXELS; i++)
    g_assert_cmpuint (enhanced[i], ==, 0);
}

static void
test_enhance_rejects_short_buffers (void)
{
  g_autofree guint8 *frame = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  g_autofree guint16 *enhanced =
    g_new (guint16, MAFP8800_FP36_ENHANCED_PIXELS);
  g_autoptr(GError) error = NULL;

  memset (enhanced,
          0xA5,
          MAFP8800_FP36_ENHANCED_PIXELS * sizeof (guint16));
  g_assert_false (mafp8800_enhance_fp36_frame (
                    frame,
                    MAFP8800_FP36_FRAME_SIZE - 1,
                    frame,
                    MAFP8800_FP36_FRAME_SIZE,
                    enhanced,
                    MAFP8800_FP36_ENHANCED_PIXELS,
                    &error));
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_ARGUMENT);
  for (gsize i = 0; i < MAFP8800_FP36_ENHANCED_PIXELS; i++)
    g_assert_cmphex (enhanced[i], ==, 0xA5A5);

  g_clear_error (&error);
  g_assert_false (mafp8800_enhance_fp36_frame (
                    frame,
                    MAFP8800_FP36_FRAME_SIZE,
                    frame,
                    MAFP8800_FP36_FRAME_SIZE,
                    enhanced,
                    MAFP8800_FP36_ENHANCED_PIXELS - 1,
                    &error));
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_NO_SPACE);
}

int
main (int argc, char *argv[])
{
  g_test_init (&argc, &argv, NULL);

  g_test_add_func ("/mafp8800/proto/complete", test_parse_complete);
  g_test_add_func ("/mafp8800/proto/incomplete-clears-output",
                   test_parse_incomplete_clears_output);
  g_test_add_func ("/mafp8800/proto/truncated-row", test_parse_truncated_row);
  g_test_add_func ("/mafp8800/proto/small-destination",
                   test_parse_small_destination);
  g_test_add_func ("/mafp8800/proto/all-truncation-boundaries",
                   test_parse_all_truncation_boundaries);
  g_test_add_func ("/mafp8800/proto/randomized-layouts",
                   test_parse_randomized_layouts);
  g_test_add_func ("/mafp8800/proto/enhance/normalizes-signed-delta",
                   test_enhance_normalizes_signed_delta);
  g_test_add_func ("/mafp8800/proto/enhance/low-contrast-is-zero",
                   test_enhance_low_contrast_is_zero);
  g_test_add_func ("/mafp8800/proto/enhance/rejects-short-buffers",
                   test_enhance_rejects_short_buffers);

  return g_test_run ();
}
