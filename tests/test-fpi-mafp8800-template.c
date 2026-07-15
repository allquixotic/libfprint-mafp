/*
 * MAFP8800 serialized-template validation tests
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include <glib.h>

#include "drivers/mafp8800-proto.h"
#include "drivers/mafp8800-template.h"

static guint8 *
valid_template_new (void)
{
  guint8 *data = g_malloc0 (MAFP8800_TEMPLATE_SIZE);
  guint8 *sample;
  guint8 *bank;
  guint32 count = GUINT32_TO_LE (1);

  mafp8800_template_initialize (data, MAFP8800_TEMPLATE_SIZE);
  mafp8800_template_set_sample_count (data, MAFP8800_TEMPLATE_SIZE, 1);
  sample = data + MAFP8800_TEMPLATE_HEADER_SIZE;
  sample[0] = MAFP8800_TEMPLATE_SAMPLE_MAGIC;
  bank = sample + 4;
  memcpy (bank, &count, sizeof (count));
  bank[4 + 16] = 42;
  bank[4 + 17] = 17;

  return data;
}

static void
assert_invalid (const guint8 *data, gsize size)
{
  g_autoptr(GError) error = NULL;
  guint sample_count = G_MAXUINT;

  g_assert_false (mafp8800_template_validate (data,
                                              size,
                                              &sample_count,
                                              &error));
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA);
  g_assert_cmpuint (sample_count, ==, 0);
}

static void
test_valid_template (void)
{
  g_autofree guint8 *data = valid_template_new ();

  g_autoptr(GError) error = NULL;
  guint sample_count = 0;

  g_assert_true (mafp8800_template_validate (data,
                                             MAFP8800_TEMPLATE_SIZE,
                                             &sample_count,
                                             &error));
  g_assert_no_error (error);
  g_assert_cmpuint (sample_count, ==, 1);
}

static void
test_invalid_sizes (void)
{
  g_autofree guint8 *data = g_malloc0 (MAFP8800_TEMPLATE_SIZE + 1);

  mafp8800_template_initialize (data, MAFP8800_TEMPLATE_SIZE);
  assert_invalid (data, MAFP8800_TEMPLATE_SIZE - 1);
  assert_invalid (data, MAFP8800_TEMPLATE_SIZE + 1);
}

static void
test_invalid_container_header (void)
{
  g_autofree guint8 *data = valid_template_new ();

  data[0] ^= 1;
  assert_invalid (data, MAFP8800_TEMPLATE_SIZE);
  data[0] ^= 1;
  data[1]++;
  assert_invalid (data, MAFP8800_TEMPLATE_SIZE);
  data[1]--;
  data[2] = 0;
  assert_invalid (data, MAFP8800_TEMPLATE_SIZE);
  data[2] = 1;
  data[3] = 1;
  assert_invalid (data, MAFP8800_TEMPLATE_SIZE);
}

static void
test_invalid_sample_header (void)
{
  g_autofree guint8 *data = valid_template_new ();
  guint8 *sample = data + MAFP8800_TEMPLATE_HEADER_SIZE;

  sample[0] ^= 1;
  assert_invalid (data, MAFP8800_TEMPLATE_SIZE);
  sample[0] ^= 1;
  sample[3] = 1;
  assert_invalid (data, MAFP8800_TEMPLATE_SIZE);
}

static void
test_invalid_keypoint_count (void)
{
  g_autofree guint8 *data = valid_template_new ();
  guint8 *bank = data + MAFP8800_TEMPLATE_HEADER_SIZE + 4;
  guint32 count = GUINT32_TO_LE (G_MAXUINT32);

  memcpy (bank, &count, sizeof (count));
  assert_invalid (data, MAFP8800_TEMPLATE_SIZE);
  count = GUINT32_TO_LE (MAFP8800_TEMPLATE_MAX_KEYPOINTS + 1);
  memcpy (bank, &count, sizeof (count));
  assert_invalid (data, MAFP8800_TEMPLATE_SIZE);
}

static void
test_invalid_keypoint_coordinate (void)
{
  g_autofree guint8 *data = valid_template_new ();
  guint8 *entry = data + MAFP8800_TEMPLATE_HEADER_SIZE + 4 + 4;

  entry[16] = 160;
  assert_invalid (data, MAFP8800_TEMPLATE_SIZE);
  entry[16] = 42;
  entry[17] = 36;
  assert_invalid (data, MAFP8800_TEMPLATE_SIZE);
}

static void
test_rejects_nonzero_unused_data (void)
{
  g_autofree guint8 *data = valid_template_new ();
  guint8 *first_unused_keypoint =
    data + MAFP8800_TEMPLATE_HEADER_SIZE + 4 + 4 +
    MAFP8800_TEMPLATE_KEYPOINT_SIZE;

  first_unused_keypoint[0] = 1;
  assert_invalid (data, MAFP8800_TEMPLATE_SIZE);
  first_unused_keypoint[0] = 0;
  data[MAFP8800_TEMPLATE_HEADER_SIZE + MAFP8800_TEMPLATE_SAMPLE_SIZE] = 1;
  assert_invalid (data, MAFP8800_TEMPLATE_SIZE);
}

static void
test_random_data_is_memory_safe (void)
{
  g_autoptr(GRand) random = g_rand_new_with_seed (0x544D504C);
  g_autofree guint8 *data = g_malloc (MAFP8800_TEMPLATE_SIZE);

  for (guint iteration = 0; iteration < 10000; iteration++)
    {
      g_autoptr(GError) error = NULL;
      guint sample_count = 0;

      for (gsize i = 0; i < MAFP8800_TEMPLATE_SIZE; i++)
        data[i] = (guint8) g_rand_int (random);

      if (mafp8800_template_validate (data,
                                      MAFP8800_TEMPLATE_SIZE,
                                      &sample_count,
                                      &error))
        g_assert_cmpuint (sample_count, <=, MAFP8800_TEMPLATE_MAX_SAMPLES);
      else
        g_assert_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA);
    }
}

static void
test_random_valid_templates (void)
{
  g_autoptr(GRand) random = g_rand_new_with_seed (0x56414C44);
  g_autofree guint8 *data = g_malloc (MAFP8800_TEMPLATE_SIZE);

  for (guint iteration = 0; iteration < 10000; iteration++)
    {
      g_autoptr(GError) error = NULL;
      guint expected_samples =
        g_rand_int_range (random, 1, MAFP8800_TEMPLATE_MAX_SAMPLES + 1);
      guint decoded_samples = 0;

      mafp8800_template_initialize (data, MAFP8800_TEMPLATE_SIZE);
      mafp8800_template_set_sample_count (data,
                                          MAFP8800_TEMPLATE_SIZE,
                                          expected_samples);

      for (guint sample_index = 0;
           sample_index < expected_samples;
           sample_index++)
        {
          guint8 *sample =
            data + MAFP8800_TEMPLATE_HEADER_SIZE +
            sample_index * MAFP8800_TEMPLATE_SAMPLE_SIZE;

          sample[0] = MAFP8800_TEMPLATE_SAMPLE_MAGIC;
          for (guint bank_index = 0;
               bank_index < MAFP8800_TEMPLATE_BANKS;
               bank_index++)
            {
              guint8 *bank =
                sample + 4 + bank_index * MAFP8800_TEMPLATE_BANK_SIZE;
              guint keypoint_count =
                g_rand_int_range (random,
                                  0,
                                  MAFP8800_TEMPLATE_MAX_KEYPOINTS + 1);
              guint32 encoded_count = GUINT32_TO_LE (keypoint_count);

              memcpy (bank, &encoded_count, sizeof (encoded_count));
              for (guint keypoint = 0;
                   keypoint < keypoint_count;
                   keypoint++)
                {
                  guint8 *entry =
                    bank + 4 + keypoint * MAFP8800_TEMPLATE_KEYPOINT_SIZE;

                  for (guint byte = 0;
                       byte < MAFP8800_TEMPLATE_DESCRIPTOR_SIZE;
                       byte++)
                    entry[byte] = (guint8) g_rand_int (random);
                  entry[16] = (guint8) g_rand_int_range (random,
                                                         0,
                                                         MAFP8800_FP36_ROWS);
                  entry[17] = (guint8) g_rand_int_range (
                    random,
                    0,
                    MAFP8800_FP36_ENHANCED_COLUMNS);
                  entry[18] = (guint8) g_rand_int (random);
                  entry[19] = (guint8) g_rand_int (random);
                }
            }
        }

      g_assert_true (mafp8800_template_validate (data,
                                                 MAFP8800_TEMPLATE_SIZE,
                                                 &decoded_samples,
                                                 &error));
      g_assert_no_error (error);
      g_assert_cmpuint (decoded_samples, ==, expected_samples);
    }
}

int
main (int argc, char *argv[])
{
  g_test_init (&argc, &argv, NULL);

  g_test_add_func ("/mafp8800/template/valid", test_valid_template);
  g_test_add_func ("/mafp8800/template/invalid-sizes", test_invalid_sizes);
  g_test_add_func ("/mafp8800/template/invalid-container-header",
                   test_invalid_container_header);
  g_test_add_func ("/mafp8800/template/invalid-sample-header",
                   test_invalid_sample_header);
  g_test_add_func ("/mafp8800/template/invalid-keypoint-count",
                   test_invalid_keypoint_count);
  g_test_add_func ("/mafp8800/template/invalid-keypoint-coordinate",
                   test_invalid_keypoint_coordinate);
  g_test_add_func ("/mafp8800/template/nonzero-unused-data",
                   test_rejects_nonzero_unused_data);
  g_test_add_func ("/mafp8800/template/random-data-memory-safe",
                   test_random_data_is_memory_safe);
  g_test_add_func ("/mafp8800/template/random-valid",
                   test_random_valid_templates);

  return g_test_run ();
}
