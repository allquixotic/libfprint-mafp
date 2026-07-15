/*
 * Versioned MAFP8800 host-template format
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include "mafp8800-proto.h"
#include "mafp8800-template.h"

void
mafp8800_template_initialize (guint8 *data, gsize size)
{
  g_return_if_fail (data != NULL);
  g_return_if_fail (size >= MAFP8800_TEMPLATE_SIZE);

  memset (data, 0, MAFP8800_TEMPLATE_SIZE);
  data[0] = MAFP8800_TEMPLATE_MAGIC;
  data[1] = MAFP8800_TEMPLATE_VERSION;
}

void
mafp8800_template_set_sample_count (guint8 *data,
                                    gsize   size,
                                    guint   sample_count)
{
  g_return_if_fail (data != NULL);
  g_return_if_fail (size >= MAFP8800_TEMPLATE_SIZE);
  g_return_if_fail (data[0] == MAFP8800_TEMPLATE_MAGIC);
  g_return_if_fail (data[1] == MAFP8800_TEMPLATE_VERSION);
  g_return_if_fail (sample_count <= MAFP8800_TEMPLATE_MAX_SAMPLES);

  data[2] = (guint8) sample_count;
}

static gboolean
bytes_are_zero (const guint8 *data, gsize size)
{
  for (gsize i = 0; i < size; i++)
    if (data[i] != 0)
      return FALSE;

  return TRUE;
}

static gboolean
validate_sample (const guint8 *sample, guint sample_index, GError **error)
{
  if (sample[0] != MAFP8800_TEMPLATE_SAMPLE_MAGIC ||
      sample[1] != 0 || sample[2] != 0 || sample[3] != 0)
    {
      g_set_error (error,
                   G_IO_ERROR,
                   G_IO_ERROR_INVALID_DATA,
                   "Invalid MAFP8800 sample %u header",
                   sample_index);
      return FALSE;
    }

  for (guint bank = 0; bank < MAFP8800_TEMPLATE_BANKS; bank++)
    {
      const guint8 *bank_data =
        sample + 4 + bank * MAFP8800_TEMPLATE_BANK_SIZE;
      const guint8 *keypoints = bank_data + 4;
      guint32 encoded_keypoint_count;
      guint keypoint_count;

      memcpy (&encoded_keypoint_count,
              bank_data,
              sizeof (encoded_keypoint_count));
      keypoint_count = GUINT32_FROM_LE (encoded_keypoint_count);
      if (keypoint_count > MAFP8800_TEMPLATE_MAX_KEYPOINTS)
        {
          g_set_error (error,
                       G_IO_ERROR,
                       G_IO_ERROR_INVALID_DATA,
                       "Invalid MAFP8800 sample %u bank %u keypoint count %u",
                       sample_index,
                       bank,
                       keypoint_count);
          return FALSE;
        }

      for (guint keypoint = 0; keypoint < keypoint_count; keypoint++)
        {
          const guint8 *entry =
            keypoints + keypoint * MAFP8800_TEMPLATE_KEYPOINT_SIZE;

          if (entry[16] >= MAFP8800_FP36_ROWS ||
              entry[17] >= MAFP8800_FP36_ENHANCED_COLUMNS)
            {
              g_set_error (error,
                           G_IO_ERROR,
                           G_IO_ERROR_INVALID_DATA,
                           "Invalid MAFP8800 sample %u bank %u keypoint %u "
                           "coordinate (%u,%u)",
                           sample_index,
                           bank,
                           keypoint,
                           entry[16],
                           entry[17]);
              return FALSE;
            }
        }

      if (!bytes_are_zero (
            keypoints + keypoint_count * MAFP8800_TEMPLATE_KEYPOINT_SIZE,
            MAFP8800_TEMPLATE_BANK_DATA_SIZE -
            keypoint_count * MAFP8800_TEMPLATE_KEYPOINT_SIZE))
        {
          g_set_error (error,
                       G_IO_ERROR,
                       G_IO_ERROR_INVALID_DATA,
                       "Non-zero unused data in MAFP8800 sample %u bank %u",
                       sample_index,
                       bank);
          return FALSE;
        }
    }

  return TRUE;
}

gboolean
mafp8800_template_validate (const guint8 *data,
                            gsize         size,
                            guint        *sample_count,
                            GError      **error)
{
  guint count;

  g_return_val_if_fail (data != NULL, FALSE);
  g_return_val_if_fail (error == NULL || *error == NULL, FALSE);

  if (sample_count)
    *sample_count = 0;

  if (size != MAFP8800_TEMPLATE_SIZE)
    {
      g_set_error (error,
                   G_IO_ERROR,
                   G_IO_ERROR_INVALID_DATA,
                   "Invalid MAFP8800 template size %" G_GSIZE_FORMAT
                   " (expected %u)",
                   size,
                   MAFP8800_TEMPLATE_SIZE);
      return FALSE;
    }

  if (data[0] != MAFP8800_TEMPLATE_MAGIC ||
      data[1] != MAFP8800_TEMPLATE_VERSION ||
      data[3] != 0)
    {
      g_set_error (error,
                   G_IO_ERROR,
                   G_IO_ERROR_INVALID_DATA,
                   "Unsupported MAFP8800 template header %02x/%u/%02x",
                   data[0],
                   data[1],
                   data[3]);
      return FALSE;
    }

  count = data[2];
  if (count == 0 || count > MAFP8800_TEMPLATE_MAX_SAMPLES)
    {
      g_set_error (error,
                   G_IO_ERROR,
                   G_IO_ERROR_INVALID_DATA,
                   "Invalid MAFP8800 template sample count %u",
                   count);
      return FALSE;
    }

  for (guint sample_index = 0; sample_index < count; sample_index++)
    {
      const guint8 *sample =
        data + MAFP8800_TEMPLATE_HEADER_SIZE +
        sample_index * MAFP8800_TEMPLATE_SAMPLE_SIZE;

      if (!validate_sample (sample, sample_index, error))
        return FALSE;
    }

  if (!bytes_are_zero (
        data + MAFP8800_TEMPLATE_HEADER_SIZE +
        count * MAFP8800_TEMPLATE_SAMPLE_SIZE,
        (MAFP8800_TEMPLATE_MAX_SAMPLES - count) *
        MAFP8800_TEMPLATE_SAMPLE_SIZE))
    {
      g_set_error_literal (error,
                           G_IO_ERROR,
                           G_IO_ERROR_INVALID_DATA,
                           "Non-zero unused MAFP8800 template samples");
      return FALSE;
    }

  if (sample_count)
    *sample_count = count;
  return TRUE;
}
