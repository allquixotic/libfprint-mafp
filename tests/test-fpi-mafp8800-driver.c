/*
 * MAFP8800 full driver lifecycle tests
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include <errno.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include <glib.h>

#include "drivers/mafp8800-proto.h"
#include "drivers/mafp8800-template.h"
#include "fp-device.h"
#include "fpi-print.h"

#define FP36_CHIP_ID 0x24
#define OPEN_CALIBRATION_FRAMES 8
#define OPEN_BACKGROUND_FRAMES 1
#define TEST_BACKGROUND_VALUE 1000
#define TEST_TEXTURED_FINGER G_MAXUINT16
#define TEST_DISTINCT_FINGER (G_MAXUINT16 - 1)
#define TEST_SCRIPT_CAPACITY 64

GType fpi_device_mafp8800_get_type (void);

typedef struct
{
  guint8   raw[MAFP8800_FP36_RAW_SIZE];
  gsize    raw_offset;

  guint    calibration_frames;
  guint    background_frames;
  guint    completed_frames;
  guint    ioctl_calls;
  guint    reset_writes;
  guint    id_reads;
  guint    register_writes;
  guint    flushes;

  guint8   current_gain;
  guint16  script[TEST_SCRIPT_CAPACITY];
  guint    script_len;
  guint    script_index;
  gboolean fail_next_image;
} DriverFixture;

typedef struct
{
  guint next_stage;
  guint calls;
} ProgressResult;

static DriverFixture fixture;

static void
fixture_reset_for_open (void)
{
  memset (&fixture, 0, sizeof (fixture));
  fixture.calibration_frames = OPEN_CALIBRATION_FRAMES;
  fixture.background_frames = OPEN_BACKGROUND_FRAMES;
}

static void
fixture_set_script (const guint16 *values, guint values_len)
{
  g_assert_cmpuint (fixture.calibration_frames, ==, 0);
  g_assert_cmpuint (fixture.background_frames, ==, 0);
  g_assert_nonnull (values);
  g_assert_cmpuint (values_len, >, 0);
  g_assert_cmpuint (values_len, <=, G_N_ELEMENTS (fixture.script));

  memcpy (fixture.script, values, values_len * sizeof (values[0]));
  fixture.script_len = values_len;
  fixture.script_index = 0;
}

static guint16
fixture_current_value (void)
{
  if (fixture.calibration_frames > 0)
    return (guint16) fixture.current_gain * 32;
  if (fixture.background_frames > 0)
    return TEST_BACKGROUND_VALUE;

  g_assert_cmpuint (fixture.script_len, >, 0);
  return fixture.script[MIN (fixture.script_index,
                             fixture.script_len - 1)];
}

static void
fixture_build_raw (void)
{
  const guint16 value = fixture_current_value ();
  gsize offset = 11;

  memset (fixture.raw, 0xee, sizeof (fixture.raw));
  for (guint row = 0; row < MAFP8800_FP36_ROWS; row++)
    {
      fixture.raw[offset++] = 0x00;
      fixture.raw[offset++] = 0x00;
      fixture.raw[offset++] = 0x0a;
      fixture.raw[offset++] = 0x50 | (row & 0x0f);
      for (guint column = 0; column < MAFP8800_FP36_COLUMNS; column++)
        {
          guint16 pixel = value;

          if (value == TEST_TEXTURED_FINGER ||
              value == TEST_DISTINCT_FINGER)
            {
              const gboolean distinct = value == TEST_DISTINCT_FINGER;
              const guint row_period = distinct ? 20 : 24;
              const guint column_period = distinct ? 14 : 12;
              const guint row_in_cell =
                (row + (distinct ? 7 : 0)) % row_period;
              const guint column_in_cell =
                (column + (distinct ? 3 : 0)) % column_period;
              const guint row_distance = MIN (row_in_cell,
                                              row_period - row_in_cell);
              const guint column_distance = MIN (column_in_cell,
                                                 column_period -
                                                 column_in_cell);
              const guint distance_squared =
                row_distance * row_distance +
                column_distance * column_distance;
              const guint local_texture =
                ((row * (distinct ? 191 : 73)) ^
                 (column * (distinct ? 47 : 151)) ^
                 (row * column * (distinct ? 11 : 7))) % 70;

              /* Smooth, separated spots survive the driver's scale-space
               * blur and produce deterministic extrema in both enrollment
               * and verification captures. */
              pixel = 360 + MIN (distance_squared * 12, 180) +
                      local_texture;
            }
          fixture.raw[offset++] = pixel >> 8;
          fixture.raw[offset++] = pixel & 0xff;
        }
      offset += row % 3;
    }

  g_assert_cmpuint (offset, <, sizeof (fixture.raw));
}

static void
fixture_complete_frame (void)
{
  fixture.completed_frames++;
  if (fixture.calibration_frames > 0)
    fixture.calibration_frames--;
  else if (fixture.background_frames > 0)
    fixture.background_frames--;
  else if (fixture.script_index < fixture.script_len)
    fixture.script_index++;
}

int __wrap_ioctl (int           fd,
                  unsigned long request,
                  ...);

int
__wrap_ioctl (int fd, unsigned long request, ...)
{
  struct spi_ioc_transfer *transfer;
  guint8 *write_buffer;
  guint8 *read_buffer;
  va_list args;

  va_start (args, request);
  transfer = va_arg (args, struct spi_ioc_transfer *);
  va_end (args);

  g_assert_cmpint (fd, >=, 0);
  g_assert_cmpuint (request, ==, SPI_IOC_MESSAGE (1));
  g_assert_nonnull (transfer);
  g_assert_cmpuint (transfer->tx_buf, !=, 0);
  g_assert_cmpuint (transfer->rx_buf, !=, 0);

  write_buffer = (guint8 *) (guintptr) transfer->tx_buf;
  read_buffer = (guint8 *) (guintptr) transfer->rx_buf;
  memset (read_buffer, 0, transfer->len);
  fixture.ioctl_calls++;

  if (transfer->len == 4)
    {
      switch (write_buffer[0])
        {
        case 0x8c:
          g_assert_cmphex (write_buffer[1], ==, 0xff);
          fixture.reset_writes++;
          fixture.raw_offset = 0;
          break;

        case 0x04:
          g_assert_cmphex (write_buffer[1], ==, 0x00);
          read_buffer[2] = FP36_CHIP_ID;
          fixture.id_reads++;
          break;

        default:
          if (write_buffer[0] == 0x18)
            fixture.current_gain = write_buffer[1];
          fixture.register_writes++;
          break;
        }
    }
  else if (transfer->len == 0x26)
    {
      g_assert_cmphex (write_buffer[0], ==, 0x78);
      fixture.flushes++;
    }
  else
    {
      g_assert_cmpuint (fixture.raw_offset + transfer->len,
                        <=,
                        sizeof (fixture.raw));
      if (fixture.raw_offset == 0)
        {
          g_assert_cmphex (write_buffer[0], ==, 0x70);
          if (fixture.fail_next_image)
            {
              fixture.fail_next_image = FALSE;
              errno = EIO;
              return -1;
            }
          fixture_build_raw ();
        }
      else
        {
          g_assert_cmphex (write_buffer[0], ==, 0xff);
        }

      memcpy (read_buffer,
              fixture.raw + fixture.raw_offset,
              transfer->len);
      fixture.raw_offset += transfer->len;
      g_assert_cmpint (transfer->cs_change, ==,
                       fixture.raw_offset < sizeof (fixture.raw));
      if (fixture.raw_offset == sizeof (fixture.raw))
        fixture_complete_frame ();
    }

  return transfer->len;
}

static FpDevice *
new_test_device (void)
{
  return g_object_new (fpi_device_mafp8800_get_type (),
                       "fpi-udev-data-spidev", "/dev/null",
                       NULL);
}

static gboolean
cancel_once (gpointer user_data)
{
  g_cancellable_cancel (G_CANCELLABLE (user_data));
  return G_SOURCE_REMOVE;
}

static void
progress_cb (FpDevice *device,
             gint      completed_stages,
             FpPrint  *print,
             gpointer  user_data,
             GError   *error)
{
  ProgressResult *result = user_data;

  g_assert_nonnull (device);
  g_assert_null (print);
  g_assert_no_error (error);
  g_assert_cmpint (completed_stages, ==, result->next_stage);
  result->next_stage++;
  result->calls++;
}

static void
fill_enrollment_script (guint16 *values)
{
  for (guint stage = 0; stage < MAFP8800_TEMPLATE_MAX_SAMPLES; stage++)
    {
      const guint offset = stage * 4;

      values[offset] = TEST_TEXTURED_FINGER;
      values[offset + 1] = TEST_TEXTURED_FINGER;
      values[offset + 2] = TEST_BACKGROUND_VALUE;
      values[offset + 3] = TEST_BACKGROUND_VALUE;
    }
}

static void
test_open_enroll_verify_cancel_close (void)
{
  guint16 enrollment_values[MAFP8800_TEMPLATE_MAX_SAMPLES * 4];
  static const guint16 verify_values[] = {
    TEST_TEXTURED_FINGER,
    TEST_TEXTURED_FINGER,
    TEST_BACKGROUND_VALUE,
    TEST_BACKGROUND_VALUE,
  };
  static const guint16 absent_values[] = {
    TEST_BACKGROUND_VALUE,
  };
  static const guint16 mismatch_values[] = {
    TEST_DISTINCT_FINGER,
    TEST_DISTINCT_FINGER,
    TEST_BACKGROUND_VALUE,
    TEST_BACKGROUND_VALUE,
  };
  static const guint8 invalid_template_data[] = {
    MAFP8800_TEMPLATE_MAGIC,
    MAFP8800_TEMPLATE_VERSION,
    1,
    0,
  };

  g_autoptr(FpDevice) device = NULL;
  g_autoptr(FpPrint) template_print = NULL;
  g_autoptr(FpPrint) enrolled_print = NULL;
  g_autoptr(FpPrint) restored_print = NULL;
  g_autoptr(FpPrint) invalid_print = NULL;
  g_autoptr(FpPrint) identified_print = NULL;
  g_autoptr(GVariant) variant = NULL;
  g_autoptr(GVariant) invalid_variant = NULL;
  g_autoptr(GPtrArray) gallery = NULL;
  g_autoptr(GError) error = NULL;
  g_autoptr(GCancellable) cancellable = NULL;
  g_autofree guchar *serialized_print = NULL;
  ProgressResult progress = {
    .next_stage = 1,
  };
  gboolean matched = TRUE;
  guint sample_count = 0;
  gsize serialized_size = 0;

  fixture_reset_for_open ();
  device = new_test_device ();
  g_assert_true (fp_device_open_sync (device, NULL, &error));
  g_assert_no_error (error);
  g_assert_true (fp_device_is_open (device));
  g_assert_cmpuint (fixture.completed_frames, ==,
                    OPEN_CALIBRATION_FRAMES + OPEN_BACKGROUND_FRAMES);
  g_assert_cmpuint (fixture.calibration_frames, ==, 0);
  g_assert_cmpuint (fixture.background_frames, ==, 0);

  fill_enrollment_script (enrollment_values);
  fixture_set_script (enrollment_values,
                      G_N_ELEMENTS (enrollment_values));
  template_print = fp_print_new (device);
  fp_print_set_finger (template_print, FP_FINGER_LEFT_INDEX);
  fp_print_set_username (template_print, "mafp8800-test-user");
  enrolled_print = fp_device_enroll_sync (device,
                                          template_print,
                                          NULL,
                                          progress_cb,
                                          &progress,
                                          &error);
  if (enrolled_print == template_print)
    g_steal_pointer (&template_print);
  g_assert_no_error (error);
  g_assert_nonnull (enrolled_print);
  g_assert_cmpuint (progress.calls, ==, MAFP8800_TEMPLATE_MAX_SAMPLES);
  g_assert_cmpuint (fixture.script_index, ==,
                    G_N_ELEMENTS (enrollment_values));

  g_object_get (enrolled_print, "fpi-data", &variant, NULL);
  g_assert_nonnull (variant);
  {
    gsize size = 0;
    const guint8 *data = g_variant_get_fixed_array (variant,
                                                    &size,
                                                    sizeof (guint8));

    g_assert_true (mafp8800_template_validate (data,
                                               size,
                                               &sample_count,
                                               &error));
  }
  g_assert_no_error (error);
  g_assert_cmpuint (sample_count, ==, MAFP8800_TEMPLATE_MAX_SAMPLES);

  g_assert_true (fp_print_serialize (enrolled_print,
                                     &serialized_print,
                                     &serialized_size,
                                     &error));
  g_assert_no_error (error);
  g_assert_cmpuint (serialized_size, >, 3);
  restored_print = fp_print_deserialize (serialized_print,
                                         serialized_size,
                                         &error);
  g_assert_no_error (error);
  g_assert_nonnull (restored_print);
  g_assert_true (fp_print_equal (enrolled_print, restored_print));
  g_assert_true (fp_print_compatible (restored_print, device));
  g_assert_cmpint (fp_print_get_finger (restored_print), ==,
                   FP_FINGER_LEFT_INDEX);
  g_assert_cmpstr (fp_print_get_username (restored_print), ==,
                   "mafp8800-test-user");
  g_clear_object (&enrolled_print);
  enrolled_print = g_steal_pointer (&restored_print);
  memset (serialized_print, 0, serialized_size);
  g_clear_pointer (&serialized_print, g_free);
  serialized_size = 0;

  for (guint attempt = 0; attempt < 10; attempt++)
    {
      fixture_set_script (verify_values, G_N_ELEMENTS (verify_values));
      g_assert_true (fp_device_verify_sync (device,
                                            enrolled_print,
                                            NULL,
                                            NULL,
                                            NULL,
                                            &matched,
                                            NULL,
                                            &error));
      g_assert_no_error (error);
      g_assert_true (matched);
      g_assert_cmpuint (fixture.script_index, ==,
                        G_N_ELEMENTS (verify_values));
      g_assert_cmpuint (fp_device_get_finger_status (device), ==,
                        FP_FINGER_STATUS_NONE);
    }

  for (guint attempt = 0; attempt < 10; attempt++)
    {
      fixture_set_script (mismatch_values, G_N_ELEMENTS (mismatch_values));
      g_assert_true (fp_device_verify_sync (device,
                                            enrolled_print,
                                            NULL,
                                            NULL,
                                            NULL,
                                            &matched,
                                            NULL,
                                            &error));
      g_assert_no_error (error);
      g_assert_false (matched);
      g_assert_cmpuint (fixture.script_index, ==,
                        G_N_ELEMENTS (mismatch_values));
      g_assert_cmpuint (fp_device_get_finger_status (device), ==,
                        FP_FINGER_STATUS_NONE);
    }

  invalid_print = fp_print_new (device);
  fpi_print_set_type (invalid_print, FPI_PRINT_RAW);
  invalid_variant = g_variant_ref_sink (
    g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE,
                               invalid_template_data,
                               G_N_ELEMENTS (invalid_template_data),
                               sizeof (guint8)));
  g_object_set (invalid_print, "fpi-data", invalid_variant, NULL);

  gallery = g_ptr_array_new ();
  g_ptr_array_add (gallery, invalid_print);
  g_ptr_array_add (gallery, enrolled_print);
  fixture_set_script (verify_values, G_N_ELEMENTS (verify_values));
  g_assert_true (fp_device_identify_sync (device,
                                          gallery,
                                          NULL,
                                          NULL,
                                          NULL,
                                          &identified_print,
                                          NULL,
                                          &error));
  g_assert_no_error (error);
  g_assert_true (identified_print == enrolled_print);

  fixture_set_script (verify_values, G_N_ELEMENTS (verify_values));
  g_assert_false (fp_device_verify_sync (device,
                                         invalid_print,
                                         NULL,
                                         NULL,
                                         NULL,
                                         &matched,
                                         NULL,
                                         &error));
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA);
  g_clear_error (&error);

  fixture_set_script (absent_values, G_N_ELEMENTS (absent_values));
  cancellable = g_cancellable_new ();
  g_timeout_add_full (G_PRIORITY_DEFAULT,
                      10,
                      cancel_once,
                      g_object_ref (cancellable),
                      g_object_unref);
  g_assert_false (fp_device_verify_sync (device,
                                         enrolled_print,
                                         cancellable,
                                         NULL,
                                         NULL,
                                         &matched,
                                         NULL,
                                         &error));
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_CANCELLED);
  g_clear_error (&error);

  g_assert_true (fp_device_close_sync (device, NULL, &error));
  g_assert_no_error (error);
  g_assert_false (fp_device_is_open (device));
}

static void
test_open_image_error_releases_device (void)
{
  g_autoptr(FpDevice) device = NULL;
  g_autoptr(GError) error = NULL;

  fixture_reset_for_open ();
  fixture.fail_next_image = TRUE;
  device = new_test_device ();
  g_assert_false (fp_device_open_sync (device, NULL, &error));
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_FAILED);
  g_assert_false (fp_device_is_open (device));
}

int
main (int argc, char **argv)
{
  g_test_init (&argc, &argv, NULL);

  g_test_add_func ("/mafp8800/driver/open-enroll-verify-identify-cancel-close",
                   test_open_enroll_verify_cancel_close);
  g_test_add_func ("/mafp8800/driver/open-image-error-releases-device",
                   test_open_image_error_releases_device);

  return g_test_run ();
}
