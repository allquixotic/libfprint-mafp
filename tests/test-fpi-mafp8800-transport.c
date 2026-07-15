/*
 * MAFP8800 asynchronous transport unit tests
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
#include "drivers/mafp8800-transport.h"
#include "test-device-fake.h"

#define TEST_FD 73
#define FP36_CHIP_ID 0x24

typedef struct
{
  guint8 raw[MAFP8800_FP36_RAW_SIZE];
  gsize  raw_offset;
  guint  raw_rows;

  guint id_ready_after;
  guint id_polls;
  guint reset_writes;
  guint register_index;
  guint flushes;
  guint image_calls;
  guint ioctl_calls;

  gboolean fail_image;
  gboolean calibration;
  guint8   current_gain;
} SpiFixture;

typedef struct
{
  GMainLoop *loop;
  GThread   *main_thread;
  GError    *error;
  gboolean   complete;
} CaptureResult;

static SpiFixture fixture;

static const guint8 expected_registers[][2] = {
  { 0x20, 0x8F },
  { 0x18, 0x37 },
  { 0x38, 0x02 },
  { 0x40, 0x00 },
  { 0x48, 0x25 },
  { 0x3C, 0x19 },
  { 0x44, 0xA1 },
};

static gsize
append_wire_row (guint8 *raw, gsize offset, guint row)
{
  raw[offset++] = 0x00;
  raw[offset++] = 0x00;
  raw[offset++] = 0x0A;
  raw[offset++] = 0x50 | (row & 0x0F);

  for (guint column = 0; column < MAFP8800_FP36_COLUMNS; column++)
    {
      guint16 value = (guint16) (row * MAFP8800_FP36_COLUMNS + column);

      if (fixture.calibration)
        value = (guint16) fixture.current_gain * 32;

      raw[offset++] = value >> 8;
      raw[offset++] = value & 0xFF;
    }

  return offset;
}

static void
fixture_reset (guint raw_rows, guint id_ready_after, gboolean fail_image)
{
  gsize offset = 11;

  memset (&fixture, 0, sizeof (fixture));
  memset (fixture.raw, 0xEE, sizeof (fixture.raw));
  fixture.raw_rows = raw_rows;
  fixture.id_ready_after = id_ready_after;
  fixture.fail_image = fail_image;

  for (guint row = 0; row < raw_rows; row++)
    {
      offset = append_wire_row (fixture.raw, offset, row);
      offset += row % 3;
    }

  g_assert_cmpuint (offset, <, sizeof (fixture.raw));
}

static void
fixture_reset_calibration (guint raw_rows)
{
  fixture_reset (raw_rows, 1, FALSE);
  fixture.calibration = TRUE;
}

static void
fixture_build_calibration_raw (void)
{
  gsize offset = 11;

  memset (fixture.raw, 0xEE, sizeof (fixture.raw));
  for (guint row = 0; row < fixture.raw_rows; row++)
    {
      offset = append_wire_row (fixture.raw, offset, row);
      offset += row % 3;
    }
  g_assert_cmpuint (offset, <, sizeof (fixture.raw));
}

int __wrap_ioctl (int fd, unsigned long request, ...);

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

  g_assert_cmpint (fd, ==, TEST_FD);
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
        case 0x8C:
          g_assert_cmphex (write_buffer[1], ==, 0xFF);
          fixture.reset_writes++;
          fixture.raw_offset = 0;
          break;

        case 0x04:
          g_assert_cmphex (write_buffer[1], ==, 0x00);
          fixture.id_polls++;
          if (fixture.id_polls >= fixture.id_ready_after)
            read_buffer[2] = FP36_CHIP_ID;
          break;

        default:
          {
            guint register_in_sequence =
              fixture.register_index % G_N_ELEMENTS (expected_registers);

            g_assert_cmphex (write_buffer[0], ==,
                             expected_registers[register_in_sequence][0]);
            if (register_in_sequence == 1)
              {
                fixture.current_gain = write_buffer[1];
                if (!fixture.calibration)
                  g_assert_cmphex (write_buffer[1], ==, 0x37);
              }
            else if (register_in_sequence == 5)
              g_assert_cmphex (write_buffer[1], ==,
                               fixture.calibration ? 0x4C : 0x19);
            else if (register_in_sequence == 6)
              g_assert_cmphex (write_buffer[1], ==,
                               fixture.calibration ? 0x54 : 0xA1);
            else
              g_assert_cmphex (write_buffer[1], ==,
                               expected_registers[register_in_sequence][1]);
            fixture.register_index++;
            break;
          }
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
          if (fixture.calibration)
            fixture_build_calibration_raw ();
        }
      else
        g_assert_cmphex (write_buffer[0], ==, 0xFF);

      fixture.image_calls++;
      if (fixture.fail_image)
        {
          errno = EIO;
          return -1;
        }

      memcpy (read_buffer, fixture.raw + fixture.raw_offset, transfer->len);
      fixture.raw_offset += transfer->len;
      g_assert_cmpint (transfer->cs_change, ==,
                       fixture.raw_offset < sizeof (fixture.raw));
    }

  return transfer->len;
}

static void
capture_complete_cb (FpiSsm   *ssm,
                     FpDevice *device,
                     GError   *error)
{
  CaptureResult *result = g_object_get_data (G_OBJECT (device),
                                             "mafp-capture-result");

  g_assert_true (g_thread_self () == result->main_thread);
  result->complete = TRUE;
  result->error = g_steal_pointer (&error);
  g_main_loop_quit (result->loop);
}

static void
run_capture (guint          raw_rows,
             guint          id_ready_after,
             gboolean       fail_image,
             gboolean       cancel_before_start,
             guint8        *frame,
             CaptureResult *result)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  g_autoptr(GCancellable) cancellable = g_cancellable_new ();
  FpiSsm *ssm;

  fixture_reset (raw_rows, id_ready_after, fail_image);
  memset (frame, 0xA5, MAFP8800_FP36_FRAME_SIZE);
  g_object_set_data (G_OBJECT (device), "mafp-capture-result", result);

  if (cancel_before_start)
    g_cancellable_cancel (cancellable);

  ssm = mafp8800_fp36_capture_new (device,
                                   TEST_FD,
                                   cancellable,
                                   0x37,
                                   0x19,
                                   0xA1,
                                   frame,
                                   MAFP8800_FP36_FRAME_SIZE);
  g_assert_nonnull (ssm);
  fpi_ssm_start (ssm, capture_complete_cb);
  if (!result->complete)
    g_main_loop_run (result->loop);
}

static void
run_gain_calibration (guint          raw_rows,
                      guint8        *gain,
                      CaptureResult *result)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  g_autoptr(GCancellable) cancellable = g_cancellable_new ();
  FpiSsm *ssm;

  fixture_reset_calibration (raw_rows);
  g_object_set_data (G_OBJECT (device), "mafp-capture-result", result);

  ssm = mafp8800_fp36_calibrate_gain_new (device,
                                          TEST_FD,
                                          cancellable,
                                          gain);
  g_assert_nonnull (ssm);
  fpi_ssm_start (ssm, capture_complete_cb);
  if (!result->complete)
    g_main_loop_run (result->loop);
}

static void
assert_frame_clear (const guint8 *frame)
{
  for (gsize i = 0; i < MAFP8800_FP36_FRAME_SIZE; i++)
    g_assert_cmphex (frame[i], ==, 0x00);
}

static void
test_capture_success (void)
{
  g_autofree guint8 *frame = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  g_autoptr(GMainLoop) loop = g_main_loop_new (NULL, FALSE);
  CaptureResult result = {
    .loop = loop,
    .main_thread = g_thread_self (),
  };

  run_capture (MAFP8800_FP36_ROWS, 3, FALSE, FALSE, frame, &result);

  g_assert_no_error (result.error);
  g_assert_cmpuint (fixture.reset_writes, ==, 1);
  g_assert_cmpuint (fixture.id_polls, ==, 3);
  g_assert_cmpuint (fixture.register_index, ==,
                    G_N_ELEMENTS (expected_registers));
  g_assert_cmpuint (fixture.flushes, ==, 1);
  g_assert_cmpuint (fixture.image_calls, >, 0);
  g_assert_cmpuint (fixture.raw_offset, ==, sizeof (fixture.raw));

  for (guint row = 0; row < MAFP8800_FP36_ROWS; row++)
    for (guint column = 0; column < MAFP8800_FP36_COLUMNS; column++)
      {
        gsize offset = ((gsize) row * MAFP8800_FP36_COLUMNS + column) * 2;
        guint16 expected = (guint16) (row * MAFP8800_FP36_COLUMNS + column);

        g_assert_cmphex (frame[offset], ==, expected & 0xFF);
        g_assert_cmphex (frame[offset + 1], ==, expected >> 8);
      }
}

static void
test_capture_incomplete_frame (void)
{
  g_autofree guint8 *frame = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  g_autoptr(GMainLoop) loop = g_main_loop_new (NULL, FALSE);
  CaptureResult result = {
    .loop = loop,
    .main_thread = g_thread_self (),
  };

  run_capture (MAFP8800_FP36_ROWS - 1, 1, FALSE, FALSE, frame, &result);

  g_assert_error (result.error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA);
  g_clear_error (&result.error);
  assert_frame_clear (frame);
}

static void
test_capture_ioctl_error (void)
{
  g_autofree guint8 *frame = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  g_autoptr(GMainLoop) loop = g_main_loop_new (NULL, FALSE);
  CaptureResult result = {
    .loop = loop,
    .main_thread = g_thread_self (),
  };

  run_capture (MAFP8800_FP36_ROWS, 1, TRUE, FALSE, frame, &result);

  g_assert_error (result.error, G_IO_ERROR, G_IO_ERROR_FAILED);
  g_clear_error (&result.error);
  assert_frame_clear (frame);
}

static void
test_capture_reset_timeout (void)
{
  g_autofree guint8 *frame = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  g_autoptr(GMainLoop) loop = g_main_loop_new (NULL, FALSE);
  CaptureResult result = {
    .loop = loop,
    .main_thread = g_thread_self (),
  };

  run_capture (MAFP8800_FP36_ROWS, G_MAXUINT, FALSE, FALSE, frame, &result);

  g_assert_error (result.error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT);
  g_clear_error (&result.error);
  g_assert_cmpuint (fixture.id_polls, ==, 20);
  g_assert_cmpuint (fixture.register_index, ==, 0);
  g_assert_cmpuint (fixture.image_calls, ==, 0);
  assert_frame_clear (frame);
}

static void
test_capture_pre_cancelled (void)
{
  g_autofree guint8 *frame = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  g_autoptr(GMainLoop) loop = g_main_loop_new (NULL, FALSE);
  CaptureResult result = {
    .loop = loop,
    .main_thread = g_thread_self (),
  };

  run_capture (MAFP8800_FP36_ROWS, 1, FALSE, TRUE, frame, &result);

  g_assert_error (result.error, G_IO_ERROR, G_IO_ERROR_CANCELLED);
  g_clear_error (&result.error);
  g_assert_cmpuint (fixture.ioctl_calls, ==, 0);
  assert_frame_clear (frame);
}

static void
test_gain_calibration_success (void)
{
  g_autoptr(GMainLoop) loop = g_main_loop_new (NULL, FALSE);
  CaptureResult result = {
    .loop = loop,
    .main_thread = g_thread_self (),
  };
  guint8 gain = 0;

  run_gain_calibration (MAFP8800_FP36_ROWS, &gain, &result);

  g_assert_no_error (result.error);
  g_assert_cmpuint (gain, ==, 28);
  g_assert_cmpuint (fixture.reset_writes, ==, 8);
  g_assert_cmpuint (fixture.id_polls, ==, 8);
  g_assert_cmpuint (fixture.register_index, ==,
                    8 * G_N_ELEMENTS (expected_registers));
  g_assert_cmpuint (fixture.flushes, ==, 8);
  g_assert_cmpuint (fixture.image_calls, >=, 8);
}

static void
test_gain_calibration_rejects_incomplete_frame (void)
{
  g_autoptr(GMainLoop) loop = g_main_loop_new (NULL, FALSE);
  CaptureResult result = {
    .loop = loop,
    .main_thread = g_thread_self (),
  };
  guint8 gain = 0xFF;

  run_gain_calibration (MAFP8800_FP36_ROWS - 1, &gain, &result);

  g_assert_error (result.error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA);
  g_clear_error (&result.error);
  g_assert_cmpuint (gain, ==, 0);
  g_assert_cmpuint (fixture.reset_writes, ==, 1);
  g_assert_cmpuint (fixture.register_index, ==,
                    G_N_ELEMENTS (expected_registers));
}

int
main (int argc, char *argv[])
{
  g_test_init (&argc, &argv, NULL);

  g_test_add_func ("/mafp8800/transport/capture/success",
                   test_capture_success);
  g_test_add_func ("/mafp8800/transport/capture/incomplete-frame",
                   test_capture_incomplete_frame);
  g_test_add_func ("/mafp8800/transport/capture/ioctl-error",
                   test_capture_ioctl_error);
  g_test_add_func ("/mafp8800/transport/capture/reset-timeout",
                   test_capture_reset_timeout);
  g_test_add_func ("/mafp8800/transport/capture/pre-cancelled",
                   test_capture_pre_cancelled);
  g_test_add_func ("/mafp8800/transport/gain/success",
                   test_gain_calibration_success);
  g_test_add_func ("/mafp8800/transport/gain/incomplete-frame",
                   test_gain_calibration_rejects_incomplete_frame);

  return g_test_run ();
}
