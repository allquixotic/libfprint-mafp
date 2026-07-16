/*
 * FpiSpiTransfer unit tests
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

#include "fpi-spi-transfer.h"
#include "test-device-fake.h"

#define TEST_FD 41
#define TEST_MASK 0xa5

typedef struct
{
  GMainLoop *loop;
  gint       expected_error;
} TransferTestData;

typedef struct
{
  gsize    length;
  gint     expected_error;
  gboolean expect_split;
  gboolean cancel_after_first;
} TransferTestParams;

typedef struct
{
  gsize length;
  gsize transferred;
  guint calls;
} ConcurrentTransferState;

typedef struct
{
  GMainLoop *loop;
  guint      remaining;
} ConcurrentResult;

static gboolean ioctl_error;
static guint ioctl_calls;
static gsize expected_length;
static gsize transferred_length;
static gboolean cancel_after_first;
static GCancellable *active_cancellable;
static gboolean concurrent_mode;
static ConcurrentTransferState concurrent_states[2];

int __wrap_ioctl (int           fd,
                  unsigned long request,
                  ...);

int
__wrap_ioctl (int fd, unsigned long request, ...)
{
  struct spi_ioc_transfer *xfer;
  va_list args;
  guint8 *buffer_wr;
  guint8 *buffer_rd;

  va_start (args, request);
  xfer = va_arg (args, struct spi_ioc_transfer *);
  va_end (args);

  if (concurrent_mode)
    {
      const guint index = fd - TEST_FD;
      ConcurrentTransferState *state;

      g_assert_cmpuint (index, <, G_N_ELEMENTS (concurrent_states));
      state = &concurrent_states[index];
      g_assert_cmpuint (request, ==, SPI_IOC_MESSAGE (1));
      g_assert_nonnull (xfer);
      g_assert_cmpuint (xfer[0].len, >, 0);
      g_assert_cmpuint (xfer[0].tx_buf, !=, 0);
      g_assert_cmpuint (xfer[0].rx_buf, !=, 0);
      g_assert_cmpint (xfer[0].cs_change, ==,
                       state->transferred + xfer[0].len < state->length);

      buffer_wr = (guint8 *) (guintptr) xfer[0].tx_buf;
      buffer_rd = (guint8 *) (guintptr) xfer[0].rx_buf;
      for (guint i = 0; i < xfer[0].len; i++)
        buffer_rd[i] = buffer_wr[i] ^ TEST_MASK;
      state->calls++;
      state->transferred += xfer[0].len;
      return xfer[0].len;
    }

  g_assert_cmpint (fd, ==, TEST_FD);
  g_assert_cmpuint (request, ==, SPI_IOC_MESSAGE (1));
  g_assert_nonnull (xfer);
  g_assert_cmpuint (xfer[0].len, >, 0);
  g_assert_cmpuint (xfer[0].tx_buf, !=, 0);
  g_assert_cmpuint (xfer[0].rx_buf, !=, 0);
  g_assert_cmpint (xfer[0].cs_change, ==,
                   transferred_length + xfer[0].len < expected_length);

  ioctl_calls++;
  if (ioctl_error)
    {
      errno = EIO;
      return -1;
    }

  buffer_wr = (guint8 *) (guintptr) xfer[0].tx_buf;
  buffer_rd = (guint8 *) (guintptr) xfer[0].rx_buf;
  for (guint i = 0; i < xfer[0].len; i++)
    buffer_rd[i] = buffer_wr[i] ^ TEST_MASK;
  transferred_length += xfer[0].len;

  if (cancel_after_first && ioctl_calls == 1)
    g_cancellable_cancel (active_cancellable);

  return xfer[0].len;
}

static void
concurrent_done_cb (FpiSpiTransfer *transfer,
                    FpDevice       *device,
                    gpointer        user_data,
                    GError         *error)
{
  ConcurrentResult *result = user_data;

  g_assert_true (FP_IS_DEVICE (device));
  g_assert_no_error (error);
  for (gsize index = 0; index < transfer->length_rd; index++)
    g_assert_cmphex (transfer->buffer_rd[index], ==,
                     transfer->buffer_wr[index] ^ TEST_MASK);

  g_assert_cmpuint (result->remaining, >, 0);
  result->remaining--;
  if (result->remaining == 0)
    g_main_loop_quit (result->loop);
}

static void
test_duplex_async_concurrent (void)
{
  const gsize length = (gsize) G_MAXUINT16 + 1;

  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  g_autoptr(GMainLoop) loop = g_main_loop_new (NULL, FALSE);
  ConcurrentResult result = {
    .loop = loop,
    .remaining = G_N_ELEMENTS (concurrent_states),
  };

  memset (concurrent_states, 0, sizeof (concurrent_states));
  concurrent_mode = TRUE;
  for (guint index = 0; index < G_N_ELEMENTS (concurrent_states); index++)
    {
      FpiSpiTransfer *transfer =
        fpi_spi_transfer_new (device, TEST_FD + index);

      concurrent_states[index].length = length;
      fpi_spi_transfer_duplex (transfer, length);
      fpi_spi_transfer_submit (transfer,
                               NULL,
                               concurrent_done_cb,
                               &result);
    }

  /* Each in-flight transfer, rather than its GTask worker, owns the device
   * until the corresponding main-context callback has returned. */
  g_clear_object (&device);
  g_main_loop_run (loop);
  concurrent_mode = FALSE;

  for (guint index = 0; index < G_N_ELEMENTS (concurrent_states); index++)
    {
      g_assert_cmpuint (concurrent_states[index].calls, >, 1);
      g_assert_cmpuint (concurrent_states[index].transferred, ==, length);
    }
}

static void
transfer_done_cb (FpiSpiTransfer *transfer,
                  FpDevice       *device,
                  gpointer        user_data,
                  GError         *error)
{
  TransferTestData *data = user_data;

  g_assert_true (FP_IS_DEVICE (device));
  g_assert_nonnull (transfer);

  if (data->expected_error >= 0)
    {
      g_assert_error (error, G_IO_ERROR, data->expected_error);
    }
  else
    {
      g_assert_no_error (error);
      for (gsize i = 0; i < transfer->length_rd; i++)
        g_assert_cmphex (transfer->buffer_rd[i], ==,
                         transfer->buffer_wr[i] ^ TEST_MASK);
    }

  g_clear_error (&error);
  g_main_loop_quit (data->loop);
}

static void
test_duplex_async (gconstpointer user_data)
{
  const TransferTestParams *params = user_data;

  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  g_autoptr(FpiSpiTransfer) transfer = NULL;
  g_autoptr(GCancellable) cancellable = g_cancellable_new ();
  g_autoptr(GMainLoop) loop = g_main_loop_new (NULL, FALSE);
  TransferTestData data = {
    .loop = loop,
    .expected_error = params->expected_error,
  };

  ioctl_error = data.expected_error == G_IO_ERROR_FAILED;
  ioctl_calls = 0;
  expected_length = params->length;
  transferred_length = 0;
  cancel_after_first = params->cancel_after_first;
  active_cancellable = cancellable;

  transfer = fpi_spi_transfer_new (device, TEST_FD);
  fpi_spi_transfer_duplex (transfer, params->length);
  transfer->buffer_wr[0] = 0x24;
  transfer->buffer_wr[1] = 0x12;
  transfer->buffer_wr[2] = 0x80;
  transfer->buffer_wr[3] = 0x08;

  fpi_spi_transfer_submit (g_steal_pointer (&transfer),
                           cancellable,
                           transfer_done_cb,
                           &data);
  g_main_loop_run (loop);

  active_cancellable = NULL;

  if (params->cancel_after_first)
    g_assert_cmpuint (ioctl_calls, ==, 1);
  else if (params->expect_split)
    g_assert_cmpuint (ioctl_calls, >, 1);
  else
    g_assert_cmpuint (ioctl_calls, ==, 1);
}

int
main (int argc, char *argv[])
{
  static const TransferTestParams success = {
    .length = 4,
    .expected_error = -1,
  };
  static const TransferTestParams split = {
    .length = (gsize) G_MAXUINT16 + 1,
    .expected_error = -1,
    .expect_split = TRUE,
  };
  static const TransferTestParams error = {
    .length = 4,
    .expected_error = G_IO_ERROR_FAILED,
  };
  static const TransferTestParams cancel = {
    .length = (gsize) G_MAXUINT16 + 1,
    .expected_error = G_IO_ERROR_CANCELLED,
    .cancel_after_first = TRUE,
  };

  g_test_init (&argc, &argv, NULL);

  g_test_add_data_func ("/spi-transfer/duplex/async/success",
                        &success,
                        test_duplex_async);
  g_test_add_data_func ("/spi-transfer/duplex/async/split",
                        &split,
                        test_duplex_async);
  g_test_add_data_func ("/spi-transfer/duplex/async/error",
                        &error,
                        test_duplex_async);
  g_test_add_data_func ("/spi-transfer/duplex/async/cancel-between-chunks",
                        &cancel,
                        test_duplex_async);
  g_test_add_func ("/spi-transfer/duplex/async/concurrent",
                   test_duplex_async_concurrent);

  return g_test_run ();
}
