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
  gboolean   expect_error;
} TransferTestData;

typedef struct
{
  gsize    length;
  gboolean expect_error;
  gboolean expect_split;
} TransferTestParams;

static gboolean ioctl_error;
static guint ioctl_calls;
static gsize expected_length;
static gsize transferred_length;

int __wrap_ioctl (int fd, unsigned long request, ...);

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

  return xfer[0].len;
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

  if (data->expect_error)
    g_assert_error (error, G_IO_ERROR, G_IO_ERROR_FAILED);
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
  g_autoptr(GMainLoop) loop = g_main_loop_new (NULL, FALSE);
  TransferTestData data = {
    .loop = loop,
    .expect_error = params->expect_error,
  };

  ioctl_error = data.expect_error;
  ioctl_calls = 0;
  expected_length = params->length;
  transferred_length = 0;

  transfer = fpi_spi_transfer_new (device, TEST_FD);
  fpi_spi_transfer_duplex (transfer, params->length);
  transfer->buffer_wr[0] = 0x24;
  transfer->buffer_wr[1] = 0x12;
  transfer->buffer_wr[2] = 0x80;
  transfer->buffer_wr[3] = 0x08;

  fpi_spi_transfer_submit (g_steal_pointer (&transfer),
                           NULL,
                           transfer_done_cb,
                           &data);
  g_main_loop_run (loop);

  if (params->expect_split)
    g_assert_cmpuint (ioctl_calls, >, 1);
  else
    g_assert_cmpuint (ioctl_calls, ==, 1);
}

int
main (int argc, char *argv[])
{
  static const TransferTestParams success = {
    .length = 4,
  };
  static const TransferTestParams split = {
    .length = (gsize) G_MAXUINT16 + 1,
    .expect_split = TRUE,
  };
  static const TransferTestParams error = {
    .length = 4,
    .expect_error = TRUE,
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

  return g_test_run ();
}
