/*
 * FPrint SPI transfer handling
 * Copyright (C) 2019-2020 Benjamin Berg <bberg@redhat.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "fpi-spi-transfer.h"
#include "fpi-log.h"
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <errno.h>

/* spidev can only handle the specified block size, which defaults to 4096. */
#define SPIDEV_BLOCK_SIZE_PARAM "/sys/module/spidev/parameters/bufsiz"
#define SPIDEV_BLOCK_SIZE_FALLBACK 4096
static gsize block_size = 0;

/**
 * SECTION:fpi-spi-transfer
 * @title: SPI transfer helpers
 * @short_description: Helpers to ease SPI transfers
 *
 * #FpiSpiTransfer is a structure to simplify the SPI transfer handling
 * for the linux spidev device. The main goal are to ease memory management
 * and provide a usable asynchronous API to libfprint drivers.
 *
 * Full-duplex transfers and transfers with a write and subsequent read are
 * supported.
 *
 * Drivers should always use this API rather than calling read/write/ioctl on
 * the spidev device.
 *
 * Setting %G_MESSAGES_DEBUG and %FP_DEBUG_TRANSFER will result in the message
 * content to be dumped.
 */


G_DEFINE_BOXED_TYPE (FpiSpiTransfer, fpi_spi_transfer, fpi_spi_transfer_ref, fpi_spi_transfer_unref)

static void
log_transfer (FpiSpiTransfer *transfer, gboolean submit, GError *error)
{
  if (fpi_log_is_debug_transfer_enabled ())
    {
      if (submit)
        {
          g_debug ("Transfer %p submitted, write length %zd, read length %zd",
                   transfer,
                   transfer->length_wr,
                   transfer->length_rd);

          if (transfer->buffer_wr)
            fp_dbg_hex_dump_data (transfer->buffer_wr, transfer->length_wr);
        }
      else
        {
          g_autofree gchar *error_str = NULL;
          if (error)
            error_str = g_strdup_printf ("with error (%s)", error->message);
          else
            error_str = g_strdup ("successfully");

          g_debug ("Transfer %p completed %s, write length %zd, read length %zd",
                   transfer,
                   error_str,
                   transfer->length_wr,
                   transfer->length_rd);
          if (transfer->buffer_rd)
            fp_dbg_hex_dump_data (transfer->buffer_rd, transfer->length_rd);
        }
    }
}

/**
 * fpi_spi_transfer_new:
 * @device: The #FpDevice the transfer is for
 * @spidev_fd: The file descriptor for the spidev device
 *
 * Creates a new #FpiSpiTransfer.
 *
 * Returns: (transfer full): A newly created #FpiSpiTransfer
 */
FpiSpiTransfer *
fpi_spi_transfer_new (FpDevice * device, int spidev_fd)
{
  FpiSpiTransfer *self;

  g_assert (FP_IS_DEVICE (device));

  if (g_once_init_enter (&block_size))
    {
      g_autoptr(GError) error = NULL;
      g_autofree char *contents = NULL;
      gsize detected_block_size = SPIDEV_BLOCK_SIZE_FALLBACK;

      if (g_file_get_contents (SPIDEV_BLOCK_SIZE_PARAM, &contents, NULL, &error))
        {
          detected_block_size = MIN (g_ascii_strtoull (contents, NULL, 0), G_MAXUINT16);
          if (detected_block_size == 0)
            {
              detected_block_size = SPIDEV_BLOCK_SIZE_FALLBACK;
              g_warning ("spidev blocksize could not be decoded, using %" G_GSIZE_FORMAT,
                         detected_block_size);
            }
        }
      else
        {
          g_message ("Failed to read spidev block size, using %" G_GSIZE_FORMAT,
                     detected_block_size);
        }

      g_once_init_leave (&block_size, detected_block_size);
    }

  self = g_slice_new0 (FpiSpiTransfer);
  self->ref_count = 1;

  /* Purely to enhance the debug log output. */
  self->length_wr = -1;
  self->length_rd = -1;

  self->device = device;
  self->spidev_fd = spidev_fd;

  return self;
}

static void
fpi_spi_transfer_free (FpiSpiTransfer *self)
{
  g_assert (self);
  g_assert_cmpint (g_atomic_int_get (&self->ref_count), ==, 0);

  if (self->free_buffer_wr && self->buffer_wr)
    self->free_buffer_wr (self->buffer_wr);
  if (self->free_buffer_rd && self->buffer_rd)
    self->free_buffer_rd (self->buffer_rd);
  self->buffer_wr = NULL;
  self->buffer_rd = NULL;

  g_slice_free (FpiSpiTransfer, self);
}

/**
 * fpi_spi_transfer_ref:
 * @self: A #FpiSpiTransfer
 *
 * Increments the reference count of @self by one.
 *
 * Returns: (transfer full): @self
 */
FpiSpiTransfer *
fpi_spi_transfer_ref (FpiSpiTransfer *self)
{
  g_return_val_if_fail (self, NULL);
  g_return_val_if_fail (g_atomic_int_get (&self->ref_count) > 0, NULL);

  g_atomic_int_inc (&self->ref_count);

  return self;
}

/**
 * fpi_spi_transfer_unref:
 * @self: A #FpiSpiTransfer
 *
 * Decrements the reference count of @self by one, freeing the structure when
 * the reference count reaches zero.
 */
void
fpi_spi_transfer_unref (FpiSpiTransfer *self)
{
  g_return_if_fail (self);
  g_return_if_fail (g_atomic_int_get (&self->ref_count) > 0);

  if (g_atomic_int_dec_and_test (&self->ref_count))
    fpi_spi_transfer_free (self);
}

/**
 * fpi_spi_transfer_write:
 * @transfer: The #FpiSpiTransfer
 * @length: The buffer size to allocate
 *
 * Prepare the write part of an SPI transfer allocating a new buffer
 * internally that will be free'ed automatically.
 */
void
fpi_spi_transfer_write (FpiSpiTransfer *transfer,
                        gsize           length)
{
  fpi_spi_transfer_write_full (transfer,
                               g_malloc0 (length),
                               length,
                               g_free);
}

/**
 * fpi_spi_transfer_write_full:
 * @transfer: The #FpiSpiTransfer
 * @buffer: The data to write.
 * @length: The size of @buffer
 * @free_func: (destroy buffer): Destroy notify for @buffer
 *
 * Prepare the write part of an SPI transfer.
 */
void
fpi_spi_transfer_write_full (FpiSpiTransfer *transfer,
                             guint8         *buffer,
                             gsize           length,
                             GDestroyNotify  free_func)
{
  g_assert (buffer != NULL);
  g_return_if_fail (transfer);

  /* Write is always before read, so ensure both are NULL. */
  g_return_if_fail (!transfer->full_duplex);
  g_return_if_fail (transfer->buffer_wr == NULL);
  g_return_if_fail (transfer->buffer_rd == NULL);

  transfer->buffer_wr = buffer;
  transfer->length_wr = length;
  transfer->free_buffer_wr = free_func;
}

/**
 * fpi_spi_transfer_read:
 * @transfer: The #FpiSpiTransfer
 * @length: The buffer size to allocate
 *
 * Prepare the read part of an SPI transfer allocating a new buffer
 * internally that will be free'ed automatically.
 */
void
fpi_spi_transfer_read (FpiSpiTransfer *transfer,
                       gsize           length)
{
  fpi_spi_transfer_read_full (transfer,
                              g_malloc0 (length),
                              length,
                              g_free);
}

/**
 * fpi_spi_transfer_read_full:
 * @transfer: The #FpiSpiTransfer
 * @buffer: Buffer to read data into.
 * @length: The size of @buffer
 * @free_func: (destroy buffer): Destroy notify for @buffer
 *
 * Prepare the read part of an SPI transfer.
 */
void
fpi_spi_transfer_read_full (FpiSpiTransfer *transfer,
                            guint8         *buffer,
                            gsize           length,
                            GDestroyNotify  free_func)
{
  g_assert (buffer != NULL);
  g_return_if_fail (transfer);
  g_return_if_fail (!transfer->full_duplex);
  g_return_if_fail (transfer->buffer_rd == NULL);

  transfer->buffer_rd = buffer;
  transfer->length_rd = length;
  transfer->free_buffer_rd = free_func;
}

/**
 * fpi_spi_transfer_duplex:
 * @transfer: The #FpiSpiTransfer
 * @length: The buffer size to allocate
 *
 * Prepare a full-duplex SPI transfer, allocating separate write and read
 * buffers internally. Both buffers are freed automatically.
 */
void
fpi_spi_transfer_duplex (FpiSpiTransfer *transfer,
                         gsize           length)
{
  guint8 *buffer_wr = g_malloc0 (length);
  guint8 *buffer_rd = g_malloc0 (length);

  fpi_spi_transfer_duplex_full (transfer,
                                buffer_wr,
                                buffer_rd,
                                length,
                                g_free,
                                g_free);
}

/**
 * fpi_spi_transfer_duplex_full:
 * @transfer: The #FpiSpiTransfer
 * @buffer_wr: The data to write.
 * @buffer_rd: Buffer to read data into.
 * @length: The size of both buffers
 * @free_func_wr: (destroy buffer_wr): Destroy notify for @buffer_wr
 * @free_func_rd: (destroy buffer_rd): Destroy notify for @buffer_rd
 *
 * Prepare a full-duplex SPI transfer. The write and read buffers must be
 * separate, equally sized allocations.
 */
void
fpi_spi_transfer_duplex_full (FpiSpiTransfer *transfer,
                              guint8         *buffer_wr,
                              guint8         *buffer_rd,
                              gsize           length,
                              GDestroyNotify  free_func_wr,
                              GDestroyNotify  free_func_rd)
{
  g_assert (buffer_wr != NULL);
  g_assert (buffer_rd != NULL);
  g_return_if_fail (transfer);
  g_return_if_fail (buffer_wr != buffer_rd);
  g_return_if_fail (transfer->buffer_wr == NULL);
  g_return_if_fail (transfer->buffer_rd == NULL);

  transfer->full_duplex = TRUE;
  transfer->buffer_wr = buffer_wr;
  transfer->length_wr = length;
  transfer->free_buffer_wr = free_func_wr;
  transfer->buffer_rd = buffer_rd;
  transfer->length_rd = length;
  transfer->free_buffer_rd = free_func_rd;
}

static void
transfer_finish_cb (GObject *source_object, GAsyncResult *res, gpointer user_data)
{
  GTask *task = G_TASK (res);

  g_autoptr(FpiSpiTransfer) transfer =
    fpi_spi_transfer_ref (g_task_get_task_data (task));
  GError *error = NULL;
  FpiSpiTransferCallback callback;

  g_return_if_fail (transfer != NULL);

  /* GTask may dispatch the main-context callback just before its worker-side
   * return helper finishes.  Wait for the final release so that it covers the
   * result buffers, error object, and GTask's completion-source handoff. */
  while (g_atomic_int_get (&transfer->worker_complete) == 0)
    g_thread_yield ();

  g_task_propagate_boolean (task, &error);

  log_transfer (transfer, FALSE, error);

  callback = transfer->callback;
  transfer->callback = NULL;
  g_atomic_int_set (&transfer->worker_complete, 0);
  g_atomic_int_set (&transfer->submitted, 0);
  callback (transfer, transfer->device, transfer->user_data, error);
}

static int
transfer_chunk (FpiSpiTransfer *transfer, gsize full_length, gsize *transferred)
{
  struct spi_ioc_transfer xfer[2] = { 0 };
  gsize skip = *transferred;
  gsize len = 0;
  int transfers = 0;
  int status;

  if (transfer->full_duplex)
    {
      g_assert_cmpint (transfer->length_wr, ==, transfer->length_rd);
      g_assert_cmpuint (skip, <, transfer->length_wr);

      xfer[0].tx_buf = (gsize) transfer->buffer_wr + skip;
      xfer[0].rx_buf = (gsize) transfer->buffer_rd + skip;
      xfer[0].len = MIN (block_size, transfer->length_wr - skip);
      len = xfer[0].len;
      transfers = 1;
    }
  else
    {
      if (transfer->buffer_wr &&
          skip < transfer->length_wr && len < block_size)
        {
          g_assert_cmpuint (transfers, <, G_N_ELEMENTS (xfer));
          xfer[transfers].tx_buf = (gsize) transfer->buffer_wr + skip;
          xfer[transfers].len = MIN (block_size, transfer->length_wr - skip);

          len += xfer[transfers].len;
          skip += xfer[transfers].len;

          transfers += 1;
        }

      if (transfer->buffer_wr)
        {
          /* A partially transferred write means the read phase cannot start
           * in this chunk.  Avoid relying on unsigned underflow to encode
           * that state. */
          skip = skip >= transfer->length_wr ?
                 skip - transfer->length_wr :
                 G_MAXSIZE;
        }

      if (transfer->buffer_rd &&
          skip < transfer->length_rd && len < block_size)
        {
          g_assert_cmpuint (transfers, <, G_N_ELEMENTS (xfer));
          xfer[transfers].rx_buf = (gsize) transfer->buffer_rd + skip;
          xfer[transfers].len = MIN (block_size, transfer->length_rd - skip);

          len += xfer[transfers].len;
          /* skip += xfer[transfers].len; */

          transfers += 1;
        }
    }

  g_assert (transfers > 0);

  /* We have not transferred everything; ask driver to not deselect the chip.
   * Unfortunately, this is inherently racy in case there are further devices
   * on the same bus. In practice, it is hopefully unlikely to be an issue,
   * but print a message once to help with debugging.
   */
  if (full_length > *transferred + len)
    {
      static gboolean warned = FALSE;

      if (!warned)
        {
          g_message ("Split SPI transfer. In case of issues, try increasing the spidev buffer size.");
          warned = TRUE;
        }

      xfer[transfers - 1].cs_change = TRUE;
    }

  /* This ioctl cannot be interrupted. */
  status = ioctl (transfer->spidev_fd, SPI_IOC_MESSAGE (transfers), xfer);

  if (status >= 0)
    *transferred += len;

  return status;
}

static void
transfer_thread_func (GTask        *task,
                      gpointer      source_object,
                      gpointer      task_data,
                      GCancellable *cancellable)
{
  FpiSpiTransfer *transfer = (FpiSpiTransfer *) task_data;
  gsize full_length;
  gsize transferred = 0;
  int status = 0;

  /* Pair with the release in fpi_spi_transfer_submit().  GTask owns the
   * transfer after submission; callers must not mutate it until completion. */
  g_assert_cmpint (g_atomic_int_get (&transfer->submitted), ==, 1);

  if (transfer->buffer_wr == NULL && transfer->buffer_rd == NULL)
    {
      GError *error = g_error_new_literal (G_IO_ERROR,
                                           G_IO_ERROR_INVALID_ARGUMENT,
                                           "Transfer with neither write or read!");

      g_task_return_error (task, error);
      g_atomic_int_set (&transfer->worker_complete, 1);
      return;
    }

  if (transfer->full_duplex)
    {
      g_assert (transfer->buffer_wr != NULL);
      g_assert (transfer->buffer_rd != NULL);
      g_assert_cmpint (transfer->length_wr, ==, transfer->length_rd);
      full_length = transfer->length_wr;
    }
  else
    {
      full_length = 0;
      if (transfer->buffer_wr)
        full_length += transfer->length_wr;
      if (transfer->buffer_rd)
        full_length += transfer->length_rd;
    }

  while (transferred < full_length && status >= 0)
    {
      /* An ioctl in progress cannot be interrupted, but cancellation must
       * prevent additional chunks from being submitted. */
      if (cancellable && g_cancellable_is_cancelled (cancellable))
        {
          GError *error = g_error_new_literal (G_IO_ERROR,
                                               G_IO_ERROR_CANCELLED,
                                               "SPI transfer was cancelled");

          g_task_return_error (task, error);
          g_atomic_int_set (&transfer->worker_complete, 1);
          return;
        }

      status = transfer_chunk (transfer, full_length, &transferred);
    }

  if (status < 0)
    {
      int saved_errno = errno;
      GError *error = g_error_new (G_IO_ERROR,
                                   g_io_error_from_errno (saved_errno),
                                   "Error invoking ioctl for SPI transfer (%d)",
                                   saved_errno);

      g_task_return_error (task, error);
      /* Publish the error, ioctl output, and completion-source handoff. */
      g_atomic_int_set (&transfer->worker_complete, 1);
    }
  else
    {
      g_task_return_boolean (task, TRUE);
      /* Publish ioctl output and the completion-source handoff. */
      g_atomic_int_set (&transfer->worker_complete, 1);
    }
}

/**
 * fpi_spi_transfer_submit:
 * @transfer: (transfer full): The transfer to submit, must have been filled.
 * @cancellable: Cancellable to use, e.g. fpi_device_get_cancellable()
 * @callback: Callback on completion or error
 * @user_data: Data to pass to callback
 *
 * Submit an SPI transfer with a specific timeout and callback functions.
 *
 * The underlying transfer cannot be cancelled. The current implementation
 * will only call @callback after the transfer has been completed.
 *
 * Note that #FpiSpiTransfer will be stolen when this function is called.
 * So that all associated data will be free'ed automatically, after the
 * callback ran unless fpi_usb_transfer_ref() is explicitly called.
 */
void
fpi_spi_transfer_submit (FpiSpiTransfer        *transfer,
                         GCancellable          *cancellable,
                         FpiSpiTransferCallback callback,
                         gpointer               user_data)
{
  g_autoptr(GTask) task = NULL;

  g_return_if_fail (transfer);
  g_return_if_fail (callback);

  /* Recycling is allowed, but not two at the same time. */
  g_return_if_fail (transfer->callback == NULL);

  transfer->callback = callback;
  transfer->user_data = user_data;
  g_assert_cmpint (g_atomic_int_get (&transfer->submitted), ==, 0);
  g_assert_cmpint (g_atomic_int_get (&transfer->worker_complete), ==, 0);

  log_transfer (transfer, TRUE, NULL);

  task = g_task_new (transfer->device,
                     cancellable,
                     transfer_finish_cb,
                     NULL);
  /* Publish all initialized fields before handing ownership to GTask. */
  g_atomic_int_set (&transfer->submitted, 1);
  g_task_set_task_data (task,
                        g_steal_pointer (&transfer),
                        (GDestroyNotify) fpi_spi_transfer_unref);

  g_task_run_in_thread (task, transfer_thread_func);
}

/**
 * fpi_spi_transfer_submit_sync:
 * @transfer: The transfer to submit, must have been filled.
 * @error: Location to store #GError to
 *
 * Synchronously submit an SPI transfer. Use of this function is discouraged
 * as it will block all other operations in the application.
 *
 * Note that you still need to fpi_spi_transfer_unref() the
 * #FpiSpiTransfer afterwards.
 *
 * Returns: #TRUE on success, otherwise #FALSE and @error will be set
 */
gboolean
fpi_spi_transfer_submit_sync (FpiSpiTransfer *transfer,
                              GError        **error)
{
  g_autoptr(GTask) task = NULL;
  GError *err = NULL;
  gboolean res;

  g_return_val_if_fail (transfer, FALSE);

  /* Recycling is allowed, but not two at the same time. */
  g_return_val_if_fail (transfer->callback == NULL, FALSE);

  log_transfer (transfer, TRUE, NULL);

  task = g_task_new (transfer->device,
                     NULL,
                     NULL,
                     NULL);
  g_task_set_task_data (task,
                        fpi_spi_transfer_ref (transfer),
                        (GDestroyNotify) fpi_spi_transfer_unref);

  g_task_run_in_thread_sync (task, transfer_thread_func);

  res = g_task_propagate_boolean (task, &err);

  log_transfer (transfer, FALSE, err);

  g_propagate_error (error, err);

  return res;
}
