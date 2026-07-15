/*
 * Asynchronous Microarray MAFP8800 transport for libfprint
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#define FP_COMPONENT "mafp8800"

#include "drivers_api.h"
#include "mafp8800-proto.h"
#include "mafp8800-transport.h"

#define MAFP_CHIP_ID_FP36 0x24
#define MAFP_RESET_ATTEMPTS 20
#define MAFP_REGISTER_TRANSFER_SIZE 4
#define MAFP_FLUSH_TRANSFER_SIZE 0x26

typedef struct
{
  guint8 address;
  guint8 value;
} MafpRegister;

typedef struct
{
  int           spi_fd;
  GCancellable *cancellable;
  guint8        *frame;
  gsize          frame_size;

  guint8 gain;
  guint8 integration;
  guint8 dac;

  guint reset_attempts;
  guint register_index;
  guint8 chip_id;
} MafpCaptureContext;

enum mafp_capture_state {
  MAFP_CAPTURE_RESET,
  MAFP_CAPTURE_RESET_WAIT,
  MAFP_CAPTURE_READ_ID,
  MAFP_CAPTURE_CHECK_ID,
  MAFP_CAPTURE_WRITE_REGISTER,
  MAFP_CAPTURE_ADVANCE_REGISTER,
  MAFP_CAPTURE_FLUSH,
  MAFP_CAPTURE_READ_IMAGE,
  MAFP_CAPTURE_COMPLETE,
  MAFP_CAPTURE_NUM_STATES,
};

static void
mafp_capture_context_free (MafpCaptureContext *context)
{
  g_clear_object (&context->cancellable);
  g_free (context);
}

static FpiSpiTransfer *
mafp_register_transfer_new (FpDevice *device,
                            int       spi_fd,
                            guint8    address,
                            guint8    value)
{
  FpiSpiTransfer *transfer = fpi_spi_transfer_new (device, spi_fd);

  fpi_spi_transfer_duplex (transfer, MAFP_REGISTER_TRANSFER_SIZE);
  transfer->buffer_wr[0] = address;
  transfer->buffer_wr[1] = value;

  return transfer;
}

static void
mafp_capture_read_id_cb (FpiSpiTransfer *transfer,
                         FpDevice       *device,
                         gpointer        user_data,
                         GError         *error)
{
  MafpCaptureContext *context = fpi_ssm_get_data (transfer->ssm);

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, g_steal_pointer (&error));
      return;
    }

  context->chip_id = transfer->buffer_rd[2];
  fpi_ssm_next_state (transfer->ssm);
}

static void
mafp_capture_image_cb (FpiSpiTransfer *transfer,
                       FpDevice       *device,
                       gpointer        user_data,
                       GError         *error)
{
  MafpCaptureContext *context = fpi_ssm_get_data (transfer->ssm);
  g_autoptr(GError) parse_error = NULL;
  guint parsed_rows = 0;

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, g_steal_pointer (&error));
      return;
    }

  if (!mafp8800_parse_fp36_rows (transfer->buffer_rd,
                                 transfer->length_rd,
                                 context->frame,
                                 context->frame_size,
                                 MAFP8800_FP36_ROWS,
                                 &parsed_rows,
                                 &parse_error))
    {
      fpi_ssm_mark_failed (transfer->ssm, g_steal_pointer (&parse_error));
      return;
    }

  fp_dbg ("Decoded complete FP36 frame (%u rows)", parsed_rows);
  fpi_ssm_next_state (transfer->ssm);
}

static void
mafp_capture_handler (FpiSsm *ssm, FpDevice *device)
{
  MafpCaptureContext *context = fpi_ssm_get_data (ssm);
  FpiSpiTransfer *transfer;
  MafpRegister registers[] = {
    { 0x20, 0x8F },
    { 0x18, context->gain },
    { 0x38, 0x02 },
    { 0x40, 0x00 },
    { 0x48, 0x25 },
    { 0x3C, context->integration },
    { 0x44, context->dac },
  };

  if (context->cancellable && g_cancellable_is_cancelled (context->cancellable))
    {
      fpi_ssm_mark_failed (ssm,
                           g_error_new_literal (G_IO_ERROR,
                                                G_IO_ERROR_CANCELLED,
                                                "FP36 capture cancelled"));
      return;
    }

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case MAFP_CAPTURE_RESET:
      context->reset_attempts = 0;
      context->register_index = 0;
      transfer = mafp_register_transfer_new (device,
                                             context->spi_fd,
                                             0x8C,
                                             0xFF);
      transfer->ssm = ssm;
      fpi_spi_transfer_submit (transfer,
                               context->cancellable,
                               fpi_ssm_spi_transfer_cb,
                               NULL);
      return;

    case MAFP_CAPTURE_RESET_WAIT:
      fpi_ssm_next_state_delayed (ssm, 1);
      return;

    case MAFP_CAPTURE_READ_ID:
      transfer = mafp_register_transfer_new (device,
                                             context->spi_fd,
                                             0x04,
                                             0x00);
      transfer->ssm = ssm;
      fpi_spi_transfer_submit (transfer,
                               context->cancellable,
                               mafp_capture_read_id_cb,
                               NULL);
      return;

    case MAFP_CAPTURE_CHECK_ID:
      if (context->chip_id == MAFP_CHIP_ID_FP36)
        {
          fpi_ssm_next_state (ssm);
          return;
        }

      context->reset_attempts++;
      if (context->reset_attempts >= MAFP_RESET_ATTEMPTS)
        {
          fpi_ssm_mark_failed (
            ssm,
            g_error_new (G_IO_ERROR,
                         G_IO_ERROR_TIMED_OUT,
                         "FP36 reset timed out after %u chip-ID polls "
                         "(last response 0x%02x)",
                         context->reset_attempts,
                         context->chip_id));
          return;
        }

      fpi_ssm_jump_to_state (ssm, MAFP_CAPTURE_RESET_WAIT);
      return;

    case MAFP_CAPTURE_WRITE_REGISTER:
      g_assert_cmpuint (context->register_index, <, G_N_ELEMENTS (registers));
      transfer = mafp_register_transfer_new (
        device,
        context->spi_fd,
        registers[context->register_index].address,
        registers[context->register_index].value);
      transfer->ssm = ssm;
      fpi_spi_transfer_submit (transfer,
                               context->cancellable,
                               fpi_ssm_spi_transfer_cb,
                               NULL);
      return;

    case MAFP_CAPTURE_ADVANCE_REGISTER:
      context->register_index++;
      if (context->register_index < G_N_ELEMENTS (registers))
        fpi_ssm_jump_to_state (ssm, MAFP_CAPTURE_WRITE_REGISTER);
      else
        fpi_ssm_next_state (ssm);
      return;

    case MAFP_CAPTURE_FLUSH:
      transfer = fpi_spi_transfer_new (device, context->spi_fd);
      fpi_spi_transfer_duplex (transfer, MAFP_FLUSH_TRANSFER_SIZE);
      transfer->buffer_wr[0] = 0x78;
      transfer->ssm = ssm;
      fpi_spi_transfer_submit (transfer,
                               context->cancellable,
                               fpi_ssm_spi_transfer_cb,
                               NULL);
      return;

    case MAFP_CAPTURE_READ_IMAGE:
      transfer = fpi_spi_transfer_new (device, context->spi_fd);
      fpi_spi_transfer_duplex (transfer, MAFP8800_FP36_RAW_SIZE);
      memset (transfer->buffer_wr, 0xFF, transfer->length_wr);
      transfer->buffer_wr[0] = 0x70;
      transfer->ssm = ssm;
      fpi_spi_transfer_submit (transfer,
                               context->cancellable,
                               mafp_capture_image_cb,
                               NULL);
      return;

    case MAFP_CAPTURE_COMPLETE:
      fpi_ssm_mark_completed (ssm);
      return;

    default:
      g_assert_not_reached ();
    }
}

/**
 * mafp8800_fp36_capture_new:
 * @device: device associated with the transfer
 * @spi_fd: open spidev file descriptor
 * @cancellable: (nullable): cancellation object for the operation
 * @gain: capture gain register value
 * @integration: capture integration register value
 * @dac: capture DAC register value
 * @frame: caller-owned decoded frame destination
 * @frame_size: size of @frame
 *
 * Builds a bounded asynchronous reset, configure, flush, and full-frame read
 * state machine. @frame must remain valid until the state machine completion
 * callback runs. It is cleared before I/O begins and remains clear after any
 * transfer, cancellation, or parser failure.
 *
 * Returns: (transfer full): a new, not-yet-started state machine
 */
FpiSsm *
mafp8800_fp36_capture_new (FpDevice     *device,
                           int           spi_fd,
                           GCancellable *cancellable,
                           guint8        gain,
                           guint8        integration,
                           guint8        dac,
                           guint8       *frame,
                           gsize         frame_size)
{
  MafpCaptureContext *context;
  FpiSsm *ssm;

  g_return_val_if_fail (FP_IS_DEVICE (device), NULL);
  g_return_val_if_fail (spi_fd >= 0, NULL);
  g_return_val_if_fail (frame != NULL, NULL);
  g_return_val_if_fail (frame_size >= MAFP8800_FP36_FRAME_SIZE, NULL);

  memset (frame, 0, MAFP8800_FP36_FRAME_SIZE);

  context = g_new0 (MafpCaptureContext, 1);
  context->spi_fd = spi_fd;
  context->cancellable = cancellable ? g_object_ref (cancellable) : NULL;
  context->gain = gain;
  context->integration = integration;
  context->dac = dac;
  context->frame = frame;
  context->frame_size = frame_size;

  ssm = fpi_ssm_new (device,
                     mafp_capture_handler,
                     MAFP_CAPTURE_NUM_STATES);
  fpi_ssm_set_data (ssm,
                    context,
                    (GDestroyNotify) mafp_capture_context_free);

  return ssm;
}
