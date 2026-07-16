/*
 * Asynchronous Microarray MAFP8800 press acquisition for libfprint
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#define FP_COMPONENT "mafp8800"

#include "drivers_api.h"
#include "mafp8800-acquisition.h"
#include "mafp8800-press.h"
#include "mafp8800-proto.h"
#include "mafp8800-transport.h"

#define MAFP_CAPTURE_INTEGRATION 0x02
#define MAFP_CAPTURE_DAC 0xA1
#define MAFP_PRESS_SAMPLE_DELAY_MS 50
#define MAFP_REMOVAL_SAMPLE_DELAY_MS 100
#define MAFP_REMOVAL_DEBOUNCE_SAMPLES 2

typedef struct
{
  int           spi_fd;
  GCancellable *cancellable;
  guint8        gain;
  const guint8 *background;
  gsize         background_size;
  guint8       *frame;
  gsize         frame_size;
  guint8       *previous;
  gboolean     *stable;
  guint        *changed_pixels;
  guint64      *stability_sad;
  guint         stability_attempts;
} MafpPressContext;

typedef struct
{
  int           spi_fd;
  GCancellable *cancellable;
  guint8        gain;
  const guint8 *background;
  gsize         background_size;
  guint8       *frame;
  gsize         frame_size;
  guint         absent_samples;
} MafpRemovalContext;

enum mafp_press_state {
  MAFP_PRESS_CAPTURE,
  MAFP_PRESS_ANALYZE_PRESENCE,
  MAFP_PRESS_STABILITY_WAIT,
  MAFP_PRESS_STABILITY_CAPTURE,
  MAFP_PRESS_ANALYZE_STABILITY,
  MAFP_PRESS_COMPLETE,
  MAFP_PRESS_NUM_STATES,
};

enum mafp_removal_state {
  MAFP_REMOVAL_CAPTURE,
  MAFP_REMOVAL_ANALYZE,
  MAFP_REMOVAL_WAIT,
  MAFP_REMOVAL_COMPLETE,
  MAFP_REMOVAL_NUM_STATES,
};

static void
mafp_clear_frame (gpointer data, gsize size)
{
  volatile guint8 *bytes = data;

  while (size-- > 0)
    *bytes++ = 0;
}

static void
mafp_press_context_free (gpointer data)
{
  MafpPressContext *context = data;

  g_clear_object (&context->cancellable);
  mafp_clear_frame (context->previous, MAFP8800_FP36_FRAME_SIZE);
  g_clear_pointer (&context->previous, g_free);
  g_free (context);
}

static void
mafp_removal_context_free (gpointer data)
{
  MafpRemovalContext *context = data;

  g_clear_object (&context->cancellable);
  g_free (context);
}

static gboolean
mafp_ssm_fail_if_cancelled (FpiSsm       *ssm,
                            GCancellable *cancellable,
                            const char   *message)
{
  if (!cancellable || !g_cancellable_is_cancelled (cancellable))
    return FALSE;

  fpi_ssm_mark_failed (ssm,
                       g_error_new_literal (G_IO_ERROR,
                                            G_IO_ERROR_CANCELLED,
                                            message));
  return TRUE;
}

static FpiSsm *
mafp_capture_new (FpDevice     *device,
                  int           spi_fd,
                  GCancellable *cancellable,
                  guint8        gain,
                  guint8       *frame,
                  gsize         frame_size)
{
  return mafp8800_fp36_capture_new (device,
                                    spi_fd,
                                    cancellable,
                                    gain,
                                    MAFP_CAPTURE_INTEGRATION,
                                    MAFP_CAPTURE_DAC,
                                    frame,
                                    frame_size);
}

static void
mafp_press_handler (FpiSsm *ssm, FpDevice *device)
{
  MafpPressContext *context = fpi_ssm_get_data (ssm);

  g_autoptr(GError) error = NULL;
  gboolean present = FALSE;
  gboolean stable = FALSE;
  guint changed = 0;
  guint64 sad = 0;

  if (mafp_ssm_fail_if_cancelled (ssm,
                                  context->cancellable,
                                  "FP36 press acquisition cancelled"))
    return;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case MAFP_PRESS_CAPTURE:
      fpi_ssm_start_subsm (ssm,
                           mafp_capture_new (device,
                                             context->spi_fd,
                                             context->cancellable,
                                             context->gain,
                                             context->frame,
                                             context->frame_size));
      return;

    case MAFP_PRESS_ANALYZE_PRESENCE:
      if (!mafp8800_fp36_measure_finger (context->background,
                                         context->background_size,
                                         context->frame,
                                         context->frame_size,
                                         &present,
                                         &changed,
                                         &error))
        {
          fpi_ssm_mark_failed (ssm, g_steal_pointer (&error));
          return;
        }

      if (context->changed_pixels)
        *context->changed_pixels = changed;

      if (!present)
        {
          fpi_ssm_jump_to_state_delayed (ssm,
                                         MAFP_PRESS_CAPTURE,
                                         MAFP_PRESS_SAMPLE_DELAY_MS);
          return;
        }

      memcpy (context->previous,
              context->frame,
              MAFP8800_FP36_FRAME_SIZE);
      context->stability_attempts = 0;
      fpi_ssm_next_state (ssm);
      return;

    case MAFP_PRESS_STABILITY_WAIT:
      fpi_ssm_next_state_delayed (ssm, MAFP_PRESS_SAMPLE_DELAY_MS);
      return;

    case MAFP_PRESS_STABILITY_CAPTURE:
      fpi_ssm_start_subsm (ssm,
                           mafp_capture_new (device,
                                             context->spi_fd,
                                             context->cancellable,
                                             context->gain,
                                             context->frame,
                                             context->frame_size));
      return;

    case MAFP_PRESS_ANALYZE_STABILITY:
      if (!mafp8800_fp36_measure_finger (context->background,
                                         context->background_size,
                                         context->frame,
                                         context->frame_size,
                                         &present,
                                         &changed,
                                         &error))
        {
          fpi_ssm_mark_failed (ssm, g_steal_pointer (&error));
          return;
        }

      if (context->changed_pixels)
        *context->changed_pixels = changed;

      if (!present)
        {
          context->stability_attempts = 0;
          fpi_ssm_jump_to_state_delayed (ssm,
                                         MAFP_PRESS_CAPTURE,
                                         MAFP_PRESS_SAMPLE_DELAY_MS);
          return;
        }

      if (!mafp8800_fp36_measure_stability (context->previous,
                                            MAFP8800_FP36_FRAME_SIZE,
                                            context->frame,
                                            context->frame_size,
                                            &stable,
                                            &sad,
                                            &error))
        {
          fpi_ssm_mark_failed (ssm, g_steal_pointer (&error));
          return;
        }

      if (context->stability_sad)
        *context->stability_sad = sad;

      context->stability_attempts++;
      if (stable)
        {
          *context->stable = TRUE;
          fpi_ssm_next_state (ssm);
          return;
        }

      if (context->stability_attempts >= MAFP8800_FP36_STABILITY_ATTEMPTS)
        {
          fpi_ssm_next_state (ssm);
          return;
        }

      memcpy (context->previous,
              context->frame,
              MAFP8800_FP36_FRAME_SIZE);
      fpi_ssm_jump_to_state (ssm, MAFP_PRESS_STABILITY_WAIT);
      return;

    case MAFP_PRESS_COMPLETE:
      fpi_ssm_mark_completed (ssm);
      return;

    default:
      g_assert_not_reached ();
    }
}

static void
mafp_removal_handler (FpiSsm *ssm, FpDevice *device)
{
  MafpRemovalContext *context = fpi_ssm_get_data (ssm);

  g_autoptr(GError) error = NULL;
  gboolean present = FALSE;

  if (mafp_ssm_fail_if_cancelled (ssm,
                                  context->cancellable,
                                  "FP36 finger-removal wait cancelled"))
    return;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case MAFP_REMOVAL_CAPTURE:
      fpi_ssm_start_subsm (ssm,
                           mafp_capture_new (device,
                                             context->spi_fd,
                                             context->cancellable,
                                             context->gain,
                                             context->frame,
                                             context->frame_size));
      return;

    case MAFP_REMOVAL_ANALYZE:
      if (!mafp8800_fp36_measure_finger (context->background,
                                         context->background_size,
                                         context->frame,
                                         context->frame_size,
                                         &present,
                                         NULL,
                                         &error))
        {
          fpi_ssm_mark_failed (ssm, g_steal_pointer (&error));
          return;
        }

      if (present)
        context->absent_samples = 0;
      else
        context->absent_samples++;

      if (context->absent_samples >= MAFP_REMOVAL_DEBOUNCE_SAMPLES)
        fpi_ssm_jump_to_state (ssm, MAFP_REMOVAL_COMPLETE);
      else
        fpi_ssm_next_state (ssm);
      return;

    case MAFP_REMOVAL_WAIT:
      fpi_ssm_jump_to_state_delayed (ssm,
                                     MAFP_REMOVAL_CAPTURE,
                                     MAFP_REMOVAL_SAMPLE_DELAY_MS);
      return;

    case MAFP_REMOVAL_COMPLETE:
      fpi_ssm_mark_completed (ssm);
      return;

    default:
      g_assert_not_reached ();
    }
}

/**
 * mafp8800_fp36_acquire_press_new:
 * @device: device associated with the transport
 * @spi_fd: open spidev file descriptor
 * @cancellable: (nullable): cancellation object for the operation
 * @gain: calibrated capture gain
 * @background: caller-owned uncovered reference frame
 * @background_size: size of @background
 * @frame: caller-owned destination for the accepted frame
 * @frame_size: size of @frame
 * @stable: caller-owned outcome, set only for a stable press
 * @changed_pixels: (nullable): caller-owned last presence score
 * @stability_sad: (nullable): caller-owned last stability score
 *
 * Waits asynchronously for a finger, then requires two sufficiently similar
 * complete frames. Twenty moving samples complete successfully with @stable
 * still %FALSE so the action layer can report a retry. Transport, parse, and
 * cancellation errors fail the state machine. All caller-owned storage must
 * remain valid through completion.
 *
 * Returns: (transfer full): a new, not-yet-started state machine
 */
FpiSsm *
mafp8800_fp36_acquire_press_new (FpDevice     *device,
                                 int           spi_fd,
                                 GCancellable *cancellable,
                                 guint8        gain,
                                 const guint8 *background,
                                 gsize         background_size,
                                 guint8       *frame,
                                 gsize         frame_size,
                                 gboolean     *stable,
                                 guint        *changed_pixels,
                                 guint64      *stability_sad)
{
  MafpPressContext *context;
  FpiSsm *ssm;

  g_return_val_if_fail (FP_IS_DEVICE (device), NULL);
  g_return_val_if_fail (spi_fd >= 0, NULL);
  g_return_val_if_fail (background != NULL, NULL);
  g_return_val_if_fail (background_size >= MAFP8800_FP36_FRAME_SIZE, NULL);
  g_return_val_if_fail (frame != NULL, NULL);
  g_return_val_if_fail (frame_size >= MAFP8800_FP36_FRAME_SIZE, NULL);
  g_return_val_if_fail (stable != NULL, NULL);

  memset (frame, 0, MAFP8800_FP36_FRAME_SIZE);
  *stable = FALSE;
  if (changed_pixels)
    *changed_pixels = 0;
  if (stability_sad)
    *stability_sad = 0;

  context = g_new0 (MafpPressContext, 1);
  context->spi_fd = spi_fd;
  context->cancellable = cancellable ? g_object_ref (cancellable) : NULL;
  context->gain = gain;
  context->background = background;
  context->background_size = background_size;
  context->frame = frame;
  context->frame_size = frame_size;
  context->previous = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  context->stable = stable;
  context->changed_pixels = changed_pixels;
  context->stability_sad = stability_sad;

  ssm = fpi_ssm_new (device, mafp_press_handler, MAFP_PRESS_NUM_STATES);
  fpi_ssm_set_data (ssm, context, mafp_press_context_free);
  return ssm;
}

/**
 * mafp8800_fp36_wait_removal_new:
 * @device: device associated with the transport
 * @spi_fd: open spidev file descriptor
 * @cancellable: (nullable): cancellation object for the operation
 * @gain: calibrated capture gain
 * @background: caller-owned uncovered reference frame
 * @background_size: size of @background
 * @scratch_frame: caller-owned capture scratch space
 * @scratch_frame_size: size of @scratch_frame
 *
 * Waits for two consecutive finger-absent samples. The debounce prevents one
 * noisy capture from advancing enrollment while the same press remains.
 *
 * Returns: (transfer full): a new, not-yet-started state machine
 */
FpiSsm *
mafp8800_fp36_wait_removal_new (FpDevice     *device,
                                int           spi_fd,
                                GCancellable *cancellable,
                                guint8        gain,
                                const guint8 *background,
                                gsize         background_size,
                                guint8       *scratch_frame,
                                gsize         scratch_frame_size)
{
  MafpRemovalContext *context;
  FpiSsm *ssm;

  g_return_val_if_fail (FP_IS_DEVICE (device), NULL);
  g_return_val_if_fail (spi_fd >= 0, NULL);
  g_return_val_if_fail (background != NULL, NULL);
  g_return_val_if_fail (background_size >= MAFP8800_FP36_FRAME_SIZE, NULL);
  g_return_val_if_fail (scratch_frame != NULL, NULL);
  g_return_val_if_fail (scratch_frame_size >= MAFP8800_FP36_FRAME_SIZE, NULL);

  memset (scratch_frame, 0, MAFP8800_FP36_FRAME_SIZE);

  context = g_new0 (MafpRemovalContext, 1);
  context->spi_fd = spi_fd;
  context->cancellable = cancellable ? g_object_ref (cancellable) : NULL;
  context->gain = gain;
  context->background = background;
  context->background_size = background_size;
  context->frame = scratch_frame;
  context->frame_size = scratch_frame_size;

  ssm = fpi_ssm_new (device, mafp_removal_handler, MAFP_REMOVAL_NUM_STATES);
  fpi_ssm_set_data (ssm, context, mafp_removal_context_free);
  return ssm;
}
