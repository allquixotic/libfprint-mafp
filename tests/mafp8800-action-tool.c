/*
 * Transient MAFP8800 enrollment and verification evaluator
 *
 * This is intentionally not installed. It exercises the public libfprint API
 * against the development driver, including an in-memory serialization round
 * trip whose buffer is wiped before release. It never writes a print to disk.
 */

#include <stdlib.h>

#include <glib-unix.h>
#include <libfprint/fprint.h>

#define OPEN_TIMEOUT_SECONDS 30
#define ENROLL_TIMEOUT_SECONDS 180
#define VERIFY_TIMEOUT_SECONDS 45
#define CLOSE_TIMEOUT_SECONDS 30
#define DEFAULT_GENUINE_ATTEMPTS 3
#define SCREEN_GENUINE_ATTEMPTS 10
#define SCREEN_IMPOSTOR_ATTEMPTS 10

typedef enum {
  ACTION_PHASE_NONE,
  ACTION_PHASE_ENROLL,
  ACTION_PHASE_VERIFY,
} ActionPhase;

typedef struct
{
  GCancellable *cancellable;
  guint         signal_source;
  guint         timeout_source;
  gboolean      interrupted;
  gboolean      timed_out;
} ActionControl;

typedef struct
{
  ActionPhase         phase;
  FpFingerStatusFlags last_status;
  gint                completed_stages;
  gboolean            expect_different_finger;
} PromptState;

static gboolean
cancel_for_signal (gpointer user_data)
{
  ActionControl *control = user_data;

  control->interrupted = TRUE;
  if (control->cancellable)
    g_cancellable_cancel (control->cancellable);
  return G_SOURCE_CONTINUE;
}

static gboolean
cancel_for_timeout (gpointer user_data)
{
  ActionControl *control = user_data;

  control->timeout_source = 0;
  control->timed_out = TRUE;
  g_cancellable_cancel (control->cancellable);
  return G_SOURCE_REMOVE;
}

static GCancellable *
control_begin (ActionControl *control, guint timeout_seconds)
{
  g_assert_null (control->cancellable);
  g_assert_cmpuint (control->timeout_source, ==, 0);

  control->timed_out = FALSE;
  control->cancellable = g_cancellable_new ();
  control->timeout_source = g_timeout_add_seconds (timeout_seconds,
                                                   cancel_for_timeout,
                                                   control);
  if (control->interrupted)
    g_cancellable_cancel (control->cancellable);
  return control->cancellable;
}

static void
control_end (ActionControl *control)
{
  g_clear_handle_id (&control->timeout_source, g_source_remove);
  g_clear_object (&control->cancellable);
}

static void
print_operation_error (const char          *operation,
                       const GError        *error,
                       const ActionControl *control)
{
  if (control->timed_out)
    g_printerr ("%s timed out: %s\n", operation, error->message);
  else if (control->interrupted)
    g_printerr ("%s cancelled: %s\n", operation, error->message);
  else
    g_printerr ("%s failed: %s\n", operation, error->message);
}

static void
finger_status_changed (FpDevice   *device,
                       GParamSpec *spec,
                       gpointer    user_data)
{
  PromptState *state = user_data;
  const FpFingerStatusFlags status = fp_device_get_finger_status (device);

  if (state->phase == ACTION_PHASE_NONE || status == state->last_status)
    {
      state->last_status = status;
      return;
    }

  if ((status & FP_FINGER_STATUS_PRESENT) &&
      !(state->last_status & FP_FINGER_STATUS_PRESENT))
    {
      g_print ("Press accepted; lift the finger when the reader is ready.\n");
    }
  else if ((status & FP_FINGER_STATUS_NEEDED) &&
           !(status & FP_FINGER_STATUS_PRESENT))
    {
      if (state->phase == ACTION_PHASE_ENROLL)
        {
          if (state->completed_stages <
              fp_device_get_nr_enroll_stages (device))
            g_print ("Place the same finger flat and hold it still...\n");
        }
      else
        {
          if (state->expect_different_finger)
            g_print ("Place a different, non-enrolled finger flat and hold "
                     "it still...\n");
          else
            g_print ("Place the enrolled finger flat and hold it still...\n");
        }
    }

  state->last_status = status;
}

static void
enroll_progress (FpDevice *device,
                 gint      completed_stages,
                 FpPrint  *print,
                 gpointer  user_data,
                 GError   *error)
{
  if (error)
    {
      g_print ("Enrollment capture needs a retry: %s\n", error->message);
      return;
    }

  ((PromptState *) user_data)->completed_stages = completed_stages;
  g_print ("Enrollment press %d/%d accepted.\n",
           completed_stages,
           fp_device_get_nr_enroll_stages (device));
}

static void
match_report (FpDevice *device,
              FpPrint  *match,
              FpPrint  *print,
              gpointer  user_data,
              GError   *error)
{
  if (error)
    g_print ("Verification capture needs a retry: %s\n", error->message);
}

static void
clear_sensitive (gpointer data, gsize size)
{
  volatile guint8 *bytes = data;

  while (size-- > 0)
    *bytes++ = 0;
}

static FpPrint *
roundtrip_print (FpPrint *print, FpDevice *device, GError **error)
{
  g_autoptr(FpPrint) restored = NULL;
  guchar *serialized = NULL;
  gsize serialized_size = 0;

  if (!fp_print_serialize (print,
                           &serialized,
                           &serialized_size,
                           error))
    goto out;

  restored = fp_print_deserialize (serialized, serialized_size, error);
  if (!restored)
    goto out;

  if (!fp_print_equal (print, restored) ||
      !fp_print_compatible (restored, device) ||
      fp_print_get_finger (restored) != fp_print_get_finger (print))
    {
      g_clear_object (&restored);
      g_set_error_literal (error,
                           G_IO_ERROR,
                           G_IO_ERROR_INVALID_DATA,
                           "Serialized print did not round-trip exactly");
    }

out:
  clear_sensitive (serialized, serialized_size);
  g_free (serialized);
  return g_steal_pointer (&restored);
}

static FpDevice *
find_mafp8800 (FpContext *context)
{
  GPtrArray *devices = fp_context_get_devices (context);
  FpDevice *found = NULL;

  for (guint index = 0; devices && index < devices->len; index++)
    {
      FpDevice *candidate = g_ptr_array_index (devices, index);

      if (!g_str_equal (fp_device_get_driver (candidate), "mafp8800"))
        continue;
      if (found)
        {
          g_printerr ("Refusing evaluation: multiple MAFP8800 devices found\n");
          return NULL;
        }
      found = candidate;
    }

  if (!found)
    g_printerr ("No MAFP8800 device was discovered\n");
  return found;
}

int
main (int argc, char *argv[])
{
  g_autoptr(FpContext) context = NULL;
  g_autoptr(FpPrint) template_print = NULL;
  g_autoptr(FpPrint) enrolled_print = NULL;
  g_autoptr(GError) error = NULL;
  ActionControl control = {0};
  PromptState prompt = {0};
  FpDevice *device = NULL;
  gulong finger_handler = 0;
  gboolean opened = FALSE;
  gboolean discover_only = FALSE;
  guint genuine_attempts = DEFAULT_GENUINE_ATTEMPTS;
  guint impostor_attempts = 0;
  guint matched_attempts = 0;
  guint false_accepts = 0;
  int result = EXIT_FAILURE;

  if (argc == 2 && g_str_equal (argv[1], "--help"))
    {
      g_print ("Usage: %s [--discover-only|--impostor|--screen]\n", argv[0]);
      g_print ("Runs transient eight-press enrollment and three verifications; "
               "stores nothing.\n");
      g_print ("--impostor adds one different-finger control; --screen runs "
               "ten genuine and ten different-finger attempts.\n");
      return EXIT_SUCCESS;
    }
  if (argc == 2 && g_str_equal (argv[1], "--discover-only"))
    {
      discover_only = TRUE;
    }
  else if (argc == 2 && g_str_equal (argv[1], "--impostor"))
    {
      impostor_attempts = 1;
    }
  else if (argc == 2 && g_str_equal (argv[1], "--screen"))
    {
      genuine_attempts = SCREEN_GENUINE_ATTEMPTS;
      impostor_attempts = SCREEN_IMPOSTOR_ATTEMPTS;
    }
  else if (argc != 1)
    {
      g_printerr ("Usage: %s [--discover-only|--impostor|--screen]\n", argv[0]);
      return 2;
    }

  control.signal_source = g_unix_signal_add_full (G_PRIORITY_HIGH,
                                                  SIGINT,
                                                  cancel_for_signal,
                                                  &control,
                                                  NULL);
  context = fp_context_new ();
  device = find_mafp8800 (context);
  if (!device)
    goto out;

  g_print ("Discovered %s using driver %s.\n",
           fp_device_get_name (device),
           fp_device_get_driver (device));
  if (discover_only)
    {
      result = EXIT_SUCCESS;
      goto out;
    }
  g_print ("Keep the reader uncovered during calibration and background "
           "capture...\n");
  finger_handler = g_signal_connect (device,
                                     "notify::finger-status",
                                     G_CALLBACK (finger_status_changed),
                                     &prompt);

  if (!fp_device_open_sync (device,
                            control_begin (&control, OPEN_TIMEOUT_SECONDS),
                            &error))
    {
      print_operation_error ("Device open", error, &control);
      control_end (&control);
      goto out;
    }
  control_end (&control);
  opened = TRUE;

  g_print ("Open and calibration completed. Enrollment requires %d presses; "
           "use the same finger and reposition it slightly each time.\n",
           fp_device_get_nr_enroll_stages (device));
  template_print = g_object_ref_sink (fp_print_new (device));
  fp_print_set_finger (template_print, FP_FINGER_RIGHT_INDEX);
  prompt.phase = ACTION_PHASE_ENROLL;
  prompt.last_status = fp_device_get_finger_status (device);
  prompt.completed_stages = 0;
  enrolled_print = fp_device_enroll_sync (
    device,
    template_print,
    control_begin (&control, ENROLL_TIMEOUT_SECONDS),
    enroll_progress,
    &prompt,
    &error);
  prompt.phase = ACTION_PHASE_NONE;
  if (!enrolled_print)
    {
      print_operation_error ("Enrollment", error, &control);
      control_end (&control);
      goto out;
    }
  control_end (&control);
  g_print ("Enrollment completed in memory; no template was written to disk.\n");
  {
    g_autoptr(FpPrint) restored_print = NULL;

    restored_print = roundtrip_print (enrolled_print, device, &error);
    if (!restored_print)
      {
        g_printerr ("In-memory print serialization round trip failed: %s\n",
                    error->message);
        goto out;
      }
    g_set_object (&enrolled_print, restored_print);
  }
  g_print ("Public FpPrint serialization round trip validated in memory; "
           "serialized buffer wiped.\n");

  g_print ("Running %u same-finger verification attempts.\n",
           genuine_attempts);
  for (guint attempt = 1; attempt <= genuine_attempts; attempt++)
    {
      gboolean matched = FALSE;

      g_print ("Verification attempt %u/%u.\n", attempt, genuine_attempts);
      prompt.phase = ACTION_PHASE_VERIFY;
      prompt.last_status = fp_device_get_finger_status (device);
      if (fp_device_verify_sync (
            device,
            enrolled_print,
            control_begin (&control, VERIFY_TIMEOUT_SECONDS),
            match_report,
            &prompt,
            &matched,
            NULL,
            &error))
        {
          control_end (&control);
          prompt.phase = ACTION_PHASE_NONE;
          if (matched)
            {
              matched_attempts++;
              g_print ("Verification attempt %u: MATCH.\n", attempt);
            }
          else
            {
              g_print ("Verification attempt %u: NO MATCH.\n", attempt);
            }
          continue;
        }

      prompt.phase = ACTION_PHASE_NONE;
      print_operation_error ("Verification", error, &control);
      control_end (&control);
      goto out;
    }
  prompt.phase = ACTION_PHASE_NONE;
  g_print ("Same-finger verification summary: %u/%u matched.\n",
           matched_attempts,
           genuine_attempts);

  if (impostor_attempts > 0)
    {
      g_print ("Distinct-finger negative control: do not use the enrolled "
               "finger. Vary the non-enrolled finger between attempts.\n");
      prompt.expect_different_finger = TRUE;
      for (guint attempt = 1; attempt <= impostor_attempts; attempt++)
        {
          gboolean matched = FALSE;

          g_print ("Different-finger attempt %u/%u.\n",
                   attempt,
                   impostor_attempts);
          prompt.phase = ACTION_PHASE_VERIFY;
          prompt.last_status = fp_device_get_finger_status (device);
          if (!fp_device_verify_sync (
                device,
                enrolled_print,
                control_begin (&control, VERIFY_TIMEOUT_SECONDS),
                match_report,
                &prompt,
                &matched,
                NULL,
                &error))
            {
              prompt.phase = ACTION_PHASE_NONE;
              prompt.expect_different_finger = FALSE;
              print_operation_error ("Different-finger verification",
                                     error,
                                     &control);
              control_end (&control);
              goto out;
            }
          control_end (&control);
          prompt.phase = ACTION_PHASE_NONE;

          if (matched)
            {
              false_accepts++;
              g_printerr ("Different-finger attempt %u: FALSE ACCEPT.\n",
                          attempt);
            }
          else
            {
              g_print ("Different-finger attempt %u: correctly rejected.\n",
                       attempt);
            }
        }
      prompt.expect_different_finger = FALSE;
      g_print ("Different-finger verification summary: %u/%u falsely "
               "matched.\n",
               false_accepts,
               impostor_attempts);
    }

  if (matched_attempts == genuine_attempts && false_accepts == 0)
    result = EXIT_SUCCESS;

out:
  if (opened)
    {
      g_autoptr(GError) close_error = NULL;

      prompt.phase = ACTION_PHASE_NONE;
      control.interrupted = FALSE;
      if (!fp_device_close_sync (
            device,
            control_begin (&control, CLOSE_TIMEOUT_SECONDS),
            &close_error))
        {
          print_operation_error ("Device close", close_error, &control);
          result = EXIT_FAILURE;
        }
      control_end (&control);
    }
  if (finger_handler)
    g_signal_handler_disconnect (device, finger_handler);
  g_clear_handle_id (&control.signal_source, g_source_remove);

  if (result == EXIT_SUCCESS)
    g_print ("Transient action evaluation completed successfully.\n");
  return result;
}
