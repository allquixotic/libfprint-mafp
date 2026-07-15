/*
 * Bounded MAFP8800 development capture utility
 *
 * This is intentionally not installed. It exercises the same asynchronous
 * transport and parser as the libfprint driver without enrollment or PAM.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

#include <glib-unix.h>

#include "drivers/mafp8800-proto.h"
#include "drivers/mafp8800-transport.h"
#include "test-device-fake.h"

#define CAPTURE_TIMEOUT_SECONDS 15
#define CAPTURE_INTEGRATION 0x02
#define CAPTURE_DAC 0xA1

typedef struct
{
  GMainLoop *loop;
  GError    *error;
  gboolean   complete;
} CaptureResult;

typedef struct
{
  GCancellable *cancellable;
  GMainLoop    *loop;
  gboolean      prompting;
} CaptureControl;

static gboolean
cancel_capture (gpointer user_data)
{
  CaptureControl *control = user_data;

  g_cancellable_cancel (control->cancellable);
  if (control->prompting)
    g_main_loop_quit (control->loop);
  return G_SOURCE_CONTINUE;
}

static gboolean
confirm_finger (gint fd, GIOCondition condition, gpointer user_data)
{
  CaptureResult *result = user_data;
  char input[128];
  ssize_t count = -1;

  if (condition & G_IO_IN)
    count = read (fd, input, sizeof (input));

  if (count > 0)
    result->complete = TRUE;
  else
    result->error = g_error_new_literal (G_IO_ERROR,
                                         G_IO_ERROR_CANCELLED,
                                         "No confirmation received");

  g_main_loop_quit (result->loop);
  return G_SOURCE_CONTINUE;
}

static void
capture_complete_cb (FpiSsm   *ssm,
                     FpDevice *device,
                     GError   *error)
{
  CaptureResult *result = g_object_get_data (G_OBJECT (device),
                                             "mafp-capture-result");

  result->complete = TRUE;
  result->error = g_steal_pointer (&error);
  g_main_loop_quit (result->loop);
}

static gboolean
write_pixels_pgm (const char    *path,
                  const guint16 *pixels,
                  guint          width,
                  guint          height,
                  GError       **error)
{
  const gsize pixel_bytes = (gsize) width * height * 2;
  char header[64];
  gint header_size;
  g_autofree guint8 *contents = NULL;
  guint16 minimum = G_MAXUINT16;
  guint16 maximum = 0;
  guint64 sum = 0;

  header_size = g_snprintf (header,
                            sizeof (header),
                            "P5\n%u %u\n65535\n",
                            width,
                            height);
  g_assert_cmpint (header_size, >, 0);
  g_assert_cmpuint ((gsize) header_size, <, sizeof (header));

  contents = g_malloc ((gsize) header_size + pixel_bytes);
  memcpy (contents, header, header_size);

  for (gsize index = 0; index < (gsize) width * height; index++)
    {
      gsize destination = (gsize) header_size + index * 2;
      guint16 value = pixels[index];

      /* Sixteen-bit binary PGM samples are stored most-significant byte
       * first, unlike the driver's host-order pixels. */
      contents[destination] = value >> 8;
      contents[destination + 1] = value & 0xFF;
      minimum = MIN (minimum, value);
      maximum = MAX (maximum, value);
      sum += value;
    }

  if (!g_file_set_contents (path,
                            (const char *) contents,
                            (gssize) ((gsize) header_size + pixel_bytes),
                            error))
    return FALSE;

  g_print ("Captured complete %ux%u frame: min=%u max=%u mean=%.1f\n",
           width,
           height,
           minimum,
           maximum,
           (double) sum / (width * height));
  return TRUE;
}

static gboolean
write_raw_frame_pgm (const char   *path,
                     const guint8 *frame,
                     GError      **error)
{
  g_autofree guint16 *pixels =
    g_new (guint16, MAFP8800_FP36_ENHANCED_PIXELS);

  for (guint row = 0; row < MAFP8800_FP36_ROWS; row++)
    for (guint column = 1; column < MAFP8800_FP36_COLUMNS; column++)
      {
        gsize source = (gsize) row * MAFP8800_FP36_ROW_SIZE + column * 2;
        gsize destination =
          (gsize) row * MAFP8800_FP36_ENHANCED_COLUMNS + column - 1;

        pixels[destination] =
          (guint16) frame[source] | ((guint16) frame[source + 1] << 8);
      }

  return write_pixels_pgm (path,
                           pixels,
                           MAFP8800_FP36_ENHANCED_COLUMNS,
                           MAFP8800_FP36_ROWS,
                           error);
}

int
main (int argc, char *argv[])
{
  g_autoptr(FpDevice) device = NULL;
  g_autoptr(GCancellable) cancellable = NULL;
  g_autoptr(GMainLoop) loop = NULL;
  g_autofree guint8 *frame = NULL;
  g_autofree guint8 *background = NULL;
  g_autofree guint16 *enhanced = NULL;
  g_autoptr(GError) error = NULL;
  CaptureResult result = { 0 };
  CaptureControl control = { 0 };
  FpiSsm *ssm;
  guint input_source = 0;
  guint signal_source;
  guint timeout_source;
  guint8 gain = 0;
  gboolean finger_mode;
  int spi_fd;

  finger_mode = argc == 4 && g_str_equal (argv[3], "--finger");
  if (argc != 3 && !finger_mode)
    {
      g_printerr ("Usage: %s /dev/spidevB.C output.pgm [--finger]\n",
                  argv[0]);
      return 2;
    }

  spi_fd = open (argv[1], O_RDWR | O_CLOEXEC);
  if (spi_fd < 0)
    {
      g_printerr ("Cannot open %s: %s\n", argv[1], g_strerror (errno));
      return 1;
    }

  umask (0077);
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  cancellable = g_cancellable_new ();
  loop = g_main_loop_new (NULL, FALSE);
  frame = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  background = g_malloc0 (MAFP8800_FP36_FRAME_SIZE);
  enhanced = g_new0 (guint16, MAFP8800_FP36_ENHANCED_PIXELS);
  result.loop = loop;
  control.cancellable = cancellable;
  control.loop = loop;
  g_object_set_data (G_OBJECT (device), "mafp-capture-result", &result);

  ssm = mafp8800_fp36_calibrate_gain_new (device,
                                          spi_fd,
                                          cancellable,
                                          &gain);
  g_assert_nonnull (ssm);

  signal_source = g_unix_signal_add (SIGINT,
                                     cancel_capture,
                                     &control);
  timeout_source = g_timeout_add_seconds (CAPTURE_TIMEOUT_SECONDS,
                                          cancel_capture,
                                          &control);

  fpi_ssm_start (ssm, capture_complete_cb);
  if (!result.complete)
    g_main_loop_run (loop);
  g_clear_handle_id (&timeout_source, g_source_remove);

  if (!result.error)
    {
      g_print ("Calibrated capture gain: %u\n", gain);
      result.complete = FALSE;
      timeout_source = g_timeout_add_seconds (CAPTURE_TIMEOUT_SECONDS,
                                              cancel_capture,
                                              &control);
      ssm = mafp8800_fp36_capture_new (device,
                                       spi_fd,
                                       cancellable,
                                       gain,
                                       CAPTURE_INTEGRATION,
                                       CAPTURE_DAC,
                                       frame,
                                       MAFP8800_FP36_FRAME_SIZE);
      g_assert_nonnull (ssm);
      fpi_ssm_start (ssm, capture_complete_cb);
      if (!result.complete)
        g_main_loop_run (loop);
      g_clear_handle_id (&timeout_source, g_source_remove);
    }

  if (!result.error && finger_mode)
    {
      memcpy (background, frame, MAFP8800_FP36_FRAME_SIZE);
      g_print ("Background captured. Place one finger flat on the reader, "
               "then press Enter (Ctrl+C cancels): ");
      fflush (stdout);
      result.complete = FALSE;
      control.prompting = TRUE;
      input_source = g_unix_fd_add (STDIN_FILENO,
                                    G_IO_IN | G_IO_HUP | G_IO_ERR | G_IO_NVAL,
                                    confirm_finger,
                                    &result);
      g_main_loop_run (loop);
      control.prompting = FALSE;
      g_clear_handle_id (&input_source, g_source_remove);

      if (!result.error && g_cancellable_is_cancelled (cancellable))
        result.error = g_error_new_literal (G_IO_ERROR,
                                            G_IO_ERROR_CANCELLED,
                                            "Capture cancelled");
    }

  if (!result.error && finger_mode)
    {
      result.complete = FALSE;
      timeout_source = g_timeout_add_seconds (CAPTURE_TIMEOUT_SECONDS,
                                              cancel_capture,
                                              &control);
      ssm = mafp8800_fp36_capture_new (device,
                                       spi_fd,
                                       cancellable,
                                       gain,
                                       CAPTURE_INTEGRATION,
                                       CAPTURE_DAC,
                                       frame,
                                       MAFP8800_FP36_FRAME_SIZE);
      g_assert_nonnull (ssm);
      fpi_ssm_start (ssm, capture_complete_cb);
      if (!result.complete)
        g_main_loop_run (loop);
      g_clear_handle_id (&timeout_source, g_source_remove);

      if (!result.error &&
          !mafp8800_enhance_fp36_frame (background,
                                        MAFP8800_FP36_FRAME_SIZE,
                                        frame,
                                        MAFP8800_FP36_FRAME_SIZE,
                                        enhanced,
                                        MAFP8800_FP36_ENHANCED_PIXELS,
                                        &result.error))
        g_assert_nonnull (result.error);
    }

  g_clear_handle_id (&signal_source, g_source_remove);
  g_clear_handle_id (&input_source, g_source_remove);
  g_clear_handle_id (&timeout_source, g_source_remove);
  close (spi_fd);

  if (result.error)
    {
      g_printerr ("Capture failed: %s\n", result.error->message);
      g_clear_error (&result.error);
      return 1;
    }

  if (!(finger_mode ?
        write_pixels_pgm (argv[2],
                          enhanced,
                          MAFP8800_FP36_ENHANCED_COLUMNS,
                          MAFP8800_FP36_ROWS,
                          &error) :
        write_raw_frame_pgm (argv[2], frame, &error)))
    {
      g_printerr ("Cannot write %s: %s\n", argv[2], error->message);
      return 1;
    }

  g_print ("Wrote private 16-bit PGM to %s\n", argv[2]);
  return 0;
}
