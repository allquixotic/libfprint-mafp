/*
 * Microarray MAFP8800 driver for libfprint
 * Copyright (C) 2026 Mark Shank
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

/*
 * This driver subclasses FpDevice directly instead of FpImageDevice because
 * the sensor's 36x160 pixel image at ~200 DPI is far too low-resolution for
 * libfprint's NBIS minutiae extraction (mindtct typically finds 0-5 minutiae,
 * insufficient for bozorth3 matching). Instead, the driver implements its own
 * SIFT-WHT binary descriptor matching pipeline tuned for this sensor geometry.
 *
 * For detailed documentation (protocol, matching algorithm, project history):
 * https://github.com/IngeniousIdiocy/mafp8800-fingerprint-driver
 */

#define FP_COMPONENT "mafp8800"

#include "drivers_api.h"
#include "mafp8800-acquisition.h"
#include "mafp8800-proto.h"
#include "mafp8800-template.h"
#include "mafp8800-transport.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <unistd.h>

/* Image geometry: 160 rows x 37 pixels, 74 bytes/row (2 hdr + 72 pixel) */
#define MAFP_ROWS MAFP8800_FP36_ROWS
#define MAFP_COLS MAFP8800_FP36_COLUMNS
#define MAFP_ROW_BYTES MAFP8800_FP36_ROW_SIZE
#define MAFP_PIXELS (MAFP_ROWS * MAFP_COLS)            /* 5920 */
#define MAFP_FRAME_BYTES MAFP8800_FP36_FRAME_SIZE
#define MAFP_ENHANCED_COLS MAFP8800_FP36_ENHANCED_COLUMNS
#define MAFP_ENHANCED_PIXELS MAFP8800_FP36_ENHANCED_PIXELS

/* Chip ID */
#define MAFP_CHIPID_FP36 0x24

/* Enrollment */
#define MAFP_ENROLL_STAGES MAFP8800_TEMPLATE_MAX_SAMPLES

/* Gaussian pyramid: 5 levels (original + 4 blurs), 4 DoG layers */
#define MAFP_PYR_LEVELS 5
#define MAFP_DOG_LEVELS 4

/* Template geometry: 2 banks × 50 keypoints × 20 bytes + headers */
#define MAFP_MAX_KP MAFP8800_TEMPLATE_MAX_KEYPOINTS
#define MAFP_NUM_BANKS MAFP8800_TEMPLATE_BANKS
#define MAFP_DESC_BYTES MAFP8800_TEMPLATE_DESCRIPTOR_SIZE
#define MAFP_KP_META MAFP8800_TEMPLATE_KEYPOINT_META_SIZE
#define MAFP_KP_SIZE MAFP8800_TEMPLATE_KEYPOINT_SIZE
#define MAFP_BANK_DATA_SZ MAFP8800_TEMPLATE_BANK_DATA_SIZE
#define MAFP_BANK_SZ MAFP8800_TEMPLATE_BANK_SIZE
#define MAFP_TPL_MAGIC MAFP8800_TEMPLATE_SAMPLE_MAGIC
#define MAFP_TPL_SAMPLE_SZ MAFP8800_TEMPLATE_SAMPLE_SIZE

/* Match scoring */
#define MAFP_MATCH_THRESH 3000       /* 0xBB8: score >= this = match */
#define MAFP_MIN_MATCH_PTS 6         /* raised from binary's 4: our simplified scoring
                                      * lacks the 9-feature sigmoid that rejects
                                      * low-quality 4-5 inlier matches */
#define MAFP_HAMMING_THRESH 48       /* max Hamming distance for descriptor match */
#define MAFP_HAMMING_RATIO 219       /* ratio test: best*256 < second*219 (≈0.855) */
#define MAFP_INLIER_DIST_SQ 9.0     /* 2303/256: binary uses Q8, we use pixel coords */
#define MAFP_MAX_INLIERS 10         /* binary hard-caps inliers at 10 */
#define MAFP_MAX_MATCHES 15         /* binary caps matches per section at 15 */
#define MAFP_MAX_KP_TOTAL 100       /* max keypoints across both DoG layers */

/* Template buffer: header + 8 enrollment samples */
#define MAFP_TPL_HDR_SZ MAFP8800_TEMPLATE_HEADER_SIZE
#define MAFP_TPL_BUF_SZ MAFP8800_TEMPLATE_SIZE

struct _FpiDeviceMafp8800
{
  FpDevice parent;

  int      spi_fd;

  guint8   gain;

  /* image buffers (160 * 37 pixels as u16 in LE) */
  guint8 *bg_frame;          /* background/image_data reference */
  guint8 *cur_frame;         /* current capture */

  /* enhanced output (MAFP_ENHANCED_PIXELS × 2 bytes, u16 LE) */
  guint16 *enhanced;

  /* asynchronous device-open state */
  guint8 open_attempts;
  guint8 chip_id;

};

G_DECLARE_FINAL_TYPE (FpiDeviceMafp8800, fpi_device_mafp8800, FPI, DEVICE_MAFP8800, FpDevice)
G_DEFINE_TYPE (FpiDeviceMafp8800, fpi_device_mafp8800, FP_TYPE_DEVICE)

static void
mafp_clear_sensitive (gpointer data, gsize size)
{
  volatile guint8 *bytes = data;

  while (size-- > 0)
    *bytes++ = 0;
}

static void
mafp_template_data_free (gpointer data)
{
  mafp_clear_sensitive (data, MAFP_TPL_BUF_SZ);
  g_free (data);
}

/* SPI transport */


static FpiSpiTransfer *
mafp_set_reg_async (FpiDeviceMafp8800 *self, guint8 reg, guint8 value)
{
  FpiSpiTransfer *transfer =
    fpi_spi_transfer_new (FP_DEVICE (self), self->spi_fd);

  fpi_spi_transfer_duplex (transfer, 4);
  transfer->buffer_wr[0] = reg;
  transfer->buffer_wr[1] = value;
  return transfer;
}

static gboolean
mafp_fp36_enhance (FpiDeviceMafp8800 *self, GError **error)
{
  return mafp8800_enhance_fp36_frame (self->bg_frame,
                                      MAFP_FRAME_BYTES,
                                      self->cur_frame,
                                      MAFP_FRAME_BYTES,
                                      self->enhanced,
                                      MAFP_ENHANCED_PIXELS,
                                      error);
}

/* Scale-space keypoint matching */
static const guint16 kern7[]  = {291, 3539, 15862, 26152, 15862, 3539, 291};
static const guint16 kern9[]  = {339, 1951, 6809, 14415, 18508, 14415, 6809, 1951, 339};
static const guint16 kern13[] = {145, 575, 1771, 4248, 7937, 11549, 13086, 11549, 7937, 4248, 1771, 575, 145};
static const guint16 kern17[] = {170, 433, 977, 1942, 3409, 5280, 7217, 8706, 9268, 8706, 7217, 5280, 3409, 1942, 977, 433, 170};

static void
mafp_compute_gradients (const guint16 *img, int rows, int cols,
                        gint32 *mag, guint16 *ori)
{
  memset (mag, 0, rows * cols * sizeof (gint32));
  memset (ori, 0, rows * cols * sizeof (guint16));

  for (int r = 1; r < rows - 1; r++)
    for (int c = 1; c < cols - 1; c++)
      {
        gint32 gx = (gint32) img[r * cols + c + 1] - (gint32) img[r * cols + c - 1];
        gint32 gy = (gint32) img[(r + 1) * cols + c] - (gint32) img[(r - 1) * cols + c];
        mag[r * cols + c] =
          (gint32) sqrt ((double) gx * gx + (double) gy * gy);
        gdouble a = atan2 ((double) gy, (double) gx);
        if (a < 0)
          a += 2.0 * G_PI;
        ori[r * cols + c] = (guint16) (a / (2.0 * G_PI) * 65536.0);
      }
}

/* Internal keypoint representation */
typedef struct
{
  guint8  row;
  guint8  col;
  guint8  dog_layer;
  guint8  polarity;    /* 0 = DoG maximum, 1 = DoG minimum */
  guint16 response;    /* absolute value of DoG extremum */
  gdouble orientation;
  guint8  desc[MAFP_DESC_BYTES];
} MafpKeypoint;

/* Correspondence for RANSAC geometric verification */
typedef struct
{
  guint8  pr, pc, gr, gc;
  guint16 p_ori, g_ori;   /* keypoint orientations for angular consistency */
} MafpCorr;

typedef struct
{
  guint correspondences;
  guint unique_correspondences;
  guint orientation_consistent_pairs;
  guint solved_transforms;
  guint inliers;
} MafpMatchMetrics;

/* Gaussian blur (separable, u16 fixed-point) */

static void
mafp_blur_h (const guint16 *src, guint16 *dst,
             const guint16 *kern, int ksize, int rows, int cols)
{
  int half = ksize / 2;

  for (int r = 0; r < rows; r++)
    for (int c = 0; c < cols; c++)
      {
        guint32 sum = 0;
        for (int k = -half; k <= half; k++)
          {
            int cc = CLAMP (c + k, 0, cols - 1);
            sum += (guint32) src[r * cols + cc] * kern[k + half];
          }
        dst[r * cols + c] = (guint16) (sum >> 16);
      }
}

static void
mafp_blur_v (const guint16 *src, guint16 *dst,
             const guint16 *kern, int ksize, int rows, int cols)
{
  int half = ksize / 2;

  for (int r = 0; r < rows; r++)
    for (int c = 0; c < cols; c++)
      {
        guint32 sum = 0;
        for (int k = -half; k <= half; k++)
          {
            int rr = CLAMP (r + k, 0, rows - 1);
            sum += (guint32) src[rr * cols + c] * kern[k + half];
          }
        dst[r * cols + c] = (guint16) (sum >> 16);
      }
}

static void
mafp_gauss_blur (const guint16 *src, guint16 *dst, guint16 *tmp,
                 const guint16 *kern, int ksize, int rows, int cols)
{
  mafp_blur_h (src, tmp, kern, ksize, rows, cols);
  mafp_blur_v (tmp, dst, kern, ksize, rows, cols);
}

/* DoG keypoint detection */

static int
mafp_detect_keypoints (gint16 **dog, MafpKeypoint *kps)
{
  int W = MAFP_ENHANCED_COLS;
  int count = 0;

  /* Exclude keypoints near edges: the 17×17 descriptor grid needs
   * ±16 pixels of margin. On a 36-wide image, that means col 5-30. */
  int margin_r = 8, margin_c = 5;

  for (int layer = 1; layer <= 2; layer++)
    for (int r = margin_r; r < MAFP_ROWS - margin_r; r++)
      for (int c = margin_c; c < W - margin_c; c++)
        {
          gint16 val = dog[layer][r * W + c];
          if (val == 0)
            continue;

          gboolean is_max = (val > 0), is_min = (val < 0);
          for (int dl = -1; dl <= 1 && (is_max || is_min); dl++)
            for (int dr = -1; dr <= 1 && (is_max || is_min); dr++)
              for (int dc = -1; dc <= 1 && (is_max || is_min); dc++)
                {
                  if (dl == 0 && dr == 0 && dc == 0)
                    continue;
                  gint16 nb = dog[layer + dl][(r + dr) * W + (c + dc)];
                  if (nb >= val)
                    is_max = FALSE;
                  if (nb <= val)
                    is_min = FALSE;
                }

          if ((is_max || is_min) && count < MAFP_MAX_KP_TOTAL)
            {
              kps[count].row       = (guint8) r;
              kps[count].col       = (guint8) c;
              kps[count].dog_layer = (guint8) layer;
              kps[count].polarity  = is_min ? 1 : 0;
              kps[count].response  = is_min ? (guint16) (-(gint32) val) :
                                     (guint16) val;
              count++;
            }
        }

  /* If we hit the cap, keep strongest by sorting on response */
  if (count >= MAFP_MAX_KP_TOTAL)
    {
      for (int i = 0; i < count - 1; i++)
        for (int j = i + 1; j < count; j++)
          if (kps[j].response > kps[i].response)
            {
              MafpKeypoint tmp = kps[i];
              kps[i] = kps[j];
              kps[j] = tmp;
            }
      count = MAFP_MAX_KP_TOTAL;
    }

  return count;
}

/* Orientation assignment */

static void
mafp_assign_orientations (guint16 **pyr, MafpKeypoint *kps, int count)
{
  int W = MAFP_ENHANCED_COLS;

  for (int ki = 0; ki < count; ki++)
    {
      int r0 = kps[ki].row, c0 = kps[ki].col;
      const guint16 *img = pyr[kps[ki].dog_layer];
      gdouble hist[36] = {0};
      int radius = 4;
      gdouble sigma = radius * 0.5;

      for (int dr = -radius; dr <= radius; dr++)
        {
          int r = r0 + dr;
          if (r < 1 || r >= MAFP_ROWS - 1)
            continue;
          for (int dc = -radius; dc <= radius; dc++)
            {
              int c = c0 + dc;
              if (c < 1 || c >= W - 1)
                continue;
              gdouble gx = (gdouble) img[r * W + c + 1] - (gdouble) img[r * W + c - 1];
              gdouble gy = (gdouble) img[(r + 1) * W + c] - (gdouble) img[(r - 1) * W + c];
              gdouble mag = sqrt (gx * gx + gy * gy);
              gdouble angle = atan2 (gy, gx);
              if (angle < 0)
                angle += 2.0 * G_PI;
              gdouble w = exp (-(dr * dr + dc * dc) / (2.0 * sigma * sigma));
              int bin = (int) (angle / (2.0 * G_PI) * 36.0);
              if (bin >= 36)
                bin = 35;
              hist[bin] += mag * w;
            }
        }

      /* Smooth with [0.25, 0.5, 0.25], 2 passes */
      for (int pass = 0; pass < 2; pass++)
        {
          gdouble tmp[36];
          for (int b = 0; b < 36; b++)
            tmp[b] = hist[(b + 35) % 36] * 0.25
                     + hist[b] * 0.5
                     + hist[(b + 1) % 36] * 0.25;
          memcpy (hist, tmp, sizeof (hist));
        }

      int best_bin = 0;
      gdouble best_val = hist[0];
      for (int b = 1; b < 36; b++)
        if (hist[b] > best_val)
          {
            best_val = hist[b];
            best_bin = b;
          }

      if (best_val < 48.0)
        {
          kps[ki].orientation = -1.0;
          continue;
        }

      /* Sub-bin parabolic interpolation */
      gdouble lv = hist[(best_bin + 35) % 36];
      gdouble rv = hist[(best_bin + 1) % 36];
      gdouble offset = 0;
      gdouble denom = lv + rv - 2.0 * best_val;
      if (fabs (denom) > 1e-6)
        offset = 0.5 * (lv - rv) / denom;
      kps[ki].orientation = ((best_bin + offset + 0.5) / 36.0) * 2.0 * G_PI;
    }
}

/* SIFT-WHT binary descriptor: 17x17 rotated gradient grid -> 128-bit output */

static void
mafp_compute_descriptor (const gint32 *grad_mag, const guint16 *grad_ori,
                         int rows, int cols,
                         int r0, int c0, guint16 kp_ori, guint8 *desc)
{
  gdouble angle = (gdouble) kp_ori / 65536.0 * 2.0 * G_PI;
  gdouble ca = cos (angle), sa = sin (angle);
  gint32 hist[128] = {0};
  gdouble sigma = 10.0;

  /* Phase 1: 17×17 grid, 2-pixel spacing, rotated by orientation */
  for (int gr = -8; gr <= 8; gr++)
    for (int gc = -8; gc <= 8; gc++)
      {
        gdouble rx = gc * 2.0 * ca - gr * 2.0 * sa;
        gdouble ry = gc * 2.0 * sa + gr * 2.0 * ca;
        int sr = r0 + (int) (ry + 0.5);
        int sc = c0 + (int) (rx + 0.5);
        if (sr < 1 || sr >= rows - 1 || sc < 1 || sc >= cols - 1)
          continue;

        gint32 mag = grad_mag[sr * cols + sc];
        if (mag == 0)
          continue;
        guint16 gori = grad_ori[sr * cols + sc];

        gdouble gauss = exp (-(rx * rx + ry * ry) / (2.0 * sigma * sigma));
        gint32 weight = (gint32) (mag * gauss);

        /* Trilinear interpolation into 4×4×8 histogram (matching binary) */
        gdouble sx_f = (rx + 7.5) / 5.0;
        gdouble sy_f = (ry + 7.5) / 5.0;
        guint16 rel = gori - kp_ori;
        gdouble o_f = (gdouble) rel / 8192.0;

        int sx0 = (int) floor (sx_f), sy0 = (int) floor (sy_f);
        int o0 = (int) floor (o_f);
        gdouble fx = sx_f - sx0, fy = sy_f - sy0, fo = o_f - o0;

        for (int si = 0; si <= 1; si++)
          for (int sj = 0; sj <= 1; sj++)
            for (int oi = 0; oi <= 1; oi++)
              {
                int bx = sx0 + si, by = sy0 + sj, bo = (o0 + oi) & 7;
                if (bx < 0 || bx > 3 || by < 0 || by > 3)
                  continue;
                gdouble w = weight;
                w *= si ? fx : (1.0 - fx);
                w *= sj ? fy : (1.0 - fy);
                w *= oi ? fo : (1.0 - fo);
                hist[bo * 16 + by * 4 + bx] += (gint32) w;
              }
      }

  /* Phase 2a: Rearrange to spatial-group-major */
  gint32 work[128];
  for (int sy = 0; sy < 4; sy++)
    for (int sx = 0; sx < 4; sx++)
      for (int o = 0; o < 8; o++)
        work[sy * 32 + sx * 8 + o] = hist[o * 16 + sy * 4 + sx];

  /* Phase 2b: Reduce orientations 8→4 */
  gint32 wht[64];
  for (int sp = 0; sp < 16; sp++)
    {
      int b = sp * 8;
      wht[sp * 4 + 0] = work[b + 0];
      wht[sp * 4 + 1] = work[b + 1] + work[b + 2] + work[b + 3];
      wht[sp * 4 + 2] = work[b + 4];
      wht[sp * 4 + 3] = work[b + 5] + work[b + 6] + work[b + 7];
    }

  /* Phase 2c: 4-point WHT on orientation */
  for (int sp = 0; sp < 16; sp++)
    {
      gint32 *p = &wht[sp * 4];
      gint32 a = p[0] + p[1], b = p[0] - p[1];
      gint32 c = p[2] + p[3], d = p[2] - p[3];
      p[0] = a + c;
      p[1] = a - c;
      p[2] = b - d;
      p[3] = b + d;
    }

  /* Phase 2d-g: 16-point spatial WHT (4 butterfly levels) */
  for (int stride = 4; stride <= 32; stride <<= 1)
    for (int i = 0; i < 64; i += stride * 2)
      for (int j = 0; j < stride; j++)
        {
          gint32 a = wht[i + j], b = wht[i + j + stride];
          wht[i + j] = a + b;
          wht[i + j + stride] = a - b;
        }

  /* Phase 2h: Bits 0-63 = sign of WHT coefficients (skip DC) */
  memset (desc, 0, MAFP_DESC_BYTES);
  for (int i = 1; i < 64; i++)
    if (wht[i] > 0)
      desc[i / 8] |= (guint8) (1 << (i % 8));

  /* Phase 2i: Bits 64-127 = median threshold of odd orientation bins */
  gint32 sorted[128];
  memcpy (sorted, work, sizeof (sorted));
  for (int i = 1; i < 128; i++)
    {
      gint32 key = sorted[i];
      int j = i - 1;
      while (j >= 0 && sorted[j] > key)
        {
          sorted[j + 1] = sorted[j];
          j--;
        }
      sorted[j + 1] = key;
    }
  gint32 median = sorted[64];

  for (int i = 0; i < 64; i++)
    if (work[2 * i + 1] > median)
      desc[8 + i / 8] |= (guint8) (1 << (i % 8));
}

/* Feature extraction: enhanced image -> 2012-byte template */

static int
mafp_extract_features (const guint16 *enhanced, guint8 *tpl)
{
  int R = MAFP_ROWS, C = MAFP_ENHANCED_COLS, N = MAFP_ENHANCED_PIXELS;

  /* Allocate pyramid and DoG on heap */
  guint16 *pyr[MAFP_PYR_LEVELS];
  gint16 *dog[MAFP_DOG_LEVELS];
  guint16 *tmp = g_new (guint16, N);

  for (int l = 0; l < MAFP_PYR_LEVELS; l++)
    pyr[l] = g_new (guint16, N);
  for (int l = 0; l < MAFP_DOG_LEVELS; l++)
    dog[l] = g_new (gint16, N);

  /* Build pyramid */
  memcpy (pyr[0], enhanced, N * sizeof (guint16));
  mafp_gauss_blur (pyr[0], pyr[1], tmp, kern7,  7,  R, C);
  mafp_gauss_blur (pyr[1], pyr[2], tmp, kern9,  9,  R, C);
  mafp_gauss_blur (pyr[2], pyr[3], tmp, kern13, 13, R, C);
  mafp_gauss_blur (pyr[3], pyr[4], tmp, kern17, 17, R, C);
  mafp_clear_sensitive (tmp, N * sizeof (guint16));
  g_free (tmp);

  /* DoG = adjacent level difference */
  for (int l = 0; l < MAFP_DOG_LEVELS; l++)
    for (int i = 0; i < N; i++)
      {
        gint32 delta = (gint32) pyr[l][i] - (gint32) pyr[l + 1][i];

        dog[l][i] = (gint16) CLAMP (delta, G_MININT16, G_MAXINT16);
      }

  /* Detect keypoints */
  MafpKeypoint kps[MAFP_MAX_KP_TOTAL];
  int n_kps = mafp_detect_keypoints (dog, kps);

  /* Assign orientations */
  mafp_assign_orientations (pyr, kps, n_kps);

  /* Compute gradient magnitude and orientation from original image */
  gint32 *grad_mag = g_new0 (gint32, N);
  guint16 *grad_ori = g_new0 (guint16, N);
  mafp_compute_gradients (pyr[1], R, C, grad_mag, grad_ori);

  /* Compute SIFT-WHT descriptors from gradient fields */
  for (int i = 0; i < n_kps; i++)
    {
      if (kps[i].orientation < 0)
        continue;
      guint16 ori16 = (guint16) (kps[i].orientation / (2.0 * G_PI) * 65536.0);
      mafp_compute_descriptor (grad_mag, grad_ori, R, C,
                               kps[i].row, kps[i].col,
                               ori16, kps[i].desc);
    }
  mafp_clear_sensitive (grad_mag, N * sizeof (gint32));
  mafp_clear_sensitive (grad_ori, N * sizeof (guint16));
  g_free (grad_mag);
  g_free (grad_ori);

  /* Serialize: [4-byte magic/pad] [bank0: 4+1000] [bank1: 4+1000] */
  memset (tpl, 0, MAFP_TPL_SAMPLE_SZ);
  tpl[0] = MAFP_TPL_MAGIC;

  for (int bank = 0; bank < MAFP_NUM_BANKS; bank++)
    {
      int target_polarity = bank;  /* bank 0 = maxima, bank 1 = minima */
      gint32 bcount = 0;
      guint8 *bdata = tpl + 4 + bank * MAFP_BANK_SZ;
      guint8 *kpdata = bdata + 4;

      for (int i = 0; i < n_kps && bcount < MAFP_MAX_KP; i++)
        {
          if (kps[i].polarity != target_polarity || kps[i].orientation < 0)
            continue;
          guint8 *p = kpdata + bcount * MAFP_KP_SIZE;
          memcpy (p, kps[i].desc, MAFP_DESC_BYTES);
          p[16] = kps[i].row;
          p[17] = kps[i].col;
          guint16 ori16 = (guint16) (kps[i].orientation / (2.0 * G_PI) * 65536.0);
          guint16 encoded_ori = GUINT16_TO_LE (ori16);

          memcpy (p + 18, &encoded_ori, sizeof (encoded_ori));
          bcount++;
        }
      guint32 encoded_count = GUINT32_TO_LE ((guint32) bcount);

      memcpy (bdata, &encoded_count, sizeof (encoded_count));
    }

  /* Cleanup */
  for (int l = 0; l < MAFP_PYR_LEVELS; l++)
    {
      mafp_clear_sensitive (pyr[l], N * sizeof (guint16));
      g_free (pyr[l]);
    }
  for (int l = 0; l < MAFP_DOG_LEVELS; l++)
    {
      mafp_clear_sensitive (dog[l], N * sizeof (gint16));
      g_free (dog[l]);
    }

  guint32 encoded_b0, encoded_b1;
  guint b0, b1;

  memcpy (&encoded_b0, tpl + 4, sizeof (encoded_b0));
  memcpy (&encoded_b1, tpl + 4 + MAFP_BANK_SZ, sizeof (encoded_b1));
  b0 = GUINT32_FROM_LE (encoded_b0);
  b1 = GUINT32_FROM_LE (encoded_b1);
  fp_dbg ("extract: %d keypoints (%d + %d)", b0 + b1, b0, b1);
  mafp_clear_sensitive (kps, sizeof (kps));
  return b0 + b1;
}

/* Template matching */

static inline int
mafp_hamming (const guint8 *a, const guint8 *b)
{
  guint64 a64[2];
  guint64 b64[2];

  memcpy (a64, a, sizeof (a64));
  memcpy (b64, b, sizeof (b64));

  return __builtin_popcountll (a64[0] ^ b64[0])
         + __builtin_popcountll (a64[1] ^ b64[1]);
}

static gboolean
mafp_solve_similarity (const MafpCorr *c,
                       gdouble *a, gdouble *b, gdouble *tx, gdouble *ty)
{
  gdouble px1 = c[0].pc, py1 = c[0].pr;
  gdouble gx1 = c[0].gc, gy1 = c[0].gr;
  gdouble px2 = c[1].pc, py2 = c[1].pr;
  gdouble gx2 = c[1].gc, gy2 = c[1].gr;

  gdouble dxp = px2 - px1, dyp = py2 - py1;
  gdouble dxg = gx2 - gx1, dyg = gy2 - gy1;
  gdouble den = dxp * dxp + dyp * dyp;

  if (den < 1.0)
    return FALSE;

  *a  = (dxp * dxg + dyp * dyg) / den;
  *b  = (dxp * dyg - dyp * dxg) / den;
  *tx = gx1 - (*a * px1 - *b * py1);
  *ty = gy1 - (*b * px1 + *a * py1);
  return fabs (*a) <= 4.0 && fabs (*b) <= 4.0;
}

static int
mafp_count_inliers (const MafpCorr *corrs, int n,
                    gdouble a, gdouble b, gdouble tx, gdouble ty,
                    MafpCorr *inlier_out, gdouble *avg_dist)
{
  int count = 0;
  gdouble total = 0;

  for (int i = 0; i < n && count < MAFP_MAX_INLIERS; i++)
    {
      gdouble ex = a * corrs[i].pc - b * corrs[i].pr + tx - corrs[i].gc;
      gdouble ey = b * corrs[i].pc + a * corrs[i].pr + ty - corrs[i].gr;
      gdouble dsq = ex * ex + ey * ey;
      if (dsq <= MAFP_INLIER_DIST_SQ)
        {
          if (inlier_out)
            inlier_out[count] = corrs[i];
          total += dsq;
          count++;
        }
    }

  if (avg_dist)
    *avg_dist = count > 0 ? total / count : 0;
  return count;
}

static int
mafp_compute_match_score (int n_inliers)
{
  if (n_inliers < MAFP_MIN_MATCH_PTS)
    return 0;
  if (n_inliers >= 8)
    return 10000;
  return MIN (n_inliers * 1250, 10000);
}

static int
mafp_match_templates (const guint8     *probe,
                      const guint8     *gallery,
                      MafpMatchMetrics *metrics)
{
  MafpCorr corrs[60];   /* 2 banks × max 15 matches */
  int n_corrs = 0;

  memset (metrics, 0, sizeof (*metrics));

  for (int bank = 0; bank < MAFP_NUM_BANKS; bank++)
    {
      const guint8 *pb = probe   + 4 + bank * MAFP_BANK_SZ;
      const guint8 *gb = gallery + 4 + bank * MAFP_BANK_SZ;
      int bank_matches = 0;

      guint32 encoded_np, encoded_ng;
      guint np, ng;

      memcpy (&encoded_np, pb, sizeof (encoded_np));
      np = MIN (GUINT32_FROM_LE (encoded_np), (guint32) MAFP_MAX_KP);
      memcpy (&encoded_ng, gb, sizeof (encoded_ng));
      ng = MIN (GUINT32_FROM_LE (encoded_ng), (guint32) MAFP_MAX_KP);

      const guint8 *pk = pb + 4, *gk = gb + 4;

      for (int pi = 0; pi < np && bank_matches < MAFP_MAX_MATCHES; pi++)
        {
          const guint8 *pd = pk + pi * MAFP_KP_SIZE;
          int best = 999, second = 999, best_gi = -1;

          for (int gi = 0; gi < ng; gi++)
            {
              int d = mafp_hamming (pd, gk + gi * MAFP_KP_SIZE);
              if (d < best)
                {
                  second = best;
                  best = d;
                  best_gi = gi;
                }
              else if (d < second)
                {
                  second = d;
                }
            }

          /* Ratio test */
          if (best_gi >= 0 && best < MAFP_HAMMING_THRESH &&
              (second >= 999 || best * 256 < second * MAFP_HAMMING_RATIO))
            {
              fp_dbg ("  corr: p(%d,%d)->g(%d,%d) ham=%d/%d",
                      pd[16], pd[17],
                      gk[best_gi * MAFP_KP_SIZE + 16],
                      gk[best_gi * MAFP_KP_SIZE + 17],
                      best, second);
              corrs[n_corrs].pr = pd[16];
              corrs[n_corrs].pc = pd[17];
              memcpy (&corrs[n_corrs].p_ori, pd + 18, 2);
              corrs[n_corrs].p_ori = GUINT16_FROM_LE (corrs[n_corrs].p_ori);
              corrs[n_corrs].gr = gk[best_gi * MAFP_KP_SIZE + 16];
              corrs[n_corrs].gc = gk[best_gi * MAFP_KP_SIZE + 17];
              memcpy (&corrs[n_corrs].g_ori, gk + best_gi * MAFP_KP_SIZE + 18, 2);
              corrs[n_corrs].g_ori = GUINT16_FROM_LE (corrs[n_corrs].g_ori);
              n_corrs++;
              bank_matches++;
            }
        }
    }

  /* Deduplicate: keep only the best match per gallery keypoint */
  {
    int deduped = 0;
    for (int i = 0; i < n_corrs; i++)
      {
        /* Check if this gallery position was already matched by a better pair */
        gboolean dup = FALSE;
        for (int j = 0; j < deduped; j++)
          if (corrs[j].gr == corrs[i].gr && corrs[j].gc == corrs[i].gc)
            {
              dup = TRUE;
              break;
            }
        if (!dup)
          corrs[deduped++] = corrs[i];
      }
    fp_dbg ("match: %d correspondences (%d after dedup)", n_corrs, deduped);
    metrics->correspondences = n_corrs;
    metrics->unique_correspondences = deduped;
    n_corrs = deduped;
  }

  if (n_corrs < MAFP_MIN_MATCH_PTS)
    return 0;

  /* Exhaustive pairwise RANSAC */
  int best_inliers = 0;
  gdouble best_avg_dist = 1e9;

  for (int i = 0; i < n_corrs - 1; i++)
    for (int j = i + 1; j < n_corrs; j++)
      {
        /* Angular consistency: both correspondences must agree on rotation */
        guint16 dp = corrs[i].p_ori - corrs[j].p_ori;
        guint16 dg = corrs[i].g_ori - corrs[j].g_ori;
        gint32 adiff = (gint16) (dp - dg);
        if (adiff < 0)
          adiff = -adiff;
        if (adiff > 1822)  /* ~10 degrees */
          continue;
        metrics->orientation_consistent_pairs++;

        MafpCorr pair[2] = { corrs[i], corrs[j] };
        gdouble a, b, tx, ty;
        if (!mafp_solve_similarity (pair, &a, &b, &tx, &ty))
          continue;
        metrics->solved_transforms++;

        /* Count inliers */
        gdouble avg_d = 0;
        MafpCorr inliers_buf[MAFP_MAX_INLIERS];
        int inl = mafp_count_inliers (corrs, n_corrs, a, b, tx, ty,
                                      inliers_buf, &avg_d);

        if (inl > best_inliers ||
            (inl == best_inliers && avg_d < best_avg_dist))
          {
            best_inliers = inl;
            best_avg_dist = avg_d;
          }

        if (inl >= MAFP_MAX_INLIERS)
          {
            best_inliers = inl;
            best_avg_dist = avg_d;
            goto done;
          }

        if (inl >= MAFP_MIN_MATCH_PTS)
          {
            /* Refit transform from inliers */
            if (inl >= 2)
              {
                MafpCorr rpair[2] = { inliers_buf[0], inliers_buf[1] };
                gdouble ra, rb, rtx, rty;
                if (mafp_solve_similarity (rpair, &ra, &rb, &rtx, &rty))
                  {
                    gdouble ravg = 0;
                    int rinl = mafp_count_inliers (corrs, n_corrs,
                                                   ra, rb, rtx, rty,
                                                   NULL, &ravg);
                    if (rinl > inl || (rinl == inl && ravg < avg_d))
                      {
                        inl = rinl;
                        avg_d = ravg;
                      }
                  }
              }

            if (inl > best_inliers ||
                (inl == best_inliers && avg_d < best_avg_dist))
              {
                best_inliers = inl;
                best_avg_dist = avg_d;
              }
          }
      }

done:;
  int score = mafp_compute_match_score (best_inliers);

  metrics->inliers = best_inliers;
  fp_dbg ("match: inliers=%d avg_dist=%.1f score=%d (thresh=%d)",
          best_inliers, best_avg_dist, score, MAFP_MATCH_THRESH);
  return score;
}


typedef struct
{
  FpiDeviceAction action;
  guint           stage;
  guint           sample_count;
  guint8         *template_buffer;
  guint8          probe_template[MAFP_TPL_SAMPLE_SZ];
  gboolean        stable;
  gboolean        retry;
  guint           changed_pixels;
  guint           probe_keypoints;
  guint64         stability_sad;
} MafpActionContext;

enum mafp_action_state {
  MAFP_ACTION_ACQUIRE_PRESS,
  MAFP_ACTION_PROCESS_PRESS,
  MAFP_ACTION_WAIT_REMOVAL,
  MAFP_ACTION_ADVANCE_ENROLLMENT,
  MAFP_ACTION_COMPLETE,
  MAFP_ACTION_NUM_STATES,
};

static void
mafp_action_context_free (gpointer data)
{
  MafpActionContext *context = data;

  if (context->template_buffer)
    {
      mafp_clear_sensitive (context->template_buffer, MAFP_TPL_BUF_SZ);
      g_clear_pointer (&context->template_buffer, g_free);
    }
  mafp_clear_sensitive (context->probe_template,
                        sizeof (context->probe_template));
  g_free (context);
}

static gboolean
mafp_match_print (FpPrint      *print,
                  const guint8 *probe_template,
                  guint         probe_keypoints,
                  gboolean     *matched,
                  GError      **error)
{
  g_autoptr(GVariant) variant = NULL;
  const guint8 *template_buffer;
  gsize template_size = 0;
  guint sample_count = 0;

  *matched = FALSE;
  g_object_get (print, "fpi-data", &variant, NULL);
  if (!variant)
    {
      g_set_error_literal (error,
                           FP_DEVICE_ERROR,
                           FP_DEVICE_ERROR_DATA_INVALID,
                           "MAFP8800 print has no driver data");
      return FALSE;
    }

  template_buffer = g_variant_get_fixed_array (variant,
                                               &template_size,
                                               sizeof (guint8));
  if (!mafp8800_template_validate (template_buffer,
                                   template_size,
                                   &sample_count,
                                   error))
    return FALSE;

  for (guint index = 0; index < sample_count; index++)
    {
      const guint8 *sample =
        template_buffer + MAFP_TPL_HDR_SZ + index * MAFP_TPL_SAMPLE_SZ;
      MafpMatchMetrics metrics;
      guint32 encoded_bank0;
      guint32 encoded_bank1;
      guint gallery_keypoints;
      int score;

      memcpy (&encoded_bank0, sample + 4, sizeof (encoded_bank0));
      memcpy (&encoded_bank1,
              sample + 4 + MAFP_BANK_SZ,
              sizeof (encoded_bank1));
      gallery_keypoints = GUINT32_FROM_LE (encoded_bank0) +
                          GUINT32_FROM_LE (encoded_bank1);
      score = mafp_match_templates (probe_template, sample, &metrics);

      g_log ("libfprint-mafp8800-telemetry",
             G_LOG_LEVEL_DEBUG,
             "probe-keypoints=%u sample=%u/%u gallery-keypoints=%u "
             "correspondences=%u unique=%u oriented-pairs=%u transforms=%u "
             "inliers=%u score=%d threshold=%d",
             probe_keypoints,
             index + 1,
             sample_count,
             gallery_keypoints,
             metrics.correspondences,
             metrics.unique_correspondences,
             metrics.orientation_consistent_pairs,
             metrics.solved_transforms,
             metrics.inliers,
             score,
             MAFP_MATCH_THRESH);

      fp_dbg ("match sample %u/%u: score=%d threshold=%d",
              index + 1,
              sample_count,
              score,
              MAFP_MATCH_THRESH);
      if (score >= MAFP_MATCH_THRESH)
        {
          *matched = TRUE;
          break;
        }
    }

  return TRUE;
}

static void
mafp_action_handler (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceMafp8800 *self = FPI_DEVICE_MAFP8800 (device);
  MafpActionContext *context = fpi_ssm_get_data (ssm);

  g_autoptr(GError) error = NULL;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case MAFP_ACTION_ACQUIRE_PRESS:
      fpi_device_report_finger_status (device, FP_FINGER_STATUS_NEEDED);
      fpi_ssm_start_subsm (
        ssm,
        mafp8800_fp36_acquire_press_new (
          device,
          self->spi_fd,
          fpi_device_get_cancellable (device),
          self->gain,
          self->bg_frame,
          MAFP_FRAME_BYTES,
          self->cur_frame,
          MAFP_FRAME_BYTES,
          &context->stable,
          &context->changed_pixels,
          &context->stability_sad));
      return;

    case MAFP_ACTION_PROCESS_PRESS:
      if (!context->stable)
        {
          fp_dbg ("press remained unstable: changed=%u SAD=%"
                  G_GUINT64_FORMAT,
                  context->changed_pixels,
                  context->stability_sad);
          if (context->action == FPI_DEVICE_ACTION_ENROLL)
            {
              fpi_device_enroll_progress (
                device,
                context->stage,
                NULL,
                fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER));
              fpi_ssm_jump_to_state (ssm, MAFP_ACTION_ACQUIRE_PRESS);
            }
          else
            {
              context->retry = TRUE;
              fpi_ssm_jump_to_state (ssm, MAFP_ACTION_COMPLETE);
            }
          return;
        }

      fpi_device_report_finger_status_changes (device,
                                               FP_FINGER_STATUS_PRESENT,
                                               FP_FINGER_STATUS_NONE);
      if (!mafp_fp36_enhance (self, &error))
        {
          fpi_ssm_mark_failed (ssm, g_steal_pointer (&error));
          return;
        }

      if (context->action == FPI_DEVICE_ACTION_ENROLL)
        {
          guint keypoints;

          g_assert_cmpuint (context->sample_count, <, MAFP_ENROLL_STAGES);
          keypoints = mafp_extract_features (
            self->enhanced,
            context->template_buffer + MAFP_TPL_HDR_SZ +
            context->sample_count * MAFP_TPL_SAMPLE_SZ);
          g_log ("libfprint-mafp8800-telemetry",
                 G_LOG_LEVEL_DEBUG,
                 "enrollment-sample=%u/%u keypoints=%u",
                 context->sample_count + 1,
                 MAFP_ENROLL_STAGES,
                 keypoints);
          context->sample_count++;
          context->stage++;
          fpi_device_enroll_progress (device,
                                      context->stage,
                                      NULL,
                                      NULL);
          fpi_ssm_next_state (ssm);
        }
      else
        {
          context->probe_keypoints =
            mafp_extract_features (self->enhanced,
                                   context->probe_template);
          fpi_ssm_jump_to_state (ssm, MAFP_ACTION_WAIT_REMOVAL);
        }
      return;

    case MAFP_ACTION_WAIT_REMOVAL:
      fpi_ssm_start_subsm (
        ssm,
        mafp8800_fp36_wait_removal_new (
          device,
          self->spi_fd,
          fpi_device_get_cancellable (device),
          self->gain,
          self->bg_frame,
          MAFP_FRAME_BYTES,
          self->cur_frame,
          MAFP_FRAME_BYTES));
      return;

    case MAFP_ACTION_ADVANCE_ENROLLMENT:
      if (context->action != FPI_DEVICE_ACTION_ENROLL)
        {
          fpi_device_report_finger_status_changes (
            device,
            FP_FINGER_STATUS_NONE,
            FP_FINGER_STATUS_NEEDED | FP_FINGER_STATUS_PRESENT);
          fpi_ssm_jump_to_state (ssm, MAFP_ACTION_COMPLETE);
          return;
        }

      fpi_device_report_finger_status_changes (device,
                                               FP_FINGER_STATUS_NEEDED,
                                               FP_FINGER_STATUS_PRESENT);
      if (context->stage < MAFP_ENROLL_STAGES)
        fpi_ssm_jump_to_state (ssm, MAFP_ACTION_ACQUIRE_PRESS);
      else
        fpi_ssm_next_state (ssm);
      return;

    case MAFP_ACTION_COMPLETE:
      fpi_ssm_mark_completed (ssm);
      return;

    default:
      g_assert_not_reached ();
    }
}

static void
mafp_action_complete (FpiSsm *ssm, FpDevice *device, GError *error)
{
  MafpActionContext *context = fpi_ssm_get_data (ssm);

  if (error)
    {
      switch (context->action)
        {
        case FPI_DEVICE_ACTION_ENROLL:
          fpi_device_enroll_complete (device, NULL, error);
          break;

        case FPI_DEVICE_ACTION_VERIFY:
          fpi_device_verify_complete (device, error);
          break;

        case FPI_DEVICE_ACTION_IDENTIFY:
          fpi_device_identify_complete (device, error);
          break;

        case FPI_DEVICE_ACTION_NONE:
        case FPI_DEVICE_ACTION_PROBE:
        case FPI_DEVICE_ACTION_OPEN:
        case FPI_DEVICE_ACTION_CLOSE:
        case FPI_DEVICE_ACTION_CAPTURE:
        case FPI_DEVICE_ACTION_LIST:
        case FPI_DEVICE_ACTION_DELETE:
        case FPI_DEVICE_ACTION_CLEAR_STORAGE:
        default:
          g_assert_not_reached ();
        }
      return;
    }

  if (context->retry)
    {
      if (context->action == FPI_DEVICE_ACTION_VERIFY)
        {
          fpi_device_verify_report (
            device,
            FPI_MATCH_ERROR,
            NULL,
            fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER));
          fpi_device_verify_complete (device, NULL);
        }
      else
        {
          g_assert_cmpint (context->action, ==, FPI_DEVICE_ACTION_IDENTIFY);
          fpi_device_identify_report (
            device,
            NULL,
            NULL,
            fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER));
          fpi_device_identify_complete (device, NULL);
        }
      return;
    }

  if (context->action == FPI_DEVICE_ACTION_ENROLL)
    {
      FpPrint *print = NULL;
      GVariant *data;
      guint8 *stored_template;

      g_assert_cmpuint (context->sample_count, ==, MAFP_ENROLL_STAGES);
      mafp8800_template_set_sample_count (context->template_buffer,
                                          MAFP_TPL_BUF_SZ,
                                          context->sample_count);
      fpi_device_get_enroll_data (device, &print);
      stored_template = g_memdup2 (context->template_buffer,
                                   MAFP_TPL_BUF_SZ);
      data = g_variant_new_from_data (G_VARIANT_TYPE_BYTESTRING,
                                      stored_template,
                                      MAFP_TPL_BUF_SZ,
                                      TRUE,
                                      mafp_template_data_free,
                                      stored_template);
      fpi_print_set_type (print, FPI_PRINT_RAW);
      fpi_print_set_device_stored (print, FALSE);
      g_object_set (print, "fpi-data", data, NULL);
      fpi_device_enroll_complete (device, g_object_ref (print), NULL);
    }
  else if (context->action == FPI_DEVICE_ACTION_VERIFY)
    {
      g_autoptr(GError) match_error = NULL;
      FpPrint *enrolled = NULL;
      gboolean matched = FALSE;

      fpi_device_get_verify_data (device, &enrolled);
      if (!mafp_match_print (enrolled,
                             context->probe_template,
                             context->probe_keypoints,
                             &matched,
                             &match_error))
        {
          fpi_device_verify_complete (device,
                                      g_steal_pointer (&match_error));
          return;
        }

      fpi_device_verify_report (device,
                                matched ? FPI_MATCH_SUCCESS : FPI_MATCH_FAIL,
                                NULL,
                                NULL);
      fpi_device_verify_complete (device, NULL);
    }
  else
    {
      GPtrArray *gallery = NULL;
      FpPrint *matched_print = NULL;

      g_assert_cmpint (context->action, ==, FPI_DEVICE_ACTION_IDENTIFY);
      fpi_device_get_identify_data (device, &gallery);
      for (guint index = 0; index < gallery->len; index++)
        {
          g_autoptr(GError) match_error = NULL;
          FpPrint *candidate = g_ptr_array_index (gallery, index);
          gboolean matched = FALSE;

          if (!mafp_match_print (candidate,
                                 context->probe_template,
                                 context->probe_keypoints,
                                 &matched,
                                 &match_error))
            {
              fp_dbg ("Skipping invalid MAFP8800 gallery print: %s",
                      match_error->message);
              continue;
            }
          if (matched)
            {
              matched_print = candidate;
              break;
            }
        }

      fpi_device_identify_report (device, matched_print, NULL, NULL);
      fpi_device_identify_complete (device, NULL);
    }
}

static void
mafp_action_start (FpDevice *device)
{
  MafpActionContext *context = g_new0 (MafpActionContext, 1);
  FpiSsm *ssm;

  context->action = fpi_device_get_current_action (device);
  g_assert_true (context->action == FPI_DEVICE_ACTION_ENROLL ||
                 context->action == FPI_DEVICE_ACTION_VERIFY ||
                 context->action == FPI_DEVICE_ACTION_IDENTIFY);
  if (context->action == FPI_DEVICE_ACTION_ENROLL)
    {
      context->template_buffer = g_malloc0 (MAFP_TPL_BUF_SZ);
      mafp8800_template_initialize (context->template_buffer,
                                    MAFP_TPL_BUF_SZ);
    }

  ssm = fpi_ssm_new (device, mafp_action_handler, MAFP_ACTION_NUM_STATES);
  fpi_ssm_set_data (ssm, context, mafp_action_context_free);
  fpi_ssm_start (ssm, mafp_action_complete);
}

/* FpDevice callbacks */

enum mafp_open_state {
  MAFP_OPEN_RESET,
  MAFP_OPEN_WAIT_FOR_ID,
  MAFP_OPEN_READ_ID,
  MAFP_OPEN_CHECK_ID,
  MAFP_OPEN_CALIBRATE_GAIN,
  MAFP_OPEN_CAPTURE_BACKGROUND,
  MAFP_OPEN_COMPLETE,
  MAFP_OPEN_NUM_STATES,
};

static void
mafp_open_read_id_cb (FpiSpiTransfer *transfer,
                      FpDevice       *dev,
                      gpointer        user_data,
                      GError         *error)
{
  FpiDeviceMafp8800 *self = FPI_DEVICE_MAFP8800 (dev);

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, g_steal_pointer (&error));
      return;
    }

  self->chip_id = transfer->buffer_rd[2];
  fpi_ssm_next_state (transfer->ssm);
}

static void
mafp_open_handler (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceMafp8800 *self = FPI_DEVICE_MAFP8800 (dev);
  FpiSpiTransfer *transfer;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case MAFP_OPEN_RESET:
      transfer = mafp_set_reg_async (self, 0x8C, 0xFF);
      transfer->ssm = ssm;
      fpi_spi_transfer_submit (transfer,
                               fpi_device_get_cancellable (dev),
                               fpi_ssm_spi_transfer_cb,
                               NULL);
      return;

    case MAFP_OPEN_WAIT_FOR_ID:
      fpi_ssm_next_state_delayed (ssm, 1);
      return;

    case MAFP_OPEN_READ_ID:
      transfer = mafp_set_reg_async (self, 0x04, 0x00);
      transfer->ssm = ssm;
      fpi_spi_transfer_submit (transfer,
                               fpi_device_get_cancellable (dev),
                               mafp_open_read_id_cb,
                               NULL);
      return;

    case MAFP_OPEN_CHECK_ID:
      if (self->chip_id == MAFP_CHIPID_FP36)
        {
          fp_info ("detected FP36 chip ID 0x%02x", self->chip_id);
          fpi_ssm_next_state (ssm);
          return;
        }

      self->open_attempts++;
      if (self->open_attempts >= 20)
        {
          fpi_ssm_mark_failed (ssm,
                               fpi_device_error_new_msg (
                                 FP_DEVICE_ERROR_PROTO,
                                 "FP36 did not respond with chip ID 0x%02x "
                                 "(last response 0x%02x)",
                                 MAFP_CHIPID_FP36,
                                 self->chip_id));
          return;
        }

      fpi_ssm_jump_to_state (ssm, MAFP_OPEN_WAIT_FOR_ID);
      return;

    case MAFP_OPEN_CALIBRATE_GAIN:
      fpi_ssm_start_subsm (
        ssm,
        mafp8800_fp36_calibrate_gain_new (
          dev,
          self->spi_fd,
          fpi_device_get_cancellable (dev),
          &self->gain));
      return;

    case MAFP_OPEN_CAPTURE_BACKGROUND:
      fpi_ssm_start_subsm (
        ssm,
        mafp8800_fp36_capture_new (
          dev,
          self->spi_fd,
          fpi_device_get_cancellable (dev),
          self->gain,
          0x02,
          0xA1,
          self->bg_frame,
          MAFP_FRAME_BYTES));
      return;

    case MAFP_OPEN_COMPLETE:
      fpi_ssm_mark_completed (ssm);
      return;

    default:
      g_assert_not_reached ();
    }
}

static void
mafp_release_resources (FpiDeviceMafp8800 *self)
{
  if (self->bg_frame)
    mafp_clear_sensitive (self->bg_frame, MAFP_FRAME_BYTES);
  if (self->cur_frame)
    mafp_clear_sensitive (self->cur_frame, MAFP_FRAME_BYTES);
  if (self->enhanced)
    mafp_clear_sensitive (self->enhanced,
                          MAFP_ENHANCED_PIXELS * sizeof (guint16));
  g_clear_pointer (&self->bg_frame, g_free);
  g_clear_pointer (&self->cur_frame, g_free);
  g_clear_pointer (&self->enhanced, g_free);

  if (self->spi_fd >= 0)
    {
      close (self->spi_fd);
      self->spi_fd = -1;
    }
  self->gain = 0;
}

static void
mafp_open_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceMafp8800 *self = FPI_DEVICE_MAFP8800 (dev);

  if (error)
    {
      mafp_release_resources (self);
      fpi_device_open_complete (dev, error);
      return;
    }

  fpi_device_open_complete (dev, NULL);
}

static void
mafp_open (FpDevice *dev)
{
  FpiDeviceMafp8800 *self = FPI_DEVICE_MAFP8800 (dev);
  const char *path = fpi_device_get_udev_data (dev, FPI_DEVICE_UDEV_SUBTYPE_SPIDEV);
  FpiSsm *ssm;

  fp_info ("opening %s", path ? path : "(null)");

  if (!path)
    {
      fpi_device_open_complete (dev,
                                fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL, "no spidev path"));
      return;
    }

  self->spi_fd = open (path, O_RDWR | O_CLOEXEC);
  if (self->spi_fd < 0)
    {
      fpi_device_open_complete (dev,
                                fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                          "open %s: %s", path, g_strerror (errno)));
      return;
    }

  self->bg_frame = g_malloc0 (MAFP_FRAME_BYTES);
  self->cur_frame = g_malloc0 (MAFP_FRAME_BYTES);
  self->enhanced = g_new0 (guint16, MAFP_ENHANCED_PIXELS);

  /* Mode, word size, and maximum speed come from the ACPI SpiSerialBus
   * descriptor. All data traffic goes through FpiSpiTransfer. */
  self->open_attempts = 0;
  self->chip_id = 0;
  ssm = fpi_ssm_new (dev, mafp_open_handler, MAFP_OPEN_NUM_STATES);
  fpi_ssm_start (ssm, mafp_open_complete);
}

static void
mafp_close (FpDevice *dev)
{
  FpiDeviceMafp8800 *self = FPI_DEVICE_MAFP8800 (dev);

  mafp_release_resources (self);
  fpi_device_close_complete (dev, NULL);
}

static void
mafp_enroll (FpDevice *dev)
{
  mafp_action_start (dev);
}
static void
mafp_verify (FpDevice *dev)
{
  mafp_action_start (dev);
}

static void
mafp_cancel (FpDevice *dev)
{
  /* The core has already cancelled fpi_device_get_cancellable(). Every
   * transfer and delayed acquisition state observes that object. */
}

/* GObject boilerplate */

static const FpIdEntry mafp_id_table[] = {
  { .udev_types = FPI_DEVICE_UDEV_SUBTYPE_SPIDEV,
    .spi_acpi_id = "MAFP8800", .driver_data = 0 },
  { .udev_types = 0 }
};

static void
fpi_device_mafp8800_init (FpiDeviceMafp8800 *self)
{
  self->spi_fd = -1;
}

static void
fpi_device_mafp8800_finalize (GObject *obj)
{
  FpiDeviceMafp8800 *self = FPI_DEVICE_MAFP8800 (obj);

  mafp_release_resources (self);
  G_OBJECT_CLASS (fpi_device_mafp8800_parent_class)->finalize (obj);
}

static void
fpi_device_mafp8800_class_init (FpiDeviceMafp8800Class *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);

  dev_class->id               = "mafp8800";
  dev_class->full_name        = "Microarray MAFP Fingerprint Sensor";
  dev_class->type             = FP_DEVICE_TYPE_UDEV;
  dev_class->id_table         = mafp_id_table;
  dev_class->scan_type        = FP_SCAN_TYPE_PRESS;
  dev_class->nr_enroll_stages = MAFP_ENROLL_STAGES;

  dev_class->open     = mafp_open;
  dev_class->close    = mafp_close;
  dev_class->enroll   = mafp_enroll;
  dev_class->verify   = mafp_verify;
  dev_class->identify = mafp_verify;
  dev_class->cancel   = mafp_cancel;

  G_OBJECT_CLASS (klass)->finalize = fpi_device_mafp8800_finalize;

  fpi_device_class_auto_initialize_features (dev_class);
}
