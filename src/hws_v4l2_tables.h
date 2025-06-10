/*  
 * hws_v4l2_tables.h
 *
 * Declarations for static lookup tables used by the HWS framegrabber
 * driver: supported model‐timings, pixel‐formats, and refresh‐rates.
 */

#ifndef HWS_V4L2_TABLES_H
#define HWS_V4L2_TABLES_H

#include <linux/types.h>
#include <media/v4l2-device.h>    /* for v4l2_model_timing_t, V4L2_MODEL_TIMING */
#include "hws.h"
#include "hws_reg.h"

/* --- supported video‐format timings --- */
extern const v4l2_model_timing_t support_videofmt[];
extern const size_t              support_videofmt_count;

/* --- supported pixel‐formats --- */
extern const framegrabber_pixfmt_t support_pixfmts[];
extern const size_t                support_pixfmts_count;

/* --- supported refresh-rates --- */
extern const int framegrabber_support_refreshrate[];
extern const size_t num_framerate_controls;

const framegrabber_pixfmt_t *
framegrabber_g_support_pixelfmt_by_fourcc(u32 fourcc);
framegrabber_pixfmt_t *v4l2_model_get_support_pixformat(int index);
v4l2_model_timing_t *v4l2_model_get_support_videoformat(int index);
int v4l2_model_get_support_framerate(int index);
int v4l2_get_suport_VideoFormatIndex(struct v4l2_format *fmt);
v4l2_model_timing_t *Get_input_framesizeIndex(int width, int height);

void framegrabber_g_Curr_input_framesize(struct hws_video *dev, int *width,
					 int *height);
const framegrabber_pixfmt_t *framegrabber_g_out_pixelfmt(struct hws_video *dev);

#endif /* HWS_V4L2_TABLES_H */

