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
#include "hws_framegrabber.h"     /* for framegrabber_pixfmt_t, REFRESHRATE_* enums */

/* --- supported video‐format timings --- */
extern const v4l2_model_timing_t support_videofmt[];
extern const size_t              support_videofmt_count;

/* --- supported pixel‐formats --- */
extern const framegrabber_pixfmt_t support_pixfmts[];
extern const size_t                support_pixfmts_count;

/* --- supported refresh-rates --- */
extern const int   framegrabber_support_refreshrate[];
extern const size_t framegrabber_support_refreshrate_count;

#endif /* HWS_V4L2_TABLES_H */

