/* SPDX-License-Identifier: GPL-2.0 */
#ifndef SCALER_SIMPLE_SCALE_H
#define SCALER_SIMPLE_SCALE_H

#include <linux/types.h>
#include <linux/bug.h>

/**
 * crop_downsample() - crop L/T and nearest-neighbour down-sample
 *
 * dst      packed-YUY2 output buffer (ow × oh × 2 bytes)
 * src      packed-YUY2 input  buffer (iw × ih × 2 bytes)
 * iw,ih    input  resolution
 * ow,oh    output resolution
 * crop_l   pixels to skip on the left   (≥0, <iw)
 * crop_t   pixels to skip on the top    (≥0, <ih)
 * step_x   copy every step_x-th pixel   (1 = none, 2 = half, 3 = ⅓ …)
 * step_y   copy every step_y-th line    (1 = none, 2 = half …)
 *
 * Returns 0 on success, -EINVAL on bad params.
 */
int crop_downsample(u8 *dst, const u8 *src,
                    u16 iw, u16 ih,
                    u16 ow, u16 oh,
                    u16 crop_l, u16 crop_t,
                    u8  step_x, u8 step_y);

/**
 * pad_copy_vertical() – add black bars (top & bottom) then memcpy active lines.
 *
 * pad_top  – number of black YUY2 lines to add at top (same at bottom)
 */
void pad_copy_vertical(u8 *dst, const u8 *src,
                       u16 in_w, u16 in_h, u16 out_h,
                       u16 pad_top);

/**
 * pad_copy_two_line() – pattern used by NTSC→HD/FHD etc.
 *
 *  • Horizontal black bars (pad_horiz) both sides of each copied line
 *  • Optional vertical pad (pad_top) before first active line
 *  • Copies every step_y-th source line.
 *  • If duplicate == true, it duplicates each generated line once more.
 */
void pad_copy_two_line(u8 *dst, const u8 *src,
                       u16 iw, u16 ih, u16 ow, u16 oh,
                       u16 pad_horiz, u16 pad_top,
                       u8  step_y, bool duplicate);

#endif /* SCALER_SIMPLE_SCALE_H */

