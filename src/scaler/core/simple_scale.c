// SPDX-License-Identifier: GPL-2.0
#include "simple_scale.h"
#include <linux/string.h>

static inline void fill_black_line(u8 *dst, u16 w)
{
        for (u16 i = 0; i < w; ++i) {
                dst[0] = 0x10;   /* Y  = 16 (black)   */
                dst[1] = 0x80;   /* Cb = 128 neutral  */
                dst += 2;
        }
}

/* ------------------------------------------------------------------------- */
int crop_downsample(u8 *dst, const u8 *src,
                    u16 iw, u16 ih,
                    u16 ow, u16 oh,
                    u16 crop_l, u16 crop_t,
                    u8  step_x, u8 step_y)
{
        if (WARN_ON_ONCE(!src || !dst || !step_x || !step_y))
                return -EINVAL;
        if (WARN_ON_ONCE(iw <= crop_l || ih <= crop_t))
                return -EINVAL;
        if (WARN_ON_ONCE((iw - crop_l * 2) / step_x != ow))
                return -EINVAL;
        if (WARN_ON_ONCE((ih - crop_t * 2) / step_y != oh))
                return -EINVAL;

        src += (crop_t * iw + crop_l) * 2;          /* byte pointer */
        for (u16 y = 0; y < oh; y++) {
                const u8 *line = src + (y * step_y) * iw * 2;
                u8 *dst_px = dst + y * ow * 2;

                for (u16 x = 0; x < ow; x++) {
                        const u8 *px = line + (x * step_x) * 2;
                        dst_px[0] = px[0];
                        dst_px[1] = px[1];
                        dst_px += 2;
                }
        }
        return 0;
}

/* ------------------------------------------------------------------------- */
void pad_copy_vertical(u8 *dst, const u8 *src,
                       u16 in_w, u16 in_h, u16 out_h,
                       u16 pad_top)
{
        const u16 line_bytes = in_w * 2;
        const u16 pad_bottom = out_h - in_h - pad_top;

        /* head pad */
        for (u16 i = 0; i < pad_top; ++i) {
                fill_black_line(dst, in_w);
                dst += line_bytes;
        }

        /* active */
        memcpy(dst, src, (size_t)in_h * line_bytes);
        dst += in_h * line_bytes;

        /* tail pad */
        for (u16 i = 0; i < pad_bottom; ++i) {
                fill_black_line(dst, in_w);
                dst += line_bytes;
        }
}

/* ------------------------------------------------------------------------- */
void pad_copy_two_line(u8 *dst, const u8 *src,
                       u16 iw, u16 ih, u16 ow, u16 oh,
                       u16 pad_horiz, u16 pad_top,
                       u8  step_y, bool duplicate)
{
        const u16 act_w   = ow - pad_horiz * 2;
        const u16 dst_pitch = ow * 2;
        const u16 src_pitch = iw * 2;

        /* top pad */
        for (u16 i = 0; i < pad_top; ++i) {
                fill_black_line(dst, ow);
                dst += dst_pitch;
        }

        for (u16 y = 0; y < ih; y += step_y) {
                const u8 *src_line = src + y * src_pitch;
                u8 *dst_line = dst;

                /* left bar */
                fill_black_line(dst_line, pad_horiz);
                dst_line += pad_horiz * 2;

                /* active pixels */
                memcpy(dst_line, src_line, act_w * 2);
                dst_line += act_w * 2;

                /* right bar */
                fill_black_line(dst_line, pad_horiz);

                if (duplicate) {
                        dst += dst_pitch;
                        memcpy(dst, dst - dst_pitch, dst_pitch);
                }
                dst += dst_pitch;
        }

        /* bottom pad to reach oh lines */
        while ((uintptr_t)(dst) < (uintptr_t)(dst_pitch * oh)) {
                fill_black_line(dst, ow);
                dst += dst_pitch;
        }
}

