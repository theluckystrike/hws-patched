/* scaler/util/fill_nv12_yuy2.c */
#include "fill_nv12_yuy2.h"
#include "../core/linecopy.h"

static void nv12_progressive(u8 *src, u8 *dst,
                             u16 w, u16 h, u16 src_pitch,
                             u16 packet_cnt);
static void nv12_interlaced (u8 *src, u8 *dst,
                             u16 w, u16 h, u16 src_pitch,
                             u16 packet_cnt);

void nv12_to_yuy2(u8 *src, u8 *dst, u16 w, u16 h, bool interlaced)
{
        const u16 src_pitch  = w * 12 / 8;   /* NV12 pitch */
        const u16 packets    = src_pitch / 24;

        if (interlaced)
                nv12_interlaced(src,dst,w,h,src_pitch,packets);
        else
                nv12_progressive(src,dst,w,h,src_pitch,packets);
}

/* ───────── helpers ───────────────────────────────────────── */

static inline void nv12_chunk_to_yuy2(u8 *dst,
                                      const u8 *y16,
                                      const u8 *uv8,
                                      bool swap_uv)
{
        copy_luma_16(dst,     y16);          /* y0-y15  */
        copy_chroma_8(dst, uv8, swap_uv);    /* two UV pairs */
}

/* progressive scan */
static void nv12_progressive(u8 *src,u8 *dst,
                             u16 w,u16 h,u16 pitch,u16 packets)
{
        for (u16 line=0; line<h; line++) {
                bool  swap_uv  = (line & 1);
                u8   *dst_line = dst + line*w*2;
                const u8 *yptr = src + line*pitch;
                const u8 *uvptr= yptr + w;

                for (u16 p=0; p<packets; p++) {
                        nv12_chunk_to_yuy2(dst_line, yptr, uvptr, swap_uv);
                        yptr  += 24;
                        uvptr += 24;
                        dst_line += 32;
                }
        }
}

// FIXME
static void nv12_progressive(…)
{
    for (u16 line=0; line<h; ++line) {
        switch (line & 3) {
        case 0:
            convert_line_pattern0(…);
            break;
        case 1:
        case 3:
            copy_prev_line(dst_line, w);
            modify_luma_only(dst_line, src_line, w);
            break;
        case 2:
            copy_prev_line(dst_line, w);
            convert_line_pattern2(…);
            break;
        }
        /* advance pointers … */
    }
    fill_black_line(dst + (h-1)*w*2, w);   /* bottom pad */
}

/* interlaced = half-height processing + duplicate lines */
static void nv12_interlaced(u8 *src,u8 *dst,
                            u16 w,u16 h,u16 pitch,u16 packets)
{
        const u16 half = h/2;
        for (u16 line=0; line<half; line++) {
                bool swap_uv   = (line & 1);
                u8  *dst_line  = dst + line*2*w*2;      /* two YUY2 lines */
                const u8 *y    = src + line*pitch;
                const u8 *uv   = y + w;

                for (u16 p=0; p<packets; p++) {
                        nv12_chunk_to_yuy2(dst_line, y, uv, swap_uv);
                        /* copy the same 32 bytes to the next interlaced line */
                        memcpy(dst_line + w*2, dst_line, 32);

                        y  += 24; uv += 24; dst_line += 32;
                }
        }

        /* bottom black pad (two lines) */
        fill_black_line(dst + (h-2)*w*2, w);
        fill_black_line(dst + (h-1)*w*2, w);
}

