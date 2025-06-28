/* scaler/util/fill_nv12_yuy2.c */
#include "fill_nv12_yuy2.h"
#include "../core/linecopy.h"

static void nv12_progressive(u8 *src, u8 *dst,
                             u16 w, u16 h, u16 src_pitch,
                             u16 packet_cnt);

static void nv12_interlaced (u8 *src, u8 *dst,
                             u16 w, u16 h, u16 src_pitch,
                             u16 packet_cnt);

/* --- local helpers (prototypes only) -------------------------- */
static void convert_line_pattern0(const u8 *y,
                                  const u8 *uv,
                                  u8       *dst,
                                  u16       packets,
                                  bool      swap_uv);

static void convert_line_pattern2(const u8 *y,
                                  const u8 *uv,
                                  u8       *dst,
                                  u16       packets);

static void modify_luma_only(const u8 *y_src,
                             u8       *dst,
                             u16       w);

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


/* -------------------------------------------------------------- */
/* Progressive NV12-to-YUY2 (height unchanged)                    */
/* Pattern repeats every 4 source lines:                          */
/*   0 = full Y+UV, maybe UV-swap                                 */
/*   1 = duplicate prev line, overwrite Y only                    */
/*   2 = duplicate prev line, copy Y in 24-byte packets           */
/*   3 = duplicate prev line, overwrite Y only                    */
/* -------------------------------------------------------------- */
static void nv12_progressive(u8 *src,u8 *dst,
                             u16 w,u16 h,u16 pitch,u16 packets)
{
        const u8 *src_line = src;        /* points to start of each NV12 line   */
        u8       *dst_line = dst;        /* points to start of each YUY2 line   */
        bool      swap_uv  = false;      /* flipped every “pattern-0” iteration */

        for (u16 line = 0; line < h; ++line) {

                /* Calculate ‘pattern’ 0-1-2-3 repeating */
                switch (line & 3) {

                /* -------- pattern 0: full Y and alternating UV ------------- */
                case 0:
                        convert_line_pattern0(src_line,
                                              src_line + w,  /* UV plane starts after Y plane */
                                              dst_line,
                                              packets,
                                              swap_uv);

                        swap_uv = !swap_uv;        /* match original bUVSel flip */
                        break;

                /* -------- pattern 1 & 3: duplicate + Y overwrite ----------- */
                case 1:
                case 3:
                        copy_prev_line(dst_line, w);                /* duplicate previous line  */
                        modify_luma_only(src_line, dst_line, w);    /* overwrite Y samples only */
                        break;

                /* -------- pattern 2: duplicate + mixed Y chunks ------------ */
                case 2:
                        copy_prev_line(dst_line, w);
                        convert_line_pattern2(src_line,
                                              src_line + w,
                                              dst_line,
                                              packets);
                        break;
                }

                /* advance to next source & destination lines */
                src_line += pitch;
                dst_line += w * 2;       /* YUY2: 2 bytes per pixel */
        }

        /* bottom padding: one black line keeps even field count */
        fill_black_line(dst + (h - 1) * w * 2, w);
}

/* ------------------------------------------------------------------ *
 *  NV12-to-YUY2 — interlaced mode
 *      • Converts the *even* source field into two identical YUY2     *
 *        lines (top + bottom).                                        *
 *      • Fills chroma alternately by swapping UV pairs every other    *
 *        2-line group, matching legacy bUVSel behaviour.              *
 *      • Duplicates each generated line once more to create           *
 *        the missing field.                                           *
 *      • Adds a 2-line black pad at the bottom (like the old code).   *
 * ------------------------------------------------------------------ */
static void nv12_interlaced(const u8 *src,
                            u8       *dst,
                            u16       w,
                            u16       h,
                            u16       pitch,     /* src pitch in bytes */
                            u16       packets)   /* 24-byte packets per line */
{
        const u16 half = h / 2;                 /* only convert top field */
        const size_t yuy2_pitch = w * 2;

        const u8 *y_line  = src;
        u8       *dst_top = dst;                /* first line of YUY2 pair */

        /* pattern repeats every 4 *source* lines (bLineSel) */
        u8 line_sel = 0;
        bool uv_swap = false;                   /* toggled inside pattern 0 */

        for (u16 line = 0; line < half; ++line) {

                switch (line_sel) {

                /* -------- pattern 0 (convert Y+UV, maybe swap) ---------- */
                case 0:
                        {
                                const u8 *uv = y_line + w;           /* UV plane */
                                u8 *dst_ptr  = dst_top;

                                for (u16 p = 0; p < packets; ++p) {
                                        nv12_chunk_to_yuy2(dst_ptr, y_line, uv, uv_swap);
                                        y_line += 24;  uv += 24;  dst_ptr += 32;
                                }
                                /* toggle just like legacy bUVSel */
                                uv_swap = !uv_swap;
                        }
                        break;

                /* -------- pattern 1 & 3 (duplicate + overwrite Y only) -- */
                case 1: case 3:
                        copy_prev_line(dst_top, w);                    /* duplicate previous YUY2 line */
                        modify_luma_only(y_line, dst_top, w);          /* overwrite Y samples only     */
                        break;

                /* -------- pattern 2 (duplicate + chunk copy) ---------- */
                case 2:
                        copy_prev_line(dst_top, w);
                        {
                                const u8 *uv = y_line + w;
                                u8 *dst_ptr  = dst_top;
                                for (u16 p = 0; p < packets; ++p) {
                                        nv12_chunk_to_yuy2(dst_ptr, y_line, uv, /*swap=*/false);
                                        y_line += 24; uv += 24; dst_ptr += 32;
                                }
                        }
                        break;
                }

                /* --------------------------------------------- *
                 *  After writing the “top” line, duplicate it   *
                 *  to create the bottom field (“copy up line”). *
                 * --------------------------------------------- */
                memcpy(dst_top + yuy2_pitch, dst_top, yuy2_pitch);

                /* advance to next line pair */
                dst_top += yuy2_pitch * 2;      /* skip 2 YUY2 lines */
                y_line  += pitch;               /* next NV12 line    */

                /* rotate the pattern counter */
                line_sel = (line_sel + 1) & 3;
        }

        /* -------- bottom black pad: 2 lines of 0x10/0x80 ------------ */
        fill_black_line(dst_top,             w);
        fill_black_line(dst_top + yuy2_pitch, w);
}


static void convert_line_pattern0(const u8 *y,
                                  const u8 *uv,
                                  u8       *dst,
                                  u16       packets,
                                  bool      swap_uv)
{
        for (u16 p = 0; p < packets; ++p) {
                nv12_chunk_to_yuy2(dst, y, uv, swap_uv);
                y   += 24;           /* advance 16 Y + 8 UV bytes in NV12   */
                uv  += 24;
                dst += 32;           /* advance 16 pixels (32 bytes) YUY2   */
        }
}

static void convert_line_pattern2(const u8 *y,
                                  const u8 *uv,
                                  u8       *dst,
                                  u16       packets)
{
        /* Same as pattern-0 but UV always pulled from current line (no swap) */
        for (u16 p = 0; p < packets; ++p) {
                nv12_chunk_to_yuy2(dst, y, uv, /*swap_uv=*/false);
                y   += 24;
                uv  += 24;
                dst += 32;
        }
}

static void modify_luma_only(const u8 *y_src,
                             u8       *dst,
                             u16       w)
{
        for (u16 x = 0; x < w; ++x) {
                dst[0] = y_src[x];   /* write Y  */
                dst    += 2;         /* skip UV byte */
        }
}

