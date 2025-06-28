#define SCALE_KEY(iw,ih,ow,oh) \
  ((((uint64_t)(iw))<<48) | (((uint64_t)(ih))<<32) | \
   (((uint64_t)(ow))<<16) |  (uint64_t)(oh))

/* ------------------------------------------------------------------ */
/* 1) Crop-and-Downsample                                            */
/*    – Crop left/top (pixels, lines)                                 */
/*    – Copy every stepX-th pixel horizontally and stepY-th line      */
#define CASE_CROP(iw,ih,ow,oh, cropL,cropT, stepX,stepY)            \
    case SCALE_KEY(iw,ih,ow,oh):                                    \
        crop_downsample(dst, src,                                    \
                        (iw),(ih),(ow),(oh),                        \
                        (cropL),(cropT),                             \
                        (stepX),(stepY));                            \
        return;

/* ------------------------------------------------------------------ */
/* 2) Vertical Pad-and-Copy                                           */
/*    – out_w == in_w                                                 */
/*    – padTop black lines are inserted at top & bottom               */
#define CASE_VPAD(iw,ih,ow,oh, padTop)                              \
    case SCALE_KEY(iw,ih,ow,oh):                                    \
        pad_copy_vertical(dst, src,                                  \
                          (iw),(ih),(oh),                            \
                          (padTop));                                 \
        return;

/* ------------------------------------------------------------------ */
/* 3) Pad-Copy-Pad two-line step                                     */
/*    – Horizontal black bars (padL) on left & right                  */
/*    – Vertical top pad (padTop) before active picture               */
/*    – Copies every stepY-th source line, optionally duplicating it  */
#define CASE_PCP(iw,ih,ow,oh, padL,padTop, stepY, dupLine)          \
    case SCALE_KEY(iw,ih,ow,oh):                                    \
        pad_copy_two_line(dst, src,                                  \
                          (iw),(ih),(ow),(oh),                       \
                          (padL),(padTop),                           \
                          (stepY), (dupLine));                       \
        return;

void run_scaler(u8 *src, u8 *dst,
                int in_w,int in_h,
                int out_w,int out_h)
{
    uint64_t key = SCALE_KEY(in_w, in_h, out_w, out_h);

    switch (key) {

        /* ───────── Crop + Downsample (integer 3→2) ───────── */
        /* 1920×1080 → 1280×720   (crop 240 px L/R, keep every 3rd px/line) */
        CASE_CROP(1920,1080,1280, 720,   240,0,   3,3)

        /* 1280×720  →  800×600   (crop 0/60, then pad later) */
        CASE_CROP(1280, 720, 800, 600,     0,60,  1,1)

        /* 1920×1080 →  800×600   (crop 240 px L/R, copy 5 of every 18 px) */
        /* keeps fast hand-tuned version because ratio isn’t an integer step */
        case SCALE_KEY(1920,1080,800,600):
                FHD_To_800X600_Scaler(src,dst,1920,1080,800,600);
                return;

        /* ───────── Vertical pad + copy (black bars) ───────── */
        /* NTSC 720×480 → PAL 720×576  (48-line top + 48-line bottom) */
        CASE_VPAD( 720, 480, 720, 576, 48)

        /* PAL 720×576 → NTSC 720×480  (crop 48-line top + bottom) */
        /* Achieved via crop helper: */
        CASE_CROP(720,576,720,480, 0,48, 1,1)

        /* ───────── Pad-Copy-Pad two-line stride cases ─────── */
        /* NTSC 720×480 → FHD 1920×1080  (240-px side pads, 60-line top, dup lines) */
        CASE_PCP( 720, 480,1920,1080, 240,60, 2, true)

        /* PAL  720×576 → FHD 1920×1080  (240-px pads, 18-line top, dup lines) */
        CASE_PCP( 720, 576,1920,1080, 240,18, 2, true)

        /* PAL 720×576 → HD 1280×720  (240-px pads, 108-line top, dup lines) */
        CASE_PCP( 720, 576,1280, 720, 240,108,2, true)

        /* NTSC 720×480 → HD 1280×720  (240-px pads, 60-line top, dup lines) */
        CASE_PCP( 720, 480,1280, 720, 240,60, 2, true)

        /* 1280×1024 pillar-box to HD and FHD use pad-copy only */
        CASE_VPAD(1280,1024,1920,1080,  0)   /* pad top/bot handled inside */
        CASE_VPAD(1280,1024,1280, 720, 152)  /* 152-line crop top/bot */

        /* 1280×1024 → 720×576 / 720×480 have bespoke fast paths */
        case SCALE_KEY(1280,1024,720,576):
                V1280X1024_PAL_Scaler(src,dst,1280,1024,720,576);
                return;
        case SCALE_KEY(1280,1024,720,480):
                V1280X1024_NTSC_Scaler(src,dst,1280,1024,720,480);
                return;

        /* 1280×720 → SD NTSC / PAL (crop and memcpy only) */
        CASE_CROP(1280,720,720,480, 0,120, 1,1)   /* top crop */
        CASE_CROP(1280,720,720,576, 0, 72, 1,1)
    default:
        /* fallback generic scaler */
        all_video_scaler(src, dst,
                         in_w, in_h,
                         out_w, out_h);
    }
}

