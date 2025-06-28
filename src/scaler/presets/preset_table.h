/* scaler/presets/preset_table.h */
struct scaler_row {
        u16 in_w, in_h, out_w, out_h;
        u8  op;            /* 0=cropDown, 1=padCopy */
        u16 crop_l, crop_t;
        u16 pad_l,  pad_t;
        u8  step_x, step_y;
        bool dup_line;
};

/* EXTRACTED from your old functions */
#define ROW(iw,ih,ow,oh,op,cl,ct,pl,pt,sx,sy,dup) \
        {iw,ih,ow,oh,op,cl,ct,pl,pt,sx,sy,dup}

static const struct scaler_row scaler_rows[] = {
        /* FHD 1920×1080 → 800×600  crop 240px left/right, take 1 of every 3 lines */
        ROW(1920,1080, 800, 600, 0, 240,0, 0,0,  3,3, false),

        /* HD 1280×720 → 800×600  crop top/bot 60 lines, take every line */
        ROW(1280, 720, 800, 600, 1, 0,60, 0,0,  1,1, true),

        /* SD NTSC (720×480) → PAL (720×576) 48-line black pad top/bot */
        ROW( 720, 480, 720, 576, 1, 0,0, 0,48, 1,1, false),

        /* PAL → NTSC (just crop 48 lines) */
        ROW( 720, 576, 720, 480, 0, 0,48, 0,0, 1,1, false),

        /* NTSC → FHD  pad 240 px left/right, copy 2-line step, duplicate line */
        ROW( 720, 480,1920,1080, 1, 0,60, 240,60, 1,2, true),

        /* other rows … */
        {0}
};
