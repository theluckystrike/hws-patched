/* scaler/presets/preset_table.c */
#include "preset_table.h"

#define PRE(iw,ih,ow,oh,fn)  { iw, ih, ow, oh, fn }

/* Keep sorted by input resolution for cache locality */
const struct scaler_preset scaler_presets[] = {
    /* 1920×1080 inputs */
    PRE(1920,1080,1280, 720, FHD_To_HD_Scaler),
    PRE(1920,1080, 800, 600, FHD_To_800X600_Scaler),
    PRE(1920,1080, 720, 480, FHD_To_SD_NTSC_Scaler),
    PRE(1920,1080, 720, 576, FHD_To_SD_PAL_Scaler),

    /* 1280×720 inputs */
    PRE(1280, 720,1920,1080, HD_To_FHD_Scaler),
    PRE(1280, 720, 800, 600, HD_To_800X600_Scaler),
    PRE(1280, 720, 720, 480, HD_To_SD_NTSC_Scaler),
    PRE(1280, 720, 720, 576, HD_To_SD_PAL_Scaler),

    /*  720×480 (NTSC) inputs */
    PRE( 720, 480, 720, 576, SD_NTSC_To_SD_PAL_Scaler),
    PRE( 720, 480,1920,1080, SD_NTSC_To_FHD_Scaler),
    PRE( 720, 480,1280, 720, SD_NTSC_To_HD_Scaler),

    /*  720×576 (PAL) inputs */
    PRE( 720, 576, 720, 480, SD_PAL_To_SD_NTSC_Scaler),
    PRE( 720, 576,1920,1080, SD_PAL_To_FHD_Scaler),
    PRE( 720, 576,1280, 720, SD_PAL_To_HD_Scaler),

    /* 1280×1024 inputs */
    PRE(1280,1024, 720, 480, V1280X1024_NTSC_Scaler),
    PRE(1280,1024, 720, 576, V1280X1024_PAL_Scaler),
    PRE(1280,1024, 800, 600, V1280X1024_To_800X600_Scaler),
    PRE(1280,1024,1920,1080, V1280X1024_To_FHD_Scaler),
    PRE(1280,1024,1280, 720, V1280X1024_To_HD_Scaler),

    /* terminator */
    { 0 }
};
