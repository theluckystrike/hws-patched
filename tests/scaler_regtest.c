/* SPDX-License-Identifier: MIT
 *
 * Unit/regression test – prove the new generic scaler produces *exactly*
 * the same bytes as the old hand-coded routines.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

/* ------------------------------------------------------------------ */
/*  Legacy prototypes – implemented in legacy_scalers.c               */
typedef uint8_t  u8;          /* keep the naming consistent          */
typedef uint32_t DWORD;

void FHD_To_800X600_Scaler   (u8*,u8*,int,int,int,int);
void HD_To_800X600_Scaler    (u8*,u8*,int,int,int,int);
void SD_NTSC_To_SD_PAL_Scaler(u8*,u8*,int,int,int,int);
void SD_PAL_To_SD_NTSC_Scaler(u8*,u8*,int,int,int,int);
void SD_NTSC_To_FHD_Scaler   (u8*,u8*,int,int,int,int);
void SD_PAL_To_FHD_Scaler    (u8*,u8*,int,int,int,int);
void SD_PAL_To_HD_Scaler     (u8*,u8*,int,int,int,int);
void SD_NTSC_To_HD_Scaler    (u8*,u8*,int,int,int,int);
void FHD_To_HD_Scaler        (u8*,u8*,int,int,int,int);
void HD_To_FHD_Scaler        (u8*,u8*,int,int,int,int);
void FHD_To_SD_NTSC_Scaler   (u8*,u8*,int,int,int,int);
void FHD_To_SD_PAL_Scaler    (u8*,u8*,int,int,int,int);
void HD_To_SD_NTSC_Scaler    (u8*,u8*,int,int,int,int);
void HD_To_SD_PAL_Scaler     (u8*,u8*,int,int,int,int);
void V1280X1024_NTSC_Scaler  (u8*,u8*,int,int,int,int);
void V1280X1024_PAL_Scaler   (u8*,u8*,int,int,int,int);

/* ------------------------------------------------------------------ */
/*  New generic scaler                                                */
void run_scaler(u8 *src,u8 *dst,
                int in_w,int in_h,
                int out_w,int out_h);    /* implemented in new code   */

/* ------------------------------------------------------------------ */
struct case_info {
        const char *name;
        int  iw, ih, ow, oh;
        void (*legacy)(u8*,u8*,int,int,int,int);
};

static struct case_info cases[] = {
        { "FHD→800×600"        ,1920,1080, 800, 600, FHD_To_800X600_Scaler   },
        { "HD →800×600"        ,1280, 720, 800, 600, HD_To_800X600_Scaler    },
        { "720×480→576 (pad)"  , 720, 480, 720, 576, SD_NTSC_To_SD_PAL_Scaler},
        { "720×576→480 (pad)"  , 720, 576, 720, 480, SD_PAL_To_SD_NTSC_Scaler},
        { "NTSC→FHD"           , 720, 480,1920,1080, SD_NTSC_To_FHD_Scaler   },
        { "PAL →FHD"           , 720, 576,1920,1080, SD_PAL_To_FHD_Scaler    },
        { "PAL →HD"            , 720, 576,1280, 720, SD_PAL_To_HD_Scaler     },
        { "NTSC→HD"            , 720, 480,1280, 720, SD_NTSC_To_HD_Scaler    },
        { "FHD→HD"             ,1920,1080,1280, 720, FHD_To_HD_Scaler        },
        { "HD →FHD"            ,1280, 720,1920,1080, HD_To_FHD_Scaler        },
        { "FHD→SD 480"         ,1920,1080, 720, 480, FHD_To_SD_NTSC_Scaler   },
        { "FHD→SD 576"         ,1920,1080, 720, 576, FHD_To_SD_PAL_Scaler    },
        { "HD →SD 480"         ,1280, 720, 720, 480, HD_To_SD_NTSC_Scaler    },
        { "HD →SD 576"         ,1280, 720, 720, 576, HD_To_SD_PAL_Scaler     },
        { "1280×1024→480"      ,1280,1024, 720, 480, V1280X1024_NTSC_Scaler  },
        { "1280×1024→576"      ,1280,1024, 720, 576, V1280X1024_PAL_Scaler   },
};

static void pattern_fill(u8 *buf, size_t bytes)
{
        for (size_t i = 0; i < bytes; ++i)
                buf[i] = (u8)((i * 73 + 17) & 0xff);  /* deterministic noise */
}

int main(void)
{
        for (size_t t = 0; t < sizeof(cases)/sizeof(cases[0]); ++t) {
                const struct case_info *c = &cases[t];

                const size_t in_sz  = (size_t)c->iw * c->ih * 2;
                const size_t out_sz = (size_t)c->ow * c->oh * 2;

                u8 *src = malloc(in_sz);
                u8 *gold= malloc(out_sz);
                u8 *test= malloc(out_sz);
                assert(src && gold && test);

                /* reproducible pseudo-video frame */
                pattern_fill(src, in_sz);
                memset(gold, 0, out_sz);
                memset(test, 0, out_sz);

                /* 1. legacy */
                c->legacy(src, gold, c->iw, c->ih, c->ow, c->oh);

                /* 2. new generic */
                run_scaler(src, test, c->iw, c->ih, c->ow, c->oh);

                /* compare */
                if (memcmp(gold, test, out_sz) != 0) {
                        fprintf(stderr,
                                "❌  MISMATCH  %-18s %4dx%-4d → %4dx%-4d\n",
                                c->name, c->iw, c->ih, c->ow, c->oh);
                        free(src); free(gold); free(test);
                        return 1;
                } else {
                        printf("✔️   PASS      %-18s %4dx%-4d → %4dx%-4d\n",
                               c->name, c->iw, c->ih, c->ow, c->oh);
                }

                free(src); free(gold); free(test);
        }

        puts("\nAll scaler outputs are identical – regression test PASSED.");
        return 0;
}

