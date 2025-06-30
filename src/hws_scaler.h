/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HWS_SCALER_H
#define HWS_SCALER_H

#include <linux/types.h>
#include "hws.h"

/* Core scaler functions */
void VideoScaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);
void All_VideoScaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);
void VideoRotate90deg(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);

/* Full-HD → 800×600, HD, SD conversions */
void FHD_To_800X600_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);
void FHD_To_HD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);
void FHD_To_SD_NTSC_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);
void FHD_To_SD_PAL_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);

/* HD → 800×600, Full-HD, SD conversions */
void HD_To_800X600_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);
void HD_To_FHD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);
void HD_To_SD_NTSC_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);
void HD_To_SD_PAL_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);

/* SD NTSC → Full-HD, HD, SD PAL conversions */
void SD_NTSC_To_FHD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);
void SD_NTSC_To_HD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);
void SD_NTSC_To_SD_PAL_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);

/* SD PAL → Full-HD, HD, SD NTSC conversions */
void SD_PAL_To_FHD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);
void SD_PAL_To_HD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);
void SD_PAL_To_SD_NTSC_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);

/* 1280×1024 → NTSC/PAL/800×600/Full-HD/HD */
void V1280X1024_NTSC_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);
void V1280X1024_PAL_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);
void V1280X1024_To_800X600_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h,int out_w,int out_h);
void V1280X1024_To_FHD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);
void V1280X1024_To_HD_Scaler(BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h);

/* Pixel-format & interlace helpers */
void SetDeInterlace(BYTE *pSrc, BYTE *pOut, int nw, int nh, int interlace);
void SetNoVideoMem(uint8_t *pDest, int w, int h);
void FillNV12ToYUY2(BYTE *pSrc, BYTE *pOut, int nw, int nh, int interlace);
void FillYUU2(BYTE *pSrc, BYTE *pOut, int nw, int nh, int interlace);

#endif /* HWS_SCALER_H */
