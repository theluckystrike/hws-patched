#include "preset_table.h"

void VideoScaler(u8 *src, u8 *dst,
                 int in_w,int in_h,int out_w,int out_h)
{
        /* linear search is fine (<40 rows). Binary if needed */
        for (const struct scaler_preset *p = scaler_presets; p->in_w; p++) {
                if (p->in_w==in_w && p->in_h==in_h &&
                    p->out_w==out_w && p->out_h==out_h) {
                        p->fn(src,dst,in_w,in_h,out_w,out_h);
                        return;
                }
        }
        all_videoscaler(src,dst,in_w,in_h,out_w,out_h);
}
