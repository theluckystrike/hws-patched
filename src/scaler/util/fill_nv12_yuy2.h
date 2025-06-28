#ifndef FILL_NV12_YUY2_H
#define FILL_NV12_YUY2_H
#include <linux/types.h>

void nv12_to_yuy2(u8 *src, u8 *dst,
                  u16 width, u16 height,
                  bool interlaced);

#endif
