#include "hws_v4l2_tables.h"
#include <linux/kernel.h>
#include "hws.h"
#include "hws_reg.h"

#define NUM_FRAMERATE_CONTROLS (ARRAY_SIZE(framegrabber_support_refreshrate))

const v4l2_model_timing_t support_videofmt[] = {
	[V4L2_MODEL_VIDEOFORMAT_1920X1080P60] =
		V4L2_MODEL_TIMING(1920, 1080, 60, 0),
	[V4L2_MODEL_VIDEOFORMAT_1280X720P60] =
		V4L2_MODEL_TIMING(1280, 720, 60, 0),
	[V4L2_MODEL_VIDEOFORMAT_720X480P60] =
		V4L2_MODEL_TIMING(720, 480, 60, 0),
	[V4L2_MODEL_VIDEOFORMAT_720X576P50] =
		V4L2_MODEL_TIMING(720, 480, 50, 0),
	[V4L2_MODEL_VIDEOFORMAT_800X600P60] =
		V4L2_MODEL_TIMING(800, 600, 60, 0),
	[V4L2_MODEL_VIDEOFORMAT_640X480P60] =
		V4L2_MODEL_TIMING(640, 480, 60, 0),
	[V4L2_MODEL_VIDEOFORMAT_1024X768P60] =
		V4L2_MODEL_TIMING(1024, 768, 60, 0),
	[V4L2_MODEL_VIDEOFORMAT_1280X768P60] =
		V4L2_MODEL_TIMING(1280, 768, 60, 0),
	[V4L2_MODEL_VIDEOFORMAT_1280X800P60] =
		V4L2_MODEL_TIMING(1280, 800, 60, 0),
	[V4L2_MODEL_VIDEOFORMAT_1280X1024P60] =
		V4L2_MODEL_TIMING(1280, 1024, 60, 0),
	[V4L2_MODEL_VIDEOFORMAT_1360X768P60] =
		V4L2_MODEL_TIMING(1360, 768, 60, 0),
	[V4L2_MODEL_VIDEOFORMAT_1440X900P60] =
		V4L2_MODEL_TIMING(1440, 900, 60, 0),
	[V4L2_MODEL_VIDEOFORMAT_1680X1050P60] =
		V4L2_MODEL_TIMING(1680, 1050, 60, 0),
	[V4L2_MODEL_VIDEOFORMAT_1080X1920P60] =
		V4L2_MODEL_TIMING(1080, 1920, 60, 0),
#if 0
    [V4L2_MODEL_VIDEOFORMAT_1920X1200P60]	= V4L2_MODEL_TIMING(1920,1200,60,0),
    [V4L2_MODEL_VIDEOFORMAT_2560X1080P60]	= V4L2_MODEL_TIMING(2560,1080,60,0),
    [V4L2_MODEL_VIDEOFORMAT_2560X1440P60]	= V4L2_MODEL_TIMING(2560,1440,60,0),
    [V4L2_MODEL_VIDEOFORMAT_3840X2160P60]	= V4L2_MODEL_TIMING(3840,2160,60,0),
    [V4L2_MODEL_VIDEOFORMAT_4096X2160P60]	= V4L2_MODEL_TIMING(4096,2160,60,0),
#endif
};

const size_t support_videofmt_count =
    ARRAY_SIZE(support_videofmt);


const framegrabber_pixfmt_t support_pixfmts[] = {
	
	[FRAMEGRABBER_PIXFMT_YUYV]={ //YUYV index=0
		.name     = "4:2:2, packed, YUYV",
		.fourcc   = V4L2_PIX_FMT_YUYV,
		.depth    = 16,
		.is_yuv   = true,
		.pixfmt_out = YUYV,
	},
#if 0
	[FRAMEGRABBER_PIXFMT_UYVY]={ //UYVY
		.name     = "4:2:2, packed, UYVY",
		.fourcc   = V4L2_PIX_FMT_UYVY,
		.depth    = 16,
		.is_yuv   = true,
		.pixfmt_out = UYVY,
	},
	[FRAMEGRABBER_PIXFMT_YVYU]={ //YVYU
		.name     = "4:2:2, packed, YVYU",
		.fourcc   = V4L2_PIX_FMT_YVYU,
		.depth    = 16,
		.is_yuv   = true,
		.pixfmt_out = YVYU,
	},
	
	[FRAMEGRABBER_PIXFMT_VYUY]={ //VYUY
		.name     = "4:2:2, packed, VYUY",
		.fourcc   = V4L2_PIX_FMT_VYUY,
		.depth    = 16,
		.is_yuv   = true,
		.pixfmt_out = VYUY,
	},

	[FRAMEGRABBER_PIXFMT_RGB565]={ //RGBP
		.name     = "RGB565 (LE)",
		.fourcc   = V4L2_PIX_FMT_RGB565, /* gggbbbbb rrrrrggg */
		.depth    = 16,
		.is_yuv   = false,
		.pixfmt_out = RGBP,
	},
	[FRAMEGRABBER_PIXFMT_RGB565X]={ //RGBR
		.name     = "RGB565 (BE)",
		.fourcc   = V4L2_PIX_FMT_RGB565X, /* rrrrrggg gggbbbbb */
		.depth    = 16,
		.is_yuv   = false,
		.pixfmt_out = RGBR,
	},
	[FRAMEGRABBER_PIXFMT_RGB555]={ //RGBO
		.name     = "RGB555 (LE)",
		.fourcc   = V4L2_PIX_FMT_RGB555, /* gggbbbbb arrrrrgg */
		.depth    = 16,
		.is_yuv   = false,
		.pixfmt_out = RGBO,
	},
	[FRAMEGRABBER_PIXFMT_RGB555X]={ //RGBQ
		.name     = "RGB555 (BE)",
		.fourcc   = V4L2_PIX_FMT_RGB555X, /* arrrrrgg gggbbbbb */
		.depth    = 16,
		.is_yuv   = false,
		.pixfmt_out = RGBQ,
	},
	[FRAMEGRABBER_PIXFMT_RGB24]={ //RGB3 index=8
		.name     = "RGB24 (LE)",
		.fourcc   = V4L2_PIX_FMT_RGB24, /* rgb */
		.depth    = 24,
		.is_yuv   = false,
		.pixfmt_out = RGB3,
	},

	[FRAMEGRABBER_PIXFMT_BGR24]={ //BGR3
		.name     = "RGB24 (BE)",
		.fourcc   = V4L2_PIX_FMT_BGR24, /* bgr */
		.depth    = 24,
		.is_yuv   = false,
		.pixfmt_out = BGR3,
	},
	[FRAMEGRABBER_PIXFMT_RGB32]={ //RGB4
		.name     = "RGB32 (LE)",
		.fourcc   = V4L2_PIX_FMT_RGB32, /* argb */
		.depth    = 32,
		.is_yuv   = false,
		.pixfmt_out = RGB4,
	},
	[FRAMEGRABBER_PIXFMT_BGR32]={ //BGR4
		.name     = "RGB32 (BE)",
		.fourcc   = V4L2_PIX_FMT_BGR32, /* bgra */
		.depth    = 32,
		.is_yuv   = false,
		.pixfmt_out = BGR4,
	},
#endif
};

const size_t support_pixfmts_count =
    ARRAY_SIZE(support_pixfmts);

const int framegrabber_support_refreshrate[] = {
	[REFRESHRATE_15] = 15,	 [REFRESHRATE_24] = 24,
	[REFRESHRATE_25] = 25,	 [REFRESHRATE_30] = 30,
	[REFRESHRATE_50] = 50,	 [REFRESHRATE_60] = 60,
	[REFRESHRATE_100] = 100, [REFRESHRATE_120] = 120,
	[REFRESHRATE_144] = 144, [REFRESHRATE_240] = 240,
};

const size_t framegrabber_support_refreshrate_count =
    ARRAY_SIZE(framegrabber_support_refreshrate);

int v4l2_model_get_support_framerate(int index)
{
	if (index < 0 || index >= NUM_FRAMERATE_CONTROLS)
		return -1;

	return (framegrabber_support_refreshrate[index]);
}

int v4l2_get_suport_VideoFormatIndex(struct v4l2_format *fmt)
{
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	int index;
	int videoIndex = -1;
	for (index = 0; index < V4L2_MODEL_VIDEOFORMAT_NUM; index++) {
		if ((pix->width == support_videofmt[index].frame_size.width) &&
		    (pix->height ==
		     support_videofmt[index].frame_size.height)) {
			videoIndex = index;
			break;
		}
	}
	return videoIndex;
}

v4l2_model_timing_t *v4l2_model_get_support_videoformat(int index)
{
	if (index < 0 || index >= V4L2_MODEL_VIDEOFORMAT_NUM)
		return NULL;

	return (v4l2_model_timing_t *)&support_videofmt[index];
}

framegrabber_pixfmt_t *v4l2_model_get_support_pixformat(int index)
{
	if (index < 0 || index >= ARRAY_SIZE(support_pixfmts))
		return NULL;

	return (framegrabber_pixfmt_t *)&support_pixfmts[index];
}

const framegrabber_pixfmt_t *
framegrabber_g_support_pixelfmt_by_fourcc(u32 fourcc)
{
	int i;
	int pixfmt_index = -1;
	for (i = 0; i < FRAMEGRABBER_PIXFMT_MAX; i++) {
		if (support_pixfmts[i].fourcc == fourcc) {
			pixfmt_index = i;
			break;
		}
	}
	if (pixfmt_index == -1)
		return NULL;

	return &support_pixfmts[pixfmt_index];
}

void framegrabber_g_Curr_input_framesize(struct hws_video *dev, int *width,
					 int *height)
{
	struct hws_pcie_dev *pdx = dev->dev;
	int index = dev->index;
	*width = pdx->m_pVCAPStatus[index][0].dwWidth;
	*height = pdx->m_pVCAPStatus[index][0].dwHeight;
}

const framegrabber_pixfmt_t *framegrabber_g_out_pixelfmt(struct hws_video *dev)
{
	return &support_pixfmts[dev->current_out_pixfmt];
}

v4l2_model_timing_t *Get_input_framesizeIndex(int width, int height)
{
	int i;
	for (i = 0; i < V4L2_MODEL_VIDEOFORMAT_NUM; i++) {
		if ((support_videofmt[i].frame_size.width == width) &&
		    (support_videofmt[i].frame_size.height == height)) {
			return (v4l2_model_timing_t *)&support_videofmt[i];
		}
	}
	return NULL;
}
