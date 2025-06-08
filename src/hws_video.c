/*
*/

#include <linux/pci.h>
#include <linux/kernel.h>
#include <media/videobuf2-core.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-dma-contig.h>
#include "hws.h"
#include "hws_reg.h"
#include "hws_scaler.h"
#include "hws_interrupt.h"

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

static void hws_adapters_init(struct hws_pcie_dev *dev);
static void hws_get_video_param(struct hws_pcie_dev *dev, int index);
//static void StartDma(int index);
//static void StopDma(int index);

//struct hws_pcie_dev  *sys_dvrs_hw_pdx=NULL;
//EXPORT_SYMBOL(sys_dvrs_hw_pdx);
//u32 *map_bar0_addr=NULL; //for sys bar0
//EXPORT_SYMBOL(map_bar0_addr);

//------------------------------------------

#define MAKE_ENTRY(__vend, __chip, __subven, __subdev, __configptr) \
	{ .vendor = (__vend),                                       \
	  .device = (__chip),                                       \
	  .subvendor = (__subven),                                  \
	  .subdevice = (__subdev),                                  \
	  .driver_data = (unsigned long)(__configptr) }

static const struct pci_device_id hws_pci_table[] = {
	MAKE_ENTRY(0x8888, 0x9534, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x8534, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x8554, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8524, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x6524, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8504, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x6504, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8532, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8512, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8501, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x6502, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x8504, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x8524, 0x8888, 0x0007, NULL),

	{}
};
//------------------
static const v4l2_model_timing_t support_videofmt[] = {
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
static const framegrabber_pixfmt_t support_pixfmts[] = {
	
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

static const int framegrabber_support_refreshrate[] = {
	[REFRESHRATE_15] = 15,	 [REFRESHRATE_24] = 24,
	[REFRESHRATE_25] = 25,	 [REFRESHRATE_30] = 30,
	[REFRESHRATE_50] = 50,	 [REFRESHRATE_60] = 60,
	[REFRESHRATE_100] = 100, [REFRESHRATE_120] = 120,
	[REFRESHRATE_144] = 144, [REFRESHRATE_240] = 240,
};
#define NUM_FRAMERATE_CONTROLS (ARRAY_SIZE(framegrabber_support_refreshrate))

//-------------------------------------------
static int v4l2_model_get_support_framerate(int index)
{
	if (index < 0 || index >= NUM_FRAMERATE_CONTROLS)
		return -1;

	return (framegrabber_support_refreshrate[index]);
}

static int v4l2_get_suport_VideoFormatIndex(struct v4l2_format *fmt)
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

static int hws_vidioc_querycap(struct file *file, void *priv,
			       struct v4l2_capability *cap)
{
	struct hws_video *videodev = video_drvdata(file);
	struct hws_pcie_dev *dev = videodev->dev;
	int vi_index;
	vi_index = videodev->index + 1 +
		   dev->m_Device_PortID * dev->m_nCurreMaxVideoChl;
	//printk( "%s\n", __func__);
	strcpy(cap->driver, KBUILD_MODNAME);
	sprintf(cap->card, "%s %d", HWS_VIDEO_NAME, vi_index);
	sprintf(cap->bus_info, "HWS-%s-%d", HWS_VIDEO_NAME, vi_index);
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	//printk( "%s(IN END  )\n", __func__);
	return 0;
}
static int hws_vidioc_enum_fmt_vid_cap(struct file *file, void *priv_fh,
				       struct v4l2_fmtdesc *f)
{
	struct hws_video *videodev = video_drvdata(file);
	int index = f->index;
	//printk( "%s(%d)\n", __func__,videodev->index);
	//printk( "%s(f->index = %d)\n", __func__,f->index);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		printk("%s.\n", __func__);
		return -EINVAL;
	}
	if (videodev) {
		const framegrabber_pixfmt_t *pixfmt;
		if (f->index < 0) {
			return -EINVAL;
		}
		if (f->index >= FRAMEGRABBER_PIXFMT_MAX) {
			return -EINVAL;
		} else {
			pixfmt = v4l2_model_get_support_pixformat(f->index);
			if (pixfmt == NULL)
				return -EINVAL;
			//printk("%s..pixfmt=%d.\n",__func__,f->index);
			f->index = index;
			f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			memcpy(f->description, pixfmt->name,
			       strlen(pixfmt->name));
			f->pixelformat = pixfmt->fourcc;
		}
	}
	return 0;
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
static int hws_vidioc_g_fmt_vid_cap(struct file *file, void *fh,
				    struct v4l2_format *fmt)
{
	struct hws_video *videodev = video_drvdata(file);
	//struct v4l2_pix_format *pix = &fmt->fmt.pix;
	const framegrabber_pixfmt_t *pixfmt;
	v4l2_model_timing_t *p_SupportmodeTiming;
	//printk( "%s(%d)\n", __func__,videodev->index);
	//printk( "w=%d,h=%d\n",fmt->fmt.pix.width,fmt->fmt.pix.height);
	pixfmt = framegrabber_g_out_pixelfmt(videodev);
	if (pixfmt) {
		//framegrabber_g_Curr_input_framesize(videodev,&width,&height);
		p_SupportmodeTiming = v4l2_model_get_support_videoformat(
			videodev->current_out_size_index);
		if (p_SupportmodeTiming == NULL)
			return -EINVAL;
		fmt->fmt.pix.width = p_SupportmodeTiming->frame_size.width;
		fmt->fmt.pix.height = p_SupportmodeTiming->frame_size.height;
		fmt->fmt.pix.field = V4L2_FIELD_NONE; //Field
		fmt->fmt.pix.pixelformat = pixfmt->fourcc;
		fmt->fmt.pix.bytesperline =
			(fmt->fmt.pix.width * pixfmt->depth) >> 3;
		fmt->fmt.pix.sizeimage =
			fmt->fmt.pix.height * fmt->fmt.pix.bytesperline;
		fmt->fmt.pix.colorspace = V4L2_COLORSPACE_REC709;
		//printk("%s....f->fmt.pix.width=%d.f->fmt.pix.height=%d.\n",__func__,fmt->fmt.pix.width,fmt->fmt.pix.height);
		return 0;
	}

	return -EINVAL;
}

static v4l2_model_timing_t *Get_input_framesizeIndex(int width, int height)
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

static int hws_vidioc_try_fmt_vid_cap(struct file *file, void *fh,
				      struct v4l2_format *f)
{
	struct hws_video *videodev = video_drvdata(file);
	v4l2_model_timing_t *pModeTiming;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	const framegrabber_pixfmt_t *fmt;
	//int TimeingIndex = f->index;
	//printk( "%s(%d)\n", __func__,videodev->index);
	//printk( "pix->height =%d  pix->width =%d \n", pix->height,pix->width);
	fmt = framegrabber_g_support_pixelfmt_by_fourcc(pix->pixelformat);
	if (!fmt) {
		printk("%s.. format not support \n", __func__);
		return -EINVAL;
	}
	pModeTiming = Get_input_framesizeIndex(pix->width, pix->height);
	if (!pModeTiming) {
		printk("%s.. format2 not support  %dX%d\n", __func__,
		       pix->width, pix->height);
		pModeTiming = v4l2_model_get_support_videoformat(
			videodev->current_out_size_index);
		if (pModeTiming == NULL)
			return -EINVAL;
		pix->field = V4L2_FIELD_NONE;
		pix->width = pModeTiming->frame_size.width;
		pix->height = pModeTiming->frame_size.height;
		pix->bytesperline = (pix->width * fmt->depth) >> 3;
		pix->sizeimage = pix->height * pix->bytesperline;
		pix->colorspace =
			V4L2_COLORSPACE_REC709; //V4L2_COLORSPACE_SMPTE170M;
		pix->priv = 0;
		//return -EINVAL;
		return 0;
	}
	pix->field = V4L2_FIELD_NONE;
	pix->width = pModeTiming->frame_size.width;
	pix->height = pModeTiming->frame_size.height;
	pix->bytesperline = (pix->width * fmt->depth) >> 3;
	pix->sizeimage = pix->height * pix->bytesperline;
	pix->colorspace = V4L2_COLORSPACE_REC709; //V4L2_COLORSPACE_SMPTE170M;
	pix->priv = 0;

	//printk("%s<<pix->width=%d.pix->height=%d.\n",__func__,pix->width,pix->height);
	return 0;

	//----------------------------------
	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct hws_video *videodev = video_drvdata(file);
	int nVideoFmtIndex;
	int err;
	unsigned long flags;
	struct hws_pcie_dev *pdx = videodev->dev;
	//printk( "%s()\n", __func__);
	nVideoFmtIndex = v4l2_get_suport_VideoFormatIndex(f);
	if (nVideoFmtIndex == -1)
		return -EINVAL;

	err = hws_vidioc_try_fmt_vid_cap(file, priv, f);
	if (0 != err)
		return err;
	spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
	videodev->current_out_size_index = nVideoFmtIndex;
	videodev->pixfmt = f->fmt.pix.pixelformat;
	videodev->current_out_width = f->fmt.pix.width;
	videodev->curren_out_height = f->fmt.pix.height;
	//printk("%s<<  current_out_size_index =%d current_out_width=%d.curren_out_height=%d.\n",__func__,videodev->current_out_size_index,videodev->current_out_width ,videodev->curren_out_height );
	spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);
	return 0;
}
static int hws_vidioc_g_std(struct file *file, void *priv, v4l2_std_id *tvnorms)
{
	struct hws_video *videodev = video_drvdata(file);
	//printk( "%s()\n", __func__);
	*tvnorms = videodev->std;
	return 0;
}

static int hws_vidioc_s_std(struct file *file, void *priv, v4l2_std_id tvnorms)
{
	struct hws_video *videodev = video_drvdata(file);
	//printk( "%s()\n", __func__);
	videodev->std = tvnorms;
	return 0;
}

int hws_vidioc_g_parm(struct file *file, void *fh,
		      struct v4l2_streamparm *setfps)
{
	struct hws_video *videodev = video_drvdata(file);
	v4l2_model_timing_t *p_SupportmodeTiming;

	//printk( "%s(%d) Frame Rate =%d \n", __func__, videodev->index,io_frame_rate);
	if (setfps->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		printk("%s..\n", __func__);
		return -EINVAL;
	}
	p_SupportmodeTiming = v4l2_model_get_support_videoformat(
		videodev->current_out_size_index);
	if (p_SupportmodeTiming == NULL)
		return -EINVAL;
	setfps->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	setfps->parm.capture.timeperframe.numerator = 1000;
	setfps->parm.capture.timeperframe.denominator =
		p_SupportmodeTiming->refresh_rate * 1000;
	//printk( "%s fps =%d \n", __func__, p_SupportmodeTiming->refresh_rate);
	return 0;
}

static int hws_vidioc_enum_framesizes(struct file *file, void *fh,
				      struct v4l2_frmsizeenum *fsize)
{
	//struct hws_video *videodev = video_drvdata(file);
	const framegrabber_pixfmt_t *pixfmt;
	v4l2_model_timing_t *p_SupportmodeTiming;
	int width = 0, height = 0;
	int frameRate;
	//printk( "%s(%d)-FrameIndex=[%d]\n", __func__,videodev->index,fsize->index);
	//----------------------------
	pixfmt = framegrabber_g_support_pixelfmt_by_fourcc(fsize->pixel_format);
	if (pixfmt == NULL) {
		//printk("%s..\n",__func__);
		return -EINVAL;
	}
	p_SupportmodeTiming = v4l2_model_get_support_videoformat(fsize->index);
	if (p_SupportmodeTiming == NULL) {
		//printk("%s. invalid framesize[%d]\n",__func__,fsize->index);
		return -EINVAL;
	}
	width = p_SupportmodeTiming->frame_size.width;
	height = p_SupportmodeTiming->frame_size.height;
	frameRate = p_SupportmodeTiming->refresh_rate;

	//printk("%s...supportframesize[%d] width=%d height=%d Framerate=%d..\n",__func__,fsize->index,width,height,frameRate); //12
	if ((width == 0) || (height == 0)) {
		//printk("%s. invalid framesize 2\n",__func__);
		return -EINVAL;
	}
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->pixel_format = pixfmt->fourcc;
	fsize->discrete.width = width;
	fsize->discrete.height = height;
	//fsize->discrete.denominator = frameRate;
	//fsize->discrete..numerator =1 ;
	//-------------------------------
	//width = videodev->current_out_width;
	//height = videodev->curren_out_height;
	//fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	//fsize->pixel_format=videodev->pixfmt;
	//fsize->discrete.width=width;
	//fsize->discrete.height=height;
	return 0;
}

static int hws_vidioc_enum_input(struct file *file, void *priv,
				 struct v4l2_input *i)
{
	//struct hws_video *videodev = video_drvdata(file);
	int Index;
	Index = i->index;
	//printk( "%s(%d)-%d Index =%d \n", __func__,videodev->index,i->index,Index);
	if (Index > 0) {
		return -EINVAL;
	}
	i->type = V4L2_INPUT_TYPE_CAMERA;
	strcpy(i->name, KBUILD_MODNAME);
	i->std = V4L2_STD_NTSC_M;
	i->capabilities = 0;
	i->status = 0;

	return 0;
}

static int hws_vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	//struct hws_video *videodev = video_drvdata(file);
	int Index;
	Index = *i;
	//printk( "%s(%d)-index =%d\n", __func__,videodev->index,Index);

#if 0
	if(Index <0 ||Index >=V4L2_MODEL_VIDEOFORMAT_NUM)
	{
		return   -EINVAL;
	}
	else
	{
		*i = Index;
	}
#else
	if (Index > 0) {
		return -EINVAL;
	} else {
		*i = 0;
	}
#endif
	return 0;
}

static int hws_vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
#if 0
	struct hws_video *videodev = video_drvdata(file);

	int Index;
	v4l2_model_timing_t *p_SupportmodeTiming;
	Index = i;
	if(Index <0 ||Index >=V4L2_MODEL_VIDEOFORMAT_NUM)
	{
		return   -EINVAL;
	}
	p_SupportmodeTiming = v4l2_model_get_support_videoformat(Index);
	videodev->current_out_size_index = Index;
	videodev->current_out_width = p_SupportmodeTiming->frame_size.width;
	videodev->curren_out_height = p_SupportmodeTiming->frame_size.height;
	printk( "%s(%d)- %dx%d \n", __func__,i,videodev->current_out_width,videodev->curren_out_height);
#endif
	//printk( "%s(%d)\n", __func__,i);
	return i ? -EINVAL : 0;
}
static int vidioc_log_status(struct file *file, void *priv)
{
	//printk( "%s()\n", __func__);
	return 0;
}

static ssize_t hws_read(struct file *file, char *buf, size_t count,
			loff_t *ppos)
{
	//printk( "%s()\n", __func__);
	return -1;
}

static int hws_open(struct file *file)
{
	struct hws_video *videodev = video_drvdata(file);
	//v4l2_model_timing_t *p_SupportmodeTiming;
	unsigned long flags;
	struct hws_pcie_dev *pdx = videodev->dev;
	//printk( "%s(ch-%d)->%d\n", __func__,videodev->index,videodev->fileindex);
	spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
	videodev->fileindex++;
	spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);
	//printk( "%s(ch-%d)END ->%d W=%d H=%d \n", __func__,videodev->index,videodev->fileindex,videodev->current_out_width,videodev->curren_out_height);
	return 0;
}

static int hws_release(struct file *file)
{
	struct hws_video *videodev = video_drvdata(file);
	unsigned long flags;
	struct hws_pcie_dev *pdx = videodev->dev;
	//printk( "%s(ch-%d)->%d\n", __func__,videodev->index,videodev->fileindex);
	spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
	if (videodev->fileindex > 0) {
		videodev->fileindex--;
	}
	spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);
	//printk( "%s(ch-%d)->%d done\n", __func__,videodev->index,videodev->fileindex);

	if (videodev->fileindex == 0) {
		if (videodev->startstreamIndex > 0) {
			//printk( "StopVideoCapture %s(%d)->%d [%d]\n", __func__,videodev->index,videodev->fileindex,videodev->startstreamIndex);
			StopVideoCapture(videodev->dev, videodev->index);
			videodev->startstreamIndex = 0;
		}
		return (vb2_fop_release(file));
	} else {
		return 0;
	}
}

//-------------------
static const struct v4l2_queryctrl g_no_ctrl = {
	.name = "42",
	.flags = V4L2_CTRL_FLAG_DISABLED,
};
static struct v4l2_queryctrl g_hws_ctrls[] = {
#if 1
	{
		V4L2_CID_BRIGHTNESS, //id
		V4L2_CTRL_TYPE_INTEGER, //type
		"Brightness", //name[32]
		MIN_VAMP_BRIGHTNESS_UNITS, //minimum
		MAX_VAMP_BRIGHTNESS_UNITS, //maximum
		1, //step
		BrightnessDefault, //default_value
		0, //flags
		{ 0, 0 }, //reserved[2]
	},
	{
		V4L2_CID_CONTRAST, //id
		V4L2_CTRL_TYPE_INTEGER, //type
		"Contrast", //name[32]
		MIN_VAMP_CONTRAST_UNITS, //minimum
		MAX_VAMP_CONTRAST_UNITS, //maximum
		1, //step
		ContrastDefault, //default_value
		0, //flags
		{ 0, 0 }, //reserved[2]
	},
	{
		V4L2_CID_SATURATION, //id
		V4L2_CTRL_TYPE_INTEGER, //type
		"Saturation", //name[32]
		MIN_VAMP_SATURATION_UNITS, //minimum
		MAX_VAMP_SATURATION_UNITS, //maximum
		1, //step
		SaturationDefault, //default_value
		0, //flags
		{ 0, 0 }, //reserved[2]
	},
	{
		V4L2_CID_HUE, //id
		V4L2_CTRL_TYPE_INTEGER, //type
		"Hue", //name[32]
		MIN_VAMP_HUE_UNITS, //minimum
		MAX_VAMP_HUE_UNITS, //maximum
		1, //step
		HueDefault, //default_value
		0, //flags
		{ 0, 0 }, //reserved[2]
	},
#endif
#if 0
	{
		V4L2_CID_AUTOGAIN,           //id
		V4L2_CTRL_TYPE_INTEGER,        //type
		"Hdcp enable",                 //name[32]
		0,                             //minimum
		1,                             //maximum
		1,                             //step
		0,                             //default_value
		0,                             //flags
		{ 0, 0 },                      //reserved[2]
	},
	{
		V4L2_CID_GAIN,           //id
		V4L2_CTRL_TYPE_INTEGER,        //type
		"Sample rate",                        //name[32]
		48000,                             //minimum
		48000,                             //maximum
		1,                             //step
		48000,                             //default_value
		0,                             //flags
		{ 0, 0 },                      //reserved[2]
	}
#endif
};

#define ARRAY_SIZE_OF_CTRL (sizeof(g_hws_ctrls) / sizeof(g_hws_ctrls[0]))

static struct v4l2_queryctrl *find_ctrlByIndex(unsigned int index)
{
	//scan supported queryctrl table
	if (index >= ARRAY_SIZE_OF_CTRL) {
		return NULL;
	} else {
		return &g_hws_ctrls[index];
	}
}

static struct v4l2_queryctrl *find_ctrl(unsigned int id)
{
	int i;
	//scan supported queryctrl table
	for (i = 0; i < ARRAY_SIZE_OF_CTRL; i++)
		if (g_hws_ctrls[i].id == id)
			return &g_hws_ctrls[i];

	return 0;
}

#if 0
static unsigned int find_Next_Ctl_ID(unsigned int id)
{
	int i;
	int nextID =-1;
	int curr_index =-1;
	//scan supported queryctrl table
	for( i=0; i<ARRAY_SIZE_OF_CTRL; i++ )
	{
		if(g_hws_ctrls[i].id==id)
		{
			curr_index = i;
			break;
		}
	}
	if(curr_index != -1)
	{
		if((curr_index +1)<ARRAY_SIZE_OF_CTRL)
		{
			nextID = g_hws_ctrls[curr_index +1].id;
		}
	}
	return nextID;
}
#endif
int hws_vidioc_g_ctrl(struct file *file, void *fh, struct v4l2_control *a) //
{
	struct hws_video *videodev = video_drvdata(file);
	struct v4l2_control *ctrl = a;
	//struct v4l2_queryctrl *found_ctrl = find_ctrl(ctrl->id);
	int ret = -EINVAL;
	//int bchs_select=0;
	if (ctrl == NULL) {
		printk("%s(ch-%d)ctrl=NULL\n", __func__, videodev->index);
		return ret;
	}
	//printk( "%s(ch-%d)\n", __func__,videodev->index);
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS: //0x00980900

		//bchs_select = V4L2_BCHS_TYPE_BRIGHTNESS;
		//adv7619_get_bchs(v4l2m_context->adv7619_handle,&BCHSinfo,bchs_select);
		ctrl->value = videodev->m_Curr_Brightness;
		//printk("%s...brightness(%d)\n",__func__,ctrl->value);
		ret = 0;
		break;

	case V4L2_CID_CONTRAST:

		//bchs_select = V4L2_BCHS_TYPE_CONTRAST;
		//printk("%s...contrast(%d)\n",__func__,bchs_select);
		ctrl->value = videodev->m_Curr_Contrast;
		ret = 0;
		break;

	case V4L2_CID_SATURATION:

		//bchs_select = V4L2_BCHS_TYPE_SATURATION;
		//printk("%s...saturation(%d)\n",__func__,bchs_select);
		ctrl->value = videodev->m_Curr_Saturation;
		ret = 0;
		break;

	case V4L2_CID_HUE:

		//bchs_select = V4L2_BCHS_TYPE_HUE;
		//printk("%s...hue(%d)\n",__func__,bchs_select);
		ctrl->value = videodev->m_Curr_Hue;
		ret = 0;
		break; //
	default:
		ctrl->value = 0;
		printk("control id %d not handled\n", ctrl->id);
		break;
	}
	//printk("%s...ctrl->value(%d)=%x\n",__func__,bchs_select,ctrl->value);
	return ret;
}

int hws_vidioc_s_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct hws_video *videodev = video_drvdata(file);
	struct v4l2_control *ctrl = a;
	struct v4l2_queryctrl *found_ctrl;
	int ret = -EINVAL;
	if (ctrl == NULL) {
		printk("%s(ch-%d)ctrl=NULL\n", __func__, videodev->index);
		return ret;
	}
	//printk( "%s(ch-%d ctrl->id =%X )\n", __func__,videodev->index,ctrl->id);
	found_ctrl = find_ctrl(ctrl->id);
	if (found_ctrl) {
		switch (found_ctrl->type) {
		case V4L2_CTRL_TYPE_INTEGER:
			if (ctrl->value >= found_ctrl->minimum ||
			    ctrl->value <= found_ctrl->maximum) {
				//printk( "%s(ch-%d ctrl->value =%X )\n", __func__,videodev->index,ctrl->value);
				switch (ctrl->id) {
				case V4L2_CID_BRIGHTNESS:
					videodev->m_Curr_Brightness =
						ctrl->value;
					break;
				case V4L2_CID_CONTRAST:
					videodev->m_Curr_Contrast = ctrl->value;
					break;
				case V4L2_CID_HUE:
					videodev->m_Curr_Hue = ctrl->value;
					break;
				case V4L2_CID_SATURATION:
					videodev->m_Curr_Saturation =
						ctrl->value;
					break;

				default:
					break;
				}
				//printk( "%s(Name:%s value =%X )\n", __func__,found_ctrl->name,ctrl->value);
				ret = 0;
			} else {
				//error
				ret = -ERANGE;
				printk("control %s out of range\n",
				       found_ctrl->name);
			}
			break;
		default: {
			//error
			printk("control type %d not handled\n",
			       found_ctrl->type);
		}
		}
	}
	//printk( "%s(ret=%d)\n", __func__,ret);
	return ret;
}
void mem_model_memset(void *s, int c, unsigned int n)
{
	memset(s, c, n);
}
static int hws_vidioc_queryctrl(struct file *file, void *fh,
				struct v4l2_queryctrl *a)
{
	struct hws_video *videodev = video_drvdata(file);
	struct v4l2_queryctrl *found_ctrl;
	unsigned int id;
	unsigned int mask_id;
	int ret = -EINVAL;
	//printk( "%s(ch-%d)\n", __func__,videodev->index);
	//printk( "%s(ctrl-id=0x%X[0x%X] )\n", __func__,a->id,(a->id)&(~V4L2_CTRL_FLAG_NEXT_CTRL));
	//-----------------------------------------------
	id = a->id & (~V4L2_CTRL_FLAG_NEXT_CTRL);
	mask_id = a->id & V4L2_CTRL_FLAG_NEXT_CTRL;
	if (mask_id == V4L2_CTRL_FLAG_NEXT_CTRL) {
		if (id == 0) {
			videodev->queryIndex = 0;
			found_ctrl = find_ctrlByIndex(videodev->queryIndex);
			*a = *found_ctrl;
			//a->id = a->id|V4L2_CTRL_FLAG_NEXT_CTRL;
			//printk("queryctrl[1] Get [%s][0x%X]\n",found_ctrl->name,a->id);
			ret = 0;
		} else {
			videodev->queryIndex++;
			found_ctrl = find_ctrlByIndex(videodev->queryIndex);
			if (found_ctrl != NULL) {
				*a = *found_ctrl;
				//a->id = a->id|V4L2_CTRL_FLAG_NEXT_CTRL;
				//printk("queryctrl[2] Get [%s][0x%X]\n",found_ctrl->name,a->id);
				ret = 0;
			} else {
				*a = g_no_ctrl;
				ret = -EINVAL;
			}
		}

	} else {
		found_ctrl = find_ctrl(id);
		if (NULL != found_ctrl) {
			*a = *found_ctrl;
			//printk("queryctrl[3] Get [%s][0x%X]\n",found_ctrl->name,a->id);
			ret = 0;
		} else {
			*a = g_no_ctrl;
			ret = -EINVAL;
		}
	}
	return ret;
}
#if 0
static int hws_vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	//struct hws_video *videodev = video_drvdata(file);
	//printk( "%s(ch-%d)\n", __func__,videodev->index);
#if 0
	StartVideoCapture(videodev->dev,videodev->index);
#endif 
	return(vb2_ioctl_streamon(file,priv,i));
	
}
static int hws_vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
	//struct hws_video *videodev = video_drvdata(file);
	//printk( "%s(ch-%d)\n", __func__,videodev->index);
#if 0
	StopVideoCapture(videodev->dev,videodev->index);
#endif 
	return(vb2_ioctl_streamoff(file,priv,i));

}
#endif
static int hws_vidioc_enum_frameintervals(struct file *file, void *fh,
					  struct v4l2_frmivalenum *fival)
{
	//struct hws_video *videodev = video_drvdata(file);
	int Index;
	int FrameRate;
	v4l2_model_timing_t *pModeTiming;
	Index = fival->index;
	//printk( "%s(CH-%d) FrameIndex =%d \n", __func__,videodev->index,Index);
	if (Index < 0 || Index >= NUM_FRAMERATE_CONTROLS) {
		return -EINVAL;
	}

	FrameRate = v4l2_model_get_support_framerate(Index);
	if (FrameRate == -1)
		return -EINVAL;
	pModeTiming = Get_input_framesizeIndex(fival->width, fival->height);
	if (pModeTiming == NULL)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1000;
	fival->discrete.denominator = FrameRate * 1000;
	//printk( "%s FrameIndex=%d W=%d H=%d  FrameRate=%d \n", __func__,Index,fival->width,fival->height,FrameRate);
	return 0;
}

int hws_vidioc_s_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct hws_video *videodev = video_drvdata(file);
	int io_frame_rate;
	int in_frame_rate;
	//int io_widht;
	//int io_hight;
	//int io_index;
	v4l2_model_timing_t *p_SupportmodeTiming;
	io_frame_rate = a->parm.capture.timeperframe.denominator /
			a->parm.capture.timeperframe.numerator;
	//io_index = a->parm.capture.timeperframe.index;
	//printk( "%s(CH-%d) io_index =%d \n", __func__,videodev->index,io_index);
	p_SupportmodeTiming = v4l2_model_get_support_videoformat(
		videodev->current_out_size_index);
	if (p_SupportmodeTiming == NULL)
		return -EINVAL;
	in_frame_rate = p_SupportmodeTiming->refresh_rate;
	//printk( "%s(ch-%d)io_frame_rate =%d  in_frame_rate =%d \n", __func__,videodev->index,io_frame_rate,in_frame_rate);
	return 0;
}

static int hws_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hws_video *vid =
		container_of(ctrl->handler, struct hws_video, ctrl_handler);
	struct hws_pcie_dev *pdx = vid->dev; /* if you keep this ptr */

	switch (ctrl->id) {
	case V4L2_CID_DV_RX_POWER_PRESENT:
		/* bit 3 (+5 V) over the two pipes for this HDMI port           */
		ctrl->val = !!(hws_read_port_hpd(pdx, vid->port) & HWS_5V_BIT);
		return 0;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0))
	case V4L2_CID_DV_RX_HOTPLUG_PRESENT:
		/* bit 0 (HPD) */
		ctrl->val = !!(hws_read_port_hpd(pdx, vid->port) & HWS_HPD_BIT);
		return 0;
#endif

	case V4L2_CID_DV_RX_IT_CONTENT_TYPE:
		ctrl->val = hdmi_content_type(vid); /* unchanged */
		return 0;

	default:
		return -EINVAL;
	}
}

#if 0
static int hws_vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
   struct hws_video *videodev = video_drvdata(file);
   printk( "%s(ch-%d)\n", __func__,videodev->index);
	//vb2_ioctl_dqbuf
    return vb2_ioctl_dqbuf(file,priv,p);
}
#endif

//----------------------------
static const struct v4l2_file_operations hws_fops = {
	.owner = THIS_MODULE,
	.open = hws_open, //v4l2_fh_open,
	.release = hws_release, //vb2_fop_release,
	.read = hws_read, //vb2_fop_read,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

static const struct v4l2_ioctl_ops hws_ioctl_fops = {
	.vidioc_querycap = hws_vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = hws_vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = hws_vidioc_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = vidioc_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = hws_vidioc_try_fmt_vid_cap,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_g_std = hws_vidioc_g_std,
	.vidioc_s_std = hws_vidioc_s_std,
	.vidioc_enum_framesizes = hws_vidioc_enum_framesizes,
	.vidioc_enum_frameintervals = hws_vidioc_enum_frameintervals,
	.vidioc_g_ctrl = hws_vidioc_g_ctrl,
	.vidioc_s_ctrl = hws_vidioc_s_ctrl,
	.vidioc_queryctrl = hws_vidioc_queryctrl,
	.vidioc_enum_input = hws_vidioc_enum_input,
	.vidioc_g_input = hws_vidioc_g_input,
	.vidioc_s_input = hws_vidioc_s_input,
	.vidioc_log_status = vidioc_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_g_parm = hws_vidioc_g_parm,
	.vidioc_s_parm = hws_vidioc_s_parm,
};

static const struct v4l2_ctrl_ops hws_ctrl_ops = {
	.g_volatile_ctrl = hws_g_volatile_ctrl,
};

static int hws_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			   unsigned int *num_planes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct hws_video *videodev = q->drv_priv;
	struct hws_pcie_dev *pdx = videodev->dev;
	unsigned long flags;
	unsigned size;
	spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
	size = 2 * videodev->current_out_width *
	       videodev->curren_out_height; // 16bit
	//printk( "%s(%d)->%d[%d?=%d]\n", __func__,videodev->index,videodev->fileindex,sizes[0],size);
	if (videodev->fileindex > 1) {
		spin_unlock_irqrestore(&pdx->videoslock[videodev->index],
				       flags);
		return -EINVAL;
	}
	//printk( "q->num_buffers = %d *num_buffers =%d \n", q->num_buffers,*num_buffers);
	//if (tot_bufs < 2)
	//	tot_bufs = 2;
	//tot_bufs = hws_buffer_count(size, tot_bufs);
	//*num_buffers = tot_bufs - q->num_buffers;
	if (*num_planes) {
		if (sizes[0] < size) {
			spin_unlock_irqrestore(
				&pdx->videoslock[videodev->index], flags);
			return -EINVAL;
		} else {
			spin_unlock_irqrestore(
				&pdx->videoslock[videodev->index], flags);
			return 0;
		}
	}
	//printk( "%s()  num_buffers:%x tot_bufs:%x\n", __func__,*num_buffers,tot_bufs);
	//printk( "%s()  sizes[0]= %d size= %d\n", __func__,sizes[0],size);
	*num_planes = 1;
	sizes[0] = size;
	spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);
	return 0;
}

static int hws_buffer_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct hwsvideo_buffer *buf =
		container_of(vbuf, struct hwsvideo_buffer, vb);
	struct hws_video *videodev = vb->vb2_queue->drv_priv;
	struct hws_pcie_dev *pdx = videodev->dev;
	u32 size;
	unsigned long flags;
	//printk( "%s(W = %d H=%d)\n", __func__,videodev->current_out_width,videodev->curren_out_height);
	spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
	size = 2 * videodev->current_out_width *
	       videodev->curren_out_height; // 16bit
	if (vb2_plane_size(vb, 0) < size) {
		spin_unlock_irqrestore(&pdx->videoslock[videodev->index],
				       flags);
		return -EINVAL;
	}
	vb2_set_plane_payload(vb, 0, size);
	buf->mem = vb2_plane_vaddr(vb, 0);
	spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);
	return 0;
}

static void hws_buffer_finish(struct vb2_buffer *vb)
{
	//struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	//struct hwsvideo_buffer *buf =
	//	container_of(vbuf, struct hwsvideo_buffer, vb);
	//struct hws_video *videodev = vb->vb2_queue->drv_priv;
	//printk( "%s()\n", __func__);
	return;
}

static void hws_buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct hws_video *videodev = vb->vb2_queue->drv_priv;
	struct hwsvideo_buffer *buf =
		container_of(vbuf, struct hwsvideo_buffer, vb);
	unsigned long flags;
	struct hws_pcie_dev *pdx = videodev->dev;

	//printk( "%s(%d)\n", __func__,videodev->index);
	spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
	list_add_tail(&buf->queue, &videodev->queue);
	spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);
}

static int hws_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct hws_video *videodev = q->drv_priv;
	unsigned long flags;
	struct hws_pcie_dev *pdx = videodev->dev;
//printk( "%s(%d)->%d\n", __func__,videodev->index,videodev->fileindex);
#if 0
	if(videodev->fileindex >1)
	{
		return -EINVAL;
	}
#endif
	videodev->seqnr = 0;
	mdelay(100);
	//---------------
	//if(videodev->fileindex==1)
	//{
	//printk( "StartVideoCapture %s(%d)->%d\n", __func__,videodev->index,videodev->fileindex);
	StartVideoCapture(videodev->dev, videodev->index);
	videodev->startstreamIndex++;
	//------------------------ reset queue
	//printk( "%s(%d)->%d  reset queue \n", __func__,videodev->index,videodev->fileindex);

	//}
	spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
	while (!list_empty(&videodev->queue)) {
		struct hwsvideo_buffer *buf = list_entry(
			videodev->queue.next, struct hwsvideo_buffer, queue);
		list_del(&buf->queue);

		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);
	//-----------------------
	return 0;
}

static void hws_stop_streaming(struct vb2_queue *q)
{
	struct hws_video *videodev = q->drv_priv;
	unsigned long flags;
	struct hws_pcie_dev *pdx = videodev->dev;
//printk( "%s(%d)->%d\n", __func__,videodev->index,videodev->fileindex);

//if(videodev->seqnr){
//vb2_wait_for_all_buffers(q);
//	mdelay(100);
//printk( "%s() vb2_wait_for_all_buffers\n", __func__);
//}
#if 1
	//-----------------------------------
	videodev->startstreamIndex--;
	if (videodev->startstreamIndex < 0)
		videodev->startstreamIndex = 0;
	if (videodev->startstreamIndex == 0) {
		//printk( "StopVideoCapture %s(%d)->%d [%d]\n", __func__,videodev->index,videodev->fileindex,videodev->startstreamIndex);
		StopVideoCapture(videodev->dev, videodev->index);
	}
	//------------------
	spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
	while (!list_empty(&videodev->queue)) {
		struct hwsvideo_buffer *buf = list_entry(
			videodev->queue.next, struct hwsvideo_buffer, queue);
		list_del(&buf->queue);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);
//-----------------------------------------------------------------
#endif
}

static const struct vb2_ops hwspcie_video_qops = {
	.queue_setup = hws_queue_setup,
	.buf_prepare = hws_buffer_prepare,
	.buf_finish = hws_buffer_finish,
	.buf_queue = hws_buffer_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.start_streaming = hws_start_streaming,
	.stop_streaming = hws_stop_streaming,
};
//-----------------------------------------
const unsigned char g_YUVColors[MAX_COLOR][3] = {
	{ 128, 16, 128 }, // BLACK
	{ 128, 235, 128 }, // WHITE
	{ 16, 211, 146 }, // YELLOW
	{ 166, 170, 16 }, // CYAN
	{ 54, 145, 34 }, // GREEN
	{ 202, 106, 222 }, // MAGENTA
	{ 90, 81, 240 }, // RED
	{ 240, 41, 109 }, // BLUE
	{ 128, 125, 128 }, // GREY
};

static void SetNoVideoMem(uint8_t *pDest, int w, int h)
{
	int x, y;
	uint8_t *pST;
	uint8_t *pNS;
	pST = (uint8_t *)pDest;
	//printk("SetNoVideoMem[%d-%d]\n",w,h);

	for (x = 0; x < w / 2; x++) {
		pST[0] = 41;
		pST[1] = 240;
		pST[2] = 41;
		pST[3] = 109;
		pST += 4;
	}

	pNS = pDest + w * 2;
	for (y = 1; y < h; y++) {
		memcpy(pNS, pDest, w * 2);
		pNS = pNS + w * 2;
	}
}

static void hws_get_video_param(struct hws_pcie_dev *dev, int index)
{
	//printk( "%s(): %x \n", __func__, index);
	int width, height;
	width = dev->m_pVCAPStatus[index][0].dwWidth;
	height = dev->m_pVCAPStatus[index][0].dwHeight;
	dev->video[index].current_out_pixfmt = 0;
	dev->video[index].current_out_size_index = 0;
	dev->video[index].current_out_width = width;
	dev->video[index].curren_out_height = height;
	dev->video[index].current_out_framerate = 60;
	dev->video[index].Interlaced = 0;
	//printk( "%s(%dx%d):  \n", __func__, width,height);
}

static void hws_adapters_init(struct hws_pcie_dev *dev)
{
	int i;
	for (i = 0; i < MAX_VID_CHANNELS; i++) {
		hws_get_video_param(dev, i);
	}
}
void hws_remove_deviceregister(struct hws_pcie_dev *dev)
{
	int i;
	struct video_device *vdev;
	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		// FIXME
		// v4l2_ctrl_handler_free(&hws->video[i].ctrl_handler);
		vdev = &(dev->video[i].vdev);
		if (vdev) {
			v4l2_device_unregister(&dev->video[i].v4l2_dev);
			vdev = NULL;
		}
	}
}
int hws_video_register(struct hws_pcie_dev *dev)
{
	struct video_device *vdev;
	struct vb2_queue *q;
	int i;
	int err = -1;
	//printk("hws_video_register Start\n");
	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		//printk("v4l2_device_register[%d]\n",i);
		err = v4l2_device_register(&dev->pdev->dev,
					   &dev->video[i].v4l2_dev);
		if (err < 0) {
			printk(KERN_ERR " v4l2_device_register 0 error! \n");
			hws_remove_deviceregister(dev);
			return -1;
		}
	}
	//printk("v4l2_device_register end\n");
	//----------------------------------------------------
	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		//printk("v4l2_device_register INT[%d]\n",i);
		vdev = &(dev->video[i].vdev);
		q = &(dev->video[i].vq);
		if (NULL == vdev) {
			printk(KERN_ERR " video_device_alloc failed !!!!! \n");
			goto fail;
		}
		dev->video[i].index = i;
		dev->video[i].dev = dev;
		dev->video[i].fileindex = 0;
		dev->video[i].startstreamIndex = 0;
		dev->video[i].std = V4L2_STD_NTSC_M;
		dev->video[i].pixfmt = V4L2_PIX_FMT_YUYV;
		//-------------------
		dev->video[i].m_Curr_Brightness = BrightnessDefault;
		dev->video[i].m_Curr_Contrast = ContrastDefault;
		dev->video[i].m_Curr_Saturation = SaturationDefault;
		dev->video[i].m_Curr_Hue = HueDefault;
		//-------------------
		vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
		vdev->v4l2_dev = &(dev->video[i].v4l2_dev);
		vdev->lock = &(dev->video[i].video_lock);
		vdev->ctrl_handler = &(dev->video[i].ctrl_handler);
		vdev->fops = &hws_fops;
		strcpy(vdev->name, KBUILD_MODNAME);
		vdev->release = video_device_release_empty;
		vdev->vfl_dir = VFL_DIR_RX;
		vdev->ioctl_ops = &hws_ioctl_fops;
		mutex_init(&(dev->video[i].video_lock));
		mutex_init(&(dev->video[i].queue_lock));
		spin_lock_init(&dev->video[i].slock);
		//printk("v4l2_device_register INT3[%d]\n",i);
		INIT_LIST_HEAD(&dev->video[i].queue);
		//printk("v4l2_device_register INT2[%d]\n",i);
		video_set_drvdata(vdev, &(dev->video[i]));

		q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		q->io_modes = VB2_READ | VB2_MMAP | VB2_USERPTR;
		//q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_READ;
		q->gfp_flags = GFP_DMA32;
		//q->min_buffers_needed = 2;
		q->drv_priv = &(dev->video[i]);
		q->buf_struct_size = sizeof(struct hwsvideo_buffer);
		q->ops = &hwspcie_video_qops;

		//q->mem_ops = &vb2_dma_contig_memops;
		//q->mem_ops = &vb2_dma_sg_memops;
		q->mem_ops = &vb2_vmalloc_memops;

		q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

		q->lock = &(dev->video[i].queue_lock);
		q->dev = &(dev->pdev->dev);
		vdev->queue = q;
		err = vb2_queue_init(q);
		if (err != 0) {
			printk(KERN_ERR " vb2_queue_init failed !!!!! \n");
			goto fail;
		}

		INIT_WORK(&dev->video[i].videowork, video_data_process);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 7, 0))
		err = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
#else
		err = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
#endif
		if (err != 0) {
			printk(KERN_ERR
			       " v4l2_device_register failed !!!!! \n");
			goto fail;
		} else {
			//printk(" video_register_device OK !!!!! \n");
		}
	}
	//printk("hws_video_register End\n");
	return 0;
fail:
	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		vdev = &dev->video[i].vdev;
		video_unregister_device(vdev);
		v4l2_device_unregister(&dev->video[i].v4l2_dev);
	}
	return err;
}

/* HDMI 0x39[3:0] - CS_DATA[27:24] 0 for reserved values*/
static const int cs_data_fs[] = {
	44100, 0,      48000, 32000, 0,	     0, 0,	0,
	88200, 768000, 96000, 0,     176000, 0, 192000, 0,
};
int hws_pcie_audio_open(struct snd_pcm_substream *substream)
{
	struct hws_audio *drv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	drv->sample_rate_out = 48000;
	drv->channels = 2;
	//printk(KERN_INFO "%s() index:%x\n",__func__,drv->index);
	runtime->hw = audio_pcm_hardware;
	drv->substream = substream;
	//snd_pcm_hw_constraint_minmax(runtime,SNDRV_PCM_HW_PARAM_RATE,setrate,setrate);
	return 0;
}

int hws_pcie_audio_close(struct snd_pcm_substream *substream)
{
	//	struct hws_audio *chip = snd_pcm_substream_chip(substream);
	//printk(KERN_INFO "%s() \n",__func__);
	return 0;
}
int hws_pcie_audio_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *hw_params)
{
	//printk(KERN_INFO "%s() \n",__func__);
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

int hws_pcie_audio_hw_free(struct snd_pcm_substream *substream)
{
	//printk(KERN_INFO "%s() \n",__func__);
	return snd_pcm_lib_free_pages(substream);
}

int hws_pcie_audio_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct hws_audio *drv = snd_pcm_substream_chip(substream);
	//struct hws_pcie_dev *dev= drv->dev;
	//int i;
	unsigned long flags;
	//printk(KERN_INFO "%s() index:%x\n",__func__,drv->index);

	spin_lock_irqsave(&drv->ring_lock, flags);
	drv->ring_size_byframes = runtime->buffer_size;
	drv->ring_wpos_byframes = 0;
	drv->period_size_byframes = runtime->period_size;
	drv->period_used_byframes = 0;
	drv->ring_offsize = 0;
	drv->ring_over_size = 0;
	spin_unlock_irqrestore(&drv->ring_lock, flags);

	return 0;
}
int hws_pcie_audio_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct hws_audio *chip = snd_pcm_substream_chip(substream);
	struct hws_pcie_dev *dev = chip->dev;
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		//HWS_PCIE_READ(HWS_DMA_BASE(chip->index), HWS_DMA_STATUS);
		//start dma
		//HWS_PCIE_WRITE(HWS_INT_BASE, HWS_DMA_MASK(chip->index), 0x00000001);
		//HWS_PCIE_WRITE(HWS_DMA_BASE(chip->index), HWS_DMA_START, 0x00000001);
		//printk(KERN_INFO "SNDRV_PCM_TRIGGER_START index:%x\n",chip->index);
		chip->ring_wpos_byframes = 0;
		chip->period_used_byframes = 0;
		StartAudioCapture(dev, chip->index);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		//stop dma
		//HWS_PCIE_WRITE(HWS_INT_BASE, HWS_DMA_MASK(chip->index), 0x000000000);
		//HWS_PCIE_WRITE(HWS_DMA_BASE(chip->index), HWS_DMA_START, 0x00000000);
		//printk(KERN_INFO "SNDRV_PCM_TRIGGER_STOP index:%x\n",chip->index);
		StopAudioCapture(dev, chip->index);
		break;
	default:
		return -EINVAL;
		break;
	}
	return 0;
}
//-------------------------------------------------

//-------------------------------------------------
static snd_pcm_uframes_t
hws_pcie_audio_pointer(struct snd_pcm_substream *substream)
{
	struct hws_audio *drv = snd_pcm_substream_chip(substream);
	//struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t pos;
	int dwAudioCh;
	unsigned long flags;
	dwAudioCh = drv->index;
	//printk(KERN_INFO "%s() index:%x\n",__func__,dwAudioCh);
	spin_lock_irqsave(&drv->ring_lock, flags); //spin_lock
	pos = drv->ring_wpos_byframes;
	spin_unlock_irqrestore(&drv->ring_lock, flags); //spin_unlock
	return pos;
}

struct snd_pcm_ops hws_pcie_pcm_ops = { .open = hws_pcie_audio_open,
					.close = hws_pcie_audio_close,
					.ioctl = snd_pcm_lib_ioctl,
					.hw_params = hws_pcie_audio_hw_params,
					.hw_free = hws_pcie_audio_hw_free,
					.prepare = hws_pcie_audio_prepare,
					.trigger = hws_pcie_audio_trigger,
					.pointer = hws_pcie_audio_pointer };

int hws_audio_register(struct hws_pcie_dev *dev)
{
	struct snd_pcm *pcm;
	struct snd_card *card;
	int ret;
	int i;
	int ai_index;
	char audioname[100];
	//printk("hws_audio_register Start\n");
	ai_index = dev->m_Device_PortID * dev->m_nCurreMaxVideoChl + 1;
	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		sprintf(audioname, "%s %d", HWS_AUDOI_NAME, i + ai_index);
		//printk("%s\n",audioname);
		ret = snd_card_new(&dev->pdev->dev, -1, audioname, THIS_MODULE,
				   sizeof(struct hws_audio), &card);
		// ret = snd_card_new(&dev->pdev->dev, audio_index[i], audio_id[i], THIS_MODULE,	sizeof(struct hws_audio), &card);
		if (ret < 0) {
			printk(KERN_ERR
			       "%s() ERROR: snd_card_new failed <%d>\n",
			       __func__, ret);
			goto fail0;
		}
		strcpy(card->driver, KBUILD_MODNAME);
		sprintf(card->shortname, "%s", audioname);
		sprintf(card->longname, "%s", card->shortname);

		ret = snd_pcm_new(card, audioname, 0, 0, 1, &pcm);
		if (ret < 0) {
			printk(KERN_ERR "%s() ERROR: snd_pcm_new failed <%d>\n",
			       __func__, ret);
			goto fail1;
		}
		dev->audio[i].index = i;
		dev->audio[i].dev = dev;
		pcm->private_data = &dev->audio[i];
		strcpy(pcm->name, audioname);
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
				&hws_pcie_pcm_ops);
		//snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,snd_dma_pci_data(dev->pdev), HWS_AUDIO_CELL_SIZE*4, HWS_AUDIO_CELL_SIZE*4);
		snd_pcm_lib_preallocate_pages_for_all(
			pcm, SNDRV_DMA_TYPE_CONTINUOUS,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
			card->dev,
#else
			snd_dma_continuous_data(GFP_KERNEL),
#endif
			audio_pcm_hardware.buffer_bytes_max,
			audio_pcm_hardware.buffer_bytes_max);
		//----------------------------
		dev->audio[i].sample_rate_out = 48000;
		dev->audio[i].channels = 2;
		dev->audio[i].resampled_buf_size =
			dev->audio[i].sample_rate_out * 2 /* sample bytes */ *
			dev->audio[i].channels /* channels */;
		//dev->audio[i].resampled_buf = vmalloc(dev->audio[i].resampled_buf_size);
		//if(dev->audio[i].resampled_buf == NULL)
		//	goto fail1;
		//-----------------
		spin_lock_init(&dev->audio[i].ring_lock);
		INIT_WORK(&dev->audio[i].audiowork, audio_data_process);
		ret = snd_card_register(card);
		if (ret < 0) {
			printk(KERN_ERR
			       "%s() ERROR: snd_card_register failed\n",
			       __func__);
			goto fail1;
		}
		dev->audio[i].card = card;
	}
	//printk("hws_audio_register End\n");
	return 0;
fail1:
	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		if (dev->audio[i].card) {
			snd_card_free(dev->audio[i].card);
			dev->audio[i].card = NULL;
		}
		if (dev->audio[i].resampled_buf) {
			vfree(dev->audio[i].resampled_buf);
			dev->audio[i].resampled_buf = NULL;
		}
	}
fail0:
	return -1;
}
//-------------------
//static unsigned long video_data[MAX_VID_CHANNELS];
//static struct tasklet_struct dpc_video_tasklet[MAX_VID_CHANNELS];
//static unsigned long audio_data[MAX_VID_CHANNELS];
//static struct tasklet_struct dpc_audio_tasklet[MAX_VID_CHANNELS];

static void WRITE_REGISTER_ULONG(struct hws_pcie_dev *pdx, u32 RegisterOffset,
				 u32 Value)
{
	//map_bar0_addr[RegisterOffset/4] = Value;
	char *bar0;
	bar0 = (char *)pdx->map_bar0_addr;
	iowrite32(Value, bar0 + RegisterOffset);
	//map_bar0_addr[RegisterOffset/4] = Value;
}

static u32 READ_REGISTER_ULONG(struct hws_pcie_dev *pdx, u32 RegisterOffset)
{
	char *bar0;
	bar0 = (char *)pdx->map_bar0_addr;
	//return(map_bar0_addr[RegisterOffset/4]);
	return (ioread32(bar0 + RegisterOffset));
}
//----------------------------------------------
static int Check_Busy(struct hws_pcie_dev *pdx)
{
	u32 statusreg;
	u32 TimeOut = 0;
	//DbgPrint(("Check Busy in !!!\n"));
	//WRITE_REGISTER_ULONG((u32)(0x4000), 0x10);
	while (1) {
		statusreg = READ_REGISTER_ULONG(pdx, HWS_REG_SYS_STATUS);
		printk("[MV] Check_Busy!!! statusreg =%X\n", statusreg);
		if (statusreg == 0xFFFFFFFF) {
			break;
		}
		if ((statusreg & HWS_SYS_DMA_BUSY_BIT) == 0) {
			break;
		}
		TimeOut++;
		msleep(10);
	}
	//WRITE_REGISTER_ULONG((u32)(0x4000), 0x10);

	//DbgPrint(("Check Busy out !!!\n"));

	return 0;
}

static void StopDsp(struct hws_pcie_dev *pdx)
{
	//int j, i;
	u32 statusreg;
	statusreg = READ_REGISTER_ULONG(pdx, HWS_REG_DEC_MODE);
	printk("[MV] Busy!!! statusreg =%X\n", statusreg);
	if (statusreg == 0xFFFFFFFF) {
		return;
	}
	WRITE_REGISTER_ULONG(pdx, HWS_REG_DEC_MODE, 0x10);
	Check_Busy(pdx);
	WRITE_REGISTER_ULONG(pdx, HWS_REG_VCAP_ENABLE, 0x00);
}


static int SetVideoFormteSize(struct hws_pcie_dev *pdx, int ch, int w, int h)
{
	int hf_size;
	int down_size;
	//int frame_size;
	//int buf_cnt;

	if (pdx->m_Device_SupportYV12 == 0) {
		hf_size = (w * h) / (16 * 128);
		hf_size = hf_size * 16 * 128;
		down_size = (w * h * 2) - hf_size;
	} else if (pdx->m_Device_SupportYV12 == 1) {
		hf_size = (w * h * 12) / (16 * 16 * 128);
		hf_size = hf_size * 16 * 128;
		down_size = ((w * h * 12) / 8) - hf_size;
	} else {
		hf_size = (w * h * 5) / (8 * 16 * 128);
		hf_size = hf_size * 16 * 128;
		down_size = ((w * h * 5) / 4) - hf_size;
	}
	pdx->m_format[ch].dwWidth = w; // Image Width
	pdx->m_format[ch].dwWidth = w; // Image Width
	pdx->m_format[ch].HLAF_SIZE = hf_size;
	pdx->m_format[ch].DWON_SIZE = down_size;
	//DbgPrint("[MV-X1]-CH0 SetVideoFormteSize = %d %d \n",m_format[ch].HLAF_SIZE,m_format[ch].DWON_SIZE);

	return 1;
}

static void DmaMemFreePool(struct hws_pcie_dev *pdx)
{
	//Trace t("DmaMemFreePool()");
	int i = 0, k;

	unsigned long phyvirt_addr;
	if (pdx->m_bBufferAllocate == TRUE) {
		//---------------
		for (i = 0; i < pdx->m_nMaxChl; i++) {
			if (pdx->m_pbyVideoBuffer[i]) {
//printk("DmaMemFreePool ::m_pbyVideoBuffer = %p\n",  pdx->m_pbyVideoBuffer[i]);
#if 0
					for (phyvirt_addr=(unsigned long)pdx->m_pbyVideoBuffer_area[i]; phyvirt_addr < ((unsigned long)pdx->m_pbyVideoBuffer_area[i] + pdx->m_MaxHWVideoBufferSize);phyvirt_addr+=PAGE_SIZE) 
					{
						// clear all pages
						ClearPageReserved(virt_to_page(phyvirt_addr));
					}
					kfree(pdx->m_pbyVideoBuffer[i]);
#else
				dma_free_coherent(&pdx->pdev->dev,
						  pdx->m_MaxHWVideoBufferSize,
						  pdx->m_pbyVideoBuffer[i],
						  pdx->m_pbyVideo_phys[i]);
#endif
				pdx->m_pbyVideoBuffer[i] = NULL;
			}
		}
		for (i = 0; i < pdx->m_nCurreMaxVideoChl; i++) {
			if (pdx->m_VideoInfo[i].m_pVideoScalerBuf != NULL) {
				vfree(pdx->m_VideoInfo[i].m_pVideoScalerBuf);
				pdx->m_VideoInfo[i].m_pVideoScalerBuf = NULL;
			}

			if (pdx->m_VideoInfo[i].m_pVideoYUV2Buf != NULL) {
				vfree(pdx->m_VideoInfo[i].m_pVideoYUV2Buf);
				pdx->m_VideoInfo[i].m_pVideoYUV2Buf = NULL;
			}

			if (pdx->m_VideoInfo[i].m_pRotateVideoBuf != NULL) {
				vfree(pdx->m_VideoInfo[i].m_pRotateVideoBuf);
				pdx->m_VideoInfo[i].m_pRotateVideoBuf = NULL;
			}
			for (k = 0; k < MAX_VIDEO_QUEUE; k++) {
				if (pdx->m_VideoInfo[i].m_pVideoBufData[k]) {
					for (phyvirt_addr =
						     (unsigned long)pdx
							     ->m_VideoInfo[i]
							     .m_pVideoData_area
								     [k];
					     phyvirt_addr <
					     ((unsigned long)pdx->m_VideoInfo[i]
						      .m_pVideoData_area[k] +
					      pdx->m_MaxHWVideoBufferSize);
					     phyvirt_addr += PAGE_SIZE) {
						// clear all pages
						ClearPageReserved(virt_to_page(
							phyvirt_addr));
					}
					kfree(pdx->m_VideoInfo[i]
						      .m_pVideoBufData[k]);
					pdx->m_VideoInfo[i].m_pVideoBufData[k] =
						NULL;
				}
			}
			//----audio release
			for (k = 0; k < MAX_AUDIO_QUEUE; k++) {
				if (pdx->m_AudioInfo[i].m_pAudioBufData[k]) {
					for (phyvirt_addr =
						     (unsigned long)pdx
							     ->m_AudioInfo[i]
							     .m_pAudioData_area
								     [k];
					     phyvirt_addr <
					     ((unsigned long)pdx->m_AudioInfo[i]
						      .m_pAudioData_area[k] +
					      pdx->m_dwAudioPTKSize);
					     phyvirt_addr += PAGE_SIZE) {
						// clear all pages
						ClearPageReserved(virt_to_page(
							phyvirt_addr));
					}
					kfree(pdx->m_AudioInfo[i]
						      .m_pAudioBufData[k]);
					pdx->m_AudioInfo[i].m_pAudioBufData[k] =
						NULL;
				}
			}
		}
		pdx->m_bBufferAllocate = FALSE;
	}
}

static int DmaMemAllocPool(struct hws_pcie_dev *pdx)
{
	u32 status = 0;
	uint8_t i, k;
	dma_addr_t phy_addr;

	unsigned long phyvirt_addr;
	if (pdx->m_bBufferAllocate == TRUE) {
		DmaMemFreePool(pdx);
	}
	//------------
	for (i = 0; i < pdx->m_nMaxChl; i++) {
		//printk("kmalloc [%d]size=%X************\n", i,pdx->m_MaxHWVideoBufferSize);
		pdx->m_pbyVideoBuffer[i] = dma_alloc_coherent(
			&pdx->pdev->dev, pdx->m_MaxHWVideoBufferSize,
			&pdx->m_pbyVideo_phys[i], GFP_KERNEL);

		if (pdx->m_pbyVideoBuffer[i] == NULL) {
			printk("m_pbyVideoBuffer[%d] mem Allocate Fail ************\n",
			       i);
			pdx->m_bBufferAllocate = TRUE;
			DmaMemFreePool(pdx);
			pdx->m_bBufferAllocate = FALSE;
			status = -1;
			return status;
		}
#if 0
			pdx->m_pbyVideoBuffer_area[i] = (char *)(((unsigned long)pdx->m_pbyVideoBuffer[i] + PAGE_SIZE -1) & PAGE_MASK);
			for (phyvirt_addr=(unsigned long)pdx->m_pbyVideoBuffer_area[i]; phyvirt_addr < ((unsigned long)pdx->m_pbyVideoBuffer_area[i] + (pdx->m_MaxHWVideoBufferSize));
			phyvirt_addr+=PAGE_SIZE) 
			{
				// reserve all pages to make them remapable
				SetPageReserved(virt_to_page(phyvirt_addr));
			} 
			memset(pdx->m_pbyVideoBuffer[i] , 0x0,(pdx->m_MaxHWVideoBufferSize) );
			phy_addr= (dma_addr_t)virt_to_phys(pdx->m_pbyVideoBuffer[i]);
			pdx->m_pbyVideo_phys[i] = phy_addr;
#else
		phy_addr = pdx->m_pbyVideo_phys[i];
#endif
		//printk("PHY= %X=%X\n",phy_addr,pdx->m_pbyVideo_phys[i]);

		pdx->m_dwVideoBuffer[i] = ((u64)phy_addr) & 0xFFFFFFFF;
		pdx->m_dwVideoHighBuffer[i] = ((u64)phy_addr >> 32) &
					      0xFFFFFFFF;
		;

		pdx->m_pbyAudioBuffer[i] =
			(BYTE *)(pdx->m_pbyVideoBuffer[i] +
				 pdx->m_MaxHWVideoBufferSize -
				 MAX_AUDIO_CAP_SIZE);
#if 0
				phy_addr= (dma_addr_t)virt_to_phys(pdx->m_pbyAudioBuffer[i]);
#else
		phy_addr = pdx->m_pbyVideo_phys[i] +
			   (pdx->m_MaxHWVideoBufferSize - MAX_AUDIO_CAP_SIZE);
#endif
		pdx->m_pbyAudio_phys[i] = phy_addr;

		pdx->m_dwAudioBuffer[i] = pdx->m_dwVideoBuffer[i] +
					  pdx->m_MaxHWVideoBufferSize -
					  MAX_AUDIO_CAP_SIZE;
		pdx->m_dwAudioBufferHigh[i] = pdx->m_dwVideoHighBuffer[i];
		//printk("[MV]Mem Video::m_dwVideoBuffer[%d] = %x\n", i, pdx->m_dwVideoBuffer[i]);
		//printk("[MV]Mem Video::m_dwVideoHighBuffer[%d] = %x\n", i, pdx->m_dwVideoHighBuffer[i]);
		//printk("[MV]Mem Audio::m_dwAudioBuffer[%d] = %x\n", i, pdx->m_dwAudioBuffer[i]);
		//printk("[MV]Mem Audio::m_dwAudioBufferHigh[%d] = %x\n", i, pdx->m_dwAudioBufferHigh[i]);
	}

	//KdPrint(("Mem allocate::m_dwAudioBuffer[%d] = %x\n", i, pdx->m_dwAudioBuffer));
	//-------------- video buffer
	for (i = 0; i < pdx->m_nCurreMaxVideoChl; i++) {
		pdx->m_VideoInfo[i].m_pVideoScalerBuf =
			vmalloc(MAX_VIDEO_HW_W * MAX_VIDEO_HW_H * 2);
		if (pdx->m_VideoInfo[i].m_pVideoScalerBuf == NULL) {
			pdx->m_bBufferAllocate = TRUE;
			DmaMemFreePool(pdx);
			pdx->m_bBufferAllocate = FALSE;
			status = -1;
			return status;
		}

		pdx->m_VideoInfo[i].m_pVideoYUV2Buf =
			vmalloc(MAX_VIDEO_HW_W * MAX_VIDEO_HW_H * 2);
		if (pdx->m_VideoInfo[i].m_pVideoYUV2Buf == NULL) {
			pdx->m_bBufferAllocate = TRUE;
			DmaMemFreePool(pdx);
			pdx->m_bBufferAllocate = FALSE;
			status = -1;
			return status;
		}

		pdx->m_VideoInfo[i].m_pRotateVideoBuf =
			vmalloc(MAX_VIDEO_HW_W * MAX_VIDEO_HW_H * 2);
		if (pdx->m_VideoInfo[i].m_pRotateVideoBuf == NULL) {
			pdx->m_bBufferAllocate = TRUE;
			DmaMemFreePool(pdx);
			pdx->m_bBufferAllocate = FALSE;
			status = -1;
			return status;
		}

		for (k = 0; k < MAX_VIDEO_QUEUE; k++) {
			pdx->m_VideoInfo[i].m_pVideoBufData[k] = kmalloc(
				(pdx->m_MaxHWVideoBufferSize), GFP_KERNEL);
			if (!pdx->m_VideoInfo[i].m_pVideoBufData[k]) {
				pdx->m_bBufferAllocate = TRUE;
				DmaMemFreePool(pdx);
				pdx->m_bBufferAllocate = FALSE;
				status = -1;
				return status;

			} else {
				pdx->m_VideoInfo[i].m_pVideoData_area[k] =
					(char *)(((unsigned long)pdx
							  ->m_VideoInfo[i]
							  .m_pVideoBufData[k] +
						  PAGE_SIZE - 1) &
						 PAGE_MASK);
				for (phyvirt_addr =
					     (unsigned long)pdx->m_VideoInfo[i]
						     .m_pVideoData_area[k];
				     phyvirt_addr <
				     ((unsigned long)pdx->m_VideoInfo[i]
					      .m_pVideoData_area[k] +
				      (pdx->m_MaxHWVideoBufferSize));
				     phyvirt_addr += PAGE_SIZE) {
					// reserve all pages to make them remapable
					SetPageReserved(
						virt_to_page(phyvirt_addr));
				}
				memset(pdx->m_VideoInfo[i].m_pVideoBufData[k],
				       0x0, pdx->m_MaxHWVideoBufferSize);
			}
		}
	}

//----------audio alloc
#if 1
	for (i = 0; i < pdx->m_nCurreMaxVideoChl; i++) {
		for (k = 0; k < MAX_AUDIO_QUEUE; k++) {
			pdx->m_AudioInfo[i].m_pAudioBufData[k] =
				kmalloc(pdx->m_dwAudioPTKSize, GFP_KERNEL);
			if (!pdx->m_AudioInfo[i].m_pAudioBufData[k]) {
				pdx->m_bBufferAllocate = TRUE;
				DmaMemFreePool(pdx);
				pdx->m_bBufferAllocate = FALSE;
				status = -1;
				return status;
			} else {
				pdx->m_AudioInfo[i].pStatusInfo[k].byLock =
					MEM_UNLOCK;
				pdx->m_AudioInfo[i].m_pAudioData_area[k] =
					(char *)(((unsigned long)pdx
							  ->m_AudioInfo[i]
							  .m_pAudioBufData[k] +
						  PAGE_SIZE - 1) &
						 PAGE_MASK);
				for (phyvirt_addr =
					     (unsigned long)pdx->m_AudioInfo[i]
						     .m_pAudioData_area[k];
				     phyvirt_addr <
				     ((unsigned long)pdx->m_AudioInfo[i]
					      .m_pAudioData_area[k] +
				      pdx->m_dwAudioPTKSize);
				     phyvirt_addr += PAGE_SIZE) {
					// reserve all pages to make them remapable
					SetPageReserved(
						virt_to_page(phyvirt_addr));
				}
			}
		}
	}
#endif
	//------------------------------------------------------------
	//KdPrint(("Mem allocate::m_pAudioData = %x\n",  pdx->m_pAudioData));
	pdx->m_bBufferAllocate = TRUE;
	//KdPrint(("DmaMemAllocPool  ed\n"));
	return 0;
}

static void StopDevice(struct hws_pcie_dev *pdx)
{ // StopDevice
	//Trace t("StopDevice()");
	int i;
	//int   device_lost =0;
	u32 statusreg;
	StopDsp(pdx);
	statusreg = READ_REGISTER_ULONG(pdx, (0x4000));
	//DbgPrint("[MV] Busy!!! statusreg =%X\n", statusreg);
	if (statusreg != 0xFFFFFFFF) {
		//set to one buffer mode
		//WRITE_REGISTER_ULONG((u32)(CVBS_IN_BASE + (25*PCIE_BARADDROFSIZE)), 0x00); //Buffer 1 address
	} else {
		pdx->m_PciDeviceLost = 1;
	}
	pdx->m_bStartRun = 0;
	if (pdx->m_PciDeviceLost == 0) {
		for (i = 0; i < MAX_VID_CHANNELS; i++) {
			EnableVideoCapture(pdx, i, 0);
			EnableAudioCapture(pdx, i, 0);
		}
	}
	//if(device_lost) return;
	DmaMemFreePool(pdx);
	//printk("StopDevice Done\n");
}
void StopKSThread(struct hws_pcie_dev *pdx)
{
	if (pdx->mMain_tsk) {
		kthread_stop(pdx->mMain_tsk);
	}
}

//----------------------------
//---------------------------------------
static void CheckCardStatus(struct hws_pcie_dev *pdx)
{
	ULONG status;
	status = READ_REGISTER_ULONG(pdx, HWS_REG_SYS_STATUS);

	//DbgPrint("CheckCardStatus =%X",status);
	if ((status & BIT(0)) != BIT(0)) {
		//DbgPrint("CheckCardStatus =%X",status);
		InitVideoSys(pdx, 1);
	}
}


static int SetQuene(struct hws_pcie_dev *pdx, int nDecoder)
{
	int status = -1;
	//KLOCK_QUEUE_HANDLE  oldirql;
	//DbgPrint("SetQuene %d",nDecoder);
	if (!pdx->m_bStartRun) {
		return -1;
	}
	if (!pdx->m_bVCapStarted[nDecoder]) {
		if (pdx->m_bVideoStop[nDecoder] == 1) {
			pdx->m_bVideoStop[nDecoder] = 0;
			//DbgPrint("KeSetEvent Exit Event[%d]\n",nDecoder);
		}

		return -1;
	}
	//-------------------
	if (pdx->m_DeviceHW_Version == 0) {
		pdx->m_dwSWFrameRate[nDecoder]++;
	}
	//-------------------
	pdx->m_nVideoBusy[nDecoder] = 1;
	//-------------------------------
	if (pdx->m_bVCapStarted[nDecoder] == TRUE) {
		status = MemCopyVideoToSteam(pdx, nDecoder);
	}
	pdx->m_nVideoBusy[nDecoder] = 0;
	return status;
}


static void SetDMAAddress(struct hws_pcie_dev *pdx)
{
	//-------------------------------------

	u32 Addrmsk;
	u32 AddrLowmsk;
	//u32 AddrPageSize;
	//u32 Addr2PageSize;
	u32 PhyAddr_A_Low;
	u32 PhyAddr_A_High;

	//u32 PhyAddr_A_Low2;
	//u32 PhyAddr_A_High2;
	//u32 PCI_Addr2;

	u32 PCI_Addr;
	//u32 AVALON_Addr;
	u32 cnt;
	//u64 m_tmp64cnt = 0;
	//u32 RDAvalon = 0;
	//u32 m_AddreeSpace = 0;
	int i = 0;
	u32 m_ReadTmp;
	u32 m_ReadTmp2;
	//u32 m_ReadTmp3;
	//u32 m_ReadTmp4;
	DWORD halfframeLength = 0;
	//DWORD m_Valude;
	PhyAddr_A_High = 0;
	PhyAddr_A_Low = 0;
	PCI_Addr = 0;

	//------------------------------------------ // re write dma register

	Addrmsk = PCI_E_BAR_ADD_MASK;
	AddrLowmsk = PCI_E_BAR_ADD_LOWMASK;

	//printk("[MV]1DispatchCreate :Addrmsk = %X  AddrPageSize =%X Addr2PageSize =%X \n", Addrmsk, AddrPageSize, Addr2PageSize);

	cnt = 0x208; // Table address
	for (i = 0; i < pdx->m_nMaxChl; i++) {
		//printk("[MV] pdx->m_pbyVideoBuffer[%d]=%x\n", i, pdx->m_pbyVideoBuffer[i]);
		if (pdx->m_pbyVideoBuffer[i]) {
			PhyAddr_A_Low = pdx->m_dwVideoBuffer[i];
			PhyAddr_A_High = pdx->m_dwVideoHighBuffer[i];

			PCI_Addr = (PhyAddr_A_Low & AddrLowmsk);
			PhyAddr_A_Low = (PhyAddr_A_Low & Addrmsk);

			//printk("[MV]1-pdx->m_dwVideoBuffer[%d]-%X\n",i,pdx->m_dwVideoBuffer[i]);
			//-------------------------------------------------------------------------------
			WRITE_REGISTER_ULONG(pdx, (PCI_ADDR_TABLE_BASE + cnt),
					     PhyAddr_A_High);
			WRITE_REGISTER_ULONG(pdx,
					     (PCI_ADDR_TABLE_BASE + cnt +
					      PCIE_BARADDROFSIZE),
					     PhyAddr_A_Low); //Entry 0
			//----------------------------------------
			m_ReadTmp = READ_REGISTER_ULONG(
				pdx, (PCI_ADDR_TABLE_BASE + cnt));
			m_ReadTmp2 = READ_REGISTER_ULONG(
				pdx, (PCI_ADDR_TABLE_BASE + cnt +
				      PCIE_BARADDROFSIZE));
			//printk("[MV]1-PCI_Addr[%d] :PhyAddr_A_Low  %X=%X  PhyAddr_A_High %X=%X\n", i, PhyAddr_A_Low, m_ReadTmp2, PhyAddr_A_High, m_ReadTmp);

			//--------------------------
			WRITE_REGISTER_ULONG(
				pdx,
				(CBVS_IN_BUF_BASE + (i * PCIE_BARADDROFSIZE)),
				((i + 1) * PCIEBAR_AXI_BASE) +
					PCI_Addr); //Buffer 1 address
			halfframeLength = pdx->m_format[i].HLAF_SIZE / 16;
			WRITE_REGISTER_ULONG(
				pdx,
				(CBVS_IN_BUF_BASE2 + (i * PCIE_BARADDROFSIZE)),
				halfframeLength); //Buffer 1 address

			m_ReadTmp = READ_REGISTER_ULONG(
				pdx,
				(CBVS_IN_BUF_BASE + (i * PCIE_BARADDROFSIZE)));
			m_ReadTmp2 = READ_REGISTER_ULONG(
				pdx,
				(CBVS_IN_BUF_BASE2 + (i * PCIE_BARADDROFSIZE)));
			//printk("[MV]1-Avalone [X64]BUF[%d]:BUF1=%X  BUF2=%X\n", i,  m_ReadTmp,  m_ReadTmp2);

			//---------------------------
		}
		cnt += 8;
#if 1
		if (pdx->m_pbyAudioBuffer[i]) {
			PhyAddr_A_Low = pdx->m_dwAudioBuffer[i];
			PhyAddr_A_High = pdx->m_dwAudioBufferHigh[i];
			PCI_Addr = (PhyAddr_A_Low & AddrLowmsk);
			PhyAddr_A_Low = (PhyAddr_A_Low & Addrmsk);
			//printk("[X1]Audio:PCI_Addr =%X\n",PCI_Addr);
			//printk("[X1]Audio:-------- - LOW=%X  HIGH =%X\n",pdx->m_dwAudioBuffer[i],pdx->m_dwAudioBufferHigh[i]);
			WRITE_REGISTER_ULONG(pdx,
					     (CBVS_IN_BUF_BASE +
					      ((8 + i) * PCIE_BARADDROFSIZE)),
					     ((i + 1) * PCIEBAR_AXI_BASE +
					      PCI_Addr)); //Buffer 1 address
			m_ReadTmp = READ_REGISTER_ULONG(
				pdx, (CBVS_IN_BUF_BASE +
				      ((8 + i) * PCIE_BARADDROFSIZE)));
			//printk("[X1]Audio:[%d] :--------BUF1: %X=%X\n",i,(PCIEBAR_AXI_BASE+PCI_Addr),m_ReadTmp);
		}
#endif
	}
	WRITE_REGISTER_ULONG(pdx, INT_EN_REG_BASE,
			     0x3ffff); //enable PCI Interruput
	//WRITE_REGISTER_ULONG(PCIEBR_EN_REG_BASE, 0xFFFFFFFF);
}

//-----------------------------------



int MainKsThreadHandle(void *arg)
{
	int need_check = 0;
	int i = 0;
	struct hws_pcie_dev *pdx = (struct hws_pcie_dev *)(arg);
	while (1) {
		need_check = 0;
		for (i = 0; i < pdx->m_nMaxChl; i++) {
			if (pdx->m_bVCapStarted[i] == 1) {
				need_check = 1;
				break;
			}
		}
		if (need_check == 1) {
			CheckVideoFmt(pdx);
		}
		ssleep(1);
		if (kthread_should_stop()) {
			break;
		}
	}
	//printk("MainKsThreadHandle Exit");
	return 0;
}
static void StartKSThread(struct hws_pcie_dev *pdx)
{
	pdx->mMain_tsk = kthread_run(MainKsThreadHandle, (void *)pdx,
				     "StartKSThread task");
}

//------------------------------

#ifndef arch_msi_check_device
int arch_msi_check_device(struct pci_dev *dev, int nvec, int type)
{
	return 0;
}
#endif

/* type = PCI_CAP_ID_MSI or PCI_CAP_ID_MSIX */
static int msi_msix_capable(struct pci_dev *dev, int type)
{
	struct pci_bus *bus;
	int ret;
	//printk("msi_msix_capable in \n");
	if (!dev || dev->no_msi) {
		printk("msi_msix_capable no_msi exit \n");
		return 0;
	}

	for (bus = dev->bus; bus; bus = bus->parent) {
		if (bus->bus_flags & PCI_BUS_FLAGS_NO_MSI) {
			printk("msi_msix_capable PCI_BUS_FLAGS_NO_MSI \n");
			return 0;
		}
	}
	ret = arch_msi_check_device(dev, 1, type);
	if (ret) {
		return 0;
	}
	ret = pci_find_capability(dev, type);
	if (!ret) {
		printk("msi_msix_capable pci_find_capability =%d\n", ret);
		return 0;
	}

	return 1;
}

static int probe_scan_for_msi(struct hws_pcie_dev *lro, struct pci_dev *pdev)
{
	//int i;
	int rc = 0;
	//int req_nvec = MAX_NUM_ENGINES + MAX_USER_IRQ;

	//BUG_ON(!lro);
	//BUG_ON(!pdev);
	//if (msi_msix_capable(pdev, PCI_CAP_ID_MSIX)) {
	//		printk("Enabling MSI-X\n");
	//		for (i = 0; i < req_nvec; i++)
	//			lro->entry[i].entry = i;
	//
	//		rc = pci_enable_msix(pdev, lro->entry, req_nvec);
	//		if (rc < 0)
	//			printk("Couldn't enable MSI-X mode: rc = %d\n", rc);

	//		lro->msix_enabled = 1;
	//		lro->msi_enabled = 0;
	//	}
	//else

	if (msi_msix_capable(pdev, PCI_CAP_ID_MSI)) {
		/* enable message signalled interrupts */
		//printk("pci_enable_msi()\n");
		rc = pci_enable_msi(pdev);
		if (rc < 0) {
			printk("Couldn't enable MSI mode: rc = %d\n", rc);
		}
		lro->msi_enabled = 1;
		lro->msix_enabled = 0;
	} else {
		//printk("MSI/MSI-X not detected - using legacy interrupts\n");
		lro->msi_enabled = 0;
		lro->msix_enabled = 0;
	}

	return rc;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)
static void enable_pcie_relaxed_ordering(struct pci_dev *dev)
{
	pcie_capability_set_word(dev, PCI_EXP_DEVCTL, PCI_EXP_DEVCTL_RELAX_EN);
}
#else
static void __devinit enable_pcie_relaxed_ordering(struct pci_dev *dev)
{
	u16 v;
	int pos;

	pos = pci_pcie_cap(dev);
	if (pos > 0) {
		pci_read_config_word(dev, pos + PCI_EXP_DEVCTL, &v);
		v |= PCI_EXP_DEVCTL_RELAX_EN;
		pci_write_config_word(dev, pos + PCI_EXP_DEVCTL, v);
	}
}
#endif
//--------------------------------------

//-------------------------------------
static void SetHardWareInfo(struct hws_pcie_dev *pdx)
{
	switch (pdx->dwDeviceID) {
	case 0x9534:
	case 0x6524:
	case 0x8524: {
		pdx->m_nCurreMaxVideoChl = 4;
		pdx->m_nCurreMaxLineInChl = 1;
		pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
		break;
	}
	case 0x8532: {
		pdx->m_nCurreMaxVideoChl = 2;
		pdx->m_nCurreMaxLineInChl = 1;
		pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
		break;
	}
	case 0x8512:
	case 0x6502: {
		pdx->m_nCurreMaxVideoChl = 2;
		pdx->m_nCurreMaxLineInChl = 0;
		pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
		break;
	}
	case 0x8501: {
		pdx->m_nCurreMaxVideoChl = 1;
		pdx->m_nCurreMaxLineInChl = 0;
		pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
		break;
	}
	default: {
		pdx->m_nCurreMaxVideoChl = 4;
		pdx->m_nCurreMaxLineInChl = 0;
		pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
		break;
	}
	}
	//-----------------------
	if (pdx->m_Device_Version > 121) {
		if ((pdx->dwDeviceID == 0x8501) &&
		    (pdx->m_Device_Version == 122)) {
			pdx->m_DeviceHW_Version = 0;
		} else {
			//DWORD m_ReadTmp;
			pdx->m_DeviceHW_Version = 1;
			//--- set DMA_MAX_SIZE
			WRITE_REGISTER_ULONG(pdx, HWS_REG_DMA_MAX_SIZE,
					     (MAX_VIDEO_SCLAER_SIZE / 16));
			//m_ReadTmp = ReadDevReg((DWORD)(CVBS_IN_BASE + (9 * PCIE_BARADDROFSIZE)));
			//DbgPrint("[MV]DMA_MAX_SIZE =%d\n",m_ReadTmp);
		}
	} else {
		pdx->m_DeviceHW_Version = 0;
	}
	//------------------
}
static int ReadChipId(struct hws_pcie_dev *pdx)
{
	//  CCIR_PACKET      reg;
	//int Chip_id1 = 0;
	int ret = 0;
	//int reg_vaule = 0;
	//int nResult;
	int i;
	//------read Dvice Version
	ULONG m_dev_ver;
	ULONG m_tmpVersion;
	ULONG m_tmpHWKey;
	//ULONG m_OEM_code_data;
	m_dev_ver = READ_REGISTER_ULONG(pdx, HWS_REG_DEVICE_INFO);

	/* Bits 7:0   = device version */
	m_tmpVersion = m_dev_ver >> 8;
	pdx->m_Device_Version = (m_tmpVersion & 0xFF);

	/* Bits 15:8  = device subversion */
	m_tmpVersion = m_dev_ver >> 16;
	pdx->m_Device_SubVersion = (m_tmpVersion & 0xFF);

	/* Bits 31:28 = YV12 support flags (4 bits) */
	pdx->m_Device_SupportYV12 = ((m_dev_ver >> 28) & 0x0F);

	/* Bits 27:24 = HW key; low two bits of that = port ID */
	m_tmpHWKey = (m_dev_ver >> 24) & 0x0F;
	pdx->m_Device_PortID = (m_tmpHWKey & 0x03);

	//n_VideoModle =	READ_REGISTER_ULONG(pdx,0x4000+(4*PCIE_BARADDROFSIZE));
	//n_VideoModle = (n_VideoModle>>8)&0xFF;
	//pdx->m_IsHDModel = 1;
	pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
	pdx->m_nMaxChl = 4;
	pdx->m_bBufferAllocate = FALSE;
	pdx->mMain_tsk = NULL;
	pdx->m_dwAudioPTKSize = MAX_DMA_AUDIO_PK_SIZE; //128*16*4;
	pdx->m_bStartRun = 0;
	pdx->m_PciDeviceLost = 0;

	//--------
	for (i = 0; i < MAX_VID_CHANNELS; i++) {
		SetVideoFormteSize(pdx, i, 1920, 1080);
	}
	//-------
	WRITE_REGISTER_ULONG(pdx, HWS_REG_DEC_MODE, 0x0);
	//ssleep(100);
	WRITE_REGISTER_ULONG(pdx, HWS_REG_DEC_MODE, 0x10);
	//ssleep(500);
	//-------
	SetHardWareInfo(pdx);
	printk("************[HW]-[VIDV]=[%d]-[%d]-[%d] ************\n",
	       pdx->m_Device_Version, pdx->m_Device_SubVersion,
	       pdx->m_Device_PortID);
	return ret;
}


MODULE_DEVICE_TABLE(pci, hws_pci_table);

static struct pci_driver hws_pci_driver = {
	.name = KBUILD_MODNAME,
	.id_table = hws_pci_table,
	.probe = hws_probe,
	.remove = hws_remove,
};


module_init(pcie_hws_init);
module_exit(pcie_hws_exit);

MODULE_DESCRIPTION("HWS driver");
MODULE_AUTHOR("Sales <sales@avmatrix.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
