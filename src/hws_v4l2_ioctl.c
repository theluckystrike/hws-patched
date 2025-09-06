// SPDX-License-Identifier: GPL-2.0-only
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/pci.h>
#include <linux/errno.h>
#include <linux/io.h>

#include <media/v4l2-ioctl.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-dv-timings.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>

#include "hws.h"
#include "hws_reg.h"
#include "hws_video.h"

static const struct v4l2_dv_timings hws_dv_modes[] = {
	{ .type = V4L2_DV_BT_656_1120, .bt = { .width = 1920, .height = 1080, .interlaced = 0 } },
	{ .type = V4L2_DV_BT_656_1120, .bt = { .width = 1280, .height =  720, .interlaced = 0 } },
	{ .type = V4L2_DV_BT_656_1120, .bt = { .width =  720, .height =  480, .interlaced = 0 } },
	{ .type = V4L2_DV_BT_656_1120, .bt = { .width =  720, .height =  576, .interlaced = 0 } },
	{ .type = V4L2_DV_BT_656_1120, .bt = { .width =  800, .height =  600, .interlaced = 0 } },
	{ .type = V4L2_DV_BT_656_1120, .bt = { .width =  640, .height =  480, .interlaced = 0 } },
	{ .type = V4L2_DV_BT_656_1120, .bt = { .width = 1024, .height =  768, .interlaced = 0 } },
	{ .type = V4L2_DV_BT_656_1120, .bt = { .width = 1280, .height =  768, .interlaced = 0 } },
	{ .type = V4L2_DV_BT_656_1120, .bt = { .width = 1280, .height =  800, .interlaced = 0 } },
	{ .type = V4L2_DV_BT_656_1120, .bt = { .width = 1280, .height = 1024, .interlaced = 0 } },
	{ .type = V4L2_DV_BT_656_1120, .bt = { .width = 1360, .height =  768, .interlaced = 0 } },
	{ .type = V4L2_DV_BT_656_1120, .bt = { .width = 1440, .height =  900, .interlaced = 0 } },
	{ .type = V4L2_DV_BT_656_1120, .bt = { .width = 1680, .height = 1050, .interlaced = 0 } },
	/* Portrait */
	{ .type = V4L2_DV_BT_656_1120, .bt = { .width = 1080, .height = 1920, .interlaced = 0 } },
};

static const size_t hws_dv_modes_cnt = ARRAY_SIZE(hws_dv_modes);

/* YUYV: 16 bpp; align to 64 as you did elsewhere */
static inline u32 hws_calc_bpl_yuyv(u32 w)     { return ALIGN(w * 2, 64); }
static inline u32 hws_calc_size_yuyv(u32 w, u32 h) { return hws_calc_bpl_yuyv(w) * h; }

static inline void hws_hw_write_bchs(struct hws_pcie_dev *hws, unsigned int ch,
				     u8 br, u8 co, u8 hu, u8 sa)
{
	u32 packed = (sa << 24) | (hu << 16) | (co << 8) | br;

	if (!hws || !hws->bar0_base || ch >= hws->max_channels)
		return;
	writel_relaxed(packed, hws->bar0_base + HWS_REG_BCHS(ch));
	(void)readl(hws->bar0_base + HWS_REG_BCHS(ch)); /* post write */
}

/* Find a timing by WxH (progressive). If multiple rates exist, we pick the first match. */
static const struct v4l2_dv_timings *hws_match_mode_by_wh(unsigned int w, unsigned int h)
{
	size_t i;

	for (i = 0; i < hws_dv_modes_cnt; i++) {
		const struct v4l2_bt_timings *bt = &hws_dv_modes[i].bt;

		if (bt->width == w && bt->height == h && bt->interlaced == 0)
			return &hws_dv_modes[i];
	}
	return NULL;
}

/* Helper: find a supported DV mode by W/H + interlace flag */
static const struct v4l2_dv_timings *
hws_find_dv_by_wh(u32 w, u32 h, bool interlaced)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(hws_dv_modes); i++) {
		const struct v4l2_dv_timings *t = &hws_dv_modes[i];
		const struct v4l2_bt_timings *bt = &t->bt;

		if (t->type != V4L2_DV_BT_656_1120)
			continue;

		if (bt->width == w && bt->height == h &&
		    !!bt->interlaced == interlaced)
			return t;
	}
	return NULL;
}

/* Helper: validate the requested timings against our table.
 * We match strictly on type, width, height, and interlace flag.
 */
static const struct v4l2_dv_timings *
hws_match_supported_dv(const struct v4l2_dv_timings *req)
{
	const struct v4l2_bt_timings *bt;

	if (!req || req->type != V4L2_DV_BT_656_1120)
		return NULL;

	bt = &req->bt;
	return hws_find_dv_by_wh(bt->width, bt->height, !!bt->interlaced);
}

/* Query the *current detected* DV timings on the input.
 * If you have a real hardware detector, call it here; otherwise we
 * derive from the cached pix state and map to the closest supported DV mode.
 */
int hws_vidioc_query_dv_timings(struct file *file, void *fh,
				struct v4l2_dv_timings *timings)
{
	struct hws_video *vid = video_drvdata(file);
	const struct v4l2_dv_timings *m;

	if (!timings)
		return -EINVAL;

	/* Map current cached WxH/interlace to one of our supported modes. */
	m = hws_find_dv_by_wh(vid->pix.width, vid->pix.height,
			      !!vid->pix.interlaced);
	if (!m)
		return -ENOLINK;

	*timings = *m;
	return 0;
}

/* Enumerate the Nth supported DV timings from our static table. */
int hws_vidioc_enum_dv_timings(struct file *file, void *fh,
			       struct v4l2_enum_dv_timings *edv)
{
	if (!edv)
		return -EINVAL;

	if (edv->pad)
		return -EINVAL;

	if (edv->index >= hws_dv_modes_cnt)
		return -EINVAL;

	edv->timings = hws_dv_modes[edv->index];
	return 0;
}

/* Get the *currently configured* DV timings. */
int hws_vidioc_g_dv_timings(struct file *file, void *fh,
			    struct v4l2_dv_timings *timings)
{
	struct hws_video *vid = video_drvdata(file);
	const struct v4l2_dv_timings *m;

	if (!timings)
		return -EINVAL;

	m = hws_find_dv_by_wh(vid->pix.width, vid->pix.height,
			      !!vid->pix.interlaced);
	if (!m)
		return -ENOLINK;

	*timings = *m;
	return 0;
}

static inline void hws_set_colorimetry_state(struct hws_pix_state *p)
{
	bool sd = p->height <= 576;

	p->colorspace   = sd ? V4L2_COLORSPACE_SMPTE170M : V4L2_COLORSPACE_REC709;
	p->ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;
	p->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	p->xfer_func    = V4L2_XFER_FUNC_DEFAULT;
}

/* Set DV timings: must match one of our supported modes.
 * If buffers are queued and this implies a size change, we reject with -EBUSY.
 * Otherwise we update pix state and (optionally) reprogram the HW.
 */
int hws_vidioc_s_dv_timings(struct file *file, void *fh,
			    struct v4l2_dv_timings *timings)
{
	struct hws_video *vid = video_drvdata(file);
	const struct v4l2_dv_timings *m;
	const struct v4l2_bt_timings *bt;
	u32 new_w, new_h;
	bool interlaced;
	int ret = 0;
	unsigned long was_busy;

	if (!timings)
		return -EINVAL;

	m = hws_match_supported_dv(timings);
	if (!m)
		return -EINVAL;

	bt = &m->bt;
	new_w = bt->width;
	new_h = bt->height;
	interlaced = !!bt->interlaced;

    lockdep_assert_held(&vid->qlock);

	/* If vb2 has active buffers and size would change, reject. */
	was_busy = vb2_is_busy(&vid->buffer_queue);
	if (was_busy &&
	    (new_w != vid->pix.width || new_h != vid->pix.height ||
	     interlaced != vid->pix.interlaced)) {
		ret = -EBUSY;
		return ret;
	}

	/* Update software pixel state (and recalc sizes) */
	vid->pix.width      = new_w;
	vid->pix.height     = new_h;
	vid->pix.field      = interlaced ? V4L2_FIELD_INTERLACED
					 : V4L2_FIELD_NONE;
	vid->pix.interlaced = interlaced;
	vid->pix.fourcc     = V4L2_PIX_FMT_YUYV;

	hws_set_colorimetry_state(&vid->pix);

	/* Recompute stride/sizeimage/half_size using your helper */
	vid->pix.bytesperline = hws_calc_bpl_yuyv(new_w);
	vid->pix.sizeimage    = hws_calc_size_yuyv(new_w, new_h);
	if (!was_busy)
		vid->alloc_sizeimage = vid->pix.sizeimage;
	return ret;
}

/* Report DV timings capability: advertise BT.656/1120 with
 * the min/max WxH derived from our table and basic progressive support.
 */
int hws_vidioc_dv_timings_cap(struct file *file, void *fh,
			      struct v4l2_dv_timings_cap *cap)
{
	u32 min_w = ~0U, min_h = ~0U;
	u32 max_w = 0,       max_h = 0;
	size_t i, n = 0;

	if (!cap)
		return -EINVAL;

	memset(cap, 0, sizeof(*cap));
	cap->type = V4L2_DV_BT_656_1120;

	for (i = 0; i < ARRAY_SIZE(hws_dv_modes); i++) {
		const struct v4l2_bt_timings *bt = &hws_dv_modes[i].bt;

		if (hws_dv_modes[i].type != V4L2_DV_BT_656_1120)
			continue;
		n++;

		if (bt->width  < min_w)
			min_w = bt->width;
		if (bt->height < min_h)
			min_h = bt->height;
		if (bt->width  > max_w)
			max_w = bt->width;
		if (bt->height > max_h)
			max_h = bt->height;
	}

	/* If the table was empty, fail gracefully. */
	if (!n || min_w == U32_MAX)
		return -ENODATA;

	cap->bt.min_width  = min_w;
	cap->bt.max_width  = max_w;
	cap->bt.min_height = min_h;
	cap->bt.max_height = max_h;

	/* We support both CEA-861- and VESA-style modes in the list. */
	cap->bt.standards =
		V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT | V4L2_DV_BT_STD_CVT;

	/* Progressive only, unless your table includes interlaced entries. */
	cap->bt.capabilities = V4L2_DV_BT_CAP_PROGRESSIVE;

	/* Leave pixelclock/porch limits unconstrained (0) for now. */
	return 0;
}

static int hws_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hws_video *vid =
		container_of(ctrl->handler, struct hws_video, control_handler);
	struct hws_pcie_dev *pdx = vid->parent;

	switch (ctrl->id) {
	case V4L2_CID_DV_RX_IT_CONTENT_TYPE:
		// FIXME
		// ctrl->val = hdmi_content_type(vid); /* unchanged */
		return 0;

	default:
		return -EINVAL;
	}
}

static int hws_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hws_video *vid =
		container_of(ctrl->handler, struct hws_video, control_handler);
	struct hws_pcie_dev *pdx = vid->parent;
	bool program = false;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		vid->current_brightness = ctrl->val;
		program = true;
		break;
	case V4L2_CID_CONTRAST:
		vid->current_contrast = ctrl->val;
		program = true;
		break;
	case V4L2_CID_SATURATION:
		vid->current_saturation = ctrl->val;
		program = true;
		break;
	case V4L2_CID_HUE:
		vid->current_hue = ctrl->val;
		program = true;
		break;
	default:
		return -EINVAL;
	}

	if (program) {
		hws_hw_write_bchs(pdx, vid->channel_index,
				  (u8)vid->current_brightness,
				  (u8)vid->current_contrast,
				  (u8)vid->current_hue,
				  (u8)vid->current_saturation);
	}
	return 0;
}

const struct v4l2_ctrl_ops hws_ctrl_ops = {
	.s_ctrl          = hws_s_ctrl,
	.g_volatile_ctrl = hws_g_volatile_ctrl,
};

int hws_vidioc_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	struct hws_video *vid = video_drvdata(file);
	struct hws_pcie_dev *pdev = vid->parent;
	int vi_index = vid->channel_index + 1; /* keep it simple */

	strscpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	snprintf(cap->card, sizeof(cap->card), "%s %d", KBUILD_MODNAME, vi_index);
	snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s", dev_name(&pdev->pdev->dev));

	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

int hws_vidioc_enum_fmt_vid_cap(struct file *file, void *priv_fh, struct v4l2_fmtdesc *f)
{
	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (f->index != 0)
		return -EINVAL; /* only one format */

	strscpy(f->description, "YUYV 4:2:2", sizeof(f->description));
	f->pixelformat = V4L2_PIX_FMT_YUYV;
	return 0;
}

int hws_vidioc_g_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *fmt)
{
	struct hws_video *vid = video_drvdata(file);

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	fmt->fmt.pix.width        = vid->pix.width;
	fmt->fmt.pix.height       = vid->pix.height;
	fmt->fmt.pix.pixelformat  = V4L2_PIX_FMT_YUYV;
	fmt->fmt.pix.field        = vid->pix.field;
	fmt->fmt.pix.bytesperline = vid->pix.bytesperline;
	fmt->fmt.pix.sizeimage    = vid->pix.sizeimage;
	fmt->fmt.pix.colorspace   = vid->pix.colorspace;
	fmt->fmt.pix.ycbcr_enc    = vid->pix.ycbcr_enc;
	fmt->fmt.pix.quantization = vid->pix.quantization;
	fmt->fmt.pix.xfer_func    = vid->pix.xfer_func;
	return 0;
}

static inline void hws_set_colorimetry_fmt(struct v4l2_pix_format *p)
{
	bool sd = p->height <= 576;

	p->colorspace   = sd ? V4L2_COLORSPACE_SMPTE170M : V4L2_COLORSPACE_REC709;
	p->ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;
	p->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	p->xfer_func    = V4L2_XFER_FUNC_DEFAULT;
}

int hws_vidioc_try_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *f)
{
    struct v4l2_pix_format *pix = &f->fmt.pix;
    u32 req_w = pix->width, req_h = pix->height;
    u32 w, h, min_bpl, bpl;
    size_t size; /* wider than u32 for overflow check */

    if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;

    /* Only YUYV */
    pix->pixelformat = V4L2_PIX_FMT_YUYV;

    /* Defaults then clamp */
    w = (req_w ? req_w : 640);
    h = (req_h ? req_h : 480);
    if (w > MAX_VIDEO_HW_W) w = MAX_VIDEO_HW_W;
    if (h > MAX_VIDEO_HW_H) h = MAX_VIDEO_HW_H;
    if (!w) w = 640;  /* hard fallback in case macros are odd */
    if (!h) h = 480;

    /* Field policy */
    pix->field = V4L2_FIELD_NONE;

    /* Stride policy for packed 16bpp, 64B align */
    min_bpl = ALIGN(w * 2, 64);

    /* Bound requested bpl to something sane, then align */
    bpl = pix->bytesperline;
    if (bpl < min_bpl)
        bpl = min_bpl;
    else {
        /* Cap at 16x width to avoid silly values that overflow sizeimage */
        u32 max_bpl = ALIGN(w * 2 * 16, 64);
        if (bpl > max_bpl)
            bpl = max_bpl;
        bpl = ALIGN(bpl, 64);
    }

    /* Overflow-safe sizeimage = bpl * h */
    if (__builtin_mul_overflow((size_t)bpl, (size_t)h, &size) || size == 0)
        return -ERANGE; /* compliance-friendly: reject impossible requests */

    pix->width        = w;
    pix->height       = h;
    pix->bytesperline = bpl;
    pix->sizeimage    = (u32)size; /* logical size, not page-aligned */

    hws_set_colorimetry_fmt(pix);
    pr_debug("try_fmt: w=%u h=%u bpl=%u size=%u field=%u\n",
             pix->width, pix->height, pix->bytesperline,
             pix->sizeimage, pix->field);
    return 0;
}


int hws_vidioc_s_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct hws_video *vid = video_drvdata(file);
	int ret;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/* Normalize the request */
	ret = hws_vidioc_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	/* Don’t allow size changes while buffers are queued */
	if (vb2_is_busy(&vid->buffer_queue)) {
		if (f->fmt.pix.width       != vid->pix.width  ||
		    f->fmt.pix.height      != vid->pix.height ||
		    f->fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
			return -EBUSY;
		}
	}

	/* Apply to driver state */
	vid->pix.width        = f->fmt.pix.width;
	vid->pix.height       = f->fmt.pix.height;
	vid->pix.fourcc       = V4L2_PIX_FMT_YUYV;
	vid->pix.field        = f->fmt.pix.field;
	vid->pix.colorspace   = f->fmt.pix.colorspace;
	vid->pix.ycbcr_enc    = f->fmt.pix.ycbcr_enc;
	vid->pix.quantization = f->fmt.pix.quantization;
	vid->pix.xfer_func    = f->fmt.pix.xfer_func;

	/* Update sizes (use helper if you prefer strict alignment math) */
    vid->pix.bytesperline = f->fmt.pix.bytesperline;          /* aligned */
    vid->pix.sizeimage    = f->fmt.pix.sizeimage;             /* logical */
	vid->pix.half_size    = vid->pix.sizeimage / 2;
	vid->pix.interlaced   = false;
	/* Or:
	 * hws_calc_sizeimage(vid, vid->pix.width, vid->pix.height, false);
	 */

	/* Refresh vb2 watermark when idle */
	if (!vb2_is_busy(&vid->buffer_queue))
		vid->alloc_sizeimage = PAGE_ALIGN(vid->pix.sizeimage);
    pr_debug("s_fmt:   w=%u h=%u bpl=%u size=%u alloc=%u\n",
         vid->pix.width, vid->pix.height, vid->pix.bytesperline,
         vid->pix.sizeimage, vid->alloc_sizeimage);

	return 0;
}

int hws_vidioc_g_parm(struct file *file, void *fh, struct v4l2_streamparm *param)
{
	struct hws_video *vid = video_drvdata(file);

	if (param->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/* Fixed 60 Hz; expose timeperframe capability */
	param->parm.capture.capability           = V4L2_CAP_TIMEPERFRAME;
	param->parm.capture.capturemode          = 0;
	param->parm.capture.timeperframe.numerator   = 1;
	param->parm.capture.timeperframe.denominator = 60;
	param->parm.capture.extendedmode         = 0;
	param->parm.capture.readbuffers          = 0;

	return 0;
}

int hws_vidioc_enum_input(struct file *file, void *priv,
			  struct v4l2_input *input)
{
	if (input->index)
		return -EINVAL;
	input->type         = V4L2_INPUT_TYPE_CAMERA;
	strscpy(input->name, KBUILD_MODNAME, sizeof(input->name));
	input->capabilities = V4L2_IN_CAP_DV_TIMINGS;
	input->status       = 0;

	return 0;
}

int hws_vidioc_g_input(struct file *file, void *priv, unsigned int *index)
{
	*index = 0;
	return 0;
}

int hws_vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	return i ? -EINVAL : 0;
}

int vidioc_log_status(struct file *file, void *priv)
{
	return 0;
}

int hws_vidioc_s_parm(struct file *file, void *fh, struct v4l2_streamparm *param)
{
	struct hws_video *vid = video_drvdata(file);
	struct v4l2_captureparm *cap;

	if (param->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	cap = &param->parm.capture;

    /* Treat 0/0 as “pick a default”, clamp anything else to 60 Hz */
    if (!cap->timeperframe.numerator || !cap->timeperframe.denominator) {
        cap->timeperframe.numerator   = 1;
        cap->timeperframe.denominator = 60;
    } else {
        cap->timeperframe.numerator   = 1;
        cap->timeperframe.denominator = 60;
    }

	cap->timeperframe.numerator   = 1;
	cap->timeperframe.denominator = 60;
	cap->capability               = V4L2_CAP_TIMEPERFRAME;
	cap->capturemode              = 0;
	cap->extendedmode             = 0;
	/* readbuffers left unchanged or zero; vb2 handles queue depth */

	return 0;
}
