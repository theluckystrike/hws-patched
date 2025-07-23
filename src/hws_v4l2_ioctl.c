/* SPDX-License-Identifier: GPL-2.0-only */
#include "hws.h"
#include "hws_reg.h"
#include "hws_v4l2_tables.h"

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

static int hws_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hws_video *vid =
		container_of(ctrl->handler, struct hws_video, ctrl_handler);
	struct hws_pcie_dev *pdx = vid->dev; /* if you keep this ptr */

	switch (ctrl->id) {
	case V4L2_CID_DV_RX_POWER_PRESENT:
		/* bit 3 (+5 V) over the two pipes for this HDMI port           */
		// FIXME
		//ctrl->val = !!(hws_read_port_hpd(pdx, vid->port) & HWS_5V_BIT);
		return 0;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0))
	case V4L2_CID_DV_RX_HOTPLUG_PRESENT:
		/* bit 0 (HPD) */
		// FIXME
		// ctrl->val = !!(hws_read_port_hpd(pdx, vid->port) & HWS_HPD_BIT);
		return 0;
#endif

	case V4L2_CID_DV_RX_IT_CONTENT_TYPE:
		// FIXME
		// ctrl->val = hdmi_content_type(vid); /* unchanged */
		return 0;

	default:
		return -EINVAL;
	}
}

const struct v4l2_ctrl_ops hws_ctrl_ops = {
	.g_volatile_ctrl = hws_g_volatile_ctrl,
};
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
int hws_vidioc_querycap(struct file *file, void *priv,
			       struct v4l2_capability *cap)
{
	struct hws_video *videodev = video_drvdata(file);
	struct hws_pcie_dev *dev = videodev->dev;
	int vi_index;
	vi_index = videodev->index + 1 +
		   dev->m_Device_PortID * dev->m_nCurreMaxVideoChl;

	strlcpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	snprintf(cap->card, sizeof(cap->card), "%s %d", HWS_VIDEO_NAME, vi_index);
	snprintf(cap->bus_info, sizeof(cap->bus_info), "HWS-%s-%d", HWS_VIDEO_NAME, vi_index);

	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

int hws_vidioc_enum_fmt_vid_cap(struct file *file, void *priv_fh,
				       struct v4l2_fmtdesc *f)
{
	struct hws_video *videodev = video_drvdata(file);
	const framegrabber_pixfmt_t *pixfmt;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	int index = f->index;

	if (videodev) {
		if (f->index < 0) {
			return -EINVAL;
		}
		if (f->index >= FRAMEGRABBER_PIXFMT_MAX) {
			return -EINVAL;
		} else {
			pixfmt = v4l2_model_get_support_pixformat(f->index);
			if (pixfmt == NULL)
				return -EINVAL;

			f->index = index;
			f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			strlcpy(f->description, pixfmt->name,
			       sizeof(f->description));
			f->pixelformat = pixfmt->fourcc;
		}
	}
	return 0;
}

int hws_vidioc_g_fmt_vid_cap(struct file *file, void *fh,
				    struct v4l2_format *fmt)
{
	struct hws_video *videodev = video_drvdata(file);
	const framegrabber_pixfmt_t *pixfmt;
	v4l2_model_timing_t *timing;
	u32 width, height;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	pixfmt = framegrabber_g_out_pixelfmt(videodev);
	if (!pixfmt)
		return -EINVAL;

	timing = v4l2_model_get_support_videoformat(
		videodev->current_out_size_index);

	if (!timing)
		return -EINVAL;

	fmt->fmt.pix.width = timing->frame_size.width;
	fmt->fmt.pix.height = timing->frame_size.height;
	fmt->fmt.pix.field = V4L2_FIELD_NONE;
	fmt->fmt.pix.pixelformat = pixfmt->fourcc;
	fmt->fmt.pix.bytesperline =
		(fmt->fmt.pix.width * pixfmt->depth) >> 3;
	fmt->fmt.pix.sizeimage =
		fmt->fmt.pix.height * fmt->fmt.pix.bytesperline;
	fmt->fmt.pix.colorspace = V4L2_COLORSPACE_REC709;

	return 0;

}

int hws_vidioc_try_fmt_vid_cap(struct file *file, void *fh,
				      struct v4l2_format *f)
{
	struct hws_video *videodev = video_drvdata(file);
	v4l2_model_timing_t *timing;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	const framegrabber_pixfmt_t *pfmt;

	pfmt = framegrabber_g_support_pixelfmt_by_fourcc(pix->pixelformat);
	if (!pfmt) {
		v4l2_err(&video->v4l2_dev,
			 "%s: unsupported pixelformat 0x%08x\n",
			 __func__, pix->pixelformat);
		return -EINVAL;
	}

	timing = Get_input_framesizeIndex(pix->width, pix->height);
	if (!timing) {
		v4l2_dbg(1, debug, &video->v4l2_dev,
			 "%s: size %ux%u not supported, falling back\n",
			 __func__, pix->width, pix->height);
		timing = v4l2_model_get_support_videoformat(
			videodev->current_out_size_index);

		if (!timing)
			return -EINVAL;
	}

	pix->field = V4L2_FIELD_NONE;
	pix->width = timing->frame_size.width;
	pix->height = timing->frame_size.height;
	pix->bytesperline = (pix->width * pfmt->depth) >> 3;
	pix->sizeimage = pix->height * pix->bytesperline;
	pix->colorspace = V4L2_COLORSPACE_REC709;
	pix->priv = 0;

	return 0;
}

int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
    struct hws_video       *video   = video_drvdata(file);
    struct hws_pcie_dev    *pdev    = video->dev;
    int                     fmt_idx;
    int                     ret;
    unsigned long           flags;

    /* Find and validate the requested format index */
    fmt_idx = v4l2_get_supported_format_index(f);
    if (fmt_idx < 0)
        return -EINVAL;

    /* Try the format via our helper first */
    ret = hws_vidioc_try_fmt_vid_cap(file, priv, f);
    if (ret)
        return ret;

    /* Apply the new format under lock */
    spin_lock_irqsave(&pdev->formats_lock[video->index], flags);
    video->current_format_index = fmt_idx;
    video->pixfmt               = f->fmt.pix.pixelformat;
    video->current_width        = f->fmt.pix.width;
    video->current_height       = f->fmt.pix.height;
    spin_unlock_irqrestore(&pdev->formats_lock[video->index], flags);

    return 0;
}

int hws_vidioc_g_std(struct file *file, void *priv, v4l2_std_id *tvnorms)
{
	struct hws_video *videodev = video_drvdata(file);
	*tvnorms = videodev->std;
	return 0;
}

int hws_vidioc_s_std(struct file *file, void *priv, v4l2_std_id tvnorms)
{
	struct hws_video *videodev = video_drvdata(file);
	videodev->std = tvnorms;
	v4l2_dbg(1, debug, &video->v4l2_dev,
		 "%s: std set to 0x%llx\n", __func__, std);

	return 0;
}

int hws_vidioc_g_parm(struct file *file, void *fh,
		      struct v4l2_streamparm *setfps)
{
	struct hws_video *videodev = video_drvdata(file);
	v4l2_model_timing_t *timing;

	/* Only video capture streaming is supported */
	if (param->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		v4l2_err(&video->v4l2_dev,
			 "%s: unsupported type %d\n",
			 __func__, param->type);
		return -EINVAL;
	}

	timing = v4l2_model_get_support_videoformat(
		videodev->current_out_size_index);
	if (!timing) {
		v4l2_err(&video->v4l2_dev,
			 "%s: invalid format index %u\n",
			 __func__, video->current_format_index);
		return -EINVAL;
	}

	setfps->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	setfps->parm.capture.timeperframe.numerator = 1000;
	setfps->parm.capture.timeperframe.denominator =
		timing->refresh_rate * 1000;

	return 0;
}

int hws_vidioc_enum_framesizes(struct file *file, void *fh,
				      struct v4l2_frmsizeenum *fsize)
{
	struct hws_video *video = video_drvdata(file);
	const framegrabber_pixfmt_t *pixfmt;
	v4l2_model_timing_t *timing;
	int width = 0, height = 0;
	int frameRate;

	pixfmt = framegrabber_g_support_pixelfmt_by_fourcc(fsize->pixel_format);
	if (!pixfmt) {
		v4l2_err(&video->v4l2_dev,
			 "%s: unsupported pixelformat 0x%08x\n",
			 __func__, fsize->pixel_format);
		return -EINVAL;
	}

	timing = v4l2_model_get_support_videoformat(fsize->index);
	if (!timing) {
		v4l2_err(&video->v4l2_dev,
			 "%s: invalid framesize index %u\n",
			 __func__, fsize->index);
		return -EINVAL;
	}

	width = timing->frame_size.width;
	height = timing->frame_size.height;
	frameRate = timing->refresh_rate;

	if (!width || !height) {
		v4l2_err(&video->v4l2_dev,
			 "%s: zero dimension at index %u\n",
			 __func__, fsize->index);
		return -EINVAL;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->pixel_format = pixfmt->fourcc;
	fsize->discrete.width = width;
	fsize->discrete.height = height;

	return 0;
}

int hws_vidioc_enum_input(struct file *file, void *priv,
				 struct v4l2_input *input)
{
	struct hws_video *video = video_drvdata(file);
	unsigned int      idx   = input->index;

	if (idx != 0) {
		v4l2_err(&video->v4l2_dev,
                 "%s: invalid input index %u\n",
                 __func__, idx);
		return -EINVAL;
	}

    input->type         = V4L2_INPUT_TYPE_CAMERA;
    strlcpy(input->name, KBUILD_MODNAME, sizeof(input->name));
    input->std          = V4L2_STD_NTSC_M;
    input->capabilities = 0;
    input->status       = 0;

    return 0;
}

int hws_vidioc_g_input(struct file *file, void *priv, unsigned int *index)
{

	struct hws_video *video = video_drvdata(file);
    if (*index != 0) {
        v4l2_err(&video->v4l2_dev,
                 "%s: invalid input index %u\n",
                 __func__, *index);
        return -EINVAL;
    }

    *index = 0;
    return 0;
}

int hws_vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
    struct hws_video *video = video_drvdata(file);

    if (index != 0) {
        v4l2_err(&video->v4l2_dev,
                 "%s: invalid input index %u\n",
                 __func__, index);
        return -EINVAL;
    }

    return 0;
}

int vidioc_log_status(struct file *file, void *priv)
{
	return 0;
}

int hws_vidioc_g_ctrl(struct file *file, void *fh,
                             struct v4l2_control *ctrl)
{
    struct hws_video *video = video_drvdata(file);

    if (!ctrl)
        return -EINVAL;

    switch (ctrl->id) {
    case V4L2_CID_BRIGHTNESS:
        ctrl->value = video->curr_brightness;
        break;
    case V4L2_CID_CONTRAST:
        ctrl->value = video->curr_contrast;
        break;
    case V4L2_CID_SATURATION:
        ctrl->value = video->curr_saturation;
        break;
    case V4L2_CID_HUE:
        ctrl->value = video->curr_hue;
        break;
    default:
        v4l2_err(&video->v4l2_dev,
                 "%s: unsupported control id 0x%x\n",
                 __func__, ctrl->id);
        return -EINVAL;
    }

    return 0;
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

int hws_vidioc_queryctrl(struct file *file, void *fh,
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

int hws_vidioc_enum_frameintervals(struct file *file, void *fh,
					  struct v4l2_frmivalenum *fival)
{
	//struct hws_video *videodev = video_drvdata(file);
	int Index;
	int FrameRate;
	v4l2_model_timing_t *pModeTiming;
	Index = fival->index;
	//printk( "%s(CH-%d) FrameIndex =%d \n", __func__,videodev->index,Index);
	if (Index < 0 || Index >= num_framerate_controls) {
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
#if 0
static int hws_vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
   struct hws_video *videodev = video_drvdata(file);
   printk( "%s(ch-%d)\n", __func__,videodev->index);
	//vb2_ioctl_dqbuf
    return vb2_ioctl_dqbuf(file,priv,p);
}
#endif
