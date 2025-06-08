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
#if 0
static int hws_vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
   struct hws_video *videodev = video_drvdata(file);
   printk( "%s(ch-%d)\n", __func__,videodev->index);
	//vb2_ioctl_dqbuf
    return vb2_ioctl_dqbuf(file,priv,p);
}
#endif
