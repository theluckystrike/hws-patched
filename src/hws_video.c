/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/pci.h>
#include <linux/kernel.h>
#include <media/videobuf2-core.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-dma-contig.h>
#include "hws.h"
#include "hws_reg.h"
#include "hws_dma.h"
#include "hws_video.h"
#include "hws_scaler.h"
#include "hws_interrupt.h"
#include "hws_v4l2_ioctl.h"
#include "hws_video_pipeline.h"

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

//struct hws_pcie_dev  *sys_dvrs_hw_pdx=NULL;
//EXPORT_SYMBOL(sys_dvrs_hw_pdx);
//u32 *map_bar0_addr=NULL; //for sys bar0
//EXPORT_SYMBOL(map_bar0_addr);


//-------------------------------------------


static ssize_t hws_read(struct file *file, char *buf, size_t count,
			loff_t *ppos)
{
	//printk( "%s()\n", __func__);
	return -1;
}

static int hws_open(struct file *file)
{
    struct hws_video *vid = video_drvdata(file);

    /* Hard-fail additional opens while a capture is active */
    if (!v4l2_fh_is_singular_file(file) && vb2_is_busy(vid->queue))
        return -EBUSY;

    return 0;          /* nothing else to count */
}

static int hws_release(struct file *file)
{
    return vb2_fop_release(file);
}

//----------------------------
static const struct v4l2_file_operations hws_fops = {
	.owner = THIS_MODULE,
	.open = hws_open,
	.release = hws_release,
	.read = hws_read,
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
	// .vidioc_g_ctrl = hws_vidioc_g_ctrl,
	// .vidioc_s_ctrl = hws_vidioc_s_ctrl,
	// .vidioc_queryctrl = hws_vidioc_queryctrl,
	.vidioc_enum_input = hws_vidioc_enum_input,
	.vidioc_g_input = hws_vidioc_g_input,
	.vidioc_s_input = hws_vidioc_s_input,
	.vidioc_log_status = vidioc_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_g_parm = hws_vidioc_g_parm,
	.vidioc_s_parm = hws_vidioc_s_parm,
};


static int hws_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			   unsigned int *num_planes, unsigned int sizes[],
			   const struct device *alloc_devs[])
{
	struct hws_video *videodev = q->drv_priv;
	struct hws_pcie_dev *pdx = videodev->dev;
	unsigned long flags;
    size_t size, tmp;

	if (vb2_is_busy(q))
        return -EBUSY;

	// FIXME: Get smart on lock scope, might be better to not spin lock
	// on this scope to do overflow math
	spin_lock_irqsave(&pdx->videoslock[videodev->channel_index], flags);

    if (check_mul_overflow(videodev->current_out_width, videodev->current_out_height, &tmp) ||
        check_mul_overflow(tmp, 2, &size)) {
		spin_unlock_irqrestore(&pdx->videoslock[videodev->channel_index],
				       flags);
        return -EOVERFLOW;
    }
    size = PAGE_ALIGN(size);

	if (*num_planes) {
        int ret = sizes[0] < size ? -EINVAL : 0;
        spin_unlock_irqrestore(
            &pdx->videoslock[videodev->channel_index], flags);
        return ret;
	}
	*num_planes = 1;
	sizes[0] = size;
    alloc_devs[0]    = &pdx->pdev->dev;
	spin_unlock_irqrestore(&pdx->videoslock[videodev->channel_index], flags);
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

	spin_lock_irqsave(&pdx->videoslock[videodev->channel_index], flags);
	size = 2 * videodev->current_out_width *
	       videodev->current_out_height; // 16bit
	if (vb2_plane_size(vb, 0) < size) {
		spin_unlock_irqrestore(&pdx->videoslock[videodev->channel_index],
				       flags);
		return -EINVAL;
	}
	vb2_set_plane_payload(vb, 0, size);
	buf->mem = vb2_plane_vaddr(vb, 0);
	spin_unlock_irqrestore(&pdx->videoslock[videodev->channel_index], flags);
	return 0;
}

static void hws_buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct hws_video *videodev = vb->vb2_queue->drv_priv;
	struct hwsvideo_buffer *buf =
		container_of(vbuf, struct hwsvideo_buffer, vb);
	unsigned long flags;
	struct hws_pcie_dev *pdx = videodev->dev;

	spin_lock_irqsave(&pdx->videoslock[videodev->channel_index], flags);
	list_add_tail(&buf->queue, &videodev->queue);
	spin_unlock_irqrestore(&pdx->videoslock[videodev->channel_index], flags);
}

static int hws_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct hws_video *videodev = q->drv_priv;
	unsigned long flags;
	struct hws_pcie_dev *pdx = videodev->dev;

	atomic_set(&videodev->sequence_number, 0);
	//---------------
	// FIXME: This function sucks
	StartVideoCapture(videodev->dev, videodev->channel_index);

	spin_lock_irqsave(&pdx->videoslock[videodev->channel_index], flags);
	while (!list_empty(&videodev->queue)) {
		struct hwsvideo_buffer *buf = list_entry(
			videodev->queue.next, struct hwsvideo_buffer, queue);
		list_del(&buf->queue);

		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&pdx->videoslock[videodev->channel_index], flags);
	//-----------------------
	return 0;
}

static void hws_stop_streaming(struct vb2_queue *q)
{
	struct hws_video *videodev = q->drv_priv;
	unsigned long flags;
	struct hws_pcie_dev *pdx = videodev->dev;
	//-----------------------------------
	StopVideoCapture(videodev->dev, videodev->index);
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
}

static const struct vb2_ops hwspcie_video_qops = {
	.queue_setup = hws_queue_setup,
	.buf_prepare = hws_buffer_prepare,
	// .buf_finish = hws_buffer_finish,
	.buf_queue = hws_buffer_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.start_streaming = hws_start_streaming,
	.stop_streaming = hws_stop_streaming,
};

int hws_video_register(struct hws_pcie_dev *dev)
{
	int i, err;
	struct video_device *vdev;
	struct vb2_queue *q;

	err = devm_v4l2_device_register(&dev->pdev->dev, &dev->v4l2_dev);
	if (err) {
		dev_err(&dev->pdev->dev,
			"v4l2_device_register failed: %d\n", err);
		return err;
	}

	for (i = 0; i < dev->cur_max_video_ch; i++) {
		struct hws_video *hws = &dev->video[i];

		/* init channel state */
		hws->channel_index      = i;
		hws->parent             = dev;
		hws->pixel_format       = V4L2_PIX_FMT_YUYV;
		hws->current_brightness = BrightnessDefault;
		hws->current_contrast   = ContrastDefault;
		hws->current_saturation = SaturationDefault;
		hws->current_hue        = HueDefault;

		/* initialise locks & lists */
		mutex_init(&hws->state_lock);
		mutex_init(&hws->capture_queue_lock);
		spin_lock_init(&hws->irq_lock);
		INIT_LIST_HEAD(&hws->capture_queue);

		/* setup video_device */
		vdev = devm_video_device_alloc(&dev->pdev->dev, 0);
		if (!vdev) {
		    dev_err(&dev->pdev->dev, "video_device_alloc failed\n");
		    err = -ENOMEM;
		    goto err_unreg_nodes;
		}
		hws->vdev = vdev;

		vdev->v4l2_dev     = &dev->v4l2_dev;
		vdev->fops         = &hws_fops;
		vdev->ioctl_ops    = &hws_ioctl_fops;
		vdev->device_caps  = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
		vdev->lock         = &hws->state_lock;
		vdev->ctrl_handler = &hws->control_handler;
		vdev->vfl_dir      = VFL_DIR_RX;
		vdev->release      = video_device_release_empty;
		video_set_drvdata(vdev, hws);
		video_device_set_name(vdev, "%s-hdmi%d",
				      KBUILD_MODNAME, i);

		/* Setup vb2 queue with designated initializer */
		q = &hws->buffer_queue;

		// FIXME: Figure out if the hw only supports 4 GB RAM
		*q = (struct vb2_queue) {
			.type            = V4L2_BUF_TYPE_VIDEO_CAPTURE,
			.io_modes        = VB2_READ | VB2_MMAP | VB2_USERPTR,
			.gfp_flags       = GFP_DMA32,
			.drv_priv        = hws,
			.buf_struct_size = sizeof(struct hwsvideo_buffer),
			.ops             = &hwspcie_video_qops,
			.mem_ops         = &vb2_vmalloc_memops,
			.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
			.lock            = &hws->capture_queue_lock,
			.dev             = &dev->pdev->dev,
		};

		vdev->queue = q;

		err = vb2_queue_init(q);
		if (err) {
			dev_err(&dev->pdev->dev,
				"vb2_queue_init(ch%d) failed: %d\n", i, err);
			goto err_unreg_nodes;
		}

		INIT_WORK(&hws->video_work, video_data_process);
		err = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
		if (err) {
			dev_err(&dev->pdev->dev,
				"video_register_device(ch%d) failed: %d\n",
				i, err);
			goto err_unreg_nodes;
		}
	}

	return 0;
err_unreg_nodes:
    while (--i >= 0) {
        struct hws_video *hws = &dev->video[i];
        vb2_queue_cleanup(&hws->buffer_queue);
        video_unregister_device(hws->vdev);
    }
    return err;
}
