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
#include "hws_dma.h"
#include "hws_scaler.h"
#include "hws_interrupt.h"

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








void StopKSThread(struct hws_pcie_dev *pdx)
{
	if (pdx->mMain_tsk) {
		kthread_stop(pdx->mMain_tsk);
	}
}

//----------------------------





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
void StartKSThread(struct hws_pcie_dev *pdx)
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



//-------------------------------------


MODULE_DEVICE_TABLE(pci, hws_pci_table);

static int __init pcie_hws_init(void)
{
        return pci_register_driver(&hws_pci_driver);
}

static void __exit pcie_hws_exit(void)
{
        pci_unregister_driver(&hws_pci_driver);
}


module_init(pcie_hws_init);
module_exit(pcie_hws_exit);

MODULE_DESCRIPTION("HWS driver");
MODULE_AUTHOR("Sales <sales@avmatrix.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
