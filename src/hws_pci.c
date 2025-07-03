/* SPDX-License-Identifier: GPL-2.0-only */
#include "hws_pci.h"

#include <media/v4l2-ctrls.h>

#include "hws.h"
#include "hws_init.h"
#include "hws_dma.h"
#include "hws_video_pipeline.h"
#include "hws_audio_pipeline.h"
#include "hws_interrupt.h"
#include "hws_video.h"

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

	{ }
};

static void enable_pcie_relaxed_ordering(struct pci_dev *dev)
{
	pcie_capability_set_word(dev, PCI_EXP_DEVCTL, PCI_EXP_DEVCTL_RELAX_EN);
}

static int hws_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
	struct hws_pcie_dev *gdev = NULL;
	int err = 0, ret = -ENODEV;
	int j, i;

	gdev = hws_alloc_dev_instance(pdev);
	gdev->pdev = pdev;

	gdev->dwDeviceID = gdev->pdev->device;
	gdev->dwVendorID = gdev->pdev->vendor;
	dev_info(&pdev->dev, "Device VID=0x%04x, DID=0x%04x\n",
		 pdev->vendor, pdev->device);

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "%s: pci_enable_device failed: %d\n",
			__func__, err);
		goto err_alloc;
	}

	enable_pcie_relaxed_ordering(pdev);
	pci_set_master(pdev);
	ret = probe_scan_for_msi(gdev, pdev);

	if (ret < 0)
		goto disable_msi;

#ifdef CONFIG_ARCH_TI816X
	pcie_set_readrq(pdev, 128);
#endif

	gdev->info.mem[0].internal_addr =
		ioremap(pci_resource_start(pdev, 0), pci_resource_len(pdev, 0));

	gdev->wq = NULL;
	gdev->auwq = NULL;
	gdev->map_bar0_addr = (u32 *)gdev->info.mem[0].internal_addr;

	if (!gdev->info.mem[0].internal_addr)
		goto err_release;

	gdev->info.mem[0].size = pci_resource_len(pdev, 0);
	gdev->info.mem[0].memtype = UIO_MEM_PHYS;

	ret = hws_irq_setup(gdev, pdev);
	if (ret)
		goto err_register;

	pci_set_drvdata(pdev, gdev);
	ReadChipId(gdev);

	for (i = 0; i < MAX_VID_CHANNELS; i++) {
		struct hws_video *vid = &gdev->video[i];
		struct v4l2_ctrl_handler *hdl = &vid->ctrl_handler;

		/* 1. Allocate the per-device control handler (room for 2 controls) */
		v4l2_ctrl_handler_init(hdl, 2);

		/* 2. Create the “5-V detect” boolean (volatile) */
		vid->detect_tx_5v_ctrl = v4l2_ctrl_new_std(
			hdl, &hws_ctrl_ops, V4L2_CID_DV_RX_POWER_PRESENT, 0, 1,
			1, 0);
		/* mark it volatile + read-only */
		vid->detect_tx_5v_ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE |
						 V4L2_CTRL_FLAG_READ_ONLY;

		/* 4. Create the “IT content-type” enum (volatile) */
		vid->content_type = v4l2_ctrl_new_std(
			hdl, &hws_ctrl_ops, V4L2_CID_DV_RX_IT_CONTENT_TYPE,
			V4L2_DV_IT_CONTENT_TYPE_NO_ITC,
			V4L2_DV_IT_CONTENT_TYPE_GRAPHICS, 1,
			V4L2_DV_IT_CONTENT_TYPE_NO_ITC);

		vid->content_type->flags |= V4L2_CTRL_FLAG_VOLATILE |
					    V4L2_CTRL_FLAG_READ_ONLY;

		v4l2_ctrl_handler_setup(hdl);

		/* 5. Bail out cleanly if ctrl creation failed */
		if (hdl->error) {
			ret = hdl->error;
			dev_err(&pdev->dev,
				"ctrl-handler init failed on channel %d: %d\n",
				i, ret);
			goto err_ctrl;
		}
	//gdev->m_nVideoIndex[i] =0;
	gdev->m_nRDVideoIndex[i] = 0;
	gdev->m_bVCapIntDone[i] = 0;
	gdev->m_nVideoBusy[i] = 0;
	gdev->m_bChangeVideoSize[i] = 0;
	gdev->m_nVideoBufferIndex[i] = 0;
	gdev->m_nVideoHalfDone[i] = 0;
	gdev->m_pVideoEvent[i] = 0;
	SetVideoFormatSize(gdev, i, 1920, 1080);
	gdev->m_bVCapStarted[i] = 0;
	gdev->m_bVideoStop[i] = 0;
	gdev->video_data[i] = 0;
	//----------------------
	gdev->m_contrast[i] = 0x80;
	gdev->m_brightness[i] = 0x80;
	gdev->m_saturation[i] = 0x80;
	gdev->m_hue[i] = 0x80;
	gdev->m_dwSWFrameRate[i] = 0;
	gdev->m_pbyVideoBuffer[i] = NULL;
	gdev->m_VideoInfo[i].dwisRuning = 0;
	gdev->m_VideoInfo[i].m_nVideoIndex = 0;
	gdev->m_VideoInfo[i].m_pVideoScalerBuf = NULL;
	gdev->m_VideoInfo[i].m_pVideoYUV2Buf = NULL;
	gdev->m_VideoInfo[i].m_pRotateVideoBuf = NULL;
	for (j = 0; j < MAX_VIDEO_QUEUE; j++) {
		gdev->m_pVCAPStatus[i][j].byLock = MEM_UNLOCK;
		gdev->m_pVCAPStatus[i][j].byField = 0;
		gdev->m_pVCAPStatus[i][j].byPath = 2;
		gdev->m_pVCAPStatus[i][j].dwWidth = 1920;
		gdev->m_pVCAPStatus[i][j].dwHeight = 1080;
		gdev->m_pVCAPStatus[i][j].dwinterlace = 0;
		gdev->m_pVCAPStatus[i][j].dwFrameRate = 60;
		gdev->m_pVCAPStatus[i][j].dwOutWidth = 1920;
		gdev->m_pVCAPStatus[i][j].dwOutHeight = 1080;
		//gdev->m_pVideoData[i][j] = NULL;
		//------------------
		gdev->video_info[i].video_buf[j] = NULL;
		// gdev->video_info[i].m_pVideoBufData1[j] = NULL;
		// gdev->video_info[i].m_pVideoBufData2[j] = NULL;
		// gdev->video_info[i].m_pVideoBufData3[j] = NULL;
		gdev->video_info[i].status[j].lock = MEM_UNLOCK;
		//----------------
		//--------audio
		gdev->m_pAudioEvent[i] = 0;
		gdev->m_bACapStarted[i] = 0;
		gdev->m_bAudioRun[i] = 0;
		gdev->m_bAudioStop[i] = 0;
		gdev->m_nAudioBusy[i] = 0;
		gdev->m_nRDAudioIndex[i] = 0;
		//sema_init(&gdev->sem_video[i],1);
		//spin_lock_init(&gdev->video_lock[i]);
		spin_lock_init(&gdev->videoslock[i]);
		spin_lock_init(&gdev->audiolock[i]);
		//mutex_init(&gdev->video_mutex[i]);
		//init_waitqueue_head(&gdev->wq_video[i]);
		//gdev->wq_flag[i]=0;
		gdev->audio_info[i].running = 0;
		gdev->audio_info[i].index = 0;
		gdev->audio[i].resampled_buf = NULL;
		for (j = 0; j < MAX_AUDIO_QUEUE; j++) {
			gdev->audio_info[i].audio_buf[j] = NULL;
			gdev->audio_info[i].status[j].lock = MEM_UNLOCK;
		}
		//gdev->video[i].v4l2_dev = NULL;
	}
	}
	//---------------------
	tasklet_init(&gdev->dpc_video_tasklet[0], DpcForIsr_Video0,
		     (unsigned long)gdev);
	tasklet_init(&gdev->dpc_video_tasklet[1], DpcForIsr_Video1,
		     (unsigned long)gdev);
	tasklet_init(&gdev->dpc_video_tasklet[2], DpcForIsr_Video2,
		     (unsigned long)gdev);
	tasklet_init(&gdev->dpc_video_tasklet[3], DpcForIsr_Video3,
		     (unsigned long)gdev);

	tasklet_init(&gdev->dpc_audio_tasklet[0], DpcForIsr_Audio0,
		     (unsigned long)gdev);
	tasklet_init(&gdev->dpc_audio_tasklet[1], DpcForIsr_Audio1,
		     (unsigned long)gdev);
	tasklet_init(&gdev->dpc_audio_tasklet[2], DpcForIsr_Audio2,
		     (unsigned long)gdev);
	tasklet_init(&gdev->dpc_audio_tasklet[3], DpcForIsr_Audio3,
		     (unsigned long)gdev);

	//----------------------
	ret = dma_mem_alloc_pool(gdev);
	if (ret != 0) {
		goto err_mem_alloc;
	}

	InitVideoSys(gdev, 0);
	StartKSThread(gdev);

	hws_adapters_init(gdev);
	gdev->wq = create_singlethread_workqueue("hws");
	gdev->auwq = create_singlethread_workqueue("hws-audio");

	if (hws_video_register(gdev))
		goto err_mem_alloc;

	if (hws_audio_register(gdev))
		goto err_mem_alloc;
	return 0;
err_mem_alloc:

	gdev->m_bBufferAllocate = TRUE;
	dma_mem_free_pool(gdev);
	gdev->m_bBufferAllocate = FALSE;
err_ctrl:
	while (--i >= 0)
		v4l2_ctrl_handler_free(&gdev->video[i].ctrl_handler);
err_register:
	iounmap(gdev->info.mem[0].internal_addr);
	hws_free_irqs(gdev);
disable_msi:
	if (gdev->msix_enabled) {
		pci_disable_msix(pdev);
		gdev->msix_enabled = 0;
	} else if (gdev->msi_enabled) {
		pci_disable_msi(pdev);
		gdev->msi_enabled = 0;
	}
err_release:
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	return err;
err_alloc:
	return -1;
}

void hws_remove(struct pci_dev *pdev)
{
	int i;
	struct video_device *vdev;
	struct hws_pcie_dev *dev = (struct hws_pcie_dev *)pci_get_drvdata(pdev);
	//----------------------------
	if (dev->map_bar0_addr == NULL)
		return;
	//StopSys(dev);
	StopDevice(dev);
	/* disable interrupts */
	hws_free_irqs(dev);
	StopKSThread(dev);
	//printk("hws_remove  0\n");
	for (i = 0; i < MAX_VID_CHANNELS; i++) {
		tasklet_kill(&dev->dpc_video_tasklet[i]);
		tasklet_kill(&dev->dpc_audio_tasklet[i]);
	}
	//-------------------------
	//printk("hws_remove  1\n");
	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		if (dev->audio[i].resampled_buf) {
			vfree(dev->audio[i].resampled_buf);
			dev->audio[i].resampled_buf = NULL;
		}
		if (dev->audio[i].card) {
			snd_card_free(dev->audio[i].card);
			dev->audio[i].card = NULL;
		}
	}
	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		vdev = &dev->video[i].vdev;
		video_unregister_device(vdev);
		v4l2_device_unregister(&dev->video[i].v4l2_dev);
		v4l2_ctrl_handler_free(&dev->video[i].ctrl_handler);
	}
	//-----------------
	if (dev->wq) {
		destroy_workqueue(dev->wq);
	}

	if (dev->auwq) {
		destroy_workqueue(dev->auwq);
	}
	dev->wq = NULL;
	dev->auwq = NULL;
	//free_irq(dev->pdev->irq, dev);

	iounmap(dev->info.mem[0].internal_addr);

	//pci_disable_device(pdev);
	if (dev->msix_enabled) {
		pci_disable_msix(pdev);
		dev->msix_enabled = 0;
	} else if (dev->msi_enabled) {
		pci_disable_msi(pdev);
		dev->msi_enabled = 0;
	}
	// FIXME: is this no longer needed because we're using device managed memory?
	kfree(dev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
	//printk("hws_remove  Done\n");
}

static struct hws_pcie_dev *hws_alloc_dev_instance(struct pci_dev *pdev)
{
    struct hws_pcie_dev *lro;

    if (WARN_ON(!pdev))
        return ERR_PTR(-EINVAL);

    lro = devm_kzalloc(&pdev->dev, sizeof(*lro), GFP_KERNEL);
    if (!lro) {
        dev_err(&pdev->dev, "failed to alloc hws_pcie_dev\n");
        return ERR_PTR(-ENOMEM);
    }

    /* no IRQ yet */
    lro->irq_line = -1;

    dev_set_drvdata(&pdev->dev, lro);
    lro->pdev = pdev;


    return lro;
}
//--------------------------------------

/* type = PCI_CAP_ID_MSI or PCI_CAP_ID_MSIX */
int msi_msix_capable(struct pci_dev *dev, int type)
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

int probe_scan_for_msi(struct hws_pcie_dev *lro, struct pci_dev *pdev)
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

void WRITE_REGISTER_ULONG(struct hws_pcie_dev *pdx, u32 RegisterOffset,
			  u32 Value)
{
	//map_bar0_addr[RegisterOffset/4] = Value;
	char *bar0;
	bar0 = (char *)pdx->map_bar0_addr;
	iowrite32(Value, bar0 + RegisterOffset);
	//map_bar0_addr[RegisterOffset/4] = Value;
}

u32 READ_REGISTER_ULONG(struct hws_pcie_dev *pdx, u32 RegisterOffset)
{
	char *bar0;
	bar0 = (char *)pdx->map_bar0_addr;
	//return(map_bar0_addr[RegisterOffset/4]);
	return (ioread32(bar0 + RegisterOffset));
}

static struct pci_driver hws_pci_driver = {
	.name = KBUILD_MODNAME,
	.id_table = hws_pci_table,
	.probe = hws_probe,
	.remove = hws_remove,
};
/*
*/
#ifndef arch_msi_check_device
int arch_msi_check_device(struct pci_dev *dev, int nvec, int type)
{
	return 0;
}
#endif

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
