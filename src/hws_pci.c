// SPDX-License-Identifier: GPL-2.0-only
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/iopoll.h>
#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/pm.h>
#include <linux/freezer.h>
#include <linux/pci_regs.h>
#include <linux/seq_file.h>

#include <media/v4l2-ctrls.h>

#include "hws.h"
#include "hws_audio.h"
#include "hws_reg.h"
#include "hws_video.h"
#include "hws_irq.h"
#include "hws_v4l2_ioctl.h"

#define DRV_NAME "hws"
#define HWS_REG_DEVICE_INFO 0x0000
#define HWS_REG_DEC_MODE 0x0004
#define HWS_BUSY_POLL_DELAY_US 10
#define HWS_BUSY_POLL_TIMEOUT_US 1000000

/* register layout inside HWS_REG_DEVICE_INFO */
#define DEVINFO_VER GENMASK(7, 0)
#define DEVINFO_SUBVER GENMASK(15, 8)
#define DEVINFO_YV12 GENMASK(31, 28)
#define DEVINFO_HWKEY GENMASK(27, 24)
#define DEVINFO_PORTID GENMASK(25, 24) /* low 2 bits of HW-key */

#define MAKE_ENTRY(__vend, __chip, __subven, __subdev, __configptr) \
	{ .vendor = (__vend),                                       \
	  .device = (__chip),                                       \
	  .subvendor = (__subven),                                  \
	  .subdevice = (__subdev),                                  \
	  .driver_data = (unsigned long)(__configptr) }

#define CH_SHIFT 2 /* need 2 bits for 0-3            */

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

static void enable_pcie_relaxed_ordering(struct pci_dev *dev)
{
	pcie_capability_set_word(dev, PCI_EXP_DEVCTL, PCI_EXP_DEVCTL_RELAX_EN);
}

static void hws_configure_hardware_capabilities(struct hws_pcie_dev *hdev)
{
	u16 id = hdev->device_id;

	/* select per-chip channel counts */
	switch (id) {
	case 0x9534:
	case 0x6524:
	case 0x8524:
		hdev->cur_max_video_ch = 4;
		hdev->cur_max_linein_ch = 1;
		break;
	case 0x8532:
		hdev->cur_max_video_ch = 2;
		hdev->cur_max_linein_ch = 1;
		break;
	case 0x8512:
	case 0x6502:
		hdev->cur_max_video_ch = 2;
		hdev->cur_max_linein_ch = 0;
		break;
	case 0x8501:
		hdev->cur_max_video_ch = 1;
		hdev->cur_max_linein_ch = 0;
		break;
	default:
		hdev->cur_max_video_ch = 4;
		hdev->cur_max_linein_ch = 0;
		break;
	}

	/* universal buffer capacity */
	hdev->max_hw_video_buf_sz = MAX_MM_VIDEO_SIZE;

	/* decide hardware-version and program DMA max size if needed */
	if (hdev->device_ver > 121) {
		if (id == 0x8501 && hdev->device_ver == 122) {
			hdev->hw_ver = 0;
		} else {
			hdev->hw_ver = 1;
			u32 dma_max = (u32)(MAX_VIDEO_SCALER_SIZE / 16);

			writel(dma_max, hdev->bar0_base + HWS_REG_DMA_MAX_SIZE);
			/* readback to flush posted MMIO write */
			(void)readl(hdev->bar0_base + HWS_REG_DMA_MAX_SIZE);
		}
	} else {
		hdev->hw_ver = 0;
	}
}

static void hws_stop_device(struct hws_pcie_dev *hws);

static int read_chip_id(struct hws_pcie_dev *hdev)
{
	u32 reg;
	int i;
	/* mirror PCI IDs for later switches */
	hdev->device_id = hdev->pdev->device;
	hdev->vendor_id = hdev->pdev->vendor;

	reg = readl(hdev->bar0_base + HWS_REG_DEVICE_INFO);

	hdev->device_ver = FIELD_GET(DEVINFO_VER, reg);
	hdev->sub_ver = FIELD_GET(DEVINFO_SUBVER, reg);
	hdev->support_yv12 = FIELD_GET(DEVINFO_YV12, reg);
	hdev->port_id = FIELD_GET(DEVINFO_PORTID, reg);

	hdev->max_hw_video_buf_sz = MAX_MM_VIDEO_SIZE;
	hdev->max_channels = 4;
	hdev->buf_allocated = false;
	hdev->main_task = NULL;
	hdev->audio_pkt_size = MAX_DMA_AUDIO_PK_SIZE;
	hdev->start_run = false;
	hdev->pci_lost = 0;

	writel(0x00, hdev->bar0_base + HWS_REG_DEC_MODE);
	writel(0x10, hdev->bar0_base + HWS_REG_DEC_MODE);

	hws_configure_hardware_capabilities(hdev);

	dev_info(&hdev->pdev->dev,
		 "chip detected: ver=%u subver=%u port=%u yv12=%u\n",
		 hdev->device_ver, hdev->sub_ver, hdev->port_id,
		 hdev->support_yv12);

	return 0;
}

static void hws_free_irq_vectors_action(void *data)
{
	pci_free_irq_vectors((struct pci_dev *)data);
}

static int main_ks_thread_handle(void *data)
{
	struct hws_pcie_dev *pdx = data;
	int i;
	bool need_check;

	set_freezable();

	while (!kthread_should_stop()) {
		/* If we’re suspending, don’t touch hardware; just sleep/freeeze */
		if (READ_ONCE(pdx->suspended)) {
			try_to_freeze();
			schedule_timeout_interruptible(msecs_to_jiffies(1000));
			continue;
		}

		need_check = false;

		/* See if any channel is running */
		for (i = 0; i < pdx->max_channels; i++) {
			if (pdx->video[i].cap_active) {
				need_check = true;
				break;
			}
		}

		if (need_check)
			/* avoid MMIO when suspended (guarded above) */
			check_video_format(pdx);

		try_to_freeze(); /* cooperate with freezer each loop */

		/* Sleep 1s or until signaled to wake/stop */
		schedule_timeout_interruptible(msecs_to_jiffies(1000));
	}

	pr_debug("%s: exiting\n", __func__);
	return 0;
}

static void hws_stop_kthread_action(void *data)
{
	struct task_struct *t = data;

	if (!IS_ERR_OR_NULL(t))
		kthread_stop(t);
}

static void hws_dump_irq_regs(struct hws_pcie_dev *hws, const char *tag)
{
	u32 ien = 0, ist = 0, vcap = 0, dec = 0, sys = 0;
	if (!hws || !hws->bar0_base)
		return;
	sys  = readl(hws->bar0_base + HWS_REG_SYS_STATUS);
	dec  = readl(hws->bar0_base + HWS_REG_DEC_MODE);
	ien  = readl(hws->bar0_base + INT_EN_REG_BASE);
	ist  = readl(hws->bar0_base + HWS_REG_INT_STATUS);
	vcap = readl(hws->bar0_base + HWS_REG_VCAP_ENABLE);
	dev_info(&hws->pdev->dev,
		 "[%s] SYS=0x%08x DEC=0x%08x INT_EN=0x%08x INT_STATUS=0x%08x VCAP_EN=0x%08x\n",
		 tag ? tag : "dump", sys, dec, ien, ist, vcap);
}

static void hws_try_enable_irqs(struct hws_pcie_dev *hws, const char *where)
{
	u32 before, after;
	if (!hws || !hws->bar0_base)
		return;

	/* If core not RUN/READY, kick the standard bring-up once. */
	if (!(readl(hws->bar0_base + HWS_REG_SYS_STATUS) & BIT(0))) {
		dev_info(&hws->pdev->dev, "[%s] SYS_STATUS not ready; init video core\n",
			 where ? where : "irqen");
		hws_init_video_sys(hws, true);
	}

	before = readl(hws->bar0_base + INT_EN_REG_BASE);
	writel(0x003FFFFF, hws->bar0_base + INT_EN_REG_BASE);
	/* flush posted write */
	after  = readl(hws->bar0_base + INT_EN_REG_BASE);
	dev_info(&hws->pdev->dev, "[%s] INT_EN before=0x%08x after=0x%08x\n",
		 where ? where : "irqen", before, after);

	if (after != 0x003FFFFF) {
		dev_warn(&hws->pdev->dev,
			 "[%s] WARNING: INT_EN didn’t latch (0x%08x). Register may be write-locked pre-RUN or offset is wrong.\n",
			 where ? where : "irqen", after);
	}
}

static int hws_alloc_seed_buffers(struct hws_pcie_dev *hws)
{
	int ch;
	/* 64 KiB is plenty for a safe dummy; align to 64 for your HW */
	const size_t need = ALIGN(64 * 1024, 64);

	for (ch = 0; ch < hws->cur_max_video_ch; ch++) {
#if defined(CONFIG_HAS_DMA) /* normal on PCIe platforms */
		void *cpu = dma_alloc_coherent(&hws->pdev->dev, need,
					       &hws->scratch_vid[ch].dma,
					       GFP_KERNEL);
#else
		void *cpu = NULL;
#endif
		if (!cpu) {
			dev_warn(&hws->pdev->dev,
				 "scratch: dma_alloc_coherent failed ch=%d\n", ch);
			/* not fatal: free earlier ones and continue without seeding */
			while (--ch >= 0) {
				if (hws->scratch_vid[ch].cpu)
					dma_free_coherent(&hws->pdev->dev,
							  hws->scratch_vid[ch].size,
							  hws->scratch_vid[ch].cpu,
							  hws->scratch_vid[ch].dma);
				hws->scratch_vid[ch].cpu = NULL;
				hws->scratch_vid[ch].size = 0;
			}
			return -ENOMEM;
		}
		hws->scratch_vid[ch].cpu  = cpu;
		hws->scratch_vid[ch].size = need;
	}
	return 0;
}

static void hws_free_seed_buffers(struct hws_pcie_dev *hws)
{
	int ch;
	for (ch = 0; ch < hws->cur_max_video_ch; ch++) {
		if (hws->scratch_vid[ch].cpu) {
			dma_free_coherent(&hws->pdev->dev, hws->scratch_vid[ch].size,
			                  hws->scratch_vid[ch].cpu, hws->scratch_vid[ch].dma);
			hws->scratch_vid[ch].cpu = NULL;
			hws->scratch_vid[ch].size = 0;
		}
	}
}

static void hws_seed_channel(struct hws_pcie_dev *hws, int ch)
{
	dma_addr_t paddr = hws->scratch_vid[ch].dma;
	u32 lo = lower_32_bits(paddr);
	u32 hi = upper_32_bits(paddr);
	u32 pci_addr = lo & PCI_E_BAR_ADD_LOWMASK;

	lo &= PCI_E_BAR_ADD_MASK;

	/* Program 64-bit BAR remap entry for this channel (table @ 0x208 + ch*8) */
	writel_relaxed(hi, hws->bar0_base + PCI_ADDR_TABLE_BASE + 0x208 + ch*8);
	writel_relaxed(lo, hws->bar0_base + PCI_ADDR_TABLE_BASE + 0x208 + ch*8 + PCIE_BARADDROFSIZE);

	/* Program capture engine per-channel base/half */
	writel_relaxed((ch + 1) * PCIEBAR_AXI_BASE + pci_addr,
	               hws->bar0_base + CVBS_IN_BUF_BASE + ch * PCIE_BARADDROFSIZE);

	/* half size: use either the current format’s half or half of scratch */
	{
		u32 half = hws->video[ch].pix.half_size ?
		           hws->video[ch].pix.half_size :
		           (u32)(hws->scratch_vid[ch].size / 2);
		writel_relaxed(half / 16,
		               hws->bar0_base + CVBS_IN_BUF_BASE2 + ch * PCIE_BARADDROFSIZE);
	}

	(void)readl(hws->bar0_base + HWS_REG_INT_STATUS); /* flush posted writes */
}

static void hws_seed_all_channels(struct hws_pcie_dev *hws)
{
	int ch;
	for (ch = 0; ch < hws->cur_max_video_ch; ch++) {
		if (hws->scratch_vid[ch].cpu)
			hws_seed_channel(hws, ch);
	}
}



static int hws_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
	struct hws_pcie_dev *hws;
	int i, ret, nvec, irq;
	unsigned long irqf = 0;
	bool has_msix_cap, has_msi_cap, using_msi;

	/* devres-backed device object */
	hws = devm_kzalloc(&pdev->dev, sizeof(*hws), GFP_KERNEL);
	if (!hws)
		return -ENOMEM;

	hws->pdev = pdev;
	hws->irq = -1;
	hws->suspended = false;
	pci_set_drvdata(pdev, hws);

	/* 1) Enable device + bus mastering */
	ret = pcim_enable_device(pdev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "pcim_enable_device\n");

	pci_set_master(pdev);

	/* 2) Map BAR0 (managed) */
	ret = pcim_iomap_regions(pdev, BIT(0), KBUILD_MODNAME);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "pcim_iomap_regions BAR0\n");
	hws->bar0_base = pcim_iomap_table(pdev)[0];

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "No 32-bit DMA support\n");

	/* 3) Optional PCIe tuning */
	enable_pcie_relaxed_ordering(pdev);
#ifdef CONFIG_ARCH_TI816X
	pcie_set_readrq(pdev, 128);
#endif

	/* 4) Identify chip & capabilities */
	read_chip_id(hws);
	dev_info(&pdev->dev, "Device VID=0x%04x DID=0x%04x\n", pdev->vendor, pdev->device);

	/* 5) Init channels */
	for (i = 0; i < hws->max_channels; i++) {
		ret = hws_video_init_channel(hws, i);
		if (ret) {
			dev_err(&pdev->dev, "video channel init failed (ch=%d)\n", i);
			goto err_unwind_channels;
		}
		ret = hws_audio_init_channel(hws, i);
		if (ret) {
			dev_err(&pdev->dev, "audio channel init failed (ch=%d)\n", i);
			goto err_unwind_channels;
		}
	}

	ret = hws_alloc_seed_buffers(hws);
	if (ret) {
		dev_warn(&pdev->dev, "continuing without scratch seed buffers\n");
		/* not fatal, but you won’t have seeded BARs until first QBUF */
	} else {
		hws_seed_all_channels(hws);
	}

	/* 6) IRQ vectors (1 vector), with robust logging */
	nvec = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_ALL_TYPES | PCI_IRQ_AFFINITY);
	if (nvec < 0) {
		ret = nvec;
		dev_err(&pdev->dev, "pci_alloc_irq_vectors: %d\n", ret);
		goto err_unwind_channels;
	}
	ret = devm_add_action_or_reset(&pdev->dev, hws_free_irq_vectors_action, pdev);
	if (ret) {
		dev_err(&pdev->dev, "devm_add_action free_irq_vectors: %d\n", ret);
		goto err_unwind_channels; /* reset already freed vectors */
	}

	has_msix_cap = !!pci_find_capability(pdev, PCI_CAP_ID_MSIX);
	has_msi_cap  = !!pci_find_capability(pdev, PCI_CAP_ID_MSI);
	using_msi    = pci_dev_msi_enabled(pdev); /* true if MSI (not MSI-X) is actually enabled */

	dev_info(&pdev->dev,
		 "irq vectors=%d (caps: msix=%d msi=%d) using=%s/INTx\n",
		 nvec, has_msix_cap, has_msi_cap,
		 using_msi ? "MSI" : (has_msix_cap ? "MSI-X (likely)" : "none"));

	/* 7) Decide IRQ flags: use IRQF_SHARED only on legacy INTx */
	if (!has_msix_cap && !has_msi_cap && !using_msi) {
		pci_intx(pdev, 1);
		irqf = IRQF_SHARED;
		dev_info(&pdev->dev, "IRQ mode: legacy INTx (shared)\n");
	} else {
		irqf = 0; /* MSI / MSI-X are not shared */
		dev_info(&pdev->dev, "IRQ mode: %s\n", using_msi ? "MSI" : "MSI-X");
	}

    hws_init_video_sys(hws, false);

	irq = pci_irq_vector(pdev, 0);
	dev_info(&pdev->dev, "requesting irq=%d irqf=0x%lx\n", irq, irqf);

	ret = devm_request_irq(&pdev->dev, irq, hws_irq_handler, irqf, dev_name(&pdev->dev), hws);
	if (ret) {
		dev_err(&pdev->dev, "request_irq(%d) failed: %d\n", irq, ret);
		goto err_unwind_channels;
	}
	hws->irq = irq;

	/* 8) Dump, bring up core if needed, try to enable device IRQs, dump again */
	hws_dump_irq_regs(hws, "post-request_irq");
	hws_try_enable_irqs(hws, "probe");
	hws_dump_irq_regs(hws, "after-enable-irqs");
	/* 9) Register V4L2/ALSA */
	ret = hws_video_register(hws);
	if (ret) {
		dev_err(&pdev->dev, "video_register: %d\n", ret);
		goto err_unwind_channels;
	}
	ret = hws_audio_register(hws);
	if (ret) {
		dev_err(&pdev->dev, "audio_register: %d\n", ret);
		hws_video_unregister(hws);
		goto err_unwind_channels;
	}

	/* 10) Background monitor thread (managed) */
	hws->main_task = kthread_run(main_ks_thread_handle, hws, "hws-mon");
	if (IS_ERR(hws->main_task)) {
		ret = PTR_ERR(hws->main_task);
		hws->main_task = NULL;
		dev_err(&pdev->dev, "kthread_run: %d\n", ret);
		goto err_unregister_va;
	}
	ret = devm_add_action_or_reset(&pdev->dev, hws_stop_kthread_action, hws->main_task);
	if (ret) {
		dev_err(&pdev->dev, "devm_add_action kthread_stop: %d\n", ret);
		goto err_unregister_va; /* reset already stopped the thread */
	}

	/* 11) Final: show the line is armed */
	dev_info(&pdev->dev, "irq handler installed on irq=%d\n", irq);

	/* Optional: if your HW requires enabling global IRQs, do it here.
	 * Example (uncomment if appropriate for your device):
	 *   writel(0x3FFFFF, hws->bar0_base + INT_EN_REG_BASE);
	 *   dev_info(&pdev->dev, "INT_EN set to 0x%08x\n",
	 *            readl(hws->bar0_base + INT_EN_REG_BASE));
	 */

	return 0;

err_unregister_va:
	hws_stop_device(hws);
	hws_audio_unregister(hws);
	hws_video_unregister(hws);
err_unwind_channels:
    hws_free_seed_buffers(hws);
	while (--i >= 0) {
		hws_video_cleanup_channel(hws, i);
		hws_audio_cleanup_channel(hws, i, true);
	}
	return ret;
}

static int hws_check_busy(struct hws_pcie_dev *pdx)
{
	void __iomem *reg = pdx->bar0_base + HWS_REG_SYS_STATUS;
	u32 val;
	int ret;

	/* poll until !(val & BUSY_BIT), sleeping HWS_BUSY_POLL_DELAY_US between reads */
	ret = readl_poll_timeout(reg, val, !(val & HWS_SYS_DMA_BUSY_BIT),
				 HWS_BUSY_POLL_DELAY_US,
				 HWS_BUSY_POLL_TIMEOUT_US);
	if (ret) {
		dev_err(&pdx->pdev->dev,
			"SYS_STATUS busy bit never cleared (0x%08x)\n", val);
		return -ETIMEDOUT;
	}

	return 0;
}

static void hws_stop_dsp(struct hws_pcie_dev *hws)
{
	u32 status;

	/* Read the decoder mode/status register */
	status = readl(hws->bar0_base + HWS_REG_DEC_MODE);
	dev_dbg(&hws->pdev->dev, "%s: status=0x%08x\n", __func__, status);

	/* If the device looks unplugged/stuck, bail out */
	if (status == 0xFFFFFFFF)
		return;

	/* Tell the DSP to stop */
	writel(0x10, hws->bar0_base + HWS_REG_DEC_MODE);

	if (hws_check_busy(hws))
		dev_warn(&hws->pdev->dev, "DSP busy timeout on stop\n");
	/* Disable video capture engine in the DSP */
	writel(0x0, hws->bar0_base + HWS_REG_VCAP_ENABLE);
}

/* Publish stop so ISR/BH won’t touch ALSA/VB2 anymore. */
static void hws_publish_stop_flags(struct hws_pcie_dev *hws)
{
	unsigned int i;

	for (i = 0; i < hws->cur_max_video_ch; ++i) {
		struct hws_video *v = &hws->video[i];

		WRITE_ONCE(v->cap_active,     false);
		WRITE_ONCE(v->stop_requested, true);
	}

	for (i = 0; i < hws->cur_max_linein_ch; ++i) {
		struct hws_audio *a = &hws->audio[i];

		WRITE_ONCE(a->stream_running, false);
		WRITE_ONCE(a->cap_active,     false);
		WRITE_ONCE(a->stop_requested, true);
	}

	smp_wmb(); /* make flags visible before we touch MMIO/queues */
}

/* Drain engines + ISR/BH after flags are published. */
static void hws_drain_after_stop(struct hws_pcie_dev *hws)
{
	u32 ackmask = 0;
	unsigned int i;

	/* Mask device enables: no new DMA starts. */
	writel(0x0, hws->bar0_base + HWS_REG_VCAP_ENABLE);
	writel(0x0, hws->bar0_base + HWS_REG_ACAP_ENABLE);
	(void)readl(hws->bar0_base + HWS_REG_INT_STATUS); /* flush */

	/* Let any in-flight DMAs finish (best-effort). */
	(void)hws_check_busy(hws);

	/* Kill video tasklets to avoid late BH completions. */
	for (i = 0; i < hws->cur_max_video_ch; ++i)
		tasklet_kill(&hws->video[i].bh_tasklet);

	/* Ack any latched VDONE/ADONE. */
	for (i = 0; i < hws->cur_max_video_ch; ++i)
		ackmask |= HWS_INT_VDONE_BIT(i);
	for (i = 0; i < hws->cur_max_linein_ch; ++i)
		ackmask |= HWS_INT_ADONE_BIT(i);
	if (ackmask) {
		writel(ackmask, hws->bar0_base + HWS_REG_INT_ACK);
		(void)readl(hws->bar0_base + HWS_REG_INT_STATUS);
	}

	/* Ensure no hard IRQ is still running. */
	if (hws->irq >= 0)
		synchronize_irq(hws->irq);
}

static void hws_stop_device(struct hws_pcie_dev *hws)
{
	u32 status = readl(hws->bar0_base + HWS_REG_PIPE_BASE(0));

	dev_dbg(&hws->pdev->dev, "%s: status=0x%08x\n", __func__, status);
	if (status == 0xFFFFFFFF) {
		hws->pci_lost = true;
		goto out;
	}

	/* Make ISR/BH a no-op, then drain engines/IRQ. */
	hws_publish_stop_flags(hws);
	hws_drain_after_stop(hws);

	/* 1) Stop the on-board DSP */
	hws_stop_dsp(hws);

out:
	hws->start_run = false;
	dev_dbg(&hws->pdev->dev, "%s: complete\n", __func__);
}

static void hws_remove(struct pci_dev *pdev)
{
	struct hws_pcie_dev *hws = pci_get_drvdata(pdev);
	int i;

	if (!hws)
		return;

	/* Stop hardware / capture cleanly (your helper) */
	hws_stop_device(hws);

	/* Unregister subsystems you registered */
	hws_audio_unregister(hws);
	hws_video_unregister(hws);

	/* Per-channel teardown */
	for (i = 0; i < hws->max_channels; i++) {
		hws_video_cleanup_channel(hws, i);
		hws_audio_cleanup_channel(hws, i, true);
	}
	/* kthread is stopped by the devm action you added in probe */
}

#ifdef CONFIG_PM_SLEEP
static int hws_pm_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct hws_pcie_dev *hws = pci_get_drvdata(pdev);

	/* Block monitor thread / any hot path from MMIO */
	WRITE_ONCE(hws->suspended, true);
	if (hws->irq >= 0)
		disable_irq(hws->irq);

	/* Gracefully quiesce userspace I/O first */
	hws_audio_pm_suspend_all(hws);          /* ALSA: stop substreams */
	hws_video_pm_suspend(hws);               /* VB2: streamoff + drain + discard */

	/* Quiesce hardware (DSP/engines) */
	hws_stop_device(hws);

	pci_save_state(pdev);
	pci_disable_device(pdev);
	pci_set_power_state(pdev, PCI_D3hot);

	return 0;
}

static int hws_pm_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct hws_pcie_dev *hws = pci_get_drvdata(pdev);
	int ret;

	/* Back to D0 and re-enable the function */
	pci_set_power_state(pdev, PCI_D0);

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(dev, "pci_enable_device: %d\n", ret);
		return ret;
	}
	pci_restore_state(pdev);
	pci_set_master(pdev);

	/* Reapply any PCIe tuning lost across D3 */
	enable_pcie_relaxed_ordering(pdev);

	/* Reinitialize chip-side capabilities / registers */
	read_chip_id(hws);

	/* IRQs can be re-enabled now that MMIO is sane */
	if (hws->irq >= 0)
		enable_irq(hws->irq);

	WRITE_ONCE(hws->suspended, false);

	/* vb2: nothing mandatory; userspace will STREAMON again when ready */
	hws_video_pm_resume(hws);

	return 0;
}

static SIMPLE_DEV_PM_OPS(hws_pm_ops, hws_pm_suspend, hws_pm_resume);
# define HWS_PM_OPS (&hws_pm_ops)
#else
# define HWS_PM_OPS NULL
#endif

static struct pci_driver hws_pci_driver = {
	.name = KBUILD_MODNAME,
	.id_table = hws_pci_table,
	.probe = hws_probe,
	.remove = hws_remove,
	.driver = {
		.pm = HWS_PM_OPS,
	},
};

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

MODULE_DESCRIPTION(DRV_NAME);
MODULE_AUTHOR("Ben Hoff <hoff.benjamin.k@gmail.com>");
MODULE_AUTHOR("Sales <sales@avmatrix.com>");
MODULE_LICENSE("GPL");
