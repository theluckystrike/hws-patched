/* SPDX-License-Identifier: GPL-2.0-only */
#include "hws_dma.h"
#include "hws_pci.h"


static void hws_dma_mem_free(struct hws_pcie_dev *hws)
{
    struct device *dev = &hws->pdev->dev;
    int i;

    if (WARN_ON(!hws))
        return;

    /* Nothing to do if we never allocated */
    if (!hws->buf_allocated)
        return;

    /* Free each per-channel DMA block */
    for (i = 0; i < hws->max_channels; ++i) {
        if (hws->video[i].buf_virt) {
            dma_free_coherent(dev,
                              hws->video[i].buf_size_bytes,
                              hws->video[i].buf_virt,
                              hws->video[i].buf_phys_addr);
            hws->video[i].buf_virt = NULL;

            /*
             * Audio tail shares the same block, so just clear the ptr
             * (no separate free_coherent required).
             */
            hws->audio[i].buf_virt = NULL;
        }
    }

    /* CPU‐only buffers (devm_ allocations) will auto-free on driver detach */

    hws->buf_allocated = false;
    dev_dbg(dev, "DMA/CPU buffer pool freed\n");
}


int hws_dma_mem_alloc(struct hws_pcie_dev *hws)
{
	struct device *dev = &hws->pdev->dev;
	size_t vbuf_sz      = hws->max_hw_video_buf_sz;
	size_t abuf_sz      = hws->audio_pkt_size;
	int i, k, ret = 0;

	if (WARN_ON(!hws))
		return -EINVAL;

	/* If we were called twice, tear existing pool down first */
	if (hws->buf_allocated) {
		hws_dma_mem_free(hws);
		hws->buf_allocated = false;
	}

	/* ─────────────────────────────────────────────── */
	/* 1) Per-channel VIDEO DMA buffers                */
	/* ─────────────────────────────────────────────── */
	for (i = 0; i < hws->max_channels; i++) {
		dma_addr_t phys;

		hws->video[i].buf_virt =
			dma_alloc_coherent(dev, vbuf_sz,
					   &hws->video[i].buf_phys_addr,
					   GFP_KERNEL);
		if (!hws->video[i].buf_virt) {
			dev_err(dev, "dma_alloc_coherent(video %d) failed\n", i);
			ret = -ENOMEM;
			goto err;
		}

		hws->video[i].buf_size_bytes = vbuf_sz;
		/* A convenient high-watermark can be half the buffer */
		hws->video[i].buf_high_wmark = vbuf_sz / 2;

		/* Carve the last AUDIO packet out of the same DMA block */
		phys = hws->video[i].buf_phys_addr + (vbuf_sz - abuf_sz);
		hws->audio[i].buf_phys_addr = phys;
		hws->audio[i].buf_virt      =
			hws->video[i].buf_virt + (vbuf_sz - abuf_sz);
		hws->audio[i].buf_size_bytes  = abuf_sz;
		hws->audio[i].buf_high_wmark  = abuf_sz / 2;
	}

	/* ─────────────────────────────────────────────── */
	/* 2) CPU-only scratch buffers for each video chan */
	/* ─────────────────────────────────────────────── */
	for (i = 0; i < hws->cur_max_video_ch; i++) {
#define SCRATCH_SZ (MAX_VIDEO_HW_W * MAX_VIDEO_HW_H * 2)
		u8 **dst[] = {
			&hws->video[i].chan_info.scaler_buf,
			&hws->video[i].chan_info.yuv2_buf,
			&hws->video[i].chan_info.rotate_buf,
		};

		for (k = 0; k < ARRAY_SIZE(dst); k++) {
			*dst[k] = devm_kzalloc(dev, SCRATCH_SZ, GFP_KERNEL);
			if (!*dst[k]) {
				dev_err(dev, "scratch buf %d/%d OOM\n", i, k);
				ret = -ENOMEM;
				goto err;
			}
		}
#undef SCRATCH_SZ

		/* Per-frame queue – video payload copies used in ISR */
		for (k = 0; k < MAX_VIDEO_QUEUE; k++) {
			hws->video[i].chan_info.queue[k] =
				devm_kmalloc(dev, vbuf_sz, GFP_KERNEL);
			if (!hws->video[i].chan_info.queue[k]) {
				dev_err(dev, "queue[%d][%d] OOM\n", i, k);
				ret = -ENOMEM;
				goto err;
			}
		}
	}

	/* ─────────────────────────────────────────────── */
	/* 3) AUDIO per-channel SW ring-buffers            */
	/* ─────────────────────────────────────────────── */
	for (i = 0; i < hws->cur_max_video_ch; i++) {
		for (k = 0; k < MAX_AUDIO_QUEUE; k++) {
			hws->audio[i].data_buf =
				devm_kmalloc(dev, abuf_sz, GFP_KERNEL);
			if (!hws->audio[i].data_buf) {
				dev_err(dev, "audio buf %d/%d OOM\n", i, k);
				ret = -ENOMEM;
				goto err;
			}
			hws->audio[i].data_area = hws->audio[i].data_buf;
			/* Runtime lock flag initialisation */
			hws->audio[i].chan_info.status[k].byLock = MEM_UNLOCK;
		}
	}

	hws->buf_allocated = true;
	dev_dbg(dev, "DMA/CPU buffer pool allocated\n");
	return 0;

err:
	hws_dma_mem_free(hws);
	return ret;
}

void set_dma_address(struct hws_pcie_dev *pdx)
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
	DWORD halfframe_length = 0;
	//DWORD m_Valude;
	PhyAddr_A_High = 0;
	PhyAddr_A_Low = 0;
	PCI_Addr = 0;

	//------------------------------------------ // re write dma register

	Addrmsk = PCI_E_BAR_ADD_MASK;
	AddrLowmsk = PCI_E_BAR_ADD_LOWMASK;

	cnt = 0x208; // Table address
	for (i = 0; i < pdx->m_nMaxChl; i++) {
		//printk("[MV] pdx->m_pbyVideoBuffer[%d]=%x\n", i, pdx->m_pbyVideoBuffer[i]);
		if (pdx->m_pbyVideoBuffer[i]) {
			PhyAddr_A_Low = pdx->m_dwVideoBuffer[i];
			PhyAddr_A_High = pdx->m_dwVideoHighBuffer[i];

			PCI_Addr = (PhyAddr_A_Low & AddrLowmsk);
			PhyAddr_A_Low = (PhyAddr_A_Low & Addrmsk);

			//printk("[MV]1-pdx->m_dwVideoBuffer[%d]-%X\n",i,pdx->m_dwVideoBuffer[i]);
			//--------------------------------------------------------------
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

			//--------------------------
			WRITE_REGISTER_ULONG(pdx, (CBVS_IN_BUF_BASE + (i * PCIE_BARADDROFSIZE)),
					     ((i + 1) * PCIEBAR_AXI_BASE) +
					PCI_Addr); //Buffer 1 address
			halfframe_length = pdx->m_format[i].HLAF_SIZE / 16;
			WRITE_REGISTER_ULONG(pdx,
                    (CBVS_IN_BUF_BASE2 + (i * PCIE_BARADDROFSIZE)),
					     halfframe_length); //Buffer 1 address

			m_ReadTmp = READ_REGISTER_ULONG(pdx, (CBVS_IN_BUF_BASE + (i * PCIE_BARADDROFSIZE)));
			m_ReadTmp2 = READ_REGISTER_ULONG(pdx, (CBVS_IN_BUF_BASE2 + (i * PCIE_BARADDROFSIZE)));

			//---------------------------
		}
		cnt += 8;
		if (pdx->m_pbyAudioBuffer[i]) {
			PhyAddr_A_Low = pdx->m_dwAudioBuffer[i];
			PhyAddr_A_High = pdx->m_dwAudioBufferHigh[i];
			PCI_Addr = (PhyAddr_A_Low & AddrLowmsk);
			PhyAddr_A_Low = (PhyAddr_A_Low & Addrmsk);

			WRITE_REGISTER_ULONG(pdx,
					     (CBVS_IN_BUF_BASE +
					      ((8 + i) * PCIE_BARADDROFSIZE)),
					     ((i + 1) * PCIEBAR_AXI_BASE +
					      PCI_Addr)); //Buffer 1 address
			m_ReadTmp = READ_REGISTER_ULONG(pdx, (CBVS_IN_BUF_BASE +
				      ((8 + i) * PCIE_BARADDROFSIZE)));
		}
	}
	WRITE_REGISTER_ULONG(pdx, INT_EN_REG_BASE,
			     0x3ffff); //enable PCI Interruput
}
