/* SPDX-License-Identifier: GPL-2.0-only */
#include "hws_interrupt.h"
#include "hws_pci.h"
#include "hws_reg.h"
#include "hws_video_pipeline.h"
#include "hws_audio_pipeline.h"

/**
 * hws_set_queue() - Fetch next video buffer and push it downstream
 * @hws:    driver instance
 * @ch:     video channel index
 *
 * Returns 0 on success or a negative errno on failure.
 */
static int hws_set_queue(struct hws_pcie_dev *hws, unsigned int ch)
{
    int ret = -ENODEV;

    /* Not yet started? */
    if (!hws->running)
        return -EINVAL;

    /* Channel not streaming?  Reset any stop flag and bail. */
    if (!hws->vcap_started[ch]) {
        if (hws->video_stop[ch]) {
            hws->video_stop[ch] = false;
            dev_dbg(&hws->pdev->dev,
                    "hws_set_queue[%u]: exit-stop cleared\n", ch);
        }
        return -ENODEV;
    }

    /* On older hardware, bump our software frame counter */
    if (hws->device_hw_version == 0)
        hws->sw_frame_rate[ch]++;

    /* Mark the channel busy while copying */
    hws->video_busy[ch] = true;

    /* Pull data from the device into the vb2 stream */
    ret = hws_memcopy_video_to_stream(hws, ch);

    /* Done copying */
    hws->video_busy[ch] = false;

    return ret;
}

//-----------------------------
/* Interrupt handler. Read/modify/write the command register to disable
 * the interrupt. */
//static irqreturn_t irqhandler(int irq, struct uio_info *info)
static irqreturn_t irqhandler(int irq, void *info)
{
    struct hws_pcie_dev *pdx = info;
    u32 sys_status = READ_REGISTER_ULONG(pdx, HWS_REG_SYS_STATUS);

    /* No DMA busy or card gone: exit early */
    if ((sys_status & HWS_SYS_DMA_BUSY_BIT) == 0 || sys_status == 0xFFFFFFFF)
        return IRQ_NONE;

    /* Read interrupt status bits */
    u32 int_state = READ_REGISTER_ULONG(pdx, HWS_REG_INT_STATUS);
    if (!int_state)
        return IRQ_NONE;  /* spurious interrupt */

    u32 ack_mask = 0;

    /* Loop until all pending bits are serviced (max 100 iterations) */
    for (u32 cnt = 0; int_state && cnt < 100; ++cnt) {
        /* ── Video channels 0–3 ───────────────────────────────────────── */
        for (int ch = 0; ch < 4; ++ch) {
            u32 vbit = HWS_INT_VDONE_BIT(ch);
            if (!(int_state & vbit))
                continue;

            /* Mark this channel’s capture-done flag */
            pdx->m_bVCapIntDone[ch] = 1;
            ack_mask |= vbit;

            if (pdx->video[ch].dma_busy == 0) {
                /* Read which half of the ring the DMA is writing to */
                u32 toggle = READ_REGISTER_ULONG(pdx, HWS_REG_VBUF_TOGGLE(ch)) & 0x01;

                if (pdx->video_data[ch] != toggle) {
                    pdx->video_data[ch]        = toggle;
                    pdx->audio[ch].wr_idx = toggle;
                    tasklet_schedule(&pdx->dpc_video_tasklet[ch]);
                } else {
                    pdx->m_nVideoHalfDone[ch] = 0;
                }
            }
        }

        /* ── Audio channels 0–3 ───────────────────────────────────────── */
        for (int ch = 0; ch < 4; ++ch) {
            u32 abit = HWS_INT_ADONE_BIT(ch);
            if (!(int_state & abit))
                continue;

            ack_mask |= abit;

            if (pdx->audio[ch].dma_busy == 0) {
                /* Read which half of the audio ring is active */
                u32 toggle = READ_REGISTER_ULONG(pdx, HWS_REG_ABUF_TOGGLE(ch)) & 0x01;

                pdx->audio[ch].wr_idx = toggle;
                pdx->audio_data[ch]         = toggle;
                tasklet_schedule(&pdx->dpc_audio_tasklet[ch]);
            }
        }

        /* Acknowledge (clear) all bits we just handled */
        WRITE_REGISTER_ULONG(pdx, HWS_REG_INT_ACK, ack_mask);

        /* Immediately clear ack_mask to avoid re-acknowledging stale bits */
        ack_mask = 0;

        /* Re‐read in case new interrupt bits popped while processing */
        int_state = READ_REGISTER_ULONG(pdx, HWS_REG_INT_STATUS);
    }

    return IRQ_HANDLED;
}


void hws_free_irqs(struct hws_pcie_dev *lro)
{
	//int i;

	//BUG_ON(!lro);

	if (lro->irq_line != -1) {
		//printk("Releasing IRQ#%d\n", lro->irq_line);
		free_irq(lro->irq_line, lro);
	}
}

static void hws_dpc_audio(unsigned long data)
{
        struct hws_pcie_dev *hws;
        u32                  ch;

        unpack_dev_ch(data, &hws, &ch);
        hws_set_audio_queue(hws, ch);          /* unchanged business logic */
}

static void hws_dpc_video(unsigned long data)
{
        struct hws_pcie_dev *hws;
        u32                  ch;
        int                  ret;

        unpack_dev_ch(data, &hws, &ch);

        ret = hws_set_queue(hws, ch);
        if (ret || !hws->video[ch].cap_active)
                return;

        if (hws->m_bVCapIntDone[ch] && hws->video[ch].irq_event) {
                hws->m_bVCapIntDone[ch] = false;

                if (!hws->m_bChangeVideoSize[ch]) {
                        queue_work(hws->video_wq, &hws->video[ch].videowork);
                } else {
                        hws->m_bChangeVideoSize[ch] = 0;
                }
        }
}



