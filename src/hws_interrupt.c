/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/compiler.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include "hws_interrupt.h"
#include "hws_reg.h"
#include "hws_video_pipeline.h"
#include "hws_audio_pipeline.h"


#define MAX_INT_LOOPS 100

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
    if (!hws->start_run)
        return -EINVAL;

    /* Channel not streaming?  Reset any stop flag and bail. */
    if (!hws->video[ch].cap_active) {

        if (atomic_read(&hws->video[ch].stop_requested)) {
	    atomic_set(&hws->video[ch].stop_requested, 0);
            dev_dbg(&hws->pdev->dev,
                    "hws_set_queue[%u]: exit-stop cleared\n", ch);
        }
        return -ENODEV;
    }

    /* On older hardware, bump our software frame counter */
    // FIXME: check if this doesn't need to be `device_ver` via m_DeviceHW_Version instead of hw_ver via m_DeviceHW_Version
    if (hws->hw_ver== 0)
	    // FIXME: `m_dwSWFrameRate`
        hws->video[ch].sw_fps++;

    /* Mark the channel busy while copying */
    atomic_set(&hws->audio[ch].dma_busy, 1);

    /* Pull data from the device into the vb2 stream */
    ret = hws_memcopy_video_to_stream(hws, ch);

    /* Done copying */
    atomic_set(&hws->audio[ch].dma_busy, 0);

    return ret;
}


irqreturn_t irqhandler(int irq, void *info)
{
    struct hws_pcie_dev *pdx = info;
    u32 sys_status = readl(pdx->bar0_base + HWS_REG_SYS_STATUS);

    /* No DMA busy or card gone: exit early */
    if ((sys_status & HWS_SYS_DMA_BUSY_BIT) == 0 || sys_status == 0xFFFFFFFF)
        return IRQ_NONE;

    /* Read interrupt status bits */
    u32 int_state = readl(pdx->bar0_base + HWS_REG_INT_STATUS);
    if (!int_state)
        return IRQ_NONE;  /* spurious interrupt */

    u32 ack_mask = 0;

    /* Loop until all pending bits are serviced (max 100 iterations) */
    for (u32 cnt = 0; int_state && cnt < MAX_INT_LOOPS; ++cnt) {
        /* ── Video channels 0–3 ───────────────────────────────────────── */
        for (int ch = 0; ch < 4; ++ch) {
            u32 vbit = HWS_INT_VDONE_BIT(ch);
            if (!(int_state & vbit))
                continue;

            /* Mark this channel’s capture-done flag */
            pdx->video[ch].irq_done_flag = 1;
            ack_mask |= vbit;

            // FIXME: migrate to WRITE_ONCE / READ_ONCE
            if (!atomic_read(&pdx->video[ch].dma_busy)) {
                /* Read which half of the ring the DMA is writing to */
                u32 toggle = readl(pdx->bar0_base + HWS_REG_VBUF_TOGGLE(ch)) & 0x01;
                dma_rmb();   /* make sure DMA writes are visible before we look at data */
                u8 last = READ_ONCE(pdx->video[ch].last_buf_half_toggle);

                if (last != toggle) {
                        WRITE_ONCE(pdx->video[ch].last_buf_half_toggle, toggle);
                        tasklet_schedule(&pdx->video[ch].video_bottom_half);
                } else {
                        pdx->video[ch].half_done_cnt = 0;
                }
            }
        }

        /* ── Audio channels 0–3 ───────────────────────────────────────── */
        for (int ch = 0; ch < 4; ++ch) {
            u32 abit = HWS_INT_ADONE_BIT(ch);
            if (!(int_state & abit))
                continue;

            ack_mask |= abit;

            if (!atomic_read(&pdx->audio[ch].dma_busy)) {
                /* Read which half of the audio ring is active */
                u32 toggle = readl(pdx->bar0_base + HWS_REG_ABUF_TOGGLE(ch)) & 0x01;

                pdx->audio[ch].wr_idx = toggle;
                pdx->audio_data[ch]         = toggle;
                tasklet_schedule(&pdx->audio[ch].audio_bottom_half);
            }
        }

        /* Acknowledge (clear) all bits we just handled */
	writel(ack_mask, pdx->bar0_base + HWS_REG_INT_ACK);

        /* Immediately clear ack_mask to avoid re-acknowledging stale bits */
        ack_mask = 0;

        /* Re‐read in case new interrupt bits popped while processing */
	int_state = readl(pdx->bar0_base + HWS_REG_INT_STATUS);
    }

    return IRQ_HANDLED;
}

int hws_set_audio_queue(struct hws_pcie_dev *hws, unsigned int ch)
{
    int ret = 0;

    dev_dbg(&hws->pdev->dev,
            "set audio queue on channel %u\n", ch);

    /* no DMA until capture has been enabled */
    if (!hws->audio[ch].cap_active)
        return -ENODEV;

    /* if stream is stopped, clear stop flag and exit */
    if (!hws->audio[ch].stream_running) {
        if (atomic_read(&hws->audio[ch].stop_requested)) {
	    atomic_set(&hws->audio[ch].stop_requested, 0);
            dev_dbg(&hws->pdev->dev,
                    "cleared stop flag on channel %u\n", ch);
        }
	atomic_set(&hws->audio[ch].dma_busy, 0);
        return 0;
    }

    /* mark DMA busy while copying */
    atomic_set(&hws->audio[ch].dma_busy, 1);
    ret = hws_copy_audio_to_stream(hws, ch);
    atomic_set(&hws->audio[ch].dma_busy, 0);

    return ret;
}

static inline void unpack_dev_ch(unsigned long data,
                                 struct hws_pcie_dev **dev, u32 *ch)
{
        *ch  = data & CH_MASK;
        *dev = (struct hws_pcie_dev *)(data & ~CH_MASK);
}

void hws_dpc_audio(unsigned long data)
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

        if (hws->video[ch].irq_done_flag && hws->video[ch].irq_event) {
                hws->video[ch].irq_done_flag  = false;

                if (!hws->video[ch].size_changed_flag) {
                        queue_work(hws->video_wq, &hws->video[ch].video_work);
                } else {
                        hws->video[ch].size_changed_flag = 0;
                }
        }
}



