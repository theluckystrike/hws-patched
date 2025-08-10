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

static inline void hws_audio_program_period(struct hws_pcie_dev *hws, u32 ch, u32 period)
{
    struct hws_audio_chan *a = &hws->audio[ch];
    struct snd_pcm_substream *ss = READ_ONCE(a->substream);
    struct snd_pcm_runtime  *rt;

    if (!ss) return;
    rt = ss->runtime;

    /* ALSA exposes the base DMA address of the whole PCM buffer */
    dma_addr_t base = rt->dma_addr;
    dma_addr_t paddr = base + (dma_addr_t)(period % a->periods) * a->period_bytes;

    /* Your device wants: (AXI base for ch) + low bits of phys addr */
    u32 lo = lower_32_bits(paddr);
    u32 pci_off = lo & ADDR_LOW_MASK;

    writel((ch + 1) * PCIEBAR_AXI_BASE + pci_off,
           hws->bar0_base + CBVS_IN_BUF_BASE + (8 + ch) * PCIE_BARADDROFSIZE);
    /* If you also need to program a length register, do it here using a->period_bytes */
}


static int hws_arm_next(struct hws_pcie_dev *hws, u32 ch)
{
    struct hws_video *v = &hws->video[ch];
    unsigned long flags;
    struct hwsvideo_buffer *buf;

    if (unlikely(READ_ONCE(v->stop_requested) || !READ_ONCE(v->cap_active)))
        return -ECANCELED;

    spin_lock_irqsave(&hws->videoslock[ch], flags);
    if (list_empty(&v->capture_queue)) {
        spin_unlock_irqrestore(&hws->videoslock[ch], flags);
        return -EAGAIN;
    }
    buf = list_first_entry(&v->capture_queue, struct hwsvideo_buffer, queue);
    list_del(&buf->queue);
    v->active = buf;
    spin_unlock_irqrestore(&hws->videoslock[ch], flags);

    /* Program HW with this VB2 buffer’s DMA address */
    hws_program_video_from_vb2(hws, ch, &buf->vb.vb2_buf);
    return 0;
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

            if (!READ_ONCE(pdx->video[ch].dma_busy)) {
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

            if (!READ_ONCE(&pdx->audio[ch].dma_busy)) {
                /* Read which half of the audio ring is active */
		/* Which half is active? (device-specific: 0/1 toggle) */
		u32 toggle = readl(pdx->bar0_base + HWS_REG_ABUF_TOGGLE(ch)) & 0x01;
		dma_rmb();  /* make device writes visible */

		/* Tell ALSA a period elapsed */
		if (pdx->audio[ch].substream)
			snd_pcm_period_elapsed(pdx->audio[ch].substream);

		/* Program DMA base for the period the HW will fill next.
		    Many devices toggle: if HW reports 'toggle', the *next* to program is 'toggle'. */
		hws_audio_program_period(pdx, ch, toggle);
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

/* called by the IRQ as tasklet_schedule(&v->video_bottom_half) */
static void hws_dpc_video(unsigned long data)
{
    struct hws_pcie_dev *hws;
    u32 ch;
    struct hws_video *v;
    struct hwsvideo_buffer *done;
    int ret;

    unpack_dev_ch(data, &hws, &ch);
    v = &hws->video[ch];

    /* if stopping or not active, do nothing */
    if (unlikely(READ_ONCE(v->stop_requested) || !READ_ONCE(v->cap_active)))
        return;

    /* 1) Complete the buffer the HW just finished (if any) */
    done = v->active;
    if (done) {
        struct vb2_v4l2_buffer *vb2v = &done->vb;

        /* make sure device writes are visible before userspace sees it */
        dma_rmb();

        vb2v->sequence = atomic_inc_return(&v->sequence_number);
        vb2v->vb2_buf.timestamp = ktime_get_ns();

        v->active = NULL; /* channel no longer owns this buffer */
        vb2_buffer_done(&vb2v->vb2_buf, VB2_BUF_STATE_DONE);
    }

    /* 2) Immediately arm the next queued buffer (if present) */
    ret = hws_arm_next(hws, ch);
    if (ret == -EAGAIN) {
        /* No queued buffers; optional: mask ch IRQ or mark queue error */
        return;
    }
    /* on success the engine is now pointed at v->active’s DMA address */
}

