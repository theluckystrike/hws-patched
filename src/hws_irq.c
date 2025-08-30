/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/compiler.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>

#include <sound/pcm.h>

#include "hws_irq.h"
#include "hws_reg.h"
#include "hws_video.h"
#include "hws_audio.h"


#define MAX_INT_LOOPS 100
#define CH_MASK     GENMASK(CH_SHIFT-1, 0)


static int hws_arm_next(struct hws_pcie_dev *hws, u32 ch)
{
    struct hws_video *v = &hws->video[ch];
    unsigned long flags;
    struct hwsvideo_buffer *buf;

    if (unlikely(READ_ONCE(hws->suspended)))
        return -EBUSY;

    if (unlikely(READ_ONCE(v->stop_requested) || !READ_ONCE(v->cap_active)))
        return -ECANCELED;


    spin_lock_irqsave(&v->irq_lock, flags);
    if (list_empty(&v->capture_queue)) {
        spin_unlock_irqrestore(&v->irq_lock, flags);
        return -EAGAIN;
    }

    buf = list_first_entry(&v->capture_queue, struct hwsvideo_buffer, list);
    list_del(&buf->list);
    v->active = buf;
    spin_unlock_irqrestore(&v->irq_lock, flags);

    /* Publish descriptor(s) before doorbell/MMIO kicks. */
    wmb();

    /* Avoid MMIO during suspend */
    if (unlikely(READ_ONCE(hws->suspended)))
        return -EBUSY;

    hws_program_video_from_vb2(hws, ch, &buf->vb.vb2_buf);
    return 0;
}

void hws_bh_video(struct tasklet_struct *t)
{
    struct hws_video *v = from_tasklet(v, t, bh_tasklet);
    struct hws_pcie_dev *hws = v->parent;
    unsigned int ch = v->channel_index;
    struct hwsvideo_buffer *done;
    int ret;

    if (unlikely(READ_ONCE(hws->suspended)))
        return;

    if (unlikely(READ_ONCE(v->stop_requested) || !READ_ONCE(v->cap_active)))
        return;

    /* 1) Complete the buffer the HW just finished (if any) */
    done = v->active;
    if (done) {
        struct vb2_v4l2_buffer *vb2v = &done->vb;

        dma_rmb(); /* device writes visible before userspace sees it */

        vb2v->sequence = ++v->sequence_number;          /* BH-only increment */
        vb2v->vb2_buf.timestamp = ktime_get_ns();

        v->active = NULL; /* channel no longer owns this buffer */
        vb2_buffer_done(&vb2v->vb2_buf, VB2_BUF_STATE_DONE);
    }

    if (unlikely(READ_ONCE(hws->suspended)))
        return;

    /* 2) Immediately arm the next queued buffer (if present) */
    ret = hws_arm_next(hws, ch);
    if (ret == -EAGAIN) {
        return;
    }
    /* On success the engine now points at v->active’s DMA address */
}


irqreturn_t irqhandler(int irq, void *info)
{
    struct hws_pcie_dev *pdx = info;
    u32 int_state, ack_mask = 0;

    /* Fast path: if suspended, quietly ack and exit */
    if (unlikely(READ_ONCE(pdx->suspended))) {
        int_state = readl(pdx->bar0_base + HWS_REG_INT_STATUS);
        if (int_state)
            writel(int_state, pdx->bar0_base + HWS_REG_INT_ACK);
        return int_state ? IRQ_HANDLED : IRQ_NONE;
    }

    u32 sys_status = readl(pdx->bar0_base + HWS_REG_SYS_STATUS);

    /* No DMA busy or card gone: exit early */
    if ((sys_status & HWS_SYS_DMA_BUSY_BIT) == 0 || sys_status == 0xFFFFFFFF)
        return IRQ_NONE;

    /* Read interrupt status bits */
    int_state = readl(pdx->bar0_base + HWS_REG_INT_STATUS);
    if (!int_state)
        return IRQ_NONE;  /* spurious interrupt */


    /* Loop until all pending bits are serviced (max 100 iterations) */
    for (u32 cnt = 0; int_state && cnt < MAX_INT_LOOPS; ++cnt) {
        for (unsigned int ch = 0; ch < pdx->cur_max_video_ch; ++ch) {
            u32 vbit = HWS_INT_VDONE_BIT(ch);

            if (!(int_state & vbit))
                continue;

            ack_mask |= vbit;

            /* If stream is active, check the device's half toggle and schedule BH */
            if (READ_ONCE(pdx->video[ch].cap_active)) {
                u32 toggle = readl(pdx->bar0_base + HWS_REG_VBUF_TOGGLE(ch)) & 0x01;
                dma_rmb(); /* ensure DMA writes visible before we inspect */

                if (READ_ONCE(pdx->video[ch].last_buf_half_toggle) != toggle) {
                    WRITE_ONCE(pdx->video[ch].last_buf_half_toggle, toggle);
                    tasklet_schedule(&pdx->video[ch].bh_tasklet);
                }
            }
        }

        for (unsigned int ch = 0; ch < pdx->cur_max_linein_ch; ++ch) {
            u32 abit = HWS_INT_ADONE_BIT(ch);
            if (!(int_state & abit))
                continue;

            ack_mask |= abit;

            /* Only service running streams */
            if (!READ_ONCE(pdx->audio[ch].cap_active) ||
                !READ_ONCE(pdx->audio[ch].stream_running))
                continue;

            /* If your HW exposes a 0/1 toggle, read it (optional) */
            pdx->audio[ch].last_period_toggle =
                readl(pdx->bar0_base + HWS_REG_ABUF_TOGGLE(ch)) & 0x01;

            /* Make device writes visible before notifying ALSA */
            dma_rmb();

		struct hws_audio *a = &pdx->audio[ch];
		struct snd_pcm_substream *ss = READ_ONCE(a->pcm_substream);
		if (ss) {
		    struct snd_pcm_runtime *rt = READ_ONCE(ss->runtime);
		    if (rt) {
			snd_pcm_uframes_t step = bytes_to_frames(rt, a->period_bytes);
			snd_pcm_uframes_t pos  = READ_ONCE(a->ring_wpos_byframes);
		       pos += step;
			if (pos >= rt->buffer_size)
			    pos -= rt->buffer_size;
			WRITE_ONCE(a->ring_wpos_byframes, pos);
			snd_pcm_period_elapsed(ss);
		    }
		}
            /* Program the period HW will fill next */
            hws_audio_program_next_period(pdx, ch);
        }

        /* Acknowledge (clear) all bits we just handled */
        writel(ack_mask, pdx->bar0_base + HWS_REG_INT_ACK);

        /* Immediately clear ack_mask to avoid re-acknowledging stale bits */
        ack_mask = 0;

        /* Re‐read in case new interrupt bits popped while processing */
        int_state = readl(pdx->bar0_base + HWS_REG_INT_STATUS);
        if (cnt + 1 == MAX_INT_LOOPS)
            dev_warn_ratelimited(&pdx->pdev->dev,
                                 "IRQ storm? status=0x%08x\n", int_state);
    }

    return IRQ_HANDLED;
}
