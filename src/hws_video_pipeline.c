/* SPDX-License-Identifier: GPL-2.0-only */
#include "hws_video_pipeline.h"

#include "hws.h"
#include "hws_reg.h"
#include "hws_pci.h"
#include "hws_scaler.h"
#include "hws_init.h"
#include "hws_dma.h"
#include "hws_audio_pipeline.h"

#define FRAME_DONE_MARK 0x55AAAA55

struct copy_ctx {
    u8  *pSrc;           /* DMA source pointer          */
    u32  *pMask;          /* mask at end of this block   */
    int pitch;           /* bytes / line                */
    int copy_sz;         /* bytes in this half          */
    int half_sz;         /* bytes in *half* of frame    */
    int buf_idx;         /* 0 = 2nd half, 1 = 1st half  */
    bool interlace;
};


static inline u32 hws_read_port_hpd(struct hws_pcie_dev *pdx, int port)
{
	/* OR the two pipes that belong to this HDMI jack */
	int pipe0 = port * 2;
	int pipe1 = pipe0 + 1;
	// FIXME
	return  READ_REGISTER_ULONG(pdx, HWS_REG_HPD(pipe0)) |
		READ_REGISTER_ULONG(pdx, HWS_REG_HPD(pipe1));
}

int set_video_format_size(struct hws_pcie_dev *pdx, int ch, int w, int h)
{
	int hf_size;
	int down_size;

	if (pdx->support_yv12 == 0) {
		hf_size = (w * h) / (16 * 128);
		hf_size = hf_size * 16 * 128;
		down_size = (w * h * 2) - hf_size;
	} else if (pdx->support_yv12 == 1) {
		hf_size = (w * h * 12) / (16 * 16 * 128);
		hf_size = hf_size * 16 * 128;
		down_size = ((w * h * 12) / 8) - hf_size;
	} else {
		hf_size = (w * h * 5) / (8 * 16 * 128);
		hf_size = hf_size * 16 * 128;
		down_size = ((w * h * 5) / 4) - hf_size;
	}
	pdx->video[ch].fmt_curr.width = w; // Image Width
	pdx->video[ch].fmt_curr.height = h; // Image Height 

	pdx->video[ch].fmt_curr.half_size = hf_size;
	pdx->video[ch].fmt_curr.down_size = down_size;

	return 1;
}

void change_video_size(struct hws_pcie_dev *pdx, int ch, int w, int h,
			    int interlace)
{
	int j;
	int halfframeLength;
	unsigned long flags;
	spin_lock_irqsave(&pdx->videoslock[ch], flags);
	for (j = 0; j < MAX_VIDEO_QUEUE; j++) {
		pdx->video[ch].queue_status[j].width = w;
		pdx->video[ch].queue_status[j].height = h;
		pdx->video[ch].queue_status[j].interlace = interlace;
	}
	spin_unlock_irqrestore(&pdx->videoslock[ch], flags);

	set_video_format_size(pdx, ch, w, h);
	halfframeLength = pdx->video[ch].fmt_curr.half_size / 16;

	// FIXME
	WRITE_REGISTER_ULONG(
		pdx, (DWORD)(CBVS_IN_BUF_BASE2 + (ch * PCIE_BARADDROFSIZE)),
		halfframeLength); //Buffer 1 address
}

int check_video_capture(struct hws_pcie_dev *pdx, int index)
{
	u32 status;
	int enable;

	// FIXME
	status = READ_REGISTER_ULONG(pdx, HWS_REG_VCAP_ENABLE);
	enable = (status >> index) & 0x01;
	return enable;
}

void hws_enable_video_capture(struct hws_pcie_dev *hws,
                                     unsigned int chan,
                                     bool on)
{
    u32 status;

    /* bail if the PCI device is lost or chan out of range */
    if (hws->pci_lost || chan >= hws->max_channels)
        return;

    /* read-modify-write the bitmask register */
    status = hws_read32(hws, HWS_REG_VCAP_ENABLE);
    if (on)
        status |= BIT(chan);
    else
        status &= ~BIT(chan);

    /* track state in our per-channel struct */
    hws->video[chan].cap_active = on;

    /* write it back */
    hws_write32(hws, HWS_REG_VCAP_ENABLE, status);
}

void check_card_status(struct hws_pcie_dev *pdx)
{
	u32 status;
	status = READ_REGISTER_ULONG(pdx, HWS_REG_SYS_STATUS);

	if ((status & BIT(0)) != BIT(0)) {
		InitVideoSys(pdx, 1);
	}
}

int StartVideoCapture(struct hws_pcie_dev *pdx, int index)
{
	int j;

	if (pdx->video[index].cap_active == 1) {
		// FIXME: legacy in func
		check_card_status(pdx);
		if (check_video_capture(pdx, index) == 0) {
			hws_enable_video_capture(pdx, index, true);
		}
		return -1;
	}
	//--------------------
	check_card_status(pdx);
	//--------------------
	//spin_lock_irqsave(&pdx->videoslock[index], flags);
	for (j = 0; j < MAX_VIDEO_QUEUE; j++) {
		pdx->m_pVCAPStatus[index][j].byLock = MEM_UNLOCK;
		pdx->m_pVCAPStatus[index][j].byPath = 2;
		pdx->m_VideoInfo[index].pStatusInfo[j].byLock = MEM_UNLOCK;
	}
	pdx->m_nRDVideoIndex[index] = 0;
	pdx->m_VideoInfo[index].dwisRuning = 1;
	pdx->m_VideoInfo[index].m_nVideoIndex = 0;
	//spin_unlock_irqrestore(&pdx->videoslock[index], flags);

	pdx->m_bChangeVideoSize[index] = 0;
	pdx->m_bVCapIntDone[index] = 1;
	pdx->m_pVideoEvent[index] = 1;
	pdx->m_nVideoBusy[index] = 0;
	pdx->video_data[index] = 0;
	hws_enable_video_capture(pdx, index, true);
	return 0;
}

void StopVideoCapture(struct hws_pcie_dev *pdx, int index)
{
	//int inc=0;

	if (pdx->video[index].cap_active == 0)
		return;
	//pdx->m_nVideoIndex[index] =0;
	pdx->m_VideoInfo[index].dwisRuning = 0;
	pdx->video[index].stop_requested = 1;
	pdx->m_pVideoEvent[index] = 0;
	pdx->m_bChangeVideoSize[index] = 0;
	hws_enable_video_capture(pdx, index, false);
	pdx->m_bVCapIntDone[index] = 0;
}

void check_video_format(struct hws_pcie_dev *pdx)
{
	int i;
	for (i = 0; i < pdx->cur_max_video_ch; i++) {
        // FIXME: figure out if we can check update_hpd_status early and exit fast
		pdx->video[index].signal_loss_cnt = get_video_status(pdx, i);

        /* If we just detected a loss on an active capture channel… */
		if ((pdx->video[index].signal_loss_cnt  == 0x1) &&
		    (pdx->video[index].cap_active == true)) {
              /* …schedule the “no‐video” handler on the vido_wq workqueue */
			queue_work(pdx->vido_wq, &pdx->video[i].videowork);
		}
	}
}

void hws_init_video_sys(struct hws_pcie_dev *hws, bool enable)
{
    int i, j;

    /* If already running and we're not resetting, nothing to do */
    if (hws->start_run && !enable)
        return;

    /* 1) reset the decoder mode register to 0 */
    hws_write32(hws, HWS_REG_DEC_MODE, 0x00000000);

    /* 2) point DMA pointers at our buffers */
    hws_set_dma_address(hws);

    /* 3) on a full reset, clear all per-channel status and indices */
    if (!enable) {
        for (i = 0; i < hws->max_channels; i++) {
            struct vcap_status *vs = hws->video[i].queue_status;

            for (j = 0; j < MAX_VIDEO_QUEUE; j++) {
                vs[j].lock      = MEM_UNLOCK;
                vs[j].path      = 2;
                vs[j].field     = 0;
                vs[j].interlace = 0;
            }

            /* reset audio write pointer */
            hws->audio[i].wr_idx = 0;

            /* helpers to arm/disable capture engines */
            hws_enable_video_capture(hws, i, false);
            EnableAudioCapture(hws, i, 0);
        }
    }

    /* 4) enable all interrupts */
    hws_write32(hws, INT_EN_REG_BASE, 0x003FFFFF);

    /* 5) “Start run”: set bit31, wait a bit, then program low 24 bits */
    hws_write32(hws, HWS_REG_DEC_MODE, 0x80000000);
    udelay(500);
    hws_write32(hws, HWS_REG_DEC_MODE, 0x80FFFFFF);
    hws_write32(hws, HWS_REG_DEC_MODE, 0x00000013);

    /* 6) record that we're now running */
    hws->start_run = true;
}

/* ──────────────────────────────────────────────────────────────── */
/* Utility: compare-and-write to a 32-bit register only if needed  */
/* ──────────────────────────────────────────────────────────────── */
static inline void write_if_diff(struct hws_pcie_dev *pdx,
                                 u32 reg_addr, u32 new_val)
{

    // FIXME
    if (READ_REGISTER_ULONG(pdx, reg_addr) != new_val)
        // FIXME
        WRITE_REGISTER_ULONG(pdx, reg_addr, new_val);
}

/* ──────────────────────────────────────────────────────────────── */
/* 0. HPD / +5 V                                                   */
/* ──────────────────────────────────────────────────────────────── */
static bool update_hpd_status(struct hws_pcie_dev *pdx, unsigned int ch)
{
    u32  hpd    = hws_read_port_hpd(pdx, ch);   /* jack-level status   */
    bool power  = !!(hpd & HWS_5V_BIT);         /* +5 V present        */
    bool hpd_hi = !!(hpd & HWS_HPD_BIT);        /* HPD line asserted   */

    /* Cache the +5 V state in V4L2 ctrl core (only when it changes) */
    if (power != pdx->video[ch].detect_tx_5v_ctrl->cur.val)
        v4l2_ctrl_s_ctrl(pdx->video[ch].detect_tx_5v_ctrl, power);

    /* “Signal present” means *both* rails up. Tweak if your HW differs. */
    return power && hpd_hi;
}

/* ──────────────────────────────────────────────────────────────── */
/* 1. Active / interlace flags                                     */
/* ──────────────────────────────────────────────────────────────── */
static bool update_active_and_interlace_flags(struct hws_pcie_dev *pdx,
                                              unsigned int ch)
{
    // FIXME
    u32  reg        = READ_REGISTER_ULONG(pdx, HWS_REG_ACTIVE_STATUS);
    bool active_vid = !!((reg >> ch) & 0x01);
    bool interlace  = !!(((reg >> 8) >> ch) & 0x01);

    /* Track interlace in driver state so other paths don’t re-decode */
    pdx->video[ch].queue_status[0].interlace = interlace;

    return active_vid;               /* false → “no video” early-out   */
}

/* ──────────────────────────────────────────────────────────────── */
/* 2a. Modern devices (HW version > 0)                              */
/* ──────────────────────────────────────────────────────────────── */
static void handle_hwv2_path(struct hws_pcie_dev *pdx, unsigned int ch)
{
    /* ── frame-rate in ─────────────────────────────────────────── */
    // FIXME
    u32 frame_rate = READ_REGISTER_ULONG(pdx, HWS_REG_FRAME_RATE(ch));
    pdx->video[ch].queue_status[0].frame_rate = frame_rate;

    /* ── output resolution (programmed) ────────────────────────── */
    // FIXME
    u32 reg          = READ_REGISTER_ULONG(pdx, HWS_REG_OUT_RES(ch));
    u16 out_res_w    =  reg        & 0xFFFF;
    u16 out_res_h    = (reg >> 16) & 0xFFFF;

    if (out_res_w != pdx->video[ch].queue_status[0].out_width ||
        out_res_h != pdx->video[ch].queue_status[0].out_height)
    {
        u32 packed =  (pdx->video[ch].queue_status[0].out_height << 16) |
                       pdx->video[ch].queue_status[0].out_width;
        write_if_diff(pdx, HWS_REG_OUT_RES(ch), packed);
    }

    /* ── output frame-rate (programmed) ────────────────────────── */
    // FIXME
    u32 out_fps = READ_REGISTER_ULONG(pdx, HWS_REG_OUT_FRAME_RATE(ch));
    if (out_fps != pdx->video[ch].queue_status[0].out_fps)
        write_if_diff(pdx, HWS_REG_OUT_FRAME_RATE(ch),
                           pdx->video[ch].queue_status[0].out_fps);

    /* ── BCHS (brightness/contrast/hue/saturation) ─────────────── */
    // FIXME
    reg = READ_REGISTER_ULONG(pdx, HWS_REG_BCHS(ch));
    u8 br =  reg        & 0xFF;
    u8 co = (reg >> 8)  & 0xFF;
    u8 hu = (reg >> 16) & 0xFF;
    u8 sa = (reg >> 24) & 0xFF;

    if (br != pdx->video[ch].current_brightness ||
        co != pdx->video[ch].current_contrast   ||
        hu != pdx->video[ch].current_hue        ||
        sa != pdx->video[ch].current_saturation)
    {

        u32 packed =  (pdx->video[ch].current_saturation << 24) |
                      (pdx->video[ch].current_hue        << 16) |
                      (pdx->video[ch].current_contrast   <<  8) |
                       pdx->video[ch].current_brightness;
        write_if_diff(pdx, HWS_REG_BCHS(ch), packed);
    }

    /* ── HDCP detect bit ───────────────────────────────────────── */
    // FIXME
    reg = READ_REGISTER_ULONG(pdx, HWS_REG_HDCP_STATUS);
    pdx->video[ch].queue_status[0].hdcp = !!((reg >> ch) & 0x01);
}

/* ──────────────────────────────────────────────────────────────── */
/* 2b. Legacy devices (SW FPS estimator)                           */
/* ──────────────────────────────────────────────────────────────── */
static void handle_legacy_path(struct hws_pcie_dev *pdx, unsigned int ch)
{
    // FIXME
    u32 sw_rate = pdx->m_dwSWFrameRate[ch];

    if (sw_rate > 10) {
        int fps;
        if      (sw_rate > 55 * 2) fps = 60;
        else if (sw_rate > 45 * 2) fps = 50;
        else if (sw_rate > 25 * 2) fps = 30;
        else if (sw_rate > 20 * 2) fps = 25;
        else                       fps = 60;  /* default fallback */

        // FIXME
        pdx->m_pVCAPStatus[ch][0].dwFrameRate = fps;
    }

    // FIXME
    pdx->m_dwSWFrameRate[ch] = 0;        /* reset estimator window */
}

/* ──────────────────────────────────────────────────────────────── */
/* 3. Live input resolution + change_video_size() trigger            */
/* ──────────────────────────────────────────────────────────────── */
static void update_live_resolution(struct hws_pcie_dev *pdx, unsigned int ch)
{
    // FIXME
    u32 reg   = READ_REGISTER_ULONG(pdx, HWS_REG_IN_RES(ch));
    u16 res_w =  reg        & 0xFFFF;
    u16 res_h = (reg >> 16) & 0xFFFF;

    bool interlace = pdx->video[ch].queue_status[0].interlace;

    bool within_hw =
        (res_w <= MAX_VIDEO_HW_W) &&
        ((!interlace &&  res_h      <= MAX_VIDEO_HW_H) ||
         ( interlace && (res_h * 2) <= MAX_VIDEO_HW_H));

    bool changed =
        (res_w      != pdx->video[ch].queue_status[0].width) ||
        (res_h      != pdx->video[ch].queue_status[0].height)||
        (interlace  != pdx->video[ch].queue_status[0].interlace);

    if (within_hw && changed)
        change_video_size(pdx, ch, res_w, res_h, interlace);
}

int get_video_status(struct hws_pcie_dev *pdx, unsigned int ch)
{
    // FIXME: I don't think this works?
    // if (!update_hpd_status(pdx, ch))
    //    return 1;                         /* no +5 V / HPD */

    if (!update_active_and_interlace_flags(pdx, ch))
        return 1;                         /* no active video */

    if (pdx->hw_ver> 0)
        handle_hwv2_path(pdx, ch);
    else
        // FIXME: legacy struct names in subfunction
        handle_legacy_path(pdx, ch);

    update_live_resolution(pdx, ch);

    return 0;                             /* success */
}


static inline bool frame_dims_ok(struct hws_pcie_dev *pdx, int dec)
{
    int w = pdx->m_pVCAPStatus[dec][0].dwWidth;
    int h = pdx->m_pVCAPStatus[dec][0].dwHeight;
    return (w > 0 && h > 0 &&
            w <= MAX_VIDEO_HW_W && h <= MAX_VIDEO_HW_H);
}

static void init_copy_ctx(struct hws_pcie_dev *pdx, int dec,
                          struct copy_ctx *c)
{
    int w  = pdx->m_pVCAPStatus[dec][0].dwWidth;
    c->interlace = pdx->m_pVCAPStatus[dec][0].dwinterlace;
    c->pitch     = pdx->support_yv12 ? w * 12 / 8 : w * 2;

    c->buf_idx   = pdx->m_nVideoBufferIndex[dec];
    c->half_sz   = pdx->video[dec].fmt_curr.half_size;
    c->copy_sz   = (c->buf_idx == 1) ? c->half_sz
                      : pdx->video[dec].fmt_curr.down_size;

    u8 *base     = pdx->video[dec].buf_virt;
    c->pSrc      = base + (c->buf_idx ? 0 : c->half_sz);
    int lines    = c->copy_sz / c->pitch;
    c->pMask     = (int *)(c->pSrc + c->copy_sz -
                           (lines - 2) * c->pitch);
}

static bool mask_skip_this_half(struct hws_pcie_dev *pdx, int dec,
                                struct copy_ctx *c)
{
    if (*c->pMask != FRAME_DONE_MARK)
        return false;                     /* ready to consume   */

    /* mask already set  → update half-done bookkeeping */
    if (pdx->m_nVideoHalfDone[dec])
        pdx->m_nVideoHalfDone[dec] = 0;
    return true;                          /* skip copy */
}

static u8 *get_write_buf(struct hws_pcie_dev *pdx, int dec,
                         struct copy_ctx *c)
{
    ACAP_VIDEO_INFO *vq = &pdx->m_VideoInfo[dec];
    int  idx = -1;

    if (c->buf_idx == 1) {        /* first half -> lock only if free */
        if (vq->pStatusInfo[vq->m_nVideoIndex].byLock == MEM_UNLOCK)
            idx = vq->m_nVideoIndex;
    } else {                      /* always take second half */
        idx = vq->m_nVideoIndex;
    }

    /* fallback in case we never advanced */
    if (idx == -1) {
        vq->pStatusInfo[vq->m_nVideoIndex].byLock = MEM_UNLOCK;
        idx = vq->m_nVideoIndex;
    }

    return (idx != -1) ? vq->m_pVideoBufData[idx] : NULL;
}

static void copy_and_update(struct hws_pcie_dev *pdx, int dec,
                            u8 *dst, struct copy_ctx *c)
{
    if (c->buf_idx == 0)          /* second half writes at +half_sz  */
        dst += c->half_sz;

    dma_sync_single_for_cpu(&pdx->pdev->dev,
                            pdx->video[dec].buf_phys_addr,
                            pdx->max_hw_video_buf_sz,
                            DMA_FROM_DEVICE);
    memcpy(dst, c->pSrc, c->copy_sz);

    if (c->buf_idx == 0) {        /* finished full frame – rotate Q  */
        unsigned long flags;
        spin_lock_irqsave(&pdx->videoslock[dec], flags);

        ACAP_VIDEO_INFO *vq = &pdx->m_VideoInfo[dec];
        int idx = vq->m_nVideoIndex;

        vq->m_nVideoIndex = (idx + 1) % MAX_VIDEO_QUEUE;

        vq->pStatusInfo[idx].dwWidth      =
            pdx->m_pVCAPStatus[dec][0].dwWidth;
        vq->pStatusInfo[idx].dwHeight     =
            pdx->m_pVCAPStatus[dec][0].dwHeight;
        vq->pStatusInfo[idx].dwinterlace  = c->interlace;
        vq->pStatusInfo[idx].byLock       = MEM_LOCK;

        spin_unlock_irqrestore(&pdx->videoslock[dec], flags);
    } else {
        pdx->m_nVideoHalfDone[dec] = 1;   /* first half done  */
    }
}


int MemCopyVideoToStream(struct hws_pcie_dev *pdx, int dec)
{
    /* 0. fast validation ─────────────────────────────────────────── */
    if (!frame_dims_ok(pdx, dec))
        return -1;

    /* 1. derive pitches, offsets, mask ptr, half/whole flags ─────── */
    struct copy_ctx ctx;
    init_copy_ctx(pdx, dec, &ctx);

    /* 2. drop out early when mask says “frame not ready” ─────────── */
    if (mask_skip_this_half(pdx, dec, &ctx))
        return -1;

    /* 3. if capture engine isn’t running just stamp the mask & quit */
    if (!pdx->m_VideoInfo[dec].dwisRuning) {
        *ctx.pMask = FRAME_DONE_MARK;
        return 0;
    }

    /* 4. lock & obtain destination buffer ‐ returns NULL on miss    */
    u8 *dst = get_write_buf(pdx, dec, &ctx);
    if (!dst) {                     /* no free buffer this pass */
        pdx->m_nVideoHalfDone[dec] = 0;
        *ctx.pMask = FRAME_DONE_MARK;
        return 0;
    }

    /* 5. copy + optional metadata update for first half ─────────── */
    copy_and_update(pdx, dec, dst, &ctx);

    /* 6. stamp mask so HW/ISR knows this block is consumed ───────── */
    *ctx.pMask = FRAME_DONE_MARK;
    return 0;
}


/* ---------- new small POD structs -------------------------------------- */

struct frame_ctx {
	/* source (capture) side */
	int             src_idx;        /* index in m_VideoInfo[] ring       */
	u8             *src;            /* locked PCIe video buffer          */
	int             src_w, src_h;   /* after interlace fix-up            */
	int             interlace;      /* 0/1 from capture status           */

	/* destination (V4L2) side */
	struct hwsvideo_buffer *dst_buf;
	int             dst_w, dst_h;

	/* transform flags */
	bool            need_rotate;
	size_t          copy_size;      /* bytes to memcpy() for YV12/NV12   */
};

/* ---------- forward declarations of helpers ---------------------------- */

/* Locks pdx->videoslock[…] only while touching shared state. */
static int  hws_fetch_buffers(struct hws_video *dev,
			      struct frame_ctx *c, unsigned long *flags);

static void hws_transform_frame(struct hws_video *dev,
				struct frame_ctx *c);

static void hws_release_src(struct hws_video *dev,
			    struct frame_ctx *c, unsigned long flags);

static void hws_fill_no_signal(struct hws_video *dev,
			       struct frame_ctx *c);

/* ----------------------------------------------------------------------- */
void video_data_process(struct work_struct *work)
{
	struct hws_video     *dev  = container_of(work, struct hws_video,
						  videowork);
	struct hws_pcie_dev  *pdx  = dev->dev;
	struct frame_ctx      ctx  = { .src_idx = -1 };
	unsigned long         flags;
	int                   ret;

	/* ------------------------------------------------------------
	 * 1. Get one capture frame & one vb2 destination buffer
	 * ------------------------------------------------------------ */
	ret = hws_fetch_buffers(dev, &ctx, &flags);
	if (ret)		/* nothing to do right now */
		return;

	/* ------------------------------------------------------------
	 * 2. Decide whether to paste “no-video” pattern or process data
	 * ------------------------------------------------------------ */
    // NOTE: if we wanted to trigger a NO VIDEO, here might be a spot, or where this value gets set, 
    // in CheckVideoFormat
    // CheckVideoFormat gets called from MainKsThreadHandle, which loops
    // endlessly and checks every second, with a slieep
	if (pdx->video[dev->index].singal_loss_cnt)
		hws_fill_no_signal(dev, &ctx);
	else
		hws_transform_frame(dev, &ctx);

	/* ------------------------------------------------------------
	 * 3. Return vb2 buffer downstream
	 * ------------------------------------------------------------ */
	ctx.dst_buf->vb.sequence           = atomic_inc_return(&dev->sequence_number);
	ctx.dst_buf->vb.vb2_buf.timestamp  = ktime_get_ns();
	ctx.dst_buf->vb.field              = V4L2_FIELD_NONE;
	vb2_buffer_done(&ctx.dst_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

	/* ------------------------------------------------------------
	 * 4. Unlock capture slot & advance ring index
	 * ------------------------------------------------------------ */
	hws_release_src(dev, &ctx, flags);
}

/* =======================================================================
 *                           Helper definitions
 * ======================================================================= */

/**
 * hws_fetch_buffers() – pop one src/dst pair.
 * Return 0 on success, -EAGAIN if no work, other negative on error.
 */
static int hws_fetch_buffers(struct hws_video *dev,
			     struct frame_ctx *c,
			     unsigned long *flags)
{
	struct hws_pcie_dev *pdx = dev->dev;
	const int           ch   = dev->index;
	int                 i;

	spin_lock_irqsave(&pdx->videoslock[ch], *flags);

	/* ---------------- capture geometry & rotation test ---------------- */
	c->src_w   = pdx->m_pVCAPStatus[ch][0].dwWidth;
	c->src_h   = pdx->m_pVCAPStatus[ch][0].dwHeight;
	c->dst_w   = dev->current_out_width;
	c->dst_h   = dev->curren_out_height;
	c->need_rotate =
		((c->src_w == 1280 && c->src_h == 720)  ||
		 (c->src_w ==  960 && c->src_h == 540)) &&
		(c->dst_w == 1080 && c->dst_h == 1920);

	/* handle interlaced input */
	c->interlace = pdx->m_pVCAPStatus[ch][0].dwinterlace;
	if (c->interlace)
		c->src_h *= 2;

	/* ---------------- select source frame from PCIe ring --------------- */
	if (pdx->video[ch].signal_loss_cnt == 0) {
		for (i = pdx->m_nRDVideoIndex[ch]; i < MAX_VIDEO_QUEUE; i++) {
			if (pdx->m_VideoInfo[ch].pStatusInfo[i].byLock ==
			    MEM_LOCK) {
				c->src_idx = i;
				c->src     = pdx->m_VideoInfo[ch]
					     .m_pVideoBufData[i];
				break;
			}
		}
		if (c->src_idx < 0) {
			spin_unlock_irqrestore(&pdx->videoslock[ch], *flags);
			return -EAGAIN;
		}
	} else { /* loss of signal – unlock everything up-front */
		for (i = 0; i < MAX_VIDEO_QUEUE; i++)
			pdx->m_VideoInfo[ch].pStatusInfo[i].byLock = MEM_UNLOCK;
	}

	/* ---------------- pop one vb2 dst buffer -------------------------- */
	if (list_empty(&dev->queue)) {
		spin_unlock_irqrestore(&pdx->videoslock[ch], *flags);
		return -EAGAIN;
	}

	c->dst_buf = list_first_entry(&dev->queue,
				      struct hwsvideo_buffer, queue);
	list_del(&c->dst_buf->queue);
	spin_unlock_irqrestore(&pdx->videoslock[ch], *flags);

	/* ---------------- compute format-specific copy size --------------- */
	switch (pdx->support_yv12) {
	case 1: /* YV12 */
		c->copy_size = c->src_w * c->src_h * 12 / 8;
		break;
	case 2: /* NV12 */
		c->copy_size = c->src_w * c->src_h * 5 / 4;
		break;
	default: /* YUY2 passthrough */
		c->copy_size = c->src_w * c->src_h * 2;
	}

	return 0;
}

/* --------------------------------------------------------------------- */
static void hws_transform_frame(struct hws_video *dev, struct frame_ctx *c)
{
	struct hws_pcie_dev *pdx = dev->dev;
	const int           ch   = dev->index;
	u8 *work = pdx->m_VideoInfo[ch].m_pVideoYUV2Buf;

	/* 1. Yx12/NV12 → YUY2 or de-interlace passthrough ---------------- */
	switch (pdx->support_yv12) {
	case 1:
		memcpy(work, c->src, c->copy_size);
		FillYUU2(work, work, c->src_w, c->src_h, c->interlace);
		break;
	case 2:
		memcpy(work, c->src, c->copy_size);
		FillNV12ToYUY2(work, work, c->src_w, c->src_h, c->interlace);
		break;
	default:
		SetDeInterlace(c->src, work,
			       c->src_w, c->src_h, c->interlace);
	}

	/* 2. Scale if geometry differs ----------------------------------- */
	if (c->src_w * c->src_h * 2 != c->dst_w * c->dst_h * 2) {
		VideoScaler(work,
			    pdx->m_VideoInfo[ch].m_pVideoScalerBuf,
			    c->src_w, c->src_h,
			    c->dst_w, c->dst_h);
		work = pdx->m_VideoInfo[ch].m_pVideoScalerBuf;
	}

	/* 3. Optional 90 deg rotation ------------------------------------ */
	if (c->need_rotate) {
		VideoRotate90deg(work,
				 pdx->m_VideoInfo[ch].m_pRotateVideoBuf,
				 c->dst_h, c->dst_w,
				 c->dst_w, c->dst_h);
		work = pdx->m_VideoInfo[ch].m_pRotateVideoBuf;
	}

	/* 4. Copy into vb2 userspace buffer ------------------------------ */
	memcpy(c->dst_buf->mem, work, c->dst_w * c->dst_h * 2);
}

/* --------------------------------------------------------------------- */
static void hws_fill_no_signal(struct hws_video *dev, struct frame_ctx *c)
{
	SetNoVideoMem(c->dst_buf->mem,
		      dev->current_out_width,
		      dev->curren_out_height);
}

/* --------------------------------------------------------------------- */
static void hws_release_src(struct hws_video *dev,
			    struct frame_ctx *c,
			    unsigned long flags)
{
	struct hws_pcie_dev *pdx = dev->dev;
	const int           ch   = dev->index;

	/* If we processed a real frame, unlock its slot and advance read idx */
	if (c->src_idx >= 0) {
		spin_lock_irqsave(&pdx->videoslock[ch], flags);

		pdx->m_VideoInfo[ch].pStatusInfo[c->src_idx].byLock = MEM_UNLOCK;
		pdx->m_nRDVideoIndex[ch] = c->src_idx + 1;
		if (pdx->m_nRDVideoIndex[ch] >= MAX_VIDEO_QUEUE)
			pdx->m_nRDVideoIndex[ch] = 0;

		spin_unlock_irqrestore(&pdx->videoslock[ch], flags);
	}
}

