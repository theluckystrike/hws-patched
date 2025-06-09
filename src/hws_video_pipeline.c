#include "hws_video_pipeline.h"
#include "hws_reg.h"
#include "hws_pci.h"
#include "hws_scaler.h"
#include "hws_init.h"
#include "hws_dma.h"
#include "hws_audio_pipeline.h"

static inline u32 hws_read_port_hpd(struct hws_pcie_dev *pdx, int port)
{
	/* OR the two pipes that belong to this HDMI jack */
	int pipe0 = port * 2;
	int pipe1 = pipe0 + 1;
	return  READ_REGISTER_ULONG(pdx, HWS_REG_HPD(pipe0)) |
		READ_REGISTER_ULONG(pdx, HWS_REG_HPD(pipe1));
}

int SetVideoFormatSize(struct hws_pcie_dev *pdx, int ch, int w, int h)
{
	int hf_size;
	int down_size;
	//int frame_size;
	//int buf_cnt;

	if (pdx->m_Device_SupportYV12 == 0) {
		hf_size = (w * h) / (16 * 128);
		hf_size = hf_size * 16 * 128;
		down_size = (w * h * 2) - hf_size;
	} else if (pdx->m_Device_SupportYV12 == 1) {
		hf_size = (w * h * 12) / (16 * 16 * 128);
		hf_size = hf_size * 16 * 128;
		down_size = ((w * h * 12) / 8) - hf_size;
	} else {
		hf_size = (w * h * 5) / (8 * 16 * 128);
		hf_size = hf_size * 16 * 128;
		down_size = ((w * h * 5) / 4) - hf_size;
	}
	pdx->m_format[ch].dwWidth = w; // Image Width
	pdx->m_format[ch].dwWidth = w; // Image Width
	pdx->m_format[ch].HLAF_SIZE = hf_size;
	pdx->m_format[ch].DWON_SIZE = down_size;
	//DbgPrint("[MV-X1]-CH0 SetVideoFormteSize = %d %d \n",m_format[ch].HLAF_SIZE,m_format[ch].DWON_SIZE);

	return 1;
}
void ChangeVideoSize(struct hws_pcie_dev *pdx, int ch, int w, int h,
			    int interlace)
{
	int j;
	int halfframeLength;
	unsigned long flags;
	spin_lock_irqsave(&pdx->videoslock[ch], flags);
	for (j = 0; j < MAX_VIDEO_QUEUE; j++) {
		pdx->m_pVCAPStatus[ch][j].dwWidth = w;
		pdx->m_pVCAPStatus[ch][j].dwHeight = h;
		pdx->m_pVCAPStatus[ch][j].dwinterlace = interlace;
	}
	spin_unlock_irqrestore(&pdx->videoslock[ch], flags);
	SetVideoFormatSize(pdx, ch, w, h);
	halfframeLength = pdx->m_format[ch].HLAF_SIZE / 16;
	WRITE_REGISTER_ULONG(
		pdx, (DWORD)(CBVS_IN_BUF_BASE2 + (ch * PCIE_BARADDROFSIZE)),
		halfframeLength); //Buffer 1 address
}

int CheckVideoCapture(struct hws_pcie_dev *pdx, int index)
{
	ULONG status;
	int enable;
	status = READ_REGISTER_ULONG(pdx, HWS_REG_VCAP_ENABLE);
	enable = (status >> index) & 0x01;
	return enable;
}

void EnableVideoCapture(struct hws_pcie_dev *pdx, int index, int en)
{
	ULONG status;
	int enable;
	if (pdx->m_PciDeviceLost)
		return;
	status = READ_REGISTER_ULONG(pdx, HWS_REG_VCAP_ENABLE);
	if (en) {
		enable = 1;
		enable = enable << index;
		status = status | enable;
	} else {
		enable = 1;
		enable = enable << index;
		enable = ~enable;
		status = status & enable;
	}
	pdx->m_bVCapStarted[index] = en;
	WRITE_REGISTER_ULONG(pdx, HWS_REG_VCAP_ENABLE, status);
	/* Optional: re‐read to verify 
     * status = READ_REGISTER_ULONG(pdx, HWS_REG_VCAP_ENABLE);
     * printk("EnableVideoCapture[%d] = 0x%08X (started=%d)\n", 
     *        index, status, pdx->m_bVCapStarted[index]);
     */
}

int StartVideoCapture(struct hws_pcie_dev *pdx, int index)
{
	int j;
	//unsigned long flags;
	if (pdx->m_bVCapStarted[index] == 1) {
		CheckCardStatus(pdx);
		if (CheckVideoCapture(pdx, index) == 0) {
			EnableVideoCapture(pdx, index, 1);
		}
		return -1;
	}
	//--------------------
	CheckCardStatus(pdx);
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
	EnableVideoCapture(pdx, index, 1);
	return 0;
}

void StopVideoCapture(struct hws_pcie_dev *pdx, int index)
{
	//int inc=0;

	if (pdx->m_bVCapStarted[index] == 0)
		return;
	//pdx->m_nVideoIndex[index] =0;
	pdx->m_VideoInfo[index].dwisRuning = 0;
	pdx->m_bVideoStop[index] = 1;
	pdx->m_pVideoEvent[index] = 0;
	pdx->m_bChangeVideoSize[index] = 0;
#if 0
	while(1)
	{
		if(pdx->m_bVideoStop[index] ==0)
		{
			break;
		}
		inc++;
		if(inc >2000)
		{
			break;
		}
		msleep(10);
	}
#endif
	EnableVideoCapture(pdx, index, 0);
	pdx->m_bVCapIntDone[index] = 0;
}

void CheckVideoFmt(struct hws_pcie_dev *pdx)
{
	//PAGED_CODE();
	int i;
	//DWORD value;
	//DWORD SetData;
	//	int ret=0;
	//int nNeed_ReInit =0;
	//	unsigned char mark=1;
	for (i = 0; i < pdx->m_nCurreMaxVideoChl; i++) {
#if 0
			value =  ReadDevReg((DWORD)(CVBS_IN_BASE + ((91+i*2)*PCIE_BARADDROFSIZE)));
			m_brightness[i] = value&0xFF;
			m_contrast[i] =  (value>>8)&0xFF;
			m_hue[i] = (value>>16)&0xFF;
			m_saturation[i] = (value>>24)&0xFF;
			//DbgPrint("[MV]value[%d]= %X\n",i,value);
			if((g_contrast[i] != m_contrast[i])||(g_brightness[i] != m_brightness[i])||(g_saturation[i] != m_saturation[i])||(g_hue[i] != m_hue[i]))
			{
				
				//DbgPrint("[MV]m_brightness[%d]= %d %d \n",i,m_brightness[i],g_brightness[i]);
				//DbgPrint("[MV]m_contrast[%d]= %d %d\n",i,m_contrast[i],g_contrast[i]);
				//DbgPrint("[MV]m_hue[%d]= %d %d \n",i,m_hue[i],g_hue[i]);
				//DbgPrint("[MV]m_saturation[%d]= %d %d \n",i,m_saturation[i],g_saturation[i] );
				SetData = g_saturation[i]<<24;
				SetData  |=g_hue[i]<<16;
				SetData  |=g_contrast[i]<<8;
				SetData  |=g_brightness[i];
				//DbgPrint("[MV]value[%d]= %X %X\n",i,value,SetData);
				WriteDevReg((DWORD)(CVBS_IN_BASE + ((91+i*2)*PCIE_BARADDROFSIZE)), SetData);
				
			}
#endif
		pdx->m_curr_No_Video[i] = Get_Video_Status(pdx, i);
		//---------------
		if ((pdx->m_curr_No_Video[i] == 0x1) &&
		    (pdx->m_bVCapStarted[i] == TRUE)) {
			//printk("[MV]check NoVideo End: [%d]\n",i);
			queue_work(pdx->wq, &pdx->video[i].videowork);
		}
		//-----------------
	}
}

void InitVideoSys(struct hws_pcie_dev *pdx, int set)
{
	// init decoder
	int i, j;
	//	DWORD dwRest=0;
	DWORD m_Valude;

	/* If we’ve already started running and set==0, do nothing. */
	if (pdx->m_bStartRun && (set == 0))
		return;

	WRITE_REGISTER_ULONG(pdx, HWS_REG_DEC_MODE, 0x00);
	SetDMAAddress(pdx);
	if (set == 0) {
		for (i = 0; i < pdx->m_nMaxChl; i++) {
			for (j = 0; j < MAX_VIDEO_QUEUE; j++) {
				pdx->m_pVCAPStatus[i][j].byLock = MEM_UNLOCK;
				pdx->m_pVCAPStatus[i][j].byPath = 2;
				pdx->m_pVCAPStatus[i][j].byField = 0;
				pdx->m_pVCAPStatus[i][j].dwinterlace = 0;
			}
			//pdx->m_nVideoIndex[i] =0;
			pdx->m_nAudioBufferIndex[i] = 0;
			EnableVideoCapture(pdx, i, 0);
			EnableAudioCapture(pdx, i, 0);
		}
	}

	WRITE_REGISTER_ULONG(pdx, INT_EN_REG_BASE, 0x3ffff);
	/* ── 5.  “Start run”: set bit 31 of decoder/register, then clear lower 24 bits ── */
	WRITE_REGISTER_ULONG(pdx, HWS_REG_DEC_MODE, 0x80000000);
	//DelayUs(500);
	m_Valude = 0x00FFFFFF;
	m_Valude |= 0x80000000;
	WRITE_REGISTER_ULONG(pdx, HWS_REG_DEC_MODE, m_Valude);
	WRITE_REGISTER_ULONG(pdx, HWS_REG_DEC_MODE, 0X13);
	pdx->m_bStartRun = 1;
	//--------------------------------------------------
}

int Get_Video_Status(struct hws_pcie_dev *pdx, unsigned int ch)
{
	u32 reg;
	int res_w = 0, res_h = 0;
	int out_res_w = 0, out_res_h = 0;
	int frame_rate = 0, out_frame_rate = 0;
	int active_video, interlace, no_video = 1, video_hdcp;
	u8 br, co, hu, sa;
	int out_val;

	/* ── 0. HPD / +5 V status ------------------------------------------------- */
	{
		u32 hpd =
			hws_read_port_hpd(pdx, ch); /* ch == HDMI jack index */
		bool power = !!(hpd & HWS_5V_BIT);
		bool hpd_hi = !!(hpd & HWS_HPD_BIT);

		/* cache & push to ctrl core only on change */
		if (power != pdx->video[ch].detect_tx_5v_ctrl->cur.val) {
			v4l2_ctrl_s_ctrl(pdx->video[ch].detect_tx_5v_ctrl, power);
		}
	}

	/* ── 1. signal present / interlace flags ─────────────────────────── */
	reg = READ_REGISTER_ULONG(pdx, HWS_REG_ACTIVE_STATUS);
	active_video = (reg >> ch) & 0x01;
	interlace = ((reg >> 8) >> ch) & 0x01;

	if (!active_video)
		return 1; /* No signal on this channel */

	no_video = 0; /* we do have video           */

	/* ── 2. device-specific path (HW rev > 0) ─────────────────────────── */
	if (pdx->m_DeviceHW_Version > 0) {
		/* input frame-rate */
		frame_rate = READ_REGISTER_ULONG(pdx, HWS_REG_FRAME_RATE(ch));
		if (frame_rate != pdx->m_pVCAPStatus[ch][0].dwFrameRate)
			pdx->m_pVCAPStatus[ch][0].dwFrameRate = frame_rate;

		/* programmed output resolution */
		reg = READ_REGISTER_ULONG(pdx, HWS_REG_OUT_RES(ch));
		out_res_w = reg & 0xFFFF;
		out_res_h = (reg >> 16) & 0xFFFF;

		if (out_res_w != pdx->m_pVCAPStatus[ch][0].dwOutWidth ||
		    out_res_h != pdx->m_pVCAPStatus[ch][0].dwOutHeight) {
			out_val = pdx->m_pVCAPStatus[ch][0].dwOutHeight;
			out_val = (out_val << 16) |
				  pdx->m_pVCAPStatus[ch][0].dwOutWidth;
			WRITE_REGISTER_ULONG(pdx, HWS_REG_OUT_RES(ch), out_val);
		}

		/* programmed output fps */
		out_frame_rate =
			READ_REGISTER_ULONG(pdx, HWS_REG_OUT_FRAME_RATE(ch));
		if (out_frame_rate != pdx->m_pVCAPStatus[ch][0].dwOutFrameRate)
			WRITE_REGISTER_ULONG(
				pdx, HWS_REG_OUT_FRAME_RATE(ch),
				pdx->m_pVCAPStatus[ch][0].dwOutFrameRate);

		/* BCHS controls packed B|C|H|S */
		reg = READ_REGISTER_ULONG(pdx, HWS_REG_BCHS(ch));
		br = reg & 0xFF;
		co = (reg >> 8) & 0xFF;
		hu = (reg >> 16) & 0xFF;
		sa = (reg >> 24) & 0xFF;

		if (br != pdx->m_brightness[ch] || co != pdx->m_contrast[ch] ||
		    hu != pdx->m_hue[ch] || sa != pdx->m_saturation[ch]) {
			out_val = pdx->m_saturation[ch];
			out_val = (out_val << 8) | pdx->m_hue[ch];
			out_val = (out_val << 8) | pdx->m_contrast[ch];
			out_val = (out_val << 8) | pdx->m_brightness[ch];
			WRITE_REGISTER_ULONG(pdx, HWS_REG_BCHS(ch), out_val);
		}

		/* HDCP detect bit */
		reg = READ_REGISTER_ULONG(pdx, HWS_REG_HDCP_STATUS);
		video_hdcp = (reg >> ch) & 0x01;
		pdx->m_pVCAPStatus[ch][0].dwhdcp = video_hdcp;

	} else { /* ── 3. legacy SW fps estimator ────────────────────────── */
		if (pdx->m_dwSWFrameRate[ch] > 10) {
			int fps = 60;
			if (pdx->m_dwSWFrameRate[ch] > 55 * 2)
				fps = 60;
			else if (pdx->m_dwSWFrameRate[ch] > 45 * 2)
				fps = 50;
			else if (pdx->m_dwSWFrameRate[ch] > 25 * 2)
				fps = 30;
			else if (pdx->m_dwSWFrameRate[ch] > 20 * 2)
				fps = 25;
			pdx->m_pVCAPStatus[ch][0].dwFrameRate = fps;
		}
		pdx->m_dwSWFrameRate[ch] = 0;
	}

	/* ── 4. live input resolution check ──────────────────────────────── */
	reg = READ_REGISTER_ULONG(pdx, HWS_REG_IN_RES(ch));
	res_w = reg & 0xFFFF;
	res_h = (reg >> 16) & 0xFFFF;

	if (((res_w <= MAX_VIDEO_HW_W) && (res_h <= MAX_VIDEO_HW_H) &&
	     !interlace) ||
	    ((res_w <= MAX_VIDEO_HW_W) && (res_h * 2 <= MAX_VIDEO_HW_H) &&
	     interlace)) {
		if (res_w != pdx->m_pVCAPStatus[ch][0].dwWidth ||
		    res_h != pdx->m_pVCAPStatus[ch][0].dwHeight ||
		    interlace != pdx->m_pVCAPStatus[ch][0].dwinterlace) {
			ChangeVideoSize(pdx, ch, res_w, res_h, interlace);
		}
	}

	return no_video; /* 0 = OK, 1 = no signal */
}

int MemCopyVideoToSteam(struct hws_pcie_dev *pdx, int nDecoder)
{
	int nIndex = -1;
	//int i=0 ;
	int status = -1;
	BYTE *bBuf = NULL;
	BYTE *pSrcBuf = NULL;
	BYTE *pDmaSrcBuf = NULL;
	int dwSrcPitch;
	//int dwMaskPitch;
	int copysize;
	int nw, nh;
	int interlace;
	int mVideoBufIndex;
	int halfsize;
	int *pMask;
	int line_cnt = 0;
	unsigned long flags;

	nw = pdx->m_pVCAPStatus[nDecoder][0].dwWidth;
	nh = pdx->m_pVCAPStatus[nDecoder][0].dwHeight;
	if ((nw < 0) || (nh < 0) || (nw > MAX_VIDEO_HW_W) ||
	    (nh > MAX_VIDEO_HW_H)) {
		return -1;
	}
	mVideoBufIndex = pdx->m_nVideoBufferIndex[nDecoder];
	interlace = pdx->m_pVCAPStatus[nDecoder][0].dwinterlace;
	if (pdx->m_Device_SupportYV12 == 1) {
		dwSrcPitch = nw * 12 / 8;
	} else {
		dwSrcPitch = nw * 2;
	}
	if (mVideoBufIndex == 1) {
		pDmaSrcBuf = pdx->m_pbyVideoBuffer[nDecoder];
		copysize = pdx->m_format[nDecoder].HLAF_SIZE;
		line_cnt = copysize / dwSrcPitch;
		halfsize = copysize;
	} else {
		copysize = pdx->m_format[nDecoder].HLAF_SIZE;
		pDmaSrcBuf = pdx->m_pbyVideoBuffer[nDecoder] + copysize;
		halfsize = copysize;
		copysize = pdx->m_format[nDecoder].DWON_SIZE;
		line_cnt = copysize / dwSrcPitch;
	}
	pMask = (int *)(pDmaSrcBuf + (copysize - (line_cnt - 2) * dwSrcPitch));
	if (*pMask == 0x55AAAA55) {
		//DbgPrint("########-*pMask-[ch=%d][index=%d]Mark=%X[%d-%d]-[%d,%d]\n",nDecoder, mVideoBufIndex,*pMask,nw,nh,line_cnt,copysize);
		//------------------------------
		if (pdx->m_nVideoHalfDone[nDecoder] == 1) {
			pdx->m_nVideoHalfDone[nDecoder] = 0;
		}
		//------------------------------
		return -1;
	} else {
		if (mVideoBufIndex == 0) {
			if (pdx->m_nVideoHalfDone[nDecoder] == 0) {
				//DbgPrint("X1:HLAF ########-*pMask- [%d] [%d]\n",nDecoder,mVideoBufIndex);
				*pMask = 0x55AAAA55;
				return -1;
			} else {
				pdx->m_nVideoHalfDone[nDecoder] = 0;
			}
		} else {
			pdx->m_nVideoHalfDone[nDecoder] = 1;
		}
	}
	//-------------------------------
	if (pdx->m_VideoInfo[nDecoder].dwisRuning == 1) {
		nIndex = -1;
		pSrcBuf = pDmaSrcBuf;
		bBuf = NULL;
		if (mVideoBufIndex == 1) {
			if (pdx->m_VideoInfo[nDecoder]
				    .pStatusInfo[pdx->m_VideoInfo[nDecoder]
							 .m_nVideoIndex]
				    .byLock == MEM_UNLOCK) {
				nIndex =
					pdx->m_VideoInfo[nDecoder].m_nVideoIndex;
				bBuf = pdx->m_VideoInfo[nDecoder]
					       .m_pVideoBufData[nIndex];
			}
		} else {
			nIndex = pdx->m_VideoInfo[nDecoder].m_nVideoIndex;
			bBuf = pdx->m_VideoInfo[nDecoder]
				       .m_pVideoBufData[nIndex];
		}
		if (nIndex == -1) {
			pdx->m_VideoInfo[nDecoder]
				.pStatusInfo[pdx->m_VideoInfo[nDecoder]
						     .m_nVideoIndex]
				.byLock = MEM_UNLOCK;
			nIndex = pdx->m_VideoInfo[nDecoder].m_nVideoIndex;
			bBuf = pdx->m_VideoInfo[nDecoder]
				       .m_pVideoBufData[nIndex];
		}
		if (nIndex != -1 && bBuf) {
			if (mVideoBufIndex == 0) {
				bBuf += halfsize;
			}
			//pci_dma_sync_single_for_cpu(pdx->pdev,pdx->m_pbyVideo_phys[nDecoder],pdx->m_MaxHWVideoBufferSize,2);
			dma_sync_single_for_cpu(&pdx->pdev->dev,
						pdx->m_pbyVideo_phys[nDecoder],
						pdx->m_MaxHWVideoBufferSize, 2);
			memcpy(bBuf, pSrcBuf, copysize);

			if (mVideoBufIndex == 0) {
				status = 0;
				spin_lock_irqsave(&pdx->videoslock[nDecoder],
						  flags);

				pdx->m_VideoInfo[nDecoder].m_nVideoIndex =
					nIndex + 1;
				if (pdx->m_VideoInfo[nDecoder].m_nVideoIndex >=
				    MAX_VIDEO_QUEUE) {
					pdx->m_VideoInfo[nDecoder]
						.m_nVideoIndex = 0;
				}
				pdx->m_VideoInfo[nDecoder]
					.pStatusInfo[nIndex]
					.dwWidth =
					pdx->m_pVCAPStatus[nDecoder][0].dwWidth;
				pdx->m_VideoInfo[nDecoder]
					.pStatusInfo[nIndex]
					.dwHeight =
					pdx->m_pVCAPStatus[nDecoder][0].dwHeight;
				pdx->m_VideoInfo[nDecoder]
					.pStatusInfo[nIndex]
					.dwinterlace = interlace;
				pdx->m_VideoInfo[nDecoder]
					.pStatusInfo[nIndex]
					.byLock = MEM_LOCK;

				spin_unlock_irqrestore(
					&pdx->videoslock[nDecoder], flags);
			}

		} else {
			//printk("No Buffer Write %d",nDecoder);
			//queue_work(pdx->wq,&pdx->video[nDecoder].videowork);
			pdx->m_nVideoHalfDone[nDecoder] = 0;
		}
	}
	*pMask = 0x55AAAA55;
	return 0;
}

void video_data_process(struct work_struct *p_work)
{
	struct hws_video *videodev =
		container_of(p_work, struct hws_video, videowork);
	struct hwsvideo_buffer *buf;
	//unsigned long flags;
	unsigned long devflags;
	int nVindex = -1;
	int i;
	//int copysize;
	BYTE *bBuf;
	int in_width;
	int in_height;
	int in_vsize;
	int out_size = 0;
	int nDecoder;
	int nCopySize;
	int interlace = 0;
	int needRotateVideo = 0;
	int nWidth;
	int nHeight;
	struct hws_pcie_dev *pdx = videodev->dev;
	nDecoder = videodev->index;

	spin_lock_irqsave(&pdx->videoslock[nDecoder], devflags);
	in_width = pdx->m_pVCAPStatus[nDecoder][0].dwWidth;
	in_height = pdx->m_pVCAPStatus[nDecoder][0].dwHeight;
	nWidth = videodev->current_out_width;
	nHeight = videodev->curren_out_height;
	out_size =
		videodev->current_out_width * videodev->curren_out_height * 2;
	if (pdx->m_Device_SupportYV12 == 1) {
		nCopySize = ((in_width * 12 * in_height) / 8);
	} else if (pdx->m_Device_SupportYV12 == 2) {
		nCopySize = ((in_width * 5 * in_height) / 4);
	} else {
		nCopySize = (in_width * in_height * 2);
	}

	if (pdx->m_pVCAPStatus[nDecoder][0].dwinterlace == 1) {
		in_height = in_height * 2;
	}
	in_vsize = in_width * in_height * 2;
	if ((in_width == 1280) && (in_height == 720) && (nWidth == 1080) &&
	    (nHeight == 1920)) {
		needRotateVideo = 1;
	} else if ((in_width == 960) && (in_height == 540) &&
		   (nWidth == 1080) && (nHeight == 1920)) {
		needRotateVideo = 1;
	}
	//printk("video_data_process [%d]dev->m_curr_No_Video[videodev->index] =%d \n",videodev->index,dev->m_curr_No_Video[videodev->index]);
	//---------------------------
	bBuf = NULL;
	if (pdx->m_curr_No_Video[nDecoder] == 0) {
		nVindex = -1;
		for (i = pdx->m_nRDVideoIndex[nDecoder]; i < MAX_VIDEO_QUEUE;
		     i++) {
			if (pdx->m_VideoInfo[nDecoder].pStatusInfo[i].byLock ==
			    MEM_LOCK) {
				nVindex = i;
				bBuf = pdx->m_VideoInfo[nDecoder]
					       .m_pVideoBufData[i];
				interlace = pdx->m_VideoInfo[nDecoder]
						    .pStatusInfo[i]
						    .dwinterlace;
				break;
			}
		}
		if (nVindex == -1) {
			//printk("video_data_process no data find [%d]\n",videodev->index);
			spin_unlock_irqrestore(&pdx->videoslock[nDecoder],
					       devflags);
			return;
		}
		if (bBuf == NULL) {
			//printk("video_data_process pSrc == NULL [%d]\n",videodev->index);
			spin_unlock_irqrestore(&pdx->videoslock[nDecoder],
					       devflags);
			return;
		}
	} else {
		//spin_lock_irqsave(&pdx->videoslock[nDecoder], devflags);
		for (i = 0; i < MAX_VIDEO_QUEUE; i++) {
			if (pdx->m_VideoInfo[nDecoder].pStatusInfo[i].byLock ==
			    MEM_LOCK) {
				pdx->m_VideoInfo[nDecoder]
					.pStatusInfo[i]
					.byLock = MEM_UNLOCK;
			}
		}
		//spin_unlock_irqrestore(&pdx->videoslock[nDecoder], devflags);
	}
	//---------------------------
	//spin_lock_irqsave(&videodev->slock, flags);
	if (list_empty(&videodev->queue)) {
		//spin_unlock_irqrestore(&videodev->slock, flags);
		//printk( "%s(%d)->%d\n", __func__,videodev->index,videodev->fileindex);
		goto vexit;
	}

	buf = list_entry(videodev->queue.next, struct hwsvideo_buffer, queue);
	list_del(&buf->queue);

	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	//buf->vb.field = videodev->pixfmt;
	buf->vb.field = V4L2_FIELD_NONE;
	if (buf->mem) {
		//----------------------
		// copy data to buffer
		if (pdx->m_curr_No_Video[nDecoder] == 0) {
			//--------------------
			if (pdx->m_Device_SupportYV12 == 1) {
				if ((in_vsize != out_size)) {
					if (pdx->m_VideoInfo[nDecoder]
						    .m_pVideoScalerBuf) {
						if (needRotateVideo == 0) {
							memcpy(pdx->m_VideoInfo[nDecoder]
								       .m_pVideoYUV2Buf,
							       bBuf, nCopySize);
							FillYUU2(
								pdx->m_VideoInfo[nDecoder]
									.m_pVideoYUV2Buf,
								pdx->m_VideoInfo[nDecoder]
									.m_pVideoScalerBuf,
								in_width,
								in_height,
								interlace);
							VideoScaler(
								pdx->m_VideoInfo[nDecoder]
									.m_pVideoScalerBuf,
								buf->mem,
								in_width,
								in_height,
								nWidth,
								nHeight);
						} else {
							if (pdx->m_VideoInfo[nDecoder]
								    .m_pRotateVideoBuf) {
								memcpy(pdx->m_VideoInfo[nDecoder]
									       .m_pVideoYUV2Buf,
								       bBuf,
								       nCopySize);
								FillYUU2(
									pdx->m_VideoInfo[nDecoder]
										.m_pVideoYUV2Buf,
									pdx->m_VideoInfo[nDecoder]
										.m_pVideoScalerBuf,
									in_width,
									in_height,
									interlace);
								VideoScaler(
									pdx->m_VideoInfo[nDecoder]
										.m_pVideoScalerBuf,
									pdx->m_VideoInfo[nDecoder]
										.m_pRotateVideoBuf,
									in_width,
									in_height,
									nWidth,
									nHeight);
								VideoRotate90deg(
									pdx->m_VideoInfo[nDecoder]
										.m_pRotateVideoBuf,
									buf->mem,
									nHeight,
									nWidth,
									nWidth,
									nHeight);
							} else {
								spin_unlock_irqrestore(
									&pdx->videoslock
										 [nDecoder],
									devflags);
								return;
							}
						}
					} else {
						spin_unlock_irqrestore(
							&pdx->videoslock
								 [nDecoder],
							devflags);
						return;
					}
				} else {
					if ((in_width == nHeight) &&
					    (in_height == nWidth)) {
						if (pdx->m_VideoInfo[nDecoder]
							    .m_pRotateVideoBuf) {
							memcpy(pdx->m_VideoInfo[nDecoder]
								       .m_pVideoYUV2Buf,
							       bBuf, nCopySize);
							FillYUU2(
								pdx->m_VideoInfo[nDecoder]
									.m_pVideoYUV2Buf,
								pdx->m_VideoInfo[nDecoder]
									.m_pRotateVideoBuf,
								in_width,
								in_height,
								interlace);
							VideoRotate90deg(
								pdx->m_VideoInfo[nDecoder]
									.m_pRotateVideoBuf,
								buf->mem,
								nHeight, nWidth,
								nWidth,
								nHeight);
						} else {
							spin_unlock_irqrestore(
								&pdx->videoslock
									 [nDecoder],
								devflags);
							return;
						}
					} else {
						memcpy(pdx->m_VideoInfo[nDecoder]
							       .m_pVideoYUV2Buf,
						       bBuf, nCopySize);
						FillYUU2(
							pdx->m_VideoInfo[nDecoder]
								.m_pVideoYUV2Buf,
							buf->mem, in_width,
							in_height, interlace);
					}
				}
			} else if (pdx->m_Device_SupportYV12 == 2) {
				if ((in_vsize != out_size)) {
					if (pdx->m_VideoInfo[nDecoder]
						    .m_pVideoScalerBuf) {
						if (needRotateVideo == 0) {
							memcpy(pdx->m_VideoInfo[nDecoder]
								       .m_pVideoYUV2Buf,
							       bBuf, nCopySize);
							FillNV12ToYUY2(
								pdx->m_VideoInfo[nDecoder]
									.m_pVideoYUV2Buf,
								pdx->m_VideoInfo[nDecoder]
									.m_pVideoScalerBuf,
								in_width,
								in_height,
								interlace);
							VideoScaler(
								pdx->m_VideoInfo[nDecoder]
									.m_pVideoScalerBuf,
								buf->mem,
								in_width,
								in_height,
								nWidth,
								nHeight);
						} else {
							if (pdx->m_VideoInfo[nDecoder]
								    .m_pRotateVideoBuf) {
								memcpy(pdx->m_VideoInfo[nDecoder]
									       .m_pVideoYUV2Buf,
								       bBuf,
								       nCopySize);
								FillNV12ToYUY2(
									pdx->m_VideoInfo[nDecoder]
										.m_pVideoYUV2Buf,
									pdx->m_VideoInfo[nDecoder]
										.m_pVideoScalerBuf,
									in_width,
									in_height,
									interlace);
								VideoScaler(
									pdx->m_VideoInfo[nDecoder]
										.m_pVideoScalerBuf,
									pdx->m_VideoInfo[nDecoder]
										.m_pRotateVideoBuf,
									in_width,
									in_height,
									nWidth,
									nHeight);
								VideoRotate90deg(
									pdx->m_VideoInfo[nDecoder]
										.m_pRotateVideoBuf,
									buf->mem,
									nHeight,
									nWidth,
									nWidth,
									nHeight);
							} else {
								spin_unlock_irqrestore(
									&pdx->videoslock
										 [nDecoder],
									devflags);
								return;
							}
						}
					} else {
						spin_unlock_irqrestore(
							&pdx->videoslock
								 [nDecoder],
							devflags);
						return;
					}
				} else {
					if ((in_width == nHeight) &&
					    (in_height == nWidth)) {
						if (pdx->m_VideoInfo[nDecoder]
							    .m_pRotateVideoBuf) {
							memcpy(pdx->m_VideoInfo[nDecoder]
								       .m_pVideoYUV2Buf,
							       bBuf, nCopySize);
							FillNV12ToYUY2(
								pdx->m_VideoInfo[nDecoder]
									.m_pVideoYUV2Buf,
								pdx->m_VideoInfo[nDecoder]
									.m_pRotateVideoBuf,
								in_width,
								in_height,
								interlace);
							VideoRotate90deg(
								pdx->m_VideoInfo[nDecoder]
									.m_pRotateVideoBuf,
								buf->mem,
								nHeight, nWidth,
								nWidth,
								nHeight);
						} else {
							spin_unlock_irqrestore(
								&pdx->videoslock
									 [nDecoder],
								devflags);
							return;
						}
					} else {
						memcpy(pdx->m_VideoInfo[nDecoder]
							       .m_pVideoYUV2Buf,
						       bBuf, nCopySize);
						FillNV12ToYUY2(
							pdx->m_VideoInfo[nDecoder]
								.m_pVideoYUV2Buf,
							buf->mem, in_width,
							in_height, interlace);
					}
				}
			} else {
				if ((in_vsize != out_size)) {
					if (pdx->m_VideoInfo[nDecoder]
						    .m_pVideoScalerBuf) {
						if (needRotateVideo == 0) {
							//RtlCopyMemory(m_pVideoScalerBuf,bBuf,nCopySize);
							SetDeInterlace(
								bBuf,
								pdx->m_VideoInfo[nDecoder]
									.m_pVideoScalerBuf,
								in_width,
								in_height,
								interlace);
							VideoScaler(
								pdx->m_VideoInfo[nDecoder]
									.m_pVideoScalerBuf,
								buf->mem,
								in_width,
								in_height,
								nWidth,
								nHeight);
						} else {
							SetDeInterlace(
								bBuf,
								pdx->m_VideoInfo[nDecoder]
									.m_pVideoScalerBuf,
								in_width,
								in_height,
								interlace);
							VideoScaler(
								pdx->m_VideoInfo[nDecoder]
									.m_pVideoScalerBuf,
								pdx->m_VideoInfo[nDecoder]
									.m_pRotateVideoBuf,
								in_width,
								in_height,
								nHeight,
								nWidth);
							VideoRotate90deg(
								pdx->m_VideoInfo[nDecoder]
									.m_pRotateVideoBuf,
								buf->mem,
								nHeight, nWidth,
								nWidth,
								nHeight);
						}
					} else {
						spin_unlock_irqrestore(
							&pdx->videoslock
								 [nDecoder],
							devflags);
						return;
					}
				} else {
					if ((in_width == nHeight) &&
					    (in_height == nWidth)) {
						if (pdx->m_VideoInfo[nDecoder]
							    .m_pRotateVideoBuf) {
							SetDeInterlace(
								bBuf,
								pdx->m_VideoInfo[nDecoder]
									.m_pRotateVideoBuf,
								in_width,
								in_height,
								interlace);
							VideoRotate90deg(
								pdx->m_VideoInfo[nDecoder]
									.m_pRotateVideoBuf,
								buf->mem,
								nHeight, nWidth,
								nWidth,
								nHeight);
						} else {
							spin_unlock_irqrestore(
								&pdx->videoslock
									 [nDecoder],
								devflags);
							return;
						}
					} else {
						SetDeInterlace(bBuf, buf->mem,
							       in_width,
							       in_height,
							       interlace);
					}
				}
			}
		} else {
			SetNoVideoMem(buf->mem, videodev->current_out_width,
				      videodev->curren_out_height);
		}
	}

	//----------------------------------------
	buf->vb.sequence = videodev->seqnr++;
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	//printk("vb2_buffer_done [%d]\n",videodev->index);
	//spin_unlock_irqrestore(&videodev->slock, flags);
vexit:
	//spin_lock_irqsave(&pdx->videoslock[nDecoder], devflags);
	if (pdx->m_curr_No_Video[nDecoder] == 0) {
		pdx->m_VideoInfo[nDecoder].pStatusInfo[nVindex].byLock =
			MEM_UNLOCK;
		pdx->m_nRDVideoIndex[nDecoder] = nVindex + 1;
		if (pdx->m_nRDVideoIndex[nDecoder] >= MAX_VIDEO_QUEUE) {
			pdx->m_nRDVideoIndex[nDecoder] = 0;
		}
	}
	spin_unlock_irqrestore(&pdx->videoslock[nDecoder], devflags);
	return;
}
