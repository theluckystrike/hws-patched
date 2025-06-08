
int ReadChipId(struct hws_pcie_dev *pdx)
{
	//  CCIR_PACKET      reg;
	//int Chip_id1 = 0;
	int ret = 0;
	//int reg_vaule = 0;
	//int nResult;
	int i;
	//------read Dvice Version
	ULONG m_dev_ver;
	ULONG m_tmpVersion;
	ULONG m_tmpHWKey;
	//ULONG m_OEM_code_data;
	m_dev_ver = READ_REGISTER_ULONG(pdx, HWS_REG_DEVICE_INFO);

	/* Bits 7:0   = device version */
	m_tmpVersion = m_dev_ver >> 8;
	pdx->m_Device_Version = (m_tmpVersion & 0xFF);

	/* Bits 15:8  = device subversion */
	m_tmpVersion = m_dev_ver >> 16;
	pdx->m_Device_SubVersion = (m_tmpVersion & 0xFF);

	/* Bits 31:28 = YV12 support flags (4 bits) */
	pdx->m_Device_SupportYV12 = ((m_dev_ver >> 28) & 0x0F);

	/* Bits 27:24 = HW key; low two bits of that = port ID */
	m_tmpHWKey = (m_dev_ver >> 24) & 0x0F;
	pdx->m_Device_PortID = (m_tmpHWKey & 0x03);

	//n_VideoModle =	READ_REGISTER_ULONG(pdx,0x4000+(4*PCIE_BARADDROFSIZE));
	//n_VideoModle = (n_VideoModle>>8)&0xFF;
	//pdx->m_IsHDModel = 1;
	pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
	pdx->m_nMaxChl = 4;
	pdx->m_bBufferAllocate = FALSE;
	pdx->mMain_tsk = NULL;
	pdx->m_dwAudioPTKSize = MAX_DMA_AUDIO_PK_SIZE; //128*16*4;
	pdx->m_bStartRun = 0;
	pdx->m_PciDeviceLost = 0;

	//--------
	for (i = 0; i < MAX_VID_CHANNELS; i++) {
		SetVideoFormteSize(pdx, i, 1920, 1080);
	}
	//-------
	WRITE_REGISTER_ULONG(pdx, HWS_REG_DEC_MODE, 0x0);
	//ssleep(100);
	WRITE_REGISTER_ULONG(pdx, HWS_REG_DEC_MODE, 0x10);
	//ssleep(500);
	//-------
	SetHardWareInfo(pdx);
	printk("************[HW]-[VIDV]=[%d]-[%d]-[%d] ************\n",
	       pdx->m_Device_Version, pdx->m_Device_SubVersion,
	       pdx->m_Device_PortID);
	return ret;
}

void SetHardWareInfo(struct hws_pcie_dev *pdx)
{
	switch (pdx->dwDeviceID) {
	case 0x9534:
	case 0x6524:
	case 0x8524: {
		pdx->m_nCurreMaxVideoChl = 4;
		pdx->m_nCurreMaxLineInChl = 1;
		pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
		break;
	}
	case 0x8532: {
		pdx->m_nCurreMaxVideoChl = 2;
		pdx->m_nCurreMaxLineInChl = 1;
		pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
		break;
	}
	case 0x8512:
	case 0x6502: {
		pdx->m_nCurreMaxVideoChl = 2;
		pdx->m_nCurreMaxLineInChl = 0;
		pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
		break;
	}
	case 0x8501: {
		pdx->m_nCurreMaxVideoChl = 1;
		pdx->m_nCurreMaxLineInChl = 0;
		pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
		break;
	}
	default: {
		pdx->m_nCurreMaxVideoChl = 4;
		pdx->m_nCurreMaxLineInChl = 0;
		pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
		break;
	}
	}
	//-----------------------
	if (pdx->m_Device_Version > 121) {
		if ((pdx->dwDeviceID == 0x8501) &&
		    (pdx->m_Device_Version == 122)) {
			pdx->m_DeviceHW_Version = 0;
		} else {
			//DWORD m_ReadTmp;
			pdx->m_DeviceHW_Version = 1;
			//--- set DMA_MAX_SIZE
			WRITE_REGISTER_ULONG(pdx, HWS_REG_DMA_MAX_SIZE,
					     (MAX_VIDEO_SCLAER_SIZE / 16));
			//m_ReadTmp = ReadDevReg((DWORD)(CVBS_IN_BASE + (9 * PCIE_BARADDROFSIZE)));
			//DbgPrint("[MV]DMA_MAX_SIZE =%d\n",m_ReadTmp);
		}
	} else {
		pdx->m_DeviceHW_Version = 0;
	}
	//------------------
}

//---------------------------------------
void CheckCardStatus(struct hws_pcie_dev *pdx)
{
	ULONG status;
	status = READ_REGISTER_ULONG(pdx, HWS_REG_SYS_STATUS);

	//DbgPrint("CheckCardStatus =%X",status);
	if ((status & BIT(0)) != BIT(0)) {
		//DbgPrint("CheckCardStatus =%X",status);
		InitVideoSys(pdx, 1);
	}
}

static void hws_get_video_param(struct hws_pcie_dev *dev, int index)
{
	//printk( "%s(): %x \n", __func__, index);
	int width, height;
	width = dev->m_pVCAPStatus[index][0].dwWidth;
	height = dev->m_pVCAPStatus[index][0].dwHeight;
	dev->video[index].current_out_pixfmt = 0;
	dev->video[index].current_out_size_index = 0;
	dev->video[index].current_out_width = width;
	dev->video[index].curren_out_height = height;
	dev->video[index].current_out_framerate = 60;
	dev->video[index].Interlaced = 0;
	//printk( "%s(%dx%d):  \n", __func__, width,height);
}

void hws_adapters_init(struct hws_pcie_dev *dev)
{
	int i;
	for (i = 0; i < MAX_VID_CHANNELS; i++) {
		hws_get_video_param(dev, i);
	}
}

void StopDevice(struct hws_pcie_dev *pdx)
{ // StopDevice
	//Trace t("StopDevice()");
	int i;
	//int   device_lost =0;
	u32 statusreg;
	StopDsp(pdx);
	statusreg = READ_REGISTER_ULONG(pdx, (0x4000));
	//DbgPrint("[MV] Busy!!! statusreg =%X\n", statusreg);
	if (statusreg != 0xFFFFFFFF) {
		//set to one buffer mode
		//WRITE_REGISTER_ULONG((u32)(CVBS_IN_BASE + (25*PCIE_BARADDROFSIZE)), 0x00); //Buffer 1 address
	} else {
		pdx->m_PciDeviceLost = 1;
	}
	pdx->m_bStartRun = 0;
	if (pdx->m_PciDeviceLost == 0) {
		for (i = 0; i < MAX_VID_CHANNELS; i++) {
			EnableVideoCapture(pdx, i, 0);
			EnableAudioCapture(pdx, i, 0);
		}
	}
	//if(device_lost) return;
	DmaMemFreePool(pdx);
	//printk("StopDevice Done\n");
}
