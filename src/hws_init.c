/* SPDX-License-Identifier: GPL-2.0-only */
#include "hws_init.h"
#include "hws_reg.h"
#include "hws_pci.h"
#include "hws_video_pipeline.h"
#include "hws_audio_pipeline.h"
#include "hws_dma.h"

static int Check_Busy(struct hws_pcie_dev *pdx)
{
	u32 statusreg;
	u32 TimeOut = 0;
	//DbgPrint(("Check Busy in !!!\n"));
	//WRITE_REGISTER_ULONG((u32)(0x4000), 0x10);
	while (1) {
		statusreg = READ_REGISTER_ULONG(pdx, HWS_REG_SYS_STATUS);
		printk("[MV] Check_Busy!!! statusreg =%X\n", statusreg);
		if (statusreg == 0xFFFFFFFF) {
			break;
		}
		if ((statusreg & HWS_SYS_DMA_BUSY_BIT) == 0) {
			break;
		}
		TimeOut++;
		msleep(10);
	}
	//WRITE_REGISTER_ULONG((u32)(0x4000), 0x10);

	//DbgPrint(("Check Busy out !!!\n"));

	return 0;
}

static void StopDsp(struct hws_pcie_dev *pdx)
{
	//int j, i;
	u32 statusreg;
	statusreg = READ_REGISTER_ULONG(pdx, HWS_REG_DEC_MODE);
	printk("[MV] Busy!!! statusreg =%X\n", statusreg);
	if (statusreg == 0xFFFFFFFF) {
		return;
	}
	WRITE_REGISTER_ULONG(pdx, HWS_REG_DEC_MODE, 0x10);
	Check_Busy(pdx);
	WRITE_REGISTER_ULONG(pdx, HWS_REG_VCAP_ENABLE, 0x00);
}

//----------------------------------------------

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
	dma_mem_free_pool(pdx);
	//printk("StopDevice Done\n");
}
