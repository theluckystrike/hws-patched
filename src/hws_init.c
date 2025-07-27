/* SPDX-License-Identifier: GPL-2.0-only */
#include "hws_init.h"
#include "hws_reg.h"
#include "hws_pci.h"
#include "hws_video_pipeline.h"
#include "hws_audio_pipeline.h"
#include "hws_dma.h"

/* how often to poll (in µs), and total timeout (in µs) */
#define HWS_BUSY_POLL_DELAY_US   10
#define HWS_BUSY_POLL_TIMEOUT_US 1000000


/**
 * hws_check_sys_busy() – wait for SYS_STATUS busy bit to clear
 * @pdx: our PCIe device state, with mmio_base already ioremap’ed
 *
 * Returns 0 if the busy bit cleared in time, or –ETIMEDOUT on timeout.
 */
static int hws_check_sys_busy(struct hws_pcie_dev *pdx)
{
    void __iomem *reg = pdx->mmio_base + HWS_REG_SYS_STATUS;
    u32 val;
    int ret;

    /* poll until !(val & BUSY_BIT), sleeping HWS_BUSY_POLL_DELAY_US between reads */
    ret = readl_poll_timeout(reg, val,
                             !(val & HWS_SYS_DMA_BUSY_BIT),
                             HWS_BUSY_POLL_DELAY_US,
                             HWS_BUSY_POLL_TIMEOUT_US);
    if (ret) {
        dev_err(&pdx->pdev->dev,
                "SYS_STATUS busy bit never cleared (0x%08x)\n", val);
        return -ETIMEDOUT;
    }

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
	// FIXME: not reading return result for -ETIMEDOUT
	Check_Busy(pdx);
	WRITE_REGISTER_ULONG(pdx, HWS_REG_VCAP_ENABLE, 0x00);
}


//---------------------------------------
void check_card_status(struct hws_pcie_dev *pdx)
{
	u32 status;
	status = READ_REGISTER_ULONG(pdx, HWS_REG_SYS_STATUS);

	if ((status & BIT(0)) != BIT(0)) {
		InitVideoSys(pdx, 1);
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
