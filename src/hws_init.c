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


//---------------------------------------
