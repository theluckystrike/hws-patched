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


//---------------------------------------
