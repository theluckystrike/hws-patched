/* SPDX-License-Identifier: GPL-2.0-only */
#include "hws_interrupt.h"
#include "hws_pci.h"
#include "hws_reg.h"
#include "hws_video_pipeline.h"
#include "hws_audio_pipeline.h"

static int SetQueue(struct hws_pcie_dev *pdx, int nDecoder)
{
	int status = -1;
	//KLOCK_QUEUE_HANDLE  oldirql;
	//DbgPrint("SetQueue %d",nDecoder);
	if (!pdx->m_bStartRun) {
		return -1;
	}
	if (!pdx->m_bVCapStarted[nDecoder]) {
		if (pdx->m_bVideoStop[nDecoder] == 1) {
			pdx->m_bVideoStop[nDecoder] = 0;
			//DbgPrint("KeSetEvent Exit Event[%d]\n",nDecoder);
		}

		return -1;
	}
	//-------------------
	if (pdx->m_DeviceHW_Version == 0) {
		pdx->m_dwSWFrameRate[nDecoder]++;
	}
	//-------------------
	pdx->m_nVideoBusy[nDecoder] = 1;
	//-------------------------------
	if (pdx->m_bVCapStarted[nDecoder] == TRUE) {
		status = MemCopyVideoToSteam(pdx, nDecoder);
	}
	pdx->m_nVideoBusy[nDecoder] = 0;
	return status;
}

//-----------------------------
/* Interrupt handler. Read/modify/write the command register to disable
 * the interrupt. */
//static irqreturn_t irqhandler(int irq, struct uio_info *info)
static irqreturn_t irqhandler(int irq, void *info)
{
    struct hws_pcie_dev *pdx = info;
    u32 sys_status = READ_REGISTER_ULONG(pdx, HWS_REG_SYS_STATUS);

    /* No DMA busy or card gone: exit early */
    if ((sys_status & HWS_SYS_DMA_BUSY_BIT) == 0 || sys_status == 0xFFFFFFFF)
        return IRQ_NONE;

    /* Read interrupt status bits */
    u32 int_state = READ_REGISTER_ULONG(pdx, HWS_REG_INT_STATUS);
    if (!int_state)
        return IRQ_NONE;  /* spurious interrupt */

    u32 ack_mask = 0;

    /* Loop until all pending bits are serviced (max 100 iterations) */
    for (u32 cnt = 0; int_state && cnt < 100; ++cnt) {
        /* ── Video channels 0–3 ───────────────────────────────────────── */
        for (int ch = 0; ch < 4; ++ch) {
            u32 vbit = HWS_INT_VDONE_BIT(ch);
            if (!(int_state & vbit))
                continue;

            /* Mark this channel’s capture-done flag */
            pdx->m_bVCapIntDone[ch] = 1;
            ack_mask |= vbit;

            if (pdx->m_nVideoBusy[ch] == 0) {
                /* Read which half of the ring the DMA is writing to */
                u32 toggle = READ_REGISTER_ULONG(pdx, HWS_REG_VBUF_TOGGLE(ch)) & 0x01;

                if (pdx->video_data[ch] != toggle) {
                    pdx->video_data[ch]        = toggle;
                    pdx->m_nVideoBufferIndex[ch] = toggle;
                    tasklet_schedule(&pdx->dpc_video_tasklet[ch]);
                } else {
                    pdx->m_nVideoHalfDone[ch] = 0;
                }
            }
        }

        /* ── Audio channels 0–3 ───────────────────────────────────────── */
        for (int ch = 0; ch < 4; ++ch) {
            u32 abit = HWS_INT_ADONE_BIT(ch);
            if (!(int_state & abit))
                continue;

            ack_mask |= abit;

            if (pdx->m_nAudioBusy[ch] == 0) {
                /* Read which half of the audio ring is active */
                u32 toggle = READ_REGISTER_ULONG(pdx, HWS_REG_ABUF_TOGGLE(ch)) & 0x01;

                pdx->m_nAudioBufferIndex[ch] = toggle;
                pdx->audio_data[ch]         = toggle;
                tasklet_schedule(&pdx->dpc_audio_tasklet[ch]);
            }
        }

        /* Acknowledge (clear) all bits we just handled */
        WRITE_REGISTER_ULONG(pdx, HWS_REG_INT_ACK, ack_mask);

        /* Immediately clear ack_mask to avoid re-acknowledging stale bits */
        ack_mask = 0;

        /* Re‐read in case new interrupt bits popped while processing */
        int_state = READ_REGISTER_ULONG(pdx, HWS_REG_INT_STATUS);
    }

    return IRQ_HANDLED;
}

int hws_irq_setup(struct hws_pcie_dev *lro, struct pci_dev *pdev)
{
	int rc = 0;
	u32 irq_flag;
	u8 val;
	//void *reg;
	//u32 w;

	//BUG_ON(!lro);

	//if (lro->msix_enabled) {
	//	rc = msix_irq_setup(lro);
	//}
	//else
	{
		if (!lro->msi_enabled) {
			pci_read_config_byte(pdev, PCI_INTERRUPT_PIN, &val);
			//printk("Legacy Interrupt register value = %d\n", val);
		}
		//irq_flag = lro->msi_enabled ? 0 : IRQF_SHARED;
		irq_flag = lro->msi_enabled ? IRQF_SHARED : 0;
		//irq_flag = IRQF_SHARED;

		rc = request_irq(pdev->irq, irqhandler, irq_flag,
				 pci_name(pdev), lro); // IRQF_TRIGGER_HIGH
		if (rc) {
			//printk("Couldn't use IRQ#%d, rc=%d\n", pdev->irq, rc);
		} else {
			lro->irq_line = (int)pdev->irq;
			//printk("Using IRQ#%d with  MSI_EN=%d \n", pdev->irq,lro->msi_enabled);
		}
	}

	return rc;
}


void hws_free_irqs(struct hws_pcie_dev *lro)
{
	//int i;

	//BUG_ON(!lro);

	//if (lro->msix_enabled) {
	//	for (i = 0; i < lro->irq_user_count; i++) {
	//		printk("Releasing IRQ#%d\n", lro->entry[i].vector);
	//		free_irq(lro->entry[i].vector, &lro->user_irq[i]);
	//	}
	//}
	//else

	if (lro->irq_line != -1) {
		//printk("Releasing IRQ#%d\n", lro->irq_line);
		free_irq(lro->irq_line, lro);
	}
}


void DpcForIsr_Audio0(unsigned long data)
{
	int index;
	struct hws_pcie_dev *pdx;
	//pdx = sys_dvrs_hw_pdx;
	pdx = (struct hws_pcie_dev *)data;
	//unsigned long *pdata = (unsigned long *)data;
	//curr_buf_index = *pdata;
	index = 0;
	SetAudioQueue(pdx, index);
}

void DpcForIsr_Audio1(unsigned long data)
{
	int index;
	struct hws_pcie_dev *pdx;
	//pdx = sys_dvrs_hw_pdx;
	pdx = (struct hws_pcie_dev *)data;
	//unsigned long *pdata = (unsigned long *)data;
	//curr_buf_index = *pdata;
	index = 1;
	SetAudioQueue(pdx, index);
}

void DpcForIsr_Audio2(unsigned long data)
{
	int index;
	struct hws_pcie_dev *pdx;
	//pdx = sys_dvrs_hw_pdx;
	pdx = (struct hws_pcie_dev *)data;
	//unsigned long *pdata = (unsigned long *)data;
	//curr_buf_index = *pdata;
	index = 2;
	SetAudioQueue(pdx, index);
}

void DpcForIsr_Audio3(unsigned long data)
{
	int index;
	struct hws_pcie_dev *pdx;
	//pdx = sys_dvrs_hw_pdx;
	pdx = (struct hws_pcie_dev *)data;
	//unsigned long *pdata = (unsigned long *)data;
	//curr_buf_index = *pdata;
	index = 3;
	SetAudioQueue(pdx, index);
}

void DpcForIsr_Video0(unsigned long data)
{
	int i = 0;
	int ret;
	//int curr_buf_index;
	struct hws_pcie_dev *pdx;
	//pdx = sys_dvrs_hw_pdx;
	pdx = (struct hws_pcie_dev *)data;
	//unsigned long *pdata = (unsigned long *)data;
	//curr_buf_index = *pdata;
	//printk("DpcForIsr_Video0\n");
	ret = SetQueue(pdx, i);
	//printk("[%X] pdx->m_bVCapStarted[i]=%d  ret=%d\n", pdx->pdev->device,pdx->m_bVCapStarted[i],ret);
	if (ret != 0) {
		return;
	}

	if (pdx->m_bVCapStarted[i] == TRUE) {
		//printk("pdx->m_bVCapIntDone[i] = %d\n", pdx->m_bVCapIntDone[i]);
		//printk("pdx->m_pVideoEvent[i] = %d\n", pdx->m_pVideoEvent[i]);

		if ((pdx->m_bVCapIntDone[i] == TRUE) && pdx->m_pVideoEvent[i]) {
			pdx->m_bVCapIntDone[i] = FALSE;
			//printk("pdx->m_bChangeVideoSize[i] = %d\n",pdx->m_bChangeVideoSize[i]);
			if ((!pdx->m_bChangeVideoSize[i]) &&
			    (pdx->m_pVideoEvent[i])) {
				//pdx->wq_flag[i] = 1;
				//wake_up_interruptible(&pdx->wq_video[i]);
				//printk("Set Event\n");
				queue_work(pdx->wq, &pdx->video[i].videowork);
			} else {
				pdx->m_bChangeVideoSize[i] = 0;
			}
		}
	}
}

void DpcForIsr_Video1(unsigned long data)
{
	int i = 1;
	int ret;
	//int curr_buf_index;
	struct hws_pcie_dev *pdx;
	pdx = (struct hws_pcie_dev *)data;
	//pdx = sys_dvrs_hw_pdx;

	//unsigned long *pdata = (unsigned long *)data;
	//curr_buf_index = *pdata;

	ret = SetQueue(pdx, i);
	if (ret != 0) {
		return;
	}

	if (pdx->m_bVCapStarted[i] == TRUE) {
		if (pdx->m_bVCapIntDone[i] == TRUE && pdx->m_pVideoEvent[i]) {
			pdx->m_bVCapIntDone[i] = FALSE;
			if (!pdx->m_bChangeVideoSize[i]) {
				if ((!pdx->m_bChangeVideoSize[i]) &&
				    (pdx->m_pVideoEvent[i])) {
					//pdx->wq_flag[i] = 1;
					//wake_up_interruptible(&pdx->wq_video[i]);
					queue_work(pdx->wq,
						   &pdx->video[i].videowork);
				}
			} else {
				pdx->m_bChangeVideoSize[i] = 0;
			}
		}
	}
}

void DpcForIsr_Video2(unsigned long data)
{
	int i = 2;
	int ret;
	//int curr_buf_index;
	struct hws_pcie_dev *pdx;
	//pdx = sys_dvrs_hw_pdx;
	pdx = (struct hws_pcie_dev *)data;
	//unsigned long *pdata = (unsigned long *)data;
	//curr_buf_index = *pdata;
	ret = SetQueue(pdx, i);
	if (ret != 0) {
		return;
	}

	if (pdx->m_bVCapStarted[i] == TRUE) {
		if (pdx->m_bVCapIntDone[i] == TRUE && pdx->m_pVideoEvent[i]) {
			pdx->m_bVCapIntDone[i] = FALSE;
			if (!pdx->m_bChangeVideoSize[i]) {
				if ((!pdx->m_bChangeVideoSize[i]) &&
				    (pdx->m_pVideoEvent[i])) {
					//pdx->wq_flag[i] = 1;
					//wake_up_interruptible(&pdx->wq_video[i]);
					queue_work(pdx->wq,
						   &pdx->video[i].videowork);
				}
			} else {
				pdx->m_bChangeVideoSize[i] = 0;
			}
		}
	}
}

void DpcForIsr_Video3(unsigned long data)
{
	int i = 3;
	int ret;
	//int curr_buf_index;
	struct hws_pcie_dev *pdx;
	//pdx = sys_dvrs_hw_pdx;
	pdx = (struct hws_pcie_dev *)data;
	//unsigned long *pdata = (unsigned long *)data;
	//curr_buf_index = *pdata;
	//mutex_lock(&pdx->video_mutex[i]);
	//printk("DpcForIsr_Video3 data = [%d]%d \n",i,curr_buf_index);

	ret = SetQueue(pdx, i);
	if (ret != 0) {
		//spin_unlock(&pdx->video_lock[i]);
		//mutex_unlock(&pdx->video_mutex[i]);
		return;
	}

	if (pdx->m_bVCapStarted[i] == TRUE) {
		if (pdx->m_bVCapIntDone[i] == TRUE && pdx->m_pVideoEvent[i]) {
			pdx->m_bVCapIntDone[i] = FALSE;
			if (!pdx->m_bChangeVideoSize[i]) {
				if ((!pdx->m_bChangeVideoSize[i]) &&
				    (pdx->m_pVideoEvent[i])) {
					//KeSetEvent(pdx->m_pVideoEvent[i], 0, FALSE);
					//printk("SetEvenT[%d]\n",i);
					//kill_fasync (&hw_async_video3, SIGIO, POLL_IN);
					//pdx->wq_flag[i] = 1;
					//wake_up_interruptible(&pdx->wq_video[i]);
					queue_work(pdx->wq,
						   &pdx->video[i].videowork);
				}
			} else {
				pdx->m_bChangeVideoSize[i] = 0;
			}
		}
	}
	//spin_unlock(&pdx->video_lock[i]);
	//mutex_unlock(&pdx->video_mutex[i]);
}
