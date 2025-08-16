/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HWS_PCIE_H
#define HWS_PCIE_H

#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-dma-sg.h>

struct snd_pcm_substream;

//---------------------------------------------
#define MAX_USER_IRQ 1
#define XDMA_CHANNEL_NUM_MAX (1)
#define MAX_NUM_ENGINES (XDMA_CHANNEL_NUM_MAX * 2)

#define  PCIE_BARADDROFSIZE 4


#define PCI_BUS_ACCESS_BASE 0x0
#define PCIEBAR_AXI_BASE 0x20000000

#define CTL_REG_ACC_BASE 0x0
#define PCI_ADDR_TABLE_BASE CTL_REG_ACC_BASE

#define INT_EN_REG_BASE  PCI_BUS_ACCESS_BASE+0x134
#define PCIEBR_EN_REG_BASE  PCI_BUS_ACCESS_BASE+0x148
#define PCIE_INT_DEC_REG_BASE PCI_BUS_ACCESS_BASE+0x138


#define CVBS_IN_BASE 0x04000
#define CBVS_IN_BUF_BASE CVBS_IN_BASE+ 16*PCIE_BARADDROFSIZE
#define CBVS_IN_BUF_BASE2 CVBS_IN_BASE+ 50*PCIE_BARADDROFSIZE

#define MAX_L_VIDEO_SIZE			0x200000	//2M

#define MAX_VIDEO_PKSIZE            2
#define HALF_VIDEO_PKSIZE           1



#define PCI_E_BAR_PAGE_SIZE 0x20000000
#define PCI_E_BAR_ADD_MASK 0xE0000000
#define PCI_E_BAR_ADD_LOWMASK 0x1FFFFFFF

#define MAX_DMA_AUDIO_PK_SIZE      128*16*2

#define MEM_LOCK					1
#define MEM_UNLOCK					0


#define PAGESIZE					4096


#define MAX_AUDIO_CAP_SIZE			10*1024			// 16KHz*2Byte*8channel/4(0.25sec)/4096(pagesize)
#define MAX_AUDIO_QURE_INDEX        8
#define MAX_AUDIO_LINE   4
#define MAX_AUDIO_SUBCHANNEL		24
#define MAX_DMA_PACK_SIZE 160


#define MAX_AUDIO_EVENT_CNT 24
#define MAX_AUDIO_BUF_CNT 50
#define MAX_AUDIO_QUEUE_CNT 20
#define MAX_AUDIO_BUF_SIZE 1920

#define MAX_VID_CHANNELS            4
#define MAX_VIDEO_QUEUE				4
#define MAX_AUDIO_QUEUE				8

#define CUR_AUDIO_PACKET_LENGTH		4096+34						// 250ms,  2*16,000=32,000 -> 1s
#define AUDIO_ONE_QUEUE_SIZE		8*CUR_AUDIO_PACKET_LENGTH

#include <linux/sizes.h>
#define MAX_MM_VIDEO_SIZE            SZ_4M

#define MAX_VIDEO_HW_W 1920
#define MAX_VIDEO_HW_H 1080
#define MAX_VIDEO_SCALER_SIZE 1920*1080*2

struct hwsmem_param 
{
	u32 index;
	u32 type;
	u32 status;
};


typedef enum 
{
   StandardNone				= 0x80000000,
   StandardNTSC				= 0x00000001,
   StandardPAL				= 0x00000002,
   StandardSECAM			= 0x00000004,
} VideoStandard_t;  

struct hws_video_fmt {
	u32 channel;     /* 0-3 */
	u32 standard;    /* NTSC / PAL / … */
	u32 odd_only;    /* 1 = odd-fields only */
	u32 width;
	u32 height;
	u32 frame;       /* NTSC : 1~30, PAL : 101~125(1~25) */
	u32 half_size;
	u32 down_size;
};


#define HWS_IOC_MAGIC      'd'

enum hws_ioc_cmd {
	HWS_IOC_MAP_BUFFER,
	HWS_IOC_START_DMA,
	HWS_IOC_STOP_DMA,
	HWS_IOC_GET_CAPTURE,
	HWS_IOC_SET_FORMAT,
};

#define HWS_IOC_MAP_BUFFER     _IOWR(HWS_IOC_MAGIC, HWS_IOC_MAP_BUFFER,  struct hwsmem_param)
#define HWS_IOC_START_DMA      _IOWR(HWS_IOC_MAGIC, HWS_IOC_START_DMA,   struct hwsmem_param)
#define HWS_IOC_STOP_DMA       _IOWR(HWS_IOC_MAGIC, HWS_IOC_STOP_DMA,    struct hwsmem_param)
#define HWS_IOC_GET_CAPTURE    _IOWR(HWS_IOC_MAGIC, HWS_IOC_GET_CAPTURE, struct hwsmem_param)
#define HWS_IOC_SET_FORMAT     _IOWR(HWS_IOC_MAGIC, HWS_IOC_SET_FORMAT,  struct hwsmem_param)

//--------------------

#define HWS_PCIE_WRITE(__addr, __offst, __data)	writel((__data), (dev->mmio + (__addr + __offst)))
#define HWS_PCIE_READ(__addr, __offst)		readl((dev->mmio + (__addr + __offst)))


#define	UNSET	(-1U)

struct hws_pcie_dev;
struct hws_adapter;

struct hwsvideo_buffer {
    struct vb2_v4l2_buffer vb;   /* must be first */
    struct list_head       list; /* driver SW queue link */
    void                  *mem;
};

/*
| Old field name           | New field name            | Moved from                      |
| ------------------------ | ------------------------- | ------------------------------- |
| `dev`                    | `parent`                  | hws\_video                      |
| `v4l2_dev`               | `v4l2_device`             | hws\_video                      |
| `vdev`                   | `video_device`            | hws\_video                      |
| `vq`                     | `buffer_queue`            | hws\_video                      |
| `queue`                  | `capture_queue`           | hws\_video                      |
| `seqnr`                  | `sequence_number`         | hws\_video                      |
| `video_lock`             | `state_lock`              | hws\_video                      |
| `queue_lock`             | `capture_queue_lock`      | hws\_video                      |
| `slock`                  | `irq_lock`                | hws\_video                      |
| `pixfmt`                 | `pixel_format`            | hws\_video                      |
| `queryIndex`             | `query_index`             | hws\_video                      |
| `index`                  | `channel_index`           | hws\_video                      |
| `videowork`              | `video_work`              | hws\_video                      |
| `Interlaced`             | `interlaced` (now *bool*) | hws\_video                      |
| `m_Curr_Brightness`      | `current_brightness`      | hws\_video                      |
| `m_Curr_Contrast`        | `current_contrast`        | hws\_video                      |
| `m_Curr_Saturation`      | `current_saturation`      | hws\_video                      |
| `m_Curr_Hue`             | `current_hue`             | hws\_video                      |
| `current_out_width`      | `output_width`            | hws\_video                      |
| `curren_out_height`      | `output_height`           | hws\_video                      |
| `current_out_framerate`  | `output_frame_rate`       | hws\_video                      |
| `current_out_pixfmt`     | `output_pixel_format`     | hws\_video                      |
| `current_out_size_index` | `output_size_index`       | hws\_video                      |
| `m_pbyVideo_phys`        | `buf_phys_addr`           | hws\_pcie\_dev                  |
| `m_pbyVideoBuffer`       | `buf_virt`                | hws\_pcie\_dev                  |
| `m_dwVideoBuffer`        | `buf_size_bytes`          | hws\_pcie\_dev                  |
| `m_dwVideoHighBuffer`    | `buf_high_wmark`          | hws\_pcie\_dev                  |
| `m_pVCAPStatus`          | `queue_status`            | hws\_pcie\_dev                  |
| `m_VideoInfo`            | `chan_info`               | hws\_pcie\_dev                  |
| `m_format`               | `fmt_curr`                | hws\_pcie\_dev                  |
| `m_bChangeVideoSize`     | `size_changed_flag`       | hws\_pcie\_dev                  |
| `m_bVCapStarted`         | `cap_active`              | hws\_pcie\_dev                  |
| `m_nVideoBusy`           | `dma_busy`                | hws\_pcie\_dev                  |
| `m_bVideoStop`           | `stop_requested`          | hws\_pcie\_dev                  |
| `m_nRDVideoIndex`        | `rd_idx`                  | hws\_pcie\_dev                  |
| `m_nVideoBufferIndex`    | `wr_idx`                  | hws\_pcie\_dev                  |
| `m_nVideoHalfDone`       | `half_done_cnt`           | hws\_pcie\_dev                  |
| `m_pVideoEvent`          | `irq_event`               | hws\_pcie\_dev                  |
| `m_bVCapIntDone`         | `irq_done_flag`           | hws\_pcie\_dev                  |
| `m_curr_No_Video`        | `signal_loss_cnt`         | hws\_pcie\_dev                  |
| *(none)*                 | `sw_fps`                  | **new diagnostic counter**      |
*/


/* ───────────────────────────────────────────────────────────────────── */
/*  Per-channel VIDEO state                                             */
/* ───────────────────────────────────────────────────────────────────── */
struct hws_video {
	/* ───── linkage ───── */
	struct hws_pcie_dev		*parent;		/* parent device */

	struct video_device		 *video_device;
	struct vb2_queue			 buffer_queue;
	struct list_head			 capture_queue;
	struct hwsvideo_buffer *cur; // the in-flight VB2 buffer


	/* ───── locking ───── */
	struct mutex			 state_lock;		  /* primary state */
	spinlock_t				 irq_lock;			  /* ISR-side */

	/* ───── indices ───── */
	int						 channel_index;

	/* ───── async helpers ───── */
	struct tasklet_struct  video_bottom_half;

	/* ───── colour controls ───── */
	int						 current_brightness;
	int						 current_contrast;
	int						 current_saturation;
	int						 current_hue;

	/* ───── V4L2 controls ───── */
	struct v4l2_ctrl_handler control_handler;
	struct v4l2_ctrl		*detect_tx_5v_control;
	struct v4l2_ctrl		*hotplug_detect_control;

	/* ───── capture queue status ───── */
	struct hws_video_fmt	 fmt_curr;			/* current format        */

	/* ───── per-channel capture state ───── */
	bool					 cap_active;		/* was vcap_started      */
	bool                     stop_requested;	/* was video_stop        */
	u8                      last_buf_half_toggle;
	bool half_seen; // if your HW uses two half-frame toggles

	/* ───── misc counters ───── */
	int						 signal_loss_cnt;	/* no-video counter      */
};


/*
| Old field name         | New field name                  | Moved from                      |
| ---------------------- | ------------------------------- | ------------------------------- |
| `dev`                  | `parent`                        | hws\_audio                      |
| `card`                 | `sound_card`                    | hws\_audio                      |
| `substream`            | `pcm_substream`                 | hws\_audio                      |
| `audiowork`            | `audio_work`                    | hws\_audio                      |
| `index`                | `channel_index`                 | hws\_audio                      |
| `pos`                  | `buffer_position`               | hws\_audio                      |
| `ring_offsize`         | `ring_offset_bytes`             | hws\_audio                      |
| `ring_over_size`       | `ring_overflow_bytes`           | hws\_audio                      |
| `ring_wpos_byframes`   | `ring_write_pos_frames`         | hws\_audio                      |
| `ring_size_byframes`   | `ring_size_frames`              | hws\_audio                      |
| `period_size_byframes` | `period_size_frames`            | hws\_audio                      |
| `period_used_byframes` | `period_used_frames`            | hws\_audio                      |
| `sample_rate_out`      | `output_sample_rate`            | hws\_audio                      |
| `channels`             | `channel_count`                 | hws\_audio                      |
| `bits_per_sample`      | `bits_per_sample` *(unchanged)* | hws\_audio                      |
| `m_pbyAudio_phys`      | `buf_phys_addr`                 | hws\_pcie\_dev                  |
| `m_pbyAudioBuffer`     | `buf_virt`                      | hws\_pcie\_dev                  |
| `m_pAudioData`         | `data_buf`                      | hws\_pcie\_dev                  |
| `m_pAudioData_area`    | `data_area`                     | hws\_pcie\_dev                  |
| `m_dwAudioBuffer`      | `buf_size_bytes`                | hws\_pcie\_dev                  |
| `m_dwAudioBufferHigh`  | `buf_high_wmark`                | hws\_pcie\_dev                  |
| `m_bACapStarted`       | `cap_active`                    | hws\_pcie\_dev                  |
| `m_nAudioBusy`         | `dma_busy`                      | hws\_pcie\_dev                  |
| `m_nAudioBufferIndex`  | `wr_idx`                        | hws\_pcie\_dev                  |
| `m_nRDAudioIndex`      | `rd_idx`                        | hws\_pcie\_dev                  |
| `m_pAudioEvent`        | `irq_event`                     | hws\_pcie\_dev                  |
| `m_bAudioRun`          | `stream_running`                | hws\_pcie\_dev                  |
| `m_bAudioStop`         | `stop_requested`                | hws\_pcie\_dev                  |
*/
/* ───────────────────────────────────────────────────────────────────── */
/*  Per-channel AUDIO state                                             */
/* ───────────────────────────────────────────────────────────────────── */
struct hws_audio {
    /* linkage */
    struct hws_pcie_dev       *parent;
    int                        channel_index;

    /* ALSA */
    struct snd_pcm_substream  *pcm_substream;

    /* stream state */
    bool                       cap_active;
    bool                       stream_running;
    bool                       stop_requested;

    /* minimal HW period tracking (optional) */
    u8                         last_period_toggle;   // or hw_period index

    /* PCM format (for HW programming) */
    u32                        output_sample_rate;
    u16                        channel_count;
    u16                        bits_per_sample;
};

/*
| Old field name            | New field name              | Now lives in   |
| ------------------------- | --------------------------- | -------------- |
| `pdev`                    | `pdev`                      | `hws_pcie_dev` |
| `audio[MAX_VID_CHANNELS]` | `audio[MAX_VID_CHANNELS]`   | `hws_pcie_dev` |
| `video[MAX_VID_CHANNELS]` | `video[MAX_VID_CHANNELS]`   | `hws_pcie_dev` |
| `videoslock[]`            | `videoslock[]`              | `hws_pcie_dev` |
| `audiolock[]`             | `audiolock[]`               | `hws_pcie_dev` |
| `map_bar0_addr`           | `bar0_base`                 | `hws_pcie_dev` |
| `wq`                      | `video_wq`                  | `hws_pcie_dev` |
| `auwq`                    | `audio_wq`                  | `hws_pcie_dev` |
| `irq_line`                | `irq_line`                  | `hws_pcie_dev` |
| `msi_enabled`             | `msi_enabled` *(now bool)*  | `hws_pcie_dev` |
| `msix_enabled`            | `msix_enabled` *(now bool)* | `hws_pcie_dev` |
| `entry[32]`               | `msix_entries[32]`          | `hws_pcie_dev` |
| `dwVendorID`              | `vendor_id` *(u16)*         | `hws_pcie_dev` |
| `dwDeviceID`              | `device_id` *(u16)*         | `hws_pcie_dev` |
| `m_Device_Version`        | `device_ver`                | `hws_pcie_dev` |
| `m_DeviceHW_Version`      | `hw_ver`                    | `hws_pcie_dev` |
| `m_Device_SubVersion`     | `sub_ver`                   | `hws_pcie_dev` |
| `m_Device_PortID`         | `port_id`                   | `hws_pcie_dev` |
| `m_Device_SupportYV12`    | `support_yv12`              | `hws_pcie_dev` |
| `m_MaxHWVideoBufferSize`  | `max_hw_video_buf_sz`       | `hws_pcie_dev` |
| `m_nMaxChl`               | `max_channels`              | `hws_pcie_dev` |
| `m_nCurreMaxVideoChl`     | `cur_max_video_ch`          | `hws_pcie_dev` |
| `m_nCurreMaxLineInChl`    | `cur_max_linein_ch`         | `hws_pcie_dev` |
| `m_bStartRun`             | `start_run` *(bool)*        | `hws_pcie_dev` |
| `m_bBufferAllocate`       | `buf_allocated`             | `hws_pcie_dev` |
| `m_dwAudioPTKSize`        | `audio_pkt_size`            | `hws_pcie_dev` |
| `mMain_tsk`               | `main_task`                 | `hws_pcie_dev` |
| `m_PciDeviceLost`         | `pci_lost`                  | `hws_pcie_dev` |
*/
struct hws_pcie_dev {
	/* ───── core objects ───── */
	struct pci_dev			*pdev;
	struct hws_audio		audio[MAX_VID_CHANNELS];
	struct hws_video		video[MAX_VID_CHANNELS];

	/* ───── BAR & workqueues ───── */
	void __iomem              *bar0_base;

	/* ───── device identity / capabilities ───── */
	u16                        vendor_id;
	u16                        device_id;
	u16                        device_ver;
	u16                        hw_ver;
	u32                        sub_ver;
	u32                        port_id;
    // TriState, used in `set_video_format_size`
	u32                        support_yv12;
	u32                        max_hw_video_buf_sz;
	u8                         max_channels;
	u8                         cur_max_video_ch;
	u8                         cur_max_linein_ch;
	bool                       start_run;

	bool                       buf_allocated;
	u32                        audio_pkt_size;

	/* ───── V4L2 framework objects ───── */
	struct v4l2_device		 v4l2_device;

	/* ───── kernel thread ───── */
	struct task_struct        *main_task;

	/* ───── error flags ───── */
	int                        pci_lost;
	
};

#endif
