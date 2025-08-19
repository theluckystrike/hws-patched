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

/* ───────────────────────────────────────────────────────────────────── */
/*  Per-channel VIDEO state                                             */
/* ───────────────────────────────────────────────────────────────────── */
struct hws_video {
	/* ───── linkage ───── */
	struct hws_pcie_dev		*parent;		/* parent device */
	struct video_device		 *video_device;

	struct vb2_queue			 buffer_queue;
	struct list_head			 capture_queue;
	struct hwsvideo_buffer *active;


	/* ───── locking ───── */
	struct mutex			 state_lock;		  /* primary state */
	spinlock_t				 irq_lock;			  /* ISR-side */

	/* ───── indices ───── */
	int						 channel_index;

	/* ───── async helpers ───── */
	struct tasklet_struct  bh_tasklet;

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
    u32 alloc_sizeimage;

	/* ───── per-channel capture state ───── */
	bool					 cap_active;		/* was vcap_started      */
	bool                     stop_requested;	/* was video_stop        */
	u8                      last_buf_half_toggle;
	bool half_seen; // if your HW uses two half-frame toggles
    u32  sequence_number;

	/* ───── misc counters ───── */
	int						 signal_loss_cnt;	/* no-video counter      */
};

struct hws_audio {
    /* linkage */
    struct hws_pcie_dev       *parent;
    int                        channel_index;

    /* ALSA */
    struct snd_pcm_substream  *pcm_substream;
	/* ring geometry (set in prepare/hw_params) */
	u32 periods;           /* runtime->periods */
	u32 period_bytes;      /* bytes per period */
	u32 next_period;       /* SW: next period to program */

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
