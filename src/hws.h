/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _HWS_PCIE_H_
#define _HWS_PCIE_H_

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

#include <sound/core.h>
#include <sound/pcm.h>

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

#define MAX_MM_VIDEO_SIZE			0x400000	//4M

#define MAX_VIDEO_HW_W 1920
#define MAX_VIDEO_HW_H 1080
#define MAX_VIDEO_SCLAER_SIZE 1920*1080*2

struct hwsmem_param 
{
	u32 index;
	u32 type;
	u32 status;
};

struct vcap_status {
	u32 lock;
	u32 channel;
	u32 size;
	u32 field;
	u32 path;
	u32 width;
	u32 height;
	u32 fps;
	u32 out_width;
	u32 out_height;
	u32 out_fps;
	u32 interlace;
	u32 hdcp;
};

struct acap_video_info
{
    u8                *video_buf[MAX_VIDEO_QUEUE];
	u8                *video_area[MAX_VIDEO_QUEUE];
	int                buf_size[4];
	struct vcap_status status[MAX_VIDEO_QUEUE];
	u32                index;
	u8                *scaler_buf;
	u8                *yuv2_buf;
	u8                *rotate_buf;
	u32                running;
};

struct acap_audio_info {
	u8                 *audio_buf[MAX_AUDIO_QUEUE];
	u8                 *audio_area[MAX_AUDIO_QUEUE];
	struct {
		u32 length;
		u32 lock;
	}                   status[MAX_AUDIO_QUEUE];
	u32                index;
	u32                running;
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


/* buffer for one video frame */
struct hwsvideo_buffer {
	/* common v4l buffer stuff -- must be first */
	struct vb2_v4l2_buffer	vb;
	struct list_head		queue;
	void *					mem;

};

struct hws_dmabuf{
	u32                 size;
	__le32					*cpu;
	dma_addr_t				dma;	
};

struct hws_video{
	struct hws_pcie_dev		*dev;
	struct v4l2_device		v4l2_dev;
	struct video_device		vdev;
	struct vb2_queue		vq;
	struct list_head		queue;
	int                     fileindex;
	int                     startstreamIndex;
	unsigned				seqnr;
	struct mutex			video_lock;
	struct mutex			queue_lock;
	spinlock_t				slock;
	v4l2_std_id				std;  //V4L2_STD_NTSC_M
	u32						pixfmt; //V4L2_PIX_FMT_YUYV(fourcc)
	int                     queryIndex;
	int						index;
	struct work_struct		videowork;
	int						Interlaced;
	//------------------------
	int m_Curr_Brightness;
	int m_Curr_Contrast;   
	int m_Curr_Saturation;
	int m_Curr_Hue;       
	//------------------------
	int current_out_width;
	int curren_out_height;
	int current_out_framerate;
	int current_out_pixfmt;
	int current_out_size_index;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *detect_tx_5v_ctrl;
	struct v4l2_ctrl *hpd_ctrl;
	struct v4l2_ctrl *content_type;
};
	
struct hws_audio{
	struct hws_pcie_dev		*dev;
	struct snd_card 		*card;	
	struct snd_pcm_substream *substream;
	struct work_struct		audiowork;
	int						pos;
	int						index;
	int                         ring_offsize;
	int                         ring_over_size;
	void                        *resampled_buf;
	u32                         resampled_buf_size;
	spinlock_t                  ring_lock;
    uint32_t                    ring_wpos_byframes;
    uint32_t                    ring_size_byframes;
    uint32_t                    period_size_byframes;
    uint32_t                    period_used_byframes;
	u32                         sample_rate_out;
    u16                         channels;
    u16                         bits_per_sample;
};
	
/*
Old name	New name
--------------------
videoslock	video_lock
audiolock	audio_lock
map_bar0_addr	bar0_base
wq / auwq	video_wq / audio_wq
video_data	video_priv
dpc_video_tasklet	video_tlet
audio_data	audio_priv
dpc_audio_tasklet	audio_tlet
irq_user_count	irq_user_cnt
dwVendorID / dwDeviceID	vendor_id / device_id
m_Device_Version / m_DeviceHW_Version	device_ver / hw_ver
m_Device_SubVersion	sub_ver
m_Device_PortID	port_id
m_Device_SupportYV12	support_yv12
m_MaxHWVideoBufferSize	max_hw_video_buf_sz
m_nMaxChl	max_channels
m_nCurreMaxVideoChl	cur_max_video_ch
m_nCurreMaxLineInChl	cur_max_linein_ch
m_bStartRun	start_run (bool)
m_pbyVideo_phys	video_phys
m_pbyVideoBuffer	video_buf
m_dwVideoBuffer / m_dwVideoHighBuffer	video_buf_len / video_buf_high
m_pVCAPStatus	vcap_status
m_bChangeVideoSize	video_size_changed (bool)
mMain_tsk	main_task
m_curr_No_Video	no_video_cnt
m_pbyAudio_phys	audio_phys
m_pbyAudioBuffer	audio_buf
m_pAudioData / m_pAudioData_area	audio_data / audio_data_area
m_dwAudioBuffer / m_dwAudioBufferHigh	audio_buf_len / audio_buf_high
m_bBufferAllocate	buf_allocated (bool)
m_bVCapStarted / m_bACapStarted	vcap_started / acap_started (bool)
m_nVideoBusy / m_nAudioBusy	video_busy / audio_busy
m_bVideoStop / m_bAudioStop	video_stop / audio_stop (bool)
m_nRDVideoIndex / m_nRDAudioIndex	rd_video_idx / rd_audio_idx
m_nVideoBufferIndex	video_buf_idx
m_nVideoHalfDone	video_half_done
m_pAudioEvent / m_pVideoEvent	audio_event / video_event
m_bVCapIntDone	vcap_int_done (bool)
m_bAudioRun	audio_running (bool)
m_dwAudioPTKSize	audio_pkt_size
m_dwSWFrameRate	sw_framerate
m_contrast etc.	contrast, brightness, saturation, hue
m_PciDeviceLost	pci_lost
entry	msix_entries
*/
struct hws_pcie_dev {
	/* ───── core objects ───── */
	struct pci_dev			*pdev;
	struct hws_audio		audio[MAX_VID_CHANNELS];
	struct hws_video		video[MAX_VID_CHANNELS];

	/* ───── synchronisation ───── */
	spinlock_t				videoslock[MAX_VID_CHANNELS];
	spinlock_t				audiolock[MAX_VID_CHANNELS];

	/* ───── BAR & workqueues ───── */
	void __iomem              *bar0_base;
	struct workqueue_struct   *video_wq;
	struct workqueue_struct   *audio_wq;
	unsigned long              video_priv[MAX_VID_CHANNELS];
	struct tasklet_struct      video_tlet[MAX_VID_CHANNELS];
	unsigned long              audio_priv[MAX_VID_CHANNELS];
	struct tasklet_struct      audio_tlet[MAX_VID_CHANNELS];

	/* ───── interrupt bookkeeping ───── */
	int                        irq_line;          /* < 0 = none */
	bool                       msi_enabled;
	bool                       msix_enabled;
	int                        irq_user_cnt;
	struct msix_entry          msix_entries[32];

	/* ───── device identity / capabilities ───── */
	u16                        vendor_id;
	u16                        device_id;
	u16                        device_ver;
	u16                        hw_ver;
	u32                        sub_ver;
	u32                        port_id;
	u32                        support_yv12;
	u32                        max_hw_video_buf_sz;
	u8                         max_channels;
	u8                         cur_max_video_ch;
	u8                         cur_max_linein_ch;
	bool                       start_run;

	/* ───── video buffers ───── */
	dma_addr_t                 video_phys[MAX_VID_CHANNELS];
	u8                        *video_buf[MAX_VID_CHANNELS];
	u32                        video_buf_len[MAX_VID_CHANNELS];
	u32                        video_buf_high[MAX_VID_CHANNELS];
	struct vcap_status         vcap_status[MAX_VID_CHANNELS][MAX_VIDEO_QUEUE];
	struct acap_video_info     video_info[MAX_VID_CHANNELS];
	struct hws_video_fmt       video_fmt[MAX_VID_CHANNELS];
	bool                       video_size_changed[MAX_VID_CHANNELS];

	/* ───── audio buffers ───── */
	dma_addr_t                 audio_phys[MAX_VID_CHANNELS];
	u8                        *audio_buf[MAX_VID_CHANNELS];
	u8                        *audio_data[MAX_VID_CHANNELS];
	u8                        *audio_data_area[MAX_VID_CHANNELS];
	u32                        audio_buf_len[MAX_VID_CHANNELS];
	u32                        audio_buf_high[MAX_VID_CHANNELS];
	bool                       buf_allocated;

	/* ───── capture state ───── */
	bool                       vcap_started[MAX_VID_CHANNELS];
	bool                       acap_started[MAX_VID_CHANNELS];
	u8                         video_busy[MAX_VID_CHANNELS];
	bool                       video_stop[MAX_VID_CHANNELS];
	int                        rd_video_idx[MAX_VID_CHANNELS];
	int                        video_buf_idx[MAX_VID_CHANNELS];
	int                        video_half_done[MAX_VID_CHANNELS];
	u8                         audio_busy[MAX_VID_CHANNELS];
	u8                         audio_buf_idx[MAX_VID_CHANNELS];
	u8                         audio_event[MAX_VID_CHANNELS];
	u8                         video_event[MAX_VID_CHANNELS];
	bool                       vcap_int_done[MAX_VID_CHANNELS];
	bool                       audio_running[MAX_VID_CHANNELS];
	bool                       audio_stop[MAX_VID_CHANNELS];
	int                        rd_audio_idx[MAX_VID_CHANNELS];
	u32                        audio_pkt_size;

	/* ───── misc per-channel counters ───── */
	int                        no_video_cnt[MAX_VID_CHANNELS];
	int                        sw_framerate[MAX_VID_CHANNELS];

	/* ───── colour controls ───── */
	u32                        contrast[MAX_VID_CHANNELS];
	u32                        brightness[MAX_VID_CHANNELS];
	u32                        saturation[MAX_VID_CHANNELS];
	u32                        hue[MAX_VID_CHANNELS];

	/* ───── kernel thread ───── */
	struct task_struct        *main_task;

	/* ───── error flags ───── */
	int                        pci_lost;
	
};

#endif
