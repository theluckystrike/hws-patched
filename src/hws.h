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

/*
| Old name              | New name             | Rationale                                               |
| --------------------- | -------------------- | ------------------------------------------------------- |
| `dev`                 | `parent`             | clarifies this is a back-pointer, not a `struct device` |
| `v4l2_dev`            | `v4l2_device`        | expands abbreviation                                    |
| `vdev`                | `video_device`       | ditto                                                   |
| `vq`                  | `buffer_queue`       | describes purpose                                       |
| `queue`               | `capture_queue`      | ditto                                                   |
| `fileindex`           | `file_index`         | snake\_case, explicit word break                        |
| `startstreamIndex`    | `stream_start_index` | clearer meaning                                         |
| `seqnr`               | `sequence_number`    | full words                                              |
| `video_lock`          | `state_lock`         | shows scope (global state)                              |
| `queue_lock`          | `capture_queue_lock` | matches queue name                                      |
| `slock`               | `irq_lock`           | explains usage                                          |
| `std`                 | `tv_standard`        | expands to TV/video standard                            |
| `pixfmt`              | `pixel_format`       | full words                                              |
| `queryIndex`          | `query_index`        | snake\_case                                             |
| `index`               | `channel_index`      | specifies what kind of index                            |
| `videowork`           | `video_work`         | snake\_case                                             |
| `Interlaced`          | `interlaced`         | lower-case boolean                                      |
| `m_Curr_*` group      | `current_*`          | remove Hungarian-style prefix                           |
| `current_out_*` group | `output_*`           | shorter but still clear                                 |
| `ctrl_handler`        | `control_handler`    | expands abbreviation                                    |
| `*_ctrl` pointers     | `*_control`          | ditto; replaces `hpd` with `hotplug_detect`             |
*/

struct hws_video {
	/* ───── linkage ───── */
	struct hws_pcie_dev     *parent;              /* parent device */

	/* ───── V4L2 framework objects ───── */
	struct v4l2_device       v4l2_device;
	struct video_device      video_device;
	struct vb2_queue         buffer_queue;
	struct list_head         capture_queue;

	/* ───── file & stream bookkeeping ───── */
	int                      file_index;
	int                      stream_start_index;
	unsigned int             sequence_number;

	/* ───── locking ───── */
	struct mutex             state_lock;          /* primary state */
	struct mutex             capture_queue_lock;  /* list_head guard */
	spinlock_t               irq_lock;            /* ISR-side */

	/* ───── format / standard ───── */
	v4l2_std_id              tv_standard;         /* e.g. V4L2_STD_NTSC_M */
	u32                      pixel_format;        /* e.g. V4L2_PIX_FMT_YUYV */

	/* ───── indices ───── */
	int                      query_index;
	int                      channel_index;

	/* ───── async helpers ───── */
	struct work_struct       video_work;

	/* ───── misc flags ───── */
	bool                     interlaced;

	/* ───── colour controls ───── */
	int                      current_brightness;
	int                      current_contrast;
	int                      current_saturation;
	int                      current_hue;

	/* ───── active output configuration ───── */
	int                      output_width;
	int                      output_height;
	int                      output_frame_rate;
	int                      output_pixel_format;
	int                      output_size_index;

	/* ───── V4L2 controls ───── */
	struct v4l2_ctrl_handler control_handler;
	struct v4l2_ctrl        *detect_tx_5v_control;
	struct v4l2_ctrl        *hotplug_detect_control;
	struct v4l2_ctrl        *content_type_control;
};	

/*
| Old name               | New name                    | Reasoning                                         |
| ---------------------- | --------------------------- | ------------------------------------------------- |
| `dev`                  | `parent`                    | mirrors video struct; clarifies intent            |
| `card`                 | `sound_card`                | expands meaning                                   |
| `substream`            | `pcm_substream`             | keeps standard “pcm” but drops abbreviation “snd” |
| `audiowork`            | `audio_work`                | snake\_case                                       |
| `index`                | `channel_index`             | specifies what is being indexed                   |
| `pos`                  | `buffer_position`           | describes purpose                                 |
| `ring_offsize`         | `ring_offset_bytes`         | explicit unit & spelling                          |
| `ring_over_size`       | `ring_overflow_bytes`       | clearer meaning                                   |
| `resampled_buf`        | `resampled_buffer`          | expands abbreviation                              |
| `resampled_buf_size`   | `resampled_buffer_size`     | ditto                                             |
| `ring_lock`            | *unchanged*                 | already descriptive                               |
| `ring_wpos_byframes`   | `ring_write_pos_frames`     | snake\_case + unit                                |
| `ring_size_byframes`   | `ring_size_frames`          | ditto                                             |
| `period_size_byframes` | `period_size_frames`        | ditto                                             |
| `period_used_byframes` | `period_used_frames`        | ditto                                             |
| `sample_rate_out`      | `output_sample_rate`        | clearer                                           |
| `channels`             | `channel_count`             | full word                                         |
| `bits_per_sample`      | *unchanged* (already clear) | —                                                 |

 */
struct hws_audio {
	/* ───── linkage ───── */
	struct hws_pcie_dev        *parent;             /* back-pointer */

	/* ───── ALSA objects ───── */
	struct snd_card            *sound_card;         /* returned by snd_card_new() */
	struct snd_pcm_substream   *pcm_substream;

	/* ───── async helper ───── */
	struct work_struct          audio_work;

	/* ───── indexing & position ───── */
	int                         channel_index;      /* 0 … MAX_VID_CHANNELS-1 */
	int                         buffer_position;    /* current play/capture head */

	/* ───── ring-buffer layout ───── */
	int                         ring_offset_bytes;      /* DMA start offset     */
	int                         ring_overflow_bytes;    /* overrun protection   */
	spinlock_t                  ring_lock;              /* guards fields below  */
	u32                         ring_write_pos_frames;  /* write pointer (frames) */
	u32                         ring_size_frames;       /* total size (frames)     */
	u32                         period_size_frames;     /* ALSA period size        */
	u32                         period_used_frames;     /* frames consumed so far  */

	/* ───── resampling workspace ───── */
	void                       *resampled_buffer;
	u32                         resampled_buffer_size;

	/* ───── PCM format ───── */
	u32                         output_sample_rate; /* Hz */
	u16                         channel_count;      /* 1 = mono, 2 = stereo … */
	u16                         bits_per_sample;    /* 16, 24, 32 …           */
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

	bool                       buf_allocated;
	u32                        audio_pkt_size;

	/* ───── kernel thread ───── */
	struct task_struct        *main_task;

	/* ───── error flags ───── */
	int                        pci_lost;
	
};

#endif
