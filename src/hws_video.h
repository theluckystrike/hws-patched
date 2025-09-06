/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef HWS_VIDEO_H
#define HWS_VIDEO_H

int hws_video_register(struct hws_pcie_dev *dev);
void hws_video_unregister(struct hws_pcie_dev *dev);
void hws_enable_video_capture(struct hws_pcie_dev *hws,
			      unsigned int chan,
			      bool on);

int hws_video_init_channel(struct hws_pcie_dev *pdev, int ch);
void hws_video_cleanup_channel(struct hws_pcie_dev *pdev, int ch);
void check_video_format(struct hws_pcie_dev *pdx);
int hws_check_card_status(struct hws_pcie_dev *hws);
void hws_init_video_sys(struct hws_pcie_dev *hws, bool enable);

void hws_program_video_from_vb2(struct hws_pcie_dev *hws,
				unsigned int ch,
				struct vb2_buffer *vb);

int hws_video_pm_suspend(struct hws_pcie_dev *hws);
void hws_video_pm_resume(struct hws_pcie_dev *hws);

#endif // HWS_VIDEO_H
