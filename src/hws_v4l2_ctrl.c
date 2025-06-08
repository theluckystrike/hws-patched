
//-------------------
static const struct v4l2_queryctrl g_no_ctrl = {
	.name = "42",
	.flags = V4L2_CTRL_FLAG_DISABLED,
};

static struct v4l2_queryctrl g_hws_ctrls[] = {
#if 1
	{
		V4L2_CID_BRIGHTNESS, //id
		V4L2_CTRL_TYPE_INTEGER, //type
		"Brightness", //name[32]
		MIN_VAMP_BRIGHTNESS_UNITS, //minimum
		MAX_VAMP_BRIGHTNESS_UNITS, //maximum
		1, //step
		BrightnessDefault, //default_value
		0, //flags
		{ 0, 0 }, //reserved[2]
	},
	{
		V4L2_CID_CONTRAST, //id
		V4L2_CTRL_TYPE_INTEGER, //type
		"Contrast", //name[32]
		MIN_VAMP_CONTRAST_UNITS, //minimum
		MAX_VAMP_CONTRAST_UNITS, //maximum
		1, //step
		ContrastDefault, //default_value
		0, //flags
		{ 0, 0 }, //reserved[2]
	},
	{
		V4L2_CID_SATURATION, //id
		V4L2_CTRL_TYPE_INTEGER, //type
		"Saturation", //name[32]
		MIN_VAMP_SATURATION_UNITS, //minimum
		MAX_VAMP_SATURATION_UNITS, //maximum
		1, //step
		SaturationDefault, //default_value
		0, //flags
		{ 0, 0 }, //reserved[2]
	},
	{
		V4L2_CID_HUE, //id
		V4L2_CTRL_TYPE_INTEGER, //type
		"Hue", //name[32]
		MIN_VAMP_HUE_UNITS, //minimum
		MAX_VAMP_HUE_UNITS, //maximum
		1, //step
		HueDefault, //default_value
		0, //flags
		{ 0, 0 }, //reserved[2]
	},
#endif
#if 0
	{
		V4L2_CID_AUTOGAIN,           //id
		V4L2_CTRL_TYPE_INTEGER,        //type
		"Hdcp enable",                 //name[32]
		0,                             //minimum
		1,                             //maximum
		1,                             //step
		0,                             //default_value
		0,                             //flags
		{ 0, 0 },                      //reserved[2]
	},
	{
		V4L2_CID_GAIN,           //id
		V4L2_CTRL_TYPE_INTEGER,        //type
		"Sample rate",                        //name[32]
		48000,                             //minimum
		48000,                             //maximum
		1,                             //step
		48000,                             //default_value
		0,                             //flags
		{ 0, 0 },                      //reserved[2]
	}
#endif
};

#define ARRAY_SIZE_OF_CTRL (sizeof(g_hws_ctrls) / sizeof(g_hws_ctrls[0]))

static struct v4l2_queryctrl *find_ctrlByIndex(unsigned int index)
{
	//scan supported queryctrl table
	if (index >= ARRAY_SIZE_OF_CTRL) {
		return NULL;
	} else {
		return &g_hws_ctrls[index];
	}
}

static struct v4l2_queryctrl *find_ctrl(unsigned int id)
{
	int i;
	//scan supported queryctrl table
	for (i = 0; i < ARRAY_SIZE_OF_CTRL; i++)
		if (g_hws_ctrls[i].id == id)
			return &g_hws_ctrls[i];

	return 0;
}

#if 0
static unsigned int find_Next_Ctl_ID(unsigned int id)
{
	int i;
	int nextID =-1;
	int curr_index =-1;
	//scan supported queryctrl table
	for( i=0; i<ARRAY_SIZE_OF_CTRL; i++ )
	{
		if(g_hws_ctrls[i].id==id)
		{
			curr_index = i;
			break;
		}
	}
	if(curr_index != -1)
	{
		if((curr_index +1)<ARRAY_SIZE_OF_CTRL)
		{
			nextID = g_hws_ctrls[curr_index +1].id;
		}
	}
	return nextID;
}
#endif

static int hws_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hws_video *vid =
		container_of(ctrl->handler, struct hws_video, ctrl_handler);
	struct hws_pcie_dev *pdx = vid->dev; /* if you keep this ptr */

	switch (ctrl->id) {
	case V4L2_CID_DV_RX_POWER_PRESENT:
		/* bit 3 (+5 V) over the two pipes for this HDMI port           */
		ctrl->val = !!(hws_read_port_hpd(pdx, vid->port) & HWS_5V_BIT);
		return 0;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0))
	case V4L2_CID_DV_RX_HOTPLUG_PRESENT:
		/* bit 0 (HPD) */
		ctrl->val = !!(hws_read_port_hpd(pdx, vid->port) & HWS_HPD_BIT);
		return 0;
#endif

	case V4L2_CID_DV_RX_IT_CONTENT_TYPE:
		ctrl->val = hdmi_content_type(vid); /* unchanged */
		return 0;

	default:
		return -EINVAL;
	}
}
