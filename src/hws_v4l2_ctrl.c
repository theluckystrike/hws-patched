#include "hws_v4l2_ctrl.h"
#include "hws.h"
#include "hws_reg.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/videodev2.h> // Core V4L2 definitions
#include <linux/errno.h>      // For error codes like EINVAL
#include <linux/string.h>     // For strcpy, memcpy, strlen
#include <media/v4l2-dev.h>     // For video_drvdata
#include <media/v4l2-ioctl.h> // For V4L2 ioctl helpers

