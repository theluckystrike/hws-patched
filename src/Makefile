obj-m = HwsCapture.o

HwsCapture-objs := \
	 hws_pci.o \
	 hws_interrupt.o \
	 hws_dma.o \
	 hws_scaler.o \
	 hws_init.o \
	 hws_video_pipeline.o \
	 hws_v4l2_tables.o \
	 hws_v4l2_ioctl.o \
	 hws_v4l2_ctrl.o \
	 hws_registration.o \
	 hws_audio_pipeline.o


EXTRA_CFLAGS += -I$(src)
# strip debug symbols
EXTRA_LDFLAGS += -S

ifneq ($(KERNELRELEASE),)
KERNELDIR ?= /lib/modules/$(KERNELRELEASE)/build
else
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
endif

PWD := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

