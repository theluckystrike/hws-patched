obj-m = HwsCapture.o

HwsCapture-objs := \
	 hws_pci.o \
	 hws_interrupt.o \
	 hws_dma.o \
	 hws_scaler.o \
	 hws_init.o \
	 hws_video_pipeline.o \
	 hws_video.o \
	 hws_v4l2_tables.o \
	 hws_v4l2_ioctl.o \
	 hws_audio_pipeline.o 

src := $(PWD)

EXTRA_CFLAGS += -I$(src)
EXTRA_LDFLAGS += -S

ifneq ($(KERNELRELEASE),)
KERNELDIR ?= /lib/modules/$(KERNELRELEASE)/build
else
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
endif

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

# --------------------------------------------------------------------
# clang-tidy setup
CLANG_TIDY    := clang-tidy
ifeq (, $(shell which $(CLANG_TIDY)))
$(error "$(CLANG_TIDY) not found! Please install clang-tidy.")
endif

TIDY_CFLAGS   := $(EXTRA_CFLAGS)
TIDY_INCLUDES := -I$(src) \
                 -I$(KERNELDIR)/include \
                 -I$(KERNELDIR)/include/uapi \
                 -I$(KERNELDIR)/arch/$(shell uname -m)/include
TIDY_DEFINES  := -D__KERNEL__ -D__KERNEL_STRICT_NAMES

C_SOURCES     := $(wildcard *.c)

.PHONY: tidy
tidy:
	@echo ">> Running clang-tidy on: $(C_SOURCES)"
	@for f in $(C_SOURCES); do \
	  echo "   - $$f"; \
	  $(CLANG_TIDY) -p . $$f -- $(TIDY_CFLAGS) $(TIDY_INCLUDES) $(TIDY_DEFINES); \
	done

