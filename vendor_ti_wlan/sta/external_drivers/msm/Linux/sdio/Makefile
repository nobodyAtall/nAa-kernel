
DK_ROOT = $(PWD)/../../../..
PLATRFORM_SRC=$(DK_ROOT)/platforms/hw/host_platform_msm/linux
WLAN_INCS := \
	$(DK_ROOT)/Txn \
	$(DK_ROOT)/external_drivers/msm/Linux/sdio \
	$(DK_ROOT)/platforms/hw/host_platform_msm/linux

EXTRA_CFLAGS += $(addprefix -I, $(WLAN_INCS))

SDIO_IN_BAND ?= n
TRACE ?= n
STRIP = y

ifeq ($(SDIO_IN_BAND),y)
EXTRA_CFLAGS += -DSDIO_IN_BAND_INTERRUPT
endif

ifeq ($(KERNEL_DEBUGGER),y)
  EXTRA_CFLAGS += -g -O2
endif	
ifeq ($(DEBUG),y)
  EXTRA_CFLAGS += -DSDIO_DEBUG -O2
else
  EXTRA_CFLAGS += -O2
  STRIP = y
endif	

ifneq ($(KERNELRELEASE),)

obj-m := sdio.o
ifeq ($(TEST),y)
obj-m += testsdio.o
endif

sdio-objs := SdioDrv.o msm_sdcc.o
ifeq ($(TEST),y)
testsdio-objs += testdrv.o $(PLATRFORM_SRC)/SdioAdapter.o
endif
else

PWD := $(shell pwd)
all:
	pwd
	@echo EXTRA_CFLAGS = $(EXTRA_CFLAGS)
	$(MAKE) CROSS_COMPILE=$(CROSS_COMPILE) ARCH=$(ARCH) -C $(KERNEL_DIR) M=$(PWD) modules
ifeq ($(STRIP),y)
	@echo $(CROSS_COMPILE)strip -g sdio.ko
	$(CROSS_COMPILE)strip -g sdio.ko
ifeq ($(TEST),y)
	$(CROSS_COMPILE)strip -g testsdio.ko
endif
endif
endif

clean:
	rm -f *.o *~ *.~* core .depend dep $(PLATRFORM_SRC)/SdioAdapter.o
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

