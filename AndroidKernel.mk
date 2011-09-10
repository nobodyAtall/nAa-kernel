#Android makefile to build kernel as a part of Android Build

ifeq ($(TARGET_PREBUILT_KERNEL),)

KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
KERNEL_CONFIG := $(KERNEL_OUT)/.config
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/Image

#To ensure building of target $(KERNEL_CONFIG),
#make the modification time of $(KERNEL_OUT) newer.
MOD_TIME_KERNEL_OUT := sleep 1; touch -am $(KERNEL_OUT)

$(KERNEL_OUT):
	mkdir -p $(KERNEL_OUT)

$(KERNEL_CONFIG): $(KERNEL_OUT)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- $(KERNEL_DEFCONFIG)

$(TARGET_PREBUILT_KERNEL): $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi-

kerneltags: $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- tags

kernelconfig: $(KERNEL_OUT) $(KERNEL_CONFIG)
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- menuconfig
	cp $(KERNEL_OUT)/.config kernel/arch/arm/configs/$(KERNEL_DEFCONFIG)
	$(MOD_TIME_KERNEL_OUT)

SEMC_BUILD_TIMESTAMP := $(shell cd kernel && scripts/timestamp_gen.sh)
SEMC_BUILD_TIMESTAMP_UTC := $(shell date --date='$(SEMC_BUILD_TIMESTAMP)' +%s)
$(KERNEL_OUT)/semc_kernel_time_stamp.prop: kernel/scripts/timestamp_gen.sh $(TARGET_PREBUILT_KERNEL)
	@echo "ro.build.date=$(SEMC_BUILD_TIMESTAMP)" > $@
	@echo "ro.build.date.utc=$(SEMC_BUILD_TIMESTAMP_UTC)" >> $@
	@echo "ro.build.user=SEMCUser" >> $@
	@echo "ro.build.host=SEMCHost" >> $@

endif
