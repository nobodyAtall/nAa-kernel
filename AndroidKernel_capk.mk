#Android makefile to build kernel as a part of Android Build

KERNEL_CONFIG_CUST := $(KERNEL_OUT_CUST)/.config

#To ensure building of target $(KERNEL_CONFIG_CUST),
#make the modification time of $(KERNEL_OUT_CUST) newer.
MOD_TIME_KERNEL_OUT := sleep 1; touch -am $(KERNEL_OUT)

$(KERNEL_OUT_CUST):
	mkdir -p $(KERNEL_OUT_CUST)

$(KERNEL_CONFIG_CUST): $(KERNEL_OUT_CUST)
	$(MAKE) -C kernel O=../$(KERNEL_OUT_CUST) ARCH=arm CROSS_COMPILE=arm-eabi- $(KERNEL_DEFCONFIG_CUST)

$(TARGET_PREBUILT_KERNEL_CUST): $(KERNEL_OUT_CUST) $(KERNEL_CONFIG_CUST)
	$(MAKE) -C kernel O=../$(KERNEL_OUT_CUST) ARCH=arm CROSS_COMPILE=arm-eabi-

kerneltags_cust: $(KERNEL_OUT_CUST) $(KERNEL_CONFIG_CUST)
	$(MAKE) -C kernel O=../$(KERNEL_OUT_CUST) ARCH=arm CROSS_COMPILE=arm-eabi- tags

kernelconfig_cust: $(KERNEL_OUT_CUST) $(KERNEL_CONFIG_CUST)
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=../$(KERNEL_OUT_CUST) ARCH=arm CROSS_COMPILE=arm-eabi- menuconfig
	cp $(KERNEL_OUT_CUST)/.config kernel/arch/arm/configs/$(KERNEL_DEFCONFIG_CUST)
	$(MOD_TIME_KERNEL_OUT)
