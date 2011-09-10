# MSM7x01A
   zreladdr-$(CONFIG_ARCH_MSM7X01A)	:= 0x10008000
params_phys-$(CONFIG_ARCH_MSM7X01A)	:= 0x10000100
initrd_phys-$(CONFIG_ARCH_MSM7X01A)	:= 0x10800000

# MSM7x25
   zreladdr-$(CONFIG_ARCH_MSM7X25)	:= 0x00208000
params_phys-$(CONFIG_ARCH_MSM7X25)	:= 0x00200100
initrd_phys-$(CONFIG_ARCH_MSM7X25)	:= 0x0A000000

# MSM7x27
ifeq ($(CONFIG_CAPTURE_KERNEL),y)
# Capture kernel
    zreladdr-$(CONFIG_ARCH_MSM7X27)    := 0x0D208000
params_phys-$(CONFIG_ARCH_MSM7X27)     := 0x0D201000
initrd_phys-$(CONFIG_ARCH_MSM7X27)     := 0x0D600000
else
# Main kernel
   zreladdr-$(CONFIG_ARCH_MSM7X27)	:= 0x00208000
params_phys-$(CONFIG_ARCH_MSM7X27)	:= 0x00200100
initrd_phys-$(CONFIG_ARCH_MSM7X27)	:= 0x0A000000
endif

# MSM7x30
   zreladdr-$(CONFIG_ARCH_MSM7X30)	:= 0x00208000
params_phys-$(CONFIG_ARCH_MSM7X30)	:= 0x00200100
initrd_phys-$(CONFIG_ARCH_MSM7X30)	:= 0x01200000

ifeq ($(CONFIG_MSM_SOC_REV_A),y)
# QSD8X50A
   zreladdr-$(CONFIG_ARCH_QSD8X50)	:= 0x00008000
params_phys-$(CONFIG_ARCH_QSD8X50)	:= 0x00000100
initrd_phys-$(CONFIG_ARCH_QSD8X50)	:= 0x04000000
else
# QSD8x50
ifeq ($(CONFIG_CRASH_DUMP),y)
# Capture kernel
    zreladdr-$(CONFIG_ARCH_QSD8X50)    := 0x37808000
params_phys-$(CONFIG_ARCH_QSD8X50)     := 0x37801000
initrd_phys-$(CONFIG_ARCH_QSD8X50)     := 0x37C00000
else
# Main kernel
   zreladdr-$(CONFIG_ARCH_QSD8X50)	:= 0x20008000
params_phys-$(CONFIG_ARCH_QSD8X50)	:= 0x20000100
initrd_phys-$(CONFIG_ARCH_QSD8X50)	:= 0x24000000
endif
endif
