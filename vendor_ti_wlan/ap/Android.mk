# Do not build unless BoardConfig define BOARD_SOFTAP_DEVICE.
ifdef BOARD_SOFTAP_DEVICEE

LOCAL_PATH:= $(call my-dir)
SRC_MAIN := platforms/os/linux
SAVED_LOCAL_PATH_AP := $(LOCAL_PATH)
# build the apps

include $(call all-subdir-makefiles)

include $(CLEAR_VARS)

# now the part that is actually built

ifneq ($(TARGET_BOARD_PLATFORM),)

LOCAL_PATH_ABS_AP := $(shell cd $(SAVED_LOCAL_PATH_AP) && pwd)

BUILD_ROOT_AP := $(TARGET_OUT_INTERMEDIATES)/WILINK_AP_OBJ
SEC_CUSTOM := $(ANDROID_PRODUCT_OUT)/obj/WILINK_AP_OBJ/$(SRC_MAIN)/sec_custom.mk

BUILD_SUBDIRS_AP := $(shell cd $(SAVED_LOCAL_PATH_AP) && find . -type d)
BUILD_SRC_AP := $(shell cd $(SAVED_LOCAL_PATH_AP) && find . -name \*[ch])
BUILD_MK_AP := $(shell cd $(SAVED_LOCAL_PATH_AP) && find . -name Makefile)
BUILD_SEC_CUSTOM_AP := $(shell cd $(SAVED_LOCAL_PATH_AP) && find . -name sec_custom.mk)

REL_SUBDIRS_AP := $(addprefix $(SAVED_LOCAL_PATH_AP)/, $(BUILD_SUBDIRS_AP))
REL_SRC_AP := $(addprefix $(SAVED_LOCAL_PATH_AP)/, $(BUILD_SRC_AP))
REL_MK_AP := $(addprefix $(SAVED_LOCAL_PATH_AP)/, $(BUILD_MK_AP))
REL_SEC_CUSTOM_AP := $(addprefix $(SAVED_LOCAL_PATH_AP)/, $(BUILD_SEC_CUSTOM_AP))

SOURCE_SCRIPT_AP := msm_env.bash

$(BUILD_ROOT_AP)/$(SRC_MAIN)/tiap_drv.ko: $(INSTALLED_KERNEL_TARGET) prep_ap
	@(cd $(BUILD_ROOT_AP)/$(SRC_MAIN); source $(SOURCE_SCRIPT_AP); $(MAKE) -f $(SEC_CUSTOM))

prep_ap: subdirs_ap src_ap mk_ap sec_custom_ap

subdirs_ap: $(REL_SUBDIRS_AP)
	@(for i in $(BUILD_SUBDIRS_AP); do mkdir -p $(BUILD_ROOT_AP)/$$i; done)

src_ap: $(REL_SRC_AP) subdirs_ap
	@(for i in $(BUILD_SRC_AP); do test -e $(BUILD_ROOT_AP)/$$i || ln -sf $(LOCAL_PATH_ABS_AP)/$$i $(BUILD_ROOT_AP)/$$i; done)

mk_ap: $(REL_MK_AP) subdirs_ap
	@(for i in $(BUILD_MK_AP); do test -e $(BUILD_ROOT_AP)/$$i || ln -sf $(LOCAL_PATH_ABS_AP)/$$i $(BUILD_ROOT_AP)/$$i; done)

sec_custom_ap: $(REL_SEC_CUSTOM_AP) subdirs_ap
	@(for i in $(BUILD_SEC_CUSTOM_AP); do test -e $(BUILD_ROOT_AP)/$$i || ln -sf $(LOCAL_PATH_ABS_AP)/$$i $(BUILD_ROOT_AP)/$$i; done)

# copy the modules

files := tiap_drv.ko

copy_to := $(addprefix $(TARGET_OUT)/lib/modules/,$(files))
copy_from := $(addprefix $(BUILD_ROOT_AP)/$(SRC_MAIN)/,$(files))

$(TARGET_OUT)/lib/modules/%.ko : $(BUILD_ROOT_AP)/$(SRC_MAIN)/%.ko | $(ACP)
	$(transform-prebuilt-to-target)

ALL_PREBUILT += $(copy_to)

endif # ifneq ($(TARGET_BOARD_PLATFORM),)

endif
