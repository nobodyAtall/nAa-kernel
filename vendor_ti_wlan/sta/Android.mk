# Do not build unless BoardConfig define BOARD_WLAN_TI_STA_DK_ROOT_SEMC.
ifdef BOARD_WLAN_TI_STA_DK_ROOT_SEMCE

LOCAL_PATH:= $(call my-dir)
SRC_MAIN := platforms/os/linux
SAVED_LOCAL_PATH := $(LOCAL_PATH)
# build the apps

include $(call all-subdir-makefiles)

include $(CLEAR_VARS)

# now the part that is actually built

ifneq ($(TARGET_BOARD_PLATFORM),)

LOCAL_PATH_ABS := $(shell cd $(SAVED_LOCAL_PATH) && pwd)

BUILD_ROOT := $(TARGET_OUT_INTERMEDIATES)/WILINK_OBJ

BUILD_SUBDIRS := $(shell cd $(SAVED_LOCAL_PATH) && find . -type d)
BUILD_SRC := $(shell cd $(SAVED_LOCAL_PATH) && find . -name \*[ch])
BUILD_MK := $(shell cd $(SAVED_LOCAL_PATH) && find . -name Makefile)

REL_SUBDIRS := $(addprefix $(SAVED_LOCAL_PATH)/, $(BUILD_SUBDIRS))
REL_SRC := $(addprefix $(SAVED_LOCAL_PATH)/, $(BUILD_SRC))
REL_MK := $(addprefix $(SAVED_LOCAL_PATH)/, $(BUILD_MK))


SOURCE_SCRIPT := msm_env.bash

$(BUILD_ROOT)/$(SRC_MAIN)/sdio.ko: $(PRODUCT_OUT)/kernel prep
	@(cd $(BUILD_ROOT)/$(SRC_MAIN); source $(SOURCE_SCRIPT); $(MAKE) -f Makefile)

$(BUILD_ROOT)/$(SRC_MAIN)/tiwlan_drv.ko: $(BUILD_ROOT)/$(SRC_MAIN)/sdio.ko

prep: subdirs src mk

subdirs: $(REL_SUBDIRS)
	@(for i in $(BUILD_SUBDIRS); do mkdir -p $(BUILD_ROOT)/$$i; done)

src: $(REL_SRC) subdirs
	@(for i in $(BUILD_SRC); do test -e $(BUILD_ROOT)/$$i || ln -sf $(LOCAL_PATH_ABS)/$$i $(BUILD_ROOT)/$$i; done)

mk: $(REL_MK) subdirs
	@(for i in $(BUILD_MK); do test -e $(BUILD_ROOT)/$$i || ln -sf $(LOCAL_PATH_ABS)/$$i $(BUILD_ROOT)/$$i; done)


# copy the modules

files := sdio.ko tiwlan_drv.ko

copy_to := $(addprefix $(TARGET_OUT)/lib/modules/,$(files))
copy_from := $(addprefix $(BUILD_ROOT)/$(SRC_MAIN)/,$(files))

$(TARGET_OUT)/lib/modules/%.ko : $(BUILD_ROOT)/$(SRC_MAIN)/%.ko | $(ACP)
	$(transform-prebuilt-to-target)

ALL_PREBUILT += $(copy_to)

endif # ifneq ($(TARGET_BOARD_PLATFORM),)

endif
