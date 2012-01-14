# Do not build unless BoardConfig define BOARD_WLAN_TI_STA_DK_ROOT_SEMC.
ifdef BOARD_WLAN_TI_STA_DK_ROOT_SEMC
LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE := wilink_ini

# files that live under /system/etc/...

# Product-specific WLAN ini files are expected to have the name
# $(TARGET_PRODUCT)-something.ini. If present, the alphabetically
# first file of that kind will be installed during the make build.
# If no such files exist tiwlan-generic.ini will be used.
all_product_inifiles := \
    $(notdir $(wildcard $(LOCAL_PATH)/tiwlan-$(TARGET_PRODUCT)-*.ini))
ifeq ($(all_product_inifiles),)
all_product_inifiles := tiwlan-generic.ini
endif
variant_inifile := $(firstword $(all_product_inifiles))

inifile := tiwlan.ini

copy_to := $(addprefix $(TARGET_OUT)/etc/,$(inifile))
copy_from := $(addprefix $(LOCAL_PATH)/,$(variant_inifile))

$(copy_to) : PRIVATE_MODULE := tiwlan_etcdir
$(copy_to) : $(TARGET_OUT)/etc/% : $(LOCAL_PATH)/$(variant_inifile) | $(ACP)
	$(transform-prebuilt-to-target)

$(variant_inifile):
	$(hide) echo "tiwlan: Using $(variant_inifile) for $(TARGET_PRODUCT) product."

ALL_PREBUILT += $(variant_inifile)

ALL_PREBUILT += $(copy_to)

###########################################################################
# The parts below needed for semc builds to create the debian packages.
# The product variant spec will tell which ini file package to actually
# select.
include $(SEMCBUILD_SYSTEM)/debian/definitions.mk
include $(SEMCBUILD_SYSTEM)/debian/envsetup.mk
all_ini_packages := $(foreach p,$(all_product_inifiles),\
    $(TARGET_OUT_DEBIAN)/$(call debian-package-name,fw,$(basename $(p)))_$(SEMC_SYSTEM_VERSION).deb)

fscontents-deb: $(all_ini_packages)

$(all_ini_packages): $(TARGET_OUT_DEBIAN)/fw-%-$(debian-package-name-tail)_$(SEMC_SYSTEM_VERSION).deb: \
            $(LOCAL_PATH)/%.ini
	$(hide) PKG_NAME=$(notdir $(patsubst %_$(SEMC_SYSTEM_VERSION).deb,%,$@)) && \
	    STAGE_DIR=`mktemp -dt` && \
	    echo target Debian: $$PKG_NAME && \
	    mkdir -p $$STAGE_DIR/imgdata/system/etc && \
	    cp $< $$STAGE_DIR/imgdata/system/etc/$(inifile) && \
	    mkdir -p $(dir $@) && \
	    createpackage $$PKG_NAME $(SEMC_SYSTEM_VERSION) \
	        -df $$STAGE_DIR/imgdata \
	        -d variant-$(TARGET_BUILD_VARIANT) \
	        -o $(dir $@) > /dev/null && \
	    ( cd $$STAGE_DIR && find -type f ) > $@.files && \
	    rm -rf $$STAGE_DIR
endif
