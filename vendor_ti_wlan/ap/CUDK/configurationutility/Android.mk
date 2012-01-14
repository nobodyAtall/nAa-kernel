LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

STATIC_LIB ?= y
DEBUG ?= y
WPA_ENTERPRISE ?= y
CONFIG_WPS = n
CONFIG_GEM = n

TI_HOSTAPD_LIB ?= y

WILINK_ROOT = ../..
CUDK_ROOT ?= $(WILINK_ROOT)/CUDK
CU_ROOT = $(CUDK_ROOT)/configurationutility

ifeq ($(DEBUG),y)
 DEBUGFLAGS = -O2 -g -DDEBUG -DTI_DBG -fno-builtin   # "-O" is needed to expand inlines
# DEBUGFLAGS+= -DDEBUG_MESSAGES
else
 DEBUGFLAGS = -O2
endif

DEBUGFLAGS+= -DHOST_COMPILE


AP_DEFINES =
ifeq ($(WPA_ENTERPRISE), y)
	AP_DEFINES += -D WPA_ENTERPRISE
endif

ifeq ($(TI_HOSTAPD_LIB), y)
        AP_DEFINES += -D TI_HOSTAPD_CLI_LIB
endif

ARMFLAGS  = -fno-common -g #-fno-builtin -Wall #-pipe

LOCAL_C_INCLUDES = \
	$(LOCAL_PATH)/inc \
	$(LOCAL_PATH)/$(CUDK_ROOT)/os/linux/inc \
	$(LOCAL_PATH)/$(CUDK_ROOT)/os/common/inc \
	$(LOCAL_PATH)/$(WILINK_ROOT)/stad/Export_Inc \
	$(LOCAL_PATH)/$(WILINK_ROOT)/stad/src/Sta_Management \
	$(LOCAL_PATH)/$(WILINK_ROOT)/stad/src/Application \
	$(LOCAL_PATH)/$(WILINK_ROOT)/utils \
	$(LOCAL_PATH)/$(WILINK_ROOT)/Txn \
	$(LOCAL_PATH)/$(WILINK_ROOT)/TWD/TWDriver \
	$(LOCAL_PATH)/$(WILINK_ROOT)/TWD/FirmwareApi \
	$(LOCAL_PATH)/$(WILINK_ROOT)/TWD/TwIf \
	$(LOCAL_PATH)/$(WILINK_ROOT)/platforms/os/linux/inc \
	$(LOCAL_PATH)/$(WILINK_ROOT)/platforms/os/common/inc \
	$(LOCAL_PATH)/$(KERNEL_DIR)/include \
	$(LOCAL_PATH)/$(WILINK_ROOT)/TWD/FW_Transfer/Export_Inc

LOCAL_SRC_FILES:= \
	src/console.c \
	src/cu_common.c \
	src/cu_cmd.c \
	src/ticon.c \
	src/cu_hostapd.c \
	src/wpa_core.c

LOCAL_CFLAGS+= -Wall -Wstrict-prototypes $(DEBUGFLAGS) -D__LINUX__ $(AP_DEFINES) -D__BYTE_ORDER_LITTLE_ENDIAN -DDRV_NAME='"tiwlan"'

LOCAL_CFLAGS += $(ARMFLAGS)

LOCAL_LDLIBS += -lpthread

LOCAL_STATIC_LIBRARIES := \
	libtiOsLibAP

ifeq ($(TI_HOSTAPD_LIB), y)
	LOCAL_STATIC_LIBRARIES += libhostapdcli
endif

LOCAL_MODULE:= tiap_cu
LOCAL_MODULE_TAGS:= eng

include $(BUILD_EXECUTABLE)

