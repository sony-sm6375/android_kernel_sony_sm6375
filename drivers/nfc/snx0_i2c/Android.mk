# Android makefile for the NFC Module

# Assume no targets will be supported
NFC_CHIPSET :=

# Check if this driver needs be built for current target
ifneq ($(findstring nxp_snx0,$(SOMC_CFG_NFC_DEVICE)),)
  NFC_CHIPSET := nxp_snx0
  NFC_SELECT := CONFIG_NFC_SNX0_DEVICES=m
endif

# Build/Package only in case of supported target
ifneq ($(NFC_CHIPSET),)

LOCAL_PATH := $(call my-dir)

# DLKM_DIR was moved for JELLY_BEAN (PLATFORM_SDK 16)
ifeq ($(call is-platform-sdk-version-at-least,16),true)
  DLKM_DIR := $(TOP)/device/qcom/common/dlkm
else
  DLKM_DIR := build/dlkm
endif # platform-sdk-version

KBUILD_OPTIONS += BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM)
KBUILD_OPTIONS += $(NFC_SELECT)

ifeq ($(SOMC_TARGET_OPERATOR), ets)
KBUILD_OPTIONS += CONFIG_NFC_ENABLE_G7_COLD_RESET=y
else
KBUILD_OPTIONS += CONFIG_NFC_ENABLE_G7_COLD_RESET=n
endif

include $(CLEAR_VARS)
LOCAL_MODULE              := $(SOMC_CFG_NFC_DEVICE)_i2c.ko
LOCAL_MODULE_KBUILD_NAME  := snx0.ko
LOCAL_MODULE_CLASS        := DLKM
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_OWNER        := somc
LOCAL_VENDOR_MODULE       := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)

include $(DLKM_DIR)/AndroidKernelModule.mk
endif  # supported target check
