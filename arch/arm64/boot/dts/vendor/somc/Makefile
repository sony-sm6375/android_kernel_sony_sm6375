ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_MACH_SONY_PDX225) += \
	blair-murray-pdx225_generic-overlay.dtbo \
	blair-murray-pdx225_sm5038_generic-overlay.dtbo
dtbo-$(CONFIG_MACH_SONY_PDX235) += \
	blair-zambezi-pdx235_generic-overlay.dtbo

ifeq ($(CONFIG_MACH_SONY_PDX225),y)
blair-murray-pdx225_generic-overlay.dtbo-base := ../qcom/blair.dtb
blair-murray-pdx225_sm5038_generic-overlay.dtbo-base := ../qcom/blair.dtb
endif
ifeq ($(CONFIG_MACH_SONY_PDX235),y)
blair-zambezi-pdx235_generic-overlay.dtbo-base := ../qcom/blair.dtb
endif
else
dtb-$(CONFIG_MACH_SONY_PDX225) += \
	blair-murray-pdx225_generic.dtb \
	blair-murray-pdx225_sm5038_generic.dtb
dtb-$(CONFIG_MACH_SONY_PDX235) += \
	blair-zambezi-pdx235_generic.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
