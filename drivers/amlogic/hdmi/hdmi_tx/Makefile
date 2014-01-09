obj-$(CONFIG_AML_HDMI_TX)		+= hdmitx.o

hdmitx-objs := hdmi_tx.o hdmi_tx_cec.o hdmi_cec_key.o hdmi_tx_video.o hdmi_tx_audio.o hdmi_tx_edid.o hdmi_tx_audio.o hdmi_tx_hdcp.o hdmi_tx_compliance.o \

ifeq ($(CONFIG_ARCH_MESON8),y)
obj-$(CONFIG_AML_HDMI_TX)  += hdmi_tx_miniboot.o
endif

obj-$(CONFIG_AML_HDMI_TX)	+= hw/

#EXTRA_CFLAGS += -O2


