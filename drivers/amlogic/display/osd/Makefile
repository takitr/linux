#
# Makefile for the AML Apollo framebuffer device driver
#


obj-$(CONFIG_FB_AM) += fb.o 

fb-objs =  osd_hw.o osd_main.o osd_dev.o  

ifeq ($(CONFIG_ARCH_MESON8),y)
fb-objs += osd_prot.o
else
fb-objs += osd_prot_dummy.o
endif

EXTRA_CFLAGS += -Iinclude/linux/osd/





