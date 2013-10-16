#
# aml_nftl
#

ifneq ($(KBUILD_SRC),)
TOP_KBUILD_SRC := $(KBUILD_SRC)/
endif

$(obj)/aml_nftl_core.o:
	@cp $(TOP_KBUILD_SRC)drivers/amlogic/amlnf/nftl/aml_nftl_core_130930.z \
        drivers/amlogic/amlnf/nftl/aml_nftl_core.o

obj-$(CONFIG_AML_NEXT_GEN_NAND)	+= aml_nftl_core.o
