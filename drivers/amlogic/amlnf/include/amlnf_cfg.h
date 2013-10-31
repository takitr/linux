
#include "amlnf_dev.h"
//#define	AML_NAND_UBOOT


/**************PHY****************/
#define	AML_SLC_NAND_SUPPORT	
#define	AML_MLC_NAND_SUPPORT	
//#define	AML_NAND_DBG
#define 	NEW_NAND_SUPPORT
//#define AML_NAND_NEW_OOB
//#if MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8
#ifdef CONFIG_ARCH_MESON8
#define CONFIG_NAND_AML_M8   
#define	AML_NAND_DBG_M8
#endif

