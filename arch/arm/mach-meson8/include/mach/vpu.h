#ifndef __VPU_H__
#define __VPU_H__

//#define CONFIG_VPU_DYNAMIC_ADJ
typedef struct {
	unsigned int h_res;
	unsigned int v_res;
	unsigned int refresh_rate;
	unsigned int clk_level;
}VPU_Conf_t;
#define VPU_MOD_START	100
typedef enum {
    VPU_VIU_OSD1 = VPU_MOD_START,
    VPU_VIU_OSD2,
	VPU_VIU_VD1,
	VPU_VIU_VD2,
	VPU_VIU_CHROMA,
	VPU_VIU_OFIFO,
	VPU_VIU_SCALE,
	VPU_VIU_OSD_SCALE,
	VPU_VIU_VDIN0,
	VPU_VIU_VDIN1,
	VPU_PIC_ROT1,
	VPU_PIC_ROT2,
	VPU_PIC_ROT3,
	VPU_DI_PRE,
	VPU_DI_POST,
	VPU_VIU2_OSD1,
    VPU_VIU2_OSD2,
	VPU_VIU2_VD1,
	VPU_VIU2_CHROMA,
	VPU_VIU2_OFIFO,
	VPU_VIU2_SCALE,
	VPU_VIU2_OSD_SCALE,
	VPU_VENCP,
	VPU_VENCL,
	VPU_VENCI,
	VPU_ISP,
	VPU_MAX,
} vpu_mod_t;

static const char* vpu_mod_table[]={
	"viu_osd1",
	"viu_osd2",
	"viu_vd1",
	"viu_vd2",
	"viu_chroma",
	"viu_ofifo",
	"viu_scale",
	"viu_osd_scale",
	"viu_vdin0",
	"viu_vdin1",
	"pic_rot1",
	"pic_rot2",
	"pic_rot3",
	"di_pre",
	"di_post",
	"viu2_osd1",
	"viu2_osd2",
	"viu2_vd1",
	"viu2_chroma",
	"viu2_ofifo",
	"viu2_scale",
	"viu2_osd_scale",
	"vencp",
	"vencl",
	"venci",
	"isp",
	"none",
};




/*
 *  Function: get_vpu_clk_vmod
 *      Get vpu clk holding frequency with specified vomd
 *
 *	Parameters:
 *      vmod - unsigned int, must be one of the following constants:
 *                 VMODE supported by VOUT
 *                 VPU_MODE_VIDEO | VPU_MODE_BIT_MASK
 *                 VPU_MODE_OSD | VPU_MODE_BIT_MASK
 *                 VPU_MODE_VPP | VPU_MODE_BIT_MASK
 *
 *  Returns:
 *      unsigned int, vpu clk frequency unit in Hz
 * 
 *	Example:
 *      video_clk = get_vpu_clk_vmod(VMODE_720P);
 *      video_clk = get_vpu_clk_vmod(VPU_MODE_VIDEO | VPU_MODE_BIT_MASK);
 *
*/
//extern unsigned int get_vpu_clk_vmod(unsigned int vmod);
/*
 *  Function: request_vpu_clk_vomd
 *      Request a new vpu clk holding frequency with specified vomd
 *      Will change vpu clk if the max level in all vmod vpu clk holdings is unequal to current vpu clk level
 *
 *	Parameters:
 *      vclk - unsigned int, vpu clk frequency unit in Hz
 *      vmod - unsigned int, must be one of the following constants:
 *                 VMODE supported by VOUT
 *                 VPU_MODE_VIDEO | VPU_MODE_BIT_MASK
 *                 VPU_MODE_OSD | VPU_MODE_BIT_MASK
 *                 VPU_MODE_VPP | VPU_MODE_BIT_MASK
 *
 *  Returns:
 *      int, 0 for success, 1 for failed
 * 
 *	Example:
 *      ret = request_vpu_clk_vomd(100000000, VMODE_720P);
 *      ret = request_vpu_clk_vomd(300000000, VPU_MODE_VIDEO | VPU_MODE_BIT_MASK);
 *
*/
//extern int request_vpu_clk_vomd(unsigned int vclk, unsigned int vmod);
/*
 *  Function: release_vpu_clk_vomd
 *      Release vpu clk holding frequency to 0 with specified vomd
 *      Will change vpu clk if the max level in all vmod vpu clk holdings is unequal to current vpu clk level
 *
 *	Parameters:
 *      vmod - unsigned int, must be one of the following constants:
 *                 VMODE supported by VOUT
 *                 VPU_MODE_VIDEO | VPU_MODE_BIT_MASK
 *                 VPU_MODE_OSD | VPU_MODE_BIT_MASK
 *                 VPU_MODE_VPP | VPU_MODE_BIT_MASK
 *
 *  Returns:
 *      int, 0 for success, 1 for failed
 * 
 *	Example:
 *      ret = release_vpu_clk_vomd(VMODE_720P);
 *      ret = release_vpu_clk_vomd(VPU_MODE_VIDEO | VPU_MODE_BIT_MASK);
 *
*/

//VPU memory power down
#define VPU_MEM_POWER_ON		0
#define VPU_MEM_POWER_DOWN		1

extern unsigned int get_vpu_clk_vmod(unsigned int vmod);
extern int request_vpu_clk_vmod(unsigned int vclk, unsigned int vmod);
extern int release_vpu_clk_vmod(unsigned int vmod);

extern void switch_vpu_mem_pd_vmod(unsigned int vmod, int flag);

#endif
