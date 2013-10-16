#ifndef __VPU_H__
#define __VPU_H__

//#define CONFIG_VPU_DYNAMIC_ADJ

typedef struct {
	unsigned int h_res;
	unsigned int v_res;
	unsigned int refresh_rate;
	unsigned int clk_level;
}VPU_Conf_t;

#define VPU_MODE_BIT_MASK	0x1000
#define VPU_MODE_NONE		0x8000
typedef enum {
    VPU_MODE_ENCP = 0,
    VPU_MODE_ENCI,
	VPU_MODE_ENCL,
	VPU_MODE_VIDEO,
	VPU_MODE_OSD,
	VPU_MODE_VPP,
	VPU_MODE_MAX,
} vpu_mode_t;

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
extern unsigned int get_vpu_clk_vmod(unsigned int vmod);
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
extern int request_vpu_clk_vomd(unsigned int vclk, unsigned int vmod);
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
extern int release_vpu_clk_vomd(unsigned int vmod);

#endif
