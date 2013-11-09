/*
 * ISP driver
 *
 * Author: Kele Bai <kele.bai@amlogic.com>
 *
 * Copyright (C) 2010 Amlogic Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <mach/am_regs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/amlogic/tvin/tvin_v4l2.h>
#include "isp_regs.h"
#include "isp_hw.h"

#define DEVICE_NAME "isp"

static unsigned int gamma_enable = 1;
module_param(gamma_enable,uint,0664);
MODULE_PARM_DESC(gamma_enable,"\n enable/disable for gamma.\n");

/*
*reg 0x00~0x07
*reg 0xaf
*/
void isp_top_init(xml_top_t *top,unsigned int w,unsigned int h)
{
	unsigned short offset[XML_TOP]={
		0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0xaf
	};
	int i = 0;	
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
	if(top){
		//for(i=0;i < XML_TOP;i++)
//			WR(ISP_VCBUS_BASE+offset[i],top->reg_map[i]);
	}
	/*config input size*/
	WR(ISP_HV_SIZE,w<<REG_HSIZE_BIT|h);
        WR(ISP_HBLANK,w<<REG_TOTAL_W_BIT|10);
	return;
}
/*
*reg 0x08~0x1f
*/
void isp_set_test_pattern(xml_tp_t *tp)
{
	int i = 0;
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
	if(tp){
		for(i=0;i<XML_TP;i++)
			WR(ISP_PAT_GEN_CTRL+i,tp->reg_map[i]);
	} else {
		
	}
	return;
}
/*
*reg 0x20~0x25
*/
void isp_set_clamp_gain(xml_cg_t *cg)
{
	int i = 0;
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
	if(cg){
		for(i=0;i<XML_CG;i++)
		        WR(ISP_CLAMPGAIN_CTRL+i, cg->reg_map[i]);
	}
	return;
}
/*
*reg 0x28~0x2c
*/
void isp_set_lens_shading(xml_ls_t *lens)
{
	int i = 0;
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
	if(lens){		
		for(i=0;i<XML_LS;i++)
		        WR(ISP_LNS_CTRL+i, lens->reg_map[i]);
	}
}
/*
*reg 0x30
*/
void isp_set_gamma_correction(xml_gc_t *gc)
{
	int i = 0;
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
	if(gc){		
		for(i=0;i<XML_GC;i++)
		        WR(ISP_GMR0_CTRL+i, gc->reg_map[i]);
	}
}
/*
*reg 0x31
*reg 0x33~0x3c
*/
void isp_set_defect_pixel_correction(xml_dp_t *dpc)
{
	int i = 0;
	unsigned short offset[XML_DP]={
		0x31,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x3b,0x3c
	};
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
	if(dpc){
		for(i=0;i<XML_DP;i++)
		        WR(ISP_VCBUS_BASE+offset[i], dpc->reg_map[i]);
	}
}
/*
*reg 0x40~0x41
*/
void isp_set_demosaicing(xml_dm_t *dms)
{
	int i = 0;
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
	if(dms){
		for(i=0;i<XML_DM;i++)
		        WR(ISP_DMS_CTRL0+i, dms->reg_map[i]);
	}
	//GBRG defaulty
	WR_BITS(ISP_DMS_CTRL0,1,DMS_XPHASE_OFST_BIT,DMS_XPHASE_OFST_WID);
	WR_BITS(ISP_DMS_CTRL0,0,DMS_YPHASE_OFST_BIT,DMS_YPHASE_OFST_WID);
}
/*
*reg 0x42~0x4a
*/
void isp_set_matrix(xml_csc_t *csc, unsigned int height)
{
	unsigned int isp_matrix_lup[2][9]=
	{
		{0x0,0x10000,0x840102,0x3203b4,0x36b00e1,0xe10344,0x3dc,0x400200,0x200},
		//rgb->601
		{0x0,0x10000,0x5e013a,0x2003cc,0x35300e1,0xe10334,0x3ec,0x400200,0x200},
		//rgb->709
	};
	unsigned int i=0, *start;
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
	if(csc){
		start = &(csc->reg_map[0]);
		if(height > 720)
			start = &(csc->reg_map[XML_CSC]);
		for(i=0;i<XML_CSC;i++)
		        WR(ISP_MATRIX_PRE_OFST0_1+i, *(start+i));
	} else {
		start = (height>720)?isp_matrix_lup[1]:isp_matrix_lup[0];
		for(i=0;i<XML_CSC;i++)
		        WR(ISP_MATRIX_PRE_OFST0_1+i, *(start+i));
	}
}
/*
*reg 0x50 
*reg 0x52 0x53 0x54 0x55 0x56 0x57 0x58 0x59 0x5a 0x5b 
*reg 0x60 0x61 0x62 0x63 0x64 0x65 0x66 
*reg 0x72 0x73 0x74 0x75 0x76 0x78 0x79 0x7a 0x7b 
*reg 0x80 0x81 0x82 0x83 0x84 0x85(special effect )
*/
void isp_set_sharpness(xml_sharp_t *sharp)
{
	int i=0;
	unsigned short offset[XML_SH]={
		0x50,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x5b,
		0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x72,0x73,0x74,0x75,
		0x76,0x78,0x79,0x7a,0x7b,0x80,0x81,0x82,0x83,0x84,0x85
	};
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
	if(sharp){
		for(i=0;i<XML_SH;i++)
			WR(ISP_VCBUS_BASE+offset[i], sharp->reg_map[i]);		
	}
}
/*
*reg 0x50~0x51
*reg 0x68~0x71 
*reg 0x85
*/
void isp_set_nr(xml_nr_t *nr)
{
	int i = 0;
	unsigned short offset[XML_NR]={
		0x50,0x51,0x68,0x69,0x6a,0x6b,0x6c,
		0x6d,0x6e,0x6f,0x70,0x71,0x85
	};
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
	if(nr){
		for(i=0;i<XML_NR;i++)
			WR(ISP_VCBUS_BASE+offset[i], nr->reg_map[i]);
	}
}
/*
*reg 0x88~0x8c
*/
void isp_set_awb_stat(xml_awb_t *awbs,unsigned int w,unsigned int h)
{
	int i = 0;
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
	if(awbs){
		for(i=0;i<XML_AWB;i++)
			WR(ISP_AWB_WIND_LR+i, awbs->reg_map[i]);
	}
	/*config awb statistics window*/
	WR(ISP_AWB_WIND_LR,w);
	WR(ISP_AWB_WIND_TB,h);
}
/*
*reg 0x8c
*reg 0x90~0x94
*/
void isp_set_ae_stat(xml_ae_t *aes,unsigned int w,unsigned int h)
{
	unsigned int i=0,xstep=0,ystep=0;
	unsigned short offset[XML_AE]={
		0x8c,0x90,0x91,0x92,0x93,0x94
	};
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
	if(aes){
		for(i=0;i<XML_AE;i++)
			WR(ISP_VCBUS_BASE+offset[i],aes->reg_map[i]);
	}
	/*config ae statistics window*/
	xstep = w>>2;
	ystep = h>>2;

	WR(ISP_AEC_WIND_XYSTART,0);
	WR(ISP_AEC_WIND_XYSTEP,xstep<<13|ystep);
	WR(ISP_AECRAW_WIND_LR,w);
	WR(ISP_AECRAW_WIND_TB,h);
}
/*
*reg 0x98~0xa8
*/
void isp_set_af_stat(xml_af_t *afs,unsigned int w,unsigned int h)
{
	unsigned int i=0,tmp_w=0,tmp_h=0;
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
	if(afs){
		for(i=0;i<XML_AF;i++)
			WR(ISP_AFC_FILTER_SEL+i, afs->reg_map[i]);
	}
	WR(ISP_AFC_FILTER_SEL,0x17722);
    /*config win0~win8 according to hv size*/
	tmp_w = w/12;
	tmp_h = h/12;
	/* wind0 h:2-5  v:2-5*/
	WR(ISP_AFC_WIND0_LR,(tmp_w<<1)<<16|tmp_w*5);
	WR(ISP_AFC_WIND0_TB,(tmp_h<<1)<<16|tmp_h*5);
	/* wind1 h:5-8  v:5-8*/
	WR(ISP_AFC_WIND1_LR,(tmp_w*5)<<16|tmp_w<<3);
	WR(ISP_AFC_WIND1_TB,(tmp_h*5)<<16|tmp_h<<3);
	/* wind2 h:8-11 v:2-5*/
	WR(ISP_AFC_WIND2_LR,(tmp_w<<3)<<16|tmp_w*11);
	WR(ISP_AFC_WIND2_TB,(tmp_h<<1)<<16|tmp_h*5);
	/* wind3 h:2-5  v:8-11*/
	WR(ISP_AFC_WIND3_LR,(tmp_w<<1)<<16|tmp_w*5);
	WR(ISP_AFC_WIND3_TB,(tmp_h<<3)<<16|tmp_h*11);
	/*wind4 h:8-11 v:8-11*/
	WR(ISP_AFC_WIND4_LR,(tmp_w<<3)<<16|tmp_w*11);
	WR(ISP_AFC_WIND4_TB,(tmp_h<<3)<<16|tmp_h*11);	
}
/*
*reg 0xac~0xae
*/
void isp_set_blenr_stat(unsigned int x0,unsigned int y0,unsigned int x1,unsigned int y1)
{
	#if 0
	int i = 0;
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
	if(blenrs){
		for(i=0;i<XML_BN;i++)
			WR(ISP_BLNR_CTRL+i, blenrs->reg_map[i]);
	}
	#endif
	/*set lpf according to sd or hd*/
	WR_BITS(ISP_BLNR_CTRL,0,BLNR_STATISTICS_EN_BIT,BLNR_STATISTICS_EN_WID);
	if(abs(y1-y0) > 720)
		WR_BITS(ISP_BLNR_CTRL,3,BLNR_LPF_MODE_BIT,BLNR_LPF_MODE_WID);
	else
		WR_BITS(ISP_BLNR_CTRL,1,BLNR_LPF_MODE_BIT,BLNR_LPF_MODE_WID);
	/*set ac adaptive*/
	WR_BITS(ISP_BLNR_CTRL,1,BLNR_AC_ADAPTIVE_BIT,BLNR_AC_ADAPTIVE_WID);
	WR(ISP_BLNR_WIND_LR, x1|x0<<16);
	WR(ISP_BLNR_WIND_TB, y1|y0<<16);
	WR_BITS(ISP_BLNR_CTRL,1,BLNR_STATISTICS_EN_BIT,BLNR_STATISTICS_EN_WID);
	
}
/*
*reg 0xb0~0xb1
*/
void isp_set_dbg(xml_dbg_t *dbg)
{
	int i = 0;
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
	if(dbg){
		for(i=0;i<XML_DBG;i++)
			WR(ISP_DBG_PIXEL_CTRL+i, dbg->reg_map[i]);
	}
}
static void isp_set_lnsd(xml_lut_ls_t *lnsd)
{
	int i = 0;
	if(lnsd){
		for(i=0;i<XML_LUT_LS;i++){
			WR(ISP_LNSD_LUT_ADDR,i);
			WR(ISP_LNSD_LUT_DATA,lnsd->reg_map[i]);
		}
	}
}
void isp_set_lnsd_mode(unsigned int mode)
{
	WR_BITS(ISP_LNS_CTRL,mode,LNS_MESH_MODE_BIT,LNS_MESH_MODE_WID);
}
static void isp_set_gamma_table(struct xml_lut_gc_s *gt)
{
	if(gt){
		set_isp_gamma_table(gt->gamma_r,GAMMA_R);
		set_isp_gamma_table(gt->gamma_g,GAMMA_G);
		set_isp_gamma_table(gt->gamma_b,GAMMA_B);
	}else{
		pr_info("%s:null pointer error.\n",__func__);
	}
}
/*
*
*/
void isp_set_def_config(xml_default_regs_t *regs,tvin_port_t fe_port,unsigned int w,unsigned int h)
{
	unsigned int mux = 0;
	switch(fe_port){			
		case TVIN_PORT_CAMERA:				
			mux = 1;				
			break;			
		case TVIN_PORT_MIPI:	
			mux = 2;				
			break;			
		default:				
			mux = 0;				
			break;		
	}
	WR_BITS(VPU_MISC_CTRL,mux,ISP_IN_SEL_BIT,ISP_IN_SEL_WID);
	isp_top_init(&regs->top,w,h);
	isp_set_test_pattern(&regs->tp);
	isp_set_clamp_gain(&regs->cg);
	isp_set_lens_shading(&regs->ls);
	isp_set_gamma_correction(&regs->gc);
	isp_set_defect_pixel_correction(&regs->dp);
	isp_set_demosaicing(&regs->dm);
	isp_set_matrix(NULL,h);
	isp_set_sharpness(&regs->sharp);
	isp_set_nr(&regs->nr);
	isp_set_blenr_stat(0,0,w-1,h-1);
	isp_set_awb_stat(&regs->awb_reg,w,h);
	isp_set_ae_stat(&regs->ae_reg,w,h);
	isp_set_af_stat(&regs->af_reg,w,h);
	//isp_set_dbg(&regs->dbg);
	isp_set_lnsd(&regs->lnsd);
	isp_set_gamma_table(&regs->lut_gc);
	pr_info("[%s..]%s: init ok.\n",DEVICE_NAME,__func__);
}
/*
*just enable test pattern
*/
void isp_set_init(unsigned int hsize,unsigned int vsize,unsigned int htotal,unsigned int vtotal)
{
        // pat gen
        WRITE_VCBUS_REG(ISP_PAT_GEN_CTRL, 0x0);
        WRITE_VCBUS_REG(ISP_PAT_XRAMP_SCAL,0x00ffffff);
        WRITE_VCBUS_REG(ISP_PAT_YRAMP_SCAL,0x00ffffff);
        WRITE_VCBUS_REG(ISP_PAT_XYIDX_OFST,0x0 );
        WRITE_VCBUS_REG(ISP_PAT_XYIDX_SCAL,0x0022003d); //default=0
        WRITE_VCBUS_REG(ISP_PAT_BAR16_RED0,0x0000ffff);
        WRITE_VCBUS_REG(ISP_PAT_BAR16_RED1,0x0000ffff);
        WRITE_VCBUS_REG(ISP_PAT_BAR16_RED2,0x80604020);
        WRITE_VCBUS_REG(ISP_PAT_BAR16_RED3,0xffe0c0a0);
        WRITE_VCBUS_REG(ISP_PAT_BAR16_GRN0,0x0000ffff); //defaulr is all 'hff
        WRITE_VCBUS_REG(ISP_PAT_BAR16_GRN1,0x00000000);
        WRITE_VCBUS_REG(ISP_PAT_BAR16_GRN2,0x80604020);
        WRITE_VCBUS_REG(ISP_PAT_BAR16_GRN3,0xffe0c0a0);
        WRITE_VCBUS_REG(ISP_PAT_BAR16_BLU0,0x00ff00ff);
        WRITE_VCBUS_REG(ISP_PAT_BAR16_BLU1,0x00ff00ff);
        WRITE_VCBUS_REG(ISP_PAT_BAR16_BLU2,0x80604020);
        WRITE_VCBUS_REG(ISP_PAT_BAR16_BLU3,0xffe0c0a0);

        WRITE_VCBUS_REG_BITS(ISP_PAT_GEN_CTRL,  0 ,20,3); // xmode: raster/bar16/burst enable
        WRITE_VCBUS_REG_BITS(ISP_PAT_GEN_CTRL,  2 ,16,3); // ymode: 1023

        // pat dft
        WRITE_VCBUS_REG(ISP_PAT_DFT_XYIDX,0x006400c8 ); //reg_isp_pat_dft_xidx='h64,reg_isp_pat_dft_yidx='hc8
        WRITE_VCBUS_REG(ISP_PAT_DFT_XYWID,0x00010004 ); //default reg_isp_pat_dft_ywid=64,default is wrong?
        WRITE_VCBUS_REG(ISP_PAT_DFT_GAIN, 0x0);
        WRITE_VCBUS_REG(ISP_PAT_HVTOTAL,(vtotal<<16)|(htotal));
        WRITE_VCBUS_REG(ISP_PAT_VDE_SLINE,0x00000007 );
        // demosaicing
        WRITE_VCBUS_REG(ISP_DMS_CTRL0, 0x00030000);
        WRITE_VCBUS_REG(ISP_DMS_CTRL1, 0x00120510);

        // color matrix
        WRITE_VCBUS_REG(ISP_MATRIX_PRE_OFST0_1,0x0);
        WRITE_VCBUS_REG(ISP_MATRIX_PRE_OFST2,  0x0);
        WRITE_VCBUS_REG(ISP_MATRIX_COEF00_01, 0x004d0096);
        WRITE_VCBUS_REG(ISP_MATRIX_COEF02_10, 0x001d03d5);
        WRITE_VCBUS_REG(ISP_MATRIX_COEF11_12, 0x03ab0080);
        WRITE_VCBUS_REG(ISP_MATRIX_COEF20_21, 0x00800395);
        WRITE_VCBUS_REG(ISP_MATRIX_COEF22,    0x000003eb);
        WRITE_VCBUS_REG(ISP_MATRIX_POS_OFST0_1, 0x00000200);
        WRITE_VCBUS_REG(ISP_MATRIX_POS_OFST2,   0x00000200);
		
        WRITE_VCBUS_REG(ISP_RST_DLY_NUM,htotal*6);
        WRITE_VCBUS_REG_BITS(ISP_PAT_GEN_CTRL,1,28,1);
}

/*
*wave init
*/
void wave_init(wave_t *wave_param)
{
	static bool init_flag = false;
        if (!init_flag) {
    	        init_flag = true;
                // clock_div & clock disable
                WRITE_CBUS_REG(HHI_ISP_LED_CLK_CNTL, wave_param->wave_clock_div);
	        // disable isp_led2_en
	        WRITE_CBUS_REG_BITS(PERIPHS_PIN_MUX_9,0,17,1); // clear pinmux_9[17] for isp_led2_en
	        // enable isp_led1_en
	        WRITE_CBUS_REG_BITS(PERIPHS_PIN_MUX_5,0,31,1); // clear pinmux_5[31] for I2C_SDA_A
	        WRITE_CBUS_REG_BITS(PERIPHS_PIN_MUX_7,0,25,1); // clear pinmux_7[25] for PWM_VS
	        WRITE_CBUS_REG_BITS(PERIPHS_PIN_MUX_9,0,16,1); // clear pinmux_9[16] for PWM_A
	        WRITE_CBUS_REG_BITS(PERIPHS_PIN_MUX_9,1,18,1); // set pinmux_9[18] for isp_led1_en
        }
}

void flash_init(bool mode_pol_inv,bool led1_pol_inv,bool pin_mux_inv,wave_t *wave_param)
{
        wave_init(wave_param);
        // MODE pin
        //if (mode_pol_inv)
		//set_mode_low();
	//else
		//set_mode_high();
	// EN/SET pin
	WRITE_CBUS_REG(ISP_LED_CTRL, (led1_pol_inv<<29)|(pin_mux_inv<<27)|(1<<26)|(1<<24));
        WRITE_CBUS_REG(ISP_LED_TIMING1, 0);
        WRITE_CBUS_REG(ISP_LED_TIMING2, 0);
        WRITE_CBUS_REG(ISP_LED_TIMING3, wave_param->latch_time_timeout);
        WRITE_CBUS_REG(ISP_LED_TIMING4, wave_param->latch_time);
        WRITE_CBUS_REG(ISP_LED_TIMING5, 0);
        WRITE_CBUS_REG(ISP_LED_TIMING6, wave_param->time_to_latch);
}

void torch_init(bool mode_pol_inv,bool led1_pol_inv,bool pin_mux_inv,bool torch_pol_inv,wave_t *wave_param,unsigned int level)
{
        unsigned int pulse_qty = 0;
	
	wave_init(wave_param);
	// MODE pin
	//if (mode_pol_inv)
		//set_mode_high();
	//else
		//set_mode_low();
	// EN/SET pin
	pulse_qty = level*wave_param->pulse_qty_max/100;
	if (pulse_qty > wave_param->pulse_qty_max - 1)
		pulse_qty = wave_param->pulse_qty_max - 1;
	if (!torch_pol_inv)
		pulse_qty = wave_param->pulse_qty_max - 1 - pulse_qty;
	WRITE_CBUS_REG(ISP_LED_CTRL, (led1_pol_inv<<29)|(pin_mux_inv<<27)|(1<<26)|(pulse_qty<<12)|(wave_param->pulse_init_time<<0));
	WRITE_CBUS_REG(ISP_LED_TIMING1, 0);
	WRITE_CBUS_REG(ISP_LED_TIMING2, (wave_param->pulse_high_time<<21)|(wave_param->pulse_low_time<<10));
	WRITE_CBUS_REG(ISP_LED_TIMING3, 0);
	WRITE_CBUS_REG(ISP_LED_TIMING4, 0x03ffffff);
	WRITE_CBUS_REG(ISP_LED_TIMING5, (wave_param->time_to_off&0x0000003f)<<26);
	WRITE_CBUS_REG(ISP_LED_TIMING6, ((wave_param->time_to_off>>6)<<26)|wave_param->time_to_latch);
}

static void wave_on(bool enable)
{
	if(enable) {
        WRITE_CBUS_REG_BITS(ISP_LED_CTRL,1,31,1);
        WRITE_CBUS_REG_BITS(HHI_ISP_LED_CLK_CNTL,1,16,1);
    } else
		WRITE_CBUS_REG_BITS(ISP_LED_CTRL,0,31,1);
}

void flash_on(bool mode_pol_inv,bool led1_pol_inv,bool pin_mux_inv,wave_t *wave_param)
{
        wave_on(false);
	flash_init(mode_pol_inv,led1_pol_inv,pin_mux_inv,wave_param);
	wave_on(true);
}

void torch_level(bool mode_pol_inv,bool led1_pol_inv,bool pin_mux_inv,bool torch_pol_inv,wave_t *wave_param,unsigned int level)
{
        wave_on(false);
	// torch on
        if (level) {
	        torch_init(mode_pol_inv,led1_pol_inv,pin_mux_inv,torch_pol_inv,wave_param,level);
		wave_on(true);
	}
}


void wave_power_manage(bool enable)
{
	if(enable)
    	// clock enable
    	        WRITE_CBUS_REG_BITS(HHI_ISP_LED_CLK_CNTL,1,16,1);
	else
	// clock disable
    	        WRITE_CBUS_REG_BITS(HHI_ISP_LED_CLK_CNTL,0,16,1);
}

/*only adjust g0 r1 b2 g3*/
void isp_set_manual_wb(xml_wb_manual_t *wb)
{
	WR(ISP_GAIN_GRBG01,wb->reg_map[0]);
	WR(ISP_GAIN_GRBG23,wb->reg_map[1]);
}

void isp_wr(unsigned int addr,unsigned int data)
{
	WR(ISP_RO_ADDR_PORT,addr);
	WR(ISP_RO_DATA_PORT,data);
}

unsigned int isp_rd(unsigned int addr)
{
	WR(ISP_RO_ADDR_PORT,addr);
        return(RD(ISP_RO_DATA_PORT));	
}
/*
*reg 0x00~0x1b
*/
void isp_get_awb_stat(isp_awb_stat_t *awb_stat)
{
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_RED_SUM);
	awb_stat->rgb.rgb_sum[0] = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_GRN_SUM);
	awb_stat->rgb.rgb_sum[1] = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_BLU_SUM);
	awb_stat->rgb.rgb_sum[2] = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_RGB_NUM);
	awb_stat->rgb.rgb_count = RD_BITS(ISP_RO_DATA_PORT,AWB_RGB_NUM_BIT,AWB_RGB_NUM_WID);

	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_LOW_UNEG_SUM);
	awb_stat->yuv_low[0].sum = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_LOW_VNEG_SUM);
	awb_stat->yuv_low[1].sum = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_LOW_UPOS_SUM);
	awb_stat->yuv_low[2].sum = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_LOW_VPOS_SUM);
	awb_stat->yuv_low[3].sum = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_LOW_UNEG_NUM);
	awb_stat->yuv_low[0].count = RD_BITS(ISP_RO_DATA_PORT,AWB_LOW_UNEG_NUM_BIT,AWB_LOW_UNEG_NUM_WID);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_LOW_VNEG_NUM);
	awb_stat->yuv_low[1].count = RD_BITS(ISP_RO_DATA_PORT,AWB_LOW_VNEG_NUM_BIT,AWB_LOW_VNEG_NUM_WID);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_LOW_UPOS_NUM);
	awb_stat->yuv_low[2].count = RD_BITS(ISP_RO_DATA_PORT,AWB_LOW_UPOS_NUM_BIT,AWB_LOW_UPOS_NUM_WID);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_LOW_VPOS_NUM);
	awb_stat->yuv_low[3].count = RD_BITS(ISP_RO_DATA_PORT,AWB_LOW_VPOS_NUM_BIT,AWB_LOW_VPOS_NUM_WID);

	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_MID_UNEG_SUM);
	awb_stat->yuv_mid[0].sum = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_MID_VNEG_SUM);
	awb_stat->yuv_mid[1].sum = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_MID_UPOS_SUM);
	awb_stat->yuv_mid[2].sum = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_MID_VPOS_SUM);
	awb_stat->yuv_mid[3].sum = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_MID_UNEG_NUM);
	awb_stat->yuv_mid[0].count = RD_BITS(ISP_RO_DATA_PORT,AWB_MID_UNEG_NUM_BIT,AWB_MID_UNEG_NUM_WID);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_MID_VNEG_NUM);
	awb_stat->yuv_mid[1].count = RD_BITS(ISP_RO_DATA_PORT,AWB_MID_VNEG_NUM_BIT,AWB_MID_VNEG_NUM_WID);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_MID_UPOS_NUM);
	awb_stat->yuv_mid[2].count = RD_BITS(ISP_RO_DATA_PORT,AWB_MID_UPOS_NUM_BIT,AWB_MID_UPOS_NUM_WID);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_MID_VPOS_NUM);
	awb_stat->yuv_mid[3].count = RD_BITS(ISP_RO_DATA_PORT,AWB_MID_VPOS_NUM_BIT,AWB_MID_VPOS_NUM_WID);

	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_HIG_UNEG_SUM);
	awb_stat->yuv_high[0].sum = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_HIG_VNEG_SUM);
	awb_stat->yuv_high[1].sum = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_HIG_UPOS_SUM);
	awb_stat->yuv_high[2].sum = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_HIG_VPOS_SUM);
	awb_stat->yuv_high[3].sum = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_HIG_UNEG_NUM);
	awb_stat->yuv_high[0].count = RD_BITS(ISP_RO_DATA_PORT,AWB_HIG_UNEG_NUM_BIT,AWB_HIG_UNEG_NUM_WID);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_HIG_VNEG_NUM);
	awb_stat->yuv_high[1].count = RD_BITS(ISP_RO_DATA_PORT,AWB_HIG_VNEG_NUM_BIT,AWB_HIG_VNEG_NUM_WID);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_HIG_UPOS_NUM);
	awb_stat->yuv_high[2].count = RD_BITS(ISP_RO_DATA_PORT,AWB_HIG_UPOS_NUM_BIT,AWB_HIG_UPOS_NUM_WID);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AWB_HIG_VPOS_NUM);
	awb_stat->yuv_high[3].count = RD_BITS(ISP_RO_DATA_PORT,AWB_HIG_VPOS_NUM_BIT,AWB_HIG_VPOS_NUM_WID);

	return;
}
/*
*reg 0x1c~0x2e
*/
void isp_get_ae_stat(isp_ae_stat_t *ae_stat)
{
	int i = 0;
	/*0x1c~0x2b*/
	for(i=0;i<16;i++){
		WR(ISP_RO_ADDR_PORT, ISP_RO_AEC_LUMA_WIND0_0+i);
		ae_stat->luma_win[i] = RD(ISP_RO_DATA_PORT);
	}

	WR(ISP_RO_ADDR_PORT, ISP_RO_AECRAW_NUM_RED);
	ae_stat->bayer_over_info[0] = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AECRAW_NUM_GREEN);
	ae_stat->bayer_over_info[1] = RD(ISP_RO_DATA_PORT);
	WR(ISP_RO_ADDR_PORT, ISP_RO_AECRAW_NUM_BLUE);
	ae_stat->bayer_over_info[2] = RD(ISP_RO_DATA_PORT);
	
	return;
}

void isp_set_ae_win(unsigned int left, unsigned int right, unsigned int top, unsigned int bottom)
{
	WR_BITS(ISP_AECRAW_WIND_LR, left, AECRAW_WIND_LEFT_BIT, AECRAW_WIND_LEFT_WID);
	WR_BITS(ISP_AECRAW_WIND_LR, right, AECRAW_WIND_RIGHT_BIT, AECRAW_WIND_RIGHT_WID);
	WR_BITS(ISP_AECRAW_WIND_TB, top, AECRAW_WIND_TOP_BIT, AECRAW_WIND_TOP_WID);
	WR_BITS(ISP_AECRAW_WIND_TB, bottom, AECRAW_WIND_BOT_BIT, AECRAW_WIND_BOT_WID);
	
	WR_BITS(ISP_AEC_WIND_XYSTART, left, AEC_WIND_XSTART_BIT, AEC_WIND_XSTART_WID);
	WR_BITS(ISP_AEC_WIND_XYSTART, top, AEC_WIND_YSTART_BIT, AEC_WIND_YSTART_WID);
	WR_BITS(ISP_AEC_WIND_XYSTEP, (right-left+1)>>2, AEC_WIND_XSTEP_BIT, AEC_WIND_XSTEP_WID);
	WR_BITS(ISP_AEC_WIND_XYSTEP, (bottom-top+1)>>2, AEC_WIND_YSTEP_BIT, AEC_WIND_YSTEP_WID);
}

void isp_set_awb_win(unsigned int left, unsigned int right, unsigned int top, unsigned int bottom)
{
	WR_BITS(ISP_AWB_WIND_LR, left, AWB_WIND_LEFT_BIT, AWB_WIND_LEFT_BIT);
	WR_BITS(ISP_AWB_WIND_LR, right, AWB_WIND_RIGHT_BIT, AWB_WIND_RIGHT_WID);
	WR_BITS(ISP_AWB_WIND_TB, top, AWB_WIND_TOP_BIT, AWB_WIND_TOP_WID);
	WR_BITS(ISP_AWB_WIND_TB, bottom, AWB_WIND_BOT_BIT, AWB_WIND_BOT_WID);
}

void isp_set_ae_thrlpf(unsigned char thr_r, unsigned char thr_g, unsigned char thr_b, unsigned char lpf)
{
	WR_BITS(ISP_AEC_THRESHOLDS, thr_r, AEC_RAWBRIGHT_R_BIT, AEC_RAWBRIGHT_R_WID);
	WR_BITS(ISP_AEC_THRESHOLDS, thr_g, AEC_RAWBRIGHT_G_BIT, AEC_RAWBRIGHT_G_WID);
	WR_BITS(ISP_AEC_THRESHOLDS, thr_b, AEC_RAWBRIGHT_B_BIT, AEC_RAWBRIGHT_B_WID);
	WR_BITS(ISP_CLAMPGAIN_CTRL, lpf, AECRAW_LPF_SEL_BIT, AECRAW_LPF_SEL_WID);
}

void isp_set_awb_yuv_thr(unsigned char yh, unsigned char yl, unsigned char u, unsigned char v)
{
	WR_BITS(ISP_AWB_UVTH_YPIECE, yh, AWB_YPIECE_HIG_BIT, AWB_YPIECE_HIG_WID);
	WR_BITS(ISP_AWB_UVTH_YPIECE, yl, AWB_YPIECE_LOW_BIT, AWB_YPIECE_LOW_WID);
	WR_BITS(ISP_AWB_UVTH_YPIECE, u, AWB_U_THRD_BIT, AWB_U_THRD_WID);
	WR_BITS(ISP_AWB_UVTH_YPIECE, v, AWB_V_THRD_BIT, AWB_V_THRD_WID);	
}

void isp_set_awb_rgb_thr(unsigned char gb, unsigned char gr, unsigned br)
{
	WR_BITS(ISP_AWB_GBGRBR_THRD, gb, AWB_GB_THRD_BIT, AWB_GB_THRD_WID);
	WR_BITS(ISP_AWB_GBGRBR_THRD, gr, AWB_GR_THRD_BIT, AWB_GR_THRD_WID);
	WR_BITS(ISP_AWB_GBGRBR_THRD, br, AWB_BR_THRD_BIT, AWB_BR_THRD_WID);	
}

void isp_get_af_stat(isp_af_stat_t * af_stat)
{
	int i = 0;
	/*0xbc~0xc6*/
	if(IS_ERR_OR_NULL(af_stat)){
		pr_info("%s null pointer error.\n",__func__);
	} else {
	for(i=0;i<16;i++)
		af_stat->luma_win[i]=isp_rd(0xbc + i);
	}
	return;
}

void isp_get_blnr_stat(isp_blnr_stat_t *blnr_stat)
{
	int i = 0;
	for(i=0;i<4;i++){
		blnr_stat->dc[i] = isp_rd(ISP_RO_BLNR_GRBG_DCSUM0+i);
		blnr_stat->ac[i] = isp_rd(ISP_RO_BLNR_GRBG_ACSUM0+i);
	}
	
	return;
}

void isp_hw_reset()
{
	WR_BITS(ISP_TIMING_MODE,1,5,1);			
	WR_BITS(ISP_FRM_SOFT_RST,1,0,1);			
	WR_BITS(ISP_FRM_SOFT_RST,0,0,1);			
	WR_BITS(ISP_FRM_SOFT_RST,1,0,1);			
	WR_BITS(ISP_FRM_SOFT_RST,0,0,1);			
	WR_BITS(ISP_TIMING_MODE,0,5,1);
}

void isp_awb_set_gain(unsigned int r,unsigned int g,unsigned int b)
{
	WR_BITS(ISP_GAIN_GRBG01, g, GAIN_GRBG0_BIT, GAIN_GRBG0_WID);
	WR_BITS(ISP_GAIN_GRBG01, r, GAIN_GRBG1_BIT, GAIN_GRBG1_WID);
	WR_BITS(ISP_GAIN_GRBG23, b, GAIN_GRBG2_BIT, GAIN_GRBG2_WID);	
	WR_BITS(ISP_GAIN_GRBG23, g, GAIN_GRBG3_BIT, GAIN_GRBG3_WID);
}

void isp_awb_get_gain(isp_awb_gain_t *awb_gain)
{
	awb_gain->b_val = RD_BITS(ISP_GAIN_GRBG23, GAIN_GRBG2_BIT, GAIN_GRBG2_WID);
	awb_gain->r_val = RD_BITS(ISP_GAIN_GRBG01, GAIN_GRBG1_BIT, GAIN_GRBG1_WID);
	awb_gain->g_val = RD_BITS(ISP_GAIN_GRBG01, GAIN_GRBG0_BIT, GAIN_GRBG0_WID);
}

void set_isp_gamma_table(unsigned short *gamma,unsigned int type)
{
	unsigned int flag = 0,i = 0; 

        // store gamma table enable/disable status
        flag = (RD_BITS(ISP_GMR0_CTRL,GMR_CORRECT_ENABLE_BIT,GMR_CORRECT_ENABLE_WID))|gamma_enable;

        // gamma table disable, gamma table vbus mode
        WR_BITS(ISP_GMR0_CTRL,0,GMR_CORRECT_ENABLE_BIT,GMR_CORRECT_ENABLE_WID);
	WR_BITS(ISP_GMR0_CTRL,1,GCLUT_ACCMODE_BIT,GCLUT_ACCMODE_WID);
        // point to gamma
        WR(ISP_GAMMA_LUT_ADDR, type);
        // write gamma
        for (i = 0; i < 257; i++) {
                WR(ISP_GAMMA_LUT_DATA, gamma[i]);
        }

        // retrieve gamma table enable/disable status, gamma table hardware mode
        WR_BITS(ISP_GMR0_CTRL, flag,GMR_CORRECT_ENABLE_BIT,GMR_CORRECT_ENABLE_WID);
	WR_BITS(ISP_GMR0_CTRL,0,GCLUT_ACCMODE_BIT,GCLUT_ACCMODE_WID);
	
}
void get_isp_gamma_table(unsigned short *gamma,unsigned int type)
{
	unsigned int flag = 0,i = 0; 

        // store gamma table enable/disable status
        flag = RD_BITS(ISP_GMR0_CTRL,GMR_CORRECT_ENABLE_BIT,GMR_CORRECT_ENABLE_WID);

        // gamma table disable, gamma table vbus mode
        WR_BITS(ISP_GMR0_CTRL,0,GMR_CORRECT_ENABLE_BIT,GMR_CORRECT_ENABLE_WID);
	WR_BITS(ISP_GMR0_CTRL,1,GCLUT_ACCMODE_BIT,GCLUT_ACCMODE_WID);
        // point to gamma
        WR(ISP_GAMMA_LUT_ADDR, type);
        // get gamma
        for (i = 0; i < 257; i++) {
                gamma[i] = RD(ISP_GAMMA_LUT_DATA);
        }

        // retrieve gamma table enable/disable status, gamma table hardware mode
        WR_BITS(ISP_GMR0_CTRL, flag,GMR_CORRECT_ENABLE_BIT,GMR_CORRECT_ENABLE_WID);
	WR_BITS(ISP_GMR0_CTRL,0,GCLUT_ACCMODE_BIT,GCLUT_ACCMODE_WID);
}

