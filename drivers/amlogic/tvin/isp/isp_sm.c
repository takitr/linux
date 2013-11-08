/*
* ISP 3A State Machine
*
* Author: Kele Bai <kele.bai@amlogic.com>
*
* Copyright (C) 2010 Amlogic Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/
#include <linux/amlogic/tvin/tvin_v4l2.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "isp_drv.h"
#include "isp_hw.h"
#include "isp_sm.h"

#define DEVICE_NAME "isp"

static struct isp_sm_s sm_state;
static unsigned int sm_debug = 0;
static unsigned int ae_debug = 0;

module_param(ae_debug,uint,0664);
MODULE_PARM_DESC(ae_debug,"\n debug flag for ae.\n");

static unsigned int ae_debug1 = 0;

module_param(ae_debug1,uint,0664);
MODULE_PARM_DESC(ae_debug1,"\n debug flag for ae.\n");

static unsigned int ae_debug2 = 0;

module_param(ae_debug2,uint,0664);
MODULE_PARM_DESC(ae_debug2,"\n debug flag for ae.\n");

static unsigned int ae_debug3 = 0;

module_param(ae_debug3,uint,0664);
MODULE_PARM_DESC(ae_debug3,"\n debug flag for ae.\n");

static unsigned int ae_debug4 = 0;

module_param(ae_debug4,uint,0664);
MODULE_PARM_DESC(ae_debug4,"\n debug flag for ae.\n");

static unsigned int ae_debug5 = 0;

module_param(ae_debug5,uint,0664);
MODULE_PARM_DESC(ae_debug5,"\n debug flag for ae.\n");

static unsigned int ae_debug6 = 0;

module_param(ae_debug6,uint,0664);
MODULE_PARM_DESC(ae_debug6,"\n debug flag for ae.\n");

static unsigned int ae_step = 1;

module_param(ae_step,uint,0664);
MODULE_PARM_DESC(ae_step,"\n debug flag for ae.\n");

static unsigned int awb_debug = 0;

module_param(awb_debug,uint,0664);
MODULE_PARM_DESC(awb_debug,"\n debug flag for awb.\n");

static unsigned int awb_debug1 = 0;

module_param(awb_debug1,uint,0664);
MODULE_PARM_DESC(awb_debug1,"\n debug flag for awb.\n");

static unsigned int awb_debug2 = 0;

module_param(awb_debug2,uint,0664);
MODULE_PARM_DESC(awb_debug2,"\n debug flag for awb.\n");

static unsigned int awb_debug3 = 1;

module_param(awb_debug3,uint,0664);
MODULE_PARM_DESC(awb_debug3,"\n debug flag for awb.\n");

static unsigned int awb_debug4 = 0;

module_param(awb_debug4,uint,0664);
MODULE_PARM_DESC(awb_debug4,"\n debug flag for awb.\n");

#define AF_FLAG_PR			0x00000001
#define AF_FLAG_AE			0x00000002
#define AF_FLAG_AWB			0x00000004

static unsigned int best_step_debug = 0;

static unsigned int af_sm_dg = 0;


volatile struct isp_ae_to_sensor_s ae_sens;


static inline int find_step(cam_function_t *func, unsigned int low, unsigned int hign, unsigned int gain)
{
	unsigned int mid;
	unsigned int rate;
	while(hign >= low)
	{
		mid = (hign + low)/2;
		if(func&&func->get_aet_gain_by_step)
			rate = func->get_aet_gain_by_step(mid);
		if(0)
			printk("mid = %d,rate = %d, gain = %d,%d,%d,%d,%d\n",mid,rate,gain,hign,low,func,func->get_aet_gain_by_step);
		if(gain < rate)
			hign = mid - 1;
		else if(gain > rate)
			low = mid + 1;
		else
			return mid;
	}
	return mid + 1;
}

void isp_sm_init(isp_dev_t *devp)
{
	sm_state.isp_ae_parm.tf_ratio = devp->wave->torch_flash_ratio; 
	sm_state.status = ISP_AE_STATUS_NULL;
	sm_state.flash = ISP_FLASH_STATUS_NULL;
	sm_state.isp_ae_parm.isp_ae_state = AE_INIT;
	sm_state.isp_ae_parm.isp_ae_enh_state = AE_ENH_INIT;
	sm_state.isp_awb_parm.isp_awb_state = AWB_INIT;
	sm_state.env = ENV_NULL;
	sm_state.ae_down = true;
	/*init for af*/
	/*init for wave*/
	sm_state.cap_sm.fr_time = devp->wave->flash_rising_time;
	sm_state.cap_sm.tr_time = devp->wave->torch_rising_time;
}

void af_sm_init(isp_dev_t *devp)
{
	/*init for af*/
	if(sm_state.af_state)
		devp->flag |= devp->af_info.flag_bk;
	if(devp->flag & ISP_FLAG_TOUCH_AF){
    	sm_state.af_state = AF_INIT;
		devp->af_info.f = devp->af_info.af_data;
	}else{
		sm_state.af_state = AF_DETECT_INIT;
		devp->af_info.f = devp->af_info.af_detect;
	}
	devp->af_info.fv_aft_af = 0;
	devp->af_info.fv_bf_af = 0;
}
void isp_ae_low_gain()
{
	sm_state.isp_ae_parm.isp_ae_state = AE_LOW_GAIN;
}

void isp_ae_base_sm(isp_dev_t *devp)
{
	struct isp_ae_stat_s *ae = &devp->isp_ae;
	struct xml_algorithm_ae_s *aep = devp->isp_ae_parm;
	struct isp_info_s *parm = &devp->info;
	struct isp_ae_sm_s *aepa = &sm_state.isp_ae_parm;
	struct cam_function_s *func = &devp->cam_param->cam_function;
	u8  sub_avg[16] = {0};
	unsigned int avg_sum = 0;
	unsigned int avg_env_sum = 0;
	unsigned int avg_env;
	unsigned int avg_envo;
	int avg, avgo;
	u8  targ;
	int i;
	static k = 0;
	int step = 0;
	unsigned short targrate;
	unsigned int targstep, newstep;
	u8 lpfcoef;
	u8 radium_outer;
	u8 radium_inner;

	//k++;

    switch(sm_state.isp_ae_parm.isp_ae_state){
		case AE_IDLE:
			break;
		case AE_INIT:
			aepa->win_l = (parm->h_active * aep->ratio_winl) >> 10;
			aepa->win_r = ((parm->h_active * aep->ratio_winr) >> 10) - 1;
			aepa->win_t = (parm->v_active * aep->ratio_wint) >> 10;
			aepa->win_b = ((parm->v_active * aep->ratio_winb) >> 10) - 1;
			printk("ae,win_l=%d,win_r=%d,win_t=%d,win_b=%d\n",aepa->win_l,aepa->win_r,aepa->win_t,aepa->win_b);
			isp_set_ae_win(aepa->win_l, aepa->win_r, aepa->win_t, aepa->win_b);
			isp_set_ae_thrlpf(aep->thr_r_mid, aep->thr_g_mid, aep->thr_b_mid, aep->lpftype_mid);
			aepa->pixel_sum = parm->h_active * parm->v_active;
			aepa->sub_pixel_sum = aepa->pixel_sum >> 4;
			aepa->alert_r = ((aepa->pixel_sum >> 4) * aep->ratio_r) >> 8;
			aepa->alert_g = ((aepa->pixel_sum >> 4) * aep->ratio_g) >> 7;	 //grgb
			aepa->alert_b = ((aepa->pixel_sum >> 4) * aep->ratio_b) >> 8;
			printk("aepa->alert_r=%d,g=%d,b=%d\n",aepa->alert_r,aepa->alert_g,aepa->alert_b);
			aepa->change_step = 0;
			if(func&&func->get_aet_max_gain)
			aepa->max_gain = func->get_aet_max_gain();
			if(func&&func->get_aet_min_gain)
			aepa->min_gain = func->get_aet_min_gain();
			if(func&&func->get_aet_max_step)
			aepa->max_step = func->get_aet_max_step();

			//if(ae_debug)
			printk("aepa->sub_pixel_sum=%d\n",aepa->sub_pixel_sum);
			printk("aepa->max_gain=%d\n",aepa->max_gain);
			printk("aepa->min_gain=%d\n",aepa->min_gain);
			printk("aepa->max_step=%d\n",aepa->max_step);
			//printk("aepa->sub_pixel_sum=%d\n",aepa->cur_gain);
			//aepa->cur_step = func->get_aet_current_step;
			sm_state.isp_ae_parm.isp_ae_state = AE_SHUTTER_ADJUST;
			break;
		case AE_ORI_SET:
			aepa->cur_step = func->get_aet_current_step;
			newstep = aepa->cur_step;
			if(aep->ae_skip[1] == 0x1)
			{
				//if(func&&func->set_aet_new_step)
				//func->set_aet_new_step(newstep,true,true);
				sm_state.isp_ae_parm.isp_ae_state = AE_REST;							
			}
			else
			{
				//if(func&&func->set_aet_new_step)
				//func->set_aet_new_step(newstep,true,false);
				sm_state.isp_ae_parm.isp_ae_state = AE_GAIN_ADJUST;
			}
			break;
		case AE_LOW_GAIN:
			aepa->cur_gain = func->get_aet_current_gain;
			targrate = (aepa->cur_gain << 10)/aepa->tf_ratio;
			//newstep = find_step(func,0,aepa->max_step,targrate);
			if(aep->ae_skip[1] == 0x1)
			{
				//if(func&&func->set_aet_new_step)
				//func->set_aet_new_step(newstep,true,true);
				sm_state.isp_ae_parm.isp_ae_state = AE_REST;							
			}
			else
			{
				//if(func&&func->set_aet_new_step)
				//func->set_aet_new_step(newstep,true,false);
				sm_state.isp_ae_parm.isp_ae_state = AE_GAIN_ADJUST;
			}
			break;
		case AE_SHUTTER_ADJUST:
			if(func&&func->get_aet_current_gain)
			aepa->cur_gain = func->get_aet_current_gain();
			if(func&&func->get_aet_current_gain)
			aepa->cur_step = func->get_aet_current_step();
			if(aepa->cur_gain == 0)
				break;
			if(ae_debug1)
			printk("cur_gain = %d,cur_step = %d\n",aepa->cur_gain,aepa->cur_step);
			while(step != AE_SUCCESS)
			{
				switch(step){
					case AE_START:
						//printk("step2 =%d,aep->alert_mode=%x\n",step,aep->alert_mode);
						if(aep->alert_mode&0x1)
						{
						if(ae_debug4)
						{
							printk("[%d]bayer_over_info[0]=%d,%d,%d\n",aepa->change_step,ae->bayer_over_info[0],ae->bayer_over_info[1],ae->bayer_over_info[2]);	
						}
							if(aepa->change_step == 1)
								step = AE_EXPOSURE_MAX_CHECK2;
							else
								step = AE_EXPOSURE_MAX_CHECK;
						}
						else
						{
						step = AE_CALCULATE_LUMA_AVG;	
						}
						break;

					case AE_EXPOSURE_MAX_CHECK:
						if((ae->bayer_over_info[0] > aepa->alert_r)||
							(ae->bayer_over_info[1] > aepa->alert_g)||
							(ae->bayer_over_info[2] > aepa->alert_b)
							)
							step = AE_EXPOSURE_DECREASE;					
						else
						{
							isp_set_ae_thrlpf(aep->thr_r_low, aep->thr_g_low, aep->thr_b_low, aep->lpftype_mid);
							aepa->change_step = 1;
							step = AE_SUCCESS;//AE_CALCULATE_LUMA_AVG;	
						}
						break;
					case AE_EXPOSURE_MAX_CHECK2:
						if((ae->bayer_over_info[0] > aepa->alert_r)||
							(ae->bayer_over_info[1] > aepa->alert_g)||
							(ae->bayer_over_info[2] > aepa->alert_b)
							)
						{
							isp_set_ae_thrlpf(aep->thr_r_mid, aep->thr_g_mid, aep->thr_b_mid, aep->lpftype_mid);	
							aepa->change_step = 0;
							step = AE_SUCCESS;	
						}
						else
						{
							step = AE_EXPOSURE_INCREASE;
						}
						break;
					case AE_EXPOSURE_DECREASE:
						newstep = aepa->cur_step - aep->stepdnb;
						step = AE_SET_NEWSTEP;
						break;
					case AE_EXPOSURE_INCREASE:
						newstep = aepa->cur_step + aep->stepup;
						step = AE_SET_NEWSTEP;
						break;
					case AE_CALCULATE_LUMA_AVG:
						for(i=0;i<16;i++)
						{
							sub_avg[i] = ae->luma_win[i]/aepa->sub_pixel_sum;
						    avg_sum += sub_avg[i] * aep->coef_cur[i];
							avg_env_sum += sub_avg[i] * aep->coef_env[i];
							if((i == ae_step)&&ae_debug2)
							printk("sub_avg[%d]=%d,ae->luma_win=%d,aep->coef_cur=%d,aep->coef_env=%d,avg_env_sum=%d,avg_sum=%d \n",i,sub_avg[i],ae->luma_win[i],aep->coef_cur[i],aep->coef_env[i],avg_env_sum,avg_sum);
						}
						//if(ae_debug1)
						//printk("avg_env_sum=%d,avg_sum=%d \n",avg_env_sum,avg_sum);
						avg = avg_sum >> 10;
						if(devp->flag & ISP_FLAG_CAPTURE)
						{
							avgo = avg/aepa->cur_gain;
							if(avgo < aep->flash_thr)
								sm_state.flash = ISP_FLASH_STATUS_ON;
							else
								sm_state.flash = ISP_FLASH_STATUS_OFF;
						}
						avg_env = avg_env_sum >> 10;
						step = AE_CALCULATE_LUMA_TARG;
						break;
					case AE_CALCULATE_LUMA_TARG:
						avg_envo = (avg_env << 10)/aepa->cur_gain;
						if(ae_debug)
						printk("avg=%d,avg_envo=%d,aepa->cur_gain=%d,avg_env=%d,avg_env_sum=%d,avg_sum=%d,luma_win[9]=%d,sub[9]=%d\n",avg,avg_envo,aepa->cur_gain,avg_env,avg_env_sum,avg_sum,ae->luma_win[9],sub_avg[9]);
						if((avg_envo <= aep->env_low)||((avg_envo <= aep->env_low2mid)&&targ == (aep->targetlow)))
						{
							targ = aep->targetlow;
							radium_inner = aep->radium_inner_l;
							radium_outer = aep->radium_outer_l;
							sm_state.env = ENV_LOW;
							isp_set_ae_thrlpf(aep->thr_r_low, aep->thr_g_low, aep->thr_b_low, aep->lpftype_low);
						}
						else if((avg_envo >= aep->env_hign)||((avg_envo >= aep->env_hign2mid)&&targ == (aep->targethigh)))
						{
							targ = aep->targethigh;
							radium_inner = aep->radium_inner_h;
							radium_outer = aep->radium_outer_h;
							sm_state.env = ENV_HIGH;
							isp_set_ae_thrlpf(aep->thr_r_high, aep->thr_g_high, aep->thr_b_high, aep->lpftype_high);
						}
						else
						{
							targ = aep->targetmid;
							radium_inner = aep->radium_inner_m;
							radium_outer = aep->radium_outer_m;
							sm_state.env = ENV_MID;
							isp_set_ae_thrlpf(aep->thr_r_mid, aep->thr_g_mid, aep->thr_b_mid, aep->lpftype_mid);
						}
						if(ae_debug6)
							printk("targ=%d\n",targ);
						step = AE_LUMA_AVG_CHECK;
						break;
					case AE_LUMA_AVG_CHECK:
						if(ae_debug5)
						printk("avg=%d,targ=%d,radium_inner=%d,radium_outer=%d\n",avg,targ,radium_inner,radium_outer);
						if(((sm_state.status == ISP_AE_STATUS_UNSTABLE)&&(((avg - targ) > radium_inner)||((targ - avg) > radium_inner)))
							||(((avg - targ) > radium_outer)||((targ - avg) > radium_outer))
							)
						{
							step = AE_EXPOSURE_ADJUST;
						}
						else
						{
							sm_state.status = ISP_AE_STATUS_STABLE;
							step = AE_SUCCESS;
							//if(func->check_mains_freq)
								//sm_state.isp_ae_parm.isp_ae_state = AE_ORI_SET;
							//else
								sm_state.isp_ae_parm.isp_ae_state = AE_SHUTTER_ADJUST;
						}				
						break;
					case AE_EXPOSURE_ADJUST:
						sm_state.status = ISP_AE_STATUS_UNSTABLE;
						if(avg == 0)
						{
							step = AE_SUCCESS;
							break;
						}
						targrate = (targ * aepa->cur_gain)/avg;	
						if(ae_debug1)
							printk("targrate = %d\n",targrate);
						if(targrate > aepa->max_gain)
							targrate = aepa->max_gain;
						if(targrate < aepa->min_gain)
							targrate = aepa->min_gain;
						targstep = find_step(func,0,aepa->max_step,targrate);
						if(targstep > aepa->max_step)
							targstep = aepa->max_step;
						if(ae_debug1)
							printk("targstep = %d,%d\n",targstep,targrate);
						lpfcoef = (devp->flag & ISP_FLAG_CAPTURE)?aep->fast_lpfcoef:aep->slow_lpfcoef;
						newstep = (aepa->cur_step*lpfcoef + targstep*(256-lpfcoef))>>8;
						if(ae_debug1)
						printk("newstep =%d,lpf =%d,%d\n",newstep,lpfcoef,aepa->cur_step);
						if((newstep >= aepa->max_step - 1)||(newstep == aepa->cur_step)||(newstep == 0))
						{
							sm_state.status = ISP_AE_STATUS_STABLE;
							step = AE_SUCCESS;
						}
						else if((newstep > aepa->cur_step)&&ae_debug3)
							step = AE_MAX_CHECK_STOP;
						else
							step = AE_SET_NEWSTEP;
						break;
					case AE_SET_NEWSTEP:
						ae_sens.send = 1;
						ae_sens.new_step = newstep;
						ae_sens.shutter = 1;
						ae_sens.gain = 1;
						//printk("ae_sens.send \n");
						sm_state.ae_down = false;
						sm_state.isp_ae_parm.isp_ae_state = AE_REST;
						step = AE_SUCCESS;
						break;
					case AE_MAX_CHECK_STOP:
						if(ae_debug4)
							printk("ae->bayer_over_info[0]=%d,%d,%d\n",ae->bayer_over_info[0],ae->bayer_over_info[1],ae->bayer_over_info[2]);
						if((ae->bayer_over_info[0] > aepa->alert_r)||
							(ae->bayer_over_info[1] > aepa->alert_g)||
							(ae->bayer_over_info[2] > aepa->alert_b)
							)
						{
							sm_state.status = ISP_AE_STATUS_STABLE;
							step = AE_SUCCESS;
						}
						else
							step = AE_SET_NEWSTEP;
						break;
					case AE_SUCCESS:
					default:
						step = 0;
						break;
				};
				//break;
			}
			break;
		case AE_GAIN_ADJUST:
			//if(func&&func->set_aet_new_step)
			//func->set_aet_new_step(newstep,false,true);
			sm_state.isp_ae_parm.isp_ae_state = AE_REST;
			break;
		case AE_REST:
			sm_state.ae_down = true;
			if(ae_sens.send == 0)
				k++;
			//printk("func->check_mains_freq =%x\n",func->check_mains_freq);
			//if(func->check_mains_freq)
			//	sm_state.isp_ae_parm.isp_ae_state = AE_ORI_SET;
			//else
			if(k > ae_step)
{
				k = 0;
				sm_state.isp_ae_parm.isp_ae_state = AE_SHUTTER_ADJUST;
			}
			break;
    }
}

// VDIN_MATRIX_YUV601_RGB
//	-16 	1.164  0	  1.596 	 0
// -128 	1.164 -0.391 -0.813 	 0
// -128 	1.164  2.018  0 		 0
static inline int matrix_yuv601_rgb_r(unsigned int y, unsigned int u, unsigned int v)
{
	return (((y-16)*1192+(u-128)*0+(v-128)*1634+0) >> 10);
}

static inline int matrix_yuv601_rgb_g(unsigned int y, unsigned int u, unsigned int v)
{
	return (((y-16)*1192+(u-128)*(-400)+(v-128)*(-833)+0) >> 10);
}

static inline int matrix_yuv601_rgb_b(unsigned int y, unsigned int u, unsigned int v)
{
	return (((y-16)*1192+(u-128)*2066+(v-128)*0+0) >> 10);
}

// VDIN_MATRIX_YUV709_RGB
//	-16 	1.164  0	  1.793 	 0
// -128 	1.164 -0.213 -0.534 	 0
// -128 	1.164  2.115  0 		 0
static inline int matrix_yuv709_rgb_r(unsigned int y, unsigned int u, unsigned int v)
{
	return (((y-16)*1192+(u-128)*0+(v-128)*1836+0) >> 10);
}

static inline int matrix_yuv709_rgb_g(unsigned int y, unsigned int u, unsigned int v)
{
	return (((y-16)*1192+(u-128)*(-218)+(v-128)*(-547)+0) >> 10);
}

static inline int matrix_yuv709_rgb_b(unsigned int y, unsigned int u, unsigned int v)
{
	return (((y-16)*1192+(u-128)*2166+(v-128)*0+0) >> 10);
}

//static unsigned int r_val = 256;
//static unsigned int g_val = 390;//256;
//static unsigned int b_val = 256;

static struct isp_awb_gain_s awb_gain;

void isp_awb_base_sm(isp_dev_t *devp)
{
	struct isp_awb_stat_s *awb = &devp->isp_awb;
	struct xml_algorithm_awb_s *awbp = devp->isp_awb_parm;
	struct isp_info_s *parm = &devp->info;
	struct isp_awb_sm_s *awba = &sm_state.isp_awb_parm;
	int step = 0;

	u16 r[5] = {0};      //0,rgb;1,ym;2,yh;3,yl;4,final.
	u16 g[5] = {0};
	u16 b[5] = {0};
	u32 count[3] = {0};   //0,mid;1,hign;2,low.
	int u[3] = {0};
	int v[3] = {0};
	u16 rg = 0;
	u16 bg = 0;
	u16 target_r;
	u16 target_b;
	//printk("sm_state.isp_awb_parm.isp_awb_state=%d\n",sm_state.isp_awb_parm.isp_awb_state);
	
	switch(sm_state.isp_awb_parm.isp_awb_state){
		case AWB_IDLE:
			break;
		case AWB_INIT:
			awba->win_l = (parm->h_active * awbp->ratio_winl) >> 10;
			awba->win_r = ((parm->h_active * awbp->ratio_winr) >> 10) - 1;
			awba->win_t = (parm->v_active * awbp->ratio_wint) >> 10;
			awba->win_b = ((parm->v_active * awbp->ratio_winb) >> 10) - 1;	
			printk("awb,win_l=%d,win_r=%d,win_t=%d,win_b=%d\n",awba->win_l,awba->win_r,awba->win_t,awba->win_b);
			isp_set_awb_win(awba->win_l, awba->win_r, awba->win_t, awba->win_b);
			awba->pixel_sum = parm->h_active * parm->v_active;
			awba->countlimitrgb = ((awba->pixel_sum >> 2) * awbp->ratio_rgb) >> 6;
			awba->countlimityh	= ((awba->pixel_sum >> 2) * awbp->ratio_yh) >> 6;
			awba->countlimitym	= ((awba->pixel_sum >> 2) * awbp->ratio_ym) >> 6;
			awba->countlimityl	= ((awba->pixel_sum >> 2) * awbp->ratio_yl) >> 6;
			awba->status = ISP_AWB_STATUS_STABLE;
			sm_state.isp_awb_parm.isp_awb_state = AWB_CHECK;
			break;
		case AWB_CHECK:
			while(step != AWB_SUCCESS)
			{
				//printk("step = %d\n",step);
				switch(step){
					case AWB_START:
						step = AWB_RGB_COUNT_CHECK;
						//printk("step1 = %d\n",step);
						break;
					case AWB_RGB_COUNT_CHECK:	
						if(awb_debug4)
							printk("awb->rgb.rgb_count=%d\n",awb->rgb.rgb_count);
						if(awb->rgb.rgb_count >= awba->countlimitrgb)
							step = AWB_CALCULATE_RGB;
						else
						{
							r[0] = 0;
							g[0] = 0;
							b[0] = 0;
							step = AWB_YUVM_COUNT_CHECK;
						}
						//printk("step55 = %d\n",step);
					    break;
					case AWB_CALCULATE_RGB:
						if(awb->rgb.rgb_count <= 0)
						{
							step = AWB_SUCCESS;
							break;
						}
						r[0] = awb->rgb.rgb_sum[0]/awb->rgb.rgb_count;
						g[0] = awb->rgb.rgb_sum[1]/awb->rgb.rgb_count;
						b[0] = awb->rgb.rgb_sum[2]/awb->rgb.rgb_count;
						step = AWB_YUVM_COUNT_CHECK;
						break;
					case AWB_YUVM_COUNT_CHECK:
						count[0] = awb->yuv_mid[0].count + awb->yuv_mid[1].count;
						if(count[0] >= awba->countlimitym)
							step = AWB_CALCULATE_YUVM;
						else
						{
							r[1] = 0;
							g[1] = 0;
							b[1] = 0;
							step = AWB_YUVH_COUNT_CHECK;
						}
						break;
					case AWB_CALCULATE_YUVM:
						if(count[0] <= 0)
						{
							step = AWB_SUCCESS;
							break;
						}
						u[0] = (awb->yuv_mid[1].sum - awb->yuv_mid[0].sum)/count[0];
						v[0] = (awb->yuv_mid[3].sum - awb->yuv_mid[2].sum)/count[0];
						
						if(parm->v_active >= 720)
						{
							r[1] = matrix_yuv709_rgb_r(awbp->yym,u[0]+128,v[0]+128);
							g[1] = matrix_yuv709_rgb_g(awbp->yym,u[0]+128,v[0]+128);
							b[1] = matrix_yuv709_rgb_b(awbp->yym,u[0]+128,v[0]+128);
						}
						else
						{
							r[1] = matrix_yuv601_rgb_r(awbp->yym,u[0]+128,v[0]+128);
							g[1] = matrix_yuv601_rgb_g(awbp->yym,u[0]+128,v[0]+128);
							b[1] = matrix_yuv601_rgb_b(awbp->yym,u[0]+128,v[0]+128);
						}
						step = AWB_YUVH_COUNT_CHECK;
						break;
					case AWB_YUVH_COUNT_CHECK:
						count[1] = awb->yuv_high[0].count + awb->yuv_high[1].count;
						if(count[1] >= awba->countlimityh)
							step = AWB_CALCULATE_YUVH;
						else
						{
							r[2] = 0;
							g[2] = 0;
							b[2] = 0;
							step = AWB_YUVL_COUNT_CHECK;
						}
						break;		
					case AWB_CALCULATE_YUVH:
						if(count[1] <= 0)
						{
							step = AWB_SUCCESS;
							break;
						}
						u[1] = (awb->yuv_high[1].sum - awb->yuv_high[0].sum)/count[1];
						v[1] = (awb->yuv_high[3].sum - awb->yuv_high[2].sum)/count[1];
						
						if(parm->v_active >= 720)
						{
							r[2] = matrix_yuv709_rgb_r(awbp->yyh,u[1]+128,v[1]+128);
							g[2] = matrix_yuv709_rgb_g(awbp->yyh,u[1]+128,v[1]+128);
							b[2] = matrix_yuv709_rgb_b(awbp->yyh,u[1]+128,v[1]+128);
						}
						else
						{
							r[2] = matrix_yuv601_rgb_r(awbp->yyh,u[1]+128,v[1]+128);
							g[2] = matrix_yuv601_rgb_g(awbp->yyh,u[1]+128,v[1]+128);
							b[2] = matrix_yuv601_rgb_b(awbp->yyh,u[1]+128,v[1]+128);
						}
						step = AWB_YUVL_COUNT_CHECK;
						break;	
					case AWB_YUVL_COUNT_CHECK:
						count[2] = awb->yuv_low[0].count + awb->yuv_low[1].count;
						if(count[2] >= awba->countlimityl)
							step = AWB_CALCULATE_YUVL;
						else
						{
							r[3] = 0;
							g[3] = 0;
							b[3] = 0;
							step = AWB_RGB_BLEND;
						}
						break;		
					case AWB_CALCULATE_YUVL:
						if(count[2] <= 0)
						{
							step = AWB_SUCCESS;
							break;
						}
						u[2] = (awb->yuv_low[1].sum - awb->yuv_low[0].sum)/count[2];
						v[2] = (awb->yuv_low[3].sum - awb->yuv_low[2].sum)/count[2];
						
						if(parm->v_active >= 720)
						{
							r[3] = matrix_yuv709_rgb_r(awbp->yyl,u[2]+128,v[2]+128);
							g[3] = matrix_yuv709_rgb_g(awbp->yyl,u[2]+128,v[2]+128);
							b[3] = matrix_yuv709_rgb_b(awbp->yyl,u[2]+128,v[2]+128);
						}
						else
						{
							r[3] = matrix_yuv601_rgb_r(awbp->yyl,u[2]+128,v[2]+128);
							g[3] = matrix_yuv601_rgb_g(awbp->yyl,u[2]+128,v[2]+128);
							b[3] = matrix_yuv601_rgb_b(awbp->yyl,u[2]+128,v[2]+128);
						}
						step = AWB_RGB_BLEND;
						break;
			        case AWB_RGB_BLEND:
						r[4] = (r[0]*awbp->coef_r[0]+r[1]*awbp->coef_r[1]+r[2]*awbp->coef_r[2]+r[3]*awbp->coef_r[3])>>8;
						g[4] = (g[0]*awbp->coef_g[0]+g[1]*awbp->coef_g[1]+g[2]*awbp->coef_g[2]+g[3]*awbp->coef_g[3])>>8;
						b[4] = (b[0]*awbp->coef_b[0]+b[1]*awbp->coef_b[1]+b[2]*awbp->coef_b[2]+b[3]*awbp->coef_b[3])>>8;
						if(awb_debug)
							printk("r=%d,%d,%d,%d,%d,g=%d,%d,%d,%d,%d,b=%d,%d,%d,%d,%d\n",r[0],r[1],r[2],r[3],r[4],g[0],g[1],g[2],g[3],g[4],b[0],b[1],b[2],b[3],b[4]);
						if(sm_state.env == ENV_HIGH)
						{
							isp_set_awb_yuv_thr(awbp->thr_yh_h, awbp->thr_yl_h, awbp->thr_du_h, awbp->thr_dv_h);
							isp_set_awb_rgb_thr(awbp->thr_gb_h, awbp->thr_gr_h, awbp->thr_br_h);
						}
						else if(sm_state.env == ENV_MID)
						{
							isp_set_awb_yuv_thr(awbp->thr_yh_m, awbp->thr_yl_m, awbp->thr_du_m, awbp->thr_dv_m);
							isp_set_awb_rgb_thr(awbp->thr_gb_m, awbp->thr_gr_m, awbp->thr_br_m);
						}
						else if(sm_state.env == ENV_LOW)
						{
							isp_set_awb_yuv_thr(awbp->thr_yh_l, awbp->thr_yl_l, awbp->thr_du_l, awbp->thr_dv_l);
							isp_set_awb_rgb_thr(awbp->thr_gb_l, awbp->thr_gr_l, awbp->thr_br_l);
						}
						step = AWB_TEMP_CHECK;
						break;
					case AWB_TEMP_CHECK:
						if((g[4] == 0)||(r[4] == 0)||(b[4] == 0))
						{
							step = AWB_SUCCESS;
							break;
						}
						rg = (r[4] << 10)/g[4];
						bg = (b[4] << 10)/g[4];
						if(awb_debug1)
							printk("rg=%d,bg=%d\n",rg,bg);
						if(((awba->status == ISP_AWB_STATUS_UNSTABLE) && ((rg > 1024 + awbp->inner_rg)||(rg < 1024 - awbp->inner_rg)))
							||((rg > 1024 + awbp->outer_rg)||(rg < 1024 - awbp->outer_rg))
							||((awba->status == ISP_AWB_STATUS_UNSTABLE) && ((bg > 1024 + awbp->inner_bg)||(bg < 1024 - awbp->inner_bg)))
							||((bg > 1024 + awbp->outer_bg)||(bg < 1024 - awbp->outer_bg))
							)
							step = AWB_TEMP_ADJUST;
						else
						{
							awba->status = ISP_AWB_STATUS_STABLE;
							step = AWB_SUCCESS;
						}
						break;
					case AWB_TEMP_ADJUST:
						awba->status = ISP_AWB_STATUS_UNSTABLE;
						isp_awb_get_gain(&awb_gain);
						if(awb_debug2)
							printk("r_val=%d,b_val=%d\n",awb_gain.r_val,awb_gain.b_val);
						target_r = (awb_gain.r_val<<10)/rg;
						target_b = (awb_gain.b_val<<10)/bg;
						//awbp->r_max = 282;
						//awbp->g_max = 512;
						//awbp->b_max = 282;
						//awbp->r_min = 200;
						//awbp->g_min = 128;
						//awbp->b_min = 200;
							
						if(awb_debug2)
							printk("target_r=%d,target_b=%d\n",target_r,target_b);
						if(target_r > awbp->r_max)
							target_r = awbp->r_max;
						if(target_r < awbp->r_min)
							target_r = awbp->r_min;
						if(target_b > awbp->b_max)
							target_b = awbp->b_max;
						if(target_b < awbp->b_min)
							target_b = awbp->b_min;
						if(awb_debug3)
							isp_awb_set_gain(target_r,awb_gain.g_val,target_b);
						//r_val = target_r;
						//b_val = target_b;
						step = AWB_SUCCESS;
						break;
					case AWB_SUCCESS:
					default:
						step = 0;
						break;
				}
			}
			break;
	}
}
unsigned long long div64(unsigned long long n, unsigned long long d) // n for numerator, d for denominator
{
    unsigned int n_bits = 0, d_bits = 0, i = 0;
    unsigned long long q = 0, t = 0; // q for quotient, t for temporary
    // invalid
    if (!d) {
        q = 0xffffffffffffffff;
    }
    // (0.5, 0]
    else if (n + n < d) {
        q = 0;
    }
    // [1.0, 0.5]
    else if (n <= d) {
        q = 1;
    }
    // [max, 1.0)
    else
    {
        // get n_bits
        for (n_bits = 1; n_bits <= 64; n_bits++)
            if (!(n >> n_bits))
                break;
        if (n_bits > 64)
		n_bits = 64;
		// get d_bits
        for (d_bits = 1; d_bits <= 64; d_bits++)
            if (!(d >> d_bits))
                break;
        if (d_bits > 64)
            d_bits = 64;
        // check integer part
        for (i = n_bits; i >= d_bits; i--) {
            q <<= 1;
            t = d << (i - d_bits);
            if (n >= t)
            {
                n -= t;
                q += 1;
            }
        }
        // check fraction part
        if (n + n >= d)
            q += 1;
    }
    return q;
}
static unsigned long long get_fv_base_blnr(isp_blnr_stat_t *blnr)
{
	unsigned long long sum_ac = 0, sum_dc = 0, mul_ac = 0;
	sum_ac  = (unsigned long long)blnr->ac[0];
	sum_ac += (unsigned long long)blnr->ac[1];
	sum_ac += (unsigned long long)blnr->ac[2];
	sum_ac += (unsigned long long)blnr->ac[3];
	
	sum_dc  = (unsigned long long)blnr->dc[0];
	sum_dc += (unsigned long long)blnr->dc[1];
	sum_dc += (unsigned long long)blnr->dc[2];
	sum_dc += (unsigned long long)blnr->dc[3];
	
	mul_ac = (sum_ac > 0x00000000ffffffff) ? 0xffffffffffffffff : sum_ac*sum_ac;

	return div64(mul_ac,sum_dc);
	
}
unsigned int get_best_step(isp_blnr_stat_t *blnr,unsigned int *step)
{
        unsigned int i = 0, cur_grid = 0, max_grid = 0, best_step = 0;
        unsigned long long sum_ac = 0, sum_dc = 0, mul_ac = 0, fv[FOCUS_GRIDS], max_fv = 0, moment = 0, sum_fv = 0;

		if(best_step_debug)				
			pr_info("%s ac[0] ac[1] ac[2] ac[3] dc[0] dc[1] dc[2] dc[3]\n", __func__);
        for (i = 0; i < FOCUS_GRIDS; i++){
                if (i && (step[i]==0)){
                        break;
                }
                max_grid = i;
                fv[i] = get_fv_base_blnr(&blnr[i]);
	        if(best_step_debug)
                        pr_info("%s %u %u %u %u %u %u %u %u\n", __func__, blnr[i].ac[0], blnr[i].ac[1], blnr[i].ac[2], blnr[i].ac[3], blnr[i].dc[0], blnr[i].dc[1], blnr[i].dc[2], blnr[i].dc[3]);
                if (max_fv < fv[i]){
		        max_fv = fv[i];
		        cur_grid = i;
	        }
        }
	// too less stroke, for power saving
        if (!cur_grid) {
	        best_step = 0;
        }
        // too much stroke
        else if (cur_grid == max_grid){
	        best_step = step[max_grid];
	}
	// work out best step with 3 grids
	else if ((cur_grid == 1) || (cur_grid == max_grid - 1)){
                moment += fv[cur_grid - 1]*(unsigned long long)step[cur_grid - 1];
                moment += fv[cur_grid    ]*(unsigned long long)step[cur_grid    ];
                moment += fv[cur_grid + 1]*(unsigned long long)step[cur_grid + 1];
                sum_fv += fv[cur_grid - 1];
                sum_fv += fv[cur_grid    ];
                sum_fv += fv[cur_grid + 1];
                best_step = (unsigned int)div64(moment,sum_fv);
	}
	// work out best step with 5 grids
        else {
                moment += (unsigned long long)fv[cur_grid - 2]*(unsigned long long)step[cur_grid - 2];
                moment += (unsigned long long)fv[cur_grid - 1]*(unsigned long long)step[cur_grid - 1];
                moment += (unsigned long long)fv[cur_grid    ]*(unsigned long long)step[cur_grid    ];
                moment += (unsigned long long)fv[cur_grid + 1]*(unsigned long long)step[cur_grid + 1];
                moment += (unsigned long long)fv[cur_grid + 2]*(unsigned long long)step[cur_grid + 2];
                sum_fv += fv[cur_grid - 2];
                sum_fv += fv[cur_grid - 1];
                sum_fv += fv[cur_grid    ];
                sum_fv += fv[cur_grid + 1];
                sum_fv += fv[cur_grid + 2];
                best_step = (unsigned int)div64(moment,sum_fv);
	}
	if(best_step_debug)
		pr_info("%s:get best step %u.\n",__func__,best_step);
	return best_step;
}
static unsigned int jitter = 5;
module_param(jitter,uint,0664);
MODULE_PARM_DESC(jitter,"\n debug flag for ae.\n");

static unsigned int delta = 9;
module_param(delta,uint,0664);
MODULE_PARM_DESC(delta,"\n debug flag for ae.\n");

static bool is_lost_focus(isp_af_info_t *af_info,xml_algorithm_af_t *af_alg)
{
	unsigned long long *fv,sum_fv=0,ave_fv=0,delta_fv=0;
	unsigned int i=0,step_cnt=0;
	bool ret = false;
	fv = kmalloc(sizeof(unsigned long long)*(af_alg->detect_step),GFP_KERNEL);

	memset(fv,0,sizeof(unsigned long long)*(af_alg->detect_step));
	
	for(i=0;i<af_alg->detect_step;i++){
		fv[i] = get_fv_base_blnr(&af_info->af_detect[i]);
		sum_fv += fv[i];
		if(af_sm_dg&0x2){
			pr_info("step[%u]:ac0=%u ac1=%u ac2=%u ac3=%u ",i,
					af_info->af_detect[i].ac[0],af_info->af_detect[i].ac[1],af_info->af_detect[i].ac[2],
					af_info->af_detect[i].ac[3]);
			pr_info("dc0=%u dc1=%u dc2=%u dc3=%u  ",af_info->af_detect[i].dc[0],af_info->af_detect[i].dc[1],
					af_info->af_detect[i].dc[2],af_info->af_detect[i].dc[3]);
			pr_info("fv=%llu.\n",fv[i]);
		}
	}
	step_cnt = af_alg->detect_step;
	ave_fv = div64(sum_fv,step_cnt);
	if(af_sm_dg&0x1)
		pr_info("ave_fv %llu.\n",ave_fv);
	delta_fv = ave_fv>af_info->fv_aft_af?(ave_fv-af_info->fv_aft_af):(af_info->fv_aft_af-ave_fv);
	delta_fv = delta_fv*(unsigned long long)1024;
	delta_fv = div64(delta_fv,af_alg->deta_ave_ratio);
	if(delta_fv > af_info->fv_aft_af){
		pr_info("1 delta_fv*1024/ave_ratio=%llu,last_ave_fv=%llu.\n",delta_fv,af_info->fv_aft_af);
		ret = true;
	}else{
		ret = false;
		if(af_sm_dg&0x1)
			pr_info("2 delta_fv*1024/ave_ratio=%llu,last_ave_fv=%llu.\n",delta_fv,af_info->fv_aft_af);
	}

	kfree(fv);

	return ret;
	
}
void isp_af_detect(isp_dev_t *devp)
{
	static unsigned int start_jf,af_delay=0;
	struct xml_algorithm_af_s *af_alg = devp->isp_af_parm;
	struct isp_af_info_s *af_info = &devp->af_info;
	
	switch(sm_state.af_state){
		case AF_DETECT_INIT:
			af_info->f = af_info->af_detect;
			af_info->cur_index = 0;
			sm_state.af_state = AF_GET_STEPS_INFO;
			break;
		case AF_GET_STEPS_INFO:	
			if(sm_state.status == ISP_AE_STATUS_STABLE){
				if(af_info->cur_index++ >= af_alg->detect_step){
					af_info->cur_index = 0;
					sm_state.af_state = AF_GET_STATUS;
					pr_info("%s state get_status.\n",__func__);
				}
			}else{
				sm_state.af_state = AF_DETECT_INIT;
				if(af_sm_dg&0x1)
					pr_info("%s ae unstable return to af init.\n",__func__);
			}
			break;
		case AF_GET_STATUS:
			if(is_lost_focus(af_info,af_alg)){
				sm_state.af_state = AF_INIT;
				if(af_sm_dg)
					pr_info("[af_sm]:lost focus.\n");
			}
			af_info->cur_index++;
			if(af_info->cur_index >= af_alg->detect_step){
				af_info->cur_index = 0;
			}
			break;
		default:
			isp_af_sm(devp);
			break;
	}
}
void isp_af_sm(isp_dev_t *devp)
{
	static unsigned int start_jf,af_delay=0;
	struct xml_algorithm_af_s *af_alg = devp->isp_af_parm;
	struct isp_af_info_s *af_info = &devp->af_info;
	struct isp_af_sm_s *sm = &sm_state.af_sm;
	unsigned long long fv_delta;
	af_delay++;
	
	switch(sm_state.af_state){
		case AF_INIT:
			if((devp->flag&ISP_FLAG_AE)&&(sm_state.ae_down)){
			/*awb brake,ae brake*/
			af_info->flag_bk = (devp->flag&ISP_FLAG_AWB)+(devp->flag&ISP_FLAG_AE);
			if(af_sm_dg&0x1)
				pr_info("%s:ae,awb flag status 0x%x.\n",__func__,af_info->flag_bk);
			devp->flag &=(~ISP_FLAG_AWB);
			devp->flag &=(~ISP_FLAG_AE);
			af_info->f = af_info->af_data;
			af_info->cur_index = 0;
			start_jf = jiffies;
			sm_state.af_state = AF_GET_OLD_FV;
		}else{
			if(af_sm_dg&0x1)
				pr_info("%s:ae isn't down.\n",__func__);
		}
			break;
		case AF_GET_OLD_FV:
			af_info->fv_bf_af = get_fv_base_blnr(&af_info->af_data[af_info->cur_index]);
			af_info->cur_index = 0;
			af_info->cur_step = af_alg->step[af_info->cur_index];
			atomic_set(&af_info->writeable,1);
			af_delay = 0;
			sm_state.af_state = AF_GET_COARSE_INFO;
			break;
		case AF_GET_COARSE_INFO:
			if((af_info->cur_index >= FOCUS_GRIDS)||(af_alg->step[af_info->cur_index]==0)){
				sm_state.af_state = AF_CALC_GREAT;
			} else if((atomic_read(&af_info->writeable) <= 0)&&(af_delay >= af_alg->field_delay)){
				af_info->cur_step = af_alg->step[af_info->cur_index];
				af_info->cur_index++;
				atomic_set(&af_info->writeable,1);
				af_delay = 0;
			} 
			break;
		case AF_CALC_GREAT:
			af_info->great_step = get_best_step(af_info->af_data,af_alg->step);
			af_info->cur_step = af_info->great_step - af_alg->jump_offset;
			atomic_set(&af_info->writeable,1);
			af_delay = 0;
			sm_state.af_state = AF_GET_FINE_INFO;
			break;
		case AF_GET_FINE_INFO:
			if((atomic_read(&af_info->writeable) <= 0)&&(af_delay >= af_alg->field_delay)){
				af_info->cur_step = af_info->great_step;
				atomic_set(&af_info->writeable,1);
				af_delay = 0;
				sm_state.af_state = AF_SUCCESS;
			}
			break;
		case AF_SUCCESS:
			if(af_delay >= 2){
		                /* get last fv */
		                af_info->fv_aft_af = get_fv_base_blnr(&af_info->af_data[af_info->cur_index]);
			        if(af_sm_dg&0x2){
				        pr_info("%s:ac0=%u ac1=%u ac2=%u ac3=%u dc0=%u dc1=%u dc2=%u dc3=%u fv=%llu.\n",__func__,
				                af_info->af_data[af_info->cur_index].ac[0],af_info->af_data[af_info->cur_index].ac[1],
				                af_info->af_data[af_info->cur_index].ac[2],af_info->af_data[af_info->cur_index].ac[3],
					        af_info->af_data[af_info->cur_index].dc[0],af_info->af_data[af_info->cur_index].dc[1],
					        af_info->af_data[af_info->cur_index].dc[2],af_info->af_data[af_info->cur_index].dc[3],af_info->fv_aft_af);
				}
				fv_delta = af_info->fv_aft_af>af_info->fv_bf_af?(af_info->fv_aft_af-af_info->fv_bf_af):(af_info->fv_bf_af-af_info->fv_aft_af);
				fv_delta = fv_delta*100;
				fv_delta = div64(fv_delta,af_alg->af_fail_ratio);
				/*af failed return to af init,retry*/
				if(af_sm_dg&0x4){
					pr_info("[af_sm..]:fv_delta %llu,fv_bf_af %llu.\n",fv_delta,af_info->fv_bf_af);
				}
				if((fv_delta > af_info->fv_bf_af)&&(af_alg->af_retry_cnt++ < af_alg->af_retry_max)){
					sm_state.af_state = AF_GET_OLD_FV;
					if(af_sm_dg&0x4)
						pr_info("[af_sm..]:fail ratio %u,%u times,return to af init retry.\n",af_alg->af_fail_ratio,af_alg->af_retry_cnt);
				} else if((fv_delta > af_info->fv_bf_af)&&(af_alg->af_retry_cnt > af_alg->af_retry_max)){
		        	/*af failed over max times,force to step 0*/
		                        /*enable awb,enable af*/
				        devp->flag |= af_info->flag_bk;
					af_info->cur_step = 0;
					atomic_set(&af_info->writeable,1);
					if(af_sm_dg&0x4)
						pr_info("[af_sm..]:fail ratio %u over,force to step 0.\n",af_alg->af_fail_ratio);
					af_alg->af_retry_cnt = 0;
					sm_state.af_state = AF_NULL;
				} else {/*af success*/
					/*enable awb,enable af*/
				        devp->flag |= af_info->flag_bk;
					af_alg->af_retry_cnt = 0;
					sm_state.af_state = AF_NULL;
				}
			}
			break;
		default:
			break;
	}
}
#define FLASH_OFF         0
#define FLASH_ON	  1
#define FLASH_TORCH       2
static void isp_set_flash(isp_dev_t *devp,unsigned flash_mode,unsigned level)
{
	if(!flash_mode)		
	torch_level(devp->flash.mode_pol_inv,devp->flash.led1_pol_inv,devp->flash.pin_mux_inv,devp->flash.torch_pol_inv,devp->wave,0);
	else if(flash_mode == FLASH_ON)
		flash_on(devp->flash.mode_pol_inv,devp->flash.led1_pol_inv,devp->flash.pin_mux_inv,devp->wave);
	else if(flash_mode == FLASH_TORCH)
		torch_level(devp->flash.mode_pol_inv,devp->flash.led1_pol_inv,devp->flash.pin_mux_inv,devp->flash.torch_pol_inv,devp->wave,level);
}

void isp_set_flash_mode(isp_dev_t *devp)
{
	if(!devp->flash.valid)
		/*no flash*/
		sm_state.cap_sm.flash_mode = FLASH_MODE_NULL;
	else
		sm_state.cap_sm.flash_mode = devp->cam_param->flash_mode;
}
int isp_capture_sm(isp_dev_t *devp)
{
	static unsigned int start_jf,multi_count=0;
	unsigned int cur_ac=0,j=0;
	xml_capture_t *parm = devp->capture_parm;
	enum tvin_buffer_ctl_e ret = TVIN_BUF_SKIP;
	struct isp_capture_sm_s *cap_sm = &sm_state.cap_sm;

	cap_sm->adj_cnt++;
	
	switch(cap_sm->capture_state){
		case CAPTURE_INIT:
			isp_set_blenr_stat(0,0,devp->info.h_active,devp->info.v_active);
			if(parm->pretime){
				cap_sm->capture_state = CAPTURE_PRE_WAIT;
				start_jf = jiffies;
				if(sm_debug)
					pr_info("%s:init->pre_wait.\n",__func__);
			} else {
			        cap_sm->capture_state = CAPTURE_FLASH_ON;
				if(sm_debug)
					pr_info("%s:init->flash_on.\n",__func__);
			}
			break;
		case CAPTURE_PRE_WAIT:
			if(time_after(jiffies,start_jf+parm->pretime)){
				cap_sm->capture_state = CAPTURE_FLASH_ON;
				start_jf = 0;
				if(sm_debug)
					pr_info("%s:pre_wait->flash_on.\n",__func__);
			}	
			break;
		case CAPTURE_FLASH_ON:
			if((sm_state.flash==ISP_FLASH_STATUS_NULL)&&(cap_sm->flash_mode==FLASH_MODE_AUTO)){
				return ret;
			} 
			if(((cap_sm->flash_mode == FLASH_MODE_AUTO)&&(sm_state.flash == ISP_FLASH_STATUS_ON))||
				(cap_sm->flash_mode == FLASH_MODE_ON))
			{
				start_jf = jiffies;
				isp_set_flash(devp,FLASH_TORCH,100);
				cap_sm->adj_cnt = 0;
				cap_sm->flash_on = 1;
				cap_sm->capture_state = CAPTURE_TR_WAIT;
				if(sm_debug)
					pr_info("[isp..]%s flash on->torch rising wait.\n",__func__);
			} else {
				/*without flash*/
				cap_sm->flash_on = 0;
				cap_sm->capture_state = CAPTURE_TUNE_AEW;
			if(sm_debug)
				pr_info("[isp..]%s flash on->tune ae wait.\n",__func__);
			}
			break;
		case CAPTURE_TR_WAIT:
			if(time_after(jiffies,cap_sm->tr_time)){
				cap_sm->capture_state = CAPTURE_TUNE_AEW;
				if(sm_debug)
				pr_info("[isp..]%s torch rising wait->tune ae wait.\n",__func__);
			}
			break;
		case CAPTURE_TUNE_AEW:
			if(sm_state.ae_down==true){
				start_jf = 0;
				if(parm->ae_en){
					cap_sm->capture_state = CAPTURE_TUNE_AE;
					/*awb brake,af brake*/
					devp->flag &=(~ISP_FLAG_AWB);
					devp->flag &=(~ISP_FLAG_AF);
					if(sm_debug)
						pr_info("[%s..]%s changed aew->ae.\n",DEVICE_NAME,__func__);
				}
				else if(parm->awb_en){
					cap_sm->capture_state = CAPTURE_TUNE_AWB;
					/*ae brake,af brake*/
					devp->flag &=(~ISP_FLAG_AE);
					devp->flag &=(~ISP_FLAG_AF);
					cap_sm->adj_cnt = 0;
					if(sm_debug)
						pr_info("[%s..]%s changed aew->awb.\n",DEVICE_NAME,__func__);
				}
				else if(parm->af_mode){
					cap_sm->capture_state = CAPTURE_TUNE_AF;
					/*ae brake,awb brake*/
					devp->flag &=(~ISP_FLAG_AE);
					devp->flag &=(~ISP_FLAG_AWB);
					if(sm_debug)
						pr_info("[%s..]%s changed aew->af.\n",DEVICE_NAME,__func__);
				}else if(cap_sm->flash_on){
					devp->flag |= ISP_FLAG_AE;
					cap_sm->capture_state = CAPTURE_LOW_GAIN;
					if(sm_debug)
						pr_info("[%s..]%s changed aew->ae.\n",DEVICE_NAME,__func__);
				}
				else{
					cap_sm->capture_state = CAPTURE_SINGLE;
					cap_sm->adj_cnt = 0;
					if(sm_debug)
						pr_info("[%s..]%s changed aew->single.\n",DEVICE_NAME,__func__);
				}
			}
			break;
		case CAPTURE_TUNE_AE:
			if(((sm_state.status==ISP_AE_STATUS_STABLE)||(sm_state.status==ISP_AE_STATUS_UNTUNEABLE))&&
				(sm_state.ae_down==true))
			{
				devp->flag &=(~ISP_FLAG_AE);
				if(parm->awb_en){
					cap_sm->capture_state = CAPTURE_TUNE_AWB;
					devp->flag |= ISP_FLAG_AWB;
					cap_sm->adj_cnt = 0;
					if(sm_debug)
						pr_info("[%s..]%s changed ae->awb.\n",DEVICE_NAME,__func__);
				}
				else if(parm->af_mode){
					cap_sm->capture_state = CAPTURE_TUNE_AF;
					devp->flag |= ISP_FLAG_AF;
					if(sm_debug)
						pr_info("[%s..]%s changed ae->af.\n",DEVICE_NAME,__func__);
				}else if(cap_sm->flash_on){
					devp->flag |= ISP_FLAG_AE;
					cap_sm->capture_state = CAPTURE_LOW_GAIN;
					if(sm_debug)
						pr_info("[%s..]%s changed ae->low gain.\n",DEVICE_NAME,__func__);
				}else {
					cap_sm->capture_state = CAPTURE_SINGLE;
					cap_sm->adj_cnt = 0;
					if(sm_debug)
						pr_info("[%s..]%s changed ae->single.\n",DEVICE_NAME,__func__);
				}
			}
			
			break;
		case CAPTURE_TUNE_AWB:
			if(cap_sm->adj_cnt > 0)
			{
				devp->flag &=(~ISP_FLAG_AWB);
				if(parm->af_mode) {
					devp->flag |= ISP_FLAG_AF;
					cap_sm->capture_state = CAPTURE_TUNE_AF;
					if(sm_debug)
						pr_info("[%s..]%s changed awb->af.\n",DEVICE_NAME,__func__);
				}else if(cap_sm->flash_on){
					devp->flag |= ISP_FLAG_AE;
					cap_sm->capture_state = CAPTURE_LOW_GAIN;
					if(sm_debug)
						pr_info("[%s..]%s changed awb->low gain.\n",DEVICE_NAME,__func__);
				}else{
					cap_sm->capture_state = CAPTURE_SINGLE;	
					cap_sm->adj_cnt = 0;
					if(sm_debug)
					        pr_info("[%s..]%s changed awb->single.\n",DEVICE_NAME,__func__);
				}
			}
			break;
		case CAPTURE_TUNE_AF:
			if(sm_state.af_state == AF_SUCCESS){
				devp->flag &=(~ISP_FLAG_AF);
				if(cap_sm->flash_on){
					devp->flag |= ISP_FLAG_AE;
					isp_ae_low_gain();
					cap_sm->capture_state = CAPTURE_LOW_GAIN;
					if(sm_debug)
						pr_info("[%s..]%s changed af->low gain.\n",DEVICE_NAME,__func__);
				}else{
					cap_sm->capture_state = CAPTURE_SINGLE;
					cap_sm->adj_cnt = 0;
					if(sm_debug)
						pr_info("[%s..]%s changed af->single.\n",DEVICE_NAME,__func__);
				}
			}
			break;
		case CAPTURE_LOW_GAIN:
			if(sm_state.ae_down==true){
				devp->flag &=(~ISP_FLAG_AE);
				cap_sm->capture_state = CAPTURE_EYE_WAIT;	
				if(sm_debug)
					pr_info("[%s..]%s changed low gain->eye wait.\n",DEVICE_NAME,__func__);
			}
			break;
		case CAPTURE_EYE_WAIT:
			if(time_after(jiffies,start_jf+parm->eyetime)){
				isp_set_flash(devp,FLASH_TORCH,0);
				start_jf = 0;
				cap_sm->capture_state = CAPTURE_POS_WAIT;	
				if(sm_debug)
				        pr_info("[%s..]%s changed eye wait->post wait:%u.\n",DEVICE_NAME,__func__,start_jf);
			}
			break;
		case CAPTURE_POS_WAIT:
			if(time_after(jiffies,start_jf+parm->postime)){
				cap_sm->capture_state = CAPTURE_FLASHW;
				isp_set_flash(devp,FLASH_ON,0);
				start_jf = 0;
				if(sm_debug)
				        pr_info("[%s..]%s changed post waite->flash wait.\n",DEVICE_NAME,__func__);
			}
			break;
		case CAPTURE_FLASHW:
			if(time_after(jiffies,start_jf+cap_sm->fr_time)){
				isp_set_flash(devp,FLASH_TORCH,0);
				ret = TVIN_BUF_NULL;
				if(sm_debug)
					pr_info("[%s..]%s flash wait end,report buffer.\n",DEVICE_NAME,__func__);
			}
			break;
		case CAPTURE_SINGLE:
			if(cap_sm->adj_cnt < parm->sigle_count){
				for(j=0;j<devp->blnr_stat.ac_len;j++)
					cur_ac += devp->blnr_stat.ac[j];
				if(cur_ac > cap_sm->max_ac_sum){
					cap_sm->max_ac_sum = cur_ac;
					ret = TVIN_BUF_TMP;
				}
			}else{
				ret = TVIN_BUF_RECYCLE_TMP;
				if(parm->multi_capture_num > 0){
					cap_sm->capture_state = CAPTURE_MULTI;
					cap_sm->adj_cnt = 1;
					if(sm_debug)
						pr_info("[%s..]%s changed single->multi.\n",DEVICE_NAME,__func__);
				}else{
					cap_sm->capture_state = CAPTURE_END;
					if(sm_debug)
						pr_info("[%s..]%s changed single->capture end.\n",DEVICE_NAME,__func__);
				}
			}
			break;
		case CAPTURE_MULTI:
			if(cap_sm->adj_cnt % parm->skip_step == 0) {
				ret = TVIN_BUF_NULL;
				if(multi_count++ > parm->multi_capture_num){
					cap_sm->capture_state = CAPTURE_END;
                                        if(sm_debug)
                                                pr_info("[%s..]%s muti capture end.\n",DEVICE_NAME,__func__);
				}
			} else {
				ret = TVIN_BUF_SKIP;
			}
			break;
		case CAPTURE_END:
			ret = TVIN_BUF_RECYCLE_TMP;
			break;
		default:
			break;
		}
	
	return ret;
}

#define MAX_ABC(a,b,c)  (max(max(a,b),c))

void isp_ae_enh_sm(isp_dev_t *devp)
{
	struct isp_ae_stat_s *ae = &devp->isp_ae;
	struct xml_algorithm_ae_s *aep = devp->isp_ae_parm;
	struct isp_info_s *parm = &devp->info;
	struct vframe_prop_s *ph = &devp->frontend.private_data;
	struct cam_function_s *func = &devp->cam_param->cam_function;
	struct isp_ae_sm_s *aepa = &sm_state.isp_ae_parm;
	int i;
	unsigned char r_max = 0, g_max = 0, b_max = 0;
	unsigned short r_rate = 0, g_rate = 0, b_rate = 0, max_rate = 0;
	unsigned short targ_gain;
	unsigned char targstep, newstep, lpfcoef;
	unsigned int avg_r, avg_g, avg_b, avg_o;

	switch(sm_state.isp_ae_parm.isp_ae_enh_state){
		case AE_ENH_IDLE:
			break;
		case AE_ENH_INIT:
			aepa->pixel_sum = parm->h_active * parm->v_active;
			aepa->countlimit_r = ((aepa->pixel_sum >> 4) * aep->ratio_histr) >> 6;
			aepa->countlimit_g = ((aepa->pixel_sum >> 4) * aep->ratio_histg) >> 6;
			aepa->countlimit_b = ((aepa->pixel_sum >> 4) * aep->ratio_histb) >> 6;
			aepa->max_gain = func->get_aet_max_gain;
			aepa->min_gain = func->get_aet_min_gain;
			aepa->max_step = func->get_aet_max_step;
			aepa->cur_gain = func->get_aet_current_gain;
			//aepa->cur_step = func->get_aet_current_step;		
			sm_state.isp_ae_parm.isp_ae_enh_state = AE_ENH_ORI_SET;
			break;
		case AE_ENH_ORI_SET:
			sm_state.ae_down = false;
			aepa->cur_step = func->get_aet_current_step;
			newstep = aepa->cur_step;
			if(aep->ae_skip[1] == 0x1)
			{
				func->set_aet_new_step(newstep,true,true);
				sm_state.isp_ae_parm.isp_ae_state = AE_ENH_REST;							
			}
			else
			{
				func->set_aet_new_step(newstep,true,false);
				sm_state.isp_ae_parm.isp_ae_state = AE_ENH_GAIN_ADJUST;
			}
			break;
		case AE_ENH_LOW_GAIN:
			sm_state.ae_down = false;
			aepa->cur_gain = func->get_aet_current_gain;
			targ_gain = (aepa->cur_gain << 10)/aepa->tf_ratio;
			//newstep = find_step(func,0,aepa->max_step,targ_gain);
			if(aep->ae_skip[1] == 0x1)
			{
				func->set_aet_new_step(newstep,true,true);
				sm_state.isp_ae_parm.isp_ae_state = AE_ENH_REST;							
			}
			else
			{
				func->set_aet_new_step(newstep,true,false);
				sm_state.isp_ae_parm.isp_ae_state = AE_ENH_GAIN_ADJUST;
			}
			break;
		case AE_ENH_SET_RGB:
			sm_state.isp_ae_parm.isp_ae_enh_state = AE_ENH_RGB_WAIT;
			break;
		case AE_ENH_RGB_WAIT:
			sm_state.isp_ae_parm.isp_ae_enh_state = AE_ENH_SET_GBR;
			break;	
	    case AE_ENH_SET_GBR:
			if(devp->flag & ISP_FLAG_CAPTURE)
			{
				if(ph->hist.pixel_sum != 0)
				avg_r = ph->hist.luma_sum/ph->hist.pixel_sum;
			}
			for(i = 63;i >= 0;i--)
			{
				if( ph->hist.gamma[i] >= aepa->countlimit_r)
				{
					r_max = i << 2;
					break;
				}
			}
			r_rate = (r_max << 18)/(awb_gain.r_val*aep->target_r);
			sm_state.isp_ae_parm.isp_ae_enh_state = AE_ENH_GBR_WAIT;
			break;
		case AE_ENH_GBR_WAIT:
			sm_state.isp_ae_parm.isp_ae_enh_state = AE_ENH_SET_BRG;
			break;
		case AE_ENH_SET_BRG:
			if(devp->flag & ISP_FLAG_CAPTURE)
			{
				if(ph->hist.pixel_sum != 0)
				avg_g = ph->hist.luma_sum/ph->hist.pixel_sum;
			}
			for(i = 63;i >= 0;i--)
			{
				if( ph->hist.gamma[i] >= aepa->countlimit_g)
				{
					g_max = i << 2;
					break;
				}
			}
			g_rate = (g_max << 18)/(awb_gain.g_val*aep->target_g);
			sm_state.isp_ae_parm.isp_ae_enh_state = AE_ENH_BRG_WAIT;
			break;
		case AE_ENH_BRG_WAIT:
			sm_state.isp_ae_parm.isp_ae_enh_state = AE_ENH_SHUTTER_ADJUST;
			break;
        case AE_ENH_SHUTTER_ADJUST:
			sm_state.ae_down = false;
			if(devp->flag & ISP_FLAG_CAPTURE)
			{
				if(ph->hist.pixel_sum != 0)
				avg_b = ph->hist.luma_sum/ph->hist.pixel_sum;
				avg_o = (263*avg_r+516*avg_g+100*avg_b)/aepa->cur_gain >> 10;
				if(avg_o < aep->flash_thr_enh)
					sm_state.flash = ISP_FLASH_STATUS_ON;
				else
					sm_state.flash = ISP_FLASH_STATUS_OFF;
			}			
			for(i = 63;i >= 0;i--)
			{
				if( ph->hist.gamma[i] >= aepa->countlimit_b)
				{
					b_max = i << 2;
					break;
				}
			}
			b_rate = (b_max << 18)/(awb_gain.b_val*aep->target_b);
			max_rate = MAX_ABC(r_rate, g_rate, b_rate);
			if(((sm_state.status == ISP_AE_STATUS_UNSTABLE)&&((max_rate > 1024 + aep->maxrate_inner)||(max_rate < 1024 - aep->maxrate_inner)))
							||((max_rate > 1024 + aep->maxrate_outer)||(max_rate < 1024 - aep->maxrate_outer))
							)
			{
				sm_state.status = ISP_AE_STATUS_UNSTABLE;;
				targ_gain = (aepa->cur_gain << 10)/max_rate;
			    if(targ_gain > aepa->max_gain)
					targ_gain = aepa->max_gain;
				if(targ_gain < aepa->min_gain)
					targ_gain = aepa->min_gain;
				//targstep = find_step(func,0,aepa->max_step,targ_gain);
				lpfcoef = (devp->flag & ISP_FLAG_CAPTURE)?aep->fast_lpfcoef:aep->slow_lpfcoef;
				newstep = (ae->curstep*lpfcoef + targstep*(256-lpfcoef))>>8;
				if(aep->ae_skip[1] == 0x1)
				{
					func->set_aet_new_step(newstep,true,true);
					sm_state.isp_ae_parm.isp_ae_enh_state = AE_ENH_REST;							
				}
				else
				{
					func->set_aet_new_step(newstep,true,false);
					sm_state.isp_ae_parm.isp_ae_enh_state = AE_ENH_GAIN_ADJUST;
				}
			}
			else
			{
				sm_state.status = ISP_AE_STATUS_STABLE;
				if(func->check_mains_freq)
					sm_state.isp_ae_parm.isp_ae_enh_state = AE_ENH_ORI_SET;
				else
					sm_state.isp_ae_parm.isp_ae_enh_state = AE_ENH_SET_RGB;
			}
			break;
		case AE_ENH_GAIN_ADJUST:
			func->set_aet_new_step(newstep,false,true);
			sm_state.isp_ae_parm.isp_ae_state = AE_REST;							
			break;
		case AE_ENH_REST:
			sm_state.ae_down = true;
			if(func->check_mains_freq)
				sm_state.isp_ae_parm.isp_ae_state = AE_ENH_ORI_SET;
			else
				sm_state.isp_ae_parm.isp_ae_state = AE_ENH_SET_RGB;
			break;
}
}

void isp_awb_enh_sm(isp_dev_t *devp)
{
	struct isp_awb_stat_s *awb = &devp->isp_awb;
	struct xml_algorithm_awb_s *awbp = devp->isp_awb_parm;
	struct isp_info_s *parm = &devp->info;
	struct isp_awb_sm_s *awba = &sm_state.isp_awb_parm;
    unsigned int count;
	int u, v;
	unsigned char r, g, b, lpfcoef;
	unsigned short rw, gw, bw, rgbw;
	unsigned short targ_r, targ_g, targ_b, new_r, new_g, new_b;
	
	switch(sm_state.isp_awb_parm.isp_awb_state){
		case AWB_IDLE:
			break;
		case AWB_INIT:
			awba->win_l = (parm->h_active * awbp->ratio_winl) >> 10;
			awba->win_r = (parm->h_active * awbp->ratio_winr) >> 10 - 1;
			awba->win_t = (parm->v_active * awbp->ratio_wint) >> 10;
			awba->win_b = (parm->v_active * awbp->ratio_winb) >> 10 - 1;			
			isp_set_awb_win(awba->win_l, awba->win_r, awba->win_t, awba->win_b);
			awba->pixel_sum = parm->h_active * parm->v_active;
			awba->countlimityuv = ((awba->pixel_sum >> 4) * awbp->ratio_yuv) >> 6;
			awba->coun = 10;
			awba->y = 21 + 11 * awba->coun;
			awba->w = ((awba->y - 16) << 8)/220;
			isp_set_awb_yuv_thr(27+11*awba->coun,15+11*awba->coun,awbp->thr_u[awba->coun],awbp->thr_v[awba->coun]);
			sm_state.isp_awb_parm.isp_awb_state = AWB_CHECK;

			break;
		case AWB_CHECK:
			count = awb->yuv_mid[0].count + awb->yuv_mid[1].count;
			if(count < awba->countlimityuv)				
				break;
			u = (awb->yuv_mid[1].sum - awb->yuv_mid[0].sum)/count;
			v = (awb->yuv_mid[3].sum - awb->yuv_mid[2].sum)/count;
			awba->y = 21 + 11 * awba->coun;
			awba->w = ((awba->y - 16) << 8)/220;
			if(parm->v_active >= 720)
			{
				r = matrix_yuv709_rgb_r(awba->y,u+128,v+128);
				g = matrix_yuv709_rgb_g(awba->y,u+128,v+128);
				b = matrix_yuv709_rgb_b(awba->y,u+128,v+128);
			}
			else
			{
				r = matrix_yuv601_rgb_r(awba->y,u+128,v+128);
				g = matrix_yuv601_rgb_g(awba->y,u+128,v+128);
				b = matrix_yuv601_rgb_b(awba->y,u+128,v+128);
			}
            rw = (r << 10)/awba->w;
			gw = (g << 10)/awba->w;
			bw = (b << 10)/awba->w;
			rgbw = MAX_ABC(abs(rw-1024),abs(gw-1024),abs(bw-1024));
			if(((awba->status == ISP_AWB_STATUS_UNSTABLE) && ((rgbw > 1024 + awbp->inner)||(rgbw < 1024 - awbp->inner)))
				||((rgbw > 1024 + awbp->outer)||(rgbw < 1024 - awbp->outer))
				)
			{
				awba->status = ISP_AWB_STATUS_UNSTABLE;
				targ_r = (awb_gain.r_val<<10)/rw;
				targ_g = (awb_gain.g_val<<10)/gw;
				targ_b = (awb_gain.b_val<<10)/bw;
				if(targ_r > awbp->rw_limith)
					targ_r = awbp->rw_limith;
				if(targ_r < awbp->rw_limitl)
					targ_r = awbp->rw_limitl;
				if(targ_g > awbp->gw_limith)
					targ_g = awbp->gw_limith;
				if(targ_g < awbp->gw_limith)
					targ_g = awbp->gw_limith;
				if(targ_b > awbp->bw_limith)
					targ_b = awbp->bw_limith;
				if(targ_b < awbp->bw_limith)
					targ_b = awbp->bw_limith;
				lpfcoef = (devp->flag & ISP_FLAG_CAPTURE)?awbp->fast_lpfcoef:awbp->slow_lpfcoef;
				new_r = (awb_gain.r_val*lpfcoef + targ_r*(256-lpfcoef))>>8;
				new_g = (awb_gain.g_val*lpfcoef + targ_g*(256-lpfcoef))>>8;
				new_b = (awb_gain.b_val*lpfcoef + targ_b*(256-lpfcoef))>>8;
				awb_gain.r_val = new_r;
				awb_gain.g_val = new_g;
				awb_gain.b_val = new_b;
			}
			else
			{
				awba->status = ISP_AWB_STATUS_STABLE;
			}
			awba->y++;
			if(awba->y > 20)
				awba->y = 0;
			isp_set_awb_yuv_thr(27+11*awba->coun,15+11*awba->coun,awbp->thr_u[awba->coun],awbp->thr_v[awba->coun]);

			break;
	}

}

void isp_ae_sm(isp_dev_t *devp)
{
	if(devp->isp_ae_parm->ae_algorithm == 1)
		isp_ae_enh_sm(devp);
	else
		isp_ae_base_sm(devp);
}

void isp_awb_sm(isp_dev_t *devp)
{
	if(devp->isp_awb_parm->awb_algorithm == 1)
		isp_awb_enh_sm(devp);
	else
		isp_awb_base_sm(devp);
}

module_param(best_step_debug,uint,0664);
MODULE_PARM_DESC(best_step_debug,"\n debug flag for calc best focus position.\n");

module_param(af_sm_dg,uint,0664);
MODULE_PARM_DESC(af_sm_dg,"\n debug flag for auto focus.\n");


