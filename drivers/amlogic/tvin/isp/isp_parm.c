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

/* Standard Linux Headers */
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/fs.h>

/* Amlogic Headers */
#include <linux/tvin/tvin_v4l2.h>

#include "isp_drv.h"  

#define DEVICE_NAME "isp"

static void isp_param_show(isp_param_t *parm, int len)
{
	int i=0,j=0;

	for(i=0;i<len;i++){
		if(ISP_U32==parm[i].type || ISP_U16== parm[i].type || ISP_U8== parm[i].type) {
			for(j=0;j<parm[i].length;j++)
				pr_info("%s %s[%d]=0x%x.\n",__func__,parm[i].name,j,*(parm[i].param+j));
		} else if(ISP_FLOAT == parm[i].type) {
			for(j=0;j<parm[i].length;j++)
				pr_info("%s %s[%d]=%f.\n",__func__,parm[i].name,j,*(parm[i].param+j));
		}
	}
}

static int isp_set_param(isp_param_t *parm,int len,char **buf)
{
	int i=0,j=0;
	for(i=0;i<len;i++) {
		if(!strcmp(parm[i].name,*buf)) {
			if(ISP_U32==parm[i].type || ISP_U8== parm[i].type || ISP_U16== parm[i].type) {
				for(j=0;j<parm[i].length&&*(buf+j+1)!=NULL;j++) {
					*(parm[i].param+j) = simple_strtol(*(buf+j+1),NULL,16);
					pr_info("%s %s[%d]=0x%x.\n",__func__,parm[i].name,j,*(parm[i].param+j));
				}
			} else if(ISP_FLOAT == parm[i].type) {
				for(j=0;j<parm[i].length&&*(buf+j+1)!=NULL;j++) {
					sscanf(*(buf+j+1),"%f",(float*)(parm[i].param+j));
					pr_info("%s %s[%d]=%f.\n",__func__,parm[i].name,j,*(parm[i].param+j));
				}
			}
			break;
		}
	}
	if(i == len) {
		pr_err("[%s..]%s:the parameter name %s is error.\n",DEVICE_NAME,__func__,buf[0]);
		return 1;
	}
	return 0;
}

void set_ae_parm(xml_algorithm_ae_t *ae_sw,char **parm)
{
	int len = AE_PARM_NUM;
	isp_param_t ae[AE_PARM_NUM]={
		{"ae_algorithm",      &ae_sw->ae_algorithm,      1,  ISP_U32},	
		{"ae_statistics",     &ae_sw->ae_statistics[0],  3,  ISP_U32},
		{"ae_exp",            &ae_sw->ae_exp[0],         3,  ISP_U32},
		{"ae_ag",             &ae_sw->ae_ag[0],          3,  ISP_U32},
		{"ae_skip",           &ae_sw->ae_skip[0],        3,  ISP_U32},
		{"alert_mode",        &ae_sw->alert_mode,        1,  ISP_U32},
		{"tune_mode",         &ae_sw->tune_mode,         1,  ISP_U32},
		{"ratio_r",           &ae_sw->ratio_r,           1,  ISP_U32},
		{"ratio_g",           &ae_sw->ratio_g,           1,  ISP_U32},
		{"ratio_b",           &ae_sw->ratio_b,           1,  ISP_U32},
		{"stepdnr",           &ae_sw->stepdnr,           1,  ISP_U32},
		{"stepdng",           &ae_sw->stepdng,           1,  ISP_U32},
		{"stepdnb",           &ae_sw->stepdnb,           1,  ISP_U32},
		{"stepup",            &ae_sw->stepup,            1,  ISP_U32},
		{"slow_lpfcoef",      &ae_sw->slow_lpfcoef,      1,  ISP_U32},
		{"fast_lpfcoef",      &ae_sw->fast_lpfcoef,      1,  ISP_U32},
		{"coef_cur",          &ae_sw->coef_cur[0],       16, ISP_U32},
		{"coef_env",          &ae_sw->coef_env[0],       16, ISP_U32},
		{"env_hign",          &ae_sw->env_hign,          1,  ISP_U32},
		{"env_hign2mid",      &ae_sw->env_hign2mid,      1,  ISP_U32},
		{"env_low2mid",       &ae_sw->env_low2mid,       1,  ISP_U32},
		{"env_low",           &ae_sw->env_low,           1,  ISP_U32},
		{"thr_r_high",        &ae_sw->thr_r_high,        1,  ISP_U32},
		{"thr_r_mid",         &ae_sw->thr_r_mid,         1,  ISP_U32},
		{"thr_r_low",         &ae_sw->thr_r_low,         1,  ISP_U32},
		{"thr_g_high",        &ae_sw->thr_g_high,        1,  ISP_U32},
		{"thr_g_mid",         &ae_sw->thr_g_mid,         1,  ISP_U32},
		{"thr_g_low",         &ae_sw->thr_g_low,         1,  ISP_U32},
		{"thr_b_high",        &ae_sw->thr_b_high,        1,  ISP_U32},
		{"thr_b_mid",         &ae_sw->thr_b_mid,         1,  ISP_U32},		
		{"thr_b_low",         &ae_sw->thr_b_low,         1,  ISP_U32},
		{"lpftype_high",      &ae_sw->lpftype_high,      1,  ISP_U32},
		{"lpftype_mid",       &ae_sw->lpftype_mid,       1,  ISP_U32},		
		{"lpftype_low",       &ae_sw->lpftype_low,       1,  ISP_U32},
		{"targethigh",        &ae_sw->targethigh,        1,  ISP_U32},
		{"targetmid",         &ae_sw->targetmid,         1,  ISP_U32},
		{"targetlow",         &ae_sw->targetlow,         1,  ISP_U32},
		{"radium_inner_h",    &ae_sw->radium_inner_h,    1,  ISP_U32},
		{"radium_outer_h",    &ae_sw->radium_outer_h,    1,  ISP_U32},
		{"radium_inner_m",    &ae_sw->radium_inner_m,    1,  ISP_U32},
		{"radium_outer_m",    &ae_sw->radium_outer_m,    1,  ISP_U32},
		{"radium_inner_l",    &ae_sw->radium_inner_l,    1,  ISP_U32},
		{"radium_outer_l",    &ae_sw->radium_outer_l,    1,  ISP_U32},		
		{"flash_thr",	      &ae_sw->flash_thr,         1,  ISP_U32},
		{"ratio_histr",	      &ae_sw->ratio_histr,       1,  ISP_U32},
		{"ratio_histg",	      &ae_sw->ratio_histg,       1,  ISP_U32},		
		{"ratio_histb",	      &ae_sw->ratio_histb,       1,  ISP_U32},
		{"target_r",	      &ae_sw->target_r,          1,  ISP_U32},
		{"target_g",	      &ae_sw->target_g,          1,  ISP_U32},
		{"target_b",	      &ae_sw->target_b,          1,  ISP_U32},
		{"maxrate_inner",     &ae_sw->maxrate_inner,     1,  ISP_U32},		
		{"maxrate_outer",     &ae_sw->maxrate_outer,     1,  ISP_U32},
		{"slow_lpfcoef_enh",  &ae_sw->slow_lpfcoef_enh,  1,  ISP_U32},
		{"fast_lpfcoef_enh",  &ae_sw->fast_lpfcoef_enh,  1,  ISP_U32},
		{"flash_thr_enh",     &ae_sw->flash_thr_enh,     1,  ISP_U32},
		
	};

	if(!strcmp(parm[0],"show")){
		isp_param_show((isp_param_t*)&ae,len);
	} else {
		isp_set_param((isp_param_t*)&ae,len,parm);
	}

}

void set_awb_parm(xml_algorithm_awb_t *awb_sw,char **parm)
{
	int len = AWB_PARM_NUM;
	isp_param_t awb[AWB_PARM_NUM]={		
		{"awb_algorithm",  &awb_sw->awb_algorithm,   1,  ISP_U32},
		{"ratio_rgb",      &awb_sw->ratio_rgb,       1,  ISP_U32},
		{"ratio_yh",       &awb_sw->ratio_yh,        1,  ISP_U32},
		{"ratio_ym",       &awb_sw->ratio_ym,        1,  ISP_U32},
		{"ratio_yl",       &awb_sw->ratio_yl,        1,  ISP_U32},
		{"yyh",            &awb_sw->yyh,             1,  ISP_U32},
		{"yym",            &awb_sw->yym,             1,  ISP_U32},
		{"yyl",            &awb_sw->yyl,             1,  ISP_U32},
		{"coef_r",         &awb_sw->coef_r[0],       4,  ISP_U32},		
		{"coef_g",         &awb_sw->coef_g[0],       4,  ISP_U32},		
		{"coef_b",         &awb_sw->coef_b[0],       4,  ISP_U32},		
		{"inner_rg",       &awb_sw->inner_rg,        1,  ISP_U32},
		{"outer_rg",       &awb_sw->outer_rg,        1,  ISP_U32},
		{"inner_bg",       &awb_sw->inner_bg,        1,  ISP_U32},
		{"outer_bg",       &awb_sw->outer_bg,        1,  ISP_U32},
		{"r_max",          &awb_sw->r_max,           1,  ISP_U32},		
		{"r_min",          &awb_sw->r_min,           1,  ISP_U32},
		{"b_max",          &awb_sw->b_max,           1,  ISP_U32},
		{"b_min",          &awb_sw->b_min,           1,  ISP_U32},
		{"thr_gb_h",	   &awb_sw->thr_gb_h,	     1,  ISP_U32},
		{"thr_gb_m",	   &awb_sw->thr_gb_m,	     1,  ISP_U32},
		{"thr_gb_l",	   &awb_sw->thr_gb_l,	     1,  ISP_U32},
		{"thr_gr_h",	   &awb_sw->thr_gr_h,	     1,  ISP_U32},
		{"thr_gr_m",	   &awb_sw->thr_gr_m,	     1,  ISP_U32},
		{"thr_gr_l",	   &awb_sw->thr_gr_l,	     1,  ISP_U32},
		{"thr_br_h",	   &awb_sw->thr_br_h,	     1,  ISP_U32},
		{"thr_br_m",	   &awb_sw->thr_br_m,	     1,  ISP_U32},
		{"thr_br_l",	   &awb_sw->thr_br_l,	     1,  ISP_U32},
		{"thr_du_h",	   &awb_sw->thr_du_h,	     1,  ISP_U32},
		{"thr_du_m",	   &awb_sw->thr_du_m,	     1,  ISP_U32},
		{"thr_du_l",	   &awb_sw->thr_du_l,	     1,  ISP_U32},
		{"thr_dv_h",	   &awb_sw->thr_dv_h,	     1,  ISP_U32},
		{"thr_dv_m",	   &awb_sw->thr_dv_m,	     1,  ISP_U32},
		{"thr_dv_l",	   &awb_sw->thr_dv_l,	     1,  ISP_U32},
		{"thr_yh_h",	   &awb_sw->thr_yh_h,	     1,  ISP_U32},
		{"thr_yh_m",	   &awb_sw->thr_yh_m,	     1,  ISP_U32},
		{"thr_yh_l",	   &awb_sw->thr_yh_l,	     1,  ISP_U32},
		{"thr_yl_h",	   &awb_sw->thr_yl_h,	     1,  ISP_U32},
		{"thr_yl_m",	   &awb_sw->thr_yl_m,	     1,  ISP_U32},
		{"thr_yl_l",	   &awb_sw->thr_yl_l,	     1,  ISP_U32},
	        {"ratio_yuv",      &awb_sw->ratio_yuv,       1,  ISP_U32},
		{"slow_lpfcoef",   &awb_sw->slow_lpfcoef,    1,  ISP_U32},
		{"fast_lpfcoef",   &awb_sw->fast_lpfcoef,    1,  ISP_U32},
		{"outer",          &awb_sw->outer,           1,  ISP_U32},
		{"inner",          &awb_sw->inner,           1,  ISP_U32},
		{"rw_limith",      &awb_sw->rw_limith,       1,  ISP_U32},
		{"rw_limitl",      &awb_sw->rw_limitl,       1,  ISP_U32},
		{"gw_limith",      &awb_sw->gw_limith,       1,  ISP_U32},
		{"gw_limitl",      &awb_sw->gw_limitl,       1,  ISP_U32},
		{"bw_limith",      &awb_sw->bw_limith,       1,  ISP_U32},
		{"bw_limitl",      &awb_sw->bw_limitl,       1,  ISP_U32},
		{"thr_u",          &awb_sw->thr_u[0],        20, ISP_U32},
		{"thr_v",          &awb_sw->thr_v[0],        20, ISP_U32},
	};
	
	if(!strcmp(parm[0],"show")){
		isp_param_show((isp_param_t*)&awb,len);
	} else {
		isp_set_param((isp_param_t*)&awb,len,parm);
	}
}

void set_af_parm(xml_algorithm_af_t *af_sw,char **parm)
{
	int len = AF_PARM_NUM;		
	isp_param_t af[AF_PARM_NUM]={
		{"f0_coef0",      &af_sw->f0_coef0,      1, ISP_U32},
		{"f0_coef1",      &af_sw->f0_coef1,      1, ISP_U32},
		{"f0_coef2",      &af_sw->f0_coef2,      1, ISP_U32},
		{"f0_coef3",      &af_sw->f0_coef3,      1, ISP_U32},
		{"f0_coef4",      &af_sw->f0_coef4,      1, ISP_U32},
		{"f0_coef5",      &af_sw->f0_coef5,      1, ISP_U32},
		{"f0_coef6",      &af_sw->f0_coef6,      1, ISP_U32},
		{"f0_coef7",      &af_sw->f0_coef7,      1, ISP_U32},		
		{"f1_coef0",      &af_sw->f1_coef0,      1, ISP_U32},
		{"f1_coef1",      &af_sw->f1_coef1,      1, ISP_U32},
		{"f1_coef2",      &af_sw->f1_coef2,      1, ISP_U32},
		{"f1_coef3",      &af_sw->f1_coef3,      1, ISP_U32},
		{"f1_coef4",      &af_sw->f1_coef4,      1, ISP_U32},
		{"f1_coef5",      &af_sw->f1_coef5,      1, ISP_U32},
		{"f1_coef6",      &af_sw->f1_coef6,      1, ISP_U32},
		{"f1_coef7",      &af_sw->f1_coef7,      1, ISP_U32},
		{"f0_coef",       &af_sw->f0_coef,       1, ISP_U32},
		{"f1_coef",       &af_sw->f1_coef,       1, ISP_U32},		
		{"adapting_time", &af_sw->adapting_time, 1, ISP_U32},
	};
	
	if(!strcmp(parm[0],"show")){
		isp_param_show((isp_param_t*)&af,len);
	} else {
		isp_set_param((isp_param_t*)&af,len,parm);
	}
}

void set_cap_parm(struct xml_capture_s *cap_sw,char **parm)
{
	int len = CAP_PARM_NUM;	
	isp_param_t cap[CAP_PARM_NUM]={
		{"ae_en",      	     &cap_sw->ae_en,      	   1, ISP_U32},
		{"awb_en",           &cap_sw->awb_en,      	   1, ISP_U32},
		{"af_mode",          &cap_sw->af_mode,      	   1, ISP_U32},
		{"sigle_count",      &cap_sw->sigle_count,         1, ISP_U32},
		{"skip_step",        &cap_sw->skip_step,      	   1, ISP_U32},
		{"multi_capture_num",&cap_sw->multi_capture_num,   1, ISP_U32},
		{"eyetime",          &cap_sw->eyetime,      	   1, ISP_U32},		
		{"pretime",          &cap_sw->pretime,      	   1, ISP_U32},
		{"postime",          &cap_sw->postime,      	   1, ISP_U32},
	};
	
	if(!strcmp(parm[0],"show")){
		isp_param_show((isp_param_t*)&cap,len);
	} else {
		isp_set_param((isp_param_t*)&cap,len,parm);
	}
	
}

void set_wave_parm(struct wave_s *wave,char **parm)
{
	int len = WAVE_PARM_NUM;	
	isp_param_t wav[WAVE_PARM_NUM]={
		{"torch_rising_time",      	&wave->torch_rising_time,      	1, ISP_U32},
		{"flash_rising_time",           &wave->flash_rising_time,      	1, ISP_U32},
		{"torch_flash_ratio",           &wave->torch_flash_ratio,      	1, ISP_U32},
		{"wave_clock_div",              &wave->wave_clock_div,          1, ISP_U32},
		{"pulse_init_time",             &wave->pulse_init_time,         1, ISP_U32},
		{"pulse_high_time",             &wave->pulse_high_time,      	1, ISP_U32},
		{"pulse_low_time",              &wave->pulse_low_time,          1, ISP_U32},
		{"time_to_latch",               &wave->time_to_latch,      	1, ISP_U32},		
		{"latch_time",                  &wave->latch_time,      	1, ISP_U32},
		{"latch_time_timeout",          &wave->latch_time_timeout,      1, ISP_U32},
		{"time_to_off",                 &wave->time_to_off,      	1, ISP_U32},
		{"pulse_qty_max",               &wave->pulse_qty_max,      	1, ISP_U32},
	};
	
	if(!strcmp(parm[0],"show")){
		isp_param_show((isp_param_t*)&wav,len);
	} else {
		isp_set_param((isp_param_t*)&wav,len,parm);
	}
	
}

