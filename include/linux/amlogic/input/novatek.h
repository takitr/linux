/*
 * include/linux/novatek_touchdriver.h
 *
 * Copyright (C) 2010 - 2011 Novatek, Inc.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef 	_LINUX_NOVATEK_TOUCH_H 
#define		_LINUX_NOVATEK_TOUCH_H

#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>


#define NOVATEK_TS_ADDR		(0x01)
#define NOVATEK_HW_ADDR		(0x7F)
#define NOVATEK_FW_ADDR 	(0x01)

//*************************TouchScreen Work Part*****************************

#define NOVATEK_TS_NAME		"nt1100x"
//#define	NOVATEK_TS_SCLK		200*1000

/*******************Step 1: define resolution *****************/
//define default resolution of the touchscreen
//#define TOUCH_MAX_HEIGHT	960//1024			
//#define TOUCH_MAX_WIDTH		1920 //	1984

//define default resolution of LCM
//#define SCREEN_MAX_HEIGHT   600
//#define SCREEN_MAX_WIDTH    1024
//#define TOOL_PRESSURE		100
//#if 0
//#define INT_PORT  	    S3C64XX_GPN(15)						//Int IO port  S3C64XX_GPL(10)
//#ifdef INT_PORT
//	#define TS_INT 		gpio_to_irq(INT_PORT)			//Interrupt Number,EINT18(119)
//	#define INT_CFG    	S3C_GPIO_SFN(2)					//IO configer as EINT
//#else
//	#define TS_INT	0
//#endif	
//#else
///**********************Step 2: Setting Interrupt******************************/
//#define BABBAGE_NOVATEK_TS_RST1  (('k'-'a')*8+7)
//#define BABBAGE_NOVATEK_TS_INT1 (('j'-'a')*8+0)
//
//#define INT_PORT BABBAGE_NOVATEK_TS_INT1
//#define TS_INT gpio_to_irq(INT_PORT)
//
//#endif
/******************Step 3: Setting Reset option**************************/

//#define Novatek_HWRST_LowLeval()    (gpio_set_value(BABBAGE_NOVATEK_TS_RST1, 0))
//#define Nvoatek_HWRST_HighLeval()   (gpio_set_value(BABBAGE_NOVATEK_TS_RST1, 1))
//whether need send cfg?
struct tp_event {
	u16	x;
	u16	y;
    	s16 id;
	u16	pressure;
	u8  touch_point;
	u8  flag;
};
#define DRIVER_SEND_CFG
#define NT11002   1
#define NT11003   0
#define IC_DEFINE   NT11002
#if IC_DEFINE == NT11002
#define IIC_BYTENUM    4
#elif IC_DEFINE == NT11003
#define IIC_BYTENUM    6
#endif
//set trigger mode
#define INT_TRIGGER		0

#define POLL_TIME		10	//actual query spacing interval:POLL_TIME+6

#define NOVATEK_MULTI_TOUCH
#ifdef NOVATEK_MULTI_TOUCH
	#define MAX_FINGER_NUM	2//10	
#else
	#define MAX_FINGER_NUM	1	
#endif
//#define  HAVE_TOUCH_KEY
#ifdef HAVE_TOUCH_KEY
#define MENU        21
#define HOME        22
#define BACK        23
#define VOLUMEDOWN  24
#define VOLUMEUP    25
#endif

//#define swap(x, y) do { typeof(x) z = x; x = y; y = z; } while (0)

struct novatek_ts_data {
	uint16_t addr;
	uint8_t bad_data;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_reset;		//use RESET flag
	int use_irq;		//use EINT flag
	int read_mode;		//read moudle mode,20110221 by andrew
	struct hrtimer timer;
	struct work_struct  work;
	char phys[32];
	int retry;
	struct early_suspend early_suspend;
	int (*power)(struct novatek_ts_data * ts, int on);
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t int_trigger_type;
	uint8_t green_wake_mode;
};

//static const char *novatek_ts_name = "nt1103-ts";
//static struct workqueue_struct *novatek_wq;
//struct i2c_client * i2c_connect_client_novatek = NULL; 
//static struct proc_dir_entry *novatek_proc_entry;
//static struct kobject *novatek_debug_kobj;
//	
//#ifdef CONFIG_HAS_EARLYSUSPEND
//static void novatek_ts_early_suspend(struct early_suspend *h);
//static void novatek_ts_late_resume(struct early_suspend *h);
//#endif 

//*****************************End of Part I *********************************

//*************************Touchkey Surpport Part*****************************
//#define HAVE_TOUCH_KEY
#ifdef HAVE_TOUCH_KEY
	#define READ_COOR_ADDR 0x00
	const uint16_t touch_key_array[]={
									  KEY_MENU,				//MENU
									  KEY_HOME,				//HOME
									  KEY_SEND				//CALL
									  
									 }; 
	#define MAX_KEY_NUM	 (sizeof(touch_key_array)/sizeof(touch_key_array[0]))
#else
	#define READ_COOR_ADDR 0x00
#endif
//*****************************End of Part II*********************************


// Chip Reset define 
#define  HW_RST      0
#define  SW_RST      1

#if 0
#define KFprintk(x...) printk(x)
#else
#define KFprintk(x...) do{} while(0)
#endif
static struct early_suspend novatek_power; //for ealy suspend
static struct i2c_client *this_client;

#define NVT_APK_DRIVER_FUNC_SUPPORT
//#define NVT_FW_UPDATE_FUNC_SUPPORT

#endif /* _LINUX_NOVATEK_TOUCH_H */
