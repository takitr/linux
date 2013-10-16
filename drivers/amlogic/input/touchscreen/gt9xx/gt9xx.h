/* drivers/input/touchscreen/gt813_827_828.h
 * 
 * 2010 - 2012 Goodix Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 * Version:1.0
 *      V1.0:2012/08/31,first release.
 */

#ifndef _LINUX_GOODIX_TOUCH_H
#define	_LINUX_GOODIX_TOUCH_H

#include "linux/amlogic/input/common.h"
//#include <linux/ctp.h>
static int goodix_get_config(void);
static int goodix_free_config(void);
struct goodix_ts_data {
    spinlock_t irq_lock;
    struct i2c_client *client;
    struct input_dev  *input_dev;
    struct hrtimer timer;
    struct work_struct  work;
    struct early_suspend early_suspend;
    s32 irq_is_disable;
    s32 use_irq;
    u16 abs_x_max;
    u16 abs_y_max;
    u8  max_touch_num;
    u8  int_trigger_type;
    u8  green_wake_mode;
    u8  chip_type;
    u8  enter_update;
    u8  gtp_is_suspend;
    u8  gtp_rawdiff_mode;
    u8  gtp_cfg_len;
};

extern u16 show_len;
extern u16 total_len;

//***************************PART1:ON/OFF define*******************************
#define GTP_CUSTOM_CFG        1
#define GTP_DRIVER_SEND_CFG   1 
#define GTP_HAVE_TOUCH_KEY    0
#define GTP_POWER_CTRL_SLEEP  1
#define GTP_AUTO_UPDATE       0
#define GTP_CHANGE_X2Y        0
#define GTP_ESD_PROTECT       0
#define GTP_CREATE_WR_NODE    0
#define GTP_ICS_SLOT_REPORT   1

#define GUP_USE_HEADER_FILE   0

#define GTP_DEBUG_ON          0
#define GTP_DEBUG_ARRAY_ON    0
#define GTP_DEBUG_FUNC_ON     0

//***************************PART2:TODO define**********************************
//STEP_1(REQUIRED):Change config table.
/*TODO: puts the config info corresponded to your TP here, the following is just 
a sample config, send this config should cause the chip cannot work normally*/
//default or float
//#define CTP_CFG_GROUP1 {0x41,0x00,0x04,0x00,0x03,0x0A,0x3D,0x08,0x02,0x15,0x14,0x05,0x5A,0x41,0x03,0x05,0x00,0x00,0x00,0x00,0x35,0x5A,0x00,0x17,0x19,0x1C,0x14,0x91,0x31,0xCD,0x2A,0x2C,0x05,0x0D,0x00,0x00,0x00,0x82,0x03,0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x64,0xFA,0x94,0x85,0x02,0x08,0x00,0x00,0x9E,0x0F,0x1C,0x86,0x13,0x1B,0x67,0x18,0x1B,0x88,0x20,0x1A,0x7D,0x23,0x1D,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x25,0x0F,0x0A,0x14,0x00,0x00,0x02,0x00,0x61,0x03,0x14,0x32,0x00,0x01,0x14,0x28,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0E,0x0F,0x10,0x11,0x12,0x13,0x15,0x16,0x17,0x18,0x19,0x1B,0x1D,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF5,0x01}
#ifdef CONFIG_OF
#define CTP_CFG_GROUP1 (u8 *)config_info
#else
#define CTP_CFG_GROUP1 {0x41,0x00,0x06,0x00,0x08,0x0A,0x35,0x00,0x01,0xC7,0x1E,0x08,0x50,0x3C,0x03,0x03,0x02,0x02,0x00,0x00,0x11,0x11,0x09,0x16,0x19,0x1B,0x14,0x90,0x2F,0xAA,0x31,0x2F,0xB8,0x08,0x00,0x00,0x01,0x9A,0x03,0x1D,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x28,0x78,0x94,0x85,0x02,0x08,0x00,0x00,0x74,0x08,0x29,0xAD,0x08,0x31,0x85,0x09,0x38,0x02,0x0A,0x43,0xD7,0x0A,0x4D,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x05,0x06,0x07,0x08,0x09,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x14,0x15,0x16,0x17,0x18,0x19,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x29,0x28,0x27,0x26,0x25,0x24,0x23,0x22,0x21,0x20,0x1F,0x1E,0x1C,0x1B,0x19,0x14,0x13,0x12,0x11,0x10,0x0F,0x0E,0x0D,0x0C,0x0A,0x08,0x07,0x06,0x04,0x02,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF0,0x01}
#endif
//TODO puts your group2 config info here,if need.
//VDDIO
#define CTP_CFG_GROUP2 {\
    }
//TODO puts your group3 config info here,if need.
//GND
#define CTP_CFG_GROUP3 {\
    }

//STEP_2(REQUIRED):Change I/O define & I/O operation mode.
#define GTP_RST_PORT    (ts_com->gpio_reset)	
#define GTP_INT_PORT    (ts_com->gpio_interrupt)
#define GTP_INT_IRQ     (ts_com->irq)
//#define GTP_INT_CFG     S3C_GPIO_SFN(0xF)

#define GTP_GPIO_AS_INPUT(pin)          do{\
                                            aml_gpio_direction_input(pin);\
                                        }while(0)
#define GTP_GPIO_AS_INT(pin)            do{\
                                            aml_gpio_direction_input(pin);\
                                            aml_gpio_to_irq(pin, GTP_INT_IRQ, ts_com->irq_edge);\
                                        }while(0)
//#define GTP_GPIO_GET_VALUE(pin)         gpio_get_value(pin)
//#define GTP_GPIO_OUTPUT(pin,level)      gpio_direction_output(pin,level)
#define GTP_GPIO_OUTPUT(pin,level)	    do{\
                                            aml_gpio_direction_output(pin, level);\
                                        }while(0)

//#define GTP_GPIO_REQUEST(pin, label)    amlogic_gpio_request(pin, "gt9xx")

//#define GTP_GPIO_FREE(pin)              amlogic_gpio_free(pin, "gt9xx")
//#define GTP_IRQ_TAB                     {IRQ_TYPE_EDGE_RISING, IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_LEVEL_LOW, IRQ_TYPE_LEVEL_HIGH}

//STEP_3(optional):Custom set some config by themself,if need.
#if GTP_CUSTOM_CFG
  #define GTP_MAX_HEIGHT   800
  #define GTP_MAX_WIDTH    480
  #define GTP_INT_TRIGGER  1    //0:Rising 1:Falling
#else
  #define GTP_MAX_HEIGHT   4096
  #define GTP_MAX_WIDTH    4096
  #define GTP_INT_TRIGGER  1
#endif
#define GTP_MAX_TOUCH         5
#define GTP_ESD_CHECK_CIRCLE  2000

//STEP_4(optional):If this project have touch key,Set touch key config.                                    
#if GTP_HAVE_TOUCH_KEY
    #define GTP_KEY_TAB	 {KEY_MENU, KEY_HOME, KEY_BACK, KEY_SEND}
#endif

//***************************PART3:OTHER define*********************************
#define GTP_DRIVER_VERSION    "V1.2<2012/10/25>"
#define GTP_I2C_NAME          "gt9xx"
#define GTP_POLL_TIME         10
#define GTP_ADDR_LENGTH       2
#define GTP_CONFIG_MAX_LENGTH 240
#define FAIL                  0
#define SUCCESS               1

//Register define
#define GTP_READ_COOR_ADDR    0x814E
#define GTP_REG_SLEEP         0x8040
#define GTP_REG_SENSOR_ID     0x814A
#define GTP_REG_CONFIG_DATA   0x8047
#define GTP_REG_VERSION       0x8140

#define RESOLUTION_LOC        3
#define TRIGGER_LOC           8

//Log define
#define GTP_INFO(fmt,arg...)           printk("<<-GTP-INFO->> "fmt"\n",##arg)
#define GTP_ERROR(fmt,arg...)          printk("<<-GTP-ERROR->> "fmt"\n",##arg)
#define GTP_DEBUG(fmt,arg...)          do{\
                                         if(GTP_DEBUG_ON)\
                                         printk("<<-GTP-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)
#define GTP_DEBUG_ARRAY(array, num)    do{\
                                         s32 i;\
                                         u8* a = array;\
                                         if(GTP_DEBUG_ARRAY_ON)\
                                         {\
                                            printk("<<-GTP-DEBUG-ARRAY->>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                printk("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    printk("\n");\
                                                }\
                                            }\
                                            printk("\n");\
                                        }\
                                       }while(0)
#define GTP_DEBUG_FUNC()               do{\
                                         if(GTP_DEBUG_FUNC_ON)\
                                         printk("<<-GTP-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)
#define GTP_SWAP(x, y)                 do{\
                                         typeof(x) z = x;\
                                         x = y;\
                                         y = z;\
                                       }while (0)

//****************************PART4:UPDATE define*******************************
//Error no
#define ERROR_NO_FILE           2   //ENOENT
#define ERROR_FILE_READ         23  //ENFILE
#define ERROR_FILE_TYPE         21  //EISDIR
#define ERROR_GPIO_REQUEST      4   //EINTR
#define ERROR_I2C_TRANSFER      5   //EIO
#define ERROR_NO_RESPONSE       16  //EBUSY
#define ERROR_TIMEOUT           110 //ETIMEDOUT

//*****************************End of Part III********************************
extern struct touch_pdata *ts_com;
u8 *config_info = NULL;
#define READ_COUNT  5
static int goodix_get_config(void)
{
	u32 offset = 0,count = 0;
	
	int file_size;
	u8 tmp[READ_COUNT];
	int i_ret;

	file_size = touch_open_fw(ts_com->config_file);
	if(file_size < 0) {
			printk("%s: no fw file\n", ts_com->owner);
			return -1;
	}

	if (config_info == NULL)
		config_info = kzalloc(sizeof(*config_info)*(file_size/5), GFP_KERNEL);
	if (config_info == NULL) {
		printk("Insufficient memory in upgrade!\n");
		return -1;
	}

	while (offset < file_size) {
    touch_read_fw(offset, READ_COUNT, &tmp[0]);
    i_ret = sscanf(&tmp[0],"0x%x,",config_info + count);
    if (i_ret == 1) {
			count++;
		}
    offset++;
	}

	touch_close_fw();

//	uint8_t *cfg_info_group1 = config_info;
//
//	printk("cfg_info_len1 = %d\n",count);
//	int i=0;
//	for (i=0; i<count; i++ ) {
//		printk("%x ", cfg_info_group1[i]);
//	}
//	printk("\n");
	return count;
}

static int goodix_free_config(void)
{
	if (config_info != NULL) {
		kfree(config_info);
		config_info = NULL;
	}
	
	return 0;
}
#endif /* _LINUX_GOODIX_TOUCH_H */
