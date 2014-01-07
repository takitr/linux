#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/major.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
//#include <linux/amports/canvas.h>
#include <asm/uaccess.h>
#include <mach/am_regs.h>
#include <linux/amlogic/hdmi_tx/hdmi_info_global.h>
#include <linux/amlogic/hdmi_tx/hdmi_tx_module.h>
#include <mach/hdmi_tx_reg.h>
#include "hdmi_tx_hdcp.h"
/*
    hdmi_tx_hdcp.c
    version 1.0
*/

// Notic: the HDCP key setting has been moved to uboot
// On MBX project, it is too late for HDCP get from 
// other devices

/* verify ksv, 20 ones and 20 zeroes*/
int hdcp_ksv_valid(unsigned char * dat)
{
    int i, j, one_num = 0;
    for(i = 0; i < 5; i++){
        for(j=0;j<8;j++) {
            if((dat[i]>>j)&0x1) {
                one_num++;
            }
        }
    }
    if(one_num == 0)
        hdmi_print(INF, HDCP "no HDCP key available\n");
    return (one_num == 20);
}

