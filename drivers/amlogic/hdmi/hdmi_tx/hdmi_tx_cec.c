/*
 * Amlogic Meson HDMI Transmitter Driver
 * HDMI CEC Driver-----------HDMI_TX
 * Copyright (C) 2011 Amlogic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the named License,
 * or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/major.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/spinlock_types.h>
#include <linux/switch.h>
#include <linux/workqueue.h>

#include <asm/uaccess.h>
#include <asm/delay.h>
#include <mach/am_regs.h>
#include <mach/power_gate.h>
#include <linux/amlogic/tvin/tvin.h>

#include <mach/gpio.h>
#include "hw/hdmi_tx_reg.h"
#include "hdmi_tx_module.h"
#include "hdmi_info_global.h"
#include "hdmi_tx_cec.h"
#include "hdmi_parameter.h"

static hdmitx_dev_t *hdmitx_device = NULL;
struct input_dev *remote_cec_dev;
DEFINE_SPINLOCK(cec_input_key);
DEFINE_SPINLOCK(cec_init_lock);
static DEFINE_MUTEX(cec_mutex);
static DECLARE_WAIT_QUEUE_HEAD(cec_key_poll);

/* global variables */
static	unsigned char    gbl_msg[MAX_MSG];
cec_global_info_t cec_global_info;
unsigned char cec_power_flag = 0;
EXPORT_SYMBOL(cec_power_flag);
unsigned char rc_long_press_pwr_key = 0;
EXPORT_SYMBOL(rc_long_press_pwr_key);
static int cec_msg_dbg_en = 0;
static int cec_fiq_flag = 0;
static void cec_gpi_receive_bits(void);
static int cec_repeat_flag = 0;
static unsigned int cec_init_ready_flag = 0;
static int repeat;
static int cec_ll_tx_polling(const unsigned char *msg, unsigned char len);

/**************************************************
 *                  cec rx work queue             *
 *                                                *
 **************************************************/

#define CEC_WQ  0
struct work_struct cec_wq_func;
void cec_do_work(unsigned long data);

void cec_do_work(unsigned long data){
    //cec_isr_post_process();
}
#if CEC_WQ
struct workqueue_struct *cec_wq;
#endif

ssize_t	cec_lang_config_state(struct switch_dev *sdev, char *buf){
    int pos=0;
    pos+=snprintf(buf+pos, PAGE_SIZE, "%c%c%c\n", (cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_lang >>16) & 0xff,
                                                  (cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_lang >> 8) & 0xff,
                                                  (cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_lang >> 0) & 0xff);
    return pos;
};

struct switch_dev lang_dev = {	// android ics switch device
	.name = "lang_config",
	.print_state = cec_lang_config_state,
	};
EXPORT_SYMBOL(lang_dev);

static DEFINE_SPINLOCK(p_tx_list_lock);
static unsigned long cec_tx_list_flags;
static unsigned int tx_msg_cnt = 0;
static struct list_head cec_tx_msg_phead = LIST_HEAD_INIT(cec_tx_msg_phead);

unsigned int menu_lang_array[] = {(((unsigned int)'c')<<16)|(((unsigned int)'h')<<8)|((unsigned int)'i'),
                                  (((unsigned int)'e')<<16)|(((unsigned int)'n')<<8)|((unsigned int)'g'),
                                  (((unsigned int)'j')<<16)|(((unsigned int)'p')<<8)|((unsigned int)'n'),
                                  (((unsigned int)'k')<<16)|(((unsigned int)'o')<<8)|((unsigned int)'r'),
                                  (((unsigned int)'f')<<16)|(((unsigned int)'r')<<8)|((unsigned int)'a'),
                                  (((unsigned int)'g')<<16)|(((unsigned int)'e')<<8)|((unsigned int)'r')
                                 };

// CEC default setting
static unsigned char * osd_name = "Amlogic MBox";
static unsigned int vendor_id = 0x00;

cec_rx_msg_buf_t cec_rx_msg_buf;
static irqreturn_t cec_isr_handler(int irq, void *dev_instance);
#ifdef CONFIG_ARCH_MESON6
static void cec_gpi_init(void);
#endif
static unsigned char cec_init_flag = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
static struct early_suspend hdmitx_cec_early_suspend_handler;
static void hdmitx_cec_early_suspend(struct early_suspend *h)
{
    if(hdmitx_device->cec_func_config & (1 << CEC_FUNC_MSAK)) {
        cec_menu_status_smp(DEVICE_MENU_INACTIVE);
        cec_inactive_source();

	    if(rc_long_press_pwr_key == 1) {
	        cec_set_standby();
	        printk(KERN_INFO "CEC: get power-off command from Romote Control\n");
	        rc_long_press_pwr_key = 0;
	    }
	}
}

static void hdmitx_cec_late_resume(struct early_suspend *h)
{
    cec_hw_reset();//for M8 CEC standby.
    if(hdmitx_device->cec_func_config & (1 << CEC_FUNC_MSAK)) {
		cec_imageview_on_smp();
		cec_active_source_smp();
		msleep(200);
		cec_active_source_smp();
		cec_menu_status_smp(DEVICE_MENU_ACTIVE);
	}
    printk(KERN_INFO "HDMITX CEC: late resume\n");
}

#endif


int cec_ll_rx( unsigned char *msg, unsigned char *len)
{
    unsigned char i;
    unsigned char rx_status;
    unsigned char data;
    unsigned char msg_log_buf[128];
    int pos;
    unsigned char n;
    unsigned char *msg_start = msg;
    int rx_msg_length;
#ifdef CONFIG_ARCH_MESON6
    printk("cec irq status: rx: 0x%x; tx: 0x%x\n", hdmi_rd_reg(CEC0_BASE_ADDR+CEC_RX_MSG_STATUS), hdmi_rd_reg(CEC0_BASE_ADDR+CEC_TX_MSG_STATUS));
    if(RX_DONE != hdmi_rd_reg(CEC0_BASE_ADDR+CEC_RX_MSG_STATUS)){
        hdmi_wr_reg(CEC0_BASE_ADDR + CEC_RX_MSG_CMD,  RX_ACK_CURRENT);
        hdmi_wr_reg(CEC0_BASE_ADDR + CEC_RX_MSG_CMD,  RX_NO_OP);
        return -1;
    }

    if(1 != hdmi_rd_reg(CEC0_BASE_ADDR+CEC_RX_NUM_MSG)){
        hdmi_wr_reg(CEC0_BASE_ADDR + CEC_RX_MSG_CMD,  RX_ACK_CURRENT);
        hdmi_wr_reg(CEC0_BASE_ADDR + CEC_RX_MSG_CMD,  RX_NO_OP);
        return -1;
    }

    rx_msg_length = hdmi_rd_reg(CEC0_BASE_ADDR + CEC_RX_MSG_LENGTH) + 1;

    hdmi_wr_reg(CEC0_BASE_ADDR + CEC_RX_MSG_CMD,  RX_ACK_CURRENT);

    for (i = 0; i < rx_msg_length && i < MAX_MSG; i++) {
        data = hdmi_rd_reg(CEC0_BASE_ADDR + CEC_RX_MSG_0_HEADER +i);
        *msg = data;
        msg++;
    }
    *len = rx_msg_length;
    rx_status = hdmi_rd_reg(CEC0_BASE_ADDR+CEC_RX_MSG_STATUS);


    hdmi_wr_reg(CEC0_BASE_ADDR + CEC_RX_MSG_CMD,  RX_NO_OP);
#endif
#ifdef CONFIG_ARCH_MESON8

    if(RX_DONE != aocec_rd_reg(CEC_RX_MSG_STATUS)){
        aocec_wr_reg(CEC_RX_MSG_CMD,  RX_ACK_CURRENT);
        aocec_wr_reg(CEC_RX_MSG_CMD,  RX_NO_OP);
        return -1;
    }
    if(1 != aocec_rd_reg(CEC_RX_NUM_MSG)){
        aocec_wr_reg(CEC_RX_MSG_CMD,  RX_ACK_CURRENT);
        aocec_wr_reg(CEC_RX_MSG_CMD,  RX_NO_OP);
        return -1;
    }
    rx_msg_length = aocec_rd_reg(CEC_RX_MSG_LENGTH) + 1;

    aocec_wr_reg(CEC_RX_MSG_CMD,  RX_ACK_CURRENT);

    for (i = 0; i < rx_msg_length && i < MAX_MSG; i++) {
        data = aocec_rd_reg(CEC_RX_MSG_0_HEADER +i);
        *msg = data;
        msg++;
    }
    *len = rx_msg_length;
    rx_status = aocec_rd_reg(CEC_RX_MSG_STATUS);

    aocec_wr_reg(CEC_RX_MSG_CMD, RX_NO_OP);
    aml_write_reg32(P_AO_CEC_INTR_CLR, aml_read_reg32(P_AO_CEC_INTR_CLR) | (1 << 2));
#endif
    if(cec_msg_dbg_en == 1){
        pos = 0;
        pos += sprintf(msg_log_buf + pos, "CEC: rx msg len: %d   dat: ", rx_msg_length);
        for(n = 0; n < rx_msg_length; n++) {
            pos += sprintf(msg_log_buf + pos, "%02x ", msg_start[n]);
        }
        pos += sprintf(msg_log_buf + pos, "\n");
        msg_log_buf[pos] = '\0';
        printk("%s", msg_log_buf);
    }
    return rx_status;
}

void cec_isr_post_process(void)
{
    /* isr post process */
    while(cec_rx_msg_buf.rx_read_pos != cec_rx_msg_buf.rx_write_pos) {
        cec_handle_message(&(cec_rx_msg_buf.cec_rx_message[cec_rx_msg_buf.rx_read_pos]));
        (cec_rx_msg_buf.rx_read_pos == cec_rx_msg_buf.rx_buf_size - 1) ? (cec_rx_msg_buf.rx_read_pos = 0) : (cec_rx_msg_buf.rx_read_pos++);
    }
}

void cec_usr_cmd_post_process(void)
{
    cec_tx_message_list_t *p, *ptmp;
    /* usr command post process */
    list_for_each_entry_safe(p, ptmp, &cec_tx_msg_phead, list) {
        cec_ll_tx(p->msg, p->length);
        unregister_cec_tx_msg(p);
    }
}
#if 0
static int detect_tv_support_cec(unsigned addr)
{
    unsigned int ret = 0;
    unsigned char msg[1];
    msg[0] = (addr<<4) | 0x0;       // 0x0, TV's root address
    ret = cec_ll_tx_polling(msg, 1);
    cec_hw_reset();
    printk("CEC: tv%shave CEC feature\n", ret ? " " : " don\'t ");
    return (hdmitx_device->tv_cec_support = ret);
}
#endif
#ifdef CONFIG_ARCH_MESON8
//--------------------------------------------------------------------------
// AO CEC0 config
//--------------------------------------------------------------------------
static void ao_cec_init(void)
{
    unsigned long data32;
    // Assert SW reset AO_CEC
    data32  = 0;
    data32 |= 0 << 1;   // [2:1]    cntl_clk: 0=Disable clk (Power-off mode); 1=Enable gated clock (Normal mode); 2=Enable free-run clk (Debug mode).
    data32 |= 1 << 0;   // [0]      sw_reset: 1=Reset
    aml_write_reg32(P_AO_CEC_GEN_CNTL, data32);
    // Enable gated clock (Normal mode).
    aml_set_reg32_bits(P_AO_CEC_GEN_CNTL, 1, 1, 1);
    // Release SW reset
    aml_set_reg32_bits(P_AO_CEC_GEN_CNTL, 0, 0, 1);

    // Enable all AO_CEC interrupt sources
    aml_set_reg32_bits(P_AO_CEC_INTR_MASKN, 0x6, 0, 3);

    // Device 0 config
    aocec_wr_reg(CEC_LOGICAL_ADDR0, (0x1 << 4) | 0x4);
}
#endif

void cec_node_init(hdmitx_dev_t* hdmitx_device)
{
    struct vendor_info_data *vend_data = NULL;

    int i, bool = 0;
    const enum _cec_log_dev_addr_e player_dev[3] = {CEC_PLAYBACK_DEVICE_1_ADDR,
                                                    CEC_PLAYBACK_DEVICE_2_ADDR,
                                                    CEC_PLAYBACK_DEVICE_3_ADDR,
                                                   };

    unsigned long cec_phy_addr;
    cec_power_flag = 1;
    //if((hdmitx_device->cec_init_ready == 0) || (hdmitx_device->hpd_state == 0)) {      // If no connect, return directly
    if((cec_init_ready_flag == 0) || (hdmitx_device->hpd_state == 0)) {      // If no connect, return directly
        printk("CEC not ready,cec_init_ready:0x%x;hpd_state:0x%x\n",hdmitx_device->cec_init_ready, hdmitx_device->hpd_state);
        printk("cec_init_ready_flag:0x%x\n", cec_init_ready_flag);
        return;
    }
    else {
        printk("CEC node init\n");
    }

    if(!(hdmitx_device->cec_func_config & (1 << CEC_FUNC_MSAK)))
        return ;
    // If VSDB is not valid, wait
    if(hdmitx_device->hdmi_info.vsdb_phy_addr.valid == 0) {
        printk("hdmitx: cec: no valid cec physical address\n");
        return ;
    }
#if 1       //todo
    if(hdmitx_device->config_data.vend_data)
        vend_data = hdmitx_device->config_data.vend_data;
    if((vend_data) && (vend_data->cec_osd_string)) {
        i = strlen(vend_data->cec_osd_string);
        if(i > 14)
            vend_data->cec_osd_string[14] = '\0';   // OSD string length must be less than 14 bytes
        osd_name = vend_data->cec_osd_string;
    }
    if((vend_data) && (vend_data->vendor_id)) {
        vendor_id = (vend_data->vendor_id ) & 0xffffff;
    }
#endif
#ifdef CONFIG_ARCH_MESON6
    aml_set_reg32_bits(P_PERIPHS_PIN_MUX_1, 1, 25, 1);
    // Clear CEC Int. state and set CEC Int. mask
    aml_write_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_STAT_CLR, aml_read_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_STAT_CLR) | (1 << 23));    // Clear the interrupt
    aml_write_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_MASK, aml_read_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_MASK) | (1 << 23));            // Enable the hdmi cec interrupt

#endif
#ifdef CONFIG_ARCH_MESON8
#if 1           // Please match with H/W cec config
// GPIOAO_12
    aml_set_reg32_bits(P_AO_RTI_PULL_UP_REG, 0, 12, 1);       // disable AO_12 internal pull-up   //fe60002c
    aml_set_reg32_bits(P_AO_RTI_PIN_MUX_REG, 1, 17, 1);      // AO_CEC pinmux                    //fe600014
    ao_cec_init();
#else
// GPIOH_3
    aml_set_reg32_bits(P_PAD_PULL_UP_EN_REG1, 0, 19, 1);    // disable gpioh_3 internal pull-up
    aml_set_reg32_bits(P_PERIPHS_PIN_MUX_1, 1, 23, 1);      // gpioh_3 cec pinmux
#endif
    cec_arbit_bit_time_set(3, 0x118, 0);
    cec_arbit_bit_time_set(5, 0x000, 0);
    cec_arbit_bit_time_set(7, 0x2aa, 0);
#endif
    cec_phy_addr = (((hdmitx_device->hdmi_info.vsdb_phy_addr.a) & 0xf) << 12) |
                   (((hdmitx_device->hdmi_info.vsdb_phy_addr.b) & 0xf) << 8)  |
                   (((hdmitx_device->hdmi_info.vsdb_phy_addr.c) & 0xf) << 4)  |
                   (((hdmitx_device->hdmi_info.vsdb_phy_addr.d) & 0xf) << 0);


	for(i = 0; i < 3; i++){
	    printk("CEC: start poll dev\n");
		cec_polling_online_dev(player_dev[i], &bool);
		hdmitx_cec_dbg_print("CEC: player_dev[%d]:0x%x\n", i, player_dev[i]);
		//hdmitx_cec_dbg_print("CEC: end poll dev\n");
		if(bool == 0){  // 0 means that no any respond
            aml_write_reg32(P_AO_DEBUG_REG1, cec_phy_addr);
		    cec_global_info.cec_node_info[cec_global_info.my_node_index].power_status = TRANS_STANDBY_TO_ON;
            cec_global_info.my_node_index = player_dev[i];
            aml_write_reg32(P_AO_DEBUG_REG3, aml_read_reg32(P_AO_DEBUG_REG3) | (cec_global_info.my_node_index & 0xf));
            cec_global_info.cec_node_info[player_dev[i]].log_addr = player_dev[i];
            // Set Physical address
            cec_global_info.cec_node_info[player_dev[i]].phy_addr.phy_addr_4 = cec_phy_addr;

            cec_global_info.cec_node_info[player_dev[i]].specific_info.audio.sys_audio_mode = OFF;
            cec_global_info.cec_node_info[player_dev[i]].specific_info.audio.audio_status.audio_mute_status = OFF;
            cec_global_info.cec_node_info[player_dev[i]].specific_info.audio.audio_status.audio_volume_status = 0;

            cec_global_info.cec_node_info[player_dev[i]].cec_version = CEC_VERSION_14A;
            cec_global_info.cec_node_info[player_dev[i]].vendor_id = vendor_id;
            cec_global_info.cec_node_info[player_dev[i]].dev_type = cec_log_addr_to_dev_type(player_dev[i]);
            cec_global_info.cec_node_info[player_dev[i]].dev_type = cec_log_addr_to_dev_type(player_dev[i]);
            strcpy(cec_global_info.cec_node_info[player_dev[i]].osd_name, osd_name); //Max num: 14Bytes
#ifdef CONFIG_ARCH_MESON6
            hdmi_wr_reg(CEC0_BASE_ADDR+CEC_LOGICAL_ADDR0, (0x1 << 4) | player_dev[i]);
#endif
#ifdef CONFIG_ARCH_MESON8
            aocec_wr_reg(CEC_LOGICAL_ADDR0, (0x1 << 4) | player_dev[i]);
#endif
     		hdmitx_cec_dbg_print("CEC: Set logical address: %d\n", player_dev[i]);

            hdmitx_cec_dbg_print("aml_read_reg32(P_AO_DEBUG_REG0):0x%x\n" ,aml_read_reg32(P_AO_DEBUG_REG0));
        	if(cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_status == DEVICE_MENU_INACTIVE)
        	    break;
            msleep(100);
			cec_report_physical_address_smp();
            msleep(150);
            cec_device_vendor_id((cec_rx_message_t*)0);

            msleep(150);
            cec_imageview_on_smp();
            msleep(100);

            // here, we need to detect whether TV is supporting the CEC function
            // if not, jump out to save system time
            //if(!detect_tv_support_cec(player_dev[i])) {
            //    printk("HDMI CEC: No TV detected.\n");
            //    break;
            //}
            cec_get_menu_language_smp();
            msleep(350);

            cec_active_source_smp();
            msleep(120);

            cec_menu_status_smp(DEVICE_MENU_ACTIVE);
            msleep(100);

            cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_status = DEVICE_MENU_ACTIVE;
            cec_global_info.cec_node_info[cec_global_info.my_node_index].power_status = POWER_ON;
            break;
		}
	}
	if(bool == 1)
		hdmitx_cec_dbg_print("CEC: Can't get a valid logical address\n");
}

void cec_node_uninit(hdmitx_dev_t* hdmitx_device)
{
    if(!(hdmitx_device->cec_func_config & (1 << CEC_FUNC_MSAK)))
       return ;
    cec_global_info.cec_node_info[cec_global_info.my_node_index].power_status = TRANS_ON_TO_STANDBY;
    cec_power_flag = 0;
    hdmitx_cec_dbg_print("CEC: cec node uninit!\n");
    cec_global_info.cec_node_info[cec_global_info.my_node_index].power_status = POWER_STANDBY;
}

static int cec_task(void *data)
{
	extern void dump_hdmi_cec_reg(void);
    hdmitx_dev_t* hdmitx_device = (hdmitx_dev_t*) data;
    cec_init_flag = 1;

#ifdef CONFIG_HAS_EARLYSUSPEND
    hdmitx_cec_early_suspend_handler.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 20;
    hdmitx_cec_early_suspend_handler.suspend = hdmitx_cec_early_suspend;
    hdmitx_cec_early_suspend_handler.resume = hdmitx_cec_late_resume;
    hdmitx_cec_early_suspend_handler.param = hdmitx_device;

    register_early_suspend(&hdmitx_cec_early_suspend_handler);
#endif

    // Get logical address

    hdmitx_cec_dbg_print("CEC: CEC task process\n");
    if(hdmitx_device->cec_func_config & (1 << CEC_FUNC_MSAK)){
        msleep(10000);
#ifdef CONFIG_ARCH_MESON6
        cec_gpi_init();
#endif
		printk("cec_func_config:0x%x\n", hdmitx_device->cec_func_config);
        cec_node_init(hdmitx_device);
    }
    while (1) {
        msleep(5);
        cec_isr_post_process();
        //cec_usr_cmd_post_process();
    }

    return 0;
}

/***************************** cec low level code end *****************************/


/***************************** cec middle level code *****************************/

void register_cec_rx_msg(unsigned char *msg, unsigned char len )
{
    unsigned long flags;
    spin_lock_irqsave(&cec_input_key,flags);
    memset((void*)(&(cec_rx_msg_buf.cec_rx_message[cec_rx_msg_buf.rx_write_pos])), 0, sizeof(cec_rx_message_t));
    memcpy(cec_rx_msg_buf.cec_rx_message[cec_rx_msg_buf.rx_write_pos].content.buffer, msg, len);

    cec_rx_msg_buf.cec_rx_message[cec_rx_msg_buf.rx_write_pos].operand_num = len >= 2 ? len - 2 : 0;
    cec_rx_msg_buf.cec_rx_message[cec_rx_msg_buf.rx_write_pos].msg_length = len;

    cec_input_handle_message();

    (cec_rx_msg_buf.rx_write_pos == cec_rx_msg_buf.rx_buf_size - 1) ? (cec_rx_msg_buf.rx_write_pos = 0) : (cec_rx_msg_buf.rx_write_pos++);
    spin_unlock_irqrestore(&cec_input_key,flags);
}

void register_cec_tx_msg(unsigned char *msg, unsigned char len )
{
    cec_tx_message_list_t* cec_usr_message_list = kmalloc(sizeof(cec_tx_message_list_t), GFP_ATOMIC);

    if (cec_usr_message_list != NULL) {
        memset(cec_usr_message_list, 0, sizeof(cec_tx_message_list_t));
        memcpy(cec_usr_message_list->msg, msg, len);
        cec_usr_message_list->length = len;

        spin_lock_irqsave(&p_tx_list_lock, cec_tx_list_flags);
        list_add_tail(&cec_usr_message_list->list, &cec_tx_msg_phead);
        spin_unlock_irqrestore(&p_tx_list_lock, cec_tx_list_flags);

        tx_msg_cnt++;
    }
}
void cec_input_handle_message(void)
{
    unsigned char   opcode;

    opcode = cec_rx_msg_buf.cec_rx_message[cec_rx_msg_buf.rx_write_pos].content.msg.opcode;

    /* process key event messages from tv */
    if((hdmitx_device->cec_func_config & (1 << CEC_FUNC_MSAK)) && cec_power_flag)
    {
        switch (opcode) {
        case CEC_OC_USER_CONTROL_PRESSED:
            // check valid msg
            {
                unsigned char opernum;
                unsigned char follower;
                opernum  = cec_rx_msg_buf.cec_rx_message[cec_rx_msg_buf.rx_write_pos].operand_num;
                follower = cec_rx_msg_buf.cec_rx_message[cec_rx_msg_buf.rx_write_pos].content.msg.header & 0x0f;
                if(opernum != 1 || follower == 0xf) break;
            }
            cec_user_control_pressed_irq();
            break;
        default:
            break;
        }
    }
}

void unregister_cec_tx_msg(cec_tx_message_list_t* cec_tx_message_list)
{

    if (cec_tx_message_list != NULL) {
        list_del(&cec_tx_message_list->list);
        kfree(cec_tx_message_list);
        cec_tx_message_list = NULL;

        if (tx_msg_cnt > 0) tx_msg_cnt--;
    }
}

void cec_hw_reset(void)
{
#ifdef CONFIG_ARCH_MESON6
    aml_write_reg32(P_HDMI_CTRL_PORT, aml_read_reg32(P_HDMI_CTRL_PORT)|(1<<16));
    hdmi_wr_reg(OTHER_BASE_ADDR+HDMI_OTHER_CTRL0, 0xc); //[3]cec_creg_sw_rst [2]cec_sys_sw_rst
    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_TX_CLEAR_BUF, 0x1);
    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_RX_CLEAR_BUF, 0x1);

    //mdelay(10);
    {//Delay some time
    	int i = 10;
    	while(i--);
    }
    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_TX_CLEAR_BUF, 0x0);
    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_RX_CLEAR_BUF, 0x0);
    hdmi_wr_reg(OTHER_BASE_ADDR+HDMI_OTHER_CTRL0, 0x0);
    aml_write_reg32(P_HDMI_CTRL_PORT, aml_read_reg32(P_HDMI_CTRL_PORT)&(~(1<<16)));
    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_CLOCK_DIV_H, 0x00 );
    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_CLOCK_DIV_L, 0xf0 );

    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_LOGICAL_ADDR0, (0x1 << 4) | cec_global_info.my_node_index);
#endif
#ifdef CONFIG_ARCH_MESON8

    //unsigned long data32;
    // Assert SW reset AO_CEC
    //data32  = 0;
    //data32 |= 0 << 1;   // [2:1]    cntl_clk: 0=Disable clk (Power-off mode); 1=Enable gated clock (Normal mode); 2=Enable free-run clk (Debug mode).
    //data32 |= 1 << 0;   // [0]      sw_reset: 1=Reset
    aml_write_reg32(P_AO_CEC_GEN_CNTL, 0x1);
    // Enable gated clock (Normal mode).
    aml_set_reg32_bits(P_AO_CEC_GEN_CNTL, 1, 1, 1);
    // Release SW reset
    aml_set_reg32_bits(P_AO_CEC_GEN_CNTL, 0, 0, 1);

    // Enable all AO_CEC interrupt sources
    aml_set_reg32_bits(P_AO_CEC_INTR_MASKN, 0x6, 0, 3);

    aocec_wr_reg(CEC_LOGICAL_ADDR0, (0x1 << 4) | cec_global_info.my_node_index);

    //Cec arbitration 3/5/7 bit time set.
    cec_arbit_bit_time_set(3, 0x118, 0);
    cec_arbit_bit_time_set(5, 0x000, 0);
    cec_arbit_bit_time_set(7, 0x2aa, 0);

    printk("hw reset :logical addr:0x%x\n", aocec_rd_reg(CEC_LOGICAL_ADDR0));

#endif
}

unsigned char check_cec_msg_valid(const cec_rx_message_t* pcec_message)
{
    unsigned char rt = 0;
    unsigned char opcode;
    unsigned char opernum;
    unsigned char follower;
    if (!pcec_message)
        return rt;

    opcode = pcec_message->content.msg.opcode;
    opernum = pcec_message->operand_num;

    switch (opcode) {
        case CEC_OC_VENDOR_REMOTE_BUTTON_UP:
        case CEC_OC_STANDBY:
        case CEC_OC_RECORD_OFF:
        case CEC_OC_RECORD_TV_SCREEN:
        case CEC_OC_TUNER_STEP_DECREMENT:
        case CEC_OC_TUNER_STEP_INCREMENT:
        case CEC_OC_GIVE_AUDIO_STATUS:
        case CEC_OC_GIVE_SYSTEM_AUDIO_MODE_STATUS:
        case CEC_OC_USER_CONTROL_RELEASED:
        case CEC_OC_GIVE_OSD_NAME:
        case CEC_OC_GIVE_PHYSICAL_ADDRESS:
        case CEC_OC_GET_CEC_VERSION:
        case CEC_OC_GET_MENU_LANGUAGE:
        case CEC_OC_GIVE_DEVICE_VENDOR_ID:
        case CEC_OC_GIVE_DEVICE_POWER_STATUS:
        case CEC_OC_TEXT_VIEW_ON:
        case CEC_OC_IMAGE_VIEW_ON:
        case CEC_OC_ABORT_MESSAGE:
        case CEC_OC_REQUEST_ACTIVE_SOURCE:
            if ( opernum == 0)  rt = 1;
            break;
        case CEC_OC_SET_SYSTEM_AUDIO_MODE:
        case CEC_OC_RECORD_STATUS:
        case CEC_OC_DECK_CONTROL:
        case CEC_OC_DECK_STATUS:
        case CEC_OC_GIVE_DECK_STATUS:
        case CEC_OC_GIVE_TUNER_DEVICE_STATUS:
        case CEC_OC_PLAY:
        case CEC_OC_MENU_REQUEST:
        case CEC_OC_MENU_STATUS:
        case CEC_OC_REPORT_AUDIO_STATUS:
        case CEC_OC_TIMER_CLEARED_STATUS:
        case CEC_OC_SYSTEM_AUDIO_MODE_STATUS:
        case CEC_OC_USER_CONTROL_PRESSED:
        case CEC_OC_CEC_VERSION:
        case CEC_OC_REPORT_POWER_STATUS:
        case CEC_OC_SET_AUDIO_RATE:
            if ( opernum == 1)  rt = 1;
            break;
        case CEC_OC_INACTIVE_SOURCE:
        case CEC_OC_SYSTEM_AUDIO_MODE_REQUEST:
        case CEC_OC_FEATURE_ABORT:
        case CEC_OC_ACTIVE_SOURCE:
        case CEC_OC_ROUTING_INFORMATION:
        case CEC_OC_SET_STREAM_PATH:
            if (opernum == 2) rt = 1;
            break;
        case CEC_OC_REPORT_PHYSICAL_ADDRESS:
        case CEC_OC_SET_MENU_LANGUAGE:
        case CEC_OC_DEVICE_VENDOR_ID:
            if (opernum == 3) rt = 1;
            break;
        case CEC_OC_ROUTING_CHANGE:
        case CEC_OC_SELECT_ANALOGUE_SERVICE:
            if (opernum == 4) rt = 1;
            break;
        case CEC_OC_VENDOR_COMMAND_WITH_ID:
            if ((opernum > 3)&&(opernum < 15))  rt = 1;
            break;
        case CEC_OC_VENDOR_REMOTE_BUTTON_DOWN:
            if (opernum < 15)  rt = 1;
            break;
        case CEC_OC_SELECT_DIGITAL_SERVICE:
            if (opernum == 7) rt = 1;
            break;
        case CEC_OC_SET_ANALOGUE_TIMER:
        case CEC_OC_CLEAR_ANALOGUE_TIMER:
            if (opernum == 11) rt = 1;
            break;
        case CEC_OC_SET_DIGITAL_TIMER:
        case CEC_OC_CLEAR_DIGITAL_TIMER:
            if (opernum == 14) rt = 1;
            break;
        case CEC_OC_TIMER_STATUS:
            if ((opernum == 1 || opernum == 3)) rt = 1;
            break;
        case CEC_OC_TUNER_DEVICE_STATUS:
            if ((opernum == 5 || opernum == 8)) rt = 1;
            break;
        case CEC_OC_RECORD_ON:
            if (opernum > 0 && opernum < 9)  rt = 1;
            break;
        case CEC_OC_CLEAR_EXTERNAL_TIMER:
        case CEC_OC_SET_EXTERNAL_TIMER:
            if ((opernum == 9 || opernum == 10)) rt = 1;
            break;
        case CEC_OC_SET_TIMER_PROGRAM_TITLE:
        case CEC_OC_SET_OSD_NAME:
            if (opernum > 0 && opernum < 15) rt = 1;
            break;
        case CEC_OC_SET_OSD_STRING:
            if (opernum > 1 && opernum < 15) rt = 1;
            break;
        case CEC_OC_VENDOR_COMMAND:
            if (opernum < 15)   rt = 1;
            break;
        default:
            rt = 1;
            break;
    }

 // for CTS12.2
    follower = pcec_message->content.msg.header & 0x0f;
    switch (opcode) {
        case CEC_OC_ACTIVE_SOURCE:
        case CEC_OC_REQUEST_ACTIVE_SOURCE:
        case CEC_OC_ROUTING_CHANGE:
        case CEC_OC_ROUTING_INFORMATION:
        case CEC_OC_SET_STREAM_PATH:
        case CEC_OC_REPORT_PHYSICAL_ADDRESS:
        case CEC_OC_SET_MENU_LANGUAGE:
        case CEC_OC_DEVICE_VENDOR_ID:
            // broadcast only
            if(follower != 0xf) rt = 0;
            break;

        case CEC_OC_IMAGE_VIEW_ON:
        case CEC_OC_TEXT_VIEW_ON:
        case CEC_OC_INACTIVE_SOURCE:
        case CEC_OC_RECORD_OFF:
        case CEC_OC_RECORD_ON:
        case CEC_OC_RECORD_STATUS:
        case CEC_OC_RECORD_TV_SCREEN:
        case CEC_OC_CLEAR_ANALOGUE_TIMER:
        case CEC_OC_CLEAR_DIGITAL_TIMER:
        case CEC_OC_CLEAR_EXTERNAL_TIMER:
        case CEC_OC_SET_ANALOGUE_TIMER:
        case CEC_OC_SET_DIGITAL_TIMER:
        case CEC_OC_SET_EXTERNAL_TIMER:
        case CEC_OC_SET_TIMER_PROGRAM_TITLE:
        case CEC_OC_TIMER_CLEARED_STATUS:
        case CEC_OC_TIMER_STATUS:
        case CEC_OC_CEC_VERSION:
        case CEC_OC_GET_CEC_VERSION:
        case CEC_OC_GIVE_PHYSICAL_ADDRESS:
        case CEC_OC_GET_MENU_LANGUAGE:
        case CEC_OC_DECK_CONTROL:
        case CEC_OC_DECK_STATUS:
        case CEC_OC_GIVE_DECK_STATUS:
        case CEC_OC_PLAY:
        case CEC_OC_GIVE_TUNER_DEVICE_STATUS:
        case CEC_OC_SELECT_ANALOGUE_SERVICE:
        case CEC_OC_SELECT_DIGITAL_SERVICE:
        case CEC_OC_TUNER_DEVICE_STATUS:
        case CEC_OC_TUNER_STEP_DECREMENT:
        case CEC_OC_TUNER_STEP_INCREMENT:
        case CEC_OC_GIVE_DEVICE_VENDOR_ID:
        case CEC_OC_VENDOR_COMMAND:
        case CEC_OC_SET_OSD_STRING:
        case CEC_OC_GIVE_OSD_NAME:
        case CEC_OC_SET_OSD_NAME:
        case CEC_OC_MENU_REQUEST:
        case CEC_OC_MENU_STATUS:
        case CEC_OC_USER_CONTROL_PRESSED:
        case CEC_OC_USER_CONTROL_RELEASED:
        case CEC_OC_GIVE_DEVICE_POWER_STATUS:
        case CEC_OC_REPORT_POWER_STATUS:
        case CEC_OC_FEATURE_ABORT:
        case CEC_OC_ABORT_MESSAGE:
        case CEC_OC_GIVE_AUDIO_STATUS:
        case CEC_OC_GIVE_SYSTEM_AUDIO_MODE_STATUS:
        case CEC_OC_REPORT_AUDIO_STATUS:
        case CEC_OC_SYSTEM_AUDIO_MODE_REQUEST:
        case CEC_OC_SYSTEM_AUDIO_MODE_STATUS:
        case CEC_OC_SET_AUDIO_RATE:
            // directly addressed only
            if(follower == 0xf) rt = 0;
            break;

        case CEC_OC_STANDBY:
        case CEC_OC_VENDOR_COMMAND_WITH_ID:
        case CEC_OC_VENDOR_REMOTE_BUTTON_DOWN:
        case CEC_OC_VENDOR_REMOTE_BUTTON_UP:
        case CEC_OC_SET_SYSTEM_AUDIO_MODE:
            // both broadcast and directly addressed
            break;

        default:
            break;
    }

    if ((rt == 0) & (opcode != 0)){
        hdmirx_cec_dbg_print("CEC: opcode & opernum not match: %x, %x\n", opcode, opernum);
    }
    return rt;
}

void cec_disable_irq(void)
{
#ifdef CONFIG_ARCH_MESON6
    //Disable irq
    aml_write_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_MASK, aml_read_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_MASK) & ~(1 << 23));
    //Clear the interrupt
    aml_write_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_STAT_CLR, aml_read_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_STAT_CLR) | (1 << 23));
#endif
#ifdef CONFIG_ARCH_MESON8
    // disable all AO_CEC interrupt sources
    aml_set_reg32_bits(P_AO_CEC_INTR_MASKN, 0x0, 0, 3);
    printk("disable:int mask:0x%x\n", aml_read_reg32(P_AO_CEC_INTR_MASKN));
#endif
}
void cec_enable_irq(void)
{
#ifdef CONFIG_ARCH_MESON6
    aml_write_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_STAT_CLR, aml_read_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_STAT_CLR) & ~(1 << 23));
    aml_write_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_MASK, aml_read_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_MASK) | (1 << 23));
#endif
#ifdef CONFIG_ARCH_MESON8
    aml_set_reg32_bits(P_AO_CEC_INTR_MASKN, 0x6, 0, 3);
    printk("enable:int mask:0x%x\n", aml_read_reg32(P_AO_CEC_INTR_MASKN));
#endif
}

static irqreturn_t cec_isr_handler(int irq, void *dev_instance)
{
    unsigned char rx_msg[MAX_MSG], rx_len;
    unsigned int intr_stat = 0;

    //cec_disable_irq();

#ifdef CONFIG_ARCH_MESON8
    intr_stat = aml_read_reg32(P_AO_CEC_INTR_STAT);
    printk("aocec irq %x\n", intr_stat);

    if(intr_stat & (1<<1)) { // aocec tx intr
        tx_irq_handle();
        //cec_enable_irq();
        return IRQ_HANDLED;
    }
#endif
    if((-1) == cec_ll_rx(rx_msg, &rx_len)){
        //cec_enable_irq();
        return IRQ_HANDLED;
    }

#if CEC_WQ
    queue_work(cec_wq, &cec_wq_func);
#else
    schedule_work(&cec_wq_func);
#endif

    register_cec_rx_msg(rx_msg, rx_len);

    //cec_enable_irq();

    return IRQ_HANDLED;
}

unsigned short cec_log_addr_to_dev_type(unsigned char log_addr)
{
    unsigned short us = CEC_UNREGISTERED_DEVICE_TYPE;
    if ((1 << log_addr) & CEC_DISPLAY_DEVICE) {
        us = CEC_DISPLAY_DEVICE_TYPE;
    } else if ((1 << log_addr) & CEC_RECORDING_DEVICE) {
        us = CEC_RECORDING_DEVICE_TYPE;
    } else if ((1 << log_addr) & CEC_PLAYBACK_DEVICE) {
        us = CEC_PLAYBACK_DEVICE_TYPE;
    } else if ((1 << log_addr) & CEC_TUNER_DEVICE) {
        us = CEC_TUNER_DEVICE_TYPE;
    } else if ((1 << log_addr) & CEC_AUDIO_SYSTEM_DEVICE) {
        us = CEC_AUDIO_SYSTEM_DEVICE_TYPE;
    }

    return us;
}
// -------------- command from cec devices ---------------------

void cec_polling_online_dev(int log_addr, int *bool)
{
    unsigned long r;
    unsigned char msg[1];

    cec_global_info.my_node_index = log_addr;
    msg[0] = (log_addr<<4) | log_addr;
#ifdef CONFIG_ARCH_MESON6
    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_LOGICAL_ADDR0, (0x1 << 4) | 0xf);
    r = cec_ll_tx(msg, 1);
#endif
#ifdef CONFIG_ARCH_MESON8
    aocec_wr_reg(CEC_LOGICAL_ADDR0, (0x1 << 4) | 0xf);
    hdmitx_cec_dbg_print("CEC_LOGICAL_ADDR0:0x%lx\n",aocec_rd_reg(CEC_LOGICAL_ADDR0));
    r = cec_ll_tx_polling(msg, 1);
    cec_hw_reset();

#endif

    if (r == 0) {
        *bool = 0;
    }else{
        memset(&(cec_global_info.cec_node_info[log_addr]), 0, sizeof(cec_node_info_t));
        cec_global_info.cec_node_info[log_addr].dev_type = cec_log_addr_to_dev_type(log_addr);
    	  *bool = 1;
    }
    if(*bool == 0) {
#ifdef CONFIG_ARCH_MESON6
        hdmi_wr_reg(CEC0_BASE_ADDR+CEC_LOGICAL_ADDR0, (0x1 << 4) | log_addr);
#endif
#ifdef CONFIG_ARCH_MESON8
        aocec_wr_reg(CEC_LOGICAL_ADDR0, (0x1 << 4) | log_addr);
#endif
    }
    hdmitx_cec_dbg_print("CEC: poll online logic device: 0x%x BOOL: %d\n", log_addr, *bool);

}


//***************************************************************
void cec_device_vendor_id(cec_rx_message_t* pcec_message)
{
    unsigned char index = cec_global_info.my_node_index;
    unsigned char msg[5];

    msg[0] = ((index & 0xf) << 4) | CEC_BROADCAST_ADDR;
    msg[1] = CEC_OC_DEVICE_VENDOR_ID;
    msg[2] = (vendor_id >> 16) & 0xff;
    msg[3] = (vendor_id >> 8) & 0xff;
    msg[4] = (vendor_id >> 0) & 0xff;

    cec_ll_tx(msg, 5);
}

void cec_report_power_status(cec_rx_message_t* pcec_message)
{
    unsigned char index = cec_global_info.my_node_index;
    unsigned char msg[3];

    msg[0] = ((index & 0xf) << 4) | CEC_TV_ADDR;
    msg[1] = CEC_OC_REPORT_POWER_STATUS;
    msg[2] = cec_global_info.cec_node_info[index].power_status;
    cec_ll_tx(msg, 3);

}

void cec_feature_abort(cec_rx_message_t* pcec_message)
{
    unsigned char index = cec_global_info.my_node_index;
    unsigned char opcode = pcec_message->content.msg.opcode;
    unsigned char src_log_addr = (pcec_message->content.msg.header >> 4 )&0xf;
    unsigned char dst_log_addr = pcec_message->content.msg.header & 0xf;
    if(dst_log_addr != 0xf){
        unsigned char msg[4];

        msg[0] = ((index & 0xf) << 4) | src_log_addr;
        msg[1] = CEC_OC_FEATURE_ABORT;
        msg[2] = opcode;
        msg[3] = CEC_UNRECONIZED_OPCODE;

        cec_ll_tx(msg, 4);
    }
}

void cec_report_version(cec_rx_message_t* pcec_message)
{
    ;//todo
}


void cec_report_physical_address_smp(void)
{
    unsigned char msg[5];
    unsigned char index = cec_global_info.my_node_index;
    unsigned char phy_addr_ab = (aml_read_reg32(P_AO_DEBUG_REG1) >> 8) & 0xff;
    unsigned char phy_addr_cd = aml_read_reg32(P_AO_DEBUG_REG1) & 0xff;

    msg[0] = ((index & 0xf) << 4) | CEC_BROADCAST_ADDR;
    msg[1] = CEC_OC_REPORT_PHYSICAL_ADDRESS;
    msg[2] = phy_addr_ab;
    msg[3] = phy_addr_cd;
    msg[4] = cec_global_info.cec_node_info[index].dev_type;

    cec_ll_tx(msg, 5);

}

void cec_imageview_on_smp(void)
{
    unsigned char msg[2];
    unsigned char index = cec_global_info.my_node_index;

    if(hdmitx_device->cec_func_config & (1 << CEC_FUNC_MSAK)) {
        if(hdmitx_device->cec_func_config & (1 << ONE_TOUCH_PLAY_MASK)) {
            msg[0] = ((index & 0xf) << 4) | CEC_TV_ADDR;
            msg[1] = CEC_OC_IMAGE_VIEW_ON;
            cec_ll_tx(msg, 2);
        }
    }
}

void cec_get_menu_language_smp(void)
{
    unsigned char msg[2];
    unsigned char index = cec_global_info.my_node_index;

    msg[0] = ((index & 0xf) << 4) | CEC_TV_ADDR;
    msg[1] = CEC_OC_GET_MENU_LANGUAGE;

    cec_ll_tx(msg, 2);

}

void cec_menu_status(cec_rx_message_t* pcec_message)
{
    unsigned char msg[3];
    unsigned char index = cec_global_info.my_node_index;
    unsigned char src_log_addr = (pcec_message->content.msg.header >> 4 )&0xf;

     if(0xf != src_log_addr) {
        msg[0] = ((index & 0xf) << 4) | src_log_addr;
        msg[1] = CEC_OC_MENU_STATUS;
        msg[2] = cec_global_info.cec_node_info[index].menu_status;
        cec_ll_tx(msg, 3);
    }
}

void cec_menu_status_smp(cec_device_menu_state_e status)
{
    unsigned char msg[3];
    unsigned char index = cec_global_info.my_node_index;

    msg[0] = ((index & 0xf) << 4) | CEC_TV_ADDR;
    msg[1] = CEC_OC_MENU_STATUS;
    if(status == DEVICE_MENU_ACTIVE){
        msg[2] = DEVICE_MENU_ACTIVE;
        cec_global_info.cec_node_info[index].menu_status = DEVICE_MENU_ACTIVE;
    }else{
        msg[2] = DEVICE_MENU_INACTIVE;
        cec_global_info.cec_node_info[index].menu_status = DEVICE_MENU_INACTIVE;
    }
    cec_ll_tx(msg, 3);
}

void cec_menu_status_smp_irq(cec_rx_message_t* pcec_message)
{
    unsigned char index = cec_global_info.my_node_index;

    if(1 == pcec_message->content.msg.operands[0]){
        cec_global_info.cec_node_info[index].menu_status = DEVICE_MENU_INACTIVE;
    }else if(0 == pcec_message->content.msg.operands[0]){
        cec_global_info.cec_node_info[index].menu_status = DEVICE_MENU_ACTIVE;
    }
}

void cec_active_source_rx(cec_rx_message_t* pcec_message)
{
    unsigned int phy_addr_active;

    phy_addr_active = (unsigned int)((pcec_message->content.msg.operands[0] << 8) |
                                    (pcec_message->content.msg.operands[1] << 0));

	if(phy_addr_active == (aml_read_reg32(P_AO_DEBUG_REG1) & 0xffff)){
	    cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_status = DEVICE_MENU_ACTIVE;
	}else{
	    cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_status = DEVICE_MENU_INACTIVE;
	}
}

void cec_active_source_smp(void)
{
    unsigned char msg[4];
    unsigned char index = cec_global_info.my_node_index;
    unsigned char phy_addr_ab = (aml_read_reg32(P_AO_DEBUG_REG1) >> 8) & 0xff;
    unsigned char phy_addr_cd = aml_read_reg32(P_AO_DEBUG_REG1) & 0xff;

    if(hdmitx_device->cec_func_config & (1 << CEC_FUNC_MSAK)) {
        if(hdmitx_device->cec_func_config & (1 << ONE_TOUCH_PLAY_MASK)) {
            msg[0] = ((index & 0xf) << 4) | CEC_BROADCAST_ADDR;
            msg[1] = CEC_OC_ACTIVE_SOURCE;
            msg[2] = phy_addr_ab;
            msg[3] = phy_addr_cd;
            cec_ll_tx(msg, 4);
        }
    }
    cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_status = DEVICE_MENU_ACTIVE;
}
void cec_active_source(cec_rx_message_t* pcec_message)
{
    unsigned char msg[4];
    unsigned char index = cec_global_info.my_node_index;
    unsigned char phy_addr_ab = (aml_read_reg32(P_AO_DEBUG_REG1) >> 8) & 0xff;
    unsigned char phy_addr_cd = aml_read_reg32(P_AO_DEBUG_REG1) & 0xff;

    msg[0] = ((index & 0xf) << 4) | CEC_BROADCAST_ADDR;
    msg[1] = CEC_OC_ACTIVE_SOURCE;
    msg[2] = phy_addr_ab;
    msg[3] = phy_addr_cd;
    cec_ll_tx(msg, 4);
    cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_status = DEVICE_MENU_ACTIVE;
}


void cec_set_stream_path(cec_rx_message_t* pcec_message)
{
    unsigned int phy_addr_active;

    phy_addr_active = (unsigned int)((pcec_message->content.msg.operands[0] << 8) |
                                    (pcec_message->content.msg.operands[1] << 0));

	if(phy_addr_active == (aml_read_reg32(P_AO_DEBUG_REG1) & 0xffff)){
	    cec_active_source_smp();
	    cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_status = DEVICE_MENU_ACTIVE;
	}else{
	    cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_status = DEVICE_MENU_INACTIVE;
	}
}
void cec_set_system_audio_mode(void)
{
    unsigned char index = cec_global_info.my_node_index;

    MSG_P1( index, CEC_TV_ADDR,
            CEC_OC_SET_SYSTEM_AUDIO_MODE,
            cec_global_info.cec_node_info[index].specific_info.audio.sys_audio_mode
            );

    cec_ll_tx(gbl_msg, 3);
    if(cec_global_info.cec_node_info[index].specific_info.audio.sys_audio_mode == ON)
        cec_global_info.cec_node_info[index].specific_info.audio.sys_audio_mode = OFF;
    else
        cec_global_info.cec_node_info[index].specific_info.audio.sys_audio_mode = ON;
}

void cec_system_audio_mode_request(void)
{
    unsigned char index = cec_global_info.my_node_index;
    unsigned char phy_addr_ab = (aml_read_reg32(P_AO_DEBUG_REG1) >> 8) & 0xff;
    unsigned char phy_addr_cd = aml_read_reg32(P_AO_DEBUG_REG1) & 0xff;

    if(cec_global_info.cec_node_info[index].specific_info.audio.sys_audio_mode == OFF){
        MSG_P2( index, CEC_AUDIO_SYSTEM_ADDR,//CEC_TV_ADDR,
                CEC_OC_SYSTEM_AUDIO_MODE_REQUEST,
                phy_addr_ab,
                phy_addr_cd
                );
        cec_ll_tx(gbl_msg, 4);
        cec_global_info.cec_node_info[index].specific_info.audio.sys_audio_mode = ON;
    }
    else{
        MSG_P0( index, CEC_AUDIO_SYSTEM_ADDR,//CEC_TV_ADDR,
                CEC_OC_SYSTEM_AUDIO_MODE_REQUEST
                );
        cec_ll_tx(gbl_msg, 2);
        cec_global_info.cec_node_info[index].specific_info.audio.sys_audio_mode = OFF;
    }
}

void cec_report_audio_status(void)
{
    unsigned char index = cec_global_info.my_node_index;

    MSG_P1( index, CEC_TV_ADDR,
            CEC_OC_REPORT_AUDIO_STATUS,
            cec_global_info.cec_node_info[index].specific_info.audio.audio_status.audio_mute_status | \
            cec_global_info.cec_node_info[index].specific_info.audio.audio_status.audio_volume_status
            );

    cec_ll_tx(gbl_msg, 3);
}
void cec_request_active_source(cec_rx_message_t* pcec_message)
{
    cec_set_stream_path(pcec_message);
}

void cec_set_imageview_on_irq(void)
{
    unsigned char index = cec_global_info.my_node_index;
    unsigned char msg[2];

    msg[0] = ((index & 0xf) << 4) | CEC_TV_ADDR;
    msg[1] = CEC_OC_IMAGE_VIEW_ON;

    cec_ll_tx(msg, 2);
}

void cec_inactive_source(void)
{
    unsigned char index = cec_global_info.my_node_index;
    unsigned char msg[4];
    unsigned char phy_addr_ab = (aml_read_reg32(P_AO_DEBUG_REG1) >> 8) & 0xff;
    unsigned char phy_addr_cd = aml_read_reg32(P_AO_DEBUG_REG1) & 0xff;

    msg[0] = ((index & 0xf) << 4) | CEC_TV_ADDR;
    msg[1] = CEC_OC_INACTIVE_SOURCE;
	msg[2] = phy_addr_ab;
	msg[3] = phy_addr_cd;

    cec_ll_tx(msg, 4);
    cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_status = DEVICE_MENU_INACTIVE;
}

void cec_inactive_source_rx(cec_rx_message_t* pcec_message)
{
    cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_status = DEVICE_MENU_INACTIVE;
}

void cec_get_version(cec_rx_message_t* pcec_message)
{
    unsigned char dest_log_addr = pcec_message->content.msg.header&0xf;
    unsigned char index = cec_global_info.my_node_index;
    unsigned char msg[3];

    if (0xf != dest_log_addr) {
        msg[0] = ((index & 0xf) << 4) | CEC_TV_ADDR;
        msg[1] = CEC_OC_CEC_VERSION;
        msg[2] = CEC_VERSION_14A;
        cec_ll_tx(msg, 3);
    }
}

void cec_give_deck_status(cec_rx_message_t* pcec_message)
{
    unsigned char index = cec_global_info.my_node_index;
    unsigned char msg[3];

    msg[0] = ((index & 0xf) << 4) | CEC_TV_ADDR;
    msg[1] = CEC_OC_DECK_STATUS;
    msg[2] = 0x1a;
    cec_ll_tx(msg, 3);
}


void cec_deck_status(cec_rx_message_t* pcec_message)
{
    unsigned char index = cec_global_info.my_node_index;

    if (cec_global_info.dev_mask & (1 << index)) {
        cec_global_info.cec_node_info[index].specific_info.playback.deck_info = pcec_message->content.msg.operands[0];
        cec_global_info.cec_node_info[index].real_info_mask |= INFO_MASK_DECK_INfO;
        hdmirx_cec_dbg_print("cec_deck_status: %x\n", cec_global_info.cec_node_info[index].specific_info.playback.deck_info);
    }
}

// STANDBY: long press our remote control, send STANDBY to TV
void cec_set_standby(void)
{
    unsigned char index = cec_global_info.my_node_index;
    unsigned char msg[2];
    msg[0] = ((index & 0xf) << 4) | CEC_BROADCAST_ADDR;
    msg[1] = CEC_OC_STANDBY;
    if(hdmitx_device->cec_func_config & (1 << CEC_FUNC_MSAK)) {
        if(hdmitx_device->cec_func_config & (1 << ONE_TOUCH_STANDBY_MASK)) {
			cec_ll_tx(msg, 2);
		}
	}
}

void cec_set_osd_name(cec_rx_message_t* pcec_message)
{
    unsigned char index = cec_global_info.my_node_index;
	unsigned char osd_len = strlen(cec_global_info.cec_node_info[index].osd_name);
    unsigned char src_log_addr = (pcec_message->content.msg.header >> 4 )&0xf;
    unsigned char msg[16];

    if(0xf != src_log_addr) {
        msg[0] = ((index & 0xf) << 4) | src_log_addr;
        msg[1] = CEC_OC_SET_OSD_NAME;
        memcpy(&msg[2], cec_global_info.cec_node_info[index].osd_name, osd_len);

        cec_ll_tx(msg, 2 + osd_len);
    }
}

void cec_set_osd_name_init(void)
{
    unsigned char index = cec_global_info.my_node_index;
	unsigned char osd_len = strlen(cec_global_info.cec_node_info[index].osd_name);
    unsigned char msg[16];

    msg[0] = ((index & 0xf) << 4) | 0;
    msg[1] = CEC_OC_SET_OSD_NAME;
    memcpy(&msg[2], cec_global_info.cec_node_info[index].osd_name, osd_len);

    cec_ll_tx(msg, 2 + osd_len);
}

void cec_vendor_cmd_with_id(cec_rx_message_t* pcec_message)
{
    ;//todo
}


void cec_set_menu_language(cec_rx_message_t* pcec_message)
{
    unsigned char index = cec_global_info.my_node_index;
    unsigned char src_log_addr = (pcec_message->content.msg.header >> 4 )&0xf;

    if(0x0 == src_log_addr) {
        cec_global_info.cec_node_info[index].menu_lang = (int)((pcec_message->content.msg.operands[0] << 16)  |
                                                               (pcec_message->content.msg.operands[1] <<  8)  |
                                                               (pcec_message->content.msg.operands[2]));

        switch_set_state(&lang_dev, cec_global_info.cec_node_info[index].menu_lang);
        cec_global_info.cec_node_info[index].real_info_mask |= INFO_MASK_MENU_LANGUAGE;
        hdmirx_cec_dbg_print("cec_set_menu_language:%c.%c.%c\n", (cec_global_info.cec_node_info[index].menu_lang >>16) & 0xff,
                                                                 (cec_global_info.cec_node_info[index].menu_lang >> 8) & 0xff,
                                                                 (cec_global_info.cec_node_info[index].menu_lang >> 0) & 0xff);
    }
}

void cec_handle_message(cec_rx_message_t* pcec_message)
{
    unsigned char	brdcst, opcode;
    unsigned char	initiator, follower;
    unsigned char   operand_num;
    unsigned char   msg_length;

    /* parse message */
    if ((!pcec_message) || (check_cec_msg_valid(pcec_message) == 0))
        return;

    initiator	= pcec_message->content.msg.header >> 4;
    follower	= pcec_message->content.msg.header & 0x0f;
    opcode		= pcec_message->content.msg.opcode;
    operand_num = pcec_message->operand_num;
    brdcst      = (follower == 0x0f);
    msg_length  = pcec_message->msg_length;

    if(0 == pcec_message->content.msg.header)
        return;

    /* process messages from tv polling and cec devices */
    if(CEC_OC_GIVE_OSD_NAME == opcode)
        cec_set_osd_name(pcec_message);
    if((hdmitx_device->cec_func_config & (1 << CEC_FUNC_MSAK)) && cec_power_flag)
    {

        switch (opcode) {
        case CEC_OC_ACTIVE_SOURCE:
            cec_active_source_rx(pcec_message);
            break;
        case CEC_OC_INACTIVE_SOURCE:
            break;
        case CEC_OC_CEC_VERSION:
            break;
        case CEC_OC_DECK_STATUS:
            break;
        case CEC_OC_DEVICE_VENDOR_ID:
            break;
        case CEC_OC_FEATURE_ABORT:
            break;
        case CEC_OC_GET_CEC_VERSION:
            cec_get_version(pcec_message);
            break;
        case CEC_OC_GIVE_DECK_STATUS:
            cec_give_deck_status(pcec_message);
            break;
        case CEC_OC_MENU_STATUS:
            cec_menu_status_smp_irq(pcec_message);
            break;
        case CEC_OC_REPORT_PHYSICAL_ADDRESS:
            break;
        case CEC_OC_REPORT_POWER_STATUS:
            break;
        case CEC_OC_SET_OSD_NAME:
            break;
        case CEC_OC_VENDOR_COMMAND_WITH_ID:
            break;
        case CEC_OC_SET_MENU_LANGUAGE:
            cec_set_menu_language(pcec_message);
            break;
        case CEC_OC_GIVE_PHYSICAL_ADDRESS:
            cec_report_physical_address_smp();
            break;
        case CEC_OC_GIVE_DEVICE_VENDOR_ID:
            cec_device_vendor_id(pcec_message);
            break;
        case CEC_OC_GIVE_OSD_NAME:
            break;
        case CEC_OC_STANDBY:
            cec_inactive_source_rx(pcec_message);
            cec_standby(pcec_message);
            break;
        case CEC_OC_SET_STREAM_PATH:
            cec_set_stream_path(pcec_message);
            break;
        case CEC_OC_REQUEST_ACTIVE_SOURCE:
            if(cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_status != DEVICE_MENU_ACTIVE)
                break;
            cec_active_source_smp();
            break;
        case CEC_OC_GIVE_DEVICE_POWER_STATUS:
            cec_report_power_status(pcec_message);
            break;
        case CEC_OC_USER_CONTROL_PRESSED:
            break;
        case CEC_OC_USER_CONTROL_RELEASED:
            break;
        case CEC_OC_IMAGE_VIEW_ON:      //not support in source
            cec_usrcmd_set_imageview_on( CEC_TV_ADDR );   // Wakeup TV
            break;
        case CEC_OC_ROUTING_CHANGE:
            cec_routing_change(pcec_message);
            break;
        case CEC_OC_ROUTING_INFORMATION:
        	cec_routing_information(pcec_message);
        	break;
        case CEC_OC_GIVE_AUDIO_STATUS:
        	cec_report_audio_status();
        	break;
        case CEC_OC_MENU_REQUEST:
            cec_menu_status(pcec_message);
            break;
        case CEC_OC_PLAY:
            hdmitx_cec_dbg_print("CEC_OC_PLAY:0x%x\n",pcec_message->content.msg.operands[0]);
            switch(pcec_message->content.msg.operands[0]){
                case 0x24:
                    input_event(remote_cec_dev, EV_KEY, KEY_PLAYPAUSE, 1);
                    input_sync(remote_cec_dev);
                    input_event(remote_cec_dev, EV_KEY, KEY_PLAYPAUSE, 0);
                    input_sync(remote_cec_dev);
                    break;
                case 0x25:
                    input_event(remote_cec_dev, EV_KEY, KEY_PLAYPAUSE, 1);
                    input_sync(remote_cec_dev);
                    input_event(remote_cec_dev, EV_KEY, KEY_PLAYPAUSE, 0);
                    input_sync(remote_cec_dev);
                    break;
                default:
                    break;
            }
            break;
        case CEC_OC_DECK_CONTROL:
            hdmitx_cec_dbg_print("CEC_OC_DECK_CONTROL:0x%x\n",pcec_message->content.msg.operands[0]);
            switch(pcec_message->content.msg.operands[0]){
                case 0x3:
                    input_event(remote_cec_dev, EV_KEY, KEY_STOP, 1);
                    input_sync(remote_cec_dev);
                    input_event(remote_cec_dev, EV_KEY, KEY_STOP, 0);
                    input_sync(remote_cec_dev);
                    break;
                default:
                    break;
            }
            break;
        case CEC_OC_GET_MENU_LANGUAGE:
        case CEC_OC_VENDOR_REMOTE_BUTTON_DOWN:
        case CEC_OC_VENDOR_REMOTE_BUTTON_UP:
        case CEC_OC_CLEAR_ANALOGUE_TIMER:
        case CEC_OC_CLEAR_DIGITAL_TIMER:
        case CEC_OC_CLEAR_EXTERNAL_TIMER:
        case CEC_OC_GIVE_SYSTEM_AUDIO_MODE_STATUS:
        case CEC_OC_GIVE_TUNER_DEVICE_STATUS:
        case CEC_OC_SET_OSD_STRING:
        case CEC_OC_SET_SYSTEM_AUDIO_MODE:
        case CEC_OC_SET_TIMER_PROGRAM_TITLE:
        case CEC_OC_SYSTEM_AUDIO_MODE_REQUEST:
        case CEC_OC_SYSTEM_AUDIO_MODE_STATUS:
        case CEC_OC_TEXT_VIEW_ON:
        case CEC_OC_TIMER_CLEARED_STATUS:
        case CEC_OC_TIMER_STATUS:
        case CEC_OC_TUNER_DEVICE_STATUS:
        case CEC_OC_TUNER_STEP_DECREMENT:
        case CEC_OC_TUNER_STEP_INCREMENT:
        case CEC_OC_VENDOR_COMMAND:
        case CEC_OC_SELECT_ANALOGUE_SERVICE:
        case CEC_OC_SELECT_DIGITAL_SERVICE:
        case CEC_OC_SET_ANALOGUE_TIMER :
        case CEC_OC_SET_AUDIO_RATE:
        case CEC_OC_SET_DIGITAL_TIMER:
        case CEC_OC_SET_EXTERNAL_TIMER:
        case CEC_OC_RECORD_OFF:
        case CEC_OC_RECORD_ON:
        case CEC_OC_RECORD_STATUS:
        case CEC_OC_RECORD_TV_SCREEN:
        case CEC_OC_REPORT_AUDIO_STATUS:
        case CEC_OC_ABORT_MESSAGE:
            cec_feature_abort(pcec_message);
            break;
        default:
            break;
        }
    }
}


// --------------- cec command from user application --------------------

void cec_usrcmd_parse_all_dev_online(void)
{
    int i;
    unsigned short tmp_mask;

    hdmirx_cec_dbg_print("cec online: ###############################################\n");
    hdmirx_cec_dbg_print("active_log_dev %x\n", cec_global_info.active_log_dev);
    for (i = 0; i < MAX_NUM_OF_DEV; i++) {
        tmp_mask = 1 << i;
        if (tmp_mask & cec_global_info.dev_mask) {
            hdmirx_cec_dbg_print("cec online: -------------------------------------------\n");
            hdmirx_cec_dbg_print("hdmi_port:     %x\n", cec_global_info.cec_node_info[i].hdmi_port);
            hdmirx_cec_dbg_print("dev_type:      %x\n", cec_global_info.cec_node_info[i].dev_type);
            hdmirx_cec_dbg_print("power_status:  %x\n", cec_global_info.cec_node_info[i].power_status);
            hdmirx_cec_dbg_print("cec_version:   %x\n", cec_global_info.cec_node_info[i].cec_version);
            hdmirx_cec_dbg_print("vendor_id:     %x\n", cec_global_info.cec_node_info[i].vendor_id);
            hdmirx_cec_dbg_print("phy_addr:      %x\n", cec_global_info.cec_node_info[i].phy_addr.phy_addr_4);
            hdmirx_cec_dbg_print("log_addr:      %x\n", cec_global_info.cec_node_info[i].log_addr);
            hdmirx_cec_dbg_print("osd_name:      %s\n", cec_global_info.cec_node_info[i].osd_name);
            hdmirx_cec_dbg_print("osd_name_def:  %s\n", cec_global_info.cec_node_info[i].osd_name_def);
            hdmirx_cec_dbg_print("menu_state:    %x\n", cec_global_info.cec_node_info[i].menu_state);

            if (cec_global_info.cec_node_info[i].dev_type == CEC_PLAYBACK_DEVICE_TYPE) {
                hdmirx_cec_dbg_print("deck_cnt_mode: %x\n", cec_global_info.cec_node_info[i].specific_info.playback.deck_cnt_mode);
                hdmirx_cec_dbg_print("deck_info:     %x\n", cec_global_info.cec_node_info[i].specific_info.playback.deck_info);
                hdmirx_cec_dbg_print("play_mode:     %x\n", cec_global_info.cec_node_info[i].specific_info.playback.play_mode);
            }
        }
    }
    hdmirx_cec_dbg_print("##############################################################\n");
}

void cec_usrcmd_get_cec_version(unsigned char log_addr)
{
    MSG_P0(cec_global_info.my_node_index, log_addr,
            CEC_OC_GET_CEC_VERSION);

    cec_ll_tx(gbl_msg, 2);
}

void cec_usrcmd_get_audio_status(unsigned char log_addr)
{
    MSG_P0(cec_global_info.my_node_index, log_addr, CEC_OC_GIVE_AUDIO_STATUS);

    cec_ll_tx(gbl_msg, 2);
}

void cec_usrcmd_get_deck_status(unsigned char log_addr)
{
    MSG_P1(cec_global_info.my_node_index, log_addr, CEC_OC_GIVE_DECK_STATUS, STATUS_REQ_ON);

    cec_ll_tx(gbl_msg, 3);
}

void cec_usrcmd_set_deck_cnt_mode(unsigned char log_addr, deck_cnt_mode_e deck_cnt_mode)
{
    MSG_P1(cec_global_info.my_node_index, log_addr, CEC_OC_DECK_CONTROL, deck_cnt_mode);

    cec_ll_tx(gbl_msg, 3);
}

void cec_usrcmd_get_device_power_status(unsigned char log_addr)
{
    MSG_P0(cec_global_info.my_node_index, log_addr, CEC_OC_GIVE_DEVICE_POWER_STATUS);

    cec_ll_tx(gbl_msg, 2);
}

void cec_usrcmd_get_device_vendor_id(unsigned char log_addr)
{
    MSG_P0(cec_global_info.my_node_index, log_addr, CEC_OC_GIVE_DEVICE_VENDOR_ID);

    cec_ll_tx(gbl_msg, 2);
}

void cec_usrcmd_get_osd_name(unsigned char log_addr)
{
    MSG_P0(cec_global_info.my_node_index, log_addr, CEC_OC_GIVE_OSD_NAME);

    cec_ll_tx(gbl_msg, 2);
}

void cec_usrcmd_get_physical_address(unsigned char log_addr)
{
    MSG_P0(cec_global_info.my_node_index, log_addr, CEC_OC_GIVE_PHYSICAL_ADDRESS);

    cec_ll_tx(gbl_msg, 2);
}

void cec_usrcmd_get_system_audio_mode_status(unsigned char log_addr)
{
    MSG_P0(cec_global_info.my_node_index, log_addr, CEC_OC_GIVE_SYSTEM_AUDIO_MODE_STATUS);

    cec_ll_tx(gbl_msg, 2);
}

void cec_usrcmd_set_standby(unsigned char log_addr)
{
    MSG_P0(cec_global_info.my_node_index, log_addr, CEC_OC_STANDBY);

    cec_ll_tx(gbl_msg, 2);
}

/////////////////////////
void cec_usrcmd_set_imageview_on(unsigned char log_addr)
{
    MSG_P0(cec_global_info.my_node_index, log_addr,
            CEC_OC_IMAGE_VIEW_ON);

    cec_ll_tx(gbl_msg, 2);
}

void cec_usrcmd_text_view_on(unsigned char log_addr)
{
    MSG_P0(cec_global_info.my_node_index, log_addr,
            CEC_OC_TEXT_VIEW_ON);

    cec_ll_tx(gbl_msg, 2);
}

void cec_usrcmd_get_tuner_device_status(unsigned char log_addr)
{
    MSG_P0(cec_global_info.my_node_index, log_addr, CEC_OC_GIVE_TUNER_DEVICE_STATUS);

    cec_ll_tx(gbl_msg, 2);
}

void cec_usrcmd_set_play_mode(unsigned char log_addr, play_mode_e play_mode)
{
    MSG_P1(cec_global_info.my_node_index, log_addr, CEC_OC_PLAY, play_mode);

    cec_ll_tx(gbl_msg, 3);
}

void cec_usrcmd_get_menu_state(unsigned char log_addr)
{
    MSG_P1(cec_global_info.my_node_index, log_addr, CEC_OC_MENU_REQUEST, MENU_REQ_QUERY);

    cec_ll_tx(gbl_msg, 3);
}

void cec_usrcmd_set_menu_state(unsigned char log_addr, menu_req_type_e menu_req_type)
{
    MSG_P1(cec_global_info.my_node_index, log_addr, CEC_OC_MENU_REQUEST, menu_req_type);

    cec_ll_tx(gbl_msg, 3);
}

void cec_usrcmd_get_menu_language(unsigned char log_addr)
{
    MSG_P0(cec_global_info.my_node_index, log_addr, CEC_OC_GET_MENU_LANGUAGE);

    cec_ll_tx(gbl_msg, 2);
}

void cec_usrcmd_get_active_source(void)
{
    MSG_P0(cec_global_info.my_node_index, 0xF, CEC_OC_REQUEST_ACTIVE_SOURCE);

    cec_ll_tx(gbl_msg, 2);
}

void cec_usrcmd_set_active_source(void)
{
    unsigned char index = cec_global_info.my_node_index;
    unsigned char phy_addr_ab = (aml_read_reg32(P_AO_DEBUG_REG1) >> 8) & 0xff;
    unsigned char phy_addr_cd = aml_read_reg32(P_AO_DEBUG_REG1) & 0xff;

    MSG_P2(index, CEC_BROADCAST_ADDR,
            CEC_OC_ACTIVE_SOURCE,
			phy_addr_ab,
			phy_addr_cd);

    cec_ll_tx(gbl_msg, 4);
}

void cec_usrcmd_set_deactive_source(unsigned char log_addr)
{
    unsigned char phy_addr_ab = (aml_read_reg32(P_AO_DEBUG_REG1) >> 8) & 0xff;
    unsigned char phy_addr_cd = aml_read_reg32(P_AO_DEBUG_REG1) & 0xff;

    MSG_P2(cec_global_info.my_node_index, log_addr, CEC_OC_INACTIVE_SOURCE,
           phy_addr_ab,
           phy_addr_cd);

    cec_ll_tx(gbl_msg, 4);
}

void cec_usrcmd_clear_node_dev_real_info_mask(unsigned char log_addr, cec_info_mask mask)
{
    cec_global_info.cec_node_info[log_addr].real_info_mask &= ~mask;
}


void cec_usrcmd_set_osd_name(cec_rx_message_t* pcec_message)
{

    unsigned char log_addr = pcec_message->content.msg.header >> 4 ;
    unsigned char index = cec_global_info.my_node_index;

    MSG_P14(index, log_addr,
            CEC_OC_SET_OSD_NAME,
            cec_global_info.cec_node_info[index].osd_name[0],
            cec_global_info.cec_node_info[index].osd_name[1],
            cec_global_info.cec_node_info[index].osd_name[2],
            cec_global_info.cec_node_info[index].osd_name[3],
            cec_global_info.cec_node_info[index].osd_name[4],
            cec_global_info.cec_node_info[index].osd_name[5],
            cec_global_info.cec_node_info[index].osd_name[6],
            cec_global_info.cec_node_info[index].osd_name[7],
            cec_global_info.cec_node_info[index].osd_name[8],
            cec_global_info.cec_node_info[index].osd_name[9],
            cec_global_info.cec_node_info[index].osd_name[10],
            cec_global_info.cec_node_info[index].osd_name[11],
            cec_global_info.cec_node_info[index].osd_name[12],
            cec_global_info.cec_node_info[index].osd_name[13]);

    cec_ll_tx(gbl_msg, 16);
}



void cec_usrcmd_set_device_vendor_id(void)
{
    unsigned char index = cec_global_info.my_node_index;

    MSG_P3(index, CEC_BROADCAST_ADDR,
            CEC_OC_DEVICE_VENDOR_ID,
            (cec_global_info.cec_node_info[index].vendor_id >> 16) & 0xff,
            (cec_global_info.cec_node_info[index].vendor_id >> 8) & 0xff,
            (cec_global_info.cec_node_info[index].vendor_id >> 0) & 0xff);

    cec_ll_tx(gbl_msg, 5);
}
void cec_usrcmd_set_report_physical_address(void)
{
    unsigned char index = cec_global_info.my_node_index;
    unsigned char phy_addr_ab = (aml_read_reg32(P_AO_DEBUG_REG1) >> 8) & 0xff;
    unsigned char phy_addr_cd = aml_read_reg32(P_AO_DEBUG_REG1) & 0xff;

    MSG_P3(index, CEC_BROADCAST_ADDR,
           CEC_OC_REPORT_PHYSICAL_ADDRESS,
           phy_addr_ab,
           phy_addr_cd,
           CEC_PLAYBACK_DEVICE_TYPE);

    cec_ll_tx(gbl_msg, 5);
}

void cec_routing_change(cec_rx_message_t* pcec_message)
{
    unsigned int phy_addr_origin;
    unsigned int phy_addr_destination;

    phy_addr_origin = (unsigned int)((pcec_message->content.msg.operands[0] << 8) |
                                    (pcec_message->content.msg.operands[1] << 0));
    phy_addr_destination = (unsigned int)((pcec_message->content.msg.operands[2] << 8) |
                                         (pcec_message->content.msg.operands[3] << 0));

	if(phy_addr_destination == (aml_read_reg32(P_AO_DEBUG_REG1) & 0xffff)){
	    cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_status = DEVICE_MENU_ACTIVE;
	}else{
	    cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_status = DEVICE_MENU_INACTIVE;
	}
}

void cec_routing_information(cec_rx_message_t* pcec_message)
{
    unsigned char index = cec_global_info.my_node_index;
    unsigned char phy_addr_ab = (aml_read_reg32(P_AO_DEBUG_REG1) >> 8) & 0xff;
    unsigned char phy_addr_cd = aml_read_reg32(P_AO_DEBUG_REG1) & 0xff;
    unsigned int phy_addr_destination;
    unsigned char msg[4];

    phy_addr_destination = (unsigned int)((pcec_message->content.msg.operands[0] << 8) |
                                         (pcec_message->content.msg.operands[1] << 0));

	if(phy_addr_destination == (aml_read_reg32(P_AO_DEBUG_REG1) & 0xffff)){
	    cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_status = DEVICE_MENU_ACTIVE;
    msg[0] = ((index & 0xf) << 4) | CEC_BROADCAST_ADDR;
    msg[1] = CEC_OC_ROUTING_INFORMATION;
    msg[2] = phy_addr_ab;
    msg[3] = phy_addr_cd;
    cec_ll_tx(msg, 4);
	}else{
	    cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_status = DEVICE_MENU_INACTIVE;
	}
}
/***************************** cec middle level code end *****************************/


/***************************** cec high level code *****************************/

static int __init cec_init(void)
{
    int i;
    extern __u16 cec_key_map[128];
    extern hdmitx_dev_t * get_hdmitx_device(void);
    hdmitx_device = get_hdmitx_device();
    cec_key_init();
    hdmitx_cec_dbg_print("CEC init\n");

#ifdef CONFIG_ARCH_MESON6
    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_CLOCK_DIV_H, 0x00 );
    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_CLOCK_DIV_L, 0xf0 );
#endif

    cec_rx_msg_buf.rx_write_pos = 0;
    cec_rx_msg_buf.rx_read_pos = 0;
    cec_rx_msg_buf.rx_buf_size = sizeof(cec_rx_msg_buf.cec_rx_message)/sizeof(cec_rx_msg_buf.cec_rx_message[0]);
    memset(cec_rx_msg_buf.cec_rx_message, 0, sizeof(cec_rx_msg_buf.cec_rx_message));

    memset(&cec_global_info, 0, sizeof(cec_global_info_t));

    kthread_run(cec_task, (void*)hdmitx_device, "kthread_cec");
#ifdef CONFIG_ARCH_MESON6
    if(request_irq(INT_HDMI_CEC, &cec_isr_handler,
                IRQF_SHARED, "amhdmitx-cec",
                (void *)hdmitx_device)){
        hdmitx_cec_dbg_print("HDMI CEC:Can't register IRQ %d\n",INT_HDMI_CEC);
        return -EFAULT;
    }
#endif
#ifdef CONFIG_ARCH_MESON8
    if(request_irq(INT_AO_CEC, &cec_isr_handler,
                IRQF_SHARED, "amhdmitx-aocec",
                (void *)hdmitx_device)){
        hdmitx_cec_dbg_print("HDMI CEC:Can't register IRQ %d\n",INT_HDMI_CEC);
        return -EFAULT;
    }
#endif

#if CEC_WQ
    cec_wq = create_workqueue("cec wq");
    if(!cec_wq)
        panic("Failed to create cec_workqueue.");
#endif

    INIT_WORK(&cec_wq_func, (void (*)(void *))cec_do_work);
    remote_cec_dev = input_allocate_device();
    if (!remote_cec_dev)
    {
        hdmitx_cec_dbg_print(KERN_ERR "remote_cec.c: Not enough memory\n");
    }
    remote_cec_dev->name = "cec_input";

    remote_cec_dev->evbit[0] = BIT_MASK(EV_KEY);
    remote_cec_dev->keybit[BIT_WORD(BTN_0)] = BIT_MASK(BTN_0);
    remote_cec_dev->id.bustype = BUS_ISA;
    remote_cec_dev->id.vendor = 0x1b8e;
    remote_cec_dev->id.product = 0x0cec;
    remote_cec_dev->id.version = 0x0001;

    for (i = 0; i < 128; i++){
          set_bit( cec_key_map[i], remote_cec_dev->keybit);
      }

    if(input_register_device(remote_cec_dev)) {
        hdmitx_cec_dbg_print(KERN_ERR "remote_cec.c: Failed to register device\n");
        input_free_device(remote_cec_dev);
    }

    hdmitx_device->cec_init_ready = 1;
    cec_init_ready_flag = 1;
    printk("hdmitx_device->cec_init_ready:0x%x", hdmitx_device->cec_init_ready);
    return 0;
}

static void __exit cec_uninit(void)
{
    if(!(hdmitx_device->cec_func_config & (1 << CEC_FUNC_MSAK))) {
        return ;
    }
    hdmitx_cec_dbg_print("CEC: cec uninit!\n");
    if (cec_init_flag == 1) {

#ifdef CONFIG_ARCH_MESON6
        aml_write_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_MASK, aml_read_reg32(P_SYS_CPU_0_IRQ_IN1_INTR_MASK) & ~(1 << 23));            // Disable the hdmi cec interrupt
        free_irq(INT_HDMI_CEC, (void *)hdmitx_device);
#endif
#ifdef CONFIG_ARCH_MESON8
        free_irq(INT_AO_CEC, (void *)hdmitx_device);
#endif
        cec_init_flag = 0;
    }
#if CEC_WQ
    if(cec_wq)
        destroy_workqueue(cec_wq);
#endif
    hdmitx_device->cec_init_ready = 0;
    cec_init_ready_flag = 0;
    input_unregister_device(remote_cec_dev);
    cec_fiq_flag = 0;
}

size_t cec_usrcmd_get_global_info(char * buf)
{
    int i = 0;
    int dev_num = 0;

    cec_node_info_t * buf_node_addr = (cec_node_info_t *)(buf + (unsigned int)(((cec_global_info_to_usr_t*)0)->cec_node_info_online));

    for (i = 0; i < MAX_NUM_OF_DEV; i++) {
        if (cec_global_info.dev_mask & (1 << i)) {
            memcpy(&(buf_node_addr[dev_num]), &(cec_global_info.cec_node_info[i]), sizeof(cec_node_info_t));
            dev_num++;
        }
    }

    buf[0] = dev_num;
    buf[1] = cec_global_info.active_log_dev;
#if 0
    hdmitx_cec_dbg_print("\n");
    hdmitx_cec_dbg_print("%x\n",(unsigned int)(((cec_global_info_to_usr_t*)0)->cec_node_info_online));
    hdmitx_cec_dbg_print("%x\n", ((cec_global_info_to_usr_t*)buf)->dev_number);
    hdmitx_cec_dbg_print("%x\n", ((cec_global_info_to_usr_t*)buf)->active_log_dev);
    hdmitx_cec_dbg_print("%x\n", ((cec_global_info_to_usr_t*)buf)->cec_node_info_online[0].hdmi_port);
    for (i=0; i < (sizeof(cec_node_info_t) * dev_num) + 2; i++) {
        hdmitx_cec_dbg_print("%x,",buf[i]);
    }
    hdmitx_cec_dbg_print("\n");
#endif
    return (sizeof(cec_node_info_t) * dev_num) + (unsigned int)(((cec_global_info_to_usr_t*)0)->cec_node_info_online);
}

void cec_usrcmd_set_lang_config(const char * buf, size_t count)
{
    char tmpbuf[128];
    int i=0;

    while((buf[i])&&(buf[i]!=',')&&(buf[i]!=' ')){
        tmpbuf[i]=buf[i];
        i++;
    }

    cec_global_info.cec_node_info[cec_global_info.my_node_index].menu_lang = simple_strtoul(tmpbuf, NULL, 16);

}
void cec_usrcmd_set_config(const char * buf, size_t count)
{
    int i = 0;
    int j = 0;
    unsigned long value;
    char param[16] = {0};

    if(count > 32){
        hdmitx_cec_dbg_print("CEC: too many args\n");
    }
    for(i = 0; i < count; i++){
        if ( (buf[i] >= '0') && (buf[i] <= 'f') ){
            param[j] = simple_strtoul(&buf[i], NULL, 16);
            j ++;
        }
        while ( buf[i] != ' ' )
            i ++;
    }
    value = aml_read_reg32(P_AO_DEBUG_REG0) & 0x1;
    aml_set_reg32_bits(P_AO_DEBUG_REG0, param[0], 0, 4);
    hdmitx_device->cec_func_config = aml_read_reg32(P_AO_DEBUG_REG0);
    if(!(hdmitx_device->cec_func_config & (1 << CEC_FUNC_MSAK))) {
        return ;
    }
    if((0 == value) && (1 == (param[0] & 1))){
        hdmitx_device->cec_init_ready = 1;
        cec_init_ready_flag = 1;
        hdmitx_device->hpd_state = 1;
#ifdef CONFIG_ARCH_MESON6
        cec_gpi_init();
#endif
        cec_node_init(hdmitx_device);
    }
    if((1 == (param[0] & 1)) && (0x2 == (value & 0x2)) && (0x0 == (param[0] & 0x2))){
        cec_menu_status_smp(DEVICE_MENU_INACTIVE);
    }
    if((1 == (param[0] & 1)) && (0x0 == (value & 0x2)) && (0x2 == (param[0] & 0x2))){
        cec_active_source_smp();
    }
    hdmirx_cec_dbg_print("cec: cec_func_config:0x%x : 0x%x\n",hdmitx_device->cec_func_config, aml_read_reg32(P_AO_DEBUG_REG0));
}


void cec_usrcmd_set_dispatch(const char * buf, size_t count)
{
    int i = 0;
    int j = 0;
    int n = 0;
    int m = 0;
    int bool = 0;
    char param[16] = {0};
    char tmpbuf[32] = {0};
    unsigned bit_set;
    unsigned time_set;
    unsigned char msg[4] = {0};

    if(count > 32){
        hdmitx_cec_dbg_print("CEC: too many args\n");
    }
    for(i = 0; i < count; i++){
        if ( (buf[i] >= '0') && (buf[i] <= 'f') ){
            param[j] = simple_strtoul(&buf[i], NULL, 16);
            j ++;
        }
        while ( buf[i] != ' ' )
            i ++;
    }

    while((buf[n]!=',')&&(buf[i]!=' ')){
        tmpbuf[n]=buf[n];
        n++;
    }
    tmpbuf[n]=0;
    hdmirx_cec_dbg_print("cec_usrcmd_set_dispatch: \n");
    if(strncmp(tmpbuf, "waocec", 6)==0){
        bit_set = simple_strtoul(tmpbuf+6, NULL, 16);
        time_set = simple_strtoul(buf+n+1, NULL, 16);
        cec_arbit_bit_time_set(bit_set, time_set, 1);
        return;
    }else if(strncmp(tmpbuf, "raocec", 6)==0){
        cec_arbit_bit_time_read();
        return;
    }
    switch (param[0]) {
    case GET_CEC_VERSION:   //0 LA
        cec_usrcmd_get_cec_version(param[1]);
        break;
    case GET_DEV_POWER_STATUS:
        cec_usrcmd_get_device_power_status(param[1]);
        break;
    case GET_DEV_VENDOR_ID:
        cec_usrcmd_get_device_vendor_id(param[1]);
        break;
    case GET_OSD_NAME:
        cec_usrcmd_get_osd_name(param[1]);
        break;
    case GET_PHYSICAL_ADDR:
        cec_usrcmd_get_physical_address(param[1]);
        break;
    case SET_STANDBY:       //d LA
        cec_usrcmd_set_standby(param[1]);
        break;
    case SET_IMAGEVIEW_ON:  //e LA
        cec_usrcmd_set_imageview_on(param[1]);
        break;
    case GIVE_DECK_STATUS:
        cec_usrcmd_get_deck_status(param[1]);
        break;
    case SET_DECK_CONTROL_MODE:
        cec_usrcmd_set_deck_cnt_mode(param[1], param[2]);
        break;
    case SET_PLAY_MODE:
        cec_usrcmd_set_play_mode(param[1], param[2]);
        break;
    case GET_SYSTEM_AUDIO_MODE:
        cec_usrcmd_get_system_audio_mode_status(param[1]);
        break;
    case GET_TUNER_DEV_STATUS:
        cec_usrcmd_get_tuner_device_status(param[1]);
        break;
    case GET_AUDIO_STATUS:
        cec_usrcmd_get_audio_status(param[1]);
        break;
    case GET_OSD_STRING:
        break;
    case GET_MENU_STATE:
        cec_usrcmd_get_menu_state(param[1]);
        break;
    case SET_MENU_STATE:
        cec_usrcmd_set_menu_state(param[1], param[2]);
        break;
    case SET_MENU_LANGAGE:
        break;
    case GET_MENU_LANGUAGE:
        cec_usrcmd_get_menu_language(param[1]);
        break;
    case GET_ACTIVE_SOURCE:     //13
        cec_usrcmd_get_active_source();
        break;
    case SET_ACTIVE_SOURCE:
        cec_usrcmd_set_active_source();
        break;
    case SET_DEACTIVE_SOURCE:
        cec_usrcmd_set_deactive_source(param[1]);
        break;
    case REPORT_PHYSICAL_ADDRESS:    //17
    	cec_usrcmd_set_report_physical_address();
    	break;
    case SET_TEXT_VIEW_ON:          //18 LA
    	cec_usrcmd_text_view_on(param[1]);
        break;
    case POLLING_ONLINE_DEV:    //19 LA
        cec_polling_online_dev(param[1], &bool);
        break;

    case CEC_OC_MENU_STATUS:
        cec_menu_status_smp(DEVICE_MENU_INACTIVE);
        break;
    case CEC_OC_ABORT_MESSAGE:

        msg[0] = 0x40;
        msg[1] = CEC_OC_FEATURE_ABORT;
        msg[2] = 0;
        msg[3] = CEC_UNRECONIZED_OPCODE;

        cec_ll_tx(msg, 4);
        break;
    default:
        break;
    }
}

/***************************** cec high level code end *****************************/

late_initcall(cec_init);
module_exit(cec_uninit);
MODULE_DESCRIPTION("AMLOGIC HDMI TX CEC driver");
MODULE_LICENSE("GPL");
//MODULE_VERSION("1.0.0");

MODULE_PARM_DESC(cec_msg_dbg_en, "\n cec_msg_dbg_en\n");
module_param(cec_msg_dbg_en, bool, 0664);


/*************************** cec arbitration cts code ******************************/
// using the cec pin as fiq gpi to assist the bus arbitration
static unsigned char msg_log_buf[128] = { 0 };
// return value: 1: successful      0: error
static int cec_ll_tx_once(const unsigned char *msg, unsigned char len)
{
    int i;
    unsigned int ret = 0xf;
    unsigned int n;
    unsigned int cnt = 30;
    int pos;
#ifdef CONFIG_ARCH_MESON6
    for (i = 0; i < len; i++) {
        hdmi_wr_reg(CEC0_BASE_ADDR+CEC_TX_MSG_0_HEADER + i, msg[i]);
    }
    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_TX_MSG_LENGTH, len-1);
    //cec_tx_start = 1;
    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_TX_MSG_CMD, RX_ACK_CURRENT);//TX_REQ_NEXT
    msleep(len * 24 + 5);

    ret = hdmi_rd_reg(CEC0_BASE_ADDR+CEC_TX_MSG_STATUS);

    if(ret == TX_DONE)
        ret = 1;
    else
        ret = 0;

    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_TX_MSG_CMD, TX_NO_OP);
#endif
#ifdef CONFIG_ARCH_MESON8
    while(aocec_rd_reg(CEC_TX_MSG_STATUS) || aocec_rd_reg(CEC_RX_MSG_STATUS)){
        msleep(5);
        if(TX_ERROR == aocec_rd_reg(CEC_TX_MSG_STATUS)){
            //aocec_wr_reg(CEC_TX_MSG_CMD, TX_ABORT);
            aocec_wr_reg(CEC_TX_MSG_CMD, TX_NO_OP);
            //cec_hw_reset();
            break;
        }
        if(!(cnt--)){
            printk("tx busy time out.\n");
            aocec_wr_reg(CEC_TX_MSG_CMD, TX_ABORT);
            aocec_wr_reg(CEC_TX_MSG_CMD, TX_NO_OP);
        }
    }
    for (i = 0; i < len; i++)
    {
        aocec_wr_reg(CEC_TX_MSG_0_HEADER + i, msg[i]);
    }
    aocec_wr_reg(CEC_TX_MSG_LENGTH, len-1);
    aocec_wr_reg(CEC_TX_MSG_CMD, RX_ACK_CURRENT);

#endif
    if(cec_msg_dbg_en == 1) {
        pos = 0;
        pos += sprintf(msg_log_buf + pos, "CEC: tx msg len: %d   dat: ", len);
        for(n = 0; n < len; n++) {
            pos += sprintf(msg_log_buf + pos, "%02x ", msg[n]);
        }
#ifdef CONFIG_ARCH_MESON6
        pos += sprintf(msg_log_buf + pos, "\nCEC: tx state: %d\n", ret);
#endif
#ifdef CONFIG_ARCH_MESON8
        pos += sprintf(msg_log_buf + pos, "\n");
#endif
        msg_log_buf[pos] = '\0';
        printk("%s", msg_log_buf);
    }
    return ret;
}
#ifdef CONFIG_ARCH_MESON8

static int cec_ll_tx_polling(const unsigned char *msg, unsigned char len)
{
    int i;
    unsigned int ret = 0xf;
    unsigned int n;
	unsigned int j = 30;
    int pos;
#ifdef CONFIG_ARCH_MESON6
    for (i = 0; i < len; i++) {
     hdmi_wr_reg(CEC0_BASE_ADDR+CEC_TX_MSG_0_HEADER + i, msg[i]);
    }
    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_TX_MSG_LENGTH, len-1);
    //cec_tx_start = 1;
    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_TX_MSG_CMD, RX_ACK_CURRENT);//TX_REQ_NEXT
    msleep(len * 24 + 5);

    ret = hdmi_rd_reg(CEC0_BASE_ADDR+CEC_TX_MSG_STATUS);

    if(ret == TX_DONE)
        ret = 1;
    else
        ret = 0;

    hdmi_wr_reg(CEC0_BASE_ADDR+CEC_TX_MSG_CMD, TX_NO_OP);
#endif
#ifdef CONFIG_ARCH_MESON8
    aml_set_reg32_bits(P_AO_CEC_INTR_MASKN, 0x0, 1, 1);
    for (i = 0; i < len; i++)
    {
        aocec_wr_reg(CEC_TX_MSG_0_HEADER + i, msg[i]);
    }
    aocec_wr_reg(CEC_TX_MSG_LENGTH, len-1);
    aocec_wr_reg(CEC_TX_MSG_CMD, RX_ACK_CURRENT);

    while((TX_DONE != aocec_rd_reg(CEC_TX_MSG_STATUS)) && (j--)){
		msleep(5);
	}

    ret = aocec_rd_reg(CEC_TX_MSG_STATUS);

    if(ret == TX_DONE)
        ret = 1;
    else
        ret = 0;
    aocec_wr_reg(CEC_TX_MSG_CMD, TX_NO_OP);
    aml_set_reg32_bits(P_AO_CEC_INTR_MASKN, 1, 1, 1);
#endif
    if(cec_msg_dbg_en == 1) {
        pos = 0;
        pos += sprintf(msg_log_buf + pos, "CEC: tx msg len: %d   dat: ", len);
        for(n = 0; n < len; n++) {
            pos += sprintf(msg_log_buf + pos, "%02x ", msg[n]);
        }
        pos += sprintf(msg_log_buf + pos, "\nCEC: tx state: %d\n", ret);
        msg_log_buf[pos] = '\0';
        printk("%s", msg_log_buf);
    }
    return ret;
}

void cec_arbit_bit_time_read(void){//11bit:bit[10:0]
    //3 bit
    printk("read 3 bit:0x%x%x \n", aocec_rd_reg(AO_CEC_TXTIME_4BIT_BIT10_8),aocec_rd_reg(AO_CEC_TXTIME_4BIT_BIT7_0));
    //5 bit
    printk("read 5 bit:0x%x%x \n", aocec_rd_reg(AO_CEC_TXTIME_2BIT_BIT10_8), aocec_rd_reg(AO_CEC_TXTIME_2BIT_BIT7_0));
    //7 bit
    printk("read 7 bit:0x%x%x \n", aocec_rd_reg(AO_CEC_TXTIME_17MS_BIT10_8), aocec_rd_reg(AO_CEC_TXTIME_17MS_BIT7_0));
}

void cec_arbit_bit_time_set(unsigned bit_set, unsigned time_set, unsigned flag){//11bit:bit[10:0]
    if(flag)
        printk("bit_set:0x%x;time_set:0x%x \n", bit_set, time_set);
    switch(bit_set){
    case 3:
        //3 bit
        if(flag)
            printk("read 3 bit:0x%x%x \n", aocec_rd_reg(AO_CEC_TXTIME_4BIT_BIT10_8),aocec_rd_reg(AO_CEC_TXTIME_4BIT_BIT7_0));
        aocec_wr_reg(AO_CEC_TXTIME_4BIT_BIT7_0, time_set & 0xff);
        aocec_wr_reg(AO_CEC_TXTIME_4BIT_BIT10_8, (time_set >> 8) & 0x7);
        if(flag)
            printk("write 3 bit:0x%x%x \n", aocec_rd_reg(AO_CEC_TXTIME_4BIT_BIT10_8),aocec_rd_reg(AO_CEC_TXTIME_4BIT_BIT7_0));
        break;
        //5 bit
    case 5:
        if(flag)
            printk("read 5 bit:0x%x%x \n", aocec_rd_reg(AO_CEC_TXTIME_2BIT_BIT10_8), aocec_rd_reg(AO_CEC_TXTIME_2BIT_BIT7_0));
        aocec_wr_reg(AO_CEC_TXTIME_2BIT_BIT7_0, time_set & 0xff);
        aocec_wr_reg(AO_CEC_TXTIME_2BIT_BIT10_8, (time_set >> 8) & 0x7);
        if(flag)
            printk("write 5 bit:0x%x%x \n", aocec_rd_reg(AO_CEC_TXTIME_2BIT_BIT10_8), aocec_rd_reg(AO_CEC_TXTIME_2BIT_BIT7_0));
        //7 bit
	case 7:
        if(flag)
            printk("read 7 bit:0x%x%x \n", aocec_rd_reg(AO_CEC_TXTIME_17MS_BIT10_8), aocec_rd_reg(AO_CEC_TXTIME_17MS_BIT7_0));
        aocec_wr_reg(AO_CEC_TXTIME_17MS_BIT7_0, time_set & 0xff);
        aocec_wr_reg(AO_CEC_TXTIME_17MS_BIT10_8, (time_set >> 8) & 0x7);
        if(flag)
            printk("write 7 bit:0x%x%x \n", aocec_rd_reg(AO_CEC_TXTIME_17MS_BIT10_8), aocec_rd_reg(AO_CEC_TXTIME_17MS_BIT7_0));
        break;
    default:
        break;
    }
}

void tx_irq_handle(void){
    unsigned tx_status = aocec_rd_reg(CEC_TX_MSG_STATUS);
    switch(tx_status){
    case TX_DONE:
      aocec_wr_reg(CEC_TX_MSG_CMD, TX_NO_OP);
      break;
    case TX_BUSY:
        aocec_wr_reg(CEC_TX_MSG_CMD, TX_ABORT);
        aocec_wr_reg(CEC_TX_MSG_CMD, TX_NO_OP);
        break;
    case TX_ERROR:
        cec_hw_reset();
        //aocec_wr_reg(CEC_TX_MSG_CMD, TX_NO_OP);
        break;
    default:
        break;
    }
    aml_write_reg32(P_AO_CEC_INTR_CLR, aml_read_reg32(P_AO_CEC_INTR_CLR) | (1 << 1));
    //aml_write_reg32(P_AO_CEC_INTR_MASKN, aml_read_reg32(P_AO_CEC_INTR_MASKN) | (1 << 2));

}
#endif
// Return value: 0: fail    1: success
int cec_ll_tx(const unsigned char *msg, unsigned char len)
{
    int ret = 0;
    int repeat = 0;
    int i;

    mutex_lock(&cec_mutex);
#ifdef CONFIG_ARCH_MESON8
    //aml_write_reg32(P_AO_CEC_INTR_MASKN, aml_read_reg32(P_AO_CEC_INTR_MASKN) & ~(1 << 2));
    cec_ll_tx_once(msg, len);
#endif
    //if transmit message error, try repeat(4) times
#ifdef CONFIG_ARCH_MESON6
    do {
        ret = cec_ll_tx_once(msg, len);
        repeat ++;
        if(repeat > 1)
            cec_hw_reset();
    } while((ret == 0) && (repeat < 3));

    cec_msg_dbg_en ? printk("cec: ret = %d\n", ret) : 0;
    if(repeat > 1) {
        printk("cec: try %d times\n", repeat);
    }
#endif
    mutex_unlock(&cec_mutex);
    return ret;
}
#ifdef CONFIG_ARCH_MESON6
static void cec_gpi_init(void)
{
    if((hdmitx_device->cec_init_ready == 0) || (hdmitx_device->hpd_state == 0)) {      // If no connect, return directly
        aml_set_reg32_bits(P_MEDIA_CPU_IRQ_IN2_INTR_STAT_CLR, 1, 0, 1); // Write 1 to clear irq.
        printk("CEC FIQ return.\n");
        return;
    }
    if(cec_fiq_flag){ //This function run only once.
        return;
    }
    cec_fiq_flag = 1;
    aml_set_reg32_bits(P_MEDIA_CPU_IRQ_IN2_INTR_MASK, 0, 0, 1);     // disable irq
    aml_set_reg32_bits(P_MEDIA_CPU_IRQ_IN2_INTR_STAT_CLR, 1, 0, 1); // Write 1 to clear irq

    aml_set_reg32_bits(P_GPIO_INTR_GPIO_SEL0, 0x76, 0, 8);      // set GPIOC_23 as GPIO IRQ #0 source
    aml_set_reg32_bits(P_GPIO_INTR_EDGE_POL, 1, 0, 1);          // interrupt mode:  0: level     1: edge
    aml_set_reg32_bits(P_GPIO_INTR_EDGE_POL, 1, 16, 1);
    request_fiq(INT_GPIO_0, &cec_gpi_receive_bits);
    printk("cec: register fiq\n");

    aml_set_reg32_bits(P_MEDIA_CPU_IRQ_IN2_INTR_MASK, 1, 0, 1);     // enable irq
}
#endif
// DELETE LATER, TEST ONLY
void cec_test_(unsigned int cmd)
{
}

