/*
 * Amlogic M6 & M8
 * camera serial input driver  -------CSI
 * Copyright (C) 2013 Amlogic, Inc.
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

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <asm/delay.h>
#include <asm/atomic.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <mach/am_regs.h>
#include <mach/mod_gate.h>

#include <mach/am_regs.h>
#include <mach/mipi_phy_reg.h>
#include <linux/amlogic/mipi/am_mipi_csi2.h>

#include "../tvin_global.h"
#include "../vdin/vdin_regs.h"
#include "../vdin/vdin_drv.h"
#include "../vdin/vdin_ctl.h"
#include "../tvin_format_table.h"
#include "../tvin_frontend.h"
#include "csi.h"

#define DEV_NAME  "amvdec_csi"
#define DRV_NAME  "amvdec_csi"
#define CLS_NAME  "amvdec_csi"
#define MOD_NAME  "amvdec_csi"

#define CSI_MAX_DEVS             1

/* Per-device (per-bank) structure */
static dev_t amcsi_devno;
static struct class *amcsi_clsp;

static void init_csi_dec_parameter(struct amcsi_dev_s *devp)
{
        enum tvin_sig_fmt_e fmt;
        const struct tvin_format_s * fmt_info_p;
        fmt = devp->para.fmt;
        fmt_info_p = tvin_get_fmt_info(fmt);

        if(!fmt_info_p) {
                printk("[amcsi..]%s:invaild fmt %d.\n",__func__, fmt);
                return;
        }

        if(fmt < TVIN_SIG_FMT_MAX)
        {
                devp->para.v_active    = fmt_info_p->v_active;
                devp->para.h_active    = fmt_info_p->h_active;
                devp->para.hsync_phase = 0;
                devp->para.vsync_phase = 0;
                devp->para.hs_bp       = 0;
                devp->para.vs_bp       = 0;
        }
}

static void reset_btcsi_module(void)
{
        printk("%s, %d\n", __func__, __LINE__);
        return;
}

/*
   NTSC or PAL input(interlace mode): CLOCK + D0~D7(with SAV + EAV )
   */
static void reinit_csi_dec(struct amcsi_dev_s *devp)
{
        printk("%s, %d\n", __func__, __LINE__);
        return;
}

static void start_amvdec_csi(struct amcsi_dev_s *devp)
{
        enum tvin_port_e port =  devp->para.port;
        if(devp->dec_status & TVIN_AMCSI_RUNNING){
                printk("[csi..] %s csi have started alreadly.\n",__func__);
                return;
        }
        devp->dec_status = TVIN_AMCSI_RUNNING;
        if(port == TVIN_PORT_MIPI){
                init_csi_dec_parameter(devp);
                reinit_csi_dec(devp);
        }
        else
        {
                devp->para.fmt  = TVIN_SIG_FMT_NULL;
                devp->para.port = TVIN_PORT_NULL;
                printk("[csi..]%s: input is not selected, please try again. \n",__func__);
                return;
        }
        devp->dec_status = TVIN_AMCSI_RUNNING;
        return;
}

static void stop_amvdec_csi(struct amcsi_dev_s *devp)
{
        if(devp->dec_status & TVIN_AMCSI_RUNNING){
                reset_btcsi_module();
                devp->dec_status = TVIN_AMCSI_STOP;
        }
        else{
                printk("[csi..] %s device is not started yet. \n",__func__);
        }
        return;
}

/*
   return true when need skip frame otherwise return false
   */
static bool amcsi_check_skip_frame(struct tvin_frontend_s * fe)
{
        struct amcsi_dev_s * devp =  container_of(fe, amcsi_dev_t, frontend);

        return false;
        if(devp->csi_parm.skip_frames > 0)
        {
                devp->csi_parm.skip_frames--;
                return true;
        }
        else
                return false;
}
int amcsi_support(struct tvin_frontend_s *fe, enum tvin_port_e port)
{
        if((port != TVIN_PORT_MIPI)) {
                printk("error 1\n");
                return -1;
        } else {
                return 0;
        }
}

static int amcsi_open(struct inode *node, struct file *file)
{
        amcsi_dev_t *csi_devp;

        /* Get the per-device structure that contains this cdev */
        csi_devp = container_of(node->i_cdev, amcsi_dev_t, cdev);
        file->private_data = csi_devp;

        return 0;

}
static int amcsi_release(struct inode *node, struct file *file)
{
        file->private_data = NULL;
        return 0;
}

static struct file_operations amcsi_fops = {
        .owner    = THIS_MODULE,
        .open     = amcsi_open,
        .release  = amcsi_release,
};
/*called by vdin && sever for v4l2 framework*/

void amcsi_start(struct tvin_frontend_s *fe, enum tvin_sig_fmt_e fmt)
{
        struct amcsi_dev_s *csi_devp;

        csi_devp = container_of(fe, amcsi_dev_t, frontend);
        start_amvdec_csi( csi_devp );

}
static void amcsi_stop(struct tvin_frontend_s * fe, enum tvin_port_e port)
{
        struct amcsi_dev_s *devp = container_of(fe, amcsi_dev_t, frontend);
        if((port != TVIN_PORT_MIPI)){
                printk("%s:invaild port %d.\n",__func__, port);
                return;
        }
        stop_amvdec_csi(devp);
}
static void amcsi_get_sig_propery(struct tvin_frontend_s *fe, struct tvin_sig_property_s *prop)
{
        struct amcsi_dev_s *devp = container_of(fe, amcsi_dev_t, frontend);
        prop->color_format = devp->para.cfmt;//devp->csi_parm.csi_ofmt;
        //prop->dest_cfmt = devp->para.cfmt;
        printk("csi_ofmt=%d, cfmt=%d\n", devp->csi_parm.csi_ofmt, devp->para.cfmt );
        prop->pixel_repeat = 0;
}

/*as use the spin_lock,
 *1--there is no sleep,
 *2--it is better to shorter the time,
 */
int amcsi_isr(struct tvin_frontend_s *fe, unsigned int hcnt)
{
        struct amcsi_dev_s *devp = container_of(fe, amcsi_dev_t, frontend);
        unsigned data1 = 0;
        unsigned data2 = 0;
        am_csi2_frame_t frame;

        frame.w = aml_get_reg32_bits( P_CSI2_PIC_SIZE_STAT, 0, 16);
        frame.h = aml_get_reg32_bits( P_CSI2_PIC_SIZE_STAT, 16,16);
        frame.err = aml_read_reg32( P_CSI2_ERR_STAT0 );
        data1 = aml_read_reg32( P_CSI2_DATA_TYPE_IN_MEM);
        data2 = aml_read_reg32( P_CSI2_GEN_STAT0);

        if(frame.err){
                mipi_error("%s,error---pixel cnt:%d, line cnt:%d. error state:0x%x.mem type:0x%x, status:0x%x\n",
                                __func__, frame.w, frame.h, frame.err, data1, data2);
                devp->overflow_cnt ++;
                aml_write_reg32( P_CSI2_ERR_STAT0, 0);
        }
        if( devp->overflow_cnt > 20){
                printk("should reset mipi\n");
                devp->overflow_cnt = 0;
        }

        return 0;
}

static ssize_t csi_attr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        ssize_t len = 0;
        struct amcsi_dev_s *csi_devp;
        int i;

        csi_devp = dev_get_drvdata(dev);

        len += sprintf(buf+len, "csi parameters below\n");
        len += sprintf(buf+len, "\tlanes=%d, channel=%d, clk_channel=%d\n"
                        "\tmode=%d, clock_lane_mode=%d, active_pixel=%d\n"
                        "\tactive_line=%d, frame_size=%d, ui_val=%dns\n"
                        "\ths_freq=%dhz, urgent=%d\n",
                        csi_devp->csi_parm.lanes,
                        csi_devp->csi_parm.channel,
                        csi_devp->csi_parm.clk_channel,
                        csi_devp->csi_parm.mode,
                        csi_devp->csi_parm.clock_lane_mode, // 0 clock gate 1: always on
                        csi_devp->csi_parm.active_pixel,
                        csi_devp->csi_parm.active_line,
                        csi_devp->csi_parm.frame_size,
                        csi_devp->csi_parm.ui_val, //ns
                        csi_devp->csi_parm.hs_freq, //hz
                        csi_devp->csi_parm.urgent);

        len += sprintf(buf+len, "csi adapter register below\n");
        for( i = CSI_ADPT_START_REG; i <= CSI_ADPT_END_REG; i ++ )
        {
                len += sprintf(buf+len, "\t[0x%04x]=0x%08x\n",
                                i, READ_CSI_ADPT_REG(i));
        }

        len += sprintf(buf+len, "csi phy register below\n");
        for( i = CSI_PHY_START_REG; i <= CSI_PHY_END_REG; i ++ )
        {
                len += sprintf(buf+len, "\t[0x%04x]=0x%08x\n",
                                i, READ_CSI_PHY_REG(i));
        }

        len += sprintf(buf+len, "csi host register below\n");
        for( i = CSI_HST_START_REG; i <= CSI_HST_END_REG; i ++ )
        {
                len += sprintf(buf+len, "\t[0x%04x]=0x%08x\n",
                               i<<2, READ_CSI_HST_REG(i));
        }

        return len;
}

static ssize_t csi_attr_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t len)
{
        struct amcsi_dev_s *csi_devp;

        unsigned int n=0, fps=0;

        unsigned char ret=0;
        char *buf_orig, *ps, *token;
        char *parm[6] = {NULL};

        if(!buf)
                return len;
        buf_orig = kstrdup(buf, GFP_KERNEL);
        //printk(KERN_INFO "input cmd : %s",buf_orig);
        csi_devp = dev_get_drvdata(dev);

        ps = buf_orig;
        while (1) {
                if ( n >=ARRAY_SIZE(parm) ){
                        printk("parm array overflow, n=%d, ARRAY_SIZE(parm)=%d\n", n, ARRAY_SIZE(parm));
                        return len;
                }
                token = strsep(&ps, " \n");
                if (token == NULL)
                        break;
                if (*token == '\0')
                        continue;
                parm[n++] = token;
        }

        if ( 0 == strcmp(parm[0],"reset")){
                printk("reset\n");
                am_mipi_csi2_init(&csi_devp->csi_parm);
        } else {
                printk("other\n");
        }

        kfree(buf_orig);
        return len;
}

static DEVICE_ATTR(hw_info, 0664, csi_attr_show, csi_attr_store);
/*
 *power on mipi module&init the parameters,such as color fmt...,will be used by vdin
 */
static int amcsi_feopen(struct tvin_frontend_s *fe, enum tvin_port_e port)
{
        struct amcsi_dev_s *csi_devp = container_of(fe, amcsi_dev_t, frontend);
        struct vdin_parm_s *parm = fe->private_data;

        if((port != TVIN_PORT_MIPI)){
                printk("[mipi..]%s:invaild port %d.\n",__func__, port);
                return -1;
        }
        /*copy the param from vdin to csi*/
        if(!memcpy(&csi_devp->para, parm, sizeof(vdin_parm_t))){
                printk("[mipi..]%s memcpy error.\n",__func__);
                return -1;
        }

        init_am_mipi_csi2_clock();// init mipi csi measure clock
        csi_devp->para.port = port;

        memcpy( &csi_devp->csi_parm, &parm->csi_hw_info, sizeof( csi_parm_t));
        csi_devp->csi_parm.skip_frames = parm->skip_count;

        am_mipi_csi2_init(&csi_devp->csi_parm);
        return 0;
        //csi_devp->skip_vdin_frame_count = parm->reserved;
}
/*
 *power off the csi module,clear the parameters
 */
static void amcsi_feclose(struct tvin_frontend_s *fe)
{
        struct amcsi_dev_s *devp = container_of(fe, amcsi_dev_t, frontend);
        enum tvin_port_e port = devp->para.port;

        if((port != TVIN_PORT_MIPI)){
                printk("[mipi..]%s:invaild port %d.\n",__func__, port);
                return;
        }

        am_mipi_csi2_uninit();

        memset(&devp->para, 0, sizeof(vdin_parm_t));
}
static struct tvin_state_machine_ops_s amcsi_machine_ops = {
        .nosig               = NULL,
        .fmt_changed         = NULL,
        .get_fmt             = NULL,
        .fmt_config          = NULL,
        .adc_cal             = NULL,
        .pll_lock            = NULL,
        .get_sig_propery     = amcsi_get_sig_propery,
        .vga_set_param       = NULL,
        .vga_get_param       = NULL,
        .check_frame_skip    = amcsi_check_skip_frame,
};
static struct tvin_decoder_ops_s amcsi_decoder_ops_s = {
        .support                = amcsi_support,
        .open                   = amcsi_feopen,
        .start                  = amcsi_start,
        .stop                   = amcsi_stop,
        .close                  = amcsi_feclose,
        .decode_isr             = amcsi_isr,
};

static int csi_add_cdev(struct cdev *cdevp,struct file_operations *fops,int minor)
{
        int ret;
        dev_t devno=MKDEV(MAJOR(amcsi_devno),minor);
        cdev_init(cdevp,fops);
        cdevp->owner=THIS_MODULE;
        ret=cdev_add(cdevp,devno,1);
        return ret;
}


static struct device *csi_create_device(struct device *parent,int minor)
{
        dev_t devno = MKDEV(MAJOR(amcsi_devno),minor);
        return  device_create(amcsi_clsp,parent,devno,NULL,"%s%d",
                        DEV_NAME,minor);
}

static void csi_delete_device(int minor)
{
        dev_t devno =MKDEV(MAJOR(amcsi_devno),minor);
        device_destroy(amcsi_clsp,devno);
}

static int amvdec_csi_probe(struct platform_device *pdev)
{
        int ret;
        struct amcsi_dev_s *devp;

        ret = 0;

        //malloc dev
        devp = kmalloc(sizeof(struct amcsi_dev_s),GFP_KERNEL);
        if(!devp){
                pr_err("%s: failed to allocate memory\n", __func__);
                goto fail_kmalloc_dev;
        }
        memset(devp,0,sizeof(struct amcsi_dev_s));

        //create cdev and register with sysfs
        ret = csi_add_cdev(&devp->cdev, &amcsi_fops, 0);
        if (ret) {
                pr_err("%s: failed to add cdev\n", __func__);
                goto fail_add_cdev;
        }
        devp->dev = csi_create_device(&pdev->dev, 0);
        if (IS_ERR(devp->dev)) {
                pr_err("%s: failed to create device\n", __func__);
                ret = PTR_ERR(devp->dev);
                goto fail_create_device;
        }
        ret = device_create_file(devp->dev,&dev_attr_hw_info);

        /*register frontend */
        sprintf(devp->frontend.name, "%s", DEV_NAME);
        tvin_frontend_init(&devp->frontend, &amcsi_decoder_ops_s, &amcsi_machine_ops, pdev->id);
        tvin_reg_frontend(&devp->frontend);
        /*set pinmux for ITU601 A and ITU601 B*/
        /* set drvdata */
        dev_set_drvdata(devp->dev, devp);
        platform_set_drvdata(pdev, devp);
        printk("amvdec_csi probe ok.\n");
        return ret;
fail_create_device:
        cdev_del(&devp->cdev);
fail_add_cdev:
        kfree(devp);
fail_kmalloc_dev:
        return ret;

}

static int amvdec_csi_remove(struct platform_device *pdev)
{
        struct amcsi_dev_s *devp;

        devp = (struct amcsi_dev_s *)platform_get_drvdata(pdev);

        tvin_unreg_frontend(&devp->frontend);

        device_remove_file(devp->dev,&dev_attr_hw_info);

        csi_delete_device(pdev->id);
        cdev_del(&devp->cdev);
        kfree((const void *)devp);
        /* free drvdata */
        dev_set_drvdata(devp->dev, NULL);
        platform_set_drvdata(pdev, NULL);
        return 0;
}

static struct platform_driver amvdec_csi_driver = {
        .probe      = amvdec_csi_probe,
        .remove     = amvdec_csi_remove,
        .driver     = {
                .name   = DRV_NAME,
        }
};

static int __init amvdec_csi_init_module(void)
{
        int ret = 0;
        struct platform_device *pdev;
        printk("amvdec_csi module: init.\n");
        ret=alloc_chrdev_region(&amcsi_devno, 0, CSI_MAX_DEVS, DEV_NAME);
        if(ret<0){
                printk("%s:failed to alloc major number\n",__func__);
                goto fail_alloc_cdev_region;
        }
        printk("%s:major %d\n",__func__,MAJOR(amcsi_devno));
        amcsi_clsp=class_create(THIS_MODULE,CLS_NAME);
        if(IS_ERR(amcsi_clsp)){
                ret=PTR_ERR(amcsi_clsp);
                printk("%s:failed to create class\n",__func__);
                goto fail_class_create;
        }
        pdev = platform_device_alloc(DEV_NAME,0);
        if(IS_ERR(pdev)){
                printk("[csi..]%s alloc platform device error.\n", __func__);
                goto fail_pdev_create;
        }
        if(platform_device_add(pdev)){
                printk("[csi..]%s failed register platform device.\n", __func__);
                goto fail_pdev_register;
        }
        if (0 != platform_driver_register(&amvdec_csi_driver)){
                printk("failed to register amvdec_csi driver\n");
                goto fail_pdrv_register;
        }

        printk("amvdec_csi module: init. ok\n");
        return 0;
fail_pdrv_register:
        platform_device_unregister(pdev);
fail_pdev_register:
        platform_device_del(pdev);
fail_pdev_create:
        class_destroy(amcsi_clsp);
fail_class_create:
        unregister_chrdev_region(amcsi_devno,CSI_MAX_DEVS);
fail_alloc_cdev_region:
        printk("amvdec_csi module: init failed, ret=%d\n", ret);
        return ret;

}

static void __exit amvdec_csi_exit_module(void)
{
        printk("amvdec_csi module remove.\n");
        printk("%s, %d\n", __func__, __LINE__);
        class_destroy(amcsi_clsp);
        unregister_chrdev_region(amcsi_devno, CSI_MAX_DEVS);
        platform_driver_unregister(&amvdec_csi_driver);
        return ;
}

module_init(amvdec_csi_init_module);
module_exit(amvdec_csi_exit_module);

MODULE_DESCRIPTION("AMLOGIC CSI input driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
