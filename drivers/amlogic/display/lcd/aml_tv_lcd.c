/*
 * AMLOGIC lcd controller driver.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <mach/register.h>
#include <mach/am_regs.h>
#include <linux/amlogic/vout/vinfo.h>
#include <linux/amlogic/vout/vout_notify.h>
#include <linux/amlogic/display/lcd.h>

#include "aml_tv_lcd.h"

#define PANEL_NAME		"panel"

extern unsigned int lvds_init(Lcd_Config_t *pConf);
extern unsigned int vbyone_init(Lcd_Config_t *pConf);

extern void _disable_display_driver(void);

static char lcd_propname[30] = "lvds_0";

static const char* lcd_type_table[]={
	"LVDS",
	"vbyone",
	"TTL",
	"invalid",
};

unsigned int (*init_lcd_port[])(Lcd_Config_t *pConf) = {
	lvds_init,
	vbyone_init,
};

//**** Special parameters just for Vbyone ***//
static Vbyone_Config_t lcd_vbyone_config={
	.lane_count = 8,	//lane:  1/2/4/6/8 lanes;
	.byte		= 4,	//byte:  3/4/5 bytes;
	.region		= 2,	//region
	.color_fmt	= 4,	//color_fmt
};

//**** Special parameters just for lvds ***//
static Lvds_Config_t lcd_lvds_config={
	.lvds_bits		= 8,	//6/8/10 bit
	.lvds_repack	= 1,	//0->JEDIA mode,  1->VESA mode
	.pn_swap		= 0,	//0->normal,         1->swap
	.dual_port		= 1,	//0->single lvds,	1->double lvds
	.port_reverse	= 1,
	.lvds_fifo_wr_mode	= 0x3, //fifo:0x3->double,  0x101->single
};

//****panel power control only for uboot ***//
static Panel_Power_Config_t lcd_panel_power =
{
	.gpio		=	GPIOH_10,		/** panel power control gpio port */
	.on_value	=	1,				/** panel power on gpio out value*/
	.off_value	=	0,				/** panel power off gpio out value*/
	.panel_on_delay		=	50,		/** panel power on delay time (unit: ms)*/
	.panel_off_delay	=	50 		/** panel power off delay time (unit: ms)*/
};

Lcd_Config_t lcd_config_dft =
{
	.lcd_basic = {
		.lcd_type = LCD_DIGITAL_LVDS,	//LCD_DIGITAL_TTL /LCD_DIGITAL_LVDS/LCD_DIGITAL_VBYONE
		.h_active = 1920,
		.v_active = 1080,
		.h_period = 2200,
		.v_period = 1125,
		.video_on_pixel = 148,
		.video_on_line  = 41,
		.screen_ratio_width   = 16,
		.screen_ratio_height  = 9,
	},

	.lcd_timing = {
		.hpll_clk = 0x500404ad,  //0x10c8 : N->bits[13:9]       M->bits[8:0]
		.hpll_od = 0x00414400,   //0x10c9 : od1->bits[17:16]   od2->bits[23:22]   od3->bits[19:18]
		.hdmi_pll_cntl5 = 0x71486900,

		.sth1_hs_addr = 44,
		.sth1_he_addr = 2156,
		.sth1_vs_addr = 0,
		.sth1_ve_addr = 1125 - 1,
		.stv1_hs_addr = 2100,
		.stv1_he_addr = 2164,
		.stv1_vs_addr = 3,
		.stv1_ve_addr = 5,
	},

	.lcd_control = {
		.lvds_config	=	&lcd_lvds_config,
		.vbyone_config	=	&lcd_vbyone_config,
	},

	.lcd_power_ctrl = {
		.panel_power	=	&lcd_panel_power,
	},
};


#ifdef CONFIG_USE_OF
static inline int aml_lcd_get_property_string(struct device_node *np,
									   const char *propname,
									   const char **out_string)
{
	int ret;

	ret = of_property_read_string(np, propname, out_string);
	if (ret) {
		TV_LCD_ERR("faild to get %s !\n",propname);
		*out_string = "invalid";
	}

	return ret;
}

static inline int aml_lcd_get_property_array(struct device_node* of_node,
												const char *propname,
												u32 *out_values, size_t sz)
{
	int ret;

	ret= of_property_read_u32_array(of_node,propname,out_values,sz);
	if (ret)
		TV_LCD_ERR("faild to get %s !\n",propname);

	return ret;
}

static int _get_lcd_config(struct platform_device *pdev)
{
	struct aml_lcd *pDev;
	struct device_node* of_node = pdev->dev.of_node;
	struct device_node* child;
	const char *str;
	unsigned int val;
	unsigned int *lcd_para = NULL;

	pDev = platform_get_drvdata(pdev);

	lcd_para = (unsigned int *)kmalloc(sizeof(unsigned int)*20, GFP_KERNEL);

	memset(lcd_para, 0, sizeof(*lcd_para));
	if (lcd_para == NULL) {
		kfree(lcd_para);
		TV_LCD_ERR("Not enough memory\n");
		return -EINVAL;
	}

	if (of_node) {
		child = of_get_child_by_name(of_node,lcd_propname);
		if (child == NULL) {
			kfree(lcd_para);
			TV_LCD_ERR("faild to get lcd_model_config!! \n");
			return -EINVAL;
		}

		if (!aml_lcd_get_property_string(child, "interface", &str)) {
			for (val = 0; val < LCD_TYPE_MAX; val++) {
				if (!strcasecmp(str, lcd_type_table[val]))
					break;
			}
			pDev->pConf->lcd_basic.lcd_type = val;
		}

		if (!aml_lcd_get_property_array(child, "basic_setting", &lcd_para[0], 6)) {
			pDev->pConf->lcd_basic.h_active = lcd_para[0];
			pDev->pConf->lcd_basic.v_active = lcd_para[1];
			pDev->pConf->lcd_basic.h_period = lcd_para[2];
			pDev->pConf->lcd_basic.v_period = lcd_para[3];
			pDev->pConf->lcd_basic.video_on_pixel = lcd_para[4];
			pDev->pConf->lcd_basic.video_on_line = lcd_para[5];
		}

		if (!aml_lcd_get_property_array(child, "lcd_timing", &lcd_para[0], 11)) {
			pDev->pConf->lcd_timing.hpll_clk = lcd_para[0];
			pDev->pConf->lcd_timing.hpll_od  = lcd_para[1];
			pDev->pConf->lcd_timing.hdmi_pll_cntl5 = lcd_para[2];
			pDev->pConf->lcd_timing.sth1_hs_addr	 = lcd_para[3];
			pDev->pConf->lcd_timing.sth1_he_addr	 = lcd_para[4];
			pDev->pConf->lcd_timing.sth1_vs_addr	 = lcd_para[5];
			pDev->pConf->lcd_timing.sth1_ve_addr	 = lcd_para[6];
			pDev->pConf->lcd_timing.stv1_hs_addr	 = lcd_para[7];
			pDev->pConf->lcd_timing.stv1_he_addr	 = lcd_para[8];
			pDev->pConf->lcd_timing.stv1_vs_addr	 = lcd_para[9];
			pDev->pConf->lcd_timing.stv1_ve_addr	 = lcd_para[10];
		}

		if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_LVDS) {
			if (!aml_lcd_get_property_array(child, "lvds_att", &lcd_para[0], 6)) {
				pDev->pConf->lcd_control.lvds_config->lvds_bits	 = lcd_para[0];
				pDev->pConf->lcd_control.lvds_config->lvds_repack  = lcd_para[1];
				pDev->pConf->lcd_control.lvds_config->pn_swap 	 = lcd_para[2];
				pDev->pConf->lcd_control.lvds_config->dual_port	 = lcd_para[3];
				pDev->pConf->lcd_control.lvds_config->port_reverse		 = lcd_para[4];
				pDev->pConf->lcd_control.lvds_config->lvds_fifo_wr_mode	 = lcd_para[5];
			}
		} else if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_VBYONE) {
			if (!aml_lcd_get_property_array(child, "vbyone_att", &lcd_para[0], 4)) {
				pDev->pConf->lcd_control.vbyone_config->lane_count	 = lcd_para[0];
				pDev->pConf->lcd_control.vbyone_config->byte		= lcd_para[1];
				pDev->pConf->lcd_control.vbyone_config->region		= lcd_para[2];
				pDev->pConf->lcd_control.vbyone_config->color_fmt 	= lcd_para[3];
			}
		} else if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_TTL) {

		}
	}

	TV_LCD_INFO("lcd_type = %s(%s)\n", lcd_type_table[pDev->pConf->lcd_basic.lcd_type],lcd_propname);
	TV_LCD_INFO("h_active = %u, v_active = %u \n", pDev->pConf->lcd_basic.h_active, pDev->pConf->lcd_basic.v_active);
	TV_LCD_INFO("h_period = %u, v_period = %u \n", pDev->pConf->lcd_basic.h_period, pDev->pConf->lcd_basic.v_period );
	TV_LCD_INFO("video_on_pixel = %u, video_on_line = %u \n", pDev->pConf->lcd_basic.video_on_pixel, pDev->pConf->lcd_basic.video_on_line);
	TV_LCD_INFO("hpll_clk = %x, hpll_od =%x hdmi_pll_cntl5 = %x \n", pDev->pConf->lcd_timing.hpll_clk, pDev->pConf->lcd_timing.hpll_od, pDev->pConf->lcd_timing.hdmi_pll_cntl5);
	TV_LCD_INFO("sth1_hs_addr = %u, sth1_he_addr = %u \n", pDev->pConf->lcd_timing.sth1_hs_addr, pDev->pConf->lcd_timing.sth1_he_addr);
	TV_LCD_INFO("sth1_vs_addr = %u, sth1_ve_addr = %u \n", pDev->pConf->lcd_timing.sth1_vs_addr, pDev->pConf->lcd_timing.sth1_ve_addr);
	TV_LCD_INFO("stv1_hs_addr = %u, stv1_he_addr = %u \n", pDev->pConf->lcd_timing.stv1_hs_addr, pDev->pConf->lcd_timing.stv1_he_addr);
	TV_LCD_INFO("stv1_vs_addr = %u, stv1_ve_addr = %u \n", pDev->pConf->lcd_timing.stv1_vs_addr, pDev->pConf->lcd_timing.stv1_ve_addr);

	if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_LVDS) {
		TV_LCD_INFO("lvds_bits = %d \n",pDev->pConf->lcd_control.lvds_config->lvds_bits);
		TV_LCD_INFO("lvds_repack = %d \n",pDev->pConf->lcd_control.lvds_config->lvds_repack);
		TV_LCD_INFO("pn_swap = %d \n",pDev->pConf->lcd_control.lvds_config->pn_swap);
		TV_LCD_INFO("dual_port = %d \n",pDev->pConf->lcd_control.lvds_config->dual_port);
		TV_LCD_INFO("port_reverse = %d \n",pDev->pConf->lcd_control.lvds_config->port_reverse);
		TV_LCD_INFO("lvds_fifo_wr_mode = %d \n",pDev->pConf->lcd_control.lvds_config->lvds_fifo_wr_mode);
	} else if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_VBYONE) {
		TV_LCD_INFO("lvds_bits = %d \n",pDev->pConf->lcd_control.lvds_config->lvds_bits);
		TV_LCD_INFO("lvds_repack = %d \n",pDev->pConf->lcd_control.lvds_config->lvds_repack);
		TV_LCD_INFO("pn_swap = %d \n",pDev->pConf->lcd_control.lvds_config->pn_swap);
		TV_LCD_INFO("dual_port = %d \n",pDev->pConf->lcd_control.lvds_config->dual_port);
		TV_LCD_INFO("port_reverse = %d \n",pDev->pConf->lcd_control.lvds_config->port_reverse);
		TV_LCD_INFO("lvds_fifo_wr_mode = %d \n",pDev->pConf->lcd_control.lvds_config->lvds_fifo_wr_mode);
	} else if (pDev->pConf->lcd_basic.lcd_type == LCD_DIGITAL_TTL) {

	}

	kfree(lcd_para);

	return 0;
}

static int _get_lcd_power_config(struct platform_device *pdev)
{
	struct aml_lcd *pDev;
	struct device_node* of_node = pdev->dev.of_node;
	struct device_node* child;

	const char *str;
	unsigned int panel_power_pin;
	unsigned int *lcd_para = NULL;

	pDev = platform_get_drvdata(pdev);

	lcd_para = (unsigned int *)kmalloc(sizeof(unsigned int)*20, GFP_KERNEL);

	if (lcd_para == NULL) {
		TV_LCD_ERR("Not enough memory\n");
		return -EINVAL;
	}
	memset(lcd_para, 0, sizeof(*lcd_para));

	if (of_node) {
		child = of_get_child_by_name(of_node,lcd_propname);
		if (child == NULL) {
			kfree(lcd_para);
			TV_LCD_ERR("faild to get lcd_model_config!! \n");
			return -EINVAL;
		}
		if (!aml_lcd_get_property_string(child, "panel_power_pin", &str)) {
			panel_power_pin = amlogic_gpio_name_map_num(str);
			if (panel_power_pin<0) {
				kfree(lcd_para);
				TV_LCD_ERR("wrong gpio number %s\n",str);
				return -EINVAL;
			}
			pDev->pConf->lcd_power_ctrl.panel_power->gpio	= panel_power_pin;
		}

		if (!aml_lcd_get_property_array(child, "panel_power_att", &lcd_para[0], 4)) {
			pDev->pConf->lcd_power_ctrl.panel_power->on_value  = lcd_para[0];
			pDev->pConf->lcd_power_ctrl.panel_power->off_value = lcd_para[1];
			pDev->pConf->lcd_power_ctrl.panel_power->panel_on_delay = lcd_para[2];
			pDev->pConf->lcd_power_ctrl.panel_power->panel_off_delay = lcd_para[3];
		}

		TV_LCD_INFO("panel_power_pin: %s--%d \n",str,pDev->pConf->lcd_power_ctrl.panel_power->gpio);
		TV_LCD_INFO("on_value = %d \n",pDev->pConf->lcd_power_ctrl.panel_power->on_value);
		TV_LCD_INFO("off_value = %d \n",pDev->pConf->lcd_power_ctrl.panel_power->off_value);
		TV_LCD_INFO("panel_on_delay = %d \n",pDev->pConf->lcd_power_ctrl.panel_power->panel_on_delay);
		TV_LCD_INFO("panel_off_delay = %d \n",pDev->pConf->lcd_power_ctrl.panel_power->panel_off_delay);
	}

	kfree(lcd_para);

return 0;

}
#endif

static void panel_power_ctrl(Bool_t status)
{
	const char *owner = "aml_tv_lcd";
	Lcd_Config_t *pConf;
	pConf = &lcd_config_dft;

	TV_LCD_INFO("statu=%s gpio=%d value=%d \n",(status ? "ON" : "OFF"),
		pConf->lcd_power_ctrl.panel_power->gpio,
		(status ?pConf->lcd_power_ctrl.panel_power->on_value:
				pConf->lcd_power_ctrl.panel_power->off_value));

	if (ON == status) {
		amlogic_gpio_request(pConf->lcd_power_ctrl.panel_power->gpio,owner);
		amlogic_gpio_direction_output(pConf->lcd_power_ctrl.panel_power->gpio,
									   pConf->lcd_power_ctrl.panel_power->on_value,owner);
		mdelay(pConf->lcd_power_ctrl.panel_power->panel_on_delay);

	} else {
		amlogic_gpio_request(pConf->lcd_power_ctrl.panel_power->gpio,owner);
		amlogic_gpio_direction_output(pConf->lcd_power_ctrl.panel_power->gpio,
									  pConf->lcd_power_ctrl.panel_power->off_value,owner);
		mdelay(pConf->lcd_power_ctrl.panel_power->panel_off_delay);
	}
}

static inline void _init_display_driver(Lcd_Config_t *pConf_t)
{
	if (pConf_t->lcd_basic.lcd_type < LCD_TYPE_MAX) {
		init_lcd_port[pConf_t->lcd_basic.lcd_type](pConf_t);
	} else {
		TV_LCD_ERR("no lcd port\n");
		init_lcd_port[LCD_DIGITAL_LVDS](pConf_t);
	}
}

static void _lcd_module_enable(void)
{
	_init_display_driver(&lcd_config_dft);
}

DEFINE_MUTEX(lcd_vout_mutex);

static const vinfo_t *lcd_get_current_info(void)
{
   return NULL;
}


static int lcd_set_current_vmode(vmode_t mode)
{
    mutex_lock(&lcd_vout_mutex);
    if (mode != VMODE_LCD) {
        mutex_unlock(&lcd_vout_mutex);
        return -EINVAL;
    }

    _lcd_module_enable();
    mutex_unlock(&lcd_vout_mutex);

    return 0;
}

static vmode_t lcd_validate_vmode(char *mode)
{
    if (mode == NULL)
        return VMODE_MAX;

    if ((strncmp(mode, PANEL_NAME, strlen(PANEL_NAME))) == 0)
        return VMODE_LCD;

    return VMODE_MAX;
}

static int lcd_vmode_is_supported(vmode_t mode)
{
    mode &= VMODE_MODE_BIT_MASK;
    if ( mode == VMODE_LCD )
		return true;

    return false;
}

static int lcd_vout_disable(vmode_t cur_vmod)
{
    return 0;
}

#ifdef  CONFIG_PM
static int lcd_suspend(void)
{
	TV_LCD_INFO("lcd_suspend \n");
    /* in freeze process do not turn off the display devices */

    _disable_display_driver();

    return 0;
}
static int lcd_resume(void)
{
    TV_LCD_INFO("lcd_resume\n");
    /* in thaw/restore process do not reset the display mode */

    lcd_set_current_vmode(VMODE_LCD);

    return 0;
}
#endif

static vout_server_t lcd_vout_server={
    .name = "lcd_vout_server",
    .op = {
        .get_vinfo = lcd_get_current_info,
        .set_vmode = lcd_set_current_vmode,
        .validate_vmode = lcd_validate_vmode,
        .vmode_is_supported=lcd_vmode_is_supported,
        .disable=lcd_vout_disable,
#ifdef  CONFIG_PM
        .vout_suspend=lcd_suspend,
        .vout_resume=lcd_resume,
#endif
    },
};

static void _init_vout(struct platform_device *pdev)
{
	struct aml_lcd *pDev;
	pDev = platform_get_drvdata(pdev);

    pDev->lcd_info.name = PANEL_NAME;
    pDev->lcd_info.mode = VMODE_LCD;
    pDev->lcd_info.width = pDev->pConf->lcd_basic.h_active;
    pDev->lcd_info.height = pDev->pConf->lcd_basic.v_active;
    pDev->lcd_info.field_height = pDev->pConf->lcd_basic.v_active;
    pDev->lcd_info.aspect_ratio_num = pDev->pConf->lcd_basic.screen_ratio_width;
    pDev->lcd_info.aspect_ratio_den = pDev->pConf->lcd_basic.screen_ratio_height;
    pDev->lcd_info.screen_real_width= pDev->pConf->lcd_basic.h_active_area;
    pDev->lcd_info.screen_real_height= pDev->pConf->lcd_basic.v_active_area;
    pDev->lcd_info.sync_duration_num = pDev->pConf->lcd_timing.sync_duration_num;
    pDev->lcd_info.sync_duration_den = pDev->pConf->lcd_timing.sync_duration_den;

	aml_write_reg32(P_VPP2_POSTBLEND_H_SIZE, pDev->lcd_info.width);

	vout_register_server(&lcd_vout_server);
}

static int lcd_reboot_notifier(struct notifier_block *nb, unsigned long state, void *cmd)
 {
	TV_LCD_INFO("state=%lu\n",state);
//	if (pDev->pConf->lcd_misc_ctrl.lcd_status == 0)
//		return NOTIFY_DONE;
	lcd_notifier_call_chain(LCD_EVENT_POWEROFF, NULL);
	_disable_display_driver();
	panel_power_ctrl(OFF);

	return NOTIFY_OK;
}


static struct notifier_block lcd_reboot_nb;
static int lcd_probe(struct platform_device *pdev)
{
	unsigned int ret = 0;
	struct aml_lcd *pDev;

	pDev = (struct aml_lcd *)kmalloc(sizeof(struct aml_lcd), GFP_KERNEL);
	if (!pDev) {
		TV_LCD_ERR("lcd error: Not enough memory.\n");
		return -ENOMEM;
	}
	memset(pDev, 0, sizeof(*pDev));

	platform_set_drvdata(pdev, pDev);
	pDev->pdev = pdev;

	pDev->pConf = &lcd_config_dft;
	pDev->pConf->lcd_control.lvds_config = (Lvds_Config_t *)(lcd_config_dft.lcd_control.lvds_config);
	pDev->pConf->lcd_control.vbyone_config = (Vbyone_Config_t *)(lcd_config_dft.lcd_control.vbyone_config);
	pDev->pConf->lcd_power_ctrl.panel_power = (Panel_Power_Config_t *)(lcd_config_dft.lcd_power_ctrl.panel_power);

	if (_get_lcd_config(pdev)) {
		kfree(pDev);
		TV_LCD_ERR("can not find lcd dtd config \n");
		return -ENOMEM;
	}
	if (_get_lcd_power_config(pdev)) {
		kfree(pDev);
		TV_LCD_ERR("can not find panel power dtd config \n");
		return -ENOMEM;
	}

	//panel_power_ctrl(ON);
	//udelay(50);
	_init_vout(pdev);
	//lcd_set_current_vmode(VMODE_LCD);
	//lcd_notifier_call_chain(LCD_EVENT_POWERON, NULL);

	lcd_reboot_nb.notifier_call = lcd_reboot_notifier;
	ret = register_reboot_notifier(&lcd_reboot_nb);
	if (ret) {
		TV_LCD_ERR("notifier register lcd_reboot_notifier fail!\n");
	}

	TV_LCD_INFO("LCD probe ok\n");

	return 0;
}

static int lcd_remove(struct platform_device *pdev)
{
	struct aml_lcd *pDev = platform_get_drvdata(pdev);

	unregister_reboot_notifier(&lcd_reboot_nb);
	platform_set_drvdata(pdev, NULL);
	if (pDev)
		kfree(pDev);

    return 0;
}

#ifdef CONFIG_USE_OF
  static const struct of_device_id lcd_dt_match[] = {
	  {
		  .compatible = "amlogic,lcd",
	  },
	  {},
  };
#endif

static struct platform_driver lcd_driver = {
	 .probe = lcd_probe,
	 .remove = lcd_remove,
	 .driver = {
		 .name = "mesonlcd",
		 .owner = THIS_MODULE,
#ifdef CONFIG_USE_OF
		 .of_match_table = lcd_dt_match,
#endif
	 },
 };

static int __init lcd_init(void)
{
	 printk("LCD driver init\n");
	 if (platform_driver_register(&lcd_driver)) {
		 printk("failed to register lcd driver module\n");
		 return -ENODEV;
	 }

	 return 0;
}

static void __exit lcd_exit(void)
{
	platform_driver_unregister(&lcd_driver);
}

subsys_initcall(lcd_init);
module_exit(lcd_exit);


static int __init lcd_boot_para_setup(char *s)
{
	if (NULL != s) {
		sprintf(lcd_propname, "%s", s);
	}

	return 0;
}
 __setup("panel_type=",lcd_boot_para_setup);


 MODULE_DESCRIPTION("Meson LCD Panel Driver");
 MODULE_LICENSE("GPL");
 MODULE_AUTHOR("Amlogic, Inc.");
