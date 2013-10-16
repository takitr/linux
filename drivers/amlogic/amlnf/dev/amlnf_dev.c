/*
 * Aml nftl dev
 *
 * (C) 2012 8
 */

#include "../include/amlnf_dev.h"

#define CONFIG_OF

#ifndef AML_NAND_UBOOT
int boot_device_flag = -1;
#endif

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/

#ifndef AML_NAND_UBOOT
/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static ssize_t nfdev_debug(struct class *class,struct class_attribute *attr,char *buf)
{
    //struct amlnf_dev* nf_dev = container_of(class, struct amlnf_dev, debug);

    //print_nftl_part(nf_dev -> aml_nftl_part);

    return 0;
}

static struct class_attribute phydev_class_attrs[] = {
    __ATTR(info,       S_IRUGO | S_IWUSR, show_nand_info,    NULL),	
    __ATTR(verify,       S_IRUGO | S_IWUSR, NULL,    verify_nand_page),
    __ATTR(dump,       S_IRUGO | S_IWUSR, NULL,    dump_nand_page),
    __ATTR(bbt_table,       S_IRUGO | S_IWUSR, NULL,    show_bbt_table),    
    __ATTR(page_read,  S_IRUGO | S_IWUSR, NULL,    nand_page_read),  
    __ATTR(page_write,  S_IRUGO | S_IWUSR, NULL,    nand_page_write),
    __ATTR(version,       S_IRUGO | S_IWUSR, show_amlnf_version_info,    NULL),
    __ATTR_NULL
};

static struct class_attribute logicdev_class_attrs[] = {
    __ATTR(part,  S_IRUGO , show_part_struct,    NULL),
    __ATTR(list,  S_IRUGO , show_list,    NULL),
    __ATTR(gcall,  S_IRUGO , do_gc_all,    NULL),
    __ATTR(gcone,  S_IRUGO , do_gc_one,    NULL),
    __ATTR(test,  S_IRUGO | S_IWUSR , NULL,    do_test),
    __ATTR_NULL
};

static struct class_attribute nfdev_class_attrs[] = {
    __ATTR(debug,  S_IRUGO , nfdev_debug,    NULL),
    __ATTR_NULL
};

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int phydev_cls_suspend(struct device *dev, pm_message_t state)
{
	
	return 0;
	
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int phydev_cls_resume(struct device *dev, pm_message_t state)
{
		return 0;
}


static struct class phydev_class = {
	.name = "amlphydev",
	.owner = THIS_MODULE,
	.suspend = phydev_cls_suspend,
	.resume = phydev_cls_resume,
};

int amlnf_pdev_register(struct amlnand_phydev *phydev)
{
	int ret = 0;

	//phydev->dev.class = &phydev_class;
	dev_set_name(&phydev->dev, phydev->name,0); 	
	dev_set_drvdata(&phydev->dev, phydev);		
	ret = device_register(&phydev->dev);		
	if (ret != 0){
		aml_nand_msg("device register failed for %s", phydev->name);
		aml_nand_free(phydev);
		goto exit_error0;
	}

	phydev->cls.name = aml_nand_malloc(strlen((const char*)phydev->name)+8);
	snprintf(phydev->cls.name, (MAX_DEVICE_NAME_LEN+8),
	  	 "%s%s", "phy_", (char *)(phydev->name));
	phydev->cls.class_attrs = phydev_class_attrs;
	ret = class_register(&phydev->cls);
	if(ret){
		aml_nand_msg(" class register nand_class fail for %s", phydev->name);	
		goto exit_error1;
	}

	return 0;

exit_error1:
	aml_nand_free(phydev->cls.name);
exit_error0:
	return ret;
}


static int amlnf_blk_open(struct block_device *bdev, fmode_t mode)
{
	return 0;
}

static int amlnf_blk_release(struct gendisk *disk, fmode_t mode)
{
	return 0;
}


static int amlnf_blk_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	return 0;
}

static int amlnf_blk_ioctl(struct block_device *bdev, fmode_t mode,
			      unsigned int cmd, unsigned long arg)
{
	return 0;
}


static const struct block_device_operations amlnf_blk_ops = {
	.owner		= THIS_MODULE,
	.open		= amlnf_blk_open,
	.release	= amlnf_blk_release,
	.ioctl		= amlnf_blk_ioctl,
	.getgeo		= amlnf_blk_getgeo,
};

#define blk_queue_plugged(q)    test_bit(18, &(q)->queue_flags)

#endif

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
 int amlnf_logic_init(unsigned flag)
 {
	 struct amlnand_phydev *phydev = NULL;
	 int i,ret=0;
	 aml_nand_msg("amlnand_add_nftl:");
	 //amlnand_show_dev_partition(aml_chip);
	 list_for_each_entry(phydev, &nphy_dev_list, list){
		 if (phydev!=NULL){
		 	if (strncmp((char*)phydev->name, NAND_BOOT_NAME, strlen((const char*)NAND_BOOT_NAME)))
				{			 
				ret = add_ntd_partitions(phydev);
				if(ret < 0){					 
					aml_nand_msg("nand add nftl failed");					 
					goto exit_error;
					}
				}
			if(!strncmp((char*)phydev->name, NAND_BOOT_NAME, strlen((const char*)NAND_BOOT_NAME))){
				ret = boot_device_register(phydev);
				if(ret < 0){	
					aml_nand_msg("boot device registe failed");
					goto exit_error;
					}
				}
			}
	 }
 exit_error:	

		return ret;
 }



/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
int amlnf_dev_init(unsigned flag)
{
	struct amlnand_phydev *phydev = NULL;
	struct amlnf_dev* nf_dev = NULL;
	
	int ret = 0;
	
#ifndef AML_NAND_UBOOT
	list_for_each_entry(phydev, &nphy_dev_list, list){
		if ((phydev != NULL)  && 
			(strncmp((char*)phydev->name, NAND_BOOT_NAME, strlen((const char*)NAND_BOOT_NAME)))){				
			ret = amlnf_pdev_register(phydev);
			if(ret < 0){
				aml_nand_msg("nand add nftl failed");
				goto exit_error0;
			}			
		}
	}

#endif

	return 0;

exit_error0:
	return ret;	
}

#ifdef AML_NAND_UBOOT
static int get_boot_device()
{

	
	if(POR_SPI_BOOT()){
		boot_device_flag = 0; // spi boot 
		aml_nand_msg("SPI BOOT: boot_device_flag %d",boot_device_flag);
		return 0;
	}

	if(POR_NAND_BOOT()){
		boot_device_flag = 1; // nand boot
		aml_nand_msg("NAND BOOT: boot_device_flag %d",boot_device_flag);
		return 0;
	}

	if(POR_EMMC_BOOT()){
		boot_device_flag = -1;
		aml_nand_msg("EMMC BOOT: not init nand");
		return -1;
	}
	if(POR_CARD_BOOT()){
		boot_device_flag = -1;
		aml_nand_msg("CARD BOOT: not init nand");
		return -1;
	}
	
	return ;
}
struct amlnand_phydev *aml_phy_get_dev(char * name)
{
	struct amlnand_phydev * phy_dev = NULL;
	
	list_for_each_entry(phy_dev, &nphy_dev_list, list){
			if(!strncmp((char*)phy_dev->name, name, MAX_DEVICE_NAME_LEN)){
				aml_nand_dbg("nand get phy dev %s ",name);
				return phy_dev;
			}
		}
		
	aml_nand_msg("nand get phy dev %s	failed",name);
	
	return NULL;
}


struct amlnf_dev* aml_nftl_get_dev(char * name)
{
	struct amlnf_dev * nf_dev = NULL;

	list_for_each_entry(nf_dev, &nf_dev_list, list){
		if(!strncmp((char*)nf_dev->name, name, MAX_NAND_PART_NAME_LEN)){
			aml_nand_dbg("nand get nftl dev %s ",name);
			return nf_dev;
		}
	}
	
	aml_nand_msg("nand get nftl dev %s  failed",name);
	
	return NULL;
}
#endif

#ifdef CONFIG_OF
static const struct of_device_id amlogic_nand_dt_match[]={
	{	.compatible = "amlogic,aml_nand",
	},
	{},
};
static inline struct aml_nand_device   *aml_get_driver_data(
			struct platform_device *pdev)
{
	if (pdev->dev.of_node) {
		const struct of_device_id *match;
		match = of_match_node(amlogic_nand_dt_match, pdev->dev.of_node);
		return (struct aml_nand_device *)match->data;
	}
}

static int get_nand_platform(struct aml_nand_device *aml_nand_dev,struct platform_device *pdev)
{
	int ret;
	const char *name,*propname;
	struct property *prop;
	const __be32 *list;
	int size,config,index;
	int selector,match_mode;
	const char *select;
	phandle phandle;
	int val=0,plat_num=0;
	unsigned char  plat_name[16];
	struct device_node *np_config;
	struct device_node *np_part;
	struct device_node *np = pdev->dev.of_node;
	
	if(pdev->dev.of_node){
		of_node_get(np);
		ret = of_property_read_u32(np,"plat-num",&plat_num);
		if(ret){
			printk("%s:%d,please config plat-num item\n",__func__,__LINE__);
			return -1;
		}
	}
	aml_nand_dbg("plat_num %d ",plat_num);
	
	return 0;
err:
	return -1;
}

#endif

#ifndef AML_NAND_UBOOT

#define R_BOOT_DEVICE_FLAG  READ_CBUS_REG(ASSIST_POR_CONFIG)

#ifdef CONFIG_NAND_AML_M8
#define POR_BOOT_VALUE 	((((R_BOOT_DEVICE_FLAG>>9)&1)<<2)|((R_BOOT_DEVICE_FLAG>>6)&3))
#else
#define POR_BOOT_VALUE 	(R_BOOT_DEVICE_FLAG & 7)
#endif

#define POR_NAND_BOOT()	 ((POR_BOOT_VALUE == 7) || (POR_BOOT_VALUE == 6))
#define POR_SPI_BOOT()  		((POR_BOOT_VALUE == 5) || (POR_BOOT_VALUE == 4))
#define POR_EMMC_BOOT()	 (POR_BOOT_VALUE == 3)
#define POR_CARD_BOOT() 	(POR_BOOT_VALUE == 0)


#define SPI_BOOT_FLAG 			0
#define NAND_BOOT_FLAG 		1
#define EMMC_BOOT_FLAG 		2
#define CARD_BOOT_FLAG 		3
#define SPI_NAND_FLAG			4
#define SPI_EMMC_FLAG			5

/***
*boot_device_flag = 0 ; indicate spi+nand boot
*boot_device_flag = 1;  indicate nand  boot
***/
int check_storage_device(void)
{
	int value = -1;
	value = boot_device_flag;	
	if((value == -1)||(value == 0)||(value == SPI_NAND_FLAG) ||(value == NAND_BOOT_FLAG) ){
				if((value == 0)||(value == -1)){
					
					if(POR_NAND_BOOT()){
						boot_device_flag = 1;
					}else if(POR_EMMC_BOOT()){
						boot_device_flag = -1;
					}else if(POR_SPI_BOOT()){
						boot_device_flag = 0;
					}else if(POR_CARD_BOOT()){
						boot_device_flag = 1;
					}
				}else{
					boot_device_flag = 0;
					if((value == NAND_BOOT_FLAG)){
						boot_device_flag = 1;
					}
				}

		}else {
		boot_device_flag = -1;
	}
	aml_nand_msg("boot_device_flag : %d",boot_device_flag);

	if((boot_device_flag == 0) || (boot_device_flag == 1)){
		return 0;
	}else{
		boot_device_flag = value;
		return -NAND_FAILED;
	}

}

static int  __init get_storage_device(char *str)
{
	int value = -1;
	value = simple_strtoul(str, NULL, 16);
	aml_nand_msg("get storage device: storage %s",str);
	aml_nand_msg("value=%d",value);
	
	boot_device_flag = value;
	
	return 0;
}

early_param("storage",get_storage_device);

#endif
#ifdef AML_NAND_UBOOT
int amlnf_init(unsigned flag)
#else
static int amlnf_init(struct platform_device *pdev)
#endif
{
	int ret = 0;
#ifndef AML_NAND_UBOOT
	unsigned flag = 0;
	ret = check_storage_device();
	if(ret < 0){
		aml_nand_msg("do not init nand");
		return 0;
	}

#ifdef CONFIG_OF

	pdev->dev.platform_data = aml_get_driver_data(pdev);
	printk("===========================================");
	printk("%s:%d,nand device tree ok,dev-name:%s\n",__func__,__LINE__,dev_name(&pdev->dev));
#endif
	ret = get_nand_platform(pdev->dev.platform_data,pdev);
#endif
	
#ifdef AML_NAND_UBOOT
	ret= amlnf_phy_init(unsigned flag);
#else
	ret = amlnf_phy_init(flag, pdev);
#endif
	if(ret){
		aml_nand_msg("nandphy_init failed and ret=0x%x", ret);
		goto exit_error0;
	}

	ret = amlnf_logic_init(flag);
	if(ret < 0){
		aml_nand_msg("amlnf_add_nftl failed and ret=0x%x", ret);
		goto exit_error0;
	}

	ret = amlnf_dev_init(flag);
	if(ret < 0){
		aml_nand_msg("amlnf_add_nftl failed and ret=0x%x", ret);
		goto exit_error0;
	}

exit_error0:
	return 0;//ret;   //fix crash bug for error case.
}

#ifdef AML_NAND_UBOOT
int amlnf_exit(unsigned flag)
#else
static int amlnf_exit(struct platform_device *pdev)
#endif
{
	int ret = 0;
#ifndef AML_NAND_UBOOT
	unsigned flag = 0;
#endif

	return 0;
}


#ifndef AML_NAND_UBOOT
static void amlnf_shutdown(struct platform_device *pdev)
{
	struct amlnand_phydev * phy_dev = NULL;
	
	if(check_storage_device() < 0){
		aml_nand_msg("without nand");
		return;
	}
	
	list_for_each_entry(phy_dev, &nphy_dev_list, list){
		if(phy_dev){
			amlnand_get_device(phy_dev, CHIP_SHUTDOWN);
			phy_dev->option |= NAND_SHUT_DOWN;
			amlnand_release_device(phy_dev);
		}
	}
	
	return ;
}

/* driver device registration */
static struct platform_driver amlnf_driver = {
	.probe		= amlnf_init,
	.remove		= amlnf_exit,
	.shutdown		= amlnf_shutdown,
	.driver		= {
		.name	= DRV_AMLNFDEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table=amlogic_nand_dt_match,
	},
};

static int __init amlnf_module_init(void)
{
	return platform_driver_register(&amlnf_driver);
}

static void __exit amlnf_module_exit(void)
{
	platform_driver_unregister(&amlnf_driver);
}

module_init(amlnf_module_init);
module_exit(amlnf_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("AML NAND TEAM");
MODULE_DESCRIPTION("aml nand flash driver");
#endif


