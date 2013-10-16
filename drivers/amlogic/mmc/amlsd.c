#include <linux/stddef.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/proc_fs.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/card.h>
#include <linux/genhd.h>
#include <mach/hardware.h>
#include <plat/regops.h>
#include <linux/slab.h>
#include <mach/sd.h>
#include <asm/cacheflush.h>
#include <asm/dma-mapping.h>
#include <asm/outercache.h>
#include <mach/power_gate.h>
#include <linux/clk.h>
#include <mach/pinmux.h>
#include <mach/am_regs.h>
#include <mach/mod_gate.h>
#include <linux/highmem.h>
#include <mach/register.h>
#include <mach/irqs.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/of_gpio.h>
#include <linux/amlogic/wifi_dt.h>
#include "amlsd.h"

/*====================================================================*/
/* Support for /proc/mtd */

// static int wifi_setup_dt(void);

static struct proc_dir_entry *proc_card;
static struct mtd_partition *card_table[16];

static inline int card_proc_info (char *buf, char* dev_name, int i)
{
	struct mtd_partition *this = card_table[i];

	if (!this)
		return 0;

	return sprintf(buf, "%s%d: %8.8llx %8.8x \"%s\"\n", dev_name,
		        i+1,(unsigned long long)this->size,
		       512*1024, this->name);
}

static int card_read_proc (char *page, char **start, off_t off, int count,
			  int *eof, void *data_unused)
{
	int len, l, i;
    off_t   begin = 0;

	len = sprintf(page, "dev:    size   erasesize  name\n");
    for (i=0; i< 16; i++) {

            l = card_proc_info(page + len, "inand", i);
            len += l;
            if (len+begin > off+count)
                    goto done;
            if (len+begin < off) {
                    begin += len;
                    len = 0;
            }
    }

    *eof = 1;

done:
    if (off >= len+begin)
            return 0;
    *start = page + (off-begin);
    return ((count < begin+len-off) ? count : begin+len-off);
}

/**
 * add_card_partition : add card partition , refer to 
 * board-****.c  inand_partition_info[]
 * @disk: add partitions in which disk
 * @part: partition table
 * @nr_part: partition numbers
 */
int add_part_table(struct mtd_partition * part, unsigned int nr_part)
{
	unsigned int i;
	uint64_t cur_offset=0;
	uint64_t offset, size;
	
	if(!part)
		return 0;

	for(i=0; i<nr_part; i++){
		offset = part[i].offset>>9;
		size = part[i].size>>9;
		if (part[i].offset== MTDPART_OFS_APPEND)
			offset = cur_offset;
		cur_offset = offset + size;
		
		card_table[i] = &part[i];
		card_table[i]->offset = offset<<9;
		card_table[i]->size = size<<9;
		card_table[i]->name = part[i].name;
	}

	if (!proc_card && (proc_card = create_proc_entry( "inand", 0, NULL )))
		proc_card->read_proc = card_read_proc;

	return 0;
}

struct hd_struct *add_emmc_each_part(struct gendisk *disk, int partno,
				sector_t start, sector_t len, int flags,
				char * pname)
{
	struct hd_struct *p;
	dev_t devt = MKDEV(0, 0);
	struct device *ddev = disk_to_dev(disk);
	struct device *pdev;
	struct disk_part_tbl *ptbl;
	const char *dname;
	int err;

	err = disk_expand_part_tbl(disk, partno);
	if (err)
		return ERR_PTR(err);
	ptbl = disk->part_tbl;

	if (ptbl->part[partno])
		return ERR_PTR(-EBUSY);

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return ERR_PTR(-EBUSY);

	if (!init_part_stats(p)) {
		err = -ENOMEM;
		goto out_free;
	}
	pdev = part_to_dev(p);

	p->start_sect = start;
	p->alignment_offset =
		queue_limit_alignment_offset(&disk->queue->limits, start);
	p->discard_alignment =
		queue_limit_discard_alignment(&disk->queue->limits, start);
	p->nr_sects = len;
	p->partno = partno;
	p->policy = get_disk_ro(disk);

	dname = dev_name(ddev);
	dev_set_name(pdev, "%s", pname);

	device_initialize(pdev);
	pdev->class = &block_class;
	pdev->type = &part_type;
	pdev->parent = ddev;

	err = blk_alloc_devt(p, &devt);
	if (err)
		goto out_free_info;
	pdev->devt = devt;

	/* delay uevent until 'holders' subdir is created */
	dev_set_uevent_suppress(pdev, 1);
	err = device_add(pdev);
	if (err)
		goto out_put;

	err = -ENOMEM;
	p->holder_dir = kobject_create_and_add("holders", &pdev->kobj);
	if (!p->holder_dir)
		goto out_del;

	dev_set_uevent_suppress(pdev, 0);

	/* everything is up and running, commence */
	rcu_assign_pointer(ptbl->part[partno], p);

	/* suppress uevent if the disk suppresses it */
	if (!dev_get_uevent_suppress(ddev))
		kobject_uevent(&pdev->kobj, KOBJ_ADD);

	hd_ref_init(p);
	return p;

out_free_info:
	free_part_info(p);
out_free:
	kfree(p);
	return ERR_PTR(err);
out_del:
	kobject_put(p->holder_dir);
	device_del(pdev);
out_put:
	put_device(pdev);
	blk_free_devt(devt);
	return ERR_PTR(err);
}

/**
 * add_emmc_partition : add emmc partition , refer to 
 * board-****.c  inand_partition_info[]
 * @disk: add partitions in which disk
 * @part: partition table
 * @nr_part: partition numbers
 */
int add_emmc_partition(struct gendisk * disk)
{
	unsigned int i;
	struct hd_struct * ret=NULL;
	uint64_t offset, size;
	
	printk("add_emmc_partition\n");
	if(!proc_card){
		printk("proc_nand NULL\n");
		return 0;
	}

	for(i=0; i<CONFIG_MMC_BLOCK_MINORS; i++){
		if(!card_table[i])
			break;
		offset = card_table[i]->offset>>9;
		if(card_table[i]->size == MTDPART_SIZ_FULL){
			size = get_capacity(disk) - offset;
			card_table[i]->size = size<<9;
		}
		else
			size = card_table[i]->size>>9;
		
		ret = add_emmc_each_part(disk, 1+i, offset, size, 0, card_table[i]->name);
		printk("[%sp%d] %20s  offset 0x%012llx, len 0x%012llx %s\n",
				disk->disk_name, 1+i, card_table[i]->name, offset<<9, 
				size<<9, IS_ERR(ret) ? "add fail":"");
	}
	return 0;
}

EXPORT_SYMBOL(add_emmc_partition);

/*-----------sg copy buffer------------*/

/**
 * aml_sg_miter_stop - stop mapping iteration for amlogic,
 * We don't disable irq in this function
 */
static void aml_sg_miter_stop(struct sg_mapping_iter *miter)
{
	unsigned long flags;

	WARN_ON(miter->consumed > miter->length);

	/* drop resources from the last iteration */
	if (miter->addr) {
		miter->__offset += miter->consumed;

		if (miter->__flags & SG_MITER_TO_SG){
			//printk("flush page addr %x, length %x\n", miter->addr, miter->length);
			flush_kernel_dcache_page(miter->page);
		}

		if (PageHighMem(miter->page)) {
			printk(KERN_DEBUG "AML_SDHC miter_stop highmem\n");
			local_irq_save(flags);
			kunmap_atomic(miter->addr);
			local_irq_restore(flags);
		}

		miter->page = NULL;
		miter->addr = NULL;
		miter->length = 0;
		miter->consumed = 0;
	}
}

/**
 * aml_sg_miter_next - proceed mapping iterator to the next mapping for amlogic,
 * We don't disable irq in this function
 */
static bool aml_sg_miter_next(struct sg_mapping_iter *miter)
{
	unsigned int off, len;
	unsigned long flags;

	/* check for end and drop resources from the last iteration */
	if (!miter->__nents)
		return false;

	aml_sg_miter_stop(miter);

	/* get to the next sg if necessary.  __offset is adjusted by stop */
	while (miter->__offset == miter->__sg->length) {
		if (--miter->__nents) {
			miter->__sg = sg_next(miter->__sg);
			miter->__offset = 0;
		} else
			return false;
	}

	/* map the next page */
	off = miter->__sg->offset + miter->__offset;
	len = miter->__sg->length - miter->__offset;

	miter->page = nth_page(sg_page(miter->__sg), off >> PAGE_SHIFT);
	off &= ~PAGE_MASK;
	miter->length = min_t(unsigned int, len, PAGE_SIZE - off);
	miter->consumed = miter->length;

    if (PageHighMem(miter->page)){
		printk(KERN_DEBUG "AML_SDHC miter_next highmem\n");
		local_irq_save(flags);
    	miter->addr = kmap_atomic(miter->page) + off;
		local_irq_restore(flags);
    }
	else
		miter->addr = page_address(miter->page) + off;
	return true;
}

/**
 * aml_sg_copy_buffer - Copy data between a linear buffer and an SG list  for amlogic,
 * We don't disable irq in this function
 **/
size_t aml_sg_copy_buffer(struct scatterlist *sgl, unsigned int nents,
			     void *buf, size_t buflen, int to_buffer)
{
	unsigned int offset = 0;
	struct sg_mapping_iter miter;
	unsigned int sg_flags = 0;

	if (to_buffer)
		sg_flags |= SG_MITER_FROM_SG;
	else
		sg_flags |= SG_MITER_TO_SG;

	sg_miter_start(&miter, sgl, nents, sg_flags);

	while (aml_sg_miter_next(&miter) && offset < buflen) {
		unsigned int len;

		len = min(miter.length, buflen - offset);

		if (to_buffer)
			memcpy(buf + offset, miter.addr, len);
		else
			memcpy(miter.addr, buf + offset, len);

		offset += len;
	}

	aml_sg_miter_stop(&miter);

	return offset;
}

EXPORT_SYMBOL(aml_sg_copy_buffer);

/*-------------------eMMC/tSD-------------------*/

int storage_flag = 0;
int  __init get_storage_device(char *str)
{
    storage_flag = simple_strtoul(str, NULL, 16);

    printk("[%s] storage_flag=%d\n", __FUNCTION__, storage_flag);
    if ((storage_flag != EMMC_BOOT_FLAG) && (storage_flag != SPI_EMMC_FLAG)) {
        printk("[%s] the storage device does NOT relate to eMMC,"
                " storage_flag=%d\n", __FUNCTION__, storage_flag);
    }

    return 0;
}
early_param("storage",get_storage_device);

bool is_emmc_exist (struct amlsd_host* host) // is eMMC/tSD exist
{
    print_tmp("host->storage_flag=%d, POR_BOOT_VALUE=%d\n", host->storage_flag, POR_BOOT_VALUE);
    if ((host->storage_flag == EMMC_BOOT_FLAG) || (host->storage_flag == SPI_EMMC_FLAG)
            || (((host->storage_flag == 0)  || (host->storage_flag == -1)) && (POR_EMMC_BOOT() || POR_SPI_BOOT()))) {
        return true;
    }

    return false;
}

/*-------------------debug---------------------*/

unsigned int sdhc_debug=0x0; // 0xffffffff;

static int __init sdhc_debug_setup(char *str)
{
	sdhc_debug = simple_strtol(str, NULL, 0);
	return 1;
}
__setup("sdhc_debug=", sdhc_debug_setup);


unsigned int sdio_debug=0x000000; // 0xffffff; // 

static int __init sdio_debug_setup(char *str)
{
	sdio_debug = simple_strtol(str, NULL, 0);
	return 1;
}
__setup("sdio_debug=", sdio_debug_setup);

void aml_dbg_print_pinmux (void)
{
    printk("Pinmux: REG2=0x%08x, REG3=0x%08x, REG4=0x%08x, REG5=0x%08x, REG6=0x%08x, REG8=0x%08x\n", 
            READ_CBUS_REG(PERIPHS_PIN_MUX_2), 
            READ_CBUS_REG(PERIPHS_PIN_MUX_3), 
            READ_CBUS_REG(PERIPHS_PIN_MUX_4), 
            READ_CBUS_REG(PERIPHS_PIN_MUX_5), 
            READ_CBUS_REG(PERIPHS_PIN_MUX_6), 
            READ_CBUS_REG(PERIPHS_PIN_MUX_8));
}

// void aml_dbg_print_pull_up (struct amlsd_host* host)
// {
    // printk("Pull-up: CMD%d, PAD_PULL_UP_REG3=%#08x\n", 
            // host->opcode, 
            // READ_CBUS_REG(PAD_PULL_UP_REG3));
// }


/*----sdhc----*/

void aml_sdhc_print_reg(struct amlsd_host* host)
{
    u32 buf[16];
    memcpy_fromio(buf, host->base, 0x30);

    printk(KERN_DEBUG "***********SDHC_REGS***********\n");
    printk(KERN_DEBUG "SDHC_ARGU: 0x%x\n", buf[SDHC_ARGU/4]);
    printk(KERN_DEBUG "SDHC_SEND: 0x%x\n", buf[SDHC_SEND/4]);
    printk(KERN_DEBUG "SDHC_CTRL: 0x%x\n", buf[SDHC_CTRL/4]);
    printk(KERN_DEBUG "SDHC_STAT: 0x%x\n", buf[SDHC_STAT/4]);
    printk(KERN_DEBUG "SDHC_CLKC: 0x%x\n", buf[SDHC_CLKC/4]);
    printk(KERN_DEBUG "SDHC_ADDR: 0x%x\n", buf[SDHC_ADDR/4]);
    printk(KERN_DEBUG "SDHC_PDMA: 0x%x\n", buf[SDHC_PDMA/4]);
    printk(KERN_DEBUG "SDHC_MISC: 0x%x\n", buf[SDHC_MISC/4]);
    printk(KERN_DEBUG "SDHC_DATA: 0x%x\n", buf[SDHC_DATA/4]);
    printk(KERN_DEBUG "SDHC_ICTL: 0x%x\n", buf[SDHC_ICTL/4]);
    printk(KERN_DEBUG "SDHC_ISTA: 0x%x\n", buf[SDHC_ISTA/4]);
    printk(KERN_DEBUG "SDHC_SRST: 0x%x\n", buf[SDHC_SRST/4]);
}

static int aml_sdhc_regs_show(struct seq_file *s, void *v)
{
	struct mmc_host * mmc = (struct mmc_host *)s->private;
	struct amlsd_platform* pdata = mmc_priv(mmc);
	struct amlsd_host* host = pdata->host;

    u32 buf[16];
    memcpy_fromio(buf, host->base, 0x30);

    seq_printf(s,"***********SDHC_REGS***********\n");
    seq_printf(s, "SDHC_ARGU: 0x%x\n", buf[SDHC_ARGU/4]);
    seq_printf(s, "SDHC_SEND: 0x%x\n", buf[SDHC_SEND/4]);
    seq_printf(s, "SDHC_CTRL: 0x%x\n", buf[SDHC_CTRL/4]);
    seq_printf(s, "SDHC_STAT: 0x%x\n", buf[SDHC_STAT/4]);
    seq_printf(s, "SDHC_CLKC: 0x%x\n", buf[SDHC_CLKC/4]);
    seq_printf(s, "SDHC_ADDR: 0x%x\n", buf[SDHC_ADDR/4]);
    seq_printf(s, "SDHC_PDMA: 0x%x\n", buf[SDHC_PDMA/4]);
    seq_printf(s, "SDHC_MISC: 0x%x\n", buf[SDHC_MISC/4]);
    seq_printf(s, "SDHC_DATA: 0x%x\n", buf[SDHC_DATA/4]);
    seq_printf(s, "SDHC_ICTL: 0x%x\n", buf[SDHC_ICTL/4]);
    seq_printf(s, "SDHC_ISTA: 0x%x\n", buf[SDHC_ISTA/4]);
    seq_printf(s, "SDHC_SRST: 0x%x\n", buf[SDHC_SRST/4]);

	return 0;
}

static int aml_sdhc_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, aml_sdhc_regs_show, inode->i_private);
}

static const struct file_operations aml_sdhc_regs_fops = {
	.owner		= THIS_MODULE,
	.open		= aml_sdhc_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*----sdio----*/

void aml_sdio_print_reg(struct amlsd_host* host)
{
    u32 buf[16];
    memcpy_fromio(buf, host->base, 0x20);

    printk("***********SDIO_REGS***********\n");
    printk("SDIO_ARGU: 0x%x\n", buf[SDIO_ARGU/4]);
    printk("SDIO_SEND: 0x%x\n", buf[SDIO_SEND/4]);
    printk("SDIO_CONF: 0x%x\n", buf[SDIO_CONF/4]);
    printk("SDIO_IRQS: 0x%x\n", buf[SDIO_IRQS/4]);
    printk("SDIO_IRQC: 0x%x\n", buf[SDIO_IRQC/4]);
    printk("SDIO_MULT: 0x%x\n", buf[SDIO_MULT/4]);
    printk("SDIO_ADDR: 0x%x\n", buf[SDIO_ADDR/4]);
    printk("SDIO_EXT: 0x%x\n", buf[SDIO_EXT/4]);
}

static int aml_sdio_regs_show(struct seq_file *s, void *v)
{
	struct mmc_host * mmc = (struct mmc_host *)s->private;
	struct amlsd_platform* pdata = mmc_priv(mmc);
	struct amlsd_host* host = pdata->host;
    u32 buf[16];
    memcpy_fromio(buf, host->base, 0x20);

    seq_printf(s,"***********SDIO_REGS***********\n");
    seq_printf(s, "SDIO_ARGU: 0x%x\n", buf[SDIO_ARGU/4]);
    seq_printf(s, "SDIO_SEND: 0x%x\n", buf[SDIO_SEND/4]);
    seq_printf(s, "SDIO_CONF: 0x%x\n", buf[SDIO_CONF/4]);
    seq_printf(s, "SDIO_IRQS: 0x%x\n", buf[SDIO_IRQS/4]);
    seq_printf(s, "SDIO_IRQC: 0x%x\n", buf[SDIO_IRQC/4]);
    seq_printf(s, "SDIO_MULT: 0x%x\n", buf[SDIO_MULT/4]);
    seq_printf(s, "SDIO_ADDR: 0x%x\n", buf[SDIO_ADDR/4]);
    seq_printf(s, "SDIO_EXT: 0x%x\n", buf[SDIO_EXT/4]);
	return 0;
}

static int aml_sdio_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, aml_sdio_regs_show, inode->i_private);
}

static const struct file_operations aml_sdio_regs_fops = {
	.owner		= THIS_MODULE,
	.open		= aml_sdio_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*----host param----*/

static int amlsd_param_show(struct seq_file *s, void *v)
{
	struct mmc_host * mmc = (struct mmc_host *)s->private;
	struct amlsd_platform* pdata = mmc_priv(mmc);

	seq_printf(s, "f_max : %d\n", pdata->f_max);
	seq_printf(s, "f_max_w : %d\n", pdata->f_max_w);
	seq_printf(s, "f_min : %d\n", pdata->f_min);
	seq_printf(s, "port : %d\n", pdata->port);
	seq_printf(s, "caps : 0x%x\n", pdata->caps);
	seq_printf(s, "ocr_avail : 0x%lx\n", pdata->ocr_avail);
	seq_printf(s, "max_req_size : %d\n", pdata->max_req_size);
	return 0;
}

static int amlsd_param_open(struct inode *inode, struct file *file)
{
	return single_open(file, amlsd_param_show, inode->i_private);
}

static const struct file_operations amlsd_param_fops = {
	.owner		= THIS_MODULE,
	.open		= amlsd_param_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void aml_sdhc_init_debugfs(struct mmc_host *mmc)
{
	struct dentry		*root;
	struct dentry		*node;

	if (!mmc->debugfs_root)
		return;

	root = debugfs_create_dir(dev_name(&mmc->class_dev), mmc->debugfs_root);
	if (IS_ERR(root)){
		/* Don't complain -- debugfs just isn't enabled */
		dev_err(&mmc->class_dev, "SDHC debugfs just isn't enabled\n");
		return;
	}
	if (!root){
		/* Complain -- debugfs is enabled, but it failed to
		 * create the directory. */
		dev_err(&mmc->class_dev, "SDHC it failed to create the directory\n");
		goto err;
	}

	debugfs_create_x32("sdhc_dbg", S_IRWXUGO, root, (u32 *)&sdhc_debug);

	node = debugfs_create_file("sdhc_regs", S_IRWXUGO, root, mmc,
                &aml_sdhc_regs_fops);
	if (IS_ERR(node))
		return;
    
	node = debugfs_create_file("params", S_IRWXUGO, root, mmc,
                &amlsd_param_fops);
	if (IS_ERR(node))
		return;

	return;

err:
	dev_err(&mmc->class_dev, "failed to initialize debugfs for slot\n");
}

void aml_sdio_init_debugfs(struct mmc_host *mmc)
{
	struct dentry		*root;
	struct dentry		*node;

	if (!mmc->debugfs_root)
		return;

	root = debugfs_create_dir(dev_name(&mmc->class_dev), mmc->debugfs_root);
	if (IS_ERR(root)){
		/* Don't complain -- debugfs just isn't enabled */
		dev_err(&mmc->class_dev, "SDIO debugfs just isn't enabled\n");
		return;
	}
	if (!root){
		/* Complain -- debugfs is enabled, but it failed to
		 * create the directory. */
		dev_err(&mmc->class_dev, "SDIO it failed to create the directory\n");
		goto err;
	}

	debugfs_create_x32("sdio_dbg", S_IRWXUGO, root, (u32 *)&sdio_debug);

	node = debugfs_create_file("sdio_regs", S_IRWXUGO, root, mmc,
                &aml_sdio_regs_fops);
	if (IS_ERR(node))
		return;
    
	node = debugfs_create_file("params", S_IRWXUGO, root, mmc,
                &amlsd_param_fops);
	if (IS_ERR(node))
		return;

	return;

err:
	dev_err(&mmc->class_dev, "failed to initialize debugfs for SDIO\n");
}

/*-------------------port function---------------------*/

//return 1: no inserted  0: inserted
int of_amlsd_detect(struct amlsd_platform* pdata)
{
	int ret=0;
	if(pdata->gpio_cd)
		ret = amlogic_get_value(pdata->gpio_cd, MODULE_NAME);
	// printk(" of_amlsd_detect port %d ret %d\n", pdata->port, ret);
	return ret;
}

void of_amlsd_irq_init(struct amlsd_platform* pdata)
{
	if(pdata->irq_in && pdata->irq_out){
		amlogic_gpio_to_irq(pdata->gpio_cd, MODULE_NAME,
				AML_GPIO_IRQ(pdata->irq_in, FILTER_NUM7,GPIO_IRQ_FALLING));
		amlogic_gpio_to_irq(pdata->gpio_cd, MODULE_NAME,
				AML_GPIO_IRQ(pdata->irq_out, FILTER_NUM7,GPIO_IRQ_RISING));
	}
}

/*set pin input*/
void of_amlsd_pwr_prepare(struct amlsd_platform* pdata)
{
}


void of_amlsd_pwr_on(struct amlsd_platform* pdata)
{
	if(pdata->gpio_power)
		amlogic_set_value(pdata->gpio_power, pdata->power_level, MODULE_NAME);
	if(pdata->port == MESON_SDIO_PORT_A)
        extern_wifi_set_enable(1); //power on wifi
}
void of_amlsd_pwr_off(struct amlsd_platform* pdata)
{
	if(pdata->gpio_power)
		amlogic_set_value(pdata->gpio_power, !pdata->power_level, MODULE_NAME);
	if(pdata->port == MESON_SDIO_PORT_A)
        extern_wifi_set_enable(0); //power off wifi
}
int of_amlsd_init(struct amlsd_platform* pdata)
{
	BUG_ON(!pdata);
	if(pdata->gpio_cd)
		amlogic_gpio_request_one(pdata->gpio_cd, GPIOF_IN, MODULE_NAME);
	if(pdata->gpio_ro)
		amlogic_gpio_request_one(pdata->gpio_ro, GPIOF_IN, MODULE_NAME);
	if(pdata->gpio_power){
		if(pdata->power_level)
			amlogic_gpio_request_one(pdata->gpio_power,
						GPIOF_OUT_INIT_LOW, MODULE_NAME);
		else
			amlogic_gpio_request_one(pdata->gpio_power,
						GPIOF_OUT_INIT_HIGH, MODULE_NAME);
	}
	if(pdata->port == MESON_SDIO_PORT_A)
		wifi_setup_dt();
	return 0;
}

void of_amlsd_xfer_pre(struct amlsd_platform* pdata)
{
    char pinctrl[30];
    char *p=pinctrl;
    int i, size=0;
    struct pinctrl* ppin;

    if (pdata->port > PORT_SDIO_C) { // so it should be PORT_SDHC_X
        strncpy(p, "sdhc_", sizeof(pinctrl));
        size = strlen(p);
        p += size;
    }

    if (pdata->mmc->ios.chip_select == MMC_CS_DONTCARE) {
        if(pdata->mmc->caps & MMC_CAP_4_BIT_DATA){
            snprintf(p, sizeof(pinctrl)-size, "%s_all_pins", pdata->pinname);
        }else{
            snprintf(p, sizeof(pinctrl)-size, "%s_1bit_pins", pdata->pinname);
        }
    } else { // MMC_CS_HIGH
        snprintf(p, sizeof(pinctrl)-size, "%s_clk_cmd_pins", pdata->pinname);
    }

    for (i = 0; i < 100; i++) {
        ppin = devm_pinctrl_get_select(&pdata->host->pdev->dev, p);
        if(!IS_ERR(ppin)) {
            pdata->host->pinctrl = ppin;
            break;
        }
        /* else -> aml_irq_cdin_thread() should be using one of the GPIO of card,
         * then we should wait here until the GPIO is free,
         * otherwise something must be wrong.
         */
        mdelay(1);
    }
    if (i == 100) {
        sdhc_err("CMD%d: get pinctrl fail.\n", pdata->host->opcode);
    }
    //printk("pre pinctrl %x, %s, ppin %x\n", pdata->host->pinctrl, p, ppin);
}

void of_amlsd_xfer_post(struct amlsd_platform* pdata)
{
    if (pdata->host->pinctrl) {
        devm_pinctrl_put(pdata->host->pinctrl);
        pdata->host->pinctrl = NULL;
    } else {
        sdhc_err("CMD%d: pdata->host->pinctrl = NULL\n", pdata->host->opcode);
    }
    // printk(KERN_ERR "CMD%d: put pinctrl\n", pdata->host->opcode);
    // aml_dbg_print_pinmux(); // for debug
}

// void of_init_pins (struct amlsd_platform* pdata)
// {
    // char pinctrl_name[30];
    // char *p=pinctrl_name;
    // int size=0;
	// struct pinctrl *pinctrl;

    // if (pdata->port > PORT_SDIO_C) { // so it should be PORT_SDHC_X
        // strncpy(p, "sdhc_", sizeof(pinctrl_name));
        // size = strlen(p);
        // p += size;
    // }
    // snprintf(p, sizeof(pinctrl_name)-size, "%s_init_pins", pdata->pinname);

    // pinctrl = of_pinctrl_sel(pdata->host, pinctrl_name);
    // if (pinctrl) {
        // sdhc_err("select %s ok\n", pinctrl_name);
        // pinctrl_put(pinctrl);
    // } else {
        // sdhc_err("select %s error\n", pinctrl_name);
    // }
// }

void aml_cs_high (struct amlsd_platform * pdata) // chip select high
{
    int ret;

    /*
	 * Non-SPI hosts need to prevent chipselect going active during
	 * GO_IDLE; that would put chips into SPI mode.  Remind them of
	 * that in case of hardware that won't pull up DAT3/nCS otherwise.
     *
     * Now the way to accomplish this is: 
     * 1) set DAT3-pin as a GPIO pin(by pinmux), and pulls up;
     * 2) send CMD0;
     * 3) set DAT3-pin as a card-dat3-pin(by pinmux);
	 */
    if ((pdata->mmc->ios.chip_select == MMC_CS_HIGH) && (pdata->gpio_dat3 != 0)
        && (pdata->jtag_pin == 0)) { // is NOT sd card

        ret = amlogic_gpio_request_one(pdata->gpio_dat3, GPIOF_OUT_INIT_HIGH, MODULE_NAME);
        CHECK_RET(ret);
        ret = amlogic_gpio_direction_output(pdata->gpio_dat3, 1, MODULE_NAME); // output high
        CHECK_RET(ret);
        // print_tmp("emmc gpio_dat3=%d\n", amlogic_get_value(pdata->gpio_dat3, MODULE_NAME));
    }
}

void aml_cs_dont_care (struct amlsd_platform * pdata) // chip select don't care
{
    int ret;

    if ((pdata->mmc->ios.chip_select == MMC_CS_DONTCARE) && (pdata->gpio_dat3 != 0)
            && (pdata->jtag_pin == 0)&& (amlogic_get_value(pdata->gpio_dat3, MODULE_NAME) >= 0)) { // not free yet
        ret = amlogic_gpio_free(pdata->gpio_dat3, MODULE_NAME);
        CHECK_RET(ret);

        // print_tmp("emmc gpio_dat3=%d\n", amlogic_get_value(pdata->gpio_dat3, MODULE_NAME));
    }
}

int aml_is_sdjtag(struct amlsd_platform * pdata)
{
    int ret, jtag;
    
    if(pdata->jtag_pin != 0) {
        ret = amlogic_gpio_request_one(pdata->jtag_pin, GPIOF_IN, MODULE_NAME);
        if(ret){
            printk("DAT1 pinmux used, return no jtag\n");
            return 0;
        }
        CHECK_RET(ret);
        jtag = amlogic_get_value(pdata->jtag_pin, MODULE_NAME);
        // print_tmp("sd jtag_pin=%d\n", amlogic_get_value(pdata->jtag_pin, MODULE_NAME));
        amlogic_gpio_free(pdata->jtag_pin, MODULE_NAME);
        if(jtag == 1){
            return 1;
        }
    }
    return 0;
}

int aml_is_sduart(struct amlsd_platform * pdata)
{
    int ret, dat3;

    if(pdata->is_sduart)
        return 1;

    if (pdata->gpio_dat3 != 0){
        ret = amlogic_gpio_request_one(pdata->gpio_dat3, GPIOF_IN, MODULE_NAME);
        if(ret){
            printk("DAT3 pinmux used, return no uart\n");
            return 0;
        }
        CHECK_RET(ret);
        dat3 = amlogic_get_value(pdata->gpio_dat3, MODULE_NAME);
        // print_tmp("sd gpio_dat3=%d\n", amlogic_get_value(pdata->gpio_dat3, MODULE_NAME));
        amlogic_gpio_free(pdata->gpio_dat3, MODULE_NAME);
        if(dat3 == 0){
            return 1;
        }
    }
    return 0;
}


extern struct pinctrl* g_uart_pinctrl;

// int n=0;
int aml_uart_switch(struct amlsd_platform* pdata, bool on)
{
    /*on : uart card pin, !on: uart ao pin*/
    if(on){
        print_tmp("aml_uart_switch %d \n\n\n\n\n", on);
        pdata->is_sduart = 1;
        aml_clr_reg32_mask(P_AO_RTI_PIN_MUX_REG, 0x1800); // UART_TX_AO_A
        aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_2, 0x3040);
        aml_set_reg32_mask(P_PERIPHS_PIN_MUX_8, 0x600);
    }else if(pdata->is_sduart){
        print_tmp("aml_uart_switch %d \n\n\n\n\n", on);
        aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_8, 0x600);
        aml_set_reg32_mask(P_AO_RTI_PIN_MUX_REG, 0x1800);
        pdata->is_sduart = 0;
    } // else
        // printk("do nothing\n");
    // print_tmp("aml_uart_switch >>>>>>>>\n\n\n\n\n");
    return 0;

#if 0
    struct pinctrl *pin=NULL;
    /*on : uart card pin, !on: uart ao pin*/
    if(!on){
        if(pdata->uart_card_pinctrl){
            printk("put CARD pinctrl %x\n", pdata->uart_card_pinctrl);
            devm_pinctrl_put(pdata->uart_card_pinctrl);
            pdata->uart_card_pinctrl = NULL;
            mdelay(100);
        }
        if(!pdata->uart_ao_pinctrl){
            printk("n=%d\n", n);
            if(n>0)
                pin = devm_pinctrl_get_select(&pdata->host->pdev->dev, "uartao_default");
            printk("get AO pinctrl %x\n", pdata->uart_ao_pinctrl);
            if(!IS_ERR(pin))
                pdata->uart_ao_pinctrl = pin;
            else
                CHECK_RET(pin);
            mdelay(100);
            n++;
        }
    }else{
        if(pdata->uart_ao_pinctrl){
            printk("put AO pinctrl %x\n", pdata->uart_ao_pinctrl);
            devm_pinctrl_put(pdata->uart_ao_pinctrl);
            pdata->uart_ao_pinctrl = NULL;
            mdelay(100);
        }
        if(!pdata->uart_card_pinctrl){
            pin = devm_pinctrl_get_select(&pdata->host->pdev->dev, "jtag_pin");
            if(!IS_ERR(pin))
                pdata->uart_card_pinctrl = pin;
            else
                CHECK_RET(pin);
            mdelay(100);
        }
    }
    printk("CARD %x, AO %x\n", pdata->uart_card_pinctrl, 
    pdata->uart_ao_pinctrl);
#endif
}

#ifndef CONFIG_AM_WIFI_SD_MMC
int wifi_setup_dt()
{
	return 0;
}
void extern_wifi_set_enable(int is_on)
{
    return;
}
#endif

