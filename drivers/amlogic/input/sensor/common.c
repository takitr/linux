#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/amlogic/input/common.h>


int sensor_setup_i2c_dev(struct i2c_board_info *i2c_info, int *i2c_bus_nr, int *gpio)
{
    int ret = -1;
    struct device_node *node = of_find_node_by_name(NULL, i2c_info->type);

    if(node)
    {
        int r;
        int irq;
        const char *status;

        ret = 0;
		r = of_property_read_string(node, "status", &status);
        if(r< 0){
            printk("%s: Failed to read status from device tree for dev %s\n", __func__, i2c_info->type);
            return -1;
        }


        if(strncmp("ok", status, 2) == 0)
        {
            u32 addr;
            char *str;

            ret = 0;
            r = of_property_read_u32(node, "address",&addr);
            if(r < 0)
            {
                printk("%s: faild to get i2c address for dev %s\n", __func__, i2c_info->type);
                return -1;
            }

            i2c_info->addr = addr;


            r = of_property_read_string(node, "i2c_bus", &str);
            if (r) {
                printk("%s: faild to get i2c_bus str for dev %s\n", __func__, i2c_info->type);
                *i2c_bus_nr = AML_I2C_BUS_B;
            } else {
                if (!strncmp(str, "i2c_bus_a", 9))
                    *i2c_bus_nr = AML_I2C_BUS_A;
                else if (!strncmp(str, "i2c_bus_b", 9))
                    *i2c_bus_nr = AML_I2C_BUS_B;
                else if (!strncmp(str, "i2c_bus_ao", 9))
                    *i2c_bus_nr = AML_I2C_BUS_AO;
                else
                    *i2c_bus_nr = AML_I2C_BUS_B;
            }


            if(gpio) 
            {
                r = of_property_read_u32(node, "irq",&irq);
                if(r < 0 || i2c_info->irq <= 0)
                {
                    i2c_info->irq = 0;
                }
                else
                {
                    const char *gpio_str;
                    i2c_info->irq = irq;
                    r = of_property_read_string(node, "gpio",&gpio_str);
                    if(r < 0)
                    {
                        printk("%s: faild to get gpio str for dev %s\n", __func__, i2c_info->type);
                        *gpio = -1; 
                    }
                    else
                    {
                        *gpio = amlogic_gpio_name_map_num(gpio_str);
                    }
                }
            }
            else
            {
                i2c_info->irq = 0;
            }
        }
    }
    
    return ret;
}

struct platform_device *sensor_dev;

int dt_sensor_setup_i2c_dev(struct device_node *node,  struct i2c_board_info *i2c_info, int *i2c_bus_nr, int *gpio)
{
        int ret = -1;
    
        int r;
        int irq;
        const char *status;


		r = of_property_read_string(node, "status", &status);
        if(r< 0){
            printk("%s: Failed to read status from device tree for dev %s\n", __func__, i2c_info->type);
            return -1;
        }

        if(strncmp("ok", status, 2) == 0)
        {
            u32 addr;
            const char *str;
            const char *name;

            r = of_property_read_string(node, "dev_name", &name);
            if(r< 0){
                printk("%s: Failed to read dev_name from device tree\n", __func__);
                return -1;
            } 

            strncpy(i2c_info->type, name, I2C_NAME_SIZE);

            r = of_property_read_u32(node, "address",&addr);
            if(r < 0)
            {
                printk("%s: faild to get i2c address for dev %s\n", __func__, i2c_info->type);
                return -1;
            }

            ret = 0;

            i2c_info->addr = addr;

            r = of_property_read_string(node, "i2c_bus", &str);
            if (r) {
                printk("%s: faild to get i2c_bus str for dev %s\n", __func__, i2c_info->type);
                *i2c_bus_nr = AML_I2C_BUS_B;
            } else {
                if (!strncmp(str, "i2c_bus_a", 9))
                    *i2c_bus_nr = AML_I2C_BUS_A;
                else if (!strncmp(str, "i2c_bus_b", 9))
                    *i2c_bus_nr = AML_I2C_BUS_B;
                else if (!strncmp(str, "i2c_bus_ao", 9))
                    *i2c_bus_nr = AML_I2C_BUS_AO;
                else
                    *i2c_bus_nr = AML_I2C_BUS_B;
            }


            r = of_property_read_u32(node, "irq",&irq);
            if(r < 0 || irq <= 0)
            {
                i2c_info->irq = 0;
            }
            else
            {
                const char *gpio_str;
                r = of_property_read_string(node, "gpio",&gpio_str);
                if(r < 0)
                {
                    printk("%s: faild to get gpio str for dev %s\n", __func__, i2c_info->type);
                    *gpio = -1; 
                    i2c_info->irq = 0;
                }
                else
                {
                    i2c_info->irq = irq;
                    *gpio = amlogic_gpio_name_map_num(gpio_str);
                }
            }
    } 
    
    return ret;
}


static int aml_sensor_probe(struct platform_device *pdev)
{
	printk("##############aml_sensor_probe start############\n");
	
	struct device_node* node = pdev->dev.of_node;
	struct device_node* child;
	struct i2c_board_info i2c_info;
	struct i2c_adapter *adapter;
    int i2c_bus_nr; 
    int gpio;

	sensor_dev = pdev;

	for_each_child_of_node(node, child) {

		memset(&i2c_info, 0, sizeof(i2c_info));

        if(!dt_sensor_setup_i2c_dev(child, &i2c_info, &i2c_bus_nr, &gpio))
        {

            adapter = i2c_get_adapter(i2c_bus_nr);
            if(!adapter)
               return -1;

            if(gpio > 0)
            {
                /* get gpio and set up isr */
            }
            i2c_new_device(adapter, &i2c_info);
        }
	}
	return 0;
}

static int aml_sensor_remove(struct platform_device *pdev)
{

}

static const struct of_device_id sensor_prober_dt_match[]={
	{	
		.compatible = "amlogic,aml_sensor",
	},
	{},
};

static struct platform_driver aml_sensor_prober_driver = {
	.probe		= aml_sensor_probe,
	.remove		= aml_sensor_remove,
	.driver		= {
		.name	= "aml_sensor",
		.owner	= THIS_MODULE,
		.of_match_table = sensor_prober_dt_match,
	},
};

static int __init aml_sensor_prober_init(void)
{
	int ret;

	ret = platform_driver_register(&aml_sensor_prober_driver);
	if (ret){
		printk(KERN_ERR"aml_cams_probre_driver register failed\n");
		return ret;
	}

	return ret;
}


static void __exit aml_sensor_prober_exit(void)
{
	platform_driver_unregister(&aml_sensor_prober_driver);
}

module_init(aml_sensor_prober_init);
module_exit(aml_sensor_prober_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Amlogic MEMS sensor prober driver");

