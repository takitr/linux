
#define GPIO_REG_BIT(reg,bit) ((reg<<5)|bit)
#define GPIO_REG(value) ((value>>5))
#define GPIO_BIT(value) ((value&0x1F))

struct amlogic_gpio_desc{
	unsigned num;
	char *name;
	unsigned int out_en_reg_bit;
	unsigned int out_value_reg_bit;
	unsigned int input_value_reg_bit;
	unsigned int map_to_irq;
	unsigned int pull_up_reg_bit;
	const char *gpio_owner;
};
struct amlogic_set_pullup{
	 int (*meson_set_pullup)(unsigned int,unsigned int,unsigned int);
};
