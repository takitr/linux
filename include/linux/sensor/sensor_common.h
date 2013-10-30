#ifndef SENSOR_COMMON_H
#define SENSOR_COMMON_H

#include <linux/i2c.h>

int sensor_setup_i2c_dev(struct i2c_board_info *i2c_info, int *i2c_bus_nr, int *gpio);

#endif
