#ifndef __AML_AUDIO_CODEC_DEV__
#define __AML_AUDIO_CODEC_DEV__
#include <linux/list.h>
#include <linux/i2c.h>

#define AML_I2C_BUS_AO 0
#define AML_I2C_BUS_A 1
#define AML_I2C_BUS_B 2
#define AML_I2C_BUS_C 3
#define AML_I2C_BUS_D 4


typedef int(*aml_audio_codec_probe_fun_t)(struct i2c_adapter *);

typedef struct {
	const char* name;
	const char* status;
	unsigned i2c_bus_type;
	unsigned i2c_addr;
    unsigned capless;
}aml_audio_codec_info_t;

extern bool is_rt5631;
extern bool is_wm8960;


#endif
