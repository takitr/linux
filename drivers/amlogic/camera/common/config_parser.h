#ifndef CONFIG_PARSER
#define CONFIG_PARSER

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/amlogic/tvin/tvin_v4l2.h>

#define EFFECT_MAX 16
#define AET_MAX 32
#define HW_MAX 16
#define WB_MAX 9
#define CAPTURE_MAX 5
#define GAMMA_MAX 257
#define SCENE_MAX 202
#define WB_SENSOR_MAX 6
#define BUFFER_SIZE 1024

enum error_code {
	NO_MEM = 1,
	READ_ERROR,
	WRONG_FORMAT,
	CHECK_FAILED,
	HEAD_FAILED,
	BODY_HEAD_FAILED,
	BODY_ELEMENT_FAILED,
};

typedef union{
    int i_val;
    float f_val;
}import_type;

typedef struct{
    int num;
    char name[40];
    unsigned int export[18];
}effect_type;

typedef struct{
    int sum;
    effect_type eff[EFFECT_MAX];
}effect_struct;

typedef struct{
    int num;
    char name[40];
    int export[64];
}hw_type;

typedef struct{
    int sum;
    hw_type hw[HW_MAX];
}hw_struct;

typedef struct{
    int num;
    char name[40];
    int export[2];
}wb_type;

typedef struct{
    int sum;
    wb_type wb[WB_MAX];
}wb_struct;


typedef struct{
    int num;
    char name[40];
    int export[SCENE_MAX];
}scene_type;

typedef struct{
    int sum;
    scene_type *scene;
}scene_struct;

typedef struct{
	int num;
	char name[40];
	int export[10];	
}capture_type;

typedef struct{
	int sum;
	capture_type capture[CAPTURE_MAX];
}capture_struct;

typedef struct sensor_aet_s {
    unsigned int exp;
    unsigned int ag;
    unsigned int vts;
    unsigned int gain;
    unsigned int fr;
} sensor_aet_t;

typedef struct sensor_aet_info_s {
    unsigned int fmt_main_fr;
    unsigned int fmt_capture; // false: preview, true: capture
    unsigned int fmt_hactive;
    unsigned int fmt_vactive;
    unsigned int fmt_rated_fr;
    unsigned int fmt_min_fr;
    unsigned int tbl_max_step;
    unsigned int tbl_rated_step;
    unsigned int tbl_max_gain;
    unsigned int tbl_min_gain;
} sensor_aet_info_t;


typedef struct{
    int num;
    char name[40];
    sensor_aet_info_t *info;
    sensor_aet_t *aet_table;
    int *manual;
}aet_type;

typedef struct{
    int sum;
    aet_type aet[AET_MAX];
}aet_struct;

typedef struct{
	int export[12];	
}wave_struct;

typedef struct{
	int export[1024];
}lenc_struct;

typedef struct{
	unsigned short gamma_r[GAMMA_MAX];
	unsigned short gamma_g[GAMMA_MAX];
	unsigned short gamma_b[GAMMA_MAX];
}gamma_struct;

typedef struct{
    int export[WB_SENSOR_MAX];
}wb_sensor_struct;

typedef struct{
    effect_struct eff;
    int effect_valid;
    hw_struct hw;
    int hw_valid;
    aet_struct aet; 
    int aet_valid;
    capture_struct capture;
    int capture_valid;
    scene_struct scene;
    int scene_valid;
    wb_struct wb;
    int wb_valid;
    wave_struct wave;
    int wave_valid;
    lenc_struct lenc;
    int lenc_valid;
    gamma_struct gamma;
    int gamma_valid;
    wb_sensor_struct wb_sensor_data;
    int wb_sensor_data_valid;
}configure;

typedef struct{
	unsigned int effect_index;
	unsigned int scenes_index;
	unsigned int wb_index;
	unsigned int capture_index;
}para_index_t;

typedef struct{
	camera_wb_flip_t wb;
	char *name;
}wb_pair_t;

typedef struct{
	camera_special_effect_t effect;
	char *name;
}effect_pair_t;

typedef struct sensor_dg_s {
    unsigned short r;
    unsigned short g;
    unsigned short b;
    unsigned short dg_default;
}sensor_dg_t;

int parse_config(char *path);
int generate_para(cam_parameter_t *para,para_index_t pindex);
void free_para(cam_parameter_t *para);

unsigned int get_aet_current_step(void);
short get_aet_current_gain(void);
short get_aet_min_gain(void);
short get_aet_max_gain(void);
unsigned int get_aet_max_step(void);
short get_aet_gain_by_step(unsigned int new_step);


int my_i2c_put_byte(struct i2c_adapter *adapter,unsigned short i2c_addr,unsigned short addr,unsigned char data);
int my_i2c_put_byte_add8(struct i2c_adapter *adapter,unsigned short i2c_addr,char *buf,int len);
int my_i2c_get_byte(struct i2c_adapter *adapter,unsigned short i2c_addr,unsigned short addr);
int my_i2c_get_word(struct i2c_adapter *adapter,unsigned short i2c_addr);
#endif

