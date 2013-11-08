#ifndef  _VIDEO_PROT_H
#define _VIDEO_PROT_H

#include <linux/kernel.h>
#include <plat/regops.h>
#include <mach/am_regs.h>
#include <linux/amlogic/amlog.h>
#include <linux/amlogic/amports/vframe.h>
#include <linux/amlogic/amports/canvas.h>

#define  CUGT           0
#define  CID_VALUE      161
#define	 CID_MODE       6
#define	 REQ_ONOFF_EN   0
#define  REQ_ON_MAX     0
#define	 REQ_OFF_MIN    0
#define  PAT_VAL        0x00000000
#define  PAT_START_PTR  1
#define  PAT_END_PTR    1
#define  HOLD_LINES     14
#define  LITTLE_ENDIAN  0

typedef struct {
    u32 status;
    u32 video_started;
    u32 viu_type;
    u32 power_down;
    u32 x_end;
    u32 y_end;
    u32 y_step;
    u32 pat_val;
    u32 is_4k2k;
    u32 angle;
    u32 angle_changed;
    u32 src_vframe_width;
    u32 src_vframe_height;
    u32 src_vframe_ratio;
    u32 src_vframe_orientation;
} video_prot_t;

//extern void early_init_prot();
void video_prot_init(video_prot_t* video_prot, vframe_t *vf);
void video_prot_set_angle(video_prot_t* video_prot, u32 angle);
void video_prot_revert_vframe(video_prot_t* video_prot, vframe_t *vf);
void video_prot_set_canvas(vframe_t *vf);
void video_prot_reset(video_prot_t* video_prot);
int get_prot_on(void);
u32 get_video_angle(void);
void set_video_angle(u32 s_value);
#endif
