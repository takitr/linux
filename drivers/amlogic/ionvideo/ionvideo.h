#ifndef _IONVIDEO_H
#define _IONVIDEO_H

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>
#include <media/v4l2-common.h>

#include <linux/mm.h>
#include <mach/mod_gate.h>

#include <linux/amlogic/amports/vframe.h>
#include <linux/amlogic/amports/vframe_provider.h>
#include <linux/amlogic/amports/vframe_receiver.h>
#include <linux/amlogic/ge2d/ge2d.h>
#include <linux/amlogic/amports/vframe.h>
#include <linux/amlogic/amports/canvas.h>

#include <linux/amlogic/vout/vout_notify.h>

#include <linux/amlogic/amports/timestamp.h>
#include <linux/amlogic/amports/tsync.h>
#include "videobuf2-ion.h"


/* Wake up at about 30 fps */
#define WAKE_NUMERATOR 30
#define WAKE_DENOMINATOR 1001

#define MAX_WIDTH 3840
#define MAX_HEIGHT 2160

#define PPMGR2_MAX_CANVAS 8
#define PPMGR2_CANVAS_INDEX 0x70

#define DUR2PTS(x) ((x) - ((x) >> 4))

#define dprintk(dev, level, fmt, arg...)                    \
    v4l2_dbg(level, debug, &dev->v4l2_dev, fmt, ## arg)

#define ppmgr2_printk(level, fmt, arg...)                   \
    do {                                                    \
        if (get_ionvideo_debug() >= level)                  \
            printk(KERN_DEBUG "ppmgr2-dev: " fmt, ## arg);  \
    } while (0)

/* ------------------------------------------------------------------
 Basic structures
 ------------------------------------------------------------------*/

struct ionvideo_fmt {
    char *name;
    u32 fourcc; /* v4l2 format id */
    u8 depth;
    bool is_yuv;
};

/* buffer for one video frame */
struct ionvideo_buffer {
    /* common v4l buffer stuff -- must be first */
    struct vb2_buffer vb;
    struct list_head list;
    struct ionvideo_fmt *fmt;
};

struct ionvideo_dmaqueue {
    struct list_head active;

    /* thread for generating video stream*/
    struct task_struct *kthread;
    wait_queue_head_t wq;
    /* Counters to control fps rate */
    int frame;
    int ini_jiffies;
};

struct ppmgr2_device {
    int dst_width;
    int dst_height;
    int ge2d_fmt;
    int canvas_id[PPMGR2_MAX_CANVAS];
    void* phy_addr[PPMGR2_MAX_CANVAS];
    int phy_size;
    int inited_canvas;

    ge2d_context_t* context;
    config_para_ex_t ge2d_config;

    int angle;
    int mirror;
    int paint_mode;
};

struct ionvideo_dev {
    struct list_head ionvideo_devlist;
    struct v4l2_device v4l2_dev;
    struct v4l2_ctrl_handler ctrl_handler;
    struct video_device vdev;

    /* controls */
    struct v4l2_ctrl *brightness;
    struct v4l2_ctrl *contrast;
    struct v4l2_ctrl *saturation;
    struct v4l2_ctrl *hue;
    struct {
        /* autogain/gain cluster */
        struct v4l2_ctrl *autogain;
        struct v4l2_ctrl *gain;
    };
    struct v4l2_ctrl *volume;
    struct v4l2_ctrl *alpha;
    struct v4l2_ctrl *button;
    struct v4l2_ctrl *boolean;
    struct v4l2_ctrl *int32;
    struct v4l2_ctrl *int64;
    struct v4l2_ctrl *menu;
    struct v4l2_ctrl *string;
    struct v4l2_ctrl *bitmask;
    struct v4l2_ctrl *int_menu;

    spinlock_t slock;
    struct mutex mutex;

    struct ionvideo_dmaqueue vidq;

    /* Several counters */
    unsigned ms;
    unsigned long jiffies;
    unsigned button_pressed;

    int mv_count; /* Controls bars movement */

    /* Input Number */
    int input;

    /* video capture */
    struct ionvideo_fmt *fmt;
    unsigned int width, height;
    struct vb2_queue vb_vidq;
    unsigned int field_count;

    u8 bars[9][3];
    u8 line[MAX_WIDTH * 8];
    unsigned int pixelsize;
    u8 alpha_component;

    struct ppmgr2_device ppmgr2_dev;
    struct vframe_receiver_s video_vf_receiver;
    u8 clear_list;
    u64 pts;
    u8 receiver_register;
};

int is_ionvideo_active(void);
unsigned get_ionvideo_debug(void);

int ppmgr2_init(struct ppmgr2_device *ppd);
int ppmgr2_canvas_config(struct ppmgr2_device *ppd, int dst_width, int dst_height, int dst_fmt, void* phy_addr, int index);
int ppmgr2_process(struct vframe_s* vf, struct ppmgr2_device *ppd, int index);
int ppmgr2_top_process(struct vframe_s* vf, struct ppmgr2_device *ppd, int index);
int ppmgr2_bottom_process(struct vframe_s* vf, struct ppmgr2_device *ppd, int index);
void ppmgr2_release(struct ppmgr2_device *ppd);
void ppmgr2_set_angle(struct ppmgr2_device *ppd, int angle);
void ppmgr2_set_mirror(struct ppmgr2_device *ppd, int mirror);
void ppmgr2_set_paint_mode(struct ppmgr2_device *ppd, int paint_mode);
int v4l_to_ge2d_format(int v4l2_format);

static inline void paint_mode_convert(int paint_mode, int* src_position, int* dst_paint_position, int* dst_plane_position);
static inline void ge2d_src_config(struct vframe_s* vf, config_para_ex_t* ge2d_config);
static inline void ge2d_mirror_config(int dst_mirror, config_para_ex_t* ge2d_config);
static inline void ge2d_angle_config(int dst_angle, config_para_ex_t* ge2d_config);
static int get_input_format(struct vframe_s* vf);
static int ge2d_paint_dst(ge2d_context_t *context, config_para_ex_t* ge2d_config, int dst_canvas_id, int dst_pixel_format, int* src_position, int* dst_paint_position, int* dst_plane_position);


#endif
