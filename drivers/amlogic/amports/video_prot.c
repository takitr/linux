/*
 * Amlogic Meson
 * frame buffer driver
 *
 * Copyright (C) 2013 Amlogic, Inc.
 *
 * Author:  Amlogic R&D Group
 *
 */
#include <linux/amlogic/amports/video_prot.h>

static int set_prot_NV21(u32 x_start, u32 x_end, u32 y_start, u32 y_end, u32 y_step, u32 angle, u32 pat_val) {

    u32 data32;
    if (angle == 0 || angle == 2) {
           data32 = (HOLD_LINES << 8) | (LITTLE_ENDIAN << 7) | (0 << 6) | (0 << 4) | (0 << 3) | (0 << 2) | (0 << 0);
           aml_write_reg32(P_VPU_PROT2_GEN_CNTL, data32);
           data32 = (HOLD_LINES << 8) | (LITTLE_ENDIAN << 7) | (0 << 6) | (1 << 4) | (0 << 3) | (0 << 2) | (0 << 0);
           aml_write_reg32(P_VPU_PROT3_GEN_CNTL, data32);

           return 0;
    }

    u32 x_start_uv = x_start >> 1;
    u32 x_end_uv = x_end >> 1;
    u32 y_start_uv = y_start >> 1;
    u32 y_end_uv = y_end >> 1;
    u32 y_len = (y_end - y_start) / (y_step + 1);
    u32 y_len_uv = (y_end_uv - y_start_uv) / (y_step + 1);

    y_end = y_start + (y_step + 1) * y_len;
    y_end_uv = y_start_uv + (y_step + 1) * y_len_uv;

    data32 = (x_end << 16) | (x_start << 0);
    aml_write_reg32(P_VPU_PROT2_X_START_END, data32);
    data32 = (x_end_uv << 16) | (x_start_uv << 0);
    aml_write_reg32(P_VPU_PROT3_X_START_END, data32);

    data32 = (y_end << 16) | (y_start << 0);
    aml_write_reg32(P_VPU_PROT2_Y_START_END, data32);
    data32 = (y_end_uv << 16) | (y_start_uv << 0);
    aml_write_reg32(P_VPU_PROT3_Y_START_END, data32);

    data32 = (y_step << 16) | (y_len << 0);
    aml_write_reg32(P_VPU_PROT2_Y_LEN_STEP, data32);
    data32 = (y_step << 16) | (y_len_uv << 0);
    aml_write_reg32(P_VPU_PROT3_Y_LEN_STEP, data32);

    data32 = (PAT_START_PTR << 4) | (PAT_END_PTR << 0);
    aml_write_reg32(P_VPU_PROT2_RPT_LOOP, data32);
    aml_write_reg32(P_VPU_PROT3_RPT_LOOP, data32);
    aml_write_reg32(P_VPU_PROT2_RPT_PAT, pat_val);
    aml_write_reg32(P_VPU_PROT3_RPT_PAT, pat_val);
    data32 = (CUGT << 20) | (CID_MODE << 16) | (CID_VALUE << 8);
    aml_write_reg32(P_VPU_PROT2_DDR, data32);
    aml_write_reg32(P_VPU_PROT3_DDR, data32);
    data32 = (REQ_ONOFF_EN << 31) | (REQ_OFF_MIN << 16) | (REQ_ON_MAX << 0);
    aml_write_reg32(P_VPU_PROT2_REQ_ONOFF, data32);
    aml_write_reg32(P_VPU_PROT3_REQ_ONOFF, data32);

    if (angle == 1) {
        data32 = (HOLD_LINES << 8) | (LITTLE_ENDIAN << 7) | (0 << 6) | (0 << 4) | (1 << 3) | (0 << 2) | (1 << 0);
        aml_write_reg32(P_VPU_PROT2_GEN_CNTL, data32);
        data32 = (HOLD_LINES << 8) | (LITTLE_ENDIAN << 7) | (0 << 6) | (1 << 4) | (1 << 3) | (0 << 2) | (1 << 0);
        aml_write_reg32(P_VPU_PROT3_GEN_CNTL, data32);
    } else if (angle == 3) {
        data32 = (HOLD_LINES << 8) | (LITTLE_ENDIAN << 7) | (0 << 6) | (0 << 4) | (0 << 3) | (1 << 2) | (1 << 0);
        aml_write_reg32(P_VPU_PROT2_GEN_CNTL, data32);
        data32 = (HOLD_LINES << 8) | (LITTLE_ENDIAN << 7) | (0 << 6) | (1 << 4) | (0 << 3) | (1 << 2) | (1 << 0);
        aml_write_reg32(P_VPU_PROT3_GEN_CNTL, data32);
    }

    return 0;
}

static int set_prot_422(u32 x_start, u32 x_end, u32 y_start, u32 y_end, u32 y_step, u32 angle, u32 pat_val) {

    u32 data32;

    if (angle == 0 || angle == 2) {
           data32 = (HOLD_LINES << 8) | (LITTLE_ENDIAN << 7) | (0 << 6) | (0 << 4) | (0 << 3) | (0 << 2) | (0 << 0);
           aml_write_reg32(P_VPU_PROT2_GEN_CNTL, data32);
           data32 = (HOLD_LINES << 8) | (LITTLE_ENDIAN << 7) | (0 << 6) | (1 << 4) | (0 << 3) | (0 << 2) | (0 << 0);
           aml_write_reg32(P_VPU_PROT3_GEN_CNTL, data32);

           return 0;
    }

    u32 x_start_uv = x_start >> 1;
    u32 x_end_uv = x_end >> 1;
    u32 y_start_uv = y_start >> 1;
    u32 y_end_uv = y_end;
    u32 y_len = (y_end - y_start) / (y_step + 1);
    u32 y_len_uv = (y_end_uv - y_start_uv) / (y_step + 1);

    y_end = y_start + (y_step + 1) * y_len;
    y_end_uv = y_start_uv + (y_step + 1) * y_len_uv;

    data32 = (x_end << 16) | (x_start << 0);
    aml_write_reg32(P_VPU_PROT2_X_START_END, data32);
    data32 = (x_end_uv << 16) | (x_start_uv << 0);
    aml_write_reg32(P_VPU_PROT3_X_START_END, data32);

    data32 = (y_end << 16) | (y_start << 0);
    aml_write_reg32(P_VPU_PROT2_Y_START_END, data32);
    data32 = (y_end_uv << 16) | (y_start_uv << 0);
    aml_write_reg32(P_VPU_PROT3_Y_START_END, data32);

    data32 = (y_step << 16) | (y_len << 0);
    aml_write_reg32(P_VPU_PROT2_Y_LEN_STEP, data32);
    data32 = (y_step << 16) | (y_len_uv << 0);
    aml_write_reg32(P_VPU_PROT3_Y_LEN_STEP, data32);

    data32 = (PAT_START_PTR << 4) | (PAT_END_PTR << 0);
    aml_write_reg32(P_VPU_PROT2_RPT_LOOP, data32);
    aml_write_reg32(P_VPU_PROT3_RPT_LOOP, data32);
    aml_write_reg32(P_VPU_PROT2_RPT_PAT, pat_val);
    aml_write_reg32(P_VPU_PROT3_RPT_PAT, pat_val);
    data32 = (CUGT << 20) | (CID_MODE << 16) | (CID_VALUE << 8);
    aml_write_reg32(P_VPU_PROT2_DDR, data32);
    aml_write_reg32(P_VPU_PROT3_DDR, data32);
    data32 = (REQ_ONOFF_EN << 31) | (REQ_OFF_MIN << 16) | (REQ_ON_MAX << 0);
    aml_write_reg32(P_VPU_PROT2_REQ_ONOFF, data32);
    aml_write_reg32(P_VPU_PROT3_REQ_ONOFF, data32);

    if (angle == 1) {
        data32 = (HOLD_LINES << 8) | (LITTLE_ENDIAN << 7) | (0 << 6) | (0 << 4) | (1 << 3) | (0 << 2) | (1 << 0);
        aml_write_reg32(P_VPU_PROT2_GEN_CNTL, data32);
        data32 = (HOLD_LINES << 8) | (LITTLE_ENDIAN << 7) | (0 << 6) | (1 << 4) | (1 << 3) | (0 << 2) | (1 << 0);
        aml_write_reg32(P_VPU_PROT3_GEN_CNTL, data32);
    } else if (angle == 3) {
        data32 = (HOLD_LINES << 8) | (LITTLE_ENDIAN << 7) | (0 << 6) | (0 << 4) | (0 << 3) | (1 << 2) | (1 << 0);
        aml_write_reg32(P_VPU_PROT2_GEN_CNTL, data32);
        data32 = (HOLD_LINES << 8) | (LITTLE_ENDIAN << 7) | (0 << 6) | (1 << 4) | (0 << 3) | (1 << 2) | (1 << 0);
        aml_write_reg32(P_VPU_PROT3_GEN_CNTL, data32);
    }

    return 0;
}

static int set_prot_444(u32 x_start, u32 x_end, u32 y_start, u32 y_end, u32 y_step, u32 angle, u32 pat_val) {

    u32 data32;

    if (angle == 0 || angle == 2) {
           data32 = (HOLD_LINES << 8) | (LITTLE_ENDIAN << 7) | (0 << 6) | (0 << 4) | (0 << 3) | (0 << 2) | (0 << 0);
           aml_write_reg32(P_VPU_PROT2_GEN_CNTL, data32);

           return 0;
    }
    u32 y_len = (y_end - y_start) / (y_step + 1);

    y_end = y_start + (y_step + 1) * y_len;

    data32 = (x_end << 16) | (x_start << 0);
    aml_write_reg32(P_VPU_PROT2_X_START_END, data32);

    data32 = (y_end << 16) | (y_start << 0);
    aml_write_reg32(P_VPU_PROT2_Y_START_END, data32);

    data32 = (y_step << 16) | (y_len << 0);
    aml_write_reg32(P_VPU_PROT2_Y_LEN_STEP, data32);

    data32 = (PAT_START_PTR << 4) | (PAT_END_PTR << 0);
    aml_write_reg32(P_VPU_PROT2_RPT_LOOP, data32);
    aml_write_reg32(P_VPU_PROT2_RPT_PAT, pat_val);
    data32 = (CUGT << 20) | (CID_MODE << 16) | (CID_VALUE << 8);
    aml_write_reg32(P_VPU_PROT2_DDR, data32);
    data32 = (REQ_ONOFF_EN << 31) | (REQ_OFF_MIN << 16) | (REQ_ON_MAX << 0);
    aml_write_reg32(P_VPU_PROT2_REQ_ONOFF, data32);

    if (angle == 1) {
        data32 = (HOLD_LINES << 8) | (LITTLE_ENDIAN << 7) | (0 << 6) | (0 << 4) | (1 << 3) | (0 << 2) | (1 << 0);
        aml_write_reg32(P_VPU_PROT2_GEN_CNTL, data32);
    } else if (angle == 3) {
        data32 = (HOLD_LINES << 8) | (LITTLE_ENDIAN << 7) | (0 << 6) | (0 << 4) | (0 << 3) | (1 << 2) | (1 << 0);
        aml_write_reg32(P_VPU_PROT2_GEN_CNTL, data32);
    }

    return 0;
}

void video_prot_reset(video_prot_t* video_prot) {
    u32 data32;

    aml_write_reg32(P_VPU_PROT2_CLK_GATE, video_prot->status);
    aml_write_reg32(P_VPU_PROT3_CLK_GATE, video_prot->status);
    aml_write_reg32(P_VD1_IF0_PROT_CNTL, (video_prot->status) << 31 | 1080 << 16 | 1080);
    aml_set_reg32_bits(P_VPU_PROT2_MMC_CTRL, video_prot->status, 12, 3);
    aml_set_reg32_bits(P_VPU_PROT3_MMC_CTRL, video_prot->status, 12, 3);

}

void video_prot_init(video_prot_t* video_prot, vframe_t *vf) {
    u32 tmp;

    if (vf->width > 1920 || vf->height > 1088) {
        video_prot->is_4k2k = 1;
        video_prot->y_step = 1;
        video_prot->pat_val = 0x80;
    } else {
        video_prot->is_4k2k = 0;
        video_prot->y_step = 0;
        video_prot->pat_val = 0x0;
    }
    video_prot->src_vframe_ratio = (vf->ratio_control & DISP_RATIO_ASPECT_RATIO_MASK) >> DISP_RATIO_ASPECT_RATIO_BIT;
    video_prot->src_vframe_width = vf->width;
    video_prot->src_vframe_height = vf->height;
    video_prot->x_end = vf->width - 1;
    video_prot->y_end = vf->height - 1;
    video_prot->viu_type = vf->type;
    video_prot->src_vframe_orientation = vf->orientation;

}

void video_prot_set_angle(video_prot_t* video_prot, u32 angle) {

    u32 data32;
    u32 angle_orientation = (angle + video_prot->src_vframe_orientation) % 4;

    video_prot->angle = angle;

    if (video_prot->viu_type & VIDTYPE_VIU_NV21) {
        set_prot_NV21(0, video_prot->x_end, 0, video_prot->y_end, video_prot->y_step, angle_orientation, video_prot->pat_val);
    } else if (video_prot->viu_type & VIDTYPE_VIU_422) {
        set_prot_422(0, video_prot->x_end, 0, video_prot->y_end, video_prot->y_step, angle_orientation, video_prot->pat_val);
    } else if (video_prot->viu_type & VIDTYPE_VIU_444) {
        set_prot_444(0, video_prot->x_end, 0, video_prot->y_end, video_prot->y_step, angle_orientation, video_prot->pat_val);
    } else {
        set_prot_NV21(0, video_prot->x_end, 0, video_prot->y_end, video_prot->y_step, 0, video_prot->pat_val);
    }
    video_prot->status = angle_orientation % 2;
}

void video_prot_revert_vframe(video_prot_t* video_prot, vframe_t *vf) {
    u32 angle_orientation = (video_prot->angle + video_prot->src_vframe_orientation) % 4;

    if (video_prot->viu_type & (VIDTYPE_VIU_444 | VIDTYPE_VIU_422 | VIDTYPE_VIU_NV21)) {
        if (angle_orientation == 1 || angle_orientation == 3) {
            if (video_prot->is_4k2k) {
                vf->width = video_prot->src_vframe_height / (video_prot->y_step + 1);
                vf->height = video_prot->src_vframe_width >> 1;
            } else {
                vf->width = video_prot->src_vframe_height;
                vf->height = video_prot->src_vframe_width;
            }
            if (video_prot->src_vframe_ratio != 0) {
                vf->ratio_control &= ~DISP_RATIO_ASPECT_RATIO_MASK;
                vf->ratio_control |= (0x10000 / video_prot->src_vframe_ratio) << DISP_RATIO_ASPECT_RATIO_BIT;
            }
        } else if (angle_orientation == 0 || angle_orientation == 2) {
            vf->width = video_prot->src_vframe_width;
            vf->height = video_prot->src_vframe_height;
            if (video_prot->src_vframe_ratio != 0) {
                vf->ratio_control &= ~DISP_RATIO_ASPECT_RATIO_MASK;
                vf->ratio_control |= video_prot->src_vframe_ratio << DISP_RATIO_ASPECT_RATIO_BIT;
            }
        }
    }

}

void video_prot_set_canvas(vframe_t *vf) {
    aml_set_reg32_bits(P_VPU_PROT2_DDR, vf->canvas0Addr & 0xff, 0, 8);
    if (!(vf->type & VIDTYPE_VIU_444)) {
        aml_set_reg32_bits(P_VPU_PROT3_DDR, (vf->canvas0Addr >> 8) & 0xff, 0, 8);
    }
}
