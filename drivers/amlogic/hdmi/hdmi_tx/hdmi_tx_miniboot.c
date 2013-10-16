#include "hdmi_tx_miniboot.h"

#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/major.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/proc_fs.h> 
#include <linux/logo/logo.h>

#include <mach/am_regs.h>
#include <mach/io.h>
#include <mach/register.h>

#include "hdmi_parameter.h"
#include "test_prm.h"

#define VFIFO2VD_TO_HDMI_LATENCY    2   // Latency in pixel clock from ENCP_VFIFO2VD request to data ready to HDMI
    #define NUM_INT_VSYNC   INT_VEC_VIU1_VSYNC

void hdmi_test_function(void);
unsigned long modulo(unsigned long a, unsigned long b);
signed int to_signed(unsigned int a);

static void aml_write_reg32_op(unsigned int _reg, const unsigned _value)
{
    aml_write_reg32(_reg, _value);
#ifdef HDMI_MINIBOOT_DEBUG
    printk(" A:0x%08x 0x%04x  wD:0x%x ", _reg, (_reg & 0xffff)>>2, _value);
    if(_value == aml_read_reg32(_reg))
        printk("\n");
    else
        printk(" rD: 0x%08x\n", aml_read_reg32(_reg));
#endif
}

static unsigned int aml_read_reg32_op(unsigned int _reg)
{
    return aml_read_reg32(_reg);
}

static void aml_set_reg32_bits_op(uint32_t _reg, const uint32_t _val, const uint32_t _start, const uint32_t _len)
{
    unsigned int tmp;
    tmp = (aml_read_reg32(_reg) & ~(((1L<<(_len))-1)<<(_start))) | ((unsigned int)(_val) << (_start));
    aml_write_reg32_op(_reg, tmp);
}

static void hdmi_wr_reg(unsigned long addr, unsigned long data)
{
    aml_write_reg32_op(P_HDMI_ADDR_PORT, addr);
    aml_write_reg32_op(P_HDMI_ADDR_PORT, addr);
    aml_write_reg32_op(P_HDMI_DATA_PORT, data);
}
//FILE: test.c

// --------------------------------------------------------
//                     C_Entry
// --------------------------------------------------------
unsigned char   field_n = 0;

void set_hdmi_enc(void)
{
    unsigned long total_pixels_venc = (TOTAL_PIXELS  / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC);
    unsigned long active_pixels_venc= (ACTIVE_PIXELS / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC);
    unsigned long front_porch_venc  = (FRONT_PORCH   / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC);
    unsigned long hsync_pixels_venc = (HSYNC_PIXELS  / (1+PIXEL_REPEAT_HDMI)) * (1+PIXEL_REPEAT_VENC);

    unsigned long de_h_begin, de_h_end;
    unsigned long de_v_begin_even, de_v_end_even, de_v_begin_odd, de_v_end_odd;
    unsigned long hs_begin, hs_end;
    unsigned long vs_adjust;
    unsigned long vs_bline_evn, vs_eline_evn, vs_bline_odd, vs_eline_odd;
    unsigned long vso_begin_evn, vso_begin_odd;

    de_h_begin = modulo(aml_read_reg32_op(P_ENCP_VIDEO_HAVON_BEGIN) + VFIFO2VD_TO_HDMI_LATENCY,  total_pixels_venc);
    de_h_end   = modulo(de_h_begin + active_pixels_venc,                        total_pixels_venc);
    aml_write_reg32_op(P_ENCP_DE_H_BEGIN, de_h_begin);
    aml_write_reg32_op(P_ENCP_DE_H_END,   de_h_end);
    // Program DE timing for even field
    de_v_begin_even = aml_read_reg32_op(P_ENCP_VIDEO_VAVON_BLINE);
    de_v_end_even   = modulo(de_v_begin_even + ACTIVE_LINES, TOTAL_LINES);
    aml_write_reg32_op(P_ENCP_DE_V_BEGIN_EVEN,de_v_begin_even);
    aml_write_reg32_op(P_ENCP_DE_V_END_EVEN,  de_v_end_even);
    // Program DE timing for odd field if needed
    if (INTERLACE_MODE) {
        // Calculate de_v_begin_odd according to enc480p_timing.v:
        //wire[10:0]	cfg_ofld_vavon_bline	= {{7{ofld_vavon_ofst1 [3]}},ofld_vavon_ofst1 [3:0]} + cfg_video_vavon_bline	+ ofld_line;
        de_v_begin_odd  = to_signed((aml_read_reg32_op(P_ENCP_VIDEO_OFLD_VOAV_OFST) & 0xf0)>>4) + de_v_begin_even + (TOTAL_LINES-1)/2;
        de_v_end_odd    = modulo(de_v_begin_odd + ACTIVE_LINES, TOTAL_LINES);
        aml_write_reg32_op(P_ENCP_DE_V_BEGIN_ODD, de_v_begin_odd);
        aml_write_reg32_op(P_ENCP_DE_V_END_ODD,   de_v_end_odd);
    }

    // Program Hsync timing
    if (de_h_end + front_porch_venc >= total_pixels_venc) {
        hs_begin    = de_h_end + front_porch_venc - total_pixels_venc;
        vs_adjust   = 1;
    } else {
        hs_begin    = de_h_end + front_porch_venc;
        vs_adjust   = 0;
    }
    hs_end  = modulo(hs_begin + hsync_pixels_venc,   total_pixels_venc);
    aml_write_reg32_op(P_ENCP_DVI_HSO_BEGIN,  hs_begin);
    aml_write_reg32_op(P_ENCP_DVI_HSO_END,    hs_end);
    
    // Program Vsync timing for even field
    if (de_v_begin_even >= SOF_LINES + VSYNC_LINES + (1-vs_adjust)) {
        vs_bline_evn = de_v_begin_even - SOF_LINES - VSYNC_LINES - (1-vs_adjust);
    } else {
        vs_bline_evn = TOTAL_LINES + de_v_begin_even - SOF_LINES - VSYNC_LINES - (1-vs_adjust);
    }
    vs_eline_evn = modulo(vs_bline_evn + VSYNC_LINES, TOTAL_LINES);
    aml_write_reg32_op(P_ENCP_DVI_VSO_BLINE_EVN, vs_bline_evn);
    aml_write_reg32_op(P_ENCP_DVI_VSO_ELINE_EVN, vs_eline_evn);
    vso_begin_evn = hs_begin;
    aml_write_reg32_op(P_ENCP_DVI_VSO_BEGIN_EVN, vso_begin_evn);
    aml_write_reg32_op(P_ENCP_DVI_VSO_END_EVN,   vso_begin_evn);
    // Program Vsync timing for odd field if needed
    if (INTERLACE_MODE) {
        vs_bline_odd = de_v_begin_odd-1 - SOF_LINES - VSYNC_LINES;
        vs_eline_odd = de_v_begin_odd-1 - SOF_LINES;
        vso_begin_odd   = modulo(hs_begin + (total_pixels_venc>>1), total_pixels_venc);
        aml_write_reg32_op(P_ENCP_DVI_VSO_BLINE_ODD, vs_bline_odd);
        aml_write_reg32_op(P_ENCP_DVI_VSO_ELINE_ODD, vs_eline_odd);
        aml_write_reg32_op(P_ENCP_DVI_VSO_BEGIN_ODD, vso_begin_odd);
        aml_write_reg32_op(P_ENCP_DVI_VSO_END_ODD,   vso_begin_odd);
    }
    aml_write_reg32_op(P_VPU_HDMI_SETTING, (0                                 << 0) | // [    0] src_sel_enci
                         (0                                 << 1) | // [    1] src_sel_encp
                         (HSYNC_POLARITY                    << 2) | // [    2] inv_hsync. 1=Invert Hsync polarity.
                         (VSYNC_POLARITY                    << 3) | // [    3] inv_vsync. 1=Invert Vsync polarity.
                         (0                                 << 4) | // [    4] inv_dvi_clk. 1=Invert clock to external DVI, (clock invertion exists at internal HDMI).
                         (((TX_INPUT_COLOR_FORMAT==0)?1:0)  << 5) | // [ 7: 5] data_comp_map. Input data is CrYCb(BRG), map the output data to desired format:
                                                                    //                          0=output CrYCb(BRG);
                                                                    //                          1=output YCbCr(RGB);
                                                                    //                          2=output YCrCb(RBG);
                                                                    //                          3=output CbCrY(GBR);
                                                                    //                          4=output CbYCr(GRB);
                                                                    //                          5=output CrCbY(BGR);
                                                                    //                          6,7=Rsrv.
                         (0                                 << 8) | // [11: 8] wr_rate. 0=A write every clk1; 1=A write every 2 clk1; ...; 15=A write every 16 clk1.
                         (0                                 <<12)   // [15:12] rd_rate. 0=A read every clk2; 1=A read every 2 clk2; ...; 15=A read every 16 clk2.
    );
    aml_set_reg32_bits_op(P_VPU_HDMI_SETTING, 1, 1, 1);  // [    1] src_sel_encp: Enable ENCP output to HDMI
    aml_write_reg32_op(P_ENCP_VIDEO_EN, 1); // Enable VENC
}

void clocks_set_sys_defaults(void)
{
    aml_write_reg32_op(P_HHI_HDMI_CLK_CNTL,  ((1 << 9)  |   // select "fclk_div4" PLL
                             (1 << 8)  |   // Enable gated clock
                             (6 << 0)) );  // Divide by 7
    aml_write_reg32_op(P_HHI_VID_PLL_CNTL,   (1    << 30)    | 
                            (0    << 29)    |
                            (0    << 18)    |   // HDMI direct divide by 1
                            (1    << 16)    |   // HDMI (lvds divider) divide by 2
                            (4    << 10)    | 
                            (495  << 0) );
    //aml_write_reg32_op(P_HHI_VPU_CLK_CNTL,   0x101 );   //TODO DELETE LATER
    //aml_write_reg32_op(P_HHI_VPU_CLK_CNTL, (aml_read_reg32_op(P_HHI_VPU_CLK_CNTL) | (1 << 8)) ); //moved to vpu.c, default config by dts
    //aml_write_reg32_op(P_HHI_VPU_MEM_PD_REG0, 0x00000000 );
    //aml_write_reg32_op(P_HHI_VPU_MEM_PD_REG1, 0x00000000 );

    //aml_write_reg32_op(P_VPU_MEM_PD_REG0, 0x00000000 );
    //aml_write_reg32_op(P_VPU_MEM_PD_REG1, 0x00000000 );
}

void vclk_set_encp_1920x1080( int pll_sel, int pll_div_sel, int vclk_sel, int upsample)
{
    unsigned long pll_reg;
    unsigned long vid_div_reg;
    unsigned int xd; 
    unsigned long data32;


#ifdef NO_2ND_PLL
    pll_sel = 0;
#endif

    if (pll_sel) { // Setting for VID2_PLL
        pll_reg = 0x00020863;
        vid_div_reg = 0x00010803;
        xd = 2; 
    } else { // Setting for VID_PLL, consideration for supporting HDMI clock
        pll_reg = 0x400611ef;
        vid_div_reg = 0x00010843;
        xd = 1; 
    }

    vid_div_reg |= (1 << 16) ; // turn clock gate on
    vid_div_reg |= (pll_sel << 15); // vid_div_clk_sel
    
   
    if(vclk_sel) {
      aml_write_reg32_op(P_HHI_VIID_CLK_CNTL, aml_read_reg32_op(P_HHI_VIID_CLK_CNTL) & ~(1 << 19) );     //disable clk_div0 
    }
    else {
      aml_write_reg32_op(P_HHI_VID_CLK_CNTL, aml_read_reg32_op(P_HHI_VID_CLK_CNTL) & ~(1 << 19) );     //disable clk_div0 
      aml_write_reg32_op(P_HHI_VID_CLK_CNTL, aml_read_reg32_op(P_HHI_VID_CLK_CNTL) & ~(1 << 20) );     //disable clk_div1 
    } 

    msleep(5);

    if(pll_sel) aml_write_reg32_op(P_HHI_VID2_PLL_CNTL,       pll_reg );    
    else aml_write_reg32_op(P_HHI_VID_PLL_CNTL,       pll_reg );

    if(pll_div_sel ) {
      aml_write_reg32_op(P_HHI_VIID_DIVIDER_CNTL,   vid_div_reg);
    }
    else {
      aml_write_reg32_op(P_HHI_VID_DIVIDER_CNTL,   vid_div_reg);
    }

    if(vclk_sel) aml_write_reg32_op(P_HHI_VIID_CLK_DIV, (aml_read_reg32_op(P_HHI_VIID_CLK_DIV) & ~(0xFF << 0)) | (xd-1) );   // setup the XD divider value
    else aml_write_reg32_op(P_HHI_VID_CLK_DIV, (aml_read_reg32_op(P_HHI_VID_CLK_DIV) & ~(0xFF << 0)) | (xd-1) );   // setup the XD divider value

    msleep(5);    

    if(vclk_sel) {
      if(pll_div_sel) aml_set_reg32_bits_op(P_HHI_VIID_CLK_CNTL, 4, 16, 3);  // Bit[18:16] - v2_cntl_clk_in_sel
      else aml_set_reg32_bits_op(P_HHI_VIID_CLK_CNTL, 0, 16, 3);  // Bit[18:16] - cntl_clk_in_sel
      aml_write_reg32_op(P_HHI_VIID_CLK_CNTL, aml_read_reg32_op(P_HHI_VIID_CLK_CNTL) |  (1 << 19) );     //enable clk_div0 
    }
    else {
      aml_set_reg32_bits_op(P_HHI_VID_CLK_DIV,
                        1,              //divide 2 for clk_div1
                        8, 8);
    
      if(pll_div_sel) aml_set_reg32_bits_op(P_HHI_VID_CLK_CNTL, 4, 16, 3);  // Bit[18:16] - v2_cntl_clk_in_sel
      else aml_set_reg32_bits_op(P_HHI_VID_CLK_CNTL, 0, 16, 3);  // Bit[18:16] - cntl_clk_in_sel
      aml_write_reg32_op(P_HHI_VID_CLK_CNTL, aml_read_reg32_op(P_HHI_VID_CLK_CNTL) |  (1 << 19) );     //enable clk_div0 
    }
    
    msleep(5);

    if(vclk_sel) {
      aml_set_reg32_bits_op(P_HHI_VIID_CLK_CNTL, 
                   (1<<0),  // Enable cntl_div1_en
                   0, 1    // cntl_div1_en
                   );
      aml_set_reg32_bits_op(P_HHI_VID_CLK_DIV, 
                   8,      // select clk_div1 
                   24, 4); // [23:20] encp_clk_sel 
      aml_set_reg32_bits_op(P_HHI_VIID_CLK_CNTL, 1, 15, 1);  //soft reset
      aml_set_reg32_bits_op(P_HHI_VIID_CLK_CNTL, 0, 15, 1);  //release soft reset
    }
    else {
      aml_set_reg32_bits_op(P_HHI_VID_CLK_CNTL, 
                   (1<<0),  // Enable cntl_div1_en
                   0, 1    // cntl_div1_en
                   );
      aml_set_reg32_bits_op(P_HHI_VID_CLK_DIV, 
                   0,      // select clk_div1 
                   24, 4); // [23:20] encp_clk_sel 
      aml_set_reg32_bits_op(P_HHI_VID_CLK_CNTL, 1, 15, 1);  //soft reset
      aml_set_reg32_bits_op(P_HHI_VID_CLK_CNTL, 0, 15, 1);  //release soft reset
    }
    
    msleep(5);

//    aml_set_reg32_bits_op(P_HHI_VID_CLK_DIV, 
//                   1|(vclk_sel<<3),      // select clk_div2 
//                   24, 4); // [23:20] encp_clk_sel 

    if(upsample) {
        aml_set_reg32_bits_op(P_HHI_VIID_CLK_DIV, 
                     0|(vclk_sel<<3)|(vclk_sel<<7)|(vclk_sel<<11),      // select clk_div1 
                     20, 12); // [31:20] dac0_clk_sel, dac1_clk_sel, dac2_clk_sel
    }
    else {
        data32 = aml_read_reg32_op(P_HHI_VID_CLK_CNTL);  // Save HHI_VID_CLK_CNTL value
        aml_set_reg32_bits_op(P_HHI_VID_CLK_CNTL, 0, 0, 1);  // Disable cntl_div1_en first before select other clock, to avoid glitch
        aml_set_reg32_bits_op(P_HHI_VIID_CLK_DIV, 
                     0x111|(vclk_sel<<3)|(vclk_sel<<7)|(vclk_sel<<11),      // select clk_div2 
                     20, 12); // [31:20] dac0_clk_sel, dac1_clk_sel, dac2_clk_sel
        aml_write_reg32_op(P_HHI_VID_CLK_CNTL, data32);   // Recover HHI_VID_CLK_CNTL value
    }

    if(vclk_sel) {
      aml_set_reg32_bits_op(P_HHI_VIID_CLK_CNTL, 
                   (3<<0),  // Enable cntl_div1_en and cntl_div2_en
                   0, 2    // cntl_div1_en
                   );
      aml_set_reg32_bits_op(P_HHI_VIID_CLK_CNTL, 1, 15, 1);  //soft reset
      aml_set_reg32_bits_op(P_HHI_VIID_CLK_CNTL, 0, 15, 1);  //release soft reset
    }
    else {
      aml_set_reg32_bits_op(P_HHI_VID_CLK_CNTL, 
                   (3<<0),  // Enable cntl_div1_en and cntl_div2_en
                   0, 2    // cntl_div1_en
                   );
      aml_set_reg32_bits_op(P_HHI_VID_CLK_CNTL, 1, 15, 1);  //soft reset
      aml_set_reg32_bits_op(P_HHI_VID_CLK_CNTL, 0, 15, 1);  //release soft reset
    }
} /* vclk_set_encp_1920x1080 */

void set_tv_enc_1920x1080p (int viu1_sel, int viu2_sel, int enable)
{
    aml_write_reg32_op(P_ENCP_VIDEO_MODE,             0x0040 | (1<<14)); // Enable Hsync and equalization pulse switch in center; bit[14] cfg_de_v = 1
    aml_write_reg32_op(P_ENCP_VIDEO_MODE_ADV,         0x0008); // Sampling rate: 1
    aml_write_reg32_op(P_ENCP_VIDEO_YFP1_HTIME,       140);
    aml_write_reg32_op(P_ENCP_VIDEO_YFP2_HTIME,       2060);
    aml_write_reg32_op(P_ENCP_VIDEO_MAX_PXCNT,        2199);
    aml_write_reg32_op(P_ENCP_VIDEO_HSPULS_BEGIN,     2156);
    aml_write_reg32_op(P_ENCP_VIDEO_HSPULS_END,       44);
    aml_write_reg32_op(P_ENCP_VIDEO_HSPULS_SWITCH,    44);
    aml_write_reg32_op(P_ENCP_VIDEO_VSPULS_BEGIN,     140);
    aml_write_reg32_op(P_ENCP_VIDEO_VSPULS_END,       2059);
    aml_write_reg32_op(P_ENCP_VIDEO_VSPULS_BLINE,     0);
    aml_write_reg32_op(P_ENCP_VIDEO_VSPULS_ELINE,     4);
    aml_write_reg32_op(P_ENCP_VIDEO_HAVON_BEGIN,      148);
    aml_write_reg32_op(P_ENCP_VIDEO_HAVON_END,        2067);
    aml_write_reg32_op(P_ENCP_VIDEO_VAVON_BLINE,      41);
    aml_write_reg32_op(P_ENCP_VIDEO_VAVON_ELINE,      1120);
    aml_write_reg32_op(P_ENCP_VIDEO_HSO_BEGIN,	    44);
    aml_write_reg32_op(P_ENCP_VIDEO_HSO_END, 		    2156);
    aml_write_reg32_op(P_ENCP_VIDEO_VSO_BEGIN,	    2100);
    aml_write_reg32_op(P_ENCP_VIDEO_VSO_END, 		    2164);
    aml_write_reg32_op(P_ENCP_VIDEO_VSO_BLINE,        3);
    aml_write_reg32_op(P_ENCP_VIDEO_VSO_ELINE,        5);
    aml_write_reg32_op(P_ENCP_VIDEO_MAX_LNCNT,        1124);

	logo_object_t  *init_logo_obj=NULL;
#ifdef CONFIG_AM_LOGO
    init_logo_obj = get_current_logo_obj();
#endif

    if(NULL==init_logo_obj || !init_logo_obj->para.loaded){
        if (viu1_sel) { // 1=Connect to ENCP
            aml_set_reg32_bits_op(P_VPU_VIU_VENC_MUX_CTRL, 2, 0, 2); // [1:0] cntl_viu1_sel_venc: 0=ENCL, 1=ENCI, 2=ENCP, 3=ENCT.
        }
    }

    if (viu2_sel) { // 1=Connect to ENCP
        aml_set_reg32_bits_op(P_VPU_VIU_VENC_MUX_CTRL, 2, 2, 2); // [3:2] cntl_viu2_sel_venc: 0=ENCL, 1=ENCI, 2=ENCP, 3=ENCT.
    }
    if (enable) {
        aml_write_reg32_op(P_ENCP_VIDEO_EN, 1); // Enable Interlace video encoder
    }
}

void hdmi_test_function(void)
{
    unsigned int tmp_add_data;
    
    // Enable APB3 fail on error
    aml_set_reg32_bits_op(P_HDMI_CTRL_PORT, 1, 15, 1);

    // Disable these interrupts: [2] tx_edid_int_rise [1] tx_hpd_int_fall [0] tx_hpd_int_rise
    hdmi_wr_reg(OTHER_BASE_ADDR + HDMI_OTHER_INTR_MASKN, 0x0);

    // HPD glitch filter
    hdmi_wr_reg(TX_HDCP_HPD_FILTER_L, 0xa0);
    hdmi_wr_reg(TX_HDCP_HPD_FILTER_H, 0xa0);

    // Disable MEM power-down
    hdmi_wr_reg(TX_MEM_PD_REG0, 0);

    // Keep TX (except register I/F) in reset, while programming the registers:
    tmp_add_data  = 0;
    tmp_add_data |= 1   << 7; // [7] tx_pixel_rstn
    tmp_add_data |= 1   << 6; // [6] tx_tmds_rstn
    tmp_add_data |= 1   << 5; // [5] tx_audio_master_rstn
    tmp_add_data |= 1   << 4; // [4] tx_audio_sample_rstn
    tmp_add_data |= 1   << 3; // [3] tx_i2s_reset_rstn
    tmp_add_data |= 1   << 2; // [2] tx_dig_reset_n_ch2
    tmp_add_data |= 1   << 1; // [1] tx_dig_reset_n_ch1
    tmp_add_data |= 1   << 0; // [0] tx_dig_reset_n_ch0
    hdmi_wr_reg(TX_SYS5_TX_SOFT_RESET_1, tmp_add_data);

    tmp_add_data  = 0;
    tmp_add_data |= 0   << 4; // [4] HDMI_CH0_RST_IN
    tmp_add_data |= 1   << 2; // [0] tx_ddc_hdcp_reset_n
    tmp_add_data |= 1   << 1; // [0] tx_ddc_edid_reset_n
    tmp_add_data |= 1   << 0; // [0] tx_dig_reset_n_ch3
    hdmi_wr_reg(TX_SYS5_TX_SOFT_RESET_2, tmp_add_data);

    tmp_add_data  = 0;
    tmp_add_data |= 0   << 7; // [7] forced_sys_trigger
    tmp_add_data |= 0   << 6; // [6] sys_trigger_config
    tmp_add_data |= 0   << 5; // [5] mem_acc_seq_mode
    tmp_add_data |= 0   << 4; // [4] mem_acc_seq_start
    tmp_add_data |= 1   << 3; // [3] forced_mem_copy_done
    tmp_add_data |= 1   << 2; // [2] mem_copy_done_config
    tmp_add_data |= 0   << 1; // [1] edid_int_forced_clear
    tmp_add_data |= 0   << 0; // [0] edid_int_auto_clear
    hdmi_wr_reg(TX_HDCP_EDID_CONFIG, tmp_add_data);

    // Enable HDMI TX PHY clock
    tmp_add_data  = 0;
    tmp_add_data  |= (1   << 1);        // [1]      phy clock enable
    aml_write_reg32_op(P_HHI_HDMI_PHY_CNTL1, tmp_add_data);


    tmp_add_data  = 0;
    tmp_add_data |= 0               << 7; // [7]   Force DTV timing (Auto)
    tmp_add_data |= 0               << 6; // [6]   Force Video Scan, only if [7]is set
    tmp_add_data |= 0               << 5; // [5]   Force Video field, only if [7]is set
    tmp_add_data |= ((VIC==39)?0:1) << 4; // [4]   disable_vic39_correction
    tmp_add_data |= 0               << 0; // [3:0] Rsrv
    hdmi_wr_reg(TX_VIDEO_DTV_TIMING, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 0                       << 7; // [7]   forced_default_phase
    tmp_add_data |= 0                       << 2; // [6:2] Rsrv
    tmp_add_data |= TX_OUTPUT_COLOR_DEPTH   << 0; // [1:0] Color_depth:0=24-bit pixel; 1=30-bit pixel; 2=36-bit pixel; 3=48-bit pixel
    hdmi_wr_reg(TX_VIDEO_DTV_MODE, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 1                       << 7; // [7]   gc_pack_mode: 0=clear color_depth and pixel_phase when GC packet is transmitting AV_mute/clear info;
                                                  //                     1=do not clear.
    tmp_add_data |= 0                       << 0; // [6:0] forced_islands_per_period_active
    hdmi_wr_reg(TX_PACKET_ALLOC_ACTIVE_1, tmp_add_data);

    tmp_add_data  = 0;
    tmp_add_data |= 0   << 7; // [7]   Force packet timing
    tmp_add_data |= 0   << 6; // [6]   PACKET ALLOC MODE
    tmp_add_data |= 58  << 0; // [5:0] PACKET_START_LATENCY
    hdmi_wr_reg(TX_PACKET_CONTROL_1, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 0   << 6; // [7:6] audio_source_select[1:0]
    tmp_add_data |= 0   << 5; // [5]   external_packet_enable
    tmp_add_data |= 1   << 4; // [4]   internal_packet_enable
    tmp_add_data |= 0   << 2; // [3:2] afe_fifo_source_select_lane_1[1:0]
    tmp_add_data |= 0   << 0; // [1:0] afe_fifo_source_select_lane_0[1:0]
    hdmi_wr_reg(TX_CORE_DATA_CAPTURE_2, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 0   << 7; // [7]   monitor_lane_1
    tmp_add_data |= 0   << 4; // [6:4] monitor_select_lane_1[2:0]
    tmp_add_data |= 1   << 3; // [3]   monitor_lane_0
    tmp_add_data |= 7   << 0; // [2:0] monitor_select_lane_0[2:0]
    hdmi_wr_reg(TX_CORE_DATA_MONITOR_1, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 0   << 3; // [7:3] Rsrv
    tmp_add_data |= 2   << 0; // [2:0] monitor_select[2:0]
    hdmi_wr_reg(TX_CORE_DATA_MONITOR_2, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 1   << 7; // [7]   forced_hdmi
    tmp_add_data |= 1   << 6; // [6]   hdmi_config
    tmp_add_data |= 0   << 4; // [5:4] Rsrv
    tmp_add_data |= 0   << 3; // [3]   bit_swap.
    tmp_add_data |= 0   << 0; // [2:0] channel_swap[2:0]
    hdmi_wr_reg(TX_TMDS_MODE, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 0   << 7; // [7]   Rsrv
    tmp_add_data |= 0   << 6; // [6]   TX_CONNECT_SEL: 0=use lower channel data[29:0]; 1=use upper channel data[59:30]
    tmp_add_data |= 0   << 0; // [5:0] Rsrv
    hdmi_wr_reg(TX_SYS4_CONNECT_SEL_1, tmp_add_data);
    
    // Normally it makes sense to synch 3 channel output with clock channel's rising edge,
    // as HDMI's serializer is LSB out first, invert tmds_clk pattern from "1111100000" to
    // "0000011111" actually enable data synch with clock rising edge.
    tmp_add_data = 1 << 4; // Set tmds_clk pattern to be "0000011111" before being sent to AFE clock channel
    hdmi_wr_reg(TX_SYS4_CK_INV_VIDEO, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 0   << 7; // [7] Rsrv
    tmp_add_data |= 0   << 6; // [6] TX_AFE_FIFO channel 2 bypass=0
    tmp_add_data |= 0   << 5; // [5] TX_AFE_FIFO channel 1 bypass=0
    tmp_add_data |= 0   << 4; // [4] TX_AFE_FIFO channel 0 bypass=0
    tmp_add_data |= 1   << 3; // [3] output enable of clk channel (channel 3)
    tmp_add_data |= 1   << 2; // [2] TX_AFE_FIFO channel 2 enable
    tmp_add_data |= 1   << 1; // [1] TX_AFE_FIFO channel 1 enable
    tmp_add_data |= 1   << 0; // [0] TX_AFE_FIFO channel 0 enable
    hdmi_wr_reg(TX_SYS5_FIFO_CONFIG, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= TX_OUTPUT_COLOR_FORMAT  << 6; // [7:6] output_color_format: 0=RGB444; 1=YCbCr444; 2=Rsrv; 3=YCbCr422.
    tmp_add_data |= TX_INPUT_COLOR_FORMAT   << 4; // [5:4] input_color_format:  0=RGB444; 1=YCbCr444; 2=Rsrv; 3=YCbCr422.
    tmp_add_data |= TX_OUTPUT_COLOR_DEPTH   << 2; // [3:2] output_color_depth:  0=24-b; 1=30-b; 2=36-b; 3=48-b.
    tmp_add_data |= TX_INPUT_COLOR_DEPTH    << 0; // [1:0] input_color_depth:   0=24-b; 1=30-b; 2=36-b; 3=48-b.
    hdmi_wr_reg(TX_VIDEO_DTV_OPTION_L, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 0                       << 4; // [7:4] Rsrv
    tmp_add_data |= TX_OUTPUT_COLOR_RANGE   << 2; // [3:2] output_color_range:  0=16-235/240; 1=16-240; 2=1-254; 3=0-255.
    tmp_add_data |= TX_INPUT_COLOR_RANGE    << 0; // [1:0] input_color_range:   0=16-235/240; 1=16-240; 2=1-254; 3=0-255.
    hdmi_wr_reg(TX_VIDEO_DTV_OPTION_H, tmp_add_data);

    tmp_add_data  = 0;
    tmp_add_data |= 0   << 7; // [7] cp_desired
    tmp_add_data |= 0   << 6; // [6] ess_config
    tmp_add_data |= 0   << 5; // [5] set_avmute
    tmp_add_data |= 1   << 4; // [4] clear_avmute
    tmp_add_data |= 0   << 3; // [3] hdcp_1_1
    tmp_add_data |= 0   << 2; // [2] Vsync/Hsync forced_polarity_select
    tmp_add_data |= 0   << 1; // [1] forced_vsync_polarity
    tmp_add_data |= 0   << 0; // [0] forced_hsync_polarity
    hdmi_wr_reg(TX_HDCP_MODE, tmp_add_data);

    // AVI frame
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x00, 0x46);              // PB0: Checksum
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x01, 0x5e);              // PB1 (Note: the value should be meaningful but is not!)
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x02, 0xa8);              // PB2 (Note: the value should be meaningful but is not!)
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x03, 0x13);              // PB3 (Note: the value should be meaningful but is not!)
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x04, VIC);               // PB4: [7]    Rsrv
                                                                        //      [6:0]  VIC
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x05, PIXEL_REPEAT_HDMI); // PB5: [7:4]  Rsrv
                                                                        //      [3:0]  PixelRepeat
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x1C, 0x82);              // HB0: packet type=0x82
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x1D, 0x02);              // HB1: packet version =0x02
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x1E, 0x0D);              // HB2: payload bytes=13
    hdmi_wr_reg(TX_PKT_REG_AVI_INFO_BASE_ADDR+0x1F, 0xFF);              // Enable AVI packet generation

    
    tmp_add_data = 0xa; // time_divider[7:0] for DDC I2C bus clock
    hdmi_wr_reg(TX_HDCP_CONFIG3, tmp_add_data);
    
    tmp_add_data  = 0;
    tmp_add_data |= 1   << 7; // [7] cp_desired 
    tmp_add_data |= 1   << 6; // [6] ess_config 
    tmp_add_data |= 0   << 5; // [5] set_avmute 
    tmp_add_data |= 0   << 4; // [4] clear_avmute 
    tmp_add_data |= 1   << 3; // [3] hdcp_1_1 
    tmp_add_data |= 0   << 2; // [2] forced_polarity 
    tmp_add_data |= 0   << 1; // [1] forced_vsync_polarity 
    tmp_add_data |= 0   << 0; // [0] forced_hsync_polarity
    hdmi_wr_reg(TX_HDCP_MODE, tmp_add_data);
    
    // --------------------------------------------------------
    // Release TX out of reset
    // --------------------------------------------------------
    aml_write_reg32_op(P_HHI_HDMI_PLL_CNTL1, 0x00040000);         // turn off phy_clk
    aml_write_reg32_op(P_HHI_HDMI_PLL_CNTL1, 0x00040003);         // turn on phy_clk
    hdmi_wr_reg(TX_SYS5_TX_SOFT_RESET_2, 0x00); // Release reset on TX digital clock channel
    hdmi_wr_reg(TX_SYS5_TX_SOFT_RESET_1, 1<<6); // Release resets all other TX digital clock domain, except tmds_clk
    hdmi_wr_reg(TX_SYS5_TX_SOFT_RESET_1, 0x00); // Final release reset on tmds_clk domain
}

// Use this self-made function rather than %, because % appears to produce wrong
// value for divisor which are not 2's exponential.
unsigned long modulo(unsigned long a, unsigned long b)
{
    if (a >= b) {
        return(a-b);
    } else {
        return(a);
    }
}
        
signed int to_signed(unsigned int a)
{
    if (a <= 7) {
        return(a);
    } else {
        return(a-16);
    }
}

void hdmi_tx_miniboot(void) 
{
printk("%s[%d]\n", __func__, __LINE__);
    clocks_set_sys_defaults();    // set MPEG, audio and default video
    aml_set_reg32_bits_op(P_HHI_HDMI_CLK_CNTL, 0, 16, 4);  // Set [19:16] hdmi_tx_pixel_clk comes from clk_div1
    vclk_set_encp_1920x1080(0,          0,              0,          0);
printk("%s[%d]\n", __func__, __LINE__);

    aml_set_reg32_bits_op(P_PERIPHS_PIN_MUX_1, 0xf, 23, 4);  //HPD SCL CEC pinmux

    hdmi_test_function();
printk("%s[%d]\n", __func__, __LINE__);

    set_tv_enc_1920x1080p(  1,          0,          0);

printk("%s[%d]\n", __func__, __LINE__);
    set_hdmi_enc();

    printk("config HPLL\n");
printk("%s[%d]\n", __func__, __LINE__);
    aml_write_reg32_op(P_HHI_VID_PLL_CNTL2, 0x69c88000);
    aml_write_reg32_op(P_HHI_VID_PLL_CNTL3, 0xca563823);
    aml_write_reg32_op(P_HHI_VID_PLL_CNTL4, 0x00238100);
    aml_write_reg32_op(P_HHI_VID_PLL_CNTL5, 0x00012286);
    aml_write_reg32_op(P_HHI_VID_PLL_CNTL,  0x6001043d);
    aml_write_reg32_op(P_HHI_VID_PLL_CNTL,  0x4001043d);
    printk("waiting HPLL lock\n");
    while(!(aml_read_reg32_op(P_HHI_VID_PLL_CNTL) & (1 << 31))) {
        ;
    }
printk("%s[%d]\n", __func__, __LINE__);
    aml_write_reg32_op(P_HHI_VID_PLL_CNTL2, 0x69c8ce00);
    aml_write_reg32_op(P_HHI_HDMI_PHY_CNTL1, 2);
    aml_write_reg32_op(P_HHI_HDMI_PHY_CNTL0, 0x08c38d0b);

printk("%s[%d]\n", __func__, __LINE__);
    return;
}

