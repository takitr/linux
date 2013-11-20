/*******************************************************************
 *
 *  Copyright C 2012 by Amlogic, Inc. All Rights Reserved.
 *
 *  Description:
 *
 *  Author: Amlogic Software
 *  Created: 2012/3/13   19:46
 *
 *******************************************************************/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <mach/am_regs.h>
#include <mach/mipi_phy_reg.h>
#include <linux/amlogic/mipi/am_mipi_csi2.h>
#include <linux/amlogic/tvin/tvin_v4l2.h>
#include "csi.h"

void init_am_mipi_csi2_clock(void)
{
        WRITE_CBUS_REG( HHI_MIPI_PHY_CLK_CNTL,  ((3 << 9)  |   // select 400Mhz (fclk_div5)
                                (1 << 8)  |   // Enable gated clock
                                (1 << 0)) );  // Divide output by 2
        return;
}


static void init_am_mipi_csi2_host(csi_parm_t* info)
{
        WRITE_CSI_HST_REG(MIPI_CSI2_HOST_CSI2_RESETN,    0); // csi2 reset
        WRITE_CSI_HST_REG(MIPI_CSI2_HOST_CSI2_RESETN,    0xffffffff); // release csi2 reset
        WRITE_CSI_HST_REG(MIPI_CSI2_HOST_DPHY_RSTZ,    0xffffffff); // release DPHY reset
        WRITE_CSI_HST_REG(MIPI_CSI2_HOST_N_LANES, (info->lanes-1)&3);  //set lanes
        WRITE_CSI_HST_REG(MIPI_CSI2_HOST_PHY_SHUTDOWNZ,    0xffffffff); // enable power
        return;
}

static int init_am_mipi_csi2_adapter(csi_parm_t* info)
{
        unsigned data32;
        WRITE_CSI_ADPT_REG(CSI2_CLK_RESET, 1<<CSI2_CFG_SW_RESET); //reset first
        WRITE_CSI_ADPT_REG(CSI2_CLK_RESET, (0<<CSI2_CFG_SW_RESET)|(0<<CSI2_CFG_CLK_AUTO_GATE_OFF)); // Bring out of reset

        data32  = 0;
        data32 |= 0<< CSI2_CFG_CLR_WRRSP; 
        data32 |= 0x3f<< CSI2_CFG_A_BRST_NUM;
        data32 |= 3<<CSI2_CFG_A_ID;  // ?? why is 3
        data32 |= info->urgent<<CSI2_CFG_URGENT_EN;
        data32 |= 0<<CSI2_CFG_DDR_ADDR_LPBK;

        if(info->mode == AM_CSI2_VDIN){
                data32 |= 1<< CSI2_CFG_DDR_EN;  ///testtest 1
                data32 |= 1<<CSI2_CFG_BUFFER_PIC_SIZE;
                data32 |= 0<<CSI2_CFG_422TO444_MODE;///testtest 1
                data32 |= 0<<CSI2_CFG_INV_FIELD ;
                data32 |= 0<<CSI2_CFG_INTERLACE_EN; 
                data32 |= 1<<CSI2_CFG_FORCE_LINE_COUNT;
                data32 |= 1<<CSI2_CFG_FORCE_PIX_COUNT;
                data32 |= 1<<CSI2_CFG_COLOR_EXPAND;
                data32 |= 0<<CSI2_CFG_ALL_TO_MEM;
        }else{
                data32 |= 1<< CSI2_CFG_DDR_EN; 
                data32 |= 0<<CSI2_CFG_BUFFER_PIC_SIZE;
                data32 |= 0<<CSI2_CFG_422TO444_MODE;
                data32 |= 0<<CSI2_CFG_INV_FIELD ;
                data32 |= 0<<CSI2_CFG_INTERLACE_EN; 
                data32 |= 0<<CSI2_CFG_FORCE_LINE_COUNT;
                data32 |= 0<<CSI2_CFG_FORCE_PIX_COUNT;
                data32 |= 0<<CSI2_CFG_COLOR_EXPAND;
                data32 |= 1<<CSI2_CFG_ALL_TO_MEM;
        }

        data32 |= info->channel<<CSI2_CFG_VIRTUAL_CHANNEL_EN; //??how to set
        WRITE_CSI_ADPT_REG(CSI2_GEN_CTRL0, data32);

        if(info->mode == AM_CSI2_VDIN)
                WRITE_CSI_ADPT_REG(CSI2_FORCE_PIC_SIZE, (info->active_line << CSI2_CFG_LINE_COUNT) | (info->active_pixel<<CSI2_CFG_PIX_COUNT));

#if 0
        if(info->frame){
                WRITE_CSI_ADPT_REG(CSI2_DDR_START_ADDR, info->frame->ddr_address);
                WRITE_CSI_ADPT_REG(CSI2_DDR_END_ADDR, info->frame->ddr_address+info->frame_size);
        }else{
                DPRINT("info->frame=%p\n", info->frame);
        }
#endif

        if(info->mode == AM_CSI2_ALL_MEM)
                WRITE_CSI_ADPT_REG(CSI2_INTERRUPT_CTRL_STAT,    1<<CSI2_CFG_VS_FAIL_INTERRUPT);///testtest appear in vdin 2165

        WRITE_CSI_ADPT_REG(CSI2_CLK_RESET,     (0<<CSI2_CFG_SW_RESET) |
                        (0<<CSI2_CFG_CLK_AUTO_GATE_OFF)|
                        (1<<CSI2_CFG_CLK_ENABLE)|  // Enable clock
                        (1<<CSI2_CFG_CLK_ENABLE_DWC));  // Enable host clock
        return 0;
}

//mipi phy run by 200MHZ ---1 cycle = 1/200000000.  timing= (reg value+1)*cycle*1000000000 ns

static void init_am_mipi_phy(csi_parm_t* info)
{
        u32 data32 = 0x80000000;
        u32 temp_data;
        u32 cycle_time = 5;//5 ns
        u32 settle = (85 + 145 + (16*info->ui_val))/2;
        settle = settle/cycle_time;
        DPRINT("(85 + 145 + (16*%d))/2/5.4945 == settle=%d\n", info->ui_val, settle);
        DPRINT("%s, %d\n", __func__, __LINE__);
        DPRINT("settle=%d\n", settle);
        settle = 25;
        //mipi_dbg("[mipi_hw]:init_am_mipi_phy ---- mipi cycle:%d ns, hs settle:%d ns,\n",cycle_time,(settle*cycle_time));

        //if(info->clock_lane_mode==1){
        //    //use always on mode
        //}
        WRITE_CSI_PHY_REG(MIPI_PHY_CTRL, data32);   //soft reset bit
        WRITE_CSI_PHY_REG(MIPI_PHY_CTRL,   0);   //release soft reset bit   
        WRITE_CSI_PHY_REG(MIPI_PHY_CLK_LANE_CTRL ,0xd8);
        WRITE_CSI_PHY_REG(MIPI_PHY_TCLK_MISS ,0x8);  // clck miss = 50 ns --(x< 60 ns)
        WRITE_CSI_PHY_REG(MIPI_PHY_TCLK_SETTLE ,0x1c);  // clck settle = 160 ns --(95ns< x < 300 ns)
        WRITE_CSI_PHY_REG(MIPI_PHY_THS_EXIT ,0x1c);   // hs exit = 160 ns --(x>100ns)
        WRITE_CSI_PHY_REG(MIPI_PHY_THS_SKIP ,0x9);   // hs skip = 55 ns --(40ns<x<55ns+4*UI)
        WRITE_CSI_PHY_REG(MIPI_PHY_THS_SETTLE ,settle);   // hs settle = 160 ns --(85 ns + 6*UI<x<145 ns + 10*UI)
        WRITE_CSI_PHY_REG(MIPI_PHY_TINIT ,0x4e20);  // >100us
        WRITE_CSI_PHY_REG(MIPI_PHY_TMBIAS ,0x100);
        WRITE_CSI_PHY_REG(MIPI_PHY_TULPS_C ,0x1000);
        WRITE_CSI_PHY_REG(MIPI_PHY_TULPS_S ,0x100);
        WRITE_CSI_PHY_REG(MIPI_PHY_TLP_EN_W ,0x0c);
        WRITE_CSI_PHY_REG(MIPI_PHY_TLPOK ,0x100);
        WRITE_CSI_PHY_REG(MIPI_PHY_TWD_INIT ,0x400000);
        WRITE_CSI_PHY_REG(MIPI_PHY_TWD_HS ,0x400000);
        WRITE_CSI_PHY_REG(MIPI_PHY_DATA_LANE_CTRL , 0x0);
        WRITE_CSI_PHY_REG(MIPI_PHY_DATA_LANE_CTRL1 , 0x3 | (0x1f << 2 ) | (0x3 << 7));     // enable data lanes pipe line and hs sync bit err.
#ifdef CONFIG_ARCH_MESON6
        WRITE_CSI_PHY_REG(MIPI_PHY_AN_CTRL0,0xa3a9); //MIPI_COMMON<15:0>=<1010,0011,1010,1001>
        WRITE_CSI_PHY_REG(MIPI_PHY_AN_CTRL1,0xcf25); //MIPI_CHCTL1<15:0>=<1100,1111,0010,0101> 
        WRITE_CSI_PHY_REG(MIPI_PHY_AN_CTRL2,0x0667); //MIPI_CHCTL2<15:0>=<0000,0110,0110,0111> 
#else
        DPRINT("HHI_GCLK_MPEG1=%x, csi2_dig_clkin=%d\n", READ_CBUS_REG(HHI_GCLK_MPEG1), (READ_CBUS_REG(HHI_GCLK_MPEG1) >> 18)&0x1);
        WRITE_CBUS_REG(HHI_CSI_PHY_CNTL0, 0xfdc1 << 16 | 0xfd01); 
        WRITE_CBUS_REG(HHI_CSI_PHY_CNTL1, 0x3f << 16 | 0xffff); 
        temp_data = READ_CBUS_REG(HHI_CSI_PHY_CNTL2);//
        //printk("temp_data=%x, == 0x8000 0fc0", temp_data);
        //printk(" MIPI_CSI2_HOST_BASE_ADDR=%x\n", MIPI_CSI2_HOST_BASE_ADDR);
#endif
        //data32 =((~(info->channel))&0xf)|(0 << 4); //enable lanes digital clock
        //data32 |= ((info->channel)<<5); //mipi_chpu  to analog
        WRITE_CSI_PHY_REG(MIPI_PHY_CTRL,  info->clk_channel<<21);// data32);
        return;
}



static void reset_am_mipi_csi2_host(void)
{
        WRITE_CSI_HST_REG(MIPI_CSI2_HOST_PHY_SHUTDOWNZ,    0); // enable power
        WRITE_CSI_HST_REG(MIPI_CSI2_HOST_DPHY_RSTZ,    0); // release DPHY reset
        WRITE_CSI_HST_REG(MIPI_CSI2_HOST_CSI2_RESETN,    0); // csi2 reset
        return;
}

static void reset_am_mipi_csi2_adapter(void)
{
        unsigned data32 = READ_CSI_ADPT_REG(CSI2_GEN_CTRL0);
        data32 &=((~0xf)<<CSI2_CFG_VIRTUAL_CHANNEL_EN);
        WRITE_CSI_ADPT_REG(CSI2_GEN_CTRL0,data32);  // disable virtual channel
        WRITE_CSI_ADPT_REG(CSI2_INTERRUPT_CTRL_STAT,   0x7<<CSI2_CFG_FIELD_DONE_INTERRUPT_CLR); // clear status,disable interrupt
        WRITE_CSI_ADPT_REG(CSI2_CLK_RESET, (1<<CSI2_CFG_SW_RESET) |
                        (1<<CSI2_CFG_CLK_AUTO_GATE_OFF)); // disable auto gate and clock

        //WRITE_CBUS_REG_BITS( HHI_MIPI_PHY_CLK_CNTL,  0, 8, 1);    // disable gated clock
        WRITE_CBUS_REG( HHI_MIPI_PHY_CLK_CNTL,  0);    // disable gated clock
        return;
}

static void reset_am_mipi_phy(void)
{
        u32 data32;
        data32 =0x1f; //disable lanes digital clock
        data32 |= 0x1<<31; //soft reset bit
        WRITE_CSI_PHY_REG(MIPI_PHY_CTRL,   data32);   
#if 1
        WRITE_CBUS_REG(HHI_CSI_PHY_CNTL0, 0xfcc1 << 16 | 0xf780); 
        WRITE_CBUS_REG(HHI_CSI_PHY_CNTL1, 0x0 << 16 | 0xffff); 
        WRITE_CBUS_REG(HHI_CSI_PHY_CNTL2, 0x0); 
#endif
        return;
}

void am_mipi_csi2_init(csi_parm_t* info)
{
        init_am_mipi_phy(info);
        init_am_mipi_csi2_host(info);
        init_am_mipi_csi2_adapter(info);
        return;
}

void am_mipi_csi2_uninit(void)
{
        reset_am_mipi_phy();
        reset_am_mipi_csi2_host();
        reset_am_mipi_csi2_adapter();
        return;
}
