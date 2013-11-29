/*
 * SDHC definitions
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __AML_SDHC_H__
#define __AML_SDHC_H__

#include <linux/types.h>
#include <linux/device.h>
#include <linux/mmc/host.h>
#include <linux/earlysuspend.h>

//LSB -> MSB, structrue for SD Card Status
typedef struct _SD_Card_Status
{
	unsigned Reserved3: 2;
	unsigned Reserved4: 1;
	unsigned AKE_SEQ_ERROR: 1;                  //Error in the sequence of authentication process.
	unsigned Reserved5: 1;
	unsigned APP_CMD: 1;                        //The card will expect ACMD, or indication that the command has been interpreted as ACMD.
	unsigned NotUsed: 2;
	
	unsigned READY_FOR_DATA: 1;                 //Corresponds to buffer empty signalling on the bus.
	unsigned CURRENT_STATE: 4;                  //The state of the card when receiving the command. 
	unsigned ERASE_RESET: 1;                    //An erase sequence was cleared beforem executing because an out of erase sequence command was received.
	unsigned CARD_ECC_DISABLED: 1;              //The command has been executed without using the internal ECC.
	unsigned WP_ERASE_SKIP: 1;                  //Only partial address space was erased due to existing write protected blocks.
	
	unsigned CID_CSD_OVERWRITE: 1;              //Can be either one of the following errors:
	unsigned Reserved1: 1;
	unsigned Reserved2: 1;
	unsigned ERROR: 1;                          //A general or an unknown error occurred during the operation.
	unsigned CC_ERROR: 1;                       //Internal card controller error
	unsigned CARD_ECC_FAILED: 1;                //Card internal ECC was applied but failed to correct the data.
	unsigned ILLEGAL_COMMAND: 1;                //Command not legal for the card state
	unsigned COM_CRC_ERROR: 1;                  //The CRC check of the previous command failed.
	
  unsigned LOCK_UNLOCK_FAILED: 1;             //Set when a sequence or password error has been detected in lock/ unlock card command or if there was an attempt to access a locked card
	unsigned CARD_IS_LOCKED: 1;                 //When set, signals that the card is locked by the host
	unsigned WP_VIOLATION: 1;                   //Attempt to program a write-protected block.
	unsigned ERASE_PARAM: 1;                    //An invalid selection of write-blocks for erase occurred.
	unsigned ERASE_SEQ_ERROR: 1;                //An error in the sequence of erase commands occurred.
	unsigned BLOCK_LEN_ERROR: 1;                //The transferred block length is not allowed for this card, or the number of transferred bytes does not match the block length.
	unsigned ADDRESS_ERROR: 1;                  //A misaligned address that did not match the block length was used in the command.
	unsigned OUT_OF_RANGE: 1;                   //The command??s argument was out of the allowed range for this card.
	
} SD_Card_Status_t;

//structure for response
typedef struct _SD_Response_R1
{
	SD_Card_Status_t card_status;           //card status
} SD_Response_R1_t;

typedef enum _SD_Error_Status_t { 
	SD_NO_ERROR                 = 0,
	SD_ERROR_OUT_OF_RANGE,                  //Bit 31
	SD_ERROR_ADDRESS,                       //Bit 30 
	SD_ERROR_BLOCK_LEN,                     //Bit 29
	SD_ERROR_ERASE_SEQ,                     //Bit 28
	SD_ERROR_ERASE_PARAM,                   //Bit 27
	SD_ERROR_WP_VIOLATION,                  //Bit 26
	SD_ERROR_CARD_IS_LOCKED,                //Bit 25
	SD_ERROR_LOCK_UNLOCK_FAILED,            //Bit 24
	SD_ERROR_COM_CRC,                       //Bit 23
	SD_ERROR_ILLEGAL_COMMAND,               //Bit 22
	SD_ERROR_CARD_ECC_FAILED,               //Bit 21
	SD_ERROR_CC,                            //Bit 20
	SD_ERROR_GENERAL,                       //Bit 19
	SD_ERROR_Reserved1,                     //Bit 18
	SD_ERROR_Reserved2,                     //Bit 17
	SD_ERROR_CID_CSD_OVERWRITE,             //Bit 16
	SD_ERROR_AKE_SEQ,                       //Bit 03
	SD_ERROR_STATE_MISMATCH,
	SD_ERROR_HEADER_MISMATCH,
	SD_ERROR_DATA_CRC,
	SD_ERROR_TIMEOUT,  
	SD_ERROR_DRIVER_FAILURE,
	SD_ERROR_WRITE_PROTECTED,
	SD_ERROR_NO_MEMORY,
	SD_ERROR_SWITCH_FUNCTION_COMUNICATION,
	SD_ERROR_NO_FUNCTION_SWITCH,
	SD_ERROR_NO_CARD_INS
} SD_Error_Status_t;


// 0 : SDHC_ARGU
typedef struct SDXC_Arg_Reg {
	unsigned long command;
} SDXC_Arg_Reg_t;

// 1 : SDHC_SEND
typedef struct SDXC_Send_Reg {
	unsigned command_index : 6;
	unsigned command_has_resp : 1;
	unsigned command_has_data : 1;
	unsigned response_length : 1;
	unsigned response_no_crc : 1;
	unsigned data_direction : 1;
	unsigned data_stop : 1;
	unsigned total_pack : 20;
} SDXC_Send_Reg_t;

// 2 : SDHC_CTRL
typedef struct SDXC_Ctrl_Reg {
	unsigned dat_width : 2;	//dat_type
	unsigned ddr_mode : 1;
	unsigned reserved1 : 1;
	unsigned pack_len : 9;		//0:512, 1:1, ..., 511:511
	unsigned rx_timeout : 7;
	unsigned rc_period : 4;
	unsigned endian : 2;		
	unsigned reserved2 : 6;		
} SDXC_Ctrl_Reg_t;

// 3 : SDHC_STAT
typedef struct SDXC_Status_Reg {
	unsigned busy : 1;
	unsigned dat_3_0 : 4;
	unsigned cmd_state : 1;
	unsigned reserved1 : 1;	
	unsigned reserved2 : 1;	
	unsigned rx_count : 6;
	unsigned tx_count : 6;
	unsigned dat_7_4 : 4;
	
	unsigned reserved3 : 8;	
} SDXC_Status_Reg_t;

// 4 : SDHC_CLKC
typedef struct SDXC_Clk_Reg {
	unsigned clk_div : 16;
	unsigned clk_in_sel : 3;
	unsigned clk_en : 1;
	unsigned rx_clk_phase_sel : 2;
	unsigned rx_clk_feedback_en : 1;
	unsigned clk_ctl_en : 1;	//???  write 0,  change, write 1
	unsigned clk_jic_control : 1;	//???
	unsigned reserved1 : 7;
} SDXC_Clk_Reg_t;

// 5 : SDHC_ADDR
typedef struct SDXC_Addr_Reg {
	unsigned long dma_addr;
} SDXC_Addr_Reg_t;

// 6 : SDHC_PDMA
typedef struct SDXC_PDMA_Reg {
	unsigned dma_mode : 1;		//0:PIO, 1:DMA
	unsigned pio_rd_resp : 3;	//0:[39:8],  1:1st long,  2:2nd long, ...,  6 or 7:cmd arg
	unsigned dma_urgent : 1;
	unsigned wr_burst : 5;
	unsigned rd_burst : 5;
	unsigned rx_threshold : 6;
	unsigned tx_threshold : 6;
	unsigned rx_manual_flush : 1;	//???
	unsigned reserved1 : 4;
} SDXC_PDMA_Reg_t;

// 7 : SDHC_MISC
typedef struct SDXC_Misc_Reg {
	unsigned cmd_line_delay : 2;	//for tuning
	unsigned dat_line_delay : 2;	//for tuning
	unsigned rx_full_threshold : 6;		//default : 30
	unsigned tx_empty_threshold : 6;	//default : 0
	unsigned burst_num : 6;
	unsigned thread_id : 6;
	unsigned manual_stop : 1;
	unsigned reserved1 : 3;
} SDXC_Misc_Reg_t;

// 8 : SDHC_DATA
typedef struct SDXC_Data_Reg {
	unsigned long value;
} SDXC_Data_Reg_t;

// 9 : SDHC_ICTL
typedef struct SDXC_Ictl_Reg {	//interrupt control
	unsigned res_ok_int_en 			: 1;	//@@
	unsigned res_timeout_int_en 	: 1;
	unsigned res_crc_err_int_en 	: 1;
	unsigned dat0_ready_int_en 		: 1;

	unsigned pack_complete_int_en 	: 1;
	unsigned pack_timeout_int_en 	: 1;
	unsigned pack_crc_err_int_en 	: 1;
	unsigned data_complete_int_en 	: 1;	//@@

	unsigned rx_fifo_int_en 	: 1;
	unsigned tx_fifo_int_en 	: 1;
	unsigned sdio_dat1_int_en 	: 1;		//@@
	unsigned dma_done_int_en 	: 1;

	unsigned rx_fifo_full_int_en 			: 1;
	unsigned tx_fifo_empty_int_en 			: 1;
	unsigned additional_sdio_dat1_int_en 	: 1;		//???

	unsigned reserved1 : 1;	
	unsigned sdio_dat1_mask_delay : 2;	
	unsigned reserved2 : 14;
} SDXC_Ictl_Reg_t;

// A : SDHC_ISTA
typedef struct SDXC_Ista_Reg {	//interrupt status
	unsigned res_ok_int 		: 1;		//@@
	unsigned res_timeout_int 	: 1;
	unsigned res_crc_err_int 	: 1;
	unsigned dat0_ready_int 	: 1;

	unsigned pack_complete_int 	: 1;
	unsigned pack_timeout_int 	: 1;
	unsigned pack_crc_err_int 	: 1;
	unsigned data_complete_int 	: 1;		//@@

	unsigned rx_fifo_int 		: 1;
	unsigned tx_fifo_int 		: 1;
	unsigned sdio_dat1_int 		: 1;		//@@
	unsigned dma_done_int 		: 1;

	unsigned rx_fifo_full_int 	: 1;
	unsigned tx_fifo_empty_int 	: 1;
	unsigned additional_sdio_dat1_int 	: 1;		//???

	unsigned reserved1 : 17;
} SDXC_Ista_Reg_t;

// B : SDHC_SRST
typedef struct SDXC_Srst_Reg {	//soft reset
	unsigned main_ctrl_srst : 1;
	unsigned rx_fifo_srst : 1;
	unsigned tx_fifo_srst : 1;
	unsigned rx_dphy_srst : 1;	//manual clear
	unsigned tx_dphy_srst : 1;	//manual clear
	unsigned dma_srst : 1;
	unsigned reserved1 : 26;
} SDXC_Srst_Reg_t;

enum aml_mmc_waitfor {
	XFER_INIT,
	XFER_START,				/* 1 */
	XFER_IRQ_OCCUR,			/* 2 */
	XFER_IRQ_FIFO_ERR,		/* 3 */
	XFER_IRQ_CRC_ERR,		/* 4 */
	XFER_IRQ_TIMEOUT_ERR,	/* 5 */
	XFER_IRQ_TASKLET_CMD,	/* 6 */
	XFER_IRQ_TASKLET_DATA,	/* 7 */
	XFER_IRQ_TASKLET_BUSY,	/* 8 */
	XFER_IRQ_UNKNOWN_IRQ,	/* 9 */
	XFER_TIMER_TIMEOUT,		/* 10 */
	XFER_TASKLET_CMD,		/* 11 */
	XFER_TASKLET_DATA,		/* 12 */
	XFER_TASKLET_BUSY,		/* 13 */
	XFER_TIMEDOUT,			/* 14 */
	XFER_FINISHED,			/* 15 */
	XFER_AFTER_START,		/* 16 */
};

struct amlsd_host;
struct amlsd_platform {
	struct amlsd_host* host;
	struct mmc_host *mmc;
	struct list_head sibling;
	unsigned long ocr_avail;
	unsigned int port; 
#define     PORT_SDIO_A     0
#define     PORT_SDIO_B     1
#define     PORT_SDIO_C     2
#define     PORT_SDHC_A     3
#define     PORT_SDHC_B     4
#define     PORT_SDHC_C     5

	unsigned int width;
	unsigned int caps;
	unsigned int caps2;

	unsigned int f_min;
	unsigned int f_max;
	unsigned int f_max_w;
	unsigned int clkc;
	unsigned int clkc_w;
	unsigned int ctrl;
	unsigned int clock;
	unsigned int eject;
	unsigned int low_burst;
	unsigned int irq_in;
	unsigned int irq_in_edge;
	unsigned int irq_out;
	unsigned int irq_out_edge;
	unsigned int gpio_cd;
	unsigned int gpio_power;
	unsigned int power_level;
	char pinname[32];
	unsigned int gpio_ro;
    unsigned int gpio_dat3;
    unsigned int jtag_pin;
    int is_sduart;
    // struct pinctrl *uart_ao_pinctrl;
	void (*irq_init)(struct amlsd_platform* pdata);

	unsigned int max_blk_count;
	unsigned int max_blk_size;
	unsigned int max_req_size;
	unsigned int max_seg_size;

	/*for inand partition: struct mtd_partition, easy porting from nand*/
	struct mtd_partition *parts;
	unsigned int nr_parts;

	struct resource* resource;
	void (*xfer_pre)(struct amlsd_platform* pdata);
	void (*xfer_post)(struct amlsd_platform* pdata);

	int (*port_init)(struct amlsd_platform* pdata);
	int (*cd)(struct amlsd_platform* pdata);
	int (*ro)(struct amlsd_platform* pdata);
	void (*pwr_pre)(struct amlsd_platform* pdata);
	void (*pwr_on)(struct amlsd_platform* pdata);
	void (*pwr_off)(struct amlsd_platform* pdata);

};

struct amlsd_host {
	/* back-link to device */
	struct device *dev;
	struct list_head sibling;
    struct platform_device *pdev;
	struct amlsd_platform * pdata;
	struct mmc_host		*mmc;
	struct mmc_request	*request;
	struct resource		*mem;
	void __iomem		*base;
	int			dma;
	char*		bn_buf;
	dma_addr_t		bn_dma_buf;
	unsigned int f_max;
	unsigned int f_max_w;
	unsigned int f_min;
	unsigned int eject;
	struct tasklet_struct cmd_tlet;
	struct tasklet_struct data_tlet;
	struct tasklet_struct busy_tlet;
	struct tasklet_struct to_tlet;
    // struct timer_list timeout_tlist;
	struct delayed_work	timeout;
	struct early_suspend amlsd_early_suspend;

	unsigned int send;
	unsigned int ctrl;
	unsigned int clkc;
	unsigned int clkc_w;
	unsigned int pdma;
	unsigned int pdma_s;
	unsigned int pdma_low;
	unsigned int misc;
	unsigned int ictl;
	unsigned int ista;
	unsigned int dma_addr;

	unsigned long		clk_rate;

	struct  mmc_request	*mrq;
	struct  mmc_request	*mrq2;
	spinlock_t	mrq_lock;
	int			cmd_is_stop;
	enum aml_mmc_waitfor	xfer_step;

	int			bus_width;
	int     port;
	int     locked;
	char		*status;
	unsigned int		ccnt, dcnt;

#ifdef CONFIG_DEBUG_FS
	struct dentry		*debug_root;
	struct dentry		*debug_state;
	struct dentry		*debug_regs;
#endif

#ifdef CONFIG_CPU_FREQ
	struct notifier_block	freq_transition;
#endif	

    u32			opcode; // add by gch for debug
	u32			arg; // add by gch for debug
    u32         time_req_sta; // request start time

    struct pinctrl *pinctrl;
    int storage_flag; // used for judging if there is a tsd/emmc
};

/*-sdio-*/

#define SDIO_ARGU       (0x0)
#define SDIO_SEND       (0x4)
#define SDIO_CONF       (0x8)
#define SDIO_IRQS       (0xc)
#define SDIO_IRQC       (0x10)
#define SDIO_MULT       (0x14)
#define SDIO_ADDR       (0x18)
#define SDIO_EXT        (0x1c)

#define CLK_DIV         (0x1f4)

struct cmd_send{
    u32 cmd_command:8; /*[7:0] Command Index*/
    u32 cmd_response_bits:8; /*[15:8]
        * 00 means no response
        * others: Response bit number(cmd bits+response bits+crc bits-1)*/
    u32 response_do_not_have_crc7:1; /*[16]
        * 0:Response need check CRC7, 1: dont need check*/
    u32 response_have_data:1; /*[17]
        * 0:Receiving Response without data, 1:Receiving response with data*/
    u32 response_crc7_from_8:1; /*[18]
        * 0:Normal CRC7, Calculating CRC7 will be from bit0 of all response bits,
        * 1:Calculating CRC7 will be from bit8 of all response bits*/
    u32 check_busy_on_dat0:1; /*[19]
        * used for R1b response 0: dont check busy on dat0, 1:need check*/
    u32 cmd_send_data:1; /*[20]
        * 0:This command is not for transmitting data, 
        * 1:This command is for transmitting data*/
    u32 use_int_window:1; /*[21]
        * 0:SDIO DAT1 interrupt window disabled, 1:Enabled*/
    u32 reserved:2;/*[23:22]*/
    u32 repeat_package_times:8; /*[31:24] Total packages to be sent*/
};

struct sdio_config{
    u32 cmd_clk_divide:10; /*[9:0] Clock rate setting, 
        * Frequency of SD equals to Fsystem/((cmd_clk_divide+1)*2)*/
    u32 cmd_disable_crc:1; /*[10]
        * 0:CRC employed, 1:dont send CRC during command being sent*/
    u32 cmd_out_at_posedge:1; /*[11]
        * Command out at negedge normally, 1:at posedge*/
    u32 cmd_argument_bits:6; /*[17:12] before CRC added, normally 39*/
    u32 do_not_delay_data:1; /*[18]
        *0:Delay one clock normally, 1:dont delay*/
    u32 data_latch_at_negedge:1; /*[19]
        * 0:Data caught at posedge normally, 1:negedge*/
    u32 bus_width:1; /*[20] 0:1bit, 1:4bit*/
    u32 m_endian:2; /*[22:21]
        * Change ENDIAN(bytes order) from DMA data (e.g. dma_din[31:0]).
        * (00: ENDIAN no change, data output equals to original dma_din[31:0];
        * 01: data output equals to {dma_din[23:16],dma_din[31:24],
        * dma_din[7:0],dma_din[15:8]};10: data output equals to 
        * {dma_din[15:0],dma_din[31:16]};11: data output equals to 
        * {dma_din[7:0],dma_din[15:8],dma_din[23:16],dma_din[31:24]})*/
    u32 sdio_write_nwr:6; /*[28:23]
        * Number of clock cycles waiting before writing data*/
    u32 sdio_write_crc_ok_status:3; /*[31:29] if CRC status 
        * equals this register, sdio write can be consider as correct*/
};

struct sdio_status_irq{
    u32 sdio_status:4; /*[3:0] Read Only
        * SDIO State Machine Current State, just for debug*/
    u32 sdio_cmd_busy:1; /*[4] Read Only
        * SDIO Command Busy, 1:Busy State*/
    u32 sdio_response_crc7_ok:1; /*[5] Read Only
        * SDIO Response CRC7 status, 1:OK*/
    u32 sdio_data_read_crc16_ok:1; /*[6] Read Only
        * SDIO Data Read CRC16 status, 1:OK*/
    u32 sdio_data_write_crc16_ok:1; /*[7] Read Only
        * SDIO Data Write CRC16 status, 1:OK*/
    u32 sdio_if_int:1; /*[8] write 1 clear this int bit
        * SDIO DAT1 Interrupt Status*/
    u32 sdio_cmd_int:1; /*[9] write 1 clear this int bit
        * Command Done Interrupt Status*/
    u32 sdio_soft_int:1; /*[10] write 1 clear this int bit
        * Soft Interrupt Status*/
    u32 sdio_set_soft_int:1; /*[11] write 1 to this bit 
        * will set Soft Interrupt, read out is m_req_sdio, just for debug*/
    u32 sdio_status_info:4; /*[15:12] 
        * used for change information between ARC and Amrisc */
    u32 sdio_timing_out_int:1; /*[16] write 1 clear this int bit
        * Timeout Counter Interrupt Status*/
    u32 amrisc_timing_out_int_en:1; /*[17]
        * Timeout Counter Interrupt Enable for AMRISC*/
    u32 arc_timing_out_int_en:1; /*[18] 
        * Timeout Counter Interrupt Enable for ARC/ARM*/
    u32 sdio_timing_out_count:13; /*[31:19]
        * Timeout Counter Preload Setting and Present Status*/
};

struct sdio_irq_config{
    u32 amrisc_if_int_en:1; /*[0]
        * 1:SDIO DAT1 Interrupt Enable for AMRISC*/
    u32 amrisc_cmd_int_en:1; /*[1]
        * 1:Command Done Interrupt Enable for AMRISC*/
    u32 amrisc_soft_int_en:1; /*[2] 
        * 1:Soft Interrupt Enable for AMRISC*/
    u32 arc_if_int_en:1; /*[3]
        * 1:SDIO DAT1 Interrupt Enable for ARM/ARC*/
    u32 arc_cmd_int_en:1; /*[4]
        * 1:Command Done Interrupt Enable for ARM/ARC*/
    u32 arc_soft_int_en:1; /*[5]
        * 1:Soft Interrupt Enable for ARM/ARC*/
    u32 sdio_if_int_config:2; /*[7:6]
        * 00:sdio_if_interrupt window will reset after data Tx/Rx or command 
        * done, others: only after command done*/
    u32 sdio_force_data:6; /*[13:8]
        * Write operation: Data forced by software
        * Read operation: {CLK,CMD,DAT[3:0]}*/
    u32 sdio_force_enable:1; /*[14] Software Force Enable
        * This is the software force mode, Software can directly 
        * write to sdio 6 ports (cmd, clk, dat0..3) if force_output_en 
        * is enabled. and hardware outputs will be bypassed.*/
    u32 soft_reset:1; /*[15] 
        * Write 1 Soft Reset, Don't need to clear it*/
    u32 sdio_force_output_en:6; /*[21:16]
        * Force Data Output Enable,{CLK,CMD,DAT[3:0]}*/
    u32 disable_mem_halt:2; /*[23:22] write and read
        * 23:Disable write memory halt, 22:Disable read memory halt*/
    u32 sdio_force_data_read:6; /*[29:24] Read Only
        * Data read out which have been forced by software*/
    u32 force_halt:1; /*[30] 1:Force halt SDIO by software
        * Halt in this sdio host controller means stop to transmit or 
        * receive data from sd card. and then sd card clock will be shutdown.
        * Software can force to halt anytime, and hardware will automatically 
        * halt the sdio when reading fifo is full or writing fifo is empty*/
    u32 halt_hole:1; /*[31]
        * 0: SDIO halt for 8bit mode, 1:SDIO halt for 16bit mode*/
};

struct sdio_mult_config{
    u32 sdio_port_sel:2; /*[1:0] 0:sdio_a, 1:sdio_b, 2:sdio_c*/
    u32 ms_enable:1; /*[2] 1:Memory Stick Enable*/
    u32 ms_sclk_always:1; /*[3] 1: Always send ms_sclk*/
    u32 stream_enable:1; /*[4] 1:Stream Enable*/
    u32 stream_8_bits_mode:1; /*[5] Stream 8bits mode*/
    u32 data_catch_level:2; /*[7:6] Level of data catch*/
    u32 write_read_out_index:1; /*[8] Write response index Enable
        * [31:16], [11:10], [7:0] is set only when  bit8 of this register is not set. 
        * And other bits are set only when bit8 of this register is also set.*/
    u32 data_catch_readout_en:1; /*[9] Data catch readout Enable*/
    u32 sdio_0_data_on_1:1; /*[10] 1:dat0 is on dat1*/
    u32 sdio_1_data_swap01:1; /*[11] 1:dat1 and dat0 swapped*/
    u32 response_read_index:4; /*[15:12] Index of internal read response*/
    u32 data_catch_finish_point:12; /*[27:16] If internal data 
        * catch counter equals this register, it indicates data catching is finished*/
    u32 reserved:4; /*[31:28]*/
};

struct sdio_extension{
    u32 cmd_argument_ext:16; /*[15:0] for future use*/
    u32 data_rw_number:14; /*[29:16] 
        * Data Read/Write Number in one packet, include CRC16 if has CRC16*/
    u32 data_rw_do_not_have_crc16:1; /*[30]
        * 0:data Read/Write has crc16, 1:without crc16*/
    u32 crc_status_4line:1; /*[31] 1:4Lines check CRC Status*/
};

struct sdio_reg{
    u32 argument; /*2308*/
    struct cmd_send send; /*2309*/
    struct sdio_config config; /*230a*/
    struct sdio_status_irq status; /*230b*/
    struct sdio_irq_config irqc; /*230c*/
    struct sdio_mult_config mult; /*230d*/
    u32 m_addr; /*230e*/
    struct sdio_extension ext;/*230f*/
};

/*-sdhc-*/

#define SDHC_ARGU				(0x00)
#define SDHC_SEND				(0x04)
#define SDHC_CTRL				(0x08)
#define SDHC_STAT				(0x0C)
#define SDHC_CLKC				(0x10)
#define SDHC_ADDR				(0x14)
#define SDHC_PDMA				(0x18)
#define SDHC_MISC				(0x1C)
#define SDHC_DATA				(0x20)
#define SDHC_ICTL				(0x24)
#define SDHC_ISTA				(0x28)
#define SDHC_SRST				(0x2C)

struct sdhc_send{
	u32 cmd_index:6; /*[5:0] command index*/
	u32 cmd_has_resp:1; /*[6] 0:no resp 1:has resp*/
	u32 cmd_has_data:1; /*[7] 0:no data 1:has data*/
	u32 resp_len:1; /*[8] 0:48bit 1:136bit*/
	u32 resp_no_crc:1; /*[9] 0:check crc7 1:don't check crc7*/
	u32 data_dir:1; /*[10] 0:data rx, 1:data tx*/
	u32 data_stop:1; /*[11] 0:rx or tx, 1:data stop,ATTN:will give rx a softreset*/
	u32 total_pack:16; /*[31:12] total package number for writing or reading*/
};

struct sdhc_ctrl{
	u32 dat_type:2; /*[1:0] 0:1bit, 1:4bits, 2:8bits, 3:reserved*/
	u32 ddr_mode:1; /*[2] 0:SDR mode, 1:Don't set it*/
	u32 reserved:1; /*[3] reserved*/
	u32 pack_len:9; /*[12:4] 0:512Bytes, 1:1, 2:2, ..., 511:511Bytes,
                    ATTN:setting limit: must be a multiple of 8bytes*/
	u32 rx_timeout:7; /*[19:13] cmd or wcrc Receiving Timeout, default 64*/
	u32 rx_period:4; /*[23:20]Period between response/cmd and next cmd, default 8*/
	u32 rx_endian:2; /*[25:24] Rx Endian Control*/
    u32 reserved1:6; /*[31:26] reserved*/
};

struct sdhc_stat{
	u32 cmd_busy:1; /*[0] 0:Ready for command, 1:busy*/
	u32 dat3_0:4; /*[4:1] DAT[3:0]*/
	u32 cmd:1; /*[5] CMD*/
    u32 reserved:2; /*[7:6] reserved*/
	u32 rxfifo_cnt:6; /*[13:8] RxFIFO count*/
	u32 txfifo_cnt:6; /*[19:14] TxFIFO count*/
	u32 dat7_4:4; /*[23:20] DAT[7:4]*/
	u32 reserved1:8; /*[31:24] Reserved*/
};

/*
* to avoid glitch issue,
* 1. clk_switch_on better be set after cfg_en be set to 1'b1
* 2. clk_switch_off shall be set before cfg_en be set to 1'b0
* 3. rx_clk/sd_clk phase diff please see SD_REGE_CLK2.
*/
struct sdhc_clkc{
	u32 clk_div:16; /*[15:0] clk_div for TX_CLK 0: don't set it,
			1:div2, 2:div3, 3:div4 ...*/
	u32 clk_src_sel:3; /*[18:16] 0:osc, 1:fclk_div4, 2:fclk_div3, 3:fclk_div5*/
    u32 clk_en:1; /*[19] clock enable*/
    u32 rx_clk_phase:2; /*[21:20] Rx clock phase_sel related to Tx clock
                    0:0, 1:90, 2:180, 3:270*/
    u32 rx_clk_feedback:1; /*[22] Rx clock feedback enable*/
    u32 clk_ctl_enable:1; /*[23] Every time parameters are set,
                    need turn it off and then turn it on*/
	u32 clk_jic:1; /*[24] Clock JIC for clock gating control*/
	u32 reserved2:7; /*[31:25] Reserved*/
};

/*
* Note1: dma_urgent is just set when bandwidth is very tight
* Note2: pio_rdresp need to be combined with REG0_ARGU; 
* For R0, when 0, reading REG0 will get the normal 32bit response;
* For R2, when 1, reading REG0 will get CID[31:0], when 2, get CID[63:32],
* and so on; 6 or 7, will get original command argument.
*/
struct sdhc_pdma{
	u32 dma_mode:1; /*[0] 0:PIO mode, 1:DMA mode*/
	u32 pio_rdresp:3; /*[3:1] 0:[39:8] 1:1st 32bits, 2:2nd ...,
			6 or 7:command argument*/
	u32 dma_urgent:1; /*[4] 0:not urgent, 1:urgent*/
	u32 wr_burst:5; /*[9:5] Number in one Write request burst(0:1,1:2...)*/
	u32 rd_burst:5; /*[14:10] Number in one Read request burst(0:1, 1:2...)*/
	u32 rxfifo_th:6; /*[20:15] RxFIFO threshold, >=rxth, will request write*/
	u32 txfifo_th:6; /*[26:21] TxFIFO threshold, <=txth, will request read*/
	u32 rxfifo_manual_flush:1; /*[27] RxFIFO manual flush(self clear)*/
	u32 reserved; /*[31:28] reserved*/
};

struct sdhc_misc{
	u32 rx_cmd_delay:2; /*[1:0] rx cmd line delay control*/
    u32 rx_dat_delay:2; /*[3:2] rx dat line delay control*/
    u32 rxfifo_th:6; /*[9:4] RXFIFO Full Threshold,default 30*/
    u32 txfifo_th:6; /*[15:10] TXFIFO Empty Threshold,default 0*/
	u32 burst_num:6; /*[21:16] Burst Number*/
	u32 thread_id:6; /*[27:22] Thread ID*/
	u32 manual_stop:1; /*[28] 0:auto stop mode, 1:manual stop mode*/
	u32 reserved2:3; /*[31:29] reserved*/
};

struct sdhc_ictl{
	u32 resp_ok:1; /*[0] Response is received OK*/
	u32 resp_timeout:1; /*[1] Response Timeout Error*/
	u32 resp_err_crc:1; /*[2] Response CRC Error*/
	u32 resp_no_busy:1; /*[3] Data bit0 change to not busy from busy*/
	u32 data_1pack_ok:1; /*[4] One Package Data Completed ok*/
	u32 data_timeout:1; /*[5] One Package Data Failed (Timeout Error)*/
	u32 data_err_crc:1; /*[6] One Package Data Failed (CRC Error)*/
	u32 data_xfer_ok:1; /*[7] Data Transfer Completed ok*/
	u32 rx_higher:1; /*[8] RxFIFO count > threshold*/
	u32 tx_lower:1; /*[9] TxFIFO count < threshold*/
	u32 dat1_irq:1; /*[10] SDIO DAT1 Interrupt*/
	u32 dma_done:1; /*[11] DMA Done*/
	u32 rxfifo_full:1; /*[12] RxFIFO Full*/
	u32 txfifo_empty:1; /*[13] TxFIFO Empty*/
	u32 addi_dat1_irq:1; /*[14] Additional SDIO DAT1 Interrupt*/
	u32 reserved:1; /*[15] reserved*/
	u32 dat1_irq_delay:2; /*[17:16] sdio dat1 interrupt mask windows clear 
			delay control,0:2cycle 1:1cycles*/
	u32 reserved1:14; /*[31:18] reserved*/
};

/*Note1: W1C is write one clear.*/
struct sdhc_ista{
	u32 resp_ok:1; /*[0] Response is received OK (W1C)*/
	u32 resp_timeout:1; /*[1] Response is received Failed (Timeout Error) (W1C)*/
	u32 resp_err_crc:1; /*[2] Response is received Failed (CRC Error) (W1C)*/
	u32 resp_no_busy:1; /*[3] Data bit0 change to not busy from busy (W1C)*/
	u32 data_1pack_ok:1; /*[4] One Package Data Completed ok (W1C)*/
	u32 data_timeout:1; /*[5] One Package Data Failed (Timeout Error) (W1C)*/
	u32 data_err_crc:1; /*[6] One Package Data Failed (CRC Error) (W1C)*/
	u32 data_xfer_ok:1; /*[7] Data Transfer Completed ok (W1C)*/
	u32 rx_higher:1; /*[8] RxFIFO count > threshold (W1C)*/
	u32 tx_lower:1; /*[9] TxFIFO count < threshold (W1C)*/
	u32 dat1_irq:1; /*[10] SDIO DAT1 Interrupt (W1C)*/
	u32 dma_done:1; /*[11] DMA Done (W1C)*/
	u32 rxfifo_full:1; /*[12] RxFIFO Full(W1C)*/
	u32 txfifo_empty:1; /*[13] TxFIFO Empty(W1C)*/
	u32 addi_dat1_irq:1; /*[14] Additional SDIO DAT1 Interrupt*/
	u32 reserved:19; /*[31:13] reserved*/
};

/*
* Note1: Soft reset for DPHY TX/RX needs programmer to set it
* and then clear it manually.*/
struct sdhc_srst{
	u32 main_ctrl:1; /*[0] Soft reset for MAIN CTRL(self clear)*/
	u32 rxfifo:1; /*[1] Soft reset for RX FIFO(self clear)*/
	u32 txfifo:1; /*[2] Soft reset for TX FIFO(self clear)*/
	u32 dphy_rx:1; /*[3] Soft reset for DPHY RX*/
	u32 dphy_tx:1; /*[4] Soft reset for DPHY TX*/
	u32 dma_if:1; /*[5] Soft reset for DMA IF(self clear)*/
	u32 reserved:26; /*[31:6] reserved*/
};

#define SDHC_CLOCK_SRC_FCLK_DIV5        1
#define SDHC_ISTA_W1C_ALL               0x1fff
#define SDHC_SRST_ALL                   0x3f

/*SD_REG1_SEND*/
#define SEND_TOTAL_PACK_MASK	        (0xfffff<<12)
#define SEND_TOTAL_PACK_SHIFT	        (12)
#define SEND_DATA_STOP			        (1 << 11)
#define SEND_RX_TX				        (0 << 11)
#define SEND_DIRECTION_TX		        (1 << 10)
#define SEND_DIRECTION_RX		        (0 << 10)
#define SEND_RESP_CRC			        (0 << 9 )
#define SEND_RESP_NO_CRC		        (1 << 9 )
#define SEND_RESP_48BIT		            (0 << 8 )
#define SEND_RESP_136BIT		        (1 << 8 )
#define SEND_HAS_DATA			        (1 << 7 )
#define SEND_NO_DATA			        (0 << 7 )
#define SEND_HAS_RESP			        (1 << 6 )
#define SEND_NO_RESP			        (0 << 6 )
#define SEND_CMD_INDEX_MASK	            (0x3f)
#define SEND_CMD_INDEX_SHIFT	        (0)

/*SD_REG2_CTRL*/
#define CTRL_WR_ENDIAN_7		        (7 << 29)
#define CTRL_RD_ENDIAN_7   	            (7 << 24)
#define CTRL_ENDIAN_3                   (3 << 24)
#define CTRL_ENDIAN_1			(1 << 24)
#define CTRL_ENDIAN_2			(2 << 24)
#define CTRL_RC_PERIOD_SHIFT	        (20)
#define CTRL_RX_TIMEOUT_SHIFT	        (13)
#define CTRL_PACK_LEN_MASK		        (0x1ff<<4)
#define CTRL_PACK_LEN_SHIFT	            (4)
#define CTRL_DDR_MODE			        (1 << 2 )
#define CTRL_SDR_MODE			        (0 << 2 )
#define CTRL_DAT_1BIT			        (0 << 0 )
#define CTRL_DAT_4BIT			        (1 << 0 )
#define CTRL_DAT_8BIT			        (2 << 0 )
#define CTRL_DAT_BIT_MASK		        (3 << 0)

/*SD_REG3_STAT*/
#define STAT_DAT7_4_SHIFT		        (20)
#define STAT_TXFIFO_CNT_SHIFT	        (14)
#define STAT_TXFIFO_CNT_MASK	        (0x3f<<14)
#define STAT_RXFIFO_CNT_SHIFT	        (8)
#define STAT_RXFIFO_CNT_MASK	        (0x3f<<8)
#define STAT_CMD_SHIFT			        (5)
#define STAT_DAT3_0_SHIFT		        (1)
#define STAT_CMD_BUSY			        (1)

#define STAT_POLL_DAT7_4			    (0xf << 20 )
#define STAT_POLL_TXFIFO_EMPTY		    (0x3f << 14 )
#define STAT_POLL_RXFIFO_EMPTY		    (0x3f << 8 )
#define STAT_POLL_CMD				    (1 << 5 )
#define STAT_POLL_DAT3_2			    (3 << 3 )
#define STAT_POLL_DAT1				    (1 << 2 )
#define STAT_POLL_DAT0				    (1 << 1 )
#define STAT_POLL_CMD_BUSY			    (1 << 0 )

/*SD_REG4_CLKC*/
#define CLKC_JIC				        (1 << 24)
#define CLKC_CLK_CTRL_ENABLE	        (1 << 23)
#define CLKC_RX_CLK_FEEDBACK	        (1 << 22)
#define CLKC_RX_PHASE_0		            (0 << 20)
#define CLKC_RX_PHASE_90		        (1 << 20)
#define CLKC_RX_PHASE_180		        (2 << 20)
#define CLKC_RX_PHASE_270		        (3 << 20)
#define CLKC_RX_PHASE_MASK		        (3 << 20)
#define CLKC_CLK_EN			            (1 << 19)
#define CLKC_CLK_SRC_SEL_DDR	        (1 << 16)
#define CLKC_CLK_SRC_SEL_FCLK_DIV2	    (1 << 16)
#define CLKC_CLK_SRC_SEL_FCLK_DIV3	    (2 << 16)
#define CLKC_CLK_SRC_SEL_FCLK_DIV5	    (3 << 16)
#define CLKC_CLK_SEC_SEL_MASK	        (7 << 16)
#define CLKC_CLK_DIV_MASK		        (0xffff)

/*SD_REG6_PDMA*/
#define PDMA_RXFIFO_FLUSH		        (1 << 27)
#define PDMA_TXFIFO_THRESHOLD_MASK		(0x3f<<21)
#define PDMA_TXFIFO_THRESHOLD_SHIFT	    (21)
#define PDMA_RXFIFO_THRESHOLD_MASK		(0x3f<<15)
#define PDMA_RXFIFO_THRESHOLD_SHIFT	    (15)
#define PDMA_RX_BURST_NUM_MASK			(0x1f<<10)
#define PDMA_RX_BURST_NUM_SHIFT		    (10)
#define PDMA_TX_BURST_NUM_MASK			(0x1f<<5)
#define PDMA_TX_BURST_NUM_SHIFT		    (5)
#define PDMA_DMA_URGENT		            (1 << 4)
#define PDMA_RD_RESP_SHIFT		        (1)
#define PDMA_RD_RESP_MASK		        (7 << 1 )
#define PDMA_DMA_MODE			        (1 << 0 )
#define PDMA_PIO_MODE			        (0 << 0 )

/*SD_REG7_MISC*/
#define MISC_RESERVED			        (7 << 29)
#define MISC_MANUAL_STOP		        (1 << 28)
#define MISC_AUTO_STOP			        (0 << 28)
#define MISC_THREAD_ID_SHIFT	        (22)
#define MISC_BURST_NUM_SHIFT	        (16)
#define MISC_TX_EMPTY_THRESHOLD_SHIFT	(10)
#define MISC_TX_EMPTY_MASK		        (0x3f << 10)
#define MISC_RX_FULL_THRESHOLD_SHIFT	(4)
#define MISC_RX_FULL_MASK		        (0x3f << 4)
#define MISC_RX_DAT_DELAY_SHIFT		    (2)
#define MISC_RX_CMD_DELAY_SHIFT		    (0)

/*SD_REG9_ICTL*/
#define ICTL_DAT1_IRQ_CLR_DELAY_2CYCLE	(0 << 16)
#define ICTL_DAT1_IRQ_CLR_DELAY_1CYCLE	(1 << 16)
#define ICTL_ADDITIONAL_DAT1_INT		(1 << 14)
#define ICTL_TXFIFO_EMPTY				(1 << 13)
#define ICTL_RXFIFO_FULL				(1 << 12)
#define ICTL_DMA_DONE					(1 << 11)
#define ICTL_DAT1_INT					(1 << 10)
#define ICTL_TXFIFO_LOWER				(1 << 9 )
#define ICTL_RXFIFO_HIGHER				(1 << 8 )
#define ICTL_DATA_TRANS_OK				(1 << 7 )
#define ICTL_1PACK_DATA_CRC_ERR		    (1 << 6 )
#define ICTL_1PACK_DATA_TIMEOUT_ERR	    (1 << 5 )
#define ICTL_1PACK_DATA_COMPLETE		(1 << 4 )
#define ICTL_DAT0_TURN_READY			(1 << 3 )
#define ICTL_RESP_CRC_ERR				(1 << 2 )
#define ICTL_RESP_TIMEOUT_ERR			(1 << 1 )
#define ICTL_RESP_RECV_OK				(1 << 0 )

/*SD_REGA_ISTA WRITE 1 CLEAR*/
#define ISTA_ADDITIONAL_DAT1_INT		(1 << 14)
#define ISTA_TXFIFO_EMPTY				(1 << 13)
#define ISTA_RXFIFO_FULL				(1 << 12)
#define ISTA_DMA_DONE					(1 << 11)
#define ISTA_DAT1_INT					(1 << 10)
#define ISTA_TXFIFO_LOWER				(1 << 9 )
#define ISTA_RXFIFO_HIGHER				(1 << 8 )
#define ISTA_DATA_TRANS_OK				(1 << 7 )
#define ISTA_1PACK_DATA_CRC_ERR		    (1 << 6 )
#define ISTA_1PACK_DATA_TIMEOUT_ERR	    (1 << 5 )
#define ISTA_1PACK_DATA_COMPLETE		(1 << 4 )
#define ISTA_DAT0_TURN_READY			(1 << 3 )
#define ISTA_RESP_CRC_ERR				(1 << 2 )
#define ISTA_RESP_TIMEOUT_ERR			(1 << 1 )
#define ISTA_RESP_RECV_OK				(1 << 0 )

#define ICTL_ALL	0x1ffff
#define ICRL_ERR	(ICTL_1PACK_DATA_CRC_ERR | \
			ICTL_1PACK_DATA_TIMEOUT_ERR | ICTL_RESP_CRC_ERR | \
			ICTL_RESP_TIMEOUT_ERR)

#define ICRL_RESP	(ICTL_RESP_CRC_ERR| ICTL_RESP_TIMEOUT_ERR | \
			ICTL_RESP_RECV_OK)
			
#define ISTA_RESP	(ISTA_RESP_CRC_ERR| ISTA_RESP_TIMEOUT_ERR | \
			ISTA_RESP_RECV_OK)

#define ICRL_1DATA_ERR	(ICTL_1PACK_DATA_CRC_ERR| ICTL_1PACK_DATA_TIMEOUT_ERR)
#define ICTL_1DATA		(ICRL_1DATA_ERR|ICTL_1PACK_DATA_COMPLETE)
#define ICTL_ALL_DATA	(ICRL_1DATA_ERR|ICTL_DATA_TRANS_OK)
#define ICTL_DATA		(ICTL_1DATA|ICTL_ALL_DATA)

#define ISTA_1DATA_ERR	(ISTA_1PACK_DATA_CRC_ERR| ISTA_1PACK_DATA_TIMEOUT_ERR)
#define ISTA_1DATA		(ISTA_1DATA_ERR|ISTA_1PACK_DATA_COMPLETE)
#define ISTA_ALL_DATA	(ISTA_1DATA_ERR|ISTA_DATA_TRANS_OK)
#define ISTA_DATA		(ISTA_1DATA|ISTA_ALL_DATA)



#define ISTA_POLL_DATA_OK	(ISTA_1PACK_DATA_CRC_ERR| \
			ISTA_1PACK_DATA_TIMEOUT_ERR| ISTA_1PACK_DATA_COMPLETE)
	
#define ISTA_POLL_RESP_OK	(ISTA_RESP_RECV_OK)
#define ISTA_POLL_DAT0_BUSY	(ISTA_DAT0_TURN_READY)

#define ISTA_TIMEDOUT	(ISTA_1PACK_DATA_TIMEOUT_ERR| \
				ISTA_RESP_TIMEOUT_ERR)

#define ISTA_CRC_ERR	(ISTA_1PACK_DATA_CRC_ERR|ISTA_RESP_CRC_ERR)

#define ISTA_ERR		(ISTA_CRC_ERR|ISTA_TIMEDOUT)


/*SD_REGB_SRST*/
#define SRST_DMA_IF		(1 << 5 )
#define SRST_DPHY_TX		(1 << 4 )
#define SRST_DPHY_RX		(1 << 3 )
#define SRST_TXFIFO		(1 << 2 )
#define SRST_RXFIFO		(1 << 1 )
#define SRST_MAIN_CTRL		(1 << 0 )
#define SRST_ALL			(SRST_DMA_IF|SRST_DPHY_TX| \
	SRST_DPHY_RX|SRST_TXFIFO|SRST_RXFIFO|SRST_MAIN_CTRL)
#define SRST_ERR			(SRST_DMA_IF|SRST_TXFIFO| \
	SRST_RXFIFO|SRST_MAIN_CTRL)

#define STAT_POLL_TIMEOUT				0xfffff

#define MMC_RSP_136_NUM					4
#define MMC_MAX_DEVICE					3
#define MMC_TIMEOUT						5000

//#define printk(a...) 
#define DBG_LINE_INFO()  printk(KERN_WARNING "[%s] : %s\n",__func__,__FILE__);
//#define DBG_LINE_INFO()  
// #define dev_err(a,s) printk(KERN_INFO s);


#define AML_MMC_DISABLED_TIMEOUT	100
#define AML_MMC_SLEEP_TIMEOUT		1000
#define AML_MMC_OFF_TIMEOUT 8000

#define SDHC_BOUNCE_REQ_SIZE		(512*1024)
#define SDIO_BOUNCE_REQ_SIZE		(128*1024)
#define MMC_TIMEOUT_MS		20

#define MESON_SDIO_PORT_A 0
#define MESON_SDIO_PORT_B 1
#define MESON_SDIO_PORT_C 2
#define MESON_SDIO_PORT_XC_A 3
#define MESON_SDIO_PORT_XC_B 4
#define MESON_SDIO_PORT_XC_C 5

void aml_sdhc_request(struct mmc_host *mmc, struct mmc_request *mrq);
int aml_sdhc_get_cd(struct mmc_host *mmc);
extern void amlsd_init_debugfs(struct mmc_host *host);


#define     SPI_BOOT_FLAG                   0
#define     NAND_BOOT_FLAG                  1
#define     EMMC_BOOT_FLAG                  2
#define     CARD_BOOT_FLAG                  3
#define     SPI_NAND_FLAG                   4
#define     SPI_EMMC_FLAG                   5

#define R_BOOT_DEVICE_FLAG  READ_CBUS_REG(ASSIST_POR_CONFIG)
#define POR_NAND_BOOT() (((R_BOOT_DEVICE_FLAG & 7) == 7) || ((R_BOOT_DEVICE_FLAG & 7) == 6))
#define POR_SPI_BOOT()  (((R_BOOT_DEVICE_FLAG & 7) == 5) || ((R_BOOT_DEVICE_FLAG & 7) == 4))
#define POR_EMMC_BOOT() ((R_BOOT_DEVICE_FLAG & 7) == 3)
#define POR_CARD_BOOT() ((R_BOOT_DEVICE_FLAG & 7) == 0)


#define print_tmp(fmt, args...) do{\
		printk("[%s] " fmt, __FUNCTION__, ##args);	\
}while(0)

#define aml_jtag_gpioao()
#define aml_jtag_sd()

#endif

