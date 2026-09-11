// SPDX-License-Identifier: GPL-2.0-only
/*
 * Cadence Torrent SD0801 PHY driver.
 *
 * Copyright 2026 Mobileye Vision Technologies Ltd.
 *
 */

#include <dt-bindings/phy/phy.h>
#include <dt-bindings/phy/phy-cadence.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#define MAX_NUM_LANES		8
#define DEFAULT_MAX_BIT_RATE	8100 /* in Mbps */

#define POLL_TIMEOUT_US		5000
#define PLL_LOCK_TIMEOUT	100000

#define TORRENT_COMMON_CDB_OFFSET	0x0

#define TORRENT_TX_LANE_CDB_OFFSET(ln, block_offset, reg_offset) \
	((0x4000U << (block_offset)) + (((ln) << 9) << (reg_offset)))

#define TORRENT_RX_LANE_CDB_OFFSET(ln, block_offset, reg_offset) \
	((0x8000U << (block_offset)) + (((ln) << 9) << (reg_offset)))

#define TORRENT_PHY_PCS_COMMON_OFFSET(block_offset) (0xC000U << (block_offset))

#define TORRENT_PHY_PCS_LANE_CDB_OFFSET(ln, block_offset, reg_offset) \
	((0xD000U << (block_offset)) + (((ln) << 8) << (reg_offset)))

#define TORRENT_PHY_PMA_COMMON_OFFSET(block_offset) (0xE000U << (block_offset))

/*
 * register offsets from DPTX PHY register block base (i.e MHDP
 * register base + 0x30a00)
 */
#define PHY_AUX_CTRL				0x04
#define PHY_RESET				0x20
#define PMA_TX_ELEC_IDLE_SHIFT			4
#define PHY_PMA_XCVR_PLLCLK_EN			0x24
#define PHY_PMA_XCVR_PLLCLK_EN_ACK		0x28
#define PHY_PMA_XCVR_POWER_STATE_REQ		0x2c
#define PHY_POWER_STATE_LN(ln)			((ln) * 8)
#define PMA_XCVR_POWER_STATE_REQ_LN_MASK	0x3FU
#define PHY_PMA_XCVR_POWER_STATE_ACK		0x30
#define PHY_PMA_CMN_READY			0x34

/*
 * register offsets from SD0801 PHY register block base (i.e MHDP
 * register base + 0x500000)
 */
#define CMN_PID_TYPE			0x0000U
#define CMN_PID_NUM			0x0003U
#define CMN_PID_REV			0x0004U
#define CMN_PID_MFG			0x0008U
#define CMN_PID_NODE			0x0009U

#define CMN_SSM_BANDGAP_TMR		0x0021U
#define CMN_SSM_BIAS_TMR		0x0022U
#define CMN_PLLSM0_PLLPRE_TMR		0x002AU
#define CMN_PLLSM0_PLLLOCK_TMR		0x002CU
#define CMN_PLLSM1_PLLPRE_TMR		0x0032U
#define CMN_PLLSM1_PLLLOCK_TMR		0x0034U
#define CMN_CDIAG_CDB_PWRI_OVRD		0x0041U
#define CMN_CDIAG_XCVRC_PWRI_OVRD	0x0047U
#define CMN_CDIAG_REFCLK_OVRD		0x004CU
#define CMN_CDIAG_REFCLK_DRV0_CTRL	0x0050U
#define CMN_BGCAL_INIT_TMR		0x0064U
#define CMN_BGCAL_ITER_TMR		0x0065U
#define CMN_IBCAL_INIT_TMR		0x0074U

#define CMN_PLLx_Mx_REG_OFFSET(o, x)   ({ u32 __x = (x); (o) + 16U * (__x) * (__x); })

#define CMN_PLLx_VCOCAL_TCTRL(x)	(0x0082U + 32U * (x))
#define CMN_PLLx_VCOCAL_INIT_TMR(x)	(0x0084U + 32U * (x))
#define CMN_PLLx_VCOCAL_ITER_TMR(x)	(0x0085U + 32U * (x))
#define CMN_PLLx_VCOCAL_REFTIM_START(x)	(0x0086U + 32U * (x))
#define CMN_PLLx_VCOCAL_PLLCNT_START(x)	(0x0088U + 32U * (x))
#define CMN_PLLx_INTDIV_Mx(x)		CMN_PLLx_Mx_REG_OFFSET(0x0090U, x)
#define CMN_PLLx_FRACDIVL_Mx(x)		CMN_PLLx_Mx_REG_OFFSET(0x0091U, x)
#define CMN_PLLx_FRACDIVH_Mx(x)		CMN_PLLx_Mx_REG_OFFSET(0x0092U, x)
#define CMN_PLLx_HIGH_THR_Mx(x)		CMN_PLLx_Mx_REG_OFFSET(0x0093U, x)
#define CMN_PLLx_DSM_DIAG_Mx(x)		CMN_PLLx_Mx_REG_OFFSET(0x0094U, x)
#define CMN_PLLx_DSM_FBH_OVRD_Mx(x)	CMN_PLLx_Mx_REG_OFFSET(0x0095U, x)
#define CMN_PLLx_DSM_FBL_OVRD_Mx(x)	CMN_PLLx_Mx_REG_OFFSET(0x0096U, x)
#define CMN_PLLx_SS_CTRL1_Mx(x)		CMN_PLLx_Mx_REG_OFFSET(0x0098U, x)
#define CMN_PLLx_SS_CTRL2_Mx(x)		CMN_PLLx_Mx_REG_OFFSET(0x0099U, x)
#define CMN_PLLx_SS_CTRL3_Mx(x)		CMN_PLLx_Mx_REG_OFFSET(0x009AU, x)
#define CMN_PLLx_SS_CTRL4_Mx(x)		CMN_PLLx_Mx_REG_OFFSET(0x009BU, x)
#define CMN_PLLx_LOCK_REFCNT_START(x)	(0x009CU + 32U * (x))
#define CMN_PLLx_LOCK_PLLCNT_START(x)	(0x009EU + 32U * (x))
#define CMN_PLLx_LOCK_PLLCNT_THR(x)	(0x009FU + 32U * (x))

#define CMN_TXPUCAL_TUNE		0x0103U
#define CMN_TXPUCAL_INIT_TMR		0x0104U
#define CMN_TXPUCAL_ITER_TMR		0x0105U
#define CMN_TXPDCAL_TUNE		0x010BU
#define CMN_TXPDCAL_INIT_TMR		0x010CU
#define CMN_TXPDCAL_ITER_TMR		0x010DU
#define CMN_RXCAL_INIT_TMR		0x0114U
#define CMN_RXCAL_ITER_TMR		0x0115U
#define CMN_SD_CAL_INIT_TMR		0x0124U
#define CMN_SD_CAL_ITER_TMR		0x0125U
#define CMN_SD_CAL_REFTIM_START		0x0126U
#define CMN_SD_CAL_PLLCNT_START		0x0128U

#define CMN_PDIAG_PLLx_CTRL_Mx(x)	(0x01A0U + 16U * (x))
#define CMN_PDIAG_PLLx_CLK_SEL_Mx(x)	(0x01A1U + 16U * (x))
#define CMN_PDIAG_PLLx_ITRIM_Mx(x)      (0x01A3U + 16U * (x))
#define CMN_PDIAG_PLLx_CP_PADJ_Mx(x)	(0x01A4U + 16U * (x))
#define CMN_PDIAG_PLLx_CP_IADJ_Mx(x)	(0x01A5U + 16U * (x))
#define CMN_PDIAG_PLLx_FILT_PADJ_Mx(x)	(0x01A6U + 16U * (x))

#define CMN_DIAG_BIAS_OVRD1		0x01E1U

/* PMA TX Lane registers */
#define TX_TXCC_CTRL			0x0040U
#define TX_TXCC_CPOST_MULT_00		0x004CU
#define TX_TXCC_CPOST_MULT_01		0x004DU
#define TX_TXCC_MGNFS_MULT_000		0x0050U
#define TX_TXCC_MGNFS_MULT_100		0x0054U
#define DRV_DIAG_TX_DRV			0x00C6U
#define XCVR_DIAG_PLLDRC_CTRL		0x00E5U
#define XCVR_DIAG_HSCLK_SEL		0x00E6U
#define XCVR_DIAG_HSCLK_DIV		0x00E7U
#define XCVR_DIAG_RXCLK_CTRL		0x00E9U
#define XCVR_DIAG_BIDI_CTRL		0x00EAU
#define XCVR_DIAG_PSC_OVRD		0x00EBU
#define TX_PSC_A0			0x0100U
#define TX_PSC_A1			0x0101U
#define TX_PSC_A2			0x0102U
#define TX_PSC_A3			0x0103U
#define TX_RCVDET_ST_TMR		0x0123U
#define TX_DIAG_ACYA			0x01E7U
#define TX_DIAG_ACYA_HBDC_MASK		0x0001U

/* PMA RX Lane registers */
#define RX_PSC_A0			0x0000U
#define RX_PSC_A1			0x0001U
#define RX_PSC_A2			0x0002U
#define RX_PSC_A3			0x0003U
#define RX_PSC_CAL			0x0006U
#define RX_SDCAL0_INIT_TMR		0x0044U
#define RX_SDCAL0_ITER_TMR		0x0045U
#define RX_SDCAL1_INIT_TMR		0x004CU
#define RX_SDCAL1_ITER_TMR		0x004DU
#define RX_CDRLF_CNFG			0x0080U
#define RX_CDRLF_CNFG2			0x0081U
#define RX_CDRLF_CNFG3			0x0082U
#define RX_SIGDET_HL_FILT_TMR		0x0090U
#define RX_REE_GCSM1_CTRL		0x0108U
#define RX_REE_GCSM1_EQENM_PH1		0x0109U
#define RX_REE_GCSM1_EQENM_PH2		0x010AU
#define RX_REE_GCSM2_CTRL		0x0110U
#define RX_REE_PERGCSM_CTRL		0x0118U
#define RX_REE_PEAK_UTHR		0x0142U
#define RX_REE_PEAK_LTHR		0x0143U
#define RX_REE_ATTEN_THR		0x0149U
#define RX_REE_TAP1_CLIP		0x0171U
#define RX_REE_TAP2TON_CLIP		0x0172U
#define RX_REE_SMGM_CTRL1		0x0177U
#define RX_REE_SMGM_CTRL2		0x0178U
#define RX_DIAG_DFE_CTRL		0x01E0U
#define RX_DIAG_DFE_AMP_TUNE_2		0x01E2U
#define RX_DIAG_DFE_AMP_TUNE_3		0x01E3U
#define RX_DIAG_REE_DAC_CTRL		0x01E4U
#define RX_DIAG_NQST_CTRL		0x01E6U
#define RX_DIAG_SIGDET_TUNE		0x01E8U
#define RX_DIAG_PI_RATE			0x01F4U
#define RX_DIAG_PI_CAP			0x01F5U
#define RX_DIAG_ACYA			0x01FFU

/* PHY PCS common registers */
#define PHY_PIPE_CMN_CTRL1		0x0000U
#define PHY_PLL_CFG			0x000EU
#define PHY_PIPE_USB3_GEN2_PRE_CFG0	0x0020U
#define PHY_PIPE_USB3_GEN2_POST_CFG0	0x0022U
#define PHY_PIPE_USB3_GEN2_POST_CFG1	0x0023U

/* PHY PCS lane registers */
#define PHY_PCS_ISO_LINK_CTRL		0x000BU

/* PHY PMA common registers */
#define PHY_PMA_CMN_CTRL1		0x0000U
#define PHY_PMA_CMN_CTRL2		0x0001U
#define PHY_PMA_PLL_RAW_CTRL		0x0003U

struct tp_reg_base {
	void __iomem *base;
	u8 reg_offset_shift;
};

struct tp_reg_field {
	void __iomem *reg_addr;
	u16 mask;
};

#define CDNS_TORRENT_OUTPUT_CLOCKS 3

static const char *const tp_clk_names[] = {
	[CDNS_TORRENT_REFCLK_DRIVER] = "refclk-driver",
	[CDNS_TORRENT_DERIVED_REFCLK] = "refclk-der",
	[CDNS_TORRENT_RECEIVED_REFCLK] = "refclk-rec",
};

enum tp_refclk_out_cmn {
	CMN_CDIAG_REFCLK_DRV0_CTRL_1,
	CMN_CDIAG_REFCLK_DRV0_CTRL_4,
	CMN_CDIAG_REFCLK_DRV0_CTRL_5,
	CMN_CDIAG_REFCLK_DRV0_CTRL_6,
	TP_REFCLK_OUT_NUM_CMN_CONFIG
};

static const int tp_refclk_driver_parent_idx[] = {
	CDNS_TORRENT_DERIVED_REFCLK, CDNS_TORRENT_RECEIVED_REFCLK
};

static const u32 tp_refclk_driver_mux_table[] = { 1, 0 };

struct tp_refclk_driver {
	struct clk_hw hw;
	struct tp_reg_field fields[TP_REFCLK_OUT_NUM_CMN_CONFIG];
	u8 parent_index;
};

#define tp_to_refclk_driver(_hw) \
	container_of(_hw, struct tp_refclk_driver, hw)

#define tp_from_derived_refclk(_hw) \
	container_of(_hw, struct tp_phy, derived_refclk)

#define tp_from_received_refclk(_hw) \
	container_of(_hw, struct tp_phy, received_refclk)

/* Revisions and revision masks */
enum tp_revision {
	REV_R100,
	REV_R110,
	REV_R120,
	REV_R130,
	REV_R140,
	REV_R150,
	REV_R200,
	REV_R210,
	REV_R220,
};

#define REV_MSK_R220		BIT(REV_R220)
#define REV_MSK_R210		BIT(REV_R210)
#define REV_MSK_R210P		(REV_MSK_R210 | REV_MSK_R220)
#define REV_MSK_R200		BIT(REV_R200)
#define REV_MSK_R200P		(REV_MSK_R200 | REV_MSK_R210P)
#define REV_MSK_R150		BIT(REV_R150)
#define REV_MSK_R150P		(REV_MSK_R150 | REV_MSK_R200P)
#define REV_MSK_R140		BIT(REV_R140)
#define REV_MSK_R140P		(REV_MSK_R140 | REV_MSK_R150P)
#define REV_MSK_R130		BIT(REV_R130)
#define REV_MSK_R130P		(REV_MSK_R130 | REV_MSK_R140P)
#define REV_MSK_R120		BIT(REV_R120)
#define REV_MSK_R120P		(REV_MSK_R120 | REV_MSK_R130P)
#define REV_MSK_R110		BIT(REV_R110)
#define REV_MSK_R110P		(REV_MSK_R110 | REV_MSK_R120P)
#define REV_MSK_R100		BIT(REV_R100)
#define REV_MSK_R100P		(REV_MSK_R100 | REV_MSK_R110P)

/* Technology tags */
#define TECH_TSMC_16		BIT(0)
#define TECH_TSMC_16_0805	BIT(1)
#define TECH_TSMC_12		BIT(2)
#define TECH_TSMC_7		BIT(3)
#define TECH_TSMC_5		BIT(4)
#define TECH_TSMC_5A		BIT(5)
#define TECH_TSMC_5_0806	BIT(6)
#define TECH_TSMC_4		BIT(7)
#define TECH_TSMC_3		BIT(8)
#define TECH_SS_7		BIT(9)
#define TECH_SS_5		BIT(10)
#define TECH_IN_16		BIT(11)
#define TECH_IN_I18A_0801	BIT(12)
#define TECH_IN_I18A_0807	BIT(13)
#define TECH_UMC_14		BIT(14)

#define TECH_SS_ALL		(TECH_SS_7 | TECH_SS_5)

#define TECH_TSMC_ALL		(TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12 | \
				 TECH_TSMC_7 | TECH_TSMC_5 | TECH_TSMC_5A | \
				 TECH_TSMC_5_0806 | TECH_TSMC_4 | TECH_TSMC_3)

#define TECH_INTEL_ALL		(TECH_IN_16 | TECH_IN_I18A_0801 | TECH_IN_I18A_0807)

#define TECH_UMC_ALL		(TECH_UMC_14)

#define TECH_ALL		(TECH_SS_ALL | TECH_TSMC_ALL | TECH_INTEL_ALL | TECH_UMC_ALL)

/* PHY standards */
enum tp_std_type {
	STD_TYPE_NONE,
	STD_TYPE_PCIE,
	STD_TYPE_USB_TXRX,
	STD_TYPE_DP,
	STD_TYPE_XAUI,
	STD_TYPE_SGMII,
	STD_TYPE_SATA,
	STD_TYPE_USXGMII,
};

#define STD_FLAG_NONE		CDNS_TORRENT_MODE_NONE
#define STD_FLAG_USB_G1		CDNS_TORRENT_MODE_USB_G1
#define STD_FLAG_USB_IO_11V	CDNS_TORRENT_MODE_USB_IO_11V
#define STD_FLAG_XAUI_LP	CDNS_TORRENT_MODE_XAUI_LP
#define STD_FLAG_SATA_LP	CDNS_TORRENT_MODE_SATA_LP
#define STD_FLAG_SGMIIP		CDNS_TORRENT_MODE_2500_BASEX
#define STD_FLAG_QSGMII		CDNS_TORRENT_MODE_QSGMII
#define STD_FLAG_PCIE_ML	0x0100
#define STD_FLAG_S_SSC		0x0200 /* Same SSC */
#define STD_FLAG_U_SSC		0x0400 /* No SSC for PCIe */

#define STD_FIND_CFG_FLAG_MASK	(STD_FLAG_USB_G1 | STD_FLAG_XAUI_LP | STD_FLAG_SATA_LP | \
				 STD_FLAG_SGMIIP | STD_FLAG_PCIE_ML)

#define STD_FLAGS_FROM_ID(std_id)	((std_id) >> 4)
#define STD_TYPE_FROM_ID(std_id)	((std_id) & 0x0F)
#define STD_ID(std_type, std_flags)	((std_flags) << 4 | (std_type))

enum tp_standard0 {
	STD0_NONE = STD_ID(STD_TYPE_NONE, STD_FLAG_NONE),
	/* PCIe Gen 1,2,3 Single Link */
	STD0_PCIE_SL = STD_ID(STD_TYPE_PCIE, STD_FLAG_NONE),
	/* PCIe Gen 1,2,3 Multiple Links */
	STD0_PCIE_ML = STD_ID(STD_TYPE_PCIE, STD_FLAG_PCIE_ML),
	/* USB 3.0 / 3.1 (Gen 1 Only) */
	STD0_USB_G1 = STD_ID(STD_TYPE_USB_TXRX, STD_FLAG_USB_G1),
	/* USB 3.0 / 3.1 */
	STD0_USB = STD_ID(STD_TYPE_USB_TXRX, STD_FLAG_NONE),
	/* Display Port / eDP */
	STD0_DP = STD_ID(STD_TYPE_DP, STD_FLAG_NONE),
	/* XAUI (1X Only) */
	STD0_XAUI_LP = STD_ID(STD_TYPE_XAUI, STD_FLAG_XAUI_LP),
	/* XAUI */
	STD0_XAUI = STD_ID(STD_TYPE_XAUI, STD_FLAG_NONE),
	/* SGMII / QSGMII */
	STD0_SGMII_1G_5G = STD_ID(STD_TYPE_SGMII, STD_FLAG_NONE),
	/* SGMII 2.5 Gbps */
	STD0_SGMII_2P5G = STD_ID(STD_TYPE_SGMII, STD_FLAG_SGMIIP),
	/* SATA (1.5 and 3.0 Only) */
	STD0_SATA_LP = STD_ID(STD_TYPE_SATA, STD_FLAG_SATA_LP),
	/* SATA */
	STD0_SATA = STD_ID(STD_TYPE_SATA, STD_FLAG_NONE),
	/* USXGMII */
	STD0_USXGMII = STD_ID(STD_TYPE_USXGMII, STD_FLAG_NONE),
};

enum tp_standard1 {
	STD1_NONE = STD_ID(STD_TYPE_NONE, STD_FLAG_NONE),
	/* USB 3.0 / 3.1 */
	STD1_USB = STD_ID(STD_TYPE_USB_TXRX, STD_FLAG_NONE),
	/* USB 3.0 / 3.1 Same SSC */
	STD1_USB_SS = STD_ID(STD_TYPE_USB_TXRX, STD_FLAG_S_SSC),
	/* USB 3.0 / 3.1 Unique SSC */
	STD1_USB_US = STD_ID(STD_TYPE_USB_TXRX, STD_FLAG_U_SSC),
	/* Display Port / eDP */
	STD1_DP = STD_ID(STD_TYPE_DP, STD_FLAG_NONE),
	/* XAUI */
	STD1_XAUI = STD_ID(STD_TYPE_XAUI, STD_FLAG_NONE),
	/* XAUI Same SSC since share PLL0 */
	STD1_XAUI_SS = STD_ID(STD_TYPE_XAUI, STD_FLAG_S_SSC),
	/* SGMII / QSGMII */
	STD1_SGMII_1G_5G = STD_ID(STD_TYPE_SGMII, STD_FLAG_NONE),
	/* SGMII 2.5 Gbps */
	STD1_SGMII_2P5G = STD_ID(STD_TYPE_SGMII, STD_FLAG_SGMIIP),
	/* SGMII Same SSC */
	STD1_SGMII_1G_5G_SS = STD_ID(STD_TYPE_SGMII, STD_FLAG_S_SSC),
	/* SGMII Unique SSC */
	STD1_SGMII_1G_5G_US = STD_ID(STD_TYPE_SGMII, STD_FLAG_U_SSC),
	/* SATA */
	STD1_SATA = STD_ID(STD_TYPE_SATA, STD_FLAG_NONE),
	/* USXGMII */
	STD1_USXGMII = STD_ID(STD_TYPE_USXGMII, STD_FLAG_NONE),
};

enum tp_pll_freq {
	PLL_UNUSED,
	PLL_DP,
	PLL_6_00_GHZ,
	PLL_6_25_GHZ,
	PLL_8_00_GHZ,
	PLL_10_00_GHZ,
	PLL_10_31_GHZ,
};

enum tp_pll_fb_div_mode {
	PLL_FB_DIV_MODE_FBR,
	PLL_FB_DIV_MODE_DR,
	PLL_FB_DIV_MODE_D32,
};

union tp_reg_cfg_key {
	struct {
		u16 std;
		u16 rev;
		u32 tech;
	};
	u64 key;
};

struct tp_reg_cfg {
	union tp_reg_cfg_key cfg;
	u32 std_flags;
	u16 val;
};

struct tp_instance {
	struct tp_std_data *std;
	struct phy *phy;
	struct reset_control *lnk_rst;
	u8 mlane;
	u8 num_lanes;
	u8 max_lanes;
};

struct tp_std_data {
	struct tp_instance phys[MAX_NUM_LANES];
	u8 nb_phys;
	u8 ssc_mode; /* CDNS_SERDES_NO_SSC, ... */
	u16 max_bit_rate;
	u32 flags;
	enum tp_std_type type;
};

#define MAX_NUM_STANDARDS 2

enum tp_pll_id { PLL0_MODE0, PLL0_MODE1, PLL1_MODE0, TP_NB_PLLS };

struct tp_pll_data {
	u32 freq_khz;
	u32 ref_clk_khz;
	bool ssc;
	bool fractional_mode;
	enum tp_pll_fb_div_mode fb_div_mode : 8;
};

struct tp_init_data {
	u8 block_offset_shift;
	u8 reg_offset_shift;
	u8 mask_available_lanes;
	bool usxgmii_refclk_rcv_out_en : 1;
	bool rxda_des_clk_2x_en : 1;
};

struct tp_phy {
	void __iomem *base; /* DPTX registers base */
	void __iomem *sd_base; /* SD0801 registers base */

	/* Protect `nb_phy_on` and concurrent register writes */
	struct mutex lock;
	struct reset_control *phy_rst;
	struct reset_control *apb_rst;

	struct device *dev;
	struct clk *clk;
	struct clk *clk1;
	u32 ref_clk_khz;
	u32 ref_clk1_khz;

	u32 tech_tag;
	enum tp_revision rev;
	bool already_configured;
	u8 mask_used_lanes;

	u8 nb_phy_on;
	u8 nb_standards;
	struct tp_pll_data plls[TP_NB_PLLS];
	struct tp_std_data std_data[MAX_NUM_STANDARDS];
	struct tp_std_data *std_ptr[MAX_NUM_STANDARDS];

	const struct tp_link_cfg *link_cfg;
	const struct tp_init_data *init_data;

	struct tp_reg_base reg_common_cdb;
	struct tp_reg_base reg_phy_pcs_common_cdb;
	struct tp_reg_base reg_phy_pma_common_cdb;
	struct tp_reg_base reg_tx_lane_cdb[MAX_NUM_LANES];
	struct tp_reg_base reg_rx_lane_cdb[MAX_NUM_LANES];
	struct tp_reg_base reg_phy_pcs_lane_cdb[MAX_NUM_LANES];
	struct tp_reg_field phy_pll_cfg;
	struct tp_reg_field phy_pipe_cmn_ctrl1_0;
	struct tp_reg_field cmn_cdiag_refclk_ovrd_4;

	struct clk_hw_onecell_data *clk_hw_data;
	struct tp_refclk_driver refclk_driver;
	struct clk_hw derived_refclk;
	struct clk_hw received_refclk;
};

enum phy_powerstate {
	POWERSTATE_A0 = 0,
	/* Powerstate A1 is unused */
	POWERSTATE_A2 = 2,
	POWERSTATE_A3 = 3,
};

#define REG_COMPUTED 0xFFFF

struct tp_link_cfg {
	enum tp_standard0 std0;
	enum tp_standard1 std1;
	enum tp_pll_freq pll_freq[TP_NB_PLLS];
	u16 phy_pll_cfg;
	u16 pll_clk_sel[TP_NB_PLLS];
	u16 xcvr_diag_plldrc_ctrl[MAX_NUM_STANDARDS];
	u16 xcvr_diag_hsclk_sel[MAX_NUM_STANDARDS];
	u16 xcvr_diag_hsclk_div[MAX_NUM_STANDARDS];
};

static const struct tp_link_cfg tp_link_cfgs[] = {
	{
		/* PLL 0 M0: PCIe, PLL 0 M1: PCIe, PLL 1: None */
		.std0 = STD0_PCIE_SL,	.std1 = STD1_NONE,
		.pll_freq = { PLL_10_00_GHZ, PLL_8_00_GHZ, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0001,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x0012 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0000 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: PCIe (No SSC), PLL 0 M1: PCIe (No SSC), PLL 1: USB */
		.std0 = STD0_PCIE_SL,	.std1 = STD1_USB_US,
		.pll_freq = { PLL_10_00_GHZ, PLL_8_00_GHZ, PLL_10_00_GHZ },
		.phy_pll_cfg = 0x0003,
		.pll_clk_sel = { 0x0601, 0x0400, 0x8600 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x00C9 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0011 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: PCIe, PLL 0 M1: PCIe, PLL 1: DP */
		.std0 = STD0_PCIE_SL,	.std1 = STD1_DP,
		.pll_freq = { PLL_10_00_GHZ, PLL_8_00_GHZ, PLL_DP },
		.phy_pll_cfg = 0x0003,
		.pll_clk_sel = { 0x0601, 0x0400, REG_COMPUTED },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x0019 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0001 },
		.xcvr_diag_hsclk_div = { 0x0001, REG_COMPUTED },
	},
	{
		/* PLL 0 M0: PCIe, PLL 0 M1: PCIe, PLL 1: XAUI */
		.std0 = STD0_PCIE_SL,	.std1 = STD1_XAUI,
		.pll_freq = { PLL_10_00_GHZ, PLL_8_00_GHZ, PLL_6_25_GHZ },
		.phy_pll_cfg = 0x0003,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0600 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x0089 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0011 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: PCIe (No SSC), PLL 0 M1: PCIe (No SSC), PLL 1: SGMII */
		.std0 = STD0_PCIE_SL,	.std1 = STD1_SGMII_1G_5G_US,
		.pll_freq = { PLL_10_00_GHZ, PLL_8_00_GHZ, PLL_10_00_GHZ },
		.phy_pll_cfg = 0x0003,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0601 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x091B },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0101 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0003 },
	},
	{
		/* PLL 0 M0: PCIe, PLL 0 M1: PCIe, PLL 1: SGMII 2.5 */
		.std0 = STD0_PCIE_SL,	.std1 = STD1_SGMII_2P5G,
		.pll_freq = { PLL_10_00_GHZ, PLL_8_00_GHZ, PLL_6_25_GHZ },
		.phy_pll_cfg = 0x0003,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0601 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x0092 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0010 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: PCIe, PLL 0 M1: PCIe, PLL 1: USXGMII */
		.std0 = STD0_PCIE_SL,	.std1 = STD1_USXGMII,
		.pll_freq = { PLL_10_00_GHZ, PLL_8_00_GHZ, PLL_10_31_GHZ },
		.phy_pll_cfg = 0x0003,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x0089 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0011 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: PCIe, PLL 0 M1: PCIe, PLL 1: SATA */
		.std0 = STD0_PCIE_SL,	.std1 = STD1_SATA,
		.pll_freq = { PLL_10_00_GHZ, PLL_8_00_GHZ, PLL_6_00_GHZ },
		.phy_pll_cfg = 0x0003,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0600 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x089A },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0111 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0013 },
	},
	{
		/* PLL 0 M0: PCIe, PLL 0 M1: Unused, PLL 1: PCIe */
		.std0 = STD0_PCIE_ML,	.std1 = STD1_NONE,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0812, 0x0012 },
		.xcvr_diag_hsclk_sel = { 0x0100, 0x0000 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: PCIe / USB, PLL 0 M1: Unused, PLL 1: PCIe */
		.std0 = STD0_PCIE_ML,	.std1 = STD1_USB_SS,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x8600, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0812, 0x0041 },
		.xcvr_diag_hsclk_sel = { 0x0100, 0x0000 },
		.xcvr_diag_hsclk_div = { 0x0013, 0x0001 },
	},
	{
		/* PLL 0 M0: PCIe / SGMII, PLL 0 M1: Unused, PLL 1: PCIe */
		.std0 = STD0_PCIE_ML,	.std1 = STD1_SGMII_1G_5G_SS,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0812, 0x0113 },
		.xcvr_diag_hsclk_sel = { 0x0100, 0x0000 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0003 },
	},
	{
		/* PLL 0 M0: USB, PLL 0 M1: Unused, PLL 1: Unused */
		.std0 = STD0_USB_G1,	.std1 = STD1_NONE,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0000,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0011, 0x0012 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0000 },
		.xcvr_diag_hsclk_div = { 0x0000, 0x0001 },
	},
	{
		/* PLL 0 M0: USB, PLL 0 M1: Unused, PLL 1: Unused */
		.std0 = STD0_USB,	.std1 = STD1_NONE,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0000,
		.pll_clk_sel = { 0x8600, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0041, 0x0012 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0000 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: USB, PLL 0 M1: Unused, PLL 1: DP */
		.std0 = STD0_USB,	.std1 = STD1_DP,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_DP },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x8600, 0x0400, REG_COMPUTED },
		.xcvr_diag_plldrc_ctrl = { 0x0041, 0x0019 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0001 },
		.xcvr_diag_hsclk_div = { 0x0001, REG_COMPUTED },
	},
	{
		/* PLL 0 M0: USB, PLL 0 M1: Unused, PLL 1: XAUI */
		.std0 = STD0_USB,	.std1 = STD1_XAUI,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_6_25_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x8600, 0x0400, 0x0600 },
		.xcvr_diag_plldrc_ctrl = { 0x0041, 0x0089 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0011 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: USB, PLL 0 M1: Unused, PLL 1: SGMII */
		.std0 = STD0_USB,	.std1 = STD1_SGMII_1G_5G,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_10_00_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x8600, 0x0400, 0x0601 },
		.xcvr_diag_plldrc_ctrl = { 0x0041, 0x0A9B },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0111 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0103 },
	},
	{
		/* PLL 0 M0: USB, PLL 0 M1: Unused, PLL 1: SATA */
		.std0 = STD0_USB,	.std1 = STD1_SATA,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_6_00_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x8600, 0x0400, 0x0600 },
		.xcvr_diag_plldrc_ctrl = { 0x0041, 0x089A },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0111 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0013 },
	},
	{
		/* PLL 0 M0: USB, PLL 0 M1: Unused, PLL 1: USXGMII */
		.std0 = STD0_USB,	.std1 = STD1_USXGMII,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_10_31_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x8600, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0041, 0x0089 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0011 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: DP, PLL 0 M1: Unused, PLL 1: Unused */
		.std0 = STD0_DP,	.std1 = STD1_NONE,
		.pll_freq = { PLL_DP, PLL_UNUSED, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0000,
		.pll_clk_sel = { REG_COMPUTED, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0011, 0x0012 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0000 },
		.xcvr_diag_hsclk_div = { REG_COMPUTED, 0x0001 },
	},
	{
		/* PLL 0 M0: DP, PLL 0 M1: Unused, PLL 1: DP */
		.std0 = STD0_DP,	.std1 = STD1_DP,
		.pll_freq = { PLL_DP, PLL_UNUSED, PLL_DP },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { REG_COMPUTED, 0x0400, REG_COMPUTED },
		.xcvr_diag_plldrc_ctrl = { 0x0011, 0x0019 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0001 },
		.xcvr_diag_hsclk_div = { REG_COMPUTED, REG_COMPUTED },
	},
	{
		/* PLL 0 M0: DP, PLL 0 M1: Unused, PLL 1: XAUI */
		.std0 = STD0_DP,	.std1 = STD1_XAUI,
		.pll_freq = { PLL_DP, PLL_UNUSED, PLL_6_25_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { REG_COMPUTED, 0x0400, 0x0600 },
		.xcvr_diag_plldrc_ctrl = { 0x0011, 0x0089 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0011 },
		.xcvr_diag_hsclk_div = { REG_COMPUTED, 0x0001 },
	},
	{
		/* PLL 0 M0: DP, PLL 0 M1: Unused, PLL 1: USXGMII */
		.std0 = STD0_DP,	.std1 = STD1_USXGMII,
		.pll_freq = { PLL_DP, PLL_UNUSED, PLL_10_31_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { REG_COMPUTED, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0011, 0x0089 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0011 },
		.xcvr_diag_hsclk_div = { REG_COMPUTED, 0x0001 },
	},
	{
		/* PLL 0 M0: XAUI, PLL 0 M1: Unused, PLL 1: Unused */
		.std0 = STD0_XAUI_LP,	.std1 = STD1_NONE,
		.pll_freq = { PLL_6_25_GHZ, PLL_UNUSED, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0000,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0001, 0x0012 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0000 },
		.xcvr_diag_hsclk_div = { 0x0000, 0x0001 },
	},
	{
		/* PLL 0 M0: XAUI, PLL 0 M1: Unused, PLL 1: Unused */
		.std0 = STD0_XAUI,	.std1 = STD1_NONE,
		.pll_freq = { PLL_6_25_GHZ, PLL_UNUSED, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0000,
		.pll_clk_sel = { 0x0600, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0001, 0x0012 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0000 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: XAUI / XAUI, PLL 0 M1: Unused, PLL 1: Unused */
		.std0 = STD0_XAUI,	.std1 = STD1_XAUI_SS,
		.pll_freq = { PLL_6_25_GHZ, PLL_UNUSED, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0000,
		.pll_clk_sel = { 0x0600, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0001, 0x0001 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0000 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: SGMII, PLL 0 M1: Unused, PLL 1: SGMII 2.5 */
		.std0 = STD0_SGMII_1G_5G,	.std1 = STD1_SGMII_2P5G,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_6_25_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0601 },
		.xcvr_diag_plldrc_ctrl = { 0x0113, 0x0092 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0010 },
		.xcvr_diag_hsclk_div = { 0x0003, 0x0001 },
	},
	{
		/* PLL 0 M0: SGMII, PLL 0 M1: Unused, PLL 1: Unused */
		.std0 = STD0_SGMII_1G_5G,	.std1 = STD1_NONE,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0000,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0113, 0x0012 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0000 },
		.xcvr_diag_hsclk_div = { 0x0003, 0x0001 },
	},
	{
		/* PLL 0 M0: SGMII 2.5, PLL 0 M1: Unused, PLL 1: Unused */
		.std0 = STD0_SGMII_2P5G,	.std1 = STD1_NONE,
		.pll_freq = { PLL_6_25_GHZ, PLL_UNUSED, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0000,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x0012 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0000 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: SGMII, PLL 0 M1: Unused, PLL 1: DP */
		.std0 = STD0_SGMII_1G_5G,	.std1 = STD1_DP,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_DP },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x0601, 0x0400, REG_COMPUTED },
		.xcvr_diag_plldrc_ctrl = { 0x0113, 0x0019 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0001 },
		.xcvr_diag_hsclk_div = { 0x0003, REG_COMPUTED },
	},
	{
		/* PLL 0 M0: SGMII 2.5, PLL 0 M1: Unused, PLL 1: DP */
		.std0 = STD0_SGMII_2P5G,	.std1 = STD1_DP,
		.pll_freq = { PLL_6_25_GHZ, PLL_UNUSED, PLL_DP },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x0601, 0x0400, REG_COMPUTED },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x0019 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0001 },
		.xcvr_diag_hsclk_div = { 0x0001, REG_COMPUTED },
	},
	{
		/* PLL 0 M0: SGMII, PLL 0 M1: Unused, PLL 1: XAUI */
		.std0 = STD0_SGMII_1G_5G,	.std1 = STD1_XAUI,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_6_25_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0600 },
		.xcvr_diag_plldrc_ctrl = { 0x0113, 0x0089 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0011 },
		.xcvr_diag_hsclk_div = { 0x0003, 0x0001 },
	},
	{
		/* PLL 0 M0: SGMII 2.5, PLL 0 M1: Unused, PLL 1: XAUI */
		.std0 = STD0_SGMII_2P5G,	.std1 = STD1_XAUI,
		.pll_freq = { PLL_6_25_GHZ, PLL_UNUSED, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0000,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x0001 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0000 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: SGMII, PLL 0 M1: Unused, PLL 1: USB */
		.std0 = STD0_SGMII_1G_5G,	.std1 = STD1_USB,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_10_00_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x0601, 0x0400, 0x8600 },
		.xcvr_diag_plldrc_ctrl = { 0x0113, 0x00C9 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0011 },
		.xcvr_diag_hsclk_div = { 0x0003, 0x0001 },
	},
	{
		/* PLL 0 M0: SGMII 2.5, PLL 0 M1: Unused, PLL 1: USB */
		.std0 = STD0_SGMII_2P5G,	.std1 = STD1_USB,
		.pll_freq = { PLL_6_25_GHZ, PLL_UNUSED, PLL_10_00_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x0601, 0x0400, 0x8600 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x00C9 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0011 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: SGMII, PLL 0 M1: Unused, PLL 1: SATA */
		.std0 = STD0_SGMII_1G_5G,	.std1 = STD1_SATA,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_6_00_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0600 },
		.xcvr_diag_plldrc_ctrl = { 0x0113, 0x089A },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0111 },
		.xcvr_diag_hsclk_div = { 0x0003, 0x0013 },
	},
	{
		/* PLL 0 M0: SGMII 2.5, PLL 0 M1: Unused, PLL 1: SATA */
		.std0 = STD0_SGMII_2P5G,	.std1 = STD1_SATA,
		.pll_freq = { PLL_6_25_GHZ, PLL_UNUSED, PLL_6_00_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0600 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x089A },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0111 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0013 },
	},
	{
		/* PLL 0 M0: SGMII, PLL 0 M1: Unused, PLL 1: USXGMII */
		.std0 = STD0_SGMII_1G_5G,	.std1 = STD1_USXGMII,
		.pll_freq = { PLL_10_00_GHZ, PLL_UNUSED, PLL_10_31_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0113, 0x0089 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0011 },
		.xcvr_diag_hsclk_div = { 0x0003, 0x0001 },
	},
	{
		/* PLL 0 M0: SGMII 2.5, PLL 0 M1: Unused, PLL 1: USXGMII */
		.std0 = STD0_SGMII_2P5G,	.std1 = STD1_USXGMII,
		.pll_freq = { PLL_6_25_GHZ, PLL_UNUSED, PLL_10_31_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x0089 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0011 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: SATA, PLL 0 M1: Unused, PLL 1: Unused */
		.std0 = STD0_SATA_LP,	.std1 = STD1_NONE,
		.pll_freq = { PLL_6_00_GHZ, PLL_UNUSED, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0000,
		.pll_clk_sel = { 0x0601, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x0012 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0000 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
	{
		/* PLL 0 M0: SATA, PLL 0 M1: Unused, PLL 1: Unused */
		.std0 = STD0_SATA,	.std1 = STD1_NONE,
		.pll_freq = { PLL_6_00_GHZ, PLL_UNUSED, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0000,
		.pll_clk_sel = { 0x0600, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x0012 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0000 },
		.xcvr_diag_hsclk_div = { 0x0013, 0x0001 },
	},
	{
		/* PLL 0 M0: SATA, PLL 0 M1: Unused, PLL 1: DP */
		.std0 = STD0_SATA,	.std1 = STD1_DP,
		.pll_freq = { PLL_6_00_GHZ, PLL_UNUSED, PLL_DP },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x0600, 0x0400, REG_COMPUTED },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x0019 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0001 },
		.xcvr_diag_hsclk_div = { 0x0013, REG_COMPUTED },
	},
	{
		/* PLL 0 M0: SATA, PLL 0 M1: Unused, PLL 1: XAUI */
		.std0 = STD0_SATA,	.std1 = STD1_XAUI,
		.pll_freq = { PLL_6_00_GHZ, PLL_UNUSED, PLL_6_25_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x0600, 0x0400, 0x0600 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x0089 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0011 },
		.xcvr_diag_hsclk_div = { 0x0013, 0x0001 },
	},
	{
		/* PLL 0 M0: SATA, PLL 0 M1: Unused, PLL 1: USXGMII */
		.std0 = STD0_SATA,	.std1 = STD1_USXGMII,
		.pll_freq = { PLL_6_00_GHZ, PLL_UNUSED, PLL_10_31_GHZ },
		.phy_pll_cfg = 0x0002,
		.pll_clk_sel = { 0x0600, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0012, 0x0089 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0011 },
		.xcvr_diag_hsclk_div = { 0x0013, 0x0001 },
	},
	{
		/* PLL 0 M0: USXGMII, PLL 0 M1: Unused, PLL 1: Unused */
		.std0 = STD0_USXGMII,	.std1 = STD1_NONE,
		.pll_freq = { PLL_10_31_GHZ, PLL_UNUSED, PLL_8_00_GHZ },
		.phy_pll_cfg = 0x0000,
		.pll_clk_sel = { 0x0400, 0x0400, 0x0400 },
		.xcvr_diag_plldrc_ctrl = { 0x0001, 0x0012 },
		.xcvr_diag_hsclk_sel = { 0x0000, 0x0000 },
		.xcvr_diag_hsclk_div = { 0x0001, 0x0001 },
	},
};

/*
 * Structure used to store values of PHY registers for voltage-related
 * coefficients, for particular voltage swing and pre-emphasis level. Values
 * are shared across all physical lanes.
 */
struct coefficients {
	/* Value of TX_TXCC_MGNFS_MULT_000 register to use */
	u16 mgnfs_mult;
	/* Value of TX_TXCC_CPOST_MULT_00 register to use */
	u16 cpost_mult;
};

/*
 * Array consists of values of voltage-related registers for sd0801 PHY. A value
 * of 0xFFFF is a placeholder for invalid combination, and will never be used.
 */
static const struct coefficients vltg_coeff[4][4] = {
	/* voltage swing 0, pre-emphasis 0->3 */
	{
		{ .mgnfs_mult = 0x002A, .cpost_mult = 0x0000 },
		{ .mgnfs_mult = 0x001F, .cpost_mult = 0x0014 },
		{ .mgnfs_mult = 0x0012, .cpost_mult = 0x0020 },
		{ .mgnfs_mult = 0x0000, .cpost_mult = 0x002A },
	},

	/* voltage swing 1, pre-emphasis 0->3 */
	{
		{ .mgnfs_mult = 0x001F, .cpost_mult = 0x0000 },
		{ .mgnfs_mult = 0x0013, .cpost_mult = 0x0012 },
		{ .mgnfs_mult = 0x0000, .cpost_mult = 0x001F },
		{ .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF },
	},

	/* voltage swing 2, pre-emphasis 0->3 */
	{
		{ .mgnfs_mult = 0x0013, .cpost_mult = 0x0000 },
		{ .mgnfs_mult = 0x0000, .cpost_mult = 0x0013 },
		{ .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF },
		{ .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF },
	},

	/* voltage swing 3, pre-emphasis 0->3 */
	{
		{ .mgnfs_mult = 0x0000, .cpost_mult = 0x0000 },
		{ .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF },
		{ .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF },
		{ .mgnfs_mult = 0xFFFF, .cpost_mult = 0xFFFF },
	}
};

static void tp_reg_base_init(const struct tp_phy *tp_phy, u32 block_offset,
			     struct tp_reg_base *ctx)
{
	ctx->base = tp_phy->sd_base + block_offset;
	ctx->reg_offset_shift = tp_phy->init_data->reg_offset_shift;
}

static void tp_reg_write(const struct tp_reg_base *ctx, u32 reg, u16 val)
{
	u32 offset = reg << ctx->reg_offset_shift;

	writew(val, ctx->base + offset);
}

static u16 tp_reg_read(const struct tp_reg_base *ctx, u32 reg)
{
	u32 offset = reg << ctx->reg_offset_shift;

	return readw(ctx->base + offset);
}

static void tp_reg_set_bits(const struct tp_reg_base *ctx, u32 reg, u16 bits)
{
	u16 val = tp_reg_read(ctx, reg);

	if ((val & bits) != bits) {
		val |= bits;
		tp_reg_write(ctx, reg, val);
	}
}

static void tp_reg_clear_bits(const struct tp_reg_base *ctx, u32 reg, u16 bits)
{
	u16 val = tp_reg_read(ctx, reg);

	if ((val & bits) != 0) {
		val &= ~bits;
		tp_reg_write(ctx, reg, val);
	}
}

static void tp_reg_field_init(const struct tp_reg_base *ctx, u32 reg, u16 mask,
			      struct tp_reg_field *field)
{
	u32 offset = reg << ctx->reg_offset_shift;

	field->reg_addr = ctx->base + offset;
	field->mask = mask;
}

static u16 tp_reg_field_read(const struct tp_reg_field *field)
{
	u16 val = readw(field->reg_addr);

	val &= field->mask;
	return val >> __ffs(field->mask);
}

static void tp_reg_field_write(const struct tp_reg_field *field, u16 val)
{
	u16 reg_val = readw(field->reg_addr);

	reg_val &= ~field->mask;
	reg_val |= val << __ffs(field->mask);
	writew(reg_val, field->reg_addr);
}

static void tp_reg_dptx_write(const struct tp_phy *tp_phy, u32 reg, u32 val)
{
	writel(val, tp_phy->base + reg);
}

static u32 tp_reg_dptx_read(const struct tp_phy *tp_phy, u32 reg)
{
	return readl(tp_phy->base + reg);
}

static void tp_dp_deassert_phy_reset(const struct tp_phy *tp_phy)
{
	u32 val = tp_reg_dptx_read(tp_phy, PHY_RESET);

	tp_reg_dptx_write(tp_phy, PHY_RESET, val | 0x0100);
}

static const char *tp_get_std_name(const struct tp_std_data *std)
{
	switch (std->type) {
	case STD_TYPE_PCIE:
		return "PCIe";
	case STD_TYPE_USB_TXRX:
		return "USB";
	case STD_TYPE_DP:
		return "DisplayPort";
	case STD_TYPE_XAUI:
		return "XAUI";
	case STD_TYPE_SGMII:
		if (std->flags & STD_FLAG_QSGMII)
			return "QSGMII";
		if (std->flags & STD_FLAG_SGMIIP)
			return "SGMII-2.5";
		return "SGMII";
	case STD_TYPE_SATA:
		return "SATA";
	case STD_TYPE_USXGMII:
		return "USXGMII";
	default:
		return "None";
	}
}

static const char *tp_get_revision_desc(enum tp_revision rev)
{
	switch (rev) {
	case REV_R100: return "100";
	case REV_R110: return "110";
	case REV_R120: return "120";
	case REV_R130: return "130";
	case REV_R140: return "140";
	case REV_R150: return "150";
	case REV_R200: return "200";
	case REV_R210: return "210";
	case REV_R220: return "220";
	default:
		break;
	}
	return "?";
}

static const char *tp_get_tech_desc(u32 tech_tag)
{
	switch (tech_tag) {
	case TECH_TSMC_16: return "TSMC-16";
	case TECH_TSMC_16_0805: return "TSMC-16-0805";
	case TECH_TSMC_12: return "TSMC-12";
	case TECH_TSMC_7: return "TSMC-7";
	case TECH_TSMC_5: return "TSMC-5";
	case TECH_SS_7: return "SS-7";
	case TECH_SS_5: return "SS-5";
	default:
		break;
	}
	return "?";
}

static enum tp_pll_fb_div_mode tp_calc_pcie_pll_fb_div_mode(const struct tp_phy *tp_phy)
{
	u32 tech_tag = tp_phy->tech_tag;
	enum tp_revision rev = tp_phy->rev;
	enum tp_pll_fb_div_mode div_mode;

	if (tech_tag & (TECH_TSMC_16 | TECH_TSMC_12)) {
		if (rev == REV_R100) {
			if (tp_phy->plls[PLL1_MODE0].freq_khz == 0)
				div_mode = PLL_FB_DIV_MODE_FBR;
			else
				div_mode = PLL_FB_DIV_MODE_D32;
		} else {
			if (tp_phy->plls[PLL0_MODE0].ref_clk_khz == 100000U)
				div_mode = PLL_FB_DIV_MODE_FBR;
			else
				div_mode = PLL_FB_DIV_MODE_DR;
		}
	} else if (tech_tag & TECH_TSMC_16_0805) {
		if (tp_phy->plls[PLL0_MODE0].ref_clk_khz == 100000U)
			div_mode = PLL_FB_DIV_MODE_FBR;
		else
			div_mode = PLL_FB_DIV_MODE_DR;
	} else {
		div_mode = PLL_FB_DIV_MODE_DR;
	}

	return div_mode;
}

static void tp_calc_pll_fb_div_modes(struct tp_phy *tp_phy)
{
	enum tp_pll_fb_div_mode div_mode;
	struct tp_std_data *std0 = tp_phy->std_ptr[0];

	if (std0->type == STD_TYPE_PCIE) {
		div_mode = tp_calc_pcie_pll_fb_div_mode(tp_phy);

		tp_phy->plls[PLL0_MODE0].fb_div_mode = div_mode;
		tp_phy->plls[PLL0_MODE1].fb_div_mode = div_mode;

		if (std0->flags & STD_FLAG_PCIE_ML)
			tp_phy->plls[PLL1_MODE0].fb_div_mode = div_mode;
		else
			tp_phy->plls[PLL1_MODE0].fb_div_mode = PLL_FB_DIV_MODE_FBR;
	} else {
		tp_phy->plls[PLL0_MODE0].fb_div_mode = PLL_FB_DIV_MODE_FBR;
		tp_phy->plls[PLL0_MODE1].fb_div_mode = PLL_FB_DIV_MODE_FBR;
		tp_phy->plls[PLL1_MODE0].fb_div_mode = PLL_FB_DIV_MODE_FBR;
	}
}

static u16 tp_get_pll_lock_cnt_subtract(const struct tp_phy *tp_phy,
					const struct tp_pll_data *pll)
{
	bool t = (tp_phy->tech_tag & (TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12)) ||
		 ((tp_phy->tech_tag & TECH_TSMC_7) && (tp_phy->rev <= REV_R110));

	return (pll->ref_clk_khz != 100000U || pll->ssc || !t) ? 1 : 0;
}

static bool tp_is_pll_fractional_mode_required(const struct tp_pll_data *pll)
{
	if (pll->ssc)
		return true;

	return (pll->freq_khz % (pll->ref_clk_khz * 2U)) != 0;
}

static u32 tp_dp_pll_freq_from_bit_rate(u32 bit_rate)
{
	/* provide the PLL frequency in kHz */
	switch (bit_rate) {
	case 1620:
	case 2430:
	case 3240:
		return 9720000;
	case 2160:
	case 4320:
		return 8640000;
	case 2700:
	case 5400:
		return 10800000;
	case 8100:
		return 8100000;
	default:
		break;
	}

	return 0;
}

static u16 tp_dp_pll_clk_sel_from_bit_rate(u32 bit_rate)
{
	/* calculates the CMN_PDIAG_PLL{0,1}_CLK_SEL_M{0,1} register value */
	switch (bit_rate) {
	case 1620:
		return 0x0F01;
	case 3240:
		return 0x0B00;
	case 2160:
	case 2430:
	case 2700:
		return 0x0701;
	case 4320:
	case 5400:
		return 0x0301;
	case 8100:
		return 0x0200;
	default:
		break;
	}

	return 0;
}

static u16 tp_dp_xcvr_diag_hsclk_div_from_bit_rate(u32 bit_rate)
{
	/* calculates the XCVR_DIAG_HSCLK_DIV register value */
	switch (bit_rate) {
	case 1620:
	case 3240:
		return 0x0002;
	case 2160:
	case 2430:
	case 2700:
		return 0x0001;
	case 4320:
	case 5400:
	case 8100:
		return 0x0000;
	default:
		break;
	}

	return 0;
}

static struct tp_std_data *tp_dp_std_from_pll_id(const struct tp_phy *tp_phy,
						 enum tp_pll_id pll_id)
{
	if (pll_id == PLL0_MODE0)
		return tp_phy->std_ptr[0];
	if (pll_id == PLL1_MODE0)
		return tp_phy->std_ptr[1];
	return NULL;
}

static u32 tp_dp_get_default_pll_freq(const struct tp_phy *tp_phy, enum tp_pll_id pll_id)
{
	struct tp_std_data *std;
	u32 freq_khz = 0;

	std = tp_dp_std_from_pll_id(tp_phy, pll_id);
	if (std)
		freq_khz = tp_dp_pll_freq_from_bit_rate(std->max_bit_rate);

	WARN(freq_khz == 0, "failed to get DP PLL frequency");
	return freq_khz;
}

static int tp_init_plls_data(struct tp_phy *tp_phy)
{
	const struct tp_link_cfg *link_cfg = tp_phy->link_cfg;
	struct tp_std_data *std0 = tp_phy->std_ptr[0];
	struct tp_std_data *std1 = tp_phy->std_ptr[1];
	struct device *dev = tp_phy->dev;

	tp_phy->plls[PLL0_MODE0].ref_clk_khz = tp_phy->ref_clk_khz;
	tp_phy->plls[PLL0_MODE1].ref_clk_khz = tp_phy->ref_clk_khz;
	tp_phy->plls[PLL1_MODE0].ref_clk_khz = tp_phy->ref_clk1_khz;

	for (u32 i = 0; i < TP_NB_PLLS; ++i) {
		u32 pll_freq_khz = 0;

		switch (link_cfg->pll_freq[i]) {
		case PLL_DP:
			pll_freq_khz = tp_dp_get_default_pll_freq(tp_phy, i);
			break;
		case PLL_6_00_GHZ:
			pll_freq_khz = 6000000;
			break;
		case PLL_6_25_GHZ:
			pll_freq_khz = 6250000;
			break;
		case PLL_8_00_GHZ:
			pll_freq_khz = 8000000;
			break;
		case PLL_10_00_GHZ:
			pll_freq_khz = 10000000;
			break;
		case PLL_10_31_GHZ:
			pll_freq_khz = 10310000;
			break;
		default:
			break;
		}

		tp_phy->plls[i].freq_khz = pll_freq_khz;
	}

	/* Check if both standards should share the same SSC since sharing PLL0 */
	if (std1 && (std1->flags & STD_FLAG_S_SSC) && std0->ssc_mode != std1->ssc_mode) {
		dev_err(dev, "Both standards must share the same SSC");
		return -EINVAL;
	}

	tp_phy->plls[PLL0_MODE0].ssc = (std0->ssc_mode == CDNS_SERDES_INTERNAL_SSC);
	tp_phy->plls[PLL0_MODE1].ssc = (std0->ssc_mode == CDNS_SERDES_INTERNAL_SSC);
	tp_phy->plls[PLL1_MODE0].ssc = std1 && (std1->ssc_mode == CDNS_SERDES_INTERNAL_SSC);

	tp_calc_pll_fb_div_modes(tp_phy);

	for (u32 i = 0; i < TP_NB_PLLS; ++i) {
		struct tp_pll_data *pll = &tp_phy->plls[i];

		pll->fractional_mode = tp_is_pll_fractional_mode_required(pll);
	}

	return 0;
}

static void tp_write_lanes_reg(const struct tp_std_data *std,
			       const struct tp_reg_base reg_base[MAX_NUM_LANES],
			       u16 reg, u16 val)
{
	for (u32 phy = 0; phy < std->nb_phys; ++phy) {
		u32 num_lanes = std->phys[phy].num_lanes;
		u32 mlane = std->phys[phy].mlane;

		for (u32 i = 0; i < num_lanes; i++)
			tp_reg_write(&reg_base[mlane + i], reg, val);
	}
}

static void tp_write_standards_lanes_reg(const struct tp_phy *tp_phy,
					 const struct tp_reg_base reg_base[MAX_NUM_LANES],
					 u16 reg, u16 val)
{
	for (u32 i = 0; i < tp_phy->nb_standards; ++i)
		tp_write_lanes_reg(tp_phy->std_ptr[i], reg_base, reg, val);
}

static void tp_find_cfg_write_lanes_reg(const struct tp_phy *tp_phy,
					const struct tp_std_data *std,
					const struct tp_reg_base reg_base[MAX_NUM_LANES],
					u16 reg, const struct tp_reg_cfg cfg[], u32 nb_cfg)
{
	union tp_reg_cfg_key key = {
		.std = BIT(std->type),
		.tech = tp_phy->tech_tag,
		.rev = BIT(tp_phy->rev),
	};
	u32 std_flags = std->flags;
	bool found = false;
	u16 val = 0;

	for (u32 i = 0U; i < nb_cfg; ++i) {
		const struct tp_reg_cfg *p_cfg = &cfg[i];

		if (((p_cfg->cfg.key & key.key) == key.key) &&
		    ((p_cfg->std_flags & std_flags) == p_cfg->std_flags)) {
			val = p_cfg->val;
			found = true;
			break;
		}
	}

	if (found)
		tp_write_lanes_reg(std, reg_base, reg, val);
}

static void tp_write_tx_lane_reg(const struct tp_phy *tp_phy, const struct tp_std_data *std,
				 u16 reg, const struct tp_reg_cfg cfg[], u32 nb_cfg)
{
	tp_find_cfg_write_lanes_reg(tp_phy, std, tp_phy->reg_tx_lane_cdb, reg, cfg, nb_cfg);
}

static void tp_write_rx_lane_reg(const struct tp_phy *tp_phy, const struct tp_std_data *std,
				 u16 reg, const struct tp_reg_cfg cfg[], u32 nb_cfg)
{
	tp_find_cfg_write_lanes_reg(tp_phy, std, tp_phy->reg_rx_lane_cdb, reg, cfg, nb_cfg);
}

static void tp_write_tx_txcc_ctrl(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	/* For DP, this is also handled by tp_dp_set_voltages() */
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x2A84,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x2A82,
		},
		{
			.cfg.std = BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x2AA0,
		},
		{
			.cfg.std = BIT(STD_TYPE_DP),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x08A4,
		},
	};

	tp_write_tx_lane_reg(tp_phy, std, TX_TXCC_CTRL, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_tx_txcc_cpost_mult_00(const struct tp_phy *tp_phy,
					   const struct tp_std_data *std)
{
	/* For DP, this is also handled by tp_dp_set_voltages() */
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_16 | TECH_TSMC_12,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x001C,
		},
		{
			.cfg.std = BIT(STD_TYPE_SGMII) | BIT(STD_TYPE_SATA) | BIT(STD_TYPE_DP) |
				   BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0000,
		},
	};

	tp_write_tx_lane_reg(tp_phy, std, TX_TXCC_CPOST_MULT_00, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_tx_txcc_cpost_mult_01(const struct tp_phy *tp_phy,
					   const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0014,
		},
	};

	tp_write_tx_lane_reg(tp_phy, std, TX_TXCC_CPOST_MULT_01, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_tx_txcc_mgnfs_mult_000(const struct tp_phy *tp_phy,
					    const struct tp_std_data *std)
{
	/* For DP, this is also handled by tp_dp_set_voltages() */
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_7,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0007,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_TSMC_7 | TECH_TSMC_5 | TECH_SS_7 | TECH_SS_5,
			.cfg.rev = REV_MSK_R100P,
			.std_flags = STD_FLAG_USB_IO_11V,
			.val = 0x0002,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_TSMC_7 | TECH_TSMC_5 | TECH_SS_7 | TECH_SS_5,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0007,
		},
		{
			.cfg.std = BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0011,
		},
		{
			.cfg.std = BIT(STD_TYPE_DP),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x002A,
		},
	};

	tp_write_tx_lane_reg(tp_phy, std, TX_TXCC_MGNFS_MULT_000, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_tx_txcc_mgnfs_mult_100(const struct tp_phy *tp_phy,
					    const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_7 | TECH_TSMC_5,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0019,
		},
		{
			.cfg.std = BIT(STD_TYPE_SGMII),
			.cfg.tech = TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12,
			.cfg.rev = REV_MSK_R100P,
			.std_flags = STD_FLAG_QSGMII,
			.val = 0x0011,
		},
	};

	tp_write_tx_lane_reg(tp_phy, std, TX_TXCC_MGNFS_MULT_100, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_drv_diag_tx_drv(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	/* For DP, this is also handled by tp_dp_set_voltages() */
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_16 | TECH_TSMC_12,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x00A3,
		},
		{
			.cfg.std = BIT(STD_TYPE_SGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.std_flags = STD_FLAG_QSGMII,
			.val = 0x0003,
		},
		{
			.cfg.std = BIT(STD_TYPE_SGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x00B3,
		},
		{
			.cfg.std = BIT(STD_TYPE_DP),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0003,
		},
	};

	tp_write_tx_lane_reg(tp_phy, std, DRV_DIAG_TX_DRV, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_xcvr_diag_rxclk_ctrl(const struct tp_phy *tp_phy,
					  const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_SGMII) | BIT(STD_TYPE_XAUI),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x4000,
		},
	};

	if (!tp_phy->init_data->rxda_des_clk_2x_en)
		return;

	tp_write_tx_lane_reg(tp_phy, std, XCVR_DIAG_RXCLK_CTRL, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_xcvr_diag_bidi_ctrl(const struct tp_phy *tp_phy,
					 const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_DP),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x000F,
		},
	};

	tp_write_tx_lane_reg(tp_phy, std, XCVR_DIAG_BIDI_CTRL, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_xcvr_diag_psc_ovrd(const struct tp_phy *tp_phy,
					const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0003,
		},
		{
			.cfg.std = BIT(STD_TYPE_SGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0002,
		},
		{
			.cfg.std = BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0004,
		},
		{
			.cfg.std = BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0000,
		},
	};

	tp_write_tx_lane_reg(tp_phy, std, XCVR_DIAG_PSC_OVRD, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_tx_psc_a0(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.std_flags = STD_FLAG_USB_G1,
			.val = 0x02FB,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x02FF,
		},
		{
			.cfg.std = BIT(STD_TYPE_DP) | BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x00FB,
		},
		{
			.cfg.std = BIT(STD_TYPE_XAUI) | BIT(STD_TYPE_SGMII) |
				   BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x00F3,
		},
	};

	tp_write_tx_lane_reg(tp_phy, std, TX_PSC_A0, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_tx_psc_a1(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.std_flags = STD_FLAG_USB_G1,
			.val = 0x06AB,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x06AF,
		},
		{
			.cfg.std = BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x04BB,
		},
	};

	tp_write_tx_lane_reg(tp_phy, std, TX_PSC_A1, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_tx_psc_a2_a3(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.std_flags = STD_FLAG_USB_G1,
			.val = 0x06AA,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x06AE,
		},
		{
			.cfg.std = BIT(STD_TYPE_DP) | BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x04AA,
		},
		{
			.cfg.std = BIT(STD_TYPE_XAUI) | BIT(STD_TYPE_SGMII) |
				   BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x04A2,
		},
	};

	tp_write_tx_lane_reg(tp_phy, std, TX_PSC_A2, cfg, ARRAY_SIZE(cfg));
	tp_write_tx_lane_reg(tp_phy, std, TX_PSC_A3, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_psc_a0(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0D1D,
		},
		{
			.cfg.std = BIT(STD_TYPE_DP),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0000,
		},
		{
			.cfg.std = BIT(STD_TYPE_XAUI),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0D1D,
		},
		{
			.cfg.std = BIT(STD_TYPE_SGMII) | BIT(STD_TYPE_SATA) |
				   BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x091D,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_PSC_A0, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_psc_a1(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0D1D,
		},
		{
			.cfg.std = BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x091D,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_PSC_A1, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_psc_a2(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0D00,
		},
		{
			.cfg.std = BIT(STD_TYPE_DP),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0000,
		},
		{
			.cfg.std = BIT(STD_TYPE_XAUI) | BIT(STD_TYPE_SGMII) |
				   BIT(STD_TYPE_SATA) | BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0900,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_PSC_A2, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_psc_a3(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0500,
		},
		{
			.cfg.std = BIT(STD_TYPE_DP),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0000,
		},
		{
			.cfg.std = BIT(STD_TYPE_XAUI) | BIT(STD_TYPE_SGMII) |
				   BIT(STD_TYPE_SATA) | BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0100,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_PSC_A3, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_psc_cal(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_DP),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0000,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_PSC_CAL, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_cdrlf_cnfg(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_16,
			.cfg.rev = REV_MSK_R130P,
			.val = 0x018E,
		},
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_12 | TECH_TSMC_7 | TECH_TSMC_5,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x010A,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX) | BIT(STD_TYPE_XAUI) |
				   BIT(STD_TYPE_SGMII) | BIT(STD_TYPE_SATA) |
				   BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x018C,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_CDRLF_CNFG, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_cdrlf_cnfg2(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_SS_7 | TECH_SS_5,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0E39,
		},
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_16,
			.cfg.rev = REV_MSK_R130P,
			.val = 0x2E33,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_CDRLF_CNFG2, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_cdrlf_cnfg3(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0003,
		},
		{
			.cfg.std = BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0007,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_CDRLF_CNFG3, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_sigdet_hl_filt_tmr(const struct tp_phy *tp_phy,
					   const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0013,
		},
		{
			.cfg.std = BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0005,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_SIGDET_HL_FILT_TMR, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_ree_gcsm1_ctrl(const struct tp_phy *tp_phy,
				       const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0000,
		},
		{
			.cfg.std = BIT(STD_TYPE_DP),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0000,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_REE_GCSM1_CTRL, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_ree_gcsm1_eqenm_ph1(const struct tp_phy *tp_phy,
					    const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x03E7,
		},
		{
			.cfg.std = BIT(STD_TYPE_XAUI) | BIT(STD_TYPE_SGMII) |
				   BIT(STD_TYPE_SATA) | BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x03C7,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_REE_GCSM1_EQENM_PH1, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_ree_gcsm1_eqenm_ph2(const struct tp_phy *tp_phy,
					    const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x01E7,
		},
		{
			.cfg.std = BIT(STD_TYPE_XAUI) | BIT(STD_TYPE_SGMII) |
				   BIT(STD_TYPE_SATA) | BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x01C7,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_REE_GCSM1_EQENM_PH2, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_ree_gcsm2_ctrl(const struct tp_phy *tp_phy,
				       const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_DP),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0000,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_REE_GCSM2_CTRL, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_ree_pergcsm_ctrl(const struct tp_phy *tp_phy,
					 const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_DP),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0000,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_REE_PERGCSM_CTRL, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_ree_peak_uthr(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_16,
			.cfg.rev = REV_MSK_R130P,
			.val = 0x0008,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_TSMC_16,
			.cfg.rev = REV_MSK_R130P,
			.val = 0x0000,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_TSMC_16_0805,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0000,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_REE_PEAK_UTHR, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_ree_peak_lthr(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_TSMC_16,
			.cfg.rev = REV_MSK_R130P,
			.val = 0x01F5,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_TSMC_16_0805,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x01F5,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_TSMC_5,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x01F2,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_REE_PEAK_LTHR, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_ree_atten_thr(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0C02,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_REE_ATTEN_THR, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_ree_tap_clip(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE) | BIT(STD_TYPE_USB_TXRX) |
				   BIT(STD_TYPE_XAUI) | BIT(STD_TYPE_SGMII) |
				   BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12 |
				    TECH_TSMC_7,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0019,
		},
		{
			.cfg.std = BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0019,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_REE_TAP1_CLIP, cfg, ARRAY_SIZE(cfg));
	tp_write_rx_lane_reg(tp_phy, std, RX_REE_TAP2TON_CLIP, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_ree_smgm_ctrl1(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0330,
		},
		{
			.cfg.std = BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0474,
		},
		{
			.cfg.std = BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0030,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_REE_SMGM_CTRL1, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_ree_smgm_ctrl2(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0300,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_REE_SMGM_CTRL2, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_diag_dfe_ctrl(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_16,
			.cfg.rev = REV_MSK_R100,
			.val = 0x0004,
		},
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_12,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0000,
		},
		{
			.cfg.std = BIT(STD_TYPE_PCIE) | BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_TSMC_7,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0006,
		},
		{
			.cfg.std = BIT(STD_TYPE_XAUI) | BIT(STD_TYPE_SGMII) |
				   BIT(STD_TYPE_SATA) | BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0000,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_DIAG_DFE_CTRL, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_diag_dfe_amp_tune_2(const struct tp_phy *tp_phy,
					    const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_16,
			.cfg.rev = REV_MSK_R130P,
			.val = 0x0C21,
		},
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_SS_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0CA1,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX) | BIT(STD_TYPE_XAUI) |
				   BIT(STD_TYPE_SGMII) | BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0C01,
		},
		{
			.cfg.std = BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12 | TECH_TSMC_7,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0C21,
		},
		{
			.cfg.std = BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_TSMC_5,
			.cfg.rev = REV_MSK_R200P,
			.val = 0x0C01,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_DIAG_DFE_AMP_TUNE_2, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_diag_dfe_amp_tune_3(const struct tp_phy *tp_phy,
					    const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_16,
			.cfg.rev = REV_MSK_R130P,
			.val = 0x0002,
		},
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_5,
			.cfg.rev = REV_MSK_R100,
			.val = 0x4005,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12 |
				    TECH_TSMC_7 | TECH_SS_7 | TECH_SS_5,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0002,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_TSMC_5,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x3113,
		},
		{
			.cfg.std = BIT(STD_TYPE_XAUI) | BIT(STD_TYPE_SGMII) |
				   BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0000,
		},
		{
			.cfg.std = BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12 | TECH_TSMC_7,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0002,
		},
		{
			.cfg.std = BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_TSMC_5,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0022,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_DIAG_DFE_AMP_TUNE_3, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_diag_ree_dac_ctrl(const struct tp_phy *tp_phy,
					  const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_16,
			.cfg.rev = REV_MSK_R130P,
			.val = 0x0005,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_DIAG_REE_DAC_CTRL, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_diag_nqst_ctrl(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x00F9,
		},
		{
			.cfg.std = BIT(STD_TYPE_XAUI) | BIT(STD_TYPE_SGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0098,
		},
		{
			.cfg.std = BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0988,
		},
		{
			.cfg.std = BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x00B9,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_DIAG_NQST_CTRL, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_diag_sigdet_tune(const struct tp_phy *tp_phy,
					 const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_16 | TECH_TSMC_12 | TECH_TSMC_7,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x1002,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12 |
				    TECH_TSMC_7 | TECH_SS_7 | TECH_SS_5,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x1004,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX),
			.cfg.tech = TECH_TSMC_5,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x1003,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_DIAG_SIGDET_TUNE, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_diag_pi_rate(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE),
			.cfg.tech = TECH_TSMC_5,
			.cfg.rev = REV_MSK_R100,
			.val = 0x0310,
		},
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX) | BIT(STD_TYPE_XAUI),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0031,
		},
		{
			.cfg.std = BIT(STD_TYPE_SGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0010,
		},
		{
			.cfg.std = BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0310,
		},
		{
			.cfg.std = BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0033,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_DIAG_PI_RATE, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_diag_pi_cap(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_USB_TXRX) | BIT(STD_TYPE_XAUI) |
				   BIT(STD_TYPE_SGMII) | BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0000,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_DIAG_PI_CAP, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_rx_diag_acya(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	static const struct tp_reg_cfg cfg[] = {
		{
			.cfg.std = BIT(STD_TYPE_PCIE) | BIT(STD_TYPE_USB_TXRX) |
				   BIT(STD_TYPE_XAUI) | BIT(STD_TYPE_SGMII) |
				   BIT(STD_TYPE_SATA),
			.cfg.tech = TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12 |
				    TECH_TSMC_7,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0001,
		},
		{
			.cfg.std = BIT(STD_TYPE_USXGMII),
			.cfg.tech = TECH_ALL,
			.cfg.rev = REV_MSK_R100P,
			.val = 0x0001,
		},
	};

	tp_write_rx_lane_reg(tp_phy, std, RX_DIAG_ACYA, cfg, ARRAY_SIZE(cfg));
}

static void tp_write_phy_pipe_usb3_gen2_cfg(const struct tp_phy *tp_phy,
					    const struct tp_std_data *std)
{
	const struct tp_reg_base *reg_base = &tp_phy->reg_phy_pcs_common_cdb;

	if (std->type != STD_TYPE_USB_TXRX)
		return;

	tp_reg_write(reg_base, PHY_PIPE_USB3_GEN2_PRE_CFG0, 0x0A0A);
	tp_reg_write(reg_base, PHY_PIPE_USB3_GEN2_POST_CFG0, 0x1000);
	tp_reg_write(reg_base, PHY_PIPE_USB3_GEN2_POST_CFG1, 0x0010);
}

static void tp_write_cmn_cdiag_pwri_ovrd(const struct tp_phy *tp_phy,
					 const struct tp_std_data *std)
{
	const struct tp_reg_base *reg_base = &tp_phy->reg_common_cdb;

	if (std->type != STD_TYPE_USB_TXRX)
		return;

	tp_reg_write(reg_base, CMN_CDIAG_CDB_PWRI_OVRD, 0x8200);
	tp_reg_write(reg_base, CMN_CDIAG_XCVRC_PWRI_OVRD, 0x8200);
}

static void tp_write_refclk_rcv_out_en(const struct tp_phy *tp_phy)
{
	bool has_usxgmii = false;

	if (!tp_phy->init_data->usxgmii_refclk_rcv_out_en)
		return;

	/* USXGMII configuration: Enable cmn_refclk_rcv_out_en */
	for (u32 i = 0; i < tp_phy->nb_standards; ++i) {
		struct tp_std_data *std = tp_phy->std_ptr[i];

		if (std->type == STD_TYPE_USXGMII) {
			has_usxgmii = true;
			break;
		}
	}

	if (has_usxgmii)
		tp_reg_write(&tp_phy->reg_phy_pma_common_cdb, PHY_PMA_CMN_CTRL1, 0x0040);
}

static void tp_write_ana_cfg(const struct tp_phy *tp_phy)
{
	const struct tp_reg_base *reg_base = &tp_phy->reg_common_cdb;

	/* TX pull-down and pull-up resistors calibration tune register */
	if (tp_phy->std_ptr[1] && tp_phy->std_ptr[0]->type != tp_phy->std_ptr[1]->type) {
		tp_reg_write(reg_base, CMN_TXPDCAL_TUNE, 0x007F);
		tp_reg_write(reg_base, CMN_TXPUCAL_TUNE, 0x007F);
	}

	/* Bias override register */
	if (tp_phy->tech_tag & TECH_TSMC_7)
		tp_reg_write(reg_base, CMN_DIAG_BIAS_OVRD1, 0x7400);
	else if (tp_phy->tech_tag & TECH_TSMC_5)
		tp_reg_write(reg_base, CMN_DIAG_BIAS_OVRD1, 0x5400);
}

static __always_inline u16 tp_calc_refclk_reg(u32 ref_clk_hz, u32 clk_div_shift, u32 val_ns)
{
	/* Compute ceiling of: refclk(Hz) * val(seconds) / clk_div */
	u32 den = DIV_ROUND_UP(1000000000U, val_ns) << clk_div_shift;

	return DIV_ROUND_UP(ref_clk_hz, den);
}

static u32 tp_calc_refclk_div_shift(u32 ref_clk_hz)
{
	if (ref_clk_hz <= 32000000U)
		return 0;
	if (ref_clk_hz <= 64000000U)
		return 1;
	if (ref_clk_hz <= 128000000U)
		return 2;
	return 3;
}

static u16 tp_calc_pll_lock_tmr(const struct tp_phy *tp_phy, u32 ref_clk_hz, u32 clk_div_shift,
				enum tp_pll_id pll_id)
{
	u32 tech_tag = tp_phy->tech_tag;
	enum tp_pll_id pll_id_m1 = (pll_id == PLL0_MODE0) ? PLL0_MODE1 : PLL1_MODE0;

	if (tech_tag & (TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12)) {
		/* PLL lock delay timer (normal exit time) */
		return tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 8360);
	}

	if (tp_phy->plls[pll_id].fractional_mode || tp_phy->plls[pll_id_m1].fractional_mode) {
		/* PLL lock delay timer (Special case: 7nm fractional mode only) */
		return tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 10000);
	}

	/* PLL lock delay timer (reduced exit time) */
	return tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 1500);
}

static void tp_write_pll_lock_tmr(const struct tp_phy *tp_phy, enum tp_pll_id pll_id)
{
	u32 ref_clk_hz = tp_phy->ref_clk_khz * 1000U;
	u32 clk_div_shift = tp_calc_refclk_div_shift(ref_clk_hz);
	u16 val = tp_calc_pll_lock_tmr(tp_phy, ref_clk_hz, clk_div_shift, pll_id);

	tp_reg_write(&tp_phy->reg_common_cdb, (pll_id == PLL0_MODE0) ?
			CMN_PLLSM0_PLLLOCK_TMR : CMN_PLLSM1_PLLLOCK_TMR, val);
}

static void tp_write_refclk_regs(const struct tp_phy *tp_phy)
{
	const struct tp_reg_base *reg_base = &tp_phy->reg_common_cdb;
	u32 tech_tag = tp_phy->tech_tag;
	u32 clk_div_shift;
	u32 ref_clk_hz;
	u32 tmp;
	u16 val;

	ref_clk_hz = tp_phy->ref_clk_khz * 1000U;

	/* Get the digital reference clock divider value */
	clk_div_shift = tp_calc_refclk_div_shift(ref_clk_hz);

	/* Bias timer value */
	tp_reg_write(reg_base, CMN_SSM_BIAS_TMR,
		     tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 1000));

	/* PLL control state machine register: */
	/* PLL pre timer */
	if ((tech_tag & (TECH_TSMC_7 | TECH_TSMC_5 | TECH_SS_7 | TECH_SS_5)) &&
	    ref_clk_hz == 100000000U) {
		/* PLL pre timer (reduced exit time) */
		val = tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 800);
	} else {
		/* PLL pre timer (normal exit time) */
		val = tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 2000);
	}
	tp_reg_write(reg_base, CMN_PLLSM0_PLLPRE_TMR, val);
	tp_reg_write(reg_base, CMN_PLLSM1_PLLPRE_TMR, val);

	/* PLL lock delay timer */
	val = tp_calc_pll_lock_tmr(tp_phy, ref_clk_hz, clk_div_shift, PLL0_MODE0);
	tp_reg_write(reg_base, CMN_PLLSM0_PLLLOCK_TMR, val);

	val = tp_calc_pll_lock_tmr(tp_phy, ref_clk_hz, clk_div_shift, PLL1_MODE0);
	tp_reg_write(reg_base, CMN_PLLSM1_PLLLOCK_TMR, val);

	/* Bandgap calibration controller register: */
	/* Initialization timer value and Iteration timer value */
	val = tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 5000);
	tp_reg_write(reg_base, CMN_BGCAL_INIT_TMR, val);
	tp_reg_write(reg_base, CMN_BGCAL_ITER_TMR, val);

	/* External bias current calibration register: */
	/* Initialization timer value */
	tp_reg_write(reg_base, CMN_IBCAL_INIT_TMR,
		     tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 1000));

	/* Resistor calibration register: */
	/* TX initialization timer value */
	val = tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 1200);
	tp_reg_write(reg_base, CMN_TXPUCAL_INIT_TMR, val);
	tp_reg_write(reg_base, CMN_TXPDCAL_INIT_TMR, val);

	/* TX iteration timer value */
	val = tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 240);
	tp_reg_write(reg_base, CMN_TXPUCAL_ITER_TMR, val);
	tp_reg_write(reg_base, CMN_TXPDCAL_ITER_TMR, val);

	/* RX initialization timer value */
	tp_reg_write(reg_base, CMN_RXCAL_INIT_TMR,
		     tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 30000));

	/* RX iteration timer value */
	tp_reg_write(reg_base, CMN_RXCAL_ITER_TMR,
		     tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 240));

	/* Signal detect clock calibration register: */
	/* Initialization timer value */
	tp_reg_write(reg_base, CMN_SD_CAL_INIT_TMR,
		     tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 80) + 4);

	/* Iteration timer value */
	tp_reg_write(reg_base, CMN_SD_CAL_ITER_TMR,
		     tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 80));

	/* Reference clock timer */
	val = tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 600) - 1;
	tp_reg_write(reg_base, CMN_SD_CAL_REFTIM_START, val);

	/* PLL clock counter with computation of actual ref timer time based on rounding */
	tmp = (val + 1U) * (500000000U >> 4);
	tmp /= ref_clk_hz >> (4 + clk_div_shift);
	tp_reg_write(reg_base, CMN_SD_CAL_PLLCNT_START, tmp - 1);

	/* Receiver detect calibration register calculations */
	/* Receiver detect time */
	tp_write_standards_lanes_reg(tp_phy, tp_phy->reg_tx_lane_cdb, TX_RCVDET_ST_TMR,
				     tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 100000));

	/* Signal detect calibration register calculations */
	/* Initialization timer value */
	val = tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 1000);
	tp_write_standards_lanes_reg(tp_phy, tp_phy->reg_rx_lane_cdb, RX_SDCAL0_INIT_TMR, val);
	tp_write_standards_lanes_reg(tp_phy, tp_phy->reg_rx_lane_cdb, RX_SDCAL1_INIT_TMR, val);

	/* Iteration timer value */
	val = tp_calc_refclk_reg(ref_clk_hz, clk_div_shift, 5000);
	tp_write_standards_lanes_reg(tp_phy, tp_phy->reg_rx_lane_cdb, RX_SDCAL0_ITER_TMR, val);
	tp_write_standards_lanes_reg(tp_phy, tp_phy->reg_rx_lane_cdb, RX_SDCAL1_ITER_TMR, val);
}

static __always_inline u16 tp_calc_pll_ref_clk_reg(u32 ref_clk_hz, u32 val_ns)
{
	/* Compute ceiling of: refclk(Hz) * val(seconds) */
	u32 den = DIV_ROUND_UP(1000000000U, val_ns);

	return DIV_ROUND_UP(ref_clk_hz, den);
}

static void tp_write_ssc_ctrl(const struct tp_phy *tp_phy, enum tp_pll_id pll_id,
			      u32 fd_div_prescale)
{
	u32 tech_tag = tp_phy->tech_tag;
	const struct tp_reg_base *reg_base = &tp_phy->reg_common_cdb;
	const struct tp_pll_data *pll = &tp_phy->plls[pll_id];
	u32 ref_clk_khz = pll->ref_clk_khz;
	u32 pll_freq_khz = pll->freq_khz;
	u32 ref_clk_hz = ref_clk_khz * 1000U;

	s32 min_delta_ppm = S32_MAX;
	u16 ssc_num_steps = 0;
	u16 ssc_amp_step = 0;
	u16 ssc_step_time = 0;

	/* Calculated spread spectrum values */
	for (u32 steps = 127; steps >= 80; --steps) {
		s32 delta_ppm;
		u32 amp_step;
		s32 ppm;
		u64 num;

		/* Compute step time: ref_clk / (steps * 31.5e3 * 2) */
		u32 step_time = DIV_ROUND_CLOSEST(ref_clk_hz, steps * 63000U);

		/* Compute SSC actual frequency */
		u32 ssc_freq = ref_clk_hz / (steps * 2U * step_time);

		/* Ignore this configuration if not between 30kHz and 33kHz */
		if (ssc_freq < 30000U || ssc_freq > 33000)
			continue;

		/* Compute Amp Step: round(-1 * ideal_ppm * 95% * 0.000001 *
		 *  ((vco_freq / (vco_div / 2 * ref_clk) +2^-17)) * 2^16 / steps)
		 * With ideal_ppm = -5000
		 */
		amp_step = div_u64(311296ULL * pll_freq_khz,
				   (ref_clk_khz << fd_div_prescale) * steps);
		amp_step = DIV_ROUND_CLOSEST(amp_step, 1000U);

		/* Compute Actual SSC PPM: (-1 * steps * amp_step / 2^16) /
		 *  ((vco_freq / (vco_div / 2 * ref_clk)) + 2^-17) * 1000000
		 * The resulting value is multiplied by -512 (for better precision)
		 */
		num = steps * amp_step * 15625U;
		num *= ref_clk_khz << fd_div_prescale;
		ppm = div_u64(num, pll_freq_khz * 2U);

		/* Exit if the error is less than 0.5 PPM */
		delta_ppm = abs(ppm - (4750 * 512));
		if (delta_ppm <= 256) {
			ssc_num_steps = steps;
			ssc_amp_step = amp_step;
			ssc_step_time = step_time;
			break;
		}

		/* Otherwise look for the smallest error */
		if (min_delta_ppm > delta_ppm) {
			ssc_num_steps = steps;
			ssc_amp_step = amp_step;
			ssc_step_time = step_time;
			min_delta_ppm = delta_ppm;
		}
	}

	/* PLL spread spectrum control register 1 */
	tp_reg_write(reg_base, CMN_PLLx_SS_CTRL1_Mx(pll_id), 1U);

	/* PLL spread spectrum control register 2 */
	if ((tech_tag & (TECH_TSMC_12 | TECH_TSMC_16 | TECH_TSMC_16_0805)) &&
	    pll->ref_clk_khz == 24000) {
		if (pll_freq_khz == 10000000)
			ssc_amp_step = 0x0416;
		else if (pll_freq_khz == 10800000)
			ssc_amp_step = 0x044F;
		else if (pll_freq_khz == 9720000)
			ssc_amp_step = 0x0401;
		else if (pll_freq_khz == 8640000)
			ssc_amp_step = 0x038F;
		else if (pll_freq_khz == 8100000)
			ssc_amp_step = 0x0342;
	} else if ((tech_tag & TECH_TSMC_7) && pll->ref_clk_khz == 24000 &&
		   pll_freq_khz == 8000000) {
		ssc_amp_step = 0x03C7;
	} else if ((tech_tag & TECH_TSMC_5) && pll->ref_clk_khz == 24000) {
		if (pll_freq_khz == 8000000)
			ssc_amp_step = 0x03C7;
		else if (pll_freq_khz == 10800000)
			ssc_amp_step = 0x03DF;
		else if (pll_freq_khz == 8100000)
			ssc_amp_step = 0x03CA;
		else if (pll_freq_khz == 9720000)
			ssc_amp_step = 0x03A2;
	}

	tp_reg_write(reg_base, CMN_PLLx_SS_CTRL2_Mx(pll_id), ssc_amp_step);

	/* PLL spread spectrum control register 3 */
	if ((tech_tag & (TECH_TSMC_12 | TECH_TSMC_16 | TECH_TSMC_16_0805)) &&
	    pll->ref_clk_khz == 24000) {
		if (pll_freq_khz == 9720000)
			ssc_num_steps = 0x007B;
		else if (pll_freq_khz == 8640000)
			ssc_num_steps = 0x007B;
	}

	tp_reg_write(reg_base, CMN_PLLx_SS_CTRL3_Mx(pll_id), ssc_num_steps);

	/* PLL spread spectrum control register 4 */
	tp_reg_write(reg_base, CMN_PLLx_SS_CTRL4_Mx(pll_id), ssc_step_time);
}

static void tp_write_pll_ctrl_reg(const struct tp_phy *tp_phy, enum tp_pll_id pll_id,
				  u32 fd_div_prescale)
{
	u32 tech_tag = tp_phy->tech_tag;
	u32 rev_msk = BIT(tp_phy->rev);
	const struct tp_pll_data *pll = &tp_phy->plls[pll_id];
	u32 pll_freq_khz = pll->freq_khz;
	u16 bits_12_15;
	u16 bit_8;
	u16 bit_5;
	u16 bit_4;
	u16 bits_1_0;
	u16 val;

	/* PLL control register */
	/* Compute bits 15:12 */
	if (tech_tag & (TECH_TSMC_5 | TECH_SS_7 | TECH_SS_5)) {
		bits_12_15 = 1;
	} else if (pll->fb_div_mode == PLL_FB_DIV_MODE_FBR) {
		if (tech_tag & (TECH_TSMC_16 | TECH_TSMC_16_0805)) {
			if (!pll->fractional_mode || pll_freq_khz < 8000000)
				bits_12_15 = 1;
			else
				bits_12_15 = 0;
		} else {
			if (!pll->fractional_mode)
				bits_12_15 = 1;
			else if (pll_freq_khz >= 8000000)
				bits_12_15 = 0;
			else if (pll_freq_khz >= 6000000)
				bits_12_15 = 3;
			else if (pll_freq_khz >= 5500000)
				bits_12_15 = 2;
			else if (pll_freq_khz >= 4750000)
				bits_12_15 = 1;
			else
				bits_12_15 = 0;
		}
	} else if (pll->fb_div_mode == PLL_FB_DIV_MODE_D32) {
		bits_12_15 = 0;
	} else {
		/* PLL_FB_DIV_MODE_DR */
		if (((tech_tag & TECH_TSMC_7) && (rev_msk & (REV_MSK_R100 | REV_MSK_R110))) ||
		    pll->ref_clk_khz != 100000)
			bits_12_15 = 0;
		else
			bits_12_15 = 1;
	}

	/* Compute bit 8: PLL VCO mode select: LC tank (0) / ring oscillator (1) */
	bit_8 = (pll_freq_khz >= 8000000) ? 0 : 1;

	/*
	 * Compute bit 5: PLL feedback divider mode select: FBR (0) /
	 * D32 (1) / DR (1)
	 */
	bit_5 = (pll->fb_div_mode == PLL_FB_DIV_MODE_FBR) ? 0 : 1;

	/* Compute bit 4: PLL feedback divider prescale */
	bit_4 = fd_div_prescale;

	/* Compute bit 1:0 */
	if (pll->fb_div_mode != PLL_FB_DIV_MODE_DR || pll->ref_clk_khz != 100000)
		bits_1_0 = 2;
	else if (tech_tag & (TECH_SS_5 | TECH_SS_7))
		bits_1_0 = 1;
	else
		bits_1_0 = 0;

	val = bits_1_0 | (bit_4 << 4) | (bit_5 << 5) | (bit_8 << 8) | (bits_12_15 << 12);

	tp_reg_write(&tp_phy->reg_common_cdb, CMN_PDIAG_PLLx_CTRL_Mx(pll_id), val);
}

static void tp_write_pll_cp_padj(const struct tp_phy *tp_phy, enum tp_pll_id pll_id)
{
	u32 tech_tag = tp_phy->tech_tag;
	u32 rev_msk = BIT(tp_phy->rev);
	const struct tp_pll_data *pll = &tp_phy->plls[pll_id];
	u16 val;

	/* PLL charge pump proportional path adjust register */
	if (pll->freq_khz < 8000000) {
		val = 0x0B17;
	} else if (pll->fractional_mode) {
		if (tech_tag & TECH_TSMC_7)
			val = 0x0B17;
		else if (tech_tag & (TECH_SS_5 | TECH_SS_7))
			val = 0x0309;
		else
			val = 0x0509; /* TSMC_5, TSMC_12, TSMC_16, TSMC_16_0805 */
	} else if (pll->freq_khz >= 10000000) {
		if (tech_tag & TECH_TSMC_7)
			val = (rev_msk & (REV_MSK_R100 | REV_MSK_R110)) ? 0x0023 : 0x0035;
		else if (tech_tag & TECH_TSMC_5)
			val = (pll_id == PLL0_MODE0) ? 0x002D : 0x0035;
		else if (tech_tag & (TECH_SS_5 | TECH_SS_7))
			val = 0x013E;
		else
			val = 0x0028; /* TSMC_12, TSMC_16, TSMC_16_0805 */
	} else {
		if (tech_tag & TECH_TSMC_16)
			val = (rev_msk & REV_MSK_R100) ? 0x0028 : 0x0128;
		else if (tech_tag & TECH_TSMC_16_0805)
			val = 0x0128;
		else if (tech_tag & TECH_TSMC_12)
			val = 0x0028;
		else if (tech_tag & TECH_TSMC_7)
			val = (rev_msk & (REV_MSK_R100 | REV_R110)) ? 0x001D : 0x0031;
		else if (tech_tag & TECH_TSMC_5)
			val = (pll_id == PLL0_MODE1) ? 0x0012 : 0x0031;
		else
			val = 0x0131; /* SS_7, SS_5 */
	}

	tp_reg_write(&tp_phy->reg_common_cdb, CMN_PDIAG_PLLx_CP_PADJ_Mx(pll_id), val);
}

static void tp_write_pll_cp_iadj(const struct tp_phy *tp_phy, enum tp_pll_id pll_id)
{
	u32 tech_tag = tp_phy->tech_tag;
	u32 rev_msk = BIT(tp_phy->rev);
	const struct tp_pll_data *pll = &tp_phy->plls[pll_id];
	u16 val;

	/* PLL charge pump integral path adjust register */
	if (pll->freq_khz < 8000000) {
		val = 0x0E01;
	} else if (pll->fractional_mode) {
		if (tech_tag & (TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12))
			val = 0x0F00;
		else if (tech_tag & (TECH_TSMC_7 | TECH_TSMC_5))
			val = 0x0E01;
		else /* SS_7, SS_5 */
			val = 0x0E02;
	} else {
		if (tech_tag & (TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12))
			val = 0x0808;
		else if (tech_tag & TECH_TSMC_7)
			val = (rev_msk & (REV_MSK_R100 | REV_MSK_R110)) ? 0x0F0D : 0x0F07;
		else if (tech_tag & TECH_TSMC_5)
			val = 0x0F07;
		else /* SS_7, SS_5 */
			val = 0x0F0D;
	}

	tp_reg_write(&tp_phy->reg_common_cdb, CMN_PDIAG_PLLx_CP_IADJ_Mx(pll_id), val);
}

static void tp_write_pll_filt_padj(const struct tp_phy *tp_phy, enum tp_pll_id pll_id)
{
	u32 tech_tag = tp_phy->tech_tag;
	u32 rev_msk = BIT(tp_phy->rev);
	const struct tp_pll_data *pll = &tp_phy->plls[pll_id];
	u16 val;

	/* PLL proportional path filter adjust register */
	if (pll->freq_khz < 8000000) {
		val = 0x0D05;
	} else if (pll->fractional_mode) {
		if (tech_tag & (TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12))
			val = 0x0F08;
		else if (tech_tag & (TECH_TSMC_7 | TECH_TSMC_5))
			val = 0x0D05;
		else /* SS_7, SS_5 */
			val = 0x0D07;
	} else {
		if (tech_tag & (TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12))
			val = 0x0000;
		else if (tech_tag & TECH_TSMC_7)
			val = ((pll_id == PLL0_MODE1) && (rev_msk & REV_MSK_R200P)) ?
				0x0001 : 0x0100;
		else if (tech_tag & TECH_TSMC_5)
			val = (pll_id == PLL0_MODE1) ? 0x0001 : 0x0100;
		else /* TECH_TSMC_7, TECH_TSMC_5, SS_7, SS_5 */
			val = 0x0100;
	}

	tp_reg_write(&tp_phy->reg_common_cdb, CMN_PDIAG_PLLx_FILT_PADJ_Mx(pll_id), val);
}

static int tp_write_pll_clk_sel(const struct tp_phy *tp_phy, enum tp_pll_id pll_id)
{
	const struct tp_link_cfg *link_cfg = tp_phy->link_cfg;
	u16 val = 0;

	if (link_cfg->pll_freq[pll_id] == PLL_DP) {
		struct tp_std_data *std = tp_dp_std_from_pll_id(tp_phy, pll_id);

		if (std)
			val = tp_dp_pll_clk_sel_from_bit_rate(std->max_bit_rate);
		if (val == 0)
			return -EINVAL;
	} else {
		val = link_cfg->pll_clk_sel[pll_id];
		if (WARN_ON(val == REG_COMPUTED))
			return -EINVAL;
	}

	tp_reg_write(&tp_phy->reg_common_cdb, CMN_PDIAG_PLLx_CLK_SEL_Mx(pll_id), val);
	return 0;
}

static void tp_write_xcvr_diag_hsclk_sel(const struct tp_phy *tp_phy, u32 std_idx)
{
	struct tp_std_data *std = tp_phy->std_ptr[std_idx];
	u16 val = tp_phy->link_cfg->xcvr_diag_hsclk_sel[std_idx];

	tp_write_lanes_reg(std, tp_phy->reg_tx_lane_cdb, XCVR_DIAG_HSCLK_SEL, val);
}

static int tp_write_xcvr_diag_hsclk_div(const struct tp_phy *tp_phy, u32 std_idx)
{
	u16 val;
	struct tp_std_data *std = tp_phy->std_ptr[std_idx];

	if (std->type == STD_TYPE_DP) {
		val = tp_dp_xcvr_diag_hsclk_div_from_bit_rate(std->max_bit_rate);
		if (val == 0)
			return -EINVAL;
	} else {
		val = tp_phy->link_cfg->xcvr_diag_hsclk_div[std_idx];
	}

	tp_write_lanes_reg(std, tp_phy->reg_tx_lane_cdb, XCVR_DIAG_HSCLK_DIV, val);
	return 0;
}

static void tp_write_xcvr_diag_plldrc_ctrl(const struct tp_phy *tp_phy, u32 std_idx)
{
	struct tp_std_data *std = tp_phy->std_ptr[std_idx];
	u16 val = tp_phy->link_cfg->xcvr_diag_plldrc_ctrl[std_idx];

	tp_write_lanes_reg(std, tp_phy->reg_tx_lane_cdb, XCVR_DIAG_PLLDRC_CTRL, val);
}

static u32 tp_calc_pll_fb_div_clk_khz(const struct tp_pll_data *pll)
{
	switch (pll->fb_div_mode) {
	case PLL_FB_DIV_MODE_FBR:
		return pll->ref_clk_khz;
	case PLL_FB_DIV_MODE_D32:
		return pll->freq_khz / 32U;
	default:
		return 500000;
	}
}

static void tp_write_pll_vco_regs(const struct tp_phy *tp_phy, enum tp_pll_id pll_id)
{
	const struct tp_reg_base *base = &tp_phy->reg_common_cdb;
	struct tp_std_data *std0 = tp_phy->std_ptr[0];
	const struct tp_pll_data *pll = &tp_phy->plls[pll_id];
	u32 ref_clk_hz = pll->ref_clk_khz * 1000U;
	u32 tech_tag = tp_phy->tech_tag;
	u32 rev_msk = BIT(tp_phy->rev);
	u16 vcocal_reftim_start;
	u16 vco_cnt_subtract;
	u32 fb_div_clk_khz;
	u16 val;

	/* Check if PCIE_SL is used with a reference clock of 100 MHz */
	bool pcie_sl_with_100mhz = (ref_clk_hz == 100000000U) && (std0->type == STD_TYPE_PCIE) &&
				   ((std0->flags & STD_FLAG_PCIE_ML) == 0);

	/* VCO calibration initial time scale control */
	if ((tech_tag & (TECH_TSMC_16 | TECH_TSMC_16_0805 | TECH_TSMC_12 | TECH_TSMC_7)) &&
	    !pcie_sl_with_100mhz)
		tp_reg_write(base, CMN_PLLx_VCOCAL_TCTRL(pll_id), 3);

	/* VCO calibration initialization time */
	tp_reg_write(base, CMN_PLLx_VCOCAL_INIT_TMR(pll_id),
		     tp_calc_pll_ref_clk_reg(ref_clk_hz, 10000));

	/* VCO calibration iteration time */
	tp_reg_write(base, CMN_PLLx_VCOCAL_ITER_TMR(pll_id),
		     tp_calc_pll_ref_clk_reg(ref_clk_hz, 160));

	/* VCO cal ref timer start */
	vco_cnt_subtract = (pll->ref_clk_khz != 100000U) || pll->ssc;
	vcocal_reftim_start = tp_calc_pll_ref_clk_reg(ref_clk_hz, 31670) - vco_cnt_subtract;

	tp_reg_write(base, CMN_PLLx_VCOCAL_REFTIM_START(pll_id), vcocal_reftim_start);

	/* Feedback divider clock period: Compute a frequency in kHz */
	fb_div_clk_khz = tp_calc_pll_fb_div_clk_khz(pll);

	/* VCO cal PLL clock counter start */
	if (!pll->ssc && pcie_sl_with_100mhz &&
	    ((tech_tag & (TECH_TSMC_5 | TECH_SS_5 | TECH_SS_7)) ||
	    ((tech_tag & TECH_TSMC_7) && (rev_msk & REV_MSK_R200P)))) {
		/* Special condition */
		val = 0x3DDA;
	} else if (pll->ssc) {
		/* SSC enabled:
		 * [(vcocal_reftim_start + 1) / ref_clk] / [1.0025 / fb_div_clk]
		 */
		u64 num = (vcocal_reftim_start + 1U) * fb_div_clk_khz;

		val = DIV_U64_ROUND_CLOSEST(num * 10000U, 10025U * pll->ref_clk_khz) - 1U;
	} else {
		/* SSC disabled:
		 * [(vcocal_reftim_start + 1) / ref_clk] / fb_div_clk
		 */
		u32 num = (vcocal_reftim_start + 1U) * fb_div_clk_khz;

		val = DIV_ROUND_CLOSEST(num, pll->ref_clk_khz) - 1U;
	}
	tp_reg_write(base, CMN_PLLx_VCOCAL_PLLCNT_START(pll_id), val);
}

static void tp_write_pll_lock_regs(const struct tp_phy *tp_phy, enum tp_pll_id pll_id)
{
	const struct tp_reg_base *base = &tp_phy->reg_common_cdb;
	const struct tp_pll_data *pll = &tp_phy->plls[pll_id];
	u32 ref_clk_hz = pll->ref_clk_khz * 1000U;
	u16 lock_refcnt_start;
	u16 lock_pllcnt_start;
	u16 lock_cnt_subtract;
	u32 fb_div_clk_khz;
	u16 val;

	lock_cnt_subtract = tp_get_pll_lock_cnt_subtract(tp_phy, pll);

	/* Lock reference clock counter start (ideal / programmed) */
	if (pll->fb_div_mode == PLL_FB_DIV_MODE_FBR) {
		if (ref_clk_hz >= 100000000U)
			lock_refcnt_start = tp_calc_pll_ref_clk_reg(ref_clk_hz, 2000);
		else
			lock_refcnt_start = tp_calc_pll_ref_clk_reg(ref_clk_hz, 8000);
	} else if (pll->fb_div_mode == PLL_FB_DIV_MODE_D32) {
		lock_refcnt_start = tp_calc_pll_ref_clk_reg(ref_clk_hz, 1000);
	} else {
		lock_refcnt_start = tp_calc_pll_ref_clk_reg(ref_clk_hz, 800);
	}

	val = lock_refcnt_start - lock_cnt_subtract;
	tp_reg_write(base, CMN_PLLx_LOCK_REFCNT_START(pll_id), val);

	/* Feedback divider clock period: Compute a frequency in kHz */
	fb_div_clk_khz = tp_calc_pll_fb_div_clk_khz(pll);

	/* Lock PLL feedback divider clock count start */
	if (pll->ssc) {
		u64 num;

		/* SSC enabled:
		 * [lock_refcnt_start / ref_clk] / [1.0025 / fb_div_clk]
		 */
		num = fb_div_clk_khz * lock_refcnt_start;
		lock_pllcnt_start = DIV_U64_ROUND_CLOSEST(num * 10000U,
							  10025U * pll->ref_clk_khz);
	} else {
		/* SSC disabled:
		 * [lock_refcnt_start / ref_clk] / [1.0 / fb_div_clk]
		 */
		lock_pllcnt_start = DIV_ROUND_CLOSEST(lock_refcnt_start * fb_div_clk_khz,
						      pll->ref_clk_khz);
	}
	val = lock_pllcnt_start - lock_cnt_subtract;
	tp_reg_write(base, CMN_PLLx_LOCK_PLLCNT_START(pll_id), val);

	/* Special case */
	if (pll->fb_div_mode == PLL_FB_DIV_MODE_FBR && !pll->ssc) {
		val = 3;
	} else {
		u32 err_in_ref_time = 0;
		u32 err_rel_ref_clk;
		u32 actual_cnt;
		u64 num;

		/* Error in reference time, in per 10000 */
		if (pll->ssc)
			err_in_ref_time = 25U - (lock_refcnt_start * 781250U) / ref_clk_hz;

		/* Actual PLL feedback divider count time multiplied by reference clock and
		 * by 10000: lock_pllcnt_start * ref_clk * 1e4 / fb_div_clk
		 */
		num = pll->ref_clk_khz * lock_pllcnt_start;
		actual_cnt = div_u64(num * (pll->ssc ? 10025U : 10000U), fb_div_clk_khz);

		/* Error in the PLL counter relative to the reference clock time, in per 10000 */
		err_rel_ref_clk = abs(10000U * lock_refcnt_start - actual_cnt)
				/ lock_refcnt_start;

		val = DIV_ROUND_UP((err_in_ref_time + err_rel_ref_clk) *
				   lock_pllcnt_start, 10000U) + 4U;
	}
	tp_reg_write(base, CMN_PLLx_LOCK_PLLCNT_THR(pll_id), val);
}

static void tp_write_pll_fd_div_regs(const struct tp_phy *tp_phy, enum tp_pll_id pll_id,
				     u32 div_ratio)
{
	const struct tp_reg_base *base = &tp_phy->reg_common_cdb;
	const struct tp_pll_data *pll = &tp_phy->plls[pll_id];
	u32 tech_tag = tp_phy->tech_tag;
	u16 high_div;
	u16 low_div;

	/* In fractional mode, we use default/reset value */
	if (pll->fractional_mode)
		return;

	if (pll_id == PLL0_MODE0 && pll->ref_clk_khz == 100000 && pll->freq_khz == 10000000) {
		/* 100 MHz refclk, 10 GHz PLL */
		if (tech_tag & (TECH_TSMC_12 | TECH_TSMC_16 | TECH_TSMC_16_0805)) {
			high_div = 30;
			low_div = 12;
		} else {
			high_div = 36;
			low_div = 6;
		}
	} else if (pll->ref_clk_khz == 100000 && pll->freq_khz == 8000000) {
		/* 100 MHz refclk, 8 GHz PLL */
		if (tech_tag & (TECH_TSMC_12 | TECH_TSMC_16 | TECH_TSMC_16_0805)) {
			high_div = 16;
			low_div = 16;
		} else {
			high_div = 28;
			low_div = 4;
		}
	} else {
		/* Generic calculations */

		/* PLL high integer divider val: EVEN(div_ratio_2 * 67/100) - 4 */
		high_div = DIV_ROUND_UP((div_ratio >> 1) * 67U, 100U);
		high_div -= (high_div & 1U) ? 3U : 4U;

		/* PLL low integer divider val: div_ratio_2 - 8 - high_div */
		low_div = (div_ratio >> 1) - 8 - high_div;
	}

	tp_reg_write(base, CMN_PLLx_DSM_FBH_OVRD_Mx(pll_id), high_div);
	tp_reg_write(base, CMN_PLLx_DSM_FBL_OVRD_Mx(pll_id), low_div);
}

static void tp_write_pll_itrim_reg(const struct tp_phy *tp_phy, enum tp_pll_id pll_id)
{
	const struct tp_reg_base *base = &tp_phy->reg_common_cdb;
	const struct tp_pll_data *pll = &tp_phy->plls[pll_id];

	if (tp_phy->tech_tag & TECH_TSMC_ALL) {
		u16 val = (pll->freq_khz >= 10000000) ? 0x003F : 0x007F;

		tp_reg_write(base, CMN_PDIAG_PLLx_ITRIM_Mx(pll_id), val);
	}
}

static void tp_write_pll_regs(const struct tp_phy *tp_phy, enum tp_pll_id pll_id)
{
	const struct tp_reg_base *base = &tp_phy->reg_common_cdb;
	const struct tp_pll_data *pll = &tp_phy->plls[pll_id];
	u32 fd_div_prescale;
	u32 div_ratio;
	u16 fracdivl;
	u16 val;

	if (pll_id != PLL0_MODE1)
		tp_write_pll_vco_regs(tp_phy, pll_id);

	/* Divide ratio */
	div_ratio = pll->freq_khz / pll->ref_clk_khz;

	/* PLL feedback divider prescale */
	fd_div_prescale = ((!pll->fractional_mode) || (div_ratio > 512)) ? 1 : 0;

	/* PLL integer divider value */
	val = (pll->fractional_mode) ? (div_ratio >> fd_div_prescale) : 0;
	tp_reg_write(base, CMN_PLLx_INTDIV_Mx(pll_id), val);

	/* PLL fractional divider value */
	if (pll->fractional_mode) {
		u32 den = pll->ref_clk_khz << fd_div_prescale;
		u32 rem = pll->freq_khz % den;

		fracdivl = DIV_ROUND_CLOSEST(rem << 16, den);
	} else {
		fracdivl = 0;
	}

	tp_reg_write(base, CMN_PLLx_FRACDIVL_Mx(pll_id), fracdivl);
	tp_reg_write(base, CMN_PLLx_FRACDIVH_Mx(pll_id), pll->fractional_mode ? 2 : 0);

	/* PLL high threshold value */
	if (pll->fractional_mode) {
		/* EVEN(CEIL((fracdivl/65536 + (div_ratio >> fd_div_prescale)) * 2 / 3)) */
		u32 num = fracdivl + (div_ratio << (16 - fd_div_prescale));

		val = DIV_ROUND_UP(num, 3 << 15);
		if (val & 1)
			val++;
	} else {
		val = 0;
	}

	tp_reg_write(base, CMN_PLLx_HIGH_THR_Mx(pll_id), val);

	/* PLL delta sigma modulator diagnostics register */
	tp_reg_write(base, CMN_PLLx_DSM_DIAG_Mx(pll_id),
		     pll->fractional_mode ? 0x0004 : 0x4004);

	/* PLL high and low integer divider value */
	tp_write_pll_fd_div_regs(tp_phy, pll_id, div_ratio);

	if (pll->ssc)
		tp_write_ssc_ctrl(tp_phy, pll_id, fd_div_prescale);

	if (pll_id != PLL0_MODE1)
		tp_write_pll_lock_regs(tp_phy, pll_id);

	tp_write_pll_ctrl_reg(tp_phy, pll_id, fd_div_prescale);
	tp_write_pll_itrim_reg(tp_phy, pll_id);
	tp_write_pll_cp_padj(tp_phy, pll_id);
	tp_write_pll_cp_iadj(tp_phy, pll_id);
	tp_write_pll_filt_padj(tp_phy, pll_id);
}

static int tp_configure_links(struct tp_phy *tp_phy)
{
	const struct tp_link_cfg *link_cfg = tp_phy->link_cfg;
	int ret;

	ret = tp_init_plls_data(tp_phy);
	if (ret)
		return ret;

	tp_reg_field_write(&tp_phy->phy_pll_cfg, link_cfg->phy_pll_cfg);

	for (u32 i = 0; i < TP_NB_PLLS; ++i) {
		ret = tp_write_pll_clk_sel(tp_phy, i);
		if (ret)
			return ret;
	}

	for (u32 i = 0; i < tp_phy->nb_standards; ++i) {
		tp_write_xcvr_diag_hsclk_sel(tp_phy, i);
		tp_write_xcvr_diag_hsclk_div(tp_phy, i);
		tp_write_xcvr_diag_plldrc_ctrl(tp_phy, i);
	}

	tp_write_refclk_rcv_out_en(tp_phy);
	tp_write_refclk_regs(tp_phy);
	tp_write_ana_cfg(tp_phy);

	for (u32 i = 0; i < TP_NB_PLLS; ++i) {
		if (tp_phy->plls[i].freq_khz)
			tp_write_pll_regs(tp_phy, i);
	}

	for (u32 i = 0; i < tp_phy->nb_standards; ++i) {
		struct tp_std_data *std = tp_phy->std_ptr[i];

		tp_write_phy_pipe_usb3_gen2_cfg(tp_phy, std);
		tp_write_cmn_cdiag_pwri_ovrd(tp_phy, std);

		tp_write_tx_txcc_ctrl(tp_phy, std);
		tp_write_tx_txcc_cpost_mult_00(tp_phy, std);
		tp_write_tx_txcc_cpost_mult_01(tp_phy, std);
		tp_write_tx_txcc_mgnfs_mult_000(tp_phy, std);
		tp_write_tx_txcc_mgnfs_mult_100(tp_phy, std);
		tp_write_drv_diag_tx_drv(tp_phy, std);
		tp_write_xcvr_diag_rxclk_ctrl(tp_phy, std);
		tp_write_xcvr_diag_bidi_ctrl(tp_phy, std);
		tp_write_xcvr_diag_psc_ovrd(tp_phy, std);
		tp_write_tx_psc_a0(tp_phy, std);
		tp_write_tx_psc_a1(tp_phy, std);
		tp_write_tx_psc_a2_a3(tp_phy, std);

		tp_write_rx_psc_a0(tp_phy, std);
		tp_write_rx_psc_a1(tp_phy, std);
		tp_write_rx_psc_a2(tp_phy, std);
		tp_write_rx_psc_a3(tp_phy, std);
		tp_write_rx_psc_cal(tp_phy, std);
		tp_write_rx_cdrlf_cnfg(tp_phy, std);
		tp_write_rx_cdrlf_cnfg2(tp_phy, std);
		tp_write_rx_cdrlf_cnfg3(tp_phy, std);
		tp_write_rx_sigdet_hl_filt_tmr(tp_phy, std);
		tp_write_rx_ree_gcsm1_ctrl(tp_phy, std);
		tp_write_rx_ree_gcsm1_eqenm_ph1(tp_phy, std);
		tp_write_rx_ree_gcsm1_eqenm_ph2(tp_phy, std);
		tp_write_rx_ree_gcsm2_ctrl(tp_phy, std);
		tp_write_rx_ree_pergcsm_ctrl(tp_phy, std);
		tp_write_rx_ree_peak_uthr(tp_phy, std);
		tp_write_rx_ree_peak_lthr(tp_phy, std);
		tp_write_rx_ree_atten_thr(tp_phy, std);
		tp_write_rx_ree_tap_clip(tp_phy, std);
		tp_write_rx_ree_smgm_ctrl1(tp_phy, std);
		tp_write_rx_ree_smgm_ctrl2(tp_phy, std);
		tp_write_rx_diag_dfe_ctrl(tp_phy, std);
		tp_write_rx_diag_dfe_amp_tune_2(tp_phy, std);
		tp_write_rx_diag_dfe_amp_tune_3(tp_phy, std);
		tp_write_rx_diag_ree_dac_ctrl(tp_phy, std);
		tp_write_rx_diag_nqst_ctrl(tp_phy, std);
		tp_write_rx_diag_sigdet_tune(tp_phy, std);
		tp_write_rx_diag_pi_rate(tp_phy, std);
		tp_write_rx_diag_pi_cap(tp_phy, std);
		tp_write_rx_diag_acya(tp_phy, std);
	}

	return 0;
}

static int tp_get_std_idx(const struct tp_phy *tp_phy, const struct tp_std_data *std)
{
	if (std == tp_phy->std_ptr[0])
		return 0;

	if (std == tp_phy->std_ptr[1])
		return 1;

	return -EINVAL;
}

static int tp_dp_update_pll_freq(struct tp_phy *tp_phy, const struct tp_std_data *std,
				 enum tp_pll_id pll_id, u32 bit_rate, bool ssc)
{
	const struct tp_link_cfg *link_cfg = tp_phy->link_cfg;
	struct tp_pll_data *pll = &tp_phy->plls[pll_id];
	u32 pll_freq_khz = 0;
	u16 val;

	if (link_cfg->pll_freq[pll_id] != PLL_DP)
		return -EINVAL;

	pll_freq_khz = tp_dp_pll_freq_from_bit_rate(bit_rate);
	if (pll_freq_khz == 0)
		return -EINVAL;

	pll->freq_khz = pll_freq_khz;
	pll->ssc = ssc;
	pll->fractional_mode = tp_is_pll_fractional_mode_required(pll);

	val = tp_dp_pll_clk_sel_from_bit_rate(bit_rate);
	tp_reg_write(&tp_phy->reg_common_cdb, CMN_PDIAG_PLLx_CLK_SEL_Mx(pll_id), val);

	val = tp_dp_xcvr_diag_hsclk_div_from_bit_rate(bit_rate);
	tp_write_lanes_reg(std, tp_phy->reg_tx_lane_cdb, XCVR_DIAG_HSCLK_DIV, val);

	tp_write_pll_lock_tmr(tp_phy, pll_id);
	tp_write_pll_regs(tp_phy, pll_id);

	return 0;
}

/*
 * Perform register operations related to setting link rate, once powerstate is
 * set and PLL disable request was processed.
 */
static int tp_dp_configure_rate(struct tp_phy *tp_phy, const struct tp_instance *inst,
				const struct phy_configure_opts_dp *dp)
{
	struct tp_reg_base *reg_base = &tp_phy->reg_phy_pma_common_cdb;
	u16 pll_mask;
	int std_idx;
	u16 val;
	int ret;

	/* Find associated PLL with the DP */
	std_idx = tp_get_std_idx(tp_phy, inst->std);
	if (std_idx < 0)
		return std_idx;

	pll_mask = 1 << std_idx;

	/*
	 * Disable the associated PLL (cmn_pll0_en or cmn_pll1_en) before
	 * re-programming the new data rate.
	 */
	tp_reg_clear_bits(reg_base, PHY_PMA_PLL_RAW_CTRL, pll_mask);

	/*
	 * Wait for PLL ready de-assertion.
	 * For PLL0 - PHY_PMA_CMN_CTRL2[2] == 1
	 * For PLL1 - PHY_PMA_CMN_CTRL2[3] == 1
	 */
	ret = read_poll_timeout(tp_reg_read, val, (val & (pll_mask << 2)) != 0, 0,
				POLL_TIMEOUT_US, false, reg_base, PHY_PMA_CMN_CTRL2);
	if (ret)
		return ret;

	ndelay(200);

	/* Reconfigure registers associated with link rate */
	return tp_dp_update_pll_freq(tp_phy, inst->std,
				     (std_idx == 0) ? PLL0_MODE0 : PLL1_MODE0,
				     dp->link_rate, dp->ssc);
}

static int tp_dp_enable_pll(const struct tp_phy *tp_phy, const struct tp_instance *inst)
{
	const struct tp_reg_base *reg_base = &tp_phy->reg_phy_pma_common_cdb;
	u16 pll_mask;
	int std_idx;
	u16 val;

	std_idx = tp_get_std_idx(tp_phy, inst->std);
	if (std_idx < 0)
		return std_idx;

	pll_mask = 1 << std_idx;

	/* Enable the associated PLL (cmn_pll0_en or cmn_pll1_en) */
	tp_reg_set_bits(reg_base, PHY_PMA_PLL_RAW_CTRL, pll_mask);

	/*
	 * Wait for PLL ready assertion.
	 * For PLL0 - PHY_PMA_CMN_CTRL2[0] == 1
	 * For PLL1 - PHY_PMA_CMN_CTRL2[1] == 1
	 */
	return read_poll_timeout(tp_reg_read, val, (val & pll_mask) != 0, 0, POLL_TIMEOUT_US,
				 false, reg_base, PHY_PMA_CMN_CTRL2);
}

/* Verify, that parameters to configure PHY with are correct. */
static int tp_dp_verify_config(const struct tp_instance *inst,
			       const struct phy_configure_opts_dp *dp)
{
	u32 num_lanes;

	/* If changing link rate was required, verify it's supported. */
	if (dp->set_rate) {
		if (tp_dp_pll_freq_from_bit_rate(dp->link_rate) == 0)
			return -EINVAL;
	}

	/* Verify lane count. */
	if (dp->set_lanes) {
		switch (dp->lanes) {
		case 1:
		case 2:
		case 4:
			/* valid lane count. */
			break;
		default:
			return -EINVAL;
		}

		/* Check against actual number of PHY's lanes. */
		if (dp->lanes > inst->max_lanes)
			return -EINVAL;

		num_lanes = dp->lanes;
	} else {
		num_lanes = inst->num_lanes;
	}

	/* If changing voltages is required, check swing and pre-emphasis levels, per-lane. */
	if (dp->set_voltages) {
		/* Lane count verified previously. */
		for (u32 i = 0; i < num_lanes; i++) {
			if (dp->voltage[i] > 3 || dp->pre[i] > 3)
				return -EINVAL;

			/* Sum of voltage swing and pre-emphasis levels cannot exceed 3. */
			if (dp->voltage[i] + dp->pre[i] > 3)
				return -EINVAL;
		}
	}

	return 0;
}

/*
 * Enable or disable PLL for selected lanes.
 */
static int tp_dptx_set_pll_en(const struct tp_phy *tp_phy, u32 num_lanes, bool enable)
{
	void __iomem *addr;
	u32 pll_ack_val;
	u32 rd_val;
	int ret;

	/* Used to determine which bits to check for or enable in PHY_PMA_XCVR_PLLCLK_EN reg */
	u32 pll_bits;
	/* Used to enable or disable lanes. */
	u32 pll_val;

	/* Select reg value and mask, depending on enabled lane count. */
	pll_val = tp_reg_dptx_read(tp_phy, PHY_PMA_XCVR_PLLCLK_EN);

	if (enable) {
		pll_bits = ((1 << num_lanes) - 1);
		pll_val |= pll_bits;
		pll_ack_val = pll_bits;
	} else {
		pll_bits = ((1 << num_lanes) - 1);
		pll_val &= (~pll_bits);
		pll_ack_val = 0;
	}

	tp_reg_dptx_write(tp_phy, PHY_PMA_XCVR_PLLCLK_EN, pll_val);

	/* Wait for acknowledgment from PHY. */
	addr = tp_phy->base + PHY_PMA_XCVR_PLLCLK_EN_ACK;
	ret = readl_poll_timeout(addr, rd_val, (rd_val & pll_bits) == pll_ack_val, 0,
				 POLL_TIMEOUT_US);
	if (ret < 0) {
		dev_err(tp_phy->dev, "timeout waiting for link PLL clock enable ack\n");
		return ret;
	}

	ndelay(100);
	return 0;
}

static int tp_dptx_set_power_state(const struct tp_phy *tp_phy, u32 mlane, u32 num_lanes,
				   enum phy_powerstate powerstate)
{
	void __iomem *addr;
	u32 value_part;
	u32 value = 0;
	u32 mask = 0;
	u32 read_val;
	int ret;

	switch (powerstate) {
	case POWERSTATE_A0:
		value_part = 0x01U;
		break;
	case POWERSTATE_A2:
		value_part = 0x04U;
		break;
	case POWERSTATE_A3:
	default:
		value_part = 0x08U;
		break;
	}

	/* Select values of registers and mask, depending on enabled lane count. */
	for (u32 i = 0; i < num_lanes; i++) {
		value |= value_part << PHY_POWER_STATE_LN(mlane + i);
		mask |= PMA_XCVR_POWER_STATE_REQ_LN_MASK << PHY_POWER_STATE_LN(mlane + i);
	}

	/* Do not got to power state A3 if already off */
	if (powerstate == POWERSTATE_A3) {
		read_val = tp_reg_dptx_read(tp_phy, PHY_PMA_XCVR_POWER_STATE_ACK);
		if ((read_val & mask) == 0)
			return 0;
	}

	/* Set power state A<n>. */
	tp_reg_dptx_write(tp_phy, PHY_PMA_XCVR_POWER_STATE_REQ, value);

	/* Wait, until PHY acknowledges power state completion. */
	addr = tp_phy->base + PHY_PMA_XCVR_POWER_STATE_ACK;
	ret = readl_poll_timeout(addr, read_val, (read_val & mask) == value, 0,
				 POLL_TIMEOUT_US);
	if (ret)
		return ret;

	tp_reg_dptx_write(tp_phy, PHY_PMA_XCVR_POWER_STATE_REQ, 0x00000000);
	ndelay(100);

	return ret;
}

static int tp_dptx_wait_pma_cmn_ready(const struct tp_phy *tp_phy)
{
	void __iomem *addr;
	unsigned int reg;
	int ret;

	addr = tp_phy->base + PHY_PMA_CMN_READY;
	ret = readl_poll_timeout(addr, reg, reg & 1, 0, POLL_TIMEOUT_US);
	if (ret < 0) {
		dev_err(tp_phy->dev, "timeout waiting for PMA common ready\n");
		return ret;
	}

	return 0;
}

static int tp_dp_run(const struct tp_phy *tp_phy, const struct tp_instance *inst)
{
	int ret;

	ret = tp_dp_enable_pll(tp_phy, inst);
	if (ret)
		return ret;

	ret = tp_dptx_wait_pma_cmn_ready(tp_phy);
	if (ret)
		return ret;

	ndelay(100);
	ret = tp_dptx_set_pll_en(tp_phy, inst->num_lanes, true);
	if (ret)
		return ret;

	ndelay(100);
	ret = tp_dptx_set_power_state(tp_phy, inst->mlane, inst->num_lanes, POWERSTATE_A2);
	if (ret)
		return ret;

	ret = tp_dptx_set_power_state(tp_phy, inst->mlane, inst->num_lanes, POWERSTATE_A0);
	if (ret)
		return ret;

	ndelay(900);
	return 0;
}

static void tp_dp_set_lanes(const struct tp_phy *tp_phy, struct tp_instance *inst, u32 num_lanes)
{
	u32 pma_tx_elec_idle_mask;
	u8 used_lane_mask;
	u8 all_lane_mask;
	u32 pll_clk_en;
	u32 pwr_state;
	u32 value;
	u32 mlane = inst->mlane;

	used_lane_mask = ((1U << num_lanes) - 1) << mlane;
	all_lane_mask = ((1U << inst->max_lanes) - 1) << mlane;

	value = tp_reg_dptx_read(tp_phy, PHY_RESET);

	/* clear pma_tx_elec_idle_ln_* bits. */
	pma_tx_elec_idle_mask = ((1U << inst->max_lanes) - 1) << mlane;
	pma_tx_elec_idle_mask <<= PMA_TX_ELEC_IDLE_SHIFT;

	value &= ~pma_tx_elec_idle_mask;

	/* Assert pma_tx_elec_idle_ln_* for disabled lanes. */
	value |= ((~used_lane_mask) << PMA_TX_ELEC_IDLE_SHIFT) & pma_tx_elec_idle_mask;

	tp_reg_dptx_write(tp_phy, PHY_RESET, value);

	/* reset the link by asserting master lane phy_l0*_reset_n low */
	value &= ~(1U << mlane);
	tp_reg_dptx_write(tp_phy, PHY_RESET, value);

	/*
	 * Assert lane reset on unused lanes and master lane so they remain in
	 * reset and powered down when re-enabling the link
	 */
	value &= ~all_lane_mask;
	value |= used_lane_mask;
	value &= ~(1U << mlane);
	tp_reg_dptx_write(tp_phy, PHY_RESET, value);

	/* Clear requested power state set PLL clock enable to 0 on all used lanes. */
	pwr_state = tp_reg_dptx_read(tp_phy, PHY_PMA_XCVR_POWER_STATE_REQ);
	pll_clk_en = tp_reg_dptx_read(tp_phy, PHY_PMA_XCVR_PLLCLK_EN);

	for (u32 i = 0; i < num_lanes; i++)
		pwr_state &= ~(PMA_XCVR_POWER_STATE_REQ_LN_MASK << PHY_POWER_STATE_LN(mlane + i));

	pll_clk_en &= ~used_lane_mask;

	tp_reg_dptx_write(tp_phy, PHY_PMA_XCVR_POWER_STATE_REQ, pwr_state);
	tp_reg_dptx_write(tp_phy, PHY_PMA_XCVR_PLLCLK_EN, pll_clk_en);

	/* release master lane reset */
	value |= 1 << mlane;
	tp_reg_dptx_write(tp_phy, PHY_RESET, value);

	/* update the number of used lanes */
	inst->num_lanes = num_lanes;
}

/* Configure link rate as required. */
static int tp_dp_set_rate(struct tp_phy *tp_phy, const struct tp_instance *inst,
			  const struct phy_configure_opts_dp *dp)
{
	int ret;

	ret = tp_dptx_set_power_state(tp_phy, inst->mlane, inst->num_lanes, POWERSTATE_A3);
	if (ret)
		return ret;

	ret = tp_dptx_set_pll_en(tp_phy, inst->num_lanes, false);
	if (ret)
		return ret;

	ndelay(200);
	ret = tp_dp_configure_rate(tp_phy, inst, dp);
	if (ret)
		return ret;

	ndelay(200);
	return ret;
}

/* Configure voltage swing and pre-emphasis for all enabled lanes. */
static void tp_dp_set_voltages(const struct tp_phy *tp_phy, const struct tp_instance *inst,
			       const struct phy_configure_opts_dp *dp)
{
	for (u32 lane = 0; lane < inst->num_lanes; lane++) {
		const struct tp_reg_base *reg_base;
		u16 val;

		reg_base = &tp_phy->reg_tx_lane_cdb[inst->mlane + lane];

		/*
		 * Write 1 to register bit TX_DIAG_ACYA[0] to freeze the
		 * current state of the analog TX driver.
		 */
		tp_reg_set_bits(reg_base, TX_DIAG_ACYA, TX_DIAG_ACYA_HBDC_MASK);

		tp_reg_write(reg_base, TX_TXCC_CTRL, 0x08A4);
		tp_reg_write(reg_base, DRV_DIAG_TX_DRV, 0x0003);
		val = vltg_coeff[dp->voltage[lane]][dp->pre[lane]].mgnfs_mult;
		tp_reg_write(reg_base, TX_TXCC_MGNFS_MULT_000, val);
		val = vltg_coeff[dp->voltage[lane]][dp->pre[lane]].cpost_mult;
		tp_reg_write(reg_base, TX_TXCC_CPOST_MULT_00, val);

		/*
		 * Write 0 to register bit TX_DIAG_ACYA[0] to allow the state of
		 * analog TX driver to reflect the new programmed one.
		 */
		tp_reg_clear_bits(reg_base, TX_DIAG_ACYA, TX_DIAG_ACYA_HBDC_MASK);
	}
}

static int tp_phy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct tp_phy *tp_phy = dev_get_drvdata(phy->dev.parent);
	struct tp_instance *inst = phy_get_drvdata(phy);
	bool run_needed = false;
	int ret;

	guard(mutex)(&tp_phy->lock);

	if (tp_phy->already_configured)
		return 0;

	if (inst->std->type != STD_TYPE_DP)
		return -EINVAL;

	ret = tp_dp_verify_config(inst, &opts->dp);
	if (ret) {
		dev_err(&phy->dev, "invalid params for phy configure\n");
		return ret;
	}

	if (opts->dp.set_lanes && opts->dp.lanes != inst->num_lanes) {
		tp_dp_set_lanes(tp_phy, inst, opts->dp.lanes);
		run_needed = true;
	}

	if (opts->dp.set_rate) {
		ret = tp_dp_set_rate(tp_phy, inst, &opts->dp);
		if (ret) {
			dev_err(&phy->dev, "Set rate failed\n");
			return ret;
		}
		run_needed = true;
	}

	if (opts->dp.set_voltages)
		tp_dp_set_voltages(tp_phy, inst, &opts->dp);

	if (phy->power_count && run_needed)
		ret = tp_dp_run(tp_phy, inst);

	return ret;
}

static int tp_phy_on(struct phy *phy)
{
	struct tp_instance *inst = phy_get_drvdata(phy);
	struct tp_phy *tp_phy = dev_get_drvdata(phy->dev.parent);
	struct tp_std_data *std = inst->std;
	u16 val;
	int ret;

	guard(mutex)(&tp_phy->lock);

	if (tp_phy->already_configured) {
		/* Give 5ms to 10ms delay for the PIPE clock to be stable */
		usleep_range(5000, 10000);
		return 0;
	}

	if (tp_phy->nb_phy_on == 0) {
		/* Take the PHY out of reset */
		ret = reset_control_deassert(tp_phy->phy_rst);
		if (ret)
			return ret;
	}

	/* Take the PHY lane group out of reset */
	ret = reset_control_deassert(inst->lnk_rst);
	if (ret)
		return ret;

	/*
	 * Wait for cmn_ready assertion
	 * PHY_PMA_CMN_CTRL1[0] == 1
	 */
	ret = read_poll_timeout(tp_reg_read, val, val & 1, 1000, PLL_LOCK_TIMEOUT, false,
				&tp_phy->reg_phy_pma_common_cdb, PHY_PMA_CMN_CTRL1);
	if (ret) {
		dev_err(tp_phy->dev, "Timeout waiting for CMN ready\n");
		return ret;
	}

	if (std->type == STD_TYPE_DP) {
		tp_dp_run(tp_phy, inst);
	} else if (std->type == STD_TYPE_PCIE || std->type == STD_TYPE_USB_TXRX) {
		struct tp_reg_base *reg_base;

		reg_base = &tp_phy->reg_phy_pcs_lane_cdb[inst->mlane];
		ret = read_poll_timeout(tp_reg_read, val, (val & 2) == 0, 1000,
					PLL_LOCK_TIMEOUT, false, reg_base,
					PHY_PCS_ISO_LINK_CTRL);
		if (ret < 0) {
			dev_err(tp_phy->dev, "Timeout waiting for PHY status ready\n");
			return ret;
		}
	}

	tp_phy->nb_phy_on++;
	return 0;
}

static int tp_phy_off(struct phy *phy)
{
	struct tp_instance *inst = phy_get_drvdata(phy);
	struct tp_phy *tp_phy = dev_get_drvdata(phy->dev.parent);
	int ret;

	guard(mutex)(&tp_phy->lock);

	ret = reset_control_assert(inst->lnk_rst);
	if (ret)
		return ret;

	if (tp_phy->nb_phy_on == 1) {
		ret = reset_control_assert(tp_phy->phy_rst);
		if (ret)
			return ret;
	}

	tp_phy->nb_phy_on--;
	return 0;
}

static int tp_dp_phy_init(const struct tp_phy *tp_phy, struct tp_instance *inst)
{
	 /* enable AUX */
	tp_reg_dptx_write(tp_phy, PHY_AUX_CTRL, 0x0003);

	/* configure the number of lanes */
	tp_dp_set_lanes(tp_phy, inst, inst->num_lanes);

	/* take out of reset */
	tp_dp_deassert_phy_reset(tp_phy);

	return 0;
}

static int tp_phy_init(struct phy *phy)
{
	struct tp_phy *tp_phy = dev_get_drvdata(phy->dev.parent);
	struct tp_instance *inst = phy_get_drvdata(phy);
	int ret = 0;

	guard(mutex)(&tp_phy->lock);

	if (tp_phy->already_configured)
		return 0;

	if (inst->std->type == STD_TYPE_DP)
		ret = tp_dp_phy_init(tp_phy, inst);

	return ret;
}

static const struct phy_ops cdns_torrent_phy_ops = {
	.init		= tp_phy_init,
	.configure	= tp_phy_configure,
	.power_on	= tp_phy_on,
	.power_off	= tp_phy_off,
	.owner		= THIS_MODULE,
};

static int tp_derived_refclk_enable(struct clk_hw *hw)
{
	struct tp_phy *tp_phy = tp_from_derived_refclk(hw);

	tp_reg_field_write(&tp_phy->cmn_cdiag_refclk_ovrd_4, 1);
	tp_reg_field_write(&tp_phy->phy_pipe_cmn_ctrl1_0, 1);

	return 0;
}

static void tp_derived_refclk_disable(struct clk_hw *hw)
{
	struct tp_phy *tp_phy = tp_from_derived_refclk(hw);

	tp_reg_field_write(&tp_phy->phy_pipe_cmn_ctrl1_0, 0);
	tp_reg_field_write(&tp_phy->cmn_cdiag_refclk_ovrd_4, 0);
}

static int tp_derived_refclk_is_enabled(struct clk_hw *hw)
{
	struct tp_phy *tp_phy = tp_from_derived_refclk(hw);
	u16 val;

	val = tp_reg_field_read(&tp_phy->cmn_cdiag_refclk_ovrd_4);
	return !!val;
}

static const struct clk_ops tp_derived_refclk_ops = {
	.enable = tp_derived_refclk_enable,
	.disable = tp_derived_refclk_disable,
	.is_enabled = tp_derived_refclk_is_enabled,
};

static int tp_derived_refclk_register(struct tp_phy *tp_phy)
{
	struct clk_init_data init_data = { .flags = 0 };
	struct device *dev = tp_phy->dev;
	const char *parent_name;
	char clk_name[100];
	struct clk_hw *hw;
	struct clk *clk;
	int ret;

	snprintf(clk_name, sizeof(clk_name), "%s_%s", dev_name(dev),
		 tp_clk_names[CDNS_TORRENT_DERIVED_REFCLK]);

	clk = devm_clk_get_optional(dev, "phy_en_refclk");
	if (IS_ERR(clk)) {
		dev_err(dev, "No parent clock for derived_refclk\n");
		return PTR_ERR(clk);
	}

	if (clk) {
		parent_name = __clk_get_name(clk);
		init_data.parent_names = &parent_name;
		init_data.num_parents = 1;
	}
	init_data.ops = &tp_derived_refclk_ops;
	init_data.name = clk_name;

	hw = &tp_phy->derived_refclk;
	hw->init = &init_data;
	ret = devm_clk_hw_register(dev, hw);
	if (ret)
		return ret;

	tp_phy->clk_hw_data->hws[CDNS_TORRENT_DERIVED_REFCLK] = hw;

	return 0;
}

static int tp_received_refclk_enable(struct clk_hw *hw)
{
	struct tp_phy *tp_phy = tp_from_received_refclk(hw);

	tp_reg_field_write(&tp_phy->phy_pipe_cmn_ctrl1_0, 1);
	return 0;
}

static void tp_received_refclk_disable(struct clk_hw *hw)
{
	struct tp_phy *tp_phy = tp_from_received_refclk(hw);

	tp_reg_field_write(&tp_phy->phy_pipe_cmn_ctrl1_0, 0);
}

static int tp_received_refclk_is_enabled(struct clk_hw *hw)
{
	struct tp_phy *tp_phy = tp_from_received_refclk(hw);
	u16 val, cmn_val;

	val = tp_reg_field_read(&tp_phy->phy_pipe_cmn_ctrl1_0);
	cmn_val = tp_reg_field_read(&tp_phy->cmn_cdiag_refclk_ovrd_4);

	return val && !cmn_val;
}

static const struct clk_ops cdns_torrent_received_refclk_ops = {
	.enable = tp_received_refclk_enable,
	.disable = tp_received_refclk_disable,
	.is_enabled = tp_received_refclk_is_enabled,
};

static int tp_received_refclk_register(struct tp_phy *tp_phy)
{
	struct clk_init_data init_data = { .flags = 0 };
	struct device *dev = tp_phy->dev;
	const char *parent_name;
	char clk_name[100];
	struct clk_hw *hw;
	struct clk *clk;
	int ret;

	snprintf(clk_name, sizeof(clk_name), "%s_%s", dev_name(dev),
		 tp_clk_names[CDNS_TORRENT_RECEIVED_REFCLK]);

	clk = devm_clk_get_optional(dev, "phy_en_refclk");
	if (IS_ERR(clk)) {
		dev_err(dev, "No parent clock for received_refclk\n");
		return PTR_ERR(clk);
	}

	if (clk) {
		parent_name = __clk_get_name(clk);
		init_data.parent_names = &parent_name;
		init_data.num_parents = 1;
	}
	init_data.ops = &cdns_torrent_received_refclk_ops;
	init_data.name = clk_name;

	hw = &tp_phy->received_refclk;
	hw->init = &init_data;
	ret = devm_clk_hw_register(dev, hw);
	if (ret)
		return ret;

	tp_phy->clk_hw_data->hws[CDNS_TORRENT_RECEIVED_REFCLK] = hw;

	return 0;
}

static int tp_refclk_driver_enable(struct clk_hw *hw)
{
	struct tp_refclk_driver *refclk = tp_to_refclk_driver(hw);

	tp_reg_field_write(&refclk->fields[CMN_CDIAG_REFCLK_DRV0_CTRL_6], 0);
	tp_reg_field_write(&refclk->fields[CMN_CDIAG_REFCLK_DRV0_CTRL_5], 1);
	tp_reg_field_write(&refclk->fields[CMN_CDIAG_REFCLK_DRV0_CTRL_1], 0);

	return 0;
}

static void tp_refclk_driver_disable(struct clk_hw *hw)
{
	struct tp_refclk_driver *refclk = tp_to_refclk_driver(hw);

	tp_reg_field_write(&refclk->fields[CMN_CDIAG_REFCLK_DRV0_CTRL_1], 1);
}

static int tp_refclk_driver_is_enabled(struct clk_hw *hw)
{
	struct tp_refclk_driver *refclk = tp_to_refclk_driver(hw);
	u16 val;

	val = tp_reg_field_read(&refclk->fields[CMN_CDIAG_REFCLK_DRV0_CTRL_1]);
	return !val;
}

static u8 tp_refclk_driver_get_parent(struct clk_hw *hw)
{
	struct tp_refclk_driver *refclk = tp_to_refclk_driver(hw);
	u16 val;

	val = tp_reg_field_read(&refclk->fields[CMN_CDIAG_REFCLK_DRV0_CTRL_4]);
	return clk_mux_val_to_index(hw, tp_refclk_driver_mux_table, 0, val);
}

static int tp_refclk_driver_set_parent(struct clk_hw *hw, u8 index)
{
	struct tp_refclk_driver *refclk = tp_to_refclk_driver(hw);
	u16 val;

	if (index >= ARRAY_SIZE(tp_refclk_driver_mux_table))
		return -EINVAL;

	val = tp_refclk_driver_mux_table[index];
	tp_reg_field_write(&refclk->fields[CMN_CDIAG_REFCLK_DRV0_CTRL_4], val);
	return 0;
}

static const struct clk_ops tp_refclk_driver_ops = {
	.enable = tp_refclk_driver_enable,
	.disable = tp_refclk_driver_disable,
	.is_enabled = tp_refclk_driver_is_enabled,
	.determine_rate = __clk_mux_determine_rate,
	.set_parent = tp_refclk_driver_set_parent,
	.get_parent = tp_refclk_driver_get_parent,
};

static int tp_refclk_driver_register(struct tp_phy *tp_phy)
{
	struct tp_refclk_driver *refclk_drv;
	struct clk_init_data init_data = { .flags = 0 };
	struct device *dev = tp_phy->dev;
	const char *parent_names[ARRAY_SIZE(tp_refclk_driver_parent_idx)];
	u32 num_parents;
	char clk_name[100];
	struct clk_hw *hw;
	int ret;

	num_parents = ARRAY_SIZE(tp_refclk_driver_parent_idx);
	for (u32 i = 0; i < num_parents; ++i) {
		hw = tp_phy->clk_hw_data->hws[tp_refclk_driver_parent_idx[i]];
		if (IS_ERR_OR_NULL(hw)) {
			dev_err(dev, "No parent clock for refclk driver clock\n");
			return IS_ERR(hw) ? PTR_ERR(hw) : -ENOENT;
		}
		parent_names[i] = clk_hw_get_name(hw);
	}

	snprintf(clk_name, sizeof(clk_name), "%s_%s", dev_name(dev),
		 tp_clk_names[CDNS_TORRENT_REFCLK_DRIVER]);

	init_data.ops = &tp_refclk_driver_ops;
	init_data.flags = CLK_SET_RATE_NO_REPARENT;
	init_data.parent_names = parent_names;
	init_data.num_parents = num_parents;
	init_data.name = clk_name;

	refclk_drv = &tp_phy->refclk_driver;

	/* Enable Derived reference clock as default */
	tp_reg_field_write(&refclk_drv->fields[CMN_CDIAG_REFCLK_DRV0_CTRL_4], 1);

	refclk_drv->hw.init = &init_data;

	hw = &refclk_drv->hw;
	ret = devm_clk_hw_register(dev, hw);
	if (ret)
		return ret;

	tp_phy->clk_hw_data->hws[CDNS_TORRENT_REFCLK_DRIVER] = hw;

	return 0;
}

static void tp_reg_fields_init(struct tp_phy *tp_phy)
{
	tp_reg_field_init(&tp_phy->reg_phy_pcs_common_cdb, PHY_PLL_CFG, GENMASK_U16(1, 0),
			  &tp_phy->phy_pll_cfg);

	tp_reg_field_init(&tp_phy->reg_phy_pcs_common_cdb, PHY_PIPE_CMN_CTRL1,
			  GENMASK_U16(0, 0), &tp_phy->phy_pipe_cmn_ctrl1_0);

	tp_reg_field_init(&tp_phy->reg_common_cdb, CMN_CDIAG_REFCLK_OVRD, GENMASK_U16(4, 4),
			  &tp_phy->cmn_cdiag_refclk_ovrd_4);

	tp_reg_field_init(&tp_phy->reg_common_cdb, CMN_CDIAG_REFCLK_DRV0_CTRL,
			  GENMASK_U16(1, 1),
			  &tp_phy->refclk_driver.fields[CMN_CDIAG_REFCLK_DRV0_CTRL_1]);

	tp_reg_field_init(&tp_phy->reg_common_cdb, CMN_CDIAG_REFCLK_DRV0_CTRL,
			  GENMASK_U16(4, 4),
			  &tp_phy->refclk_driver.fields[CMN_CDIAG_REFCLK_DRV0_CTRL_4]);

	tp_reg_field_init(&tp_phy->reg_common_cdb, CMN_CDIAG_REFCLK_DRV0_CTRL,
			  GENMASK_U16(5, 5),
			  &tp_phy->refclk_driver.fields[CMN_CDIAG_REFCLK_DRV0_CTRL_5]);

	tp_reg_field_init(&tp_phy->reg_common_cdb, CMN_CDIAG_REFCLK_DRV0_CTRL,
			  GENMASK_U16(6, 6),
			  &tp_phy->refclk_driver.fields[CMN_CDIAG_REFCLK_DRV0_CTRL_6]);
}

static void tp_reg_bases_init(struct tp_phy *tp_phy)
{
	u32 block_offset;
	u8 block_shift = tp_phy->init_data->block_offset_shift;
	u8 reg_shift = tp_phy->init_data->reg_offset_shift;

	for (u32 i = 0; i < MAX_NUM_LANES; i++) {
		block_offset = TORRENT_TX_LANE_CDB_OFFSET(i, block_shift, reg_shift);
		tp_reg_base_init(tp_phy, block_offset, &tp_phy->reg_tx_lane_cdb[i]);

		block_offset = TORRENT_RX_LANE_CDB_OFFSET(i, block_shift, reg_shift);
		tp_reg_base_init(tp_phy, block_offset, &tp_phy->reg_rx_lane_cdb[i]);

		block_offset = TORRENT_PHY_PCS_LANE_CDB_OFFSET(i, block_shift, reg_shift);
		tp_reg_base_init(tp_phy, block_offset, &tp_phy->reg_phy_pcs_lane_cdb[i]);
	}

	tp_reg_base_init(tp_phy, TORRENT_COMMON_CDB_OFFSET, &tp_phy->reg_common_cdb);

	block_offset = TORRENT_PHY_PCS_COMMON_OFFSET(block_shift);
	tp_reg_base_init(tp_phy, block_offset, &tp_phy->reg_phy_pcs_common_cdb);

	block_offset = TORRENT_PHY_PMA_COMMON_OFFSET(block_shift);
	tp_reg_base_init(tp_phy, block_offset, &tp_phy->reg_phy_pma_common_cdb);
}

static void tp_clk_cleanup(const struct tp_phy *tp_phy)
{
	struct device *dev = tp_phy->dev;

	of_clk_del_provider(dev->of_node);
}

static int tp_clk_register(struct tp_phy *tp_phy)
{
	struct device *dev = tp_phy->dev;
	struct device_node *node = dev->of_node;
	struct clk_hw_onecell_data *data;
	int ret;

	data = devm_kzalloc(dev, struct_size(data, hws, CDNS_TORRENT_OUTPUT_CLOCKS),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->num = CDNS_TORRENT_OUTPUT_CLOCKS;
	tp_phy->clk_hw_data = data;

	ret = tp_derived_refclk_register(tp_phy);
	if (ret) {
		dev_err(dev, "failed to register derived refclk\n");
		return ret;
	}

	ret = tp_received_refclk_register(tp_phy);
	if (ret) {
		dev_err(dev, "failed to register received refclk\n");
		return ret;
	}

	ret = tp_refclk_driver_register(tp_phy);
	if (ret) {
		dev_err(dev, "failed to register refclk driver\n");
		return ret;
	}

	ret = of_clk_add_hw_provider(node, of_clk_hw_onecell_get, data);
	if (ret) {
		dev_err(dev, "Failed to add clock provider: %s\n", node->name);
		return ret;
	}

	return 0;
}

static int tp_of_get_reset(struct tp_phy *tp_phy)
{
	struct device *dev = tp_phy->dev;
	struct reset_control *rst;

	rst = devm_reset_control_get_exclusive_by_index(dev, 0);
	if (IS_ERR(rst)) {
		dev_err(dev, "%s: failed to get reset\n", dev->of_node->full_name);
		return PTR_ERR(rst);
	}
	tp_phy->phy_rst = rst;

	rst = devm_reset_control_get_optional_exclusive(dev, "torrent_apb");
	if (IS_ERR(rst)) {
		dev_err(dev, "%s: failed to get apb reset\n", dev->of_node->full_name);
		return PTR_ERR(rst);
	}
	tp_phy->apb_rst = rst;
	return 0;
}

static int tp_of_get_clk(struct tp_phy *tp_phy)
{
	struct device *dev = tp_phy->dev;
	struct clk *clk;

	/* refclk: Input reference clock for PLL0 */
	clk = devm_clk_get(dev, "refclk");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "phy ref clock not found\n");

	tp_phy->clk = clk;

	/* refclk1: Input reference clock for PLL1 */
	clk = devm_clk_get_optional(dev, "pll1_refclk");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "phy PLL1 ref clock not found\n");

	tp_phy->clk1 = clk;
	return 0;
}

static bool tp_is_ref_freq_supported(enum tp_std_type std_type, u32 ref_clk_hz)
{
	static const u32 freq_set1[] = { 19200000, 20000000, 24000000,	25000000,  26000000,
					 27000000, 60000000, 100000000, 156250000, 0 };

	static const u32 freq_set2[] = { 19200000, 20000000,  24000000,	 25000000, 26000000,
					 27000000, 100000000, 156250000, 0 };

	static const u32 freq_set3[] = { 19200000,  20000000,  24000000, 25000000,
					 26000000,  27000000,  60000000, 100000000,
					 125000000, 156250000, 0 };

	static const u32 freq_set4[] = { 19200000,  20000000,  24000000, 25000000,
					 26000000,  27000000,  37500000, 75000000,
					 100000000, 150000000, 0 };

	const u32 *freq_list;

	switch (std_type) {
	case STD_TYPE_PCIE:
	case STD_TYPE_USXGMII:
		freq_list = freq_set1;
		break;

	case STD_TYPE_USB_TXRX:
	case STD_TYPE_DP:
	case STD_TYPE_XAUI:
		freq_list = freq_set2;
		break;

	case STD_TYPE_SGMII:
		freq_list = freq_set3;
		break;

	case STD_TYPE_SATA:
		freq_list = freq_set4;
		break;
	default:
		return false;
	}

	for (const u32 *p_freq = freq_list; *p_freq; p_freq++) {
		if (ref_clk_hz == *p_freq)
			return true;
	}
	return false;
}

static int tp_ref_clk_init(const struct tp_phy *tp_phy, struct clk *clk, const char *name,
			   u32 *p_ref_clk_khz)
{
	struct device *dev = tp_phy->dev;
	u32 ref_clk_rate;
	int ret;

	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(dev, "Failed to prepare %s ref clock: %d\n", name, ret);
		return ret;
	}

	ref_clk_rate = clk_get_rate(clk);
	if (!ref_clk_rate) {
		dev_err(dev, "Failed to get %s ref clock rate\n", name);
		ret = -EINVAL;
		goto disable_clk;
	}

	for (u32 i = 0; i < tp_phy->nb_standards; ++i) {
		if (!tp_is_ref_freq_supported(tp_phy->std_ptr[i]->type, ref_clk_rate)) {
			dev_err(dev, "Invalid ref clock rate: %u Hz\n", ref_clk_rate);
			ret = -EINVAL;
			goto disable_clk;
		}
	}

	*p_ref_clk_khz = ref_clk_rate / 1000U;
	return 0;

disable_clk:
	clk_disable_unprepare(clk);
	return ret;
}

static int tp_ref_clks_init(struct tp_phy *tp_phy)
{
	int ret;

	ret = tp_ref_clk_init(tp_phy, tp_phy->clk, "main", &tp_phy->ref_clk_khz);
	if (ret)
		return ret;

	if (!tp_phy->clk1) {
		tp_phy->ref_clk1_khz = tp_phy->ref_clk_khz;
		return 0;
	}

	ret = tp_ref_clk_init(tp_phy, tp_phy->clk1, "PLL1", &tp_phy->ref_clk1_khz);
	if (ret)
		clk_disable_unprepare(tp_phy->clk);

	return ret;
}

static int tp_read_techno_revision(struct tp_phy *tp_phy)
{
	struct tp_reg_base *reg_base = &tp_phy->reg_common_cdb;
	u16 techno_node;
	u16 techno_mfg;

	if (tp_reg_read(reg_base, CMN_PID_TYPE) != 0x7364)
		return -ENODEV;

	switch (tp_reg_read(reg_base, CMN_PID_REV)) {
	case 0x100:
		tp_phy->rev = REV_R100;
		break;
	case 0x110:
		tp_phy->rev = REV_R110;
		break;
	case 0x120:
		tp_phy->rev = REV_R120;
		break;
	case 0x130:
		tp_phy->rev = REV_R130;
		break;
	case 0x140:
		tp_phy->rev = REV_R140;
		break;
	case 0x150:
		tp_phy->rev = REV_R150;
		break;
	case 0x200:
		tp_phy->rev = REV_R200;
		break;
	case 0x210:
		tp_phy->rev = REV_R210;
		break;
	case 0x220:
		tp_phy->rev = REV_R220;
		break;
	default:
		return -EINVAL;
	}

	techno_mfg = tp_reg_read(reg_base, CMN_PID_MFG);
	techno_node = tp_reg_read(reg_base, CMN_PID_NODE);

	if (techno_mfg == 't') {
		u16 product_num = tp_reg_read(reg_base, CMN_PID_NUM);

		switch (techno_node) {
		case 0x16:
			tp_phy->tech_tag = (product_num == 0x0805) ?
				TECH_TSMC_16_0805 : TECH_TSMC_16;
			break;
		case 0x12:
			tp_phy->tech_tag = TECH_TSMC_12;
			break;
		case 0x07:
			tp_phy->tech_tag = TECH_TSMC_7;
			break;
		case 0x05:
			tp_phy->tech_tag = TECH_TSMC_5;
			break;
		default:
			return -EINVAL;
		}
	} else if (techno_mfg == 's') {
		switch (techno_node) {
		case 0x07:
			tp_phy->tech_tag = TECH_SS_7;
			break;
		case 0x05:
			tp_phy->tech_tag = TECH_SS_5;
			break;
		default:
			return -EINVAL;
		}
	} else {
		return -EINVAL;
	}

	dev_dbg(tp_phy->dev, "Revision: %s, tech: %s\n", tp_get_revision_desc(tp_phy->rev),
		tp_get_tech_desc(tp_phy->tech_tag));
	return 0;
}

static int tp_set_used_lanes(struct tp_phy *tp_phy, u32 mlane, u32 num_lanes)
{
	u8 mask_available_lanes;
	u8 mask_lanes;

	if (((mlane + num_lanes) > MAX_NUM_LANES) || num_lanes == 0)
		goto invalid_lanes;

	mask_lanes = GENMASK_U8(mlane + num_lanes - 1, mlane);
	mask_available_lanes = tp_phy->init_data->mask_available_lanes;

	if ((mask_lanes & mask_available_lanes) != mask_lanes)
		goto invalid_lanes;

	if (mask_lanes & tp_phy->mask_used_lanes)
		goto invalid_lanes;

	tp_phy->mask_used_lanes |= mask_lanes;
	return 0;

invalid_lanes:
	dev_err(tp_phy->dev, "Invalid lane configuration\n");
	return -EINVAL;
}

static bool tp_link_can_share_std(const struct tp_std_data *std, enum tp_std_type std_type,
				  u32 std_flags, u32 ssc_mode)
{
	if (std_type == STD_TYPE_DP || std_type == STD_TYPE_XAUI)
		return false;

	return (std->type == std_type) && (std->flags == std_flags) &&
	       (std->ssc_mode == ssc_mode);
}

static int tp_add_phy_from_of_subnode(struct tp_phy *tp_phy, struct device_node *child)
{
	enum tp_std_type std_type = STD_TYPE_NONE;
	struct tp_std_data *standard = NULL;
	u32 ssc_mode = CDNS_SERDES_NO_SSC;
	struct device *dev = tp_phy->dev;
	struct tp_instance *inst = NULL;
	struct reset_control *lnk_rst;
	u32 std_flags = STD_FLAG_NONE;
	u32 supported_flags = 0;
	u32 phy_type = PHY_NONE;
	u32 num_lanes = 0;
	int ret = -EINVAL;
	struct phy *gphy;
	u32 mlane = 0;

	/* First check the master lane and the number of lanes */
	if (of_property_read_u32(child, "reg", &mlane)) {
		dev_err(dev, "%s: No \"reg\"-property.\n", child->full_name);
		return -EINVAL;
	}

	if (of_property_read_u32(child, "cdns,num-lanes", &num_lanes)) {
		dev_err(dev, "%s: No \"cdns,num-lanes\"-property.\n", child->full_name);
		return -EINVAL;
	}

	ret = tp_set_used_lanes(tp_phy, mlane, num_lanes);
	if (ret)
		return ret;

	/* Then retrieve associated standard */
	if (of_property_read_u32(child, "cdns,phy-type", &phy_type)) {
		dev_err(dev, "%s: No \"cdns,phy-type\"-property.\n", child->full_name);
		return -EINVAL;
	}

	/* Get optional mode */
	of_property_read_u32(child, "cdns,phy-mode", &std_flags);

	/* Get SSC mode */
	of_property_read_u32(child, "cdns,ssc-mode", &ssc_mode);

	switch (phy_type) {
	case PHY_TYPE_PCIE:
		std_type = STD_TYPE_PCIE;
		break;
	case PHY_TYPE_USB3:
		std_type = STD_TYPE_USB_TXRX;
		supported_flags = CDNS_TORRENT_MODE_USB_G1 | CDNS_TORRENT_MODE_USB_IO_11V;
		break;
	case PHY_TYPE_DP:
		std_type = STD_TYPE_DP;
		break;
	case PHY_TYPE_XAUI:
		std_type = STD_TYPE_XAUI;
		supported_flags = CDNS_TORRENT_MODE_XAUI_LP;
		break;
	case PHY_TYPE_SGMII:
		std_type = STD_TYPE_SGMII;
		supported_flags = CDNS_TORRENT_MODE_2500_BASEX | CDNS_TORRENT_MODE_QSGMII;
		break;
	case PHY_TYPE_QSGMII:
		std_type = STD_TYPE_SGMII;
		std_flags |= CDNS_TORRENT_MODE_QSGMII;
		supported_flags = CDNS_TORRENT_MODE_QSGMII;
		break;
	case PHY_TYPE_SATA:
		std_type = STD_TYPE_SATA;
		supported_flags = CDNS_TORRENT_MODE_SATA_LP;
		break;
	case PHY_TYPE_USXGMII:
		std_type = STD_TYPE_USXGMII;
		break;
	default:
		dev_err(dev, "Unsupported protocol: %u\n", phy_type);
		return -EINVAL;
	}

	if (std_flags & ~supported_flags) {
		dev_err(dev, "Unsupported modes 0x%X for this PHY type %u\n",
			std_flags, std_type);
		return -EINVAL;
	}

	for (u32 i = 0; i < MAX_NUM_STANDARDS; ++i) {
		struct tp_std_data *std = &tp_phy->std_data[i];

		if (std->type == STD_TYPE_NONE) {
			std->type = std_type;
			std->flags = std_flags;
			std->ssc_mode = ssc_mode;
			standard = std;
			tp_phy->nb_standards += 1;
			break;

		} else if (tp_link_can_share_std(std, std_type, std_flags, ssc_mode)) {
			standard = std;
			break;
		}
	}

	if (!standard) {
		dev_err(dev, "%s: Cannot declare more than 2 standards.\n", child->full_name);
		return -EINVAL;
	}

	if (standard->nb_phys >= MAX_NUM_LANES)
		return -EINVAL;

	/* Get the PHY instance */
	inst = &standard->phys[standard->nb_phys];
	standard->nb_phys += 1;

	inst->std = standard;
	inst->mlane = mlane;
	inst->num_lanes = num_lanes;
	inst->max_lanes = num_lanes;
	lnk_rst = of_reset_control_array_get_exclusive(child);
	if (IS_ERR(lnk_rst)) {
		dev_err(dev, "%s: failed to get reset\n", child->full_name);
		return PTR_ERR(lnk_rst);
	}
	inst->lnk_rst = lnk_rst;

	gphy = devm_phy_create(dev, child, &cdns_torrent_phy_ops);
	if (IS_ERR(gphy))
		return PTR_ERR(gphy);

	if (phy_type == PHY_TYPE_DP) {
		u32 max_bit_rate = DEFAULT_MAX_BIT_RATE;

		switch (num_lanes) {
		case 1:
		case 2:
		case 4:
			/* valid number of lanes */
			break;
		default:
			dev_err(dev, "unsupported number of lanes: %u\n", num_lanes);
			return -EINVAL;
		}

		of_property_read_u32(child, "cdns,max-bit-rate", &max_bit_rate);
		if (tp_dp_pll_freq_from_bit_rate(max_bit_rate) == 0) {
			dev_err(dev, "unsupported max bit rate: %u Mbps\n", max_bit_rate);
			return -EINVAL;
		}

		standard->max_bit_rate = max_bit_rate;
		gphy->attrs.bus_width = num_lanes;
		gphy->attrs.max_link_rate = max_bit_rate;
		gphy->attrs.mode = PHY_MODE_DP;
	}

	inst->phy = gphy;
	phy_set_drvdata(gphy, inst);

	return 0;
}

static void tp_check_multiple_pcie_links(struct tp_phy *tp_phy)
{
	unsigned int num_pcie_links = 0;

	/* First count the number of subnode with PCIe protocol. */
	for (u32 i = 0; i < tp_phy->nb_standards; ++i) {
		struct tp_std_data *std = &tp_phy->std_data[i];

		if (std->type == STD_TYPE_PCIE)
			num_pcie_links += std->nb_phys;
	}

	/*
	 * PCIe Multilink configuration can be supported along with a
	 * non-PCIe protocol. Such configurations are handled by adding a flag
	 * named STD_FLAG_PCIE_ML
	 */
	if (num_pcie_links > 1) {
		for (u32 i = 0; i < tp_phy->nb_standards; ++i) {
			struct tp_std_data *std = &tp_phy->std_data[i];

			if (std->type == STD_TYPE_PCIE)
				std->flags |= STD_FLAG_PCIE_ML;
		}
	}
}

static bool tp_find_link_cfg_is_same_std(const struct tp_std_data *std, u32 std_id)
{
	u32 link_cfg_flags = STD_FLAGS_FROM_ID(std_id) & STD_FIND_CFG_FLAG_MASK;
	enum tp_std_type link_cfg_std_type = STD_TYPE_FROM_ID(std_id);

	return (std->type == link_cfg_std_type) &&
	       ((std->flags & STD_FIND_CFG_FLAG_MASK) == link_cfg_flags);
}

static const struct tp_link_cfg *tp_find_link_cfg_from_standards(const struct tp_std_data *std0,
								 const struct tp_std_data *std1)
{
	for (u32 i = 0; i < ARRAY_SIZE(tp_link_cfgs); ++i) {
		const struct tp_link_cfg *link_cfg = &tp_link_cfgs[i];

		if (tp_find_link_cfg_is_same_std(std0, link_cfg->std0) &&
		    tp_find_link_cfg_is_same_std(std1, link_cfg->std1))
			return link_cfg;
	}

	return NULL;
}

static int tp_find_link_cfg(struct tp_phy *tp_phy)
{
	const struct tp_link_cfg *link_cfg;
	struct device *dev = tp_phy->dev;
	u8 nb_std = tp_phy->nb_standards;

	if (nb_std == 0)
		return -EINVAL;

	link_cfg = tp_find_link_cfg_from_standards(&tp_phy->std_data[0], &tp_phy->std_data[1]);
	if (!link_cfg) {
		link_cfg = tp_find_link_cfg_from_standards(&tp_phy->std_data[1],
							   &tp_phy->std_data[0]);
		if (!link_cfg) {
			dev_err(dev, "configuration with %s and %s is not supported\n",
				tp_get_std_name(&tp_phy->std_data[0]),
				tp_get_std_name(&tp_phy->std_data[1]));
			return -EINVAL;
		}

		WARN_ON(nb_std != 2);
		tp_phy->std_ptr[0] = &tp_phy->std_data[1];
		tp_phy->std_ptr[1] = &tp_phy->std_data[0];
	} else {
		tp_phy->std_ptr[0] = &tp_phy->std_data[0];
		tp_phy->std_ptr[1] = (nb_std > 1) ? &tp_phy->std_data[1] : NULL;
	}

	tp_phy->std_ptr[0]->flags |= STD_FLAGS_FROM_ID(link_cfg->std0);
	if (tp_phy->std_ptr[1])
		tp_phy->std_ptr[1]->flags |= STD_FLAGS_FROM_ID(link_cfg->std1);

	tp_phy->link_cfg = link_cfg;
	return 0;
}

static int tp_dp_reg_base_init(struct platform_device *pdev, struct tp_phy *tp_phy)
{
	bool has_dp = false;

	for (u32 i = 0; i < tp_phy->nb_standards; ++i) {
		if (tp_phy->std_ptr[i]->type == STD_TYPE_DP) {
			has_dp = true;
			break;
		}
	}

	if (has_dp) {
		/* DPTX registers */
		void __iomem *base = devm_platform_ioremap_resource(pdev, 1);

		if (IS_ERR(base))
			return PTR_ERR(base);

		tp_phy->base = base;
	}

	return 0;
}

static void tp_phy_cleanup(const struct tp_phy *tp_phy)
{
	reset_control_assert(tp_phy->phy_rst);
	reset_control_assert(tp_phy->apb_rst);

	for (u32 i = 0; i < MAX_NUM_STANDARDS; ++i) {
		const struct tp_std_data *std = &tp_phy->std_data[i];

		for (u32 j = 0; j < std->nb_phys; ++j) {
			reset_control_assert(std->phys[j].lnk_rst);
			reset_control_put(std->phys[j].lnk_rst);
		}
	}

	clk_disable_unprepare(tp_phy->clk1);
	clk_disable_unprepare(tp_phy->clk);
	tp_clk_cleanup(tp_phy);
}

static bool tp_is_already_configured(const struct tp_phy *tp_phy)
{
	u16 val = tp_reg_read(&tp_phy->reg_phy_pma_common_cdb, PHY_PMA_CMN_CTRL1);

	return (val & 0x0001) != 0;
}

static void tp_refclk_driver_suspend(struct tp_phy *tp_phy)
{
	struct clk_hw *hw = tp_phy->clk_hw_data->hws[CDNS_TORRENT_REFCLK_DRIVER];
	struct tp_refclk_driver *refclk = tp_to_refclk_driver(hw);

	refclk->parent_index = tp_refclk_driver_get_parent(hw);
}

static int tp_refclk_driver_resume(const struct tp_phy *tp_phy)
{
	struct clk_hw *hw = tp_phy->clk_hw_data->hws[CDNS_TORRENT_REFCLK_DRIVER];
	struct tp_refclk_driver *refclk = tp_to_refclk_driver(hw);

	return tp_refclk_driver_set_parent(hw, refclk->parent_index);
}

static int tp_suspend_noirq(struct device *dev)
{
	struct tp_phy *tp_phy = dev_get_drvdata(dev);

	tp_refclk_driver_suspend(tp_phy);

	for (u32 i = 0; i < tp_phy->nb_standards; ++i) {
		struct tp_std_data *std = tp_phy->std_ptr[i];

		for (u32 j = 0; j < std->nb_phys; ++j)
			reset_control_assert(std->phys[i].lnk_rst);
	}

	reset_control_assert(tp_phy->phy_rst);
	reset_control_assert(tp_phy->apb_rst);

	if (tp_phy->already_configured) {
		tp_phy->already_configured = false;
	} else {
		clk_disable_unprepare(tp_phy->clk1);
		clk_disable_unprepare(tp_phy->clk);
	}

	return 0;
}

static int tp_resume_noirq(struct device *dev)
{
	int ret;
	struct tp_phy *tp_phy = dev_get_drvdata(dev);

	ret = tp_refclk_driver_resume(tp_phy);
	if (ret)
		return ret;

	ret = tp_ref_clks_init(tp_phy);
	if (ret)
		return ret;

	/* Enable APB */
	reset_control_deassert(tp_phy->apb_rst);

	ret = tp_configure_links(tp_phy);
	if (ret)
		goto put_lnk_rst;

	return 0;

put_lnk_rst:

	for (u32 i = 0; i < tp_phy->nb_standards; ++i) {
		struct tp_std_data *std = tp_phy->std_ptr[i];

		for (u32 j = 0; j < std->nb_phys; ++j)
			reset_control_assert(std->phys[i].lnk_rst);
	}

	reset_control_assert(tp_phy->apb_rst);
	reset_control_assert(tp_phy->phy_rst);

	clk_disable_unprepare(tp_phy->clk1);
	clk_disable_unprepare(tp_phy->clk);

	return ret;
}

static DEFINE_NOIRQ_DEV_PM_OPS(tp_pm_ops, tp_suspend_noirq, tp_resume_noirq);

static int cdns_torrent_phy_probe(struct platform_device *pdev)
{
	struct tp_phy *tp_phy;
	struct device *dev = &pdev->dev;
	struct phy_provider *phy_provider;
	const struct tp_init_data *init_data;
	struct device_node *child = NULL;
	int ret;

	/* Get init data for this PHY */
	init_data = of_device_get_match_data(dev);
	if (!init_data)
		return -EINVAL;

	tp_phy = devm_kzalloc(dev, sizeof(*tp_phy), GFP_KERNEL);
	if (!tp_phy)
		return -ENOMEM;

	dev_set_drvdata(dev, tp_phy);
	mutex_init(&tp_phy->lock);
	tp_phy->dev = dev;
	tp_phy->init_data = init_data;

	tp_phy->sd_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tp_phy->sd_base))
		return PTR_ERR(tp_phy->sd_base);

	ret = of_get_available_child_count(dev->of_node);
	if (ret == 0) {
		dev_err(dev, "No available link subnodes found\n");
		return -EINVAL;
	}

	tp_reg_bases_init(tp_phy);
	tp_reg_fields_init(tp_phy);

	ret = tp_read_techno_revision(tp_phy);
	if (ret)
		return ret;

	ret = tp_clk_register(tp_phy);
	if (ret)
		return ret;

	ret = tp_of_get_reset(tp_phy);
	if (ret)
		goto cleanup;

	ret = tp_of_get_clk(tp_phy);
	if (ret)
		goto cleanup;

	tp_phy->already_configured = tp_is_already_configured(tp_phy);
	if (!tp_phy->already_configured) {
		ret = tp_ref_clks_init(tp_phy);
		if (ret)
			goto cleanup;

		/* Enable APB */
		reset_control_deassert(tp_phy->apb_rst);
	}

	for_each_available_child_of_node(dev->of_node, child) {
		/* PHY subnode name must be 'phy'. */
		if (!of_node_name_eq(child, "phy"))
			continue;

		ret = tp_add_phy_from_of_subnode(tp_phy, child);
		if (ret)
			goto cleanup;
	}
	child = NULL;

	if ((WARN_ON(tp_phy->nb_standards > MAX_NUM_STANDARDS)) || tp_phy->nb_standards == 0) {
		ret = -EINVAL;
		goto cleanup;
	}

	tp_check_multiple_pcie_links(tp_phy);

	ret = tp_find_link_cfg(tp_phy);
	if (ret)
		goto cleanup;

	ret = tp_dp_reg_base_init(pdev, tp_phy);
	if (ret)
		goto cleanup;

	if (!tp_phy->already_configured) {
		ret = tp_configure_links(tp_phy);
		if (ret)
			goto cleanup;
	}

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		ret = PTR_ERR(phy_provider);
		goto cleanup;
	}

	for (u32 i = 0; i < tp_phy->nb_standards; ++i) {
		struct tp_std_data *std = &tp_phy->std_data[i];
		const char *name = tp_get_std_name(std);

		for (u32 j = 0; j < std->nb_phys; ++j)
			dev_dbg(dev, "- %s (%d lanes)", name, std->phys[j].num_lanes);
	}

	return 0;

cleanup:
	if (child)
		of_node_put(child);

	tp_phy_cleanup(tp_phy);
	return ret;
}

static void cdns_torrent_phy_remove(struct platform_device *pdev)
{
	struct tp_phy *tp_phy = platform_get_drvdata(pdev);

	tp_phy_cleanup(tp_phy);
}

static const struct tp_init_data cdns_map_torrent = {
	.block_offset_shift = 0x2,
	.reg_offset_shift = 0x2,
	.mask_available_lanes = 0x0F,
};

static const struct tp_init_data ti_j72xx_map_torrent = {
	.block_offset_shift = 0x0,
	.reg_offset_shift = 0x1,
	.mask_available_lanes = 0x0F,
	.usxgmii_refclk_rcv_out_en = true,
	.rxda_des_clk_2x_en = true,
};

static const struct of_device_id cdns_torrent_phy_of_match[] = {
	{
		.compatible = "cdns,torrent-phy",
		.data = &cdns_map_torrent,
	},
	{
		.compatible = "ti,j721e-serdes-10g",
		.data = &ti_j72xx_map_torrent,
	},
	{
		.compatible = "ti,j7200-serdes-10g",
		.data = &ti_j72xx_map_torrent,
	},
	{}
};
MODULE_DEVICE_TABLE(of, cdns_torrent_phy_of_match);

static struct platform_driver cdns_torrent_phy_driver = {
	.probe	= cdns_torrent_phy_probe,
	.remove	= cdns_torrent_phy_remove,
	.driver	= {
		.name	= "cdns-torrent-phy",
		.of_match_table	= cdns_torrent_phy_of_match,
		.pm	= pm_sleep_ptr(&tp_pm_ops),
	}
};
module_platform_driver(cdns_torrent_phy_driver);

MODULE_AUTHOR("Benjamin Robin <benjamin.robin@bootlin.com>");
MODULE_DESCRIPTION("Cadence Torrent PHY driver");
MODULE_LICENSE("GPL");
