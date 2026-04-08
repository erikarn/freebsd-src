/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Adrian Chadd <adrian@FreeBSD.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef	__QCOM_CLK_APLL_REG_H__
#define	__QCOM_CLK_APLL_REG_H__

/*
 * Some registers are fixed for all of the PLLs supported
 * by this driver.  Others are different based on the
 * specific node/chipset.
 */


#define QCOM_APLL_HW_REG_PLL_MODE(p)		((p)->reg_offset + 0x0)
# define PLL_OUTCTRL		(1 << 0)
# define PLL_BYPASSNL		(1 << 1)
# define PLL_RESET_N		(1 << 2)
# define PLL_OFFLINE_REQ	(1 << 7)
# define PLL_LOCK_COUNT_SHIFT	8
# define PLL_LOCK_COUNT_MASK	0x3f
# define PLL_BIAS_COUNT_SHIFT	14
# define PLL_BIAS_COUNT_MASK	0x3f
# define PLL_VOTE_FSM_ENA	(1 << 20)
# define PLL_FSM_ENA		(1 << 20)
# define PLL_VOTE_FSM_RESET	(1 << 21)
# define PLL_UPDATE		(1 << 22)
# define PLL_UPDATE_BYPASS	(1 << 23)
# define PLL_FSM_LEGACY_MODE	(1 << 24)
# define PLL_OFFLINE_ACK	(1 << 28)
# define ALPHA_PLL_ACK_LATCH	(1 << 29)
# define PLL_ACTIVE_FLAG	(1 << 30)
# define PLL_LOCK_DET		(1 << 31)


/*
 * PLL width parameters.
 */

/*
 * From Linux: Even though 40 bits are present, use only 32 for ease
 * of calculation.
 */
#define	QCOM_CLK_APLL_ALPHA_REG_BITWIDTH	40
#define	QCOM_CLK_APLL_ALPHA_REG_16BIT_WIDTH	16
#define	QCOM_CLK_APLL_ALPHA_BITWIDTH		32U
#define	QCOM_CLK_APLL_ALPHA_SHIFT(w)		\
	    MIN(w, QCOM_CLK_APLL_ALPHA_BITWIDTH)



#define	QCOM_CLK_APLL_HW_REG_PLL_L_VAL(p)		\
	    ((p)->reg_offset + (p)->ops->regmap[PLL_OFF_L_VAL])
#define	QCOM_CLK_APLL_HW_REG_PLL_CAL_L_VAL(p)	\
	    ((p)->reg_offset + (p)->ops->regmap[PLL_OFF_CAL_L_VAL])
#define	QCOM_CLK_APLL_HW_REG_PLL_ALPHA_VAL(p)	\
	    ((p)->reg_offset + (p)->ops->regmap[PLL_OFF_ALPHA_VAL])
#define	QCOM_CLK_APLL_HW_REG_PLL_ALPHA_VAL_U(p)	\
	    ((p)->reg_offset + (p)->ops->regmap[PLL_OFF_ALPHA_VAL_U])

#define QCOM_CLK_APLL_HW_PLL_USER_CTL(p)		\
	    ((p)->reg_offset + (p)->ops->regmap[PLL_OFF_USER_CTL])
#if 0
# define PLL_POST_DIV_SHIFT	8
# define PLL_POST_DIV_MASK(p)	GENMASK((p)->width ? (p)->width - 1 : 3, 0)
# define PLL_ALPHA_MSB		BIT(15)
# define PLL_ALPHA_EN		BIT(24)
# define PLL_ALPHA_MODE		BIT(25)
# define PLL_VCO_SHIFT		20
# define PLL_VCO_MASK		0x3
#endif


/* LUCID EVO PLL specific settings and offsets */
#define	LUCID_EVO_PCAL_NOT_DONE		(1 << 8)
#define	LUCID_EVO_ENABLE_VOTE_RUN	(1 << 25)
#define	LUCID_EVO_PLL_L_VAL_MASK	0x0000ffff
#define	LUCID_EVO_PLL_CAL_L_VAL_SHIFT	16
#define	LUCID_OLE_PLL_RINGOSC_CAL_L_VAL_SHIFT	24

#if 0
#define PLL_USER_CTL(p)		((p)->offset + (p)->regs[PLL_OFF_USER_CTL])
# define PLL_POST_DIV_SHIFT	8
# define PLL_POST_DIV_MASK(p)	GENMASK((p)->width ? (p)->width - 1 : 3, 0)
# define PLL_ALPHA_MSB		BIT(15)
# define PLL_ALPHA_EN		BIT(24)
# define PLL_ALPHA_MODE		BIT(25)
# define PLL_VCO_SHIFT		20
# define PLL_VCO_MASK		0x3

#define PLL_USER_CTL_U(p)	((p)->offset + (p)->regs[PLL_OFF_USER_CTL_U])
#define PLL_USER_CTL_U1(p)	((p)->offset + (p)->regs[PLL_OFF_USER_CTL_U1])

#define PLL_CONFIG_CTL(p)	((p)->offset + (p)->regs[PLL_OFF_CONFIG_CTL])
#define PLL_CONFIG_CTL_U(p)	((p)->offset + (p)->regs[PLL_OFF_CONFIG_CTL_U])
#define PLL_CONFIG_CTL_U1(p)	((p)->offset + (p)->regs[PLL_OFF_CONFIG_CTL_U1])
#define PLL_CONFIG_CTL_U2(p)	((p)->offset + (p)->regs[PLL_OFF_CONFIG_CTL_U2])
#define PLL_TEST_CTL(p)		((p)->offset + (p)->regs[PLL_OFF_TEST_CTL])
#define PLL_TEST_CTL_U(p)	((p)->offset + (p)->regs[PLL_OFF_TEST_CTL_U])
#define PLL_TEST_CTL_U1(p)     ((p)->offset + (p)->regs[PLL_OFF_TEST_CTL_U1])
#define PLL_TEST_CTL_U2(p)     ((p)->offset + (p)->regs[PLL_OFF_TEST_CTL_U2])
#define PLL_TEST_CTL_U3(p)     ((p)->offset + (p)->regs[PLL_OFF_TEST_CTL_U3])
#define PLL_STATUS(p)		((p)->offset + (p)->regs[PLL_OFF_STATUS])
#endif

#define	QCOM_APLL_HW_REG_PLL_OPMODE(p)			\
	    ((p)->reg_offset + (p)->ops->regmap[PLL_OFF_OPMODE])

/* TODO: is this used by other registers besides OPMODE? */
#define	PLL_STANDBY		0x0
#define	PLL_RUN			0x1
#define	PLL_OUT_MASK		0x7

#define	PLL_RATE_MARGIN		500

#if 0
#define PLL_FRAC(p)		((p)->offset + (p)->regs[PLL_OFF_FRAC])

#endif /* #if 0 */

#if 0

const u8 clk_alpha_pll_regs[][PLL_OFF_MAX_REGS] = {
	[CLK_ALPHA_PLL_TYPE_DEFAULT] =  {
		[PLL_OFF_L_VAL] = 0x04,
		[PLL_OFF_ALPHA_VAL] = 0x08,
		[PLL_OFF_ALPHA_VAL_U] = 0x0c,
		[PLL_OFF_USER_CTL] = 0x10,
		[PLL_OFF_USER_CTL_U] = 0x14,
		[PLL_OFF_CONFIG_CTL] = 0x18,
		[PLL_OFF_TEST_CTL] = 0x1c,
		[PLL_OFF_TEST_CTL_U] = 0x20,
		[PLL_OFF_STATUS] = 0x24,
	},
	[CLK_ALPHA_PLL_TYPE_HUAYRA] =  {
		[PLL_OFF_L_VAL] = 0x04,
		[PLL_OFF_ALPHA_VAL] = 0x08,
		[PLL_OFF_USER_CTL] = 0x10,
		[PLL_OFF_CONFIG_CTL] = 0x14,
		[PLL_OFF_CONFIG_CTL_U] = 0x18,
		[PLL_OFF_TEST_CTL] = 0x1c,
		[PLL_OFF_TEST_CTL_U] = 0x20,
		[PLL_OFF_STATUS] = 0x24,
	},
	[CLK_ALPHA_PLL_TYPE_HUAYRA_APSS] =  {
		[PLL_OFF_L_VAL] = 0x08,
		[PLL_OFF_ALPHA_VAL] = 0x10,
		[PLL_OFF_USER_CTL] = 0x18,
		[PLL_OFF_CONFIG_CTL] = 0x20,
		[PLL_OFF_CONFIG_CTL_U] = 0x24,
		[PLL_OFF_STATUS] = 0x28,
		[PLL_OFF_TEST_CTL] = 0x30,
		[PLL_OFF_TEST_CTL_U] = 0x34,
	},
	[CLK_ALPHA_PLL_TYPE_HUAYRA_2290] =  {
		[PLL_OFF_L_VAL] = 0x04,
		[PLL_OFF_ALPHA_VAL] = 0x08,
		[PLL_OFF_USER_CTL] = 0x0c,
		[PLL_OFF_CONFIG_CTL] = 0x10,
		[PLL_OFF_CONFIG_CTL_U] = 0x14,
		[PLL_OFF_CONFIG_CTL_U1] = 0x18,
		[PLL_OFF_TEST_CTL] = 0x1c,
		[PLL_OFF_TEST_CTL_U] = 0x20,
		[PLL_OFF_TEST_CTL_U1] = 0x24,
		[PLL_OFF_OPMODE] = 0x28,
		[PLL_OFF_STATUS] = 0x38,
	},
	[CLK_ALPHA_PLL_TYPE_BRAMMO] =  {
		[PLL_OFF_L_VAL] = 0x04,
		[PLL_OFF_ALPHA_VAL] = 0x08,
		[PLL_OFF_ALPHA_VAL_U] = 0x0c,
		[PLL_OFF_USER_CTL] = 0x10,
		[PLL_OFF_CONFIG_CTL] = 0x18,
		[PLL_OFF_TEST_CTL] = 0x1c,
		[PLL_OFF_STATUS] = 0x24,
	},
	[CLK_ALPHA_PLL_TYPE_FABIA] =  {
		[PLL_OFF_L_VAL] = 0x04,
		[PLL_OFF_USER_CTL] = 0x0c,
		[PLL_OFF_USER_CTL_U] = 0x10,
		[PLL_OFF_CONFIG_CTL] = 0x14,
		[PLL_OFF_CONFIG_CTL_U] = 0x18,
		[PLL_OFF_TEST_CTL] = 0x1c,
		[PLL_OFF_TEST_CTL_U] = 0x20,
		[PLL_OFF_STATUS] = 0x24,
		[PLL_OFF_OPMODE] = 0x2c,
		[PLL_OFF_FRAC] = 0x38,
	},
	[CLK_ALPHA_PLL_TYPE_TRION] = {
		[PLL_OFF_L_VAL] = 0x04,
		[PLL_OFF_CAL_L_VAL] = 0x08,
		[PLL_OFF_USER_CTL] = 0x0c,
		[PLL_OFF_USER_CTL_U] = 0x10,
		[PLL_OFF_USER_CTL_U1] = 0x14,
		[PLL_OFF_CONFIG_CTL] = 0x18,
		[PLL_OFF_CONFIG_CTL_U] = 0x1c,
		[PLL_OFF_CONFIG_CTL_U1] = 0x20,
		[PLL_OFF_TEST_CTL] = 0x24,
		[PLL_OFF_TEST_CTL_U] = 0x28,
		[PLL_OFF_TEST_CTL_U1] = 0x2c,
		[PLL_OFF_STATUS] = 0x30,
		[PLL_OFF_OPMODE] = 0x38,
		[PLL_OFF_ALPHA_VAL] = 0x40,
	},
	[CLK_ALPHA_PLL_TYPE_AGERA] =  {
		[PLL_OFF_L_VAL] = 0x04,
		[PLL_OFF_ALPHA_VAL] = 0x08,
		[PLL_OFF_USER_CTL] = 0x0c,
		[PLL_OFF_CONFIG_CTL] = 0x10,
		[PLL_OFF_CONFIG_CTL_U] = 0x14,
		[PLL_OFF_TEST_CTL] = 0x18,
		[PLL_OFF_TEST_CTL_U] = 0x1c,
		[PLL_OFF_STATUS] = 0x2c,
	},
	[CLK_ALPHA_PLL_TYPE_ZONDA] =  {
		[PLL_OFF_L_VAL] = 0x04,
		[PLL_OFF_ALPHA_VAL] = 0x08,
		[PLL_OFF_USER_CTL] = 0x0c,
		[PLL_OFF_CONFIG_CTL] = 0x10,
		[PLL_OFF_CONFIG_CTL_U] = 0x14,
		[PLL_OFF_CONFIG_CTL_U1] = 0x18,
		[PLL_OFF_TEST_CTL] = 0x1c,
		[PLL_OFF_TEST_CTL_U] = 0x20,
		[PLL_OFF_TEST_CTL_U1] = 0x24,
		[PLL_OFF_OPMODE] = 0x28,
		[PLL_OFF_STATUS] = 0x38,
	},
	[CLK_ALPHA_PLL_TYPE_LUCID_EVO] = {
		[PLL_OFF_OPMODE] = 0x04,
		[PLL_OFF_STATUS] = 0x0c,
		[PLL_OFF_L_VAL] = 0x10,
		[PLL_OFF_ALPHA_VAL] = 0x14,
		[PLL_OFF_USER_CTL] = 0x18,
		[PLL_OFF_USER_CTL_U] = 0x1c,
		[PLL_OFF_CONFIG_CTL] = 0x20,
		[PLL_OFF_CONFIG_CTL_U] = 0x24,
		[PLL_OFF_CONFIG_CTL_U1] = 0x28,
		[PLL_OFF_TEST_CTL] = 0x2c,
		[PLL_OFF_TEST_CTL_U] = 0x30,
		[PLL_OFF_TEST_CTL_U1] = 0x34,
	},
	[CLK_ALPHA_PLL_TYPE_LUCID_OLE] = {
		[PLL_OFF_OPMODE] = 0x04,
		[PLL_OFF_STATE] = 0x08,
		[PLL_OFF_STATUS] = 0x0c,
		[PLL_OFF_L_VAL] = 0x10,
		[PLL_OFF_ALPHA_VAL] = 0x14,
		[PLL_OFF_USER_CTL] = 0x18,
		[PLL_OFF_USER_CTL_U] = 0x1c,
		[PLL_OFF_CONFIG_CTL] = 0x20,
		[PLL_OFF_CONFIG_CTL_U] = 0x24,
		[PLL_OFF_CONFIG_CTL_U1] = 0x28,
		[PLL_OFF_TEST_CTL] = 0x2c,
		[PLL_OFF_TEST_CTL_U] = 0x30,
		[PLL_OFF_TEST_CTL_U1] = 0x34,
		[PLL_OFF_TEST_CTL_U2] = 0x38,
	},
	[CLK_ALPHA_PLL_TYPE_PONGO_ELU] = {
		[PLL_OFF_OPMODE] = 0x04,
		[PLL_OFF_STATE] = 0x08,
		[PLL_OFF_STATUS] = 0x0c,
		[PLL_OFF_L_VAL] = 0x10,
		[PLL_OFF_USER_CTL] = 0x14,
		[PLL_OFF_USER_CTL_U] = 0x18,
		[PLL_OFF_CONFIG_CTL] = 0x1c,
		[PLL_OFF_CONFIG_CTL_U] = 0x20,
		[PLL_OFF_CONFIG_CTL_U1] = 0x24,
		[PLL_OFF_CONFIG_CTL_U2] = 0x28,
		[PLL_OFF_TEST_CTL] = 0x2c,
		[PLL_OFF_TEST_CTL_U] = 0x30,
		[PLL_OFF_TEST_CTL_U1] = 0x34,
		[PLL_OFF_TEST_CTL_U2] = 0x38,
		[PLL_OFF_TEST_CTL_U3] = 0x3c,
	},
	[CLK_ALPHA_PLL_TYPE_TAYCAN_ELU] = {
		[PLL_OFF_OPMODE] = 0x04,
		[PLL_OFF_STATE] = 0x08,
		[PLL_OFF_STATUS] = 0x0c,
		[PLL_OFF_L_VAL] = 0x10,
		[PLL_OFF_ALPHA_VAL] = 0x14,
		[PLL_OFF_USER_CTL] = 0x18,
		[PLL_OFF_USER_CTL_U] = 0x1c,
		[PLL_OFF_CONFIG_CTL] = 0x20,
		[PLL_OFF_CONFIG_CTL_U] = 0x24,
		[PLL_OFF_CONFIG_CTL_U1] = 0x28,
		[PLL_OFF_TEST_CTL] = 0x2c,
		[PLL_OFF_TEST_CTL_U] = 0x30,
	},
	[CLK_ALPHA_PLL_TYPE_RIVIAN_EVO] = {
		[PLL_OFF_OPMODE] = 0x04,
		[PLL_OFF_STATUS] = 0x0c,
		[PLL_OFF_L_VAL] = 0x10,
		[PLL_OFF_USER_CTL] = 0x14,
		[PLL_OFF_USER_CTL_U] = 0x18,
		[PLL_OFF_CONFIG_CTL] = 0x1c,
		[PLL_OFF_CONFIG_CTL_U] = 0x20,
		[PLL_OFF_CONFIG_CTL_U1] = 0x24,
		[PLL_OFF_TEST_CTL] = 0x28,
		[PLL_OFF_TEST_CTL_U] = 0x2c,
	},
	[CLK_ALPHA_PLL_TYPE_DEFAULT_EVO] =  {
		[PLL_OFF_L_VAL] = 0x04,
		[PLL_OFF_ALPHA_VAL] = 0x08,
		[PLL_OFF_ALPHA_VAL_U] = 0x0c,
		[PLL_OFF_TEST_CTL] = 0x10,
		[PLL_OFF_TEST_CTL_U] = 0x14,
		[PLL_OFF_USER_CTL] = 0x18,
		[PLL_OFF_USER_CTL_U] = 0x1c,
		[PLL_OFF_CONFIG_CTL] = 0x20,
		[PLL_OFF_STATUS] = 0x24,
	},
	[CLK_ALPHA_PLL_TYPE_BRAMMO_EVO] =  {
		[PLL_OFF_L_VAL] = 0x04,
		[PLL_OFF_ALPHA_VAL] = 0x08,
		[PLL_OFF_ALPHA_VAL_U] = 0x0c,
		[PLL_OFF_TEST_CTL] = 0x10,
		[PLL_OFF_TEST_CTL_U] = 0x14,
		[PLL_OFF_USER_CTL] = 0x18,
		[PLL_OFF_CONFIG_CTL] = 0x1C,
		[PLL_OFF_STATUS] = 0x20,
	},
	[CLK_ALPHA_PLL_TYPE_STROMER] = {
		[PLL_OFF_L_VAL] = 0x08,
		[PLL_OFF_ALPHA_VAL] = 0x10,
		[PLL_OFF_ALPHA_VAL_U] = 0x14,
		[PLL_OFF_USER_CTL] = 0x18,
		[PLL_OFF_USER_CTL_U] = 0x1c,
		[PLL_OFF_CONFIG_CTL] = 0x20,
		[PLL_OFF_STATUS] = 0x28,
		[PLL_OFF_TEST_CTL] = 0x30,
		[PLL_OFF_TEST_CTL_U] = 0x34,
	},
	[CLK_ALPHA_PLL_TYPE_STROMER_PLUS] =  {
		[PLL_OFF_L_VAL] = 0x04,
		[PLL_OFF_USER_CTL] = 0x08,
		[PLL_OFF_USER_CTL_U] = 0x0c,
		[PLL_OFF_CONFIG_CTL] = 0x10,
		[PLL_OFF_TEST_CTL] = 0x14,
		[PLL_OFF_TEST_CTL_U] = 0x18,
		[PLL_OFF_STATUS] = 0x1c,
		[PLL_OFF_ALPHA_VAL] = 0x24,
		[PLL_OFF_ALPHA_VAL_U] = 0x28,
	},
	[CLK_ALPHA_PLL_TYPE_ZONDA_OLE] =  {
		[PLL_OFF_L_VAL] = 0x04,
		[PLL_OFF_ALPHA_VAL] = 0x08,
		[PLL_OFF_USER_CTL] = 0x0c,
		[PLL_OFF_USER_CTL_U] = 0x10,
		[PLL_OFF_CONFIG_CTL] = 0x14,
		[PLL_OFF_CONFIG_CTL_U] = 0x18,
		[PLL_OFF_CONFIG_CTL_U1] = 0x1c,
		[PLL_OFF_CONFIG_CTL_U2] = 0x20,
		[PLL_OFF_TEST_CTL] = 0x24,
		[PLL_OFF_TEST_CTL_U] = 0x28,
		[PLL_OFF_TEST_CTL_U1] = 0x2c,
		[PLL_OFF_OPMODE] = 0x30,
		[PLL_OFF_STATUS] = 0x3c,
	},
	[CLK_ALPHA_PLL_TYPE_NSS_HUAYRA] =  {
		[PLL_OFF_L_VAL] = 0x04,
		[PLL_OFF_ALPHA_VAL] = 0x08,
		[PLL_OFF_TEST_CTL] = 0x0c,
		[PLL_OFF_TEST_CTL_U] = 0x10,
		[PLL_OFF_USER_CTL] = 0x14,
		[PLL_OFF_CONFIG_CTL] = 0x18,
		[PLL_OFF_CONFIG_CTL_U] = 0x1c,
		[PLL_OFF_STATUS] = 0x20,
	},

};
EXPORT_SYMBOL_GPL(clk_alpha_pll_regs);

#define	ALPHA_PLL_STATUS_REG_SHIFT	8

#define PLL_HUAYRA_M_WIDTH		8
#define PLL_HUAYRA_M_SHIFT		8
#define PLL_HUAYRA_M_MASK		0xff
#define PLL_HUAYRA_N_SHIFT		0
#define PLL_HUAYRA_N_MASK		0xff
#define PLL_HUAYRA_ALPHA_WIDTH		16

/* TRION PLL specific settings and offsets */
#define TRION_PLL_CAL_VAL	0x44
#define TRION_PCAL_DONE		BIT(26)

/* LUCID PLL specific settings and offsets */
#define LUCID_PCAL_DONE		BIT(27)

/* LUCID 5LPE PLL specific settings and offsets */
#define LUCID_5LPE_PCAL_DONE		BIT(11)
#define LUCID_5LPE_ALPHA_PLL_ACK_LATCH	BIT(13)
#define LUCID_5LPE_PLL_LATCH_INPUT	BIT(14)
#define LUCID_5LPE_ENABLE_VOTE_RUN	BIT(21)

/* PONGO ELU PLL specific setting and offsets */
#define PONGO_PLL_OUT_MASK		GENMASK(1, 0)
#define PONGO_PLL_L_VAL_MASK		GENMASK(11, 0)
#define PONGO_XO_PRESENT		BIT(10)
#define PONGO_CLOCK_SELECT		BIT(12)

/* ZONDA PLL specific */
#define ZONDA_PLL_OUT_MASK	0xf
#define ZONDA_STAY_IN_CFA	BIT(16)
#define ZONDA_PLL_FREQ_LOCK_DET	BIT(29)

#endif /* #if 0 */

#endif	/* __QCOM_CLK_APLL_REG_H__ */
