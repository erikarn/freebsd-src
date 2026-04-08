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

#ifndef	__QCOM_CLK_APLL_H__
#define	__QCOM_CLK_APLL_H__

typedef enum {
	QCOM_CLK_APLL_TYPE_FIXED_LUCID_EVO = 1,
	QCOM_CLK_APLL_TYPE_POSTDIV_LUCID_EVO = 2,
	QCOM_CLK_APLL_TYPE_LUCID_EVO = 3,

	QCOM_CLK_APLL_TYPE_FIXED_LUCID_OLE = 4,
	QCOM_CLK_APLL_TYPE_POSTDIV_LUCID_OLE = 5,
} qcom_clk_apll_type_t;

typedef enum {
	PLL_OFF_L_VAL,
	PLL_OFF_CAL_L_VAL,
	PLL_OFF_ALPHA_VAL,
	PLL_OFF_ALPHA_VAL_U,
	PLL_OFF_USER_CTL,
	PLL_OFF_USER_CTL_U,
	PLL_OFF_USER_CTL_U1,
	PLL_OFF_CONFIG_CTL,
	PLL_OFF_CONFIG_CTL_U,
	PLL_OFF_CONFIG_CTL_U1,
	PLL_OFF_CONFIG_CTL_U2,
	PLL_OFF_TEST_CTL,
	PLL_OFF_TEST_CTL_U,
	PLL_OFF_TEST_CTL_U1,
	PLL_OFF_TEST_CTL_U2,
	PLL_OFF_TEST_CTL_U3,
	PLL_OFF_STATE,
	PLL_OFF_STATUS,
	PLL_OFF_OPMODE,
	PLL_OFF_FRAC,
	PLL_OFF_CAL_VAL,
	PLL_OFF_MAX_REGS
} qcom_clk_apll_regmap_idx_t;

// XXX TODO
struct clk_div_table;

struct qcom_clk_apll_def {
	struct clknode_init_def clkdef;
	qcom_clk_apll_type_t apll_type;

	/* Offset of register inside clock controller register space */
	uint32_t reg_offset;

	/* enable offset/flag */
	uint32_t enable_offset;
	uint32_t enable_shift;

	/* post-div clocks */
	uint32_t post_div_width;
	uint32_t post_div_shift;
	const struct clk_div_table *post_div_table;
};

#define F_APLL_LUCID_OLE_FIXED(_id, _cname, _parent, _roffset,		\
	_eoffset, _eshift)						\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _cname,						\
	.clkdef.parent_names = (const char *[]){_parent},		\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.reg_offset = _roffset,						\
	.enable_offset = _eoffset,					\
	.enable_shift = _eshift,					\
	.apll_type = QCOM_CLK_APLL_TYPE_FIXED_LUCID_OLE,		\
}

#define F_APLL_LUCID_OLE_POSTDIV(_id, _cname, _parent, _roffset,	\
	_post_div_shift, _post_div_width, _post_div_table)		\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _cname,						\
	.clkdef.parent_names = (const char *[]){_parent},		\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.reg_offset = _roffset,						\
	.post_div_shift = _post_div_shift,				\
	.post_div_width = _post_div_width,				\
	.post_div_table = _post_div_table,				\
	.apll_type = QCOM_CLK_APLL_TYPE_POSTDIV_LUCID_OLE,		\
}

extern	int qcom_clk_apll_register(struct clkdom *clkdom,
	    struct qcom_clk_apll_def *clkdef);

#endif	/* __QCOM_CLK_APLL_H__ */
