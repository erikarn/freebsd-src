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
} qcom_clk_apll_type_t;

struct qcom_clk_apll_def {
	struct clknode_init_def clkdef;
	qcom_clk_apll_type_t apll_type;
	uint32_t reg_offset;
	uint32_t enable_offset;
	uint32_t enable_shift;
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
	.apll_type = QCOM_CLK_APLL_TYPE_FIXED_LUCID_EVO,		\
}

extern	int qcom_clk_apll_register(struct clkdom *clkdom,
	    struct qcom_clk_apll_def *clkdef);

#endif	/* __QCOM_CLK_APLL_H__ */
