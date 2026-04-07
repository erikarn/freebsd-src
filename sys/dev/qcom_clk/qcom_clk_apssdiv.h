/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021 Adrian Chadd <adrian@FreeBSD.org>
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

#ifndef	__QCOM_CLK_APSS_H__
#define	__QCOM_CLK_APSS_H__

#include "qcom_clk_freqtbl.h"

struct qcom_clk_apssdiv_def {
	struct clknode_init_def clkdef;
	uint32_t div_offset;
	uint32_t div_width;
	uint32_t div_shift;
	uint32_t enable_offset;
	uint32_t enable_shift;
	const struct qcom_clk_freq_tbl *freq_tbl;
};

/* APSS DIV clock */
#define F_APSSDIV(_id, _cname, _parent, _doffset, _dshift, _dwidth,	\
    _eoffset, _eshift, _freqtbl)					\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _cname,						\
	.clkdef.parent_names = (const char *[]){_parent},		\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.div_offset = _doffset,						\
	.div_width = _dwidth,						\
	.div_shift = _dshift,						\
	.enable_offset = _eoffset,					\
	.enable_shift = _eshift,					\
	.freq_tbl = _freqtbl,						\
}

extern	int qcom_clk_apssdiv_register(struct clkdom *clkdom,
	    struct qcom_clk_apssdiv_def *clkdef);

#endif	/* __QCOM_CLK_APSS_H__ */
