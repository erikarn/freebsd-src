/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026, Adrian Chadd <adrian@FreeBSD.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Shared definitions for creating instances of qcom clock nodes */

#ifndef	__QCOM_CLK_NODEINST_H__
#define	__QCOM_CLK_NODEINST_H__

/*
 * Comment it out for now - have the callers required to include
 * what they want to use.
 */
#if 0
#include <dev/qcom_clk/qcom_clk_freqtbl.h>
#include <dev/qcom_clk/qcom_clk_fepll.h>
#include <dev/qcom_clk/qcom_clk_fdiv.h>
#include <dev/qcom_clk/qcom_clk_apssdiv.h>
#include <dev/qcom_clk/qcom_clk_rcg2.h>
#include <dev/qcom_clk/qcom_clk_branch2.h>
#include <dev/qcom_clk/qcom_clk_ro_div.h>
#endif

/* Fixed rate clock. */
#define F_RATE(_id, cname, _freq)					\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = cname,						\
	.clkdef.parent_names = NULL,					\
	.clkdef.parent_cnt = 0,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.freq = _freq,							\
}

/* Linked clock. */
#define F_LINK(_id, _cname)						\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _cname,						\
	.clkdef.parent_names = NULL,					\
	.clkdef.parent_cnt = 0,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
}


/* FEPLL clock */
#define F_FEPLL(_id, _cname, _parent, _reg, _fs, _fw, _rs, _rw)		\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _cname,						\
	.clkdef.parent_names = (const char *[]){_parent},		\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.offset = _reg,							\
	.fdbkdiv_shift = _fs,						\
	.fdbkdiv_width = _fw,						\
	.refclkdiv_shift = _rs,						\
	.refclkdiv_width = _rw,						\
}

/* Fixed divisor clock */
#define F_FDIV(_id, _cname, _parent, _divisor)				\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _cname,						\
	.clkdef.parent_names = (const char *[]){_parent},		\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.divisor = _divisor,						\
}

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

/* read-only div table */
#define	F_RO_DIV(_id, _cname, _parent, _offset, _shift, _width, _tbl)	\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _cname,						\
	.clkdef.parent_names = (const char *[]){_parent},		\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.offset = _offset,						\
	.width = _width,						\
	.shift = _shift,						\
	.div_tbl = _tbl,						\
}

/* RCG2 clock */
#define F_RCG2(_id, _cname, _parents, _rcgr, _hid_width, _mnd_width,	\
    _safe_src_idx, _safe_pre_parent_idx, _cfg_offset, _flags,		\
    _freq_tbl)								\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _cname,						\
	.clkdef.parent_names = _parents,				\
	.clkdef.parent_cnt = nitems(_parents),				\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.cmd_rcgr = _rcgr,						\
	.hid_width = _hid_width,					\
	.mnd_width = _mnd_width,					\
	.safe_src_idx = _safe_src_idx,					\
	.flags= _flags,							\
	.safe_pre_parent_idx = _safe_pre_parent_idx,			\
	.freq_tbl = _freq_tbl,						\
}

/* branch2 gate nodes */
#define	F_BRANCH2(_id, _cname, _parent, _eo, _es, _hr, _hs, _haltreg,	\
    _type, _voted, _flags)						\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _cname,						\
	.clkdef.parent_names = (const char *[]){_parent},		\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.enable_offset = _eo,						\
	.enable_shift = _es,						\
	.hwcg_reg = _hr,						\
	.hwcg_bit = _hs,						\
	.halt_reg = _haltreg,						\
	.halt_check_type = _type,					\
	.halt_check_voted = _voted,					\
	.flags = _flags,						\
}
#define	F_BRANCH2_NOPARENT(_id, _cname, _eo, _es, _hr, _hs, _haltreg,	\
    _type, _voted, _flags)						\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _cname,						\
	.clkdef.parent_names = NULL,					\
	.clkdef.parent_cnt = 0,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.enable_offset = _eo,						\
	.enable_shift = _es,						\
	.hwcg_reg = _hr,						\
	.hwcg_bit = _hs,						\
	.halt_reg = _haltreg,						\
	.halt_check_type = _type,					\
	.halt_check_voted = _voted,					\
	.flags = _flags,						\
}

#endif	/* __QCOM_CLK_NODEINST_H__ */
