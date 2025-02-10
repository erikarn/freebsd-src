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

#ifndef	__QCOM_CLK_RPMH_BCM_H__
#define	__QCOM_CLK_RPMH_BCM_H__

#include "qcom_clk_freqtbl.h"

typedef enum {
	QCOM_CLK_RPMH_BCM_SLEEP_STATE = 0,		/* Nothing using it */
	QCOM_CLK_RPMH_BCM_WAKE_ONLY_STATE = 1,	/* Resume pre power-down */
	QCOM_CLK_RPMH_BCM_ACTIVE_ONLY_STATE = 2,	/* Active/AMC mode */
} qcom_rpmh_bcm_state_t;

struct qcom_clk_rpmh_bcm_def {
	struct clknode_init_def clkdef;

	uint8_t div;
	const char *res_name;
	uint32_t res_addr;
	uint32_t res_on_val;
	uint32_t state;
	uint32_t aggr_state;
	uint32_t last_sent_aggr_state;
	uint32_t valid_state_mask;
	uint32_t unit;
	const char *peer_name;

	/* TODO: peer/sibling clock pointer */

	const struct qcom_clk_freq_tbl *freq_tbl;
};

extern	int qcom_clk_rpmh_bcm_register(struct clkdom *clkdom,
	    struct qcom_clk_rpmh_bcm_def *clkdef);

#endif	/* __QCOM_CLK_RPMH_BCM_H__ */
