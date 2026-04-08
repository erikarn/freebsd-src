/*-
 * Copyright (c) 2026 Adrian Chadd <adrian@FreeBSD.org>.
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/clk/clk_div.h>
#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_mux.h>

#include "qcom_clk_apll.h"
#include "qcom_clk_apll_var.h"
#include "qcom_clk_apll_trion.h"
#include "qcom_clk_apll_reg.h"

#include "clkdev_if.h"

/**
 * @brief Return if the clock is enabled
 */
static bool
qcom_clk_apll_trion_is_enabled(struct clknode *clk)
{
	device_t dev = clknode_get_device(clk);
	struct qcom_clk_apll_sc *sc = clknode_get_softc(clk);
	uint32_t mode_val, opmode_val;
	int ret;

	ret = CLKDEV_READ_4(dev, QCOM_APLL_HW_REG_PLL_MODE(sc), &mode_val);
	if (ret)
		return (false);
	ret = CLKDEV_READ_4(dev, QCOM_APLL_HW_REG_PLL_OPMODE(sc), &opmode_val);
	if (ret)
		return (false);

	return ((opmode_val & PLL_RUN) && (mode_val & PLL_OUTCTRL));
}

int
qcom_clk_apll_trion_recalc(struct clknode *clk, uint64_t *freq)
{
#if 0
	struct qcom_clk_apll_sc *sc;
	sc = clknode_get_softc(clk);
#endif

	printf("%s: called, TODO\n", __func__);
	return (ENXIO);
}

int
qcom_clk_apll_trion_init(struct clknode *clk)
{
#if 0
	struct qcom_clk_apll_sc *sc;
	sc = clknode_get_softc(clk);
#endif

	printf("%s: TODO\n", __func__);

	/*
	 * There's only a single parent here for an fixed divisor,
	 * so just set it to 0; the caller doesn't need to supply it.
	 */
	clknode_init_parent_idx(clk, 0);

	return (0);
}

int
qcom_clk_apll_trion_set_gate(struct clknode *clk, bool enable)
{
	printf("%s: TODO\n", __func__);
	return (ENXIO);
}

int
qcom_clk_apll_trion_get_gate(struct clknode *clk, bool *enable)
{
	bool ret;

	ret = qcom_clk_apll_trion_is_enabled(clk);
	*enable = ret;
	return (0);
}

/*
 * Set frequency
 *
 * fin - the parent frequency, if exists
 * fout - starts as the requested frequency, ends with the configured
 *        or dry-run frequency
 * Flags - CLK_SET_DRYRUN, CLK_SET_ROUND_UP, CLK_SET_ROUND_DOWN
 * retval - 0, ERANGE
 */
int
qcom_clk_apll_trion_set_freq(struct clknode *clk, uint64_t fin,
    uint64_t *fout, int flags, int *stop)
{
	printf("%s: TODO\n", __func__);
	return (ENXIO);
}

struct qcom_clk_apll_ops qcom_clk_apll_ops_trion = {
	.op_init = qcom_clk_apll_trion_init,
	.op_recalc = qcom_clk_apll_trion_recalc,
	.op_set_gate = qcom_clk_apll_trion_set_gate,
	.op_get_gate = qcom_clk_apll_trion_get_gate,
	.op_set_freq = qcom_clk_apll_trion_set_freq,
};
