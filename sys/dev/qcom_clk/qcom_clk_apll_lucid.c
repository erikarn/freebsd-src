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
#include "qcom_clk_apll_reg.h"
#include "qcom_clk_apll_utils.h"
#include "qcom_clk_apll_lucid.h"
#include "qcom_clk_apll_trion.h"

#include "clkdev_if.h"

static uint32_t qcom_clk_apll_lucid_ole_regmap[] = {
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
};

static int
qcom_clk_apll_lucid_evo_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	struct qcom_clk_apll_sc *sc = clknode_get_softc(clk);
	device_t dev = clknode_get_device(clk);
	uint32_t l, a;
	uint64_t parent_freq, my_freq;
	int ret, width;

	device_printf(dev, "%s: called\n", __func__);

	ret = CLKDEV_READ_4(dev, QCOM_CLK_APLL_HW_REG_PLL_L_VAL(sc), &l);
	if (ret != 0)
		return (ret);
	l &= LUCID_EVO_PLL_L_VAL_MASK;
	ret = CLKDEV_READ_4(dev, QCOM_CLK_APLL_HW_REG_PLL_ALPHA_VAL(sc), &a);
	if (ret != 0)
		return (ret);

	/*
	 * TODO: I'm not sure why it's doing this.
	 *
	 * My guess is its handling 32 vs 64 bit registers and
	 * honestly at this point it should be a hardware quirk in ops
	 * rather than this.
	 *
	 * TODO: yeah, stick this into utils.c.
	 */
	if (QCOM_CLK_APLL_HW_REG_PLL_ALPHA_VAL_U(sc) - QCOM_CLK_APLL_HW_REG_PLL_ALPHA_VAL(sc) == 4)
		width = QCOM_CLK_APLL_ALPHA_REG_BITWIDTH;
	else
		width = QCOM_CLK_APLL_ALPHA_REG_16BIT_WIDTH;

	parent_freq = *freq;
	my_freq = qcom_clk_apll_calc_freq(parent_freq, l, a, width);

	device_printf(dev, "%s: parent_freq=%llu, my_freq=%llu\n",
	    __func__, (unsigned long long) parent_freq,
	    (unsigned long long) my_freq);
	*freq = my_freq;
	return (0);
}

int
qcom_clk_apll_lucid_init(struct clknode *clk)
{
	device_t dev = clknode_get_device(clk);
#if 0
	struct qcom_clk_apll_sc *sc;
	sc = clknode_get_softc(clk);
#endif

	device_printf(dev, "%s: called\n", __func__);

	/*
	 * There's only a single parent here for an fixed divisor,
	 * so just set it to 0; the caller doesn't need to supply it.
	 */
	clknode_init_parent_idx(clk, 0);

	return (0);
}

struct qcom_clk_apll_ops qcom_clk_apll_ops_lucid_ole = {
	.regmap = qcom_clk_apll_lucid_ole_regmap,
	.op_init = qcom_clk_apll_lucid_init,
	.op_recalc = qcom_clk_apll_lucid_evo_recalc_freq,
//	.op_set_freq = qcom_clk_apll_trion_set_freq,
	.op_get_gate = qcom_clk_apll_trion_get_gate,
//	.op_set_gate = qcom_clk_apll_trion_set_gate,
};
