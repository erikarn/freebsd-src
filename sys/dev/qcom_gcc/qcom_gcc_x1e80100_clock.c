/*-
 * Copyright (c) 2025 Adrian Chadd <adrian@FreeBSD.org>
 * All rights reserved.
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
#include <sys/kthread.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/clk/clk_link.h>

#include <dev/qcom_clk/qcom_clk_nodeinst.h>
#include <dev/qcom_clk/qcom_clk_apll.h>

#include <dt-bindings/clock/qcom,x1e80100-gcc.h>

#include "qcom_gcc_var.h"
#include "qcom_gcc_x1e80100.h"

#define	CBCR_CLOCK_ENABLE		0x00000001

static void
qcom_gcc_x1e80100_branch_set_clk_en(struct qcom_gcc_softc *sc, uint32_t cbcr)
{
	uint32_t reg;

	reg = bus_read_4(sc->reg, cbcr);
	reg |= CBCR_CLOCK_ENABLE;
	bus_write_4(sc->reg, cbcr, reg);
}

/*
 * link clocks - xo_board ? Is that what DT_BI_TCXO refers to?
 */
static struct clk_link_def qcom_gcc_x1e80100_link_clks[] = {
	F_LINK(0, "xo-board"),
};

/*
 * apll_lucid_ole_fixed clocks:
 *
 * gcc_gpll0 / 0x52030, bit 0, DT_BI_TCXO / xo_board, GCC_GPLL0
 * gcc_gpll4 / 0x52030, bit 4, DT_BI_TCXO / xo_board, GCC_GPLL4
 * gcc_gpll7 / 0x52030, bit 7, DT_BI_TCXO / xo_board, GCC_GPLL7
 * gcc_gpll8 / 0x52030, bit 8, DT_BI_TCXO / xo_board, GCC_GPLL8
 * gcc_gpll9 / 0x52030, bit 9, DT_BI_TCXO / xo_board, GCC_GPLL9
 */
static struct qcom_clk_apll_def qcom_gcc_x1e80100_apll_fixed_clks[] = {
	F_APLL_LUCID_OLE_FIXED(GCC_GPLL0, "gcc_gpll0", "xo-board", 0x0000,
	    0x52030, 0),
	F_APLL_LUCID_OLE_FIXED(GCC_GPLL0, "gcc_gpll4", "xo-board", 0x4000,
	    0x52030, 4),
	F_APLL_LUCID_OLE_FIXED(GCC_GPLL0, "gcc_gpll7", "xo-board", 0x7000,
	    0x52030, 7),
	F_APLL_LUCID_OLE_FIXED(GCC_GPLL0, "gcc_gpll8", "xo-board", 0x8000,
	    0x52030, 8),
	F_APLL_LUCID_OLE_FIXED(GCC_GPLL0, "gcc_gpll9", "xo-board", 0x9000,
	    0x52030, 9),
};

static void
qcom_gcc_x1e80100_register_link_clocks(struct qcom_gcc_softc *sc)
{
	int i, rv;

	for (i = 0; i < nitems(qcom_gcc_x1e80100_link_clks); i++) {
		rv = clknode_link_register(sc->clkdom,
		    &qcom_gcc_x1e80100_link_clks[i]);
		if (rv != 0) {
			device_printf(sc->dev,
			    "%s: failed to register link clock (%s) - %d\n",
			    __func__,
			    qcom_gcc_x1e80100_link_clks[i].clkdef.name, rv);
		}
	}
}

static void
qcom_gcc_x1e80100_register_apll_lucid_ole_fixed_clocks(struct qcom_gcc_softc *sc)
{
	int i, rv;

	for (i = 0; i < nitems(qcom_gcc_x1e80100_apll_fixed_clks); i++) {
		rv = qcom_clk_apll_register(sc->clkdom,
		    &qcom_gcc_x1e80100_apll_fixed_clks[i]);
		if (rv != 0) {
			device_printf(sc->dev,
			    "%s: failed to register apll fixed clock (%s) - %d\n",
			    __func__,
			    qcom_gcc_x1e80100_apll_fixed_clks[i].clkdef.name, rv);
		}
	}
}

/*
 * apll_lucid_ole_postdiv clocks:
 *
 * gcc_gpll0_out_even / GCC_GPLL0_OUT_EVEN
 *
 * TODO: this is the only one, but it has a divisor table, register
 * array, shifts?  I need to go see exactly what this clock is
 * actually doing versus the apll_lucid_ole_fixed clock.
 */

void
qcom_gcc_x1e80100_clock_setup(struct qcom_gcc_softc *sc)
{
	device_printf(sc->dev, "%s: called\n", __func__);

	sc->clkdom = clkdom_create(sc->dev);

	/* Keep some clocks always on */

	/*
	 * qcom_branch_set_clk_en() on:
	 *
	 * 0x26004 - GCC_CAMERA_AHB_CLK
	 * 0x26028 - GCC_CAMERA_XO_CLK
	 * 0x27004 - GCC_DISP_AHB_CLK
	 * 0x27018 - GCC_DISP_XO_CLK
	 * 0x32004 - GCC_VIDEO_AHB_CLK
	 * 0x32030 - GCC_VIDEO_XO_CLK
	 * 0x71004 - GCC_CPU_CFG_AHB_CLK
	 */
	qcom_gcc_x1e80100_branch_set_clk_en(sc, 0x26004);
	qcom_gcc_x1e80100_branch_set_clk_en(sc, 0x26028);
	qcom_gcc_x1e80100_branch_set_clk_en(sc, 0x27004);
	qcom_gcc_x1e80100_branch_set_clk_en(sc, 0x27018);
	qcom_gcc_x1e80100_branch_set_clk_en(sc, 0x32004);
	qcom_gcc_x1e80100_branch_set_clk_en(sc, 0x32030);
	qcom_gcc_x1e80100_branch_set_clk_en(sc, 0x71004);

	/*
	 * Clear GDSC_SLEEP_ENA_VOTE to stop votes being auto-removed
	 * in sleep.
	 */

	bus_write_4(sc->reg, 0x52224, 0x0);

	/* Register the link clocks */
	qcom_gcc_x1e80100_register_link_clocks(sc);

	/* TODO: register RCG DFS clocks */

	/* TODO: register the rest of the clock tree */
	qcom_gcc_x1e80100_register_apll_lucid_ole_fixed_clocks(sc);

	clkdom_finit(sc->clkdom);
}
