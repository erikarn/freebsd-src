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
#include <sys/interrupt.h>
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/gpio.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_extern.h>

#include <machine/bus.h>
#include <machine/cpu.h>

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_pinctrl.h>

#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/qcom_tcsr/qcom_tcsr_var.h>

#include <dev/clk/clk.h>
#include <dev/clk/clk_link.h>
#include <dev/qcom_clk/qcom_clk_branch2.h>
#include <dev/qcom_clk/qcom_clk_branch2_reg.h>
#include <dev/qcom_clk/qcom_clk_nodeinst.h>

#include <dt-bindings/clock/qcom,x1e80100-tcsr.h>

/*
 * The X1E80100 support actually exposes a bunch of early clock
 * branch enables.  So those need to be exported here rather
 * than reset controllers and such.
 */

static struct clk_link_def qcom_clk_tcsr_x1e80100_link_clks[] = {
	F_LINK(0, "xo-board"),
};

static struct qcom_clk_branch2_def qcom_clk_tcsr_x1e80100_branch2_clks[] = {
	F_BRANCH2_NOPARENT(TCSR_EDP_CLKREF_EN,
	    "tcsr_edp_clkref_en", 0x15130, 0, 0, 0, 0x15130, 0, false,
	    QCOM_CLK_BRANCH2_BRANCH_HALT_DELAY),

	/*
	 * TODO: In Linux, the parent is DT_BI_TCXO_PAD, which I /think/
	 * is the xo-board clock. I /think/.
	 */
	F_BRANCH2(TCSR_PCIE_2L_4_CLKREF_EN,
	    "tcsr_pcie_2l_4_clkref_en", "xo-board",
	    0x15100, 0, 0, 0, 0x15100, 0, false,
	    QCOM_CLK_BRANCH2_BRANCH_HALT_DELAY),

	F_BRANCH2(TCSR_PCIE_2L_5_CLKREF_EN,
	    "tcsr_pcie_2l_5_clkref_en", "xo-board",
	    0x15104, 0, 0, 0, 0x15104, 0, false,
	    QCOM_CLK_BRANCH2_BRANCH_HALT_DELAY),

	F_BRANCH2(TCSR_PCIE_8L_CLKREF_EN,
	    "tcsr_pcie_8l_clkref_en", "xo-board",
	    0x15108, 0, 0, 0, 0x15108, 0, false,
	    QCOM_CLK_BRANCH2_BRANCH_HALT_DELAY),

	F_BRANCH2(TCSR_USB3_MP0_CLKREF_EN,
	    "tcsr_usb3_mp0_clkref_en", "xo-board",
	    0x1510c, 0, 0, 0, 0x1510c, 0, false,
	    QCOM_CLK_BRANCH2_BRANCH_HALT_DELAY),

	F_BRANCH2(TCSR_USB3_MP1_CLKREF_EN,
	    "tcsr_usb3_mp1_clkref_en", "xo-board",
	    0x15110, 0, 0, 0, 0x15110, 0, false,
	    QCOM_CLK_BRANCH2_BRANCH_HALT_DELAY),

	F_BRANCH2(TCSR_USB2_1_CLKREF_EN,
	    "tcsr_usb2_1_clkref_en", "xo-board",
	    0x15114, 0, 0, 0, 0x15114, 0, false,
	    QCOM_CLK_BRANCH2_BRANCH_HALT_DELAY),

	F_BRANCH2(TCSR_UFS_PHY_CLKREF_EN,
	    "tcsr_ufs_phy_clkref_en", "xo-board",
	    0x15118, 0, 0, 0, 0x15118, 0, false,
	    QCOM_CLK_BRANCH2_BRANCH_HALT_DELAY),

	F_BRANCH2(TCSR_USB4_1_CLKREF_EN,
	    "tcsr_usb4_1_clkref_en", "xo-board",
	    0x15120, 0, 0, 0, 0x15120, 0, false,
	    QCOM_CLK_BRANCH2_BRANCH_HALT_DELAY),

	F_BRANCH2(TCSR_USB4_2_CLKREF_EN,
	    "tcsr_usb4_2_clkref_en", "xo-board",
	    0x15124, 0, 0, 0, 0x15124, 0, false,
	    QCOM_CLK_BRANCH2_BRANCH_HALT_DELAY),

	F_BRANCH2(TCSR_USB2_2_CLKREF_EN,
	    "tcsr_usb2_2_clkref_en", "xo-board",
	    0x15128, 0, 0, 0, 0x15128, 0, false,
	    QCOM_CLK_BRANCH2_BRANCH_HALT_DELAY),

	F_BRANCH2(TCSR_PCIE_4L_CLKREF_EN,
	    "tcsr_pcie_4l_clkref_en", "xo-board",
	    0x1512c, 0, 0, 0, 0x1512c, 0, false,
	    QCOM_CLK_BRANCH2_BRANCH_HALT_DELAY),
};

int
qcom_tcsr_init_x1e80100(struct qcom_tcsr_softc *sc)
{
	int i, rv;

	sc->sc_clkdom = clkdom_create(sc->sc_dev);


	/* Link clocks */
	for (i = 0; i < nitems(qcom_clk_tcsr_x1e80100_link_clks); i++) {
		rv = clknode_link_register(sc->sc_clkdom,
		    &qcom_clk_tcsr_x1e80100_link_clks[i]);
		if (rv != 0) {
			device_printf(sc->sc_dev,
			    "%s: failed to register link clock %d (%s) - error %d\n",
			    __func__, i,
			    qcom_clk_tcsr_x1e80100_link_clks[i].clkdef.name,
			    rv);
		}
	}

	/* branch2 clocks */
	for (i = 0; i < nitems(qcom_clk_tcsr_x1e80100_branch2_clks); i++) {
		rv = qcom_clk_branch2_register(sc->sc_clkdom,
		    &qcom_clk_tcsr_x1e80100_branch2_clks[i]);
		if (rv != 0) {
			device_printf(sc->sc_dev,
			    "%s: failed to register branch2 clock %d (%s) - error %d\n",
			    __func__, i,
			    qcom_clk_tcsr_x1e80100_branch2_clks[i].clkdef.name,
			    rv);
		}

	}

	clkdom_finit(sc->sc_clkdom);
	return (0);
}
