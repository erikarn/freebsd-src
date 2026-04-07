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

void
qcom_gcc_x1e80100_clock_setup(struct qcom_gcc_softc *sc)
{
	device_printf(sc->dev, "%s: called\n", __func__);

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

	/* TODO: register RCG DFS clocks */

	/* TODO: register the rest of the clock tree */
}
