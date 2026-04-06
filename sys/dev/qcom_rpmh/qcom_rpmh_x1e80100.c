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

/* Driver for Qualcomm RPMH X1E80100 specific clocks */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/sglist.h>
#include <sys/random.h>
#include <sys/stdatomic.h>
#include <sys/mutex.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/rman.h>
#include <sys/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_link.h>
#include <dev/clk/clk_div.h>

#include <dt-bindings/clock/qcom,rpmh.h>

#include <dev/qcom_clk/qcom_clk_rpmh.h>
#include <dev/qcom_clk/qcom_clk_rpmh_bcm.h>

#include "clkdev_if.h"

#include "qcom_rpmh_clk_var.h"

#define	QCOM_CLK_RPMH_ARC_EN_OFFSET		0
#define	QCOM_CLK_RPMH_VRM_EN_OFFSET		4

#define	F_BCM(_id, _name, _res_name)					\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _name,						\
	.clkdef.parent_names = NULL,					\
	.clkdef.parent_cnt = 0,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.valid_state_mask = (1 << QCOM_CLK_RPMH_BCM_ACTIVE_ONLY_STATE),	\
	.div = 1,							\
}

/*
 * Each RPMH clock definition is actually two clocks -
 * + the normal clock name;
 * + the same clock name with _ao appended (always on).
 *
 * The linux code uses pointers to the clock def structs; since we don't
 * have that here, I'll have to include the cross-referenced names here and
 * then eventually figure out how to do a post-setup tree walk to
 * look up each others clock nodes.
 *
 * TODO: all of the above
 *
 * TODO: the linux code references xo and xo_board; will need to figure
 * that mess out too.
 */
#define	F_RPMH_NORMAL(_id, _name, _peer_name, _res_name, _res_en_offset, _res_on, _div)	\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _name,						\
	.clkdef.parent_names = (const char *[]){ "xo-board" },		\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.valid_state_mask = (1 << QCOM_CLK_RPMH_WAKE_ONLY_STATE) |	\
		(1 << QCOM_CLK_RPMH_ACTIVE_ONLY_STATE) |		\
		(1 << QCOM_CLK_RPMH_SLEEP_STATE),			\
	.peer_name = _peer_name,					\
	.res_name = _res_name,						\
	.res_addr = _res_en_offset,					\
	.res_on_val = _res_on,						\
	.div = _div,							\
}

#define	F_RPMH_AO(_id, _name, _peer_name, _res_name, _res_en_offset, _res_on, _div)	\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _name,						\
	.clkdef.parent_names = (const char *[]){ "xo-board" },		\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.valid_state_mask = (1 << QCOM_CLK_RPMH_WAKE_ONLY_STATE) |	\
		(1 << QCOM_CLK_RPMH_ACTIVE_ONLY_STATE),			\
	.peer_name = _peer_name,					\
	.res_name = _res_name,						\
	.res_addr = _res_en_offset,					\
	.res_on_val = _res_on,						\
	.div = _div,							\
}

#define	F_RPMH_LINK(_name)						\
{									\
	.clkdef.id = 0,							\
	.clkdef.name = _name,						\
	.clkdef.parent_names = NULL,					\
	.clkdef.parent_cnt = 0,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
}

/* TODO: this isn't technically true; need to do the x1e specific ones once i know this compiles */
/* double TODO: BCM clocks don't exist on x1e */
static struct qcom_clk_rpmh_bcm_def bcm_clks[] = {
	F_BCM(RPMH_CE_CLK, "ce", "CE0"),
	/* hwkm, HK0 */
	F_BCM(RPMH_HWKM_CLK, "hwkm", "HK0"),
	/* ipa, IP0 */
	F_BCM(RPMH_IPA_CLK, "ipa", "IP0"),
	/* pka, PKA0 */
	F_BCM(RPMH_PKA_CLK, "pka", "PKA0"),
	/* qpic_clk, QP0 */
	F_BCM(RPMH_QPIC_CLK, "qpic_clk", "QP0"),
};

static struct clk_link_def rpmh_link_clks[] = {
	F_RPMH_LINK("xo-board"),
};

/*
 * Ok, these are the x1e80100 clocks
 */
static struct qcom_clk_rpmh_def rpmh_clks[] = {
	/*
	 * TODO: These first two will need some further digging -
	 * the device tree actually defines them as fixed-factor-clock with
	 * divisors, so should we actually also register them by name
	 * here?  Especially since this (and linux) looks like it's
	 * registering them as divide by 2 clocks, but the device tree
	 * is ALSO defining them as divide by 2, so will it be div-4 ?
	 */
	F_RPMH_NORMAL(RPMH_CXO_CLK, "bi_tcxo_div2", "xo.lvl", "bi_tcxo_ao_div2", QCOM_CLK_RPMH_ARC_EN_OFFSET, 0x3, 2),
	F_RPMH_AO(RPMH_CXO_CLK_A, "bi_tcxo_ao_div2", "xo.lvl", "bi_tcxo_div2", QCOM_CLK_RPMH_ARC_EN_OFFSET, 0x3, 2),
};

int
qcom_rpmh_x1e80100_init(struct qcom_rpmh_clk_softc *sc)
{
	int i, rv;

	(void) bcm_clks;
	device_printf(sc->dev, "%s: called\n", __func__);

	sc->clkdom = clkdom_create(sc->dev);

	for (i = 0; i < nitems(rpmh_link_clks); i++) {
		device_printf(sc->dev, "%s: registering link clock %i (%s)\n",
		    __func__, i, rpmh_link_clks[i].clkdef.name);
		rv = clknode_link_register(sc->clkdom, &rpmh_link_clks[i]);
		if (rv != 0) {
			device_printf(sc->dev, "%s: failed to register clock\n", __func__);
		}
	}

	for (i = 0; i < nitems(rpmh_clks); i++) {
		device_printf(sc->dev, "%s: registering clock %i (%s)\n",
		    __func__, i, rpmh_clks[i].clkdef.name);
		rv = qcom_clk_rpmh_register(sc->clkdom, rpmh_clks + i);
		if (rv != 0) {
			printf("%s: register (%s) failed - %d\n", __func__,
			    rpmh_clks[i].clkdef.name, rv);
		}
	}

	clkdom_finit(sc->clkdom);

	return (0);
}
