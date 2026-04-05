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

#include "qcom_clk_freqtbl.h"
#include "qcom_clk_rpmh_bcm.h"
#include "qcom_clk_rpmh_bcm_reg.h"

#include "clkdev_if.h"

#if 0
#define DPRINTF(dev, msg...) device_printf(dev, msg);
#else
#define DPRINTF(dev, msg...)
#endif

struct qcom_clk_rpmh_bcm_sc {
	struct clknode *clknode;
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

	/* TODO: peer/sibling pointer */

	const struct qcom_clk_freq_tbl *freq_tbl;
};

static int
qcom_clk_rpmh_bcm_recalc(struct clknode *clk, uint64_t *freq)
{
	struct qcom_clk_rpmh_bcm_sc *sc;
#if 0
	uint32_t cfg, m = 0, n = 0, hid_div = 0;
	uint32_t mode = 0, mask;
#endif

	sc = clknode_get_softc(clk);

#if 0
	/* Read the MODE, CFG, M and N parameters */
	CLKDEV_DEVICE_LOCK(clknode_get_device(sc->clknode));
	CLKDEV_READ_4(clknode_get_device(sc->clknode),
	    QCOM_CLK_RCG2_CFG_OFFSET(sc),
	    &cfg);
	if (sc->mnd_width != 0) {
		mask = (1U << sc->mnd_width) - 1;
		CLKDEV_READ_4(clknode_get_device(sc->clknode),
		    QCOM_CLK_RCG2_M_OFFSET(sc), &m);
		CLKDEV_READ_4(clknode_get_device(sc->clknode),
		    QCOM_CLK_RCG2_N_OFFSET(sc), &n);
		m = m & mask;
		n = ~ n;
		n = n & mask;
		n = n + m;
		mode = (cfg & QCOM_CLK_RCG2_CFG_MODE_MASK)
		    >> QCOM_CLK_RCG2_CFG_MODE_SHIFT;
	}
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(sc->clknode));

	/* Fetch the divisor */
	mask = (1U << sc->hid_width) - 1;
	hid_div = (cfg >> QCOM_CLK_RCG2_CFG_SRC_DIV_SHIFT) & mask;

	/* Calculate the rate based on the parent rate and config */
	*freq = qcom_clk_rcg2_calc_rate(*freq, mode, m, n, hid_div);
#endif

	*freq = 0;
	printf("%s: TODO: device=%s, res=%s\n", __func__,
	    device_get_nameunit(clknode_get_device(sc->clknode)), sc->res_name);

	return (0);
}

static int
qcom_clk_rpmh_bcm_init(struct clknode *clk, device_t dev)
{
	struct qcom_clk_rpmh_bcm_sc *sc;
#if 0
	uint32_t reg;
	uint32_t idx;
	bool enabled __unused;
#endif

	sc = clknode_get_softc(clk);
#if 0
	/*
	 * Read the mux setting to set the right parent.
	 * Whilst here, read the config to get whether we're enabled
	 * or not.
	 */
	CLKDEV_DEVICE_LOCK(clknode_get_device(sc->clknode));
	/* check if rcg2 root clock is enabled */
	CLKDEV_READ_4(clknode_get_device(sc->clknode),
	    QCOM_CLK_RCG2_CMD_REGISTER(sc), &reg);
	if (reg & QCOM_CLK_RCG2_CMD_ROOT_OFF)
		enabled = false;
	else
		enabled = true;

	/* mux settings */
	CLKDEV_READ_4(clknode_get_device(sc->clknode),
	    QCOM_CLK_RCG2_CFG_OFFSET(sc), &reg);
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(sc->clknode));

	idx = (reg & QCOM_CLK_RCG2_CFG_SRC_SEL_MASK)
	    >> QCOM_CLK_RCG2_CFG_SRC_SEL_SHIFT;
	DPRINTF(clknode_get_device(sc->clknode),
	    "%s: mux index %u, enabled=%d\n",
	    __func__, idx, enabled);
	clknode_init_parent_idx(clk, idx);

	/*
	 * If we could be sure our parent clocks existed here in the tree,
	 * we could calculate our current frequency by fetching the parent
	 * frequency and then do our divider math.  Unfortunately that
	 * currently isn't the case.
	 */
#endif

	clknode_init_parent_idx(clk, 0);
	printf("%s: TODO: device=%s, res=%s\n", __func__,
	    device_get_nameunit(clknode_get_device(sc->clknode)), sc->res_name);
	return (0);
}

static clknode_method_t qcom_clk_rpmh_bcm_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		qcom_clk_rpmh_bcm_init),
	CLKNODEMETHOD(clknode_recalc_freq,	qcom_clk_rpmh_bcm_recalc),
	CLKNODEMETHOD_END
};

DEFINE_CLASS_1(qcom_clk_rpmh_bcm, qcom_clk_rpmh_bcm_class, qcom_clk_rpmh_bcm_methods,
   sizeof(struct qcom_clk_rpmh_bcm_sc), clknode_class);

int
qcom_clk_rpmh_bcm_register(struct clkdom *clkdom,
    struct qcom_clk_rpmh_bcm_def *clkdef)
{
	struct clknode *clk;
	struct qcom_clk_rpmh_bcm_sc *sc;

	/*
	 * Right now the rpmh_bcm code isn't supporting turning off the clock
	 * or limiting it to the lowest parent clock.  But, do set the
	 * flags appropriately.
	 */
#if 0
	if (clkdef->flags & QCOM_CLK_RCG2_FLAGS_CRITICAL)
#endif
		clkdef->clkdef.flags |= CLK_NODE_CANNOT_STOP;

	clk = clknode_create(clkdom, &qcom_clk_rpmh_bcm_class, &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);
	sc->clknode = clk;

	sc->div = clkdef->div;
	sc->res_name = clkdef->res_name;
	sc->res_addr = clkdef->res_addr;
	sc->res_on_val =  clkdef->res_on_val;
	sc->state = clkdef->state;
	sc->aggr_state = clkdef->aggr_state;
	sc->last_sent_aggr_state = clkdef->last_sent_aggr_state;
	sc->valid_state_mask = clkdef->valid_state_mask;
	sc->unit = clkdef->unit;
	sc->peer_name = clkdef->peer_name;

	/* TODO: peer/sibling pointer */

	clknode_register(clkdom, clk);

	return (0);
}
