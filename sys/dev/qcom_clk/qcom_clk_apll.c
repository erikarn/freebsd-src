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

#include "qcom_clk_apll_lucid.h"
#include "qcom_clk_apll_trion.h"

#include "clkdev_if.h"

/*
 * This is the collection of APLL hardware drivers in various MSM SoCs.
 * There's a bunch of different generations of APLL devices; the goal
 * of this module is to implement the ones we need via an internal HAL
 * rather than having two dozen slight variations of a theme.
 */

#if 0
#define DPRINTF(dev, msg...) device_printf(dev, "apll: " msg);
#else
#define DPRINTF(dev, msg...)
#endif

static int
qcom_clk_apll_recalc(struct clknode *clk, uint64_t *freq)
{
	struct qcom_clk_apll_sc *sc;

	sc = clknode_get_softc(clk);

	/* Check; return implemented */
	if (sc->ops->op_recalc == NULL)
		return (ENXIO);

	return (sc->ops->op_recalc(clk, freq));
}

static int
qcom_clk_apll_init(struct clknode *clk, device_t dev)
{
	struct qcom_clk_apll_sc *sc;

	sc = clknode_get_softc(clk);

	/* Check; return implemented */
	if (sc->ops->op_init == NULL)
		return (ENXIO);

	return (sc->ops->op_init(clk));
}

static int
qcom_clk_apll_set_gate(struct clknode *clk, bool enable)
{
	struct qcom_clk_apll_sc *sc;

	sc = clknode_get_softc(clk);

	/* Check; return implemented */
	if (sc->ops->op_set_gate == NULL)
		return (ENXIO);

	return (sc->ops->op_set_gate(clk, enable));
}

static int
qcom_clk_apll_get_gate(struct clknode *clk, bool *enable)
{
	struct qcom_clk_apll_sc *sc;

	sc = clknode_get_softc(clk);

	/* Check; return implemented */
	if (sc->ops->op_get_gate == NULL)
		return (ENXIO);

	return (sc->ops->op_get_gate(clk, enable));
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
static int
qcom_clk_apll_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *stop)
{
	struct qcom_clk_apll_sc *sc;

	sc = clknode_get_softc(clk);

	/* Check; return implemented */
	if (sc->ops->op_set_freq == NULL)
		return (ENXIO);

	return (sc->ops->op_set_freq(clk, fin, fout, flags, stop));
}

static clknode_method_t qcom_clk_apll_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		qcom_clk_apll_init),
	CLKNODEMETHOD(clknode_recalc_freq,	qcom_clk_apll_recalc),
	CLKNODEMETHOD(clknode_set_gate,		qcom_clk_apll_set_gate),
	CLKNODEMETHOD(clknode_get_gate,		qcom_clk_apll_get_gate),
	CLKNODEMETHOD(clknode_set_freq,		qcom_clk_apll_set_freq),

	/* XXX TODO: figure out the equivalents for these linux methods */

	/* XXX is_enabled */
	/* XXX prepare */
	/* XXX unprepare */
	CLKNODEMETHOD_END
};

DEFINE_CLASS_1(qcom_clk_apll, qcom_clk_apll_class,
    qcom_clk_apll_methods, sizeof(struct qcom_clk_apll_sc),
    clknode_class);

int
qcom_clk_apll_register(struct clkdom *clkdom,
    struct qcom_clk_apll_def *clkdef)
{
	struct clknode *clk;
	struct qcom_clk_apll_sc *sc;
	struct qcom_clk_apll_ops *ops;

	switch (clkdef->apll_type) {
	case QCOM_CLK_APLL_TYPE_FIXED_LUCID_OLE:
		ops = &qcom_clk_apll_ops_lucid_ole;
		break;
	default:
		printf("%s: unknown apll type (%d)\n", __func__,
		    clkdef->apll_type);
		return (1);
	}

	clk = clknode_create(clkdom, &qcom_clk_apll_class, &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);
	sc->clknode = clk;

	sc->reg_offset = clkdef->reg_offset;
	sc->enable_offset = clkdef->enable_offset;
	sc->enable_shift = clkdef->enable_shift;
	sc->post_div_shift = clkdef->post_div_shift;
	sc->post_div_width = clkdef->post_div_width;
	sc->post_div_table = clkdef->post_div_table;
	sc->apll_type = clkdef->apll_type;
	sc->ops = ops;

	clknode_register(clkdom, clk);

	return (0);
}
