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

struct qcom_clk_apll_sc {
	struct clknode *clknode;
	uint32_t reg_offset;
	uint32_t enable_offset;
	uint32_t enable_shift;
	qcom_clk_apll_type_t apll_type;
	const struct qcom_clk_freq_tbl *freq_tbl;
};

static int
qcom_clk_apll_recalc(struct clknode *clk, uint64_t *freq)
{
#if 0
	struct qcom_clk_apll_sc *sc;
	uint32_t reg, cdiv;

	sc = clknode_get_softc(clk);
#endif
	if (freq == NULL || *freq == 0) {
		printf("%s: called; NULL or 0 frequency\n", __func__);
		return (ENXIO);
	}

	*freq = 0;
	printf("%s: TODO\n", __func__);
	return (0);
}

static int
qcom_clk_apll_init(struct clknode *clk, device_t dev)
{

	printf("%s: TODO\n", __func__);
	/*
	 * There's only a single parent here for an fixed divisor,
	 * so just set it to 0; the caller doesn't need to supply it.
	 */
	clknode_init_parent_idx(clk, 0);

	return (0);
}

static int
qcom_clk_apll_set_gate(struct clknode *clk, bool enable)
{
#if 0
	struct qcom_clk_apssdiv_sc *sc;
	uint32_t reg;

	sc = clknode_get_softc(clk);

	if (sc->enable_offset == 0) {
		return (ENXIO);
	}

	DPRINTF(clknode_get_device(sc->clknode),
	    "%s: called; enable=%d\n", __func__, enable);

	CLKDEV_DEVICE_LOCK(clknode_get_device(sc->clknode));
	CLKDEV_READ_4(clknode_get_device(sc->clknode), sc->enable_offset,
	    &reg);
	if (enable) {
		reg |= (1U << sc->enable_shift);
	} else {
		reg &= ~(1U << sc->enable_shift);
	}
	CLKDEV_WRITE_4(clknode_get_device(sc->clknode), sc->enable_offset,
	    reg);
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(sc->clknode));
#endif
	printf("%s: TODO\n", __func__);
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
static int
qcom_clk_apll_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *stop)
{
#if 0
	const struct qcom_clk_freq_tbl *f;
	struct qcom_clk_apssdiv_sc *sc;
	uint64_t f_freq;
	uint32_t reg;

	sc = clknode_get_softc(clk);

	/* There are no further PLLs to set in this chain */
	*stop = 1;

	/* Search the table for a suitable frequency */
	f = qcom_clk_freq_tbl_lookup(sc->freq_tbl, *fout);
	if (f == NULL) {
		return (ERANGE);
	}

	/*
	 * Calculate what the resultant frequency would be based on the
	 * parent PLL.
	 */
	f_freq = qcom_clk_apssdiv_calc_rate(clk, fin, f->pre_div);

	DPRINTF(clknode_get_device(sc->clknode),
	    "%s: dryrun: %d, fin=%llu fout=%llu f_freq=%llu pre_div=%u"
	    " target_freq=%llu\n",
	    __func__,
	    !! (flags & CLK_SET_DRYRUN),
	    fin, *fout, f_freq, f->pre_div, f->freq);

	if (flags & CLK_SET_DRYRUN) {
		*fout = f_freq;
		return (0);
	}

	/*
	 * Program in the new pre-divisor.
	 */
	CLKDEV_DEVICE_LOCK(clknode_get_device(sc->clknode));
	CLKDEV_READ_4(clknode_get_device(sc->clknode), sc->div_offset, &reg);
	reg &= ~(((1U << sc->div_width) - 1) << sc->div_shift);
	reg |= (f->pre_div << sc->div_shift);
	CLKDEV_WRITE_4(clknode_get_device(sc->clknode), sc->div_offset, reg);
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(sc->clknode));

	/*
	 * The linux driver notes there's no status/completion bit to poll.
	 * So sleep for a bit and hope that's enough time for it to
	 * settle.
	 */
	DELAY(1);

	*fout = f_freq;
#endif
	printf("%s: TODO\n", __func__);
	return (ENXIO);
}

static clknode_method_t qcom_clk_apll_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		qcom_clk_apll_init),
	CLKNODEMETHOD(clknode_recalc_freq,	qcom_clk_apll_recalc),
	CLKNODEMETHOD(clknode_set_gate,		qcom_clk_apll_set_gate),
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

	clk = clknode_create(clkdom, &qcom_clk_apll_class, &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);
	sc->clknode = clk;

	sc->reg_offset = clkdef->reg_offset;
	sc->enable_offset = clkdef->enable_offset;
	sc->enable_shift = clkdef->enable_shift;
	sc->apll_type = clkdef->apll_type;

	clknode_register(clkdom, clk);

	return (0);
}
