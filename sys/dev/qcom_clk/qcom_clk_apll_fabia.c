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
#include "qcom_clk_apll_fabia.h"
#include "qcom_clk_apll_reg.h"

#include "clkdev_if.h"

/**
 * @brief Recalculate rate based on parent frequency and div config.
 */
int
qcom_clk_apll_postdiv_fabia_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	device_t dev = clknode_get_device(clk);
	struct qcom_clk_apll_sc *sc = clknode_get_softc(clk);
	uint32_t val;
	int ret, div, i;

	ret = CLKDEV_READ_4(dev, QCOM_CLK_APLL_HW_PLL_USER_CTL(sc), &val);

	if (ret != 0)
		return (ret);

	val = val >> sc->post_div_shift;
	val &= ((1 << sc->post_div_width) - 1);

	div = 1;

	for (i = 0; sc->post_div_table[i].divider != 0; i++) {
		if (val == sc->post_div_table[i].value) {
			div = sc->post_div_table[i].divider;
			break;
		}
	}

	*freq = *freq / div;
	return (0);
}
