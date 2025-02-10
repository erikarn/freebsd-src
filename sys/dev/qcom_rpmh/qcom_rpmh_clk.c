/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025, Adrian Chadd <adrian@FreeBSD.org>
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

/* Driver for Qualcomm RPMH clock, found in Snapdragon SoCs */

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

#include "clkdev_if.h"

#include "qcom_rpmh_clk_var.h"

static int	qcom_rpmh_clk_probe(device_t);
static int	qcom_rpmh_clk_attach(device_t);
static int	qcom_rpmh_clk_detach(device_t);

struct qcom_rpmh_clk_chipset_list_entry {
	const char *ofw;
	const char *desc;
	qcom_rpmh_clk_chipset_t chipset;
};

static struct qcom_rpmh_clk_chipset_list_entry qcom_rpmh_clk_chipset_list[] = {
	{ "qcom,x1e80100-rpmh-clk",
	    "Qualcomm Snapdragon X1E80100 RPMH Clock Controller",
	    QCOM_RPMH_CHIPSET_X1E80100 },
	{ NULL, NULL, 0 },
};

static int
qcom_rpmh_clk_modevent(module_t mod, int type, void *unused)
{
	int error;

	switch (type) {
	case MOD_LOAD:
	case MOD_QUIESCE:
	case MOD_UNLOAD:
	case MOD_SHUTDOWN:
		error = 0;
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}

	return (error);
}

static int
qcom_rpmh_clk_probe(device_t dev)
{
	struct qcom_rpmh_clk_softc *sc;
	int i;

	sc = device_get_softc(dev);

	if (! ofw_bus_status_okay(dev))
		return (ENXIO);

	for (i = 0; qcom_rpmh_clk_chipset_list[i].ofw != NULL; i++) {
		const struct qcom_rpmh_clk_chipset_list_entry *ce;

		ce = &qcom_rpmh_clk_chipset_list[i];
		if (ofw_bus_is_compatible(dev, ce->ofw) == 0)
			continue;
		device_set_desc(dev, ce->desc);
		sc->sc_chipset = ce->chipset;
		device_printf(dev, "%s: found!\n", __func__);
		return (0);
	}

	return (ENXIO);
}

static int
qcom_rpmh_clk_attach(device_t dev)
{
	struct qcom_rpmh_clk_softc *sc;

	sc = device_get_softc(dev);

	device_printf(dev, "%s: called!\n", __func__);
	/* Found a compatible device! */
	sc->dev = dev;

#if 1
	/*
	 * Setup the hardware callbacks, before any further initialisation
	 * is performed.
	 */
	switch (sc->sc_chipset) {
	case QCOM_RPMH_CHIPSET_X1E80100:
		qcom_rpmh_x1e80100_init(sc);
		break;
	case QCOM_RPMH_CHIPSET_NONE:
		device_printf(dev, "Invalid chipset (%d)\n", sc->sc_chipset);
		return (ENXIO);
	}
#endif

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

#if 0
	/*
	 * Setup and register as a clock provider.
	 */
	switch (sc->sc_chipset) {
	case QCOM_RPMH_CHIPSET_X1E80100:
		qcom_rpmh_x1e80100_clock_setup(sc);
		break;
	case QCOM_RPMH_CHIPSET_NONE:
		device_printf(dev, "Invalid chipset (%d)\n", sc->sc_chipset);
		return (ENXIO);
	}
#endif

	return (0);
}

static int
qcom_rpmh_clk_detach(device_t dev)
{
	struct qcom_rpmh_clk_softc *sc;

	sc = device_get_softc(dev);

	(void) sc;

	/*
	 * TBD - deregistering clock resources.
	 */

	return (0);
}

static device_method_t qcom_rpmh_clk_methods[] = {
	/* Device methods. */
	DEVMETHOD(device_probe,		qcom_rpmh_clk_probe),
	DEVMETHOD(device_attach,	qcom_rpmh_clk_attach),
	DEVMETHOD(device_detach,	qcom_rpmh_clk_detach),

	DEVMETHOD_END
};

static driver_t qcom_rpmh_clk_driver = {
	"qcom_rpmh_clk",
	qcom_rpmh_clk_methods,
	sizeof(struct qcom_rpmh_clk_softc)
};

EARLY_DRIVER_MODULE(qcom_rpmh_clk, simplebus, qcom_rpmh_clk_driver,
    qcom_rpmh_clk_modevent, NULL, BUS_PASS_CPU + BUS_PASS_ORDER_EARLY + 1);
EARLY_DRIVER_MODULE(qcom_rpmh_clk, ofwbus, qcom_rpmh_clk_driver,
    qcom_rpmh_clk_modevent, NULL, BUS_PASS_CPU + BUS_PASS_ORDER_EARLY + 1);
MODULE_VERSION(qcom_rpmh_clk, 1);
