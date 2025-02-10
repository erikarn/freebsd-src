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

/* Driver for Qualcomm RPM, found in Snapdragon SoCs */

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

#include <dev/fdt/simplebus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "clkdev_if.h"

#include "qcom_rpmh_var.h"

static int	qcom_rpmh_modevent(module_t, int, void *);

static int	qcom_rpmh_probe(device_t);
static int	qcom_rpmh_attach(device_t);
static int	qcom_rpmh_detach(device_t);

static int
qcom_rpmh_modevent(module_t mod, int type, void *unused)
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
qcom_rpmh_probe(device_t dev)
{
	struct qcom_rpmh_softc *sc;

	sc = device_get_softc(dev);
	(void) sc;

	if (! ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_is_compatible(dev, "qcom,rpmh-rsc") != 1)
		return (ENXIO);

	device_set_desc(dev, "Qualcomm Snapdragon RPMH bus/provider");
	device_printf(dev, "%s: found!\n", __func__);
	return (0);
}

static int
qcom_rpmh_attach(device_t dev)
{
	struct qcom_rpmh_softc *sc;
	phandle_t node, child;

	sc = device_get_softc(dev);

	device_printf(dev, "%s: called!\n", __func__);

	/* Found a compatible device! */
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	node = ofw_bus_get_node(dev);
	simplebus_init(dev, node);
	for (child = OF_child(node); child > 0; child = OF_peer(child)) {
		simplebus_add_device(dev, child, 0, NULL, -1, NULL);
	}
	bus_attach_children(dev);

	return (0);
}

static int
qcom_rpmh_detach(device_t dev)
{
	struct qcom_rpmh_softc *sc;
	int error;

	sc = device_get_softc(dev);

	error = bus_generic_detach(dev);
	if (error)
		return (error);

	(void) sc;

	/* TBD - deregistering child/bus */

	/* TBD - deregister resources */

	return (0);
}

static device_method_t qcom_rpmh_methods[] = {
	/* Device methods. */
	DEVMETHOD(device_probe,		qcom_rpmh_probe),
	DEVMETHOD(device_attach,	qcom_rpmh_attach),
	DEVMETHOD(device_detach,	qcom_rpmh_detach),

	DEVMETHOD_END
};

static driver_t qcom_rpmh_driver = {
	"qcom_rpmh",
	qcom_rpmh_methods,
	sizeof(struct qcom_rpmh_softc)
};

EARLY_DRIVER_MODULE(qcom_rpmh, simplebus, qcom_rpmh_driver,
    qcom_rpmh_modevent, NULL, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
EARLY_DRIVER_MODULE(qcom_rpmh, ofwbus, qcom_rpmh_driver,
    qcom_rpmh_modevent, NULL, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(qcom_rpmh, 1);
