/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021, Adrian Chadd <adrian@FreeBSD.org>
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
#include <dev/qcom_tcsr/qcom_tcsr_reg.h>

/*
 * The linux-msm branches that support IPQ4018 use "ipq,tcsr".
 * The openwrt addons use qcom,tcsr.  So for now support both.
 *
 * Also, it's not quite clear yet (since this is the first port!)
 * whether these options and registers are specific to the QCA IPQ401x
 * part or show up in different linux branches as different registers
 * but with the same driver/naming here.  Let's hope that doesn't
 * happen.
 */

struct qcom_tcsr_chipset_list {
	qcom_tcsr_chipset_t id;
	const char *ofw_str;
	const char *desc_str;
	int (*attach_func)(struct qcom_tcsr_softc *);
};

static struct qcom_tcsr_chipset_list qcom_tcsr_chipsets[] = {
	{ QCOM_TCSR_CHIPSET_IPQ4018, "qcom,tcsr",
	    "Qualcomm IP401x Core Top Control and Status Driver",
	    qcom_tcsr_init_ipq4018 },
	{ QCOM_TCSR_CHIPSET_IPQ4018, "ipq,tcsr",
	    "Qualcomm IP401x Core Top Control and Status Driver",
	    qcom_tcsr_init_ipq4018 },
	{ QCOM_TCSR_CHIPSET_X1E80100, "qcom,x1e80100-tcsr",
	    "Qualcomm X1E80100 Core Top Control, Clock and Status Driver",
	    qcom_tcsr_init_x1e80100 },
	{ 0, NULL, NULL, NULL },
};

static int
qcom_tcsr_probe(device_t dev)
{
	struct qcom_tcsr_softc *sc = device_get_softc(dev);
	struct qcom_tcsr_chipset_list *ql;
	int i;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	for (i = 0; qcom_tcsr_chipsets[i].id != 0; i++) {
		ql = &qcom_tcsr_chipsets[i];
		if (ofw_bus_is_compatible(dev, ql->ofw_str) == 1) {
			sc->sc_chipset = ql->id;
			sc->sc_attach_func = ql->attach_func;
			device_set_desc(dev, ql->desc_str);
			return (BUS_PROBE_DEFAULT);
		}
	}
	return (ENXIO);
}

static int
qcom_tcsr_attach(device_t dev)
{
	struct qcom_tcsr_softc *sc = device_get_softc(dev);
	int rid, ret;

	sc->sc_dev = dev;

	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->sc_mem_res) {
		device_printf(dev, "ERROR: Could not map memory\n");
		ret = ENXIO;
		goto error;
	}

	ret = sc->sc_attach_func(sc);
	if (ret != 0)
		goto error;

	return (0);

error:
	if (sc->sc_mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem_res);
	mtx_destroy(&sc->sc_mtx);
	return (ret);
}

static int
qcom_tcsr_detach(device_t dev)
{
	struct qcom_tcsr_softc *sc = device_get_softc(dev);

	if (sc->sc_mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem_res);

	mtx_destroy(&sc->sc_mtx);

	return (0);
}

static device_method_t qcom_tcsr_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		qcom_tcsr_probe),
	DEVMETHOD(device_attach,	qcom_tcsr_attach),
	DEVMETHOD(device_detach,	qcom_tcsr_detach),

	DEVMETHOD_END
};

static driver_t qcom_tcsr_driver = {
	"qcom_tcsr",
	qcom_tcsr_methods,
	sizeof(struct qcom_tcsr_softc),
};

/*
 * This has to be run early, before the rest of the hardware is potentially
 * probed/attached.
 */
EARLY_DRIVER_MODULE(qcom_tcsr, simplebus, qcom_tcsr_driver, 0, 0,
    BUS_PASS_CPU + BUS_PASS_ORDER_EARLY);
MODULE_VERSION(qcom_tcsr, 1);
