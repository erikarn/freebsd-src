/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2021 Adrian Chadd <adrian@FreeBSD.org>
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/gpio.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <dev/gpio/gpiobusvar.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/qcom_ess_edma/qcom_ess_edma_var.h>
#include <dev/qcom_ess_edma/qcom_ess_edma_reg.h>
#include <dev/qcom_ess_edma/qcom_ess_edma_hw.h>

static int
qcom_ess_edma_probe(device_t dev)
{

	if (! ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_is_compatible(dev, "qcom,ess-edma") == 0)
		return (ENXIO);

	device_set_desc(dev,
	    "Qualcomm Atheross TLMM IPQ4018/IPQ4019 GPIO/Pinmux driver");
	return (0);
}

static int
qcom_ess_edma_release_intr(struct qcom_ess_edma_softc *sc,
    struct qcom_ess_edma_intr *intr)
{

	if (intr->irq_res == NULL)
		return (0);

	if (intr->irq_intr != NULL)
		bus_teardown_intr(sc->sc_dev, intr->irq_res, intr->irq_intr);
	if (intr->irq_res != NULL)
		bus_release_resource(sc->sc_dev, SYS_RES_IRQ, intr->irq_rid,
		    intr->irq_res);

	return (0);
}

static int
qcom_ess_edma_detach(device_t dev)
{
	struct qcom_ess_edma_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < QCOM_ESS_EDMA_NUM_TX_IRQS; i++) {
		(void) qcom_ess_edma_release_intr(sc, &sc->sc_tx_irq[i]);
	}
	for (i = 0; i < QCOM_ESS_EDMA_NUM_RX_IRQS; i++) {
		(void) qcom_ess_edma_release_intr(sc, &sc->sc_rx_irq[i]);
	}

	if (sc->sc_mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->sc_mem_rid,
		    sc->sc_mem_res);
	mtx_destroy(&sc->sc_mtx);

	return(0);
}

static int
qcom_ess_edma_filter(void *arg)
{
	struct qcom_ess_edma_intr *intr = arg;
	struct qcom_ess_edma_softc *sc = intr->sc;

	/* XXX TODO */

	device_printf(sc->sc_dev, "%s: called; rid=%d\n", __func__,
	    intr->irq_rid);

	return (FILTER_HANDLED);
}

static void
qcom_ess_edma_intr(void *arg)
{
	struct qcom_ess_edma_intr *intr = arg;
	struct qcom_ess_edma_softc *sc = intr->sc;

	/* XXX TODO */

	device_printf(sc->sc_dev, "%s: called; rid=%d\n", __func__,
	    intr->irq_rid);
}

static int
qcom_ess_edma_setup_intr(struct qcom_ess_edma_softc *sc,
    struct qcom_ess_edma_intr *intr, int rid)
{

	intr->sc = sc;
	intr->irq_rid = rid;
	intr->irq_res = bus_alloc_resource_any(sc->sc_dev,
	    SYS_RES_IRQ, &intr->irq_rid, RF_ACTIVE);
	if (intr->irq_res == NULL) {
		device_printf(sc->sc_dev,
		    "ERROR: couldn't allocate IRQ %d\n",
		    rid);
		return (ENXIO);
	}

	if ((bus_setup_intr(sc->sc_dev, intr->irq_res,
	    INTR_TYPE_MISC | INTR_MPSAFE,
	    qcom_ess_edma_filter, qcom_ess_edma_intr, sc, &intr->irq_intr))) {
		device_printf(sc->sc_dev,
		    "ERROR: unable to register interrupt handler for"
		    " IRQ %d\n", rid);
		return (ENXIO);
	}

	return (0);
}

static int
qcom_ess_edma_attach(device_t dev)
{
	struct qcom_ess_edma_softc *sc = device_get_softc(dev);
	int i, ret;

	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	sc->sc_dev = dev;
	sc->sc_debug = 0;

	/* Map control/status registers. */
	sc->sc_mem_rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->sc_mem_rid, RF_ACTIVE);

	if (sc->sc_mem_res == NULL) {
		device_printf(dev, "couldn't map memory\n");
		goto error;
	}

	/* Allocate TX IRQs */
	for (i = 0; i < QCOM_ESS_EDMA_NUM_TX_IRQS; i++) {
		if (qcom_ess_edma_setup_intr(sc, &sc->sc_tx_irq[i], i) != 0)
			goto error;
	}

	/* Allocate RX IRQs */
	for (i = 0; i < QCOM_ESS_EDMA_NUM_RX_IRQS; i++) {
		if (qcom_ess_edma_setup_intr(sc, &sc->sc_rx_irq[i],
		    i + QCOM_ESS_EDMA_NUM_TX_IRQS) != 0)
			goto error;
	}

	/* Default receive frame size */
	sc->sc_config.rx_buf_size = 2048;

	/* Default RSS paramters */
	sc->sc_config.rss_type =
	    EDMA_RSS_TYPE_IPV4TCP | EDMA_RSS_TYPE_IPV6_TCP
	    | EDMA_RSS_TYPE_IPV4_UDP | EDMA_RSS_TYPE_IPV6UDP
	    | EDMA_RSS_TYPE_IPV4 | EDMA_RSS_TYPE_IPV6;

	/* Default queue parameters */
	sc->sc_config.tx_ring_count = EDMA_TX_RING_SIZE;
	sc->sc_config.rx_ring_count = EDMA_RX_RING_SIZE;

	/* Default interrupt masks */
	sc->sc_config.rx_intr_mask = EDMA_RX_IMR_NORMAL_MASK;
	sc->sc_config.tx_intr_mask = EDMA_TX_IMR_NORMAL_MASK;
	sc->sc_state.misc_intr_mask = 0;
	sc->sc_state.wol_intr_mask = 0;
	sc->sc_state.intr_sw_idx_w = EDMA_INTR_SW_IDX_W_TYPE;

	EDMA_LOCK(sc);

	/* allocate tx queues */

	/* allocate rx queues */

	/* allocate tx rings */

	/* allocate rx rings */

	/*
	 * (if there's no ess-switch / we're a single phy, we
	 * still need to reset the ess fabric.  Worry about this
	 * later.)
	 */

	/* disable all interrupts */
	ret = qcom_ess_edma_hw_intr_disable(sc);
	if (ret != 0) {
		device_printf(sc->sc_dev,
		    "Failed to disable interrupts (%d)\n",
		    ret);
		goto error_locked;
	}

	/* reset edma */
	ret = qcom_ess_edma_hw_stop(sc);

	/* configure edma */

	/* setup rss indirection table */
	ret = qcom_ess_edma_hw_configure_rss_table(sc);

	/* setup load balancing table */
	ret = qcom_ess_edma_hw_configure_load_balance_table(sc);

	/* configure virtual queue */
	ret = qcom_ess_edma_hw_configure_tx_virtual_queue(sc);

	/* configure AXI burst max */
	ret = qcom_ess_edma_hw_configure_default_axi_transaction_size(sc);

	/* enable IRQs */

	/* enable TX control */

	/* enable RX control */

	EDMA_UNLOCK(sc);

	device_printf(dev, "%s: TODO\n", __func__);
	return (0);

error_locked:
	EDMA_UNLOCK(sc);
error:
	qcom_ess_edma_detach(dev);
	return (ENXIO);
}

static device_method_t qcom_ess_edma_methods[] = {
	/* Driver */
	DEVMETHOD(device_probe, qcom_ess_edma_probe),
	DEVMETHOD(device_attach, qcom_ess_edma_attach),
	DEVMETHOD(device_detach, qcom_ess_edma_detach),

	{0, 0},
};

static driver_t qcom_ess_edma_driver = {
	"essedma",
	qcom_ess_edma_methods,
	sizeof(struct qcom_ess_edma_softc),
};
static devclass_t qcom_ess_edma_devclass;


DRIVER_MODULE(qcom_ess_edma, simplebus, qcom_ess_edma_driver,
    qcom_ess_edma_devclass, NULL, 0);
DRIVER_MODULE(qcom_ess_edma, ofwbus, qcom_ess_edma_driver,
    qcom_ess_edma_devclass, NULL, 0);
MODULE_VERSION(qcom_ess_edma, 1);

/* XXX TODO dependencies (eg ethernet) */
