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
#include <sys/mbuf.h>

#include <net/ethernet.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <dev/gpio/gpiobusvar.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/qcom_ess_edma/qcom_ess_edma_var.h>
#include <dev/qcom_ess_edma/qcom_ess_edma_reg.h>
#include <dev/qcom_ess_edma/qcom_ess_edma_hw.h>
#include <dev/qcom_ess_edma/qcom_ess_edma_desc.h>
#include <dev/qcom_ess_edma/qcom_ess_edma_rx.h>
#include <dev/qcom_ess_edma/qcom_ess_edma_debug.h>
#include <dev/qcom_ess_edma/qcom_ess_edma_gmac.h>

static int
qcom_ess_edma_probe(device_t dev)
{

	if (! ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_is_compatible(dev, "qcom,ess-edma") == 0)
		return (ENXIO);

	device_set_desc(dev,
	    "Qualcomm Atheros IPQ4018/IPQ4019 Ethernet driver");
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

	for (i = 0; i < QCOM_ESS_EDMA_NUM_TX_RINGS; i++) {
		(void) qcom_ess_edma_desc_ring_free(sc, &sc->sc_tx_ring[i]);
	}

	for (i = 0; i < QCOM_ESS_EDMA_NUM_RX_RINGS; i++) {
		(void) qcom_ess_edma_rx_ring_clean(sc, &sc->sc_rx_ring[i]);
		(void) qcom_ess_edma_desc_ring_free(sc, &sc->sc_rx_ring[i]);
	}

	if (sc->sc_dma_tag) {
		bus_dma_tag_destroy(sc->sc_dma_tag);
		sc->sc_dma_tag = NULL;
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

	(void) sc;

	/* XXX TODO */
#if 0
	device_printf(sc->sc_dev, "%s: called; rid=%d\n", __func__,
	    intr->irq_rid);
#endif

	return (FILTER_SCHEDULE_THREAD);
}

static void
qcom_ess_edma_intr(void *arg)
{
	struct qcom_ess_edma_intr *intr = arg;
	struct qcom_ess_edma_softc *sc = intr->sc;

	QCOM_ESS_EDMA_DPRINTF(sc, QCOM_ESS_EDMA_DBG_INTERRUPT,
	    "%s: called; rid=%d\n", __func__,
	    intr->irq_rid);

	intr->stats.num_intr++;

	/*
	 * The RX queue in question starts at QCOM_ESS_EDMA_NUM_TX_IRQS.
	 * (eg if it's 16 then rid 16 == RX queue 0.)
	 */
	if (intr->irq_rid < QCOM_ESS_EDMA_NUM_TX_IRQS) {
		/* Transmit queue */
		device_printf(sc->sc_dev, "%s: called; TX queue %d, TODO\n",
		    __func__, intr->irq_rid);
		/* XXX TODO */
	} else {
		struct mbufq mq;
		int rx_queue;

		mbufq_init(&mq, EDMA_RX_RING_SIZE);

		rx_queue = intr->irq_rid - QCOM_ESS_EDMA_NUM_TX_IRQS;
		/* Receive queue */
		QCOM_ESS_EDMA_DPRINTF(sc, QCOM_ESS_EDMA_DBG_INTERRUPT,
		    "%s: called; RX queue %d\n",
		    __func__, rx_queue);

		EDMA_LOCK(sc);
		/*
		 * XXX TODO: should put this in the edma filter routine?
		 *
		 * Disable the interrupt for this ring.
		 */
		(void) qcom_ess_edma_hw_intr_rx_intr_set_enable(sc, rx_queue,
		    false);

		/*
		 * Do receive work, get completed mbufs.
		 */
		(void) qcom_ess_edma_rx_ring_complete(sc, rx_queue, &mq);

		/*
		 * ACK the interrupt.
		 */
		(void) qcom_ess_edma_hw_intr_rx_ack(sc, rx_queue);

		/*
		 * Re-enable interrupt for this ring.
		 */
		(void) qcom_ess_edma_hw_intr_rx_intr_set_enable(sc, rx_queue,
		    true);

		EDMA_UNLOCK(sc);

		/* Push frames into networking stack */
		(void) qcom_ess_edma_gmac_receive_frames(sc, rx_queue, &mq);
	}
}

static int
qcom_ess_edma_setup_intr(struct qcom_ess_edma_softc *sc,
    struct qcom_ess_edma_intr *intr, int rid)
{

	QCOM_ESS_EDMA_DPRINTF(sc, QCOM_ESS_EDMA_DBG_INTERRUPT,
	    "%s: setting up interrupt id %d\n", __func__, rid);
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
	    INTR_TYPE_NET | INTR_MPSAFE,
	    qcom_ess_edma_filter, qcom_ess_edma_intr, intr,
	        &intr->irq_intr))) {
		device_printf(sc->sc_dev,
		    "ERROR: unable to register interrupt handler for"
		    " IRQ %d\n", rid);
		return (ENXIO);
	}

	return (0);
}

static int
qcom_ess_edma_sysctl_dump_state(SYSCTL_HANDLER_ARGS)
{
	struct qcom_ess_edma_softc *sc = arg1;
	int val = 0;
	int error;
	int i;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || !req->newptr)
		return (error);
	if (val == 0)
		return (0);

	EDMA_LOCK(sc);
	for (i = 0; i < QCOM_ESS_EDMA_NUM_RX_RINGS; i++) {
		device_printf(sc->sc_dev,
		    "RXQ[%d]: prod=%u, cons=%u, hw prod=%u, hw cons=%u,"
		    " REG_SW_CONS_IDX=0x%08x\n",
		    i,
		    sc->sc_rx_ring[i].next_to_fill,
		    sc->sc_rx_ring[i].next_to_clean,
		    EDMA_REG_READ(sc,
		        EDMA_REG_RFD_IDX_Q(i)) & EDMA_RFD_PROD_IDX_BITS,
		    qcom_ess_edma_hw_rfd_get_cons_index(sc, i),
		    EDMA_REG_READ(sc, EDMA_REG_RX_SW_CONS_IDX_Q(i)));
		device_printf(sc->sc_dev,
		    "RXQ[%d]: num_added=%llu, num_cleaned=%llu,"
		    " num_dropped=%llu, num_enqueue_full=%llu,"
		    " num_rx_no_gmac=%llu, num_rx_ok=%llu\n",
		    i,
		    sc->sc_rx_ring[i].stats.num_added,
		    sc->sc_rx_ring[i].stats.num_cleaned,
		    sc->sc_rx_ring[i].stats.num_dropped,
		    sc->sc_rx_ring[i].stats.num_enqueue_full,
		    sc->sc_rx_ring[i].stats.num_rx_no_gmac,
		    sc->sc_rx_ring[i].stats.num_rx_ok);
	}

	for (i = 0; i < QCOM_ESS_EDMA_NUM_RX_IRQS; i++) {
		device_printf(sc->sc_dev, "INTR_RXQ[%d]: num_intr=%llu\n",
		    i,
		    sc->sc_rx_irq[i].stats.num_intr);
	}

	device_printf(sc->sc_dev, "EDMA_REG_TXQ_CTRL=0x%08x\n",
	    EDMA_REG_READ(sc, EDMA_REG_TXQ_CTRL));
	device_printf(sc->sc_dev, "EDMA_REG_RXQ_CTRL=0x%08x\n",
	    EDMA_REG_READ(sc, EDMA_REG_RXQ_CTRL));
	device_printf(sc->sc_dev, "EDMA_REG_RX_DESC0=0x%08x\n",
	    EDMA_REG_READ(sc, EDMA_REG_RX_DESC0));
	device_printf(sc->sc_dev, "EDMA_REG_RX_DESC1=0x%08x\n",
	    EDMA_REG_READ(sc, EDMA_REG_RX_DESC1));
	device_printf(sc->sc_dev, "EDMA_REG_RX_ISR=0x%08x\n",
	    EDMA_REG_READ(sc, EDMA_REG_RX_ISR));
	device_printf(sc->sc_dev, "EDMA_REG_TX_ISR=0x%08x\n",
	    EDMA_REG_READ(sc, EDMA_REG_TX_ISR));
	device_printf(sc->sc_dev, "EDMA_REG_MISC_ISR=0x%08x\n",
	    EDMA_REG_READ(sc, EDMA_REG_MISC_ISR));
	device_printf(sc->sc_dev, "EDMA_REG_WOL_ISR=0x%08x\n",
	    EDMA_REG_READ(sc, EDMA_REG_WOL_ISR));

	EDMA_UNLOCK(sc);

	return (0);
}

static int
qcom_ess_edma_attach_sysctl(struct qcom_ess_edma_softc *sc)
{
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(sc->sc_dev);
	struct sysctl_oid *tree = device_get_sysctl_tree(sc->sc_dev);

	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "debug", CTLFLAG_RW, &sc->sc_debug, 0,
	    "debugging flags");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "state", CTLTYPE_INT | CTLFLAG_RW, sc,
	    0, qcom_ess_edma_sysctl_dump_state, "I", "");

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

	(void) qcom_ess_edma_attach_sysctl(sc);

	/* Create parent DMA tag. */
	ret = bus_dma_tag_create(
	    bus_get_dma_tag(sc->sc_dev),	/* parent */
	    1, 0,				/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,		/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filter, filterarg */
	    BUS_SPACE_MAXSIZE_32BIT,		/* maxsize */
	    0,					/* nsegments */
	    BUS_SPACE_MAXSIZE_32BIT,		/* maxsegsize */
	    0,					/* flags */
	    NULL, NULL,				/* lockfunc, lockarg */
	    &sc->sc_dma_tag);
	if (ret != 0) {
		device_printf(sc->sc_dev,
		    "ERROR: failed to create parent DMA tag\n");
		goto error;
	}

	/* Map control/status registers. */
	sc->sc_mem_rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->sc_mem_rid, RF_ACTIVE);

	if (sc->sc_mem_res == NULL) {
		device_printf(dev, "ERROR: couldn't map MMIO space\n");
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

	/*
	 * Parse out the gmac count so we can start parsing out
	 * the gmac list and create us some ifnets.
	 */
	if (OF_getencprop(ofw_bus_get_node(dev), "qcom,num_gmac",
	    &sc->sc_config.num_gmac, sizeof(uint32_t)) > 0) {
		device_printf(sc->sc_dev, "Creating %d GMACs\n",
		    sc->sc_config.num_gmac);
	} else {
		device_printf(sc->sc_dev, "Defaulting to 1 GMAC\n");
		sc->sc_config.num_gmac = 1;
	}
	if (sc->sc_config.num_gmac > QCOM_ESS_EDMA_MAX_NUM_GMACS) {
		device_printf(sc->sc_dev, "Capping GMACs to %d\n",
		    QCOM_ESS_EDMA_MAX_NUM_GMACS);
		sc->sc_config.num_gmac = QCOM_ESS_EDMA_MAX_NUM_GMACS;
	}

	/*
	 * And now, create some gmac entries here; we'll create the
	 * ifnet's once this is all done.
	 */
	for (i = 0; i < sc->sc_config.num_gmac; i++) {
		ret = qcom_ess_edma_gmac_parse(sc, i);
		if (ret != 0) {
			device_printf(sc->sc_dev,
			    "Failed to parse gmac%d\n", i);
			goto error;
		}
	}

	/* allocate tx rings */
	for (i = 0; i < QCOM_ESS_EDMA_NUM_TX_RINGS; i++) {
		if (qcom_ess_edma_desc_ring_setup(sc, &sc->sc_tx_ring[i],
		    sc->sc_config.tx_ring_count,
		    sizeof(struct qcom_ess_edma_sw_desc_tx),
		    sizeof(struct qcom_ess_edma_tx_desc),
		    ESS_EDMA_TX_BUFFER_ALIGN) != 0)
			goto error;
	}

	/* allocate rx rings */
	for (i = 0; i < QCOM_ESS_EDMA_NUM_RX_RINGS; i++) {
		if (qcom_ess_edma_desc_ring_setup(sc, &sc->sc_rx_ring[i],
		    sc->sc_config.rx_ring_count,
		    sizeof(struct qcom_ess_edma_sw_desc_rx),
		    sizeof(struct qcom_ess_edma_rx_free_desc),
		    ESS_EDMA_RX_BUFFER_ALIGN) != 0)
			goto error;
		if (qcom_ess_edma_rx_ring_setup(sc, &sc->sc_rx_ring[i]) != 0)
			goto error;
	}

	/*
	 * map the gmac instances <-> port masks, so incoming frames know
	 * where they need to be forwarded to.
	 */
	for (i = 0; i < QCOM_ESS_EDMA_MAX_NUM_PORTS; i++)
		sc->sc_gmac_port_map[i] = -1;
	for (i = 0; i < sc->sc_config.num_gmac; i++) {
		ret = qcom_ess_edma_gmac_setup_port_mapping(sc, i);
		if (ret != 0) {
			device_printf(sc->sc_dev,
			    "Failed to setup port mpapping for gmac%d\n", i);
			goto error;
		}
	}


	/* Create ifnets */
	for (i = 0; i < sc->sc_config.num_gmac; i++) {
		ret = qcom_ess_edma_gmac_create_ifnet(sc, i);
		if (ret != 0) {
			device_printf(sc->sc_dev,
			    "Failed to create ifnet for gmac%d\n", i);
			goto error;
		}
	}

	/*
	 * (if there's no ess-switch / we're a single phy, we
	 * still need to reset the ess fabric.  Worry about this
	 * later.)
	 */

	EDMA_LOCK(sc);

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

	/* fill RX ring here, explicitly */
	for (i = 0; i < QCOM_ESS_EDMA_NUM_RX_RINGS; i++) {
		(void) qcom_ess_edma_rx_ring_fill(sc, i,
		    sc->sc_config.rx_ring_count);
	}

	/* configure TX/RX rings; RSS config; initial interrupt rates, etc */
	ret = qcom_ess_edma_hw_setup(sc);
	ret = qcom_ess_edma_hw_setup_tx(sc);
	ret = qcom_ess_edma_hw_setup_rx(sc);
	ret = qcom_ess_edma_hw_setup_txrx_desc_rings(sc);

	/* setup rss indirection table */
	ret = qcom_ess_edma_hw_configure_rss_table(sc);

	/* setup load balancing table */
	ret = qcom_ess_edma_hw_configure_load_balance_table(sc);

	/* configure virtual queue */
	ret = qcom_ess_edma_hw_configure_tx_virtual_queue(sc);

	/* configure AXI burst max */
	ret = qcom_ess_edma_hw_configure_default_axi_transaction_size(sc);

	/* enable IRQs */
	ret = qcom_ess_edma_hw_intr_enable(sc);

	/* enable TX control */
	ret = qcom_ess_edma_hw_tx_enable(sc);

	/* enable RX control */
	ret = qcom_ess_edma_hw_rx_enable(sc);

	EDMA_UNLOCK(sc);

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
