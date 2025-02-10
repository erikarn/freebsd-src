/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 Adrian Chadd <adrian@FreeBSD.org>.
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
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
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
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/endian.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/machdep.h>
#include <machine/resource.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/simplebus.h>
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "qcom_cmd_db_hw.h"
#include "qcom_cmd_db.h"

struct qcom_cmd_db_softc {
	unsigned long sc_regaddr;
	unsigned long sc_reglen;
	void *sc_reg_vmaddr;
	bool sc_is_setup;
	struct mtx sc_mtx;
};

#define	DPRINTF(sc, fmt, ...) \
	printf("[cmd-db] "fmt, ##__VA_ARGS__)

static struct qcom_cmd_db_softc qcom_cmd_db = { 0 };

/**
 * Return a string representation of the underlying hardware type
 * being queried.
 */
static const char *
qcom_cmd_db_hw_type_str(qcom_cmd_db_hw_type_t hw)
{
	switch (hw){
	case QCOM_CMD_DB_HW_TYPE_INVALID:
		return "INVALID";
	case QCOM_CMD_DB_HW_TYPE_ARC:
		return "ARC";
	case QCOM_CMD_DB_HW_TYPE_VRM:
		return "VRM";
	case QCOM_CMD_DB_HW_TYPE_BCM:
		return "BCM";
	default:
		return "<unknown>";
	}
}

/**
 * Return a pointer to the specific entry for the given RSC entry.
 *
 * This bounds-checks the given rsc ID, entry ID and the returned header offset
 * and will return NULL if the header_offset is not within the
 * memory window.
 */
static const struct qcom_cmd_db_entry_hdr *
qcom_cmd_db_get_rsc_entry(struct qcom_cmd_db_softc *sc, uint32_t rsc_id,
    uint32_t ent_id)
{
	const struct qcom_cmd_db_hdr *hdr;
	const struct qcom_cmd_db_entry_hdr *ent;
	const struct qcom_cmd_db_rsc_hdr *rsc;
	uint32_t offset;

	if (!sc->sc_is_setup)
		return (NULL);

	if (rsc_id >= QCOM_CMD_DB_MAX_SLAVE_ID)
		return (NULL);

	hdr = qcom_cmd_db.sc_reg_vmaddr;
	rsc = &hdr->rsc_header[rsc_id];
	if (rsc->slave_id == QCOM_CMD_DB_HW_TYPE_INVALID)
		return (NULL);
	if (ent_id >= le16toh(rsc->count))
		return (NULL);

	/* Bounds check! */
	offset = sizeof(struct qcom_cmd_db_hdr) +
	    le16toh(rsc->header_offset) +
	    (sizeof(struct qcom_cmd_db_entry_hdr) * ent_id);

	if (offset > sc->sc_reglen) {
		DPRINTF(sc, "%s: bounds check failed (%u + %u + %u > %u)\n",
		    __func__,
		    (unsigned int) sizeof(struct qcom_cmd_db_hdr),
		    (unsigned int) le16toh(rsc->header_offset),
		    (unsigned int) sizeof(struct qcom_cmd_db_entry_hdr) * ent_id,
		    (unsigned int) sc->sc_reglen);
		return (NULL);
	}

	/* Return a pointer */
	ent = (const void *)
	    (((const uint8_t *) (sc->sc_reg_vmaddr)) + offset);
	return (ent);
}

/**
 * Attempt to find the memory range and initialise the various
 * bus space bits needed to access it.
 */
static bool
qcom_cmd_db_find_mem(struct qcom_cmd_db_softc *sc)
{
#define	FDT_REG_CELLS 4
	pcell_t reg[FDT_REG_CELLS];
	phandle_t child, root;
	int addr_cells, size_cells;
	int rv;

	root = OF_finddevice("/reserved-memory");
	if (root == -1) {
		DPRINTF(sc, "couldn't find /reserved-memory\n");
		return false;
	}

	rv = fdt_addrsize_cells(root, &addr_cells, &size_cells);
	if (rv != 0) {
		DPRINTF(sc, "error calling fdt_addrsize_cells (%d)\n", rv);
		return false;
	}
	if (addr_cells + size_cells > FDT_REG_CELLS) {
		DPRINTF(sc,
		    "Error: addr %d + size %d cells > FDT_REG_CELLS (%d)\n",
		    addr_cells, size_cells, FDT_REG_CELLS);
		return false;
	}

	for (child = OF_child(root); child != 0; child = OF_peer(child)) {
		char *compat_name;
		unsigned long reg_addr, reg_len;

		if (!OF_hasprop(child, "reg"))
			continue;
		rv = OF_getprop(child, "reg", reg, sizeof(reg));
		if (rv <= 0)
			continue;
		fdt_data_to_res(reg, addr_cells, size_cells, &reg_addr, &reg_len);

		if (!OF_hasprop(child, "compatible"))
			continue;
		rv = OF_getprop_alloc(child, "compatible", (void **) &compat_name);
		if (rv <= 0)
			continue;
		if (strcmp(compat_name, "qcom,cmd-db") != 0) {
			OF_prop_free(compat_name); compat_name = NULL;
			continue;
		}

		/* It's found */
		DPRINTF(sc, "found (0x%0lx -> 0x%lx)\n", reg_addr,
		    reg_addr + reg_len - 1);

		OF_prop_free(compat_name); compat_name = NULL;

		/* Set up the KVA mapping */
		sc->sc_reg_vmaddr = pmap_mapdev_attr(reg_addr, reg_len,
		    VM_MEMATTR_DEVICE);
		if (sc->sc_reg_vmaddr == NULL) {
			DPRINTF(sc, "failed to map KVA\n");
			return (false);
		}
		sc->sc_regaddr = reg_addr;
		sc->sc_reglen = reg_len;
		sc->sc_is_setup = true;

		return (true);
	}

	return (false);
}
#undef	FDT_REG_CELLS

static bool
qcom_cmd_check_header_magic(struct qcom_cmd_db_softc *sc)
{
	struct qcom_cmd_db_hdr *hdr;

	if (!sc->sc_is_setup)
		return (false);

	hdr = qcom_cmd_db.sc_reg_vmaddr;

	if (le32toh(hdr->magic) != QCOM_CMD_DB_HEADER_MAGIC)
		return (false);
	return (true);
}

static void
qcom_cmd_print_header_magic(struct qcom_cmd_db_softc *sc)
{
	struct qcom_cmd_db_hdr *hdr;

	if (!sc->sc_is_setup)
		return;

	hdr = qcom_cmd_db.sc_reg_vmaddr;

	DPRINTF(sc, "version=%d, magic=0x%08x\n",
	    hdr->version, le32toh(hdr->magic));
}

static void
qcom_cmd_db_dump(struct qcom_cmd_db_softc *sc)
{
	struct qcom_cmd_db_hdr *hdr;
	struct qcom_cmd_db_rsc_hdr *rsc;
	int i, j;

	if (!sc->sc_is_setup)
		return;

	hdr = qcom_cmd_db.sc_reg_vmaddr;
	for (i = 0; i < QCOM_CMD_DB_MAX_SLAVE_ID; i++) {
		const struct qcom_cmd_db_entry_hdr *ent;
		rsc = &hdr->rsc_header[i];
		if (rsc->slave_id == QCOM_CMD_DB_HW_TYPE_INVALID)
			continue;
		DPRINTF(sc, "  Slave %d (%s) (version %d.%d)\n",
		    le16toh(rsc->slave_id),
		    qcom_cmd_db_hw_type_str(le16toh(rsc->slave_id)),
		    le16toh(rsc->version) >> 8,
		    le16toh(rsc->version) & 0xff);

		DPRINTF(sc, "  -> header offset %d, data offset %d, count %d\n",
		    le16toh(rsc->header_offset),
		    le16toh(rsc->data_offset),
		    le16toh(rsc->count));
		for (j = 0; j < le16toh(rsc->count); j++) {
			ent = qcom_cmd_db_get_rsc_entry(sc, i, j);
			if (ent == NULL)
				continue;
			DPRINTF(sc, "  -> [%d]: id: %.*s, resource addr=0x%x, aux data offset=0x%x, len=%d\n",
			    j,
			    (int) strnlen(ent->id, QCOM_CMD_DB_ENTRY_HDR_ID_SIZE),
			    ent->id,
			    le32toh(ent->addr),
			    le16toh(ent->offset),
			    le16toh(ent->len));
		}
	}
}

/**
 * Lookup a 32 bit resource address by id string.
 */
bool
qcom_cmd_db_lookup_addr_by_id(const char *id, uint32_t *addr)
{
	return (false);
}

/**
 * Lookup the auxiliary data and length by id string.
 */
bool
qcom_cmd_db_lookup_aux_data_by_id(const char *id, void *addr, uint16_t *len)
{
	return (false);
}

static void
qcom_cmd_db_init(void *dummy)
{

	mtx_init(&qcom_cmd_db.sc_mtx, "qcom_cmd_db", NULL, MTX_DEF);
	qcom_cmd_db.sc_is_setup = false;

	/*
	 * Iterate over reserved-memory, looking for a node with
	 * compatible = "qcom,cmd-db".
	 */
	if (qcom_cmd_db_find_mem(&qcom_cmd_db) == false)
		return;

	/* Print the current version/magic; see if it's ready */
	qcom_cmd_print_header_magic(&qcom_cmd_db);

	/* Don't continue with this setup path if it's not ready */
	if (!qcom_cmd_check_header_magic(&qcom_cmd_db))
		return;

	/* Dump out the tables */
	qcom_cmd_db_dump(&qcom_cmd_db);
}

SYSINIT(ti_cpu_ident, SI_SUB_CPU, SI_ORDER_FIRST, qcom_cmd_db_init, NULL);
