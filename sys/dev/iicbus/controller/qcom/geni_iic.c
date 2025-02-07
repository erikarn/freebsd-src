/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Poul-Henning Kamp <phk@FreeBSD.org>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * QualComm GENI I2C controller
 *
 * The GENI is actually a multi-protocol serial controller, so a lot of
 * this can probably be shared if we ever get to those protocols.
 *
 * The best open "documentation" of the hardware is the Linux device driver
 * from which much was learned, and we tip our hat to the authors of it.
 */

#include <sys/cdefs.h>

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/endian.h>
#include <sys/time.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/sx.h>
#include <sys/bus.h>

#include <machine/bus.h>
#include <sys/rman.h>

#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>

#include <dev/iicbus/controller/qcom/geni_iic_var.h>

#define GENI_ALL_REGISTERS(THIS_MACRO) \
	THIS_MACRO(GENI_FORCE_DEFAULT_REG,	0x020) \
	THIS_MACRO(GENI_OUTPUT_CTRL,		0x024) \
	THIS_MACRO(GENI_STATUS,			0x040) \
	THIS_MACRO(GENI_SER_M_CLK_CFG,		0x048) \
	THIS_MACRO(GENI_SER_S_CLK_CFG,		0x04c) \
	THIS_MACRO(GENI_IF_DISABLE_RO,		0x064) \
	THIS_MACRO(GENI_FW_REVISION_RO,		0x068) \
	THIS_MACRO(GENI_CLK_SEL,		0x07c) \
	THIS_MACRO(GENI_CFG_SEQ_START,		0x084) \
	THIS_MACRO(GENI_BYTE_GRANULARITY,	0x254) \
	THIS_MACRO(GENI_DMA_MODE_EN,		0x258) \
	THIS_MACRO(GENI_TX_PACKING_CFG0,	0x260) \
	THIS_MACRO(GENI_TX_PACKING_CFG1,	0x264) \
	THIS_MACRO(GENI_I2C_TX_TRANS_LEN,	0x26c) \
	THIS_MACRO(GENI_I2C_RX_TRANS_LEN,	0x270) \
	THIS_MACRO(GENI_I2C_SCL_COUNTERS,	0x278) \
	THIS_MACRO(GENI_RX_PACKING_CFG0,	0x284) \
	THIS_MACRO(GENI_RX_PACKING_CFG1,	0x288) \
	THIS_MACRO(GENI_M_CMD0,			0x600) \
	THIS_MACRO(GENI_M_CMD_CTRL_REG,		0x604) \
	THIS_MACRO(GENI_M_IRQ_STATUS,		0x610) \
	THIS_MACRO(GENI_M_IRQ_EN,		0x614) \
	THIS_MACRO(GENI_M_IRQ_CLEAR,		0x618) \
	THIS_MACRO(GENI_M_IRQ_EN_SET,		0x61c) \
	THIS_MACRO(GENI_M_IRQ_EN_CLEAR,		0x620) \
	THIS_MACRO(GENI_S_CMD0,			0x630) \
	THIS_MACRO(GENI_S_CMD_CTRL_REG,		0x634) \
	THIS_MACRO(GENI_S_IRQ_STATUS,		0x640) \
	THIS_MACRO(GENI_S_IRQ_EN,		0x644) \
	THIS_MACRO(GENI_S_IRQ_CLEAR,		0x648) \
	THIS_MACRO(GENI_S_IRQ_EN_SET,		0x64c) \
	THIS_MACRO(GENI_S_IRQ_EN_CLEAR,		0x650) \
	THIS_MACRO(GENI_TX_FIFOn,		0x700) \
	THIS_MACRO(GENI_RX_FIFOn,		0x780) \
	THIS_MACRO(GENI_TX_FIFO_STATUS,		0x800) \
	THIS_MACRO(GENI_RX_FIFO_STATUS,		0x804) \
	THIS_MACRO(GENI_TX_WATERMARK_REG,	0x80c) \
	THIS_MACRO(GENI_RX_WATERMARK_REG,	0x810) \
	THIS_MACRO(GENI_RX_RFR_WATERMARK_REG,	0x814) \
	THIS_MACRO(GENI_IOS,			0x908) \
	THIS_MACRO(GENI_M_GP_LENGTH,		0x910) \
	THIS_MACRO(GENI_S_GP_LENGTH,		0x914) \
	THIS_MACRO(GENI_DMA_TX_IRQ_STAT,	0xc40) \
	THIS_MACRO(GENI_DMA_TX_IRQ_CLR,		0xc44) \
	THIS_MACRO(GENI_DMA_TX_IRQ_EN,		0xc48) \
	THIS_MACRO(GENI_DMA_TX_IRQ_EN_CLR,	0xc4c) \
	THIS_MACRO(GENI_DMA_TX_IRQ_EN_SET,	0xc50) \
	THIS_MACRO(GENI_DMA_TX_FSM_RST,		0xc58) \
	THIS_MACRO(GENI_DMA_RX_IRQ_STAT,	0xd40) \
	THIS_MACRO(GENI_DMA_RX_IRQ_CLR,		0xd44) \
	THIS_MACRO(GENI_DMA_RX_IRQ_EN,		0xd48) \
	THIS_MACRO(GENI_DMA_RX_IRQ_EN_CLR,	0xd4c) \
	THIS_MACRO(GENI_DMA_RX_IRQ_EN_SET,	0xd50) \
	THIS_MACRO(GENI_DMA_RX_LEN_IN,		0xd54) \
	THIS_MACRO(GENI_DMA_RX_FSM_RST,		0xd58) \
	THIS_MACRO(GENI_IRQ_EN,			0xe1c) \
	THIS_MACRO(GENI_HW_PARAM_0,		0xe24) \
	THIS_MACRO(GENI_HW_PARAM_1,		0xe28)

enum geni_registers {
#define ITER_MACRO(name, offset) name = offset,
	GENI_ALL_REGISTERS(ITER_MACRO)
#undef ITER_MACRO
};

static void
geni_dump_regs(geniiic_softc_t *sc)
{
	device_printf(sc->dev, "Register Dump\n");
#define DUMP_MACRO(name, offset) \
	device_printf(sc->dev, "    %08x %04x " #name "\n", bus_read_4(sc->regs_res, offset), offset); 
	GENI_ALL_REGISTERS(DUMP_MACRO)
#undef DUMP_MACRO
}

static driver_filter_t geniiic_intr;

static int
geniiic_intr(void *cookie)
{
	uint32_t m_status, rx_fifo_status;
	int retval = FILTER_STRAY;
	geniiic_softc_t *sc = cookie;

	mtx_lock_spin(&sc->intr_lock);
        m_status = bus_read_4(sc->regs_res, GENI_M_IRQ_STATUS);

	rx_fifo_status = bus_read_4(sc->regs_res, GENI_RX_FIFO_STATUS);
	if (sc->rx_buf != NULL && rx_fifo_status & 0x3f) {
		unsigned gotlen = (((rx_fifo_status & 0x3f) << 2)-1) * 4 + ((rx_fifo_status >> 28) & 7);
		unsigned cnt;
		for (cnt = 0; cnt < (rx_fifo_status & 0x3f); cnt++) {
			uint32_t data = bus_read_4(sc->regs_res, GENI_RX_FIFOn);
			if (sc->rx_len && gotlen) { *sc->rx_buf++ = data >>  0; sc->rx_len--; gotlen--;}
			if (sc->rx_len && gotlen) { *sc->rx_buf++ = data >>  8; sc->rx_len--; gotlen--;}
			if (sc->rx_len && gotlen) { *sc->rx_buf++ = data >> 16; sc->rx_len--; gotlen--;}
			if (sc->rx_len && gotlen) { *sc->rx_buf++ = data >> 24; sc->rx_len--; gotlen--;}
		}
	}
	if (m_status & (1<<26)) {
		bus_write_4(sc->regs_res, GENI_M_IRQ_CLEAR, (1<<26));
		retval = FILTER_HANDLED;
	}

	if (m_status & (1<<0)) {
		sc->rx_complete = true;
		bus_write_4(sc->regs_res, GENI_M_IRQ_EN_CLEAR, (1<<0));
		bus_write_4(sc->regs_res, GENI_M_IRQ_EN_CLEAR, (1<<26));
		bus_write_4(sc->regs_res, GENI_M_IRQ_CLEAR, (1<<0));
		wakeup(sc);
		retval = FILTER_HANDLED;
	}
	sc->cmd_status = m_status;

	if (sc->rx_buf == NULL) {
		device_printf(sc->dev, "Interrupt m_stat %x rx_fifo_status %x retval %d\n", m_status, rx_fifo_status, retval);
		bus_write_4(sc->regs_res, GENI_M_IRQ_EN, 0);
		bus_write_4(sc->regs_res, GENI_M_IRQ_CLEAR, m_status);
		device_printf(sc->dev, "Interrupt M_IRQ_STATUS 0x%x M_IRQ_EN 0x%x\n",
			bus_read_4(sc->regs_res, GENI_M_IRQ_STATUS), bus_read_4(sc->regs_res, GENI_M_IRQ_EN));
		device_printf(sc->dev, "Interrupt S_IRQ_STATUS 0x%x S_IRQ_EN 0x%x\n",
			bus_read_4(sc->regs_res, GENI_S_IRQ_STATUS), bus_read_4(sc->regs_res, GENI_S_IRQ_EN));
		device_printf(sc->dev, "Interrupt GENI_DMA_TX_IRQ_STAT 0x%x GENI_DMA_RX_IRQ_STAT 0x%x\n",
			bus_read_4(sc->regs_res, GENI_DMA_TX_IRQ_STAT), bus_read_4(sc->regs_res, GENI_DMA_RX_IRQ_STAT));
		device_printf(sc->dev, "Interrupt GENI_DMA_TX_IRQ_EN 0x%x GENI_DMA_RX_IRQ_EN 0x%x\n",
			bus_read_4(sc->regs_res, GENI_DMA_TX_IRQ_EN), bus_read_4(sc->regs_res, GENI_DMA_RX_IRQ_EN));
		bus_write_4(sc->regs_res, GENI_DMA_TX_IRQ_EN_CLR, bus_read_4(sc->regs_res, GENI_DMA_TX_IRQ_STAT));
		bus_write_4(sc->regs_res, GENI_DMA_TX_IRQ_CLR, bus_read_4(sc->regs_res, GENI_DMA_TX_IRQ_STAT));
		bus_write_4(sc->regs_res, GENI_DMA_RX_IRQ_EN_CLR, bus_read_4(sc->regs_res, GENI_DMA_RX_IRQ_STAT));
		bus_write_4(sc->regs_res, GENI_DMA_RX_IRQ_CLR, bus_read_4(sc->regs_res, GENI_DMA_RX_IRQ_STAT));
	}
	mtx_unlock_spin(&sc->intr_lock);
	return(retval);
}

static int
geniiic_wait_m_ireq(geniiic_softc_t *sc, uint32_t bits)
{
        uint32_t status;
        int timeout;

        for (timeout = 0; timeout < 10000; timeout++) {
                status = bus_read_4(sc->regs_res, GENI_M_IRQ_STATUS);
                if (status & bits) {
			//device_printf(sc->dev, "wait done status=%x bits=%x\n", status, bits);
                        return (0);
		}
                DELAY(10);
        }
	//geni_dump_regs(sc);
	//device_printf(sc->dev, "geniiic_wait_m_ireq timeout status=%x bits=%x\n", status, bits);
        return (IIC_ETIMEOUT);
}

static int
geniiic_read(geniiic_softc_t *sc, uint8_t slave, uint8_t *buf, uint16_t len, bool nonfinal)
{
	uint32_t cmd, istatus;

	//device_printf(sc->dev, "read(0x%x, %d, %p, %d, %d)\n", slave >> 1, len, buf, repeated_start, stop);

	istatus = bus_read_4(sc->regs_res, GENI_M_IRQ_STATUS);
	bus_write_4(sc->regs_res, GENI_M_IRQ_CLEAR, istatus);

	sc->rx_complete = false;
	sc->rx_fifo = false;
	sc->rx_buf = buf;
	sc->rx_len = len;
	bus_write_4(sc->regs_res, GENI_I2C_RX_TRANS_LEN, len);
	cmd = (0x2 << 27); 	// GENI_M_CMD0_OPCODE_I2C_READ << M_OPCODE_SHFT
	cmd |= slave << 9;	// GENI_M_CMD0_SLV_ADDR_SHIFT
	if (nonfinal)
		cmd |= (1<<2);		// GENI_M_CMD0_STOP_STRETCH
	bus_write_4(sc->regs_res, GENI_RX_WATERMARK_REG, sc->rx_fifo_size - 4);
	bus_write_4(sc->regs_res, GENI_M_IRQ_EN, (1<<0) | (1<<26));		// CMD_DONE, RX_FIFO_WATERMARK
	bus_write_4(sc->regs_res, GENI_IRQ_EN, (1<<2));				// M_IRQ
	bus_write_4(sc->regs_res, GENI_M_CMD0, cmd);

	mtx_lock_spin(&sc->intr_lock);
	sc->rx_fifo = false;
	unsigned msec;
	for (msec = 0; msec < 100; msec++) {
		msleep_spin_sbt(sc, &sc->intr_lock, "geniwait", SBT_1MS, SBT_1MS / 10, 0);
		if (sc->rx_complete)
			break;
	}
	if (msec > sc->worst) {
		device_printf(sc->dev, "Tworst from %u to %u\n", sc->worst, msec);
		if (msec != 100)
		    sc->worst = msec;
	}

	if (!sc->rx_complete) {
		bus_write_4(sc->regs_res, GENI_M_CMD_CTRL_REG, (1<<2));		// S_GENI_CMD_CANCEL
		bus_write_4(sc->regs_res, GENI_IRQ_EN, 0);
		device_printf(sc->dev, "Incomplete read\n");
	}

	sc->rx_buf = NULL;
	len = sc->rx_len;
	sc->rx_len = 0;

	mtx_unlock_spin(&sc->intr_lock);


	if (len == 0)
		return(IIC_NOERR);

	device_printf(sc->dev, "read result slave=0x%x len=0x%x, cmd=0x%x cmd_status=0x%x\n", slave, len, cmd, sc->cmd_status);
	if (sc->cmd_status & (1<<10))
		return(IIC_ESTATUS);
	return (IIC_EBUSERR);
}

static int
geniiic_write(geniiic_softc_t *sc, uint8_t slave, uint8_t *buf, uint16_t len, bool nonfinal)
{
	//device_printf(sc->dev, "write(0x%x, %d, %p, %d, %d)\n", slave >> 1, len, buf, repeated_start, stop);

	uint32_t status, data, cmd;
	int timeout, error;

	status = bus_read_4(sc->regs_res, GENI_M_IRQ_STATUS);
	bus_write_4(sc->regs_res, GENI_M_IRQ_CLEAR, status);

	bus_write_4(sc->regs_res, GENI_I2C_TX_TRANS_LEN, len);
	cmd = (0x1 << 27); 	// GENI_M_CMD0_OPCODE_I2C_WRITE << M_OPCODE_SHFT
	cmd |= slave << 9;	// GENI_M_CMD0_SLV_ADDR_SHIFT
	if (nonfinal)
		cmd |= (1<<2);		// GENI_M_CMD0_STOP_STRETCH
	bus_write_4(sc->regs_res, GENI_M_CMD0, cmd);
	for(timeout = 0; len > 0 && timeout < 100; timeout++) {
		status = bus_read_4(sc->regs_res, GENI_TX_FIFO_STATUS);
		if (status < 16) {
			data = 0;
			if (len) { data |= *buf <<  0; buf++; len--; }
			if (len) { data |= *buf <<  8; buf++; len--; }
			if (len) { data |= *buf << 16; buf++; len--; }
			if (len) { data |= *buf << 24; buf++; len--; }
//device_printf(sc->dev, "tx_fifo_status 0x%x data 0x%x\n", status, data);
			bus_write_4(sc->regs_res, GENI_TX_FIFOn, data);
		} else {
			DELAY(10);
		}
	}
	//geni_dump_regs(sc);
	error = geniiic_wait_m_ireq(sc, 1);	// GENI_M_IRQ_CMD_DONE
	if (len == 0 && error == 0)
		return(IIC_NOERR);
	device_printf(sc->dev, "write result len=%d, error=%d cmd=0x%x\n", len, error, cmd);
	return (IIC_EBUSERR);
}

int
geniiic_transfer(device_t dev, struct iic_msg *msgs, uint32_t nmsgs)
{
	geniiic_softc_t *sc = device_get_softc(dev);
	uint32_t i;
	int error;

	if (sc->nfail > 4) {
		pause_sbt("geniic_fail", SBT_1S * 5, SBT_1S, 0);
		return (IIC_ERESOURCE);
	}

	sx_xlock(&sc->real_bus_lock);

#if 0
	device_printf(dev, "transfer:\n");
	for (i = 0; i < nmsgs; i++) {
	    device_printf(dev, "  [%d] slave=0x%x, flags=0x%x len=0x%x buf=%p\n", i, msgs[i].slave, msgs[i].flags, msgs[i].len, msgs[i].buf);
	}
#endif

	error = 0;
	for (i = 0; i < nmsgs; i++) {
		bool nonfinal = (i < nmsgs - 1) && (msgs[i].flags & IIC_M_NOSTOP);
		if (msgs[i].flags & IIC_M_RD) {
			error = geniiic_read(sc, msgs[i].slave >> 1, msgs[i].buf, msgs[i].len, nonfinal);
		} else {
			error = geniiic_write(sc, msgs[i].slave >> 1, msgs[i].buf, msgs[i].len, nonfinal);
		}
	}
	if (error) {
		device_printf(dev, "transfer exit %d\n", error);
		for (i = 0; i < nmsgs; i++) {
		    device_printf(dev, "  [%d] slave=0x%x, flags=0x%x len=0x%x buf=%p\n", i, msgs[i].slave, msgs[i].flags, msgs[i].len, msgs[i].buf);
		}
	}
	if (error) {
		geniiic_reset(dev, 0, 0, NULL);
	}
	if (error)
		sc->nfail++;
	else
		sc->nfail = 0;
	sx_xunlock(&sc->real_bus_lock);
	return (error);
}

int
geniiic_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{
	geniiic_softc_t *sc = device_get_softc(dev);
	unsigned u;

	device_printf(dev, "reset\n");
	bus_write_4(sc->regs_res, GENI_M_IRQ_EN, 0);
	bus_write_4(sc->regs_res, GENI_M_IRQ_CLEAR, ~0);
	bus_write_4(sc->regs_res, GENI_DMA_TX_IRQ_EN_CLR, ~0);
	bus_write_4(sc->regs_res, GENI_DMA_TX_IRQ_CLR, ~0);
	bus_write_4(sc->regs_res, GENI_DMA_RX_IRQ_EN_CLR, ~0);
	bus_write_4(sc->regs_res, GENI_DMA_RX_IRQ_CLR, ~0);
	bus_write_4(sc->regs_res, GENI_M_CMD_CTRL_REG, (1<<1));		// S_GENI_CMD_ABORT
	bus_write_4(sc->regs_res, GENI_DMA_RX_FSM_RST, 1);
	for (u = 0; u < 1000; u++) {
		if (bus_read_4(sc->regs_res, GENI_DMA_RX_IRQ_STAT) & 0x8)
			break;
		DELAY(10);
	}
	if (u > 0)
		device_printf(dev, "RXRESET time %u\n", u);
	bus_write_4(sc->regs_res, GENI_DMA_TX_FSM_RST, 1);
	for (u = 0; u < 1000; u++) {
		if (bus_read_4(sc->regs_res, GENI_DMA_TX_IRQ_STAT) & 0x8)
			break;
		DELAY(10);
	}
	if (u > 0)
		device_printf(dev, "TXRESET time %u\n", u);
	return (0);
}

int
geniiic_callback(device_t dev, int index, caddr_t data)
{
	geniiic_softc_t *sc = device_get_softc(dev);
	int error = 0;

	return(0);
	switch (index) {
	case IIC_REQUEST_BUS:
		if (sx_try_xlock(&sc->bus_lock) == 0)
			error = IIC_EBUSBSY;
		else
			sc->bus_locked = true;
		break;

	case IIC_RELEASE_BUS:
		if (!sc->bus_locked) {
			device_printf(dev, "Unlocking unlocked bus\n");
		}
		sc->bus_locked = false;
		sx_xunlock(&sc->bus_lock);
		break;

	default:
		device_printf(dev, "callback unknown %d\n", index);
		error = errno2iic(EINVAL);
	}

	return (error);
}

int
geniiic_attach(geniiic_softc_t *sc)
{
	int error = 0;

	if (bootverbose)
		geni_dump_regs(sc);
	mtx_init(&sc->intr_lock, "geniiic intr lock", NULL, MTX_SPIN);
	sx_init(&sc->real_bus_lock, "geniiic real bus lock");
	sx_init(&sc->bus_lock, "geniiic bus lock");

	sc->rx_fifo_size = (bus_read_4(sc->regs_res, GENI_HW_PARAM_1) >> 16) & 0x3f;
	device_printf(sc->dev, "  RX fifo size= 0x%x\n", sc->rx_fifo_size);

	// We might want to set/check the following registers:
	//	GENI_BYTE_GRANULARITY	(0x00000000)
	//	GENI_TX_PACKING_CFG0	(0x0007f8fe)
	//	GENI_TX_PACKING_CFG1	(000ffefe)
	//	GENI_RX_PACKING_CFG0	(0x0007f8fe)
	//	GENI_RX_PACKING_CFG1	(000ffefe)

	sc->iicbus = device_add_child(sc->dev, "iicbus", DEVICE_UNIT_ANY);
	if (sc->iicbus == NULL) {
		device_printf(sc->dev, "iicbus driver not found\n");
		return(ENXIO);
	}

	error = bus_setup_intr(sc->dev, sc->intr_res, INTR_TYPE_MISC | INTR_MPSAFE,
			       geniiic_intr, NULL, sc, &sc->intr_handle);
	if (error) {
		device_printf(sc->dev,
			      "Unable to setup irq: error %d\n", error);
	}

	if (error) {
		device_printf(sc->dev,
			      "failed to attach child: error %d\n", error);
	} else {
		bus_attach_children(sc->dev);
	}
	return (error);
}

int
geniiic_detach(geniiic_softc_t *sc)
{
	int error = 0;

	if (device_is_attached(sc->dev)) {
		error = bus_generic_detach(sc->dev);
		if (error)
			return (error);
	}
#if 0
	if (sc->iicbus) {
		device_delete_child(sc->dev, sc->iicbus);
	}
#endif

	bus_write_4(sc->regs_res, GENI_M_IRQ_EN, 0);

	if (sc->intr_handle) {
		bus_teardown_intr(sc->dev, sc->intr_res, sc->intr_handle);
	}

	sx_xlock(&sc->bus_lock);
	sx_xlock(&sc->real_bus_lock);

	geniiic_reset(sc->dev, 0, 0, NULL);
	sc->iicbus = NULL;
	sc->intr_handle = NULL;

	sx_xunlock(&sc->real_bus_lock);
	sx_xunlock(&sc->bus_lock);

	sx_destroy(&sc->real_bus_lock);
	sx_destroy(&sc->bus_lock);

	mtx_destroy(&sc->intr_lock);
	return (error);
}

int
geniiic_suspend(geniiic_softc_t *sc)
{
	int error;

	device_printf(sc->dev, "suspend method is NO-OP (good luck!)\n");

	error = bus_generic_suspend(sc->dev);

	return (error);
}

int geniiic_resume(geniiic_softc_t *sc)
{
	int error;

	device_printf(sc->dev, "resume method is NO-OP (good luck!)\n");

	error = bus_generic_resume(sc->dev);

	return (error);
}

DRIVER_MODULE(iicbus, geniiic, iicbus_driver, NULL, NULL);
DRIVER_MODULE(acpi_iicbus, geniiic, acpi_iicbus_driver, NULL, NULL);
MODULE_DEPEND(geniiic, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
MODULE_VERSION(geniiic, 1);
