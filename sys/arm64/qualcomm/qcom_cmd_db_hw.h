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

#ifndef	__QCOM_CMD_DB_HW_H__
#define	__QCOM_CMD_DB_HW_H__

#define	QCOM_CMD_DB_NUM_PRIORITY	2
#define	QCOM_CMD_DB_MAX_SLAVE_ID	8	/* TODO: should be NUM_SLAVES or something, the max id is 0..7 */
#define	QCOM_CMD_DB_ENTRY_HDR_ID_SIZE	8

/* Header magic - little-endian */
#define	QCOM_CMD_DB_HEADER_MAGIC	0x0c0330db

typedef enum {
	QCOM_CMD_DB_HW_TYPE_INVALID = 0,
	QCOM_CMD_DB_HW_TYPE_ARC = 3,
	QCOM_CMD_DB_HW_TYPE_VRM = 4,
	QCOM_CMD_DB_HW_TYPE_BCM = 5,
} qcom_cmd_db_hw_type_t;

/**
 * @id: resource identifier
 * @priority: little endian, not used
 * @addr: little endian, address of the resource
 * @len: length of the data
 * @offset: start of data, offset from qcom_cmd_db_rsc_hdr:data_offset
 */
struct qcom_cmd_db_entry_hdr {
	uint8_t id[QCOM_CMD_DB_ENTRY_HDR_ID_SIZE];
	uint32_t priority[QCOM_CMD_DB_NUM_PRIORITY];
	uint32_t addr;
	uint16_t len;
	uint16_t offset;
} __packed;

/**
 * @slave_id:
 * @header_offset:
 * @data_offset:
 * @count:
 * @version: version, high 8 bits = major, low 8 bits = minor
 */
struct qcom_cmd_db_rsc_hdr {
	uint16_t slave_id;
	uint16_t header_offset;
	uint16_t data_offset;
	uint16_t count;
	uint16_t version;
	uint16_t reserved[3];
} __packed;

struct qcom_cmd_db_hdr {
	uint32_t version;
	uint32_t magic;
	struct qcom_cmd_db_rsc_hdr rsc_header[QCOM_CMD_DB_MAX_SLAVE_ID];
	uint32_t checksum;
	uint32_t reserved;
	/* Data follows */
} __packed;

#endif	/* __QCOM_CMD_DB_HW_H__ */
