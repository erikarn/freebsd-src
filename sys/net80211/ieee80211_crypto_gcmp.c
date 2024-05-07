/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2002-2008 Sam Leffler, Errno Consulting
 * Copyright (c) 2024 Adrian Chadd <adrian@FreeBSD.org>
 * All rights reserved.
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

#include <sys/cdefs.h>
/*
 * IEEE 802.11 AES-GCMP crypto support.
 *
 * Part of this module is derived from similar code in the Host
 * AP driver. The code is used with the consent of the author and
 * it's license is included below.
 */
#include "opt_wlan.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <sys/socket.h>

#include <net/if.h>
#include <net/if_media.h>
#include <net/ethernet.h>

#include <net80211/ieee80211_var.h>

#include <crypto/rijndael/rijndael.h>

#define AES_BLOCK_LEN 16

/*
 * buffer is 2x the AES_BLOCK_LEN, but the AAD contents may be variable
 * and are padded.
 */
#define	GCM_AAD_LEN	(AES_BLOCK_LEN * 2)

/* GCMP is always 128 bit / 16 byte MIC */
#define	GCMP_MIC_LEN	16
#define	GCMP_PN_LEN	6
#define	GCMP_IV_LEN	12

struct gcmp_ctx {
	struct ieee80211vap *cc_vap;	/* for diagnostics+statistics */
	struct ieee80211com *cc_ic;
	rijndael_ctx	     cc_aes;
};

static	void *gcmp_attach(struct ieee80211vap *, struct ieee80211_key *);
static	void gcmp_detach(struct ieee80211_key *);
static	int gcmp_setkey(struct ieee80211_key *);
static	int gcmp_256_setkey(struct ieee80211_key *);
static	void gcmp_setiv(struct ieee80211_key *, uint8_t *);
static	int gcmp_encap(struct ieee80211_key *, struct mbuf *);
static	int gcmp_decap(struct ieee80211_key *, struct mbuf *, int);
static	int gcmp_enmic(struct ieee80211_key *, struct mbuf *, int);
static	int gcmp_demic(struct ieee80211_key *, struct mbuf *, int);

static const struct ieee80211_cipher gcmp_128 = {
	.ic_name	= "AES-GCMP",
	.ic_cipher	= IEEE80211_CIPHER_AES_GCM_128,
	.ic_header	= IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN +
			  IEEE80211_WEP_EXTIVLEN,
	.ic_trailer	= GCMP_MIC_LEN,
	.ic_miclen	= 0,
	.ic_attach	= gcmp_attach,
	.ic_detach	= gcmp_detach,
	.ic_setkey	= gcmp_setkey,
	.ic_setiv	= gcmp_setiv,
	.ic_encap	= gcmp_encap,
	.ic_decap	= gcmp_decap,
	.ic_enmic	= gcmp_enmic,
	.ic_demic	= gcmp_demic,
};

static const struct ieee80211_cipher gcmp_256 = {
	.ic_name	= "AES-GCMP-256",
	.ic_cipher	= IEEE80211_CIPHER_AES_GCM_256,
	.ic_header	= IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN +
			  IEEE80211_WEP_EXTIVLEN,
	.ic_trailer	= GCMP_MIC_LEN,
	.ic_miclen	= 0,
	.ic_attach	= gcmp_attach,
	.ic_detach	= gcmp_detach,
	.ic_setkey	= gcmp_256_setkey,
	.ic_setiv	= gcmp_setiv,
	.ic_encap	= gcmp_encap,
	.ic_decap	= gcmp_decap,
	.ic_enmic	= gcmp_enmic,
	.ic_demic	= gcmp_demic,
};


static	int gcmp_encrypt(struct ieee80211_key *, struct mbuf *, int hdrlen);
static	int gcmp_decrypt(struct ieee80211_key *, u_int64_t pn,
		struct mbuf *, int hdrlen);

/* number of references from net80211 layer */
static	int nrefs = 0;

static void *
gcmp_attach(struct ieee80211vap *vap, struct ieee80211_key *k)
{
	struct gcmp_ctx *ctx;

	ctx = (struct gcmp_ctx *) IEEE80211_MALLOC(sizeof(struct gcmp_ctx),
		M_80211_CRYPTO, IEEE80211_M_NOWAIT | IEEE80211_M_ZERO);
	if (ctx == NULL) {
		vap->iv_stats.is_crypto_nomem++;
		return NULL;
	}
	ctx->cc_vap = vap;
	ctx->cc_ic = vap->iv_ic;
	nrefs++;			/* NB: we assume caller locking */
	return ctx;
}

static void
gcmp_detach(struct ieee80211_key *k)
{
	struct gcmp_ctx *ctx = k->wk_private;

	IEEE80211_FREE(ctx, M_80211_CRYPTO);
	KASSERT(nrefs > 0, ("imbalanced attach/detach"));
	nrefs--;			/* NB: we assume caller locking */
}

static int
gcmp_get_trailer_len(struct ieee80211_key *k)
{
	return k->wk_cipher->ic_trailer;
}

static int
gcmp_get_header_len(struct ieee80211_key *k)
{
	return k->wk_cipher->ic_header;
}

static int
gcmp_setkey(struct ieee80211_key *k)
{
	struct gcmp_ctx *ctx = k->wk_private;

	if (k->wk_keylen != (128/NBBY)) {
		IEEE80211_DPRINTF(ctx->cc_vap, IEEE80211_MSG_CRYPTO,
			"%s: Invalid key length %u, expecting %u\n",
			__func__, k->wk_keylen, 128/NBBY);
		return 0;
	}
	if (k->wk_flags & IEEE80211_KEY_SWENCRYPT)
		rijndael_set_key(&ctx->cc_aes, k->wk_key, k->wk_keylen*NBBY);
	return 1;
}

static int
gcmp_256_setkey(struct ieee80211_key *k)
{
	struct gcmp_ctx *ctx = k->wk_private;

	if (k->wk_keylen != (256/NBBY)) {
		IEEE80211_DPRINTF(ctx->cc_vap, IEEE80211_MSG_CRYPTO,
			"%s: Invalid key length %u, expecting %u\n",
			__func__, k->wk_keylen, 256/NBBY);
		return 0;
	}
	if (k->wk_flags & IEEE80211_KEY_SWENCRYPT)
		rijndael_set_key(&ctx->cc_aes, k->wk_key, k->wk_keylen*NBBY);
	return 1;
}

static void
gcmp_setiv(struct ieee80211_key *k, uint8_t *ivp)
{
	struct gcmp_ctx *ctx = k->wk_private;
	struct ieee80211vap *vap = ctx->cc_vap;
	uint8_t keyid;

	keyid = ieee80211_crypto_get_keyid(vap, k) << 6;

	k->wk_keytsc++;
	ivp[0] = k->wk_keytsc >> 0;		/* PN0 */
	ivp[1] = k->wk_keytsc >> 8;		/* PN1 */
	ivp[2] = 0;				/* Reserved */
	ivp[3] = keyid | IEEE80211_WEP_EXTIV;	/* KeyID | ExtID */
	ivp[4] = k->wk_keytsc >> 16;		/* PN2 */
	ivp[5] = k->wk_keytsc >> 24;		/* PN3 */
	ivp[6] = k->wk_keytsc >> 32;		/* PN4 */
	ivp[7] = k->wk_keytsc >> 40;		/* PN5 */
}

/*
 * Add privacy headers appropriate for the specified key.
 */
static int
gcmp_encap(struct ieee80211_key *k, struct mbuf *m)
{
	const struct ieee80211_frame *wh;
	struct gcmp_ctx *ctx = k->wk_private;
	struct ieee80211com *ic = ctx->cc_ic;
	uint8_t *ivp;
	int hdrlen;
	int is_mgmt;

	hdrlen = ieee80211_hdrspace(ic, mtod(m, void *));
	wh = mtod(m, const struct ieee80211_frame *);
	is_mgmt = IEEE80211_IS_MGMT(wh);

	/*
	 * Check to see if we need to insert IV/MIC.
	 *
	 * Some offload devices don't require the IV to be inserted
	 * as part of the hardware encryption.
	 */
	if (is_mgmt && (k->wk_flags & IEEE80211_KEY_NOIVMGT))
		return 1;
	if ((! is_mgmt) && (k->wk_flags & IEEE80211_KEY_NOIV))
		return 1;

	/*
	 * Copy down 802.11 header and add the IV, KeyID, and ExtIV.
	 */
	M_PREPEND(m, gcmp_get_header_len(k), IEEE80211_M_NOWAIT);
	if (m == NULL)
		return 0;
	ivp = mtod(m, uint8_t *);
	ovbcopy(ivp + gcmp_get_header_len(k), ivp, hdrlen);
	ivp += hdrlen;

	gcmp_setiv(k, ivp);

	/*
	 * Finally, do software encrypt if needed.
	 */
	if ((k->wk_flags & IEEE80211_KEY_SWENCRYPT) &&
	    !gcmp_encrypt(k, m, hdrlen))
		return 0;

	return 1;
}

/*
 * Add MIC to the frame as needed.
 */
static int
gcmp_enmic(struct ieee80211_key *k, struct mbuf *m, int force)
{

	return 1;
}

static __inline uint64_t
READ_6(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5)
{
	uint32_t iv32 = (b0 << 0) | (b1 << 8) | (b2 << 16) | (b3 << 24);
	uint16_t iv16 = (b4 << 0) | (b5 << 8);
	return (((uint64_t)iv16) << 32) | iv32;
}

/*
 * Validate and strip privacy headers (and trailer) for a
 * received frame. The specified key should be correct but
 * is also verified.
 */
static int
gcmp_decap(struct ieee80211_key *k, struct mbuf *m, int hdrlen)
{
	const struct ieee80211_rx_stats *rxs;
	struct gcmp_ctx *ctx = k->wk_private;
	struct ieee80211vap *vap = ctx->cc_vap;
	struct ieee80211_frame *wh;
	uint8_t *ivp, tid;
	uint64_t pn;

	rxs = ieee80211_get_rx_params_ptr(m);

	if ((rxs != NULL) && (rxs->c_pktflags & IEEE80211_RX_F_IV_STRIP))
		goto finish;

	/*
	 * Header should have extended IV and sequence number;
	 * verify the former and validate the latter.
	 */
	wh = mtod(m, struct ieee80211_frame *);
	ivp = mtod(m, uint8_t *) + hdrlen;
	if ((ivp[IEEE80211_WEP_IVLEN] & IEEE80211_WEP_EXTIV) == 0) {
		/*
		 * No extended IV; discard frame.
		 */
		IEEE80211_NOTE_MAC(vap, IEEE80211_MSG_CRYPTO, wh->i_addr2,
			"%s", "missing ExtIV for AES-GCM cipher");
		vap->iv_stats.is_rx_gcmpformat++;
		return 0;
	}
	tid = ieee80211_gettid(wh);
	pn = READ_6(ivp[0], ivp[1], ivp[4], ivp[5], ivp[6], ivp[7]);
	if (pn <= k->wk_keyrsc[tid] &&
	    (k->wk_flags & IEEE80211_KEY_NOREPLAY) == 0) {
		/*
		 * Replay violation.
		 */
		ieee80211_notify_replay_failure(vap, wh, k, pn, tid);
		vap->iv_stats.is_rx_gcmpreplay++;
		return 0;
	}

	/*
	 * Check if the device handled the decrypt in hardware.
	 * If so we just strip the header; otherwise we need to
	 * handle the decrypt in software.  Note that for the
	 * latter we leave the header in place for use in the
	 * decryption work.
	 */
	if ((k->wk_flags & IEEE80211_KEY_SWDECRYPT) &&
	    !gcmp_decrypt(k, pn, m, hdrlen))
		return 0;

finish:
	/*
	 * Copy up 802.11 header and strip crypto bits.
	 */
	if (! ((rxs != NULL) && (rxs->c_pktflags & IEEE80211_RX_F_IV_STRIP))) {
		ovbcopy(mtod(m, void *), mtod(m, uint8_t *) + gcmp_get_header_len(k),
		    hdrlen);
		m_adj(m, gcmp_get_header_len(k));
	}

	/*
	 * XXX TODO: see if MMIC_STRIP also covers CCMP MIC trailer.
	 */
	if (! ((rxs != NULL) && (rxs->c_pktflags & IEEE80211_RX_F_MMIC_STRIP)))
		m_adj(m, -gcmp_get_trailer_len(k));

	/*
	 * Ok to update rsc now.
	 */
	if (! ((rxs != NULL) && (rxs->c_pktflags & IEEE80211_RX_F_IV_STRIP))) {
		k->wk_keyrsc[tid] = pn;
	}

	return 1;
}

/*
 * Verify and strip MIC from the frame.
 */
static int
gcmp_demic(struct ieee80211_key *k, struct mbuf *m, int force)
{
	return 1;
}

static __inline void
xor_block(uint8_t *b, const uint8_t *a, size_t len)
{
	int i;
	for (i = 0; i < len; i++)
		b[i] ^= a[i];
}

/*
 * Galois/Counter Mode (GCM) and GMAC with AES
 *
 * Copyright (c) 2012, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#define BIT(x)	(1U << (x))

static inline void WPA_PUT_BE64(uint8_t *a, uint64_t val)
{
        a[0] = val >> 56;
        a[1] = val >> 48;
        a[2] = val >> 40;
        a[3] = val >> 32;
        a[4] = val >> 24;
        a[5] = val >> 16;
        a[6] = val >> 8;
        a[7] = val & 0xff;
}

static inline void WPA_PUT_BE32(uint8_t *a, uint32_t val)
{
        a[0] = (val >> 24) & 0xff;
        a[1] = (val >> 16) & 0xff;
        a[2] = (val >> 8) & 0xff;
        a[3] = val & 0xff;
}

static inline uint32_t WPA_GET_BE32(const uint8_t *a)
{
        return ((uint32_t) a[0] << 24) | (a[1] << 16) | (a[2] << 8) | a[3];
}

static void inc32(uint8_t *block)
{
	uint32_t val;
	val = WPA_GET_BE32(block + AES_BLOCK_LEN - 4);
	val++;
	WPA_PUT_BE32(block + AES_BLOCK_LEN - 4, val);
}

static void shift_right_block(uint8_t *v)
{
	uint32_t val;

	val = WPA_GET_BE32(v + 12);
	val >>= 1;
	if (v[11] & 0x01)
		val |= 0x80000000;
	WPA_PUT_BE32(v + 12, val);

	val = WPA_GET_BE32(v + 8);
	val >>= 1;
	if (v[7] & 0x01)
		val |= 0x80000000;
	WPA_PUT_BE32(v + 8, val);

	val = WPA_GET_BE32(v + 4);
	val >>= 1;
	if (v[3] & 0x01)
		val |= 0x80000000;
	WPA_PUT_BE32(v + 4, val);

	val = WPA_GET_BE32(v);
	val >>= 1;
	WPA_PUT_BE32(v, val);
}


/* Multiplication in GF(2^128) */
static void gf_mult(const uint8_t *x, const uint8_t *y, uint8_t *z)
{
	uint8_t v[16];
	int i, j;

	memset(z, 0, 16); /* Z_0 = 0^128 */
	memcpy(v, y, 16); /* V_0 = Y */

	for (i = 0; i < 16; i++) {
		for (j = 0; j < 8; j++) {
			if (x[i] & BIT(7 - j)) {
				/* Z_(i + 1) = Z_i XOR V_i */
				xor_block(z, v, AES_BLOCK_LEN);
			} else {
				/* Z_(i + 1) = Z_i */
			}

			if (v[15] & 0x01) {
				/* V_(i + 1) = (V_i >> 1) XOR R */
				shift_right_block(v);
				/* R = 11100001 || 0^120 */
				v[0] ^= 0xe1;
			} else {
				/* V_(i + 1) = V_i >> 1 */
				shift_right_block(v);
			}
		}
	}
}


static void ghash_start(uint8_t *y)
{
	/* Y_0 = 0^128 */
	memset(y, 0, 16);
}


static void ghash(const uint8_t *h, const uint8_t *x, size_t xlen, uint8_t *y)
{
	size_t m, i;
	const uint8_t *xpos = x;
	uint8_t tmp[16];

	m = xlen / 16;

	for (i = 0; i < m; i++) {
		/* Y_i = (Y^(i-1) XOR X_i) dot H */
		xor_block(y, xpos, AES_BLOCK_LEN);
		xpos += 16;

		/* dot operation:
		 * multiplication operation for binary Galois (finite) field of
		 * 2^128 elements */
		gf_mult(y, h, tmp);
		memcpy(y, tmp, 16);
	}

	if (x + xlen > xpos) {
		/* Add zero padded last block */
		size_t last = x + xlen - xpos;
		memcpy(tmp, xpos, last);
		memset(tmp + last, 0, sizeof(tmp) - last);

		/* Y_i = (Y^(i-1) XOR X_i) dot H */
		xor_block(y, tmp, AES_BLOCK_LEN);

		/* dot operation:
		 * multiplication operation for binary Galois (finite) field of
		 * 2^128 elements */
		gf_mult(y, h, tmp);
		memcpy(y, tmp, 16);
	}

	/* Return Y_m */
}


/*
 * Execute the GCTR call with the counter block icb
 * on payload x (size len), output into y.
 */
static void
aes_gctr(rijndael_ctx *aes, const uint8_t *icb,
    const uint8_t *x, size_t xlen, uint8_t *y)
{
	size_t i, n, last;
	uint8_t cb[AES_BLOCK_LEN], tmp[AES_BLOCK_LEN];
	const uint8_t *xpos = x;
	uint8_t *ypos = y;

	if (xlen == 0)
		return;

	n = xlen / 16;

	memcpy(cb, icb, AES_BLOCK_LEN);

	/* Full blocks */
	for (i = 0; i < n; i++) {
		rijndael_encrypt(aes, cb, ypos);
		xor_block(ypos, xpos, AES_BLOCK_LEN);
		xpos += AES_BLOCK_LEN;
		ypos += AES_BLOCK_LEN;
		inc32(cb);
	}

	last = x + xlen - xpos;
	if (last) {
		/* Last, partial block */
		rijndael_encrypt(aes, cb, tmp);
		for (i = 0; i < last; i++)
			*ypos++ = *xpos++ ^ tmp[i];
	}
}

static void
aes_gcm_init_hash_subkey(rijndael_ctx *aes, uint8_t *H)
{
	/* Generate hash subkey H = AES_K(0^128) */
	memset(H, 0, AES_BLOCK_LEN);

	rijndael_encrypt(aes, H, H);
}

static void
aes_gcm_prepare_j0(const uint8_t *iv, size_t iv_len, const uint8_t *H,
    uint8_t *J0)
{
	uint8_t len_buf[16];

	if (iv_len == 12) {
		/* Prepare block J_0 = IV || 0^31 || 1 [len(IV) = 96] */
		memcpy(J0, iv, iv_len);
		memset(J0 + iv_len, 0, AES_BLOCK_LEN - iv_len);
		J0[AES_BLOCK_LEN - 1] = 0x01;
	} else {
		/*
		 * s = 128 * ceil(len(IV)/128) - len(IV)
		 * J_0 = GHASH_H(IV || 0^(s+64) || [len(IV)]_64)
		 */
		ghash_start(J0);
		ghash(H, iv, iv_len, J0);
		WPA_PUT_BE64(len_buf, 0);
		WPA_PUT_BE64(len_buf + 8, iv_len * 8);
		ghash(H, len_buf, sizeof(len_buf), J0);
	}
}


static void
aes_gcm_gctr(rijndael_ctx *aes, const uint8_t *J0, const uint8_t *in,
    size_t len, uint8_t *out)
{
	uint8_t J0inc[AES_BLOCK_LEN];

	if (len == 0)
		return;

	memcpy(J0inc, J0, AES_BLOCK_LEN);
	inc32(J0inc);

	aes_gctr(aes, J0inc, in, len, out);
}

static void
aes_gcm_ghash(const uint8_t *H, const uint8_t *aad, size_t aad_len,
    const uint8_t *crypt, size_t crypt_len, uint8_t *S)
{
	uint8_t len_buf[16];

	/*
	 * u = 128 * ceil[len(C)/128] - len(C)
	 * v = 128 * ceil[len(A)/128] - len(A)
	 * S = GHASH_H(A || 0^v || C || 0^u || [len(A)]64 || [len(C)]64)
	 * (i.e., zero padded to block size A || C and lengths of each in bits)
	 */
	ghash_start(S);
	ghash(H, aad, aad_len, S);
	ghash(H, crypt, crypt_len, S);
	WPA_PUT_BE64(len_buf, aad_len * 8);
	WPA_PUT_BE64(len_buf + 8, crypt_len * 8);
	ghash(H, len_buf, sizeof(len_buf), S);
#if 0
	wpa_hexdump_key(MSG_EXCESSIVE, "S = GHASH_H(...)", S, 16);
#endif
}

/**
 * aes_gcm_ae - GCM-AE_K(IV, P, A)
 */
static void
aes_gcm_ae(rijndael_ctx *aes, const uint8_t *iv, size_t iv_len,
    const uint8_t *plain, size_t plain_len,
    const uint8_t *aad, size_t aad_len, uint8_t *crypt, uint8_t *tag)
{
	uint8_t H[AES_BLOCK_LEN];
	uint8_t J0[AES_BLOCK_LEN];
	uint8_t S[GCMP_MIC_LEN];

	aes_gcm_init_hash_subkey(aes, H);

	aes_gcm_prepare_j0(iv, iv_len, H, J0);

	/* C = GCTR_K(inc_32(J_0), P) */
	aes_gcm_gctr(aes, J0, plain, plain_len, crypt);

	aes_gcm_ghash(H, aad, aad_len, crypt, plain_len, S);

	/* T = MSB_t(GCTR_K(J_0, S)) */
	aes_gctr(aes, J0, S, sizeof(S), tag);

	/* Return (C, T) */
}


/**
 * aes_gcm_ad - GCM-AD_K(IV, C, A, T)
 *
 * Return 0 if OK, -1 if decrypt failure.
 */
static int
aes_gcm_ad(rijndael_ctx *aes, const uint8_t *iv, size_t iv_len,
    const uint8_t *crypt, size_t crypt_len,
    const uint8_t *aad, size_t aad_len, const uint8_t *tag, uint8_t *plain)
{
	uint8_t H[AES_BLOCK_LEN];
	uint8_t J0[AES_BLOCK_LEN];
	uint8_t S[16], T[GCMP_MIC_LEN];

	aes_gcm_init_hash_subkey(aes, H);

	aes_gcm_prepare_j0(iv, iv_len, H, J0);

	/* P = GCTR_K(inc_32(J_0), C) */
	aes_gcm_gctr(aes, J0, crypt, crypt_len, plain);

	aes_gcm_ghash(H, aad, aad_len, crypt, crypt_len, S);

	/* T' = MSB_t(GCTR_K(J_0, S)) */
	aes_gctr(aes, J0, S, sizeof(S), T);

	if (memcmp(tag, T, 16) != 0) {
		return -1;
	}

	return 0;
}

#if 0
int
aes_gmac(const uint8_t *key, size_t key_len, const uint8_t *iv, size_t iv_len,
    const uint8_t *aad, size_t aad_len, uint8_t *tag)
{
	return aes_gcm_ae(key, key_len, iv, iv_len, NULL, 0, aad, aad_len,
	    NULL, tag);
}
#endif

/* Back to FreeBSD ... */

/*
 * TODO: eventually refactor this out as shared between CCM and GCM
 *
 * NOTE: the first two bytes are a 16 bit big-endian length, likely for CCM.
 * This isn't required for GCM.
 */
static int
gcmp_init_aad(const struct ieee80211_frame *wh, uint8_t *aad)
{
	int aad_len;

	memset(aad, 0, GCM_AAD_LEN);

#define	IS_QOS_DATA(wh)	IEEE80211_QOS_HAS_SEQ(wh)
	/* AAD:
	 * FC with bits 4..6 and 11..13 masked to zero; 14 is always one
	 * A1 | A2 | A3
	 * SC with bits 4..15 (seq#) masked to zero
	 * A4 (if present)
	 * QC (if present)
	 */
	aad[0] = 0;	/* AAD length >> 8 */
	/* NB: aad[1] set below */

	aad[2] = wh->i_fc[0] & 0x8f;	/* XXX magic #s */
	/* TODO: 12.5.3.3.3 - bit 14 should always be set; bit 15 masked to 0 if QoS control field, unmasked otherwise */
	aad[3] = wh->i_fc[1] & 0xc7;	/* XXX magic #s */
	/* NB: we know 3 addresses are contiguous */
	memcpy(aad + 4, wh->i_addr1, 3 * IEEE80211_ADDR_LEN);
	aad[22] = wh->i_seq[0] & IEEE80211_SEQ_FRAG_MASK;
	aad[23] = 0; /* all bits masked */
	/*
	 * Construct variable-length portion of AAD based
	 * on whether this is a 4-address frame/QOS frame.
	 * We always zero-pad to 32 bytes before running it
	 * through the cipher.
	 */
	if (IEEE80211_IS_DSTODS(wh)) {
		IEEE80211_ADDR_COPY(aad + 24,
			((const struct ieee80211_frame_addr4 *)wh)->i_addr4);
		if (IS_QOS_DATA(wh)) {
			const struct ieee80211_qosframe_addr4 *qwh4 =
				(const struct ieee80211_qosframe_addr4 *) wh;
			aad[30] = qwh4->i_qos[0] & 0x0f;/* just priority bits */
			aad[31] = 0;
			aad_len = aad[1] = 22 + IEEE80211_ADDR_LEN + 2;
		} else {
			*(uint16_t *)&aad[30] = 0;
			aad_len = aad[1] = 22 + IEEE80211_ADDR_LEN;
		}
	} else {
		if (IS_QOS_DATA(wh)) {
			const struct ieee80211_qosframe *qwh =
				(const struct ieee80211_qosframe*) wh;
			aad[24] = qwh->i_qos[0] & 0x0f;	/* just priority bits */
			aad[25] = 0;
			aad_len = aad[1] = 22 + 2;
		} else {
			*(uint16_t *)&aad[24] = 0;
			aad_len = aad[1] = 22;
		}
		*(uint16_t *)&aad[26] = 0;
		*(uint32_t *)&aad[28] = 0;
	}
#undef	IS_QOS_DATA

	return aad_len;
}

/*
 * Populate the 12 byte / 96 bit IV buffer.
 */
static int
gcmp_init_iv(uint8_t *iv, const struct ieee80211_frame *wh, u_int64_t pn)
{
	uint8_t j_pn[GCMP_PN_LEN];

	/* Construct the pn buffer */
	j_pn[0] = pn >> 40;
	j_pn[1] = pn >> 32;
	j_pn[2] = pn >> 24;
	j_pn[3] = pn >> 16;
	j_pn[4] = pn >> 8;
	j_pn[5] = pn >> 0;

	memcpy(iv, wh->i_addr2, IEEE80211_ADDR_LEN);
	memcpy(iv + IEEE80211_ADDR_LEN, j_pn, GCMP_PN_LEN);

	return GCMP_IV_LEN; /* 96 bits */
}


/*
 * Encrypt an mbuf.
 *
 * This uses a temporary memory buffer to encrypt; the
 * current AES-GCM code expects things in a contiguous buffer
 * and this avoids the need of breaking out the GCTR and
 * GHASH routines into using mbuf iterators.
 */

static int
gcmp_encrypt(struct ieee80211_key *key, struct mbuf *m0, int hdrlen)
{
	struct gcmp_ctx *ctx = key->wk_private;
	struct ieee80211_frame *wh;
	struct ieee80211vap *vap = ctx->cc_vap;
	struct mbuf *m = m0;
	int data_len, aad_len, iv_len, ret;
	uint8_t aad[GCM_AAD_LEN];
	uint8_t T[GCMP_MIC_LEN];
	uint8_t iv[GCMP_IV_LEN];
	uint8_t *p_pktbuf = NULL;
	uint8_t *c_pktbuf = NULL;

	wh = mtod(m, struct ieee80211_frame *);
	data_len = m->m_pkthdr.len - (hdrlen + gcmp_get_header_len(key));

	ctx->cc_vap->iv_stats.is_crypto_gcmp++;

	p_pktbuf = IEEE80211_MALLOC(data_len, M_TEMP,
	    IEEE80211_M_NOWAIT | IEEE80211_M_ZERO);
	if (p_pktbuf == NULL) {
		IEEE80211_NOTE_MAC(vap, IEEE80211_MSG_CRYPTO, wh->i_addr2,
		    "%s", "AES-GCM encrypt failed; couldn't allocate buffer");
		/* XXX counter */
		return 0;
	}
	c_pktbuf = IEEE80211_MALLOC(data_len, M_TEMP,
	    IEEE80211_M_NOWAIT | IEEE80211_M_ZERO);
	if (c_pktbuf == NULL) {
		IEEE80211_NOTE_MAC(vap, IEEE80211_MSG_CRYPTO, wh->i_addr2,
		    "%s", "AES-GCM encrypt failed; couldn't allocate buffer");
		/* XXX counter */
		IEEE80211_FREE(p_pktbuf, M_TEMP);
		return 0;
	}

	/* Initialise AAD */
	aad_len = gcmp_init_aad(wh, aad);

	/* Initialise local Nonce to work on */
	/* TODO: rename iv stuff here to nonce */
	iv_len = gcmp_init_iv(iv, wh, key->wk_keytsc);

	/* Copy mbuf data part into plaintext pktbuf */
	m_copydata(m0, hdrlen + gcmp_get_header_len(key), data_len,
	    p_pktbuf);

	/* Run encrypt */
	aes_gcm_ae(&ctx->cc_aes, iv, iv_len,
	    p_pktbuf, data_len,
	    aad + 2, aad_len,
	    c_pktbuf,
	    T);

	/* Copy data back over mbuf */
	m_copyback(m0, hdrlen + gcmp_get_header_len(key), data_len,
	    c_pktbuf);

	/* Append MIC */
	ret = m_append(m0, gcmp_get_trailer_len(key), T);
	if (ret == 0) {
		IEEE80211_NOTE_MAC(vap, IEEE80211_MSG_CRYPTO, wh->i_addr2,
		    "%s", "AES-GCM encrypt failed; couldn't append T");
		/* XXX counter on error */
	}

	IEEE80211_FREE(p_pktbuf, M_TEMP);
	IEEE80211_FREE(c_pktbuf, M_TEMP);

	return ret;
}

static int
gcmp_decrypt(struct ieee80211_key *key, u_int64_t pn, struct mbuf *m, int hdrlen)
{
	struct gcmp_ctx *ctx = key->wk_private;
	struct ieee80211vap *vap = ctx->cc_vap;
	struct ieee80211_frame *wh;
	int data_len, aad_len, iv_len, ret;
	uint8_t aad[GCM_AAD_LEN];
	uint8_t T[GCMP_MIC_LEN];
	uint8_t iv[GCMP_IV_LEN];
	uint8_t *p_pktbuf = NULL;
	uint8_t *c_pktbuf = NULL;

	wh = mtod(m, struct ieee80211_frame *);

	/* Data length doesn't include the MIC at the end */
	data_len = m->m_pkthdr.len -
	    (hdrlen + gcmp_get_header_len(key) + GCMP_MIC_LEN);

	ctx->cc_vap->iv_stats.is_crypto_gcmp++;

	p_pktbuf = IEEE80211_MALLOC(data_len, M_TEMP,
	    IEEE80211_M_NOWAIT | IEEE80211_M_ZERO);
	if (p_pktbuf == NULL) {
		/* XXX counter */
		return 0;
	}
	c_pktbuf = IEEE80211_MALLOC(data_len, M_TEMP,
	    IEEE80211_M_NOWAIT | IEEE80211_M_ZERO);
	if (c_pktbuf == NULL) {
		/* XXX counter */
		IEEE80211_FREE(p_pktbuf, M_TEMP);
		return 0;
	}

	/* Initialise AAD */
	aad_len = gcmp_init_aad(wh, aad);

	/* Initialise local IV copy to work on */
	iv_len = gcmp_init_iv(iv, wh, pn);

	/* Copy mbuf into ciphertext pktbuf */
	m_copydata(m, hdrlen + gcmp_get_header_len(key), data_len,
	    c_pktbuf);

	/* Copy the MIC into the tag buffer */
	m_copydata(m, hdrlen + gcmp_get_header_len(key) + data_len,
	    GCMP_MIC_LEN, T);

	/* Run decrypt */
	ret = aes_gcm_ad(&ctx->cc_aes, iv, iv_len,
	    c_pktbuf, data_len,
	    aad + 2, aad_len,
	    T, p_pktbuf);

	if (ret != 0) {
		/* Decrypt failure */
		ctx->cc_vap->iv_stats.is_rx_gcmpmic++;
		IEEE80211_NOTE_MAC(vap, IEEE80211_MSG_CRYPTO, wh->i_addr2,
		    "%s", "AES-GCM decrypt failed; MIC mismatch");
		IEEE80211_FREE(p_pktbuf, M_TEMP);
		IEEE80211_FREE(c_pktbuf, M_TEMP);
		return 0;
	}

	/* Copy data back over mbuf */
	m_copyback(m, hdrlen + gcmp_get_header_len(key), data_len,
	    p_pktbuf);

	IEEE80211_FREE(p_pktbuf, M_TEMP);
	IEEE80211_FREE(c_pktbuf, M_TEMP);

	return 1;
}

/*
 * Module glue.
 */
IEEE80211_CRYPTO_MODULE(gcmp_128, 1);
IEEE80211_CRYPTO_MODULE_ADD(gcmp_256);
