
.PATH: ${SRCTOP}/sys/net80211
.PATH: ${SRCTOP}/sys/crypto/rijndael

KMOD=	wlan_bip_gmac
SRCS=	ieee80211_crypto_bip_gmac.c
SRCS+=	rijndael-alg-fst.c rijndael-api.c
SRCS+=	opt_wlan.h

.include <bsd.kmod.mk>
