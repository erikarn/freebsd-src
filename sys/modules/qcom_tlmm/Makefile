.PATH:	${SRCTOP}/sys/dev/qcom_tlmm

KMOD=	qcom_tlmm

SRCS=	qcom_tlmm.c qcom_tlmm_debug.c qcom_tlmm_pin.c qcom_tlmm_pinmux.c
SRCS+=	qcom_tlmm_ipq4018.c qcom_tlmm_ipq4018_hw.c
SRCS+=	qcom_tlmm_x1e.c qcom_tlmm_x1e_hw.c

CFLAGS+=  -I. -I${SRCTOP}/sys/dev/qcom_tlmm

.include <bsd.kmod.mk>
