.include <src.opts.mk>

CONFGROUPS=	CONFETC CONFETCEXEC CONFETCDEFAULTS
CONFETCDIR=	/etc
CONFETC=	network.subr rc rc.initdiskless rc.subr rc.shutdown rc.bsdextended
CONFETCPACKAGE=	rc

.if ${MK_IPFW} != "no"
CONFETC+=	rc.firewall
.endif
CONFETCMODE=	644
CONFETCEXEC=	netstart pccard_ether rc.resume rc.suspend
CONFETCEXECDIR=	/etc
CONFETCEXECMODE=	755
CONFETCEXECPACKAGE=	rc
CONFETCDEFAULTSDIR=	/etc/defaults
CONFETCDEFAULTS=	rc.conf
CONFETCDEFAULTSPACKAGE=	rc

FILESGROUPS=	LIBEXEC_SCRIPTS
LIBEXEC_SCRIPTS=	debug.sh hooks.sh safe_eval.sh
LIBEXEC_SCRIPTSDIR=	/libexec
LIBEXEC_SCRIPTSMODE=	755
LIBEXEC_SCRIPTSPACKAGE=	rc

SUBDIR+=	rc.d

HAS_TESTS=
SUBDIR.${MK_TESTS}+=	tests

.include <bsd.prog.mk>
