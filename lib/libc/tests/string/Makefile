# ensure libc functions are tested, not clang's builtins
CFLAGS+=	-fno-builtin

ATF_TESTS_C+=		bcmp_test
ATF_TESTS_C+=		ffs_test
ATF_TESTS_C+=		ffsl_test
ATF_TESTS_C+=		ffsll_test
ATF_TESTS_C+=		fls_test
ATF_TESTS_C+=		flsl_test
ATF_TESTS_C+=		flsll_test
ATF_TESTS_C+=		memccpy_test
ATF_TESTS_C+=		memcmp_test
ATF_TESTS_C+=		memrchr_test
ATF_TESTS_C+=		memset2_test
ATF_TESTS_C+=		memset_s_test
ATF_TESTS_C+=		strncmp_test
ATF_TESTS_C+=		stpncpy_test
ATF_TESTS_C+=		strnlen_test
ATF_TESTS_C+=		strcmp2_test
ATF_TESTS_C+=		strcspn_test
ATF_TESTS_C+=		strerror2_test
ATF_TESTS_C+=		strlcpy_test
ATF_TESTS_C+=		strspn_test
ATF_TESTS_C+=		strverscmp_test
ATF_TESTS_C+=		strxfrm_test
ATF_TESTS_C+=		timingsafe_bcmp_test
ATF_TESTS_C+=		timingsafe_memcmp_test
ATF_TESTS_C+=		wcscasecmp_test
ATF_TESTS_C+=		wcscoll_test
ATF_TESTS_C+=		wcsnlen_test

# TODO: popcount, stresep

NETBSD_ATF_TESTS_C+=	memchr_test
NETBSD_ATF_TESTS_C+=	memcpy_test
NETBSD_ATF_TESTS_C+=	memmem_test
NETBSD_ATF_TESTS_C+=	memset_test
NETBSD_ATF_TESTS_C+=	strcat_test
NETBSD_ATF_TESTS_C+=	strchr_test
NETBSD_ATF_TESTS_C+=	strchrnul_test
NETBSD_ATF_TESTS_C+=	strcmp_test
NETBSD_ATF_TESTS_C+=	strcpy_test
NETBSD_ATF_TESTS_C+=	strerror_test
NETBSD_ATF_TESTS_C+=	strlen_test
NETBSD_ATF_TESTS_C+=	strpbrk_test
NETBSD_ATF_TESTS_C+=	strrchr_test
NETBSD_ATF_TESTS_C+=	swab_test

SRCS.memset2_test=	memset_test.c
SRCS.strcmp2_test=	strcmp_test.c
SRCS.strerror2_test=	strerror_test.c

.include "../Makefile.netbsd-tests"

LIBADD.memchr_test+=	md
LIBADD.memcpy_test+=	md

.include <bsd.test.mk>
