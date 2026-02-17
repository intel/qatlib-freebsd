#########################################################################
#
# @par
#   SPDX-License-Identifier: BSD-3-Clause
#   Copyright(c) 2007-2026 Intel Corporation
# 
#   These contents may have been developed with support from one or more
#   Intel-operated generative artificial intelligence solutions.
############################################################################

KMOD=	usdm_drv
SRCS=	qae_mem_drv_utils.c qae_mem_drv.c qae_mem_utils.c

NO_OBJ= 1

CFLAGS+= -fms-extensions

CFLAGS+= -D__KERNEL__
CFLAGS+= -I${CMN_ROOT} -I${CMN_ROOT}/include
CFLAGS+= -DKERNEL_SPACE

CFLAGS+= -Wformat -Wformat-security

CFLAGS+= -O2 -D_FORTIFY_SOURCE=2
CFLAGS+= -fno-delete-null-pointer-checks -fwrapv
CFLAGS+= -Wno-missing-field-initializers -fno-strict-aliasing -fstack-protector

distclean: clean cleandepend

install: beforebuild reload
	@if [ `grep "${KMOD}_load" /boot/loader.conf | wc -l` != "0" ]; then \
		sed -i '' /${KMOD}_load/d /boot/loader.conf;                 \
	fi
	@echo "${KMOD}_load=\"YES\"" >> /boot/loader.conf
	@cp ${PROG} ${KMODDIR}/${PROG}

uninstall: unload
	@rm -rf $(KMODDIR)/${PROG}
	@if [ `grep "${KMOD}_load" /boot/loader.conf | wc -l` != "0" ]; then \
		sed -i '' /${KMOD}_load/d /boot/loader.conf;                 \
	fi

.include <bsd.kmod.mk>
