#################################################################
#
#   SPDX-License-Identifier: BSD-3-Clause
#   Copyright(c) 2007-2026 Intel Corporation
# 
#   These contents may have been developed with support from one or more
#   Intel-operated generative artificial intelligence solutions.
#
#################################################################

ACLOCAL_AMFLAGS = -Im4
ICP_OS=freebsd

noinst_LTLIBRARIES = libosal.la
libosal_la_SOURCES = \
	quickassist/utilities/osal/src/$(ICP_OS)/user_space/OsalSemaphore.c \
	quickassist/utilities/osal/src/$(ICP_OS)/user_space/OsalThread.c \
	quickassist/utilities/osal/src/$(ICP_OS)/user_space/OsalMutex.c \
	quickassist/utilities/osal/src/$(ICP_OS)/user_space/OsalSpinLock.c \
	quickassist/utilities/osal/src/$(ICP_OS)/user_space/OsalAtomic.c \
	quickassist/utilities/osal/src/$(ICP_OS)/user_space/OsalServices.c \
	quickassist/utilities/osal/src/$(ICP_OS)/user_space/OsalIOMMUMgmt.c \
	quickassist/utilities/osal/src/$(ICP_OS)/user_space/OsalCryptoInterface.c

libosal_la_CFLAGS = -I$(srcdir)/quickassist/utilities/osal/src/$(ICP_OS)/user_space \
		    -I$(srcdir)/quickassist/utilities/osal/src/$(ICP_OS)/user_space/include \
		    -I$(srcdir)/quickassist/utilities/osal/include \
		    -DOSAL_ENSURE_ON -DUSE_OPENSSL \
		    -DICP_WITHOUT_IOMMU \
		    -Wno-deprecated-declarations \
		    $(COMMON_FLAGS)

noinst_LTLIBRARIES += libadf.la
libadf_la_SOURCES = \
	quickassist/lookaside/access_layer/src/qat_direct/src/adf_process_proxy.c \
	quickassist/lookaside/access_layer/src/qat_direct/src/adf_user_ETring_mgr_dp.c \
	quickassist/lookaside/access_layer/src/qat_direct/src/adf_user_init.c \
	quickassist/lookaside/access_layer/src/qat_direct/src/uio_user_ring.c \
	quickassist/lookaside/access_layer/src/qat_direct/src/uio_user_bundles.c \
	quickassist/lookaside/access_layer/src/qat_direct/src/uio_user_cfg.c \
	quickassist/lookaside/access_layer/src/qat_direct/src/uio_user_device.c \
	quickassist/lookaside/access_layer/src/qat_direct/src/uio_user_debug.c \
	quickassist/lookaside/access_layer/src/qat_direct/src/uio_user_dyn.c \
	quickassist/lookaside/access_layer/src/qat_direct/src/uio_user_shared_mutex.c \
	quickassist/lookaside/access_layer/src/qat_direct/src/adf_user_transport_ctrl.c
libadf_la_CFLAGS = -I$(srcdir)/quickassist/utilities/libusdm_drv \
		   -I$(srcdir)/quickassist/utilities/osal/include \
		   -I$(srcdir)/quickassist/utilities/osal/src/$(ICP_OS)/user_space/include \
		   -I$(srcdir)/quickassist/include \
		   -I$(srcdir)/quickassist/include/lac \
		   -I$(srcdir)/quickassist/qat/drivers/crypto/qat/qat_common \
		   -I$(srcdir)/quickassist/lookaside/access_layer/src/common/include \
		   -I$(srcdir)/quickassist/lookaside/access_layer/include \
		   -I$(srcdir)/quickassist/lookaside/access_layer/src/qat_direct/include \
		   -I$(srcdir)/quickassist/lookaside/access_layer/src/qat_direct/src \
		   -I$(srcdir)/quickassist/lookaside/access_layer/src/qat_direct/src/include/accel_mgr \
		   -I$(srcdir)/quickassist/lookaside/access_layer/src/qat_direct/src/include/platform \
		   -I$(srcdir)/quickassist/lookaside/access_layer/src/qat_direct/src/include/transport \
		   -I$(srcdir)/quickassist/lookaside/access_layer/src/qat_direct/src/include/user_proxy \
		   -D USER_SPACE \
		   -D CONFIG_USE_UIO_DYNAMIC \
		   -D CONFIG_USE_UIO_BUNDLE_ADDR \
		   $(COMMON_FLAGS)

lib_LTLIBRARIES = lib@LIBUSDMNAME@.la
lib@LIBUSDMNAME@_la_SOURCES = \
        quickassist/utilities/libusdm_drv/user_space/qae_mem_utils_common.c \
        quickassist/utilities/libusdm_drv/user_space/qae_mem_common.c \
        quickassist/utilities/libusdm_drv/user_space/uio/qae_mem_utils_uio.c \
        quickassist/utilities/libusdm_drv/user_space/uio/qae_mem_hugepage_utils_uio.c
lib@LIBUSDMNAME@_la_CFLAGS = -I$(srcdir)/quickassist/utilities/libusdm_drv \
			     -I$(srcdir)/quickassist/utilities/libusdm_drv/include \
			     -I$(srcdir)/quickassist/utilities/libusdm_drv/user_space \
			     -I$(srcdir)/quickassist/utilities/libusdm_drv/user_space/uio \
			     $(COMMON_FLAGS)
lib@LIBUSDMNAME@_la_LDFLAGS = -version-info $(LIBUSDM_VERSION) \
			      $(COMMON_LDFLAGS) \
			      -export-symbols-regex '^(qae)'

# Creating CRC libs from asm files using nasm with automake-like output
libcrc_SOURCES_DIR = quickassist/lookaside/access_layer/src/common/compression/
crc64_ecma_norm_by8.lo:
	@echo "  CCAS     $(libcrc_SOURCES_DIR)$@"
	@$(LIBTOOL) --mode=compile --quiet \
	nasm -f elf64 -X gnu $(libcrc_SOURCES_DIR)$(@:.lo=.S) -I$(libcrc_SOURCES_DIR) -o $@ -prefer-non-pic

crc32_gzip_refl_by8.lo:
	@echo "  CCAS     $(libcrc_SOURCES_DIR)$@"
	@$(LIBTOOL) --mode=compile --quiet \
	nasm -f elf64 -X gnu $(libcrc_SOURCES_DIR)$(@:.lo=.S) -I$(libcrc_SOURCES_DIR) -o $@ -prefer-non-pic

export CMN_ROOT=$(abs_srcdir)/quickassist/utilities/libusdm_drv
SUBDIRS = quickassist/utilities/libusdm_drv/freebsd/kernel_space
DIST_SUBDIRS = quickassist/utilities/libusdm_drv/freebsd/kernel_space

lib_LTLIBRARIES += lib@LIBQATNAME@.la
lib@LIBQATNAME@_la_SOURCES = \
	quickassist/lookaside/access_layer/src/common/compression/dc_buffers.c \
	quickassist/lookaside/access_layer/src/common/compression/dc_chain.c \
	quickassist/lookaside/access_layer/src/common/compression/dc_datapath.c \
	quickassist/lookaside/access_layer/src/common/compression/dc_dp.c \
	quickassist/lookaside/access_layer/src/common/compression/dc_header_footer.c \
	quickassist/lookaside/access_layer/src/common/compression/dc_session.c \
	quickassist/lookaside/access_layer/src/common/compression/dc_stats.c \
	quickassist/lookaside/access_layer/src/common/compression/dc_err_sim.c \
	quickassist/lookaside/access_layer/src/common/compression/dc_ns_datapath.c \
	quickassist/lookaside/access_layer/src/common/compression/dc_ns_header_footer.c \
	quickassist/lookaside/access_layer/src/common/compression/dc_crc32.c \
	quickassist/lookaside/access_layer/src/common/compression/dc_crc64.c \
	quickassist/lookaside/access_layer/src/common/compression/icp_sal_dc_err_sim.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/diffie_hellman/lac_dh_control_path.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/diffie_hellman/lac_dh_data_path.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/diffie_hellman/lac_dh_interface_check.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/diffie_hellman/lac_dh_stats.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/dsa/lac_dsa.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/dsa/lac_dsa_interface_check.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/ecc/lac_ec.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/ecc/lac_ec_common.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/ecc/lac_ec_montedwds.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/ecc/lac_ec_nist_curves.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/ecc/lac_ecdh.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/ecc/lac_ecdsa.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/large_number/lac_ln.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/large_number/lac_ln_interface_check.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/pke_common/lac_pke_mmp.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/pke_common/lac_pke_qat_comms.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/pke_common/lac_pke_utils.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/prime/lac_prime.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/prime/lac_prime_interface_check.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/rsa/lac_rsa.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/rsa/lac_rsa_control_path.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/rsa/lac_rsa_decrypt.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/rsa/lac_rsa_encrypt.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/rsa/lac_rsa_interface_check.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/rsa/lac_rsa_keygen.c \
	quickassist/lookaside/access_layer/src/common/crypto/asym/rsa/lac_rsa_stats.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/drbg/lac_sym_drbg_api.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/key/lac_sym_key.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/lac_sym_alg_chain.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/lac_sym_api.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/lac_sym_auth_enc.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/lac_sym_cb.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/lac_sym_cipher.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/lac_sym_compile_check.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/lac_sym_dp.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/lac_sym_hash.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/lac_sym_partial.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/lac_sym_queue.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/lac_sym_stats.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/nrbg/lac_sym_nrbg_api.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/qat/lac_sym_qat.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/qat/lac_sym_qat_cipher.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/qat/lac_sym_qat_constants_table.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/qat/lac_sym_qat_hash.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/qat/lac_sym_qat_hash_defs_lookup.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/qat/lac_sym_qat_key.c \
	quickassist/lookaside/access_layer/src/common/crypto/sym/lac_sym_hash_sw_precomputes.c \
	quickassist/lookaside/access_layer/src/common/ctrl/sal_compression.c \
	quickassist/lookaside/access_layer/src/common/ctrl/sal_create_services.c \
	quickassist/lookaside/access_layer/src/common/ctrl/sal_ctrl_services.c \
	quickassist/lookaside/access_layer/src/common/ctrl/sal_get_instances.c \
	quickassist/lookaside/access_layer/src/common/ctrl/sal_list.c \
	quickassist/lookaside/access_layer/src/common/ctrl/sal_crypto.c \
	quickassist/lookaside/access_layer/src/common/qat_comms/sal_qat_cmn_msg.c \
	quickassist/lookaside/access_layer/src/common/utils/lac_buffer_desc.c \
	quickassist/lookaside/access_layer/src/common/utils/lac_log_message.c \
	quickassist/lookaside/access_layer/src/common/utils/lac_mem.c \
	quickassist/lookaside/access_layer/src/common/utils/lac_mem_pools.c \
	quickassist/lookaside/access_layer/src/common/utils/lac_sync.c \
	quickassist/lookaside/access_layer/src/common/utils/sal_service_state.c \
	quickassist/lookaside/access_layer/src/common/utils/sal_statistics.c \
	quickassist/lookaside/access_layer/src/common/utils/sal_string_parse.c \
	quickassist/lookaside/access_layer/src/common/utils/sal_user_process.c \
	quickassist/lookaside/access_layer/src/common/utils/sal_versions.c \
	quickassist/lookaside/access_layer/src/device/dev_info.c \
	quickassist/lookaside/access_layer/src/user/sal_user.c \
	quickassist/lookaside/access_layer/src/user/sal_user_dyn_instance.c \
	quickassist/lookaside/access_layer/src/user/sal_user_du.c \
	quickassist/lookaside/access_layer/src/user/sal_user_tl.c

lib@LIBQATNAME@_la_CFLAGS = -I$(srcdir)/quickassist/utilities/libusdm_drv \
			    -I$(srcdir)/quickassist/utilities/osal/include \
			    -I$(srcdir)/quickassist/utilities/osal/src/$(ICP_OS)/user_space/include \
			    -I$(srcdir)/quickassist/include \
			    -I$(srcdir)/quickassist/include/lac \
			    -I$(srcdir)/quickassist/include/dc \
			    -I$(srcdir)/quickassist/qat/drivers/crypto/qat/qat_common \
			    -I$(srcdir)/quickassist/lookaside/access_layer/src/common/compression/include \
			    -I$(srcdir)/quickassist/lookaside/access_layer/src/common/crypto/sym/include \
			    -I$(srcdir)/quickassist/lookaside/access_layer/src/common/crypto/asym/include \
			    -I$(srcdir)/quickassist/lookaside/firmware/include \
			    -I$(srcdir)/quickassist/lookaside/access_layer/src/common/include \
			    -I$(srcdir)/quickassist/lookaside/access_layer/include \
			    -I$(srcdir)/quickassist/lookaside/access_layer/src/qat_direct/include \
			    -I$(srcdir)/quickassist/lookaside/access_layer/src/qat_direct/common/include \
			    -I$(srcdir)/quickassist/lookaside/access_layer/src/qat_direct/vfio \
			    -D USER_SPACE \
			    -D LAC_BYTE_ORDER=__LITTLE_ENDIAN \
			    $(COMMON_FLAGS)
lib@LIBQATNAME@_la_LIBADD = libosal.la libadf.la lib@LIBUSDMNAME@.la -lcrypto
lib@LIBQATNAME@_la_LIBADD += crc32_gzip_refl_by8.lo crc64_ecma_norm_by8.lo
lib@LIBQATNAME@_la_LDFLAGS = -version-info $(LIBQAT_VERSION) \
			     $(COMMON_LDFLAGS) \
			     -export-symbols-regex '^(cpa|icp_sal)'

pkgincludedir = $(includedir)/qat
pkginclude_HEADERS = \
	quickassist/include/cpa.h \
	quickassist/include/cpa_dev.h \
	quickassist/include/cpa_types.h \
	quickassist/include/lac/cpa_cy_common.h \
	quickassist/include/lac/cpa_cy_dh.h \
	quickassist/include/lac/cpa_cy_drbg.h \
	quickassist/include/lac/cpa_cy_dsa.h \
	quickassist/include/lac/cpa_cy_ecdh.h \
	quickassist/include/lac/cpa_cy_ecdsa.h \
	quickassist/include/lac/cpa_cy_ecsm2.h \
	quickassist/include/lac/cpa_cy_ec.h \
	quickassist/include/lac/cpa_cy_im.h \
	quickassist/include/lac/cpa_cy_key.h \
	quickassist/include/lac/cpa_cy_kpt.h \
	quickassist/include/lac/cpa_cy_ln.h \
	quickassist/include/lac/cpa_cy_nrbg.h \
	quickassist/include/lac/cpa_cy_prime.h \
	quickassist/include/lac/cpa_cy_rsa.h \
	quickassist/include/lac/cpa_cy_sym_dp.h \
	quickassist/include/lac/cpa_cy_sym.h \
	quickassist/include/dc/cpa_dc.h \
	quickassist/include/dc/cpa_dc_dp.h \
	quickassist/include/dc/cpa_dc_chain.h \
	quickassist/lookaside/access_layer/include/icp_buffer_desc.h \
	quickassist/lookaside/access_layer/include/icp_sal_congestion_mgmt.h \
	quickassist/lookaside/access_layer/include/icp_sal_poll.h \
	quickassist/lookaside/access_layer/include/icp_sal_user.h \
	quickassist/lookaside/access_layer/include/icp_sal.h \
	quickassist/lookaside/access_layer/include/icp_sal_dev_versions.h \
	quickassist/lookaside/access_layer/include/icp_sal_versions.h \
	quickassist/lookaside/access_layer/include/icp_sal_dc_error_simulation.h \
	quickassist/lookaside/access_layer/include/icp_sal_hb_fail_simulation.h \
	quickassist/lookaside/access_layer/include/icp_sal_iommu.h \
	quickassist/lookaside/access_layer/include/icp_sal_ring_exception_simulation.h \
	quickassist/utilities/libusdm_drv/qae_mem.h

EXTRA_DIST = LICENSE \
	autogen.sh \
	quickassist/lookaside/firmware/include \
	quickassist/include \
	quickassist/lookaside/access_layer/src/common \
	quickassist/lookaside/access_layer/include \
	quickassist/lookaside/access_layer/src/qat_direct \
	quickassist/utilities/osal \
	quickassist/utilities/libusdm_drv/include \
	quickassist/utilities/libusdm_drv/user_space/qae_mem_hugepage_utils.h \
	quickassist/utilities/libusdm_drv/user_space/qae_mem_user_utils.h \
	quickassist/utilities/libusdm_drv/user_space/qae_page_table.h \
	quickassist/lookaside/access_layer/src/sample_code

######################
#  Build parameters  #
######################

COMMON_LDFLAGS = -pie -z relro -z now -z noexecstack
AM_LDFLAGS = $(COMMON_LDFLAGS)

COMMON_FLAGS = -Wformat \
	       -Wformat-security \
	       -Werror=format-security \
	       -D_FORTIFY_SOURCE=2 \
	       -fstack-protector-strong \
	       -fPIE \
	       -fPIC \
	       -fno-strict-overflow \
	       -fno-delete-null-pointer-checks \
	       -fwrapv \
	       -D MAX_MR_ROUND=$(MAX_MR) \
	       -D _FILE_OFFSET_BITS=64 \
	       -DADF_PLATFORM_ACCELDEV \
	       $(NO_UNUSED_CMDLINE_ARG_CFLAGS) \
	       $(MCX16_CFLAGS)

if ICP_DEBUG_AC
ICP_DEBUG = 1
COMMON_FLAGS += -DICP_DEBUG
endif

if ICP_PARAM_CHECK_AC
ICP_PARAM_CHECK = y
COMMON_FLAGS += -DICP_PARAM_CHECK
endif

if DISABLE_STATS_AC
DISABLE_STATS = 1
COMMON_FLAGS += -DDISABLE_STATS
endif

if ICP_LOG_SYSLOG_AC
ICP_LOG_SYSLOG = 1
COMMON_FLAGS += -DICP_LOG_SYSLOG
endif

if ICP_TRACE_AC
ICP_TRACE = 1
COMMON_FLAGS += -DICP_TRACE
endif

if ICP_DC_ERROR_SIMULATION_AC
ICP_DC_ERROR_SIMULATION = 1
COMMON_FLAGS += -DICP_DC_ERROR_SIMULATION
endif

if QAT_LEGACY_ALGORITHMS_AC
QAT_LEGACY_ALGORITHMS = y
COMMON_FLAGS += -DQAT_LEGACY_ALGORITHMS
else
QAT_LEGACY_ALGORITHMS = n
endif

include Samples.am
