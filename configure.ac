#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([qatlib], [23.09.0], [qat@intel.com])
AC_CONFIG_AUX_DIR([.])
AC_CONFIG_MACRO_DIRS([m4])
AC_CONFIG_HEADERS([config.h])

AM_INIT_AUTOMAKE([-Wall -Werror -Wno-portability foreign subdir-objects tar-pax])

AM_SILENT_RULES([yes])
AC_USE_SYSTEM_EXTENSIONS

AC_SUBST([LIBQAT_VERSION], [1:0:0])
AC_SUBST([LIBUSDM_VERSION], [1:0:1])

# Checks for programs.
AC_PROG_AWK
AC_PROG_CC
AC_PROG_LN_S
AC_PROG_MAKE_SET
AM_PROG_AR
AC_PATH_TOOL(PKGCONFIG, pkgconf)

LT_PREREQ([2.4])
LT_INIT

# Checks for libraries.
AC_CHECK_LIB([pthread], [pthread_create])
AC_CHECK_LIB([crypto], [AES_decrypt])

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h inttypes.h limits.h stddef.h stdint.h stdlib.h string.h sys/ioctl.h sys/param.h sys/socket.h sys/time.h syslog.h unistd.h utmpx.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL
AC_TYPE_UID_T
AC_C_INLINE
AC_TYPE_INT16_T
AC_TYPE_INT32_T
AC_TYPE_INT64_T
AC_TYPE_INT8_T
AC_TYPE_PID_T
AC_C_RESTRICT
AC_TYPE_SIZE_T
AC_TYPE_SSIZE_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_FUNC_FORK
AC_FUNC_MALLOC
AC_FUNC_MMAP
AC_FUNC_STRNLEN
AC_CHECK_FUNCS([alarm clock_gettime getpagesize gettimeofday memmove memset munmap select socket strerror strstr strtoul strtoull])

# Check for openssl header
AC_CHECK_HEADERS([openssl/md5.h],
    [],
    [AC_MSG_ERROR([openssl/md5.h not found])]
)

# Check if compiler supports mcx16
saved_cflags="$CFLAGS"
CFLAGS=-mcx16
AC_MSG_CHECKING([whether $CC supports -mcx16])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([])],
    [if $GREP "warning" conftest.err >/dev/null; then AC_MSG_RESULT([no]); else MCX16_CFLAGS="-mcx16"; AC_MSG_RESULT([yes]); fi],
    [AC_MSG_RESULT([no])])
CFLAGS="$saved_cflags"
AC_SUBST(MCX16_CFLAGS)

# Check if compiler supports -Wno-unused-command-line-argument
saved_cflags="$CFLAGS"
CFLAGS=-Wno-unused-command-line-argument
AC_MSG_CHECKING([whether $CC supports -Wno-unused-command-line-argument])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([])],
    [if $GREP "warning" conftest.err >/dev/null; then AC_MSG_RESULT([no]); else NO_UNUSED_CMDLINE_ARG_CFLAGS="-Wno-unused-command-line-argument"; AC_MSG_RESULT([yes]); fi],
    [AC_MSG_RESULT([no])])
CFLAGS="$saved_cflags"
AC_SUBST(NO_UNUSED_CMDLINE_ARG_CFLAGS)

# Check for pkgconfig
AC_MSG_CHECKING([for pkg-config])
AS_IF([test "x${PKGCONFIG}" = "x"],
    [
     AC_MSG_RESULT(no)
     have_pkgconfig="no"
    ],
    [
     AC_MSG_RESULT(yes)
     have_pkgconfig="yes"
    ]
)

AC_CHECK_PROG(YASM_BINARY, yasm, yes , no)

# SAMPLES
AC_ARG_ENABLE(samples,
        AS_HELP_STRING([--enable-samples], [Create sample programs]),
	[enable_samples=${enableval}], [enable_samples="yes"])
AC_MSG_CHECKING(whether to build samples)
AC_MSG_RESULT(${enable_samples})
AM_CONDITIONAL(SAMPLES, test "${enable_samples}" != "no")

# MAX_MR
AC_ARG_VAR(MAX_MR, [Number of Miller Rabin rounds for prime operations. Setting this to a smaller value reduces the memory usage required by the driver (default: 50).])
if ! test "$MAX_MR"; then
    MAX_MR=50
fi

# ICP_DEBUG
AC_ARG_ENABLE(icp-debug,
    AS_HELP_STRING([--enable-icp-debug], [Enables debugging.]),
    [icp_debug=true], [icp_debug=false]
)
AM_CONDITIONAL([ICP_DEBUG_AC], [test x$icp_debug = xtrue])


#ICP_PARAM_CHECK
AC_ARG_ENABLE([param-check],
    [AS_HELP_STRING([--disable-param-check], [Disables parameters checking in the top-level APIs. (Use for performance optimization.)])],
    [case "${enableval}" in
        yes) disable_param_check=false;;
         no)  disable_param_check=true;;
          *)   disable_param_check=false;;
    esac],
    [disable_param_check=false]
)
AM_CONDITIONAL([ICP_PARAM_CHECK_AC], [test x$disable_param_check = xfalse])


# DISABLE_STATS
AC_ARG_ENABLE(stats,
    AS_HELP_STRING([--disable-stats], [Disables statistic collection (Use for performance optimization).]),
    [disable_stats=true], [disable_stats=false]
)
AM_CONDITIONAL([DISABLE_STATS_AC], [test x$disable_stats = xtrue])


# ICP_LOG_SYSLOG
AC_ARG_ENABLE(icp-log-syslog,
    AS_HELP_STRING([--enable-icp-log-syslog], [Enables debugging messages to be outputted to the system log instead of standard output.]),
    [icp_log_syslog=true], [icp_log_syslog=false]
)
AM_CONDITIONAL([ICP_LOG_SYSLOG_AC], [test x$icp_log_syslog = xtrue])


# ICP_TRACE
AC_ARG_ENABLE(icp-trace,
    AS_HELP_STRING([--enable-icp-trace], [Enables tracing for the Cryptography API.]),
    [icp_trace=true], [icp_trace=false]
)
AM_CONDITIONAL([ICP_TRACE_AC], [test x$icp_trace = xtrue])

# ICP_DC_ERROR_SIMULATION
AC_ARG_ENABLE(dc-error-simulation,
    AS_HELP_STRING([--enable-dc-error-simulation], [Enables Data Compression Error Simulation.]),
    [dc_error_simulation=true], [dc_error_simulation=false]
)
AM_CONDITIONAL([ICP_DC_ERROR_SIMULATION_AC], [test x$dc_error_simulation = xtrue])

AC_ARG_ENABLE(legacy-lib-names,
    AS_HELP_STRING([--enable-legacy-lib-names], [Enables legacy names for libraries.]),
    [
        AC_SUBST([LIBQATNAME], "qat_s")
        AC_SUBST([LIBUSDMNAME], "usdm_drv_s")
    ],
    [
        AC_SUBST([LIBQATNAME], "qat")
        AC_SUBST([LIBUSDMNAME], "usdm")
    ])

AC_ARG_ENABLE(legacy-algorithms,
	AS_HELP_STRING([--enable-legacy-algorithms], [Enable legacy crypto algorithms.]),
	[enable_legacy_algorithms="yes"], [enable_legacy_algorithms="no"])

AM_CONDITIONAL(QAT_LEGACY_ALGORITHMS_AC, test "$enable_legacy_algorithms" = "yes" )

# Config files.
AC_CONFIG_FILES([Makefile])

# Substitutions in spec file
AC_SUBST([PACKAGE])
AC_SUBST([VERSION])

AC_OUTPUT
