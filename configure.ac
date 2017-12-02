#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])

AC_INIT([wiringPi], [2.0.0], [http://wiringpi.com/])

# versions for the libraries
AC_SUBST(libwiringPi_la_version, 2:0:0)
AC_SUBST(libwiringPiDev_la_version, 2:0:0)

# install locations for headers
AC_SUBST(includedir,"${includedir}/wiringPi")


AC_ARG_ENABLE([devlib],
    [AS_HELP_STRING(
        [--disable-devlib],
        [disable building of the device library]
    )],
    [build_devlib=${enableval}],
    [build_devlib=yes]
)
AM_CONDITIONAL([BUILD_DEVLIB], [test x$build_devlib = xyes])

AC_ARG_ENABLE([gpio-tool],
    [AS_HELP_STRING(
        [--disable-gpio-tool],
        [disable building of the gpio tool]
    )],
    [build_gpio=${enableval}],
    [build_gpio=yes]
)

## check for devlib if gpio should be build
if test x$build_gpio = xyes ; then
    if test x$build_devlib != xyes ; then
        AC_MSG_WARN([devlib not enabled, disabling gpio])
        build_gpio=no
    fi
fi

AM_CONDITIONAL([BUILD_GPIO], [test x$build_gpio = xyes])

# allow the user to set suid
AC_ARG_ENABLE([suid],
    [AS_HELP_STRING(
        [--enable-suid],
        [make the gpio binary suid [default=no]]
    )],
    [case "${enableval}" in
        yes) enable_suid=true;;
        no)  enable_suid=false;;
        *) AC_MSG_ERROR([bad value ${enableval} for --enable-suid]) ;;
    esac],[enable_suid=false])
AM_CONDITIONAL([WITH_SUID], [test x$enable_suid = xtrue])

AC_CONFIG_MACRO_DIR([m4])

LT_INIT([disable-static])
AM_INIT_AUTOMAKE([1.9])


AC_CONFIG_SRCDIR([devLib/ds1302.c])
AC_CONFIG_HEADERS([config.h])

# Checks for programs.
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_MAKE_SET

# Checks for libraries.
AC_CHECK_LIB([m], [round])
AC_CHECK_LIB([pthread], [pthread_create])

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h stdint.h stdlib.h string.h sys/ioctl.h sys/time.h termios.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_INLINE
AC_TYPE_INT32_T
AC_TYPE_INT8_T
AC_TYPE_PID_T
AC_TYPE_UID_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_FUNC_CHOWN
AC_FUNC_FORK
AC_FUNC_MALLOC
AC_FUNC_MMAP
AC_CHECK_FUNCS([floor gettimeofday memset rint strcasecmp strerror strtol strtoul])

AC_CONFIG_FILES([Makefile
                 devLib/Makefile
                 examples/Gertboard/Makefile
                 examples/Makefile
                 examples/PiFace/Makefile
                 examples/PiGlow/Makefile
                 examples/q2w/Makefile
                 gpio/Makefile
                 wiringPi/Makefile
                 wiringPi.pc
                 wiringPiDev.pc])
AC_OUTPUT
