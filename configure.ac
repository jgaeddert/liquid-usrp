## 
## Copyright (c) 2010 Joseph Gaeddert
## Copyright (c) 2010 Virginia Polytechnic Institute & State University
##
## This file is part of liquid.
##
## liquid is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## liquid is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with liquid.  If not, see <http://www.gnu.org/licenses/>.

# 
# liquid usrp library configure
# Process with autoconf to generate configure script
#

AC_INIT([liquid-usrp],[0.1.0],[liquid@vt.edu])
#AC_CONFIG_SRCDIR([libliquidusrp.c])

# permit auxiliary scripts directory (e.g. config.sub, config.guess, install-sh)
AC_CONFIG_AUX_DIR(scripts/)

# Autoheader
AC_CONFIG_HEADER(config.h)
AH_TOP([
#ifndef __LIQUID_CONFIG_H__
#define __LIQUID_CONFIG_H__
])
AH_BOTTOM([
#endif // __LIQUID_CONFIG_H__
])

# Configure options

# Check for necessary programs
AC_PROG_CC
AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_RANLIB

# Check for necessary libraries, library functions
AC_FUNC_ERROR_AT_LINE
AC_FUNC_MALLOC
AC_FUNC_REALLOC
# AC_CHECK_LIB (library, function, [action-if-found], [action-if-not-found], [other-libraries])
AC_CHECK_LIB([c],     [main],             [],[AC_MSG_ERROR(Could not use standard C library)],   [])
AC_CHECK_LIB([m],     [main],             [],[AC_MSG_ERROR(Could not use standard math library)],[])
AC_CHECK_LIB([liquid],[liquid_libversion],[],[AC_MSG_ERROR(Need liquid-dsp library!)],           [])
AC_CHECK_LIB([uhd],   [main],             [],[AC_MSG_ERROR(Need uhd library!)],                  [])

# AC_CHECK_FUNC(function, [action-if-found], [action-if-not-found])
AC_CHECK_FUNC([malloc],  [],[AC_MSG_ERROR(Could not use malloc())])
AC_CHECK_FUNC([realloc], [],[AC_MSG_ERROR(Could not use realloc())],)
AC_CHECK_FUNC([free],    [],[AC_MSG_ERROR(Could not use free())],)
AC_CHECK_FUNC([memset],  [],[AC_MSG_ERROR(Could not use memset())],)
AC_CHECK_FUNC([memmove], [],[AC_MSG_ERROR(Could not use memove())],)

# Check for necessary header files
AC_LANG_PUSH([C++])
AC_CHECK_HEADERS([uhd/usrp/multi_usrp.hpp],
                 [],
                 [AC_MSG_ERROR(missing headers)])
if test -z "$HAVE_stdio.h"
then
    AC_MSG_ERROR([Need stdio.h!])
fi
AC_LANG_POP

# Check for optional header files, libraries
AC_CHECK_HEADERS(fec.h fftw3.h)
AC_CHECK_LIB([fftw3f], [fftwf_plan_dft_1d], [],
             [AC_MSG_WARN(fftw3 library useful but not required)],
             [])
AC_CHECK_LIB([fec], [create_viterbi27], [],
             [AC_MSG_WARN(fec library useful but not required)],
             [])

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_SIZE_T
AC_TYPE_UINT32_T
AC_TYPE_UINT8_T

# Check size of certain variables
AC_CHECK_SIZEOF(int)
AC_CHECK_SIZEOF(unsigned int)

# Check canonical system
AC_CANONICAL_TARGET

case $target_os in
darwin*)
    SH_LIB=libliquidusrp.dylib
    REBIND=""
    ;;
*)
    SH_LIB=libliquidusrp.so
    REBIND=ldconfig
    ;;
esac



#
# autoconf variable substitutions
#
AC_SUBST(LIBS)              # shared libraries (-lc, -lm, etc.)
AC_SUBST(SH_LIB)            # output shared library target
AC_SUBST(REBIND)            # rebinding tool (e.g. ldconfig)
AC_CONFIG_FILES([makefile])
AC_OUTPUT
