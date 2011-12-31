# Don't expend macros beginning by JRL_.
m4_pattern_forbid([^JRL_])
AC_PREREQ(2.65)

## --------------------- ##
## Package configuration ##
## --------------------- ##

AC_INIT([kppInterface], [1.9.1], [hpp@laas.fr], [kppInterface])

# Auxiliary files.
AC_CONFIG_AUX_DIR([build-aux])

# Automake.
AM_INIT_AUTOMAKE([1.9.6 dist-bzip2 -Wall nostdinc])

## ------------- ##
## C++ compiler  ##
## ------------- ##

# Search for a C++ compiler.
AC_LANG([C++])
AC_PROG_CXX

# Speed-up compilation.
if test "$GXX" = yes; then
  CXXFLAGS="$CXXFLAGS -pipe"
fi

# Macros are stored in build-aux.
AC_CONFIG_MACRO_DIR([build-aux])

## ------------------- ##
## Additionnal tools.  ##
## ------------------- ##

# Search for misc. common tools.
AC_PROG_MKDIR_P
AC_PROG_SED
AC_PROG_INSTALL

# Customize warnings.
dnl JRL_CXX_WARNINGS([[-Wall],
dnl                    [-Wcast-align],
dnl                    [-Wcast-qual],
dnl                    [-Wformat],
dnl                    [-Wmissing-prototypes],
dnl                    [-Wstrict-prototypes],
dnl                    [-Wwrite-strings],
dnl                    [-Wbad-function-cast],
dnl                    [-Wmissing-declarations],
dnl                    [-Wnested-externs]])

# Search for Libtool.
LT_INIT([dlopen])

# Search for Doxygen.
JRL_PROG_DOXYGEN

## ------------------------- ##
## Search for dependencies.  ##
## ------------------------- ##

# Kineo
LICENSE_HOST=`uname -n`
AC_SUBST(LICENSE_HOST)

PKG_CHECK_MODULES(KITE, Kite >= 2.06)
KITE_PREFIX=`$PKG_CONFIG Kite --variable=prefix`
AC_SUBST(KITE_PREFIX)
AC_SUBST(KITE_CFLAGS)
AC_SUBST(KITE_LIBS)

AC_SUBST(KINEO_ADDON_BUILDER,[${KITE_PREFIX}/bin/KineoAddonBuilder])
AC_SUBST(KINEO_MODULE_DIR, ["x${KITE_PREFIX}/modules"])

AC_SUBST(KINEO_PREFIX, [${KITE_PREFIX}])
AC_SUBST(KINEO_LIBDIR, [${KITE_PREFIX}/lib])

# HPP packages
define(hpputil_reqd, [hpp-util >= 0.4])
AC_SUBST(HPPUTIL_REQD, "hpputil_reqd")
PKG_CHECK_MODULES(HPPUTIL, hpputil_reqd)
HPPUTIL_DOCDIR=`$PKG_CONFIG hpp-util --variable=docdir`
AC_SUBST(HPPUTIL_DOCDIR)

define(hppcore_reqd, [hpp-core >= 2.0])
AC_SUBST(HPPCORE_REQD, "hppcore_reqd")
PKG_CHECK_MODULES(HPPCORE, hppcore_reqd)
HPPCORE_DOCDIR=`$PKG_CONFIG hpp-core --variable=docdir`
AC_SUBST(HPPCORE_DOCDIR)

define(hppcobaserver_reqd, [hpp-corbaserver >= 1.13])
AC_SUBST(HPPCOBASERVER_REQD, "hppcobaserver_reqd")
PKG_CHECK_MODULES(HPPCORBASERVER, hppcobaserver_reqd)
HPPCORBASERVER_DOCDIR=`$PKG_CONFIG hpp-corbaserver --variable=docdir`
AC_SUBST(HPPCORBASERVER_DOCDIR)

define(kwsplus_reqd, [kwsPlus >= 2.1])
AC_SUBST([KWSPLUS_REQD], ["kwsplus_reqd"])
PKG_CHECK_MODULES([KWSPLUS], [kwsplus_reqd])
KWSPLUS_DOCDIR=`$PKG_CONFIG kwsPlus --variable=docdir`
AC_SUBST([KWSPLUS_DOCDIR])
KWSPLUS_PREFIX=`$PKG_CONFIG kwsPlus --variable=prefix`
AC_SUBST([KWSPLUS_PREFIX])


## ------------ ##
## Pkg-config.  ##
## ------------ ##

AC_SUBST([PKGCONFIG_DESCRIPTION], ['Kpp interface'])
AC_SUBST([PKGCONFIG_URL], [])
AC_SUBST([PKGCONFIG_REQUIRES],
	['kwsPlus >= 2.1, hpp-core >= 2.0, hpp-util >= 0.4, hpp-corbaserver >= 1.13, Kite >= 2.06'])
AC_SUBST([PKGCONFIG_CONFLICTS], [])

AC_SUBST([PKGCONFIG_LIBS],
	["-L${libdir}/modules/${LICENSE_HOST} -lkppInterface"])

AC_SUBST([PKGCONFIG_CFLAGS], [])
AC_SUBST([PKGCONFIG_EXTRA], [])

## -------------- ##
## Write output.  ##
## -------------- ##

# Create config.h
AC_CONFIG_HEADERS([config.h])

# Creating Makefiles.
AC_CONFIG_FILES([
	Makefile
	include/Makefile
	src/Makefile
	doc/Makefile
])
AC_OUTPUT
