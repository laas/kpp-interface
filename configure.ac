# Don't expend macros beginning by JRL_.
m4_pattern_forbid([^JRL_])
AC_PREREQ([2.59])

## --------------------- ##
## Package configuration ##
## --------------------- ##
AC_INIT([kppInterface], [1.6.1.99], [hpp@laas.fr], [kppInterface])

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
JRL_CXX_WARNINGS([[-Wall],
                   [-Wcast-align],
                   [-Wcast-qual],
                   [-Wformat],
                   [-Wmissing-prototypes],
                   [-Wstrict-prototypes],
                   [-Wwrite-strings],
                   [-Wbad-function-cast],
                   [-Wmissing-declarations],
                   [-Wnested-externs]])

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

PKG_CHECK_MODULES(KINEOPATHPLANNER, KineoPathPlanner)
KINEOPATHPLANNER_PREFIX=`$PKG_CONFIG KineoPathPlanner --variable=prefix`
AC_SUBST(KINEOPATHPLANNER_PREFIX)
AC_SUBST(KINEOPATHPLANNER_CFLAGS)
AC_SUBST(KINEOPATHPLANNER_LIBS)

AC_SUBST(KINEO_ADDON_BUILDER,
	[${KINEOPATHPLANNER_PREFIX}/bin/KineoAddonBuilder])
AC_SUBST(KINEO_MODULE_DIR, ["x${KINEOPATHPLANNER_PREFIX}/modules"])

AC_SUBST(KINEO_PREFIX, [${KINEOPATHPLANNER_PREFIX}])
AC_SUBST(KINEO_LIBDIR, [${KINEOPATHPLANNER_PREFIX}/lib])

# HPP packages
define(hppcore_reqd, [hppCore >= 1.11])
AC_SUBST(HPPCORE_REQD, "hppcore_reqd")
PKG_CHECK_MODULES(HPPCORE, hppcore_reqd)
HPPCORE_DOCDIR=`$PKG_CONFIG hppCore --variable=docdir`
AC_SUBST(HPPCORE_DOCDIR)

define(hppcobaserver_reqd, [hppCorbaServer >= 1.8.3])
AC_SUBST(HPPCOBASERVER_REQD, "hppcobaserver_reqd")
PKG_CHECK_MODULES(HPPCORBASERVER, hppcobaserver_reqd)
HPPCORBASERVER_DOCDIR=`$PKG_CONFIG hppCorbaServer --variable=docdir`
AC_SUBST(HPPCORBASERVER_DOCDIR)

define(kwsplus_reqd, [kwsPlus >= 1.8])
AC_SUBST(KWSPLUS_REQD, "kwsplus_reqd")
PKG_CHECK_MODULES(KWSPLUS, kwsplus_reqd)
KWSPLUS_DOCDIR=`$PKG_CONFIG kwsPlus --variable=docdir`
AC_SUBST(KWSPLUS_DOCDIR)
KWSPLUS_PREFIX=`$PKG_CONFIG kwsPlus --variable=prefix`
AC_SUBST(KWSPLUS_PREFIX)


## ------------ ##
## Pkg-config.  ##
## ------------ ##

AC_SUBST([PKGCONFIG_DESCRIPTION], ['Kpp interface'])
AC_SUBST([PKGCONFIG_URL], [])
AC_SUBST([PKGCONFIG_REQUIRES],
	['kwsPlus >= 1.8, hppCorbaServer >= 1.8.3, KineoPathPlanner'])
AC_SUBST([PKGCONFIG_CONFLICTS], [])

AC_SUBST([PKGCONFIG_LIBS],
	["-L${libdir}/modules/${LICENSE_HOST} -lkppInterface"])

AC_SUBST([PKGCONFIG_CFLAGS], [])

## -------------- ##
## Write output.  ##
## -------------- ##

# Create config.h
AC_CONFIG_HEADERS([config.h])

# Creating Makefiles.
AC_OUTPUT([
	Makefile
	include/Makefile
	src/Makefile
	doc/Makefile
])
