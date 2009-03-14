
dnl 
dnl configure.ac file for package kppInterface 
dnl

AC_INIT([kppInterface], [1.5], [hpp@laas.fr])
AC_PREREQ(2.59)

if test "$CFLAGS$CXXFLAGS" = ""; then
  setdefaultflags=yes
else
  setdefaultflags=no
fi

AM_INIT_AUTOMAKE([foreign no-define])

AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_LIBTOOL

if test "$setdefaultflags" = "yes"; then
  if test "$GCC" = "yes"; then
    CFLAGS="$CFLAGS -Wall"
    AC_MSG_NOTICE([adding default "-Wall" c flags])
  fi
  if test "$GXX" = "yes"; then
    CXXFLAGS="$CXXFLAGS -Wall"
    AC_MSG_NOTICE([adding default "-Wall" c++ flags])
  fi
fi

LICENCE_HOST=`uname -n`
AC_SUBST(LICENCE_HOST)

PKG_CHECK_MODULES(KINEOPATHPLANNER, KineoPathPlanner)
KINEOPATHPLANNER_PREFIX=`$PKG_CONFIG KineoPathPlanner --variable=prefix`
AC_SUBST(KINEOPATHPLANNER_PREFIX)
AC_SUBST(KINEOPATHPLANNER_CFLAGS)
AC_SUBST(KINEOPATHPLANNER_LIBS)

AC_SUBST(KINEO_ADDON_BUILD,[${KINEOPATHPLANNER_PREFIX}/bin/KineoAddonBuilder])
AC_SUBST(KINEO_MODULE_DIR,["x${KINEOPATHPLANNER_PREFIX}/modules"])
AC_SUBST(KINEO_LIBDIR,[${KINEOPATHPLANNER_PREFIX}/lib])

define(hppcore_reqd, [hppCore >= 1.7])
AC_SUBST(HPPCORE_REQD, "hppcore_reqd")
PKG_CHECK_MODULES(HPPCORE, hppcore_reqd)
HPPCORE_DOCDIR=`$PKG_CONFIG hppCore --variable=docdir`
AC_SUBST(HPPCORE_DOCDIR)

define(hppcobaserver_reqd, [hppCorbaServer >= 1.7])
AC_SUBST(HPPCOBASERVER_REQD, "hppcobaserver_reqd")
PKG_CHECK_MODULES(HPPCORBASERVER, hppcobaserver_reqd)
HPPCORBASERVER_DOCDIR=`$PKG_CONFIG hppCorbaServer --variable=docdir`
AC_SUBST(HPPCORBASERVER_DOCDIR)

define(kwsplus_reqd, [kwsPlus >= 1.5])
AC_SUBST(KWSPLUS_REQD, "kwsplus_reqd")
PKG_CHECK_MODULES(KWSPLUS, kwsplus_reqd)
KWSPLUS_DOCDIR=`$PKG_CONFIG kwsPlus --variable=docdir`
AC_SUBST(KWSPLUS_DOCDIR)
KWSPLUS_PREFIX=`$PKG_CONFIG kwsPlus --variable=prefix`
AC_SUBST(KWSPLUS_PREFIX)

AC_OUTPUT(
	kppInterface.pc
	Makefile
	include/Makefile
	src/Makefile
	doc/Makefile
	doc/package.dox
	)
