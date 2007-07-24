
dnl 
dnl configure.ac file for package kppInterface 
dnl

AC_INIT([kppInterface], [1.0], [support@company.com])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE

AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_LIBTOOL

PKG_CHECK_MODULES(KINEOPATHPLANNER, KineoPathPlanner)
AC_SUBST(KINEOPATHPLANNER_CFLAGS)
AC_SUBST(KINEOPATHPLANNER_LIBS)

PKG_CHECK_MODULES(HPPCORBASERVER, hppCorbaServer)
HPPCORBASERVER_PREFIX=`$PKG_CONFIG hppCorbaServer --variable=prefix`
AC_SUBST(HPPCORBASERVER_CFLAGS)
AC_SUBST(HPPCORBASERVER_LIBS)
AC_SUBST(HPPCORBASERVER_PREFIX)

AC_OUTPUT(
	kppInterface.pc
	Makefile
	include/Makefile
	src/Makefile
	doc/Makefile
	doc/package.dox
	)
