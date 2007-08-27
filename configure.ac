
dnl 
dnl configure.ac file for package kppInterface 
dnl

AC_INIT([kppInterface], [1.0], [gepetto@laas.fr])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE([foreign no-define])

AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_LIBTOOL

PKG_CHECK_MODULES(KINEOPATHPLANNER, KineoPathPlanner)

define(hppcobaserver_reqd, [hppCorbaServer >= 1.0])
AC_SUBST(HPPCOBASERVER_REQD, "hppcobaserver_reqd")
PKG_CHECK_MODULES(HPPCORBASERVER, hppcobaserver_reqd)
HPPCORBASERVER_PREFIX=`$PKG_CONFIG hppCorbaServer --variable=prefix`
AC_SUBST(HPPCORBASERVER_PREFIX)

AC_OUTPUT(
	kppInterface.pc
	Makefile
	include/Makefile
	src/Makefile
	doc/Makefile
	doc/package.dox
	)
