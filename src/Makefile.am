include $(top_srcdir)/build-aux/init.mk
include $(top_srcdir)/build-aux/kineo.mk

# Add available warnings flags.
AM_CXXFLAGS += $(WARNING_CXXFLAGS)

AM_CPPFLAGS += 				\
	@HPPCORBASERVER_CFLAGS@ 	\
	@KINEOPATHPLANNER_CFLAGS@ 	\
	@KWSPLUS_CFLAGS@

kmoduledir = $(libdir)/modules/$(LICENSE_HOST)
kmodule_LTLIBRARIES =			\
	libkppInterface.la		\
	libkppInterfaceStartAll.la

kmodule_DATA =				\
	libkppInterface.kab		\
	libkppInterfaceStartAll.kab

BUILT_SOURCES = $(kmodule_DATA)
CLEANFILES += $(kmodule_DATA)


COMMON_LIBADD = 			\
	@HPPCORBASERVER_LIBS@ 		\
	@KINEOPATHPLANNER_LIBS@ 	\
	@KWSPLUS_LIBS@

COMMON_SOURCES = 			\
	kppInterface.cpp 		\
	kppCommandSolvePlanner.cpp 	\
	kppCommandStartCorbaServer.cpp 	\
	kppCommandPlannerPanel.cpp 	\
	kppCommandInit.cpp 		\
	kppCommandSetConfig.cpp 	\
	kppPlannerPanelController.cpp 	\
	kppPlannerPanel.cpp 		\
	kwsGraphicRoadmapDelegate.cpp 	\
	kwsGraphicRoadmap.cpp

libkppInterface_la_SOURCES = $(COMMON_SOURCES)
libkppInterface_la_CPPFLAGS = $(AM_CPPFLAGS)
libkppInterface_la_LIBADD = $(COMMON_LIBADD)
libkppInterface_la_LDFLAGS = -release $(PACKAGE_VERSION)

libkppInterfaceStartAll_la_SOURCES = $(COMMON_SOURCES)
libkppInterfaceStartAll_la_CPPFLAGS = $(AM_CPPFLAGS) -DKPP_INTERFACE_START_ALL
libkppInterfaceStartAll_la_LIBADD = $(COMMON_LIBADD)
libkppInterfaceStartAll_la_LDFLAGS = -release $(PACKAGE_VERSION)
