# 
#  Copyright 2007 LAAS-CNRS
# 

KMODULE=		kppInterface

kmoduledir=		$(libdir)/modules/$(LICENCE_HOST)
kmodule_LTLIBRARIES = 	libkppInterface.la

install-data-hook:
	-KPP_INSTALL_DIR=$(KINEOPATHPLANNER_PREFIX) \
	LD_LIBRARY_PATH=$(KINEOPATHPLANNER_PREFIX)/lib \
	  $(KINEO_ADDON_BUILD) -m $(kmoduledir)/lib$(KMODULE).so


libkppInterface_la_SOURCES = \
	kppInterface.cpp \
	kppCommandSolvePlanner.cpp \
	kppCommandStartCorbaServer.cpp \
	kwsGraphicRoadmap.cpp

libkppInterface_la_LDFLAGS= \
	@HPPCORBASERVER_LIBS@		\
	-release $(PACKAGE_VERSION)

libkppInterface_la_CPPFLAGS = \
	-I$(top_srcdir)/include\
	@HPPCORBASERVER_CFLAGS@ \
	@KINEOPATHPLANNER_CFLAGS@
