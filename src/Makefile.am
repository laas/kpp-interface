# 
#  Copyright 
# 

lib_LTLIBRARIES = libkppInterface.la
libkppInterface_la_SOURCES = \
	kppInterface.cpp \
	kppCommandSolvePlanner.cpp \
	kwsGraphicRoadmap.cpp

libkppInterface_la_LDFLAGS= \
	@HPPCORBASERVER_LIBS@		\
	-release $(PACKAGE_VERSION)

libkppInterface_la_CPPFLAGS = \
	-I$(top_srcdir)/include\
	@HPPCORBASERVER_CFLAGS@ \
	@KINEOPATHPLANNER_CFLAGS@
