# 
#  Copyright 
# 

SUBDIRS = \
	src \
	include

kppInterfacedir = $(libdir)/pkgconfig
kppInterface_DATA = kppInterface.pc

install-data-hook: documentation

documentation: 
	cd doc; $(MAKE) install
