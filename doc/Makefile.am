# 
#  Copyright 2007 JRL AIST CNRS
#
#  Author: Florent Lamiraux
# 


kppInterfacedir = $(datadir)/doc/kppInterface/html

EXTRA_DIST = \
	header.html \
	footer.html \
	package.css \
	pictures/HRP2.jpg \
	additionalDoc/package.h 


install: html-doc
	mkdir -p $(kppInterfacedir)
	$(INSTALL_DATA) html/* $(kppInterfacedir)
	mkdir -p $(kppInterfacedir)/pictures
	$(INSTALL_DATA) $(srcdir)/pictures/* $(kppInterfacedir)/pictures
	$(INSTALL_DATA) kppInterface.doxytag $(kppInterfacedir)

html-doc: package.dox
	doxygen $<
