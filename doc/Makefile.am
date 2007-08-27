# 
#  Copyright 
# 


kppInterfacedir = $(datadir)/doc/kppInterface

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
	mkdir -p $(datadir)/doc/doxytag
	$(INSTALL_DATA) kppInterface.doxytag $(datadir)/doc/doxytag

html-doc: package.dox
	doxygen $<
