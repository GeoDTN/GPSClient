## sub directory

GPSClient_SOURCES = \
       gps_read.h \
        gps_read.cpp 
       #GPSEvent.h \
       #GPSEvent.cpp
#########################################
#-std=c++0x
#-std=gnu++0x
#CXXFLAGS=-std=c++0x
#-std=gnu++11
#-std=c++11
##############################################
AM_CPPFLAGS = -I$(top_srcdir)/src $(ibrdtn_CFLAGS)
AM_LDFLAGS = $(ibrdtn_LIBS) -lgps

if ANDROID
noinst_DATA = Android.mk
CLEANFILES = Android.mk
else
noinst_LTLIBRARIES = libGPSClient.la
libGPSClient_la_SOURCES= $(GPSClient_SOURCES)
endif

Android.mk: Makefile.am
	$(ANDROGENIZER) -:PROJECT dtnd \
		-:REL_TOP $(top_srcdir) -:ABS_TOP $(abs_top_srcdir) \
		-:STATIC libdtnd_rtGPSClient \
		-:SOURCES $(GPSClient_SOURCES) \
		-:CPPFLAGS $(CPPFLAGS) $(AM_CPPFLAGS) \
		-:LDFLAGS $(AM_LDFLAGS) \
		> $@
