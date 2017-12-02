AM_CPPFLAGS = -I$(top_srcdir)/wiringPi

# headers
include_HEADERS = \
	ds1302.h \
	maxdetect.h \
	gertboard.h \
	piFace.h \
	lcd128x64.h \
	lcd.h \
	piNes.h \
	piGlow.h

# wiringPiDev.{so,a}
lib_LTLIBRARIES = libwiringPiDev.la
libwiringPiDev_la_LDFLAGS = -version-info $(libwiringPiDev_la_version)
libwiringPiDev_la_LIBADD = $(top_builddir)/wiringPi/libwiringPi.la
libwiringPiDev_la_SOURCES = \
	ds1302.c \
	maxdetect.c \
	gertboard.c \
	piFace.c \
	lcd128x64.c \
	lcd.c \
	piNes.c \
	piGlow.c
