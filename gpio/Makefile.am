AM_CPPFLAGS = -I$(top_srcdir)/wiringPi -I$(top_srcdir)/devLib

bin_PROGRAMS = gpio
gpio_LDADD   = $(top_builddir)/devLib/libwiringPiDev.la
gpio_SOURCES = gpio.c extensions.c readall.c pins.c
