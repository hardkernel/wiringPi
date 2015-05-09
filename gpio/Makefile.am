AM_CPPFLAGS = -I$(top_srcdir)/wiringPi -I$(top_srcdir)/devLib

bin_PROGRAMS = gpio
gpio_LDADD   = $(top_builddir)/devLib/libwiringPiDev.la
gpio_SOURCES = gpio.c extensions.c readall.c pins.c

if WITH_SUID
install-exec-hook:
	chmod 4755 $(DESTDIR)$(bindir)/gpio
endif
