lib_LTLIBRARIES = libwiringPi.la

libwiringPi_la_SOURCES = \
	ads1115.c \
	bmp180.c \
	drcNet.c \
	drcSerial.c \
	ds18b20.c \
	htu21d.c \
	max31855.c \
	max5322.c \
	mcp23008.c \
	mcp23016.c \
	mcp23017.c \
	mcp23s08.c \
	mcp23s17.c \
	mcp3002.c \
	mcp3004.c \
	mcp3422.c \
	mcp4802.c \
	odroidc1.c \
	odroidc2.c \
	odroidc4.c \
	odroidc5.c \
	odroidhc4.c \
	odroidm1.c \
	odroidm1s.c \
	odroidm2.c \
	odroidn1.c \
	odroidn2.c \
	odroidxu3.c \
	pcf8574.c \
	pcf8591.c \
	piHiPri.c \
	piThread.c \
	pseudoPins.c \
	rht03.c \
	sn3218.c \
	softPwm.c \
	softServo.c \
	softTone.c \
	sr595.c \
	wiringPi.c \
	wiringPiI2C.c \
	wiringPiSPI.c \
	wiringSerial.c \
	wiringShift.c \
	wpiExtensions.c

libwiringPi_la_CFLAGS = \
	@EXTRA_CFLAGS@ \
	$(LIBCRYPT_CFLAGS)

libwiringPi_la_LDFLAGS = \
	-version-info @LIBWIRINGPI_VERSION@ \
	-lpthread -lm -lrt $(LIBCRYPT_LIBS)
