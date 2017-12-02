if BUILD_DEVLIB
DEVLIB_DIR = devLib
DEVLIB_PC  = wiringPiDev.pc
endif

if BUILD_GPIO
GPIO_DIR = gpio
endif

SUBDIRS = wiringPi $(DEVLIB_DIR) $(GPIO_DIR)

# pkg-config files
pkgconfiglibdir = $(libdir)/pkgconfig
pkgconfiglib_DATA = wiringPi.pc $(DEVLIB_PC)
