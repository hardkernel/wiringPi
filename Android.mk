LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_C_INCLUDES += \
    $(LOCAL_PATH)/wiringPi

LOCAL_MODULE    := wiringPi
LOCAL_SRC_FILES := \
                   wiringPi/ads1115.c      \
                   wiringPi/mcp23008.c        \
                   wiringPi/mcp4802.c         \
                   wiringPi/piHiPri.c         \
                   wiringPi/sr595.c           \
                   wiringPi/bmp180.c          \
                   wiringPi/mcp23016.c        \
                   wiringPi/odroidc1.c        \
                   wiringPi/odroidn2.c        \
                   wiringPi/piThread.c        \
                   wiringPi/wiringOdroid.c    \
                   wiringPi/mcp23017.c        \
                   wiringPi/odroidc2.c        \
                   wiringPi/drcSerial.c       \
                   wiringPi/mcp23s08.c        \
                   wiringPi/odroidn1.c        \
                   wiringPi/wiringOdroidI2C.c     \
                   wiringPi/ds18b20.c         \
                   wiringPi/mcp23s17.c        \
                   wiringPi/sn3218.c          \
                   wiringPi/wiringOdroidSPI.c     \
                   wiringPi/htu21d.c          \
                   wiringPi/mcp3002.c         \
                   wiringPi/odroidxu3.c       \
                   wiringPi/softPwm.c         \
                   wiringPi/wiringSerial.c    \
                   wiringPi/max31855.c        \
                   wiringPi/mcp3004.c         \
                   wiringPi/pcf8574.c         \
                   wiringPi/softServo.c       \
                   wiringPi/wiringShift.c     \
                   wiringPi/max5322.c         \
                   wiringPi/mcp3422.c         \
                   wiringPi/pcf8591.c         \
                   wiringPi/softTone.c        \
                   wiringPi/wpiExtensions.c

LOCAL_CFLAGS    += -UNDEBUG -DANDROID -Wno-return-type

LOCAL_LDLIBS    := -ldl -llog
include $(BUILD_SHARED_LIBRARY)


include $(CLEAR_VARS)
LOCAL_C_INCLUDES += \
    $(LOCAL_PATH)/wiringPi

LOCAL_MODULE    := wiringPiDev
LOCAL_SRC_FILES := \
    devLib/ds1302.c     \
    devLib/maxdetect.c  \
    devLib/piNes.c      \
    devLib/gertboard.c  \
    devLib/lcd128x64.c  \
    devLib/lcd.c        \
    devLib/piGlow.c     \
    devLib/scrollPhat.c \
    devLib/piFace.c

LOCAL_SHARED_LIBRARIES := libwiringPi

LOCAL_LDLIBS    := -ldl -llog

LOCAL_CFLAGS    += -UNDEBUG -DANDROID  -Wno-return-type
include $(BUILD_SHARED_LIBRARY)


include $(CLEAR_VARS)

LOCAL_MODULE    := gpio
LOCAL_SRC_FILES := \
    gpio/gpio.c    \
    gpio/readall.c \
    gpio/pins.c

LOCAL_C_INCLUDES +=   $(LOCAL_PATH)/wiringPi $(LOCAL_PATH)/devLib/

LOCAL_CFLAGS    += -UNDEBUG -DANDROID -Wno-return-type
LOCAL_LDLIBS := -llog
LOCAL_SHARED_LIBRARIES := libwiringPi libwiringPiDev

include $(BUILD_EXECUTABLE)
