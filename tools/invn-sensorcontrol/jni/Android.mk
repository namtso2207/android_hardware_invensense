LOCAL_PATH := $(call my-dir)/../

ifneq ($(NDK_ROOT),)

include $(CLEAR_VARS)
LOCAL_MODULE := invn-sensorcontrol
LOCAL_CFLAGS := -Wall -Wextra -Werror
LOCAL_SRC_FILES :=                \
  SensorEventQueue.cpp            \
  SensorDirectChannel.cpp         \
  SharedMemoryDirectChannel.cpp   \
  HardwareBufferDirectChannel.cpp \
  SensorControl.cpp               \
  InvnSensorControl.cpp
LOCAL_LDLIBS := -landroid
include $(BUILD_EXECUTABLE)

endif
