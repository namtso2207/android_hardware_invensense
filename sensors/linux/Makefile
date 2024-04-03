# HAL source files location
HAL_SRC_DIR := ..

# InvenSense chip
INVENSENSE_CHIP ?= icm20948

# InvenSense Sensors HAL configuration
INV_SENSORS_HAL_VERSION_MAJOR := 13
INV_SENSORS_HAL_VERSION_MINOR := 1
INV_SENSORS_HAL_VERSION_PATCH := 1
INV_SENSORS_HAL_VERSION_SUFFIX := -linux
$(info InvenSense MA Sensors HAL version $(INV_SENSORS_HAL_VERSION_MAJOR).$(INV_SENSORS_HAL_VERSION_MINOR).$(INV_SENSORS_HAL_VERSION_PATCH)$(INV_SENSORS_HAL_VERSION_SUFFIX))
# Firmware file directory
INV_FIRMWARE_PATH := /lib/firmware
# Calibration file path
INV_CALIBRATION_FILE := /etc/inv_cal_data.bin
# Compass and/or pressure on primary bus (false by default)
INV_COMPASS_ON_PRIMARY_BUS := false
INV_PRESSURE_ON_PRIMARY_BUS := false
# Step Counter mode, int or poll
# (only applicable for DMP step counter)
INV_STEPCOUNT_INT_MODE := false
INV_DEBUG_DRIVER := false

# ICM20648/ICM20948 specific configuration
# No pre-built library is required
ifneq (,$(filter $(INVENSENSE_CHIP), icm20648 icm20948))
INV_ALGO_LIB := false
else
INV_ALGO_LIB := true
endif

# HiFi support
ifneq (,$(filter $(INVENSENSE_CHIP), icm20690 icm20602 icm42600 iim42600 icm43600 icm45600))
$(info HiFi support with accel 16g FSR and high odr)
CPPFLAGS += -DINV_HIFI_SUPPORT
CPPFLAGS += -DINV_HIFI_ACCEL_16G
CPPFLAGS += -DINV_HIFI_HIGH_ODR
endif

# No secondary i2c bus for certain chips
ifneq (,$(filter $(INVENSENSE_CHIP), icm20608d icm20609 icm20609I icm20602 iam20680 icm42600 iim42600 icm43600))
INV_COMPASS_ON_PRIMARY_BUS := true
INV_PRESSURE_ON_PRIMARY_BUS := true
endif

# Automotive parameters
ifneq (,$(filter $(INVENSENSE_CHIP), iam20680))
CPPFLAGS += -DINV_FUSION_AGM_AUTOMOTIVE
CPPFLAGS += -DINV_GYRO_250DPS
endif

# Use LLVM libc++ with Clang, GNU libstdc++ by default
ifeq ($(CXX), clang++)
CXX_STL = -lc++
CXXFLAGS += -stdlib=libc++
else
CXX_STL = -lstdc++
endif

# Common flags
CPPFLAGS += -DLOG_TAG=\"InvnSensors\"
CPPFLAGS += -DFIRMWARE_PATH=\"$(INV_FIRMWARE_PATH)\"
CPPFLAGS += -DCAL_BIN_CUSTOM_FILE=\"$(INV_CALIBRATION_FILE)\"
CPPFLAGS += -Iandroid_linux
COMMON_FLAGS += -O2
COMMON_FLAGS += -Wall -Wextra -Werror
COMMON_FLAGS += -fPIC -pthread
CFLAGS += -std=gnu11 $(COMMON_FLAGS)
CXXFLAGS += -std=gnu++11 $(COMMON_FLAGS)

# mllite library
MLLITE := libmllite.a
CPPFLAGS += -I$(HAL_SRC_DIR)/software/core/mllite
CPPFLAGS += -I$(HAL_SRC_DIR)/software/core/mllite/linux
CPPFLAGS += -I$(HAL_SRC_DIR)/software/core/driver/include
CPPFLAGS += -DLINUX
MLLITE_SRC_C_FILES := $(HAL_SRC_DIR)/software/core/mllite/start_manager.c
MLLITE_SRC_C_FILES += $(HAL_SRC_DIR)/software/core/mllite/data_builder.c
MLLITE_SRC_C_FILES += $(HAL_SRC_DIR)/software/core/mllite/mlmath.c
MLLITE_SRC_C_FILES += $(HAL_SRC_DIR)/software/core/mllite/mpl.c
MLLITE_SRC_C_FILES += $(HAL_SRC_DIR)/software/core/mllite/ml_math_func.c
MLLITE_SRC_C_FILES += $(HAL_SRC_DIR)/software/core/mllite/hal_outputs.c
MLLITE_SRC_C_FILES += $(HAL_SRC_DIR)/software/core/mllite/message_layer.c
MLLITE_SRC_C_FILES += $(HAL_SRC_DIR)/software/core/mllite/results_holder.c
MLLITE_SRC_C_FILES += $(HAL_SRC_DIR)/software/core/mllite/storage_manager.c
MLLITE_SRC_C_FILES += $(HAL_SRC_DIR)/software/core/mllite/linux/inv_sysfs_utils.c
MLLITE_SRC_C_FILES += $(HAL_SRC_DIR)/software/core/mllite/linux/ml_android_to_imu.c
MLLITE_SRC_C_FILES += $(HAL_SRC_DIR)/software/core/mllite/linux/ml_sysfs_helper.c
MLLITE_SRC_C_FILES += $(HAL_SRC_DIR)/software/core/mllite/linux/ml_load_dmp.c
MLLITE_SRC_C_FILES += $(HAL_SRC_DIR)/software/core/mllite/linux/ml_stored_data.c
MLLITE_SRC_C_FILES += $(HAL_SRC_DIR)/software/core/mllite/linux/ml_sensor_parsing.c
MLLITE_OBJ_FILES := $(MLLITE_SRC_C_FILES:.c=.o) $(MLLITE_SRC_CPP_FILES:.cpp=.o)

# InvenSense Sensors HAL
INVNSENSORS := libinvnsensors.so
CPPFLAGS += -DINV_SENSORS_HAL_VERSION_MAJOR=$(INV_SENSORS_HAL_VERSION_MAJOR)
CPPFLAGS += -DINV_SENSORS_HAL_VERSION_MINOR=$(INV_SENSORS_HAL_VERSION_MINOR)
CPPFLAGS += -DINV_SENSORS_HAL_VERSION_PATCH=$(INV_SENSORS_HAL_VERSION_PATCH)
CPPFLAGS += -DINV_SENSORS_HAL_VERSION_SUFFIX=\"$(INV_SENSORS_HAL_VERSION_SUFFIX)\"
CPPFLAGS += -DINVENSENSE_COMPASS_CAL
ifeq ($(INV_STEPCOUNT_INT_MODE), true)
CPPFLAGS += -DINV_STEPCOUNT_INT_MODE
endif
ifeq ($(INV_DEBUG_DRIVER), true)
CPPFLAGS += -DDEBUG_DRIVER=1
endif
INVNSENSORS_SRC_CPP_FILES := $(HAL_SRC_DIR)/sensors_mpl.cpp
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/SensorBase.cpp
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/MPLSensor.cpp
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/MPLSupport.cpp
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/MPLAdapter.cpp
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/InputEventReader.cpp
INVNSENSORS_SRC_C_FILES += $(HAL_SRC_DIR)/inv_iio_buffer.c
ifeq ($(INV_ALGO_LIB),true)
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/MPLBuilder.cpp
CPPFLAGS += -DINV_USE_ALGO_LIB
CPPFLAGS += -I$(HAL_SRC_DIR)/software/fusion_agm/arm64/include
INVNSENSORS_LDFLAGS += -L$(HAL_SRC_DIR)/software/fusion_agm/arm64/release/lib
INVNSENSORS_LIBS += -lInvnAlgoAGMOpen
endif
ifeq ($(INV_COMPASS_ON_PRIMARY_BUS),true)
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/CompassSensor.IIO.primary.cpp
CPPFLAGS += -DCOMPASS_ON_PRIMARY_BUS
else
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/CompassSensor.IIO.secondary.cpp
endif
ifeq ($(INV_PRESSURE_ON_PRIMARY_BUS),true)
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/PressureSensor.IIO.primary.cpp
CPPFLAGS += -DPRESSURE_ON_PRIMARY_BUS
else
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/PressureSensor.IIO.secondary.cpp
endif
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/LightSensor.IIO.secondary.cpp
INVNSENSORS_LDFLAGS += $(LDFLAGS) -L./ -shared -lpthread $(CXX_STL)
INVNSENSORS_LIBS += -lmllite
INVNSENSORS_OBJ_FILES := $(INVNSENSORS_SRC_C_FILES:.c=.o) $(INVNSENSORS_SRC_CPP_FILES:.cpp=.o)

# test application
TESTAPP := test-sensors-hal
TESTAPP_SRC_CPP_FILES := test-sensors-hal.cpp
TESTAPP_LDFLAGS = $(LDFLAGS) -ldl -lpthread $(CXX_STL)
TESTAPP_OBJ_FILES := $(TESTAPP_SRC_C_FILES:.c=.o) $(TESTAPP_SRC_CPP_FILES:.cpp=.o)

.PHONY: all clean

all: $(INVNSENSORS) $(TESTAPP)

clean:
	-rm -f $(MLLITE_OBJ_FILES) $(MLLITE)
	-rm -f $(INVNSENSORS_OBJ_FILES) $(INVNSENSORS)
	-rm -f $(TESTAPP_OBJ_FILES) $(TESTAPP)

$(MLLITE): $(MLLITE_OBJ_FILES)
	$(AR) cr $@ $^

$(INVNSENSORS): $(MLLITE) $(INVNSENSORS_OBJ_FILES)
	$(CXX) $(CXXFLAGS) $^ $(INVNSENSORS_LDFLAGS) $(INVNSENSORS_LIBS) -o $@

$(TESTAPP): $(TESTAPP_OBJ_FILES)
	$(CXX) $(CXXFLAGS) $^ $(TESTAPP_LDFLAGS) -o $@
