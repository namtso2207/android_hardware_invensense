# Path to Sensors HAL
SENSORS_HAL_SRC_PATH := ../../../

# Path to libmllite.a
MLLITE_LIB_PATH := $(SENSORS_HAL_SRC_PATH)/linux

# Calibration file path
INV_CALIBRATION_FILE := /etc/inv_cal_data.bin

# Header files for libmllite
CFLAGS += -I $(SENSORS_HAL_SRC_PATH)/software/core/mllite
CFLAGS += -I $(SENSORS_HAL_SRC_PATH)/software/core/driver/include
CFLAGS += -I $(SENSORS_HAL_SRC_PATH)/software/core/mllite/linux

CFLAGS += -DCAL_BIN_CUSTOM_FILE=\"$(INV_CALIBRATION_FILE)\"
CFLAGS += -Wall -Wextra -Werror
CFLAGS += -std=gnu11

INVSELFTEST := inv_self_test
INVSELFTEST_SRC_FILES += ../inv_self_test.c
INVSELFTEST_LDFLAGS += $(LDFLAGS) -L $(MLLITE_LIB_PATH)
INVSELFTEST_LIBS += -lmllite -lm

INVSELFTEST_OBJ_FILES := $(INVSELFTEST_SRC_FILES:.c=.o)


.PHONY: all clean

all: $(INVSELFTEST)

clean:
	-rm -f $(INVSELFTEST_OBJ_FILES) $(INVSELFTEST)

$(INVSELFTEST): $(INVSELFTEST_OBJ_FILES)
	$(CC) $(CFLAGS) $^ $(INVSELFTEST_LDFLAGS) $(INVSELFTEST_LIBS) -o $@

