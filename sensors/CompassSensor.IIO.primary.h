/*
 * Copyright (C) 2014 The Android Open Source Project
 * Copyright (C) 2017-2017 InvenSense, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef COMPASS_SENSOR_H
#define COMPASS_SENSOR_H

#include <stdlib.h>
#include <stdint.h>

#include "sensors.h"
#include "SensorBase.h"

#include "inv_iio_buffer.h"

#define MAX_CHIP_ID_LEN (20)
#define COMPASS_ON_PRIMARY "in_magn_x_raw"

enum COMPASS_SENSOR{
    SENSOR_MAG,
    SENSOR_MAG_CAL,
    SENSOR_MAG_WAKE,
    SENSOR_MAG_CAL_WAKE,
    SENSOR_MAG_NUM
};

struct inv_compass_sensor {
    int64_t delay;
    bool on;
};

class CompassSensor : public SensorBase {

public:
    CompassSensor();
    virtual ~CompassSensor();

    virtual int enable(int32_t handle, int enabled);
    virtual int setDelay(int32_t handle, int64_t ns);
    virtual int64_t getMinDelay() { return mMinDelay; }
    int isCompassSensorPresent();
    // unnecessary for MPL
    virtual int readEvents(sensors_event_t *data, int count)
        { (void)data; (void)count; return 0; }

    int readSample(int *data, int64_t *timestamp);
    int providesCalibration() { return 0; }
    void getOrientationMatrix(signed char *orient);
    int getAccuracy();
    int getSensitivity();
    void fillList(struct sensor_t *list);
    int isIntegrated() { return (0); }
    int getEngineRate();

private:
    enum sysfs_attr {
        BUFFER_ENABLE,
        BUFFER_LENGTH,
        COMPASS_X_ENABLE,
        COMPASS_X_INDEX,
        COMPASS_X_TYPE,
        COMPASS_Y_ENABLE,
        COMPASS_Y_INDEX,
        COMPASS_Y_TYPE,
        COMPASS_Z_ENABLE,
        COMPASS_Z_INDEX,
        COMPASS_Z_TYPE,
        TIMESTAMP_ENABLE,
        TIMESTAMP_INDEX,
        TIMESTAMP_TYPE,
        COMPASS_RATE,
        COMPASS_X_SCALE,
        COMPASS_X_OFFSET,
        COMPASS_Y_SCALE,
        COMPASS_Y_OFFSET,
        COMPASS_Z_SCALE,
        COMPASS_Z_OFFSET,
        TIMESTAMP_SCALE,
        TIMESTAMP_OFFSET,
        COMPASS_ORIENT,
        SYSFS_ATTR_NB,
    };
    char *compassSysFs[SYSFS_ATTR_NB];

    char dev_full_name[MAX_CHIP_ID_LEN];

    enum scan_elements {
        MAG_X_CHANNEL,
        MAG_Y_CHANNEL,
        MAG_Z_CHANNEL,
        TIMESTAMP_CHANNEL,
        CHANNELS_NB,
    };
    struct buffer_scan {
        struct inv_iio_buffer_channel channels[CHANNELS_NB];
        ssize_t addresses[CHANNELS_NB];
        size_t size;
    } compassBufferScan;

    // implementation specific
    signed char mCompassOrientation[9];
    int mEnable;
    int64_t mDelay;
    int64_t mMinDelay;
    int64_t mMaxDelay;
    int64_t mTimestamp;
    char mIIOBuffer[CHANNELS_NB * 8 * IIO_BUFFER_LENGTH];
    struct inv_compass_sensor mCompassSensors[SENSOR_MAG_NUM];

    void enable_iio_sysfs(void);
    int inv_init_sysfs_attributes(void);
};

/*****************************************************************************/

#endif  // COMPASS_SENSOR_H
