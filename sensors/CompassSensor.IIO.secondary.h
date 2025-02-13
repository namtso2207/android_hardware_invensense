/*
 * Copyright (C) 2014 The Android Open Source Project
 * Copyright (C) 2016-2017 InvenSense, Inc.
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

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

// TODO fixme, need input_event
#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <poll.h>

#include "SensorBase.h"
#include "InputEventReader.h"

class CompassSensor : public SensorBase {

public:
    CompassSensor();
    virtual ~CompassSensor();

    virtual int getFd() const;
    virtual int enable(int32_t handle, int enabled);
    virtual int setDelay(int32_t handle, int64_t ns);
    virtual int getEnable(int32_t handle);
    virtual int64_t getDelay(int32_t handle);
    virtual int64_t getMinDelay() { return -1; } // stub

    // unnecessary for MPL
    virtual int readEvents(sensors_event_t *data, int count) {
        (void) data; (void)count; return 0; }

    int isCompassSensorPresent();
    int turnOffCompassFifo(void);
    int turnOnCompassFifo(void);
    int readSample(int *data, int64_t *timestamp);
    int providesCalibration() { return 0; }
    void getOrientationMatrix(signed char *orient);
    int getSensitivity();
    int getAccuracy() { return 0; }
    void fillList(struct sensor_t *list);
    int isIntegrated() { return (1); }
    int isYasCompass(void) { return (0); }
    int checkCoilsReset(void) { return(-1); }

private:
    char sensor_name[200];

    struct sysfs_attrbs {
       char *calib_compass_enable;
       char *compass_enable;
       char *in_magn_enable;
       char *compass_fifo_enable;
       char *compass_x_fifo_enable;
       char *compass_y_fifo_enable;
       char *compass_z_fifo_enable;
       char *calib_compass_rate;
       char *in_magn_rate;
       char *compass_scale;
       char *compass_orient;

       char *in_magn_wake_enable;
       char *in_magn_wake_rate;
       char *calib_compass_wake_enable;
       char *calib_compass_wake_rate;

    } compassSysFs;

    // implementation specific
    signed char mCompassOrientationMatrix[9];
    int mCachedCompassData[3];
    int compass_fd;
    int64_t mCompassTimestamp;
    InputEventCircularReader mCompassInputReader;
    int64_t mDelay;
    int mEnable;
    char *pathP;

    void processCompassEvent(const input_event *event);
    int inv_init_sysfs_attributes(void);
};

/*****************************************************************************/

#endif  // COMPASS_SENSOR_H
