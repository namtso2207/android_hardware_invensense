/*
 * Copyright (C) 2014-2017 InvenSense, Inc.
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

#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>
#include <stdio.h>
#include <time.h>

#include <cutils/properties.h>

#include "Log.h"
#include "MPLLogger.h"

static bool check_property()
{
    char value[PROPERTY_VALUE_MAX];
    property_get("vendor.invn.hal.logger", value, "0");

    if (atoi(value))
        return true;

    return false;
}

void MPLLogger::createLogFileName()
{
    struct tm *tm;
    time_t now;
    char date_str[32];

    now = time(NULL);
    tm = gmtime(&now);
    strftime(date_str, sizeof(date_str), "%FT%TZ", tm);

    mFileName = mPath + '/' + mPrefix + '_' + date_str + ".log";
}

MPLLogger::MPLLogger(std::string path, std::string prefix) : mEnabled(false), mLogFile(nullptr), mPath(path), mPrefix(prefix)
{
    mLogProp = check_property();
    mLogPropTime = time(NULL);
}

MPLLogger::~MPLLogger()
{
    if (mLogFile != nullptr) {
        fclose(mLogFile);
        mLogFile = nullptr;
    }
}

int MPLLogger::startLog(const sensor_id sensor_type, int64_t timestamp)
{
    const time_t now = time(NULL);

    // check every second
    if (now > mLogPropTime) {
        mLogProp = check_property();
        mLogPropTime = now;
    }

    if (!mLogProp) {
        if (mEnabled) {
            if (mLogFile != nullptr) {
                fclose(mLogFile);
                mLogFile = nullptr;
                LOGI("HAL:Logger Stop logging in file [%s]", mFileName.c_str());
            }
            mEnabled = false;
        }
        return -1;
    }

    if (!mEnabled) {
        createLogFileName();
        mLogFile = fopen(mFileName.c_str(), "a");
        if (mLogFile == nullptr) {
            LOGE("HAL:Logger Cannot open log file [%s]", mFileName.c_str());
            return -1;
        }
        mEnabled = true;
        LOGI("HAL:Logger Start logging in file [%s]", mFileName.c_str());
    }

    return fprintf(mLogFile, "D %s 0x%08x 0 %" PRId64,
                   mSensorStrings[sensor_type].c_str(), sensor_type, timestamp);
}

void MPLLogger::logEvents(const sensor_id sensor_type, const float *values, int accuracy, int64_t timestamp)
{
    int ret;

    ret = startLog(sensor_type, timestamp);
    if (ret < 0)
        return;

    switch (sensor_type) {
    // 3-axis data
    case SENSOR_ACCELEROMETER:
    case SENSOR_MAGNETOMETER:
    case SENSOR_GYROSCOPE:
    case SENSOR_ORIENTATION:
    case SENSOR_GRAVITY:
    case SENSOR_LINEAR_ACCELERATION:
        fprintf(mLogFile, " %.9f %.9f %.9f %d",
                values[0], values[1], values[2],
                accuracy);
        break;
    case SENSOR_UNCAL_GYROSCOPE:
        fprintf(mLogFile, " %.9f %.9f %.9f %.9f %.9f %.9f -1",
                values[0], values[1], values[2],
                values[3], values[4], values[5]);
        break;
    case SENSOR_UNCAL_MAGNETOMETER:
        fprintf(mLogFile, " %.9f %.9f %.9f %.9f %.9f %.9f",
                values[0], values[1], values[2],
                values[3], values[4], values[5]);
        break;
    case SENSOR_GAME_ROTATION_VECTOR:
        fprintf(mLogFile, " %.9f %.9f %.9f %.9f %.6f %d",
                values[0], values[1], values[2], values[3], -1.0, accuracy);
        break;
    case SENSOR_ROTATION_VECTOR:
        fprintf(mLogFile, " %.9f %.9f %.9f %.9f %.9f 0 %d",
                values[0], values[1], values[2], values[3], values[4], accuracy);
        break;        
    case SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
        fprintf(mLogFile, " %.9f %.9f %.9f %.9f %.9f",
                values[0], values[1], values[2], values[3], values[4]);
        break;
    // scalar data
    case SENSOR_LIGHT:
    case SENSOR_PRESSURE:
    case SENSOR_PROXIMITY:
    case SENSOR_TEMPERATURE:
    case SENSOR_HUMIDITY:
    case SENSOR_AMBIENT_TEMPERATURE:
        fprintf(mLogFile, " %.9f", values[0]);
        break;
    // no data
    case SENSOR_SIGNIFICANT_MOTION:
    case SENSOR_STEP_DETECTOR:
    case SENSOR_TILT_DETECTOR:
    case SENSOR_WAKE_GESTURE:
    case SENSOR_GLANCE_GESTURE:
    case SENSOR_PICK_UP_GESTURE:
    case SENSOR_DOUBLE_TAP:
        break;
    default:
        break;
    }

    fprintf(mLogFile, "\r\n");
}

void MPLLogger::logEvents(const sensor_id sensor_type, const int32_t *values, int64_t timestamp)
{
    int ret;

    ret = startLog(sensor_type, timestamp);
    if (ret < 0)
        return;

    switch (sensor_type) {
    case SENSOR_RAW_ACCELEROMETER:
    case SENSOR_RAW_GYROSCOPE:
    case SENSOR_RAW_MAGNETOMETER:
        fprintf(mLogFile, " %d %d %d", values[0], values[1], values[2]);
        break;
    case SENSOR_RAW_TEMPERATURE:
        fprintf(mLogFile, " %d", values[0]);
        break;
    default:
        break;
    }

    fprintf(mLogFile, "\r\n");
}

void MPLLogger::logEvents(const sensor_id sensor_type, uint64_t value, int64_t timestamp)
{
    int ret;

    ret = startLog(sensor_type, timestamp);
    if (ret < 0)
        return;

    switch (sensor_type) {
    case SENSOR_STEP_COUNTER:
        fprintf(mLogFile, " %" PRIu64, value);
        break;
    default:
        break;
    }

    fprintf(mLogFile, "\r\n");
}
