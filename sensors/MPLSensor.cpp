/*
 * Copyright (C) 2014-2021 InvenSense, Inc.
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

#define LOG_NDEBUG 0

//see also the EXTRA_VERBOSE define in the MPLSensor.h header file

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <float.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/select.h>
#include <sys/syscall.h>
#include <dlfcn.h>
#include <pthread.h>
#include <string.h>

#include <vector>
#include <string>

#ifdef DIRECT_REPORT
#include <utility>
#include <memory>
#include <sstream>
#include <vector>
#include <unordered_map>
#include "directchannel.h"
#include "SensorChannelMux.h"
#endif

#include "Log.h"
#include "MPLSensor.h"
#include "MPLAdapter.h"
#include "LightSensor.IIO.secondary.h"
#include "MPLSupport.h"
#include "sensor_params.h"
#include "sensors.h"
#include "SensorsList.h"

#include "invensense.h"
#include "ml_stored_data.h"
#include "ml_load_dmp.h"
#include "ml_sysfs_helper.h"
#include "ml_sensor_parsing.h"
#include "ml_android_to_imu.h"

// #define TESTING
// #define DEBUG_TIME_PROFILE
// #define DEBUG_BATCHING 1
#define DEBUG_OUTPUT_CONTROL 0
#define MAX_SYSFS_ATTRB (sizeof(struct sysfs_attrbs) / sizeof(char*))

#if defined DEBUG_DRIVER
#pragma message("HAL:build Invensense sensor debug driver mode")
#endif

/* firmware files directory */
#ifndef FIRMWARE_PATH
#define FIRMWARE_PATH "/system/vendor/firmware"
#endif

/*******************************************************************************
 * MPLSensor class implementation
 ******************************************************************************/

static int64_t prevtime;
static int64_t currentime;
static int64_t htime;
static char sysfs_in_power_on[PATH_MAX];

static const uint64_t sysfsId[] = {
    INV_THREE_AXIS_RAW_GYRO,
    INV_THREE_AXIS_GYRO,
    INV_THREE_AXIS_ACCEL,
    INV_THREE_AXIS_RAW_COMPASS,
    INV_THREE_AXIS_COMPASS,
    INV_ONE_AXIS_PRESSURE,
    INV_ONE_AXIS_LIGHT,
    INV_THREE_AXIS_RAW_GYRO_WAKE,
    INV_THREE_AXIS_GYRO_WAKE,
    INV_THREE_AXIS_ACCEL_WAKE,
    INV_THREE_AXIS_RAW_COMPASS_WAKE,
    INV_THREE_AXIS_COMPASS_WAKE,
    INV_ONE_AXIS_PRESSURE_WAKE,
    INV_ONE_AXIS_LIGHT_WAKE,
    VIRTUAL_SENSOR_9AXES_MASK,
    VIRTUAL_SENSOR_9AXES_MASK_WAKE,
    VIRTUAL_SENSOR_GYRO_6AXES_MASK,
    VIRTUAL_SENSOR_GYRO_6AXES_MASK_WAKE,
    VIRTUAL_SENSOR_MAG_6AXES_MASK,
    VIRTUAL_SENSOR_MAG_6AXES_MASK_WAKE,
    VIRTUAL_SENSOR_LPQ_MASK,
};

// following extended initializer list would only be available with -std=c++11
//  or -std=gnu+11
MPLSensor::MPLSensor(CompassSensor *compass, PressureSensor *pressure)
    : SensorBase(NULL, NULL),
    mMasterSensorMask(INV_ALL_SENSORS_WAKE),
    mLocalSensorMask(0),
    mPollTime(-1),
    mGyroAccuracy(0),
    mAccelAccuracy(0),
    mCompassAccuracy(0),
    iio_fd(-1),
    gyro_temperature_fd(-1),
    accel_x_offset_fd(-1),
    accel_y_offset_fd(-1),
    accel_z_offset_fd(-1),
    accel_x_dmp_bias_fd(-1),
    accel_y_dmp_bias_fd(-1),
    accel_z_dmp_bias_fd(-1),
    gyro_x_offset_fd(-1),
    gyro_y_offset_fd(-1),
    gyro_z_offset_fd(-1),
    gyro_x_dmp_bias_fd(-1),
    gyro_y_dmp_bias_fd(-1),
    gyro_z_dmp_bias_fd(-1),
    compass_x_dmp_bias_fd(-1),
    compass_y_dmp_bias_fd(-1),
    compass_z_dmp_bias_fd(-1),
    dmp_sign_motion_fd(-1),
    mDmpSignificantMotionEnabled(0),
    dmp_pedometer_fd(-1),
    mDmpPedometerEnabled(0),
    mDmpStepCountEnabled(0),
    dmp_pickup_fd(-1),
    mDmpPickupEnabled(0),
    dmp_tilt_fd(-1),
    mEnabled(0),
    mEnabledCached(0),
    mBatchEnabled(0),
    mOldBatchEnabledMask(0),
    mTempCurrentTime(0),
    mAccelScale(8),
    mGyroScale(2000),
    mGyroDmpScaleFactor(41031322),
    mCompassScale(0),
    mFactoryGyroBiasAvailable(false),
    mFactoryGyroBiasLoaded(false),
    mFactoryGyroBiasLpLoaded(false),
    mGyroBiasAvailable(false),
    mGyroBiasLpAvailable(false),
    mGyroBiasApplied(false),
    mFactoryAccelBiasAvailable(false),
    mFactoryAccelBiasLoaded(false),
    mFactoryAccelBiasLpLoaded(false),
    mAccelBiasAvailable(false),
    mAccelBiasLpAvailable(false),
    mAccelBiasApplied(false),
    mCompassBiasAvailable(false),
    mCompassBiasApplied(false),
    mRetryCalFileLoad(false),
    mGyroAccuracyLib(0),
    mGyroLpAccuracyLib(0),
    mAccelAccuracyLib(0),
    mAccelLpAccuracyLib(0),
    mChipDetected(false),
    mGyroLpMode(false),
    mAccelLpMode(false),
    mStepCounterIntMode(false),
    mLastStepCountReadTrigTimestamp(0),
    mFeatureActiveMask(0),
    mPedUpdate(0),
    mPedWakeUpdate(0),
    mTiltUpdate(0),
    mTapUpdate(0),
    mPickupUpdate(0),
    mPressureUpdate(0),
    mEisUpdate(0),
    mEisAuthenticationUpdate(0),
    mGyroSensorTimestamp(0),
    mAccelSensorTimestamp(0),
    mQuatSensorTimestamp(0),
    mQuatSensorLastTimestamp(0),
    m6QuatSensorTimestamp(0),
    m6QuatSensorLastTimestamp(0),
    mGeoQuatSensorTimestamp(0),
    mGeoQuatSensorLastTimestamp(0),
    mStepSensorTimestamp(0),
    mStepSensorWakeTimestamp(0),
    mAlsSensorTimestamp(0),
    mAlsSensorWakeTimestamp(0),
    mLPQTimestamp(0),
    mSensorTimestamp(0),
    mCompassTimestamp(0),
    mPressureTimestamp(0),
    mPressureWakeTimestamp(0),
    mEisTimestamp(0),
    mEisAuthenticationTimestamp(0),
    mRawGyroCustomTimestamp(0),
    mRawAccelCustomTimestamp(0),
    mRawMagCustomTimestamp(0),
    mImuTemperatureTimestamp(0),
    mLastStepCount(0),
    mStepCount(0),
    mLastStepCountSave(0),
    mLastStepCountWake(0),
    mStepCountWake(0),
    mLastStepCountSaveWake(0),
    mStepCounterHandlerNotCalled(true),
    mStepCounterHandlerNotCalledWake(true),
    mSkipReadEvents(0),
    mPressureSensorPresent(0),
    mLightSensorPresent(0),
    mCustomSensorPresent(0),
    mEmptyDataMarkerDetected(0),
    mOisEnabled(0) {

    VFUNC_LOG;

    int ret;

    mCompassSensor = compass;
    mPressureSensor = pressure;

    LOGV_IF(EXTRA_VERBOSE,
            "HAL:MPLSensor constructor : NumSensors = %d", TotalNumSensors);

    pthread_mutex_init(&mHALMutex, NULL);
    memset(mGyroOrientationMatrix, 0, sizeof(mGyroOrientationMatrix));
    memset(mAccelOrientationMatrix, 0, sizeof(mAccelOrientationMatrix));
    memset(mCompassOrientationMatrix, 0, sizeof(mCompassOrientationMatrix));
    memset(mGyroLocation, 0, sizeof(mGyroLocation));
    memset(mAccelLocation, 0, sizeof(mAccelLocation));
    memset(mCompassLocation, 0, sizeof(mCompassLocation));
    memset(mPressureLocation, 0, sizeof(mPressureLocation));
    memset(mLightLocation, 0, sizeof(mLightLocation));
    memset(mProximityLocation, 0, sizeof(mProximityLocation));
    mFlushSensorEnabledVector.reserve(TotalNumSensors);
    memset(mEnabledTime, 0, sizeof(mEnabledTime));
    memset(mChipTemperatureTimestamps, 0, sizeof(mChipTemperatureTimestamps));

    /* setup sysfs paths */
    inv_init_sysfs_attributes();

    /* get chip name */
    if (inv_get_chip_name(chip_ID) != INV_SUCCESS) {
        LOGE("HAL:ERR- Failed to get chip ID\n");
        mChipDetected = false;
    } else {
        LOGV_IF(PROCESS_VERBOSE, "HAL:Chip ID= %s\n", chip_ID);
        mChipDetected = true;
    }

    /* check pressure sensor */
    if (mPressureSensor->isPressureSensorPresent()) {
        mPressureSensorPresent = 1;
    }

    /* check light/prox sensor */
    mLightSensor = new LightSensor((const char*)mSysfsPath);
    if (mLightSensor) {
        if (mLightSensor->isLightSensorPresent())
            mLightSensorPresent = 1;
    }

    /* check custom sensor (Raw accel, Raw gyro, Raw compass) */
    if (CUSTOM_SENSOR)
        mCustomSensorPresent = 1;

    /* print software version string */
    LOGI("InvenSense MA Sensors HAL version %d.%d.%d%s\n",
         INV_SENSORS_HAL_VERSION_MAJOR, INV_SENSORS_HAL_VERSION_MINOR,
         INV_SENSORS_HAL_VERSION_PATCH, INV_SENSORS_HAL_VERSION_SUFFIX);

    enable_iio_sysfs();

    /* Load DMP image */
    loadDMP(chip_ID);

    mCalibrationMode = adapter_get_calibration_mode(chip_ID);
    adapter_set_mode(mCalibrationMode);
#if 0
    /* Get Soft Iron Matrix with sensitivity and scale */
    int tempOrientationMatrix[9];
    if (inv_get_soft_iron_matrix(tempOrientationMatrix, mCompassSoftIron) < 0) {
        LOGV_IF(EXTRA_VERBOSE, "HAL:error getting soft iron matrix");
    } else {
        LOGV_IF(EXTRA_VERBOSE, "HAL:compass soft iron matrix: %d %d %d", mCompassSoftIron[0],
                mCompassSoftIron[4], mCompassSoftIron[8]);
    }
#else
    //get the compass sensitity adjustment

    if (inv_get_compass_sens(mCompassSens) < 0) {
        LOGV_IF(EXTRA_VERBOSE, "HAL:error getting compass sensitivity !");
    } else {
        LOGV_IF(EXTRA_VERBOSE, "HAL:compass sensitivity :  %d %d %d",
                mCompassSens[0], mCompassSens[0], mCompassSens[0]);
    }

    float inSoftIronMatrix[9]= {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    //(chip-frame based) soft iron matrix of compass from customer, it normally needs to be transposed before beeing used here

    float SoftIronMatrix0[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}; //default soft iron matrix of compass
    for (int ii=0; ii < 9; ii++)
        mCompassSoftIron[ii] = (int)(SoftIronMatrix0[ii] * ROT_MATRIX_SCALE_LONG); //init

    for (int ii=0; ii < 9; ii++) {
        if (fabs(inSoftIronMatrix[ii]) > 1.0){//check
            LOGE("input soft iron matrix value is invalid !");
            break;
        } else
            mCompassSoftIron[ii] = (int)(inSoftIronMatrix[ii] * ROT_MATRIX_SCALE_LONG);
    }
#endif
    /* open temperature fd for temp comp */
    LOGV_IF(EXTRA_VERBOSE, "HAL:gyro temperature path: %s", mpu.temperature);
    gyro_temperature_fd = open(mpu.temperature, O_RDONLY);
    if (gyro_temperature_fd == -1) {
        LOGE("HAL:could not open temperature node");
    } else {
        LOGV_IF(EXTRA_VERBOSE,
                "HAL:temperature_fd opened: %s", mpu.temperature);
    }

    /* set gyro FSR: 2000dps for standard Android and 250dps for Automotive */
    int gyro_fsr;
#ifdef INV_GYRO_250DPS
    /* Automotive use case */
    mGyroScale = 250;
    gyro_fsr = 0;
#else
    mGyroScale = 2000;
    gyro_fsr = 3;
#endif
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            gyro_fsr, mpu.gyro_fsr, (long long)getTimestamp());
    ret = write_sysfs_int(mpu.gyro_fsr, gyro_fsr);
    if (ret) {
        LOGE("HAL:Error %d setting gyro FSR", ret);
    }

    /* read gyro FSR */
    LOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%lld)", mpu.gyro_fsr, (long long)getTimestamp());
    ret = read_sysfs_int(mpu.gyro_fsr, &mGyroScale);
    if (ret) {
        LOGE("HAL:Error %d reading gyro FSR", ret);
    }
    LOGV_IF(EXTRA_VERBOSE, "HAL:Gyro FSR used %d", mGyroScale);

    /* read gyro DMP FSR */
    LOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%lld)", mpu.gyro_sf, (long long)getTimestamp());
    ret = read_sysfs_int(mpu.gyro_sf, &mGyroDmpScaleFactor);
    if (ret) {
        LOGE("HAL:Error %d reading gyro DMP FSR", ret);
    }
    LOGV_IF(EXTRA_VERBOSE, "HAL:Gyro Dmp scale factor used %d", mGyroDmpScaleFactor);

    /* open Factory Gyro Bias fd */
    /* mFactoryGyBias contains bias values that will be used for device offset */
    memset(mFactoryGyroBias, 0, sizeof(mFactoryGyroBias));
    memset(mFactoryGyroBiasLp, 0, sizeof(mFactoryGyroBiasLp));
    memset(mGyroBiasUiMode, 0, sizeof(mGyroBiasUiMode));
    LOGV_IF(EXTRA_VERBOSE, "HAL:factory gyro x offset path: %s", mpu.in_gyro_x_offset);
    LOGV_IF(EXTRA_VERBOSE, "HAL:factory gyro y offset path: %s", mpu.in_gyro_y_offset);
    LOGV_IF(EXTRA_VERBOSE, "HAL:factory gyro z offset path: %s", mpu.in_gyro_z_offset);
    gyro_x_offset_fd = open(mpu.in_gyro_x_offset, O_RDWR);
    gyro_y_offset_fd = open(mpu.in_gyro_y_offset, O_RDWR);
    gyro_z_offset_fd = open(mpu.in_gyro_z_offset, O_RDWR);
    if (gyro_x_offset_fd == -1 ||
            gyro_y_offset_fd == -1 || gyro_z_offset_fd == -1) {
        LOGE_IF(0,"HAL:could not open factory gyro calibrated bias");
    } else {
        LOGV_IF(EXTRA_VERBOSE,
                "HAL:gyro_offset opened");
    }

    /* open Gyro Bias fd */
    /* mGyroBias contains bias values that will be used for framework */
    /* mGyroChipBias contains bias values that will be used for dmp */
    if (!(mCalibrationMode & mpl_gyro_cal)) {
        LOGV_IF(EXTRA_VERBOSE, "HAL: gyro x dmp bias path: %s", mpu.in_gyro_x_dmp_bias);
        LOGV_IF(EXTRA_VERBOSE, "HAL: gyro y dmp bias path: %s", mpu.in_gyro_y_dmp_bias);
        LOGV_IF(EXTRA_VERBOSE, "HAL: gyro z dmp bias path: %s", mpu.in_gyro_z_dmp_bias);
        gyro_x_dmp_bias_fd = open(mpu.in_gyro_x_dmp_bias, O_RDWR);
        gyro_y_dmp_bias_fd = open(mpu.in_gyro_y_dmp_bias, O_RDWR);
        gyro_z_dmp_bias_fd = open(mpu.in_gyro_z_dmp_bias, O_RDWR);
        if (gyro_x_dmp_bias_fd == -1 ||
                gyro_y_dmp_bias_fd == -1 || gyro_z_dmp_bias_fd == -1) {
            LOGW("HAL:could not open gyro DMP calibrated bias");
        } else {
            LOGV_IF(EXTRA_VERBOSE,
                    "HAL:gyro_dmp_bias opened");
        }
    }

    /* set accel fsr to driver */
    int accel_fsr; // 0:2g 1:4g 2:8g 3:16g
#ifdef INV_HIFI_ACCEL_16G
    /* 16g */
    accel_fsr = 3;
    mAccelScale = 16;
#else
    /* 8g */
    accel_fsr = 2;
    mAccelScale = 8;
#endif
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            accel_fsr, mpu.accel_fsr, (long long)getTimestamp());
    ret = write_sysfs_int(mpu.accel_fsr, accel_fsr);
    if (ret) {
        LOGE("HAL:Error %d setting accel FSR", ret);
    }

    /* read accel FSR to calcuate accel scale later */
    LOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%lld)", mpu.accel_fsr, (long long)getTimestamp());
    ret = read_sysfs_int(mpu.accel_fsr, &mAccelScale);
    if (ret) {
        LOGE("HAL:Error %d reading accel FSR", ret);
    }
    LOGV_IF(EXTRA_VERBOSE, "HAL:Accel FSR used %d", mAccelScale);;

    /* open Factory Accel Bias fd */
    /* mFactoryAccelBias contains bias values that will be used for device offset */
    memset(mFactoryAccelBias, 0, sizeof(mFactoryAccelBias));
    memset(mFactoryAccelBiasLp, 0, sizeof(mFactoryAccelBiasLp));
    memset(mAccelBiasUiMode, 0, sizeof(mAccelBiasUiMode));
    LOGV_IF(EXTRA_VERBOSE, "HAL:factory accel x offset path: %s", mpu.in_accel_x_offset);
    LOGV_IF(EXTRA_VERBOSE, "HAL:factory accel y offset path: %s", mpu.in_accel_y_offset);
    LOGV_IF(EXTRA_VERBOSE, "HAL:factory accel z offset path: %s", mpu.in_accel_z_offset);
    accel_x_offset_fd = open(mpu.in_accel_x_offset, O_RDWR);
    accel_y_offset_fd = open(mpu.in_accel_y_offset, O_RDWR);
    accel_z_offset_fd = open(mpu.in_accel_z_offset, O_RDWR);
    if (accel_x_offset_fd == -1 ||
            accel_y_offset_fd == -1 || accel_z_offset_fd == -1) {
        LOGW("HAL:could not open factory accel calibrated bias");
    } else {
        LOGV_IF(EXTRA_VERBOSE,
                "HAL:accel_offset opened");
    }

    /* open Accel Bias fd */
    /* mAccelBias contains bias that will be used for dmp */
    if (!(mCalibrationMode & mpl_accel_cal)) {
        LOGV_IF(EXTRA_VERBOSE, "HAL:accel x dmp bias path: %s", mpu.in_accel_x_dmp_bias);
        LOGV_IF(EXTRA_VERBOSE, "HAL:accel y dmp bias path: %s", mpu.in_accel_y_dmp_bias);
        LOGV_IF(EXTRA_VERBOSE, "HAL:accel z dmp bias path: %s", mpu.in_accel_z_dmp_bias);
        accel_x_dmp_bias_fd = open(mpu.in_accel_x_dmp_bias, O_RDWR);
        accel_y_dmp_bias_fd = open(mpu.in_accel_y_dmp_bias, O_RDWR);
        accel_z_dmp_bias_fd = open(mpu.in_accel_z_dmp_bias, O_RDWR);
        if (accel_x_dmp_bias_fd == -1 ||
                accel_y_dmp_bias_fd == -1 || accel_z_dmp_bias_fd == -1) {
            LOGW("HAL:could not open accel DMP calibrated bias");
        } else {
            LOGV_IF(EXTRA_VERBOSE,
                    "HAL:accel_dmp_bias opened");
        }
    }

    /* open Compass Bias fd */
    /* mCompassBias contains bias that will be used for dmp */
    if (!(mCalibrationMode & mpl_compass_cal)) {
        LOGV_IF(EXTRA_VERBOSE, "HAL:compass x dmp bias path: %s", mpu.in_compass_x_dmp_bias);
        LOGV_IF(EXTRA_VERBOSE, "HAL:compass y dmp bias path: %s", mpu.in_compass_y_dmp_bias);
        LOGV_IF(EXTRA_VERBOSE, "HAL:compass z dmp bias path: %s", mpu.in_compass_z_dmp_bias);
        compass_x_dmp_bias_fd = open(mpu.in_compass_x_dmp_bias, O_RDWR);
        compass_y_dmp_bias_fd = open(mpu.in_compass_y_dmp_bias, O_RDWR);
        compass_z_dmp_bias_fd = open(mpu.in_compass_z_dmp_bias, O_RDWR);
        if (compass_x_dmp_bias_fd == -1 ||
                compass_y_dmp_bias_fd == -1 || compass_z_dmp_bias_fd == -1) {
            LOGW("HAL:could not open compass DMP calibrated bias");
        } else {
            LOGV_IF(EXTRA_VERBOSE,
                    "HAL:compass_dmp_bias opened");
        }
    }

    if (!(mCalibrationMode & mpl_smd)) {
        char dummy[4];
        dmp_sign_motion_fd = open(mpu.event_smd, O_RDONLY | O_NONBLOCK);
        if (dmp_sign_motion_fd < 0) {
            LOGE("HAL:ERR couldn't open dmp_sign_motion node");
        } else {
            // read dummy data per driver's request
            int ret = read(dmp_sign_motion_fd, dummy, 4);
            (void)ret;
            LOGV_IF(ENG_VERBOSE,
                "HAL:dmp_sign_motion_fd opened : %d", dmp_sign_motion_fd);
        }
    }
    if (!(mCalibrationMode & mpl_pedo)) {
        char dummy[4];
        dmp_pedometer_fd = open(mpu.event_pedometer, O_RDONLY | O_NONBLOCK);
        if (dmp_pedometer_fd < 0) {
            LOGE("HAL:ERR couldn't open dmp_pedometer node");
        } else {
            // read dummy data per driver's request
            int ret = read(dmp_pedometer_fd, dummy, 4);
            (void)ret;
            LOGV_IF(ENG_VERBOSE,
                "HAL:dmp_pedometer_fd opened : %d", dmp_pedometer_fd);
        }
    }
    if (!(mCalibrationMode & mpl_tilt)) {
        char dummy[4];
        dmp_tilt_fd = open(mpu.event_tilt, O_RDONLY | O_NONBLOCK);
        if (dmp_tilt_fd < 0) {
            LOGE("HAL:ERR couldn't open dmp_tilt node");
        } else {
            // read dummy data per driver's request
            int ret = read(dmp_tilt_fd, dummy, 4);
            (void)ret;
            LOGV_IF(ENG_VERBOSE,
                "HAL:dmp_tilt_fd opened : %d", dmp_tilt_fd);
        }
    }
    if (!(mCalibrationMode & mpl_pickup)) {
        char dummy[4];
        dmp_pickup_fd = open(mpu.event_pickup, O_RDONLY | O_NONBLOCK);
        if (dmp_pickup_fd < 0) {
            LOGE("HAL:ERR couldn't open dmp_pickup node");
        } else {
            // read dummy data per driver's request
            int ret = read(dmp_pickup_fd, dummy, 4);
            (void)ret;
            LOGV_IF(ENG_VERBOSE,
                "HAL:dmp_pickup_fd opened : %d", dmp_pickup_fd);
        }
    }

#if DEBUG_DRIVER
    /* setup Dmp Calibration */
    enableDmpCalibration(1);
#endif

    /* setup sensor bias */
    initBias();

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            0, mpu.debug_determine_engine_on, (long long)getTimestamp());
    write_sysfs_int(mpu.debug_determine_engine_on, 0);

#if DEBUG_DRIVER
    /* debug driver */
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            1, mpu.debug_determine_engine_on, (long long)getTimestamp());
    write_sysfs_int(mpu.debug_determine_engine_on, 1);

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            1, mpu.d_misc_gyro_recalibration, (long long)getTimestamp());
    write_sysfs_int(mpu.d_misc_gyro_recalibration, 1);

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            0, mpu.accel_accuracy_enable, (long long)getTimestamp());
    write_sysfs_int(mpu.accel_accuracy_enable, 0);
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            0, mpu.anglvel_accuracy_enable, (long long)getTimestamp());
    write_sysfs_int(mpu.anglvel_accuracy_enable, 0);
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            0, mpu.magn_accuracy_enable, (long long)getTimestamp());
    write_sysfs_int(mpu.magn_accuracy_enable, 0);
#endif

    /* setup MPL */
    inv_constructor_init();

    /* setup orientation matrix and scale */
    inv_set_device_properties();

    /* initialize adapter */
    adapter_init(&MPLSensor::setPower, chip_ID);

    /* initialize sensor data */
    memset(mPendingEvents, 0, sizeof(mPendingEvents));

    /* Pending Events for HW and Virtual Sensors */
    mPendingEvents[RotationVector].version = sizeof(sensors_event_t);
    mPendingEvents[RotationVector].sensor = ID_RV;
    mPendingEvents[RotationVector].type = SENSOR_TYPE_ROTATION_VECTOR;

    mPendingEvents[GameRotationVector].version = sizeof(sensors_event_t);
    mPendingEvents[GameRotationVector].sensor = ID_GRV;
    mPendingEvents[GameRotationVector].type = SENSOR_TYPE_GAME_ROTATION_VECTOR;

    mPendingEvents[LinearAccel].version = sizeof(sensors_event_t);
    mPendingEvents[LinearAccel].sensor = ID_LA;
    mPendingEvents[LinearAccel].type = SENSOR_TYPE_LINEAR_ACCELERATION;
    mPendingEvents[LinearAccel].acceleration.status
        = SENSOR_STATUS_UNRELIABLE;

    mPendingEvents[Gravity].version = sizeof(sensors_event_t);
    mPendingEvents[Gravity].sensor = ID_GR;
    mPendingEvents[Gravity].type = SENSOR_TYPE_GRAVITY;
    mPendingEvents[Gravity].acceleration.status = SENSOR_STATUS_UNRELIABLE;

    mPendingEvents[Gyro].version = sizeof(sensors_event_t);
    mPendingEvents[Gyro].sensor = ID_GY;
    mPendingEvents[Gyro].type = SENSOR_TYPE_GYROSCOPE;
    mPendingEvents[Gyro].gyro.status = SENSOR_STATUS_UNRELIABLE;

    mPendingEvents[RawGyro].version = sizeof(sensors_event_t);
    mPendingEvents[RawGyro].sensor = ID_RG;
    mPendingEvents[RawGyro].type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;

    mPendingEvents[Accelerometer].version = sizeof(sensors_event_t);
    mPendingEvents[Accelerometer].sensor = ID_A;
    mPendingEvents[Accelerometer].type = SENSOR_TYPE_ACCELEROMETER;
    mPendingEvents[Accelerometer].acceleration.status
        = SENSOR_STATUS_UNRELIABLE;

    mPendingEvents[RawAccelerometer].version = sizeof(sensors_event_t);
    mPendingEvents[RawAccelerometer].sensor = ID_RA;
    mPendingEvents[RawAccelerometer].type = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;

    /* Invensense compass calibration */
    mPendingEvents[MagneticField].version = sizeof(sensors_event_t);
    mPendingEvents[MagneticField].sensor = ID_M;
    mPendingEvents[MagneticField].type = SENSOR_TYPE_MAGNETIC_FIELD;
    mPendingEvents[MagneticField].magnetic.status =
        SENSOR_STATUS_UNRELIABLE;

    mPendingEvents[RawMagneticField].version = sizeof(sensors_event_t);
    mPendingEvents[RawMagneticField].sensor = ID_RM;
    mPendingEvents[RawMagneticField].type = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;

    mPendingEvents[Orientation].version = sizeof(sensors_event_t);
    mPendingEvents[Orientation].sensor = ID_O;
    mPendingEvents[Orientation].type = SENSOR_TYPE_ORIENTATION;
    mPendingEvents[Orientation].orientation.status
        = SENSOR_STATUS_UNRELIABLE;

    mPendingEvents[GeomagneticRotationVector].version = sizeof(sensors_event_t);
    mPendingEvents[GeomagneticRotationVector].sensor = ID_GMRV;
    mPendingEvents[GeomagneticRotationVector].type
        = SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR;

    mPendingEvents[StepCounter].version = sizeof(sensors_event_t);
    mPendingEvents[StepCounter].sensor = ID_SC;
    mPendingEvents[StepCounter].type = SENSOR_TYPE_STEP_COUNTER;

    mPendingEvents[SignificantMotion].version = sizeof(sensors_event_t);
    mPendingEvents[SignificantMotion].sensor = ID_SM;
    mPendingEvents[SignificantMotion].type = SENSOR_TYPE_SIGNIFICANT_MOTION;

    mPendingEvents[StepDetector].version = sizeof(sensors_event_t);
    mPendingEvents[StepDetector].sensor = ID_P;
    mPendingEvents[StepDetector].type = SENSOR_TYPE_STEP_DETECTOR;

    mPendingEvents[Tap].version = sizeof(sensors_event_t);
    mPendingEvents[Tap].sensor = ID_TAP;
    mPendingEvents[Tap].type = SENSOR_TYPE_TAP_GESTURE_SENSOR;

    mPendingEvents[Tilt].version = sizeof(sensors_event_t);
    mPendingEvents[Tilt].sensor = ID_T;
    mPendingEvents[Tilt].type = SENSOR_TYPE_TILT_DETECTOR;

    mPendingEvents[Pickup].version = sizeof(sensors_event_t);
    mPendingEvents[Pickup].sensor = ID_PICK;
    mPendingEvents[Pickup].type = SENSOR_TYPE_PICK_UP_GESTURE;

    mPendingEvents[StationaryDetect].version = sizeof(sensors_event_t);
    mPendingEvents[StationaryDetect].sensor = ID_STADET;
    mPendingEvents[StationaryDetect].type = SENSOR_TYPE_STATIONARY_DETECT;

    mPendingEvents[MotionDetect].version = sizeof(sensors_event_t);
    mPendingEvents[MotionDetect].sensor = ID_MOTDET;
    mPendingEvents[MotionDetect].type = SENSOR_TYPE_MOTION_DETECT;

    mPendingEvents[EISGyroscope].version = sizeof(sensors_event_t);
    mPendingEvents[EISGyroscope].sensor = ID_EISGY;
    mPendingEvents[EISGyroscope].type = SENSOR_TYPE_EIS_GYROSCOPE;

    mPendingEvents[EISAuthentication].version = sizeof(sensors_event_t);
    mPendingEvents[EISAuthentication].sensor = ID_EISAUTHENTICATION;
    mPendingEvents[EISAuthentication].type = SENSOR_TYPE_EIS_AUTHENTICATION;

    mPendingEvents[LPQ].version = sizeof(sensors_event_t);
    mPendingEvents[LPQ].sensor = ID_LPQ;
    mPendingEvents[LPQ].type = SENSOR_TYPE_LPQ;

    mPendingEvents[Heading].version = sizeof(sensors_event_t);
    mPendingEvents[Heading].sensor = ID_HEADING;
    mPendingEvents[Heading].type = SENSOR_TYPE_HEADING;

    /* Pending Events for Wakeup Sensors */
    mPendingEvents[RotationVector_Wake].version = sizeof(sensors_event_t);
    mPendingEvents[RotationVector_Wake].sensor = ID_RVW;
    mPendingEvents[RotationVector_Wake].type = SENSOR_TYPE_ROTATION_VECTOR;

    mPendingEvents[GameRotationVector_Wake].version = sizeof(sensors_event_t);
    mPendingEvents[GameRotationVector_Wake].sensor = ID_GRVW;
    mPendingEvents[GameRotationVector_Wake].type = SENSOR_TYPE_GAME_ROTATION_VECTOR;

    mPendingEvents[LinearAccel_Wake].version = sizeof(sensors_event_t);
    mPendingEvents[LinearAccel_Wake].sensor = ID_LAW;
    mPendingEvents[LinearAccel_Wake].type = SENSOR_TYPE_LINEAR_ACCELERATION;
    mPendingEvents[LinearAccel_Wake].acceleration.status
        = SENSOR_STATUS_UNRELIABLE;

    mPendingEvents[Gravity_Wake].version = sizeof(sensors_event_t);
    mPendingEvents[Gravity_Wake].sensor = ID_GRW;
    mPendingEvents[Gravity_Wake].type = SENSOR_TYPE_GRAVITY;
    mPendingEvents[Gravity_Wake].acceleration.status = SENSOR_STATUS_UNRELIABLE;

    mPendingEvents[Gyro_Wake].version = sizeof(sensors_event_t);
    mPendingEvents[Gyro_Wake].sensor = ID_GYW;
    mPendingEvents[Gyro_Wake].type = SENSOR_TYPE_GYROSCOPE;
    mPendingEvents[Gyro_Wake].gyro.status = SENSOR_STATUS_UNRELIABLE;

    mPendingEvents[RawGyro_Wake].version = sizeof(sensors_event_t);
    mPendingEvents[RawGyro_Wake].sensor = ID_RGW;
    mPendingEvents[RawGyro_Wake].type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;

    mPendingEvents[Accelerometer_Wake].version = sizeof(sensors_event_t);
    mPendingEvents[Accelerometer_Wake].sensor = ID_AW;
    mPendingEvents[Accelerometer_Wake].type = SENSOR_TYPE_ACCELEROMETER;
    mPendingEvents[Accelerometer_Wake].acceleration.status
        = SENSOR_STATUS_UNRELIABLE;

    mPendingEvents[RawAccelerometer_Wake].version = sizeof(sensors_event_t);
    mPendingEvents[RawAccelerometer_Wake].sensor = ID_RAW;
    mPendingEvents[RawAccelerometer_Wake].type = SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;

    /* Invensense compass calibration */
    mPendingEvents[MagneticField_Wake].version = sizeof(sensors_event_t);
    mPendingEvents[MagneticField_Wake].sensor = ID_MW;
    mPendingEvents[MagneticField_Wake].type = SENSOR_TYPE_MAGNETIC_FIELD;
    mPendingEvents[MagneticField_Wake].magnetic.status =
        SENSOR_STATUS_UNRELIABLE;

    mPendingEvents[RawMagneticField_Wake].version = sizeof(sensors_event_t);
    mPendingEvents[RawMagneticField_Wake].sensor = ID_RMW;
    mPendingEvents[RawMagneticField_Wake].type = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;

    mPendingEvents[Orientation_Wake].version = sizeof(sensors_event_t);
    mPendingEvents[Orientation_Wake].sensor = ID_OW;
    mPendingEvents[Orientation_Wake].type = SENSOR_TYPE_ORIENTATION;
    mPendingEvents[Orientation_Wake].orientation.status
        = SENSOR_STATUS_UNRELIABLE;

    mPendingEvents[GeomagneticRotationVector_Wake].version = sizeof(sensors_event_t);
    mPendingEvents[GeomagneticRotationVector_Wake].sensor = ID_GMRVW;
    mPendingEvents[GeomagneticRotationVector_Wake].type
        = SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR;

    mPendingEvents[StepCounter_Wake].version = sizeof(sensors_event_t);
    mPendingEvents[StepCounter_Wake].sensor = ID_SCW;
    mPendingEvents[StepCounter_Wake].type = SENSOR_TYPE_STEP_COUNTER;

    mPendingEvents[StepDetector_Wake].version = sizeof(sensors_event_t);
    mPendingEvents[StepDetector_Wake].sensor = ID_PW;
    mPendingEvents[StepDetector_Wake].type = SENSOR_TYPE_STEP_DETECTOR;

    mPendingEvents[Heading_Wake].version = sizeof(sensors_event_t);
    mPendingEvents[Heading_Wake].sensor = ID_HEADINGW;
    mPendingEvents[Heading_Wake].type = SENSOR_TYPE_HEADING;

    if (mPressureSensorPresent) {
        mPendingEvents[Pressure].version = sizeof(sensors_event_t);
        mPendingEvents[Pressure].sensor = ID_PS;
        mPendingEvents[Pressure].type = SENSOR_TYPE_PRESSURE;

        mPendingEvents[Pressure_Wake].version = sizeof(sensors_event_t);
        mPendingEvents[Pressure_Wake].sensor = ID_PSW;
        mPendingEvents[Pressure_Wake].type = SENSOR_TYPE_PRESSURE;
    }

    if (mLightSensorPresent) {
        mPendingEvents[Light].version = sizeof(sensors_event_t);
        mPendingEvents[Light].sensor = ID_L;
        mPendingEvents[Light].type = SENSOR_TYPE_LIGHT;

        mPendingEvents[Proximity].version = sizeof(sensors_event_t);
        mPendingEvents[Proximity].sensor = ID_PR;
        mPendingEvents[Proximity].type = SENSOR_TYPE_PROXIMITY;

        mPendingEvents[Light_Wake].version = sizeof(sensors_event_t);
        mPendingEvents[Light_Wake].sensor = ID_LW;
        mPendingEvents[Light_Wake].type = SENSOR_TYPE_LIGHT;

        mPendingEvents[Proximity_Wake].version = sizeof(sensors_event_t);
        mPendingEvents[Proximity_Wake].sensor = ID_PRW;
        mPendingEvents[Proximity_Wake].type = SENSOR_TYPE_PROXIMITY;
    }

    if (mCustomSensorPresent) {
        mPendingEvents[GyroRaw].version = sizeof(sensors_event_t);
        mPendingEvents[GyroRaw].sensor = ID_GRC;
        mPendingEvents[GyroRaw].type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;

        mPendingEvents[AccelerometerRaw].version = sizeof(sensors_event_t);
        mPendingEvents[AccelerometerRaw].sensor = ID_ARC;
        mPendingEvents[AccelerometerRaw].type = SENSOR_TYPE_ACCELEROMETER;
        mPendingEvents[AccelerometerRaw].acceleration.status = SENSOR_STATUS_UNRELIABLE;

        mPendingEvents[MagneticFieldRaw].version = sizeof(sensors_event_t);
        mPendingEvents[MagneticFieldRaw].sensor = ID_MRC;
        mPendingEvents[MagneticFieldRaw].type = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
    }

    /* Event Handlers for HW and Virtual Sensors */
    mHandlers[RotationVector] = &MPLSensor::rvHandler;
    mHandlers[GameRotationVector] = &MPLSensor::grvHandler;
    mHandlers[LinearAccel] = &MPLSensor::laHandler;
    mHandlers[Gravity] = &MPLSensor::gravHandler;
    mHandlers[Gyro] = &MPLSensor::gyroHandler;
    mHandlers[RawGyro] = &MPLSensor::rawGyroHandler;
    mHandlers[Accelerometer] = &MPLSensor::accelHandler;
    mHandlers[RawAccelerometer] = &MPLSensor::rawAccelHandler;
    mHandlers[MagneticField] = &MPLSensor::compassHandler;
    mHandlers[RawMagneticField] = &MPLSensor::rawCompassHandler;
    mHandlers[Orientation] = &MPLSensor::orienHandler;
    mHandlers[GeomagneticRotationVector] = &MPLSensor::gmHandler;
    mHandlers[Tilt] = &MPLSensor::tiltHandler;
    mHandlers[Tap] = &MPLSensor::tapHandler;
    mHandlers[Pickup] = &MPLSensor::pickupHandler;
    mHandlers[StationaryDetect] = &MPLSensor::stationaryDetectHandler;
    mHandlers[MotionDetect] = &MPLSensor::motionDetectHandler;
    mHandlers[EISGyroscope] = &MPLSensor::eisHandler;
    mHandlers[EISAuthentication] = &MPLSensor::eisAuthenticationHandler;
    mHandlers[StepCounter] = &MPLSensor::scHandler;
    mHandlers[StepDetector] = &MPLSensor::sdHandler;
    mHandlers[LPQ] = &MPLSensor::lpqHandler;
    mHandlers[Heading] = &MPLSensor::headingHandler;
    /* Event Handlers for Wakeup Sensors */
    mHandlers[RotationVector_Wake] = &MPLSensor::rvwHandler;
    mHandlers[GameRotationVector_Wake] = &MPLSensor::grvwHandler;
    mHandlers[LinearAccel_Wake] = &MPLSensor::lawHandler;
    mHandlers[Gravity_Wake] = &MPLSensor::gravwHandler;
    mHandlers[Gyro_Wake] = &MPLSensor::gyrowHandler;
    mHandlers[RawGyro_Wake] = &MPLSensor::rawGyrowHandler;
    mHandlers[Accelerometer_Wake] = &MPLSensor::accelwHandler;
    mHandlers[RawAccelerometer_Wake] = &MPLSensor::rawAccelwHandler;
    mHandlers[MagneticField_Wake] = &MPLSensor::compasswHandler;
    mHandlers[RawMagneticField_Wake] = &MPLSensor::rawCompasswHandler;
    mHandlers[Orientation_Wake] = &MPLSensor::orienwHandler;
    mHandlers[GeomagneticRotationVector_Wake] = &MPLSensor::gmwHandler;
    mHandlers[StepCounter_Wake] = &MPLSensor::scwHandler;
    mHandlers[StepDetector_Wake] = &MPLSensor::sdwHandler;
    mHandlers[Heading_Wake] = &MPLSensor::headingwHandler;
    /* Event Handler for Secondary devices */
    if (mPressureSensorPresent) {
        mHandlers[Pressure] = &MPLSensor::psHandler;
        mHandlers[Pressure_Wake] = &MPLSensor::pswHandler;
    }
    if (mLightSensorPresent) {
        mHandlers[Light] = &MPLSensor::lightHandler;
        mHandlers[Proximity] = &MPLSensor::proxHandler;
        mHandlers[Light_Wake] = &MPLSensor::lightwHandler;
        mHandlers[Proximity_Wake] = &MPLSensor::proxwHandler;
    }

    if (mCustomSensorPresent) {
        mHandlers[GyroRaw] = &MPLSensor::gyroRawHandler;
        mHandlers[AccelerometerRaw] = &MPLSensor::accelRawHandler;
        mHandlers[MagneticFieldRaw] = &MPLSensor::compassRawHandler;
    }

    /* initialize delays to reasonable values */
    for (int i = 0; i < TotalNumSensors; i++) {
        mDelays[i] = NS_PER_SECOND;
        mBatchDelays[i] = NS_PER_SECOND;
        mBatchTimeouts[i] = 100000000000LL;
    }

    /* initialize Compass Bias */
    memset(mCompassBias, 0, sizeof(mCompassBias));
    memset(mDmpCompassBias, 0, sizeof(mDmpCompassBias));

    /* initialize Accel Bias */
    memset(mFactoryAccelBias, 0, sizeof(mFactoryAccelBias));
    memset(mAccelBias, 0, sizeof(mAccelBias));
    memset(mAccelDmpBias, 0, sizeof(mAccelDmpBias));
    memset(mAccelDmpBiasLp, 0, sizeof(mAccelDmpBiasLp));

    /* initialize Gyro Bias */
    memset(mGyroBias, 0, sizeof(mGyroBias));
    memset(mGyroChipBias, 0, sizeof(mGyroChipBias));
    memset(mDmpGyroBias, 0, sizeof(mDmpGyroBias));
    memset(mDmpGyroBiasLp, 0, sizeof(mDmpGyroBiasLp));

    /* load calibration file */
    if (!loadCalFile())
        mRetryCalFileLoad = true; // failed loading cal file

    /* set pedometer int mode */
    if (!(mCalibrationMode & mpl_pedo)) {
#ifdef INV_STEPCOUNT_INT_MODE
        LOGI("HAL:StepCounter INT mode");
        mStepCounterIntMode = true;
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                1, mpu.pedometer_int_mode, (long long)getTimestamp());
        write_sysfs_int(mpu.pedometer_int_mode, 1);
#else
        LOGI("HAL:StepCounter Poll mode");
        mStepCounterIntMode = false;
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                0, mpu.pedometer_int_mode, (long long)getTimestamp());
        write_sysfs_int(mpu.pedometer_int_mode, 0);
#endif
    }

#ifdef DIRECT_REPORT
    /* add direct report support for accel, gyro, mag, calibrated and uncalibrated and raw, wake-up or normal */
    mDirectChannelHandle = 1;
    mSensorToChannel.emplace(SENSORS_ACCELERATION_HANDLE,
                             std::unordered_map<int32_t, DirectChannelTimingInfo>());
    mSensorToChannel.emplace(SENSORS_GYROSCOPE_HANDLE,
                             std::unordered_map<int32_t, DirectChannelTimingInfo>());
    mSensorToChannel.emplace(SENSORS_MAGNETIC_FIELD_HANDLE,
                             std::unordered_map<int32_t, DirectChannelTimingInfo>());
    mSensorToChannel.emplace(SENSORS_RAW_ACCELERATION_HANDLE,
                             std::unordered_map<int32_t, DirectChannelTimingInfo>());
    mSensorToChannel.emplace(SENSORS_RAW_GYROSCOPE_HANDLE,
                             std::unordered_map<int32_t, DirectChannelTimingInfo>());
    mSensorToChannel.emplace(SENSORS_RAW_MAGNETIC_FIELD_HANDLE,
                             std::unordered_map<int32_t, DirectChannelTimingInfo>());
    mSensorToChannel.emplace(SENSORS_ACCELERATION_WAKEUP_HANDLE,
                             std::unordered_map<int32_t, DirectChannelTimingInfo>());
    mSensorToChannel.emplace(SENSORS_GYROSCOPE_WAKEUP_HANDLE,
                             std::unordered_map<int32_t, DirectChannelTimingInfo>());
    mSensorToChannel.emplace(SENSORS_MAGNETIC_FIELD_WAKEUP_HANDLE,
                             std::unordered_map<int32_t, DirectChannelTimingInfo>());
    mSensorToChannel.emplace(SENSORS_RAW_ACCELERATION_WAKEUP_HANDLE,
                             std::unordered_map<int32_t, DirectChannelTimingInfo>());
    mSensorToChannel.emplace(SENSORS_RAW_GYROSCOPE_WAKEUP_HANDLE,
                             std::unordered_map<int32_t, DirectChannelTimingInfo>());
    mSensorToChannel.emplace(SENSORS_RAW_MAGNETIC_FIELD_WAKEUP_HANDLE,
                             std::unordered_map<int32_t, DirectChannelTimingInfo>());
    if (mCustomSensorPresent) {
        mSensorToChannel.emplace(SENSORS_ACCELERATION_RAW_HANDLE,
                                 std::unordered_map<int32_t, DirectChannelTimingInfo>());
        mSensorToChannel.emplace(SENSORS_GYROSCOPE_RAW_HANDLE,
                                 std::unordered_map<int32_t, DirectChannelTimingInfo>());
        mSensorToChannel.emplace(SENSORS_MAGNETIC_FIELD_RAW_HANDLE,
                                 std::unordered_map<int32_t, DirectChannelTimingInfo>());
    }
#endif

    /* disable all sensors and features */
    enableRawGyro(0);
    enableGyro(0);
    enableAccel(0);
    enableCompass(0);
    enableRawCompass(0);
    enablePressure(0);
    enableBatch(0);

    /* disable Quaternion from DMP */
    enableLPQuaternion(0);
    enableLPQuaternionWake(0);

    prevtime = getTimestamp();
    currentime = getTimestamp();
}

void MPLSensor::enable_iio_sysfs(void)
{
    VFUNC_LOG;

    char iio_device_node[MAX_CHIP_ID_LEN];
    FILE *tempFp = NULL;

    // turn off chip in case
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            0, mpu.chip_enable, (long long)getTimestamp());
    tempFp = fopen(mpu.chip_enable, "w");
    if (tempFp == NULL) {
        LOGE("HAL:could not open chip enable");
    } else {
        if (fprintf(tempFp, "%d", 0) < 0) {
            LOGE("HAL:could not write chip enable");
        }
        fclose(tempFp);
    }

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            1, mpu.scan_el_en, (long long)getTimestamp());
    tempFp = fopen(mpu.scan_el_en, "w");
    if (tempFp == NULL) {
        LOGE("HAL:could not open scan element enable");
    } else {
        if (fprintf(tempFp, "%d", 1) < 0) {
            LOGE("HAL:could not write scan element enable");
        }
        fclose(tempFp);
    }

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            IIO_BUFFER_LENGTH, mpu.buffer_length, (long long)getTimestamp());
    tempFp = fopen(mpu.buffer_length, "w");
    if (tempFp == NULL) {
        LOGE("HAL:could not open buffer length");
    } else {
        if (fprintf(tempFp, "%d", IIO_BUFFER_LENGTH) < 0) {
            LOGE("HAL:could not write buffer length");
        }
        fclose(tempFp);
    }

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            1, mpu.chip_enable, (long long)getTimestamp());
    tempFp = fopen(mpu.chip_enable, "w");
    if (tempFp == NULL) {
        LOGE("HAL:could not open chip enable");
    } else {
        if (fprintf(tempFp, "%d", 1) < 0) {
            LOGE("HAL:could not write chip enable");
        }
        fclose(tempFp);
    }

    inv_get_iio_device_node(iio_device_node);
    iio_fd = open(iio_device_node, O_RDONLY);
    if (iio_fd < 0) {
        LOGE("HAL:could not open iio device node");
    } else {
        LOGV_IF(ENG_VERBOSE, "HAL:iio iio_fd opened : %d", iio_fd);
    }
}

int MPLSensor::inv_constructor_init(void)
{
    VFUNC_LOG;

    inv_error_t result = inv_init_mpl();
    if (result) {
        LOGE("HAL:inv_init_mpl() failed");
        return result;
    }
    result = inv_constructor_default_enable();
    result = inv_start_mpl();
    if (result) {
        LOGE("HAL:inv_start_mpl() failed");
        LOG_RESULT_LOCATION(result);
        return result;
    }

    return result;
}

int MPLSensor::inv_constructor_default_enable(void)
{
    VFUNC_LOG;

    inv_error_t result;

    result = inv_enable_hal_outputs();

    return result;
}

/* TODO: create function pointers to calculate scale */
void MPLSensor::inv_set_device_properties(void)
{
    VFUNC_LOG;

    inv_get_sensors_orientation();

    adapter_set_sample_rate(DEFAULT_MPL_GYRO_RATE, ID_GY);
    adapter_set_sample_rate(DEFAULT_MPL_COMPASS_RATE, ID_M);

    /* gyro setup */
    mGyroOrientationScalar = inv_orientation_matrix_to_scalar(mGyroOrientationMatrix);
    adapter_set_gyro_orientation_and_scale(mGyroOrientationScalar, mGyroScale << 15);
    LOGI_IF(EXTRA_VERBOSE, "HAL: Set orient %d, MPL Gyro Scale %d", mGyroOrientationScalar, mGyroScale << 15);

    /* accel setup */
    mAccelOrientationScalar = inv_orientation_matrix_to_scalar(mAccelOrientationMatrix);
    adapter_set_accel_orientation_and_scale(mAccelOrientationScalar, mAccelScale << 15);
    LOGI_IF(EXTRA_VERBOSE,
            "HAL: Set orient %d, MPL Accel Scale %d", mAccelOrientationScalar, mAccelScale << 15);

    /* compass setup */
    mCompassSensor->getOrientationMatrix(mCompassOrientationMatrix);
    mCompassOrientationScalar = inv_orientation_matrix_to_scalar(mCompassOrientationMatrix);

    mCompassScale = mCompassSensor->getSensitivity();
    adapter_set_compass_orientation_and_scale(mCompassOrientationScalar, mCompassScale, mCompassSens, mCompassSoftIron);
    LOGI_IF(EXTRA_VERBOSE,
            "HAL: Set MPL Compass Scale %d", mCompassScale);
}

void MPLSensor::loadDMP(char *chipID)
{
    VFUNC_LOG;

    int fd;
    FILE *fptr;
    int firmware_loaded = 0;
    int ret;

    if (isMpuNonDmp()) {
        return;
    }

    /* load DMP firmware */
    LOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%lld)", mpu.firmware_loaded, (long long)getTimestamp());
    fptr = fopen(mpu.firmware_loaded, "r");
    if (fptr == NULL) {
        LOGE("HAL:could not open DMP state");
    } else {
        LOGV_IF(EXTRA_VERBOSE, "HAL:load DMP");
	if (strcmp(chip_ID, "ICM45600") == 0) {
	       ret = fscanf(fptr, "%d", &firmware_loaded);
	       LOGI("HAL: firmware loaded=%d (%d)", firmware_loaded, ret);
        } else {
	        /* Modify to load other chipsets */
	        int tempID = 20648;
	        if (strcmp(chipID, "ICM20608D") == 0)
	            tempID = 206080;
	        if (inv_load_dmp(mSysfsPath, tempID, FIRMWARE_PATH) < 0) {
	            LOGE("HAL:load DMP failed");
	        } else {
	            LOGV_IF(EXTRA_VERBOSE, "HAL:DMP loaded");
	        }
	}
	fclose(fptr);
    }

    /* initialize DMP */
    char version_buf[16];
    int version_number = adapter_get_version();
    fd = open(mpu.dmp_init, O_WRONLY);
    if (fd < 0) {
        LOGE("HAL:could not initialize DMP");
    } else {
        if (write_attribute_sensor(fd, version_number) < 0) {
            LOGE("HAL:Error checking DMP version");
            return;
        }
        fd = open(mpu.dmp_init, O_RDONLY);
        if (read_attribute_sensor(fd, version_buf, sizeof(version_buf)) < 0) {
            LOGE("HAL:Error reading DMP version");
            return;
        } else {
            int tempVersion = atoi(version_buf);
            adapter_set_version(tempVersion);
            LOGV_IF(EXTRA_VERBOSE, "HAL:read DMP version=%d", tempVersion);
        }
    }
}

void MPLSensor::inv_get_sensors_orientation(void)
{
    VFUNC_LOG;

    FILE *fptr;

    // get gyro orientation
    LOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%lld)", mpu.gyro_orient, (long long)getTimestamp());
    fptr = fopen(mpu.gyro_orient, "r");
    if (fptr != NULL) {
        int om[9];
        if (fscanf(fptr, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    &om[0], &om[1], &om[2], &om[3], &om[4], &om[5],
                    &om[6], &om[7], &om[8]) < 0) {
            LOGE("HAL:Could not read gyro mounting matrix");
        } else {
            LOGV_IF(EXTRA_VERBOSE,
                    "HAL:gyro mounting matrix: "
                    "%+d %+d %+d %+d %+d %+d %+d %+d %+d",
                    om[0], om[1], om[2], om[3], om[4], om[5], om[6], om[7], om[8]);

            mGyroOrientationMatrix[0] = om[0];
            mGyroOrientationMatrix[1] = om[1];
            mGyroOrientationMatrix[2] = om[2];
            mGyroOrientationMatrix[3] = om[3];
            mGyroOrientationMatrix[4] = om[4];
            mGyroOrientationMatrix[5] = om[5];
            mGyroOrientationMatrix[6] = om[6];
            mGyroOrientationMatrix[7] = om[7];
            mGyroOrientationMatrix[8] = om[8];
        }
        fclose(fptr);
    }

    // get accel orientation
    LOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%lld)", mpu.accel_orient, (long long)getTimestamp());
    fptr = fopen(mpu.accel_orient, "r");
    if (fptr != NULL) {
        int om[9];
        if (fscanf(fptr, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    &om[0], &om[1], &om[2], &om[3], &om[4], &om[5],
                    &om[6], &om[7], &om[8]) < 0) {
            LOGE("HAL:could not read accel mounting matrix");
        } else {
            LOGV_IF(EXTRA_VERBOSE,
                    "HAL:accel mounting matrix: "
                    "%+d %+d %+d %+d %+d %+d %+d %+d %+d",
                    om[0], om[1], om[2], om[3], om[4], om[5], om[6], om[7], om[8]);

            mAccelOrientationMatrix[0] = om[0];
            mAccelOrientationMatrix[1] = om[1];
            mAccelOrientationMatrix[2] = om[2];
            mAccelOrientationMatrix[3] = om[3];
            mAccelOrientationMatrix[4] = om[4];
            mAccelOrientationMatrix[5] = om[5];
            mAccelOrientationMatrix[6] = om[6];
            mAccelOrientationMatrix[7] = om[7];
            mAccelOrientationMatrix[8] = om[8];
        }
        fclose(fptr);
    }

    // Fill sensors location here if needed
    // mGyroLocation = {0, 0, 0};
    // mAccelLocation = {0, 0, 0};
    // mCompassLocation = {0, 0, 0};
    // mPressureLocation = {0, 0, 0};
    // mLightLocation = {0, 0, 0};
    // mProximityLocation = {0, 0, 0};
}

MPLSensor::~MPLSensor()
{
    VFUNC_LOG;

    /* Close open fds */
    if (iio_fd >= 0) {
        close(iio_fd);
    }
    if (gyro_temperature_fd >= 0) {
        close(gyro_temperature_fd);
    }
    if (accel_x_offset_fd >= 0) {
        close(accel_x_offset_fd);
    }
    if (accel_y_offset_fd >= 0) {
        close(accel_y_offset_fd);
    }
    if (accel_z_offset_fd >= 0) {
        close(accel_z_offset_fd);
    }
    if (accel_x_dmp_bias_fd >= 0) {
        close(accel_x_dmp_bias_fd);
    }
    if (accel_y_dmp_bias_fd >= 0) {
        close(accel_y_dmp_bias_fd);
    }
    if (accel_z_dmp_bias_fd >= 0) {
        close(accel_z_dmp_bias_fd);
    }
    if (gyro_x_offset_fd >= 0) {
        close(gyro_x_offset_fd);
    }
    if (gyro_y_offset_fd >= 0) {
        close(gyro_y_offset_fd);
    }
    if (gyro_z_offset_fd >= 0) {
        close(gyro_z_offset_fd);
    }
    if (gyro_x_dmp_bias_fd >= 0) {
        close(gyro_x_dmp_bias_fd);
    }
    if (gyro_y_dmp_bias_fd >= 0) {
        close(gyro_y_dmp_bias_fd);
    }
    if (gyro_z_dmp_bias_fd >= 0) {
        close(gyro_z_dmp_bias_fd);
    }
    if (compass_x_dmp_bias_fd >= 0) {
        close(compass_x_dmp_bias_fd);
    }
    if (compass_y_dmp_bias_fd >= 0) {
        close(compass_y_dmp_bias_fd);
    }
    if (compass_z_dmp_bias_fd >= 0) {
        close(compass_z_dmp_bias_fd);
    }
    if (dmp_sign_motion_fd >= 0) {
        close(dmp_sign_motion_fd);
    }
    if (dmp_pedometer_fd >= 0) {
        close(dmp_pedometer_fd);
    }
    if (dmp_pickup_fd >= 0) {
        close(dmp_pickup_fd);
    }
    if (dmp_tilt_fd >= 0) {
        close(dmp_tilt_fd);
    }

    free(sysfs_names_ptr);
    delete[] mCurrentSensorMask;
    delete[] mSysfsMask;
}

void MPLSensor::inv_set_engine_rate(uint32_t rate, uint64_t mask)
{
    uint32_t i;

    for (i = 0; i < mNumSysfs; i++) {
        if (mask == mSysfsMask[i].sensorMask) {
            mSysfsMask[i].engineRate = rate;
        }
    }
}

void MPLSensor::inv_write_sysfs(uint32_t delay, uint64_t mask, const char *sysfs_rate)
{
    int tempFd, res;
    float rate;
    int period;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %.0f > %s (%lld)",
        NS_PER_SECOND_FLOAT / delay, sysfs_rate, (long long)getTimestamp());
    tempFd = open(sysfs_rate, O_RDWR);
    res = write_attribute_sensor(tempFd, NS_PER_SECOND_FLOAT / delay);
    if (res < 0) {
        LOGE("HAL:%s update delay error", sysfs_rate);
    }
    read_sysfs_float(sysfs_rate, &rate);
    if (rate == 0) return;
    period = (double)NS_PER_SECOND / (double)rate;
    inv_set_engine_rate(period, mask);

    LOGV_IF(SYSFS_VERBOSE, "read engine %s rate= %d\n", sysfs_rate, period);
}

void MPLSensor::inv_read_sysfs_engine(uint64_t mask, const char *_rate)
{
    int rate = 0;

#ifdef COMPASS_ON_PRIMARY_BUS
    if (mask == INV_THREE_AXIS_RAW_COMPASS ||
        mask == INV_THREE_AXIS_RAW_COMPASS_WAKE ||
        mask == INV_THREE_AXIS_COMPASS ||
        mask == INV_THREE_AXIS_COMPASS_WAKE) {
        rate = mCompassSensor->getEngineRate();
        if (rate == 0) return;
        rate = NS_PER_SECOND/rate;
        inv_set_engine_rate(rate, mask);
        LOGV_IF(SYSFS_VERBOSE, "read primary compass rate= %d\n", rate);
        return;
    }
#endif

    static char sysfs_rate[MAX_SYSFS_NAME_LEN + 1];

    snprintf(sysfs_rate, sizeof(sysfs_rate), "%s/%s", mSysfsPath, _rate);
    read_sysfs_int(sysfs_rate, &rate);
    if (rate == 0) return;
    rate = NS_PER_SECOND/rate;
    inv_set_engine_rate(rate, mask);

    LOGV_IF(SYSFS_VERBOSE, "read engine %s rate= %d\n", sysfs_rate, rate);
}

void MPLSensor::setRawGyroRate(uint64_t delay)
{
    inv_write_sysfs(delay, INV_THREE_AXIS_RAW_GYRO, mpu.gyro_rate);
    inv_read_sysfs_engine(INV_THREE_AXIS_RAW_GYRO_WAKE, (char*)"in_anglvel_wake_rate");
}

void MPLSensor::setRawGyroRateWake(uint64_t delay)
{
    inv_write_sysfs(delay, INV_THREE_AXIS_RAW_GYRO_WAKE, mpu.gyro_wake_rate);
    inv_read_sysfs_engine(INV_THREE_AXIS_RAW_GYRO, (char*)"in_anglvel_rate");
}

void MPLSensor::setGyroRate(uint64_t delay)
{
    inv_write_sysfs(delay, INV_THREE_AXIS_GYRO, mpu.calib_gyro_rate);
    inv_read_sysfs_engine(INV_THREE_AXIS_GYRO_WAKE, (char*)"in_calib_anglvel_wake_rate");
}

void MPLSensor::setGyroRateWake(uint64_t delay)
{
    inv_write_sysfs(delay, INV_THREE_AXIS_GYRO_WAKE, mpu.calib_gyro_wake_rate);
    inv_read_sysfs_engine(INV_THREE_AXIS_GYRO, (char*)"in_calib_anglvel_rate");
}

void MPLSensor::setAccelRate(uint64_t delay)
{
    inv_write_sysfs(delay, INV_THREE_AXIS_ACCEL, mpu.accel_rate);
    inv_read_sysfs_engine(INV_THREE_AXIS_ACCEL_WAKE, (char*)"in_accel_wake_rate");
}

void MPLSensor::setAccelRateWake(uint64_t delay)
{
    inv_write_sysfs(delay, INV_THREE_AXIS_ACCEL_WAKE, mpu.accel_wake_rate);
    inv_read_sysfs_engine(INV_THREE_AXIS_ACCEL, (char*)"in_accel_rate");
}

void MPLSensor::set6AxesRate(uint64_t delay)
{
    inv_write_sysfs(delay, VIRTUAL_SENSOR_GYRO_6AXES_MASK, mpu.six_axis_q_rate);
    inv_read_sysfs_engine(VIRTUAL_SENSOR_GYRO_6AXES_MASK_WAKE, (char*)"in_6quat_wake_rate");
}

void MPLSensor::set3AxesRate(uint64_t delay)
{
    inv_write_sysfs(delay, VIRTUAL_SENSOR_LPQ_MASK, mpu.three_axis_q_rate);
    inv_read_sysfs_engine(VIRTUAL_SENSOR_LPQ_MASK, (char*)"in_3quat_rate");
}

void MPLSensor::set6AxesRateWake(uint64_t delay)
{
    inv_write_sysfs(delay, VIRTUAL_SENSOR_GYRO_6AXES_MASK_WAKE, mpu.six_axis_q_wake_rate);
    inv_read_sysfs_engine(VIRTUAL_SENSOR_GYRO_6AXES_MASK, (char*)"in_6quat_rate");
}

void MPLSensor::set9AxesRate(uint64_t delay)
{
    inv_write_sysfs(delay, VIRTUAL_SENSOR_9AXES_MASK, mpu.nine_axis_q_rate);
    inv_read_sysfs_engine(VIRTUAL_SENSOR_9AXES_MASK_WAKE, (char*)"in_9quat_wake_rate");
}

void MPLSensor::set9AxesRateWake(uint64_t delay)
{
    inv_write_sysfs(delay, VIRTUAL_SENSOR_9AXES_MASK_WAKE, mpu.nine_axis_q_wake_rate);
    inv_read_sysfs_engine(VIRTUAL_SENSOR_9AXES_MASK, (char*)"in_9quat_rate");
}

void MPLSensor::set6AxesMagRate(uint64_t delay)
{
    inv_write_sysfs(delay, VIRTUAL_SENSOR_MAG_6AXES_MASK, mpu.in_geomag_rate);
    inv_read_sysfs_engine(VIRTUAL_SENSOR_MAG_6AXES_MASK_WAKE, (char*)"in_geomag_wake_rate");
}

void MPLSensor::set6AxesMagRateWake(uint64_t delay)
{
    inv_write_sysfs(delay, VIRTUAL_SENSOR_MAG_6AXES_MASK_WAKE, mpu.in_geomag_wake_rate);
    inv_read_sysfs_engine(VIRTUAL_SENSOR_MAG_6AXES_MASK, (char*)"in_geomag_rate");
}

void MPLSensor::setRawMagRate(uint64_t delay)
{
    char rate[MAX_SYSFS_NAME_LEN];

    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_magn_rate");
    mCompassSensor->setDelay(ID_RM, delay);
    inv_read_sysfs_engine(INV_THREE_AXIS_RAW_COMPASS, rate);
    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_magn_wake_rate");
    inv_read_sysfs_engine(INV_THREE_AXIS_RAW_COMPASS_WAKE, rate);
}

void MPLSensor::setRawMagRateWake(uint64_t delay)
{
    char rate[MAX_SYSFS_NAME_LEN];

    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_magn_wake_rate");
    mCompassSensor->setDelay(ID_RMW, delay);
    inv_read_sysfs_engine(INV_THREE_AXIS_RAW_COMPASS_WAKE, rate);
    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_magn_rate");
    inv_read_sysfs_engine(INV_THREE_AXIS_RAW_COMPASS, rate);
}

void MPLSensor::setMagRate(uint64_t delay)
{
    char rate[MAX_SYSFS_NAME_LEN];

    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_calib_magn_rate");
    mCompassSensor->setDelay(ID_M, delay);
    inv_read_sysfs_engine(INV_THREE_AXIS_COMPASS, rate);
    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_calib_magn_wake_rate");
    inv_read_sysfs_engine(INV_THREE_AXIS_COMPASS_WAKE, rate);
}

void MPLSensor::setMagRateWake(uint64_t delay)
{
    char rate[MAX_SYSFS_NAME_LEN];

    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_calib_magn_wake_rate");
    mCompassSensor->setDelay(ID_MW, delay);
    inv_read_sysfs_engine(INV_THREE_AXIS_COMPASS_WAKE, rate);
    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_calib_magn_rate");
    inv_read_sysfs_engine(INV_THREE_AXIS_COMPASS, rate);
}

void MPLSensor::setPressureRate(uint64_t delay)
{
    char rate[MAX_SYSFS_NAME_LEN];

    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_pressure_rate");
    mPressureSensor->setDelay(ID_PS, delay);
    inv_read_sysfs_engine(INV_ONE_AXIS_PRESSURE, rate);
    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_pressure_wake_rate");
    inv_read_sysfs_engine(INV_ONE_AXIS_PRESSURE_WAKE, rate);
}

void MPLSensor::setPressureRateWake(uint64_t delay)
{
    char rate[MAX_SYSFS_NAME_LEN];

    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_pressure_wake_rate");
    mPressureSensor->setDelay(ID_PSW, delay);
    inv_read_sysfs_engine(INV_ONE_AXIS_PRESSURE_WAKE, rate);
    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_pressure_rate");
    inv_read_sysfs_engine(INV_ONE_AXIS_PRESSURE, rate);
}

void MPLSensor::setLightRate(uint64_t delay)
{
    char rate[MAX_SYSFS_NAME_LEN];

    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_als_px_rate");
    mLightSensor->setDelay(ID_L, delay);
    inv_read_sysfs_engine(INV_ONE_AXIS_LIGHT, rate);
    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_als_px_wake_rate");
    inv_read_sysfs_engine(INV_ONE_AXIS_LIGHT_WAKE, rate);
}

void MPLSensor::setLightRateWake(uint64_t delay)
{
    char rate[MAX_SYSFS_NAME_LEN];

    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_als_px_wake_rate");
    mLightSensor->setDelay(ID_LW, delay);
    inv_read_sysfs_engine(INV_ONE_AXIS_LIGHT_WAKE, rate);
    memset(rate, 0, sizeof(rate));
    sprintf(rate, "in_als_px_rate");
    inv_read_sysfs_engine(INV_ONE_AXIS_LIGHT, rate);
}

int MPLSensor::set6AxisQuaternionRate(int64_t wanted)
{
    VFUNC_LOG;
    int res = 0;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            int(NS_PER_SECOND_FLOAT / wanted), mpu.six_axis_q_rate,
            (long long)getTimestamp());
    res = write_sysfs_int(mpu.six_axis_q_rate, NS_PER_SECOND_FLOAT / wanted);
    res = write_sysfs_int(mpu.nine_axis_q_rate, NS_PER_SECOND_FLOAT / wanted);
    res = write_sysfs_int(mpu.in_geomag_rate, NS_PER_SECOND_FLOAT / wanted);

    LOGV_IF(PROCESS_VERBOSE,
            "HAL:DMP six axis rate %.2f Hz", NS_PER_SECOND_FLOAT / wanted);

    return res;
}

int MPLSensor::set6AxisQuaternionWakeRate(int64_t wanted)
{
    VFUNC_LOG;
    int res = 0;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            int(NS_PER_SECOND_FLOAT / wanted), mpu.six_axis_q_wake_rate,
            (long long)getTimestamp());
    res = write_sysfs_int(mpu.six_axis_q_wake_rate, NS_PER_SECOND_FLOAT / wanted);
    res = write_sysfs_int(mpu.nine_axis_q_wake_rate, NS_PER_SECOND_FLOAT / wanted);
    res = write_sysfs_int(mpu.in_geomag_wake_rate, NS_PER_SECOND_FLOAT / wanted);

    LOGV_IF(PROCESS_VERBOSE,
            "HAL:DMP six axis wake rate %.2f Hz", NS_PER_SECOND_FLOAT / wanted);

    return res;
}

int MPLSensor::enable9AxisQuaternion(int en)
{
    VFUNC_LOG;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.nine_axis_q_on, (long long)getTimestamp());
    if (write_sysfs_int(mpu.nine_axis_q_on, en) < 0) {
        LOGE("HAL:ERR can't write DMP nine_axis_q_on");
        return -1;
    }
    if (!en) {
        mFeatureActiveMask &= ~INV_DMP_9AXIS_QUATERNION;
        LOGV_IF(ENG_VERBOSE, "HAL:9 Axis Quat disabled");
    } else {
        mFeatureActiveMask |= INV_DMP_9AXIS_QUATERNION;
        LOGV_IF(PROCESS_VERBOSE, "HAL:9 Axis Quat enabled");
    }
    return 0;
}

int MPLSensor::enable9AxisQuaternionWake(int en)
{
    VFUNC_LOG;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.nine_axis_q_wake_on, (long long)getTimestamp());
    if (write_sysfs_int(mpu.nine_axis_q_wake_on, en) < 0) {
        LOGE("HAL:ERR can't write DMP nine_axis_q_wake_on");
        return -1;
    }
    if (!en) {
        mFeatureActiveMask &= ~(INV_DMP_9AXIS_QUATERNION_WAKE);
        LOGV_IF(ENG_VERBOSE, "HAL:9 Axis Quat wake disabled");
    } else {
        mFeatureActiveMask |= (INV_DMP_9AXIS_QUATERNION_WAKE);
        LOGV_IF(PROCESS_VERBOSE, "HAL:9 Axis Quat wake enabled");
    }
    return 0;
}

int MPLSensor::enableCompass6AxisQuaternion(int en)
{
    VFUNC_LOG;

    int res = 0;
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.in_geomag_enable, (long long)getTimestamp());
    if (write_sysfs_int(mpu.in_geomag_enable, en) < 0) {
        LOGE("HAL:ERR can't write geomag enable");
        res = -1;
    }
    return res;
}

int MPLSensor::enableCompass6AxisQuaternionWake(int en)
{
    VFUNC_LOG;

    int res = 0;
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.in_geomag_wake_enable, (long long)getTimestamp());
    if (write_sysfs_int(mpu.in_geomag_wake_enable, en) < 0) {
        LOGE("HAL:ERR can't write geomag wake enable");
        res = -1;
    }
    return res;
}

int MPLSensor::enableLPQuaternion(int en)
{
    VFUNC_LOG;

    if (mCalibrationMode & mpl_quat)
        return 0;

    if (!en) {
        enableQuaternionData(0);
        mFeatureActiveMask &= ~INV_DMP_6AXIS_QUATERNION;
        LOGV_IF(ENG_VERBOSE, "HAL:LP Quat disabled");
    } else {
        if (enableQuaternionData(1) < 0) {
            LOGE("HAL:ERR can't enable LP Quaternion");
        } else {
            mFeatureActiveMask |= INV_DMP_6AXIS_QUATERNION;
            LOGV_IF(ENG_VERBOSE, "HAL:LP Quat enabled");
        }
    }
    return 0;
}

int MPLSensor::enableLPQuaternionWake(int en)
{
    VFUNC_LOG;

    if (mCalibrationMode & mpl_quat)
        return 0;

    if (!en) {
        enableQuaternionDataWake(0);
        mFeatureActiveMask &= ~(INV_DMP_6AXIS_QUATERNION_WAKE);
        LOGV_IF(ENG_VERBOSE, "HAL:LP Quat wake disabled");
    } else {
        if (enableQuaternionDataWake(1) < 0) {
            LOGE("HAL:ERR can't enable LP Quaternion");
        } else {
            mFeatureActiveMask |= (INV_DMP_6AXIS_QUATERNION_WAKE);
            LOGV_IF(ENG_VERBOSE, "HAL:LP Quat wake  enabled");
        }
    }
    return 0;
}

int MPLSensor::enableQuaternionData(int en)
{
    VFUNC_LOG;

    int res = 0;

    // Enable DMP quaternion
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.six_axis_q_on, (long long)getTimestamp());
    if (write_sysfs_int(mpu.six_axis_q_on, en) < 0) {
        LOGE("HAL:ERR can't write DMP three_axis_q__on");
        res = -1;
    }

    if (!en) {
        LOGV_IF(ENG_VERBOSE, "HAL:DMP quaternion data was turned off");
        inv_quaternion_sensor_was_turned_off();
    } else {
        LOGV_IF(ENG_VERBOSE, "HAL:Enabling three axis quat");
    }

    return res;
}

int MPLSensor::enableQuaternionDataWake(int en)
{
    VFUNC_LOG;

    int res = 0;

    // Enable DMP quaternion
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.six_axis_q_wake_on, (long long)getTimestamp());
    if (write_sysfs_int(mpu.six_axis_q_wake_on, en) < 0) {
        LOGE("HAL:ERR can't write DMP three_axis_q__on");
        res = -1;
    }

    if (!en) {
        LOGV_IF(ENG_VERBOSE, "HAL:DMP quaternion data was turned off");
        inv_quaternion_sensor_was_turned_off();
    } else {
        LOGV_IF(ENG_VERBOSE, "HAL:Enabling three axis quat");
    }

    return res;
}

int MPLSensor::setQuaternionRate(int64_t wanted)
{
    VFUNC_LOG;
    int res = 0;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            int(NS_PER_SECOND_FLOAT / wanted), mpu.six_axis_q_rate,
            (long long)getTimestamp());
    res = write_sysfs_int(mpu.six_axis_q_rate, NS_PER_SECOND_FLOAT / wanted);
    res = write_sysfs_int(mpu.nine_axis_q_rate, NS_PER_SECOND_FLOAT / wanted);
    res = write_sysfs_int(mpu.in_geomag_rate, NS_PER_SECOND_FLOAT / wanted);
    LOGV_IF(PROCESS_VERBOSE,
            "HAL:DMP six axis rate %.2f Hz", NS_PER_SECOND_FLOAT / wanted);

    return res;
}

int MPLSensor::getDmpTiltFd()
{
    VFUNC_LOG;

    LOGV_IF(EXTRA_VERBOSE, "getDmpTiltFd returning %d",
            dmp_tilt_fd);
    return dmp_tilt_fd;
}

int MPLSensor::readDmpTiltEvents(sensors_event_t* data, int count)
{
    VFUNC_LOG;

    char dummy[4];
    int tilt;
    FILE *fp;
    int numEventReceived = 0;

    /* Technically this step is not necessary for now  */
    /* In the future, we may have meaningful values */
    fp = fopen(mpu.event_tilt, "r");
    if (fp == NULL) {
        LOGE("HAL:cannot open event_tilt");
        return 0;
    } else {
        if (fscanf(fp, "%d\n", &tilt) < 0) {
            LOGE("HAL:cannot read event_tilt");
        }
        fclose(fp);
    }

    if (mDmpTiltEnabled && count > 0) {
        mTiltUpdate = 1;
        /* Handles return event */
        LOGI("HAL: TILT detected");
        int update = tiltHandler(&mPendingEvents[Tilt]);
        if (update && count > 0) {
            *data++ = mPendingEvents[Tilt];
            count--;
            numEventReceived++;
        }
    }

    // read dummy data per driver's request
    lseek(dmp_tilt_fd, 0, SEEK_SET);
    int ret = read(dmp_tilt_fd, dummy, 4);
    (void)ret;

    return numEventReceived;
}

int MPLSensor::enableDmpTilt(int en)
{
    VFUNC_LOG;

    int res = 0;

    //Enable DMP Tilt Function
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.tilt_on, (long long)getTimestamp());
    if (write_sysfs_int(mpu.tilt_on, en) < 0) {
        LOGE("HAL:ERR can't enable Tilt");
        res = -1;
    }
    if (en) {
        mFeatureActiveMask |= INV_DMP_TILT;
    } else {
        mFeatureActiveMask &= ~INV_DMP_TILT;
    }
    return res;
}

int MPLSensor::getDmpPickupFd()
{
    VFUNC_LOG;

    LOGV_IF(EXTRA_VERBOSE, "getDmpPickupFd returning %d",
            dmp_pickup_fd);
    return dmp_pickup_fd;
}

int MPLSensor::readDmpPickupEvents(sensors_event_t* data, int count)
{
    VFUNC_LOG;

    char dummy[4];
    int pickup;
    FILE *fp;
    int numEventReceived = 0;

    /* Technically this step is not necessary for now  */
    /* In the future, we may have meaningful values */
    fp = fopen(mpu.event_pickup, "r");
    if (fp == NULL) {
        LOGE("HAL:cannot open event_pickup");
        return 0;
    } else {
        if (fscanf(fp, "%d\n", &pickup) < 0) {
            LOGE("HAL:cannot read event_pickup");
        }
        fclose(fp);
    }

    if (mDmpPickupEnabled && count > 0) {
        mPickupUpdate = 1;
        /* Handles return event */
        LOGI("HAL: PICKUP detected");
        int update = pickupHandler(&mPendingEvents[Pickup]);
        if (update && count > 0) {
            *data++ = mPendingEvents[Pickup];
            count--;
            numEventReceived++;
        }
    }

    // read dummy data per driver's request
    lseek(dmp_pickup_fd, 0, SEEK_SET);
    int ret = read(dmp_pickup_fd, dummy, 4);
    (void)ret;

    return numEventReceived;
}

int MPLSensor::enableDmpTap(int en)
{
    VFUNC_LOG;

    int res = 0;

    /* Enable DMP Tap Function */
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.tap_on, (long long)getTimestamp());
    if (write_sysfs_int(mpu.tap_on, en) < 0) {
        LOGE("HAL:ERR can't enable Tap");
        res = -1;
    }
    if (en) {
        mFeatureActiveMask |= INV_DMP_TAP;
    } else {
        mFeatureActiveMask &= ~INV_DMP_TAP;
    }
    return res;
}

int MPLSensor::enableDmpEis(int en)
{
    VFUNC_LOG;

    int res = 0;

    //Enable EIS Function
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.eis_on, (long long)getTimestamp());
    if (write_sysfs_int(mpu.eis_on, en) < 0) {
        LOGE("HAL:ERR can't enable Eis");
        res = -1;
    }

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.eis_data_on, (long long)getTimestamp());
    if (write_sysfs_int(mpu.eis_data_on, en) < 0) {
        LOGE("HAL:ERR can't enable Eis");
        res = -1;
    }

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            225, mpu.eis_rate, (long long)getTimestamp());
    if (write_sysfs_int(mpu.eis_rate, 225) < 0) {
        LOGE("HAL:ERR can't set  Eis rate");
        res = -1;
    }

    return res;
}

int MPLSensor::enableDmpEisAuthentication(int en)
{
    VFUNC_LOG;

    int res = 0;

    //Enable EIS Authentication

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.dmp_init, (long long)getTimestamp());
    if (write_sysfs_int(mpu.dmp_init, en) < 0) {
        LOGE("HAL:ERR can't enable Eis");
        res = -1;
    }

    return res;
}

int MPLSensor::enableDmpPickup(int en)
{
    VFUNC_LOG;

    int res = 0;

    //Toggle Pick up detection
    if (en) {
        LOGV_IF(ENG_VERBOSE, "HAL:Enabling Pickup gesture");
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                1, mpu.pickup_on, (long long)getTimestamp());
        if (write_sysfs_int(mpu.pickup_on, 1) < 0) {
            LOGE("HAL:ERR can't write DMP pickup_on");
            res = -1;
        }
        mFeatureActiveMask |= INV_DMP_PICKUP;
    } else {
        LOGV_IF(ENG_VERBOSE, "HAL:Disabling Pickup gesture");
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                0, mpu.pickup_on, (long long)getTimestamp());
        if (write_sysfs_int(mpu.pickup_on, 0) < 0) {
            LOGE("HAL:ERR write DMP pickup_on");
        }
        mFeatureActiveMask &= ~INV_DMP_PICKUP;
    }

    return res;
}

int MPLSensor::enableDmpStationaryDetect(int en)
{
    VFUNC_LOG;

    int res = 0;

    //Toggle Stationary detect
    if (en) {
        LOGV_IF(ENG_VERBOSE, "HAL:Enabling Stationary detect");
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                1, mpu.stationary_detect_on, (long long)getTimestamp());
        if (write_sysfs_int(mpu.stationary_detect_on, 1) < 0) {
            LOGE("HAL:ERR can't write DMP stationary detect");
            res = -1;
        }
        mFeatureActiveMask |= INV_DMP_STA_DETECT;
    } else {
        LOGV_IF(ENG_VERBOSE, "HAL:Disabling Stationary detect");
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                0, mpu.stationary_detect_on, (long long)getTimestamp());
        if (write_sysfs_int(mpu.stationary_detect_on, 0) < 0) {
            LOGE("HAL:ERR write DMP stationary detect");
        }
        mFeatureActiveMask &= ~INV_DMP_STA_DETECT;
    }

    return res;
}

int MPLSensor::enableDmpMotionDetect(int en)
{
    VFUNC_LOG;

    int res = 0;

    //Toggle Stationary detect
    if (en) {
        LOGV_IF(ENG_VERBOSE, "HAL:Enabling Motion detect");
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                1, mpu.motion_detect_on, (long long)getTimestamp());
        if (write_sysfs_int(mpu.motion_detect_on, 1) < 0) {
            LOGE("HAL:ERR can't write DMP motion detect");
            res = -1;
        }
        mFeatureActiveMask |= INV_DMP_MOT_DETECT;
    } else {
        LOGV_IF(ENG_VERBOSE, "HAL:Disabling Motion detect");
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                0, mpu.motion_detect_on, (long long)getTimestamp());
        if (write_sysfs_int(mpu.motion_detect_on, 0) < 0) {
            LOGE("HAL:ERR write DMP motion detect");
        }
        mFeatureActiveMask &= ~INV_DMP_MOT_DETECT;
    }

    return res;
}

int MPLSensor::enableLPQ(int en)
{
    VFUNC_LOG;

    int res = 0;

    if (en) {
        LOGV_IF(ENG_VERBOSE, "HAL:Enabling 3 axis LPQ");
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                1, mpu.three_axis_q_on, (long long)getTimestamp());

        if (write_sysfs_int(mpu.three_axis_q_on, 1) < 0) {
            LOGE("HAL:ERR can't write DMP LPQ 3 axis on");
            res = -1;
        }
        mFeatureActiveMask |= INV_DMP_LPQ;
    } else {
        LOGV_IF(ENG_VERBOSE, "HAL:Disabling 3 axis LPQ ");
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                0, mpu.three_axis_q_on, (long long)getTimestamp());

        if (write_sysfs_int(mpu.three_axis_q_on, 0) < 0) {
            LOGE("HAL:ERR can't write DMP LPQ 3 axis off");
            res = -1;
        }
        mFeatureActiveMask &= ~INV_DMP_LPQ;
    }

    return res;
}

/* pedometerMode:
 *   0: Step Counter
 *   1: Step Detector
 *   2: Step Counter Wake
 *   3: Step Detector Wake */
int MPLSensor::enableDmpPedometer(int en, int pedometerMode)
{
    VFUNC_LOG;
    int res = 0;

    /* set poll time */
    // new value have to be set to mDmpStepCountEnabled in advance
    if (!mStepCounterIntMode && mDmpStepCountEnabled && !(mCalibrationMode & mpl_pedo))
        mPollTime = STEP_COUNT_POLL_TIME_MS;
    else
        mPollTime = -1;

    /* set sysfs even for non DMP chip to tell driver */

    if (pedometerMode == 1) {
        //Enable DMP Pedometer Step Detect Function
        if (en) {
            mFeatureActiveMask |= INV_DMP_STEP_DETECTOR;
        } else {
            mFeatureActiveMask &= ~INV_DMP_STEP_DETECTOR;
        }
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                en, mpu.pedometer_on, (long long)getTimestamp());
        if (write_sysfs_int(mpu.pedometer_on, en) < 0) {
            LOGE("HAL:ERR can't enable Android Pedometer");
            res = -1;
            return res;
        }
    } else if (pedometerMode == 0) {
        //Enable DMP Pedometer Step Counter Function
        if (en) {
            mFeatureActiveMask |= INV_DMP_STEP_COUNTER;
        } else {
            mFeatureActiveMask &= ~INV_DMP_STEP_COUNTER;
        }
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                en, mpu.pedometer_counter_on, (long long)getTimestamp());
        if (write_sysfs_int(mpu.pedometer_counter_on, en) < 0) {
            LOGE("HAL:ERR can't enable Android Pedometer");
            res = -1;
            return res;
        }
    } else if (pedometerMode == 2) {
        //Enable DMP Pedometer Step Counter Function
        if (en) {
            mFeatureActiveMask |= INV_DMP_STEP_COUNTER_WAKE;
        } else {
            mFeatureActiveMask &= ~INV_DMP_STEP_COUNTER_WAKE;
        }
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                en, mpu.pedometer_counter_wake_on, (long long)getTimestamp());
        if (write_sysfs_int(mpu.pedometer_counter_wake_on, en) < 0) {
            LOGE("HAL:ERR can't enable Android Pedometer");
            res = -1;
            return res;
        }
    } else {
        //Enable DMP Pedometer Step Detect Function
        if (en) {
            mFeatureActiveMask |= INV_DMP_STEP_DETECTOR_WAKE;
        } else {
            mFeatureActiveMask &= ~INV_DMP_STEP_DETECTOR_WAKE;
        }
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                en, mpu.pedometer_wake_on, (long long)getTimestamp());
        if (write_sysfs_int(mpu.pedometer_wake_on, en) < 0) {
            LOGE("HAL:ERR can't enable Android Pedometer");
            res = -1;
            return res;
        }
    }

    if (!mBatchEnabled && (resetDataRates() < 0))
        return res;

    return res;
}

int MPLSensor::trigDmpPedometerCountRead(void)
{
    VFUNC_LOG;
    int64_t now;
    int res = 0;

    /* only for DMP pedometer */
    if (mCalibrationMode & mpl_pedo)
        return 0;

    /* check int or poll mode */
    if (mStepCounterIntMode)
        return 0;

    /* check if step counter is enabled */
    if (!mDmpStepCountEnabled)
        return 0;

    /* check timestamp not to access DMP too frequently */
    now = getTimestamp();
    if (now < (mLastStepCountReadTrigTimestamp + (int64_t)STEP_COUNT_POLL_TIME_MS * 1000000 * 9 / 10)) {
        LOGV_IF(EXTRA_VERBOSE,
                "HAL:skip step count trigger (elapsed time since last trigger %lld ms)",
                (long long)(now - mLastStepCountReadTrigTimestamp) / 1000000LL);
        return 0;
    }
    mLastStepCountReadTrigTimestamp = now;
    LOGV_IF(EXTRA_VERBOSE, "HAL:trigger step count read");

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            1, mpu.pedometer_counter_send, (long long)getTimestamp());
    if (write_sysfs_int(mpu.pedometer_counter_send, 1) < 0) {
        LOGE("HAL:ERR can't access %s", mpu.pedometer_counter_send);
        res = -1;
    }

    return res;
}

int MPLSensor::enableGyroEngine(int en)
{
    VFUNC_LOG;

    int res = 0;
    (void) en;

#if DEBUG_DRIVER
    /* need to also turn on/off the master enable */
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.gyro_enable, (long long)getTimestamp());
    write_sysfs_int(mpu.gyro_enable, en);
#endif
    return res;
}

int MPLSensor::enableRawGyro(int en)
{
    VFUNC_LOG;

    int res = 0;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.gyro_fifo_enable, (long long)getTimestamp());
    res += write_sysfs_int(mpu.gyro_fifo_enable, en);

    adapter_gyro_reset_timestamp();

    if (!en) {
        LOGV_IF(EXTRA_VERBOSE, "HAL:MPL:inv_gyro_was_turned_off");
        inv_gyro_was_turned_off();
    }

    return res;
}

int MPLSensor::enableRawGyroWake(int en)
{
    VFUNC_LOG;

    int res = 0;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.gyro_wake_fifo_enable, (long long)getTimestamp());
    res += write_sysfs_int(mpu.gyro_wake_fifo_enable, en);

    adapter_gyro_reset_timestamp();

    if (!en) {
        LOGV_IF(EXTRA_VERBOSE, "HAL:MPL:inv_gyro_was_turned_off");
        inv_gyro_was_turned_off();
    }

    return res;
}

int MPLSensor::enableGyro(int en)
{
    VFUNC_LOG;

    int res = 0;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.calib_gyro_enable, (long long)getTimestamp());
    res = write_sysfs_int(mpu.calib_gyro_enable, en);

    adapter_gyro_reset_timestamp();

    if (!en) {
        LOGV_IF(EXTRA_VERBOSE, "HAL:MPL:inv_gyro_was_turned_off");
        inv_gyro_was_turned_off();
    }

    return res;
}

int MPLSensor::enableGyroWake(int en)
{
    VFUNC_LOG;

    int res = 0;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.calib_gyro_wake_enable, (long long)getTimestamp());
    res = write_sysfs_int(mpu.calib_gyro_wake_enable, en);

    adapter_gyro_reset_timestamp();

    if (!en) {
        LOGV_IF(EXTRA_VERBOSE, "HAL:MPL:inv_gyro_was_turned_off");
        inv_gyro_was_turned_off();
    }

    return res;
}

int MPLSensor::enableAccel(int en)
{
    VFUNC_LOG;

    int res = 0;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.accel_fifo_enable, (long long)getTimestamp());
    res += write_sysfs_int(mpu.accel_fifo_enable, en);

    adapter_accel_reset_timestamp();

    if (!en) {
        LOGV_IF(EXTRA_VERBOSE, "HAL:MPL:inv_accel_was_turned_off");
        inv_accel_was_turned_off();
    }

    return res;
}

int MPLSensor::enableAccelWake(int en)
{
    VFUNC_LOG;

    int res = 0;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.accel_wake_fifo_enable, (long long)getTimestamp());
    res += write_sysfs_int(mpu.accel_wake_fifo_enable, en);

    adapter_accel_reset_timestamp();

    if (!en) {
        LOGV_IF(EXTRA_VERBOSE, "HAL:MPL:inv_accel_was_turned_off");
        inv_accel_was_turned_off();
    }

    return res;
}

int MPLSensor::enableRawCompass(int en)
{
    VFUNC_LOG;

    int res = 0;

    if (mCompassSensor->isCompassSensorPresent()) {
        /* TODO: handle ID_RM if third party compass cal is used */
        res = mCompassSensor->enable(ID_RM, en);

        adapter_compass_reset_timestamp();

        if (en == 0 || res != 0) {
            LOGV_IF(EXTRA_VERBOSE, "HAL:MPL:inv_compass_was_turned_off %d", res);
            inv_compass_was_turned_off();
        }
    }

    return res;
}

int MPLSensor::enableRawCompassWake(int en)
{
    VFUNC_LOG;

    int res = 0;

    if (mCompassSensor->isCompassSensorPresent()) {
        /* TODO: handle ID_RM if third party compass cal is used */
        res = mCompassSensor->enable(ID_RMW, en);

        adapter_compass_reset_timestamp();

        if (en == 0 || res != 0) {
            LOGV_IF(EXTRA_VERBOSE, "HAL:MPL:inv_compass_was_turned_off %d", res);
            inv_compass_was_turned_off();
        }
    }

    return res;
}

int MPLSensor::enableCompass(int en)
{
    VFUNC_LOG;

    int res = 0;

    if (mCompassSensor->isCompassSensorPresent()) {
        /* TODO: handle ID_RM if third party compass cal is used */
        res = mCompassSensor->enable(ID_M, en);

        adapter_compass_reset_timestamp();

        if (en == 0 || res != 0) {
            LOGV_IF(EXTRA_VERBOSE, "HAL:MPL:inv_compass_was_turned_off %d", res);
            inv_compass_was_turned_off();
        }
    }

    return res;
}

int MPLSensor::enableCompassWake(int en)
{
    VFUNC_LOG;

    int res = 0;

    if (mCompassSensor->isCompassSensorPresent()) {
        /* TODO: handle ID_RM if third party compass cal is used */
        res = mCompassSensor->enable(ID_MW, en);

        adapter_compass_reset_timestamp();

        if (en == 0 || res != 0) {
            LOGV_IF(EXTRA_VERBOSE, "HAL:MPL:inv_compass_was_turned_off %d", res);
            inv_compass_was_turned_off();
        }
    }

    return res;
}

int MPLSensor::enablePressure(int en)
{
    VFUNC_LOG;

    int res = 0;

    if (mPressureSensorPresent)
        res = mPressureSensor->enable(ID_PS, en);

    return res;
}

int MPLSensor::enablePressureWake(int en)
{
    VFUNC_LOG;

    int res = 0;

    if (mPressureSensorPresent)
        res = mPressureSensor->enable(ID_PSW, en);

    return res;
}

int MPLSensor::enableLight(int en)
{
    VFUNC_LOG;

    int res = 0;

    if (mLightSensor)
        res = mLightSensor->enable(ID_L, en);

    return res;
}

int MPLSensor::enableLightWake(int en)
{
    VFUNC_LOG;

    int res = 0;

    if (mLightSensor)
        res = mLightSensor->enable(ID_LW, en);

    return res;
}

int MPLSensor::enableProximity(int en)
{
    VFUNC_LOG;

    int res = 0;

    if (mLightSensor)
        res = mLightSensor->enable(ID_PR, en);

    return res;
}

int MPLSensor::enableProximityWake(int en)
{
    VFUNC_LOG;

    int res = 0;

    if (mLightSensor)
        res = mLightSensor->enable(ID_PRW, en);

    return res;
}

/* use this function for initialization */
int MPLSensor::enableBatch(int64_t timeout)
{
    VFUNC_LOG;

    int res = 0;

    res = write_sysfs_int(mpu.batchmode_timeout, timeout);
    if (timeout == 0) {
        res = write_sysfs_int(mpu.six_axis_q_on, 0);
        res = write_sysfs_int(mpu.nine_axis_q_on, 0);
    }

    if (timeout == 0) {
        LOGV_IF(EXTRA_VERBOSE, "HAL:MPL:batchmode timeout is zero");
    }

    return res;
}

int MPLSensor::enableDmpCalibration(int en)
{
    VFUNC_LOG;

    int res = 0;
    int fd = 0;

    /* enable gyro calibration */
    fd = open(mpu.gyro_cal_enable, O_RDWR);
    if (fd < 0) {
        LOGE("HAL:ERR couldn't open gyro_cal_enable node");
    } else {
        LOGV_IF(ENG_VERBOSE,
                "HAL:gyro_cal_enable opened : %d", fd);
        if (write_attribute_sensor(fd, en) < 0) {
            LOGE("HAL:Error writing to gyro_cal_enable");
            return 1;
        }
        LOGV_IF(ENG_VERBOSE, "HAL:Invensense DMP gyro cal enabled");
    }

    /* enable accel calibration */
    fd = open(mpu.accel_cal_enable, O_RDWR);
    if (fd < 0) {
        LOGE("HAL:ERR couldn't open accel_cal_enable node");
    } else {
        LOGV_IF(ENG_VERBOSE,
                "HAL:accel_cal_enable opened : %d", fd);
        if (write_attribute_sensor(fd, en) < 0) {
            LOGE("HAL:Error writing to accel_cal_enable");
            return 1;
        }
        LOGV_IF(ENG_VERBOSE, "HAL:Invensense DMP accel cal enabled");
    }

    /* enable compass calibration */
    fd = open(mpu.compass_cal_enable, O_RDWR);
    if (fd < 0) {
        LOGE("HAL:ERR couldn't open compass_cal_enable node");
    } else {
        LOGV_IF(ENG_VERBOSE,
                "HAL:compass_cal_enable opened : %d", fd);
        if (write_attribute_sensor(fd, en) < 0) {
            LOGE("HAL:Error writing to compass_cal_enable");
            return 1;
        }
        LOGV_IF(ENG_VERBOSE, "HAL:Invensense DMP compass cal enabled");
    }
    return res;
}

int MPLSensor::enableOisSensor(int en)
{
    VFUNC_LOG;
    int res = 0;
    int fd = 0;

    /* enable Ois sensor */
    fd = open(mpu.ois_enable, O_RDWR);
    if (fd < 0) {
        LOGE("HAL:ERR couldn't open ois_enable node");
    } else {
        LOGV_IF(ENG_VERBOSE,
                "HAL:ois_enable opened : %d", fd);
        if (write_attribute_sensor(fd, en) < 0) {
            LOGE("HAL:Error writing to ois_enable");
            return 1;
        }
        LOGV_IF(ENG_VERBOSE, "HAL:OIS is %s", en ? "enabled": "disabled");
    }

    return res;
}

int MPLSensor::enable(int32_t handle, int en)
{
    VFUNC_LOG;

#ifdef DIRECT_REPORT
    const struct sensor_t *sensor;
    int32_t what = -1;

    getHandle(handle, what, sensor);
    if (what < 0) {
        LOGV_IF(ENG_VERBOSE, "HAL:can't find handle %d", handle);
        return -EINVAL;
    }

    if (!en) {
        pthread_mutex_lock(&mHALMutex);
        SensorChannelMux &mux = mSensorChannelMuxes[handle];
        const SensorChannelMux::SensorParams params = mux.getParams();
        mux.disableChannel(0);
        bool is_enabled = mux.getEnable();
        const SensorChannelMux::SensorParams new_params = mux.getParams();
        pthread_mutex_unlock(&mHALMutex);
        if (is_enabled) {
            if (params.periodNs != new_params.periodNs || params.latencyNs != new_params.latencyNs) {
                return doBatch(handle, 0, new_params.periodNs, new_params.latencyNs);
            }
            return 0;
        }
    } else {
        pthread_mutex_lock(&mHALMutex);
        mPollChannelTiming[handle].dataTimestamp = 0;
        mPollChannelTiming[handle].dataPeriod = 0;
        mPollChannelTiming[handle].lastTimestamp = 0;
        pthread_mutex_unlock(&mHALMutex);
    }
#endif

    return doEnable(handle, en);
}

int MPLSensor::doEnable(int32_t handle, int en)
{
    VFUNC_LOG;

    const struct sensor_t *sensor;
    int32_t what = -1;
    const char *name;
    int err = 0;
    int batchMode = 0;

    if (!mChipDetected)
        return -EINVAL;

    if (mRetryCalFileLoad)
        loadCalFile();
    mRetryCalFileLoad = false; // no more retry

    getHandle(handle, what, sensor);
    if (what < 0) {
        LOGV_IF(ENG_VERBOSE, "HAL:can't find handle %d", handle);
        return -EINVAL;
    }
    name = sensor->name;

    if (!en)
        mBatchEnabled &= ~(1LL << handle);

    LOGV_IF(ENG_VERBOSE, "HAL:handle = %d en = %d", handle, en);

    if (en) {
        pthread_mutex_lock(&mHALMutex);
        mAdditionalInfoEnabledVector.push_back(handle);
        pthread_mutex_unlock(&mHALMutex);
    }

    switch (handle) {
        case ID_EISGY:
            {
            /* get the current bias */
            bool old_gyro_lp = mGyroLpMode;
            bool old_accel_lp = mAccelLpMode;
            getDmpGyroBias(false);
            getDmpAccelBias(false);

            enableDmpEis(en);

            /* set bias to algo according to new power mode */
            bool new_gyro_lp, new_accel_lp;
            getCurrentPowerMode(&new_gyro_lp, &new_accel_lp);
            mGyroLpMode = new_gyro_lp;
            if (new_gyro_lp != old_gyro_lp) {
                setDmpGyroBias(false);
            }
            mAccelLpMode = new_accel_lp;
            if (new_accel_lp != old_accel_lp) {
                setDmpAccelBias(false);
            }
            }
            break;
        case ID_EISAUTHENTICATION:
            enableDmpEisAuthentication(en);
            break;
        case ID_LPQ:
            enableLPQ(en);
            break;
        case ID_PICK:
            LOGV_IF(PROCESS_VERBOSE, "HAL:enable - sensor %s (handle %d) %s -> %s",
                    name, handle,
                    (mDmpPickupEnabled? "en": "dis"),
                    (en? "en" : "dis"));
            enableDmpPickup(en);
            mDmpPickupEnabled = !!en;
            break;
        case ID_T:
            LOGV_IF(PROCESS_VERBOSE, "HAL:enable - sensor %s (handle %d) %s -> %s",
                    name, handle,
                    (mDmpTiltEnabled? "en": "dis"),
                    (en? "en" : "dis"));
            enableDmpTilt(en);
            mDmpTiltEnabled = !!en;
            break;
        case ID_TAP:
            LOGV_IF(PROCESS_VERBOSE, "HAL:enable - sensor %s (handle %d) %s -> %s",
                    name, handle,
                    (mDmpTapEnabled? "en": "dis"),
                    (en? "en" : "dis"));
            enableDmpTap(en);
            mDmpTapEnabled = !!en;
            break;
        case ID_STADET:
            LOGV_IF(PROCESS_VERBOSE, "HAL:enable - sensor %s (handle %d) %s -> %s",
                    name, handle,
                    (mDmpStationaryDetEnabled? "en": "dis"),
                    (en? "en" : "dis"));
            enableDmpStationaryDetect(en);
            mDmpStationaryDetEnabled = !!en;
            break;
        case ID_MOTDET:
            LOGV_IF(PROCESS_VERBOSE, "HAL:enable - sensor %s (handle %d) %s -> %s",
                    name, handle,
                    (mDmpMotionDetEnabled? "en": "dis"),
                    (en? "en" : "dis"));
            enableDmpMotionDetect(en);
            mDmpMotionDetEnabled = !!en;
            break;
        case ID_SC:
            LOGV_IF(PROCESS_VERBOSE, "HAL:enable - sensor %s (handle %d) %s -> %s",
                    name, handle,
                    (mDmpStepCountEnabled? "en": "dis"),
                    (en? "en" : "dis"));
            mDmpStepCountEnabled = !!en;
            mEnabled &= ~(1LL << what);
            mEnabled |= (uint64_t(en) << what);
            mEnabledCached = mEnabled;
            if (!(mCalibrationMode & mpl_pedo)) {
                /* save timestamp before control DMP not to miss
                 * the initial step event */
                if (en)
                    mEnabledTime[StepCounter] = getTimestamp();
                else {
                    mEnabledTime[StepCounter] = 0;
                    mStepCounterHandlerNotCalled = true;
                }
            }
            enableDmpPedometer(en, 0);
            batchMode = computeBatchSensorMask(mEnabled, mBatchEnabled);
            /* skip setBatch if there is no need to */
            if (((int)mOldBatchEnabledMask != batchMode) || batchMode) {
                setBatch(batchMode);
            }
            mOldBatchEnabledMask = batchMode;
            if (mCalibrationMode & mpl_pedo) {
                /* save timestamp after library API call */
                if (en)
                    mEnabledTime[StepCounter] = getTimestamp();
                else {
                    mEnabledTime[StepCounter] = 0;
                    mStepCounterHandlerNotCalled = true;
                }
            }
            return 0;
        case ID_P:
            LOGV_IF(PROCESS_VERBOSE, "HAL:enable - sensor %s (handle %d) %s -> %s",
                    name, handle,
                    (mDmpPedometerEnabled? "en": "dis"),
                    (en? "en" : "dis"));
            mDmpPedometerEnabled = !!en;
            mEnabled &= ~(1LL << what);
            mEnabled |= (uint64_t(en) << what);
            mEnabledCached = mEnabled;
            enableDmpPedometer(en, 1);
            batchMode = computeBatchSensorMask(mEnabled, mBatchEnabled);
            /* skip setBatch if there is no need to */
            if (((int)mOldBatchEnabledMask != batchMode) || batchMode) {
                setBatch(batchMode);
            }
            mOldBatchEnabledMask = batchMode;
            if (en)
                mEnabledTime[StepDetector] = getTimestamp();
            else
                mEnabledTime[StepDetector] = 0;
            return 0;
        case ID_SM:
            LOGV_IF(PROCESS_VERBOSE, "HAL:enable - sensor %s (handle %d) %s -> %s",
                    name, handle,
                    (mDmpSignificantMotionEnabled? "en": "dis"),
                    (en? "en" : "dis"));
            enableDmpSignificantMotion(en);
            mDmpSignificantMotionEnabled = !!en;
            if (en)
                mEnabledTime[SignificantMotion] = getTimestamp();
            else
                mEnabledTime[SignificantMotion] = 0;
            return 0;
        case ID_SCW:
            LOGV_IF(PROCESS_VERBOSE, "HAL:enable - sensor %s (handle %d) %s -> %s",
                    name, handle,
                    (mDmpStepCountEnabled? "en": "dis"),
                    (en? "en" : "dis"));
            mDmpStepCountEnabled = !!en;
            mEnabled &= ~(1LL << what);
            mEnabled |= (uint64_t(en) << what);
            mEnabledCached = mEnabled;
            if (!(mCalibrationMode & mpl_pedo)) {
                /* save timestamp before control DMP not to miss
                 * the initial step event */
                if (en)
                    mEnabledTime[StepCounter_Wake] = getTimestamp();
                else {
                    mEnabledTime[StepCounter_Wake] = 0;
                    mStepCounterHandlerNotCalledWake = true;
                }
            }
            enableDmpPedometer(en, 2);
            batchMode = computeBatchSensorMask(mEnabled, mBatchEnabled);
            /* skip setBatch if there is no need to */
            if (((int)mOldBatchEnabledMask != batchMode) || batchMode) {
                setBatch(batchMode);
            }
            mOldBatchEnabledMask = batchMode;
            if (mCalibrationMode & mpl_pedo) {
                /* save timestamp after library API call */
                if (en)
                    mEnabledTime[StepCounter_Wake] = getTimestamp();
                else {
                    mEnabledTime[StepCounter_Wake] = 0;
                    mStepCounterHandlerNotCalledWake = true;
                }
            }
            return 0;
        case ID_PW:
            LOGV_IF(PROCESS_VERBOSE, "HAL:enable - sensor %s (handle %d) %s -> %s",
                    name, handle,
                    (mDmpPedometerEnabled? "en": "dis"),
                    (en? "en" : "dis"));
            mDmpPedometerEnabled = !!en;
            mEnabled &= ~(1LL << what);
            mEnabled |= (uint64_t(en) << what);
            mEnabledCached = mEnabled;
            enableDmpPedometer(en, 3);
            batchMode = computeBatchSensorMask(mEnabled, mBatchEnabled);
            /* skip setBatch if there is no need to */
            if (((int)mOldBatchEnabledMask != batchMode) || batchMode) {
                setBatch(batchMode);
            }
            mOldBatchEnabledMask = batchMode;
            if (en)
                mEnabledTime[StepDetector_Wake] = getTimestamp();
            else
                mEnabledTime[StepDetector_Wake] = 0;
            return 0;
        case ID_L:
            mLightInitEvent = (en) ? true : false;
            break;
        case ID_PR:
            mProxiInitEvent = (en) ? true : false;
            break;
        case ID_LW:
            mLightWakeInitEvent = (en) ? true : false;
            break;
        case ID_PRW:
            mProxiWakeInitEvent = (en) ? true : false;
            break;
        case ID_ARC:
            what = AccelerometerRaw;
            name = "Raw Accelerometer";
            break;
        case ID_GRC:
            what = GyroRaw;
            name = "Raw Gyroscope";
            break;
        case ID_MRC:
            what = MagneticFieldRaw;
            name = "Raw Magneric Field";
            break;
        case ID_OIS:
            what = OisSensor;
            name = "Ois Sensor";
            LOGV_IF(PROCESS_VERBOSE, "HAL:enable - sensor %s (handle %d) %s -> %s",
                name, handle,
                (mOisEnabled? "en": "dis"),
                (en? "en" : "dis"));
            {
            /* get the current bias */
            bool old_gyro_lp = mGyroLpMode;
            bool old_accel_lp = mAccelLpMode;
            getDmpGyroBias(false);
            getDmpAccelBias(false);

            enableOisSensor(en);
            mOisEnabled = !!en;

            /* set bias to algo according to new power mode */
            bool new_gyro_lp, new_accel_lp;
            getCurrentPowerMode(&new_gyro_lp, &new_accel_lp);
            mGyroLpMode = new_gyro_lp;
            if (new_gyro_lp != old_gyro_lp) {
                setDmpGyroBias(false);
            }
            mAccelLpMode = new_accel_lp;
            if (new_accel_lp != old_accel_lp) {
                setDmpAccelBias(false);
            }
            }
            return 0;
        default:
            break;
    }

    uint64_t newState = en ? 1 : 0;
    uint64_t sen_mask;

    LOGV_IF(PROCESS_VERBOSE, "HAL:enable - sensor %s (handle %d) %s -> %s",
            name, handle,
            ((mEnabled & (1LL << what)) ? "en" : "dis"),
            (((newState) << what) ? "en" : "dis"));
    LOGV_IF(ENG_VERBOSE,
            "HAL:%s sensor state change what=%d", name, what);

    if (((newState) << what) != (mEnabled & (1LL << what))) {
        uint64_t flags = newState;
        uint64_t lastEnabled = mEnabled, changed = 0;

        /* save the old mEnabled to be used by readEvents() until
         * hardware control is done */
        if (en)
            mEnabledCached = mEnabled;

        mEnabled &= ~(1LL << what);
        mEnabled |= (uint64_t(flags) << what);

        /* update mEnabledCached here when disable to stop calling
         * data handlers at readEvents() */
        if (!en)
            mEnabledCached = mEnabled;

        LOGV_IF(ENG_VERBOSE, "HAL:flags = 0x%0llx", (unsigned long long)flags);
        changed = mLocalSensorMask;
        computeLocalSensorMask(mEnabled); // will update mLocalSensorMask
        LOGV_IF(ENG_VERBOSE, "HAL:enable : mEnabled = 0x%llx lastEnabled = 0x%llx", (unsigned long long)mEnabled, (unsigned long long)lastEnabled);
        LOGV_IF(ENG_VERBOSE, "HAL:enable : local mask = 0x%llx master mask = 0x%llx", (unsigned long long)mLocalSensorMask, (unsigned long long)mMasterSensorMask);
        sen_mask = mLocalSensorMask & mMasterSensorMask;
        LOGV_IF(ENG_VERBOSE, "HAL:sen_mask= 0x%0llx beforechanged=0x%0llx", (unsigned long long)sen_mask, (unsigned long long)changed);
        changed ^= mLocalSensorMask;
        LOGV_IF(ENG_VERBOSE, "HAL:changed = 0x%0llx", (unsigned long long)changed);
        enableSensors(sen_mask, flags, changed);

        /* save the current time to filter old data */
        if (en)
            mEnabledTime[what] = getTimestamp();
        else
            mEnabledTime[what] = 0;

        /* update mEnabledCached here when enable, after hardware control */
        if (en)
            mEnabledCached = mEnabled;
    }

    return err;
}

void MPLSensor::computeLocalSensorMask(uint64_t enabled_sensors)
{
    VFUNC_LOG;
    int i;

    mLocalSensorMask = 0;
    for (i = 0; i < ID_NUMBER; i++) {
        if (enabled_sensors & (1LL << i)) {
            mLocalSensorMask |= mCurrentSensorMask[i].sensorMask;
        }
    }
    LOGV_IF(ENG_VERBOSE, "mpl calibration final mLocalSensorMask=0x0%llx, enabledsensor=0x0%llx",
            (unsigned long long)mLocalSensorMask, (unsigned long long)enabled_sensors);
}

int MPLSensor::enableSensors(uint64_t sensors, int en, uint64_t changed)
{
    VFUNC_LOG;

    inv_error_t res = -1;
    struct sysfs_mask *localSensorMask;
    uint32_t i;

    /* save the latest bias to file system when disabling any AGM sensor */
    if (!en) {
        if (changed & (INV_THREE_AXIS_GYRO | INV_THREE_AXIS_GYRO_WAKE |
                       INV_THREE_AXIS_RAW_GYRO | INV_THREE_AXIS_RAW_GYRO_WAKE |
                       INV_THREE_AXIS_ACCEL | INV_THREE_AXIS_ACCEL_WAKE |
                       INV_THREE_AXIS_COMPASS | INV_THREE_AXIS_COMPASS_WAKE |
                       INV_THREE_AXIS_RAW_COMPASS | INV_THREE_AXIS_RAW_COMPASS_WAKE |
                       VIRTUAL_SENSOR_GYRO_6AXES_MASK |
                       VIRTUAL_SENSOR_GYRO_6AXES_MASK_WAKE |
                       VIRTUAL_SENSOR_MAG_6AXES_MASK |
                       VIRTUAL_SENSOR_MAG_6AXES_MASK_WAKE |
                       VIRTUAL_SENSOR_9AXES_MASK |
                       VIRTUAL_SENSOR_9AXES_MASK_WAKE |
                       VIRTUAL_SENSOR_LPQ_MASK)) {
            getDmpGyroBias(true);
            getDmpAccelBias(true);
            getDmpCompassBias();
            inv_store_calibration();
            LOGV_IF(PROCESS_VERBOSE, "HAL:Cal file updated");
        }
    }

    /* get the current bias */
    bool old_gyro_lp = mGyroLpMode;
    bool old_accel_lp = mAccelLpMode;
    getDmpGyroBias(false);
    getDmpAccelBias(false);

    LOGV_IF(ENG_VERBOSE, "HAL:enableSensors - sensors: 0x%0llx changed: 0x%0llx",
        (unsigned long long)sensors, (unsigned long long)changed);
    for (i = 0; i < mNumSysfs; i++) {
        localSensorMask = &mSysfsMask[i];
        if (localSensorMask->sensorMask && (changed & localSensorMask->sensorMask) && localSensorMask->enable) {
            (this->*(localSensorMask->enable))(!!(sensors & localSensorMask->sensorMask));
        }
    }
    for (i = 0; i < mNumSysfs; i++) {
        localSensorMask = &mSysfsMask[i];
        if (localSensorMask->sensorMask && localSensorMask->enable) {
            localSensorMask->en = (!!(sensors & localSensorMask->sensorMask));
        }
    }

    /* to batch or not to batch */
    int batchMode = computeBatchSensorMask(mEnabled, mBatchEnabled);

    /* skip setBatch if there is no need to */
    if (((int)mOldBatchEnabledMask != batchMode) || batchMode || changed) {
        setBatch(batchMode);
    }
    mOldBatchEnabledMask = batchMode;

    if (!batchMode && (resetDataRates() < 0)) {
        LOGE("HAL:ERR can't reset output rate back to original setting");
    }

    /* set bias to algo according to new power mode */
    bool new_gyro_lp, new_accel_lp;
    getCurrentPowerMode(&new_gyro_lp, &new_accel_lp);
    mGyroLpMode = new_gyro_lp;
    if (new_gyro_lp != old_gyro_lp) {
        setDmpGyroBias(false);
    }
    mAccelLpMode = new_accel_lp;
    if (new_accel_lp != old_accel_lp) {
        setDmpAccelBias(false);
    }

    return res;
}

/* check if batch mode should be turned on or not */
int MPLSensor::computeBatchSensorMask(uint64_t enableSensors, uint64_t tempBatchSensor)
{
    VFUNC_LOG;

    uint32_t i;

    mFeatureActiveMask &= ~INV_DMP_BATCH_MODE;

    LOGV_IF(ENG_VERBOSE,
            "HAL:computeBatchSensorMask: enableSensors=0x0%llx tempBatchSensor=0x0%llx",
            (unsigned long long)enableSensors, (unsigned long long)tempBatchSensor);

    /* handle initialization and gesture sensor only case */
    if (enableSensors == 0 || tempBatchSensor == 0)
        return 0;

    /* if any continuous sensor is enabled, disable batch mode */
    for (i = 0; i < ID_NUMBER; i++) {
        if ((enableSensors & (1LL << i)) && !(tempBatchSensor & (1LL << i))) {
            if (INV_ALL_CONTINUOUS_SENSORS & (1LL << i)) {
                LOGV_IF(ENG_VERBOSE, "HAL:computeBatchSensorMask: "
                        "sensor %d enabled on continuous mode", i);
                return 0;
            }
        }
    }

    /* step detector could be batched if DMP supports */
    if ((enableSensors & (1LL << StepDetector)) && !(tempBatchSensor & (1LL << StepDetector))) {
        LOGV_IF(ENG_VERBOSE, "HAL:computeBatchSensorMask: "
                "batch is disabled because of step detector");
        return 0;
    }
    if ((enableSensors & (1LL << StepDetector_Wake)) && !(tempBatchSensor & (1LL << StepDetector_Wake))) {
        LOGV_IF(ENG_VERBOSE, "HAL:computeBatchSensorMask: "
                "batch is disabled because of step detector wake");
        return 0;
    }

    /* if enabled gesture is supported by HAL, disable batch */
    if (mCalibrationMode & mpl_pedo && (mDmpStepCountEnabled || mDmpPedometerEnabled)) {
        LOGV_IF(ENG_VERBOSE, "HAL:computeBatchSensorMask: "
                "batch is disabled as step counter, detector is supported by HAL");
        return 0;
    }
    if (mCalibrationMode & mpl_smd && mDmpSignificantMotionEnabled) {
        LOGV_IF(ENG_VERBOSE, "HAL:computeBatchSensorMask: "
                "batch is disabled as smd is supported by HAL");
        return 0;
    }
    if (mCalibrationMode & mpl_pickup && mDmpPickupEnabled) {
        LOGV_IF(ENG_VERBOSE, "HAL:computeBatchSensorMask: "
                "batch is disabled as pickup is supported by HAL");
        return 0;
    }
    if (mCalibrationMode & mpl_tilt && mDmpTiltEnabled) {
        LOGV_IF(ENG_VERBOSE, "HAL:computeBatchSensorMask: "
                "batch is disabled as tilt is supported by HAL");
        return 0;
    }
    if (mCalibrationMode & mpl_md && (mDmpStationaryDetEnabled || mDmpMotionDetEnabled)) {
        LOGV_IF(ENG_VERBOSE, "HAL:computeBatchSensorMask: "
                "batch is disabled as motion detect is supported by HAL");
        return 0;
    }
    if (mCalibrationMode & mpl_tap && mDmpTapEnabled) {
        LOGV_IF(ENG_VERBOSE, "HAL:computeBatchSensorMask: "
                "batch is disabled as tap is supported by HAL");
        return 0;
    }

    /* gesture sensors to disable batch wherever it is supported */
    if (mDmpPickupEnabled || mDmpTiltEnabled) {
        LOGV_IF(ENG_VERBOSE, "HAL:computeBatchSensorMask: "
                "batch is disabled because of pickup or tilt");
        return 0;
    }

    mFeatureActiveMask |= INV_DMP_BATCH_MODE;

    return 1;
}

/* This function is called by enable() */
int MPLSensor::setBatch(int en)
{
    VFUNC_LOG;

    int res = 0;

    writeBatchTimeout(en);

    if (en) {
        if (setBatchDataRates() < 0)
            LOGE("HAL:ERR can't set batch data rates");
    } else {
        if (resetDataRates() < 0)
            LOGE("HAL:ERR can't reset output rate back to original setting");
    }

    return res;
}

int MPLSensor::writeBatchTimeout(int en)
{
    VFUNC_LOG;

    int64_t timeoutInMs = 0;

    LOGV_IF(ENG_VERBOSE,
            "HAL:writeBatchTimeout: "
            "mFeatureActiveMask=0x%016llx, mEnabled=0x%016llx, mBatchEnabled=0x%016llx",
            (unsigned long long)mFeatureActiveMask, (unsigned long long)mEnabled, (unsigned long long)mBatchEnabled);

    if (en) {
        /* take the minimum batchmode timeout */
        int64_t timeout = 100000000000LL;
        int64_t ns = 0;
        for (uint32_t i = 0; i < ID_NUMBER; i++) {
            if ((mEnabled & (1LL << i)) && (mBatchEnabled & (1LL << i))) {
                LOGV_IF(ENG_VERBOSE, "sensor=%d, timeout=%lld", i, (long long)mBatchTimeouts[i]);
                ns = mBatchTimeouts[i];
                timeout = (ns < timeout) ? ns : timeout;
            }
        }
        /* Convert ns to millisecond */
        timeoutInMs = timeout / 1000000;
    } else {
        timeoutInMs = 0;
    }

    LOGV_IF(PROCESS_VERBOSE,
            "HAL: batch timeout set to %lld ms", (long long)timeoutInMs);

    if (mBatchTimeoutInMs != timeoutInMs) {
        /* write required timeout to sysfs */
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %lld > %s (%lld)",
                (long long)timeoutInMs, mpu.batchmode_timeout, (long long)getTimestamp());
        if (write_sysfs_int(mpu.batchmode_timeout, timeoutInMs) < 0) {
            LOGE("HAL:ERR can't write batchmode_timeout");
        }
    }
    /* remember last timeout value */
    mBatchTimeoutInMs = timeoutInMs;

    return 0;
}

int MPLSensor::output_control(sensors_event_t* s, get_sensor_data_func func,
        int sensor_value_size, float *sensor_values, int8_t *sensor_accuracy,
        int sensor_id)
{
    float values[6];
    int8_t accuracy;
    int64_t timestamp = -1;
    static int cnt0;
    int update=0;
    int checkBatchEnable;
    int64_t *sensor_timestamp, sensor_time, *timestamp0, Rate0;

    checkBatchEnable = ((mFeatureActiveMask & INV_DMP_BATCH_MODE)? 1:0);
    sensor_timestamp = &s->timestamp;
    sensor_time = mEnabledTime[sensor_id];
    timestamp0 = &mCurrentSensorMask[sensor_id].timestamp;
    Rate0 = invCheckOutput(sensor_id);

    if (checkBatchEnable) {
        update = func(sensor_values, sensor_accuracy, sensor_timestamp);
        if (!sensor_time || !(s->timestamp > sensor_time) || (*timestamp0 == *sensor_timestamp))
            update = 0;
        LOGV_IF(DEBUG_OUTPUT_CONTROL, "output_cotnrol pass through");
        *timestamp0 = *sensor_timestamp;
        return update;
    }

    memset(values, 0, sensor_value_size*sizeof(float));
    update = func(&(values[0]), &accuracy, &timestamp);
    memcpy(sensor_values, values, sensor_value_size*sizeof(float));
    *sensor_accuracy = accuracy;
    *sensor_timestamp = timestamp;
    LOGV_IF(DEBUG_OUTPUT_CONTROL, ">>%d 0: (Rate0:%lld) timestamp=%lld, prev timestamp0=%lld\n", sensor_id, (long long)Rate0, (long long)timestamp, (long long)(*timestamp0));

    if (sensor_id != -1)
        LOGV_IF(DEBUG_OUTPUT_CONTROL, ">>%d 1: (Rate0:%lld) timestamp=%lld, prev timestamp0=%lld\n", sensor_id, (long long)Rate0, (long long)timestamp, (long long)(*timestamp0));

    if ((timestamp != -1)&&(float)(timestamp - *timestamp0) > Rate0){ //report sensor data
        *timestamp0 = timestamp;
        cnt0 = 1;
        LOGV_IF(DEBUG_OUTPUT_CONTROL, ">>%d 1.2 (log) - [%d] timestamp=%lld, prev timestamp0=%lld update=%d", sensor_id, cnt0, (long long)timestamp, (long long)(*timestamp0), update);
        if (sensor_id != -1)
            LOGV_IF(DEBUG_OUTPUT_CONTROL, ">>%d 1.2 (report) - [%d] timestamp=%lld, prev timestamp0=%lld\n", sensor_id, cnt0, (long long)timestamp, (long long)(*timestamp0));
    } else {
        cnt0 ++;
        if (sensor_id != -1)
            LOGV_IF(DEBUG_OUTPUT_CONTROL, ">>%d 1.1 (skip) - [%d] timestamp=%lld, prev timestamp0=%lld\n",  sensor_id, cnt0, (long long)timestamp, (long long)(*timestamp0));
        else
            LOGV_IF(DEBUG_OUTPUT_CONTROL, ">>%d 1.1 (skip) - [%d] timestamp=%lld, prev timestamp0=%lld\n",  sensor_id, cnt0, (long long)timestamp, (long long)(*timestamp0));
        return 0;
    }

    if (sensor_id != -1)
        LOGV_IF(0, ">>%d 2: (gyroRate0:%lld) timestamp=%lld, gyro_timestamp0=%lld\n", sensor_id, (long long)Rate0, (long long)timestamp, (long long)(*timestamp0));

    if (!sensor_time || !(s->timestamp > sensor_time))
        update = 0;
    else
        update = 1;

    return update;
}

uint64_t MPLSensor::invCheckOutput(uint32_t sensor)
{
    int64_t tmp, orig_delay;
    uint32_t engine_delay = 0;

    LOGI_IF(HANDLER_DATA, "HAL:invCheckOutput - sensor=%d, sname=%s", sensor, mCurrentSensorMask[sensor].sname.c_str());

    if (sensor >= ID_NUMBER)
        return 0;

    if (*mCurrentSensorMask[sensor].engineRateAddr)
        engine_delay = *(mCurrentSensorMask[sensor].engineRateAddr);

    orig_delay = mDelays[sensor];

    if (engine_delay)
        tmp = orig_delay / engine_delay;
    else
        return 0;

    if (tmp)
        tmp = tmp * 2 -1;

    LOGI_IF(HANDLER_DATA, "HAL:invCheckOutput - orig delay=%lld, engine delay=%d, final=%llu",
            (long long)orig_delay, engine_delay, (unsigned long long)((uint64_t)tmp * engine_delay / 2));

    return tmp * engine_delay / 2;
}

/*  these handlers transform mpl data into one of the Android sensor types */
int MPLSensor::gyroHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;

    update = output_control(s, adapter_get_sensor_type_gyroscope, 3, s->gyro.v,
                &s->gyro.status, Gyro);
    if (update) {
        if (mGyroAccuracy > s->gyro.status)
            s->gyro.status = mGyroAccuracy;
    }

    LOGV_IF(HANDLER_DATA, "HAL:gyro data : %+f %+f %+f -- %lld - %d - %d",
            s->gyro.v[0], s->gyro.v[1], s->gyro.v[2], (long long)(s->timestamp), s->gyro.status, update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_GYROSCOPE, s->gyro.v, s->gyro.status, s->timestamp);
    }

    return update;
}

int MPLSensor::gyrowHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;

    update = output_control(s, adapter_get_sensor_type_gyroscope, 3, s->gyro.v,
                &s->gyro.status, Gyro_Wake);
    if (update) {
        if (mGyroAccuracy > s->gyro.status)
            s->gyro.status = mGyroAccuracy;
    }

    LOGV_IF(HANDLER_DATA, "HAL:gyro wake data : %+f %+f %+f -- %lld - %d - %d",
            s->gyro.v[0], s->gyro.v[1], s->gyro.v[2], (long long)(s->timestamp), s->gyro.status, update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_GYROSCOPE, s->gyro.v, s->gyro.status, s->timestamp);
    }

    return update;
}

int MPLSensor::rawGyroHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;
    float cal_gyro[6];
    float raw_gyro[6];
    float bias[3];
    int8_t status;
    int64_t ts;
    static float prev_bias[3] = {0.f, 0.f, 0.f};

    memset(bias, 0, sizeof(bias));

    update = output_control(s, adapter_get_sensor_type_gyroscope_raw, 3, raw_gyro,
                &status, RawGyro);

    s->uncalibrated_gyro.uncalib[0] = raw_gyro[0];
    s->uncalibrated_gyro.uncalib[1] = raw_gyro[1];
    s->uncalibrated_gyro.uncalib[2] = raw_gyro[2];

    adapter_get_sensor_type_gyroscope(cal_gyro, &status, &ts);
    if (ts == s->timestamp && (mGyroAccuracy > 0 || (mCalibrationMode & mpl_gyro_cal))) {
        inv_calculate_bias(cal_gyro, s->uncalibrated_gyro.uncalib, bias);
        memcpy(s->uncalibrated_gyro.bias, bias, sizeof(bias));
        LOGV_IF(HANDLER_DATA,"HAL:gyro bias data: %+f %+f %+f -- %lld - %d",
                s->uncalibrated_gyro.bias[0], s->uncalibrated_gyro.bias[1],
                s->uncalibrated_gyro.bias[2], (long long)(s->timestamp), update);
        prev_bias[0] = bias[0];
        prev_bias[1] = bias[1];
        prev_bias[2] = bias[2];
    } else {
        memcpy(s->uncalibrated_gyro.bias, prev_bias, sizeof(prev_bias));
    }

    LOGV_IF(HANDLER_DATA, "HAL:raw gyro data : %+f %+f %+f -- %lld - %d",
            s->uncalibrated_gyro.uncalib[0], s->uncalibrated_gyro.uncalib[1],
            s->uncalibrated_gyro.uncalib[2], (long long)(s->timestamp), update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_UNCAL_GYROSCOPE, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::rawGyrowHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;
    float cal_gyro[6];
    float raw_gyro[6];
    float bias[3];
    int8_t status;
    int64_t ts;

    update = output_control(s, adapter_get_sensor_type_gyroscope_raw, 3, raw_gyro,
                &status, RawGyro_Wake);

    memset(bias, 0, sizeof(bias));

    if (update) {
        s->uncalibrated_gyro.uncalib[0] = raw_gyro[0];
        s->uncalibrated_gyro.uncalib[1] = raw_gyro[1];
        s->uncalibrated_gyro.uncalib[2] = raw_gyro[2];
    }

    adapter_get_sensor_type_gyroscope(cal_gyro, &status, &ts);
    if (ts == s->timestamp && (mGyroAccuracy > 0 || (mCalibrationMode & mpl_gyro_cal))) {
        inv_calculate_bias(cal_gyro, s->uncalibrated_gyro.uncalib, bias);
        memcpy(s->uncalibrated_gyro.bias, bias, sizeof(bias));
        LOGV_IF(HANDLER_DATA,"HAL:gyro bias data: %+f %+f %+f -- %lld - %d",
                s->uncalibrated_gyro.bias[0], s->uncalibrated_gyro.bias[1],
                s->uncalibrated_gyro.bias[2], (long long)(s->timestamp), update);
    }

    LOGV_IF(HANDLER_DATA, "HAL:raw gyro wake data : %+f %+f %+f -- %lld - %d",
            s->uncalibrated_gyro.uncalib[0], s->uncalibrated_gyro.uncalib[1],
            s->uncalibrated_gyro.uncalib[2], (long long)(s->timestamp), update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_UNCAL_GYROSCOPE, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::gyroRawHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = 0;
    int data[3];
    int i;

    /* convert to body frame */
    for (i = 0; i < 3 ; i++) {
        data[i] = mRawGyroCustom[0] * mGyroOrientationMatrix[i * 3] +
                  mRawGyroCustom[1] * mGyroOrientationMatrix[i * 3 + 1] +
                  mRawGyroCustom[2] * mGyroOrientationMatrix[i * 3 + 2];
    }
    s->uncalibrated_gyro.uncalib[0] = data[0];
    s->uncalibrated_gyro.uncalib[1] = data[1];
    s->uncalibrated_gyro.uncalib[2] = data[2];
    s->timestamp = mGyroSensorTimestamp;

    LOGV_IF(HANDLER_DATA, "HAL:raw gyro data (custom) : %+f %+f %+f -- %lld - %d",
            s->uncalibrated_gyro.uncalib[0], s->uncalibrated_gyro.uncalib[1],
            s->uncalibrated_gyro.uncalib[2], (long long)(s->timestamp), 1);

    if (mRawGyroCustomTimestamp != mGyroSensorTimestamp)
        update = 1;

    mRawGyroCustomTimestamp = mGyroSensorTimestamp;

    return update;
}

int MPLSensor::accelHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;

    update = output_control(s, adapter_get_sensor_type_accelerometer, 3, s->acceleration.v,
                &s->acceleration.status, Accelerometer);
    if (update) {
        if (mAccelAccuracy > s->acceleration.status)
            s->acceleration.status = mAccelAccuracy;
    }

    LOGV_IF(HANDLER_DATA, "HAL:accel data : %+f %+f %+f -- %lld - %d - %d",
            s->acceleration.v[0], s->acceleration.v[1], s->acceleration.v[2],
            (long long)(s->timestamp), s->acceleration.status, update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_ACCELEROMETER, s->acceleration.v, s->acceleration.status, s->timestamp);
    }

    return update;
}

int MPLSensor::accelwHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;

    update = output_control(s, adapter_get_sensor_type_accelerometer, 3, s->acceleration.v,
                &s->acceleration.status, Accelerometer_Wake);
    if (update) {
        if (mAccelAccuracy > s->acceleration.status)
            s->acceleration.status = mAccelAccuracy;
    }

    LOGV_IF(HANDLER_DATA, "HAL:accel wake data : %+f %+f %+f -- %lld - %d - %d",
            s->acceleration.v[0], s->acceleration.v[1], s->acceleration.v[2],
            (long long)(s->timestamp), s->acceleration.status, update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_ACCELEROMETER, s->acceleration.v, s->acceleration.status, s->timestamp);
    }

    return update;
}

int MPLSensor::rawAccelHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;
    float cal_accel[6];
    float raw_accel[6];
    float bias[3];
    int8_t status;
    int64_t ts;
    static float prev_bias[3] = {0.f, 0.f, 0.f};

    memset(bias, 0, sizeof(bias));

    update = output_control(s, adapter_get_sensor_type_accelerometer_raw, 3, raw_accel,
                &status, RawAccelerometer);

    s->uncalibrated_accelerometer.uncalib[0] = raw_accel[0];
    s->uncalibrated_accelerometer.uncalib[1] = raw_accel[1];
    s->uncalibrated_accelerometer.uncalib[2] = raw_accel[2];

    adapter_get_sensor_type_accelerometer(cal_accel, &status, &ts);
    if (ts == s->timestamp && (mAccelAccuracy > 0 || (mCalibrationMode & mpl_accel_cal))) {
        inv_calculate_bias(cal_accel, s->uncalibrated_accelerometer.uncalib, bias);
        memcpy(s->uncalibrated_accelerometer.bias, bias, sizeof(bias));
        LOGV_IF(HANDLER_DATA,"HAL:accel bias data: %+f %+f %+f -- %lld - %d",
                s->uncalibrated_accelerometer.bias[0], s->uncalibrated_accelerometer.bias[1],
                s->uncalibrated_accelerometer.bias[2], (long long)(s->timestamp), update);
        prev_bias[0] = bias[0];
        prev_bias[1] = bias[1];
        prev_bias[2] = bias[2];
    } else {
        memcpy(s->uncalibrated_accelerometer.bias, prev_bias, sizeof(prev_bias));
    }

    LOGV_IF(HANDLER_DATA, "HAL:raw accel data : %+f %+f %+f -- %lld - %d",
            s->uncalibrated_accelerometer.uncalib[0], s->uncalibrated_accelerometer.uncalib[1],
            s->uncalibrated_accelerometer.uncalib[2], (long long)(s->timestamp), update);

    /* TODO, not yet supported by log format
    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_UNCAL_ACCELEROMETER, s->data, 0, s->timestamp);
    } */

    return update;
}

int MPLSensor::rawAccelwHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;
    float cal_accel[6];
    float raw_accel[6];
    float bias[3];
    int8_t status;
    int64_t ts;
    static float prev_bias[3] = {0.f, 0.f, 0.f};

    memset(bias, 0, sizeof(bias));

    update = output_control(s, adapter_get_sensor_type_accelerometer_raw, 3, raw_accel,
                &status, RawAccelerometer_Wake);

    s->uncalibrated_accelerometer.uncalib[0] = raw_accel[0];
    s->uncalibrated_accelerometer.uncalib[1] = raw_accel[1];
    s->uncalibrated_accelerometer.uncalib[2] = raw_accel[2];

    adapter_get_sensor_type_accelerometer(cal_accel, &status, &ts);
    if (ts == s->timestamp && (mAccelAccuracy > 0 || (mCalibrationMode & mpl_accel_cal))) {
        inv_calculate_bias(cal_accel, s->uncalibrated_accelerometer.uncalib, bias);
        memcpy(s->uncalibrated_accelerometer.bias, bias, sizeof(bias));
        LOGV_IF(HANDLER_DATA,"HAL:accel bias data: %+f %+f %+f -- %lld - %d",
                s->uncalibrated_accelerometer.bias[0], s->uncalibrated_accelerometer.bias[1],
                s->uncalibrated_accelerometer.bias[2], (long long)(s->timestamp), update);
        prev_bias[0] = bias[0];
        prev_bias[1] = bias[1];
        prev_bias[2] = bias[2];
    } else {
        memcpy(s->uncalibrated_accelerometer.bias, prev_bias, sizeof(prev_bias));
    }

    LOGV_IF(HANDLER_DATA, "HAL:raw accel wake data : %+f %+f %+f -- %lld - %d",
            s->uncalibrated_accelerometer.uncalib[0], s->uncalibrated_accelerometer.uncalib[1],
            s->uncalibrated_accelerometer.uncalib[2], (long long)(s->timestamp), update);

    /* TODO, not yet supported by log format
    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_UNCAL_ACCELEROMETER, s->data, 0, s->timestamp);
    } */

    return update;
}

int MPLSensor::accelRawHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = 0;
    int data[3];
    int i;

    /* convert to body frame */
    for (i = 0; i < 3 ; i++) {
        data[i] = mRawAccelCustom[0] * mAccelOrientationMatrix[i * 3] +
                  mRawAccelCustom[1] * mAccelOrientationMatrix[i * 3 + 1] +
                  mRawAccelCustom[2] * mAccelOrientationMatrix[i * 3 + 2];
    }
    s->acceleration.v[0] = data[0];
    s->acceleration.v[1] = data[1];
    s->acceleration.v[2] = data[2];
    s->timestamp = mAccelSensorTimestamp;

    LOGV_IF(HANDLER_DATA, "HAL:raw accel data : %+f %+f %+f -- %lld - %d",
            s->acceleration.v[0], s->acceleration.v[1],
            s->acceleration.v[2], (long long)(s->timestamp), 1);
    if (mRawAccelCustomTimestamp != mAccelSensorTimestamp)
        update = 1;

    mRawAccelCustomTimestamp = mAccelSensorTimestamp;

    return update;
}

int MPLSensor::compassHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;

    update = output_control(s, adapter_get_sensor_type_magnetic_field, 3, s->magnetic.v,
                &s->magnetic.status, MagneticField);
    if (update) {
        // If magnetic dist is detected, accuracy will be 2
        if (mCompassAccuracy == 1 && mCompassAccuracy > s->magnetic.status) {
            s->magnetic.status = mCompassAccuracy;
        } else {
            mCompassAccuracy = s->magnetic.status;
        }
    }

    LOGV_IF(HANDLER_DATA, "HAL:compass data: %+f %+f %+f -- %lld - %d - %d",
            s->magnetic.v[0], s->magnetic.v[1], s->magnetic.v[2],
            (long long)(s->timestamp), s->magnetic.status, update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_MAGNETOMETER, s->magnetic.v, s->magnetic.status, s->timestamp);
    }

    return update;
}

int MPLSensor::compasswHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;

    update = output_control(s, adapter_get_sensor_type_magnetic_field, 3, s->magnetic.v,
                &s->magnetic.status, MagneticField_Wake);
    if (update) {
        // If magnetic dist is detected, accuracy will be 2
        if (mCompassAccuracy == 1 && mCompassAccuracy > s->magnetic.status) {
            s->magnetic.status = mCompassAccuracy;
        } else {
            mCompassAccuracy = s->magnetic.status;
        }
    }

    LOGV_IF(HANDLER_DATA, "HAL:compass wake data: %+f %+f %+f -- %lld - %d - %d",
            s->magnetic.v[0], s->magnetic.v[1], s->magnetic.v[2],
            (long long)(s->timestamp), s->magnetic.status, update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_MAGNETOMETER, s->magnetic.v, s->magnetic.status, s->timestamp);
    }

    return update;
}

int MPLSensor::rawCompassHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;
    int8_t status;
    int64_t ts;
    float bias[3];
    float cal_compass[6];
    float raw_compass[6];

    static long long int timestamp_compass_prev = 0;

    memset(bias, 0, sizeof(bias));
    //TODO: need to handle uncalib data and bias for 3rd party compass
    if (mCompassSensor->providesCalibration()) {
        update = mCompassSensor->readRawSample(s->uncalibrated_magnetic.uncalib, &s->timestamp);
    } else {
        update = output_control(s, adapter_get_sensor_type_magnetic_field_raw, 3, raw_compass,
                    &status, RawMagneticField);
    }
    if (timestamp_compass_prev ==  s->timestamp)
        return 0; // not updated raw compass

    timestamp_compass_prev = s->timestamp;

    if (update) {
        adapter_get_sensor_type_magnetic_field(cal_compass, &status, &ts);
        inv_calculate_bias(cal_compass, raw_compass, bias);
        memcpy(s->uncalibrated_magnetic.bias, bias, sizeof(bias));
        s->uncalibrated_magnetic.uncalib[0] = raw_compass[0];
        s->uncalibrated_magnetic.uncalib[1] = raw_compass[1];
        s->uncalibrated_magnetic.uncalib[2] = raw_compass[2];

        LOGV_IF(HANDLER_DATA, "HAL:compass bias data: %+f %+f %+f %d -- %lld - %d",
            s->uncalibrated_magnetic.bias[0], s->uncalibrated_magnetic.bias[1],
            s->uncalibrated_magnetic.bias[2], mCompassAccuracy, (long long)(s->timestamp), update);
    }

    LOGV_IF(HANDLER_DATA, "HAL:raw compass data: %+f %+f %+f %d -- %lld diff %lld - %d",
            s->uncalibrated_magnetic.uncalib[0], s->uncalibrated_magnetic.uncalib[1],
            s->uncalibrated_magnetic.uncalib[2], mCompassAccuracy, (long long)(s->timestamp), s->timestamp -
            timestamp_compass_prev, update);
    if (!mEnabledTime[RawMagneticField] || !(s->timestamp > mEnabledTime[RawMagneticField]))
        update = 0;

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_UNCAL_MAGNETOMETER, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::rawCompasswHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;
    float bias[3];
    float cal_compass[6];
    float raw_compass[6];

    memset(bias, 0, sizeof(bias));
    //TODO: need to handle uncalib data and bias for 3rd party compass
    if (mCompassSensor->providesCalibration()) {
        update = mCompassSensor->readRawSample(s->uncalibrated_magnetic.uncalib, &s->timestamp);
    } else {
        int8_t status;
        int64_t ts;

        update = output_control(s, adapter_get_sensor_type_magnetic_field_raw, 3, raw_compass,
                    &status, RawMagneticField_Wake);
        adapter_get_sensor_type_magnetic_field(cal_compass, &status, &ts);
        inv_calculate_bias(cal_compass, raw_compass, bias);
    }
    if (update) {
        memcpy(s->uncalibrated_magnetic.bias, bias, sizeof(bias));

        s->uncalibrated_magnetic.uncalib[0] = raw_compass[0];
        s->uncalibrated_magnetic.uncalib[1] = raw_compass[1];
        s->uncalibrated_magnetic.uncalib[2] = raw_compass[2];

        LOGV_IF(HANDLER_DATA, "HAL:compass bias data: %+f %+f %+f %d -- %lld - %d",
                s->uncalibrated_magnetic.bias[0], s->uncalibrated_magnetic.bias[1],
                s->uncalibrated_magnetic.bias[2], mCompassAccuracy, (long long)(s->timestamp), update);
    }
    if (!mEnabledTime[RawMagneticField_Wake] || !(s->timestamp > mEnabledTime[RawMagneticField_Wake]))
        update = 0;

    LOGV_IF(HANDLER_DATA, "HAL:compass wake raw data: %+f %+f %+f %d -- %lld - %d",
            s->uncalibrated_magnetic.uncalib[0], s->uncalibrated_magnetic.uncalib[1],
            s->uncalibrated_magnetic.uncalib[2], mCompassAccuracy, (long long)(s->timestamp), update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_UNCAL_MAGNETOMETER, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::compassRawHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = 0;
    int data[3];
    int i;

    /* convert to body frame */
    for (i = 0; i < 3 ; i++) {
        data[i] = mRawMagCustom[0] * mCompassOrientationMatrix[i * 3] +
                  mRawMagCustom[1] * mCompassOrientationMatrix[i * 3 + 1] +
                  mRawMagCustom[2] * mCompassOrientationMatrix[i * 3 + 2];
    }
    s->uncalibrated_magnetic.uncalib[0] = data[0];
    s->uncalibrated_magnetic.uncalib[1] = data[1];
    s->uncalibrated_magnetic.uncalib[2] = data[2];
    s->timestamp = mCompassTimestamp;

    LOGV_IF(HANDLER_DATA, "HAL:compass raw data: %+f %+f %+f -- %lld - %d",
            s->uncalibrated_magnetic.uncalib[0], s->uncalibrated_magnetic.uncalib[1],
            s->uncalibrated_magnetic.uncalib[2], (long long)(s->timestamp), 1);

    if (mRawMagCustomTimestamp != mCompassTimestamp)
        update = 1;

    mRawMagCustomTimestamp = mCompassTimestamp;

    return update;
}

/*
   Rotation Vector handler.
NOTE: rotation vector does not have an accuracy or status by definition
 */
int MPLSensor::rvHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int8_t status;
    int update;
    update = output_control(s, adapter_get_sensor_type_rotation_vector, 5, s->data,
                &status, RotationVector);

    if (!(mCalibrationMode & mpl_quat)) {
        /* convert Q29 radian (Upper 2 bytes) to float radian */
        s->data[4] = (float)ABS(mHeadingAccuracy) / (1 << 13);
    }
    LOGV_IF(HANDLER_DATA, "HAL:rv data: %+f %+f %+f %+f %+f %+lld - %d",
            s->data[0], s->data[1], s->data[2], s->data[3], s->data[4],
            (long long)(s->timestamp), update);
#if DEBUG_TIME_PROFILE
    if (update)
        printTimeProfile(prevtime, 0, s->timestamp, 0);
#endif

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_ROTATION_VECTOR, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::rvwHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int8_t status;
    int update;

    update = output_control(s, adapter_get_sensor_type_rotation_vector, 4, s->data,
                &status, RotationVector_Wake);

    if (!(mCalibrationMode & mpl_quat)) {
        /* convert Q29 radian (Upper 2 bytes) to float radian */
        s->data[4] = (float)ABS(mHeadingAccuracy) / (1 << 13);
    }
    LOGV_IF(HANDLER_DATA, "HAL:rv wake data: %+f %+f %+f %+f %+f %+lld - %d",
            s->data[0], s->data[1], s->data[2], s->data[3], s->data[4],
            (long long)(s->timestamp), update);
#if DEBUG_TIME_PROFILE
    if (update)
        printTimeProfile(prevtime, 0, s->timestamp, 0);
#endif

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_ROTATION_VECTOR, s->data, 0, s->timestamp);
    }

    return update;
}

//lpq handler for 3 axis
int MPLSensor::lpqHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int8_t status;
    int update = 0;

    update = output_control(s, adapter_get_sensor_type_lpq, 3, s->data,
                            &status, LPQ);

    LOGV_IF(HANDLER_DATA, "HAL:lpq data: %+f %+f %+f %+lld - %d",
            s->data[0], s->data[1], s->data[2], (long long)(s->timestamp), update);
    return update;
}

/*
   Game Rotation Vector handler.
NOTE: rotation vector does not have an accuracy or status
 */
int MPLSensor::grvHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int8_t status;
    int update = 0;

    update = output_control(s, adapter_get_sensor_type_game_rotation_vector, 4, s->data,
                &status, GameRotationVector);

    LOGV_IF(HANDLER_DATA, "HAL:grv data: %+f %+f %+f %+f %+lld - %d",
            s->data[0], s->data[1], s->data[2], s->data[3], (long long)(s->timestamp),
            update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_GAME_ROTATION_VECTOR, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::grvwHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int8_t status;
    int update;

    update = output_control(s, adapter_get_sensor_type_game_rotation_vector, 4, s->data,
                &status, GameRotationVector_Wake);

    LOGV_IF(HANDLER_DATA, "HAL:grv wake data: %+f %+f %+f %+f %+f %+lld - %d",
            s->data[0], s->data[1], s->data[2], s->data[3], s->data[4], (long long)(s->timestamp),
            update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_GAME_ROTATION_VECTOR, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::laHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;

    update = output_control(s, adapter_get_sensor_type_linear_acceleration, 3, s->acceleration.v,
                &s->acceleration.status, LinearAccel);

    if (mAccelAccuracy > s->acceleration.status)
        s->acceleration.status = mAccelAccuracy;

    LOGV_IF(HANDLER_DATA, "HAL:la data: %+f %+f %+f - %lld - %d - %d",
            s->acceleration.v[0], s->acceleration.v[1], s->acceleration.v[2], (long long)(s->timestamp), s->acceleration.status, update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_LINEAR_ACCELERATION, s->acceleration.v, s->acceleration.status, s->timestamp);
    }

    return update;
}

int MPLSensor::lawHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;

    update = output_control(s, adapter_get_sensor_type_linear_acceleration, 3, s->acceleration.v,
                &s->acceleration.status, LinearAccel_Wake);

    if (mAccelAccuracy > s->acceleration.status)
        s->acceleration.status = mAccelAccuracy;

    LOGV_IF(HANDLER_DATA, "HAL:la wake data: %+f %+f %+f - %lld - %d - %d",
            s->acceleration.v[0], s->acceleration.v[1], s->acceleration.v[2], (long long)(s->timestamp), s->acceleration.status, update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_LINEAR_ACCELERATION, s->acceleration.v, s->acceleration.status, s->timestamp);
    }

    return update;
}

int MPLSensor::gravHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;

    update = output_control(s, adapter_get_sensor_type_gravity, 3, s->acceleration.v,
                &s->acceleration.status, Gravity);

    if (mAccelAccuracy > s->acceleration.status)
        s->acceleration.status = mAccelAccuracy;

    LOGV_IF(HANDLER_DATA, "HAL:gr data: %+f %+f %+f - %lld - %d - %d",
            s->acceleration.v[0], s->acceleration.v[1], s->acceleration.v[2], (long long)(s->timestamp), s->acceleration.status, update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_GRAVITY, s->acceleration.v, s->acceleration.status, s->timestamp);
    }

    return update;
}

int MPLSensor::gravwHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;

    update = output_control(s, adapter_get_sensor_type_gravity, 3, s->acceleration.v,
                &s->acceleration.status, Gravity_Wake);

    if (mAccelAccuracy > s->acceleration.status)
        s->acceleration.status = mAccelAccuracy;

    LOGV_IF(HANDLER_DATA, "HAL:gr wake data: %+f %+f %+f - %lld - %d - %d",
            s->acceleration.v[0], s->acceleration.v[1], s->acceleration.v[2], (long long)(s->timestamp), s->acceleration.status, update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_GRAVITY, s->acceleration.v, s->acceleration.status, s->timestamp);
    }

    return update;
}

int MPLSensor::orienHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;

    update = output_control(s, adapter_get_sensor_type_orientation, 3, s->orientation.v,
                &s->orientation.status, Orientation);

    if (mCompassAccuracy > s->orientation.status)
        s->orientation.status = mCompassAccuracy;

    LOGV_IF(HANDLER_DATA, "HAL:or data: %f %f %f - %lld - %d -%d",
            s->orientation.v[0], s->orientation.v[1], s->orientation.v[2],
            (long long)(s->timestamp), s->orientation.status, update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_ORIENTATION, s->orientation.v, s->orientation.status, s->timestamp);
    }

    return update;
}

int MPLSensor::orienwHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update;

    update = output_control(s, adapter_get_sensor_type_orientation, 3, s->orientation.v,
                &s->orientation.status, Orientation_Wake);

    if (mCompassAccuracy > s->orientation.status)
        s->orientation.status = mCompassAccuracy;

    LOGV_IF(HANDLER_DATA, "HAL:or wake data: %f %f %f - %lld - %d -%d",
            s->orientation.v[0], s->orientation.v[1], s->orientation.v[2],
            (long long)(s->timestamp), s->orientation.status, update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_ORIENTATION, s->orientation.v, s->orientation.status, s->timestamp);
    }

    return update;
}

int MPLSensor::smHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = 1;

    /* When event is triggered, set data to 1 */
    s->data[0] = 1.f;

    /* Capture timestamp in HAL */
    s->timestamp = getTimestamp();

    LOGV_IF(HANDLER_DATA, "HAL:sm data: %f - %lld - %d",
            s->data[0], (long long)(s->timestamp), update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_SIGNIFICANT_MOTION, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::gmHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int8_t status;
    int update = 0;

    update = output_control(s, adapter_get_sensor_type_geomagnetic_rotation_vector, 5, s->data,
                &status, GeomagneticRotationVector);

    if (!(mCalibrationMode & mpl_quat)) {
        /* convert Q29 radian (Upper 2 bytes) to float radian */
        s->data[4] = (float)ABS(mHeadingAccuracy) / (1 << 13);
    }

    LOGV_IF(HANDLER_DATA, "HAL:gm data: %+f %+f %+f %+f %+f %+lld - %d",
            s->data[0], s->data[1], s->data[2], s->data[3], s->data[4], (long long)(s->timestamp), update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_GEOMAGNETIC_ROTATION_VECTOR, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::gmwHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int8_t status;
    int update = 0;

    update = output_control(s, adapter_get_sensor_type_geomagnetic_rotation_vector, 5, s->data,
                &status, GeomagneticRotationVector_Wake);
    if (!(mCalibrationMode & mpl_quat)) {
        /* convert Q29 radian (Upper 2 bytes) to float radian */
        s->data[4] = (float)ABS(mHeadingAccuracy) / (1 << 13);
    }

    LOGV_IF(HANDLER_DATA, "HAL:gm wake data: %+f %+f %+f %+f %+f %+lld - %d",
            s->data[0], s->data[1], s->data[2], s->data[3], s->data[4], (long long)(s->timestamp), update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_GEOMAGNETIC_ROTATION_VECTOR, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::psHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = 0;

    s->pressure = mCachedPressureData / 100.f / 100.f; //hpa (millibar)
    s->timestamp = mPressureTimestamp;
    update = mPressureUpdate;
    mPressureUpdate = 0;

    LOGV_IF(HANDLER_DATA, "HAL:ps data: %+f - %+lld - %d",
            s->pressure, (long long)(s->timestamp), update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_PRESSURE, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::pswHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = 0;

    s->pressure = mCachedPressureWakeData / 100.f / 100.f; //hpa (millibar)
    s->timestamp = mPressureWakeTimestamp;
    update = mPressureWakeUpdate;
    mPressureWakeUpdate = 0;

    LOGV_IF(HANDLER_DATA, "HAL:ps wake data: %+f - %+lld - %d",
            s->pressure, (long long)(s->timestamp), update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_PRESSURE, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::sdHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = mPedUpdate;

    /* When event is triggered, set data to 1 */
    s->data[0] = 1;

    mPedUpdate = 0;

    if (mCalibrationMode & mpl_pedo) {
        // not supported
        update = 0;
    } else {
        /* get current timestamp */
        s->timestamp = (int64_t)mStepSensorTimestamp;
    }

    LOGV_IF(HANDLER_DATA, "HAL:sd data: %f - %lld - %d",
        s->data[0], (long long)(s->timestamp), update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_STEP_DETECTOR, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::sdwHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = mPedWakeUpdate;

    /* When event is triggered, set data to 1 */
    s->data[0] = 1;

    mPedWakeUpdate = 0;

    if (mCalibrationMode & mpl_pedo) {
        // not supported
        update = 0;
    } else {
        /* get current timestamp */
        s->timestamp = (int64_t)mStepSensorTimestamp;
    }

    LOGV_IF(HANDLER_DATA, "HAL:sd wake data: %f - %lld - %d",
            s->data[0], (long long)(s->timestamp), update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_STEP_DETECTOR, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::scHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = 0;
    int stepReceived = 0;

    if (mCalibrationMode & mpl_pedo) {
        // not supported
    } else {
        if ((mStepSensorTimestamp > mEnabledTime[StepCounter]) && mEnabledTime[StepCounter]) {
            s->timestamp = mStepSensorTimestamp;
            // mStepCount is set already with DMP value
            stepReceived = 1;
        }
    }
    if (stepReceived) {
        /* First time after enabled */
        if (mStepCounterHandlerNotCalled) {
            mStepCounterHandlerNotCalled = false;
            mLastStepCount = mStepCount;
            update = 1;
        }
        /* Step count is updated */
        if (mStepCount != mLastStepCount) {
            if (mStepCount > mLastStepCount) {
                mLastStepCountSave += mStepCount - mLastStepCount;
            } else if (mStepCount < mLastStepCount) {
                /* Assume step count value from DMP/library is 32bit */
                mLastStepCountSave += UINT32_MAX - mLastStepCount + 1;
                mLastStepCountSave += mStepCount;
            }
            mLastStepCount = mStepCount;
            update = 1;
        }
        s->u64.step_counter = mLastStepCountSave;
    }

    LOGV_IF(HANDLER_DATA, "HAL:sc data: %lld - %lld - %d",
            (long long)(s->u64.step_counter), (long long)(s->timestamp), update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_STEP_COUNTER, s->u64.step_counter, s->timestamp);
    }

    return update;
}

int MPLSensor::scwHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = 0;
    int stepReceived = 0;

    if (mCalibrationMode & mpl_pedo) {
        // not supported
    } else {
        if ((mStepSensorWakeTimestamp > mEnabledTime[StepCounter_Wake]) && mEnabledTime[StepCounter_Wake]) {
            s->timestamp = mStepSensorWakeTimestamp;
            // mStepCountWake is set already with DMP value
            stepReceived = 1;
        }
    }
    if (stepReceived) {
        /* First time after enabled */
        if (mStepCounterHandlerNotCalledWake) {
            mStepCounterHandlerNotCalledWake = false;
            mLastStepCountWake = mStepCountWake;
            update = 1;
        }
        /* Step count is updated */
        if (mStepCountWake != mLastStepCountWake) {
            if (mStepCountWake > mLastStepCountWake) {
                mLastStepCountSaveWake += mStepCountWake - mLastStepCountWake;
            } else if (mStepCountWake < mLastStepCountWake) {
                /* Assume step count value from DMP/library is 32bit */
                mLastStepCountSaveWake += UINT32_MAX - mLastStepCountWake + 1;
                mLastStepCountSaveWake += mStepCountWake;
            }
            mLastStepCountWake = mStepCountWake;
            update = 1;
        }
        s->u64.step_counter = mLastStepCountSaveWake;
    }

    LOGV_IF(HANDLER_DATA, "HAL:sc wake data: %llu - %lld - %d",
            (unsigned long long)(s->u64.step_counter), (long long)(s->timestamp), update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_STEP_COUNTER, s->u64.step_counter, s->timestamp);
    }

    return update;
}

int MPLSensor::tiltHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = mTiltUpdate;

    mTiltUpdate = 0;

    /* When event is triggered, set data to 1 */
    s->data[0] = 1.f;

    if (mCalibrationMode & mpl_tilt) {
        // not supported
        update = 0;
    } else {
        /* Capture timestamp in HAL */
        s->timestamp = getTimestamp();
    }

    LOGV_IF(HANDLER_DATA, "HAL:tilt data: %f - %lld - %d",
            s->data[0], (long long)(s->timestamp), update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_TILT_DETECTOR, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::tapHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = mTapUpdate;
    int data[3];
    int i;

    mTapUpdate = 0;

    /* convert to body frame */
    for (i = 0; i < 3 ; i++) {
        data[i] = mCachedTapData[0] * mAccelOrientationMatrix[i * 3] +
                  mCachedTapData[1] * mAccelOrientationMatrix[i * 3 + 1] +
                  mCachedTapData[2] * mAccelOrientationMatrix[i * 3 + 2];
    }
    s->data[0] = data[0];
    s->data[1] = data[1];
    s->data[2] = data[2];
    s->timestamp = mTapTimestamp;

    LOGV_IF(HANDLER_DATA, "HAL:tap data: %f %f %f - %lld - %d",
            s->data[0], s->data[1], s->data[2], (long long)(s->timestamp), update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_DOUBLE_TAP, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::pickupHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = mPickupUpdate;

    mPickupUpdate = 0;

    /* When event is triggered, set data to 1 */
    s->data[0] = 1.f;

    if (mCalibrationMode & mpl_pickup) {
        // not supported
        update = 0;
    } else {
        /* Capture timestamp in HAL */
        s->timestamp = getTimestamp();
    }

    if (update) {
        /* Disable as Pickup is one shot */
        enable(ID_PICK, 0);
    }

    LOGV_IF(HANDLER_DATA, "HAL:pickup data: %f - %lld - %d",
            s->data[0], (long long)(s->timestamp), update);

    if (update) {
        mMPLLog.logEvents(MPLLogger::SENSOR_PICK_UP_GESTURE, s->data, 0, s->timestamp);
    }

    return update;
}

int MPLSensor::stationaryDetectHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = 1;

    /* When event is triggered, set data to 1 */
    s->data[0] = 1.f;

    if (mCalibrationMode & mpl_md) {
        // not supported
	update = 0;
    } else {
        /* Capture timestamp in HAL */
        s->timestamp = getTimestamp();
    }

    LOGV_IF(HANDLER_DATA, "HAL:stationary detect data: %f - %lld - %d",
            s->data[0], (long long)(s->timestamp), update);

    return update;
}

int MPLSensor::motionDetectHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = 1;

    /* When event is triggered, set data to 1 */
    s->data[0] = 1.f;

    if (mCalibrationMode & mpl_md) {
        // not supported
        update = 0;
    } else {
        /* Capture timestamp in HAL */
        s->timestamp = getTimestamp();
    }

    LOGV_IF(HANDLER_DATA, "HAL:motion detect data: %f - %lld - %d",
            s->data[0], (long long)(s->timestamp), update);

    return update;
}

int MPLSensor::eisHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = 0;
    int8_t status = 0;

    update = adapter_get_sensor_type_eis_gyroscope(s->data, &status, &s->timestamp);

    LOGV_IF(HANDLER_DATA, "HAL:eis data: %f %f %f %f - %lld - %d",
            s->data[0], s->data[1], s->data[2], s->data[3], (long long)(s->timestamp), update);
    update = mEisUpdate;
    mEisUpdate = 0;
    return update;
}

int MPLSensor::eisAuthenticationHandler(sensors_event_t* s)
{
    VHANDLER_LOG;
    int update = 0;
    int8_t status = 0;

    update = adapter_get_sensor_type_eis_authentication(s->data, &status, &s->timestamp);

    LOGV_IF(HANDLER_DATA, "HAL:eis auth data: %f %f %f %f - %lld - %d",
            s->data[0], s->data[1], s->data[2], s->data[3], (long long)(s->timestamp), update);
    update = mEisAuthenticationUpdate;
    mEisAuthenticationUpdate = 0;
    return update;
}

int MPLSensor::metaHandler(int sensor, sensors_event_t* s, int flags)
{
    VHANDLER_LOG;
    int update = 1;

    /* initalize SENSOR_TYPE_META_DATA */
    s->version = META_DATA_VERSION;
    s->sensor = 0;
    s->reserved0 = 0;
    s->timestamp = 0LL;

    switch(flags) {
        case META_DATA_FLUSH_COMPLETE:
            s->type = SENSOR_TYPE_META_DATA;
            s->meta_data.what = flags;
            s->meta_data.sensor = sensor;

            LOGV_IF(HANDLER_DATA,
                    "HAL:flush complete data: type=%d what=%d, "
                    "sensor=%d - %lld - %d",
                    s->type, s->meta_data.what, s->meta_data.sensor,
                    (long long)(s->timestamp), update);
            break;

        default:
            LOGW("HAL: Meta flags not supported");
            break;
    }

    return update;
}

int MPLSensor::lightHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    if ((int)s->data[0] == mCachedAlsData[0]) {
        if (mLightInitEvent)
            mLightInitEvent = false;
        else
            return 0;
    }

    s->data[0] = (float)mCachedAlsData[0];
    s->timestamp = mAlsSensorTimestamp;

    LOGV_IF(HANDLER_DATA, "HAL:light data: %+f - %lld - %d",
            s->data[0], (long long)(s->timestamp), 1);

    mMPLLog.logEvents(MPLLogger::SENSOR_LIGHT, s->data, 0, s->timestamp);

    return 1;
}

int MPLSensor::lightwHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    if ((int)s->data[0] == mCachedAlsWakeData[0]) {
        if (mLightWakeInitEvent)
            mLightWakeInitEvent = false;
        else
            return 0;
    }

    s->data[0] = mCachedAlsWakeData[0];
    s->timestamp = mAlsSensorWakeTimestamp;

    LOGV_IF(HANDLER_DATA, "HAL:light wake data: %+f - %lld - %d",
            s->data[0], (long long)(s->timestamp), 1);

    mMPLLog.logEvents(MPLLogger::SENSOR_LIGHT, s->data, 0, s->timestamp);

    return 1;
}

int MPLSensor::proxHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    if (mProxiInitEvent) {
        s->data[0] = -1;
        mProxiInitEvent = false;
    }

    if (mCachedAlsData[1] > PROXIMITY_RANGE) {
        if ((int)s->data[0] == 0)
            return 0;
        s->data[0] = 0;
    } else {
        if ((int)s->data[0] == 5)
            return 0;
        s->data[0] = 5;
    }
    s->timestamp = mAlsSensorTimestamp;

    LOGV_IF(HANDLER_DATA, "HAL:prox data: %+f - %+d - %lld - %d",
            s->data[0], mCachedAlsData[1], (long long)(s->timestamp), 1);

    mMPLLog.logEvents(MPLLogger::SENSOR_PROXIMITY, s->data, 0, s->timestamp);

    return 1;
}

int MPLSensor::proxwHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    if (mProxiWakeInitEvent) {
        s->data[0] = -1;
        mProxiWakeInitEvent = false;
    }

    if (mCachedAlsWakeData[1] > PROXIMITY_RANGE) {
        if ((int)s->data[0] == 0)
            return 0;
        s->data[0] = 0;
    } else {
        if ((int)s->data[0] == 5)
            return 0;
        s->data[0] = 5;
    }
    s->timestamp = mAlsSensorWakeTimestamp;

    LOGV_IF(HANDLER_DATA, "HAL:prox wake data: %+f - %+d - %lld - %d",
            s->data[0], mCachedAlsWakeData[1], (long long)(s->timestamp), 1);

    mMPLLog.logEvents(MPLLogger::SENSOR_PROXIMITY, s->data, 0, s->timestamp);

    return 1;
}

int MPLSensor::headingHandler(sensors_event_t *s)
{
    VHANDLER_LOG;

    int8_t status;
    int update;

    update = output_control(s, adapter_get_sensor_type_heading, 2, s->data,
                &status, Heading);

    LOGV_IF(HANDLER_DATA, "HAL:heading data: %+f - %+f - %+lld - %d",
            s->data[0], s->data[1], (long long)(s->timestamp), update);

    /* TODO missing type in log format
    if (update) {
            mMPLLog.logEvents(MPLLogger::SENSOR_HEADING, s->data, 0, s->timestamp);
    } */

    return update;
}

int MPLSensor::headingwHandler(sensors_event_t *s)
{
    VHANDLER_LOG;

    int8_t status;
    int update;

    update = output_control(s, adapter_get_sensor_type_heading, 2, s->data,
                &status, Heading_Wake);

    LOGV_IF(HANDLER_DATA, "HAL:heading wake data: %+f - %+f - %+lld - %d",
            s->data[0], s->data[1], (long long)(s->timestamp), update);

    /* TODO missing type in log format
    if (update) {
            mMPLLog.logEvents(MPLLogger::SENSOR_HEADING, s->data, 0, s->timestamp);
    } */

    return update;
}

void MPLSensor::getHandle(int32_t handle, int32_t &what, const struct sensor_t * &sensor) const
{
    VFUNC_LOG;

    what = -1;
    sensor = nullptr;

    if (handle < 0 || handle >= ID_NUMBER) {
        LOGV_IF(ENG_VERBOSE, "HAL:handle over = %d",handle);
        return;
    }

    for (uint32_t i = 0; i < mNumSensors; i++) {
        if (handle == mCurrentSensorList[i].handle) {
            what = handle;
            sensor = &mCurrentSensorList[i];
            break;
        }
    }

    LOGI_IF(EXTRA_VERBOSE, "HAL:getHandle - what=%d, sname=%s", what, sensor->name);
    return;
}

int MPLSensor::setDelay(int32_t handle, int64_t ns)
{
    VFUNC_LOG;

    const struct sensor_t *sensor;
    int32_t what = -1;

    if (!mChipDetected)
        return -EINVAL;

    getHandle(handle, what, sensor);
    if (what < 0)
        return -EINVAL;

    if (ns < 0)
        return -EINVAL;

    LOGV_IF(PROCESS_VERBOSE,
            "setDelay : %lld ns, (%.2f Hz)", (long long)ns, NS_PER_SECOND_FLOAT / ns);

    // limit all rates to reasonable ones */
    if (ns < 5000000LL) {
        ns = 5000000LL;
    }

    /* store request rate to mDelays arrary for each sensor */
    int64_t previousDelay = mDelays[what];
    mDelays[what] = ns;

    switch (what) {
        case StepCounter:
            /* set limits of delivery rate of events */
            LOGV_IF(ENG_VERBOSE, "step count rate is not applicable");
            break;
        case StepDetector:
        case SignificantMotion:
            LOGV_IF(ENG_VERBOSE, "Step Detect, SMD, SO rate=%lld ns", (long long)ns);
            break;
        case Gyro:
        case RawGyro:
        case GyroRaw:
        case Accelerometer:
        case AccelerometerRaw:
            // need to update delay since they are different
            // resetDataRates was called earlier
            // LOGV("what=%d mEnabled=%d mDelays[%d]=%lld previousDelay=%lld",
            // what, mEnabled, what, mDelays[what], previousDelay);
            if ((mEnabled & (1LL << what)) && (previousDelay != mDelays[what])) {
                LOGV_IF(ENG_VERBOSE,
                        "HAL:need to update delay due to resetDataRates");
                break;
            }
            for (int i = Gyro;
                    i <= Accelerometer + mCompassSensor->isIntegrated();
                    i++) {
                if (i != what && (mEnabled & (1LL << i)) && ns > mDelays[i]) {
                    LOGV_IF(ENG_VERBOSE,
                            "HAL:ignore delay set due to sensor %d", i);
                    return 0;
                }
            }
            break;
        case MagneticField:
        case RawMagneticField:
        case MagneticFieldRaw:
            // need to update delay since they are different
            // resetDataRates was called earlier
            if ((mEnabled & (1LL << what)) && (previousDelay != mDelays[what])) {
                LOGV_IF(ENG_VERBOSE,
                        "HAL:need to update delay due to resetDataRates");
                break;
            }
            if (mCompassSensor->isIntegrated() &&
                    (((mEnabled & (1LL << Gyro)) && ns > mDelays[Gyro]) ||
                     ((mEnabled & (1LL << RawGyro)) && ns > mDelays[RawGyro]) ||
                     ((mEnabled & (1LL << Accelerometer)) &&
                      ns > mDelays[Accelerometer])) &&
                    !checkBatchEnabled()) {
                /* if request is slower rate, ignore request */
                LOGV_IF(ENG_VERBOSE,
                        "HAL:ignore delay set due to gyro/accel");
                return 0;
            }
            break;
        case Orientation:
        case RotationVector:
        case GameRotationVector:
        case GeomagneticRotationVector:
        case LinearAccel:
        case Gravity:
        case Heading:
            for (int i = 0; i < ID_NUMBER; i++) {
                if (i != what && (mEnabled & (1LL << i)) && ns > mDelays[i]) {
                    LOGV_IF(ENG_VERBOSE,
                            "HAL:ignore delay set due to sensor %d", i);
                    return 0;
                }
            }
            break;
    }

    int res = update_delay();
    return res;
}

int MPLSensor::update_delay(void)
{
    return 0;
}

/**
 *  Should be called after reading at least one of gyro
 *  compass or accel data. (Also okay for handling all of them).
 *  @returns 0, if successful, error number if not.
 */
int MPLSensor::readEvents(sensors_event_t* data, int count)
{
    VHANDLER_LOG;

    inv_execute_on_data();

    int numEventReceived = 0;

    // send additional info on enable
    if (!mAdditionalInfoEnabledVector.empty()) {
        int sendEvent = additionalInfoHandler(mAdditionalInfoEnabledVector[0], data, count);
        if (sendEvent > 0) {
            data += sendEvent;
            count -= sendEvent;
            numEventReceived += sendEvent;
        }
        pthread_mutex_lock(&mHALMutex);
        mAdditionalInfoEnabledVector.erase(mAdditionalInfoEnabledVector.begin());
        pthread_mutex_unlock(&mHALMutex);
    }

    // handle flush complete event
    if (!mFlushSensorEnabledVector.empty()) {
        int sensor = mFlushSensorEnabledVector[0];
        if (!mEmptyDataMarkerDetected) {
            // turn off sensors in data_builder
            mEmptyDataMarkerDetected = 0;
        }
        sensors_event_t temp;
        int sendEvent = metaHandler(sensor, &temp, META_DATA_FLUSH_COMPLETE);
        if (sendEvent == 1 && count > 0) {
            *data++ = temp;
            count--;
            numEventReceived++;
        }
        // send additional info on flush
        sendEvent = additionalInfoHandler(sensor, data, count);
        if (sendEvent > 0) {
            data += sendEvent;
            count -= sendEvent;
            numEventReceived += sendEvent;
        } else {
            // save sensor to send info later
            pthread_mutex_lock(&mHALMutex);
            mAdditionalInfoEnabledVector.push_back(sensor);
            pthread_mutex_unlock(&mHALMutex);
        }
        pthread_mutex_lock(&mHALMutex);
        mFlushSensorEnabledVector.erase(mFlushSensorEnabledVector.begin());
        pthread_mutex_unlock(&mHALMutex);
    }

    if (mSkipReadEvents) {
        mSkipReadEvents = 0;
        return numEventReceived;
    }

    for (int i = 0; i < ID_NUMBER; i++) {
        int update = 0;

        // load up virtual sensors
        if (mEnabledCached & (1LL << i)) {
            update = CALL_MEMBER_FN(this, mHandlers[i])(mPendingEvents + i);
            if (update && (count > 0)) {
#ifdef DIRECT_REPORT
                pthread_mutex_lock(&mHALMutex);
                SensorChannelMux &mux = mSensorChannelMuxes[i];
                int64_t timestamp = mPendingEvents[i].timestamp;
                auto channel = mSensorToChannel.find(i);
                if (channel != mSensorToChannel.end() && mux.isRegistered(0)) {
                    auto &timing = mPollChannelTiming[i];
                    if ((uint64_t)timestamp > timing.dataTimestamp) {
                        // compute effective sensor data period
                        if (timing.dataTimestamp != 0) {
                            timing.dataPeriod = (uint64_t)timestamp - timing.dataTimestamp;
                        }
                        timing.dataTimestamp = timestamp;
                        // send data if interval is large enough or if next sample would be too far away
                        uint64_t delta = timestamp - timing.lastTimestamp;
                        uint64_t period = timing.rateLevel;
                        bool cond1 = intervalLargeEnough(delta, period);
                        bool cond2 = (delta + timing.dataPeriod) > (period + (period >> 3));	// > 112.5% of period
                        if (cond1 || cond2) {
                            *data++ = mPendingEvents[i];
                            count--;
                            numEventReceived++;
                            timing.lastTimestamp = timestamp;
                            int sendEvent = periodicAdditionalInfoHandler(i, data, count);
                            if (sendEvent > 0) {
                                data += sendEvent;
                                count -= sendEvent;
                                numEventReceived += sendEvent;
                            }
                        }
                    }
                } else {
#endif
                    *data++ = mPendingEvents[i];
                    count--;
                    numEventReceived++;
                    int sendEvent = periodicAdditionalInfoHandler(i, data, count);
                    if (sendEvent > 0) {
                        data += sendEvent;
                        count -= sendEvent;
                        numEventReceived += sendEvent;
                    }
#ifdef DIRECT_REPORT
                }
                pthread_mutex_unlock(&mHALMutex);
                sendDirectReportEvent(&mPendingEvents[i], 1);
#endif
            }
        }
    }

    return numEventReceived;
}

// Sensor Placement event
int MPLSensor::additionalInfoSensorPlacement(int handle, unsigned int seq, sensors_event_t* event)
{
    static const signed char identityOrientation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const float *location;
    const signed char *orient;

    if (handle < 0 || handle >= TotalNumSensors) {
        return -1;
    }

    switch (handle) {
    case Gyro:
    case Gyro_Wake:
    case RawGyro:
    case RawGyro_Wake:
    case GyroRaw:
    case EISGyroscope:
    case OisSensor:
        orient = mGyroOrientationMatrix;
        location = mGyroLocation;
        break;
    case MagneticField:
    case MagneticField_Wake:
    case RawMagneticField:
    case RawMagneticField_Wake:
    case MagneticFieldRaw:
        orient = mCompassOrientationMatrix;
        location = mCompassLocation;
        break;
    case Pressure:
    case Pressure_Wake:
        orient = identityOrientation;
        location = mPressureLocation;
        break;
    case Light:
    case Light_Wake:
        orient = identityOrientation;
        location = mLightLocation;
        break;
    case Proximity:
    case Proximity_Wake:
        orient = identityOrientation;
        location = mProximityLocation;
        break;
    // use accelerometer orientation for all other sensors
    default:
        orient = mAccelOrientationMatrix;
        location = mAccelLocation;
        break;
    }

    memset(event, 0, sizeof(*event));
    event->version = sizeof(sensors_event_t);
    event->sensor = handle;
    event->type = SENSOR_TYPE_ADDITIONAL_INFO;
    event->timestamp = seq;
    event->additional_info.type = AINFO_SENSOR_PLACEMENT;
    // inverse orientation matrix, Android system to sensor system
    for (int i = 0; i < 3; ++i) {
        float *data = &event->additional_info.data_float[i * 4];
        data[0] = orient[i];
        data[1] = orient[i + 3];
        data[2] = orient[i + 6];
        data[3] = location[i];
    }

    return 1;
}

// Internal Temperature event
int MPLSensor::additionalInfoInternalTemperature(int handle, unsigned int seq, sensors_event_t *event)
{
    int ret;

    switch (handle) {
    case Gyro:
    case Gyro_Wake:
    case RawGyro:
    case RawGyro_Wake:
    case GyroRaw:
    case EISGyroscope:
    case Accelerometer:
    case Accelerometer_Wake:
    case RawAccelerometer:
    case RawAccelerometer_Wake:
    case AccelerometerRaw:
        // Push temperature payload
        updateImuTemperature();
        memset(event, 0, sizeof(*event));
        event->version = sizeof(sensors_event_t);
        event->sensor = handle;
        event->type = SENSOR_TYPE_ADDITIONAL_INFO;
        event->timestamp = seq;
        event->additional_info.type = AINFO_INTERNAL_TEMPERATURE;
        event->additional_info.data_float[0] = (float)mCachedImuTemperature / 100.f;
        ret = 1;
        break;
    default:
        ret = 0;
        break;
    }

    return ret;
}

int MPLSensor::additionalInfoHandler(int handle, sensors_event_t* data, int count)
{
    VHANDLER_LOG;

    const struct sensor_t *sensor;
    int32_t what = -1;
    sensors_event_t marker, event;
    int numEventReceived = 0;
    unsigned int seq = 0;
    int ret;

    // minimum 3 events: begin - data - end
    if (count < 3)
        return -1;

    // reserve place for begin and end frames
    count -= 2;

    // check if additional info is supported
    getHandle(handle, what, sensor);
    if (what < 0) {
        LOGE("HAL:addInfo sensor %d not found", handle);
        return -EINVAL;
    }
    if (!(sensor->flags & SENSOR_FLAG_ADDITIONAL_INFO)) {
        return 0;
    }

    // initalize marker event
    memset(&marker, 0, sizeof(marker));
    marker.version = sizeof(sensors_event_t);
    marker.sensor = handle;
    marker.type = SENSOR_TYPE_ADDITIONAL_INFO;

    // Push begin frame
    marker.timestamp = seq++;
    marker.additional_info.type = AINFO_BEGIN;
    *data++ = marker;
    numEventReceived++;

    // Sensor placement
    ret = additionalInfoSensorPlacement(handle, seq, &event);
    if (ret == 1) {
        *data++ = event;
        count--;
        numEventReceived++;
        seq++;
    }

    if (count > 0) {
            // Internal Temperature
            ret = additionalInfoInternalTemperature(handle, seq, &event);
            if (ret == 1) {
                *data++ = event;
                count--;
                numEventReceived++;
                seq++;
            }
            mChipTemperatureTimestamps[handle] = getTimestamp();
    }

    // Push end frame
    marker.timestamp = seq++;
    marker.additional_info.type = AINFO_END;
    *data++ = marker;
    numEventReceived++;

    LOGV_IF(HANDLER_DATA, "HAL:additionalInfo %d data", numEventReceived);

    return numEventReceived;
}

int MPLSensor::periodicAdditionalInfoHandler(int handle, sensors_event_t* data, int count)
{
#define INV_CHIP_TEMPERATURE_REPORT_PERIOD_MS       40      // internal temperature @25Hz (40ms)
    VHANDLER_LOG;

    const int64_t now = getTimestamp();
    const struct sensor_t *sensor;
    int32_t what = -1;
    sensors_event_t marker, events[1];
    unsigned int ind = 0;
    unsigned int maxEvents;
    int numEvents = 0;
    int ret;

    // we need at least 3 free data events for returning additional info frames
    if (count < 3) {
        return -1;
    }
    maxEvents = count - 2;

    // check if additional info is supported
    getHandle(handle, what, sensor);
    if (what < 0) {
        LOGE("HAL:addInfo sensor %d not found", handle);
        return -EINVAL;
    }
    if (!(sensor->flags & SENSOR_FLAG_ADDITIONAL_INFO)) {
        return 0;
    }

    // forge internal temperature frame
    if ((now - mChipTemperatureTimestamps[handle]) > ((INV_CHIP_TEMPERATURE_REPORT_PERIOD_MS - 1) * 1000000LL)) {
        ret = additionalInfoInternalTemperature(handle, ind + 1, &events[ind]);
        if (ret == 1) {
            ++ind;
        }
        mChipTemperatureTimestamps[handle] = now;
    }

    // return immediately if there is no data frame
    if (ind == 0) {
        return 0;
    }

    // initalize marker event
    memset(&marker, 0, sizeof(marker));
    marker.version = sizeof(sensors_event_t);
    marker.sensor = handle;
    marker.type = SENSOR_TYPE_ADDITIONAL_INFO;

    // Push begin frame
    marker.timestamp = 0;
    marker.additional_info.type = AINFO_BEGIN;
    *data++ = marker;
    numEvents++;

    // Push data frames
    for (unsigned int i = 0; i < ind && i < maxEvents; ++i) {
        *data++ = events[i];
        numEvents++;
    }

    // Push end frame
    marker.timestamp = ind + 1;
    marker.additional_info.type = AINFO_END;
    *data++ = marker;
    numEvents++;

    return numEvents;
}

// collect data for MPL (but NOT sensor service currently), from driver layer
void MPLSensor::buildMpuEvent(void)
{
    VHANDLER_LOG;

    mSkipReadEvents = 0;
    int64_t latestTimestamp=0;
    size_t nbyte = BYTES_PER_SENSOR;
    int data_format = 0;
    int mask = 0;
    uint64_t sensors = ((mLocalSensorMask & INV_THREE_AXIS_GYRO)? 1 : 0) +
            ((mLocalSensorMask & INV_THREE_AXIS_RAW_GYRO)? 1 : 0) +
            ((mLocalSensorMask & INV_THREE_AXIS_ACCEL)? 1 : 0) +
            (((mLocalSensorMask & INV_THREE_AXIS_COMPASS)
              && mCompassSensor->isIntegrated())? 1 : 0) +
            (((mLocalSensorMask & INV_THREE_AXIS_RAW_COMPASS)
              && mCompassSensor->isIntegrated())? 1 : 0) +
            ((mLocalSensorMask & INV_ONE_AXIS_PRESSURE)? 1 : 0);
    long long data_out[4];
    char *rdata;
    int rsize = 0;
    char outBuffer[32];
    int what_sensor = -1;
    int i;

    int data_available;
    FILE* tempFp = NULL;

    data_available = 0;
    tempFp = fopen(mpu.available_data, "r");
    if (tempFp == NULL) {
        LOGE("HAL:could not open buffer data available");
    } else {
        if (fscanf(tempFp, "%d\n", &data_available) < 0) {
            LOGE("HAL:cannot read available data");
        }
        fclose(tempFp);
    }
 //   LOGI("buffer available data is %d", data_available);

    rsize = read(iio_fd, outBuffer, nbyte);

#ifdef TESTING
    LOGV_IF(INPUT_DATA,
            "HAL:input outBuffer:r=%d, n=%d,"
            "%d, %d, %d, %d, %d, %d, %d, %d",
            (int)rsize, nbyte,
            outBuffer[0], outBuffer[1], outBuffer[2], outBuffer[3],
            outBuffer[4], outBuffer[5], outBuffer[6], outBuffer[7]);
#endif


    rdata = (char *)data_out;

    /* 1. skip one sample if data comes in faster than host can consume.
     * 2. check the available data to decide whether to flush or not.
	   3. We choose half of the ring buffer as the threshold because we
	   need to leave enough space if the host is too slow. We also dont
	   want to skip sample if host is only a little slowed down.
	   4. The current ring buffer size is 32K while one sample is only 32 bytes.
	   Setting 16K bytes as threshold leave enough buffer for system not to
	   be overflowed while keep data not to be dropped easily */
    if ((rsize > 0) && (data_available > (IIO_BUFFER_LENGTH / 2))) {
	    //LOGI("flushing available=%d", data_available);
	    data_format = inv_sensor_parsing(outBuffer, rdata, rsize);
	    mSkipReadEvents = 1;
            return;
    }

    if (rsize < 0) {
        /* IIO buffer might have old data.
           Need to flush it if no sensor is on, to avoid infinite
           read loop.*/
        LOGE("HAL:input data file descriptor not available - (%s)",
                strerror(errno));
        data_format = inv_sensor_parsing(outBuffer, rdata, rsize);
        if (sensors == 0) {
            rsize = read(iio_fd, outBuffer, MAX_READ_SIZE);
            if (rsize > 0) {
                LOGV_IF(ENG_VERBOSE, "HAL:input data flush rsize=%d", (int)rsize);
#ifdef TESTING
                LOGV_IF(INPUT_DATA,
                        "HAL:input outBuffer:r=%d, n=%d,"
                        "%d, %d, %d, %d, %d, %d, %d, %d",
                        (int)rsize, nbyte,
                        outBuffer[0], outBuffer[1], outBuffer[2], outBuffer[3],
                        outBuffer[4], outBuffer[5], outBuffer[6], outBuffer[7]);
#endif
                return;
            }
        }
    }

    data_format = inv_sensor_parsing(outBuffer, rdata, rsize);

    if (data_format == 0) {
        mSkipReadEvents = 1;
        return;
    }

    if (checkValidHeader(data_format) == 0) {
        LOGE("HAL:input invalid data_format 0x%02X", data_format);
        return;
    }

    if (data_format == DATA_FORMAT_STEP) {
        rdata += BYTES_PER_SENSOR;
        latestTimestamp = *((long long*) (rdata));
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "STEP DETECTED:0x%x - ts: %lld", data_format, (long long)latestTimestamp);
        mPedUpdate = data_format;
        mask = DATA_FORMAT_STEP;
        // cancels step bit
        //data_format &= (~DATA_FORMAT_STEP);
    }

    if (data_format == DATA_FORMAT_GYRO_ACCURACY) {
        setAccuracy(0, *((short *) (rdata + 2)));
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "GYRO ACCURACY DETECTED:%d", mGyroAccuracy);
        getDmpGyroBias(true);
    } else if (data_format == DATA_FORMAT_COMPASS_ACCURACY) {
        setAccuracy(2, *((short *) (rdata + 2)));
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "COMPASS ACCURACY DETECTED:%d", mCompassAccuracy);
        getDmpCompassBias();
    } else if (data_format == DATA_FORMAT_ACCEL_ACCURACY) {
        setAccuracy(1, *((short *) (rdata + 2)));
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "ACCEL ACCURACY DETECTED:%d", mAccelAccuracy);
        getDmpAccelBias(true);
    } else if (data_format == DATA_FORMAT_MARKER) {
        what_sensor = *((int *) (rdata + 4));
        pthread_mutex_lock(&mHALMutex);
        mFlushSensorEnabledVector.push_back(what_sensor);
        pthread_mutex_unlock(&mHALMutex);
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "MARKER DETECTED what:%d", what_sensor);
    } else if (data_format == DATA_FORMAT_EMPTY_MARKER) {
        what_sensor = *((int *) (rdata + 4));
        pthread_mutex_lock(&mHALMutex);
        mFlushSensorEnabledVector.push_back(what_sensor);
        pthread_mutex_unlock(&mHALMutex);
        mEmptyDataMarkerDetected = 1;
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "EMPTY MARKER DETECTED what:%d", what_sensor);
    } else if (data_format == DATA_FORMAT_ALS) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "ALS DETECTED:0x%x", data_format);
        mCachedAlsData[0] = *((short *) (rdata + 2));
        mCachedAlsData[1] = *((short *) (rdata + 4));
        rdata += BYTES_PER_SENSOR;
        mAlsSensorTimestamp = *((long long*) (rdata));
        mask = DATA_FORMAT_ALS;
    } else if (data_format == DATA_FORMAT_ALS_WAKE) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "ALS WAKE DETECTED:0x%x", data_format);
        mCachedAlsWakeData[0] = *((short *) (rdata + 2));
        mCachedAlsWakeData[1] = *((short *) (rdata + 4));
        rdata += BYTES_PER_SENSOR;
        mAlsSensorWakeTimestamp = *((long long*) (rdata));
        mask = DATA_FORMAT_ALS_WAKE;
    } else if (data_format == DATA_FORMAT_6_AXIS || data_format == DATA_FORMAT_6_AXIS_WAKE) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "6AXIS DETECTED:0x%x", data_format);
        mCached6AxisQuaternionData[0] = *((int *) (rdata + 4));
        mCached6AxisQuaternionData[1] = *((int *) (rdata + 8));
        mCached6AxisQuaternionData[2] = *((int *) (rdata + 12));
        rdata += QUAT_ONLY_LAST_PACKET_OFFSET;
        m6QuatSensorTimestamp = *((long long*) (rdata));
        mask = DATA_FORMAT_6_AXIS;
    } else if (data_format == DATA_FORMAT_LPQ) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "3AXIS DETECTED:0x%x", data_format);
        mCachedLPQData[0] = *((int *) (rdata + 4));
        mCachedLPQData[1] = *((int *) (rdata + 8));
        mCachedLPQData[2] = *((int *) (rdata + 12));
        rdata += QUAT_ONLY_LAST_PACKET_OFFSET;
        mLPQTimestamp = *((long long*) (rdata));
        mask = DATA_FORMAT_LPQ;
    } else if (data_format == DATA_FORMAT_9_AXIS || data_format == DATA_FORMAT_9_AXIS_WAKE) {
#if DEBUG_TIME_PROFILE
        prevtime = getTimestamp();
#endif
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "9AXIS DETECTED:0x%x", data_format);
        mHeadingAccuracy = *((short *) (rdata + 2));
        mCached9AxisQuaternionData[0] = *((int *) (rdata + 4));
        mCached9AxisQuaternionData[1] = *((int *) (rdata + 8));
        mCached9AxisQuaternionData[2] = *((int *) (rdata + 12));
        rdata += QUAT_ONLY_LAST_PACKET_OFFSET;
        mQuatSensorTimestamp = *((long long*) (rdata));
        mask = DATA_FORMAT_9_AXIS;
    } else if (data_format == DATA_FORMAT_GEOMAG || data_format == DATA_FORMAT_GEOMAG_WAKE) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "GEOMAG DETECTED:0x%x", data_format);
        mHeadingAccuracy = *((short *) (rdata + 2));
        mCachedGeomagData[0] = *((int *) (rdata + 4));
        mCachedGeomagData[1] = *((int *) (rdata + 8));
        mCachedGeomagData[2] = *((int *) (rdata + 12));
        rdata += QUAT_ONLY_LAST_PACKET_OFFSET;
        mGeoQuatSensorTimestamp = *((long long*)
                (rdata));
        mask = DATA_FORMAT_GEOMAG;
    } else if (data_format == DATA_FORMAT_STEP_COUNT) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "STEP COUNTER DETECTED:0x%x", data_format);
        mStepCount = *((uint32_t *) (rdata + 4));
        rdata += BYTES_PER_SENSOR;
        mStepSensorTimestamp = *((long long*) (rdata));
        mask = DATA_FORMAT_STEP_COUNT;
    } else if (data_format == DATA_FORMAT_STEP_COUNT_WAKE) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "STEP COUNTER WAKE DETECTED:0x%x", data_format);
        mStepCountWake = *((uint32_t *) (rdata + 4));
        rdata += BYTES_PER_SENSOR;
        mStepSensorWakeTimestamp = *((long long*) (rdata));
        mask = DATA_FORMAT_STEP_COUNT_WAKE;
    } else if (data_format == DATA_FORMAT_STEP_DETECT) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "STEP DETECTOR DETECTED:0x%x", data_format);
        rdata += BYTES_PER_SENSOR;
        mStepSensorTimestamp = *((long long*) (rdata));
        mask = DATA_FORMAT_STEP_DETECT;
        mPedUpdate = data_format;
    } else if (data_format == DATA_FORMAT_STEP_DETECT_WAKE) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "STEP DETECTOR WAKE DETECTED:0x%x", data_format);
        rdata += BYTES_PER_SENSOR;
        mStepSensorTimestamp = *((long long*) (rdata));
        mask = DATA_FORMAT_STEP_DETECT;
        mPedWakeUpdate |= DATA_FORMAT_STEP_DETECT_WAKE;
    } else if (data_format == DATA_FORMAT_TAP) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "TAP DETECTED:0x%x", data_format);
        mCachedTapData[0] = *((short *) (rdata + 2));
        mCachedTapData[1] = *((short *) (rdata + 4));
        mCachedTapData[1] = *((short *) (rdata + 6));
        rdata += BYTES_PER_SENSOR;
        mTapTimestamp = *((long long*) (rdata));
        mask = DATA_FORMAT_TAP;
        mTapUpdate = 1;
    } else if (data_format == DATA_FORMAT_RAW_GYRO ||
            data_format == DATA_FORMAT_RAW_GYRO_WAKE) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "RAW GYRO DETECTED:0x%x", data_format);
        mCachedGyroData[0] = *((int *) (rdata + 4));
        mCachedGyroData[1] = *((int *) (rdata + 8));
        mCachedGyroData[2] = *((int *) (rdata + 12));

        if (mCalibrationMode & mpl_gyro_cal) {
            mRawGyroCustom[0] = mCachedGyroData[0];
            mRawGyroCustom[1] = mCachedGyroData[1];
            mRawGyroCustom[2] = mCachedGyroData[2];
        }

        if (!mOisEnabled) {
            LOGV_IF(ENG_VERBOSE && INPUT_DATA, "Apply gyro bias for UI");
            for (i = 0; i < 3; i++)
                mCachedGyroData[i] += mGyroBiasUiMode[i];
        }
        rdata += QUAT_ONLY_LAST_PACKET_OFFSET;
        mGyroSensorTimestamp = *((long long*) (rdata));
        mask = DATA_FORMAT_RAW_GYRO;
    } else if (data_format == DATA_FORMAT_GYRO || data_format == DATA_FORMAT_GYRO_WAKE) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "CAL GYRO DETECTED:0x%x", data_format);
        mCachedCalGyroData[0] = *((int *) (rdata + 4));
        mCachedCalGyroData[1] = *((int *) (rdata + 8));
        mCachedCalGyroData[2] = *((int *) (rdata + 12));
        rdata += QUAT_ONLY_LAST_PACKET_OFFSET;
        mGyroSensorTimestamp = *((long long*) (rdata));
        mask = DATA_FORMAT_GYRO;
    } else if (data_format == DATA_FORMAT_ACCEL ||
            data_format == DATA_FORMAT_ACCEL_WAKE) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "ACCEL DETECTED:0x%x", data_format);
        mCachedAccelData[0] = *((int *) (rdata + 4));
        mCachedAccelData[1] = *((int *) (rdata + 8));
        mCachedAccelData[2] = *((int *) (rdata + 12));
        if (mCalibrationMode & mpl_accel_cal) {
            mRawAccelCustom[0] = mCachedAccelData[0];
            mRawAccelCustom[1] = mCachedAccelData[1];
            mRawAccelCustom[2] = mCachedAccelData[2];
        }
        if (!mOisEnabled) {
            /* accel is always continous mode if gyro is enabled */
            if (((GYRO_MPL_SENSOR & mLocalSensorMask) == 0)
                    && ((AUX_MPL_SENSOR & mLocalSensorMask) == 0)) {
                LOGV_IF(ENG_VERBOSE && INPUT_DATA, "Apply accel bias for UI");
                for (i = 0; i < 3; i++)
                    mCachedAccelData[i] += mAccelBiasUiMode[i];
            }
        }
        rdata += QUAT_ONLY_LAST_PACKET_OFFSET;
        mAccelSensorTimestamp = *((long long*) (rdata));
        mask = DATA_FORMAT_ACCEL;
    } else if (data_format == DATA_FORMAT_RAW_COMPASS ||
            data_format == DATA_FORMAT_RAW_COMPASS_WAKE) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "RAW COMPASS DETECTED:0x%x", data_format);
        if (mCompassSensor->isIntegrated()) {
            mCachedCompassData[0] = *((int *) (rdata + 4));
            mCachedCompassData[1] = *((int *) (rdata + 8));
            mCachedCompassData[2] = *((int *) (rdata + 12));

            mRawMagCustom[0] = (mCachedCompassData[0] >> 16);
            mRawMagCustom[1] = (mCachedCompassData[1] >> 16);
            mRawMagCustom[2] = (mCachedCompassData[2] >> 16);

            rdata += QUAT_ONLY_LAST_PACKET_OFFSET;
            mCompassTimestamp = *((long long*) (rdata));
            mask = DATA_FORMAT_RAW_COMPASS;
        }
    } else if (data_format == DATA_FORMAT_COMPASS || data_format == DATA_FORMAT_COMPASS_WAKE) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "COMPASS DETECTED:0x%x", data_format);
        if (mCompassSensor->isIntegrated()) {
            mCachedCompassData[0] = *((int *) (rdata + 4));
            mCachedCompassData[1] = *((int *) (rdata + 8));
            mCachedCompassData[2] = *((int *) (rdata + 12));
            rdata += QUAT_ONLY_LAST_PACKET_OFFSET;
            mCompassTimestamp = *((long long*) (rdata));
            mask = DATA_FORMAT_COMPASS;
        }
    } else if (data_format == DATA_FORMAT_PRESSURE) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "PRESSURE DETECTED:0x%x", data_format);
        if (mPressureSensor->isIntegrated()) {
            mCachedPressureData =
                ((*((short *)(rdata + 4))) << 16) +
                (*((unsigned short *) (rdata + 6))); // Pa x 100
            rdata += BYTES_PER_SENSOR;
            mPressureTimestamp = *((long long*) (rdata));
            if (mCachedPressureData != 0) {
                mask = DATA_FORMAT_PRESSURE;
            }
        }
    } else if (data_format == DATA_FORMAT_PRESSURE_WAKE) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "PRESSURE WAKE DETECTED:0x%x", data_format);
        if (mPressureSensor->isIntegrated()) {
            mCachedPressureWakeData =
                ((*((short *)(rdata + 4))) << 16) +
                (*((unsigned short *) (rdata + 6)));
            rdata += BYTES_PER_SENSOR;
            mPressureWakeTimestamp = *((long long*) (rdata));
            if (mCachedPressureWakeData != 0) {
                mask = DATA_FORMAT_PRESSURE_WAKE;
            }
        }
    } else if (data_format == DATA_FORMAT_EIS_GYROSCOPE) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "EIS DETECTED:0x%x", data_format);
        mCachedEisData[0] = *((int *) (rdata + 4));
        mCachedEisData[1] = *((int *) (rdata + 8));
        mCachedEisData[2] = *((int *) (rdata + 12));
        mCachedEisData[3] = *((int *) (rdata + 16));
        rdata += BYTES_QUAT_DATA;
        mEisTimestamp = *((long long*) (rdata));
        mask = DATA_FORMAT_EIS_GYROSCOPE;
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "EIS DATA [%d] [%d] [%d] [%d] [%lld]",mCachedEisData[0], mCachedEisData[1], mCachedEisData[2], mCachedEisData[3], (long long)mEisTimestamp);
    } else if (data_format == DATA_FORMAT_EIS_AUTHENTICATION) {
        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "EIS AUTH DETECTED:0x%x", data_format);
        mCachedEisAuthenticationData[0] = *((int *) (rdata + 4));
        rdata += 8;
        mask = DATA_FORMAT_EIS_AUTHENTICATION;

        LOGV_IF(ENG_VERBOSE && INPUT_DATA, "EIS AUTH DATA [%d] [%d] [%d] [%d] [%lld]",mCachedEisAuthenticationData[0], mCachedEisAuthenticationData[1], mCachedEisAuthenticationData[2], mCachedEisAuthenticationData[3], (long long)mEisAuthenticationTimestamp);
    } else {
        LOGI("Data format [%d]",data_format);
    }

    /* handle data read */
    if (mask == DATA_FORMAT_GYRO || mask == DATA_FORMAT_RAW_GYRO) {
        int status = 0;
        updateImuTemperature();
        if (mask == DATA_FORMAT_GYRO) {
            status |= INV_CALIBRATED;
            LOGV_IF(INPUT_DATA,
                    "HAL:input build_gyro_cal: %+8d %+8d %+8d - %lld",
                    mCachedCalGyroData[0], mCachedCalGyroData[1],
                    mCachedCalGyroData[2], (long long)mGyroSensorTimestamp);
            adapter_build_gyro_dmp(mCachedCalGyroData, mCachedImuTemperature, status, mGyroSensorTimestamp);
        } else {
            LOGV_IF(INPUT_DATA,
                    "HAL:input build_gyro: %+8d %+8d %+8d - %lld",
                    mCachedGyroData[0], mCachedGyroData[1],
                    mCachedGyroData[2], (long long)mGyroSensorTimestamp);
            mMPLLog.logEvents(MPLLogger::SENSOR_RAW_GYROSCOPE, mCachedGyroData, mGyroSensorTimestamp);
            adapter_build_gyro_dmp(mCachedGyroData, mCachedImuTemperature, status, mGyroSensorTimestamp);

            if (mCustomSensorPresent) {
                if (!(mCalibrationMode & mpl_gyro_cal))
                    read_sysfs_int_array(mpu.gyro_raw_data, mRawGyroCustom);
                LOGV_IF(INPUT_DATA,
                    "HAL:input build_gyro(reg): %+8d %+8d %+8d - %lld",
                mRawGyroCustom[0], mRawGyroCustom[1],
                mRawGyroCustom[2], (long long)mGyroSensorTimestamp);
            }
        }
        latestTimestamp = mGyroSensorTimestamp;
    }

    if (mask == DATA_FORMAT_EIS_GYROSCOPE) {
        updateImuTemperature();
        adapter_build_eis_gyro(mCachedEisData, mCachedImuTemperature, INV_CALIBRATED, mEisTimestamp);
        LOGV_IF(INPUT_DATA,
                "HAL:input build_Eis: %+8d %+8d %+8d %d - %lld",
                mCachedEisData[0], mCachedEisData[1],
                mCachedEisData[2], mCachedEisData[3], (long long)mEisTimestamp);
        latestTimestamp = mEisTimestamp;
        mEisUpdate = 1;
    }

    if (mask == DATA_FORMAT_EIS_AUTHENTICATION) {
        adapter_build_eis_authentication(mCachedEisAuthenticationData, INV_CALIBRATED, mEisAuthenticationTimestamp);
        LOGV_IF(INPUT_DATA,
                "HAL:input build_Eis_Authentication: %+8d %+8d %+8d %d - %lld",
                mCachedEisAuthenticationData[0], mCachedEisAuthenticationData[1],
                mCachedEisAuthenticationData[2], mCachedEisAuthenticationData[3], (long long)mEisAuthenticationTimestamp);
        latestTimestamp = mEisAuthenticationTimestamp;
        mEisAuthenticationUpdate = 1;
    }

    if (mask == DATA_FORMAT_ACCEL) {
        updateImuTemperature();
        mMPLLog.logEvents(MPLLogger::SENSOR_RAW_ACCELEROMETER, mCachedAccelData, mAccelSensorTimestamp);
        adapter_build_accel_dmp(mCachedAccelData, mCachedImuTemperature, INV_CALIBRATED, mAccelSensorTimestamp);
        LOGV_IF(INPUT_DATA,
                "HAL:input build_accel: %+8d %+8d %+8d - %lld",
                mCachedAccelData[0], mCachedAccelData[1], mCachedAccelData[2], (long long)mAccelSensorTimestamp);
        if (mCustomSensorPresent) {
            if (!(mCalibrationMode & mpl_accel_cal))
                read_sysfs_int_array(mpu.accel_raw_data, mRawAccelCustom);
            LOGV_IF(INPUT_DATA,
                    "HAL:input build_accel(reg): %+8d %+8d %+8d - %lld",
                    mRawAccelCustom[0],
                    mRawAccelCustom[1],
                    mRawAccelCustom[2],
                    (long long)mAccelSensorTimestamp);
        }
        latestTimestamp = mAccelSensorTimestamp;
    }

    if (((mask == DATA_FORMAT_RAW_COMPASS) || (mask == DATA_FORMAT_COMPASS)) &&
                                     mCompassSensor->isIntegrated()) {
        int status = 0;
        if (mask == DATA_FORMAT_COMPASS) {
            status |= INV_CALIBRATED;
            status |= mCompassAccuracy & 3; // apply accuracy from DMP
        } else {
            status |= (INV_RAW_DMP | INV_CALIBRATED);
        }
        if (mCompassSensor->providesCalibration()) {
            status = mCompassSensor->getAccuracy(); // Always return 0
            status |= INV_CALIBRATED;
        }
        mMPLLog.logEvents(MPLLogger::SENSOR_RAW_MAGNETOMETER, mCachedCompassData, mCompassTimestamp);
        adapter_build_compass(mCachedCompassData, status,
                mCompassTimestamp);
        LOGV_IF(INPUT_DATA,
                "HAL:input build_compass: %+8d %+8d %+8d - %lld",
                mCachedCompassData[0], mCachedCompassData[1],
                mCachedCompassData[2], (long long)mCompassTimestamp);
        if (mCustomSensorPresent && (mask == DATA_FORMAT_RAW_COMPASS)) {
            if (!(mCalibrationMode & mpl_compass_cal))
                read_sysfs_int_array(mpu.compass_raw_data, mRawMagCustom);
            LOGV_IF(INPUT_DATA,
                    "HAL:input build_compass(reg): %+8d %+8d %+8d - %lld",
                    mRawMagCustom[0],
                    mRawMagCustom[1],
                    mRawMagCustom[2],
                    (long long)mCompassTimestamp);
        }
        latestTimestamp = mCompassTimestamp;
    }

    if ((mask == DATA_FORMAT_ALS) && mLightSensor->isIntegrated()) {
        if (mLocalSensorMask & INV_ONE_AXIS_LIGHT) {
            latestTimestamp = mAlsSensorTimestamp;
            LOGV_IF(INPUT_DATA,
                    "HAL:input build_light: %+8d - %lld",
                    mCachedAlsData[0], (long long)mAlsSensorTimestamp);
            LOGV_IF(INPUT_DATA,
                    "HAL:input build_prox: %+8d - %lld",
                    mCachedAlsData[1], (long long)mAlsSensorTimestamp);
        }
    }

    if ((mask == DATA_FORMAT_ALS_WAKE) && mLightSensor->isIntegrated()) {
        if (mLocalSensorMask & INV_ONE_AXIS_LIGHT_WAKE) {
            latestTimestamp = mAlsSensorWakeTimestamp;
            LOGV_IF(INPUT_DATA,
                    "HAL:input build_light wake: %+8d - %lld",
                    mCachedAlsWakeData[0], (long long)mAlsSensorWakeTimestamp);
            LOGV_IF(INPUT_DATA,
                    "HAL:input build_prox wake: %+8d - %lld",
                    mCachedAlsWakeData[1], (long long)mAlsSensorWakeTimestamp);
        }
    }

    if (mask == DATA_FORMAT_6_AXIS) {
        /* if bias was applied to DMP bias,
           set status bits to disable gyro bias cal */
        int status = 0;
        if (mGyroBiasApplied == true) {
            LOGV_IF(0, "HAL:input dmp bias is used");
            status |= INV_QUAT_6AXIS;
        }
        status |= INV_CALIBRATED | INV_QUAT_6AXIS | INV_QUAT_3ELEMENT; /* default 32 (16/32bits) */
        adapter_build_quat(mCached6AxisQuaternionData,
                status,
                m6QuatSensorTimestamp);
        LOGV_IF(INPUT_DATA,
                "HAL:input build_quat-6x: %+8d %+8d %+8d - %lld",
                mCached6AxisQuaternionData[0], mCached6AxisQuaternionData[1],
                mCached6AxisQuaternionData[2], (long long)m6QuatSensorTimestamp);
        latestTimestamp = m6QuatSensorTimestamp;
    }

    if (mask == DATA_FORMAT_LPQ) {
        /* if bias was applied to DMP bias,
           set status bits to disable gyro bias cal */
        int status = 0;
        if (mGyroBiasApplied == true) {
            LOGV_IF(0, "HAL:input dmp bias is used");
            status |= INV_QUAT_3AXIS;
        }
        status |= INV_CALIBRATED | INV_QUAT_3AXIS; /* default 32 (16/32bits) */
        adapter_build_quat(mCachedLPQData,
                status,
                mLPQTimestamp);
        LOGV_IF(INPUT_DATA,
                "HAL:input build_quat-3x: %+8d %+8d %+8d - %lld",
                mCachedLPQData[0], mCachedLPQData[1],
                mCachedLPQData[2], (long long)mLPQTimestamp);
        latestTimestamp = mLPQTimestamp;
    }

    if (mask == DATA_FORMAT_9_AXIS) {
        int status = 0;
        if (mGyroBiasApplied == true) {
            LOGV_IF(0, "HAL:input dmp bias is used");
            status |= INV_QUAT_9AXIS;
        }
        status |= INV_CALIBRATED | INV_QUAT_9AXIS | INV_QUAT_3ELEMENT; /* default 32 (16/32bits) */
        adapter_build_quat(mCached9AxisQuaternionData,
                status,
                mQuatSensorTimestamp);
        LOGV_IF(INPUT_DATA,
                "HAL:input build_quat-9x: %+8d %+8d %+8d - %d - %lld",
                mCached9AxisQuaternionData[0], mCached9AxisQuaternionData[1],
                mCached9AxisQuaternionData[2], mHeadingAccuracy, (long long)mQuatSensorTimestamp);
        latestTimestamp = mQuatSensorTimestamp;
    }

    if (mask == DATA_FORMAT_GEOMAG) {
        int status = 0;
        if (mGyroBiasApplied == true) {
            LOGV_IF(0, "HAL:input dmp bias is used");
            status |= INV_GEOMAG;
        }
        status |= INV_CALIBRATED | INV_GEOMAG | INV_QUAT_3ELEMENT; /* default 32
                                                                      (16/32bits) */
        adapter_build_quat(mCachedGeomagData,
                status,
                mGeoQuatSensorTimestamp);
        LOGV_IF(INPUT_DATA,
                "HAL:input build_quat-geomag: %+8d %+8d %+8d - %d - %lld",
                mCachedGeomagData[0],
                mCachedGeomagData[1],
                mCachedGeomagData[2],
                mHeadingAccuracy,
                (long long)mGeoQuatSensorTimestamp);
        latestTimestamp = mGeoQuatSensorTimestamp;
    }

    if ((mask == DATA_FORMAT_PRESSURE) && mPressureSensor->isIntegrated()) {
        int status = 0;
        if (mLocalSensorMask & INV_ONE_AXIS_PRESSURE) {
            latestTimestamp = mPressureTimestamp;
            mPressureUpdate = 1;
            adapter_build_pressure(mCachedPressureData,
                    status,
                    mPressureTimestamp);
            LOGV_IF(INPUT_DATA,
                    "HAL:input build_pressure: %+8d - %lld",
                    mCachedPressureData, (long long)mPressureTimestamp);
        }
    }

    if ((mask == DATA_FORMAT_PRESSURE_WAKE) && mPressureSensor->isIntegrated()) {
        int status = 0;
        if (mLocalSensorMask & INV_ONE_AXIS_PRESSURE_WAKE) {
            latestTimestamp = mPressureWakeTimestamp;
            mPressureWakeUpdate = 1;
            adapter_build_pressure(mCachedPressureWakeData,
                    status,
                    mPressureWakeTimestamp);
            LOGV_IF(INPUT_DATA,
                    "HAL:input build_pressure wake: %+8d - %lld",
                    mCachedPressureWakeData, (long long)mPressureWakeTimestamp);
        }
    }

    /* take the latest timestamp */
    if (mask == DATA_FORMAT_STEP) {
        /* work around driver output duplicate step detector bit */
        if (latestTimestamp > mStepSensorTimestamp) {
            mStepSensorTimestamp = latestTimestamp;
            LOGV_IF(INPUT_DATA,
                    "HAL:input build step: 1 - %lld", (long long)mStepSensorTimestamp);
        } else {
            mPedUpdate = 0;
        }
    }

    if (mask == DATA_FORMAT_STEP_COUNT) {
        LOGV_IF(INPUT_DATA,
                "HAL:input build step count: %lld - %lld", (long long)mStepCount, (long long)mStepSensorTimestamp);
    }

    if (mask == DATA_FORMAT_STEP_COUNT_WAKE) {
        LOGV_IF(INPUT_DATA,
                "HAL:input build step count: %lld - %lld", (long long)mStepCountWake, (long long)mStepSensorWakeTimestamp);
    }

    if (mask == DATA_FORMAT_STEP_DETECT) {
        LOGV_IF(INPUT_DATA,
                "HAL:input build step detector %lld", (long long)mStepSensorTimestamp);
    }

    if (mask == DATA_FORMAT_STEP_DETECT_WAKE) {
        LOGV_IF(INPUT_DATA,
                "HAL:input build step wake detector %lld", (long long)mStepSensorTimestamp);
    }

    if (mask == DATA_FORMAT_TAP) {
        LOGV_IF(INPUT_DATA,
                "HAL:input build tap %lld", (long long)mTapTimestamp);
    }

}

int MPLSensor::checkValidHeader(unsigned short data_format)
{
    LOGV_IF(ENG_VERBOSE && INPUT_DATA, "check data_format=%x", data_format);

    if (data_format == DATA_FORMAT_STEP ||
            data_format == DATA_FORMAT_COMPASS_ACCURACY ||
            data_format == DATA_FORMAT_ACCEL_ACCURACY ||
            data_format == DATA_FORMAT_GYRO_ACCURACY)
        return 1;

    if ((data_format == DATA_FORMAT_STEP_DETECT) ||
            (data_format == DATA_FORMAT_STEP) ||
            (data_format == DATA_FORMAT_GEOMAG) ||
            (data_format == DATA_FORMAT_STEP_DETECT_WAKE) ||
            (data_format == DATA_FORMAT_6_AXIS) ||
            (data_format == DATA_FORMAT_9_AXIS) ||
            (data_format == DATA_FORMAT_ALS) ||
            (data_format == DATA_FORMAT_COMPASS) ||
            (data_format == DATA_FORMAT_RAW_COMPASS) ||
            (data_format == DATA_FORMAT_GYRO) ||
            (data_format == DATA_FORMAT_RAW_GYRO) ||
            (data_format == DATA_FORMAT_ACCEL) ||
            (data_format == DATA_FORMAT_PRESSURE) ||
            (data_format == DATA_FORMAT_EMPTY_MARKER) ||
            (data_format == DATA_FORMAT_MARKER) ||
            (data_format == DATA_FORMAT_ACCEL_WAKE) ||
            (data_format == DATA_FORMAT_RAW_COMPASS_WAKE) ||
            (data_format == DATA_FORMAT_COMPASS_WAKE) ||
            (data_format == DATA_FORMAT_RAW_GYRO_WAKE) ||
            (data_format == DATA_FORMAT_GYRO_WAKE) ||
            (data_format == DATA_FORMAT_ALS_WAKE) ||
            (data_format == DATA_FORMAT_PRESSURE_WAKE) ||
            (data_format == DATA_FORMAT_6_AXIS_WAKE) ||
            (data_format == DATA_FORMAT_9_AXIS_WAKE) ||
            (data_format == DATA_FORMAT_GEOMAG_WAKE) ||
            (data_format == DATA_FORMAT_STEP_COUNT) ||
            (data_format == DATA_FORMAT_STEP_COUNT_WAKE) ||
            (data_format == DATA_FORMAT_TAP) ||
            (data_format == DATA_FORMAT_EIS_GYROSCOPE) ||
            (data_format == DATA_FORMAT_EIS_AUTHENTICATION) ||
            (data_format == DATA_FORMAT_LPQ))
        return 1;
    else {
        LOGV_IF(ENG_VERBOSE, "bad data_format = %x", data_format);
        return 0;
    }
}

/* use for both MPUxxxx and third party compass */
void MPLSensor::buildCompassEvent(void)
{
    VHANDLER_LOG;

    int done = 0;

    done = mCompassSensor->readSample(mCachedCompassData, &mCompassTimestamp);

    if (done > 0) {
        int status = 0;
        if (mCompassSensor->providesCalibration()) {
            status = mCompassSensor->getAccuracy();
            status |= INV_CALIBRATED;
        }

        if (mLocalSensorMask & INV_THREE_AXIS_COMPASS
                || mLocalSensorMask & INV_THREE_AXIS_RAW_COMPASS
                || mLocalSensorMask & INV_THREE_AXIS_COMPASS_WAKE
                || mLocalSensorMask & INV_THREE_AXIS_RAW_COMPASS_WAKE ) {
            mMPLLog.logEvents(MPLLogger::SENSOR_RAW_MAGNETOMETER, mCachedCompassData, mCompassTimestamp);
            adapter_build_compass(mCachedCompassData, status,
                    mCompassTimestamp);
            LOGV_IF(INPUT_DATA,
                    "HAL:input inv_build_compass: %+8d %+8d %+8d - %lld",
                    mCachedCompassData[0], mCachedCompassData[1],
                    mCachedCompassData[2], (long long)mCompassTimestamp);
            mSkipReadEvents = 0;
        }
    }
}

void MPLSensor::buildPressureEvent(void)
{
    VHANDLER_LOG;

    int done;

    done = mPressureSensor->readSample(&mCachedPressureData, &mPressureTimestamp);

    if (done > 0) {
        int status = 0;
        if (mLocalSensorMask & INV_ONE_AXIS_PRESSURE) {
            mPressureUpdate = 1;
            adapter_build_pressure(mCachedPressureData,
                    status,
                    mPressureTimestamp);
            LOGV_IF(INPUT_DATA,
                    "HAL:input build_pressure: %+8d - %lld",
                    mCachedPressureData, (long long)mPressureTimestamp);
        }
    }
}

int MPLSensor::getFd(void) const
{
    VFUNC_LOG;
    LOGV_IF(EXTRA_VERBOSE, "getFd returning %d", iio_fd);
    return iio_fd;
}

int MPLSensor::getPollTime(void)
{
    VFUNC_LOG;
    return mPollTime;
}

bool MPLSensor::hasPendingEvents(void) const
{
    VFUNC_LOG;
    // if we are using the polling workaround, force the main
    // loop to check for data every time
    return (mPollTime != -1);
}

int MPLSensor::inv_read_temperature(long long *data)
{
    VHANDLER_LOG;

    int count = 0;
    char raw_buf[40];
    int raw = 0;

    long long timestamp = 0;

    memset(raw_buf, 0, sizeof(raw_buf));
    count = read_attribute_sensor(gyro_temperature_fd, raw_buf,
            sizeof(raw_buf));
    if (count < 0) {
        LOGE("HAL:error reading gyro temperature");
        return -1;
    }

    count = sscanf(raw_buf, "%d %lld", &raw, &timestamp);

    if (count < 0) {
        LOGW("HAL:error parsing gyro temperature count=%d", count);
        return -1;
    }

    LOGV_IF(ENG_VERBOSE && INPUT_DATA,
            "HAL:temperature raw = %d, timestamp = %lld, count = %d",
            raw, timestamp, count);
    data[0] = raw; // degrees Celsius scaled by 100
    data[1] = timestamp;

    return 0;
}

int MPLSensor::updateImuTemperature()
{
    VFUNC_LOG;

    const int64_t now = getTimestamp();
    const int64_t period = (40LL - 1LL) * 1000000LL;    // 40ms (25Hz), 1ms resolution

    if ((now - mImuTemperatureTimestamp) > period) {
        long long temperature[2];
        float val;
        int ret;

        ret = inv_read_temperature(temperature);
        if (ret < 0) {
            return ret;
        }
        mCachedImuTemperature = temperature[0];
        mImuTemperatureTimestamp = temperature[1];

        val = (float)mCachedImuTemperature / 100.0f;
        mMPLLog.logEvents(MPLLogger::SENSOR_TEMPERATURE, &val, 0, mImuTemperatureTimestamp);
    }

    return 0;
}

int MPLSensor::inv_read_sensor_bias(int fd, int *data)
{
    VFUNC_LOG;

    if (fd == -1) {
        return -1;
    }

    char buf[50];
    char x[15], y[15], z[15];

    memset(buf, 0, sizeof(buf));
    int count = read_attribute_sensor(fd, buf, sizeof(buf));
    if (count < 1) {
        LOGE("HAL:Error reading gyro bias");
        return -1;
    }
    count = sscanf(buf, "%[^','],%[^','],%[^',']", x, y, z);
    if (count) {
        /* scale appropriately for MPL */
        LOGV_IF(ENG_VERBOSE,
                "HAL:pre-scaled bias: X:Y:Z (%d, %d, %d)",
                atoi(x), atoi(y), atoi(z));

        data[0] = (int)(atoi(x) / 10000 * (1L << 16));
        data[1] = (int)(atoi(y) / 10000 * (1L << 16));
        data[2] = (int)(atoi(z) / 10000 * (1L << 16));

        LOGV_IF(ENG_VERBOSE,
                "HAL:scaled bias: X:Y:Z (%d, %d, %d)",
                data[0], data[1], data[2]);
    }
    return 0;
}

int MPLSensor::inject_sensor_data(const sensors_event_t* data)
{
    char out_data[16] = {0, };
    int size;
    FILE* fp = NULL;

    /*
    LOGE("HAL sensor_event handle=%d ts=%lld data=%.2f, %.2f, %.2f %.2f %.2f %.2f", data->sensor, data->timestamp,
            data->data[0], data->data[1], data->data[2], data->data[3], data->data[4], data->data[5]);
    */

    inv_android_to_dmp((sensors_event_t*)data, out_data, &size);

    switch(data->type) {
        case SENSOR_TYPE_ACCELEROMETER :
            fp = fopen(mpu.misc_bin_poke_accel, "wb");
            break;
        case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
            fp = fopen(mpu.misc_bin_poke_gyro, "wb");
            break;
        case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
            fp = fopen(mpu.misc_bin_poke_mag, "wb");
            break;
        default :
            return -1;
    }

    if (fp == NULL) {
        LOGE("HAL: cannot open injection sysfs");
        return -1;
    }

    if (fwrite(out_data, 1, 12, fp) == 0) {
        LOGE("HAL: write to data injection sysfs is failed");
        fclose(fp);
        return -1;
    }

    fclose(fp);

    return 0;
}

int MPLSensor::isDataInjectionSupported()
{
    FILE *fp = NULL;

    fp = fopen(mpu.info_poke_mode, "r");

    if (fp == NULL) {
        LOGI("HAL: Injection is not supported");
        return -1;
    }

    LOGI("HAL: Injection is supported");

    fclose(fp);
    return 0;
}

void MPLSensor::setDataInjectionMode(int mode)
{
    write_sysfs_int(mpu.info_poke_mode, mode);
}

void MPLSensor::fillSensorWith1K()
{
    uint32_t i;
    // TODO update only selected sensors
    for (i = 0; i < mNumSensors; i++)
        mCurrentSensorList[i].minDelay = 1000;

}

/** fill in the sensor list based on which sensors are configured.
 *  return the number of configured sensors.
 *  parameter list must point to a memory region
 *  parameter len gives the length of the buffer pointed to by list
 */
int MPLSensor::populateSensorList(struct sensor_t *list, int len)
{
    VFUNC_LOG;
    unsigned int i;
    size_t listSize;
    int maxNumSensors;
    struct sensor_t *selectedList;

    /* fill sensor list according to sensor type */
    if (strcmp(chip_ID, "ICM20608D") == 0) {
        selectedList = s20608DSensorList;
        listSize = sizeof(s20608DSensorList);
        LOGI("ICM 20608D/20609/20689 sensor list is used");
    } else if (strcmp(chip_ID, "ICM20602") == 0) {
        if (mCompassSensor->isCompassSensorPresent()) {
            selectedList = s20602SensorWithCompassList;
            listSize = sizeof(s20602SensorWithCompassList);
            LOGI("ICM 20602 sensor list is used");
        } else {
            selectedList = s20602SensorList;
            listSize = sizeof(s20602SensorList);
            LOGI("ICM 20602 sensor list is used (without compass)");
        }
    } else if (strcmp(chip_ID, "ICM20690") == 0) {
        if (mCompassSensor->isCompassSensorPresent()) {
            selectedList = s20690SensorList;
            listSize = sizeof(s20690SensorList);
            LOGI("ICM 20690 sensor list is used");
        } else {
            selectedList = s20690WithoutCompassSensorList;
            listSize = sizeof(s20690WithoutCompassSensorList);
            LOGI("ICM 20690 sensor list is used (without compass)");
        }
    } else if (strcmp(chip_ID, "IAM20680") == 0) {
        if (mCompassSensor->isCompassSensorPresent()) {
            selectedList = s20680SensorWithCompassList;
            listSize = sizeof(s20680SensorWithCompassList);
            LOGI("IAM 20680 sensor list is used");
        } else {
            selectedList = s20680SensorList;
            listSize = sizeof(s20680SensorList);
            LOGI("IAM 20680 sensor list is used (without compass)");
        }
    } else if (strcmp(chip_ID, "ICM42600") == 0) {
        if (mCompassSensor->isCompassSensorPresent()) {
            selectedList = s42600SensorList;
            listSize = sizeof(s42600SensorList);
            LOGI("ICM 42600 sensor list is used");
        } else {
            selectedList = s42600WithoutCompassSensorList;
            listSize = sizeof(s42600WithoutCompassSensorList);
            LOGI("ICM 42600 sensor list is used (without compass)");
        }
    } else if (strcmp(chip_ID, "ICM43600") == 0) {
        if (mCompassSensor->isCompassSensorPresent()) {
            selectedList = s43600SensorList;
            listSize = sizeof(s43600SensorList);
            LOGI("ICM 43600 sensor list is used");
        } else {
            selectedList = s43600WithoutCompassSensorList;
            listSize = sizeof(s43600WithoutCompassSensorList);
            LOGI("ICM 43600 sensor list is used (without compass)");
        }
    } else if (strcmp(chip_ID, "ICM45600") == 0) {
        if (mCompassSensor->isCompassSensorPresent()) {
            selectedList = s45600SensorList;
            listSize = sizeof(s45600SensorList);
            LOGI("ICM 45600 sensor list is used");
        } else {
            selectedList = s45600WithoutCompassSensorList;
            listSize = sizeof(s45600WithoutCompassSensorList);
            LOGI("ICM 45600 sensor list is used (without compass)");
        }
    } else {
        selectedList = s20648SensorList;
        listSize = sizeof(s20648SensorList);
        LOGI("ICM 20648 sensor list is used");
    }

    if (listSize > sizeof(mCurrentSensorList)) {
        LOGE("HAL:ERR base sensor list is too large");
        return -ENOMEM;
    }

    /* copy selected sensor list to buffer */
    memcpy(mCurrentSensorList, selectedList, listSize);
    mNumSensors = listSize / sizeof(sensor_t);

    maxNumSensors = sizeof(mCurrentSensorList) / sizeof(sensor_t);

    /* add sensor to the list if necessary */
    if (mPressureSensorPresent) {
        LOGI("HAL:Adding presure sensor");
        mNumSensors += mPressureSensor->populateSensorList(mCurrentSensorList + mNumSensors, maxNumSensors - mNumSensors);
    }

    if (mLightSensorPresent) {
        LOGI("HAL:Adding light sensor");
        mNumSensors += mLightSensor->populateSensorList(mCurrentSensorList + mNumSensors, maxNumSensors - mNumSensors);
    }

    if (mCustomSensorPresent) {
        size_t currentSize = sizeof(sCustomSensorList) / sizeof(sensor_t);
        if (maxNumSensors - mNumSensors >= currentSize) {
            LOGI("HAL:Adding custom sensor");
            memcpy(mCurrentSensorList + mNumSensors, sCustomSensorList, currentSize * sizeof(sensor_t));
            mNumSensors += currentSize;
        } else {
            LOGW("HAL:no space to add custom sensor");
        }
        bool magPresent = false;
        for (i = 0; i < mNumSensors; i++) {
            if (mCurrentSensorList[i].handle == SENSORS_RAW_MAGNETIC_FIELD_HANDLE) {
                magPresent = true;
                break;
            }
        }
        if (magPresent) {
            currentSize = sizeof(sCustomMagSensorList) / sizeof(sensor_t);
            if (maxNumSensors - mNumSensors >= currentSize) {
                LOGI("HAL:Adding custom mag sensor");
                memcpy(mCurrentSensorList + mNumSensors, sCustomMagSensorList, currentSize * sizeof(sensor_t));
                mNumSensors += currentSize;
            } else {
                LOGW("HAL:no space to add custom mag sensor");
            }
        }
    }

    /* update the list */

    /* update compass */
    if (mCompassSensor->isCompassSensorPresent()) {
        for (i = 0; i < mNumSensors; i++) {
            if (mCurrentSensorList[i].handle == SENSORS_MAGNETIC_FIELD_HANDLE ||
                mCurrentSensorList[i].handle == SENSORS_RAW_MAGNETIC_FIELD_HANDLE ||
                mCurrentSensorList[i].handle == SENSORS_MAGNETIC_FIELD_WAKEUP_HANDLE ||
                mCurrentSensorList[i].handle == SENSORS_RAW_MAGNETIC_FIELD_WAKEUP_HANDLE) {
                mCompassSensor->fillList(&mCurrentSensorList[i]);
#ifdef DIRECT_REPORT
                fillDirectReportFlags(&mCurrentSensorList[i]);
#endif
            }
        }
    }

    /* update pressure */
    if (mPressureSensorPresent) {
        for (i = 0; i < mNumSensors; i++) {
            if (mCurrentSensorList[i].handle == SENSORS_PRESSURE_HANDLE ||
                mCurrentSensorList[i].handle == SENSORS_PRESSURE_WAKEUP_HANDLE) {
                mPressureSensor->fillList(&mCurrentSensorList[i]);
            }
        }
    }

    /* update light */
    if (mLightSensorPresent) {
        for (i = 0; i < mNumSensors; i++) {
            if (mCurrentSensorList[i].handle == SENSORS_LIGHT_HANDLE ||
                mCurrentSensorList[i].handle == SENSORS_LIGHT_WAKEUP_HANDLE ||
                mCurrentSensorList[i].handle == SENSORS_PROXIMITY_HANDLE ||
                mCurrentSensorList[i].handle == SENSORS_PROXIMITY_WAKEUP_HANDLE) {
                mPressureSensor->fillList(&mCurrentSensorList[i]);
            }
        }
    }

    /* update sensors according to hardware for ICM20648 */
    LOGI("HAL:Update sensor information");
    fillAccel(chip_ID, mCurrentSensorList);
    fillGyro(chip_ID, mCurrentSensorList);
    fillAGMSensors(mCurrentSensorList);
    fillAMSensors(mCurrentSensorList);
    fillAGSensors(mCurrentSensorList);
    fillGestureSensors(mCurrentSensorList);

    /* 1kHz support */
    if (SENSOR_1K_SUPPORT && ((!strcmp(chip_ID, "ICM20602")) || (!strcmp(chip_ID, "ICM20690"))))
        fillSensorWith1K();

    if (len < (int)(sizeof(struct sensor_t) * mNumSensors)) {
        LOGE("HAL:ERR sensor list is too small, not populating");
        return -ENOMEM;
    }

    memcpy(list, mCurrentSensorList, sizeof(struct sensor_t) * mNumSensors);
    LOGI("HAL:%d sensors are populated", mNumSensors);
    fillSensorMaskArray();

    return mNumSensors;
}

void MPLSensor::fillSensorMaskArray()
{
    uint32_t i, j, ind;
    VFUNC_LOG;

    mCurrentSensorMask = new MPLSensor::sensor_mask[ID_NUMBER];
    mSysfsMask         = new MPLSensor::sysfs_mask[ARRAY_SIZE(sysfsId)];

    for (i = 0; i < ID_NUMBER; i++) {
        mCurrentSensorMask[i].sensorMask = 0;
        mCurrentSensorMask[i].engineRateAddr = 0;
    }
    for (i = 0; i < ARRAY_SIZE(sysfsId); i++) {
        mSysfsMask[i].sensorMask = 0;
        mSysfsMask[i].setRate = NULL;
        mSysfsMask[i].enable = NULL;
    }

    for (ind = 0; ind < mNumSensors; ind++) {
        i = mCurrentSensorList[ind].handle;
        mCurrentSensorMask[i].sname = mCurrentSensorList[ind].name;
        mCurrentSensorMask[i].wakeOn = false;
        mCurrentSensorMask[i].timestamp = 0;

        switch (i) {
        case SENSORS_GYROSCOPE_HANDLE:
            if (mCalibrationMode & mpl_gyro_cal) {
                mCurrentSensorMask[i].sensorMask = INV_THREE_AXIS_RAW_GYRO;
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_RAW_GYRO;
            } else {
                mCurrentSensorMask[i].sensorMask = INV_THREE_AXIS_GYRO;
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_GYRO;
            }
            break;
        case SENSORS_RAW_GYROSCOPE_HANDLE:
        case SENSORS_GYROSCOPE_RAW_HANDLE:
            mCurrentSensorMask[i].sensorMask = INV_THREE_AXIS_RAW_GYRO;
            mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_RAW_GYRO;
            if (!(mCalibrationMode & mpl_gyro_cal)) {
                mCurrentSensorMask[i].sensorMask |= INV_THREE_AXIS_GYRO;
            }
            break;
        case SENSORS_ACCELERATION_HANDLE:
        case SENSORS_ACCELERATION_RAW_HANDLE:
        case SENSORS_RAW_ACCELERATION_HANDLE:
            mCurrentSensorMask[i].sensorMask = INV_THREE_AXIS_ACCEL;
            mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_ACCEL;
            break;
        case SENSORS_MAGNETIC_FIELD_HANDLE:
            if (mCalibrationMode & mpl_compass_cal) {
                mCurrentSensorMask[i].sensorMask = INV_THREE_AXIS_RAW_COMPASS;
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_RAW_COMPASS;
            } else {
                mCurrentSensorMask[i].sensorMask = INV_THREE_AXIS_COMPASS;
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_COMPASS;
            }
            break;
        case SENSORS_RAW_MAGNETIC_FIELD_HANDLE:
        case SENSORS_MAGNETIC_FIELD_RAW_HANDLE:
            mCurrentSensorMask[i].sensorMask = INV_THREE_AXIS_RAW_COMPASS;
            mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_RAW_COMPASS;
            if (!(mCalibrationMode & mpl_compass_cal)) {
                mCurrentSensorMask[i].sensorMask |= INV_THREE_AXIS_COMPASS;
            }
            break;
        case SENSORS_ORIENTATION_HANDLE:
        case SENSORS_ROTATION_VECTOR_HANDLE:
        case SENSORS_HEADING_HANDLE:
            if (mCalibrationMode & mpl_quat) {
                mCurrentSensorMask[i].sensorMask = (INV_THREE_AXIS_RAW_GYRO | INV_THREE_AXIS_ACCEL
                                    | INV_THREE_AXIS_RAW_COMPASS);
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_RAW_GYRO;
            } else {
                mCurrentSensorMask[i].sensorMask = VIRTUAL_SENSOR_9AXES_MASK;
                mCurrentSensorMask[i].engineMask = VIRTUAL_SENSOR_9AXES_MASK;
            }
            break;
        case SENSORS_GAME_ROTATION_VECTOR_HANDLE:
            if (mCalibrationMode & mpl_quat) {
                mCurrentSensorMask[i].sensorMask = (INV_THREE_AXIS_RAW_GYRO | INV_THREE_AXIS_ACCEL);
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_RAW_GYRO;
            } else {
                mCurrentSensorMask[i].sensorMask = VIRTUAL_SENSOR_GYRO_6AXES_MASK;
                mCurrentSensorMask[i].engineMask = VIRTUAL_SENSOR_GYRO_6AXES_MASK;
            }
            break;
        case SENSORS_LPQ_HANDLE:
            mCurrentSensorMask[i].sensorMask = VIRTUAL_SENSOR_LPQ_MASK;
            mCurrentSensorMask[i].engineMask = VIRTUAL_SENSOR_LPQ_MASK;
            break;
        case SENSORS_LINEAR_ACCEL_HANDLE:
        case SENSORS_GRAVITY_HANDLE:
            if (mCalibrationMode & mpl_quat) {
                mCurrentSensorMask[i].sensorMask = (INV_THREE_AXIS_RAW_GYRO | INV_THREE_AXIS_ACCEL);
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_RAW_GYRO;
            } else {
                mCurrentSensorMask[i].sensorMask = (VIRTUAL_SENSOR_GYRO_6AXES_MASK | INV_THREE_AXIS_ACCEL);
                mCurrentSensorMask[i].engineMask = VIRTUAL_SENSOR_GYRO_6AXES_MASK;
            }
            break;
        case SENSORS_GEOMAGNETIC_ROTATION_VECTOR_HANDLE:
            if (mCalibrationMode & mpl_quat) {
                mCurrentSensorMask[i].sensorMask = (INV_THREE_AXIS_ACCEL | INV_THREE_AXIS_RAW_COMPASS);
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_ACCEL;
            } else {
                mCurrentSensorMask[i].sensorMask = VIRTUAL_SENSOR_MAG_6AXES_MASK;
                mCurrentSensorMask[i].engineMask = VIRTUAL_SENSOR_MAG_6AXES_MASK;
            }
            break;
        case SENSORS_PRESSURE_HANDLE:
            mCurrentSensorMask[i].sensorMask = INV_ONE_AXIS_PRESSURE;
            mCurrentSensorMask[i].engineMask = INV_ONE_AXIS_PRESSURE;
            break;
        case SENSORS_LIGHT_HANDLE:
            mCurrentSensorMask[i].sensorMask = INV_ONE_AXIS_LIGHT;
            mCurrentSensorMask[i].engineMask = INV_ONE_AXIS_LIGHT;
            break;
        case SENSORS_PROXIMITY_HANDLE:
            mCurrentSensorMask[i].sensorMask = INV_ONE_AXIS_LIGHT;
            mCurrentSensorMask[i].engineMask = INV_ONE_AXIS_LIGHT;
            break;
        case SENSORS_GYROSCOPE_WAKEUP_HANDLE:
            if (mCalibrationMode & mpl_gyro_cal) {
                mCurrentSensorMask[i].sensorMask = INV_THREE_AXIS_RAW_GYRO_WAKE;
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_RAW_GYRO_WAKE;
            } else {
                mCurrentSensorMask[i].sensorMask = INV_THREE_AXIS_GYRO_WAKE;
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_GYRO_WAKE;
            }
            mCurrentSensorMask[i].wakeOn = true;
            break;
        case SENSORS_RAW_GYROSCOPE_WAKEUP_HANDLE:
            mCurrentSensorMask[i].sensorMask = INV_THREE_AXIS_RAW_GYRO_WAKE;
            mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_RAW_GYRO_WAKE;
            if (!(mCalibrationMode & mpl_gyro_cal)) {
                mCurrentSensorMask[i].sensorMask |= INV_THREE_AXIS_GYRO_WAKE;
            }
            mCurrentSensorMask[i].wakeOn = true;
            break;
        case SENSORS_ACCELERATION_WAKEUP_HANDLE:
        case SENSORS_RAW_ACCELERATION_WAKEUP_HANDLE:
            mCurrentSensorMask[i].sensorMask = INV_THREE_AXIS_ACCEL_WAKE;
            mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_ACCEL_WAKE;
            mCurrentSensorMask[i].wakeOn = true;
            break;
        case SENSORS_MAGNETIC_FIELD_WAKEUP_HANDLE:
            if (mCalibrationMode & mpl_compass_cal) {
                mCurrentSensorMask[i].sensorMask = INV_THREE_AXIS_RAW_COMPASS_WAKE;
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_RAW_COMPASS_WAKE;
            } else {
                mCurrentSensorMask[i].sensorMask = INV_THREE_AXIS_COMPASS_WAKE;
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_COMPASS_WAKE;
            }
            mCurrentSensorMask[i].wakeOn = true;
            break;
        case SENSORS_RAW_MAGNETIC_FIELD_WAKEUP_HANDLE:
            mCurrentSensorMask[i].sensorMask = INV_THREE_AXIS_RAW_COMPASS_WAKE;
            mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_RAW_COMPASS_WAKE;
            if (!(mCalibrationMode & mpl_compass_cal)) {
                mCurrentSensorMask[i].sensorMask |= INV_THREE_AXIS_COMPASS_WAKE;
            }
            mCurrentSensorMask[i].wakeOn = true;
            break;
        case SENSORS_ORIENTATION_WAKEUP_HANDLE:
        case SENSORS_ROTATION_VECTOR_WAKEUP_HANDLE:
        case SENSORS_HEADING_WAKEUP_HANDLE:
            if (mCalibrationMode & mpl_quat) {
                mCurrentSensorMask[i].sensorMask =  (INV_THREE_AXIS_RAW_GYRO_WAKE | INV_THREE_AXIS_ACCEL_WAKE
                                    | INV_THREE_AXIS_RAW_COMPASS_WAKE);
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_RAW_GYRO_WAKE;
            } else {
                mCurrentSensorMask[i].sensorMask = VIRTUAL_SENSOR_9AXES_MASK_WAKE;
                mCurrentSensorMask[i].engineMask = VIRTUAL_SENSOR_9AXES_MASK_WAKE;
            }
            mCurrentSensorMask[i].wakeOn = true;
            break;
        case SENSORS_GAME_ROTATION_VECTOR_WAKEUP_HANDLE:
            if (mCalibrationMode & mpl_quat) {
                mCurrentSensorMask[i].sensorMask = (INV_THREE_AXIS_RAW_GYRO_WAKE | INV_THREE_AXIS_ACCEL_WAKE);
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_RAW_GYRO_WAKE;
            } else {
                mCurrentSensorMask[i].sensorMask = VIRTUAL_SENSOR_GYRO_6AXES_MASK_WAKE;
                mCurrentSensorMask[i].engineMask = VIRTUAL_SENSOR_GYRO_6AXES_MASK_WAKE;
            }
            mCurrentSensorMask[i].wakeOn = true;
            break;
        case SENSORS_LINEAR_ACCEL_WAKEUP_HANDLE:
        case SENSORS_GRAVITY_WAKEUP_HANDLE:
            if (mCalibrationMode & mpl_quat) {
                mCurrentSensorMask[i].sensorMask = (INV_THREE_AXIS_RAW_GYRO_WAKE | INV_THREE_AXIS_ACCEL_WAKE);
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_RAW_GYRO_WAKE;
            } else {
                mCurrentSensorMask[i].sensorMask = (VIRTUAL_SENSOR_GYRO_6AXES_MASK_WAKE | INV_THREE_AXIS_ACCEL_WAKE);
                mCurrentSensorMask[i].engineMask = VIRTUAL_SENSOR_GYRO_6AXES_MASK_WAKE;
            }
            mCurrentSensorMask[i].wakeOn = true;
            break;
        case SENSORS_GEOMAGNETIC_ROTATION_VECTOR_WAKEUP_HANDLE:
            if (mCalibrationMode & mpl_quat) {
                mCurrentSensorMask[i].sensorMask = (INV_THREE_AXIS_ACCEL_WAKE
                                    | INV_THREE_AXIS_RAW_COMPASS_WAKE);
                mCurrentSensorMask[i].engineMask = INV_THREE_AXIS_ACCEL_WAKE;
            } else {
                mCurrentSensorMask[i].sensorMask = VIRTUAL_SENSOR_MAG_6AXES_MASK_WAKE;
                mCurrentSensorMask[i].engineMask = VIRTUAL_SENSOR_MAG_6AXES_MASK_WAKE;
            }
            mCurrentSensorMask[i].wakeOn = true;
            break;
        case SENSORS_PRESSURE_WAKEUP_HANDLE:
            mCurrentSensorMask[i].sensorMask = INV_ONE_AXIS_PRESSURE_WAKE;
            mCurrentSensorMask[i].engineMask = INV_ONE_AXIS_PRESSURE_WAKE;
            mCurrentSensorMask[i].wakeOn = true;
            break;
        case SENSORS_LIGHT_WAKEUP_HANDLE:
            mCurrentSensorMask[i].sensorMask = INV_ONE_AXIS_LIGHT_WAKE;
            mCurrentSensorMask[i].engineMask = INV_ONE_AXIS_LIGHT_WAKE;
            mCurrentSensorMask[i].wakeOn = true;
            break;
        case SENSORS_PROXIMITY_WAKEUP_HANDLE:
            mCurrentSensorMask[i].sensorMask = INV_ONE_AXIS_LIGHT_WAKE;
            mCurrentSensorMask[i].engineMask = INV_ONE_AXIS_LIGHT_WAKE;
            mCurrentSensorMask[i].wakeOn = true;
            break;
        default:
            break;
        }
    }

    mNumSysfs = 0;
    for (ind = 0; ind < ARRAY_SIZE(sysfsId); ind++) {
        bool hasSys;

        hasSys = false;
        for (j = 0; j < ID_NUMBER; j++) {
            if (mCurrentSensorMask[j].sensorMask) {
                if (sysfsId[ind] & mCurrentSensorMask[j].sensorMask) {
                    hasSys = true;
                    break;
                }
            }
        }
        if (hasSys) {
            i = mNumSysfs;
            mNumSysfs++;
            mSysfsMask[i].sensorMask = sysfsId[ind];
            mSysfsMask[i].minimumNonBatchRate = NS_PER_SECOND;

            switch (sysfsId[ind]) {
            case INV_THREE_AXIS_GYRO:
                mSysfsMask[i].enable = &MPLSensor::enableGyro;
                mSysfsMask[i].setRate = &MPLSensor::setGyroRate;
                if (mCalibrationMode & (mpl_gyro_cal | mpl_quat))
                    mSysfsMask[i].minimumNonBatchRate = MIN_NON_BATCH_RATE_GYRO;
                break;
            case INV_THREE_AXIS_RAW_GYRO:
                mSysfsMask[i].enable = &MPLSensor::enableRawGyro;
                mSysfsMask[i].setRate = &MPLSensor::setRawGyroRate;
                if (mCalibrationMode & (mpl_gyro_cal | mpl_quat))
                    mSysfsMask[i].minimumNonBatchRate = MIN_NON_BATCH_RATE_GYRO;
                break;
            case INV_THREE_AXIS_ACCEL:
                mSysfsMask[i].enable = &MPLSensor::enableAccel;
                mSysfsMask[i].setRate = &MPLSensor::setAccelRate;
                if (mCalibrationMode & (mpl_accel_cal | mpl_quat))
                    mSysfsMask[i].minimumNonBatchRate = MIN_NON_BATCH_RATE_ACCEL;
                break;
            case INV_THREE_AXIS_COMPASS:
                mSysfsMask[i].enable = &MPLSensor::enableCompass;
                mSysfsMask[i].setRate = &MPLSensor::setMagRate;
                mSysfsMask[i].minimumNonBatchRate = NS_PER_SECOND;
                break;
            case INV_THREE_AXIS_RAW_COMPASS:
                mSysfsMask[i].enable = &MPLSensor::enableRawCompass;
                mSysfsMask[i].setRate = &MPLSensor::setRawMagRate;
                mSysfsMask[i].minimumNonBatchRate = NS_PER_SECOND;
                break;
            case VIRTUAL_SENSOR_9AXES_MASK:
                mSysfsMask[i].enable = &MPLSensor::enable9AxisQuaternion;
                mSysfsMask[i].setRate = &MPLSensor::set9AxesRate;
                break;
            case VIRTUAL_SENSOR_GYRO_6AXES_MASK:
                mSysfsMask[i].enable = &MPLSensor::enableLPQuaternion;
                mSysfsMask[i].setRate = &MPLSensor::set6AxesRate;
                break;
            case VIRTUAL_SENSOR_LPQ_MASK:
                                mSysfsMask[i].enable = &MPLSensor::enableLPQ;
                                mSysfsMask[i].setRate = &MPLSensor::set3AxesRate;
                                break;
            case VIRTUAL_SENSOR_MAG_6AXES_MASK:
                mSysfsMask[i].enable = &MPLSensor::enableCompass6AxisQuaternion;
                mSysfsMask[i].setRate = &MPLSensor::set6AxesMagRate;
                break;
            case INV_ONE_AXIS_PRESSURE:
                mSysfsMask[i].enable = &MPLSensor::enablePressure;
                mSysfsMask[i].setRate = &MPLSensor::setPressureRate;
                break;
            case INV_ONE_AXIS_LIGHT:
                mSysfsMask[i].enable = &MPLSensor::enableLight;
                mSysfsMask[i].setRate = &MPLSensor::setLightRate;
                break;
            case INV_THREE_AXIS_GYRO_WAKE:
                mSysfsMask[i].enable = &MPLSensor::enableGyroWake;
                mSysfsMask[i].setRate = &MPLSensor::setGyroRateWake;
                if (mCalibrationMode & (mpl_gyro_cal | mpl_quat))
                    mSysfsMask[i].minimumNonBatchRate = MIN_NON_BATCH_RATE_GYRO;
                break;
            case INV_THREE_AXIS_RAW_GYRO_WAKE:
                mSysfsMask[i].enable = &MPLSensor::enableRawGyroWake;
                mSysfsMask[i].setRate = &MPLSensor::setRawGyroRateWake;
                if (mCalibrationMode & (mpl_gyro_cal | mpl_quat))
                    mSysfsMask[i].minimumNonBatchRate = MIN_NON_BATCH_RATE_GYRO;
                break;
            case INV_THREE_AXIS_ACCEL_WAKE:
                mSysfsMask[i].enable = &MPLSensor::enableAccelWake;
                mSysfsMask[i].setRate = &MPLSensor::setAccelRateWake;
                if (mCalibrationMode & (mpl_accel_cal | mpl_quat))
                    mSysfsMask[i].minimumNonBatchRate = MIN_NON_BATCH_RATE_ACCEL;
                break;
            case INV_THREE_AXIS_COMPASS_WAKE:
                mSysfsMask[i].enable = &MPLSensor::enableCompassWake;
                mSysfsMask[i].setRate = &MPLSensor::setMagRateWake;
                mSysfsMask[i].minimumNonBatchRate = NS_PER_SECOND;
                break;
            case INV_THREE_AXIS_RAW_COMPASS_WAKE:
                mSysfsMask[i].enable = &MPLSensor::enableRawCompassWake;
                mSysfsMask[i].setRate = &MPLSensor::setRawMagRateWake;
                mSysfsMask[i].minimumNonBatchRate = NS_PER_SECOND;
                break;
            case VIRTUAL_SENSOR_9AXES_MASK_WAKE:
                mSysfsMask[i].enable = &MPLSensor::enable9AxisQuaternionWake;
                mSysfsMask[i].setRate = &MPLSensor::set9AxesRateWake;
                break;
            case VIRTUAL_SENSOR_GYRO_6AXES_MASK_WAKE:
                mSysfsMask[i].enable = &MPLSensor::enableLPQuaternionWake;
                mSysfsMask[i].setRate = &MPLSensor::set6AxesRateWake;
                break;
            case VIRTUAL_SENSOR_MAG_6AXES_MASK_WAKE:
                mSysfsMask[i].enable = &MPLSensor::enableCompass6AxisQuaternionWake;
                mSysfsMask[i].setRate = &MPLSensor::set6AxesMagRateWake;
                break;
            case INV_ONE_AXIS_PRESSURE_WAKE:
                mSysfsMask[i].enable = &MPLSensor::enablePressureWake;
                mSysfsMask[i].setRate = &MPLSensor::setPressureRateWake;
                break;
            case INV_ONE_AXIS_LIGHT_WAKE:
                mSysfsMask[i].enable = &MPLSensor::enableLightWake;
                mSysfsMask[i].setRate = &MPLSensor::setLightRateWake;
                break;
            default:
                break;
            }
        }
    }
    for (i = 0; i < ID_NUMBER; i++) {
        for (j = 0; j < mNumSysfs; j++) {
            if (mSysfsMask[j].sensorMask == mCurrentSensorMask[i].engineMask) {
                LOGI("HAL:sensor=%s, engine=%d, sensorid=%d", mCurrentSensorMask[i].sname.c_str(), j, i);
                mCurrentSensorMask[i].engineRateAddr = &mSysfsMask[j].engineRate;
            }
        }
    }
    LOGI("HAL:Sysfs num=%d, total=%d", mNumSysfs, (int)ARRAY_SIZE(sysfsId));

    return;
}

/* fill accle metadata */
void MPLSensor::fillAccel(const char* accel, struct sensor_t *list)
{
    VFUNC_LOG;

    unsigned int i;

    if (accel == NULL)
        return;

    for (i = 0; i < mNumSensors; i++) {
        if (list[i].handle == SENSORS_ACCELERATION_HANDLE ||
            list[i].handle == SENSORS_RAW_ACCELERATION_HANDLE ||
            list[i].handle == SENSORS_ACCELERATION_WAKEUP_HANDLE ||
            list[i].handle == SENSORS_RAW_ACCELERATION_WAKEUP_HANDLE ||
            list[i].handle == SENSORS_ACCELERATION_RAW_HANDLE) {

            if (strcmp(accel, "ICM20648") == 0) {
                list[i].power = ACCEL_ICM20648_POWER;
                list[i].minDelay = ACCEL_ICM20648_MINDELAY;
                list[i].maxDelay = ACCEL_ICM20648_MAXDELAY;
            } else if (strcmp(accel, "ICM20602") == 0) {
                list[i].power = ACCEL_ICM20602_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = ACCEL_ICM20602_MINDELAY_HIFI;
#else
                list[i].minDelay = ACCEL_ICM20602_MINDELAY;
#endif
                list[i].maxDelay = ACCEL_ICM20602_MAXDELAY;
            } else if (strcmp(accel, "ICM20690") == 0) {
                list[i].power = ACCEL_ICM20690_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = ACCEL_ICM20690_MINDELAY_HIFI;
#else
                list[i].minDelay = ACCEL_ICM20690_MINDELAY;
#endif
                list[i].maxDelay = ACCEL_ICM20690_MAXDELAY;
            } else if (strcmp(accel, "ICM20608D") == 0) {
                list[i].power = ACCEL_ICM20608D_POWER;
                list[i].minDelay = ACCEL_ICM20608D_MINDELAY;
                list[i].maxDelay = ACCEL_ICM20608D_MAXDELAY;
            } else if (strcmp(accel, "IAM20680") == 0) {
                list[i].power = ACCEL_IAM20680_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = ACCEL_IAM20680_MINDELAY_HIFI;
#else
                list[i].minDelay = ACCEL_IAM20680_MINDELAY;
#endif
                list[i].maxDelay = ACCEL_IAM20680_MAXDELAY;
            } else if (strcmp(accel, "ICM42600") == 0) {
                list[i].power = ACCEL_ICM42600_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = ACCEL_ICM42600_MINDELAY_HIFI;
#else
                list[i].minDelay = ACCEL_ICM42600_MINDELAY;
#endif
                list[i].maxDelay = ACCEL_ICM42600_MAXDELAY;
            } else if (strcmp(accel, "ICM43600") == 0) {
                list[i].power = ACCEL_ICM43600_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = ACCEL_ICM43600_MINDELAY_HIFI;
#else
                list[i].minDelay = ACCEL_ICM43600_MINDELAY;
#endif
                list[i].maxDelay = ACCEL_ICM43600_MAXDELAY;
            } else if (strcmp(accel, "ICM45600") == 0) {
                list[i].power = ACCEL_ICM45600_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = ACCEL_ICM45600_MINDELAY_HIFI;
#else
                list[i].minDelay = ACCEL_ICM45600_MINDELAY;
#endif
                list[i].maxDelay = ACCEL_ICM45600_MAXDELAY;
            }

            list[i].maxRange = (float)mAccelScale * GRAVITY_EARTH;
            list[i].resolution = (float)mAccelScale * GRAVITY_EARTH / 32768.f;
#ifdef DIRECT_REPORT
            fillDirectReportFlags(&list[i]);
#endif
        }
    }
}

/* fill gyro metadata */
void MPLSensor::fillGyro(const char* gyro, struct sensor_t *list)
{
    VFUNC_LOG;

    unsigned int i;

    if (gyro == NULL)
        return;

    for (i = 0; i < mNumSensors; i++) {
        if (list[i].handle == SENSORS_GYROSCOPE_HANDLE ||
            list[i].handle == SENSORS_RAW_GYROSCOPE_HANDLE ||
            list[i].handle == SENSORS_EIS_GYROSCOPE_HANDLE ||
            list[i].handle == SENSORS_EIS_AUTHENTICATION_HANDLE ||
            list[i].handle == SENSORS_LPQ_HANDLE ||
            list[i].handle == SENSORS_GYROSCOPE_WAKEUP_HANDLE ||
            list[i].handle == SENSORS_RAW_GYROSCOPE_WAKEUP_HANDLE ||
            list[i].handle == SENSORS_GYROSCOPE_RAW_HANDLE) {

            if (strcmp(gyro, "ICM20648") == 0) {
                list[i].power = GYRO_ICM20648_POWER;
                list[i].minDelay = GYRO_ICM20648_MINDELAY;
                list[i].maxDelay = GYRO_ICM20648_MAXDELAY;
            } else if (strcmp(gyro, "ICM20602") == 0) {
                list[i].power = GYRO_ICM20602_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = GYRO_ICM20602_MINDELAY_HIFI;
#else
                list[i].minDelay = GYRO_ICM20602_MINDELAY;
#endif
                list[i].maxDelay = GYRO_ICM20602_MAXDELAY;
            } else if (strcmp(gyro, "ICM20690") == 0) {
                list[i].power = GYRO_ICM20690_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = GYRO_ICM20690_MINDELAY_HIFI;
#else
                list[i].minDelay = GYRO_ICM20690_MINDELAY;
#endif
                list[i].maxDelay = GYRO_ICM20690_MAXDELAY;
            } else if (strcmp(gyro, "ICM20608D") == 0) {
                list[i].power = GYRO_ICM20608D_POWER;
                list[i].minDelay = GYRO_ICM20608D_MINDELAY;
                list[i].maxDelay = GYRO_ICM20608D_MAXDELAY;
            } else if (strcmp(gyro, "IAM20680") == 0) {
                list[i].power = GYRO_IAM20680_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = GYRO_IAM20680_MINDELAY_HIFI;
#else
                list[i].minDelay = GYRO_IAM20680_MINDELAY;
#endif
                list[i].maxDelay = GYRO_IAM20680_MAXDELAY;
            } else if (strcmp(gyro, "ICM42600") == 0) {
                list[i].power = GYRO_ICM42600_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = GYRO_ICM42600_MINDELAY_HIFI;
#else
                list[i].minDelay = GYRO_ICM42600_MINDELAY;
#endif
                list[i].maxDelay = GYRO_ICM42600_MAXDELAY;
            } else if (strcmp(gyro, "ICM43600") == 0) {
                list[i].power = GYRO_ICM43600_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = GYRO_ICM43600_MINDELAY_HIFI;
#else
                list[i].minDelay = GYRO_ICM43600_MINDELAY;
#endif
                list[i].maxDelay = GYRO_ICM43600_MAXDELAY;
            } else if (strcmp(gyro, "ICM45600") == 0) {
                list[i].power = GYRO_ICM45600_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = GYRO_ICM45600_MINDELAY_HIFI;
#else
                list[i].minDelay = GYRO_ICM45600_MINDELAY;
#endif
                list[i].maxDelay = GYRO_ICM45600_MAXDELAY;
            }

            list[i].maxRange = (float)mGyroScale * M_PI / 180.f;
            list[i].resolution = (float)mGyroScale * M_PI / (180.f * 32768.f);
#ifdef DIRECT_REPORT
            if (list[i].handle == SENSORS_GYROSCOPE_HANDLE ||
                    list[i].handle == SENSORS_RAW_GYROSCOPE_HANDLE ||
                    list[i].handle == SENSORS_GYROSCOPE_WAKEUP_HANDLE ||
                    list[i].handle == SENSORS_RAW_GYROSCOPE_WAKEUP_HANDLE ||
                    list[i].handle == SENSORS_GYROSCOPE_RAW_HANDLE) {
                fillDirectReportFlags(&list[i]);
            }
#endif
        }
    }
}

/* fill 9x sensor metadata */
void MPLSensor::fillAGMSensors(struct sensor_t *list)
{
    VFUNC_LOG;

    unsigned int i;
    int gyro_idx = -1;
    int accel_idx = -1;
    int mag_idx = -1;

    for (i = 0; i < mNumSensors; i++) {
        if (list[i].handle == SENSORS_GYROSCOPE_HANDLE)
            gyro_idx = i;
        if (list[i].handle == SENSORS_ACCELERATION_HANDLE)
            accel_idx = i;
        if (list[i].handle == SENSORS_MAGNETIC_FIELD_HANDLE)
            mag_idx = i;
    }

    for (i = 0; i < mNumSensors; i++) {
        if (list[i].handle == SENSORS_ORIENTATION_HANDLE ||
            list[i].handle == SENSORS_ROTATION_VECTOR_HANDLE ||
            list[i].handle == SENSORS_HEADING_HANDLE ||
            list[i].handle == SENSORS_ORIENTATION_WAKEUP_HANDLE ||
            list[i].handle == SENSORS_ROTATION_VECTOR_WAKEUP_HANDLE ||
            list[i].handle == SENSORS_HEADING_WAKEUP_HANDLE) {

            list[i].power = 0;
            list[i].minDelay = 0;
            list[i].maxDelay = 0;

            if (gyro_idx >= 0) {
                list[i].power += list[gyro_idx].power;
                list[i].minDelay = list[gyro_idx].minDelay;
                if (list[gyro_idx].maxDelay < (MIN_SENSOR_FUSION_RATE / 1000)) {
                    list[i].maxDelay = list[gyro_idx].maxDelay;
                } else {
                    list[i].maxDelay = MIN_SENSOR_FUSION_RATE / 1000;
                }
            }
            if (accel_idx >= 0)
                list[i].power += list[accel_idx].power;
            if (mag_idx >= 0)
                list[i].power += list[mag_idx].power;
        }
    }
}

/* fill 6x (a+m) sensor metadata */
void MPLSensor::fillAMSensors(struct sensor_t *list)
{
    VFUNC_LOG;

    unsigned int i;
    int accel_idx = -1;
    int mag_idx = -1;

    for (i = 0; i < mNumSensors; i++) {
        if (list[i].handle == SENSORS_ACCELERATION_HANDLE)
            accel_idx = i;
        if (list[i].handle == SENSORS_MAGNETIC_FIELD_HANDLE)
            mag_idx = i;
    }

    for (i = 0; i < mNumSensors; i++) {
        if (list[i].handle == SENSORS_GEOMAGNETIC_ROTATION_VECTOR_HANDLE ||
            list[i].handle == SENSORS_GEOMAGNETIC_ROTATION_VECTOR_WAKEUP_HANDLE) {

            list[i].power = 0;
            list[i].minDelay = 0;
            list[i].maxDelay = 0;

            if (accel_idx >= 0) {
                list[i].power += list[accel_idx].power;
                list[i].minDelay = list[accel_idx].minDelay;
                if (list[accel_idx].maxDelay < (MIN_MAG_FUSION_RATE / 1000)) {
                    list[i].maxDelay = list[accel_idx].maxDelay;
                } else {
                    list[i].maxDelay = MIN_MAG_FUSION_RATE / 1000;
                }
            }
            if (mag_idx >= 0)
                list[i].power += list[mag_idx].power;
        }
    }
}

/* fill 6x (a+g) sensor metadata */
void MPLSensor::fillAGSensors(struct sensor_t *list)
{
    VFUNC_LOG;

    unsigned int i;
    int gyro_idx = -1;
    int accel_idx = -1;

    for (i = 0; i < mNumSensors; i++) {
        if (list[i].handle == SENSORS_GYROSCOPE_HANDLE)
            gyro_idx = i;
        if (list[i].handle == SENSORS_ACCELERATION_HANDLE)
            accel_idx = i;
    }

    for (i = 0; i < mNumSensors; i++) {
        if (list[i].handle == SENSORS_GAME_ROTATION_VECTOR_HANDLE ||
            list[i].handle == SENSORS_LINEAR_ACCEL_HANDLE ||
            list[i].handle == SENSORS_GRAVITY_HANDLE ||
            list[i].handle == SENSORS_GAME_ROTATION_VECTOR_WAKEUP_HANDLE ||
            list[i].handle == SENSORS_LINEAR_ACCEL_WAKEUP_HANDLE ||
            list[i].handle == SENSORS_GRAVITY_WAKEUP_HANDLE) {

            list[i].power = 0;
            list[i].minDelay = 0;
            list[i].maxDelay = 0;

            if (gyro_idx >= 0) {
                list[i].power += list[gyro_idx].power;
                list[i].minDelay = list[gyro_idx].minDelay;
                if (list[gyro_idx].maxDelay < (MIN_SENSOR_FUSION_RATE / 1000)) {
                    list[i].maxDelay = list[gyro_idx].maxDelay;
                } else {
                    list[i].maxDelay = MIN_SENSOR_FUSION_RATE / 1000;
                }
            }
            if (accel_idx >= 0)
                list[i].power += list[accel_idx].power;
        }
    }

    for (i = 0; i < mNumSensors; i++) {
        if (list[i].handle == SENSORS_LINEAR_ACCEL_HANDLE ||
            list[i].handle == SENSORS_LINEAR_ACCEL_WAKEUP_HANDLE) {
            if (accel_idx >= 0) {
                list[i].maxRange = list[accel_idx].maxRange;
                list[i].resolution = list[accel_idx].resolution;
            }
        }
        if (list[i].handle == SENSORS_GRAVITY_HANDLE ||
            list[i].handle == SENSORS_GRAVITY_WAKEUP_HANDLE) {
            if (accel_idx >= 0) {
                list[i].maxRange = GRAVITY_EARTH;
                list[i].resolution = list[accel_idx].resolution;
            }
        }
    }
}

/* fill accel based gesture sensor metadata */
void MPLSensor::fillGestureSensors(struct sensor_t *list)
{
    VFUNC_LOG;

    unsigned int i;
    int accel_idx = -1;

    for (i = 0; i < mNumSensors; i++) {
        if (list[i].handle == SENSORS_ACCELERATION_HANDLE)
            accel_idx = i;
    }

    for (i = 0; i < mNumSensors; i++) {
        if (list[i].handle == SENSORS_SIGNIFICANT_MOTION_HANDLE ||
            list[i].handle == SENSORS_PEDOMETER_HANDLE ||
            list[i].handle == SENSORS_STEP_COUNTER_HANDLE ||
            list[i].handle == SENSORS_WAKE_UP_TILT_DETECTOR_HANDLE ||
            list[i].handle == SENSORS_PICK_UP_GESTURE_HANDLE ||
            list[i].handle == SENSORS_STATIONARY_DETECT_HANDLE ||
            list[i].handle == SENSORS_MOTION_DETECT_HANDLE ||
            list[i].handle == SENSORS_PEDOMETER_WAKEUP_HANDLE ||
            list[i].handle == SENSORS_STEP_COUNTER_WAKEUP_HANDLE ||
            list[i].handle == SENSORS_SCREEN_ORIENTATION_HANDLE) {

            list[i].power = 0;

            if (accel_idx >= 0) {
                list[i].power = list[accel_idx].power;
            }
        }
    }
}

int MPLSensor::inv_init_sysfs_attributes(void)
{
    VFUNC_LOG;

    unsigned char i = 0;
    char sysfs_path[MAX_SYSFS_NAME_LEN];
    char *sptr;
    char **dptr;

    memset(sysfs_path, 0, sizeof(sysfs_path));

    sysfs_names_ptr =
        (char*)malloc(sizeof(char[MAX_SYSFS_ATTRB][MAX_SYSFS_NAME_LEN]));
    sptr = sysfs_names_ptr;
    if (sptr != NULL) {
        dptr = (char**)&mpu;
        do {
            *dptr++ = sptr;
            memset(sptr, 0, sizeof(char));
            sptr += sizeof(char[MAX_SYSFS_NAME_LEN]);
        } while (++i < MAX_SYSFS_ATTRB);
    } else {
        LOGE("HAL:couldn't alloc mem for sysfs paths");
        return -1;
    }

    // get absolute IIO path & build MPU's sysfs paths
    inv_get_sysfs_path(sysfs_path);

    memcpy(mSysfsPath, sysfs_path, sizeof(sysfs_path));
    sprintf(mpu.chip_enable, "%s%s", sysfs_path, "/buffer/enable");
    sprintf(mpu.buffer_length, "%s%s", sysfs_path, "/buffer/length");
    sprintf(mpu.available_data, "%s%s", sysfs_path, "/buffer/data_available");

    sprintf(mpu.scan_el_en, "%s%s", sysfs_path,
            "/scan_elements/in_accel_en");
    sprintf(mpu.scan_el_index, "%s%s", sysfs_path,
            "/scan_elements/in_accel_index");
    sprintf(mpu.scan_el_type, "%s%s", sysfs_path,
            "/scan_elements/in_accel_type");

    sprintf(mpu.dmp_firmware, "%s%s", sysfs_path, "/misc_bin_dmp_firmware");
    sprintf(mpu.firmware_loaded, "%s%s", sysfs_path, "/info_firmware_loaded");
    sprintf(mpu.dmp_on, "%s%s", sysfs_path, "/debug_dmp_on");
    sprintf(mpu.dmp_int_on, "%s%s", sysfs_path, "/dmp_int_on");
    sprintf(mpu.dmp_event_int_on, "%s%s", sysfs_path, "/debug_event_int_on");
    sprintf(mpu.tap_on, "%s%s", sysfs_path, "/event_tap_enable");

    sprintf(mpu.self_test, "%s%s", sysfs_path, "/misc_self_test");
    sprintf(mpu.dmp_init, "%s%s", sysfs_path, "/in_sc_auth");

    /* gyro sysfs */
    sprintf(mpu.temperature, "%s%s", sysfs_path, "/out_temperature");
    sprintf(mpu.gyro_enable, "%s%s", sysfs_path, "/debug_gyro_enable");
    sprintf(mpu.gyro_orient, "%s%s", sysfs_path, "/info_anglvel_matrix");
    sprintf(mpu.gyro_fifo_enable, "%s%s", sysfs_path, "/in_anglvel_enable");
    sprintf(mpu.gyro_fsr, "%s%s", sysfs_path, "/in_anglvel_scale");
    sprintf(mpu.gyro_sf, "%s%s", sysfs_path, "/info_gyro_sf");
    sprintf(mpu.gyro_rate, "%s%s", sysfs_path, "/in_anglvel_rate");
    sprintf(mpu.calib_gyro_enable, "%s%s", sysfs_path, "/in_calib_anglvel_enable");
    sprintf(mpu.calib_gyro_rate, "%s%s", sysfs_path, "/in_calib_anglvel_rate");
    sprintf(mpu.calib_gyro_wake_enable, "%s%s", sysfs_path, "/in_calib_anglvel_wake_enable");
    sprintf(mpu.calib_gyro_wake_rate, "%s%s", sysfs_path, "/in_calib_anglvel_wake_rate");
    sprintf(mpu.gyro_wake_fifo_enable, "%s%s", sysfs_path, "/in_anglvel_wake_enable");
    sprintf(mpu.gyro_wake_rate, "%s%s", sysfs_path, "/in_anglvel_wake_rate");
    sprintf(mpu.gyro_raw_data, "%s%s", sysfs_path, "/in_anglvel_raw");

    /* compass raw sysfs */
    sprintf(mpu.compass_raw_data, "%s%s", sysfs_path, "/in_magn_raw");

    /* accel sysfs */
    sprintf(mpu.accel_enable, "%s%s", sysfs_path, "/debug_accel_enable");
    sprintf(mpu.accel_orient, "%s%s", sysfs_path, "/info_accel_matrix");
    sprintf(mpu.accel_fifo_enable, "%s%s", sysfs_path, "/in_accel_enable");
    sprintf(mpu.accel_rate, "%s%s", sysfs_path, "/in_accel_rate");
    sprintf(mpu.accel_fsr, "%s%s", sysfs_path, "/in_accel_scale");
    sprintf(mpu.accel_wake_fifo_enable, "%s%s", sysfs_path, "/in_accel_wake_enable");
    sprintf(mpu.accel_wake_rate, "%s%s", sysfs_path, "/in_accel_wake_rate");
    sprintf(mpu.accel_raw_data, "%s%s", sysfs_path, "/in_accel_raw");

    // DMP uses these values
    sprintf(mpu.in_accel_x_dmp_bias, "%s%s", sysfs_path, "/in_accel_x_dmp_bias");
    sprintf(mpu.in_accel_y_dmp_bias, "%s%s", sysfs_path, "/in_accel_y_dmp_bias");
    sprintf(mpu.in_accel_z_dmp_bias, "%s%s", sysfs_path, "/in_accel_z_dmp_bias");

    // MPU uses these values
    sprintf(mpu.in_accel_x_offset, "%s%s", sysfs_path, "/in_accel_x_offset");
    sprintf(mpu.in_accel_y_offset, "%s%s", sysfs_path, "/in_accel_y_offset");
    sprintf(mpu.in_accel_z_offset, "%s%s", sysfs_path, "/in_accel_z_offset");

    // DMP uses these bias values
    sprintf(mpu.in_gyro_x_dmp_bias, "%s%s", sysfs_path, "/in_anglvel_x_dmp_bias");
    sprintf(mpu.in_gyro_y_dmp_bias, "%s%s", sysfs_path, "/in_anglvel_y_dmp_bias");
    sprintf(mpu.in_gyro_z_dmp_bias, "%s%s", sysfs_path, "/in_anglvel_z_dmp_bias");

    // MPU uses these bias values
    sprintf(mpu.in_gyro_x_offset, "%s%s", sysfs_path, "/in_anglvel_x_offset");
    sprintf(mpu.in_gyro_y_offset, "%s%s", sysfs_path, "/in_anglvel_y_offset");
    sprintf(mpu.in_gyro_z_offset, "%s%s", sysfs_path, "/in_anglvel_z_offset");

    // DMP uses these bias values
    sprintf(mpu.in_compass_x_dmp_bias, "%s%s", sysfs_path, "/in_magn_x_dmp_bias");
    sprintf(mpu.in_compass_y_dmp_bias, "%s%s", sysfs_path, "/in_magn_y_dmp_bias");
    sprintf(mpu.in_compass_z_dmp_bias, "%s%s", sysfs_path, "/in_magn_z_dmp_bias");

    sprintf(mpu.three_axis_q_on, "%s%s", sysfs_path, "/in_3quat_enable");
    sprintf(mpu.three_axis_q_rate, "%s%s", sysfs_path, "/in_3quat_rate");

    sprintf(mpu.six_axis_q_on, "%s%s", sysfs_path, "/in_6quat_enable");
    sprintf(mpu.six_axis_q_rate, "%s%s", sysfs_path, "/in_6quat_rate");
    sprintf(mpu.six_axis_q_wake_on, "%s%s", sysfs_path, "/in_6quat_wake_enable");
    sprintf(mpu.six_axis_q_wake_rate, "%s%s", sysfs_path, "/in_6quat_wake_rate");

    sprintf(mpu.nine_axis_q_on, "%s%s", sysfs_path, "/in_9quat_enable");
    sprintf(mpu.nine_axis_q_rate, "%s%s", sysfs_path, "/in_9quat_rate");
    sprintf(mpu.nine_axis_q_wake_on, "%s%s", sysfs_path, "/in_9quat_wake_enable");
    sprintf(mpu.nine_axis_q_wake_rate, "%s%s", sysfs_path, "/in_9quat_wake_rate");

    sprintf(mpu.six_axis_q_value, "%s%s", sysfs_path, "/six_axes_q_value");

    sprintf(mpu.in_geomag_enable, "%s%s", sysfs_path, "/in_geomag_enable");
    sprintf(mpu.in_geomag_rate, "%s%s", sysfs_path, "/in_geomag_rate");
    sprintf(mpu.in_geomag_wake_enable, "%s%s", sysfs_path, "/in_geomag_wake_enable");
    sprintf(mpu.in_geomag_wake_rate, "%s%s", sysfs_path, "/in_geomag_wake_rate");

    snprintf(sysfs_in_power_on, sizeof(sysfs_in_power_on), "%s%s", sysfs_path, "/in_power_on");

    sprintf(mpu.display_orientation_on, "%s%s", sysfs_path,
            "/event_display_orientation_on");
    sprintf(mpu.event_display_orientation, "%s%s", sysfs_path,
            "/poll_display_orientation");
    sprintf(mpu.event_smd, "%s%s", sysfs_path,
            "/poll_smd");
    sprintf(mpu.smd_enable, "%s%s", sysfs_path,
            "/event_smd_enable");
    sprintf(mpu.event_pickup, "%s%s", sysfs_path,
            "/poll_pick_up");
    sprintf(mpu.event_tilt, "%s%s", sysfs_path,
            "/poll_tilt");
    sprintf(mpu.event_activity, "%s%s", sysfs_path,
            "/poll_activity");
    sprintf(mpu.batchmode_timeout, "%s%s", sysfs_path,
            "/misc_batchmode_timeout");
    sprintf(mpu.batchmode_wake_fifo_full_on, "%s%s", sysfs_path,
            "/batchmode_wake_fifo_full_on");
    sprintf(mpu.flush_batch, "%s%s", sysfs_path,
            "/misc_flush_batch");
    sprintf(mpu.pickup_on, "%s%s", sysfs_path,
            "/event_pick_up_enable");
    sprintf(mpu.tilt_on, "%s%s", sysfs_path,
            "/event_tilt_enable");
    sprintf(mpu.tap_on, "%s%s", sysfs_path,
            "/event_tap_enable");
    sprintf(mpu.stationary_detect_on, "%s%s", sysfs_path,
            "/event_stationary_detect_enable");
    sprintf(mpu.motion_detect_on, "%s%s", sysfs_path,
            "/event_motion_detect_enable");
    sprintf(mpu.eis_on, "%s%s", sysfs_path,
            "/event_eis_enable");
    sprintf(mpu.eis_data_on, "%s%s", sysfs_path,
            "/in_eis_enable");
    sprintf(mpu.eis_rate, "%s%s", sysfs_path,
            "/in_eis_rate");
    sprintf(mpu.pedometer_on, "%s%s", sysfs_path,
            "/in_step_detector_enable");
    sprintf(mpu.pedometer_wake_on, "%s%s", sysfs_path,
            "/in_step_detector_wake_enable");
    sprintf(mpu.pedometer_counter_on, "%s%s", sysfs_path,
            "/in_step_counter_enable");
    sprintf(mpu.pedometer_counter_wake_on, "%s%s", sysfs_path,
            "/in_step_counter_wake_enable");
    sprintf(mpu.pedometer_counter_send, "%s%s", sysfs_path,
            "/in_step_counter_send");
    sprintf(mpu.pedometer_int_on, "%s%s", sysfs_path,
            "/params_pedometer_int_on");
    sprintf(mpu.pedometer_int_mode, "%s%s", sysfs_path,
            "/params_pedometer_int_mode");
    sprintf(mpu.event_pedometer, "%s%s", sysfs_path,
            "/poll_pedometer");
    sprintf(mpu.pedometer_steps, "%s%s", sysfs_path,
            "/out_pedometer_steps");
    sprintf(mpu.pedometer_step_thresh, "%s%s", sysfs_path,
            "/params_ped_step_thresh");
    sprintf(mpu.pedometer_counter, "%s%s", sysfs_path,
            "/out_pedometer_counter");
    sprintf(mpu.motion_lpa_on, "%s%s", sysfs_path,
            "/motion_lpa_on");
    sprintf(mpu.gyro_cal_enable, "%s%s", sysfs_path,
            "/debug_gyro_cal_enable");
    sprintf(mpu.accel_cal_enable, "%s%s", sysfs_path,
            "/debug_accel_cal_enable");
    sprintf(mpu.compass_cal_enable, "%s%s", sysfs_path,
            "/debug_compass_cal_enable");
    sprintf(mpu.accel_accuracy_enable, "%s%s", sysfs_path,
            "/debug_accel_accuracy_enable");
    sprintf(mpu.anglvel_accuracy_enable, "%s%s", sysfs_path,
            "/debug_anglvel_accuracy_enable");
    sprintf(mpu.magn_accuracy_enable, "%s%s", sysfs_path,
            "/debug_magn_accuracy_enable");
    sprintf(mpu.debug_determine_engine_on, "%s%s", sysfs_path,
            "/debug_determine_engine_on");
    sprintf(mpu.d_compass_enable, "%s%s", sysfs_path,
            "/debug_compass_enable");
    sprintf(mpu.d_misc_gyro_recalibration, "%s%s", sysfs_path,
            "/misc_gyro_recalibration");
    sprintf(mpu.d_misc_accel_recalibration, "%s%s", sysfs_path,
            "/misc_accel_recalibration");
    sprintf(mpu.d_misc_compass_recalibration, "%s%s", sysfs_path,
            "/misc_magn_recalibration");
    sprintf(mpu.d_misc_accel_cov, "%s%s", sysfs_path,
            "/misc_bin_accel_covariance");
    sprintf(mpu.d_misc_current_compass_cov, "%s%s", sysfs_path,
            "/misc_bin_cur_magn_covariance");
    sprintf(mpu.d_misc_compass_cov, "%s%s", sysfs_path,
            "/misc_bin_magn_covariance");
    sprintf(mpu.d_misc_ref_mag_3d, "%s%s", sysfs_path,
            "/misc_ref_mag_3d");
    sprintf(mpu.ois_enable, "%s%s", sysfs_path,
            "/in_ois_enable");

    /* Data injection sysfs */
    sprintf(mpu.info_poke_mode, "%s%s", sysfs_path, "/info_poke_mode");
    sprintf(mpu.misc_bin_poke_accel, "%s%s", sysfs_path, "/misc_bin_poke_accel");
    sprintf(mpu.misc_bin_poke_gyro, "%s%s", sysfs_path, "/misc_bin_poke_gyro");
    sprintf(mpu.misc_bin_poke_mag, "%s%s", sysfs_path, "/misc_bin_poke_mab");

    sprintf(mpu.gyro_lp_mode, "%s%s", sysfs_path,
            "/info_gyro_lp_mode");
    sprintf(mpu.accel_lp_mode, "%s%s", sysfs_path,
            "/info_accel_lp_mode");

    return 0;
}

bool MPLSensor::isMpuNonDmp(void)
{
    VFUNC_LOG;
    if (!strcmp(chip_ID, "ICM20602")
        || !strcmp(chip_ID, "ICM20690")
        || !strcmp(chip_ID, "IAM20680")
        || !strcmp(chip_ID, "ICM42600")
        || !strcmp(chip_ID, "ICM43600"))
        return true;
    else
        return false;
}

void MPLSensor::resetCalStatus(int recal)
{
    VFUNC_LOG;

#if DEBUG_DRIVER
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)", recal,
            mpu.d_misc_gyro_recalibration, (long long)getTimestamp());
    if (write_sysfs_int(mpu.d_misc_gyro_recalibration, recal) < 0) {
        LOGE("HAL:Error writing to misc_gyro_recalibration");
    }
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)", recal,
            mpu.d_misc_accel_recalibration, (long long)getTimestamp());
    if (write_sysfs_int(mpu.d_misc_accel_recalibration, recal) < 0) {
        LOGE("HAL:Error writing to misc_accel_recalibration");
    }
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)", recal,
            mpu.d_misc_compass_recalibration, (long long)getTimestamp());
    if (write_sysfs_int(mpu.d_misc_compass_recalibration, recal) < 0) {
        LOGE("HAL:Error writing to misc_magn_recalibration");
    }
#else
    (void)recal;
#endif

    return;
}

void MPLSensor::getFactoryGyroBias()
{
    VFUNC_LOG;

    /* Get Values from MPL */
    inv_get_gyro_bias(mFactoryGyroBias);
    inv_get_gyro_bias_2(mFactoryGyroBiasLp); /* lp mode */

    /* adjust factory bias for lp according to fsr */
    if (mFactoryGyroBias[0] && mFactoryGyroBias[1] && mFactoryGyroBias[2]) {
        LOGV_IF(ENG_VERBOSE, "Factory Gyro Bias OIS %d %d %d",
                mFactoryGyroBias[0], mFactoryGyroBias[1], mFactoryGyroBias[2]);
        LOGV_IF(ENG_VERBOSE, "Factory Gyro Bias LP  %d %d %d",
                mFactoryGyroBiasLp[0], mFactoryGyroBiasLp[1], mFactoryGyroBiasLp[2]);
        mFactoryGyroBiasAvailable = true;
        mFactoryGyroBiasLoaded = true;
        if (mFactoryGyroBiasLp[0] && mFactoryGyroBiasLp[1] && mFactoryGyroBiasLp[2]) {
            /* bias in calibration file is 2000dps scaled by 1<<16 */
            int i;
            for (i = 0; i < 3; i++)
                mGyroBiasUiMode[i] = ((mFactoryGyroBias[i] - mFactoryGyroBiasLp[i])
                        * (2000 / mGyroScale)) >>16;
            mFactoryGyroBiasLpLoaded = true;
        }
    }
    LOGV_IF(ENG_VERBOSE, "Gyro Bias UI (LSB) %d %d %d",
            mGyroBiasUiMode[0], mGyroBiasUiMode[1], mGyroBiasUiMode[2]);

    return;
}

/* set bias from factory cal file to MPU offset (in chip frame)
   x = values store in cal file --> (v * 2^16 / (2000/250))
   (hardware unit @ 2000dps scaled by 2^16)
   set the same values to driver
 */
void MPLSensor::setFactoryGyroBias()
{
    VFUNC_LOG;
    int scaleRatio = 1;
    int offsetScale = 1;
    LOGV_IF(ENG_VERBOSE, "HAL: scaleRatio used =%d", scaleRatio);
    LOGV_IF(ENG_VERBOSE, "HAL: offsetScale used =%d", offsetScale);

    /* Write to Driver */
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            (((int) (((float) mFactoryGyroBias[0]) * scaleRatio)) / offsetScale),
            mpu.in_gyro_x_offset, (long long)getTimestamp());
    if (write_attribute_sensor_continuous(gyro_x_offset_fd,
                (((int) (((float) mFactoryGyroBias[0]) * scaleRatio)) / offsetScale)) < 0) {
        LOGE("HAL:Error writing to gyro_x_offset");
        return;
    }
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            (((int) (((float) mFactoryGyroBias[1]) * scaleRatio)) / offsetScale),
            mpu.in_gyro_y_offset, (long long)getTimestamp());
    if (write_attribute_sensor_continuous(gyro_y_offset_fd,
                (((int) (((float) mFactoryGyroBias[1]) * scaleRatio)) / offsetScale)) < 0) {
        LOGE("HAL:Error writing to gyro_y_offset");
        return;
    }
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            (((int) (((float) mFactoryGyroBias[2]) * scaleRatio)) / offsetScale),
            mpu.in_gyro_z_offset, (long long)getTimestamp());
    if (write_attribute_sensor_continuous(gyro_z_offset_fd,
                (((int) (((float) mFactoryGyroBias[2]) * scaleRatio)) / offsetScale)) < 0) {
        LOGE("HAL:Error writing to gyro_z_offset");
        return;
    }
    mFactoryGyroBiasAvailable = false;
    setAccuracy(0, 1);
    LOGV_IF(EXTRA_VERBOSE, "HAL:Factory Gyro Calibrated Bias Applied");

    return;
}

/*
 * get gyro bias from calibration file (scaled and in body frame)
 */
void MPLSensor::getGyroBias()
{
    VFUNC_LOG;

    int *temp = NULL;

    /* Get Values from MPL */
    inv_get_mpl_gyro_bias(mDmpGyroBias, temp);
    LOGV_IF(ENG_VERBOSE, "get_mpl_gyro_bias [LN mode] %d %d %d",
            mDmpGyroBias[0], mDmpGyroBias[1], mDmpGyroBias[2]);
    if (mDmpGyroBias[0] && mDmpGyroBias[1] && mDmpGyroBias[2]) {
        mGyroBiasAvailable = true;
    }

    /* Get Values from MPL for LP mode */
    inv_get_mpl_gyro_bias_2(mDmpGyroBiasLp, temp);
    LOGV_IF(ENG_VERBOSE, "get_mpl_gyro_bias [LP mode] %d %d %d",
            mDmpGyroBiasLp[0], mDmpGyroBiasLp[1], mDmpGyroBiasLp[2]);
    if (mDmpGyroBiasLp[0] && mDmpGyroBiasLp[1] && mDmpGyroBiasLp[2]) {
        mGyroBiasLpAvailable = true;
    }

    return;
}

/*
   Set Gyro Bias to DMP, from MPL - load cal
   DMP units: bias * 46850825 / 2^30 = bias / 2 (in body frame)
 */
void MPLSensor::setDmpGyroBias(bool sysfs)
{
    VFUNC_LOG;

    if (mCalibrationMode & mpl_gyro_cal) {
        int accuracy = 0;
        if (mGyroLpMode) {
            if (mGyroBiasLpAvailable) {
                accuracy = 2;
            }
            if (mGyroLpAccuracyLib > accuracy) {
                accuracy = mGyroLpAccuracyLib;
            }
            adapter_set_mpl_gyro_bias(mDmpGyroBiasLp, accuracy);
            if (mFactoryGyroBiasLpLoaded && (accuracy < 1))
                accuracy = 1;
            setAccuracy(0, accuracy, 1);
            LOGV_IF(EXTRA_VERBOSE, "HAL:Gyro Calibrated Bias Applied [LP mode] %d %d %d accuracy=%d",
                    mDmpGyroBiasLp[0], mDmpGyroBiasLp[1], mDmpGyroBiasLp[2], accuracy);
        } else {
            if (mGyroBiasAvailable) {
                accuracy = 2;
            }
            if (mGyroAccuracyLib > accuracy) {
                accuracy = mGyroAccuracyLib;
            }
            adapter_set_mpl_gyro_bias(mDmpGyroBias, accuracy);
            if (mFactoryGyroBiasLoaded && (accuracy < 1))
                accuracy = 1;
            setAccuracy(0, accuracy, 1);
            LOGV_IF(EXTRA_VERBOSE, "HAL:Gyro Calibrated Bias Applied [LN mode] %d %d %d accuracy=%d",
                    mDmpGyroBias[0], mDmpGyroBias[1], mDmpGyroBias[2], accuracy);
        }
    } else {
        if (sysfs) {
            /* Write to Driver */
            LOGV_IF(SYSFS_VERBOSE && INPUT_DATA, "HAL:sysfs:echo %d > %s (%lld)",
                    mDmpGyroBias[0], mpu.in_gyro_x_dmp_bias, (long long)getTimestamp());
            if (write_attribute_sensor_continuous(gyro_x_dmp_bias_fd, mDmpGyroBias[0]) < 0) {
                LOGE("HAL:Error writing to gyro_x_dmp_bias");
                return;
            }
            LOGV_IF(SYSFS_VERBOSE && INPUT_DATA, "HAL:sysfs:echo %d > %s (%lld)",
                    mDmpGyroBias[1], mpu.in_gyro_y_dmp_bias, (long long)getTimestamp());
            if (write_attribute_sensor_continuous(gyro_y_dmp_bias_fd, mDmpGyroBias[1]) < 0) {
                LOGE("HAL:Error writing to gyro_y_dmp_bias");
                return;
            }
            LOGV_IF(SYSFS_VERBOSE && INPUT_DATA, "HAL:sysfs:echo %d > %s (%lld)",
                    mDmpGyroBias[2], mpu.in_gyro_z_dmp_bias, (long long)getTimestamp());
            if (write_attribute_sensor_continuous(gyro_z_dmp_bias_fd, mDmpGyroBias[2]) < 0) {
                LOGE("HAL:Error writing to gyro_z_dmp_bias");
                return;
            }
            mGyroBiasAvailable = false;
        }
    }
    mGyroBiasApplied = true;
    LOGV_IF(EXTRA_VERBOSE, "HAL:Gyro DMP Calibrated Bias Applied");

    return;
}

/*
   Get Gyro Bias from DMP, then store in MPL - store cal
   DMP units: bias * 46850825 / 2^30 = bias / 2 (in body frame)
 */
void MPLSensor::getDmpGyroBias(bool sysfs)
{
    VFUNC_LOG;

    char buf[sizeof(int) *4];
    int bias[3];
    int updated = 0;
    int accuracy = 0;

    memset(bias, 0, sizeof(bias));

    if (mCalibrationMode & mpl_gyro_cal) {
        int temperature = 0; // dummy
        adapter_get_mpl_gyro_bias(bias, &temperature, &accuracy, &updated);
    } else {
        if (sysfs) {
            /* Read from Driver */
            if (read_attribute_sensor(gyro_x_dmp_bias_fd, buf, sizeof(buf)) < 0) {
                LOGE("HAL:Error reading gyro_x_dmp_bias");
                return;
            }
            if (sscanf(buf, "%d", &bias[0]) < 0) {
                LOGE("HAL:Error reading gyro bias");
                return;
            }
            if (read_attribute_sensor(gyro_y_dmp_bias_fd, buf, sizeof(buf)) < 0) {
                LOGE("HAL:Error reading gyro_y_dmp_bias");
                return;
            }
            if (sscanf(buf, "%d", &bias[1]) < 0) {
                LOGE("HAL:Error reading gyro bias");
                return;
            }
            if (read_attribute_sensor(gyro_z_dmp_bias_fd, buf, sizeof(buf)) < 0) {
                LOGE("HAL:Error reading gyro_z_dmp_bias");
                return;
            }
            if (sscanf(buf, "%d", &bias[2]) < 0) {
                LOGE("HAL:Error reading gyro bias");
                return;
            }
        }
    }
    LOGV_IF(SYSFS_VERBOSE && INPUT_DATA, "HAL:Dmp Gyro Bias read %d %d %d", bias[0], bias[1], bias[2]);
    if (bias[0] && bias[1] && bias[2]) {
        if (mCalibrationMode & mpl_gyro_cal) {
            if (updated) {
                if (mGyroLpMode) {
                    mDmpGyroBiasLp[0] = bias[0];
                    mDmpGyroBiasLp[1] = bias[1];
                    mDmpGyroBiasLp[2] = bias[2];
                    inv_set_mpl_gyro_bias_2(mDmpGyroBiasLp, 3);
                    LOGV_IF(EXTRA_VERBOSE, "HAL:Gyro DMP Calibrated Bias Obtained [LP mode] %d %d %d accuracy=%d",
                            mDmpGyroBiasLp[0], mDmpGyroBiasLp[1], mDmpGyroBiasLp[2], accuracy);
                    mGyroBiasLpAvailable = true;
                    mGyroLpAccuracyLib = accuracy;
                } else {
                    mDmpGyroBias[0] = bias[0];
                    mDmpGyroBias[1] = bias[1];
                    mDmpGyroBias[2] = bias[2];
                    inv_set_mpl_gyro_bias(mDmpGyroBias, 3);
                    LOGV_IF(EXTRA_VERBOSE, "HAL:Gyro DMP Calibrated Bias Obtained [LN mode] %d %d %d accuracy=%d",
                            mDmpGyroBias[0], mDmpGyroBias[1], mDmpGyroBias[2], accuracy);
                    mGyroBiasAvailable = true;
                    mGyroAccuracyLib = accuracy;
                }
            }
        } else {
            mDmpGyroBias[0] = bias[0];
            mDmpGyroBias[1] = bias[1];
            mDmpGyroBias[2] = bias[2];
            inv_set_mpl_gyro_bias(mDmpGyroBias, 3);
            LOGV_IF(EXTRA_VERBOSE, "HAL:Gyro DMP Calibrated Bias Obtained");
            mGyroBiasAvailable = true;
        }
    }
    return;
}

void MPLSensor::getFactoryAccelBias()
{
    VFUNC_LOG;

    /* Get Values from MPL */
    inv_get_accel_bias(mFactoryAccelBias);
    inv_get_accel_bias_2(mFactoryAccelBiasLp);
    if (mFactoryAccelBias[0] && mFactoryAccelBias[1] && mFactoryAccelBias[2]) {
        LOGV_IF(ENG_VERBOSE, "Factory Accel Bias OIS %d %d %d",
                mFactoryAccelBias[0], mFactoryAccelBias[1], mFactoryAccelBias[2]);
        LOGV_IF(ENG_VERBOSE, "Factory Accel Bias LP  %d %d %d",
                mFactoryAccelBiasLp[0], mFactoryAccelBiasLp[1], mFactoryAccelBiasLp[2]);
        mFactoryAccelBiasAvailable = true;
        mFactoryAccelBiasLoaded = true;
        if (mFactoryAccelBiasLp[0] && mFactoryAccelBiasLp[1] && mFactoryAccelBias[2]) {
            /* bias in calibration file is 2g scaled by 1<<16 */
            int i;
            for (i = 0; i < 3; i++)
                mAccelBiasUiMode[i] = ((mFactoryAccelBias[i] - mFactoryAccelBiasLp[i])
                        * 2 / mAccelScale)>>16;
            mFactoryAccelBiasLpLoaded = true;
        }
    }
    LOGV_IF(ENG_VERBOSE, "Accel Bias UI (LSB) %d %d %d",
            mAccelBiasUiMode[0], mAccelBiasUiMode[1], mAccelBiasUiMode[2]);

    return;
}

void MPLSensor::setFactoryAccelBias()
{
    VFUNC_LOG;

    if (mFactoryAccelBiasAvailable == false)
        return;

    /* add scaling here - depends on self test parameters */
    int scaleRatio = 1;
    int offsetScale = 1;
    int tempBias;

    LOGV_IF(ENG_VERBOSE, "HAL: scaleRatio used =%d", scaleRatio);
    LOGV_IF(ENG_VERBOSE, "HAL: offsetScale used =%d", offsetScale);

    /* Write to Driver */
    tempBias = mFactoryAccelBias[0] * scaleRatio / offsetScale;
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            tempBias, mpu.in_accel_x_offset, (long long)getTimestamp());
    if (write_attribute_sensor_continuous(accel_x_offset_fd, tempBias) < 0) {
        LOGE("HAL:Error writing to accel_x_offset");
        return;
    }
    tempBias = mFactoryAccelBias[1] * scaleRatio / offsetScale;
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            tempBias, mpu.in_accel_y_offset, (long long)getTimestamp());
    if (write_attribute_sensor_continuous(accel_y_offset_fd, tempBias) < 0) {
        LOGE("HAL:Error writing to accel_y_offset");
        return;
    }
    tempBias = mFactoryAccelBias[2] * scaleRatio / offsetScale;
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            tempBias, mpu.in_accel_z_offset, (long long)getTimestamp());
    if (write_attribute_sensor_continuous(accel_z_offset_fd, tempBias) < 0) {
        LOGE("HAL:Error writing to accel_z_offset");
        return;
    }
    mFactoryAccelBiasAvailable = false;
    setAccuracy(1, 3);
    LOGV_IF(EXTRA_VERBOSE, "HAL:Factory Accel Calibrated Bias Applied = %d", mAccelAccuracy);

    return;
}

void MPLSensor::getAccelBias()
{
    VFUNC_LOG;
    int temp;

    /* Get Values from MPL */
    inv_get_mpl_accel_bias(mAccelDmpBias, &temp);
    LOGV_IF(ENG_VERBOSE, "get_mpl_accel_bias [LN mode] %d %d %d",
            mAccelDmpBias[0], mAccelDmpBias[1], mAccelDmpBias[2]);
    if (mAccelDmpBias[0] && mAccelDmpBias[1] && mAccelDmpBias[2]) {
        mAccelBiasAvailable = true;
    }

    /* Get Values from MPL for LP mode */
    inv_get_mpl_accel_bias_2(mAccelDmpBiasLp, &temp);
    LOGV_IF(ENG_VERBOSE, "get_mpl_accel_bias [LP mode] %d %d %d",
            mAccelDmpBiasLp[0], mAccelDmpBiasLp[1], mAccelDmpBiasLp[2]);
    if (mAccelDmpBiasLp[0] && mAccelDmpBiasLp[1] && mAccelDmpBiasLp[2]) {
        mAccelBiasLpAvailable = true;
    }

    return;
}

/*
   Get Accel Bias from DMP, then set to MPL (store cal)
   DMP units: bias * 536870912 / 2^30 = bias / 2 (in body frame)
 */
void MPLSensor::getDmpAccelBias(bool sysfs)
{
    VFUNC_LOG;
    char buf[sizeof(int) *4];
    int data;
    int bias[3];
    int updated = 0;
    int accuracy = 0;

    memset(bias, 0, sizeof(bias));

    if (mCalibrationMode & mpl_accel_cal) {
        int temperature = 0; // dummy
        adapter_get_mpl_accel_bias(bias, &temperature, &accuracy, &updated);
    } else {
        if (sysfs) {
            /* read from driver */
            memset(buf, 0, sizeof(buf));
            if (read_attribute_sensor(
                        accel_x_dmp_bias_fd, buf, sizeof(buf)) < 0) {
                LOGE("HAL:Error reading accel_x_dmp_bias");
                return;
            }
            sscanf(buf, "%d", &data);
            bias[0] = data;

            memset(buf, 0, sizeof(buf));
            if (read_attribute_sensor(
                        accel_y_dmp_bias_fd, buf, sizeof(buf)) < 0) {
                LOGE("HAL:Error reading accel_y_dmp_bias");
                return;
            }
            sscanf(buf, "%d", &data);
            bias[1] = data;

            memset(buf, 0, sizeof(buf));
            if (read_attribute_sensor(
                        accel_z_dmp_bias_fd, buf, sizeof(buf)) < 0) {
                LOGE("HAL:Error reading accel_z_dmp_bias");
                return;
            }
            sscanf(buf, "%d", &data);
            bias[2] = data;
        }
    }
    LOGV_IF(SYSFS_VERBOSE && INPUT_DATA, "HAL:Dmp Accel Bias read %d %d %d",
            bias[0], bias[1], bias[2]);
    if (bias[0] && bias[1] && bias[2]) {
        if (mCalibrationMode & mpl_accel_cal) {
            if (updated) {
                if (mAccelLpMode) {
                    mAccelDmpBiasLp[0] = bias[0];
                    mAccelDmpBiasLp[1] = bias[1];
                    mAccelDmpBiasLp[2] = bias[2];
                    inv_set_accel_bias_mask_2(mAccelDmpBiasLp, 3, 7); //(3=highest accuracy; 7=all axis)
                    LOGV_IF(EXTRA_VERBOSE, "HAL:Accel DMP Calibrated Bias Obtained [LP mode] %d %d %d accuracy=%d",
                            mAccelDmpBiasLp[0], mAccelDmpBiasLp[1], mAccelDmpBiasLp[2], accuracy);
                    mAccelBiasLpAvailable =  true;
                    mAccelLpAccuracyLib = accuracy;
                } else {
                    mAccelDmpBias[0] = bias[0];
                    mAccelDmpBias[1] = bias[1];
                    mAccelDmpBias[2] = bias[2];
                    inv_set_accel_bias_mask(mAccelDmpBias, 3, 7); //(3=highest accuracy; 7=all axis)
                    LOGV_IF(EXTRA_VERBOSE, "HAL:Accel DMP Calibrated Bias Obtained [LN mode] %d %d %d accuracy=%d",
                            mAccelDmpBias[0], mAccelDmpBias[1], mAccelDmpBias[2], accuracy);
                    mAccelBiasAvailable =  true;
                    mAccelAccuracyLib = accuracy;
                }
            }
        } else {
            mAccelDmpBias[0] = bias[0];
            mAccelDmpBias[1] = bias[1];
            mAccelDmpBias[2] = bias[2];
            inv_set_accel_bias_mask(mAccelDmpBias, 3, 7); //(3=highest accuracy; 7=all axis)
            LOGV_IF(EXTRA_VERBOSE, "HAL:Accel DMP Calibrated Bias Obtained");
            mAccelBiasAvailable =  true;
            setAccuracy(1, 3);
        }
    }
    return;
}

/*    set accel bias obtained from cal file to DMP - load cal
      DMP expects: bias * 536870912 / 2^30 = bias / 2 (in body frame)
 */
void MPLSensor::setDmpAccelBias(bool sysfs)
{
    VFUNC_LOG;

    if (mCalibrationMode & mpl_accel_cal) {
        int accuracy = 0;
        if (mAccelLpMode) {
            if (mAccelBiasLpAvailable) {
                accuracy = 3;
            }
            if (mAccelLpAccuracyLib > accuracy) {
                accuracy = mAccelLpAccuracyLib;
            }
            adapter_set_mpl_accel_bias(mAccelDmpBiasLp, accuracy);
            if (mFactoryAccelBiasLpLoaded && (accuracy < 1))
                accuracy = 3;
            setAccuracy(1, accuracy, 1);
            LOGV_IF(EXTRA_VERBOSE, "HAL:Accel Calibrated Bias Applied [LP mode] %d %d %d accuracy=%d",
                    mAccelDmpBiasLp[0], mAccelDmpBiasLp[1], mAccelDmpBiasLp[2], accuracy);
        } else {
            if (mAccelBiasAvailable) {
                accuracy = 3;
            }
            if (mAccelAccuracyLib > accuracy) {
                accuracy = mAccelAccuracyLib;
            }
            adapter_set_mpl_accel_bias(mAccelDmpBias, accuracy);
            if (mFactoryAccelBiasLoaded && (accuracy < 1))
                accuracy = 3;
            setAccuracy(1, accuracy, 1);
            LOGV_IF(EXTRA_VERBOSE, "HAL:Accel Calibrated Bias Applied [LN mode] %d %d %d accuracy = %d",
                    mAccelDmpBias[0], mAccelDmpBias[1], mAccelDmpBias[2], accuracy);
        }
    } else {
        if (sysfs) {
            /* write to driver */
            LOGV_IF(SYSFS_VERBOSE && INPUT_DATA, "HAL:sysfs:echo %d > %s (%lld)",
                    mAccelDmpBias[0], mpu.in_accel_x_dmp_bias, (long long)getTimestamp());
            if ((write_attribute_sensor_continuous(
                            accel_x_dmp_bias_fd, mAccelDmpBias[0])) < 0) {
                LOGE("HAL:Error writing to accel_x_dmp_bias");
                return;
            }
            LOGV_IF(SYSFS_VERBOSE && INPUT_DATA, "HAL:sysfs:echo %d > %s (%lld)",
                    mAccelDmpBias[1], mpu.in_accel_y_dmp_bias, (long long)getTimestamp());
            if ((write_attribute_sensor_continuous(
                            accel_y_dmp_bias_fd, mAccelDmpBias[1])) < 0) {
                LOGE("HAL:Error writing to accel_y_dmp_bias");
                return;
            }
            LOGV_IF(SYSFS_VERBOSE && INPUT_DATA, "HAL:sysfs:echo %d > %s (%lld)",
                    mAccelDmpBias[2], mpu.in_accel_z_dmp_bias, (long long)getTimestamp());
            if ((write_attribute_sensor_continuous(
                            accel_z_dmp_bias_fd, mAccelDmpBias[2])) < 0) {
                LOGE("HAL:Error writing to accel_z_dmp_bias");
                return;
            }
            mAccelBiasAvailable = false;
        }
    }
    mAccelBiasApplied = true;
    LOGV_IF(EXTRA_VERBOSE, "HAL:Accel DMP Calibrated Bias Applied");

    return;
}

/*
   Get Compass Bias from DMP, then store in MPL - store cal
   DMP units: scaled in body frame
 */
void MPLSensor::getDmpCompassBias()
{
    VFUNC_LOG;

    char buf[sizeof(int) *4];
    int bias[3];

    if (mCalibrationMode & mpl_compass_cal) {
        memset(bias, 0, sizeof(bias));
        adapter_get_mpl_compass_bias(bias);
    } else {
        /* Read from Driver */
        if (read_attribute_sensor(compass_x_dmp_bias_fd, buf, sizeof(buf)) < 0) {
            LOGE("HAL:Error reading compass_x_dmp_bias fd %d", compass_x_dmp_bias_fd);
            return;
        }
        if (sscanf(buf, "%d", &bias[0]) < 0) {
            LOGE("HAL:Error reading compass bias x buf [%s]", buf);
            return;
        }
        if (read_attribute_sensor(compass_y_dmp_bias_fd, buf, sizeof(buf)) < 0) {
            LOGE("HAL:Error reading compass_y_dmp_bias fd %d", compass_y_dmp_bias_fd);
            return;
        }
        if (sscanf(buf, "%d", &bias[1]) < 0) {
            LOGE("HAL:Error reading compass bias y buf [%s]", buf);
            return;
        }
        if (read_attribute_sensor(compass_z_dmp_bias_fd, buf, sizeof(buf)) < 0) {
            LOGE("HAL:Error reading compass_z_dmp_bias fd %d", compass_z_dmp_bias_fd);
            return;
        }
        if (sscanf(buf, "%d", &bias[2]) < 0) {
            LOGE("HAL:Error reading compass bias z buf [%s]", buf);
            return;
        }
    }
    LOGV_IF(SYSFS_VERBOSE && INPUT_DATA, "HAL:Dmp Compass Bias read %d %d %d", bias[0], bias[1], bias[2]);
    if (bias[0] && bias[1] && bias[2]) {
        mDmpCompassBias[0] = bias[0];
        mDmpCompassBias[1] = bias[1];
        mDmpCompassBias[2] = bias[2];
        mCompassBiasAvailable = 1;
        inv_set_compass_bias(mDmpCompassBias, 3);
        LOGV_IF(EXTRA_VERBOSE, "HAL:Compass DMP Calibrated Bias Obtained");
    }
    return;
}

/*    set compass bias obtained from cal file to DMP - load cal
      DMP expects: scaled in body frame
 */
void MPLSensor::setDmpCompassBias()
{
    VFUNC_LOG;

    if (mCalibrationMode & mpl_compass_cal) {
        adapter_set_mpl_compass_bias(mDmpCompassBias, 1);
        setAccuracy(2, 1);
    }
    /* write to driver */
    LOGV_IF(SYSFS_VERBOSE && INPUT_DATA, "HAL:sysfs:echo %d > %s (%lld)",
            mDmpCompassBias[0], mpu.in_compass_x_dmp_bias, (long long)getTimestamp());
    if ((write_attribute_sensor_continuous(
                    compass_x_dmp_bias_fd, mDmpCompassBias[0])) < 0) {
        LOGE("HAL:Error writing to compass_x_dmp_bias");
        return;
    }
    LOGV_IF(SYSFS_VERBOSE && INPUT_DATA, "HAL:sysfs:echo %d > %s (%lld)",
            mDmpCompassBias[1], mpu.in_compass_y_dmp_bias, (long long)getTimestamp());
    if ((write_attribute_sensor_continuous(
                    compass_y_dmp_bias_fd, mDmpCompassBias[1])) < 0) {
        LOGE("HAL:Error writing to compass_y_dmp_bias");
        return;
    }
    LOGV_IF(SYSFS_VERBOSE && INPUT_DATA, "HAL:sysfs:echo %d > %s (%lld)",
            mDmpCompassBias[2], mpu.in_compass_z_dmp_bias, (long long)getTimestamp());
    if ((write_attribute_sensor_continuous(
                    compass_z_dmp_bias_fd, mDmpCompassBias[2])) < 0) {
        LOGE("HAL:Error writing to compass_z_dmp_bias");
        return;
    }

    mCompassBiasApplied = true;
    mCompassBiasAvailable = false;
    LOGV_IF(EXTRA_VERBOSE, "HAL:Compass DMP Calibrated Bias Applied");

    return;
}

void MPLSensor::getCompassBias()
{
    VFUNC_LOG;

    /* Get Values from MPL */
    inv_get_compass_bias(mDmpCompassBias);
    LOGV_IF(ENG_VERBOSE, "Compass Bias %d %d %d",
            mDmpCompassBias[0], mDmpCompassBias[1], mDmpCompassBias[2]);
    if (mDmpCompassBias[0] && mDmpCompassBias[1] && mDmpCompassBias[2]) {
        mCompassBiasAvailable = true;
    }
    return;
}

int MPLSensor::checkBatchEnabled(void)
{
    VFUNC_LOG;
    return ((mFeatureActiveMask & INV_DMP_BATCH_MODE)? 1:0);
}

/* precondition: framework disallows this case, ie enable continuous sensor, */
/* and enable batch sensor */
/* if one sensor is in continuous mode, HAL disallows enabling batch for this sensor */
/* or any other sensors */
int MPLSensor::batch(int handle, int flags, int64_t period_ns, int64_t timeout)
{
    VFUNC_LOG;

#ifdef DIRECT_REPORT
    const struct sensor_t *sensor;
    int32_t what = -1;
    int ret;

    getHandle(handle, what, sensor);
    if (what < 0) {
        LOGE("HAL:batch sensors %d not found", handle);
        return -EINVAL;
    }

    pthread_mutex_lock(&mHALMutex);
    SensorChannelMux &mux = mSensorChannelMuxes[handle];
    const SensorChannelMux::SensorParams old_params = mux.getParams();
    SensorChannelMux::SensorParams request_params = {
        .periodNs = period_ns,
        .latencyNs = timeout,
    };
    mPollChannelTiming[handle].rateLevel = period_ns;
    mux.registerChannel(0, request_params);
    const SensorChannelMux::SensorParams new_params = mux.getParams();
    pthread_mutex_unlock(&mHALMutex);
    LOGI_IF(ENG_VERBOSE, "HAL:directReport: old_params (%lld, %lld) - new_params (%lld, %lld)",
            (long long)old_params.periodNs, (long long)old_params.latencyNs, (long long)new_params.periodNs, (long long)new_params.latencyNs);

    if (new_params.periodNs != old_params.periodNs || new_params.latencyNs != old_params.latencyNs) {
        ret = doBatch(handle, flags, new_params.periodNs, new_params.latencyNs);
    } else {
        ret = NO_ERROR;
    }

    return ret;
#else
    return doBatch(handle, flags, period_ns, timeout);
#endif
}

int MPLSensor::doBatch(int handle, int flags, int64_t period_ns, int64_t timeout)
{
    VFUNC_LOG;

    int res = 0;
    const struct sensor_t *sensor;
    int32_t what = -1;
    int batchMode;

    if (!mChipDetected)
        return -EINVAL;

    LOGI_IF(DEBUG_BATCHING || ENG_VERBOSE,
            "HAL:batch called - handle=%d, flags=%d, period=%lld, timeout=%lld",
            handle, flags, (long long)period_ns, (long long)timeout);

    /* check if the handle is valid */
    getHandle(handle, what, sensor);
    if (what < 0) {
        LOGE("HAL:batch sensors %d not found", handle);
        return -EINVAL;
    }

    /* check if we can support issuing interrupt before FIFO fills-up */
    /* in a given timeout.                                          */
    if (flags & SENSORS_BATCH_WAKE_UPON_FIFO_FULL) {
        LOGE("HAL: batch SENSORS_BATCH_WAKE_UPON_FIFO_FULL is not supported");
        return -EINVAL;
    }

    /* OIS Sensor is dummy, do nothing */
    if (uint32_t(what) == OisSensor) {
        LOGI_IF(ENG_VERBOSE, "HAL:batch Ois Sensor is dummy, return.");
        return 0;
    }

    if (period_ns != sensor->maxDelay * 1000) {
        /* Round up in Hz when requested frequency has fractional digit.
         * This is necessary to pass CTS Verifier Device Suspend Test.
         * Note: not round up if requested frequency is the same as maxDelay */
        int period_ns_int;
        period_ns_int = (NS_PER_SECOND + (period_ns - 1))/ period_ns;
        period_ns = NS_PER_SECOND / period_ns_int;
    }
    if (period_ns > sensor->maxDelay * 1000)
        period_ns = sensor->maxDelay * 1000;
    if (period_ns < sensor->minDelay * 1000)
        period_ns = sensor->minDelay * 1000;

    /* just stream with no error return, if the sensor does not support batch mode */
    if (sensor->fifoMaxEventCount != 0) {
        LOGV_IF(PROCESS_VERBOSE, "HAL: batch - select sensor (handle %d)", handle);
    } else if (timeout > 0) {
        LOGV_IF(PROCESS_VERBOSE, "HAL: sensor (handle %d) does not support batch mode", handle);
        timeout = 0;
    }

    LOGV_IF(PROCESS_VERBOSE,
            "HAL:batch : %s %lld ns, (%.2f Hz) timeout=%lld ns",
            sensor->name, (long long)period_ns,
            NS_PER_SECOND_FLOAT / period_ns, (long long)timeout);

    /* return from here when dry run */
    if (flags & SENSORS_BATCH_DRY_RUN) {
        LOGI_IF(PROCESS_VERBOSE,
                "HAL:batch - dry run mode is set (%d)", SENSORS_BATCH_DRY_RUN);
        return 0;
    }

    /* get the current bias */
    bool old_gyro_lp = mGyroLpMode;
    bool old_accel_lp = mAccelLpMode;
    if (mEnabled) {
        /* only when sensors are enabled already */
        getDmpGyroBias(false);
        getDmpAccelBias(false);
    }

    int tempBatch = 0;
    if (timeout > 0) {
        tempBatch = mBatchEnabled | (1LL << what);
    } else {
        tempBatch = mBatchEnabled & ~(1LL << what);
    }

    if (!computeBatchSensorMask(mEnabled, tempBatch)) {
        batchMode = 0;
    } else {
        batchMode = 1;
    }

    /* starting from code below,  we will modify hardware */
    /* first edit global batch mode mask */

    if (timeout == 0) {
        mBatchEnabled &= ~(1LL << what);
        mBatchDelays[what] = NS_PER_SECOND;
        mDelays[what] = period_ns;
        mBatchTimeouts[what] = 100000000000LL;
    } else {
        mBatchEnabled |= (1LL << what);
        mBatchDelays[what] = period_ns;
        mDelays[what] = period_ns;
        mBatchTimeouts[what] = timeout;
    }

    if (((int)mOldBatchEnabledMask != batchMode) || batchMode) {
        /* remember batch mode that is set  */
        mOldBatchEnabledMask = batchMode;
        writeBatchTimeout(batchMode);
    }

    if (batchMode == 1) {
        /* set batch rates */
        if (setBatchDataRates() < 0) {
            LOGE("HAL:ERR can't set batch data rates");
        }
    } else {
        /* reset sensor rate */
        if (resetDataRates() < 0) {
            LOGE("HAL:ERR can't reset output rate back to original setting");
        }
    }

    /* set bias to algo according to new power mode */
    bool new_gyro_lp, new_accel_lp;
    getCurrentPowerMode(&new_gyro_lp, &new_accel_lp);
    mGyroLpMode = new_gyro_lp;
    if (new_gyro_lp != old_gyro_lp) {
        setDmpGyroBias(false);
    }
    mAccelLpMode = new_accel_lp;
    if (new_accel_lp != old_accel_lp) {
        setDmpAccelBias(false);
    }

    return res;
}

/* Send empty event when:        */
/* 1. batch mode is not enabled  */
/* 2. no data in HW FIFO         */
/* return status zero if (2)     */
int MPLSensor::flush(int handle)
{
    VFUNC_LOG;

    const struct sensor_t *sensor;
    int what = -1;

    if (!mChipDetected)
        return -EINVAL;

    getHandle(handle, what, sensor);
    if (what < 0) {
        LOGE("HAL:flush - what=%d is invalid", what);
        return -EINVAL;
    }

    LOGV_IF(PROCESS_VERBOSE, "HAL: flush - select sensor %s (handle %d)", sensor->name, handle);

    if ((sensor->flags & SENSOR_FLAG_MASK_REPORTING_MODE) == SENSOR_FLAG_ONE_SHOT_MODE) {
        LOGE_IF(ENG_VERBOSE, "HAL: flush - sensor %s is one-shot mode", sensor->name);
        return -EINVAL;
    }

    if (!(mEnabled & (1LL << what))) {
        LOGE_IF(ENG_VERBOSE, "HAL: flush - sensor %s not enabled", sensor->name);
        return -EINVAL;
    }

    if (!(mBatchEnabled & (1LL << what))) {
        LOGV_IF(PROCESS_VERBOSE, "HAL:flush - batch mode not enabled for sensor %s (handle %d)", sensor->name, handle);
    }

    /*write sysfs */
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            handle, mpu.flush_batch, (long long)getTimestamp());

    if (write_sysfs_int(mpu.flush_batch, handle) < 0) {
        LOGE("HAL:ERR can't write flush_batch");
    }

    return 0;
}

int MPLSensor::getDmpPedometerFd()
{
    VFUNC_LOG;
    LOGV_IF(EXTRA_VERBOSE, "getDmpPedometerFd returning %d", dmp_pedometer_fd);
    return dmp_pedometer_fd;
}

int MPLSensor::readDmpPedometerEvents(sensors_event_t* data, int count, int32_t id)
{
    VFUNC_LOG;

    char dummy[4];

    int numEventReceived = 0;
    int update = 0;

    LOGI_IF(0, "HAL: Read Pedometer Event ID=%d", id);
    switch (id) {
        case ID_P:
            if (mDmpPedometerEnabled && count > 0) {
                mPedUpdate = 1;
                /* Handles return event */
                LOGI("HAL: Step detected");
                update = sdHandler(&mPendingEvents[StepDetector]);
            }

            if (update && count > 0) {
                *data++ = mPendingEvents[StepDetector];
                count--;
                numEventReceived++;
            }
            break;
    }

    // read dummy data per driver's request
    // only required if actual irq is issued
    lseek(dmp_pedometer_fd, 0, SEEK_SET);
    int ret = read(dmp_pedometer_fd, dummy, 4);
    (void)ret;

    return numEventReceived;
}

int MPLSensor::getDmpSignificantMotionFd()
{
    VFUNC_LOG;

    LOGV_IF(EXTRA_VERBOSE, "getDmpSignificantMotionFd returning %d",
            dmp_sign_motion_fd);
    return dmp_sign_motion_fd;
}

int MPLSensor::readDmpSignificantMotionEvents(sensors_event_t* data, int count)
{
    VFUNC_LOG;

    char dummy[4];
    int significantMotion;
    FILE *fp;
    int numEventReceived = 0;

    /* Technically this step is not necessary for now  */
    /* In the future, we may have meaningful values */
    fp = fopen(mpu.event_smd, "r");
    if (fp == NULL) {
        LOGE("HAL:cannot open event_smd");
        return 0;
    } else {
        if (fscanf(fp, "%d\n", &significantMotion) < 0) {
            LOGE("HAL:cannot read event_smd");
        }
        fclose(fp);
    }
    LOGV_IF(EXTRA_VERBOSE,"HAL: Significant motion opened");

    if (mDmpSignificantMotionEnabled && count > 0) {
        /* By implementation, smd is disabled once an event is triggered */
        /* Handles return event */
        LOGI("HAL: SMD detected");
        int update = smHandler(&mPendingEvents[SignificantMotion]);
        if (update && count > 0) {
            *data++ = mPendingEvents[SignificantMotion];
            count--;
            numEventReceived++;

            /* reset smd state */
            mDmpSignificantMotionEnabled = 0;
            mFeatureActiveMask &= ~INV_DMP_SIGNIFICANT_MOTION;

            /* auto disable this sensor */
            enableDmpSignificantMotion(0);
        }
    }

    // read dummy data per driver's request
    lseek(dmp_sign_motion_fd, 0, SEEK_SET);
    int ret = read(dmp_sign_motion_fd, dummy, 4);
    (void)ret;

    return numEventReceived;
}

int MPLSensor::enableDmpSignificantMotion(int en)
{
    VFUNC_LOG;

    int res = 0;

    //Toggle significant montion detection
    if (en) {
        LOGV_IF(ENG_VERBOSE, "HAL:Enabling Significant Motion");
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                1, mpu.smd_enable, (long long)getTimestamp());
        if (write_sysfs_int(mpu.smd_enable, 1) < 0) {
            LOGE("HAL:ERR can't write DMP smd_enable");
            res = -1;
        }
        mFeatureActiveMask |= INV_DMP_SIGNIFICANT_MOTION;
    } else {
        LOGV_IF(ENG_VERBOSE, "HAL:Disabling Significant Motion");
        LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
                0, mpu.smd_enable, (long long)getTimestamp());
        if (write_sysfs_int(mpu.smd_enable, 0) < 0) {
            LOGE("HAL:ERR write DMP smd_enable");
        }
        mFeatureActiveMask &= ~INV_DMP_SIGNIFICANT_MOTION;
    }

    return res;
}

/* set batch data rate */
/* this function should be optimized */
int MPLSensor::setBatchDataRates()
{
    VFUNC_LOG;

    int res = 0;
    int64_t wanted = NS_PER_SECOND;

    int64_t gyroRate = wanted;
    int64_t accelRate = wanted;
    int64_t compassRate = wanted;
    int64_t pressureRate = wanted;
    int64_t quatRate = wanted;
    int64_t nineAxisRate = wanted;
    int64_t compassSixAxisRate = wanted;

    int64_t gyroWakeRate = wanted;
    int64_t accelWakeRate = wanted;
    int64_t compassWakeRate = wanted;
    int64_t quatWakeRate = wanted;
    int64_t nineAxisWakeRate = wanted;
    int64_t compassSixAxisWakeRate = wanted;

    int mplGyroRate = wanted / 1000LL;
    int mplAccelRate = wanted / 1000LL;
    int mplCompassRate = wanted / 1000LL;
    int mplQuatRate = wanted / 1000LL;
    int mplQuatWakeRate = wanted / 1000LL;
    int mplNineAxisQuatRate = wanted / 1000LL;
    int mplNineAxisQuatWakeRate = wanted / 1000LL;
    int mplCompassQuatRate = wanted / 1000LL;
    int mplCompassQuatWakeRate = wanted / 1000LL;
    uint32_t i;

    /* take care of case where only one type of gyro sensors or
       compass sensors is turned on */
    /* Gyro */
    if (mBatchEnabled & (1LL << Gyro) || mBatchEnabled & (1LL << RawGyro)) {
        gyroRate = (mBatchDelays[Gyro] <= mBatchDelays[RawGyro]) ?
            (mBatchEnabled & (1LL << Gyro) ? mBatchDelays[Gyro] : mBatchDelays[RawGyro]):
            (mBatchEnabled & (1LL << RawGyro) ? mBatchDelays[RawGyro] : mBatchDelays[Gyro]);
    } else {
        mBatchDelays[Gyro] = wanted;
        mBatchDelays[RawGyro] = wanted;
    }

    /* Gyro Wakeup */
    if (mBatchEnabled & (1LL << Gyro_Wake) || mBatchEnabled & (1LL << RawGyro_Wake)) {
        gyroWakeRate = (mBatchDelays[Gyro_Wake] <= mBatchDelays[RawGyro_Wake]) ?
            (mBatchEnabled & (1LL << Gyro_Wake) ? mBatchDelays[Gyro_Wake] : mBatchDelays[RawGyro_Wake]):
            (mBatchEnabled & (1LL << RawGyro_Wake) ? mBatchDelays[RawGyro_Wake] : mBatchDelays[Gyro_Wake]);
    } else {
        mBatchDelays[Gyro_Wake] = wanted;
        mBatchDelays[RawGyro_Wake] = wanted;
    }
    gyroRate = gyroRate < gyroWakeRate ? gyroRate : gyroWakeRate;

    /* Mag */
    if (mBatchEnabled & (1LL << MagneticField) || mBatchEnabled & (1LL << RawMagneticField)) {
        compassRate = (mBatchDelays[MagneticField] <= mBatchDelays[RawMagneticField]) ?
            (mBatchEnabled & (1LL << MagneticField) ? mBatchDelays[MagneticField] :
             mBatchDelays[RawMagneticField]) :
            (mBatchEnabled & (1LL << RawMagneticField) ? mBatchDelays[RawMagneticField] :
             mBatchDelays[MagneticField]);
    } else {
        mBatchDelays[RawMagneticField] = wanted;
        mBatchDelays[MagneticField] = wanted;
    }

    /* Mag Wakeup */
    if (mBatchEnabled & (1LL << MagneticField_Wake) || mBatchEnabled & (1LL << RawMagneticField_Wake)) {
        compassRate = (mBatchDelays[MagneticField_Wake] <= mBatchDelays[RawMagneticField_Wake]) ?
            (mBatchEnabled & (1LL << MagneticField_Wake) ? mBatchDelays[MagneticField_Wake] :
             mBatchDelays[RawMagneticField_Wake]) :
            (mBatchEnabled & (1LL << RawMagneticField_Wake) ? mBatchDelays[RawMagneticField_Wake] :
             mBatchDelays[MagneticField_Wake]);
    } else {
        mBatchDelays[RawMagneticField_Wake] = wanted;
        mBatchDelays[MagneticField_Wake] = wanted;
    }
    compassRate = compassRate < compassWakeRate ? compassRate : compassWakeRate;

    /* Accel */
    if (mBatchEnabled & (1LL << Accelerometer) || mBatchEnabled & (1LL << RawAccelerometer)) {
        accelRate = (mBatchDelays[Accelerometer] <= mBatchDelays[RawAccelerometer]) ?
            (mBatchEnabled & (1LL << Accelerometer) ? mBatchDelays[Accelerometer] : mBatchDelays[RawAccelerometer]):
            (mBatchEnabled & (1LL << RawAccelerometer) ? mBatchDelays[RawAccelerometer] : mBatchDelays[Accelerometer]);
    } else {
        mBatchDelays[Accelerometer] = wanted;
        mBatchDelays[RawAccelerometer] = wanted;
    }

    /* Accel Wakeup */
    if (mBatchEnabled & (1LL << Accelerometer_Wake) || mBatchEnabled & (1LL << RawAccelerometer_Wake)) {
        accelWakeRate = (mBatchDelays[Accelerometer_Wake] <= mBatchDelays[RawAccelerometer_Wake]) ?
            (mBatchEnabled & (1LL << Accelerometer_Wake) ? mBatchDelays[Accelerometer_Wake] : mBatchDelays[RawAccelerometer_Wake]):
            (mBatchEnabled & (1LL << RawAccelerometer_Wake) ? mBatchDelays[RawAccelerometer_Wake] : mBatchDelays[Accelerometer_Wake]);
    } else {
        mBatchDelays[Accelerometer_Wake] = wanted;
        mBatchDelays[RawAccelerometer_Wake] = wanted;
    }
    accelRate = accelRate < accelWakeRate ? accelRate : accelWakeRate;

    /* Pressure */
    if (mBatchEnabled & (1LL << Pressure) || mBatchEnabled & (1LL << Pressure_Wake)) {
        pressureRate = mBatchDelays[Pressure] < mBatchDelays[Pressure_Wake] ?
            mBatchDelays[Pressure] : mBatchDelays[Pressure_Wake];
    } else {
        mBatchDelays[Pressure] = wanted;
        mBatchDelays[Pressure_Wake] = wanted;
    }

    int64_t tempQuatRate = mBatchDelays[GameRotationVector] < mBatchDelays[GameRotationVector_Wake] ?
            mBatchDelays[GameRotationVector] : mBatchDelays[GameRotationVector_Wake];
    if ((mCalibrationMode & mpl_quat) ||
                (mFeatureActiveMask & INV_DMP_6AXIS_QUATERNION) ||
        (mFeatureActiveMask & INV_DMP_6AXIS_QUATERNION_WAKE)) {
        if ((mBatchEnabled & (1LL << GameRotationVector)) ||
            (mBatchEnabled & (1LL << GameRotationVector_Wake)) ||
            (mBatchEnabled & (1LL << LinearAccel)) ||
            (mBatchEnabled & (1LL << LinearAccel_Wake)) ||
            (mBatchEnabled & (1LL << Gravity)) ||
            (mBatchEnabled & (1LL << Gravity_Wake))) {
            quatRate = mBatchDelays[GameRotationVector];
            quatRate = (mBatchDelays[LinearAccel] < quatRate) ? mBatchDelays[LinearAccel] : quatRate;
            quatRate = (mBatchDelays[Gravity] < quatRate) ? mBatchDelays[Gravity] : quatRate;
            quatWakeRate = mBatchDelays[GameRotationVector_Wake];
            quatWakeRate = (mBatchDelays[LinearAccel_Wake] < quatWakeRate) ? mBatchDelays[LinearAccel_Wake] : quatWakeRate;
            quatWakeRate = (mBatchDelays[Gravity_Wake] < quatWakeRate) ? mBatchDelays[Gravity_Wake] : quatWakeRate;
            mplQuatRate = (int) quatRate / 1000LL;
            mplQuatRate = lookupSensorRate(mplQuatRate);
            mplQuatWakeRate = (int) quatWakeRate / 1000LL;
            mplQuatWakeRate = lookupSensorRate(mplQuatWakeRate);
            adapter_set_sample_rate(mplQuatRate, ID_GRV);
            adapter_set_sample_rate(mplQuatWakeRate, ID_GRVW);
            set6AxisQuaternionRate(mplQuatRate * 1000LL);
            set6AxisQuaternionWakeRate(mplQuatWakeRate * 1000LL);
            LOGV_IF(EXTRA_VERBOSE,
                    "HAL:Batch rv final sample rate: %lld", (long long)tempQuatRate);
            LOGV_IF(EXTRA_VERBOSE,"HAL:Batch grv sample rate: %lld - %d",
                    (long long)quatRate, mplQuatRate);
            LOGV_IF(EXTRA_VERBOSE,"HAL:Batch grv wake sample rate: %lld - %d",
                    (long long)quatWakeRate, mplQuatWakeRate);
        } else {
            mBatchDelays[GameRotationVector] = wanted;
            mBatchDelays[LinearAccel] = wanted;
            mBatchDelays[Gravity] = wanted;
            mBatchDelays[GameRotationVector_Wake] = wanted;
            mBatchDelays[LinearAccel_Wake] = wanted;
            mBatchDelays[Gravity_Wake] = wanted;
        }
    } else {
        mplQuatRate = (int) tempQuatRate / 1000LL;
        mplQuatRate = lookupSensorRate(mplQuatRate);
        quatRate = mplQuatRate * 1000LL;
        adapter_set_sample_rate(mplQuatRate, ID_GRV);
        adapter_set_sample_rate(mplQuatRate, ID_GRVW);
    }

    mplGyroRate = (int) gyroRate / 1000LL;
    mplAccelRate = (int) accelRate / 1000LL;
    mplCompassRate = (int) compassRate / 1000LL;

    /* converts to ICM20648 rate */
    mplGyroRate = lookupSensorRate(mplGyroRate);
    mplAccelRate = lookupSensorRate(mplAccelRate);
    mplCompassRate = lookupSensorRate(mplCompassRate);

    gyroRate = mplGyroRate * 1000LL;
    accelRate = mplAccelRate * 1000LL;
    compassRate = mplCompassRate * 1000LL;

    /* set rate in MPL */
    /* compass can only do 100Hz max */
    adapter_set_sample_rate(mplGyroRate, ID_GY);
    adapter_set_sample_rate(mplAccelRate, ID_A);
    adapter_set_sample_rate(mplCompassRate, ID_M);

    adapter_set_sample_rate(mplGyroRate, ID_GYW);
    adapter_set_sample_rate(mplAccelRate, ID_AW);
    adapter_set_sample_rate(mplCompassRate, ID_MW);

    LOGV_IF(PROCESS_VERBOSE,
            "HAL:MPL gyro sample rate: (mpl)=%d us (mpu)=%.2f Hz", mplGyroRate, NS_PER_SECOND_FLOAT / gyroRate);
    LOGV_IF(PROCESS_VERBOSE,
            "HAL:MPL accel sample rate: (mpl)=%d us (mpu)=%.2f Hz", mplAccelRate, NS_PER_SECOND_FLOAT / accelRate);
    LOGV_IF(PROCESS_VERBOSE,
            "HAL:MPL compass sample rate: (mpl)=%d us (mpu)=%.2f Hz", mplCompassRate, NS_PER_SECOND_FLOAT / compassRate);
    LOGV_IF(PROCESS_VERBOSE,
            "HAL:MPL quat sample rate: (mpl)=%d us (mpu)=%.2f Hz", mplQuatRate, NS_PER_SECOND_FLOAT / quatRate);

    if (!(mCalibrationMode & mpl_quat)) {
        if (mBatchEnabled & (1LL << RotationVector) ||
            mBatchEnabled & (1LL << RotationVector_Wake) ||
            mBatchEnabled & (1LL << Orientation) ||
            mBatchEnabled & (1LL << Orientation_Wake) ||
            mBatchEnabled & (1LL << Heading) ||
            mBatchEnabled & (1LL << Heading_Wake)) {
            nineAxisRate = mBatchDelays[RotationVector];
            nineAxisRate = (mBatchDelays[Orientation] < nineAxisRate) ? mBatchDelays[Orientation] : nineAxisRate;
            nineAxisRate = (mBatchDelays[Heading] < nineAxisRate) ? mBatchDelays[Heading] : nineAxisRate;
            nineAxisWakeRate = mBatchDelays[RotationVector_Wake];
            nineAxisWakeRate = (mBatchDelays[Orientation_Wake] < nineAxisWakeRate) ? mBatchDelays[Orientation_Wake] : nineAxisWakeRate;
            nineAxisWakeRate = (mBatchDelays[Heading_Wake] < nineAxisWakeRate) ? mBatchDelays[Heading_Wake] : nineAxisWakeRate;
        } else {
            mBatchDelays[RotationVector] = wanted;
            mBatchDelays[RotationVector_Wake] = wanted;
            mBatchDelays[Orientation] = wanted;
            mBatchDelays[Orientation_Wake] = wanted;
            mBatchDelays[Heading] = wanted;
            mBatchDelays[Heading_Wake] = wanted;
        }

        if (mBatchEnabled & (1LL << GeomagneticRotationVector) || mBatchEnabled & (1LL << GeomagneticRotationVector_Wake)) {
            compassSixAxisRate = mBatchDelays[GeomagneticRotationVector];
            compassSixAxisWakeRate = mBatchDelays[GeomagneticRotationVector_Wake];
        } else {
            mBatchDelays[GeomagneticRotationVector] = wanted;
            mBatchDelays[GeomagneticRotationVector_Wake] = wanted;
        }

        mplNineAxisQuatRate = (int) nineAxisRate / 1000LL;
        mplNineAxisQuatWakeRate = (int) nineAxisWakeRate / 1000LL;
        mplCompassQuatRate = (int) compassSixAxisRate / 1000LL;
        mplCompassQuatWakeRate = (int) compassSixAxisWakeRate / 1000LL;

        /* converts to ICM20648 rate */
        mplNineAxisQuatRate = lookupSensorRate(mplNineAxisQuatRate);
        mplNineAxisQuatWakeRate = lookupSensorRate(mplNineAxisQuatWakeRate);
        mplCompassQuatRate = lookupSensorRate(mplCompassQuatRate);
        mplCompassQuatWakeRate = lookupSensorRate(mplCompassQuatWakeRate);

        nineAxisRate = mplNineAxisQuatRate * 1000LL;
        nineAxisWakeRate = mplNineAxisQuatWakeRate * 1000LL;
        compassSixAxisRate = mplCompassQuatRate * 1000LL;
        compassSixAxisWakeRate = mplCompassQuatWakeRate * 1000LL;

        adapter_set_sample_rate((int)mplNineAxisQuatRate, ID_RV);
        adapter_set_sample_rate((int)mplNineAxisQuatWakeRate, ID_RVW);
        adapter_set_sample_rate((int)mplCompassQuatRate, ID_GMRV);
        adapter_set_sample_rate((int)mplCompassQuatWakeRate, ID_GMRVW);

        LOGV_IF(PROCESS_VERBOSE,
            "HAL:MPL nine quat sample rate: (mpl)=%d us (mpu)=%.2f Hz", mplNineAxisQuatRate, NS_PER_SECOND_FLOAT / nineAxisRate);
        LOGV_IF(PROCESS_VERBOSE,
            "HAL:MPL compass quat sample rate: (mpl)=%d us (mpu)=%.2f Hz", mplCompassQuatRate, NS_PER_SECOND_FLOAT / compassSixAxisRate);
        LOGV_IF(PROCESS_VERBOSE,
            "HAL:MPL nine quat wake sample rate: (mpl)=%d us (mpu)=%.2f Hz", mplNineAxisQuatWakeRate, NS_PER_SECOND_FLOAT / nineAxisWakeRate);
        LOGV_IF(PROCESS_VERBOSE,
            "HAL:MPL compass quat wake sample rate: (mpl)=%d us (mpu)=%.2f Hz", mplCompassQuatWakeRate, NS_PER_SECOND_FLOAT / compassSixAxisWakeRate);
    }

    for (i = 0; i < mNumSysfs; i++) {
        if (mSysfsMask[i].sensorMask) {
            switch (mSysfsMask[i].sensorMask) {
            case INV_THREE_AXIS_GYRO:
            case INV_THREE_AXIS_RAW_GYRO:
                mSysfsMask[i].nominalRate = gyroRate;
                break;
            case INV_THREE_AXIS_GYRO_WAKE:
            case INV_THREE_AXIS_RAW_GYRO_WAKE:
                mSysfsMask[i].nominalRate = gyroWakeRate;
                break;
            case INV_THREE_AXIS_ACCEL:
                mSysfsMask[i].nominalRate = accelRate;
                break;
            case INV_THREE_AXIS_ACCEL_WAKE:
                mSysfsMask[i].nominalRate = accelWakeRate;
                break;
            case INV_THREE_AXIS_COMPASS:
            case INV_THREE_AXIS_COMPASS_WAKE:
            case INV_THREE_AXIS_RAW_COMPASS:
            case INV_THREE_AXIS_RAW_COMPASS_WAKE:
                mSysfsMask[i].nominalRate = compassRate;
                break;
            case VIRTUAL_SENSOR_9AXES_MASK:
                mSysfsMask[i].nominalRate = nineAxisRate;
                break;
            case VIRTUAL_SENSOR_9AXES_MASK_WAKE:
                mSysfsMask[i].nominalRate = nineAxisWakeRate;
                break;
            case VIRTUAL_SENSOR_GYRO_6AXES_MASK:
                mSysfsMask[i].nominalRate = quatRate;
                break;
            case VIRTUAL_SENSOR_GYRO_6AXES_MASK_WAKE:
                mSysfsMask[i].nominalRate = quatWakeRate;
                break;
            case VIRTUAL_SENSOR_MAG_6AXES_MASK:
                mSysfsMask[i].nominalRate = compassSixAxisRate;
                break;
            case VIRTUAL_SENSOR_MAG_6AXES_MASK_WAKE:
                mSysfsMask[i].nominalRate = compassSixAxisWakeRate;
                break;
            case INV_ONE_AXIS_PRESSURE:
            case INV_ONE_AXIS_PRESSURE_WAKE:
                mSysfsMask[i].nominalRate = pressureRate;
                break;
            case INV_ONE_AXIS_LIGHT:
                break;
            default:
                break;
            }
        }
    }

   for (i = 0; i < mNumSysfs; i++) {
       if (mSysfsMask[i].sensorMask && mSysfsMask[i].setRate && mSysfsMask[i].en) {
        (this->*(mSysfsMask[i].setRate))(mSysfsMask[i].nominalRate);
       }
   }

    return res;
}

/* Set sensor rate */
/* this function should be optimized */
int MPLSensor::resetDataRates()
{
    VFUNC_LOG;

    int64_t wanted = NS_PER_SECOND;
    int64_t ns =  wanted;
    uint32_t i, j;
    int64_t gestureRate = wanted;
    int64_t engineRate = wanted;

    LOGV_IF(ENG_VERBOSE,"HAL:resetDataRates mEnabled=%lld", (long long)mEnabled);

    if (!(mCalibrationMode & dmp)) {
        /* search the minimum delay requested across all enabled sensors */
        /* skip setting rates if it is not changed */
        for (i = 0; i < ID_NUMBER; i++) {
            if (mEnabled & (1LL << i)) {
                ns = mDelays[i];
                if ((wanted == ns) && (i != Pressure)) {
                    LOGV_IF(ENG_VERBOSE, "skip resetDataRates : same delay mDelays[%d]=%lld", i, (long long)mDelays[i]);
                }
                if (i != StepDetector && i != StepDetector_Wake && i != StepCounter &&
                        i != StepCounter_Wake && i != Tilt &&  i != Pickup &&
                        i != StationaryDetect && i != MotionDetect) {
                    LOGV_IF(ENG_VERBOSE, "resetDataRates - mDelays[%d]=%lld", i, (long long)mDelays[i]);
                    wanted = wanted < mDelays[i] ? wanted : mDelays[i];
                }
            }
        }
        LOGV_IF(ENG_VERBOSE,"HAL:resetDataRates wanted=%lld", (long long)wanted);
        wanted = lookupSensorRate(wanted/1000) * 1000;
   }

   for (i = 0; i < mNumSysfs; i++)
        mSysfsMask[i].nominalRate = mSysfsMask[i].minimumNonBatchRate;

   for (i = 0; i < ID_NUMBER; i++) {
       if (mEnabled & (1LL << i)) {
           int64_t ns = mDelays[i];
           wanted = lookupSensorRate(ns/1000) * 1000;
           for (j = 0; j < mNumSysfs; j++) {
                if (mSysfsMask[j].sensorMask & mCurrentSensorMask[i].sensorMask) {
                    if (mSysfsMask[j].nominalRate > wanted) {
                        mSysfsMask[j].nominalRate = wanted;
                    }
                }
           }
           LOGV_IF(ENG_VERBOSE,"HAL:resetDataRates sensor %s wanted=%lld",
                mCurrentSensorMask[i].sname.c_str(), (long long)wanted);
       }
    }

    /* set minimum rate for Game RV, LA and Gravity sensor */
    if (mEnabled & ((1LL << SENSORS_GAME_ROTATION_VECTOR_HANDLE)
                 | (1LL << SENSORS_LINEAR_ACCEL_HANDLE)
                 | (1LL << SENSORS_GRAVITY_HANDLE)
                 | (1LL << SENSORS_GAME_ROTATION_VECTOR_WAKEUP_HANDLE)
                 | (1LL << SENSORS_LINEAR_ACCEL_WAKEUP_HANDLE)
                 | (1LL << SENSORS_GRAVITY_WAKEUP_HANDLE))) {
        if (mCalibrationMode & mpl_quat) {
            for (i = 0; i < mNumSysfs; i++) {
                if ((mSysfsMask[i].sensorMask == INV_THREE_AXIS_GYRO) ||
                    (mSysfsMask[i].sensorMask == INV_THREE_AXIS_GYRO_WAKE) ||
                    (mSysfsMask[i].sensorMask == INV_THREE_AXIS_RAW_GYRO) ||
                    (mSysfsMask[i].sensorMask == INV_THREE_AXIS_RAW_GYRO_WAKE) ||
                    (mSysfsMask[i].sensorMask == INV_THREE_AXIS_ACCEL) ||
                    (mSysfsMask[i].sensorMask == INV_THREE_AXIS_ACCEL_WAKE)) {
                    if (mSysfsMask[i].nominalRate > MIN_SENSOR_FUSION_RATE)
                        mSysfsMask[i].nominalRate = MIN_SENSOR_FUSION_RATE;
                    LOGV_IF(PROCESS_VERBOSE,"6axes gyro/accel rate = %lld\n", NS_PER_SECOND/mSysfsMask[i].nominalRate);
                }
            }
        }
    }

    if (mEnabled & ((1LL << SENSORS_ORIENTATION_HANDLE)
                 | (1LL << SENSORS_ROTATION_VECTOR_HANDLE)
                 | (1LL << SENSORS_HEADING_HANDLE)
                 | (1LL << SENSORS_ORIENTATION_WAKEUP_HANDLE)
                 | (1LL << SENSORS_ROTATION_VECTOR_WAKEUP_HANDLE)
                 | (1LL << SENSORS_HEADING_WAKEUP_HANDLE))) {
        if (mCalibrationMode & mpl_quat) {
            for (i = 0; i < mNumSysfs; i++) {
                if ((mSysfsMask[i].sensorMask == INV_THREE_AXIS_RAW_COMPASS) ||
                    (mSysfsMask[i].sensorMask == INV_THREE_AXIS_RAW_COMPASS_WAKE)) {
                    if (mSysfsMask[i].nominalRate > MIN_SENSOR_FUSION_RATE)
                        mSysfsMask[i].nominalRate = MIN_SENSOR_FUSION_RATE;
                    LOGV_IF(PROCESS_VERBOSE, "9axes Mag rate = %lld\n", NS_PER_SECOND/mSysfsMask[i].nominalRate);
                }
                //set gyro/accel minimum rate for RV and Orientation sensor
                if ((mSysfsMask[i].sensorMask == INV_THREE_AXIS_GYRO) ||
                    (mSysfsMask[i].sensorMask == INV_THREE_AXIS_GYRO_WAKE) ||
                    (mSysfsMask[i].sensorMask == INV_THREE_AXIS_RAW_GYRO) ||
                    (mSysfsMask[i].sensorMask == INV_THREE_AXIS_RAW_GYRO_WAKE) ||
                    (mSysfsMask[i].sensorMask == INV_THREE_AXIS_ACCEL) ||
                    (mSysfsMask[i].sensorMask == INV_THREE_AXIS_ACCEL_WAKE)) {
                        if (mSysfsMask[i].nominalRate > MIN_SENSOR_FUSION_RATE)
                            mSysfsMask[i].nominalRate = MIN_SENSOR_FUSION_RATE;
                        LOGV_IF(PROCESS_VERBOSE,"9axes gyro/accel rate = %lld\n", NS_PER_SECOND/mSysfsMask[i].nominalRate);
                }
            }
        }
    }

    if (mEnabled & ((1LL << SENSORS_GEOMAGNETIC_ROTATION_VECTOR_HANDLE)
                 | (1LL << SENSORS_GEOMAGNETIC_ROTATION_VECTOR_WAKEUP_HANDLE))) {
        if (mCalibrationMode & mpl_quat) {
            for (i = 0; i < mNumSysfs; i++) {
                if ((mSysfsMask[i].sensorMask == INV_THREE_AXIS_RAW_COMPASS) ||
                   (mSysfsMask[i].sensorMask == INV_THREE_AXIS_RAW_COMPASS_WAKE) ||
                   (mSysfsMask[i].sensorMask == INV_THREE_AXIS_ACCEL) ||
                   (mSysfsMask[i].sensorMask == INV_THREE_AXIS_ACCEL_WAKE)) {
                    if (mSysfsMask[i].nominalRate > MIN_MAG_FUSION_RATE)
                        mSysfsMask[i].nominalRate = MIN_MAG_FUSION_RATE;
                    LOGV_IF(PROCESS_VERBOSE, "Geomag Mag rate = %lld\n", NS_PER_SECOND/mSysfsMask[i].nominalRate);
                }
            }
        }
    }
    for (i = 0; i < mNumSysfs; i++)
        mSysfsMask[i].engineRate = NS_PER_SECOND;

    for (i = 0; i < mNumSysfs; i++) {
        if (mSysfsMask[i].sensorMask && mSysfsMask[i].setRate && mSysfsMask[i].en) {
            (this->*(mSysfsMask[i].setRate))(mSysfsMask[i].nominalRate);
       }
    }

    /* set mpl data rate */
    for (i = 0; i < mNumSysfs; i++) {
        if (mSysfsMask[i].sensorMask && mSysfsMask[i].en) {
                engineRate = mSysfsMask[i].engineRate;
            switch (mSysfsMask[i].sensorMask) {
                case INV_THREE_AXIS_GYRO:
                case INV_THREE_AXIS_RAW_GYRO:
                case INV_THREE_AXIS_GYRO_WAKE:
                case INV_THREE_AXIS_RAW_GYRO_WAKE:
                    adapter_set_sample_rate((int)engineRate/1000LL, ID_GY);
                    adapter_set_sample_rate((int)engineRate/1000LL, ID_GYW);
                    LOGV_IF(PROCESS_VERBOSE,
                        "HAL:MPL gyro sample rate: (mpl)=%lld us (mpu)=%.2f Hz",
                        engineRate/1000LL, NS_PER_SECOND_FLOAT / engineRate);
                    LOGV_IF(PROCESS_VERBOSE,
                        "HAL:MPL gyro wake sample rate: (mpl)=%lld us (mpu)=%.2f Hz",
                         engineRate/1000LL, NS_PER_SECOND_FLOAT / engineRate);
                    break;
                case INV_THREE_AXIS_ACCEL:
                case INV_THREE_AXIS_ACCEL_WAKE:
                    adapter_set_sample_rate((int)engineRate/1000LL, ID_A);
                    adapter_set_sample_rate((int)engineRate/1000LL, ID_AW);
                    LOGV_IF(PROCESS_VERBOSE,
                        "HAL:MPL accel wake sample rate: (mpl)=%lld us (mpu)=%.2f Hz",
                        engineRate/1000LL, NS_PER_SECOND_FLOAT / engineRate);
                    LOGV_IF(PROCESS_VERBOSE,
                        "HAL:MPL accel sample rate: (mpl)=%lld us (mpu)=%.2f Hz",
                        engineRate/1000LL, NS_PER_SECOND_FLOAT / engineRate);
                    break;
                case INV_THREE_AXIS_COMPASS:
                case INV_THREE_AXIS_RAW_COMPASS:
                case INV_THREE_AXIS_COMPASS_WAKE:
                case INV_THREE_AXIS_RAW_COMPASS_WAKE:
                    adapter_set_sample_rate((int)engineRate/1000LL, ID_M);
                    LOGV_IF(PROCESS_VERBOSE,
                            "HAL:MPL compass sample rate: (mpl)=%lld us (mpu)=%.2f Hz",
                            engineRate/1000LL, NS_PER_SECOND_FLOAT / engineRate);
                    LOGV_IF(PROCESS_VERBOSE,
                            "HAL:MPL compass wake sample rate: (mpl)=%lld us (mpu)=%.2f Hz",
                             engineRate/1000LL, NS_PER_SECOND_FLOAT / engineRate);
                    break;
                case VIRTUAL_SENSOR_9AXES_MASK:
                case VIRTUAL_SENSOR_9AXES_MASK_WAKE:
                    adapter_set_orientation_sample_rate((int)engineRate/1000LL, ID_O);
                    adapter_set_sample_rate((int)engineRate/1000LL, ID_RV);
                    adapter_set_sample_rate((int)engineRate/1000LL, ID_RVW);
                    LOGV_IF(PROCESS_VERBOSE,
                            "HAL:MPL 9quat sample rate: (mpl)=%lld us (mpu)=%.2f Hz",
                             engineRate/1000LL, NS_PER_SECOND_FLOAT / engineRate);
                    LOGV_IF(PROCESS_VERBOSE,
                             "HAL:MPL 9quat wake sample rate: (mpl)=%lld us (mpu)=%.2f Hz",
                             engineRate/1000LL, 1000000.f / engineRate);
                    break;
                case VIRTUAL_SENSOR_GYRO_6AXES_MASK:
                case VIRTUAL_SENSOR_GYRO_6AXES_MASK_WAKE:
                        adapter_set_linear_acceleration_sample_rate((int)engineRate/1000LL, ID_LA);
                        adapter_set_gravity_sample_rate((int)engineRate/1000LL, ID_GR);
                        adapter_set_sample_rate((int)engineRate/1000LL, ID_GRV);
                        adapter_set_sample_rate((int)engineRate/1000LL, ID_GRVW);
                        LOGV_IF(PROCESS_VERBOSE,
                            "HAL:MPL 6quat sample rate: (mpl)=%lld us (mpu)=%.2f Hz",
                             engineRate/1000LL, NS_PER_SECOND_FLOAT / engineRate);
                    break;
                case VIRTUAL_SENSOR_MAG_6AXES_MASK:
                case VIRTUAL_SENSOR_MAG_6AXES_MASK_WAKE:
                        adapter_set_sample_rate((int)engineRate/1000LL, ID_GMRV);
                        adapter_set_sample_rate((int)engineRate/1000LL, ID_GMRVW);
                        LOGV_IF(PROCESS_VERBOSE,
                            "HAL:MPL mag quat sample rate: (mpl)=%lld us (mpu)=%.2f Hz",
                             engineRate/1000LL, NS_PER_SECOND_FLOAT / engineRate);
                    break;
                default:
                    break;
            }
        }
    }

    /* set mpl data rate for gesture algo */
    if (mCalibrationMode & (mpl_smd | mpl_pedo | mpl_pickup | mpl_tilt | mpl_tap | mpl_md)) {
        gestureRate = 20000000LL;
        for (i = 0; i < mNumSysfs; i++) {
            if (mSysfsMask[i].en && ((mSysfsMask[i].sensorMask & INV_THREE_AXIS_ACCEL)
                || (mSysfsMask[i].sensorMask & INV_THREE_AXIS_ACCEL_WAKE))){
                if (gestureRate > mSysfsMask[i].engineRate)
                    gestureRate = mSysfsMask[i].engineRate;
            }
        }
        adapter_set_sample_rate(gestureRate/1000LL, ID_P); // gesture algothms are sharing sampling rate
        LOGV_IF(PROCESS_VERBOSE, "gestureRate = %llu", gestureRate/1000LL);

        adapter_set_sample_rate((int)gestureRate/1000LL, ID_A);
        adapter_set_sample_rate((int)gestureRate/1000LL, ID_AW);
        LOGV_IF(PROCESS_VERBOSE,
                          "HAL:MPL accel wake sample rate: (mpl)=%lld us (mpu)=%.2f Hz",
                            gestureRate/1000LL, NS_PER_SECOND_FLOAT / gestureRate);

        LOGV_IF(PROCESS_VERBOSE,
                          "HAL:MPL accel sample rate: (mpl)=%lld us (mpu)=%.2f Hz",
                                 gestureRate/1000LL, NS_PER_SECOND_FLOAT / gestureRate);
    }

   inv_set_quat_sample_rate(ISENSOR_RATE_220HZ);

   return 0;
}

void MPLSensor::initBias()
{
    VFUNC_LOG;

    if (!(mCalibrationMode & mpl_accel_cal)) {
        LOGV_IF(ENG_VERBOSE, "HAL:inititalize dmp and device offsets to 0");
        if (write_attribute_sensor_continuous(accel_x_dmp_bias_fd, 0) < 0) {
            LOGE("HAL:Error writing to accel_x_dmp_bias");
        }
        if (write_attribute_sensor_continuous(accel_y_dmp_bias_fd, 0) < 0) {
            LOGE("HAL:Error writing to accel_y_dmp_bias");
        }
        if (write_attribute_sensor_continuous(accel_z_dmp_bias_fd, 0) < 0) {
            LOGE("HAL:Error writing to accel_z_dmp_bias");
        }
    }

    if (write_attribute_sensor_continuous(accel_x_offset_fd, 0) < 0) {
        LOGE("HAL:Error writing to accel_x_offset");
    }
    if (write_attribute_sensor_continuous(accel_y_offset_fd, 0) < 0) {
        LOGE("HAL:Error writing to accel_y_offset");
    }
    if (write_attribute_sensor_continuous(accel_z_offset_fd, 0) < 0) {
        LOGE("HAL:Error writing to accel_z_offset");
    }

    if (!(mCalibrationMode & mpl_gyro_cal)) {
        if (write_attribute_sensor_continuous(gyro_x_dmp_bias_fd, 0) < 0) {
            LOGE("HAL:Error writing to gyro_x_dmp_bias");
        }
        if (write_attribute_sensor_continuous(gyro_y_dmp_bias_fd, 0) < 0) {
            LOGE("HAL:Error writing to gyro_y_dmp_bias");
        }
        if (write_attribute_sensor_continuous(gyro_z_dmp_bias_fd, 0) < 0) {
            LOGE("HAL:Error writing to gyro_z_dmp_bias");
        }
    }

    if (write_attribute_sensor_continuous(gyro_x_offset_fd, 0) < 0) {
        LOGE("HAL:Error writing to gyro_x_offset");
    }
    if (write_attribute_sensor_continuous(gyro_y_offset_fd, 0) < 0) {
        LOGE("HAL:Error writing to gyro_y_offset");
    }
    if (write_attribute_sensor_continuous(gyro_z_offset_fd, 0) < 0) {
        LOGE("HAL:Error writing to gyro_z_offset");
    }

    return;
}

int MPLSensor::lookupSensorRate(int inputRateUs) const
{
    if (adapter_get_internal_sampling_rate(chip_ID) == 1000)
        return inputRateUs;

    switch (inputRateUs) {
        case ASENSOR_RATE_200HZ:
            return ISENSOR_RATE_220HZ;
        case ASENSOR_RATE_100HZ:
            return ISENSOR_RATE_110HZ;
        case ASENSOR_RATE_50HZ:
            return ISENSOR_RATE_55HZ;
        case ASENSOR_RATE_30HZ:
            return ISENSOR_RATE_31HZ;
    }
    return inputRateUs;
}

/* set global accuracy flags.  sensor 0=gyro 1=accel 2=mag */
void MPLSensor::setAccuracy(int sensor, int accuracy)
{
    if (sensor == 0) {
        mGyroAccuracy = mGyroAccuracy < accuracy ? accuracy : mGyroAccuracy;
        return;
    }

    if (sensor == 1) {
        mAccelAccuracy = mAccelAccuracy < accuracy ? accuracy : mAccelAccuracy;
        return;
    }

    if (sensor == 2) {
        /* compass accuracy could drop due to disturbance */
        mCompassAccuracy = accuracy;
        return;
    }

    return;
}

/* set global accuracy flags.  sensor 0=gyro 1=accel 2=mag with force flag*/
void MPLSensor::setAccuracy(int sensor, int accuracy, int force)
{
    if (sensor == 0) {
        if (force)
            mGyroAccuracy = accuracy;
        else
            mGyroAccuracy = mGyroAccuracy < accuracy ? accuracy : mGyroAccuracy;
        return;
    }

    if (sensor == 1) {
        if (force)
            mAccelAccuracy = accuracy;
        else
            mAccelAccuracy = mAccelAccuracy < accuracy ? accuracy : mAccelAccuracy;
        return;
    }

    if (sensor == 2) {
        /* compass accuracy could drop due to disturbance */
        mCompassAccuracy = accuracy;
        return;
    }

    return;
}

void MPLSensor::printTimeProfile(int64_t &previousTime, int64_t currentTime, int64_t sensorTs, int64_t lastSensorTs)
{
    if (currentTime == 0) {
        currentTime = getTimestamp();
    }
    int64_t dhtime = sensorTs - lastSensorTs;
    if (lastSensorTs == 0) {
        dhtime = sensorTs - htime;
        htime = sensorTs;
    }
    int64_t dtime = currentTime - previousTime;
    int64_t stime = currentTime - sensorTs;
    int64_t mCurrentRate = dhtime;
    float_t interval = (dhtime - mCurrentRate) / mCurrentRate;  //non-batch only

    if (stime != 0) {
        (void)dtime, (void)interval;
        //LOGV("time_profile: dhtime=%lldms interval err=%.2fp", dhtime/1000000, interval * 100);
        //LOGV("time_profile: dtime=%lldms sync err=%lldms", dtime/1000000, stime/1000000);
        //LOGV("time_profile: ctime=%lld ts=%lld diff=%lld", currentTime, sensorTs, stime/1000000);
    }
}

void MPLSensor::setPower(int32_t rw, uint32_t* data)
{
    VFUNC_LOG;

    int var;

    var = *data;

    if (rw) {
        if (write_sysfs_int(sysfs_in_power_on, var)) {
            LOGE("HAL:ERR can't set in_power_on value");
            return;
        } else {
            if (read_sysfs_int(sysfs_in_power_on, &var)) {
                LOGE("HAL:ERR can't get in_power_on value");
                return;
            }
            *data = var;
        }
    } else {
        if (write_sysfs_int(sysfs_in_power_on, var)) {
            LOGE("HAL:ERR can't set in_power_on value");
        }
    }
}

void MPLSensor::getCurrentPowerMode(bool *GyroLp, bool *AccelLp)
{
    VFUNC_LOG;

    int data = 0;

    *GyroLp = false;
    *AccelLp = false;
    if (mCalibrationMode & mpl_gyro_cal) {
        if (read_sysfs_int(mpu.gyro_lp_mode, &data) < 0) {
            LOGE("HAL:ERR can't get gyro lp_mode status");
        } else {
            if (data) {
                *GyroLp = true;
            } else {
                *GyroLp = false;
            }
        }
    }
    if (mCalibrationMode & mpl_accel_cal) {
        if (read_sysfs_int(mpu.accel_lp_mode, &data) < 0) {
            LOGE("HAL:ERR can't get accel lp_mode status");
        } else {
            if (data) {
                *AccelLp = true;
            } else {
                *AccelLp = false;
            }
        }
    }
    LOGV_IF(ENG_VERBOSE, "HAL: Got the currnet lp mode status: gyro: %s, accel: %s",
            *GyroLp ? "LP" : "LN", *AccelLp ? "LP" : "LN");
}

bool MPLSensor::loadCalFile(void)
{
    inv_error_t rv;
    bool ret = false;

    rv = inv_load_calibration();
    if (rv == INV_SUCCESS) {
        LOGI("HAL:Calibration file successfully loaded");
        getGyroBias();
        if (mGyroBiasLpAvailable) { // set LP mode bias if available
            mGyroLpMode = true;
            setDmpGyroBias(true);
        } else if (mGyroBiasAvailable) {
            mGyroLpMode = false;
            setDmpGyroBias(true);
        }
        getAccelBias();
        if (mAccelBiasLpAvailable) { // set LP mode bias if available
            mAccelLpMode = true;
            setDmpAccelBias(true);
        } else if (mAccelBiasAvailable) {
            mAccelLpMode = false;
            setDmpAccelBias(true);
        }
        getCompassBias();
        if (mCompassBiasAvailable) {
            setDmpCompassBias();
        }
        getFactoryGyroBias();
        if (mFactoryGyroBiasAvailable) {
            setFactoryGyroBias();
        }
        getFactoryAccelBias();
        if (mFactoryAccelBiasAvailable) {
            setFactoryAccelBias();
        }
        if (mGyroBiasAvailable || mAccelBiasAvailable ||
                mCompassBiasAvailable) {
        }
        resetCalStatus(0);
        ret = true; // success
    } else {
        LOGW("HAL:Could not open or load MPL calibration file (%d)", rv);
        resetCalStatus(1);
    }
    return ret;
}

#ifdef DIRECT_REPORT
void MPLSensor::fillDirectReportFlags(struct sensor_t *sensor)
{
    int direct_rate;
    int max_rate;

    // restrict direct report support and max rate
    switch (sensor->type) {
    case SENSOR_TYPE_ACCELEROMETER:
    case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
    case SENSOR_TYPE_GYROSCOPE:
    case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
        max_rate = SENSOR_DIRECT_RATE_VERY_FAST;
        break;
    case SENSOR_TYPE_MAGNETIC_FIELD:
    case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
        max_rate = SENSOR_DIRECT_RATE_NORMAL;
        break;
    default:
        max_rate = SENSOR_DIRECT_RATE_STOP;
        break;
    }

    // Very fast minimal rate is 440Hz
    if (sensor->minDelay <= (1000000L / 440)) {
        direct_rate = SENSOR_DIRECT_RATE_VERY_FAST;
    // Fast minimal rate is 110Hz
    } else if (sensor->minDelay <= (1000000L / 110)) {
        direct_rate = SENSOR_DIRECT_RATE_FAST;
    // Normal minimal rate is 27.5Hz
    } else if (sensor->minDelay <= (10000000L / 275)) {
        direct_rate = SENSOR_DIRECT_RATE_NORMAL;
    } else {
        // direct report not supported
        direct_rate = SENSOR_DIRECT_RATE_STOP;
    }

    // limit to max rate
    if (direct_rate > max_rate) {
        direct_rate = max_rate;
    }

    if (direct_rate != SENSOR_DIRECT_RATE_STOP) {
        sensor->flags |= (direct_rate << SENSOR_FLAG_SHIFT_DIRECT_REPORT)
                | SENSOR_FLAG_DIRECT_CHANNEL_ASHMEM;
    }
}

int MPLSensor::register_direct_channel(const struct sensors_direct_mem_t* mem,
                                       int channel_handle)
{
    if (mem) {
        // add
        return addDirectChannel(mem);
    } else {
        // remove
        removeDirectChannel(channel_handle);
        return NO_ERROR;
    }
}

int MPLSensor::config_direct_report(int sensor_handle, int channel_handle,
                                    const struct sensors_direct_cfg_t *config)
{
    int rate_level = config->rate_level;

    return configDirectReport(sensor_handle, channel_handle, rate_level);
}

int MPLSensor::addDirectChannel(const struct sensors_direct_mem_t *mem)
{
    std::unique_ptr<DirectChannelBase> ch;
    int ret = NO_MEMORY;

    Mutex::Autolock autoLock(mDirectChannelLock);
    for (const auto& c : mDirectChannel) {
        if (c.second->memoryMatches(mem)) {
            // cannot reusing same memory
            return BAD_VALUE;
        }
    }
    switch(mem->type) {
        case SENSOR_DIRECT_MEM_TYPE_ASHMEM:
            ch = std::make_unique<AshmemDirectChannel>(mem);
            break;
        case SENSOR_DIRECT_MEM_TYPE_GRALLOC:
            // Gralloc disabled
            // ch = std::make_unique<GrallocDirectChannel>(mem);
            return BAD_VALUE;
            break;
        default:
            return INVALID_OPERATION;
    }

    if (ch) {
        if (ch->isValid()) {
            ret = mDirectChannelHandle++;
            mDirectChannel.insert(std::make_pair(ret, std::move(ch)));
        } else {
            ret = ch->getError();
            LOGW("HAL:Direct channel object(type:%d) has error %d upon init", mem->type, ret);
        }
    }

    return ret;
}

int MPLSensor::removeDirectChannel(int channel_handle)
{
    // make sure no active sensor in this channel
    std::vector<int32_t> activeSensorList;
    stopAllDirectReportOnChannel(channel_handle, &activeSensorList);

    // sensor service is responsible for stop all sensors before remove direct
    // channel. Thus, this is an error.
    if (!activeSensorList.empty()) {
        std::stringstream ss;
        std::copy(activeSensorList.begin(), activeSensorList.end(),
                std::ostream_iterator<int32_t>(ss, ","));
        LOGW("HAL:Removing channel %d when sensors (%s) are not stopped.",
                channel_handle, ss.str().c_str());
    }

    // remove the channel record
    Mutex::Autolock autoLock(mDirectChannelLock);
    mDirectChannel.erase(channel_handle);
    return NO_ERROR;
}

int MPLSensor::configDirectReport(int sensor_handle, int channel_handle, int rate_level)
{
    const struct sensor_t *sensor;
    int what = -1;
    int max_rate;
    int ret = BAD_VALUE;

    if (sensor_handle == -1 && rate_level == SENSOR_DIRECT_RATE_STOP) {
        return stopAllDirectReportOnChannel(channel_handle, nullptr);
    }

    getHandle(sensor_handle, what, sensor);
    if (what < 0) {
        LOGV_IF(ENG_VERBOSE, "HAL:can't find handle %d", sensor_handle);
        return BAD_VALUE;
    }

    if (((sensor->flags & SENSOR_FLAG_MASK_DIRECT_CHANNEL) >> SENSOR_FLAG_SHIFT_DIRECT_CHANNEL) == 0) {
        LOGV_IF(ENG_VERBOSE, "HAL:sensor %d doesn't support direct report", sensor_handle);
        return BAD_VALUE;
    }

    max_rate = (sensor->flags & SENSOR_FLAG_MASK_DIRECT_REPORT) >> SENSOR_FLAG_SHIFT_DIRECT_REPORT;
    if (rate_level > max_rate) {
        LOGV_IF(ENG_VERBOSE, "HAL:not supported rate level %d", rate_level);
        return BAD_VALUE;
    }

    // manage direct channel data structure
    Mutex::Autolock autoLock(mDirectChannelLock);
    auto i = mDirectChannel.find(channel_handle);
    if (i == mDirectChannel.end()) {
        return BAD_VALUE;
    }

    auto j = mSensorToChannel.find(sensor_handle);
    if (j == mSensorToChannel.end()) {
        return BAD_VALUE;
    }

    j->second.erase(channel_handle);
    if (rate_level != SENSOR_DIRECT_RATE_STOP) {
        j->second.insert(std::make_pair(channel_handle, (DirectChannelTimingInfo){0, 0, 0, rate_level}));
    }

    pthread_mutex_lock(&mHALMutex);
    SensorChannelMux &mux = mSensorChannelMuxes[sensor_handle];
    bool enable = mux.getEnable();
    const SensorChannelMux::SensorParams old_params = mux.getParams();
    SensorChannelMux::SensorParams req_params = {
        .periodNs = rateLevelToDeviceSamplingPeriodNs(sensor_handle, rate_level),
        .latencyNs = 0,
    };
    if (rate_level != SENSOR_DIRECT_RATE_STOP) {
        mux.registerChannel(channel_handle, req_params);
    } else {
        mux.disableChannel(channel_handle);
    }
    bool mux_enabled = mux.getEnable();
    const SensorChannelMux::SensorParams new_params = mux.getParams();
    pthread_mutex_unlock(&mHALMutex);

    // turn sensor off
    if (enable && !mux_enabled) {
        return doEnable(sensor_handle, 0);
    }

    // change sensor parameters
    if (new_params.periodNs != old_params.periodNs || new_params.latencyNs != old_params.latencyNs) {
        ret = doBatch(sensor_handle, 0, new_params.periodNs, new_params.latencyNs);
        if (ret != NO_ERROR) {
            return ret;
        }
    }
    // turn sensor on
    if (!enable && mux_enabled) {
        ret = doEnable(sensor_handle, 1);
        if (ret != NO_ERROR) {
            return ret;
        }
    }

    if (rate_level != SENSOR_DIRECT_RATE_STOP) {
        return SensorChannelMux::getSensorToken(sensor_handle);
    } else {
	return NO_ERROR;
    }
}

int MPLSensor::stopAllDirectReportOnChannel(int channel_handle, std::vector<int32_t> *activeSensorList)
{
    int ret;

    Mutex::Autolock autoLock(mDirectChannelLock);
    if (mDirectChannel.find(channel_handle) == mDirectChannel.end()) {
        return BAD_VALUE;
    }

    std::vector<int32_t> sensorToStop;
    for (auto &it : mSensorToChannel) {
        auto j = it.second.find(channel_handle);
        if (j != it.second.end()) {
            it.second.erase(j);
            if (it.second.empty()) {
                sensorToStop.push_back(it.first);
            }
        }
    }

    if (activeSensorList != nullptr) {
        *activeSensorList = sensorToStop;
    }

    for (auto &sensor : sensorToStop) {
        pthread_mutex_lock(&mHALMutex);
        SensorChannelMux &mux = mSensorChannelMuxes[sensor];
        const SensorChannelMux::SensorParams old_params = mux.getParams();
        mux.disableChannel(channel_handle);
        bool mux_enabled = mux.getEnable();
        const SensorChannelMux::SensorParams new_params = mux.getParams();
        pthread_mutex_unlock(&mHALMutex);
        if (!mux_enabled) {
            ret = doEnable(sensor, 0);
            LOGE_IF(ret != NO_ERROR, "HAL: ERR: cannot stop direct report sensor %d", sensor);
        } else {
            if (new_params.periodNs != old_params.periodNs || new_params.latencyNs != old_params.latencyNs) {
                ret = doBatch(sensor, 0, new_params.periodNs, new_params.latencyNs);
            } else {
                ret = NO_ERROR;
            }
            LOGE_IF(ret != NO_ERROR, "HAL: ERR: cannot change sensor %d configuration", sensor);
        }
    }

    return NO_ERROR;
}

void MPLSensor::sendDirectReportEvent(const sensors_event_t *nev, size_t n)
{
    // short circuit to avoid lock operation
    if (n == 0) {
        return;
    }

    // no intention to block sensor delivery thread. when lock is needed ignore
    // the event (this only happens when the channel is reconfiured, so it's ok
    if (mDirectChannelLock.tryLock() == NO_ERROR) {
        while (n--) {
            auto i = mSensorToChannel.find(nev->sensor);
            if (i != mSensorToChannel.end()) {
                for (auto &j : i->second) {
                    if ((uint64_t)nev->timestamp > j.second.dataTimestamp) {
                        // compute effective sensor data period
                        if (j.second.dataTimestamp != 0) {
                            j.second.dataPeriod = (uint64_t)nev->timestamp - j.second.dataTimestamp;
                        }
                        j.second.dataTimestamp = nev->timestamp;
                        // send data if interval is large enough or if next sample would be too far away
                        uint64_t delta = nev->timestamp - j.second.lastTimestamp;
                        uint64_t period = rateLevelToDeviceSamplingPeriodNs(nev->sensor, j.second.rateLevel);
                        bool cond1 = intervalLargeEnough(delta, period);
                        bool cond2 = (delta + j.second.dataPeriod) > (period + (period >> 3));	// > 112.5% of period
                        if (cond1 || cond2) {
                            mDirectChannel[j.first]->write(nev);
                            j.second.lastTimestamp = nev->timestamp;
                        }
                    }
                }
            }
            ++nev;
        }
        mDirectChannelLock.unlock();
    }
}

int64_t MPLSensor::rateLevelToDeviceSamplingPeriodNs(int handle, int rateLevel) const
{
    int32_t what = -1;
    const struct sensor_t *sensor;
    uint32_t ratePeriodUs;

    if (mSensorToChannel.find(handle) == mSensorToChannel.end()) {
        return INT64_MAX;
    }

    // recover sensor min delay
    getHandle(handle, what, sensor);
    if (what < 0) {
        return INT64_MAX;
    }

    if (sensor->minDelay == 0) {
        return INT64_MAX;
    }

    // direct rates nominal frequencies
    switch (rateLevel) {
    case SENSOR_DIRECT_RATE_VERY_FAST:
        // 800Hz
        ratePeriodUs = 1000000UL / 800UL;
        break;
    case SENSOR_DIRECT_RATE_FAST:
        // 200Hz
        ratePeriodUs = 1000000UL / 200UL;
        break;
    case SENSOR_DIRECT_RATE_NORMAL:
        // 50Hz
        ratePeriodUs = 1000000UL / 50UL;
        break;
    default:
        return INT64_MAX;
    }

    // adapt nominal frequencies to sensor capabilities
    if (ratePeriodUs < (uint32_t)sensor->minDelay) {
        ratePeriodUs = sensor->minDelay;
    }
    ratePeriodUs = lookupSensorRate(ratePeriodUs);

    return (int64_t)ratePeriodUs * 1000LL;
}
#endif
