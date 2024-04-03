/*
 * Copyright (C) 2016-2019 InvenSense, Inc.
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

#ifndef ANDROID_MPL_SENSOR_H
#define ANDROID_MPL_SENSOR_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <poll.h>
#include <time.h>
#include <pthread.h>

#include <vector>
#include <string>

#include "sensors.h"
#include "SensorBase.h"
#include "MPLLogger.h"

#ifdef DIRECT_REPORT
#include <utils/Mutex.h>
#include <memory>
#include <vector>
#include <unordered_map>
#include "directchannel.h"
#include "SensorChannelMux.h"
#endif

#ifndef INVENSENSE_COMPASS_CAL
#pragma message("unified HAL for AKM")
#include "CompassSensor.AKM.h"
#endif

#ifdef COMPASS_ON_PRIMARY_BUS
#pragma message("Compass on Primary Bus")
#include "CompassSensor.IIO.primary.h"
#else
#pragma message("Compass on Secondary Bus")
#include "CompassSensor.IIO.secondary.h"
#endif

#ifdef PRESSURE_ON_PRIMARY_BUS
#pragma message("Pressure on Primary Bus")
#include "PressureSensor.IIO.primary.h"
#else
#pragma message("Pressure on Secondary Bus")
#include "PressureSensor.IIO.secondary.h"
#endif

class LightSensor;

/*
 * Version defines
 */
#ifndef INV_SENSORS_HAL_VERSION_MAJOR
#  define INV_SENSORS_HAL_VERSION_MAJOR             0
#endif
#ifndef INV_SENSORS_HAL_VERSION_MINOR
#  define INV_SENSORS_HAL_VERSION_MINOR             0
#endif
#ifndef INV_SENSORS_HAL_VERSION_PATCH
#  define INV_SENSORS_HAL_VERSION_PATCH             0
#endif
#ifndef INV_SENSORS_HAL_VERSION_SUFFIX
#  define INV_SENSORS_HAL_VERSION_SUFFIX            "-dev"
#endif

/*****************************************************************************/
/* Sensors Enable/Disable Mask
 *****************************************************************************/
#define MAX_CHIP_ID_LEN                 (20)

#define INV_THREE_AXIS_RAW_GYRO         (1LL << RawGyro)
#define INV_THREE_AXIS_GYRO             (1LL << Gyro)
#define INV_THREE_AXIS_ACCEL            ((1LL << Accelerometer) | (1LL << RawAccelerometer))
#define INV_THREE_AXIS_RAW_COMPASS      (1LL << RawMagneticField)
#define INV_THREE_AXIS_COMPASS          (1LL << MagneticField)
#define INV_ONE_AXIS_PRESSURE           (1LL << Pressure)
#define INV_ONE_AXIS_LIGHT              (1LL << Light)

#define INV_THREE_AXIS_RAW_GYRO_WAKE         (1LL << RawGyro_Wake)
#define INV_THREE_AXIS_GYRO_WAKE             (1LL << Gyro_Wake)
#define INV_THREE_AXIS_ACCEL_WAKE            ((1LL << Accelerometer_Wake) | (1LL << RawAccelerometer_Wake))
#define INV_THREE_AXIS_RAW_COMPASS_WAKE      (1LL << RawMagneticField_Wake)
#define INV_THREE_AXIS_COMPASS_WAKE          (1LL << MagneticField_Wake)
#define INV_ONE_AXIS_PRESSURE_WAKE           (1LL << Pressure_Wake)
#define INV_ONE_AXIS_LIGHT_WAKE              (1LL << Light_Wake)
#define INV_ALL_SENSORS_WAKE                 (0xFFFFFFFFFFFFFFFF)

#define INV_ALL_CONTINUOUS_SENSORS      ((1LL << Gyro) | \
                                         (1LL << RawGyro) | \
                                         (1LL << Accelerometer) | \
                                         (1LL << MagneticField) | \
                                         (1LL << RawMagneticField) | \
                                         (1LL << Orientation) | \
                                         (1LL << RotationVector) | \
                                         (1LL << GameRotationVector) | \
                                         (1LL << LinearAccel) | \
                                         (1LL << Gravity) | \
                                         (1LL << GeomagneticRotationVector) | \
                                         (1LL << LPQ) | \
                                         (1LL << Heading) | \
                                         (1LL << Gyro_Wake) | \
                                         (1LL << RawGyro_Wake) | \
                                         (1LL << Accelerometer_Wake) | \
                                         (1LL << MagneticField_Wake) | \
                                         (1LL << RawMagneticField_Wake) | \
                                         (1LL << Orientation_Wake) | \
                                         (1LL << RotationVector_Wake) | \
                                         (1LL << GameRotationVector_Wake) | \
                                         (1LL << LinearAccel_Wake) | \
                                         (1LL << Gravity_Wake) | \
                                         (1LL << GeomagneticRotationVector_Wake) | \
                                         (1LL << Pressure) | \
                                         (1LL << Pressure_Wake) | \
                                         (1LL << Heading_Wake) | \
                                         (1LL << AccelerometerRaw) | \
                                         (1LL << GyroRaw) | \
                                         (1LL << MagneticFieldRaw))

#define GYRO_MPL_SENSOR             (INV_THREE_AXIS_RAW_GYRO \
                                     | INV_THREE_AXIS_GYRO \
                                     | INV_THREE_AXIS_RAW_GYRO_WAKE \
                                     | INV_THREE_AXIS_GYRO_WAKE)
#define AUX_MPL_SENSOR              (INV_THREE_AXIS_RAW_COMPASS \
                                     | INV_THREE_AXIS_COMPASS \
                                     | INV_ONE_AXIS_PRESSURE \
                                     | INV_ONE_AXIS_LIGHT \
                                     | INV_THREE_AXIS_RAW_COMPASS_WAKE \
                                     | INV_THREE_AXIS_COMPASS_WAKE \
                                     | INV_ONE_AXIS_PRESSURE_WAKE \
                                     | INV_ONE_AXIS_LIGHT_WAKE)

// mask of virtual sensors that require gyro + accel + compass data
#define VIRTUAL_SENSOR_9AXES_MASK (         \
        (1LL << Orientation)                \
        | (1LL << RotationVector)           \
        | (1LL << Heading)                  \
)
// mask of virtual sensors that require gyro + accel + compass data
#define VIRTUAL_SENSOR_9AXES_MASK_WAKE (    \
        (1LL << Orientation_Wake)           \
        | (1LL << RotationVector_Wake)      \
        | (1LL << Heading_Wake)             \
)
// mask of virtual sensors that require gyro + accel data (but no compass data)
#define VIRTUAL_SENSOR_GYRO_6AXES_MASK (    \
        (1LL << GameRotationVector)         \
        | (1LL << LinearAccel)              \
        | (1LL << Gravity)                  \
)
// mask of virtual sensors that require gyro + accel data (but no compass data)
#define VIRTUAL_SENSOR_LPQ_MASK (    \
        (1LL << LPQ)         \
)

// mask of virtual sensors that require gyro + accel data (but no compass data)
#define VIRTUAL_SENSOR_GYRO_6AXES_MASK_WAKE (   \
        (1LL << GameRotationVector_Wake)        \
        | (1LL << LinearAccel_Wake)             \
        | (1LL << Gravity_Wake)                 \
)
// mask of virtual sensors that require mag + accel data (but no gyro data)
#define VIRTUAL_SENSOR_MAG_6AXES_MASK (         \
        (1LL << GeomagneticRotationVector)      \
)
#define VIRTUAL_SENSOR_MAG_6AXES_MASK_WAKE (    \
        (1LL << GeomagneticRotationVector_Wake) \
)
// mask of all virtual sensors
#define VIRTUAL_SENSOR_ALL_MASK (           \
        VIRTUAL_SENSOR_9AXES_MASK           \
        | VIRTUAL_SENSOR_GYRO_6AXES_MASK    \
        | VIRTUAL_SENSOR_MAG_6AXES_MASK     \
)
#define VIRTUAL_SENSOR_ALL_MASK_WAKE (          \
        VIRTUAL_SENSOR_9AXES_MASK_WAKE          \
        | VIRTUAL_SENSOR_GYRO_6AXES_MASK_WAKE   \
        | VIRTUAL_SENSOR_MAG_6AXES_MASK_WAKE    \
)

// bit mask of current DMP active features (mFeatureActiveMask)
#define INV_DMP_BATCH_MODE            0x00000001 //batch mode
#define INV_DMP_SIGNIFICANT_MOTION    0x00000002 //significant motion
#define INV_DMP_TILT                  0x00000004 //Tilt Event
#define INV_DMP_PICKUP                0x00000008 //Pick up Gesture
#define INV_DMP_STEP_COUNTER          0x00000010 //Step counter
#define INV_DMP_STEP_DETECTOR         0x00000020 //Step detector
#define INV_DMP_STEP_COUNTER_WAKE     0x00000040 //Step counter wakeup
#define INV_DMP_STEP_DETECTOR_WAKE    0x00000080 //Step detector wakeup
#define INV_DMP_6AXIS_QUATERNION      0x00000100 //3 elements without real part, 32 bit each
#define INV_DMP_9AXIS_QUATERNION      0x00000200 //3 elements without real part, 32 bit each
#define INV_DMP_6AXIS_QUATERNION_WAKE 0x00000400 //3 elements without real part, 32 bit each
#define INV_DMP_9AXIS_QUATERNION_WAKE 0x00000800 //3 elements without real part, 32 bit each
#define INV_DMP_LPQ                   0x00001000 //LPQ
#define INV_DMP_EIS_GYROSCOPE         0x00002000 //Eis event
#define INV_DMP_MOT_DETECT            0x00004000 //Motion detect
#define INV_DMP_STA_DETECT            0x00008000 //Stationary detect
#define INV_DMP_TAP                   0x00010000 //Tap event

// data header format used by kernel driver.
#define DATA_FORMAT_WAKEUP           0x8000

#define DATA_FORMAT_ACCEL            1
#define DATA_FORMAT_RAW_GYRO         2
#define DATA_FORMAT_RAW_COMPASS      3
#define DATA_FORMAT_ALS              4
#define DATA_FORMAT_6_AXIS           5
#define DATA_FORMAT_9_AXIS           6
#define DATA_FORMAT_PED_QUAT         7
#define DATA_FORMAT_GEOMAG           8
#define DATA_FORMAT_PRESSURE         9
#define DATA_FORMAT_GYRO             10
#define DATA_FORMAT_COMPASS          11
#define DATA_FORMAT_STEP_COUNT       12
#define DATA_FORMAT_STEP_DETECT      13
#define DATA_FORMAT_STEP             14
#define DATA_FORMAT_ACTIVITY         15
#define DATA_FORMAT_PICKUP           16
#define DATA_FORMAT_EMPTY_MARKER     17
#define DATA_FORMAT_MARKER           18
#define DATA_FORMAT_COMPASS_ACCURACY 19
#define DATA_FORMAT_ACCEL_ACCURACY   20
#define DATA_FORMAT_GYRO_ACCURACY    21
#define DATA_FORMAT_EIS_GYROSCOPE    36
#define DATA_FORMAT_EIS_AUTHENTICATION    37
#define DATA_FORMAT_LPQ              38
#define DATA_FORMAT_TAP              40


#define DATA_FORMAT_ACCEL_WAKE            (DATA_FORMAT_ACCEL | DATA_FORMAT_WAKEUP)
#define DATA_FORMAT_RAW_GYRO_WAKE         (DATA_FORMAT_RAW_GYRO | DATA_FORMAT_WAKEUP)
#define DATA_FORMAT_RAW_COMPASS_WAKE      (DATA_FORMAT_RAW_COMPASS | DATA_FORMAT_WAKEUP)
#define DATA_FORMAT_ALS_WAKE              (DATA_FORMAT_ALS | DATA_FORMAT_WAKEUP)
#define DATA_FORMAT_6_AXIS_WAKE           (DATA_FORMAT_6_AXIS | DATA_FORMAT_WAKEUP)
#define DATA_FORMAT_9_AXIS_WAKE           (DATA_FORMAT_9_AXIS | DATA_FORMAT_WAKEUP)
#define DATA_FORMAT_PED_QUAT_WAKE         (DATA_FORMAT_PED_QUAT | DATA_FORMAT_WAKEUP)
#define DATA_FORMAT_GEOMAG_WAKE           (DATA_FORMAT_GEOMAG | DATA_FORMAT_WAKEUP)
#define DATA_FORMAT_PRESSURE_WAKE         (DATA_FORMAT_PRESSURE | DATA_FORMAT_WAKEUP)
#define DATA_FORMAT_GYRO_WAKE             (DATA_FORMAT_GYRO | DATA_FORMAT_WAKEUP)
#define DATA_FORMAT_COMPASS_WAKE          (DATA_FORMAT_COMPASS | DATA_FORMAT_WAKEUP)
#define DATA_FORMAT_STEP_COUNT_WAKE       (DATA_FORMAT_STEP_COUNT | DATA_FORMAT_WAKEUP)
#define DATA_FORMAT_STEP_DETECT_WAKE      (DATA_FORMAT_STEP_DETECT | DATA_FORMAT_WAKEUP)

#define BYTES_PER_SENSOR                8
#define BYTES_PER_SENSOR_PACKET         24
#define QUAT_ONLY_LAST_PACKET_OFFSET    16
#define BYTES_QUAT_DATA                 24
#define MAX_READ_SIZE                   (BYTES_PER_SENSOR_PACKET + 8)
#define MAX_SUSPEND_BATCH_PACKET_SIZE   1024
#define MAX_PACKET_SIZE                 80 //8 * 4 + (2 * 24)
#define NS_PER_SECOND                   1000000000LL
#define NS_PER_SECOND_FLOAT             1000000000.f

#define ASENSOR_RATE_200HZ 5000
#define ASENSOR_RATE_100HZ 10000
#define ASENSOR_RATE_50HZ  20000
#define ASENSOR_RATE_30HZ  33333
#define ISENSOR_RATE_220HZ 4545
#define ISENSOR_RATE_110HZ 9090
#define ISENSOR_RATE_55HZ 18181
#define ISENSOR_RATE_31HZ 32258

/* define minimum rate for sensor fusion */
#ifdef INV_USE_ALGO_LIB
#define MIN_SENSOR_FUSION_RATE 20000000 //50hz
#else
#define MIN_SENSOR_FUSION_RATE 10000000 //100hz
#endif
#define MIN_MAG_FUSION_RATE    80000000 //12.5hz
#define MIN_MAG_RATE           100000000 //10hz

/* min rate for sensors in ns */
#define MIN_NON_BATCH_RATE_GYRO     1000000000ULL
#define MIN_NON_BATCH_RATE_ACCEL    1000000000ULL

/* Poll timeout in main loop when step counter is enabled */
#define STEP_COUNT_POLL_TIME_MS     1000 // 1sec

#ifdef DIRECT_REPORT
using namespace android;
#endif

class MPLSensor: public SensorBase
{
    typedef int (MPLSensor::*hfunc_t)(sensors_event_t*);

public:

    MPLSensor(CompassSensor *, PressureSensor *);
    virtual ~MPLSensor();

    virtual int setDelay(int32_t handle, int64_t ns);
    virtual int enable(int32_t handle, int enabled);
    virtual int batch(int handle, int flags, int64_t period_ns, int64_t timeout);
    virtual int flush(int handle);
    virtual int inject_sensor_data(const sensors_event_t* data);
#ifdef DIRECT_REPORT
    virtual int register_direct_channel(const struct sensors_direct_mem_t* mem,
                                        int channel_handle);
    virtual int config_direct_report(int sensor_handle, int channel_handle,
                                     const struct sensors_direct_cfg_t *config);
#endif

    int checkBatchEnabled();
    int setBatch(int en);
    int writeBatchTimeout(int en);
    void getHandle(int32_t handle, int32_t &what, const struct sensor_t * &sensor) const;

    virtual int readEvents(sensors_event_t *data, int count);
    virtual int getFd() const;
    virtual int getPollTime();
    virtual bool hasPendingEvents() const;
    int isDataInjectionSupported();
    void setDataInjectionMode(int mode);
    int populateSensorList(struct sensor_t *list, int len);
    void fillSensorWith1K();

    void buildCompassEvent();
    void buildPressureEvent();
    void buildMpuEvent();
    int checkValidHeader(unsigned short data_format);


    int getDmpSignificantMotionFd();
    int readDmpSignificantMotionEvents(sensors_event_t* data, int count);
    int enableDmpSignificantMotion(int);
    int significantMotionHandler(sensors_event_t* data);

    int enableDmpPedometer(int, int);
    int trigDmpPedometerCountRead(void);
    int readDmpPedometerEvents(sensors_event_t* data, int count, int32_t id);
    int getDmpPedometerFd();

    int getDmpTiltFd();
    int readDmpTiltEvents(sensors_event_t* data, int count);
    int enableDmpTilt(int);

    int getDmpPickupFd();
    int readDmpPickupEvents(sensors_event_t* data, int count);
    int enableDmpPickup(int);

    int enableDmpTap(int);

    int enableDmpStationaryDetect(int);
    int enableDmpMotionDetect(int);

    int enableDmpEis(int);
    int enableDmpEisAuthentication(int);

    int enableOisSensor(int);

    int enableLPQ(int);

protected:
    CompassSensor *mCompassSensor;
    PressureSensor *mPressureSensor;
    LightSensor *mLightSensor;

    int doEnable(int32_t handle, int en);
    int doBatch(int handle, int flags, int64_t period_ns, int64_t timeout);
    int gyroHandler(sensors_event_t *data);
    int rawGyroHandler(sensors_event_t *data);
    int accelHandler(sensors_event_t *data);
    int rawAccelHandler(sensors_event_t *data);
    int compassHandler(sensors_event_t *data);
    int rawCompassHandler(sensors_event_t *data);
    int rvHandler(sensors_event_t *data);
    int grvHandler(sensors_event_t *data);
    int laHandler(sensors_event_t *data);
    int gravHandler(sensors_event_t *data);
    int orienHandler(sensors_event_t *data);
    int smHandler(sensors_event_t *data);
    int pHandler(sensors_event_t *data);
    int gmHandler(sensors_event_t *data);
    int psHandler(sensors_event_t *data);
    int sdHandler(sensors_event_t *data);
    int scHandler(sensors_event_t *data);
    int lightHandler(sensors_event_t *data);
    int proxHandler(sensors_event_t *data);
    int tiltHandler(sensors_event_t *data);
    int tapHandler(sensors_event_t *data);
    int pickupHandler(sensors_event_t *data);
    int stationaryDetectHandler(sensors_event_t *data);
    int motionDetectHandler(sensors_event_t *data);
    int eisHandler(sensors_event_t *data);
    int eisAuthenticationHandler(sensors_event_t *data);
    int metaHandler(int sensor, sensors_event_t *data, int flags);
    int lpqHandler(sensors_event_t *data);
    int headingHandler(sensors_event_t *data);

    int gyrowHandler(sensors_event_t *data);
    int rawGyrowHandler(sensors_event_t *data);
    int accelwHandler(sensors_event_t *data);
    int rawAccelwHandler(sensors_event_t *data);
    int compasswHandler(sensors_event_t *data);
    int rawCompasswHandler(sensors_event_t *data);
    int rvwHandler(sensors_event_t *data);
    int grvwHandler(sensors_event_t *data);
    int lawHandler(sensors_event_t *data);
    int gravwHandler(sensors_event_t *data);
    int orienwHandler(sensors_event_t *data);
    int pwHandler(sensors_event_t *data);
    int gmwHandler(sensors_event_t *data);
    int pswHandler(sensors_event_t *data);
    int sdwHandler(sensors_event_t *data);
    int scwHandler(sensors_event_t *data);
    int lightwHandler(sensors_event_t *data);
    int proxwHandler(sensors_event_t *data);
    int headingwHandler(sensors_event_t *data);

    int gyroRawHandler(sensors_event_t *data);
    int accelRawHandler(sensors_event_t *data);
    int compassRawHandler(sensors_event_t *data);

    int additionalInfoSensorPlacement(int handle, unsigned int seq, sensors_event_t* event);
    int additionalInfoInternalTemperature(int handle, unsigned int seq, sensors_event_t *event);
    int additionalInfoHandler(int handle, sensors_event_t *data, int count);
    int periodicAdditionalInfoHandler(int handle, sensors_event_t* data, int count);

    void calcOrientationSensor(float *Rx, float *Val);
    virtual int update_delay();

    void inv_set_device_properties();
    int inv_constructor_init();
    int inv_constructor_default_enable();
    typedef int (*get_sensor_data_func)(float *values, int8_t *accuracy, int64_t *timestamp);
    int output_control(sensors_event_t* s, get_sensor_data_func func,
    int sensor_value_size, float *sensor_values, int8_t *sensor_accuracy,
    int sensor_id);
    void inv_set_engine_rate(uint32_t rate, uint64_t mask);
    void setRawGyroRate(uint64_t delay);
    void setRawGyroRateWake(uint64_t delay);
    void setGyroRate(uint64_t delay);
    void setGyroRateWake(uint64_t delay);
    void setAccelRate(uint64_t delay);
    void setAccelRateWake(uint64_t delay);
    void set6AxesRate(uint64_t delay);
    void set3AxesRate(uint64_t delay);
    void set6AxesRateWake(uint64_t delay);
    void setRawMagRate(uint64_t delay);
    void setRawMagRateWake(uint64_t delay);
    void setMagRate(uint64_t delay);
    void setMagRateWake(uint64_t delay);
    void set9AxesRate(uint64_t delay);
    void set9AxesRateWake(uint64_t delay);
    void set6AxesMagRate(uint64_t delay);
    void set6AxesMagRateWake(uint64_t delay);
    void setPressureRate(uint64_t delay);
    void setPressureRateWake(uint64_t delay);
    void setLightRate(uint64_t delay);
    void setLightRateWake(uint64_t delay);
    int enableCompass6AxisQuaternion(int en);
    int set6AxisQuaternionRate(int64_t wanted);
    int set6AxisQuaternionWakeRate(int64_t wanted);
    int enable9AxisQuaternion(int);
    int enableLPQuaternion(int);
    int enableQuaternionData(int);
    int setQuaternionRate(int64_t wanted);
    int enable9AxisQuaternionWake(int en);
    int enableCompass6AxisQuaternionWake(int en);
    int enableLPQuaternionWake(int en);
    int enableQuaternionDataWake(int en);
    int enableAccelPedometer(int);
    int enableAccelPedData(int);
    int enableGyroEngine(int en);
    int enableRawGyro(int en);
    int enableRawGyroWake(int en);
    int enableGyro(int en);
    int enableGyroWake(int en);
    int enableAccel(int en);
    int enableAccelWake(int en);
    int enableRawCompass(int en);
    int enableRawCompassWake(int en);
    int enableCompass(int en);
    int enableCompassWake(int en);
    int enablePressure(int en);
    int enablePressureWake(int en);
    int enableLight(int en);
    int enableLightWake(int en);
    int enableProximity(int en);
    int enableProximityWake(int en);
    int enableBatch(int64_t timeout);
    int enableDmpCalibration(int en);
    void computeLocalSensorMask(uint64_t enabled_sensors);
    int computeBatchSensorMask(uint64_t enableSensor, uint64_t checkNewBatchSensor);
    int enableSensors(uint64_t sensors, int en, uint64_t changed);
    uint64_t invCheckOutput(uint32_t sensor);
    int inv_read_temperature(long long *data);
    int inv_read_sensor_bias(int fd, int *data);
    void inv_get_sensors_orientation(void);
    int inv_init_sysfs_attributes(void);
    void enable_iio_sysfs(void);
    void setAccuracy(int sensor, int accuracy);
    void setAccuracy(int sensor, int accuracy, int force);
    void inv_write_sysfs(uint32_t delay, uint64_t mask, const char *sysfs_rate);
    void inv_read_sysfs_engine(uint64_t mask, const char *sysfs_rate);

    static void setPower(int32_t rw, uint32_t* data);

    void getCurrentPowerMode(bool *GyroLp, bool *AccelLp);
    bool loadCalFile(void);

    uint64_t mMasterSensorMask;
    uint64_t mLocalSensorMask;
    int mPollTime;
    int mGyroAccuracy;      // value indicating the quality of the gyro calibr.
    int mAccelAccuracy;     // value indicating the quality of the accel calibr.
    int mCompassAccuracy;   // value indicating the quality of the compass calibr.
    int mHeadingAccuracy;   // value indicating the 9 axis heading accuracy interval
    struct pollfd mPollFds[5];
    pthread_mutex_t mHALMutex;

    char mIIOBuffer[(16 + 8 * 3 + 8) * IIO_BUFFER_LENGTH];

    int iio_fd;
    int gyro_temperature_fd;
    int accel_x_offset_fd;
    int accel_y_offset_fd;
    int accel_z_offset_fd;

    int accel_x_dmp_bias_fd;
    int accel_y_dmp_bias_fd;
    int accel_z_dmp_bias_fd;

    int gyro_x_offset_fd;
    int gyro_y_offset_fd;
    int gyro_z_offset_fd;

    int gyro_x_dmp_bias_fd;
    int gyro_y_dmp_bias_fd;
    int gyro_z_dmp_bias_fd;

    int compass_x_dmp_bias_fd;
    int compass_y_dmp_bias_fd;
    int compass_z_dmp_bias_fd;

    int dmp_sign_motion_fd;
    int mDmpSignificantMotionEnabled;

    int dmp_pedometer_fd;
    int mDmpPedometerEnabled;
    int mDmpStepCountEnabled;

    int dmp_pickup_fd;
    int mDmpPickupEnabled;

    int dmp_tilt_fd;
    int mDmpTiltEnabled;

    int mDmpTapEnabled;

    //int dmp_stationary_det_fd;
    int mDmpStationaryDetEnabled;

    //int dmp_motion_det_fd;
    int mDmpMotionDetEnabled;

    uint64_t mEnabled;
    uint64_t mEnabledCached;
    uint64_t mBatchEnabled;
    std::vector<int> mFlushSensorEnabledVector;
    uint32_t mOldBatchEnabledMask;
    int64_t mBatchTimeoutInMs;
    sensors_event_t mPendingEvents[TotalNumSensors];
    int64_t mDelays[TotalNumSensors];
    int64_t mBatchDelays[TotalNumSensors];
    int64_t mBatchTimeouts[TotalNumSensors];
    int32_t mSensorUpdate[TotalNumSensors];
    hfunc_t mHandlers[TotalNumSensors];
    int64_t mEnabledTime[TotalNumSensors];
    int mCachedGyroData[3];
    int mCachedCalGyroData[3];
    int mCachedAccelData[3];
    int mCachedCompassData[3];
    int mCachedQuaternionData[3];
    int mCached6AxisQuaternionData[3];
    int mCached9AxisQuaternionData[3];
    int mCachedGeomagData[3];
    int mCachedPedQuaternionData[3];
    int mCachedTapData[3];
    int mCachedPressureData;
    int mCachedPressureWakeData;
    int mCachedAlsData[3];
    int mCachedAlsWakeData[3];
    int mCachedEisData[4];
    int mCachedEisAuthenticationData[4];
    int mCachedLPQData[3];
    int mCachedImuTemperature;
    int mCompassSoftIron[9];
    int mCompassSens[3];
    uint32_t mNumSensors;
    uint32_t mNumSysfs;

    bool mFirstRead;
    int64_t mTempCurrentTime;
    int mAccelScale;
    int mGyroScale;
    int mGyroDmpScaleFactor;
    int mCompassScale;
    float mCompassBias[3];
    bool mFactoryGyroBiasAvailable;
    bool mFactoryGyroBiasLoaded;
    bool mFactoryGyroBiasLpLoaded;
    int mFactoryGyroBias[3];
    int mFactoryGyroBiasLp[3];
    int mGyroBiasUiMode[3];
    bool mGyroBiasAvailable;
    bool mGyroBiasLpAvailable;
    bool mGyroBiasApplied;
    int mDmpGyroBias[3];
    int mDmpGyroBiasLp[3];
    float mGyroBias[3];    //in body frame
    int mGyroChipBias[3]; //in chip frame
    bool mFactoryAccelBiasAvailable;
    bool mFactoryAccelBiasLoaded;
    bool mFactoryAccelBiasLpLoaded;
    int mFactoryAccelBias[3];
    int mFactoryAccelBiasLp[3];
    int mAccelBiasUiMode[3];
    bool mAccelBiasAvailable;
    bool mAccelBiasLpAvailable;
    bool mAccelBiasApplied;
    int mAccelDmpBias[3];
    int mAccelDmpBiasLp[3];
    int mAccelBias[3];    //in chip frame
    int mDmpCompassBias[3];
    int mRawGyroCustom[3];
    int mRawAccelCustom[3];
    int mRawMagCustom[3];
    bool mCompassBiasAvailable;
    bool mCompassBiasApplied;
    bool mRetryCalFileLoad;

    int mGyroAccuracyLib;
    int mGyroLpAccuracyLib;
    int mAccelAccuracyLib;
    int mAccelLpAccuracyLib;

    bool mChipDetected;

    bool mGyroLpMode;
    bool mAccelLpMode;

    bool mStepCounterIntMode;
    int64_t mLastStepCountReadTrigTimestamp;

    char chip_ID[MAX_CHIP_ID_LEN];
    char mSysfsPath[MAX_SYSFS_NAME_LEN];

    signed char mGyroOrientationMatrix[9];
    signed char mAccelOrientationMatrix[9];
    signed char mCompassOrientationMatrix[9];

    std::vector<int> mAdditionalInfoEnabledVector;
    float mGyroLocation[3];
    float mAccelLocation[3];
    float mCompassLocation[3];
    float mPressureLocation[3];
    float mLightLocation[3];
    float mProximityLocation[3];

    unsigned short mGyroOrientationScalar;
    unsigned short mAccelOrientationScalar;
    unsigned short mCompassOrientationScalar;

    /**
     *  struct sensor_mask - data structure facing Android sensor. Each Android sensor has one entry.
     *  @sensorMask:     define this sensor depends what sysfs entries.
                            Different calibration mode may have different result.
     *  @sname:          String that show the sensor name.
     *  @engineMask:     If the sensor depends on multiple sysfs entries, this will show the one that
                            defines the rate of sensor. If only one sysfs entry, it is then this entry.
     *  @engineRateAddr: Pointer pointed to the sysfs entry that engine rate can be derived from.
                            The final output rate will depends on this rate.
     *  @timestamp:      timestamp for current sensor.
     *  @wakeOn:      wake sensor or not.
     */
    struct sensor_mask {
        uint64_t sensorMask;
        std::string sname;
        uint64_t engineMask;
        uint32_t *engineRateAddr;
        int64_t timestamp;
        bool wakeOn;
    };

    /**
     *  struct sysfs_mask - data structure facing sysfs file. Each sysfs has one entry.
     *  @sensorMask:     Define mask of this sysfs entries. This mask will be used in sensorMask of
                           structure sensor_mask.
     *  @enable:     enable function that write into sysfs entry enable function.
     *  @setRate:        set the sysfs rate function.
     *  @engineRate:     The read back value of sysfs entry after setting rate. The read back value may
                             not the same as the one write into the sysfs entry as not all the rate are
                             supported. This makes the decimation more accurate.
     *  @nominalRate:    The rate that is being write into sysfs rate set function. It is derived from
                             the Android sensors that uses this sysfs entry.
     *  @minimumNonBatchRate:   Some sysfs has a minimum rate requirements.
     *  @en:             Whether this sysfs entry is enabled or not.
     */
    struct sysfs_mask {
        uint64_t sensorMask;
        int (MPLSensor::*enable)(int);
        void (MPLSensor::*setRate)(uint64_t);
        uint32_t engineRate;
        int64_t nominalRate;
        uint64_t minimumNonBatchRate;
        bool en;
    };

    struct sensor_mask *mCurrentSensorMask;
    struct sysfs_mask *mSysfsMask;

    struct sysfs_attrbs {
       char *chip_enable;
       char *available_data;
       char *dmp_firmware;
       char *firmware_loaded;
       char *dmp_on;
       char *dmp_int_on;
       char *dmp_event_int_on;
       char *tap_on;
       char *self_test;
       char *dmp_init;
       char *temperature;

       char *gyro_enable;
       char *gyro_fsr;
       char *gyro_sf;
       char *gyro_orient;
       char *gyro_fifo_enable;
       char *gyro_rate;
       char *gyro_raw_data;

       char *compass_raw_data;

       char *gyro_wake_fifo_enable;
       char *gyro_wake_rate;

       char *calib_gyro_enable;
       char *calib_gyro_rate;
       char *calib_gyro_wake_enable;
       char *calib_gyro_wake_rate;

       char *accel_enable;
       char *accel_fsr;
       char *accel_bias;
       char *accel_orient;
       char *accel_fifo_enable;
       char *accel_rate;
       char *accel_raw_data;

       char *accel_wake_fifo_enable;
       char *accel_wake_rate;

       char *three_axis_q_on; //formerly quaternion_on
       char *three_axis_q_rate;

       char *six_axis_q_on;
       char *six_axis_q_rate;
       char *six_axis_q_wake_on;
       char *six_axis_q_wake_rate;
       char *six_axis_q_value;

       char *nine_axis_q_on;
       char *nine_axis_q_rate;
       char *nine_axis_q_wake_on;
       char *nine_axis_q_wake_rate;

       char *in_geomag_enable;
       char *in_geomag_rate;
       char *in_geomag_wake_enable;
       char *in_geomag_wake_rate;

       char *scan_el_en;
       char *scan_el_index;
       char *scan_el_type;

       char *buffer_length;

       char *display_orientation_on;
       char *event_display_orientation;

       char *in_accel_x_offset;
       char *in_accel_y_offset;
       char *in_accel_z_offset;

       char *in_accel_x_dmp_bias;
       char *in_accel_y_dmp_bias;
       char *in_accel_z_dmp_bias;

       char *in_gyro_x_offset;
       char *in_gyro_y_offset;
       char *in_gyro_z_offset;

       char *in_gyro_x_dmp_bias;
       char *in_gyro_y_dmp_bias;
       char *in_gyro_z_dmp_bias;

       char *in_compass_x_dmp_bias;
       char *in_compass_y_dmp_bias;
       char *in_compass_z_dmp_bias;

       char *event_smd;
       char *smd_enable;
       char *batchmode_timeout;
       char *batchmode_wake_fifo_full_on;
       char *flush_batch;

       char *tilt_on;
       char *event_tilt;

       char *pickup_on;
       char *event_pickup;

       char *stationary_detect_on;
       //char *event_stationary_detect;

       char *motion_detect_on;
       //char *event_motion_detect;

       char *eis_on;
       char *eis_data_on;
       char *eis_rate;

       char *activity_on;
       char * event_activity;

       char *pedometer_on;
       char *pedometer_wake_on;
       char *pedometer_counter_on;
       char *pedometer_counter_wake_on;
       char *pedometer_counter_send;
       char *pedometer_int_on;
       char *pedometer_int_mode;
       char *event_pedometer;
       char *pedometer_steps;
       char *pedometer_step_thresh;
       char *pedometer_counter;

       char *motion_lpa_on;

       char *gyro_cal_enable;
       char *accel_cal_enable;
       char *compass_cal_enable;

       char *accel_accuracy_enable;
       char *anglvel_accuracy_enable;
       char *magn_accuracy_enable;

       char *debug_determine_engine_on;
       char *d_compass_enable;
       char *d_misc_gyro_recalibration;
       char *d_misc_accel_recalibration;
       char *d_misc_compass_recalibration;

       char *d_misc_accel_cov;
       char *d_misc_compass_cov;
       char *d_misc_current_compass_cov;
       char *d_misc_ref_mag_3d;

       char *ois_enable;
       char *info_poke_mode;
       char *misc_bin_poke_accel;
       char *misc_bin_poke_gyro;
       char *misc_bin_poke_mag;
       char *gyro_lp_mode;
       char *accel_lp_mode;
    } mpu;

    char *sysfs_names_ptr;
    uint64_t mFeatureActiveMask;
    int mPedUpdate;
    int mPedWakeUpdate;
    int mTiltUpdate;
    int mTapUpdate;
    int mPickupUpdate;
    int mPressureUpdate;
    int mPressureWakeUpdate;
    int mEisUpdate;
    int mEisAuthenticationUpdate;
    int64_t mGyroSensorTimestamp;
    int64_t mAccelSensorTimestamp;
    int64_t mQuatSensorTimestamp;
    int64_t mQuatSensorLastTimestamp;
    int64_t m6QuatSensorTimestamp;
    int64_t m6QuatSensorLastTimestamp;
    int64_t mGeoQuatSensorTimestamp;
    int64_t mGeoQuatSensorLastTimestamp;
    int64_t mStepSensorTimestamp;
    int64_t mStepSensorWakeTimestamp;
    int64_t mTapTimestamp;
    int64_t mAlsSensorTimestamp;
    int64_t mAlsSensorWakeTimestamp;
    int64_t mLPQTimestamp;
    int64_t mSensorTimestamp;
    int64_t mCompassTimestamp;
    int64_t mPressureTimestamp;
    int64_t mPressureWakeTimestamp;
    int64_t mEisTimestamp;
    int64_t mEisAuthenticationTimestamp;
    int64_t mRawGyroCustomTimestamp;
    int64_t mRawAccelCustomTimestamp;
    int64_t mRawMagCustomTimestamp;
    int64_t mImuTemperatureTimestamp;
    int64_t mChipTemperatureTimestamps[TotalNumSensors];
    uint64_t mLastStepCount;
    uint64_t mStepCount;
    uint64_t mLastStepCountSave;
    uint64_t mLastStepCountWake;
    uint64_t mStepCountWake;
    uint64_t mLastStepCountSaveWake;
    bool mStepCounterHandlerNotCalled;
    bool mStepCounterHandlerNotCalledWake;
    bool mLightInitEvent;
    bool mLightWakeInitEvent;
    bool mProxiInitEvent;
    bool mProxiWakeInitEvent;
    uint32_t mSkipReadEvents;
    bool mPressureSensorPresent;
    bool mLightSensorPresent;
    bool mCustomSensorPresent;
    bool mEmptyDataMarkerDetected;

    int mCalibrationMode;
    int mOisEnabled;

    struct sensor_t mCurrentSensorList[ID_NUMBER];

private:
    /* added for dynamic get sensor list */
    void fillSensorMaskArray();
    void fillAccel(const char* accel, struct sensor_t *list);
    void fillGyro(const char* gyro, struct sensor_t *list);
    void fillAGMSensors(struct sensor_t *list);
    void fillAMSensors(struct sensor_t *list);
    void fillAGSensors(struct sensor_t *list);
    void fillGestureSensors(struct sensor_t *list);
    void loadDMP(char *chipID);
    bool isMpuNonDmp();
    void resetCalStatus(int recal);
    void getCompassBias();
    void getFactoryGyroBias();
    void setFactoryGyroBias();
    void getGyroBias();
    void setDmpGyroBias(bool sysfs);
    void getDmpGyroBias(bool sysfs);
    void getFactoryAccelBias();
    void setFactoryAccelBias();
    void getAccelBias();
    void getDmpAccelBias(bool sysfs);
    void setDmpAccelBias(bool sysfs);
    void setDmpCompassBias();
    void getDmpCompassBias();
    int setBatchDataRates();
    int resetDataRates();
    void initBias();
    int lookupSensorRate(int inputRateUs) const;
    void printTimeProfile(int64_t &previousTime, int64_t currentTime, int64_t sensorTs, int64_t lastSensorTs);
    int updateImuTemperature();

#ifdef DIRECT_REPORT
    static void fillDirectReportFlags(struct sensor_t *sensor);
    int addDirectChannel(const struct sensors_direct_mem_t *mem);
    int removeDirectChannel(int channel_handle);
    int configDirectReport(int sensor_handle, int channel_handle, int rate_level);
    int stopAllDirectReportOnChannel(int channel_handle, std::vector<int32_t> *unstoppedSensors);
    void sendDirectReportEvent(const sensors_event_t *nev, size_t n);
    int64_t rateLevelToDeviceSamplingPeriodNs(int handle, int rateLevel) const;
    inline static bool intervalLargeEnough(uint64_t actual, uint64_t desired) {
        return (actual + (actual >> 4) + (actual >> 5)) >= desired; // >= 93.75% of desired
    }

    struct DirectChannelTimingInfo {
        uint64_t dataTimestamp;
        uint64_t dataPeriod;
        uint64_t lastTimestamp;
        int rateLevel;
    };
    Mutex mDirectChannelLock;
    //sensor_handle=>(channel_handle => DirectChannelTimingInfo)
    std::unordered_map<int32_t, std::unordered_map<int32_t, DirectChannelTimingInfo> > mSensorToChannel;
    //channel_handle=>ptr of Channel obj
    std::unordered_map<int32_t, std::unique_ptr<DirectChannelBase>> mDirectChannel;
    int32_t mDirectChannelHandle;
    std::unordered_map<int32_t, SensorChannelMux> mSensorChannelMuxes;
    std::unordered_map<int32_t, DirectChannelTimingInfo> mPollChannelTiming;
#endif

    MPLLogger mMPLLog{"/data/vendor/sensor", "AndroidSensors"};
};

extern "C" {
    void setCallbackObject(MPLSensor*);
    MPLSensor *getCallbackObject();
}

#endif  // ANDROID_MPL_SENSOR_H
