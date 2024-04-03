/*
 * Copyright (C) 2014-2018 InvenSense, Inc.
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

#ifndef ANDROID_SENSORS_H
#define ANDROID_SENSORS_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include <hardware/hardware.h>
#include <hardware/sensors.h>

__BEGIN_DECLS

/*****************************************************************************/

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#endif

/**
 * SENSOR_TYPE_EIS_GYROSCOPE
 * reporting-mode: special
 *
 * A sensor of this type triggers when the event of fsync from DMP
 * This sensor return gyro X,Y ans Z axis values when fsync is triggered
 *
 * Implement the non-wake-up versio of this sensor.
 */
#ifndef SENSOR_TYPE_EIS_GYROSCOPE
#  define SENSOR_TYPE_EIS_GYROSCOPE                 (SENSOR_TYPE_DEVICE_PRIVATE_BASE + 1)
#endif
#ifndef SENSOR_STRING_TYPE_EIS_GYROSCOPE
#  define SENSOR_STRING_TYPE_EIS_GYROSCOPE          "com.invensense.sensor.eis_gyroscope"
#endif

/**
 * SENSOR_TYPE_EIS_AUTHENTICATION
 * reporting-mode: special
 *
 * A sensor of this type triggers when the event of fsync from DMP
 * This sensor return gyro X,Y ans Z axis values when fsync is triggered
 *
 */
#ifndef SENSOR_TYPE_EIS_AUTHENTICATION
#  define SENSOR_TYPE_EIS_AUTHENTICATION            (SENSOR_TYPE_DEVICE_PRIVATE_BASE + 2)
#endif
#ifndef SENSOR_STRING_TYPE_EIS_AUTHENTICATION
#  define SENSOR_STRING_TYPE_EIS_AUTHENTICATION     "com.invensense.sensor.eis_authentication"
#endif

/**
 * SENSOR_TYPE_LPQ
 * reporting-mode: special
 *
 *
 */
#ifndef SENSOR_TYPE_LPQ
#  define SENSOR_TYPE_LPQ                           (SENSOR_TYPE_DEVICE_PRIVATE_BASE + 3)
#endif
#ifndef SENSOR_STRING_TYPE_LPQ
#  define SENSOR_STRING_TYPE_LPQ                    "com.invensense.sensor.lpq"
#endif

/**
 * SENSOR_TYPE_OIS_SENSOR
 * reporting-mode: special
 *
 * This sensor indicates HAL that OIS feature is enabled.
 *
 */
#ifndef SENSOR_TYPE_OIS_SENSOR
#  define SENSOR_TYPE_OIS_SENSOR                    (SENSOR_TYPE_DEVICE_PRIVATE_BASE + 4)
#endif
#ifndef SENSOR_STRING_TYPE_OIS_SENSOR
#  define SENSOR_STRING_TYPE_OIS_SENSOR             "com.invensense.sensor.ois_sensor"
#endif

/**
 * SENSOR_TYPE_TAP_GESTURE_SENSOR
 * reporting-mode: special
 *
 * TBD
 *
 */
#ifndef SENSOR_TYPE_TAP_GESTURE_SENSOR
#  define SENSOR_TYPE_TAP_GESTURE_SENSOR                    (SENSOR_TYPE_DEVICE_PRIVATE_BASE + 5)
#endif
#ifndef SENSOR_STRING_TYPE_TAP_GESTURE_SENSOR
#  define SENSOR_STRING_TYPE_TAP_GESTURE_SENSOR             "com.invensense.sensor.tap_gesture_sensor"
#endif

/**
 * SENSOR_TYPE_HEADING
 * Android Heading Sensor for Android < 13
 */
#ifndef SENSOR_STRING_TYPE_HEADING
#  define SENSOR_TYPE_HEADING				(SENSOR_TYPE_DEVICE_PRIVATE_BASE + 6)
#  define SENSOR_STRING_TYPE_HEADING			"android.sensor.heading"
#endif

enum {
    ID_GY = 0,
    ID_RG,
    ID_A,
    ID_RA,
    ID_M,
    ID_RM,
    ID_O,
    ID_RV,
    ID_GRV,
    ID_LA,
    ID_GR,
    ID_SM,
    ID_P,
    ID_SC,
    ID_GMRV,
    ID_T,
    ID_PICK,
    ID_STADET,
    ID_MOTDET,
    ID_EISGY,
    ID_EISAUTHENTICATION,
    ID_LPQ,
    ID_HEADING,

    ID_GYW,
    ID_RGW,
    ID_AW,
    ID_RAW,
    ID_MW,
    ID_RMW,
    ID_OW,
    ID_RVW,
    ID_GRVW,
    ID_LAW,
    ID_GRW,
    ID_PW,
    ID_SCW,
    ID_GMRVW,
    ID_PS,
    ID_PSW,
    ID_L,
    ID_LW,
    ID_PR,
    ID_PRW,
    ID_HEADINGW,
    ID_ARC, // Accel raw custom
    ID_GRC, // gyro raw custom
    ID_MRC, // magnetic raw custom
    ID_OIS, // OIS sensor
    ID_SO,
    ID_TAP,
    ID_NUMBER
};

#if (ID_NUMBER > 64)
#  error "Limitation: sensors ids must be < 64"
#endif

enum {
    Gyro = ID_GY,
    RawGyro = ID_RG,
    Accelerometer = ID_A,
    RawAccelerometer = ID_RA,
    MagneticField = ID_M,
    RawMagneticField = ID_RM,
    Orientation = ID_O,
    RotationVector = ID_RV,
    GameRotationVector = ID_GRV,
    LinearAccel = ID_LA,
    Gravity = ID_GR,
    SignificantMotion = ID_SM,
    StepDetector = ID_P,
    StepCounter = ID_SC,
    GeomagneticRotationVector = ID_GMRV,
    Tilt = ID_T,
    Pickup = ID_PICK,
    StationaryDetect = ID_STADET,
    MotionDetect = ID_MOTDET,
    EISGyroscope = ID_EISGY,
    EISAuthentication = ID_EISAUTHENTICATION,
    LPQ = ID_LPQ,
    Heading = ID_HEADING,
    Gyro_Wake = ID_GYW,
    RawGyro_Wake = ID_RGW,
    Accelerometer_Wake = ID_AW,
    RawAccelerometer_Wake = ID_RAW,
    MagneticField_Wake = ID_MW,
    RawMagneticField_Wake = ID_RMW,
    Orientation_Wake = ID_OW,
    RotationVector_Wake = ID_RVW,
    GameRotationVector_Wake = ID_GRVW,
    LinearAccel_Wake = ID_LAW,
    Gravity_Wake = ID_GRW,
    StepDetector_Wake = ID_PW,
    StepCounter_Wake = ID_SCW,
    GeomagneticRotationVector_Wake = ID_GMRVW,
    Pressure = ID_PS,
    Pressure_Wake = ID_PSW,
    Light = ID_L,
    Light_Wake = ID_LW,
    Proximity = ID_PR,
    Proximity_Wake = ID_PRW,
    Heading_Wake = ID_HEADINGW,
    AccelerometerRaw = ID_ARC,
    GyroRaw = ID_GRC,
    MagneticFieldRaw = ID_MRC,
    OisSensor = ID_OIS,
    ScreenOrientation = ID_SO,
    Tap = ID_TAP,
    TotalNumSensors = ID_NUMBER,
};

/* Physical parameters of the sensors supported by Invensense MPL */
#define SENSORS_GYROSCOPE_HANDLE                   (ID_GY)
#define SENSORS_RAW_GYROSCOPE_HANDLE               (ID_RG)
#define SENSORS_ACCELERATION_HANDLE                (ID_A)
#define SENSORS_RAW_ACCELERATION_HANDLE            (ID_RA)
#define SENSORS_MAGNETIC_FIELD_HANDLE              (ID_M)
#define SENSORS_RAW_MAGNETIC_FIELD_HANDLE          (ID_RM)
#define SENSORS_ORIENTATION_HANDLE                 (ID_O)
#define SENSORS_ROTATION_VECTOR_HANDLE             (ID_RV)
#define SENSORS_GAME_ROTATION_VECTOR_HANDLE        (ID_GRV)
#define SENSORS_LINEAR_ACCEL_HANDLE                (ID_LA)
#define SENSORS_GRAVITY_HANDLE                     (ID_GR)
#define SENSORS_SIGNIFICANT_MOTION_HANDLE          (ID_SM)
#define SENSORS_PEDOMETER_HANDLE                   (ID_P)
#define SENSORS_STEP_COUNTER_HANDLE                (ID_SC)
#define SENSORS_GEOMAGNETIC_ROTATION_VECTOR_HANDLE (ID_GMRV)
#define SENSORS_PRESSURE_HANDLE                    (ID_PS)
#define SENSORS_LIGHT_HANDLE                       (ID_L)
#define SENSORS_PROXIMITY_HANDLE                   (ID_PR)
#define SENSORS_WAKE_UP_TILT_DETECTOR_HANDLE       (ID_T)
#define SENSORS_PICK_UP_GESTURE_HANDLE             (ID_PICK)
#define SENSORS_STATIONARY_DETECT_HANDLE           (ID_STADET)
#define SENSORS_MOTION_DETECT_HANDLE               (ID_MOTDET)
#define SENSORS_EIS_GYROSCOPE_HANDLE               (ID_EISGY)
#define SENSORS_EIS_AUTHENTICATION_HANDLE          (ID_EISAUTHENTICATION)
#define SENSORS_LPQ_HANDLE                         (ID_LPQ)
#define SENSORS_HEADING_HANDLE                     (ID_HEADING)

#define SENSORS_GYROSCOPE_WAKEUP_HANDLE                   (ID_GYW)
#define SENSORS_RAW_GYROSCOPE_WAKEUP_HANDLE               (ID_RGW)
#define SENSORS_ACCELERATION_WAKEUP_HANDLE                (ID_AW)
#define SENSORS_RAW_ACCELERATION_WAKEUP_HANDLE            (ID_RAW)
#define SENSORS_MAGNETIC_FIELD_WAKEUP_HANDLE              (ID_MW)
#define SENSORS_RAW_MAGNETIC_FIELD_WAKEUP_HANDLE          (ID_RMW)
#define SENSORS_ORIENTATION_WAKEUP_HANDLE                 (ID_OW)
#define SENSORS_ROTATION_VECTOR_WAKEUP_HANDLE             (ID_RVW)
#define SENSORS_GAME_ROTATION_VECTOR_WAKEUP_HANDLE        (ID_GRVW)
#define SENSORS_LINEAR_ACCEL_WAKEUP_HANDLE                (ID_LAW)
#define SENSORS_GRAVITY_WAKEUP_HANDLE                     (ID_GRW)
#define SENSORS_PEDOMETER_WAKEUP_HANDLE                   (ID_PW)
#define SENSORS_STEP_COUNTER_WAKEUP_HANDLE                (ID_SCW)
#define SENSORS_GEOMAGNETIC_ROTATION_VECTOR_WAKEUP_HANDLE (ID_GMRVW)
#define SENSORS_PRESSURE_WAKEUP_HANDLE                    (ID_PSW)
#define SENSORS_LIGHT_WAKEUP_HANDLE                       (ID_LW)
#define SENSORS_PROXIMITY_WAKEUP_HANDLE                   (ID_PRW)
#define SENSORS_HEADING_WAKEUP_HANDLE                     (ID_HEADINGW)
#define SENSORS_ACCELERATION_RAW_HANDLE                   (ID_ARC)
#define SENSORS_GYROSCOPE_RAW_HANDLE                      (ID_GRC)
#define SENSORS_MAGNETIC_FIELD_RAW_HANDLE                 (ID_MRC)
#define SENSORS_OIS_SENSOR_HANDLE                         (ID_OIS)
#define SENSORS_SCREEN_ORIENTATION_HANDLE                 (ID_SO)
#define SENSORS_TAP_GESTURE_HANDLE                        (ID_TAP)

/*****************************************************************************/

/*
 * The SENSORS Module
 */

/* secondary compass */
#define EVENT_TYPE_ICOMPASS_X      REL_X
#define EVENT_TYPE_ICOMPASS_Y      REL_Y
#define EVENT_TYPE_ICOMPASS_Z      REL_Z

// conversion of acceleration data to SI units (m/s^2)
#define RANGE_A                     (4*GRAVITY_EARTH)
#define RESOLUTION_A                (GRAVITY_EARTH / LSG)
#define CONVERT_A                   (GRAVITY_EARTH / LSG)
#define CONVERT_A_X                 (CONVERT_A)
#define CONVERT_A_Y                 (CONVERT_A)
#define CONVERT_A_Z                 (CONVERT_A)

/* AKM  compasses */
#define EVENT_TYPE_MAGV_X           ABS_RX
#define EVENT_TYPE_MAGV_Y           ABS_RY
#define EVENT_TYPE_MAGV_Z           ABS_RZ
#define EVENT_TYPE_MAGV_STATUS      ABS_RUDDER

/* conversion of magnetic data to uT units */
#define CONVERT_M                   (0.06f)

/* conversion of sensor rates */
#define DEFAULT_MPL_GYRO_RATE           (20000L)     //us
#define DEFAULT_MPL_COMPASS_RATE        (20000L)     //us

#define DEFAULT_HW_GYRO_RATE            (100)        //Hz
#define DEFAULT_HW_ACCEL_RATE           (20)         //ms
#define DEFAULT_HW_COMPASS_RATE         (20000000L)  //ns
#define DEFAULT_HW_AKMD_COMPASS_RATE    (200000000L) //ns

#define RATE_200HZ                      5000000LL
#define RATE_15HZ                       66667000LL
#define RATE_5HZ                        200000000LL

/* convert radian to degree */
#define RAD_P_DEG                      (3.14159f/180.f)

#define PROXIMITY_RANGE                 280

__END_DECLS

#endif  // ANDROID_SENSORS_H
