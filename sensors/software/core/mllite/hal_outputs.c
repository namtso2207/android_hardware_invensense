/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */

/**
 *   @defgroup  HAL_Outputs hal_outputs
 *   @brief     Motion Library - HAL Outputs
 *              Sets up common outputs for HAL
 *
 *   @{
 *       @file  hal_outputs.c
 *       @brief HAL Outputs.
 */

#undef MPL_LOG_NDEBUG
#define MPL_LOG_NDEBUG 1 /* Use 0 to turn on MPL_LOGV output */
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MLLITE"
//#define MPL_LOG_9AXIS_DEBUG 1

#include <string.h>

#include "hal_outputs.h"
#include "log.h"
#include "ml_math_func.h"
#include "mlmath.h"
#include "start_manager.h"
#include "data_builder.h"
#include "results_holder.h"

/* commenting this define out will bypass the low pass filter noise reduction
   filter for compass data.
   Disable this only for testing purpose (e.g. comparing the raw and calibrated
   compass data, since the former is unfiltered and the latter is filtered,
   leading to a small difference in the readings sample by sample).
   Android specifications require this filter to be enabled to have the Magnetic
   Field output's standard deviation fall below 0.5 uT.
   */
/* This filter is no longer required for newer compass devices such as
 * AK09916 in ICM20948.
 */
//#define CALIB_COMPASS_NOISE_REDUCTION

struct hal_output_t {
    int accuracy_mag;    /**< Compass accuracy */
    //int accuracy_gyro;   /**< Gyro Accuracy */
    //int accuracy_accel;  /**< Accel Accuracy */
    int accuracy_quat;   /**< quat Accuracy */

    inv_time_t nav_timestamp;
    inv_time_t gam_timestamp;
    //inv_time_t accel_timestamp;
    inv_time_t mag_timestamp;
    int nav_quat[4];
    int gyro_status;
    int gyro_cal_status;
    int accel_status;
    int compass_status;
    int compass_cal_status;
    int nine_axis_status;
    int quat_status;
    inv_biquad_filter_t lp_filter[3];
    float compass_float[3];
    int linear_acceleration_sample_rate_us;
    int orientation_sample_rate_us;
    int rotation_vector_sample_rate_us;
    int gravity_sample_rate_us;
    int orientation_6_axis_sample_rate_us;
    int orientation_geomagnetic_sample_rate_us;
    int rotation_vector_6_axis_sample_rate_us;
    int geomagnetic_rotation_vector_sample_rate_us;
};

static struct hal_output_t hal_out;

void inv_set_linear_acceleration_sample_rate(int sample_rate_us)
{
    hal_out.linear_acceleration_sample_rate_us = sample_rate_us;
}

void inv_set_orientation_sample_rate(int sample_rate_us)
{
    hal_out.orientation_sample_rate_us = sample_rate_us;
}

void inv_set_rotation_vector_sample_rate(int sample_rate_us)
{
    hal_out.rotation_vector_sample_rate_us = sample_rate_us;
}

void inv_set_gravity_sample_rate(int sample_rate_us)
{
    hal_out.gravity_sample_rate_us = sample_rate_us;
}

void inv_set_orientation_6_axis_sample_rate(int sample_rate_us)
{
    hal_out.orientation_6_axis_sample_rate_us = sample_rate_us;
}

void inv_set_orientation_geomagnetic_sample_rate(int sample_rate_us)
{
    hal_out.geomagnetic_rotation_vector_sample_rate_us = sample_rate_us;
}

void inv_set_rotation_vector_6_axis_sample_rate(int sample_rate_us)
{
    hal_out.rotation_vector_6_axis_sample_rate_us = sample_rate_us;
}

void inv_set_geomagnetic_rotation_vector_sample_rate(int sample_rate_us)
{
    hal_out.geomagnetic_rotation_vector_sample_rate_us = sample_rate_us;
}

/* Converts fixed point to m/s^2. Fixed point has 1g = 2^16.
 * So this 9.80665 / 2^16 */
#define ACCEL_CONVERSION 0.000149637603759766f

/** Acceleration (m/s^2) in body frame.
* @param[out] values Acceleration in m/s^2 includes gravity. So while not in motion, it
*             should return a vector of magnitude near 9.81 m/s^2
* @param[out] accuracy Accuracy of the measurment, 0 is least accurate, while 3 is most accurate.
* @param[out] timestamp The timestamp for this sensor. Derived from the timestamp sent to
*             inv_build_accel().
* @return     Returns 1 if the data was updated or 0 if it was not updated.
*/
int inv_get_sensor_type_accelerometer(float *values, int8_t *accuracy,
                                       inv_time_t * timestamp)
{
    int status;
    int accel[3];
    inv_get_accel_set(accel, accuracy, timestamp);

    values[0] = accel[0] * ACCEL_CONVERSION;
    values[1] = accel[1] * ACCEL_CONVERSION;
    values[2] = accel[2] * ACCEL_CONVERSION;
    if (hal_out.accel_status & INV_NEW_DATA)
        status = 1;
    else
        status = 0;
    MPL_LOGV("accel values:%f %f %f -%d -%lld", values[0], values[1],
                              values[2], status, (long long)(*timestamp));
    return status;
}

/** Acceleration (m/s^2) in body frame (custom).
* @param[out] values Acceleration in m/s^2 includes gravity. So while not in motion, it
*             should return a vector of magnitude near 9.81 m/s^2
* @param[out] accuracy Accuracy of the measurment, 0 is least accurate, while 3 is most accurate.
* @param[out] timestamp The timestamp for this sensor. Derived from the timestamp sent to
*             inv_build_accel().
* @return     Returns 1 if the data was updated or 0 if it was not updated.
*/
int inv_get_sensor_type_accelerometer_custom(float *values, int8_t *accuracy,
                                       inv_time_t * timestamp)
{
    int status;
    int accel[3];
    inv_get_accel_set_custom(accel, accuracy, timestamp);
    MPL_LOGV("accel(custom) %d %d %d",accel[0], accel[1], accel[2]);
    values[0] = accel[0] * ACCEL_CONVERSION;
    values[1] = accel[1] * ACCEL_CONVERSION;
    values[2] = accel[2] * ACCEL_CONVERSION;
    if (hal_out.accel_status & INV_NEW_DATA)
        status = 1;
    else
        status = 0;
    MPL_LOGV("accel values (custom):%f %f %f -%d -%lld", values[0], values[1],
                              values[2], status, (long long)(*timestamp));
    return status;
}

/** Linear Acceleration (m/s^2) in Body Frame.
* @param[out] values Linear Acceleration in body frame, length 3, (m/s^2). May show
*             accel biases while at rest.
* @param[out] accuracy Accuracy of the measurment, 0 is least accurate, while 3 is most accurate.
* @param[out] timestamp The timestamp for this sensor. Derived from the timestamp sent to
*             inv_build_accel().
* @return     Returns 1 if the data was updated or 0 if it was not updated.
*/
int inv_get_sensor_type_linear_acceleration(float *values, int8_t *accuracy,
        inv_time_t * timestamp)
{
    int gravity[3], accel[3];
    inv_time_t timestamp1;

    inv_get_accel_set(accel, accuracy, &timestamp1);
    inv_get_gravity_6x(gravity);
    inv_get_6axis_quaternion_timestamp(timestamp);
    accel[0] -= gravity[0] >> 14;
    accel[1] -= gravity[1] >> 14;
    accel[2] -= gravity[2] >> 14;
    values[0] = accel[0] * ACCEL_CONVERSION;
    values[1] = accel[1] * ACCEL_CONVERSION;
    values[2] = accel[2] * ACCEL_CONVERSION;

    return 1;
}

/** Gravity vector (m/s^2) in Body Frame.
* @param[out] values Gravity vector in body frame, length 3, (m/s^2)
* @param[out] accuracy Accuracy of the measurment, 0 is least accurate, while 3 is most accurate.
* @param[out] timestamp The timestamp for this sensor. Derived from the timestamp sent to
*             inv_build_accel().
* @return     Returns 1 if the data was updated or 0 if it was not updated.
*/
int inv_get_sensor_type_gravity(float *values, int8_t *accuracy,
                                 inv_time_t * timestamp)
{
    int gravity[3];

    *accuracy = (int8_t) hal_out.accuracy_quat;
    inv_get_gravity_6x(gravity);
    inv_get_6axis_quaternion_timestamp(timestamp);
    values[0] = (gravity[0] >> 14) * ACCEL_CONVERSION;
    values[1] = (gravity[1] >> 14) * ACCEL_CONVERSION;
    values[2] = (gravity[2] >> 14) * ACCEL_CONVERSION;

    return 1;
}

/* Converts fixed point to rad/sec. Fixed point has 1 dps = 2^16.
 * So this is: pi / 2^16 / 180 */
#define GYRO_CONVERSION 2.66316109007924e-007f

/** Gyroscope calibrated data (rad/s) in body frame.
* @param[out] values Rotation Rate in rad/sec.
* @param[out] accuracy Accuracy of the measurment, 0 is least accurate, while 3 is most accurate.
* @param[out] timestamp The timestamp for this sensor. Derived from the timestamp sent to
*             inv_build_gyro().
* @return     Returns 1 if the data was updated or 0 if it was not updated.
*/
int inv_get_sensor_type_gyroscope(float *values, int8_t *accuracy,
                                   inv_time_t * timestamp)
{
    int gyro[3];
    int status;

    inv_get_gyro_set(gyro, accuracy, timestamp);

    values[0] = gyro[0] * GYRO_CONVERSION;
    values[1] = gyro[1] * GYRO_CONVERSION;
    values[2] = gyro[2] * GYRO_CONVERSION;
    if (hal_out.gyro_cal_status & INV_NEW_DATA)
        status = 1;
    else
        status = 0;
    return status;
}

/** Gyroscope raw data (rad/s) in body frame.
* @param[out] values Rotation Rate in rad/sec.
* @param[out] accuracy Accuracy of the measurment, 0 is least accurate,
*                      while 3 is most accurate.
* @param[out] timestamp The timestamp for this sensor. Derived from the
*                       timestamp sent to inv_build_gyro().
* @return     Returns 1 if the data was updated or 0 if it was not updated.
*/
int inv_get_sensor_type_gyroscope_raw(float *values, int8_t *accuracy,
                                      inv_time_t * timestamp)
{
    int gyro[3];
    int status;

    inv_get_gyro_set_raw(gyro, accuracy, timestamp);
    values[0] = gyro[0] * GYRO_CONVERSION;
    values[1] = gyro[1] * GYRO_CONVERSION;
    values[2] = gyro[2] * GYRO_CONVERSION;
    if (hal_out.gyro_status & INV_NEW_DATA)
        status = 1;
    else
        status = 0;
    return status;
}

/** Gyroscope raw data (rad/s) in body frame.
* @param[out] values Rotation Rate in rad/sec.
* @param[out] accuracy Accuracy of the measurment, 0 is least accurate,
*                      while 3 is most accurate.
* @param[out] timestamp The timestamp for this sensor. Derived from the
*                       timestamp sent to inv_build_gyro().
* @return     Returns 1 if the data was updated or 0 if it was not updated.
*/
int inv_get_sensor_type_gyroscope_raw_custom(float *values, int8_t *accuracy,
                                      inv_time_t * timestamp)
{
    int gyro[3];
    int status;

    inv_get_gyro_set_raw_custom(gyro, accuracy, timestamp);
    values[0] = gyro[0] * GYRO_CONVERSION;
    values[1] = gyro[1] * GYRO_CONVERSION;
    values[2] = gyro[2] * GYRO_CONVERSION;
    MPL_LOGV("MLLITE : Gyro custom %f %f %f",values[0], values[1], values[2]);

    if (hal_out.gyro_status & INV_NEW_DATA)
        status = 1;
    else
        status = 0;
    return status;
}


/** EIS Gyroscope  data (rad/s) in body frame.
* @param[out] values Rotation Rate in rad/sec.
* @param[out] accuracy Accuracy of the measurment, 0 is least accurate, while 3 is most accurate.
* @param[out] timestamp The timestamp for this sensor. Derived from the timestamp sent to
*             inv_build_eis_gyro().
* @return     Returns 1 if the data was updated or 0 if it was not updated.
*/
int inv_get_sensor_type_eis_gyroscope(float *values, int8_t *accuracy,
                                   inv_time_t * timestamp)
{
    int eis_gyro[4];
    int status;

    inv_get_eis_gyro_set(eis_gyro, accuracy, timestamp);

    values[0] = eis_gyro[0] * GYRO_CONVERSION;
    values[1] = eis_gyro[1] * GYRO_CONVERSION;
    values[2] = eis_gyro[2] * GYRO_CONVERSION;
    values[3] = eis_gyro[3];

    if (eis_gyro[0] == 0 && eis_gyro[1] == 0 && eis_gyro[2] == 0)
	status = 0;
    else
	status = 1;

    MPL_LOGV("MLLITE : EIS GYRO %f %f %f %f",values[0], values[1], values[2], values[3]);
    return status;
}

/** EIS   Authentication data
* @param[out] values Rotation Rate in rad/sec.
* @param[out] accuracy Accuracy of the measurment, 0 is least accurate, while 3 is most accurate.
* @param[out] timestamp The timestamp for this sensor. Derived from the timestamp sent to
*             inv_build_eis_gyro().
* @return     Returns 1 if the data was updated or 0 if it was not updated.
*/
int inv_get_sensor_type_eis_authentication(float *values, int8_t *accuracy,
                                   inv_time_t * timestamp)
{
    int eis_authentication[4];
    int status;

    inv_get_eis_authentication_set(eis_authentication, accuracy, timestamp);

    values[0] = eis_authentication[0];

    if (eis_authentication[0] == 0)
        status = 0;
    else
        status = 1;

    return status;
}

/**
* This corresponds to Sensor.TYPE_ROTATION_VECTOR.
* The rotation vector represents the orientation of the device as a combination
* of an angle and an axis, in which the device has rotated through an angle @f$\theta@f$
* around an axis {x, y, z}. <br>
* The three elements of the rotation vector are
* {x*sin(@f$\theta@f$/2), y*sin(@f$\theta@f$/2), z*sin(@f$\theta@f$/2)}, such that the magnitude of the rotation
* vector is equal to sin(@f$\theta@f$/2), and the direction of the rotation vector is
* equal to the direction of the axis of rotation.
*
* The three elements of the rotation vector are equal to the last three components of a unit quaternion
* {x*sin(@f$\theta@f$/2), y*sin(@f$\theta@f$/2), z*sin(@f$\theta@f$/2)>. The 4th element is cos(@f$\theta@f$/2).
*
* Elements of the rotation vector are unitless. The x,y and z axis are defined in the same way as the acceleration sensor.
* The reference coordinate system is defined as a direct orthonormal basis, where:
*
*   -X is defined as the vector product Y.Z (It is tangential to the ground at the device's current location and roughly points East).
*   -Y is tangential to the ground at the device's current location and points towards the magnetic North Pole.
*   -Z points towards the sky and is perpendicular to the ground.
* @param[out] values
*               Length 5, 4th element being the w angle of the originating 4
*               elements quaternion and 5th element being the heading accuracy
*               at 95%.
* @param[out] accuracy Accuracy is not defined
* @param[out] timestamp Timestamp. In (ns) for Android.
* @return     Returns 1 if the data was updated or 0 if it was not updated.
*/
int inv_get_sensor_type_rotation_vector(float *values, int8_t *accuracy,
        inv_time_t * timestamp)
{
	int quat_9_axis[4];

    *accuracy = 0; // dummy
    inv_get_quaternion(quat_9_axis);
    inv_get_quaternion_timestamp(timestamp);

    if (quat_9_axis[0] >= 0) {
        values[0] = quat_9_axis[1] * INV_TWO_POWER_NEG_30;
        values[1] = quat_9_axis[2] * INV_TWO_POWER_NEG_30;
        values[2] = quat_9_axis[3] * INV_TWO_POWER_NEG_30;
        values[3] = quat_9_axis[0] * INV_TWO_POWER_NEG_30;
    } else {
        values[0] = -quat_9_axis[1] * INV_TWO_POWER_NEG_30;
        values[1] = -quat_9_axis[2] * INV_TWO_POWER_NEG_30;
        values[2] = -quat_9_axis[3] * INV_TWO_POWER_NEG_30;
        values[3] = -quat_9_axis[0] * INV_TWO_POWER_NEG_30;
    }
    values[4] = 1;

    return 1;
}

int inv_get_sensor_type_lpq(float *values, int8_t *accuracy,
        inv_time_t * timestamp)
{
    int status;
    float quat_3_axis[4];

    *accuracy = 0; // dummy
    inv_get_3axis_quaternion(quat_3_axis, timestamp);

#ifdef INV_IPL
    values[0] = quat_3_axis[0];
    values[1] = quat_3_axis[1];
    values[2] = quat_3_axis[2];
    values[3] = quat_3_axis[3];
#endif
    values[4] = 0;
    if (hal_out.quat_status & INV_QUAT_3AXIS)
    {
        status = hal_out.quat_status & INV_NEW_DATA? 1 : 0;
    }
    MPL_LOGV("values:%f %f %f %f %f -%d -%lld", values[0], values[1],
                              values[2], values[3], values[4], status, (long long)(*timestamp));

    return 1;
}

int inv_get_sensor_type_rotation_vector_6_axis(float *values, int8_t *accuracy,
        inv_time_t * timestamp)
{
    int status;
    int accel[3], quat_6_axis[4];
    inv_time_t timestamp1;
    inv_get_accel_set(accel, accuracy, &timestamp1);
    inv_get_6axis_quaternion(quat_6_axis, &timestamp1);
    inv_get_6axis_quaternion_timestamp(timestamp);

    if (quat_6_axis[0] >= 0) {
        values[0] = quat_6_axis[1] * INV_TWO_POWER_NEG_30;
        values[1] = quat_6_axis[2] * INV_TWO_POWER_NEG_30;
        values[2] = quat_6_axis[3] * INV_TWO_POWER_NEG_30;
        values[3] = quat_6_axis[0] * INV_TWO_POWER_NEG_30;
    } else {
        values[0] = -quat_6_axis[1] * INV_TWO_POWER_NEG_30;
        values[1] = -quat_6_axis[2] * INV_TWO_POWER_NEG_30;
        values[2] = -quat_6_axis[3] * INV_TWO_POWER_NEG_30;
        values[3] = -quat_6_axis[0] * INV_TWO_POWER_NEG_30;
    }
    //This sensor does not report an estimated heading accuracy
    values[4] = 0;
    if (hal_out.quat_status & INV_QUAT_3AXIS)
    {
        status = hal_out.quat_status & INV_NEW_DATA? 1 : 0;
    }
    else {
        status = hal_out.accel_status & INV_NEW_DATA? 1 : 0;
    }
    MPL_LOGV("values:%f %f %f %f %f -%d -%lld", values[0], values[1],
                              values[2], values[3], values[4], status, (long long)timestamp1);

    return 1;
}

/**
* This corresponds to Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR.
* Similar to SENSOR_TYPE_ROTATION_VECTOR, but using a magnetometer
* instead of using a gyroscope.
* Fourth element = estimated_accuracy in radians (heading confidence).
* @param[out] values Length 4.
* @param[out] accuracy is not defined.
* @param[out] timestamp in (ns) for Android.
* @return     Returns 1 if the data was updated, 0 otherwise.
*/
int inv_get_sensor_type_geomagnetic_rotation_vector(float *values, int8_t *accuracy,
        inv_time_t * timestamp)
{
    int compass[3], quat_geomagnetic[4];
    int status;
    inv_time_t timestamp1;
    inv_get_compass_set(compass, accuracy, &timestamp1);
    inv_get_geomagnetic_quaternion(quat_geomagnetic, &timestamp1);
    inv_get_geomag_quaternion_timestamp(timestamp);

    if (quat_geomagnetic[0] >= 0) {
        values[0] = quat_geomagnetic[1] * INV_TWO_POWER_NEG_30;
        values[1] = quat_geomagnetic[2] * INV_TWO_POWER_NEG_30;
        values[2] = quat_geomagnetic[3] * INV_TWO_POWER_NEG_30;
        values[3] = quat_geomagnetic[0] * INV_TWO_POWER_NEG_30;
    } else {
        values[0] = -quat_geomagnetic[1] * INV_TWO_POWER_NEG_30;
        values[1] = -quat_geomagnetic[2] * INV_TWO_POWER_NEG_30;
        values[2] = -quat_geomagnetic[3] * INV_TWO_POWER_NEG_30;
        values[3] = -quat_geomagnetic[0] * INV_TWO_POWER_NEG_30;
    }
    values[4] = inv_get_accel_compass_confidence_interval();

    status = 1;

    MPL_LOGV("values:%f %f %f %f %f -%d", values[0], values[1],
                              values[2], values[3], values[4], status);

    return status;
}

/** Compass data (uT) in body frame.
* @param[out] values Compass data in (uT), length 3. May be calibrated by having
*             biases removed and sensitivity adjusted
* @param[out] accuracy Accuracy 0 to 3, 3 = most accurate
* @param[out] timestamp Timestamp. In (ns) for Android.
* @return     Returns 1 if the data was updated or 0 if it was not updated.
*/
int inv_get_sensor_type_magnetic_field(float *values, int8_t *accuracy,
                                        inv_time_t * timestamp)
{
    int status;
    int i;

    *timestamp = hal_out.mag_timestamp;
    *accuracy = (int8_t) hal_out.accuracy_mag;

    for (i = 0; i < 3; i++)
        values[i] = hal_out.compass_float[i];
    if (hal_out.compass_cal_status & INV_NEW_DATA)
        status = 1;
    else
        status = 0;
    return status;
}

/* Converts fixed point to uT. Fixed point has 1 uT = 2^16.
 * So this is: 1 / 2^16*/
#define COMPASS_CONVERSION 1.52587890625e-005f

/** Compass raw data (uT) in body frame.
* @param[out] values Compass data in (uT), length 3. May be calibrated by having
*             biases removed and sensitivity adjusted
* @param[out] accuracy Accuracy 0 to 3, 3 = most accurate
* @param[out] timestamp Timestamp. In (ns) for Android.
* @return     Returns 1 if the data was updated or 0 if it was not updated.
*/
int inv_get_sensor_type_magnetic_field_raw(float *values, int8_t *accuracy,
                                           inv_time_t * timestamp)
{
    int mag[3];
    int status;
    int i;

    inv_get_compass_set_raw(mag, accuracy, timestamp);

    /* Converts fixed point to uT. Fixed point has 1 uT = 2^16.
     * So this is: 1 / 2^16*/
#define COMPASS_CONVERSION 1.52587890625e-005f

    for (i = 0; i < 3; i++) {
        values[i] = (float)mag[i] * COMPASS_CONVERSION;
    }
    if (hal_out.compass_status & INV_NEW_DATA)
        status = 1;
    else
        status = 0;
    return status;
}


/** Compass raw data (uT) in body frame (custom).
* @param[out] values Compass data in (uT), length 3. May be calibrated by having
*             biases removed and sensitivity adjusted
* @param[out] accuracy Accuracy 0 to 3, 3 = most accurate
* @param[out] timestamp Timestamp. In (ns) for Android.
* @return     Returns 1 if the data was updated or 0 if it was not updated.
*/
int inv_get_sensor_type_magnetic_field_raw_custom(float *values, int8_t *accuracy,
                                           inv_time_t * timestamp)
{
    int mag[3];
    int status;
    int i;

    inv_get_compass_set_raw_custom(mag, accuracy, timestamp);

    for (i = 0; i < 3; i++) {
        values[i] = (float)mag[i] * COMPASS_CONVERSION;
    }
    if (hal_out.compass_status & INV_NEW_DATA)
        status = 1;
    else
        status = 0;
    return status;
}

static void inv_get_rotation_geomagnetic(float r[3][3])
{
    int rot[9], quat_geo[4];
    float conv = 1.f / (1L<<30);
    inv_time_t timestamp;
    inv_get_geomagnetic_quaternion(quat_geo, &timestamp);
    inv_quaternion_to_rotation(quat_geo, rot);
    r[0][0] = rot[0]*conv;
    r[0][1] = rot[1]*conv;
    r[0][2] = rot[2]*conv;
    r[1][0] = rot[3]*conv;
    r[1][1] = rot[4]*conv;
    r[1][2] = rot[5]*conv;
    r[2][0] = rot[6]*conv;
    r[2][1] = rot[7]*conv;
    r[2][2] = rot[8]*conv;
}
static void google_orientation_geomagnetic(float *g)
{
    float rad2deg = (float)(180.0 / M_PI);
    float R[3][3];
    inv_get_rotation_geomagnetic(R);
    g[0] = atan2f(-R[1][0], R[0][0]) * rad2deg;
    g[1] = atan2f(-R[2][1], R[2][2]) * rad2deg;
    g[2] = asinf ( R[2][0])          * rad2deg;
    if (g[0] < 0)
        g[0] += 360;
}

static void inv_get_rotation_6_axis(float r[3][3])
{
    int rot[9], quat_6_axis[4];
    float conv = 1.f / (1L<<30);
    inv_time_t timestamp;

    inv_get_6axis_quaternion(quat_6_axis, &timestamp);
    inv_quaternion_to_rotation(quat_6_axis, rot);
    r[0][0] = rot[0]*conv;
    r[0][1] = rot[1]*conv;
    r[0][2] = rot[2]*conv;
    r[1][0] = rot[3]*conv;
    r[1][1] = rot[4]*conv;
    r[1][2] = rot[5]*conv;
    r[2][0] = rot[6]*conv;
    r[2][1] = rot[7]*conv;
    r[2][2] = rot[8]*conv;
}

static void google_orientation_6_axis(float *g)
{
    float rad2deg = (float)(180.0 / M_PI);
    float R[3][3];

    inv_get_rotation_6_axis(R);

    g[0] = atan2f(-R[1][0], R[0][0]) * rad2deg;
    g[1] = atan2f(-R[2][1], R[2][2]) * rad2deg;
    g[2] = asinf ( R[2][0])          * rad2deg;
    if (g[0] < 0)
        g[0] += 360;
}

static void inv_get_rotation(float r[3][3])
{
    int rot[9], quat_9_axis[4];
    float conv = 1.f / (1L<<30);

    inv_get_quaternion(quat_9_axis);
    inv_quaternion_to_rotation(quat_9_axis, rot);
    r[0][0] = rot[0]*conv;
    r[0][1] = rot[1]*conv;
    r[0][2] = rot[2]*conv;
    r[1][0] = rot[3]*conv;
    r[1][1] = rot[4]*conv;
    r[1][2] = rot[5]*conv;
    r[2][0] = rot[6]*conv;
    r[2][1] = rot[7]*conv;
    r[2][2] = rot[8]*conv;
}

static void google_orientation(float *g)
{
    float rad2deg = (float)(180.0 / M_PI);
    float R[3][3];

    inv_get_rotation(R);

    g[0] = atan2f(-R[1][0], R[0][0]) * rad2deg;
    g[1] = atan2f(-R[2][1], R[2][2]) * rad2deg;
    g[2] = asinf ( R[2][0])          * rad2deg;
    if (g[0] < 0)
        g[0] += 360;
}
/*
 compute google heading
[in]  r, quaternion, 4 elements
[out] g, degree in google orientation
*/
void quat_to_google_orientation(int *quat_axis, float *g)
{
    float rad2deg = (float)(180.0 / M_PI);
    float r[3][3];

    int rot[9];
    float conv = 1.f / (1L<<30);

    inv_quaternion_to_rotation(quat_axis, rot);
    r[0][0] = rot[0]*conv;
    r[0][1] = rot[1]*conv;
    r[0][2] = rot[2]*conv;
    r[1][0] = rot[3]*conv;
    r[1][1] = rot[4]*conv;
    r[1][2] = rot[5]*conv;
    r[2][0] = rot[6]*conv;
    r[2][1] = rot[7]*conv;
    r[2][2] = rot[8]*conv;

    g[0] = atan2f(-r[1][0], r[0][0]) * rad2deg;
    g[1] = atan2f(-r[2][1], r[2][2]) * rad2deg;
    g[2] = asinf ( r[2][0])          * rad2deg;
    if (g[0] < 0)
        g[0] += 360;
}

/** This corresponds to Sensor.TYPE_ORIENTATION. All values are angles in degrees.
* @param[out] values Length 3, Degrees.<br>
*        - values[0]: Azimuth, angle between the magnetic north direction
*         and the y-axis, around the z-axis (0 to 359). 0=North, 90=East, 180=South, 270=West<br>
*        - values[1]: Pitch, rotation around x-axis (-180 to 180), with positive values
*         when the z-axis moves toward the y-axis.<br>
*        - values[2]: Roll, rotation around y-axis (-90 to 90), with positive
*          values when the x-axis moves toward the z-axis.<br>
*
* @note  This definition is different from yaw, pitch and roll used in aviation
*        where the X axis is along the long side of the plane (tail to nose).
*        Note: This sensor type exists for legacy reasons, please use getRotationMatrix()
*        in conjunction with remapCoordinateSystem() and getOrientation() to compute
*        these values instead.
*        Important note: For historical reasons the roll angle is positive in the
*        clockwise direction (mathematically speaking, it should be positive in
*        the counter-clockwise direction).
* @param[out] accuracy Accuracy of the measurment, 0 is least accurate, while 3 is most accurate.
* @param[out] timestamp The timestamp for this sensor.
* @return     Returns 1 if the data was updated or 0 if it was not updated.
*/
int inv_get_sensor_type_orientation(float *values, int8_t *accuracy,
                                     inv_time_t * timestamp)
{
    *accuracy = (int8_t) hal_out.accuracy_quat;
    google_orientation(values);
    inv_get_quaternion_timestamp(timestamp);

    return 1;
}

int inv_get_sensor_type_orientation_6_axis(float *values, int8_t *accuracy,
                                     inv_time_t * timestamp)
{
    int accel[3];
    inv_time_t timestamp1;
    inv_get_accel_set(accel, accuracy, &timestamp1);

    google_orientation_6_axis(values);
    inv_get_6axis_quaternion_timestamp(timestamp);

    return 1;
}

int inv_get_sensor_type_orientation_geomagnetic(float *values, int8_t *accuracy,
                                     inv_time_t * timestamp)
{
    int compass[3], quat_geomagnetic[4];
    inv_time_t timestamp1;
    inv_get_compass_set(compass, accuracy, &timestamp1);
    #if 0
    inv_get_geomagnetic_quaternion(quat_geomagnetic, &timestamp1);
    #endif
    inv_get_quaternion(quat_geomagnetic);

    google_orientation_geomagnetic(values);
    inv_get_geomag_quaternion_timestamp(timestamp);

    return 1;
}

int inv_get_sensor_type_heading(float *values, int8_t *accuracy,
                                inv_time_t * timestamp)
{
    const float rad2deg = (float)(180.0 / M_PI);
    float orient[3];

    google_orientation(orient);
    values[0] = orient[0];
    values[1] = 1 * rad2deg;
    *accuracy = 0;
    inv_get_quaternion_timestamp(timestamp);

    return 1;
}

/** Main callback to generate HAL outputs. Typically not called by library users.
* @param[in] sensor_cal Input variable to take sensor data whenever there is new
* sensor data.
* @return Returns INV_SUCCESS if successful or an error code if not.
*/
inv_error_t inv_generate_hal_outputs(struct inv_sensor_cal_t *sensor_cal)
{
    int use_sensor = 0;
    int sr = 1000;
    int compass[3];
    int8_t accuracy;
    int i;
    (void) sensor_cal;

    inv_get_quaternion_set(hal_out.nav_quat, &hal_out.accuracy_quat,
                           &hal_out.nav_timestamp);
    hal_out.gyro_status = sensor_cal->gyro.status;
    hal_out.gyro_cal_status = sensor_cal->gyro.cal_status;
    hal_out.accel_status = sensor_cal->accel.status;
    hal_out.compass_status = sensor_cal->compass.status;
    hal_out.compass_cal_status = sensor_cal->compass.cal_status;
    hal_out.quat_status = sensor_cal->quat.status;
#if MPL_LOG_9AXIS_DEBUG
    MPL_LOGV("hal_out:g=%d", hal_out.gyro_status);
#endif
    // Find the highest sample rate and tie generating 9-axis to that one.
    if (sensor_cal->gyro.status & INV_SENSOR_ON) {
        sr = sensor_cal->gyro.sample_rate_ms;
        use_sensor = 0;
    }
    if ((sensor_cal->accel.status & INV_SENSOR_ON) && (sr > sensor_cal->accel.sample_rate_ms)) {
        sr = sensor_cal->accel.sample_rate_ms;
        use_sensor = 1;
    }
    if (((sensor_cal->compass.status & INV_SENSOR_ON) && (sr > sensor_cal->compass.sample_rate_ms)) ||
        ((sensor_cal->compass.cal_status & INV_SENSOR_ON) && (sr > sensor_cal->compass.sample_rate_ms)) ) {
        sr = sensor_cal->compass.sample_rate_ms;
        use_sensor = 2;
    }
    if ((sensor_cal->quat.status & INV_SENSOR_ON) && (sr > sensor_cal->quat.sample_rate_ms)) {
        sr = sensor_cal->quat.sample_rate_ms;
        use_sensor = 3;
    }

    // If the timestamp did not change, remove the new data flag
    if (sensor_cal->gyro.timestamp_prev == sensor_cal->gyro.timestamp) {
        hal_out.gyro_status &= ~INV_NEW_DATA;
    }
    if (sensor_cal->gyro.timestamp_cal_prev == sensor_cal->gyro.timestamp_cal) {
            hal_out.gyro_cal_status &= ~INV_NEW_DATA;
    }
    if (sensor_cal->accel.timestamp_prev == sensor_cal->accel.timestamp) {
        hal_out.accel_status &= ~INV_NEW_DATA;
    }
    if (sensor_cal->compass.timestamp_prev == sensor_cal->compass.timestamp) {
        hal_out.compass_status &= ~INV_NEW_DATA;
    }
    if (sensor_cal->compass.timestamp_cal_prev == sensor_cal->compass.timestamp_cal) {
        hal_out.compass_cal_status &= ~INV_NEW_DATA;
    }
    if (sensor_cal->quat.timestamp_prev == sensor_cal->quat.timestamp) {
        hal_out.quat_status &= ~INV_NEW_DATA;
    }

    // Only output 9-axis if all 9 sensors are on.
    if (sensor_cal->quat.status & INV_SENSOR_ON) {
        // If quaternion sensor is on, gyros are not required as quaternion already has that part
        if ((sensor_cal->accel.status & sensor_cal->compass.status & sensor_cal->compass.cal_status & INV_SENSOR_ON) == 0) {
            use_sensor = -1;
        }
    } else {
        if ((sensor_cal->gyro.status & sensor_cal->gyro.cal_status & sensor_cal->accel.status & sensor_cal->compass.status & sensor_cal->compass.cal_status & INV_SENSOR_ON) == 0) {
            use_sensor = -1;
        }
    }
#if MPL_LOG_9AXIS_DEBUG
    MPL_LOGI("use_sensor=%d", use_sensor);
#endif
    switch (use_sensor) {
    case 0:
  //      hal_out.nine_axis_status = (sensor_cal->gyro.status & INV_NEW_DATA) ? 1 : 0;
  //      hal_out.nav_timestamp = sensor_cal->gyro.timestamp;
        break;
    case 1:
  //      hal_out.nine_axis_status = (sensor_cal->accel.status & INV_NEW_DATA) ? 1 : 0;
   //     hal_out.nav_timestamp = sensor_cal->accel.timestamp;
        break;
    case 2:
  //      hal_out.nine_axis_status = (sensor_cal->compass.status & INV_NEW_DATA) ? 1 : 0;
  //      hal_out.nine_axis_status = (sensor_cal->compass.cal_status & INV_NEW_DATA) ? 1 : 0;
   //     if (sensor_cal->compass.status & INV_NEW_DATA)
  //          hal_out.nav_timestamp = sensor_cal->compass.timestamp;
  //      else if (sensor_cal->compass.cal_status & INV_NEW_DATA)
   //         hal_out.nav_timestamp = sensor_cal->compass.timestamp_cal;
        break;
    case 3:
        hal_out.nine_axis_status = (sensor_cal->quat.status & INV_NEW_DATA) ? 1 : 0;
        hal_out.nav_timestamp = sensor_cal->quat.timestamp;
        break;
    default:
        hal_out.nine_axis_status = 0; // Don't output quaternion related info
        break;
    }
#if MPL_LOG_9AXIS_DEBUG
    MPL_LOGI("nav ts: %lld", hal_out.nav_timestamp);
#endif
    /* Converts fixed point to uT. Fixed point has 1 uT = 2^16.
     * So this is: 1 / 2^16*/
    #define COMPASS_CONVERSION 1.52587890625e-005f

    inv_get_compass_set(compass, &accuracy, &(hal_out.mag_timestamp) );
    hal_out.accuracy_mag = (int)accuracy;

#ifndef CALIB_COMPASS_NOISE_REDUCTION
    for (i = 0; i < 3; i++) {
        hal_out.compass_float[i] = (float)compass[i] * COMPASS_CONVERSION;
    }
#else
    for (i = 0; i < 3; i++) {
        if ( ((sensor_cal->compass.status & (INV_NEW_DATA | INV_CONTIGUOUS)) == INV_NEW_DATA) ||
             ((sensor_cal->compass.cal_status & (INV_NEW_DATA | INV_CONTIGUOUS)) == INV_NEW_DATA)) {
            // set the state variables to match output with input
            inv_calc_state_to_match_output(&hal_out.lp_filter[i], (float)compass[i]);
        }
        if (((sensor_cal->compass.status & (INV_NEW_DATA | INV_RAW_DATA)) == (INV_NEW_DATA | INV_RAW_DATA)) ||
            ((sensor_cal->compass.cal_status & (INV_NEW_DATA | INV_RAW_DATA)) == (INV_NEW_DATA | INV_RAW_DATA)) ) {
            hal_out.compass_float[i] = inv_biquad_filter_process(&hal_out.lp_filter[i], (float)compass[i]) * COMPASS_CONVERSION;
        } else if (((sensor_cal->compass.status & INV_NEW_DATA) == INV_NEW_DATA) ||
                   ((sensor_cal->compass.cal_status & INV_NEW_DATA) == INV_NEW_DATA) ){
            hal_out.compass_float[i] = (float)compass[i] * COMPASS_CONVERSION;
        }
    }
#endif // CALIB_COMPASS_NOISE_REDUCTION
    return INV_SUCCESS;
}

/** Turns off generation of HAL outputs.
* @return Returns INV_SUCCESS if successful or an error code if not.
 */
inv_error_t inv_stop_hal_outputs(void)
{
    inv_error_t result;
    result = inv_unregister_data_cb(inv_generate_hal_outputs);
    return result;
}

/** Turns on generation of HAL outputs. This should be called after inv_stop_hal_outputs()
* to turn generation of HAL outputs back on. It is automatically called by inv_enable_hal_outputs().
* @return Returns INV_SUCCESS if successful or an error code if not.
*/
inv_error_t inv_start_hal_outputs(void)
{
    inv_error_t result;
    result =
        inv_register_data_cb(inv_generate_hal_outputs,
                             INV_PRIORITY_HAL_OUTPUTS,
                             INV_GYRO_NEW | INV_ACCEL_NEW | INV_MAG_NEW | INV_QUAT_NEW | INV_PRESSURE_NEW);
    return result;
}

/* file name: lowPassFilterCoeff_1_6.c */
float compass_low_pass_filter_coeff[5] =
{+2.000000000000f, +1.000000000000f, -1.279632424998f, +0.477592250073f, +0.049489956269f};

/** Initializes hal outputs class. This is called automatically by the
* enable function. It may be called any time the feature is enabled, but
* is typically not needed to be called by outside callers.
* @return Returns INV_SUCCESS if successful or an error code if not.
*/
inv_error_t inv_init_hal_outputs(void)
{
    int i;
    memset(&hal_out, 0, sizeof(hal_out));
    for (i=0; i<3; i++)  {
        inv_init_biquad_filter(&hal_out.lp_filter[i], compass_low_pass_filter_coeff);
    }

    return INV_SUCCESS;
}

/** Turns on creation and storage of HAL type results.
* @return Returns INV_SUCCESS if successful or an error code if not.
*/
inv_error_t inv_enable_hal_outputs(void)
{
    inv_error_t result;

        // don't need to check the result for inv_init_hal_outputs
        // since it's always INV_SUCCESS
        inv_init_hal_outputs();

    result = inv_register_mpl_start_notification(inv_start_hal_outputs);
    return result;
}

/** Turns off creation and storage of HAL type results.
*/
inv_error_t inv_disable_hal_outputs(void)
{
    inv_error_t result;

    inv_stop_hal_outputs(); // Ignore error if we have already stopped this
    result = inv_unregister_mpl_start_notification(inv_start_hal_outputs);
    return result;
}

/** Calculate compass bias (uT) in body frame.
* @param[out] cal Calibrated compass data (uT), length 3.  *** NOTE:  non-essential, can be removed
* @param[out] raw Uncalibrated compass data (uT), length 3.  *** NOTE:  non-essential, can be removed
* @param[out] bias raw-cal (uT), length 3
*
*/
void inv_calculate_compass_bias(float *cal, float *raw, float *bias)
{

    int8_t mag_accuracy;
    inv_time_t mag_timestamp;

	if (!cal || !raw)			// joAnne: output these for now... for debugging
        return;

	// get the calibrated data
	inv_get_sensor_type_magnetic_field(cal, &mag_accuracy, &mag_timestamp);

	if (0 == mag_accuracy)		// data isnt calibrated
		return;
	else
	{
		int8_t raw_accuracy;
		inv_time_t rawTimeStamp;

		// get the raw data
		inv_get_sensor_type_magnetic_field_raw(raw, &raw_accuracy, &rawTimeStamp);

		// TODO:  consider making this a function call
		bias[0] = raw[0] - cal[0];
		bias[1] = raw[1] - cal[1];
		bias[2] = raw[2] - cal[2];
		return;
	}
}
// TODO:  Make into a more generic function for all sensors, by passing in sensor type?

/** Calculate gyro bias (rad/s) in body frame.
* @param[out] cal Calibrated gyro data (rad/s), length 3.  *** NOTE:  non-essential, can be removed
* @param[out] raw Uncalibrated gyro data (rad/s), length 3.  *** NOTE:  non-essential, can be removed
* @param[out] bias raw-cal (rad/s), length 3
*
*/

void inv_calculate_gyro_bias(float *cal, float *raw, float *bias)
{

    int8_t cal_accuracy;
    inv_time_t cal_timestamp;

	if (!cal || !raw)			// joAnne: output these for now... for debugging
        return;

	// get the calibrated data
	inv_get_sensor_type_gyroscope(cal, &cal_accuracy, &cal_timestamp);

	if (0 == cal_accuracy)		// data isnt calibrated
		return;
	else
	{
		int8_t raw_accuracy;
		inv_time_t rawTimeStamp;

		// get the raw data
		inv_get_sensor_type_gyroscope_raw(raw, &raw_accuracy, &rawTimeStamp);

		// TODO:  consider making this a function call
		bias[0] = raw[0] - cal[0];
		bias[1] = raw[1] - cal[1];
		bias[2] = raw[2] - cal[2];
		return;
	}
}

/**
 * @}
 */



